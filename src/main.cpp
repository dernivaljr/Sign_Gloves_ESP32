#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050_light.h>
#include <DFRobotDFPlayerMini.h>
#include <HardwareSerial.h>

// =====================================================
// ================== MODOS DO APLICATIVO ==============
// =====================================================
#define MODE_DIAGNOSTIC 0   // scan I2C + leituras ADC/MPU no Serial
#define MODE_CAPTURE    1   // sua captura + OLED + SD + DFPlayer
#define MODE_HWTEST     2   // autoteste OLED/SD/DFPlayer

#define APP_MODE MODE_HWTEST   // <<< Troque para MODE_CAPTURE quando for calibrar

// =====================================================
// ================== PINAGEM (ESP32 DevKit V1) ========
// =====================================================
// OLED I2C e MPU6050 I2C
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// OLED
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   32
#define OLED_RESET      -1
const uint8_t OLED_ADDR = 0x3C;

// Flex (ADC) — 36 e 39 são somente ENTRADA (ok para ADC)
#define FLEX1_PIN 36
#define FLEX2_PIN 39
#define FLEX3_PIN 34
#define FLEX4_PIN 35
#define FLEX5_PIN 32

// Botões/LEDs — pinos “seguros” (não estrangulam boot/flash)
#define SWITCH_PIN     26
#define BUTTON_PIN     25
#define LED_GREEN_PIN   4
#define LED_RED_PIN    13   // 13 está livre no nosso mapeamento

// DFPlayer (Serial1)
#define DF_RX_PIN 16   // ESP32 RX de DFPlayer (liga no TX do DFPlayer)
#define DF_TX_PIN 17   // ESP32 TX de DFPlayer (liga no RX do DFPlayer)

// SD (VSPI “padrão seguro”)
#define SD_CS_PIN    27
#define SD_MISO_PIN  19
#define SD_MOSI_PIN  23
#define SD_SCK_PIN   18

// =====================================================
// ================== OBJETOS ==========================
// =====================================================
HardwareSerial dfSerial(1);
DFRobotDFPlayerMini dfPlayer;
MPU6050 mpu(Wire);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
SPIClass spiSD(VSPI);

// =====================================================
// ================== DADOS/CAPTURA ====================
// =====================================================
const int SAMPLES           = 200; // 2 s @ 100 Hz
const int NUM_SENSORS       = 8;   // 5 flex + 3 eixos accel
const int NUM_WORDS         = 10;
const int CAPTURES_PER_WORD = 3;
const int TOTAL_CAPTURES    = NUM_WORDS * CAPTURES_PER_WORD;
const int SAMPLE_DELAY      = 10;  // 100 Hz

float    dataMatrix[SAMPLES][NUM_SENSORS];
int      captureCount = 0;
bool     captureMode  = true;
int      capturesPerWord[NUM_WORDS] = {0};
String   words[NUM_WORDS] = {"ola","por favor","obrigado","bom dia","é","meu","nome","boa tarde","ajuda","boa noite"};
uint16_t currentIndex = 1;
uint32_t lastPlayMillis = 0;

// =====================================================
// ================== TIPOS / UTILS ====================
// =====================================================
struct AnalogSensorConfig {
  const char* name;
  int pin;
};

AnalogSensorConfig analogSensors[] = {
  {"Dedo polegar",   FLEX5_PIN}, // 32
  {"Dedo indicador", FLEX3_PIN}, // 34
  {"Dedo medio",     FLEX4_PIN}, // 35
  {"Dedo anelar",    FLEX1_PIN}, // 36
  {"Dedo minimo",    FLEX2_PIN}, // 39
};
constexpr size_t ANALOG_SENSOR_COUNT = sizeof(analogSensors)/sizeof(analogSensors[0]);

static inline void printDivider() { Serial.println(F("----------------------------------------")); }
static inline void logMessage(const String& m) { Serial.println(m); }

// =====================================================
// ================== BLOCO DIAGNÓSTICO ================
// =====================================================
void configureAnalogInputs() {
  for (size_t i = 0; i < ANALOG_SENSOR_COUNT; ++i) pinMode(analogSensors[i].pin, INPUT);
}

void scanI2CBus() {
  Serial.println(F("Varredura I2C..."));
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) { Serial.print(F(" - 0x")); Serial.println(addr, HEX); ++found; }
  }
  if (!found) Serial.println(F("Nenhum dispositivo I2C encontrado."));
  printDivider();
}

void readAnalogSensors() {
  Serial.println(F("=== Flex (ADC) ==="));
  for (size_t i = 0; i < ANALOG_SENSOR_COUNT; ++i) {
    int raw = analogRead(analogSensors[i].pin);
    float v = raw * (3.3f/4095.0f);
    Serial.print(analogSensors[i].name); Serial.print(F(" [GPIO "));
    Serial.print(analogSensors[i].pin); Serial.print(F("]: "));
    Serial.print(raw); Serial.print(F(" (~"));
    Serial.print(v,2); Serial.println(F(" V)"));
  }
  printDivider();
}

// MPU raw helpers
bool writeMPU(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(0x68);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool readMPUBlock(uint8_t startReg, uint8_t* buf, size_t len) {
  Wire.beginTransmission(0x68);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) return false;
  size_t r = Wire.requestFrom((uint8_t)0x68, (uint8_t)len);
  if (r != len) return false;
  for (size_t i=0;i<len;++i) buf[i]=Wire.read();
  return true;
}

bool configureMPU6050_raw() {
  Serial.println(F("Inicializando MPU6050 (raw)..."));
  Wire.beginTransmission(0x68);
  if (Wire.endTransmission() != 0) {
    Serial.println(F("MPU6050 não respondeu (0x68). Cheque fiação/alimentação."));
    return false;
  }
  if (!writeMPU(0x6B, 0x00)) return false; // Wake
  if (!writeMPU(0x1C, 0x08)) return false; // Accel ±4g
  if (!writeMPU(0x1B, 0x08)) return false; // Gyro ±500°/s
  Serial.println(F("MPU6050 OK"));
  return true;
}

void readMPU6050_raw() {
  uint8_t raw[14];
  if (!readMPUBlock(0x3B, raw, sizeof(raw))) { Serial.println(F("Falha ao ler MPU6050.")); return; }
  auto toI16 = [&](int i){ return (int16_t)((raw[i]<<8)|raw[i+1]); };

  const float accelScale = 8192.0f; // ±4g
  const float gyroScale  = 65.5f;   // ±500°/s
  float ax = toI16(0)/accelScale*9.80665f;
  float ay = toI16(2)/accelScale*9.80665f;
  float az = toI16(4)/accelScale*9.80665f;
  float tc = toI16(6)/340.0f + 36.53f;
  float gx = toI16(8)/gyroScale;
  float gy = toI16(10)/gyroScale;
  float gz = toI16(12)/gyroScale;

  Serial.println(F("=== Accel (m/s^2) ==="));
  Serial.printf("X: %.3f | Y: %.3f | Z: %.3f\n", ax, ay, az);
  Serial.println(F("=== Gyro (°/s) ==="));
  Serial.printf("X: %.3f | Y: %.3f | Z: %.3f\n", gx, gy, gz);
  Serial.print(F("Temp (°C): ")); Serial.println(tc,2);
  printDivider();
}

// =====================================================
// ================== BLOCO CAPTURA ====================
// =====================================================
void calculateMean(float meanVector[], const float* matrix) {
  for (int j=0;j<NUM_SENSORS;++j) {
    float s=0.0f; for (int i=0;i<SAMPLES;++i) s += matrix[i*NUM_SENSORS + j];
    meanVector[j] = s / SAMPLES;
  }
}

void saveMeanToSD(int wordIndex, float meanVector[]) {
  String filename = "/" + words[wordIndex] + "_mean.csv";
  File f = SD.open(filename, FILE_WRITE);
  if (!f) { logMessage("Erro ao abrir " + filename); return; }
  f.println("Sensor,Value");
  for (int j=0;j<NUM_SENSORS;++j) f.println(String(j)+","+String(meanVector[j],4));
  f.close();
  logMessage("Salvo: " + filename);
}

void captureMovement() {
  int wordIndex   = captureCount / CAPTURES_PER_WORD;
  int captureIdx  = capturesPerWord[wordIndex];

  display.clearDisplay();
  display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0); display.println("Captura:");
  display.setCursor(0,10); display.println(words[wordIndex]);
  display.setCursor(100,10); display.print(captureIdx+1);
  display.display();

  for (int i=0;i<SAMPLES;++i) {
    dataMatrix[i][0] = analogRead(FLEX1_PIN)/4095.0f;
    dataMatrix[i][1] = analogRead(FLEX2_PIN)/4095.0f;
    dataMatrix[i][2] = analogRead(FLEX3_PIN)/4095.0f;
    dataMatrix[i][3] = analogRead(FLEX4_PIN)/4095.0f;
    dataMatrix[i][4] = analogRead(FLEX5_PIN)/4095.0f;

    mpu.update();
    dataMatrix[i][5] = mpu.getAccX();
    dataMatrix[i][6] = mpu.getAccY();
    dataMatrix[i][7] = mpu.getAccZ();

    delay(SAMPLE_DELAY);
  }

  float mean[NUM_SENSORS] = {0};
  calculateMean(mean, (float*)dataMatrix);
  saveMeanToSD(wordIndex, mean);

  capturesPerWord[wordIndex]++; captureCount++;
  if (capturesPerWord[wordIndex] == CAPTURES_PER_WORD) logMessage("Concluídas: " + words[wordIndex]);
  if (captureCount >= TOTAL_CAPTURES) { logMessage("Calibração concluída!"); captureMode=false; }
}

// =====================================================
// ================== AUTOTESTE ========================
// =====================================================
bool testOLED() {
  Serial.println(F("[OLED] Inicializando..."));
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("[OLED] FALHA: 0x3C não respondeu"));
    return false;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);  display.println(F("OLED OK"));
  display.setCursor(0, 10); display.println(F("SSD1306 128x32"));
  display.display();
  Serial.println(F("[OLED] OK"));
  return true;
}

bool testSD() {
  Serial.println(F("[SD] Inicializando..."));
  spiSD.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN, spiSD)) { Serial.println(F("[SD] FALHA: SD.begin()")); return false; }

  File f = SD.open("/selftest.txt", FILE_WRITE);
  if (!f) { Serial.println(F("[SD] FALHA: open write")); return false; }
  f.println("Selftest SD OK");
  f.close();

  f = SD.open("/selftest.txt", FILE_READ);
  if (!f) { Serial.println(F("[SD] FALHA: open read")); return false; }
  String line = f.readStringUntil('\n');
  f.close();
  if (line.indexOf("Selftest SD OK") < 0) { Serial.println(F("[SD] FALHA: conteudo")); return false; }

  Serial.println(F("[SD] Listando raiz:"));
  File root = SD.open("/");
  File file;
  while ((file = root.openNextFile())) {
    Serial.print(" - "); Serial.print(file.name());
    if (!file.isDirectory()) { Serial.print(" ("); Serial.print(file.size()); Serial.println(" bytes)"); }
    else Serial.println(" <DIR>");
    file.close();
  }
  root.close();
  Serial.println(F("[SD] OK"));
  return true;
}

bool testDFPlayer() {
  Serial.println(F("[DFP] Inicializando..."));
  dfSerial.begin(9600, SERIAL_8N1, DF_RX_PIN, DF_TX_PIN);
  if (!dfPlayer.begin(dfSerial)) { Serial.println(F("[DFP] FALHA: dfPlayer.begin()")); return false; }
  dfPlayer.volume(22); // 0..30
  delay(300);

  for (int i = 1; i <= 10; ++i) {
    Serial.print(F("[DFP] Play ")); Serial.println(i);
    dfPlayer.play(i);
    uint32_t t0 = millis();
    while (millis() - t0 < 1000) { delay(10); } // ~1s por faixa
  }
  Serial.println(F("[DFP] OK (verifique audio 1..10)"));
  return true;
}

// =====================================================
// ================== SETUP / LOOP =====================
// =====================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

#if APP_MODE == MODE_HWTEST
  Serial.println();
  Serial.println(F("===== AUTOTESTE: OLED + SD + DFPlayer ====="));
  bool okOLED = testOLED();
  bool okSD   = testSD();
  bool okDFP  = testDFPlayer();

  Serial.println(F("----------------------------------------"));
  Serial.print(F("OLED : ")); Serial.println(okOLED ? F("OK") : F("FALHA"));
  Serial.print(F("SD   : ")); Serial.println(okSD   ? F("OK") : F("FALHA"));
  Serial.print(F("DFP  : ")); Serial.println(okDFP  ? F("OK") : F("FALHA"));
  Serial.println(F("----------------------------------------"));

  if (okOLED) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);  display.println(F("SELFTEST:"));
    display.setCursor(0, 10); display.print(F("SD:  "));  display.println(okSD ? F("OK") : F("ERRO"));
    display.setCursor(0, 20); display.print(F("DFP: ")); display.println(okDFP ? F("OK") : F("ERRO"));
    display.display();
  }

#elif APP_MODE == MODE_CAPTURE
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    logMessage("Erro: OLED nao inicializado!");
    while (true) { delay(1000); }
  }
  display.clearDisplay(); display.display();

  if (mpu.begin() != 0) { logMessage("Erro: MPU6050 nao inicializado!"); while(true){delay(1000);} }
  logMessage("MPU6050 inicializado.");

  dfSerial.begin(9600, SERIAL_8N1, DF_RX_PIN, DF_TX_PIN);
  if (!dfPlayer.begin(dfSerial)) { logMessage("Erro: DFPlayer nao inicializado!"); while(true){delay(1000);} }
  dfPlayer.volume(20);
  logMessage("DFPlayer inicializado.");

  spiSD.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN, spiSD)) { logMessage("Erro: SD Card nao inicializado!"); while(true){delay(1000);} }
  logMessage("SD Card inicializado.");
  logMessage("Setup concluido. Pressione o botao para capturar.");

#else // MODE_DIAGNOSTIC
  Serial.println();
  Serial.println(F("===== DIAGNOSTICO (ESP32 DevKit V1) ====="));
  configureAnalogInputs();
  scanI2CBus();
  if (!configureMPU6050_raw()) {
    Serial.println(F("Erro na configuracao do MPU6050. Parando."));
    while (true) { delay(1000); }
  }
#endif
}

void loop() {
#if APP_MODE == MODE_HWTEST
  delay(1000);

#elif APP_MODE == MODE_CAPTURE
  if (digitalRead(BUTTON_PIN) == LOW && captureMode) {
    digitalWrite(LED_GREEN_PIN, HIGH);
    captureMovement();
    digitalWrite(LED_GREEN_PIN, LOW);
    delay(500);
  }
  if (!captureMode && millis() - lastPlayMillis >= 2000) {
    dfPlayer.play(currentIndex);
    lastPlayMillis = millis();
    currentIndex = (currentIndex % NUM_WORDS) + 1;
  }

#else // MODE_DIAGNOSTIC
  readAnalogSensors();
  readMPU6050_raw();
  Serial.println(F("Aguarde 2s..."));
  printDivider();
  delay(2000);
#endif
}
