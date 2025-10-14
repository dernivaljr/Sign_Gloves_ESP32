#include <Arduino.h>
<<<<<<< HEAD
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050_light.h>
#include <DFRobotDFPlayerMini.h>
#include <SPI.h>
#include <SD.h>
#include <HardwareSerial.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define FLEX1_PIN 36
#define FLEX2_PIN 39
#define FLEX3_PIN 34
#define FLEX4_PIN 35
#define FLEX5_PIN 32
#define MPU_SDA_PIN 21
#define MPU_SCL_PIN 22
#define DF_RX_PIN 16
#define DF_TX_PIN 17
#define SWITCH_PIN 1
#define BUTTON_PIN 2
#define LED_GREEN_PIN 4
#define LED_RED_PIN 5
#define SD_CS_PIN 27
#define SD_MISO_PIN 12
#define SD_MOSI_PIN 13
#define SD_SCK_PIN 14

const int SAMPLES = 200; // 2 segundos a 100 Hz
const int NUM_SENSORS = 8; // 5 flex + 3 eixos MPU
const int NUM_WORDS = 10;
const int CAPTURES_PER_WORD = 3;
const int TOTAL_CAPTURES = NUM_WORDS * CAPTURES_PER_WORD;
const int SAMPLE_DELAY = 10; // 1000 ms / 100 Hz

HardwareSerial dfSerial(1);
DFRobotDFPlayerMini dfPlayer;
MPU6050 mpu(Wire);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
SPIClass spiSD;

float dataMatrix[SAMPLES][NUM_SENSORS];
int captureCount = 0;
bool captureMode = true;
int capturesPerWord[NUM_WORDS] = {0};
String words[NUM_WORDS] = {"ola", "por favor", "obrigado", "bom dia", "é", "meu", "nome", "boa tarde", "ajuda", "boa noite"};
uint16_t currentIndex = 1;
uint32_t lastPlayMillis = 0;

void logMessage(const String& message) {
  Serial.println(message);
}

void calculateMean(float meanVector[NUM_SENSORS], float* matrix, int wordIndex) {
  for (int j = 0; j < NUM_SENSORS; j++) {
    float sum = 0.0;
    for (int i = 0; i < SAMPLES; i++) {
      sum += matrix[i * NUM_SENSORS + j];
    }
    meanVector[j] = sum / SAMPLES;
  }
  String meanDebug = "Média para '" + words[wordIndex] + "': ";
  for (int j = 0; j < NUM_SENSORS; j++) {
    meanDebug += String(meanVector[j], 4) + " ";
  }
  logMessage(meanDebug);
}

void saveMeanToSD(int wordIndex, float meanVector[NUM_SENSORS]) {
  String filename = "/" + words[wordIndex] + "_mean.csv";
  File file = SD.open(filename, FILE_WRITE);
  if (file) {
    file.println("Sensor,Value");
    for (int j = 0; j < NUM_SENSORS; j++) {
      file.println(String(j) + "," + String(meanVector[j], 4));
    }
    file.close();
    logMessage("Salva média para '" + words[wordIndex] + "' em " + filename);
  } else {
    logMessage("Erro ao salvar média para '" + words[wordIndex] + "'");
  }
}

void captureMovement() {
  int wordIndex = captureCount / CAPTURES_PER_WORD;
  int captureIndex = capturesPerWord[wordIndex];

  String msg = "Capturando " + String(captureIndex + 1) + " para '" + words[wordIndex] + "'";
  logMessage(msg);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Captura:");
  display.setCursor(0, 10);
  display.println(words[wordIndex]);
  display.setCursor(60, 10);
  display.print(captureIndex + 1);
  display.display();

  for (int i = 0; i < SAMPLES; i++) {
    dataMatrix[i][0] = analogRead(FLEX1_PIN) / 4095.0;
    dataMatrix[i][1] = analogRead(FLEX2_PIN) / 4095.0;
    dataMatrix[i][2] = analogRead(FLEX3_PIN) / 4095.0;
    dataMatrix[i][3] = analogRead(FLEX4_PIN) / 4095.0;
    dataMatrix[i][4] = analogRead(FLEX5_PIN) / 4095.0;

    mpu.update();
    dataMatrix[i][5] = mpu.getAccX();
    dataMatrix[i][6] = mpu.getAccY();
    dataMatrix[i][7] = mpu.getAccZ();

    delay(SAMPLE_DELAY);
  }

  float meanVector[NUM_SENSORS] = {0.0};
  calculateMean(meanVector, (float*)dataMatrix, wordIndex);
  saveMeanToSD(wordIndex, meanVector);

  capturesPerWord[wordIndex]++;
  captureCount++;

  if (capturesPerWord[wordIndex] == CAPTURES_PER_WORD) {
    logMessage("Capturas para '" + words[wordIndex] + "' concluídas.");
  }
  if (captureCount >= TOTAL_CAPTURES) {
    logMessage("Calibração concluída!");
    captureMode = false;
  }
}
=======
#include <HardwareSerial.h>
#include <Wire.h>

// Algumas configurações de build do core ESP32 desabilitam a declaração
// automática do objeto global `Serial`. Declaramos manualmente aqui para
// garantir que os esboços que usam apenas PlatformIO/Arduino encontrem o
// símbolo durante a compilação, independentemente do valor de
// CONFIG_DISABLE_HAL_LOCKS. Fornecemos uma definição fraca que instancia
// o objeto na UART0 somente quando o core não fizer isso automaticamente.
HardwareSerial Serial(0) __attribute__((weak));

// === CONFIGURAÇÕES ===
// Ajuste a lista abaixo com os pinos ADC usados pelos sensores flex.
// No ESP32 DevKit V1 é recomendável usar apenas pinos ADC0/ADC1 (32-39).
struct AnalogSensorConfig {
  const char *name;
  uint8_t pin;
};
>>>>>>> codex/send-test-code-for-esp32-modules-qnbph3

AnalogSensorConfig analogSensors[] = {
    {"Dedo polegar", 32},
    {"Dedo indicador", 33},
    {"Dedo médio", 34},
    {"Dedo anelar", 35},
    {"Dedo mínimo", 36}, // GPIO36 (VP) – somente entrada
};
constexpr size_t ANALOG_SENSOR_COUNT = sizeof(analogSensors) / sizeof(AnalogSensorConfig);

// Pinos padrão do barramento I2C no ESP32 DevKit V1
constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;

constexpr uint8_t MPU6050_ADDR = 0x68; // endereço padrão do MPU6050

// === FUNÇÕES AUXILIARES ===
void printDivider() {
  Serial.println(F("----------------------------------------"));
}

void configureAnalogInputs() {
  for (size_t i = 0; i < ANALOG_SENSOR_COUNT; ++i) {
    pinMode(analogSensors[i].pin, INPUT);
  }
}

void scanI2CBus() {
  Serial.println(F("Iniciando varredura I2C..."));
  uint8_t deviceCount = 0;

  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      Serial.print(F("Dispositivo encontrado no endereço 0x"));
      Serial.println(address, HEX);
      ++deviceCount;
    } else if (error == 4) {
      Serial.print(F("Erro desconhecido no endereço 0x"));
      Serial.println(address, HEX);
    }
  }

  if (deviceCount == 0) {
    Serial.println(F("Nenhum dispositivo I2C encontrado."));
  } else {
    Serial.print(deviceCount);
    Serial.println(F(" dispositivo(s) I2C detectado(s)."));
  }
  printDivider();
}

void readAnalogSensors() {
  Serial.println(F("=== Leituras dos Sensores Flex (ADC) ==="));
  for (size_t i = 0; i < ANALOG_SENSOR_COUNT; ++i) {
    int raw = analogRead(analogSensors[i].pin);
    float voltage = raw * (3.3f / 4095.0f); // Conversão aproximada

    Serial.print(analogSensors[i].name);
    Serial.print(F(" (GPIO "));
    Serial.print(analogSensors[i].pin);
    Serial.print(F("): valor bruto = "));
    Serial.print(raw);
    Serial.print(F(", tensão ≈ "));
    Serial.print(voltage, 2);
    Serial.println(F(" V"));
  }
  printDivider();
}

bool writeMPU6050Register(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readMPU6050Registers(uint8_t startReg, uint8_t *buffer, size_t length) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  size_t read = Wire.requestFrom(MPU6050_ADDR, static_cast<uint8_t>(length));
  if (read != length) {
    return false;
  }

  for (size_t i = 0; i < length; ++i) {
    buffer[i] = Wire.read();
  }
  return true;
}

void readMPU6050() {
  uint8_t rawData[14];
  if (!readMPU6050Registers(0x3B, rawData, sizeof(rawData))) {
    Serial.println(F("Falha ao ler dados do MPU6050."));
    printDivider();
    return;
  }

  auto toInt16 = [&](size_t index) {
    return static_cast<int16_t>((rawData[index] << 8) | rawData[index + 1]);
  };

  const float accelScale = 8192.0f; // sensibilidade configurada para ±4g
  const float gyroScale = 65.5f;    // sensibilidade configurada para ±500 °/s

  float accelX = toInt16(0) / accelScale * 9.80665f;
  float accelY = toInt16(2) / accelScale * 9.80665f;
  float accelZ = toInt16(4) / accelScale * 9.80665f;

  float tempC = toInt16(6) / 340.0f + 36.53f;

  float gyroX = toInt16(8) / gyroScale;
  float gyroY = toInt16(10) / gyroScale;
  float gyroZ = toInt16(12) / gyroScale;

  Serial.println(F("=== MPU6050: Acelerômetro (m/s²) ==="));
  Serial.print(F("X: "));
  Serial.print(accelX, 3);
  Serial.print(F(" | Y: "));
  Serial.print(accelY, 3);
  Serial.print(F(" | Z: "));
  Serial.println(accelZ, 3);

  Serial.println(F("=== MPU6050: Giroscópio (°/s) ==="));
  Serial.print(F("X: "));
  Serial.print(gyroX, 3);
  Serial.print(F(" | Y: "));
  Serial.print(gyroY, 3);
  Serial.print(F(" | Z: "));
  Serial.println(gyroZ, 3);

  Serial.println(F("=== MPU6050: Temperatura (°C) ==="));
  Serial.println(tempC, 2);
  printDivider();
}

bool configureMPU6050() {
  Serial.println(F("Inicializando MPU6050..."));

  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println(F("MPU6050 não respondeu no endereço 0x68. Verifique alimentação e fiação."));
    printDivider();
    return false;
  }

  // Acorda o sensor (reg. PWR_MGMT_1)
  if (!writeMPU6050Register(0x6B, 0x00)) {
    Serial.println(F("Falha ao acordar o MPU6050."));
    printDivider();
    return false;
  }

  // Configura o acelerômetro para ±4g (reg. ACCEL_CONFIG)
  if (!writeMPU6050Register(0x1C, 0x08)) {
    Serial.println(F("Falha ao configurar alcance do acelerômetro."));
    printDivider();
    return false;
  }

  // Configura o giroscópio para ±500 °/s (reg. GYRO_CONFIG)
  if (!writeMPU6050Register(0x1B, 0x08)) {
    Serial.println(F("Falha ao configurar alcance do giroscópio."));
    printDivider();
    return false;
  }

  Serial.println(F("MPU6050 configurado com sucesso!"));
  printDivider();
  return true;
}

// === FUNÇÕES PRINCIPAIS ===
void setup() {
  Serial.begin(115200);
<<<<<<< HEAD
  while (!Serial)
    ;

  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);

  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    logMessage("Erro: OLED não inicializado!");
    while (1)
      ;
=======
  while (!Serial) {
    delay(10);
>>>>>>> codex/send-test-code-for-esp32-modules-qnbph3
  }
  display.clearDisplay();

<<<<<<< HEAD
  if (mpu.begin() != 0) {
    logMessage("Erro: MPU6050 não inicializado!");
    while (1)
      ;
  }
  logMessage("MPU6050 inicializado.");

  dfSerial.begin(9600, SERIAL_8N1, DF_RX_PIN, DF_TX_PIN);
  if (!dfPlayer.begin(dfSerial)) {
    logMessage("Erro: DFPlayer não inicializado!");
    while (1)
      ;
  }
  dfPlayer.volume(20);
  logMessage("DFPlayer inicializado.");

  spiSD.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN, spiSD)) {
    logMessage("Erro: SD Card não inicializado!");
    while (1)
      ;
  }
  logMessage("SD Card inicializado.");

  logMessage("Setup concluído. Pressione o botão para capturar.");
}

void loop() {
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
=======
  Serial.println();
  Serial.println(F("===== Diagnóstico da Luva com ESP32 DevKit V1 ====="));
  printDivider();

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Serial.print(F("Barramento I2C iniciado (SDA="));
  Serial.print(I2C_SDA_PIN);
  Serial.print(F(", SCL="));
  Serial.print(I2C_SCL_PIN);
  Serial.println(F(")"));
  printDivider();

  configureAnalogInputs();
  Serial.println(F("Entradas analógicas configuradas."));
  printDivider();

  scanI2CBus();
  if (!configureMPU6050()) {
    Serial.println(F("Interrompendo leituras devido a erro na configuração do MPU6050."));
    while (true) {
      delay(1000);
    }
  }
}

void loop() {
  readAnalogSensors();
  readMPU6050();

  Serial.println(F("Aguarde 2 segundos para nova leitura..."));
  printDivider();
  delay(2000);
>>>>>>> codex/send-test-code-for-esp32-modules-qnbph3
}
