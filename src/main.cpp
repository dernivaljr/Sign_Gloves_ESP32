#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050_light.h>
#include <DFRobotDFPlayerMini.h>
#include <SPI.h>
#include <SD.h>

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

void setup() {
  Serial.begin(115200);
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
  }
  display.clearDisplay();

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
}
