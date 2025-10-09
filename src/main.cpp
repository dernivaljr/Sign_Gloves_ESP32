/*
 * === Projeto: Luva Tradutora de Libras ESP32-C3 Super Mini ===
 * Descrição: Sistema para reconhecimento de gestos de Libras com SD Card para armazenamento de calibração.
 * Autor: Desenvolvido com Grok (xAI)
 * Data de Início: Setembro 2025
 *
 * === Histórico de Versões ===
 * v1.26: Integração de SD Card com remanejamento de pinos para Super Mini.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050_light.h>
#include <DFRobotDFPlayerMini.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>

// Declaração forward para logMessage
void logMessage(const String &message);

// ==== PINOS REMAPEADOS PARA SUPER MINI + SD ====
constexpr int DF_RX_PIN = 1;       // UART0 RX (GPIO1)
constexpr int DF_TX_PIN = 0;       // UART0 TX (GPIO0)
constexpr int MPU_SDA_PIN = 2;     // I2C SDA (GPIO2)
constexpr int MPU_SCL_PIN = 3;     // I2C SCL (GPIO3)
constexpr int FLEX_PIN = 4;        // Apenas 1 sensor flexível disponível
constexpr int BUTTON_PIN = 11;     // Botão de captura (removido conflito com SD_CS)
constexpr int MODE_SWITCH_PIN = 12; // Alterna calibração/tradução
constexpr int LED_GREEN_PIN = 8;   // LED verde (onboard)
constexpr int LED_RED_PIN = 9;     // LED vermelho externo
constexpr int SD_CS_PIN = 10;      // Chip Select do SD
constexpr int SD_MISO_PIN = 5;
constexpr int SD_MOSI_PIN = 6;
constexpr int SD_SCK_PIN = 7;
constexpr uint32_t DF_BAUD = 9600;
constexpr uint8_t VOLUME = 25;
constexpr uint16_t FIRST_IDX = 1;
constexpr uint16_t LAST_IDX = 10;
constexpr uint32_t PLAY_GAP_MS = 1500;
constexpr uint32_t DF_TIMEOUT_MS = 3000;
constexpr uint32_t LED_ON_MS = 1000;
constexpr int SAMPLES = 50;            // 2 segundos @ 25 Hz
constexpr int NUM_SENSORS = 4;         // 1 flex + 3 acelerômetros
constexpr int NUM_WORDS = 10;
constexpr int CAPTURES_PER_WORD = 3;
constexpr int TOTAL_CAPTURES = NUM_WORDS * CAPTURES_PER_WORD;
constexpr int BUFFER_SIZE = 200;
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1

// ==== OBJETOS GLOBAIS ====
HardwareSerial DFSerial(0);
DFRobotDFPlayerMini dfp;
MPU6050 mpu(Wire);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
SPIClass spiSD(HSPI);

// ==== ESTADO DE EXECUÇÃO ====
float dataMatrix[SAMPLES][NUM_SENSORS];
uint32_t lastPlayMillis = 0;
uint16_t currentIndex = FIRST_IDX;
int captureCount = 0;
bool captureMode = true;
bool lastModeState = true;
bool waitingNext = true;
bool deviceConnected = false;
bool mpuCalibrated = false;

// Acúmulo de amostras durante calibração
float calibrationSums[NUM_WORDS][NUM_SENSORS] = {{0.0f}};
int calibrationSampleCounts[NUM_WORDS] = {0};
int capturesPerWord[NUM_WORDS] = {0};

// Lista de palavras (sem acentuação para compatibilidade com arquivos/OLED)
String words[NUM_WORDS] = {
  "ola",
  "por favor",
  "obrigado",
  "bom dia",
  "e",
  "meu",
  "nome",
  "boa tarde",
  "ajuda",
  "boa noite"
};

char logBuffer[BUFFER_SIZE] = "";
int bufferIndex = 0;

// ==== Bluetooth BLE ====
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
BLEServer *pServer = nullptr;
BLECharacteristic *pCharacteristic = nullptr;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    deviceConnected = true;
    bufferIndex = 0;
    logBuffer[0] = '\0';
    logMessage("[BLE] Dispositivo conectado");
  }

  void onDisconnect(BLEServer *server) override {
    deviceConnected = false;
    logMessage("[BLE] Dispositivo desconectado");
    BLEDevice::startAdvertising();
  }
};

// ==== LOG BUFFERIZADO PARA SERIAL E BLE ====
void logMessage(const String &message) {
  Serial.println(message);

  if (!deviceConnected || pCharacteristic == nullptr) {
    return;
  }

  int msgLength = message.length();
  if (bufferIndex + msgLength + 1 < BUFFER_SIZE) {
    strcpy(logBuffer + bufferIndex, message.c_str());
    bufferIndex += msgLength;
    logBuffer[bufferIndex++] = '\n';
    logBuffer[bufferIndex] = '\0';
  } else {
    bufferIndex = 0;
    strcpy(logBuffer, message.c_str());
    bufferIndex = msgLength;
    logBuffer[bufferIndex++] = '\n';
    logBuffer[bufferIndex] = '\0';
  }

  pCharacteristic->setValue(reinterpret_cast<uint8_t *>(logBuffer), bufferIndex);
  pCharacteristic->notify();
  delay(10);
}

// ==== UTILITÁRIOS DO DFPLAYER ====
void printDFEvent(uint8_t type, int value) {
  if (type == 11 && value == 2) {
    return; // ruído recorrente
  }

  String message;
  switch (type) {
    case DFPlayerCardInserted:
      message = F("[DFP] Cartao SD inserido");
      break;
    case DFPlayerCardRemoved:
      message = F("[DFP] Cartao SD removido");
      break;
    case DFPlayerCardOnline:
      message = F("[DFP] Cartao SD online");
      break;
    case DFPlayerUSBInserted:
      message = F("[DFP] USB inserido");
      break;
    case DFPlayerUSBRemoved:
      message = F("[DFP] USB removido");
      break;
    case DFPlayerUSBOnline:
      message = F("[DFP] USB online");
      break;
    case DFPlayerPlayFinished:
      message = "[DFP] Faixa finalizada: " + String(value);
      waitingNext = true;
      lastPlayMillis = millis();
      break;
    case DFPlayerError:
      message = F("[DFP] Erro detectado");
      waitingNext = true;
      digitalWrite(LED_RED_PIN, HIGH);
      delay(LED_ON_MS);
      digitalWrite(LED_RED_PIN, LOW);
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println("Erro DFPlayer");
      display.display();
      break;
    default:
      message = "[DFP] Evento desconhecido: tipo=" + String(type) + " valor=" + String(value);
      break;
  }

  logMessage(message);
}

bool playIndex(uint16_t idx) {
  if (idx < FIRST_IDX || idx > LAST_IDX) {
    logMessage("[DFP] Indice fora do range: " + String(idx));
    digitalWrite(LED_RED_PIN, HIGH);
    delay(LED_ON_MS);
    digitalWrite(LED_RED_PIN, LOW);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Indice invalido");
    display.display();
    return false;
  }

  logMessage("[DFP] Reproduzindo indice " + String(idx) + " (" + words[min(idx - 1, static_cast<uint16_t>(NUM_WORDS - 1))] + ")");
  dfp.playMp3Folder(idx);
  waitingNext = false;
  currentIndex = idx;

  unsigned long start = millis();
  while (millis() - start < DF_TIMEOUT_MS) {
    if (dfp.available()) {
      uint8_t type = dfp.readType();
      int value = dfp.read();
      printDFEvent(type, value);
      if (type == DFPlayerPlayFinished) {
        return true;
      }
    }
  }

  logMessage("[DFP] Timeout aguardando termino do audio");
  waitingNext = true;
  digitalWrite(LED_RED_PIN, HIGH);
  delay(LED_ON_MS);
  digitalWrite(LED_RED_PIN, LOW);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Timeout audio");
  display.display();
  return false;
}

// ==== SD CARD HELPERS ====
String meanFilenameForWord(int wordIndex) {
  return "/" + words[wordIndex] + "_mean.csv";
}

void saveMeanToSD(int wordIndex, const float meanVector[NUM_SENSORS]) {
  String filename = meanFilenameForWord(wordIndex);
  File file = SD.open(filename, FILE_WRITE);
  if (!file) {
    logMessage("[SD] Falha ao abrir arquivo para escrita: " + filename);
    return;
  }

  file.println("Sensor,Valor");
  for (int j = 0; j < NUM_SENSORS; ++j) {
    file.println(String(j) + "," + String(meanVector[j], 5));
  }
  file.close();
  logMessage("[SD] Media salva em " + filename);
}

bool loadMeanFromSD(int wordIndex, float meanVector[NUM_SENSORS]) {
  String filename = meanFilenameForWord(wordIndex);
  File file = SD.open(filename, FILE_READ);
  if (!file) {
    logMessage("[SD] Media nao encontrada para '" + words[wordIndex] + "'");
    for (int j = 0; j < NUM_SENSORS; ++j) {
      meanVector[j] = NAN;
    }
    return false;
  }

  int lineNumber = 0;
  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.isEmpty()) {
      continue;
    }
    if (lineNumber == 0) {
      lineNumber++;
      continue; // cabeçalho
    }
    int commaIndex = line.indexOf(',');
    if (commaIndex < 0) {
      continue;
    }
    int sensor = line.substring(0, commaIndex).toInt();
    float value = line.substring(commaIndex + 1).toFloat();
    if (sensor >= 0 && sensor < NUM_SENSORS) {
      meanVector[sensor] = value;
    }
    lineNumber++;
  }
  file.close();

  String debug = "=== Media carregada para '" + words[wordIndex] + "' ===\n";
  for (int j = 0; j < NUM_SENSORS; ++j) {
    debug += "Sensor " + String(j) + ": " + String(meanVector[j], 5) + "\n";
  }
  logMessage(debug);
  return true;
}

void resetCalibrationState() {
  captureCount = 0;
  for (int i = 0; i < NUM_WORDS; ++i) {
    capturesPerWord[i] = 0;
    calibrationSampleCounts[i] = 0;
    for (int j = 0; j < NUM_SENSORS; ++j) {
      calibrationSums[i][j] = 0.0f;
    }
  }
  mpuCalibrated = false;
}

// ==== CAPTURA DE MOVIMENTO ====
void captureMovement() {
  int wordIndex = captureCount / CAPTURES_PER_WORD;
  int captureIndex = capturesPerWord[wordIndex];

  String header = "Capturando movimento " + String(captureIndex + 1) + " para: " + words[wordIndex];
  logMessage(header);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Capturando:");
  display.setCursor(0, 10);
  display.println(words[wordIndex]);
  display.setCursor(0, 20);
  display.print("Mov ");
  display.print(captureIndex + 1);
  display.display();

  float captureSums[NUM_SENSORS] = {0.0f};
  bool mpuError = false;

  for (int i = 0; i < SAMPLES; ++i) {
    dataMatrix[i][0] = analogRead(FLEX_PIN) / 4095.0f;
    captureSums[0] += dataMatrix[i][0];

    mpu.update();
    float accX = mpu.getAccX();
    float accY = mpu.getAccY();
    float accZ = mpu.getAccZ();
    if (accX != 0.0f || accY != 0.0f || accZ != 0.0f) {
      dataMatrix[i][1] = accX / 2.0f;
      dataMatrix[i][2] = accY / 2.0f;
      dataMatrix[i][3] = accZ / 2.0f;
    } else {
      mpuError = true;
      dataMatrix[i][1] = dataMatrix[i][2] = dataMatrix[i][3] = 0.0f;
    }

    captureSums[1] += dataMatrix[i][1];
    captureSums[2] += dataMatrix[i][2];
    captureSums[3] += dataMatrix[i][3];

    delay(40);
  }

  for (int j = 0; j < NUM_SENSORS; ++j) {
    calibrationSums[wordIndex][j] += captureSums[j];
  }
  calibrationSampleCounts[wordIndex] += SAMPLES;

  capturesPerWord[wordIndex]++;
  captureCount++;

  logMessage("Captura " + String(captureIndex + 1) + " concluida para '" + words[wordIndex] + "'");

  if (mpuError) {
    logMessage("[MPU] Algumas leituras retornaram zero. Verifique sensores.");
  }

  if (capturesPerWord[wordIndex] == CAPTURES_PER_WORD) {
    float meanVector[NUM_SENSORS];
    for (int j = 0; j < NUM_SENSORS; ++j) {
      if (calibrationSampleCounts[wordIndex] > 0) {
        meanVector[j] = calibrationSums[wordIndex][j] /
                        static_cast<float>(calibrationSampleCounts[wordIndex]);
      } else {
        meanVector[j] = 0.0f;
      }
    }
    saveMeanToSD(wordIndex, meanVector);
  }

  if (captureCount >= TOTAL_CAPTURES) {
    logMessage("Calibracao concluida! Entrando em modo traducao.");
    captureMode = false;
    lastModeState = false;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Modo traducao");
    display.setCursor(0, 10);
    display.println("Pressione botao");
    display.display();
    playIndex(FIRST_IDX);
  }
}

// ==== RECONHECIMENTO ====
void computeCaptureMean(float meanVector[NUM_SENSORS]) {
  for (int j = 0; j < NUM_SENSORS; ++j) {
    meanVector[j] = 0.0f;
  }

  for (int i = 0; i < SAMPLES; ++i) {
    for (int j = 0; j < NUM_SENSORS; ++j) {
      meanVector[j] += dataMatrix[i][j];
    }
  }

  for (int j = 0; j < NUM_SENSORS; ++j) {
    meanVector[j] /= static_cast<float>(SAMPLES);
  }
}

float calculateDistance(const float vec1[NUM_SENSORS], const float vec2[NUM_SENSORS]) {
  float sum = 0.0f;
  for (int j = 0; j < NUM_SENSORS; ++j) {
    float diff = vec1[j] - vec2[j];
    sum += diff * diff;
  }
  return sqrtf(sum);
}

void readAndRecognize() {
  if (!waitingNext) {
    if (millis() - lastPlayMillis < PLAY_GAP_MS) {
      logMessage("[Gesture] Aguardando termino do audio atual...");
      return;
    }
    waitingNext = true;
  }

  logMessage("Capturando movimento para traducao...");

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Reconhecendo...");
  display.display();

  bool mpuError = false;
  for (int i = 0; i < SAMPLES; ++i) {
    dataMatrix[i][0] = analogRead(FLEX_PIN) / 4095.0f;

    mpu.update();
    float accX = mpu.getAccX();
    float accY = mpu.getAccY();
    float accZ = mpu.getAccZ();
    if (accX != 0.0f || accY != 0.0f || accZ != 0.0f) {
      dataMatrix[i][1] = accX / 2.0f;
      dataMatrix[i][2] = accY / 2.0f;
      dataMatrix[i][3] = accZ / 2.0f;
    } else {
      mpuError = true;
      dataMatrix[i][1] = dataMatrix[i][2] = dataMatrix[i][3] = 0.0f;
    }

    delay(40);
  }

  float captureMean[NUM_SENSORS];
  computeCaptureMean(captureMean);

  String captureDebug = "=== Media capturada ===\n";
  for (int j = 0; j < NUM_SENSORS; ++j) {
    captureDebug += "Sensor " + String(j) + ": " + String(captureMean[j], 5) + "\n";
  }
  logMessage(captureDebug);

  float distances[NUM_WORDS];
  for (int w = 0; w < NUM_WORDS; ++w) {
    float storedMean[NUM_SENSORS];
    bool haveMean = loadMeanFromSD(w, storedMean);
    if (!haveMean) {
      distances[w] = INFINITY;
      continue;
    }
    distances[w] = calculateDistance(captureMean, storedMean);
  }

  int bestIndex = -1;
  float bestDistance = INFINITY;
  for (int w = 0; w < NUM_WORDS; ++w) {
    if (distances[w] < bestDistance) {
      bestDistance = distances[w];
      bestIndex = w;
    }
  }

  String distLog = "=== Distancias ===\n";
  for (int w = 0; w < NUM_WORDS; ++w) {
    distLog += words[w] + ": " + (isinf(distances[w]) ? String("N/A") : String(distances[w], 5)) + "\n";
  }
  logMessage(distLog);

  const float RECOGNITION_THRESHOLD = 0.35f; // ajustado empiricamente
  if (bestIndex >= 0 && bestDistance < RECOGNITION_THRESHOLD && !mpuError) {
    String success = "Reconhecida: " + words[bestIndex] + " (" + String(bestDistance, 5) + ")";
    logMessage(success);
    digitalWrite(LED_GREEN_PIN, HIGH);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Reconhecida:");
    display.setCursor(0, 10);
    display.println(words[bestIndex]);
    display.display();
    playIndex(bestIndex + 1);
    delay(LED_ON_MS / 2);
    digitalWrite(LED_GREEN_PIN, LOW);
  } else {
    logMessage("Gestos nao correspondem a palavras calibradas.");
    digitalWrite(LED_RED_PIN, HIGH);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Nao reconhecida");
    display.display();
    delay(LED_ON_MS / 2);
    digitalWrite(LED_RED_PIN, LOW);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Pressione botao");
  display.setCursor(0, 10);
  display.println("para novo gesto");
  display.display();
}

// ==== SETUP ====
void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  logMessage("\n=== Luva Tradutora v1.26 ===");

  pinMode(FLEX_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);

  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
  Wire.setClock(100000);

  byte status = mpu.begin();
  if (status != 0) {
    logMessage("[MPU] Falha ao inicializar. Status: " + String(status));
    while (true) {
      digitalWrite(LED_RED_PIN, !digitalRead(LED_RED_PIN));
      delay(250);
    }
  }
  logMessage("[MPU] Inicializado com sucesso");

  DFSerial.begin(DF_BAUD, SERIAL_8N1, DF_RX_PIN, DF_TX_PIN);
  logMessage("[DFP] Inicializando...");
  if (!dfp.begin(DFSerial, true, true)) {
    logMessage("[DFP] Falha ao inicializar");
    while (true) {
      digitalWrite(LED_RED_PIN, !digitalRead(LED_RED_PIN));
      delay(250);
    }
  }
  dfp.volume(VOLUME);
  logMessage("[DFP] Volume atual: " + String(dfp.readVolume()));

  spiSD.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN, spiSD)) {
    logMessage("[SD] Falha ao inicializar");
    while (true) {
      digitalWrite(LED_RED_PIN, !digitalRead(LED_RED_PIN));
      delay(250);
    }
  }
  logMessage("[SD] Cartao inicializado");

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    logMessage("[OLED] Falha ao inicializar");
    while (true) {
      digitalWrite(LED_RED_PIN, !digitalRead(LED_RED_PIN));
      delay(250);
    }
  }
  display.setRotation(2);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Luva Tradutora");
  display.setCursor(0, 10);
  display.println("Inicializando...");
  display.display();

  BLEDevice::init("LuvaTradutora");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  BLEDevice::startAdvertising();
  logMessage("[BLE] Servidor iniciado");

  for (int offset = 0; offset < NUM_WORDS; offset += 3) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Palavras:");
    int y = 10;
    for (int i = offset; i < offset + 3 && i < NUM_WORDS; ++i) {
      display.setCursor(0, y);
      display.println(words[i]);
      y += 10;
    }
    display.display();
    delay(2000);
  }

  for (int i = 0; i < 3; ++i) {
    digitalWrite(LED_GREEN_PIN, HIGH);
    digitalWrite(LED_RED_PIN, HIGH);
    delay(150);
    digitalWrite(LED_GREEN_PIN, LOW);
    digitalWrite(LED_RED_PIN, LOW);
    delay(150);
  }

  lastModeState = digitalRead(MODE_SWITCH_PIN) == HIGH;
  captureMode = lastModeState;
  if (captureMode) {
    logMessage("[Setup] Modo calibracao ativo. Pressione o botao.");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Modo calibracao");
    display.setCursor(0, 10);
    display.println("Pressione botao");
    display.display();
  } else {
    logMessage("[Setup] Modo traducao ativo. Pressione o botao para reconhecer gestos.");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Modo traducao");
    display.setCursor(0, 10);
    display.println("Pressione botao");
    display.display();
    playIndex(FIRST_IDX);
  }
}

// ==== LOOP PRINCIPAL ====
void loop() {
  if (dfp.available()) {
    uint8_t type = dfp.readType();
    int value = dfp.read();
    printDFEvent(type, value);
  }

  bool currentModeState = digitalRead(MODE_SWITCH_PIN) == HIGH;
  if (currentModeState != lastModeState) {
    captureMode = currentModeState;
    if (captureMode) {
      logMessage("[Mode] Entrou em modo calibracao");
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Modo calibracao");
      display.setCursor(0, 10);
      display.println("Pressione botao");
      display.display();
      resetCalibrationState();
    } else {
      logMessage("[Mode] Entrou em modo traducao");
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Modo traducao");
      display.setCursor(0, 10);
      display.println("Pressione botao");
      display.display();
      waitingNext = true;
      playIndex(FIRST_IDX);
    }
    lastModeState = currentModeState;
    delay(250);
  }

  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(30); // debounce inicial
    while (digitalRead(BUTTON_PIN) == LOW) {
      delay(10);
    }
    delay(30); // debounce final

    if (captureMode && !mpuCalibrated) {
      logMessage("[MPU] Calibrando offsets. Mantenha a luva estatica...");
      mpu.calcOffsets(true, true);
      logMessage("[MPU] Calibracao concluida.");
      mpuCalibrated = true;
      digitalWrite(LED_GREEN_PIN, HIGH);
      delay(400);
      digitalWrite(LED_GREEN_PIN, LOW);
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("MPU calibrado");
      display.display();
      delay(1000);
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Modo calibracao");
      display.setCursor(0, 10);
      display.println("Pressione botao");
      display.display();
    } else if (captureMode) {
      captureMovement();
    } else {
      readAndRecognize();
    }
  }
}
