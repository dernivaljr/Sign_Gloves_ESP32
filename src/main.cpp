/*
 * === Projeto: Luva Tradutora de Libras ESP32-C3 ===
 * Descrição: Sistema para reconhecimento de gestos de Libras usando sensores flexíveis, MPU6050, DFPlayer e BLE.
 * Autor: Desenvolvido com Grok (xAI)
 * Data de Início: Setembro 2025
 *
 * === Histórico de Versões ===
 * v0.1 (Inicial - Setembro 2025): Versão base com 3 palavras (MAE, PAI, AGUA), BLE, OLED, MPU6050 e DFPlayer.
 * v0.2 (Versão com Adafruit SSD1306): Migração para Adafruit_GFX e SSD1306 para compatibilidade, sem acentos.
 * v1.0 (Original): Código base com 10 palavras, travamento na inicialização.
 * v1.1 (550e8400-e29b-41d4-a716-446655440000): Reduzido para 3 palavras, ajustes em memória (SAMPLES=50), e melhor depuração.
 * v1.2 (85f43d5d-e922-49bf-bcb3-1af1540847d0, 1a3015d3-1698-4826-bbc4-28f75ff42955): Correção de debug e display.
 * v1.3 (85f43d5d-e922-49bf-bcb3-1af1540847d0, 4d1c7904-ae7d-4775-88fa-f247b7f2a938): Otimização de debug e tempo.
 * v1.4 (85f43d5d-e922-49bf-bcb3-1af1540847d0, e5401e7f-72e8-4257-a75b-16901a6ed40b): Redução de DF_TIMEOUT_MS para 2000ms.
 * v1.5 (85f43d5d-e922-49bf-bcb3-1af1540847d0, b8e9f7d2-9d4f-4e2c-8a9c-3f0b2a4e9d1c): Ajuste de OLED para 2 palavras e adição de cabeçalho.
 * v1.6 (85f43d5d-e922-49bf-bcb3-1af1540847d0, fe9dfaab-0013-4167-a804-358822eb6687): Correção de exibição de palavras e áudio de teste.
 * v1.7 (85f43d5d-e922-49bf-bcb3-1af1540847d0, d1c7e9b9-8a3f-4e6a-9e2c-5f0d1a3e4b5c): Correção do timeout do DFPlayer.
 * v1.8 (85f43d5d-e922-49bf-bcb3-1af1540847d0, b2f9e7a3-6c1d-4f8e-8e9a-2d3b5c7f1e0d): Otimização de áudio e BLE.
 * v1.9 (85f43d5d-e922-49bf-bcb3-1af1540847d0, c4e9d2f1-7b3a-4f8d-9c1e-5a6b8d2e3f9a): Correção de escopo de logMessage.
 * v1.10 (85f43d5d-e922-49bf-bcb3-1af1540847d0, 61863272-7677-4f9c-b8da-c78185baaf55): Correção de array words e validação de arquivos.
 * v1.11 (85f43d5d-e922-49bf-bcb3-1af1540847d0, a9e5f2b0-3d4e-4f9a-9c7d-2b1e8c4f5d6a): Adição de suporte a SD para calibração incremental.
 * v1.12 (85f43d5d-e922-49bf-bcb3-1af1540847d0, f3b7e9c1-4a2d-4e8e-9b2a-7c5d8e1f2a3b): Adaptação para SD integrado ao DFPlayer, redução de amostras.
 * v1.13 (85f43d5d-e922-49bf-bcb3-1af1540847d0, e7f9c2a1-5d8e-4b2a-8f9c-3a1b5d2e7f9a): Uso de EEPROM para salvar médias euclidianas.
 * v1.14 (85f43d5d-e922-49bf-bcb3-1af1540847d0, a1b2c3d4-e5f6-4g7h-8i9j-0k1l2m3n4o5p): Otimização de memória com SAMPLES=50 e alocação dinâmica.
 * v1.15 (85f43d5d-e922-49bf-bcb3-1af1540847d0, f7e9d1a2-3b4c-4d5e-9f8a-0c6b7d2e3f9b): Correção de acesso a matriz dinâmica.
 * v1.16 (85f43d5d-e922-49bf-bcb3-1af1540847d0, b9c8e7f6-3d2a-4f9e-8b5c-1e4f7a9d0b2c): Atualização da lista de palavras.
 * v1.17 (85f43d5d-e922-49bf-bcb3-1af1540847d0, ad163a5c-4511-45c3-ba49-5a72350e96c6): Ajuste para 2 segundos de captura (100 amostras).
 * v1.18 (85f43d5d-e922-49bf-bcb3-1af1540847d0, e9f2c8d1-6a3b-4e9d-8c7f-2b5d1e4f9a0b): Correção da declaração de tempMatrix em captureMovement.
 * v1.19 (85f43d5d-e922-49bf-bcb3-1af1540847d0, f4d2e9c0-8a3b-4f9d-9e7f-1b5c2d3e4f6a): Redução de SAMPLES para 50 para otimizar memória.
 * v1.20 (85f43d5d-e922-49bf-bcb3-1af1540847d0, c7e9d2f1-8a3b-4f9d-9e7f-2b5c1d3e4f6a): Redução da frequência para 25 Hz com 2 segundos.
 * v1.21 (85f43d5d-e922-49bf-bcb3-1af1540847d0, b9c8e7f6-3d2a-4f9e-8b5c-1e4f7a9d0b2c): Depuração de distâncias iguais no modo tradução.
 * v1.22 (85f43d5d-e922-49bf-bcb3-1af1540847d0, f9a2b3c4-d5e6-4f7g-8h9i-j0k1l2m3n4o5): Depuração de médias na EEPROM e calibração.
 * v1.23 (85f43d5d-e922-49bf-bcb3-1af1540847d0, 1a5b5d0a-2c1d-3e1f-4g2h-5i3j4k5l6m7n): Ajuste de precisão nos valores e calibração do MPU no modo calibração.
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
#include <EEPROM.h>

// Declaração forward para logMessage
void logMessage(const String& message);

// ==== PINOS E AJUSTES ====
constexpr int DF_RX_PIN = 5;     // ESP32-C3 RX1 <- DFPlayer TX
constexpr int DF_TX_PIN = 6;     // ESP32-C3 TX1 -> DFPlayer RX
constexpr int MPU_SDA_PIN = 8;   // I2C SDA (MPU6050 e OLED)
constexpr int MPU_SCL_PIN = 9;   // I2C SCL (MPU6050 e OLED)
constexpr int FLEX_PINS[] = {0, 1, 2, 3, 4}; // GPIO 0-4 (sensores flexíveis)
constexpr int BUTTON_PIN = 10;   // Botão de captura
constexpr int MODE_SWITCH_PIN = 7; // Switch (HIGH = calibração, LOW = tradução)
constexpr int LED_GREEN_PIN = 20; // LED verde (palavra reconhecida)
constexpr int LED_RED_PIN = 21;  // LED vermelho (erro/não reconhecida)
constexpr uint32_t DF_BAUD = 9600;
constexpr uint8_t VOLUME = 25;   // 0..30 (máximo) 25 provisóriamente
constexpr uint16_t FIRST_IDX = 1; // /mp3/0001.mp3 (ola)
constexpr uint16_t LAST_IDX = 10; // /mp3/0010.mp3 (ajustado para 10 arquivos)
constexpr uint32_t DF_TIMEOUT_MS = 3000; // Aumentado para 3000ms
constexpr uint32_t LED_ON_MS = 1000; // Tempo LEDs acesos
constexpr int SAMPLES = 50;      // 50 amostras em 2 segundos (25 Hz)
constexpr int NUM_SENSORS = 11;  // 5 flex + 3 accel + 3 gyro
constexpr int NUM_WORDS = 10;    // Ajustado para 10 palavras
constexpr int CAPTURES_PER_WORD = 3; // 3 capturas por palavra
constexpr int TOTAL_CAPTURES = NUM_WORDS * CAPTURES_PER_WORD; // 30 capturas
const int BUFFER_SIZE = 200;     // Tamanho máximo do buffer de logs
#define SCREEN_WIDTH 128 // Largura do OLED
#define SCREEN_HEIGHT 32 // Altura do OLED
#define OLED_RESET -1    // Reset pin (não usado, -1 se não conectado)

// ==== OBJETOS GLOBAIS ====
HardwareSerial DFSerial(1);
DFRobotDFPlayerMini dfp;
MPU6050 mpu(Wire);
bool mpuInitialized = false;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Objeto OLED

// ==== MATRIZES E ESTADO ====
float dataMatrix[SAMPLES][NUM_SENSORS];
int captureCount = 0;
bool captureMode = true;
int capturesPerWord[NUM_WORDS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Inicializado para 10 palavras
String words[NUM_WORDS] = {"ola", "por favor", "obrigado", "bom dia", "e", "meu", "nome", "boa tarde", "ajuda", "boa noite"}; // Nova lista de palavras sem acentos
uint16_t currentIndex = FIRST_IDX;
uint32_t lastPlayMillis = 0;
bool waitingNext = true;
bool lastModeState = true;
char logBuffer[BUFFER_SIZE] = ""; // Buffer para acumular logs
int bufferIndex = 0;
int displayOffset = 0; // Para rolagem das palavras no OLED
bool mpuCalibrated = false; // Flag para calibração do MPU
float wordAccum[NUM_WORDS][NUM_SENSORS] = {{0.0f}};
int wordSampleCount[NUM_WORDS] = {0};

// ==== Bluetooth BLE ===
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("[BLE] Dispositivo conectado");
    bufferIndex = 0; // Limpa buffer ao conectar
    logBuffer[0] = '\0';
    logMessage("[BLE] Enviando logs via BLE iniciados");
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("[BLE] Dispositivo desconectado");
    BLEDevice::startAdvertising();
  }
};

// ==== FUNÇÃO DE LOG (ACUMULA E ENVIA COM DELAY) ====
void logMessage(const String& message) {
  Serial.println(message);

  if (deviceConnected) {
    int msgLength = message.length();
    if (bufferIndex + msgLength + 1 < BUFFER_SIZE) {
      strcpy(logBuffer + bufferIndex, message.c_str());
      bufferIndex += msgLength;
      logBuffer[bufferIndex++] = '\n';
      logBuffer[bufferIndex] = '\0';

      pCharacteristic->setValue(logBuffer);
      pCharacteristic->notify();
      delay(10);
      Serial.println("[BLE] Log enviado: " + message); // Confirmação de envio
    } else {
      bufferIndex = 0;
      strcpy(logBuffer, message.c_str());
      bufferIndex = msgLength;
      logBuffer[bufferIndex] = '\0';
      pCharacteristic->setValue(logBuffer);
      pCharacteristic->notify();
      delay(10);
      Serial.println("[BLE] Log enviado: " + message); // Confirmação de envio
    }
  }
}

// ==== LOG DE EVENTOS DO DFPLAYER ====
void printDFEvent(uint8_t type, int value) {
  if (type == 11 && value == 2) return; // Ignorar evento desconhecido repetitivo
  String message;
  switch (type) {
    case DFPlayerCardInserted:  message = F("[DFP] Cartao SD inserido"); break;
    case DFPlayerCardRemoved:   message = F("[DFP] Cartao SD removido"); break;
    case DFPlayerCardOnline:    message = F("[DFP] Cartao SD online"); break;
    case DFPlayerUSBInserted:   message = F("[DFP] USB inserido"); break;
    case DFPlayerUSBRemoved:    message = F("[DFP] USB removido"); break;
    case DFPlayerUSBOnline:     message = F("[DFP] USB online"); break;
    case DFPlayerPlayFinished:  message = "[DFP] Terminado faixa: " + String(value);
                                waitingNext = true;
                                lastPlayMillis = millis();
                                logMessage("[DFP] Faixa terminada: " + String(value));
                                break;
    case DFPlayerError:
      message = F("[DFP] Erro: ");
      switch (value) {
        case Busy:             message += F("Ocupado"); break;
        case Sleeping:         message += F("Dormindo"); break;
        case SerialWrongStack: message += F("Pilha serial errada"); break;
        case CheckSumNotMatch: message += F("Checksum nao corresponde"); break;
        case FileIndexOut:     message += F("Indice de arquivo fora"); break;
        case FileMismatch:     message += F("Arquivo nao corresponde"); break;
        case Advertise:        message += F("Publicidade"); break;
        default:               message += "Desconhecido (" + String(value) + ")"; break;
      }
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

// ==== TOCAR POR ÍNDICE (/mp3/000N.mp3) ====
bool playIndex(uint16_t idx) {
  if (idx < FIRST_IDX || idx > LAST_IDX) {
    String msg = "[DFP] Indice fora do range";
    logMessage(msg);
    digitalWrite(LED_RED_PIN, HIGH);
    delay(LED_ON_MS);
    digitalWrite(LED_RED_PIN, LOW);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Indice fora do range");
    display.display();
    return false;
  }

  String msg = "[DFP] Enviando comando para /mp3/" + String(idx, DEC) + ".mp3 (" + (idx <= NUM_WORDS ? words[idx - 1] : "desconhecido") + ")";
  logMessage(msg);
  dfp.playMp3Folder(idx);
  waitingNext = false;
  currentIndex = idx;
  lastPlayMillis = millis();

  unsigned long startTime = millis();
  while (millis() - startTime < DF_TIMEOUT_MS) {
    if (dfp.available()) {
      uint8_t type = dfp.readType();
      int value = dfp.read();
      String eventMsg = "[DFP] Evento recebido: tipo=" + String(type) + ", valor=" + String(value);
      logMessage(eventMsg);
      if (type == DFPlayerPlayFinished) {
        logMessage("[DFP] Faixa terminada detectada");
        return true; // Aceita qualquer término como sucesso
      }
    }
  }
  String timeoutMsg = "[DFP] Timeout aguardando resposta do DFPlayer após " + String(DF_TIMEOUT_MS) + "ms";
  logMessage(timeoutMsg);
  digitalWrite(LED_RED_PIN, HIGH);
  delay(LED_ON_MS);
  digitalWrite(LED_RED_PIN, LOW);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Timeout DFPlayer");
  display.display();
  waitingNext = true;
  return false;
}

// ==== CALCULAR MÉDIA POR SENSOR ====
void calculateMean(int wordIndex, float meanVector[NUM_SENSORS]) {
  if (wordIndex < 0 || wordIndex >= NUM_WORDS) {
    memset(meanVector, 0, NUM_SENSORS * sizeof(float));
    return;
  }

  if (wordSampleCount[wordIndex] == 0) {
    memset(meanVector, 0, NUM_SENSORS * sizeof(float));
  } else {
    for (int j = 0; j < NUM_SENSORS; j++) {
      meanVector[j] = wordAccum[wordIndex][j] / static_cast<float>(wordSampleCount[wordIndex]);
    }
  }

  String meanDebug = "=== Media Calculada para '" + words[wordIndex] + "' ===\n";
  for (int j = 0; j < NUM_SENSORS; j++) {
    meanDebug += "Sensor " + String(j) + ": " + String(meanVector[j], 4) + "\n";
  }
  logMessage(meanDebug);
}

// ==== SALVAR MÉDIA NA EEPROM ====
void saveMeanToEEPROM(int wordIndex, const float meanVector[NUM_SENSORS]) {
  int address = wordIndex * (NUM_SENSORS * sizeof(float));
  EEPROM.put(address, meanVector);
  EEPROM.commit();
  logMessage("[EEPROM] Salva media para '" + words[wordIndex] + "' em endereco " + String(address));
}

// ==== CARREGAR MÉDIA DA EEPROM ====
void loadMeanFromEEPROM(int wordIndex, float meanVector[NUM_SENSORS]) {
  int address = wordIndex * (NUM_SENSORS * sizeof(float));
  EEPROM.get(address, meanVector);
  String meanDebug = "=== Media Carregada para '" + words[wordIndex] + "' ===\n";
  for (int j = 0; j < NUM_SENSORS; j++) {
    meanDebug += "Sensor " + String(j) + ": " + String(meanVector[j], 4) + "\n";
  }
  logMessage(meanDebug);
}

// ==== CAPTURAR MOVIMENTO ====
void captureMovement() {
  int wordIndex = captureCount / CAPTURES_PER_WORD;
  String currentWord = words[wordIndex];
  int captureIndex = capturesPerWord[wordIndex];

  String msg = "Capturando movimento " + String(captureIndex + 1) + " para a palavra: " + currentWord;
  logMessage(msg);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Capturando:");
  display.setCursor(0, 10);
  display.println(currentWord);
  display.setCursor(60, 10);
  display.print("Mov ");
  display.print(captureIndex + 1);
  display.display();

  bool mpuError = false;
  for (int i = 0; i < SAMPLES; i++) {
    dataMatrix[i][0] = analogRead(FLEX_PINS[0]) / 4095.0f;
    dataMatrix[i][1] = analogRead(FLEX_PINS[1]) / 4095.0f;
    dataMatrix[i][2] = analogRead(FLEX_PINS[2]) / 4095.0f;
    dataMatrix[i][3] = analogRead(FLEX_PINS[3]) / 4095.0f;
    dataMatrix[i][4] = analogRead(FLEX_PINS[4]) / 4095.0f;

    mpu.update();
    if (mpu.getAccX() != 0 || mpu.getAccY() != 0 || mpu.getAccZ() != 0) {
      dataMatrix[i][5] = mpu.getAccX() / 2.0f; // Ajustado para maior precisão (±16g padrão)
      dataMatrix[i][6] = mpu.getAccY() / 2.0f;
      dataMatrix[i][7] = mpu.getAccZ() / 2.0f;
      dataMatrix[i][8] = mpu.getGyroX() / 250.0f; // Ajustado para maior precisão (±250°/s)
      dataMatrix[i][9] = mpu.getGyroY() / 250.0f;
      dataMatrix[i][10] = mpu.getGyroZ() / 250.0f;
    } else {
      mpuError = true;
      String errMsg = "Erro ao ler MPU6050 na amostra " + String(i);
      logMessage(errMsg);
      digitalWrite(LED_RED_PIN, HIGH);
      delay(LED_ON_MS);
      digitalWrite(LED_RED_PIN, LOW);
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println("Erro MPU6050");
      display.display();
      for (int j = 5; j < NUM_SENSORS; j++) {
        dataMatrix[i][j] = 0.0f;
      }
    }
    delay(40); // 40ms por amostra para 50 amostras em 2 segundos (25 Hz)
  }

  String matrixMsg = "=== Matriz Capturada ===\n";
  for (int i = 0; i < SAMPLES; i++) {
    matrixMsg += "Amostra " + String(i) + ": ";
    for (int j = 0; j < NUM_SENSORS; j++) {
      matrixMsg += String(dataMatrix[i][j], 4) + " ";
    }
    matrixMsg += "\n";
  }
  logMessage(matrixMsg);

  for (int j = 0; j < NUM_SENSORS; j++) {
    float columnSum = 0.0f;
    for (int i = 0; i < SAMPLES; i++) {
      columnSum += dataMatrix[i][j];
    }
    wordAccum[wordIndex][j] += columnSum;
  }
  wordSampleCount[wordIndex] += SAMPLES;

  String confirmMsg = "Captura " + String(captureIndex + 1) + " da palavra " + currentWord + " OK";
  logMessage(confirmMsg);

  capturesPerWord[wordIndex]++;
  captureCount++;

  if (capturesPerWord[wordIndex] == CAPTURES_PER_WORD) {
    float meanVector[NUM_SENSORS] = {0.0f};
    calculateMean(wordIndex, meanVector);
    saveMeanToEEPROM(wordIndex, meanVector);
  }

  if (captureCount >= TOTAL_CAPTURES) {
    String endMsg = "Captura de 3 movimentos por palavra concluida! Entrando em modo de traducao.";
    logMessage(endMsg);
    captureMode = false;
    lastModeState = false;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Modo Traducao");
    display.setCursor(0, 10);
    display.println("Pressione botao");
    display.display();
    playIndex(FIRST_IDX); // Toca áudio de teste após calibração
  }
}

// ==== CALCULAR DISTÂNCIA EUCLIDIANA ====
float calculateDistance(float matrix1[SAMPLES][NUM_SENSORS], const float meanVector2[NUM_SENSORS]) {
  float distance = 0.0f;
  for (int i = 0; i < SAMPLES; i++) {
    for (int j = 0; j < NUM_SENSORS; j++) {
      float diff = matrix1[i][j] - meanVector2[j];
      distance += diff * diff;
    }
  }
  float normalizedDistance = sqrt(distance / (SAMPLES * NUM_SENSORS));
  logMessage("[Debug] Distancia calculada (bruta): " + String(distance, 4) + ", normalizada: " + String(normalizedDistance, 4));
  return normalizedDistance;
}

// ==== MODO DE LEITURA E RECONHECIMENTO ====
void readAndRecognize() {
  if (!waitingNext) {
    if (millis() - lastPlayMillis > DF_TIMEOUT_MS) {
      String msg = "[DFP] Timeout aguardando audio. Liberando para nova tentativa.";
      logMessage(msg);
      digitalWrite(LED_RED_PIN, HIGH);
      delay(LED_ON_MS / 2);
      digitalWrite(LED_RED_PIN, LOW);
      waitingNext = true;
    } else {
      String msg = "[Gesture] Aguardando termino do audio anterior...";
      logMessage(msg);
      return;
    }
  }

  String msg = "Capturando novo movimento para reconhecimento...";
  logMessage(msg);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Reconhecendo...");
  display.display();

  bool mpuError = false;
  for (int i = 0; i < SAMPLES; i++) {
    dataMatrix[i][0] = analogRead(FLEX_PINS[0]) / 4095.0f;
    dataMatrix[i][1] = analogRead(FLEX_PINS[1]) / 4095.0f;
    dataMatrix[i][2] = analogRead(FLEX_PINS[2]) / 4095.0f;
    dataMatrix[i][3] = analogRead(FLEX_PINS[3]) / 4095.0f;
    dataMatrix[i][4] = analogRead(FLEX_PINS[4]) / 4095.0f;

    mpu.update();
    if (mpu.getAccX() != 0 || mpu.getAccY() != 0 || mpu.getAccZ() != 0) {
      dataMatrix[i][5] = mpu.getAccX() / 2.0f; // Ajustado para maior precisão (±16g padrão)
      dataMatrix[i][6] = mpu.getAccY() / 2.0f;
      dataMatrix[i][7] = mpu.getAccZ() / 2.0f;
      dataMatrix[i][8] = mpu.getGyroX() / 250.0f; // Ajustado para maior precisão (±250°/s)
      dataMatrix[i][9] = mpu.getGyroY() / 250.0f;
      dataMatrix[i][10] = mpu.getGyroZ() / 250.0f;
    } else {
      mpuError = true;
      String errMsg = "Erro ao ler MPU6050 na amostra " + String(i);
      logMessage(errMsg);
      digitalWrite(LED_RED_PIN, HIGH);
      delay(LED_ON_MS / 2);
      digitalWrite(LED_RED_PIN, LOW);
      for (int j = 5; j < NUM_SENSORS; j++) {
        dataMatrix[i][j] = 0.0f;
      }
    }
    delay(40); // 40ms por amostra para 50 amostras em 2 segundos (25 Hz)
  }

  String matrixDebug = "=== Matriz Capturada para Reconhecimento ===\n";
  for (int i = 0; i < SAMPLES; i++) {
    matrixDebug += "Amostra " + String(i) + ": ";
    for (int j = 0; j < NUM_SENSORS; j++) {
      matrixDebug += String(dataMatrix[i][j], 4) + " ";
    }
    matrixDebug += "\n";
  }
  logMessage(matrixDebug);

  float avgDistances[NUM_WORDS] = {0.0f};
  for (int w = 0; w < NUM_WORDS; w++) {
    float loadedMean[NUM_SENSORS] = {0.0f};
    loadMeanFromEEPROM(w, loadedMean);
    avgDistances[w] = calculateDistance(dataMatrix, loadedMean);
  }

  String distMsg = "=== Distancias Medias ===\n";
  for (int w = 0; w < NUM_WORDS; w++) {
    distMsg += "Distancia para " + words[w] + ": " + String(avgDistances[w], 4) + "\n";
  }
  logMessage(distMsg);

  float minDist = avgDistances[0];
  int minIndex = 0;
  for (int w = 1; w < NUM_WORDS; w++) {
    if (avgDistances[w] < minDist) {
      minDist = avgDistances[w];
      minIndex = w;
    }
  }

  if (minDist < 10.0f && !mpuError) {
    String recMsg = "Palavra reconhecida: " + words[minIndex];
    logMessage(recMsg);
    digitalWrite(LED_GREEN_PIN, HIGH);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Reconhecida:");
    display.setCursor(0, 10);
    display.println(words[minIndex]);
    display.display();
    playIndex(minIndex + 1);
    delay(LED_ON_MS / 2);
    digitalWrite(LED_GREEN_PIN, LOW);
  } else {
    String noRecMsg = "Palavra nao reconhecida ou erro no MPU6050.";
    logMessage(noRecMsg);
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

  String readyMsg = "Pronto para nova captura. Pressione o botao.";
  logMessage(readyMsg);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Pronto para");
  display.setCursor(0, 10);
  display.println("nova captura");
  display.display();
}

// ==== SETUP ====
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  logMessage("\n=== Luva Tradutora de Libras ESP32-C3 ===");

  // Configurar pinos
  for (int i = 0; i < 5; i++) {
    pinMode(FLEX_PINS[i], INPUT);
  }
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);
  logMessage("[Setup] Pinos configurados.");

  // Inicializar I2C
  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
  Wire.setClock(100000);
  logMessage("[Setup] I2C inicializado.");

  // Inicializar OLED Adafruit
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Tente 0x3D se 0x3C falhar
    logMessage("[OLED] Erro ao inicializar display!");
    while (1) {
      digitalWrite(LED_RED_PIN, HIGH);
      delay(500);
      digitalWrite(LED_RED_PIN, LOW);
      delay(500);
    }
  }
  display.setRotation(2); // Rotação 180 graus
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Luva Tradutora");
  display.setCursor(0, 10);
  display.println("Inicializando...");
  display.display();
  logMessage("[OLED] Adafruit_SSD1306 inicializado.");

  // Inicializar MPU6050
  byte status = mpu.begin();
  if (status == 0) {
    logMessage("[MPU] MPU6050 inicializado com sucesso!");
    mpuInitialized = true;
  } else {
    logMessage("[MPU] Erro ao inicializar MPU6050. Status: " + String(status));
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Erro MPU6050");
    display.display();
    while (1) {
      digitalWrite(LED_RED_PIN, HIGH);
      delay(500);
      digitalWrite(LED_RED_PIN, LOW);
      delay(500);
    }
  }

  // Inicializar DFPlayer
  DFSerial.begin(DF_BAUD, SERIAL_8N1, DF_RX_PIN, DF_TX_PIN);
  logMessage("[DFP] Inicializando...");
  int retries = 5; // Aumentado para 5 tentativas
  bool dfpInitialized = false;
  while (retries > 0 && !dfpInitialized) {
    if (dfp.begin(DFSerial, true, true)) {
      logMessage("[DFP] Init OK");
      dfpInitialized = true;
    } else {
      logMessage("[DFP] Init falhou. Tentativa restante: " + String(retries));
      retries--;
      delay(2000); // Aumentado delay para 2s
    }
  }
  if (!dfpInitialized) {
    logMessage("[DFP] Init falhou apos tentativas. Verifique fiacao, 5V, cartao SD.");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Erro DFPlayer");
    display.display();
    while (1) {
      digitalWrite(LED_RED_PIN, HIGH);
      delay(500);
      digitalWrite(LED_RED_PIN, LOW);
      delay(500);
    }
  }
  dfp.volume(VOLUME);
  int currentVolume = dfp.readVolume();
  logMessage("[DFP] Volume configurado para: " + String(currentVolume));
  dfp.EQ(DFPLAYER_EQ_NORMAL);

  int count = dfp.readFileCountsInFolder(0x02); // Lê o número de arquivos na pasta /mp3
  logMessage("[DFP] Arquivos em /mp3 (reportados): " + String(count));
  if (count != LAST_IDX) {
    logMessage("[DFP] Alerta: Numero de arquivos reportados (" + String(count) + ") nao corresponde ao esperado (" + String(LAST_IDX) + "). Verifique o cartao SD (formato FAT32, apenas arquivos .mp3 de 0001 a 0010).");
    digitalWrite(LED_RED_PIN, HIGH);
    delay(LED_ON_MS);
    digitalWrite(LED_RED_PIN, LOW);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Erro SD Count");
    display.display();
  }

  // Inicializar EEPROM
  if (!EEPROM.begin(512)) {
    logMessage("[EEPROM] Erro ao inicializar EEPROM!");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Erro EEPROM");
    display.display();
    while (1) {
      digitalWrite(LED_RED_PIN, HIGH);
      delay(500);
      digitalWrite(LED_RED_PIN, LOW);
      delay(500);
    }
  }
  logMessage("[EEPROM] Inicializada com sucesso (512 bytes).");

  // Inicializar BLE
  BLEDevice::init("LuvaTradutora");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                     CHARACTERISTIC_UUID,
                     BLECharacteristic::PROPERTY_READ |
                     BLECharacteristic::PROPERTY_WRITE |
                     BLECharacteristic::PROPERTY_NOTIFY
                   );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  BLEDevice::startAdvertising();
  logMessage("[BLE] Servidor iniciado");

  // Exibir todas as palavras no OLED com rolagem
  for (int offset = 0; offset < NUM_WORDS; offset += 3) { // Ajustado para 3 palavras por tela
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Palavras:");
    int y = 10;
    for (int i = offset; i < offset + 3 && i < NUM_WORDS; i++) {
      display.setCursor(0, y);
      display.println(words[i]);
      y += 10;
    }
    display.display();
    delay(2000); // Exibe por 2 segundos por página
  }

  // Piscar LEDs
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_GREEN_PIN, HIGH);
    digitalWrite(LED_RED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_GREEN_PIN, LOW);
    digitalWrite(LED_RED_PIN, LOW);
    delay(200);
  }

  // Verificar estado inicial do switch e tocar áudio de teste
  lastModeState = digitalRead(MODE_SWITCH_PIN) == HIGH;
  captureMode = lastModeState;
  if (captureMode) {
    logMessage("[Setup] Modo de calibracao ativado. Pressione o botao de captura para iniciar.");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Modo Calibracao");
    display.setCursor(0, 10);
    display.println("Pressione botao");
    display.display();
  } else {
    logMessage("[Setup] Modo de traducao ativado. Pressione o botao de captura para reconhecer gestos.");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Modo Traducao");
    display.setCursor(0, 10);
    display.println("Pressione botao");
    display.display();
    logMessage("[DFP] Testando reproducao de /mp3/0001.mp3...");
    playIndex(FIRST_IDX); // Toca áudio de teste na inicialização
  }
}

// ==== LOOP ====
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
      captureCount = 0;
      mpuCalibrated = false;
      for (int i = 0; i < NUM_WORDS; i++) {
        capturesPerWord[i] = 0;
        wordSampleCount[i] = 0;
        for (int j = 0; j < NUM_SENSORS; j++) {
          wordAccum[i][j] = 0.0f;
        }
      }
      logMessage("[Mode] Modo de calibracao ativado. Pressione o botao de captura para iniciar.");
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println("Modo Calibracao");
      display.setCursor(0, 10);
      display.println("Pressione botao");
      display.display();
    } else {
      logMessage("[Mode] Modo de traducao ativado. Pressione o botao de captura para reconhecer gestos.");
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println("Modo Traducao");
      display.setCursor(0, 10);
      display.println("Pressione botao");
      display.display();
      logMessage("[DFP] Testando reproducao de /mp3/0001.mp3...");
      playIndex(FIRST_IDX); // Toca áudio de teste ao mudar para modo tradução
    }
    lastModeState = currentModeState;
  }

  if (digitalRead(BUTTON_PIN) == LOW) {
    if (captureMode && !mpuCalibrated) {
      logMessage("[MPU] Calibrando offsets... Mantenha a luva estatica.");
      mpu.calcOffsets(true, true); // Calibra giroscópio e acelerômetro
      logMessage("[MPU] Calibracao concluida.");
      mpuCalibrated = true;
      digitalWrite(LED_GREEN_PIN, HIGH);
      delay(500);
      digitalWrite(LED_GREEN_PIN, LOW);
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println("Calibrado");
      display.display();
      delay(1000);
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Modo Calibracao");
      display.setCursor(0, 10);
      display.println("Pressione botao");
      display.display();
      while (digitalRead(BUTTON_PIN) == LOW);
      delay(50);
      return;
    } else if (captureMode) {
      captureMovement();
    } else {
      readAndRecognize();
    }
    while (digitalRead(BUTTON_PIN) == LOW);
    delay(50);
  }
}
