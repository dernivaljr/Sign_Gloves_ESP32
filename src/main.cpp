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
constexpr uint16_t LAST_IDX = 3; // /mp3/0003.mp3 (obrigado)
constexpr uint32_t PLAY_GAP_MS = 1500; // Intervalo entre faixas
constexpr uint32_t DF_TIMEOUT_MS = 5000; // Timeout DFPlayer
constexpr uint32_t LED_ON_MS = 1000; // Tempo LEDs acesos
constexpr int SAMPLES = 50;      // 50 amostras em 1 segundo (50 Hz)
constexpr int NUM_SENSORS = 11;  // 5 flex + 3 accel + 3 gyro
constexpr int NUM_WORDS = 3;     // ola, por favor, obrigado
constexpr int CAPTURES_PER_WORD = 3; // 3 capturas por palavra
constexpr int TOTAL_CAPTURES = NUM_WORDS * CAPTURES_PER_WORD; // 9 capturas
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
float referenceMatrices[NUM_WORDS][CAPTURES_PER_WORD][SAMPLES][NUM_SENSORS];
int captureCount = 0;
bool captureMode = true;
int capturesPerWord[NUM_WORDS] = {0, 0, 0};
String words[NUM_WORDS] = {"ola", "por favor", "obrigado"}; // Lista de palavras sem acentos
uint16_t currentIndex = FIRST_IDX;
uint32_t lastPlayMillis = 0;
bool waitingNext = true;
bool lastModeState = true;
char logBuffer[BUFFER_SIZE] = ""; // Buffer para acumular logs
int bufferIndex = 0;

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
    } else {
      bufferIndex = 0;
      strcpy(logBuffer, message.c_str());
      bufferIndex = msgLength;
      logBuffer[bufferIndex] = '\0';
      pCharacteristic->setValue(logBuffer);
      pCharacteristic->notify();
      delay(10);
    }
  }
}

// ==== LOG DE EVENTOS DO DFPLAYER ====
void printDFEvent(uint8_t type, int value) {
  String message;
  switch (type) {
    case DFPlayerCardInserted:  message = F("[DFP] Cartao SD inserido"); break;
    case DFPlayerCardRemoved:   message = F("[DFP] Cartao SD removido"); break;
    case DFPlayerCardOnline:    message = F("[DFP] Cartao SD online"); break;
    case DFPlayerUSBInserted:   message = F("[DFP] USB inserido"); break;
    case DFPlayerUSBRemoved:    message = F("[DFP] USB removido"); break;
    case DFPlayerUSBOnline:     message = F("[DFP] USB online"); break;
    case DFPlayerPlayFinished:  message = "[DFP] Terminado faixa: " + String(value);
                                waitingNext = true; lastPlayMillis = millis(); break;
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

  String msg = "[DFP] Reproduzindo /mp3/" + String(idx, DEC) + ".mp3 (" + words[idx - 1] + ")";
  logMessage(msg);
  dfp.playMp3Folder(idx);
  waitingNext = false;
  currentIndex = idx;

  unsigned long startTime = millis();
  while (millis() - startTime < DF_TIMEOUT_MS) {
    if (dfp.available()) {
      uint8_t type = dfp.readType();
      int value = dfp.read();
      printDFEvent(type, value);
      if (type == DFPlayerPlayFinished && value == idx) {
        return true;
      }
    }
  }
  String timeoutMsg = "[DFP] Timeout aguardando resposta do DFPlayer";
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
    dataMatrix[i][0] = analogRead(FLEX_PINS[0]) / 4095.0;
    dataMatrix[i][1] = analogRead(FLEX_PINS[1]) / 4095.0;
    dataMatrix[i][2] = analogRead(FLEX_PINS[2]) / 4095.0;
    dataMatrix[i][3] = analogRead(FLEX_PINS[3]) / 4095.0;
    dataMatrix[i][4] = analogRead(FLEX_PINS[4]) / 4095.0;

    mpu.update();
    if (mpu.getAccX() != 0 || mpu.getAccY() != 0 || mpu.getAccZ() != 0) {
      dataMatrix[i][5] = mpu.getAccX() / 8.0;
      dataMatrix[i][6] = mpu.getAccY() / 8.0;
      dataMatrix[i][7] = mpu.getAccZ() / 8.0;
      dataMatrix[i][8] = mpu.getGyroX() / 500.0;
      dataMatrix[i][9] = mpu.getGyroY() / 500.0;
      dataMatrix[i][10] = mpu.getGyroZ() / 500.0;
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
        dataMatrix[i][j] = 0.0;
      }
    }
    delay(20); // 20ms por amostra para 50 amostras em 1 segundo (50 Hz)
  }

  for (int i = 0; i < SAMPLES; i++) {
    for (int j = 0; j < NUM_SENSORS; j++) {
      referenceMatrices[wordIndex][captureIndex][i][j] = dataMatrix[i][j];
    }
  }

  String matrixMsg = "=== Matriz Capturada ===\n";
  for (int i = 0; i < SAMPLES; i++) {
    matrixMsg += "Amostra " + String(i) + ": ";
    for (int j = 0; j < NUM_SENSORS; j++) {
      matrixMsg += String(dataMatrix[i][j], 2) + " ";
    }
    matrixMsg += "\n";
  }
  logMessage(matrixMsg);

  String confirmMsg = "Captura " + String(captureIndex + 1) + " da palavra " + currentWord + " OK";
  logMessage(confirmMsg);

  capturesPerWord[wordIndex]++;
  captureCount++;

  if (captureCount >= TOTAL_CAPTURES) {
    String endMsg = "Captura de 3 movimentos por palavra concluida! Entrando em modo de traducao.";
    logMessage(endMsg);
    captureMode = false;
    lastModeState = false;
    String testMsg = "[DFP] Testando reproducao de /mp3/0001.mp3...";
    logMessage(testMsg);
    playIndex(FIRST_IDX);
  }
}

// ==== CALCULAR DISTÂNCIA EUCLIDIANA ====
float calculateDistance(float matrix1[SAMPLES][NUM_SENSORS], float matrix2[SAMPLES][NUM_SENSORS]) {
  float distance = 0.0;
  for (int i = 0; i < SAMPLES; i++) {
    for (int j = 0; j < NUM_SENSORS; j++) {
      float diff = matrix1[i][j] - matrix2[i][j];
      distance += diff * diff;
    }
  }
  return sqrt(distance);
}

// ==== MODO DE LEITURA E RECONHECIMENTO ====
void readAndRecognize() {
  if (!waitingNext) {
    if (millis() - lastPlayMillis > DF_TIMEOUT_MS) {
      String msg = "[DFP] Timeout aguardando audio. Liberando para nova tentativa.";
      logMessage(msg);
      digitalWrite(LED_RED_PIN, HIGH);
      delay(LED_ON_MS);
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
    dataMatrix[i][0] = analogRead(FLEX_PINS[0]) / 4095.0;
    dataMatrix[i][1] = analogRead(FLEX_PINS[1]) / 4095.0;
    dataMatrix[i][2] = analogRead(FLEX_PINS[2]) / 4095.0;
    dataMatrix[i][3] = analogRead(FLEX_PINS[3]) / 4095.0;
    dataMatrix[i][4] = analogRead(FLEX_PINS[4]) / 4095.0;

    mpu.update();
    if (mpu.getAccX() != 0 || mpu.getAccY() != 0 || mpu.getAccZ() != 0) {
      dataMatrix[i][5] = mpu.getAccX() / 8.0;
      dataMatrix[i][6] = mpu.getAccY() / 8.0;
      dataMatrix[i][7] = mpu.getAccZ() / 8.0;
      dataMatrix[i][8] = mpu.getGyroX() / 500.0;
      dataMatrix[i][9] = mpu.getGyroY() / 500.0;
      dataMatrix[i][10] = mpu.getGyroZ() / 500.0;
    } else {
      mpuError = true;
      String errMsg = "Erro ao ler MPU6050 na amostra " + String(i);
      logMessage(errMsg);
      digitalWrite(LED_RED_PIN, HIGH);
      delay(LED_ON_MS);
      digitalWrite(LED_RED_PIN, LOW);
      for (int j = 5; j < NUM_SENSORS; j++) {
        dataMatrix[i][j] = 0.0;
      }
    }
    delay(20); // 20ms por amostra para 50 amostras em 1 segundo (50 Hz)
  }

  float avgDistances[NUM_WORDS] = {0.0, 0.0, 0.0};
  for (int w = 0; w < NUM_WORDS; w++) {
    float totalDistance = 0.0;
    for (int c = 0; c < CAPTURES_PER_WORD; c++) {
      totalDistance += calculateDistance(dataMatrix, referenceMatrices[w][c]);
    }
    avgDistances[w] = totalDistance / CAPTURES_PER_WORD;
  }

  String distMsg = "=== Distâncias Médias ===\n";
  for (int w = 0; w < NUM_WORDS; w++) {
    distMsg += "Distância para " + words[w] + ": " + String(avgDistances[w], 2) + "\n";
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

  if (minDist < 10.0 && !mpuError) {
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
    delay(LED_ON_MS);
    digitalWrite(LED_GREEN_PIN, LOW);
  } else {
    String noRecMsg = "Palavra não reconhecida ou erro no MPU6050.";
    logMessage(noRecMsg);
    digitalWrite(LED_RED_PIN, HIGH);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Não reconhecida");
    display.display();
    delay(LED_ON_MS);
    digitalWrite(LED_RED_PIN, LOW);
  }

  String readyMsg = "Pronto para nova captura. Pressione o botão.";
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
    logMessage("[DFP] Init falhou após tentativas. Verifique fiação, 5V, cartão SD.");
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

  int count = dfp.readFileCountsInFolder(0x02);
  logMessage("[DFP] Arquivos em /mp3 (reportados): " + String(count));
  if (count < 0) {
    logMessage("[DFP] Erro ao ler cartão SD. Verifique formato (FAT32) e arquivos.");
    digitalWrite(LED_RED_PIN, HIGH);
    delay(LED_ON_MS);
    digitalWrite(LED_RED_PIN, LOW);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Erro SD Card");
    display.display();
  }

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

  // Inicializar matrizes
  for (int w = 0; w < NUM_WORDS; w++) {
    for (int c = 0; c < CAPTURES_PER_WORD; c++) {
      for (int i = 0; i < SAMPLES; i++) {
        for (int j = 0; j < NUM_SENSORS; j++) {
          referenceMatrices[w][c][i][j] = 0.0;
        }
      }
    }
  }

  // Exibir palavras no OLED
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Palavras:");
  int y = 10;
  for (int i = 0; i < NUM_WORDS && y < SCREEN_HEIGHT; i++) {
    display.setCursor(0, y);
    display.println(words[i]);
    y += 10;
  }
  display.display();
  delay(2000); // Exibe por 2 segundos

  // Piscar LEDs
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_GREEN_PIN, HIGH);
    digitalWrite(LED_RED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_GREEN_PIN, LOW);
    digitalWrite(LED_RED_PIN, LOW);
    delay(200);
  }

  // Verificar estado inicial do switch
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
    playIndex(FIRST_IDX);
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
      for (int i = 0; i < NUM_WORDS; i++) {
        capturesPerWord[i] = 0;
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
      playIndex(FIRST_IDX);
    }
    lastModeState = currentModeState;
  }

  if (digitalRead(BUTTON_PIN) == LOW) {
    if (captureMode) {
      captureMovement();
    } else {
      readAndRecognize();
    }
    while (digitalRead(BUTTON_PIN) == LOW);
    delay(100);
  }
}