// ======================= Sign Gloves ESP32 – Captura & Reprodução =======================
// Placa: ESP32 DevKit V1  | Framework: Arduino (PlatformIO)
// Periféricos: OLED 128x32 I2C (0x3C), DFPlayer Mini (Serial1), SD (SPI), MPU6050 (I2C),
//              5 flex (ADC), botão de captura (GPIO25), switch de modo (GPIO26),
//              LED verde (GPIO4), LED vermelho (GPIO5)
//
// Áudios em: /mp3/0001.mp3 .. /mp3/0010.mp3  (usar dfPlayer.playMp3Folder(idx))
//
// Modo:  GPIO26 HIGH = CAPTURA  |  GPIO26 LOW = REPRODUÇÃO
// Botão: clique único inicia uma janela de 2s (CAPTURA: grava réplica; REPRODUÇÃO: reconhece e toca)
// ========================================================================================

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DFRobotDFPlayerMini.h>
#include <MPU6050_light.h>
#include <algorithm>

// -------------------- Pinos --------------------
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  32
#define OLED_RESET     -1

// Flex (ADC)
#define FLEX1_PIN 32
#define FLEX2_PIN 33
#define FLEX3_PIN 34
#define FLEX4_PIN 35
#define FLEX5_PIN 36  // VP (apenas entrada)

// I2C
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// DFPlayer (Serial1)
#define DF_RX_PIN 16   // DFPlayer TX -> ESP32 RX16
#define DF_TX_PIN 17   // DFPlayer RX -> ESP32 TX17

// SD (SPI)
#define SD_CS_PIN   27
#define SD_MISO_PIN 12
#define SD_MOSI_PIN 13
#define SD_SCK_PIN  14

// Controles/LEDs
#define SWITCH_PIN      26   // HIGH=CAPTURA, LOW=REPRODUCAO
#define BUTTON_PIN      25   // clique único inicia 2s
#define LED_GREEN_PIN   4
#define LED_RED_PIN     5

// -------------------- Objetos --------------------
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
HardwareSerial dfSerial(1);
DFRobotDFPlayerMini dfPlayer;
SPIClass spiSD;
MPU6050 mpu(Wire);

// -------------------- Parâmetros de captura --------------------
static const uint16_t FS_HZ = 100;          // ~100 Hz
static const uint16_t CAPTURE_MS = 2000;    // 2 s
static const uint16_t N_SAMPLES = FS_HZ * (CAPTURE_MS / 1000);
static const uint8_t  N_FLEX = 5;
static const uint8_t  N_GYRO = 3;
static const uint8_t  N_ACCEL = 3;
static const uint8_t  N_FEAT = N_FLEX + N_GYRO + N_ACCEL; // 11 features
static const uint8_t  N_WORDS = 10;
static const uint8_t  N_REPS = 3;

// Pesos (aumenta importância de IMU)
static const float W_FLEX = 1.0f;
static const float W_GYRO = 1.6f;
static const float W_ACC  = 1.3f;

// Critérios
static const float STILL_GYRO_DPS = 5.0f;     // mão parada < 5 dps
static const uint16_t STILL_MS = 300;         // por 0.3 s
static const float MIN_RATIO = 1.15f;         // margem baixa se d2/d1 < 1.15

// Radius clamp (para z-score estável)
static const float RADIUS_MIN = 2.0f;
static const float RADIUS_MAX = 15.0f;

// -------------------- Palavras / nomes de arquivo --------------------
const char* WORDS[N_WORDS] = {
  "ola", "por favor", "obrigado", "bom dia", "e",
  "meu", "nome", "boa tarde", "ajuda", "boa noite"
};

// nomes “seguros” (sem acento/espços) p/ arquivos no SD
const char* SAFE[N_WORDS] = {
  "ola", "por_favor", "obrigado", "bom_dia", "e",
  "meu", "nome", "boa_tarde", "ajuda", "boa_noite"
};

// -------------------- Estado --------------------
bool oledOK=false, sdOK=false, dfOK=false, mpuOK=false;

float gyroOff[3]  = {0,0,0};   // offset simples do gyro (calib rápida)
float accOff[3]   = {0,0,0};   // offset média bruta accel (não remove gravidade)

uint8_t wordIdx = 0;           // índice da palavra em captura
uint8_t repIdx[N_WORDS] = {0}; // 0..3 (contador de capturas feitas por palavra)

// Centroides e "radius" (intra-cluster) por palavra
float centroid[N_WORDS][N_FEAT]; // média das 3 réplicas
float radiusW[N_WORDS];          // média das distâncias das reps ao centróide (clamped)

// Debounce
uint32_t lastBtnMs = 0;

// -------------------- Helpers de LED --------------------
void ledOK()    { digitalWrite(LED_GREEN_PIN, HIGH); delay(120); digitalWrite(LED_GREEN_PIN, LOW); }
void ledError() { digitalWrite(LED_RED_PIN,   HIGH); delay(250); digitalWrite(LED_RED_PIN,   LOW); }

// -------------------- Utilidades --------------------
void clearCentroids() {
  memset(centroid, 0, sizeof(centroid));
  memset(radiusW, 0, sizeof(radiusW));
}

float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

float vecDist(const float *a, const float *b, uint8_t n) {
  float s=0;
  for (uint8_t i=0;i<n;i++) {
    float d = a[i]-b[i];
    s += d*d;
  }
  return sqrtf(s);
}

float norm3(float x, float y, float z) {
  return sqrtf(x*x + y*y + z*z);
}

bool waitStill(uint16_t msNeeded, float gyroThreshDps) {
  uint32_t t0 = millis();
  uint32_t okSince = 0;
  Serial.println("[PREP] Aguardando mao parada (gyro_norm < 5 dps por 0.3s)...");
  while (true) {
    mpu.update();
    float gx = mpu.getGyroX() - gyroOff[0];
    float gy = mpu.getGyroY() - gyroOff[1];
    float gz = mpu.getGyroZ() - gyroOff[2];
    float gnorm = norm3(gx,gy,gz);
    if (gnorm < gyroThreshDps) {
      if (okSince==0) okSince = millis();
      if (millis() - okSince >= msNeeded) return true;
    } else {
      okSince = 0;
    }
    if (millis() - t0 > 5000) return true; // timeout amigável
    delay(5);
  }
}

void drawSmall(const String& l1, const String& l2="") {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(l1);
  if (l2.length()) display.println(l2);
  display.display();
}

// captura 2s -> média (11 features) com pesos aplicados
bool captureMean(float outFeat[N_FEAT], bool logSummary) {
  float acc[N_FEAT]={0};

  uint32_t tStart = millis();
  uint32_t tEnd = tStart + CAPTURE_MS;
  uint16_t n=0;

  while (millis() < tEnd) {
    float f1 = analogRead(FLEX1_PIN)/4095.0f;
    float f2 = analogRead(FLEX2_PIN)/4095.0f;
    float f3 = analogRead(FLEX3_PIN)/4095.0f;
    float f4 = analogRead(FLEX4_PIN)/4095.0f;
    float f5 = analogRead(FLEX5_PIN)/4095.0f;

    mpu.update();
    float gx = (mpu.getGyroX()-gyroOff[0]);
    float gy = (mpu.getGyroY()-gyroOff[1]);
    float gz = (mpu.getGyroZ()-gyroOff[2]);

    float ax = (mpu.getAccX()-accOff[0])*9.80665f;
    float ay = (mpu.getAccY()-accOff[1])*9.80665f;
    float az = (mpu.getAccZ()-accOff[2])*9.80665f;

    acc[0]+=f1; acc[1]+=f2; acc[2]+=f3; acc[3]+=f4; acc[4]+=f5;
    acc[5]+=gx; acc[6]+=gy; acc[7]+=gz;
    acc[8]+=ax; acc[9]+=ay; acc[10]+=az;

    n++;
    delay(1000/FS_HZ);
  }

  if (n==0) return false;
  for (uint8_t i=0;i<N_FEAT;i++) outFeat[i]=acc[i]/n;

  // aplica pesos
  for (uint8_t i=0;i<N_FLEX;i++) outFeat[i] *= W_FLEX;
  for (uint8_t i=0;i<N_GYRO;i++) outFeat[5+i] *= W_GYRO;      // 5..7
  for (uint8_t i=0;i<N_ACCEL;i++) outFeat[8+i] *= W_ACC;      // 8..10

  if (logSummary) {
    float gnorm = norm3(outFeat[5]/W_GYRO, outFeat[6]/W_GYRO, outFeat[7]/W_GYRO);
    float anorm = norm3(outFeat[8]/W_ACC, outFeat[9]/W_ACC, outFeat[10]/W_ACC);
    Serial.printf("[CAPTURA] mean flex=[%.3f,%.3f,%.3f,%.3f,%.3f]  gyro|=%.3f  accel|=%.3f\n",
      outFeat[0]/W_FLEX, outFeat[1]/W_FLEX, outFeat[2]/W_FLEX, outFeat[3]/W_FLEX, outFeat[4]/W_FLEX,
      gnorm, anorm);
  }
  return true;
}

bool saveCSV(const char* baseName, uint8_t rep, const float feat[N_FEAT]) {
  if (!sdOK) return false;
  char path[64];
  snprintf(path, sizeof(path), "/%s_rep%u.csv", baseName, (unsigned)rep);
  File f = SD.open(path, FILE_WRITE);
  if (!f) return false;
  f.println("idx,feat");
  for (uint8_t i=0;i<N_FEAT;i++) {
    f.print(i); f.print(','); f.println(feat[i], 6);
  }
  f.close();
  Serial.printf("[CAPTURA] Salvo: %s\n", path);
  return true;
}

void recomputeCentroid(uint8_t w) {
  float sum[N_FEAT]={0};
  float reps[3][N_FEAT]={0};
  uint8_t have=0;

  for (uint8_t r=1;r<=N_REPS;r++) {
    char path[64];
    snprintf(path,sizeof(path),"/%s_rep%u.csv", SAFE[w], (unsigned)r);
    File f = SD.open(path, FILE_READ);
    if (!f) continue;
    String header = f.readStringUntil('\n');
    uint8_t k=0;
    while (f.available() && k<N_FEAT) {
      String line = f.readStringUntil('\n');
      int comma = line.indexOf(',');
      if (comma>0) {
        float v = line.substring(comma+1).toFloat();
        reps[r-1][k]=v;
        sum[k]+=v;
        k++;
      }
    }
    f.close();
    if (k==N_FEAT) have++;
  }

  if (have==0) { memset(centroid[w],0,sizeof(centroid[w])); radiusW[w]=RADIUS_MAX; return; }

  for (uint8_t i=0;i<N_FEAT;i++) centroid[w][i]=sum[i]/have;

  float rsum=0; uint8_t rc=0;
  for (uint8_t r=0;r<N_REPS;r++) {
    bool valid=true;
    for (uint8_t i=0;i<N_FEAT;i++) if (reps[r][i]==0 && centroid[w][i]==0) { valid=false; break; }
    if (!valid) continue;
    rsum += vecDist(reps[r], centroid[w], N_FEAT);
    rc++;
  }
  float rad = (rc? (rsum/rc) : RADIUS_MIN);
  radiusW[w] = clampf(rad, RADIUS_MIN, RADIUS_MAX);
}

void recomputeAll() {
  for (uint8_t w=0; w<N_WORDS; w++) recomputeCentroid(w);
}

// -------------------- Setup --------------------
void setup() {
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN,   OUTPUT);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_RED_PIN,   LOW);

  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  delay(200);

  // I2C
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // OLED
  Serial.println("[OLED] Inicializando...");
  oledOK = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (oledOK) { 
    Serial.println("[OLED] OK"); 
    display.clearDisplay(); display.display();
  } else { 
    Serial.println("[OLED] FALHA"); 
    ledError(); 
  }

  // SD
  Serial.println("[SD] Inicializando...");
  spiSD.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  sdOK = SD.begin(SD_CS_PIN, spiSD);
  if (sdOK) {
    Serial.println("[SD] OK");
  } else {
    Serial.println("[SD] FALHA");
    ledError();
  }

  // DFPlayer
  Serial.println("[DFP] Inicializando...");
  dfSerial.begin(9600, SERIAL_8N1, DF_RX_PIN, DF_TX_PIN);
  dfOK = dfPlayer.begin(dfSerial, true, false);
  if (dfOK) {
    dfPlayer.volume(30); // max
    Serial.println("[DFP] OK");
  } else {
    Serial.println("[DFP] FALHA");
    ledError();
  }

  // MPU6050
  Serial.println("[MPU] Inicializando...");
  byte rc = mpu.begin();
  if (rc==0) {
    mpu.setAccConfig(0x08);   // ±4g
    mpu.setGyroConfig(0x08);  // ±500°/s
    mpuOK = true;
    Serial.println("[MPU] OK");
  } else {
    mpuOK = false;
    Serial.printf("[MPU] FALHA (code=%u)\n", rc);
    ledError();
  }

  // Calibração rápida de baseline
  if (mpuOK) {
    Serial.println("[CAL] Medindo baseline (1s)...");
    uint32_t t0=millis();
    float gsum[3]={0}, asum[3]={0};
    uint16_t n=0;
    while (millis()-t0 < 1000) {
      mpu.update();
      gsum[0]+=mpu.getGyroX(); gsum[1]+=mpu.getGyroY(); gsum[2]+=mpu.getGyroZ();
      asum[0]+=mpu.getAccX();  asum[1]+=mpu.getAccY();  asum[2]+=mpu.getAccZ();
      n++; delay(5);
    }
    if (n>0) {
      gyroOff[0]=gsum[0]/n; gyroOff[1]=gsum[1]/n; gyroOff[2]=gsum[2]/n;
      accOff[0]=asum[0]/n;  accOff[1]=asum[1]/n;  accOff[2]=asum[2]/n;
    }
    Serial.printf("[CAL] GyroOff=(%.3f,%.3f,%.3f) AccOff=(%.3f,%.3f,%.3f)\n",
      gyroOff[0],gyroOff[1],gyroOff[2],accOff[0],accOff[1],accOff[2]);
  }

  // Recalcula centroides dos CSV (se já houver)
  if (sdOK) recomputeAll();

  // Status inicial
  Serial.println();
  Serial.println("=== STATUS INICIAL ===");
  Serial.printf("OLED=%s SD=%s DF=%s MPU=%s\n",
    oledOK?"OK":"FALHA", sdOK?"OK":"FALHA", dfOK?"OK":"FALHA", mpuOK?"OK":"FALHA");
  Serial.println("Switch MODO (GPIO26): HIGH=CAPTURA, LOW=REPRODUCAO.");
  Serial.println("Botao CAPTURE (GPIO25): clique unico -> 2s (captura/reproducao).");

  // Mensagem OLED
  if (oledOK) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("Pronto");
    display.println("HIGH=CAPTURA");
    display.println("LOW=PLAY (clique)");
    display.display();
  }
}

// -------------------- Loop --------------------
void loop() {
  // Log de mudança de modo (switch)
  static int lastMode=-1;
  int mode = digitalRead(SWITCH_PIN);
  if (mode != lastMode) {
    Serial.printf("[MODO] Switch(GPIO26)=%s -> %s\n",
                  mode==HIGH?"HIGH":"LOW",
                  mode==HIGH?"CAPTURA":"REPRODUCAO");
    lastMode = mode;
  }

  // ===================== MODO CAPTURA =====================
  if (mode == HIGH) {
    // mostra próxima palavra
    uint8_t r = repIdx[wordIdx] + 1;
    if (r<=N_REPS) {
      Serial.printf("[CAPTURA] Proxima: \"%s\" (rep %u/%u) — clique para iniciar\n",
        WORDS[wordIdx], (unsigned)r, (unsigned)N_REPS);
    }

    // aguardar clique (debounce)
    while (digitalRead(SWITCH_PIN)==HIGH) {
      if (digitalRead(BUTTON_PIN)==LOW) {
        uint32_t now = millis();
        if (now - lastBtnMs > 200) { // debounce 200ms
          lastBtnMs = now;

          // espera mão parada
          if (mpuOK) waitStill(STILL_MS, STILL_GYRO_DPS);

          // toca áudio da palavra atual
          if (dfOK) dfPlayer.playMp3Folder(wordIdx+1);

          // mensagem e OLED
          char l1[64];
          snprintf(l1,sizeof(l1),"\"%s\" rep %u — 2s", WORDS[wordIdx], (unsigned)(repIdx[wordIdx]+1));
          if (oledOK) { display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE); display.setCursor(0,0); display.println("CAPTURA"); display.println(l1); display.display(); }
          Serial.printf("[CAPTURA] \"%s\" rep %u — iniciando 2s...\n",
            WORDS[wordIdx], (unsigned)(repIdx[wordIdx]+1));

          // captura mean (ponderado)
          float feat[N_FEAT];
          if (!captureMean(feat, true)) {
            Serial.println("[ERRO] Falha na captura (amostras=0)");
            ledError();
            break;
          }

          // salva CSV
          if (sdOK) {
            if (!saveCSV(SAFE[wordIdx], repIdx[wordIdx]+1, feat)) {
              Serial.println("[ERRO] Falha ao salvar CSV");
              ledError();
            }
          }

          // atualiza réplica e, se fechou 3/3, recomputa centróide e avança palavra
          repIdx[wordIdx]++;
          if (repIdx[wordIdx] >= N_REPS) {
            Serial.printf("[CAPTURA] Concluidas 3 capturas de \"%s\". Siga para a proxima.\n", WORDS[wordIdx]);
            if (sdOK) recomputeCentroid(wordIdx);
            if (wordIdx+1 < N_WORDS) {
              wordIdx++;
              Serial.printf("[CAPTURA] Avancando: \"%s\" (rep 1/%u)\n", WORDS[wordIdx], (unsigned)N_REPS);
            } else {
              Serial.println("[CAPTURA] Todas as palavras concluidas!");
            }
          } else {
            Serial.printf("[CAPTURA] Proxima réplica de \"%s\" (rep %u/%u) — clique para iniciar\n",
              WORDS[wordIdx], (unsigned)(repIdx[wordIdx]+1), (unsigned)N_REPS);
          }

          ledOK(); // sucesso
        }
        // espera soltar
        while (digitalRead(BUTTON_PIN)==LOW) delay(5);
      }
      delay(5);
    }

    delay(10);
    return;
  }

  // ===================== MODO REPRODUÇÃO =====================
  // >>>>> CORREÇÃO: Somente reconhece AO CLICAR o botão (não mais automático) <<<<<
  if (digitalRead(SWITCH_PIN)==LOW) {
    // dica no serial (uma vez a cada ciclo)
    static uint32_t lastHint=0;
    if (millis()-lastHint > 1000) {
      Serial.println("[PLAY] Pronto para reconhecer — clique o botao para iniciar 2s.");
      lastHint = millis();
    }

    // espera clique com debounce
    if (digitalRead(BUTTON_PIN)==LOW) {
      uint32_t now = millis();
      if (now - lastBtnMs > 200) {
        lastBtnMs = now;

        // mão parada antes de capturar
        if (mpuOK) waitStill(STILL_MS, STILL_GYRO_DPS);

        if (oledOK) { display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE); display.setCursor(0,0); display.println("PLAY"); display.println("capturando 2s..."); display.display(); }
        Serial.println("[PLAY] Capturando 2s para reconhecimento...");

        float x[N_FEAT];
        if (!captureMean(x, false)) {
          Serial.println("[ERRO] Falha na captura de play");
          ledError();
          // espera soltar
          while (digitalRead(BUTTON_PIN)==LOW) delay(5);
          return;
        }

        // distâncias/z para cada palavra
        float d[N_WORDS];
        float z[N_WORDS];

        for (uint8_t w=0; w<N_WORDS; w++) {
          d[w] = vecDist(x, centroid[w], N_FEAT);
          float rad = (radiusW[w] > 0 ? radiusW[w] : RADIUS_MIN);
          float radc = clampf(rad, RADIUS_MIN, RADIUS_MAX);
          z[w] = d[w] / radc;
        }

        for (uint8_t w=0; w<N_WORDS; w++) {
          Serial.printf("[PLAY] %-11s d=%.3f  z=%.3f  lim=%.3f\n",
            WORDS[w], d[w], z[w], radiusW[w]*1.6f);
        }

        int best=0, second=1;
        if (d[1] < d[0]) { best=1; second=0; }
        for (uint8_t w=2; w<N_WORDS; w++) {
          if (d[w] < d[best]) { second = best; best = w; }
          else if (d[w] < d[second]) { second = w; }
        }

        float ratio = d[second]/d[best];
        bool lowMargin = (ratio < MIN_RATIO);

        Serial.printf("[DECISION] \"%s\"  d1=%.3f d2=%.3f  z1=%.3f z2=%.3f  ratio=%.3f  (min=%.3f)\n",
          WORDS[best], d[best], d[second], z[best], z[second], ratio, MIN_RATIO);

        if (dfOK) {
          dfPlayer.playMp3Folder(best+1); // 1..10
          Serial.printf("[PLAY] Reconhecido: \"%s\" -> faixa %u\n", WORDS[best], (unsigned)(best+1));
        }

        if (lowMargin) { Serial.println("[WARN] Margem baixa: gesto similar a outra palavra"); ledError(); }
        else           { ledOK(); }

        // espera soltar o botão antes de permitir outro reconhecimento
        while (digitalRead(BUTTON_PIN)==LOW) delay(5);
      }
    }

    delay(5);
    return;
  }
}
