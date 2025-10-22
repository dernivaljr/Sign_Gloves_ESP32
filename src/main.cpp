#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050_light.h>
#include <DFRobotDFPlayerMini.h>
#include <SPI.h>
#include <SD.h>

// ======================= PINOS =======================
#define OLED_SDA        21
#define OLED_SCL        22

#define DF_RX_PIN       16  // DFPlayer->ESP32 RX
#define DF_TX_PIN       17  // ESP32->DFPlayer TX

#define FLEX1_PIN       36
#define FLEX2_PIN       39
#define FLEX3_PIN       34
#define FLEX4_PIN       35
#define FLEX5_PIN       32

// SD em VSPI
#define SD_CS_PIN       27
#define SD_SCK_PIN      18
#define SD_MISO_PIN     19
#define SD_MOSI_PIN     23

#define BUTTON_PIN      25  // Botão (clique único -> 2s de janela)
#define MODE_SWITCH_PIN 26  // HIGH = CAPTURA, LOW = REPRODUCAO

#define LED_GREEN_PIN   4
#define LED_RED_PIN     5

// ======================= OLED =======================
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  32
#define OLED_RESET     -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ======================= DFPLAYER =======================
HardwareSerial dfSerial(1);
DFRobotDFPlayerMini dfPlayer;

// ======================= SD =======================
SPIClass spiSD(VSPI);

// ======================= MPU6050 =======================
MPU6050 mpu(Wire);
struct Baseline {
  float gx=0, gy=0, gz=0;
  float ax=0, ay=0, az=0;
} base;

// ======================= PROJETO =======================
static const uint8_t NUM_WORDS = 10;
static const uint8_t REPS_PER_WORD = 3;
static const uint16_t SAMPLE_RATE_HZ = 50;     // 50 Hz
static const uint16_t WINDOW_MS = 2000;        // 2 s
static const uint16_t SAMPLES = (WINDOW_MS * SAMPLE_RATE_HZ) / 1000;

static const char* words[NUM_WORDS] = {
  "ola","por favor","obrigado","bom dia","é","meu","nome","boa tarde","ajuda","boa noite"
};
// slugs ASCII para arquivos no SD
static const char* slugs[NUM_WORDS] = {
  "ola","por_favor","obrigado","bom_dia","e","meu","nome","boa_tarde","ajuda","boa_noite"
};
// faixas MP3 (1..10) — arquivos em /mp3/0001.mp3 ... /mp3/0010.mp3
static const uint8_t trackOfWord[NUM_WORDS] = {1,2,3,4,5,6,7,8,9,10};

enum Mode {CAPTURE=0, PLAY=1};

// ====== pesos (depois da normalização) ======
float W_FLEX  = 1.2f;   // média dos 5 flex (0..1)
float W_GYRO  = 0.25f;  // termo relativo 0..1
float W_ACCEL = 0.25f;  // termo relativo 0..1

// ====== “bases” para normalização relativa (evita penalizar Δ pequeno em gestos fortes) ======
const float GYRO_BASE  = 50.0f;   // dps
const float ACCEL_BASE = 1.20f;   // m/s²

// buffers e estado (armazenamos cada réplica)
float meanFlex[NUM_WORDS][REPS_PER_WORD][5]; // 5 flex
float meanGyro[NUM_WORDS][REPS_PER_WORD];    // norma
float meanAccel[NUM_WORDS][REPS_PER_WORD];   // norma

uint8_t repDone[NUM_WORDS] = {0}; // quantas réplicas por palavra
uint8_t currentWord = 0;

bool oledOK=false, sdOK=false, dfOK=false, mpuOK=false;

// ======================= UTIL =======================
void ledGreenBlink(uint8_t times=1, uint16_t on=80, uint16_t off=80) {
  for (uint8_t i=0;i<times;i++){
    digitalWrite(LED_GREEN_PIN, HIGH);
    delay(on);
    digitalWrite(LED_GREEN_PIN, LOW);
    delay(off);
  }
}
void ledRedBlink(uint8_t times=1, uint16_t on=120, uint16_t off=120) {
  for (uint8_t i=0;i<times;i++){
    digitalWrite(LED_RED_PIN, HIGH);
    delay(on);
    digitalWrite(LED_RED_PIN, LOW);
    delay(off);
  }
}

bool buttonClicked() {
  static uint8_t last = HIGH;
  uint8_t now = digitalRead(BUTTON_PIN);
  bool clicked = (last==HIGH && now==LOW);
  last = now;
  return clicked;
}

enum Mode readMode() {
  return (digitalRead(MODE_SWITCH_PIN)==HIGH) ? CAPTURE : PLAY;
}

void oledMsg(const String& l1, const String& l2="") {
  if (!oledOK) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(l1);
  if (l2.length()) display.println(l2);
  display.display();
}

// ===== Espera mão parada com timeout + debug leve =====
bool waitHandStill(uint16_t holdMs=300, float gyroThreshDps=5.0f, float accelThresh=1.5f, uint16_t maxWaitMs=2500) {
  uint32_t tHoldStart = millis();
  uint32_t tOverallStart = millis();
  uint32_t lastPrint = 0;

  while (true) {
    mpu.update();

    float gx = (mpu.getGyroX() - base.gx);
    float gy = (mpu.getGyroY() - base.gy);
    float gz = (mpu.getGyroZ() - base.gz);
    float ax = (mpu.getAccX() - base.ax);
    float ay = (mpu.getAccY() - base.ay);
    float az = (mpu.getAccZ() - base.az);

    float gnorm = sqrtf(gx*gx + gy*gy + gz*gz);
    float anorm = sqrtf(ax*ax + ay*ay + az*az);

    if (millis() - lastPrint > 250) {
      Serial.printf("[PREP] gyro|=%.2f dps  accel|=%.2f m/s2\n", gnorm, anorm);
      lastPrint = millis();
    }

    if (gnorm < gyroThreshDps && anorm < accelThresh) {
      if (millis() - tHoldStart >= holdMs) return true;
    } else {
      tHoldStart = millis();
    }

    if (millis() - tOverallStart >= maxWaitMs) {
      Serial.println("[PREP] Timeout de imobilidade — seguindo assim mesmo.");
      return false;
    }
    delay(5);
  }
}

// ======================= SD HELPERS =======================
bool ensureRefsFolder() {
  if (!sdOK) return false;
  if (!SD.exists("/refs")) {
    if (!SD.mkdir("/refs")) {
      Serial.println("[SD] Falha ao criar /refs");
      return false;
    }
  }
  return true;
}

bool saveCSV(uint8_t w, uint8_t r,
             const float flex[5], float gyroNorm, float accelNorm)
{
  if (!sdOK) return false;
  char path[64];
  snprintf(path, sizeof(path), "/refs/%s_rep%u.csv", slugs[w], (unsigned)(r+1));
  File f = SD.open(path, FILE_WRITE);
  if (!f) {
    Serial.printf("[SD] Falha ao abrir %s\n", path);
    return false;
  }
  f.println("sensor,value");
  for (int i=0;i<5;i++) f.printf("flex%d,%.6f\n", i+1, flex[i]);
  f.printf("gyro_norm,%.6f\n",  gyroNorm);
  f.printf("accel_norm,%.6f\n", accelNorm);
  f.close();
  Serial.printf("[CAPTURA] Salvo: %s\n", path);
  return true;
}

// ======================= CAPTURA DE JANELA =======================
struct WindowMeans {
  float flex[5];
  float gyroNorm;
  float accelNorm;
};

WindowMeans captureWindow(uint16_t ms=WINDOW_MS, uint16_t rateHz=SAMPLE_RATE_HZ) {
  WindowMeans wm{};
  const uint16_t period = 1000 / rateHz;
  uint32_t tStart = millis();
  uint16_t n=0;

  double sumFlex[5]={0,0,0,0,0};
  double sumGyro=0.0, sumAccel=0.0;

  while (millis()-tStart < ms) {
    mpu.update();

    int raw1 = analogRead(FLEX1_PIN);
    int raw2 = analogRead(FLEX2_PIN);
    int raw3 = analogRead(FLEX3_PIN);
    int raw4 = analogRead(FLEX4_PIN);
    int raw5 = analogRead(FLEX5_PIN);

    float f1 = raw1 / 4095.0f;
    float f2 = raw2 / 4095.0f;
    float f3 = raw3 / 4095.0f;
    float f4 = raw4 / 4095.0f;
    float f5 = raw5 / 4095.0f;

    float gx = (mpu.getGyroX() - base.gx);
    float gy = (mpu.getGyroY() - base.gy);
    float gz = (mpu.getGyroZ() - base.gz);
    float ax = (mpu.getAccX() - base.ax);
    float ay = (mpu.getAccY() - base.ay);
    float az = (mpu.getAccZ() - base.az);

    float gnorm = sqrtf(gx*gx + gy*gy + gz*gz);
    float anorm = sqrtf(ax*ax + ay*ay + az*az);

    sumFlex[0]+=f1; sumFlex[1]+=f2; sumFlex[2]+=f3; sumFlex[3]+=f4; sumFlex[4]+=f5;
    sumGyro  += gnorm;
    sumAccel += anorm;
    n++;

    uint32_t tMark = millis();
    while (millis()-tMark < period) { delay(1); }
  }

  if (n==0) n=1;
  for (int i=0;i<5;i++) wm.flex[i] = sumFlex[i]/n;
  wm.gyroNorm  = sumGyro/n;
  wm.accelNorm = sumAccel/n;

  return wm;
}

// ======================= DISTÂNCIA (RELATIVA) =======================
// dflex: média das diferenças absolutas (0..1)
// dg, da: diferenças RELATIVAS 0..1 (|Δ|/max(max(ref,probe),BASE))
struct DistParts { float dflex, dg, da, total; float refG, refA; };

static inline float relDiff(float a, float b, float baseRef) {
  float M = max(max(a,b), baseRef);
  float v = fabsf(a-b) / M;
  if (v > 1.0f) v = 1.0f;
  return v;
}

DistParts weightedDistanceParts(const WindowMeans& probe, const WindowMeans& ref) {
  DistParts p{};
  float df = 0.0f;
  for (int i=0;i<5;i++){
    df += fabsf(probe.flex[i] - ref.flex[i]);
  }
  p.dflex = df / 5.0f;

  p.dg = relDiff(probe.gyroNorm,  ref.gyroNorm,  GYRO_BASE);
  p.da = relDiff(probe.accelNorm, ref.accelNorm, ACCEL_BASE);
  p.refG = ref.gyroNorm;
  p.refA = ref.accelNorm;

  p.total = W_FLEX*p.dflex + W_GYRO*p.dg + W_ACCEL*p.da;
  return p;
}

float weightedDistance(const WindowMeans& a, const WindowMeans& b) {
  return weightedDistanceParts(a,b).total;
}

// ======================= SETUP =======================
void setup() {
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(MODE_SWITCH_PIN, INPUT);

  Serial.begin(115200);
  delay(200);

  // OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  oledOK = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (oledOK) { oledMsg("OLED OK"); }
  else { Serial.println("[OLED] FALHA"); }

  // SD robusto
  Serial.println("[SD] Inicializando...");
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  spiSD.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

  sdOK = SD.begin(SD_CS_PIN, spiSD, 10 * 1000 * 1000);
  if (!sdOK) {
    Serial.println("[SD] 10MHz falhou. Tentando 4MHz...");
    sdOK = SD.begin(SD_CS_PIN, spiSD, 4 * 1000 * 1000);
  }
  if (sdOK) { Serial.println("[SD] OK"); ensureRefsFolder(); }
  else      { Serial.println("[SD] FALHA - verifique pinos/3V3/FAT32"); }

  // DFPlayer
  dfSerial.begin(9600, SERIAL_8N1, DF_RX_PIN, DF_TX_PIN);
  dfOK = dfPlayer.begin(dfSerial);
  if (dfOK) { dfPlayer.volume(30); Serial.println("[DFP] OK"); }
  else      { Serial.println("[DFP] FALHA"); }

  // MPU6050
  Serial.println("[MPU] Inicializando...");
  mpuOK = (mpu.begin()==0);
  if (mpuOK) {
    mpu.calcGyroOffsets();   // v1.1.0 sem argumentos

    // baseline 1s parado
    Serial.println("[CAL] Medindo baseline (1s)...");
    uint32_t t0 = millis();
    uint16_t n=0;
    double sgx=0,sgy=0,sgz=0,sax=0,say=0,saz=0;
    while (millis()-t0 < 1000) {
      mpu.update();
      sgx += mpu.getGyroX(); sgy+=mpu.getGyroY(); sgz+=mpu.getGyroZ();
      sax += mpu.getAccX();  say+=mpu.getAccY();  saz+=mpu.getAccZ();
      n++;
      delay(5);
    }
    if (n==0) n=1;
    base.gx = sgx/n; base.gy = sgy/n; base.gz = sgz/n;
    base.ax = sax/n; base.ay = say/n; base.az = saz/n;
    Serial.printf("[CAL] GyroOff=(%.3f,%.3f,%.3f) AccOff=(%.3f,%.3f,%.3f)\n",
                  base.gx,base.gy,base.gz, base.ax,base.ay,base.az);
  } else {
    Serial.println("[MPU] FALHA");
  }

  // STATUS
  Serial.println();
  Serial.println("=== STATUS INICIAL ===");
  Serial.printf("OLED=%s SD=%s DF=%s MPU=%s\n",
    oledOK?"OK":"FALHA", sdOK?"OK":"FALHA", dfOK?"OK":"FALHA", mpuOK?"OK":"FALHA");
  Serial.println("Switch MODO (GPIO26): HIGH=CAPTURA, LOW=REPRODUCAO.");
  Serial.println("Botao CAPTURE (GPIO25): clique unico -> 2s (captura/reproducao).");
  Serial.println();

  oledMsg("Pronto", sdOK?"SD OK":"SD FAIL");
}

// ======================= LOOP =======================
void loop() {
  Mode mode = readMode();

  // ====== CAPTURA ======
  if (mode == CAPTURE) {
    static bool toldNext = false;

    if (currentWord < NUM_WORDS) {
      uint8_t r = repDone[currentWord];
      if (!toldNext) {
        Serial.printf("[CAPTURA] Proxima: \"%s\" (rep %u/%u) — clique para iniciar\n",
                      words[currentWord], r+1, REPS_PER_WORD);
        oledMsg("CAPTURA", String(words[currentWord]) + " rep " + String(r+1));
        toldNext = true;
      }

      if (buttonClicked()) {
        if (dfOK) {
          // arquivos em /mp3/0001.mp3..0010.mp3
          dfPlayer.playMp3Folder(trackOfWord[currentWord]);
        }

        Serial.println("[PREP] Aguardando mao parada (gyro<5 dps e accel<1.5 m/s2 por 0.3s)...");
        waitHandStill(300, 5.0f, 1.5f, 2500);

        Serial.printf("[CAPTURA] \"%s\" rep %u — iniciando 2s...\n",
                      words[currentWord], r+1);

        WindowMeans wm = captureWindow(WINDOW_MS, SAMPLE_RATE_HZ);

        for (int i=0;i<5;i++) meanFlex[currentWord][r][i] = wm.flex[i];
        meanGyro[currentWord][r]  = wm.gyroNorm;
        meanAccel[currentWord][r] = wm.accelNorm;

        Serial.printf("[CAPTURA] mean flex=[%.3f,%.3f,%.3f,%.3f,%.3f]  gyro|=%.3f  accel|=%.3f\n",
          wm.flex[0],wm.flex[1],wm.flex[2],wm.flex[3],wm.flex[4], wm.gyroNorm, wm.accelNorm);

        if (sdOK) saveCSV(currentWord, r, wm.flex, wm.gyroNorm, wm.accelNorm);

        repDone[currentWord]++;
        ledGreenBlink(1, 80, 60);

        if (repDone[currentWord] >= REPS_PER_WORD) {
          Serial.printf("[CAPTURA] Concluidas %u capturas de \"%s\". Siga para a proxima.\n",
                        REPS_PER_WORD, words[currentWord]);
          currentWord++;
          toldNext = false;
        } else {
          Serial.printf("[CAPTURA] Proxima réplica de \"%s\" (rep %u/%u) — clique para iniciar\n",
                        words[currentWord], repDone[currentWord]+1, REPS_PER_WORD);
          oledMsg("CAPTURA", String(words[currentWord]) + " rep " + String(repDone[currentWord]+1));
          toldNext = true;
        }
      }
    } else {
      static bool once = false;
      if (!once) {
        Serial.println("[CAPTURA] Todas as palavras concluidas!");
        oledMsg("CAPTURA", "Concluida");
        once = true;
      }
    }

  // ====== REPRODUCAO ======
  } else { // PLAY
    static bool toldReady = false;
    if (!toldReady) {
      Serial.println("[PLAY] Pronto para reconhecer — clique o botao para iniciar 2s.");
      oledMsg("REPRODUCAO", "Clique para ler");
      toldReady = true;
    }

    if (buttonClicked()) {
      toldReady = false;

      bool haveAny=false;
      for (uint8_t w=0; w<NUM_WORDS; ++w) if (repDone[w]>0) { haveAny=true; break; }
      if (!haveAny) {
        Serial.println("[PLAY] Sem base de referencia. Entre em CAPTURA primeiro.");
        ledRedBlink(2,120,120);
        return;
      }

      Serial.println("[PREP] Aguardando mao parada (gyro<5 dps e accel<1.5 m/s2 por 0.3s)...");
      waitHandStill(300, 5.0f, 1.5f, 2500);

      Serial.println("[PLAY] Capturando 2s para reconhecimento...");
      WindowMeans probe = captureWindow(WINDOW_MS, SAMPLE_RATE_HZ);
      Serial.printf("[PLAY] probe: gyro|=%.3f accel|=%.3f\n", probe.gyroNorm, probe.accelNorm);

      float bestD = 1e9f; int bestW=-1; int bestRep=-1;
      float secondD = 1e9f; int secondW=-1;

      // Para heurística flex-first
      float bestDFlex = 1e9f; int bestDFlexW=-1;
      float secondDFlex = 1e9f;

      // avalia por réplica (min distance por palavra)
      for (uint8_t w=0; w<NUM_WORDS; ++w) {
        if (repDone[w]==0) continue;

        float bestWordD = 1e9f; int bestWordRep=-1; DistParts bestParts{};
        float bestWordDFlex = 1e9f;

        for (int r=0; r<repDone[w]; ++r) {
          WindowMeans ref{};
          for (int i=0;i<5;i++) ref.flex[i] = meanFlex[w][r][i];
          ref.gyroNorm  = meanGyro[w][r];
          ref.accelNorm = meanAccel[w][r];

          DistParts parts = weightedDistanceParts(probe, ref);
          float d = parts.total;

          if (parts.dflex < bestWordDFlex) bestWordDFlex = parts.dflex;

          if (d < bestWordD) {
            bestWordD = d; bestWordRep = r; bestParts = parts;
          }
        }

        // manter ranking por dflex
        if (bestWordDFlex < bestDFlex) {
          secondDFlex = bestDFlex;
          bestDFlex = bestWordDFlex;
          bestDFlexW = w;
        } else if (bestWordDFlex < secondDFlex) {
          secondDFlex = bestWordDFlex;
        }

        Serial.printf("[PLAY] %-11s d=%.3f  (dflex=%.3f dg=%.3f da=%.3f)  rep=%d  ref(gyro=%.2f,acc=%.2f)\n",
          words[w], bestWordD, bestParts.dflex, bestParts.dg, bestParts.da, bestWordRep+1, bestParts.refG, bestParts.refA);

        // atualiza melhor e segundo no combinado
        if (bestWordD < bestD) {
          secondD=bestD; secondW=bestW;
          bestD=bestWordD; bestW=w; bestRep=bestWordRep;
        } else if (bestWordD < secondD) {
          secondD=bestWordD; secondW=w;
        }
      }

      // Heurística flex-first: se flex do melhor estiver muito à frente, privilegia
      bool flexFirst = false;
      if (bestDFlexW>=0 && bestDFlex < 0.04f && bestDFlex < 0.5f * secondDFlex) {
        flexFirst = true;
        bestW = bestDFlexW;
      }

      float ratio = (secondD<=0.0f) ? 999.0f : (bestD / secondD);
      const float MIN_RATIO = 0.80f;

      if (bestW>=0 && (ratio < MIN_RATIO || flexFirst)) {
        Serial.printf("[DECISION] \"%s\"  d1=%.3f d2=%.3f  ratio=%.3f (min=%.3f)%s\n",
                      words[bestW], bestD, secondD, ratio, MIN_RATIO, flexFirst?"  [flex-first]":"");
        if (dfOK) dfPlayer.playMp3Folder(trackOfWord[bestW]);
        ledGreenBlink(2, 120, 80);
        oledMsg("Reconhecido:", words[bestW]);
      } else {
        Serial.printf("[DECISION] Indefinido  d1=%.3f d2=%.3f  ratio=%.3f (min=%.3f)\n",
                      bestD, secondD, ratio, MIN_RATIO);
        ledRedBlink(2, 120, 120);
        oledMsg("Nao reconhecido", "");
      }
    }
  }
}
