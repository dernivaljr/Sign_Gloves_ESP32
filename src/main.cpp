#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HardwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include "driver/adc.h" // analogSetAttenuation (ESP32)

// ================== PINAGEM (ESP32 DevKit V1) ==================
#define I2C_SDA_PIN   21
#define I2C_SCL_PIN   22

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  32
#define OLED_RESET     -1
const uint8_t OLED_ADDR = 0x3C;

#define LED_GREEN_PIN  4
#define LED_RED_PIN    5    // LED vermelho = GPIO5

#define BUTTON_CAPTURE 25   // inicia UMA captura (clique)
#define BUTTON_MODE    26   // switch: HIGH=CAPTURA, LOW=REPRODUCAO

#define DF_RX_PIN      16   // ESP32 RX1  <= TX DFPlayer
#define DF_TX_PIN      17   // ESP32 TX1  => RX DFPlayer

#define SD_CS_PIN      27
#define SD_MISO_PIN    19
#define SD_MOSI_PIN    23
#define SD_SCK_PIN     18

// ================== FLEX (ADC) ==================
#define FLEX1_PIN 36
#define FLEX2_PIN 39
#define FLEX3_PIN 34
#define FLEX4_PIN 35
#define FLEX5_PIN 32

// ================== MPU6050 ==================
#define MPU_ADDR 0x68

// ================== CAPTAÇÃO ==================
static const int SAMPLES      = 100;     // 100 amostras
static const int CAPTURE_MS   = 2000;    // 2 segundos por captura
static const int SAMPLE_DELAY = CAPTURE_MS / SAMPLES; // ~20 ms (≈50 Hz)
static const int FEATS        = 11;      // 5 flex + 3 gyro + 3 accel

// ========= ORDEM das PALAVRAS / TRILHAS (1..10) =========
static const int NUM_WORDS = 10;
String words[NUM_WORDS] = {
  "ola",        // 1
  "por favor",  // 2
  "obrigado",   // 3
  "bom dia",    // 4
  "é",          // 5
  "meu",        // 6
  "nome",       // 7
  "boa tarde",  // 8
  "ajuda",      // 9
  "boa noite"   // 10
};

static const int REPS_PER_WORD = 3;

// ================== OBJETOS ==================
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
SPIClass spiSD(VSPI);
HardwareSerial dfSerial(1);
DFRobotDFPlayerMini dfPlayer;

// ================== ESTADO ==================
bool sdOK=false, oledOK=false, dfOK=false, mpuOK=false;

// MODO via estado do SWITCH (não toggle):
enum AppMode { MODE_CAPTURE, MODE_PLAY };
#define MODE_WHEN_HIGH MODE_CAPTURE  // HIGH => CAPTURA; LOW => PLAY (troque se quiser inverter)

uint8_t currentWord = 0; // 0..9
uint8_t currentRep  = 1; // 1..3

uint32_t lastBlink=0; bool ledState=false;

// ====== pesos por feature (balancear escalas) ======
static const float FEAT_W[FEATS] =
  {1,1,1,1,1,   0.30f,0.30f,0.30f,   0.50f,0.50f,0.50f};

// ====== rejeição (tuning) ======
static const float REJECT_THR_BASE = 6.0f;  // limiar absoluto (ajustado para sua escala atual)
static const float K_RADIUS        = 1.6f;  // limite por raio: d_best <= K * radius_best
static const float MARGIN_RATIO    = 0.20f; // exige d2/d1 >= 1.20 (20%) para aceitar
static const float Z_MARGIN_MIN    = 0.30f; // exige (z2 - z1) >= 0.30
static const float Z_MIN_RADIUS    = 0.80f; // evita raio muito pequeno -> z infinito

// ====== offsets do MPU (baseline) ======
float g_off[3]={0}, a_off[3]={0};

// ================== LED helpers ==================
void ledGreenPulse(uint16_t on_ms=180){ digitalWrite(LED_GREEN_PIN, HIGH); delay(on_ms); digitalWrite(LED_GREEN_PIN, LOW); }
void ledRedPulse(uint16_t on_ms=250){ digitalWrite(LED_RED_PIN, HIGH); delay(on_ms); digitalWrite(LED_RED_PIN, LOW); }

static inline void blinkAlive() {
  if (millis()-lastBlink >= 500) {
    lastBlink = millis();
    ledState = !ledState;
    // “batimento” verde — vermelho só para erro/rejeição
    digitalWrite(LED_GREEN_PIN, ledState);
  }
}

// espera o botão ser SOLTO, mas com timeout para não travar
static inline bool waitRelease(uint8_t pin, uint32_t timeout_ms=3000) {
  uint32_t t0 = millis();
  while (digitalRead(pin) == LOW) {
    blinkAlive();
    if (millis() - t0 > timeout_ms) {
      Serial.printf("[BOTAO] Aviso: tempo de espera para soltar (pin %u) excedeu %lu ms. Seguindo mesmo assim.\n",
                    pin, (unsigned long)timeout_ms);
      return false; // não bloqueia o sistema
    }
    delay(10);
  }
  return true;
}

void oledMsg(const String& l1, const String& l2="") {
  if (!oledOK) return;
  display.clearDisplay();
  display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0); display.println(l1);
  if (l2.length()) { display.setCursor(0,12); display.println(l2); }
  display.display();
}

// --- helpers I2C/MPU ---
bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t val){
  Wire.beginTransmission(addr);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission()==0;
}
bool i2cReadBlock(uint8_t addr, uint8_t startReg, uint8_t* buf, size_t len){
  Wire.beginTransmission(addr);
  Wire.write(startReg);
  if (Wire.endTransmission(false)!=0) return false;
  size_t r = Wire.requestFrom(addr, (uint8_t)len);
  if (r!=len) return false;
  for(size_t i=0;i<len;i++) buf[i]=Wire.read();
  return true;
}
static inline int16_t toI16(uint8_t hi, uint8_t lo){ return (int16_t)(((uint16_t)hi<<8)|lo); }

bool mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission()!=0) return false;
  if (!i2cWrite(MPU_ADDR,0x6B,0x00)) return false; // wake
  if (!i2cWrite(MPU_ADDR,0x1C,0x08)) return false; // accel ±4g
  if (!i2cWrite(MPU_ADDR,0x1B,0x08)) return false; // gyro ±500 dps
  // Filtro digital: DLPF_CFG=4 (~20 Hz)
  i2cWrite(MPU_ADDR, 0x1A, 0x04);
  // Sample Rate = 1kHz/(1+9)=100 Hz (com DLPF ativo)
  i2cWrite(MPU_ADDR, 0x19, 0x09);
  return true;
}
bool mpuRead(float& gx,float& gy,float& gz,float& ax,float& ay,float& az){
  uint8_t raw[14];
  if (!i2cReadBlock(MPU_ADDR,0x3B,raw,sizeof(raw))) return false;
  const float aS=8192.0f; // accel ±4g
  const float gS=65.5f;   // gyro ±500 dps
  ax= toI16(raw[0],raw[1]) / aS * 9.80665f;
  ay= toI16(raw[2],raw[3]) / aS * 9.80665f;
  az= toI16(raw[4],raw[5]) / aS * 9.80665f;
  gx= toI16(raw[8],raw[9])   / gS;
  gy= toI16(raw[10],raw[11]) / gS;
  gz= toI16(raw[12],raw[13]) / gS;
  // subtrai offsets (baseline)
  gx -= g_off[0]; gy -= g_off[1]; gz -= g_off[2];
  ax -= a_off[0]; ay -= a_off[1]; az -= a_off[2];
  return true;
}

bool initOLED(){ if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) return false; display.clearDisplay(); display.display(); return true; }
bool initSD(){
  spiSD.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if(!SD.begin(SD_CS_PIN, spiSD)) return false;
  File f = SD.open("/_ok.txt", FILE_WRITE); if(!f) return false; f.println("ok"); f.close(); return true;
}
bool initDFP(){
  dfSerial.begin(9600, SERIAL_8N1, DF_RX_PIN, DF_TX_PIN);
  delay(300);
  for(int i=0;i<5;i++){
    if(dfPlayer.begin(dfSerial)){
      dfPlayer.volume(30);                // volume MÁXIMO
      dfPlayer.EQ(DFPLAYER_EQ_NORMAL);
      return true;
    }
    delay(200);
  }
  return false;
}

// ================== FEATURES (11) ==================
// oversampling + EMA para reduzir ruído
static inline float readFlex(uint8_t pin){
  uint32_t acc = 0;
  for (int i=0;i<8;i++) acc += analogRead(pin);
  float avg = (float)acc / 8.0f;          // 0..4095
  static float ema[40] = {0};             // um slot por possível pino
  float x = avg / 4095.0f;                // 0..1
  const float alpha = 0.20f;              // EMA leve
  ema[pin] = (1-alpha)*ema[pin] + alpha*x;
  return ema[pin];
}

void captureFeatures(float feat[FEATS]) {
  double sum[FEATS]={0};
  uint32_t t0 = millis();
  for (int i=0;i<SAMPLES;i++){
    float ax,ay,az, gx,gy,gz;

    float f1=readFlex(FLEX1_PIN);
    float f2=readFlex(FLEX2_PIN);
    float f3=readFlex(FLEX3_PIN);
    float f4=readFlex(FLEX4_PIN);
    float f5=readFlex(FLEX5_PIN);

    if(!mpuRead(gx,gy,gz, ax,ay,az)) { gx=gy=gz=ax=ay=az=0; }

    sum[0]+=f1; sum[1]+=f2; sum[2]+=f3; sum[3]+=f4; sum[4]+=f5;
    sum[5]+=gx; sum[6]+=gy; sum[7]+=gz;
    sum[8]+=ax; sum[9]+=ay; sum[10]+=az;

    // manter ~50 Hz
    uint32_t target = (uint32_t)((i+1)*SAMPLE_DELAY);
    uint32_t elapsed= millis()-t0;
    if (elapsed<target) delay(target-elapsed);

    blinkAlive();
  }
  for (int j=0;j<FEATS;j++) feat[j]= sum[j] / (double)SAMPLES;
}

// ====== Filenames seguros (ASCII) ======
String slugify(const String& s) {
  String out; out.reserve(s.length()+4);
  for (size_t i=0;i<s.length();++i){
    char c = s[i];
    if      (c==' ')  out += '_';
    else if (c=='á'||c=='à'||c=='â'||c=='ã'||c=='Á'||c=='À'||c=='Â'||c=='Ã') out += 'a';
    else if (c=='é'||c=='ê'||c=='É'||c=='Ê') out += 'e';
    else if (c=='í'||c=='Í') out += 'i';
    else if (c=='ó'||c=='ô'||c=='õ'||c=='Ó'||c=='Ô'||c=='Õ') out += 'o';
    else if (c=='ú'||c=='Ú') out += 'u';
    else if (c=='ç'||c=='Ç') out += 'c';
    else out += c;
  }
  return out;
}

String gestureFile(const String& name, int rep){
  return "/" + slugify(name) + "_rep" + String(rep) + ".csv";
}

bool saveFeaturesCSV(const String& name, int rep, const float feat[FEATS]){
  File f = SD.open(gestureFile(name,rep), FILE_WRITE);
  if(!f) return false;
  f.println("sensor,value");
  f.println("flex1," + String(feat[0],6));
  f.println("flex2," + String(feat[1],6));
  f.println("flex3," + String(feat[2],6));
  f.println("flex4," + String(feat[3],6));
  f.println("flex5," + String(feat[4],6));
  f.println("gyro_x," + String(feat[5],6));
  f.println("gyro_y," + String(feat[6],6));
  f.println("gyro_z," + String(feat[7],6));
  f.println("accel_x," + String(feat[8],6));
  f.println("accel_y," + String(feat[9],6));
  f.println("accel_z," + String(feat[10],6));
  f.close();
  return true;
}

bool loadFeaturesCSV(const String& name, int rep, float feat[FEATS]){
  File f = SD.open(gestureFile(name,rep), FILE_READ);
  if(!f) return false;
  String line = f.readStringUntil('\n'); // header
  for (int i=0;i<FEATS && f.available(); i++){
    line = f.readStringUntil('\n');
    int comma = line.indexOf(',');
    if (comma<0) { f.close(); return false; }
    feat[i] = line.substring(comma+1).toFloat();
  }
  f.close();
  return true;
}

float euclidDist(const float a[FEATS], const float b[FEATS]){
  double s=0;
  for(int i=0;i<FEATS;i++){
    double d=(a[i]-b[i])*FEAT_W[i]; // ponderado
    s+=d*d;
  }
  return sqrt(s);
}

// ====== pré-checagem de imobilidade (gyro_norm < limiar por 0.3s) ======
void waitStableHand(float gyro_lpf = 5.0f){
  Serial.println("[PREP] Aguardando mao parada (gyro_norm < 5 dps por 0.3s)...");
  uint32_t t0=millis(), ok_ms=0;
  while (millis()-t0 < 1500) { // limita a 1.5s
    float gx,gy,gz, ax,ay,az;
    if (!mpuRead(gx,gy,gz, ax,ay,az)) break;
    float norm = sqrtf(gx*gx+gy*gy+gz*gz);
    if (norm < gyro_lpf) ok_ms += 30; else ok_ms = 0;
    if (ok_ms >= 300) break; // 0.3s parado
    delay(30);
  }
}

// ================== AÇÕES ==================
void doCaptureOne(uint8_t gestureIdx, uint8_t repIdx){
  String gName = words[gestureIdx];

  Serial.printf("[CAPTURA] Proxima palavra: \"%s\"  (rep %u/%u)\n", gName.c_str(), repIdx, REPS_PER_WORD);

  // Aguarda mao parada antes de iniciar a janela
  waitStableHand();

  Serial.printf("[CAPTURA] Palavra \"%s\"  captura %u/%u: iniciando agora por 2s...\n",
                gName.c_str(), repIdx, REPS_PER_WORD);
  oledMsg("Capturando", gName+" (rep "+String(repIdx)+")");

  // feedback auditivo APENAS NO INÍCIO DA CAPTURA (volume máximo)
  if (dfOK) { dfPlayer.volume(30); dfPlayer.play(gestureIdx+1); }

  float feat[FEATS];
  captureFeatures(feat);

  bool ok = saveFeaturesCSV(gName, repIdx, feat);
  if (ok) {
    Serial.printf("[CAPTURA] Terminada captura %u da palavra \"%s\". Arquivo: %s\n",
                  repIdx, gName.c_str(), gestureFile(gName,repIdx).c_str());
    oledMsg("Salvo:", gestureFile(gName,repIdx));
    ledGreenPulse(); // sucesso de captura
  } else {
    Serial.printf("[CAPTURA] ERRO ao salvar captura %u da palavra \"%s\"!\n", repIdx, gName.c_str());
    oledMsg("ERRO salvar", gName);
    ledRedPulse();
  }

  if (repIdx < REPS_PER_WORD) {
    Serial.printf("[CAPTURA] Iniciar captura %u da palavra \"%s\" quando pressionar o botao.\n",
                  (uint8_t)(repIdx+1), gName.c_str());
  } else {
    Serial.printf("[CAPTURA] Concluidas %u capturas da palavra \"%s\". Avance para a proxima palavra.\n",
                  REPS_PER_WORD, gName.c_str());
  }

  delay(200);
}

void doPlayRecognize(){
  // Aguarda mao parada antes de iniciar a janela
  waitStableHand();

  // captura atual
  Serial.println("[PLAY] Capturando janela de 2s para reconhecimento (início imediato)...");
  oledMsg("Reproducao","Capturando 2s...");
  float cur[FEATS]; captureFeatures(cur);

  // --- CENTRÓIDE + raio, decisão por z-score + margens relativas ---
  int bestWord=-1;
  float bestD=1e9, secondD=1e9;
  float bestZ=1e9, secondZ=1e9;
  float bestRadius=-1.0f;

  for (int w=0; w<NUM_WORDS; w++){
    // carregar todas as réplicas disponíveis
    float refs[REPS_PER_WORD][FEATS];
    bool has[REPS_PER_WORD]={false,false,false};
    float sum[FEATS]={0};
    int used=0;

    Serial.printf("[PLAY] --- %s ---\n", words[w].c_str());
    for (int r=1; r<=REPS_PER_WORD; r++){
      if (!loadFeaturesCSV(words[w], r, refs[r-1])) continue;
      has[r-1]=true;
      float drep = euclidDist(cur, refs[r-1]);
      Serial.printf("[PLAY]   rep %d  dist=%.6f\n", r, drep);
      for (int k=0;k<FEATS;k++) sum[k]+=refs[r-1][k];
      used++;
    }

    if (used==0){
      Serial.println("[PLAY]   (sem base)");
      continue;
    }

    // centróide
    float centroid[FEATS];
    for (int k=0;k<FEATS;k++) centroid[k] = sum[k] / (float)used;

    // raio (média das distâncias das réplicas ao centróide)
    double rsum=0; int rc=0;
    for (int r=0;r<REPS_PER_WORD;r++){
      if (!has[r]) continue;
      rsum += euclidDist(refs[r], centroid);
      rc++;
    }
    float radius = (rc>0) ? (float)(rsum/rc) : -1.0f;

    float dcent = euclidDist(cur, centroid);
    float denom = max(radius, Z_MIN_RADIUS);
    float z = dcent / denom;

    Serial.printf("[PLAY]   centroid(%d reps) dist=%.6f  radius=%.6f  z=%.3f  limite=%.6f\n",
                  used, dcent, radius, z, (radius>0? K_RADIUS*radius : REJECT_THR_BASE));

    // ranking por z (normalizado por raio)
    if (z < bestZ) {
      secondZ = bestZ; secondD = bestD;
      bestZ = z; bestD = dcent; bestWord = w; bestRadius = radius;
    } else if (z < secondZ) {
      secondZ = z; secondD = dcent;
    }
  }

  if (bestWord<0){
    oledMsg("Sem base","Capture primeiro");
    Serial.println("[PLAY] Nenhuma base encontrada no SD. Faça as capturas no modo CAPTURA.");
    ledRedPulse();
    return;
  }

  // Regras de aceitação
  float limit = (bestRadius > 0) ? (K_RADIUS * bestRadius) : REJECT_THR_BASE;
  float ratio = (bestD > 0 && secondD<1e8) ? (secondD / bestD) : 999.0f;
  float zmargin = (secondZ<1e8) ? (secondZ - bestZ) : 999.0f;

  bool passRadius = (bestD <= limit) || (bestD <= REJECT_THR_BASE);
  bool passMargin = (ratio >= 1.0f + MARGIN_RATIO) || (zmargin >= Z_MARGIN_MIN);

  Serial.printf("[DECISION] word=\"%s\"  d1=%.6f d2=%.6f ratio=%.3f  z1=%.3f z2=%.3f zmargin=%.3f  radius=%.6f limit=%.6f  passRadius=%s passMargin=%s\n",
                words[bestWord].c_str(), bestD, secondD, ratio, bestZ, secondZ, zmargin, bestRadius, limit,
                passRadius?"OK":"NO", passMargin?"OK":"NO");

  if (!(passRadius && passMargin)) {
    if (!passRadius) Serial.printf("[PLAY] Rejeitado por raio/limite: d=%.6f > limite=%.6f (ou > %.6f)\n", bestD, limit, REJECT_THR_BASE);
    if (!passMargin) Serial.printf("[PLAY] Rejeitado por margem: ratio=%.3f (min=%.3f) ou zmargin=%.3f (min=%.3f)\n",
                                   ratio, 1.0f+MARGIN_RATIO, zmargin, Z_MARGIN_MIN);
    oledMsg("Nao reconhecido", "Tente novamente");
    ledRedPulse();
    return;
  }

  Serial.printf("[PLAY] Reconhecido: \"%s\"  d=%.6f  z=%.3f  (limit=%.6f, ratio=%.3f, zmargin=%.3f) -> tocando faixa %d.\n",
                words[bestWord].c_str(), bestD, bestZ, limit, ratio, zmargin, bestWord+1);

  oledMsg("Reconhecido", words[bestWord]);
  ledGreenPulse(); // sucesso de reconhecimento
  if (dfOK){ dfPlayer.volume(30); dfPlayer.play(bestWord+1); }  // volume MÁXIMO
  delay(400);
}

// ================== SETUP / LOOP ==================
void setup(){
  Serial.begin(115200); while(!Serial){delay(10);}

  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN,   OUTPUT);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);

  pinMode(BUTTON_CAPTURE, INPUT_PULLUP);
  pinMode(BUTTON_MODE,    INPUT_PULLUP);

  // ADC estável para flex
  analogSetWidth(12);
  analogSetAttenuation(ADC_11db);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  oledOK = initOLED();
  sdOK   = initSD();
  dfOK   = initDFP();
  mpuOK  = mpuInit();

  if (!oledOK || !sdOK || !dfOK || !mpuOK){
    Serial.printf("[ERRO] Inicializacao: OLED=%s SD=%s DF=%s MPU=%s\n",
      oledOK?"OK":"ERRO", sdOK?"OK":"ERRO", dfOK?"OK":"ERRO", mpuOK?"OK":"ERRO");
    oledMsg("ERRO INICIAL", "Verifique HW");
    ledRedPulse(600);
  }

  // Baseline do MPU (1s em neutro)
  if (mpuOK) {
    Serial.println("[CAL] Medindo baseline (1s)...");
    double gs[3]={0}, as[3]={0};
    const int N=100;
    for(int i=0;i<N;i++){
      uint8_t raw[14];
      i2cReadBlock(MPU_ADDR,0x3B,raw,sizeof(raw));
      const float aS=8192.0f, gS=65.5f;
      float _ax= toI16(raw[0],raw[1]) / aS * 9.80665f;
      float _ay= toI16(raw[2],raw[3]) / aS * 9.80665f;
      float _az= toI16(raw[4],raw[5]) / aS * 9.80665f;
      float _gx= toI16(raw[8],raw[9])   / gS;
      float _gy= toI16(raw[10],raw[11]) / gS;
      float _gz= toI16(raw[12],raw[13]) / gS;
      gs[0]+=_gx; gs[1]+=_gy; gs[2]+=_gz;
      as[0]+=_ax; as[1]+=_ay; as[2]+=_az;
      delay(10);
    }
    for(int k=0;k<3;k++){ g_off[k]=gs[k]/N; a_off[k]=as[k]/N; }
    Serial.printf("[CAL] GyroOff=(%.3f,%.3f,%.3f) AccOff=(%.3f,%.3f,%.3f)\n",
      g_off[0],g_off[1],g_off[2], a_off[0],a_off[1],a_off[2]);
  }

  Serial.println("\n=== STATUS INICIAL ===");
  Serial.printf("OLED=%s SD=%s DF=%s MPU=%s\n",
    oledOK?"OK":"ERRO", sdOK?"OK":"ERRO", dfOK?"OK":"ERRO", mpuOK?"OK":"ERRO");
  Serial.println("Switch MODO (GPIO26): HIGH=CAPTURA, LOW=REPRODUCAO.");
  Serial.println("Botao CAPTURE (GPIO25): clique unico inicia 2s de captacao.");

  oledMsg("Pronto", "HIGH:CAPT  LOW:PLAY");
}

void loop(){
  blinkAlive();

  // Modo definido pelo estado do SWITCH (não toggle)
  AppMode appMode = (digitalRead(BUTTON_MODE)==HIGH) ? MODE_WHEN_HIGH
                                                     : (MODE_WHEN_HIGH==MODE_CAPTURE ? MODE_PLAY : MODE_CAPTURE);

  // Log quando o modo alterar
  static AppMode lastMode = MODE_PLAY;
  if (appMode != lastMode){
    Serial.printf("[MODO] Alterado: %s -> %s\n",
      lastMode==MODE_CAPTURE ? "CAPTURA" : "REPRODUCAO",
      appMode==MODE_CAPTURE ? "CAPTURA" : "REPRODUCAO");
    lastMode = appMode;
  }

  // Mostra no OLED o estado atual (sem flood)
  static uint32_t lastOLED=0;
  if (oledOK && millis()-lastOLED>500) {
    lastOLED=millis();
    display.clearDisplay();
    display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println(appMode==MODE_CAPTURE ? "Modo: CAPTURA" : "Modo: REPRODUCAO");

    if (appMode==MODE_CAPTURE) {
      display.setCursor(0,12);
      display.print("Prox: ");
      display.println(words[currentWord]);
      display.setCursor(0,24);
      display.print("Captura: ");
      display.print(currentRep); display.print("/"); display.print(REPS_PER_WORD);
    } else {
      display.setCursor(0,12); display.println("Clique p/ reconhecer");
    }
    display.display();
  }

  // Botão CAPTURE: clique inicia IMEDIATAMENTE a ação do modo (sem esperar soltar)
  static uint32_t lastCapDeb=0;
  static bool lastBtnState = HIGH;
  bool curBtnState = digitalRead(BUTTON_CAPTURE); // HIGH = solto, LOW = pressionado
  if (lastBtnState == HIGH && curBtnState == LOW && (millis()-lastCapDeb>200)) {
    lastCapDeb = millis();

    // log de modo e switch
    Serial.printf("[MODO] Switch(GPIO26)=%s -> %s\n",
                  (digitalRead(BUTTON_MODE)==HIGH ? "HIGH" : "LOW"),
                  (appMode==MODE_CAPTURE ? "CAPTURA" : "REPRODUCAO"));

    if (!oledOK || !sdOK || !mpuOK) {
      Serial.println("[ERRO] Subsistema faltando (OLED/SD/MPU).");
      oledMsg("ERRO","OLED/SD/MPU?");
      ledRedPulse();
    } else {
      if (appMode==MODE_CAPTURE){
        Serial.printf("[CAPTURA] Proxima palavra: \"%s\"  (rep %u/%u)\n",
                      words[currentWord].c_str(), currentRep, REPS_PER_WORD);
        Serial.printf("[CAPTURA] Palavra atual: \"%s\"  captura %u/%u\n",
                      words[currentWord].c_str(), currentRep, REPS_PER_WORD);
        doCaptureOne(currentWord, currentRep);

        if (currentRep < REPS_PER_WORD) {
          currentRep++;
        } else {
          currentRep = 1;
          currentWord = (currentWord + 1) % NUM_WORDS;
          Serial.printf("[CAPTURA] Avancando para proxima palavra: \"%s\" (rep 1/%u)\n",
                        words[currentWord].c_str(), REPS_PER_WORD);
        }

        Serial.printf("[CAPTURA] Proxima palavra: \"%s\" (captura %u/%u). Pressione o botao para iniciar.\n",
                      words[currentWord].c_str(), currentRep, REPS_PER_WORD);
        oledMsg("Prox palavra",
                words[currentWord] + " (rep " + String(currentRep) + "/" + String(REPS_PER_WORD) + ")");

      } else { // MODE_PLAY
        doPlayRecognize();
      }
    }

    // após executar a ação, aguarda o botão ser solto para evitar re-disparos
    waitRelease(BUTTON_CAPTURE, 1500);
  }
  lastBtnState = curBtnState;
}
