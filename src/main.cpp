#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050_light.h>
#include <SPI.h>
#include <SD.h>
#include <vector>
#include <algorithm>
#include <math.h>
#include <stdarg.h>

// ==== BLE primeiro (evita conflito com macro 'Advertise' do DFPlayer) ====
#include <NimBLEDevice.h>

// ==== DFPlayer depois; desfaz macro problemática se existir ====
#include <DFRobotDFPlayerMini.h>
#ifdef Advertise
#undef Advertise
#endif

// ======================= CONFIG/TUNING =======================
#define SAMPLE_RATE_HZ   50
#define DISCARD_MS       300
#define MIN_HOLD_MS      600
#define MAX_HOLD_MS      4000
#define MIN_SAMPLES      12

#define W_FLEX_DEFAULT   1.20f
#define W_GYRO_DEFAULT   0.35f
#define W_ACCEL_DEFAULT  0.35f

#define GYRO_BASE        50.0f
#define ACCEL_BASE       1.20f

#define K_NEIGHBORS      3
#define FLEX_GATE_PLUS   0.020f
#define MARGIN_RATIO     1.08f

// ======================= PINOS (ESP32 DevKit V1) =======================
#define OLED_SDA        21
#define OLED_SCL        22
#define DF_RX_PIN       16
#define DF_TX_PIN       17
#define FLEX1_PIN       36
#define FLEX2_PIN       39
#define FLEX3_PIN       34
#define FLEX4_PIN       35
#define FLEX5_PIN       32
#define SD_CS_PIN       27
#define SD_SCK_PIN      18
#define SD_MISO_PIN     19
#define SD_MOSI_PIN     23
#define BUTTON_PIN      25   // GRAVA enquanto estiver LOW
#define MODE_SWITCH_PIN 26   // HIGH=CAPTURA  LOW=REPRODUCAO
#define LED_GREEN_PIN   4
#define LED_RED_PIN     5

// ======================= OLED/DF/SD/MPU =======================
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   32
#define OLED_RESET      -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
HardwareSerial dfSerial(1);
DFRobotDFPlayerMini dfPlayer;
SPIClass spiSD(VSPI);
MPU6050 mpu(Wire);

struct Baseline { float gx=0,gy=0,gz=0; float ax=0,ay=0,az=0; } base;

// ======================= VOCAB =======================
static const uint8_t NUM_WORDS = 10;
static const uint8_t REPS_PER_WORD = 3;

static const char* words[NUM_WORDS] = {
  "ola","por favor","obrigado","bom dia","e","meu","nome","boa tarde","ajuda","boa noite"
};
static const char* slugs[NUM_WORDS] = {
  "ola","por_favor","obrigado","bom_dia","e","meu","nome","boa_tarde","ajuda","boa_noite"
};
static const uint8_t trackOfWord[NUM_WORDS] = {1,2,3,4,5,6,7,8,9,10};

enum Mode { CAPTURE=0, PLAY=1 };

// Pesos runtime
float W_FLEX  = W_FLEX_DEFAULT;
float W_GYRO  = W_GYRO_DEFAULT;
float W_ACCEL = W_ACCEL_DEFAULT;

// buffers de referência (até 3 réplicas por palavra)
float meanFlex[NUM_WORDS][REPS_PER_WORD][5]{};
float meanGyro[NUM_WORDS][REPS_PER_WORD]{};
float meanAccel[NUM_WORDS][REPS_PER_WORD]{};
uint8_t repDone[NUM_WORDS]{};
uint8_t currentWord = 0;

bool oledOK=false, sdOK=false, dfOK=false, mpuOK=false;

// ======================= BLE (NUS minimal) =======================
// UUIDs do Nordic UART Service (NUS)
static const char* NUS_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* NUS_RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"; // write
static const char* NUS_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"; // notify/read

NimBLEServer*         bleServer        = nullptr;
NimBLECharacteristic* bleTxChar        = nullptr;
NimBLECharacteristic* bleRxChar        = nullptr;
volatile bool         bleConnected     = false;
volatile bool         bleJustConnected = false;

// ====== OLED: ícone BLE persistente ======
static inline void drawBleIconSmall(int x, int y){
  // desenha "BT" compacto 8x8
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(x, y);
  display.print("BT");
}

static inline void commitDisplay(){
  if(bleConnected){
    // canto superior direito
    drawBleIconSmall(SCREEN_WIDTH-14, 0);
  }
  display.display();
}

// callbacks BLE
class MyServerCallbacks : public NimBLEServerCallbacks {
public:
  void onConnect(NimBLEServer* s) override {
    (void)s;
    bleConnected = true;
    bleJustConnected = true;
  }
  void onDisconnect(NimBLEServer* s) override {
    bleConnected = false;
    bleJustConnected = false;
    s->getAdvertising()->start();
  }
};

class MyRxCallbacks : public NimBLECharacteristicCallbacks {
public:
  void onWrite(NimBLECharacteristic* c) override {
    (void)c; // (podemos usar futuramente para comandos)
  }
};

void BLE_init(const char* devName="Sign Gloves"){
  NimBLEDevice::init(devName);
  NimBLEDevice::setPower(ESP_PWR_LVL_P7);

  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new MyServerCallbacks());

  NimBLEService* service = bleServer->createService(NUS_SERVICE_UUID);

  bleTxChar = service->createCharacteristic(
      NUS_TX_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  bleTxChar->setValue("BOOT\r\n"); // leitura inicial opcional

  bleRxChar = service->createCharacteristic(
      NUS_RX_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
  bleRxChar->setCallbacks(new MyRxCallbacks());

  service->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE_UUID);
  adv->start();
}

// Envia texto pelo TX (notify) – simples: apenas se conectado
void BLE_send(const char* msg){
  if(!bleConnected || !bleTxChar) return;
  const size_t MAX_CHUNK = 20; // MTU padrão segura
  size_t len = strlen(msg);
  const char* p = msg;
  while(len > 0){
    size_t n = (len > MAX_CHUNK) ? MAX_CHUNK : len;
    bleTxChar->setValue((uint8_t*)p, n);
    bleTxChar->notify();
    p   += n;
    len -= n;
    delay(1);
  }
}

// Logger restrito ao necessário
void dbgPrintf(const char* fmt, ...) {
  static char buf[384];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap); // fmt antes de ap
  va_end(ap);
  Serial.print(buf);
  BLE_send(buf);
}
void dbgPrintln(const char* s){ dbgPrintf("%s\r\n", s); }

// ======================= UTILS =======================
void ledGreen(bool on){ digitalWrite(LED_GREEN_PIN,on?HIGH:LOW); }
void ledRed(bool on){ digitalWrite(LED_RED_PIN,on?HIGH:LOW); }
void ledGreenBlink(uint8_t n=1,uint16_t on=80,uint16_t off=80){ for(uint8_t i=0;i<n;i++){ledGreen(true);delay(on);ledGreen(false);delay(off);} }
void ledRedBlink(uint8_t n=1,uint16_t on=120,uint16_t off=120){ for(uint8_t i=0;i<n;i++){ledRed(true);delay(on);ledRed(false);delay(off);} }

void oledMsgSmall(const String& l1,const String& l2=""){
  if(!oledOK) return;
  display.clearDisplay();
  display.setTextWrap(false);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(l1);
  if(l2.length()) display.println(l2);
  commitDisplay(); // <- mantém o BT
}

// Texto grande auto-ajustável ao 128x32
void oledRecognizedBigAuto(const String& wordUpper){
  if(!oledOK) return;
  display.clearDisplay();
  display.setTextWrap(false);

  uint8_t sizes[3] = {3,2,1};
  int chosen = 1;
  int16_t x1,y1; uint16_t w,h;
  for(uint8_t s: sizes){
    display.setTextSize(s);
    display.getTextBounds(wordUpper, 0, 0, &x1, &y1, &w, &h);
    if(w <= SCREEN_WIDTH && h <= SCREEN_HEIGHT){ chosen = s; break; }
  }
  display.setTextSize(chosen);
  display.getTextBounds(wordUpper, 0, 0, &x1, &y1, &w, &h);

  int y = (SCREEN_HEIGHT - (int)h)/2;
  int x = (SCREEN_WIDTH  - (int)w)/2;
  if(x<0) x=0; if(y<0) y=0;

  display.setCursor(x,y);
  display.println(wordUpper);
  commitDisplay(); // <- mantém o BT
}

inline Mode readMode(){ return (digitalRead(MODE_SWITCH_PIN)==HIGH)?CAPTURE:PLAY; }

// ===== Botão com debounce =====
struct DebouncedBtn {
  bool stable=false, lastStable=false, raw=false;
  uint32_t tLast=0;
  const uint16_t debounceMs=30;
  void update(){
    bool r = (digitalRead(BUTTON_PIN)==LOW);
    uint32_t now=millis();
    if(r!=raw){ raw=r; tLast=now; }
    if(now - tLast >= debounceMs){ lastStable=stable; stable=raw; }
  }
  bool pressedEdge()  { return (!lastStable && stable); }
  bool releasedEdge() { return ( lastStable && !stable); }
  bool isPressed()    { return stable; }
} btn;

// ======================= SD helpers =======================
bool ensureRefsFolder(){
  if(!sdOK) return false;
  if(!SD.exists("/refs")) return SD.mkdir("/refs");
  return true;
}
bool clearRefsFolder(){
  if(!sdOK) return false;
  if(!ensureRefsFolder()) return false;
  File dir = SD.open("/refs");
  if(!dir) return false;
  while(true){
    File f = dir.openNextFile();
    if(!f) break;
    if(!f.isDirectory()){
      String name = f.name();
      f.close();
      SD.remove(name.c_str());
      dbgPrintf("[SD] Removido: %s\r\n", name.c_str());
    }else f.close();
  }
  dir.close();
  return true;
}
bool saveCSV(uint8_t w,uint8_t r,const float flex[5],float gyroNorm,float accelNorm){
  if(!sdOK) return false;
  char path[64]; snprintf(path,sizeof(path),"/refs/%s_rep%u.csv",slugs[w],(unsigned)(r+1));
  File f = SD.open(path, FILE_WRITE);
  if(!f){ dbgPrintf("[SD] Falha ao abrir %s\r\n",path); return false; }
  f.println("sensor,value");
  for(int i=0;i<5;i++) f.printf("flex%d,%.6f\n",i+1,flex[i]);
  f.printf("gyro_norm,%.6f\n",gyroNorm);
  f.printf("accel_norm,%.6f\n",accelNorm);
  f.close();
  dbgPrintf("[CAPTURA] Salvo: %s\r\n",path);
  return true;
}
bool loadCSV_toWM(const char* path, float outFlex[5], float& outGyro, float& outAccel){
  if(!sdOK) return false;
  File f = SD.open(path, FILE_READ);
  if(!f){ return false; }
  for(int i=0;i<5;i++) outFlex[i]=0.0f;
  outGyro=0.0f; outAccel=0.0f;
  while(f.available()){
    String line = f.readStringUntil('\n');
    line.trim();
    if(line.length()==0) continue;
    if(line.startsWith("sensor")) continue;
    int comma = line.indexOf(',');
    if(comma<0) continue;
    String key = line.substring(0, comma);
    String sval= line.substring(comma+1);
    sval.trim();
    float val = sval.toFloat();
    if     (key=="flex1") outFlex[0]=val;
    else if(key=="flex2") outFlex[1]=val;
    else if(key=="flex3") outFlex[2]=val;
    else if(key=="flex4") outFlex[3]=val;
    else if(key=="flex5") outFlex[4]=val;
    else if(key=="gyro_norm")  outGyro = val;
    else if(key=="accel_norm") outAccel= val;
  }
  f.close();
  return true;
}
bool loadAllRefsFromSD(){
  if(!sdOK) return false;
  if(!ensureRefsFolder()) return false;
  memset(meanFlex,0,sizeof(meanFlex));
  memset(meanGyro,0,sizeof(meanGyro));
  memset(meanAccel,0,sizeof(meanAccel));
  memset(repDone,0,sizeof(repDone));
  currentWord=0;
  uint16_t loaded=0;
  for(uint8_t w=0; w<NUM_WORDS; w++){
    for(uint8_t r=0; r<REPS_PER_WORD; r++){
      char path[64]; snprintf(path,sizeof(path),"/refs/%s_rep%u.csv", slugs[w], (unsigned)(r+1));
      if(SD.exists(path)){
        float flex[5]; float g=0,a=0;
        if(loadCSV_toWM(path, flex, g, a)){
          for(int i=0;i<5;i++) meanFlex[w][r][i]=flex[i];
          meanGyro[w][r]=g;
          meanAccel[w][r]=a;
          if(repDone[w] < r+1) repDone[w] = r+1;
          loaded++;
        }
      }
    }
  }
  for(uint8_t w=0; w<NUM_WORDS; w++){
    if(repDone[w] < REPS_PER_WORD){ currentWord = w; break; }
    if(w==NUM_WORDS-1) currentWord = NUM_WORDS;
  }
  dbgPrintf("[SD] Referencias carregadas: %u\r\n", loaded);
  return (loaded>0);
}

// ======================= Feature window =======================
struct WindowMeans{ float flex[5]; float gyroNorm; float accelNorm; };
static inline float relDiff(float a,float b,float baseRef){
  float M=fmaxf(fmaxf(a,b),baseRef);
  float v=fabsf(a-b)/M; if(v>1.f) v=1.f; return v;
}
struct DistParts{ float dflex,dg,da,total; };
DistParts weightedDistanceParts(const WindowMeans& probe,const WindowMeans& ref){
  DistParts p{};
  float df=0; for(int i=0;i<5;i++) df += fabsf(probe.flex[i]-ref.flex[i]);
  p.dflex=df/5.f;
  float dg = relDiff(probe.gyroNorm,  ref.gyroNorm,  GYRO_BASE);
  float da = relDiff(probe.accelNorm, ref.accelNorm, ACCEL_BASE);
  p.dg=dg; p.da=da;
  p.total = W_FLEX*p.dflex + W_GYRO*dg + W_ACCEL*da;
  return p;
}

// ======================= Gravador =======================
struct Recorder {
  bool active=false;
  uint32_t tStart=0, tLastSample=0;
  double sumFlex[5]{};
  double sumGyro=0, sumAccel=0;
  uint16_t n=0;

  void start(){
    active=true; tStart=millis(); tLastSample=0;
    sumFlex[0]=sumFlex[1]=sumFlex[2]=sumFlex[3]=sumFlex[4]=0;
    sumGyro=0; sumAccel=0; n=0;
  }
  void tick(){
    if(!active) return;
    uint32_t now=millis();
    const uint16_t period = 1000 / SAMPLE_RATE_HZ;
    if(tLastSample==0 || (now - tLastSample) >= period){
      tLastSample=now;
      mpu.update();
      int raw1=analogRead(FLEX1_PIN), raw2=analogRead(FLEX2_PIN),
          raw3=analogRead(FLEX3_PIN), raw4=analogRead(FLEX4_PIN),
          raw5=analogRead(FLEX5_PIN);
      float f1=raw1/4095.f, f2=raw2/4095.f, f3=raw3/4095.f, f4=raw4/4095.f, f5=raw5/4095.f;
      float gx=(mpu.getGyroX()-base.gx);
      float gy=(mpu.getGyroY()-base.gy);
      float gz=(mpu.getGyroZ()-base.gz);
      float ax=(mpu.getAccX()-base.ax);
      float ay=(mpu.getAccY()-base.ay);
      float az=(mpu.getAccZ()-base.az);
      float gnorm=sqrtf(gx*gx+gy*gy+gz*gz);
      float anorm=sqrtf(ax*ax+ay*ay+az*az);
      if(now - tStart >= DISCARD_MS){
        sumFlex[0]+=f1; sumFlex[1]+=f2; sumFlex[2]+=f3; sumFlex[3]+=f4; sumFlex[4]+=f5;
        sumGyro+=gnorm; sumAccel+=anorm; n++;
      }
    }
  }
  bool enough() const { return (millis()-tStart)>=MIN_HOLD_MS && n>=MIN_SAMPLES; }
  WindowMeans stop(){
    active=false;
    WindowMeans wm{};
    if(n==0){ for(int i=0;i<5;i++) wm.flex[i]=0; wm.gyroNorm=0; wm.accelNorm=0; return wm; }
    for(int i=0;i<5;i++) wm.flex[i]=sumFlex[i]/n;
    wm.gyroNorm = sumGyro/n;
    wm.accelNorm= sumAccel/n;
    return wm;
  }
} rec;

// ======================= Sessão CAPTURA =======================
void resetCaptureSession(bool announce=true){
  memset(meanFlex,0,sizeof(meanFlex));
  memset(meanGyro,0,sizeof(meanGyro));
  memset(meanAccel,0,sizeof(meanAccel));
  memset(repDone,0,sizeof(repDone));
  currentWord=0;
  if(announce){
    dbgPrintln("[CAPTURA] Nova sessao");
    oledMsgSmall("CAPTURA", String(words[currentWord])+" rep 1 (pronto)");
  }
}

// ======================= Setup =======================
void setup(){
  pinMode(LED_GREEN_PIN,OUTPUT);
  pinMode(LED_RED_PIN,OUTPUT);
  pinMode(BUTTON_PIN,INPUT_PULLUP);
  pinMode(MODE_SWITCH_PIN,INPUT);
  ledGreen(false); ledRed(false);

  Serial.begin(115200); delay(150);

  // BLE
  BLE_init("Sign Gloves");
  delay(150);

  // I2C / OLED
  Wire.begin(OLED_SDA,OLED_SCL);
oledOK = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
if(oledOK){
  display.setRotation(2);          // <<< inverte 180° para uso na luva
  oledMsgSmall("OLED OK");
} else {
  dbgPrintln("[OLED] FALHA");
}


  // SD
  dbgPrintln("[SD] Inicializando...");
  pinMode(SD_CS_PIN,OUTPUT); digitalWrite(SD_CS_PIN,HIGH);
  spiSD.begin(SD_SCK_PIN,SD_MISO_PIN,SD_MOSI_PIN,SD_CS_PIN);
  sdOK=SD.begin(SD_CS_PIN, spiSD, 10*1000*1000);
  if(!sdOK){ dbgPrintln("[SD] 10MHz falhou. 4MHz..."); sdOK=SD.begin(SD_CS_PIN, spiSD, 4*1000*1000); }
  dbgPrintln(sdOK? "[SD] OK":"[SD] FALHA");

  // DFPlayer
  dfSerial.begin(9600, SERIAL_8N1, DF_RX_PIN, DF_TX_PIN);
  dfOK=dfPlayer.begin(dfSerial);
  if(dfOK){ dfPlayer.volume(25); dbgPrintln("[DFP] OK"); } else dbgPrintln("[DFP] FALHA");

  // MPU
  dbgPrintln("[MPU] Inicializando...");
  mpuOK=(mpu.begin()==0);
  if(mpuOK){
    mpu.calcGyroOffsets();
    uint32_t t0=millis(); uint16_t n=0; double sgx=0,sgy=0,sgz=0,sax=0,say=0,saz=0;
    while(millis()-t0<1000){ mpu.update(); sgx+=mpu.getGyroX(); sgy+=mpu.getGyroY(); sgz+=mpu.getGyroZ(); sax+=mpu.getAccX(); say+=mpu.getAccY(); saz+=mpu.getAccZ(); n++; delay(5); }
    if(n==0) n=1;
    base.gx=sgx/n; base.gy=sgy/n; base.gz=sgz/n; base.ax=sax/n; base.ay=say/n; base.az=saz/n;
    dbgPrintln("[MPU] OK");
  }else{
    dbgPrintln("[MPU] FALHA");
  }

  // status inicial
  dbgPrintf("[STATUS] OLED=%s SD=%s DF=%s MPU=%s\r\n",
            oledOK?"OK":"FALHA", sdOK?"OK":"FALHA", dfOK?"OK":"FALHA", mpuOK?"OK":"FALHA");

  Mode modeStart = readMode();
  if(modeStart==CAPTURE){
    if(sdOK){ clearRefsFolder(); }
    resetCaptureSession(true);
  }else{
    bool ok = loadAllRefsFromSD();
    if(!ok) dbgPrintln("[PLAY] Sem referencias em /refs");
    oledMsgSmall("REPRODUCAO","Segure para ler");
  }
}

// ======================= Loop =======================
struct Neighbor{ uint8_t w,r; float d,dflex,dg,da; };

void loop(){
  static Mode lastMode=readMode();
  Mode mode=readMode();

  static int lastShownWord = -1;
  static int lastShownRep  = -1;

  if(bleJustConnected){
    bleJustConnected = false;
    // indicador persistente já aparece no OLED
    dbgPrintln("[BLE] Conectado");
  }

  if(mode!=lastMode){
    if(mode==CAPTURE){
      if(sdOK){ clearRefsFolder(); }
      resetCaptureSession(true);
      lastShownWord = -1;
      lastShownRep  = -1;
      dbgPrintln("[MODO] CAPTURA");
    } else {
      bool ok = loadAllRefsFromSD();
      if(!ok) dbgPrintln("[PLAY] Sem referencias no SD. Faça CAPTURA primeiro.");
      oledMsgSmall("REPRODUCAO","Segure para ler");
      dbgPrintln("[MODO] REPRODUCAO");
    }
    lastMode=mode;
  }

  btn.update();
  if(rec.active) rec.tick();

  // ====== CAPTURA ======
  if(mode==CAPTURE){
    if(!rec.active && currentWord<NUM_WORDS){
      int repIdx = repDone[currentWord] + 1;
      if(lastShownWord!=currentWord || lastShownRep!=repIdx){
        lastShownWord = currentWord;
        lastShownRep  = repIdx;
        // log enxuto: palavra + repetição
        dbgPrintf("[CAPTURA] %s rep %d PRONTO\r\n", words[currentWord], repIdx);
        oledMsgSmall("CAPTURA", String(words[currentWord])+" rep "+String(repIdx)+" (pronto)");
      }
    }

    if(currentWord<NUM_WORDS){
      if(btn.pressedEdge()){
        ledGreen(true);
        oledMsgSmall("CAPTURA", String(words[currentWord])+" (gravando)");
        if(dfOK) dfPlayer.playMp3Folder(trackOfWord[currentWord]);
        rec.start();
      }
      if(btn.releasedEdge() && rec.active){
        WindowMeans wm = rec.stop();
        ledGreen(false);
        if(!rec.enough()){
          oledMsgSmall("CAPTURA","Muito curto — tente de novo");
          ledRedBlink(2);
        }else{
          uint8_t r=repDone[currentWord];
          for(int i=0;i<5;i++) meanFlex[currentWord][r][i]=wm.flex[i];
          meanGyro[currentWord][r]=wm.gyroNorm;
          meanAccel[currentWord][r]=wm.accelNorm;
          if(sdOK) saveCSV(currentWord, r, wm.flex, wm.gyroNorm, wm.accelNorm);
          repDone[currentWord]++;
          // confirmação de captura OK (curta)
          dbgPrintf("[OK] %s rep %u\r\n", words[currentWord], r+1);
          ledGreenBlink(1);
          if(repDone[currentWord] >= REPS_PER_WORD){
            currentWord++;
          }
          if(currentWord<NUM_WORDS){
            oledMsgSmall("CAPTURA", String(words[currentWord])+" rep "+String(repDone[currentWord]+1)+" (pronto)");
            lastShownWord = -1; lastShownRep = -1;
          }else{
            oledMsgSmall("CAPTURA","Concluida — troque de modo");
            dbgPrintln("[FIM] Capturas concluidas");
          }
        }
      }
    }

  // ====== REPRODUÇÃO ======
  }else{
    if(btn.pressedEdge()){
      ledGreen(true);
      rec.start();
      oledMsgSmall("REPRODUCAO","Gravando...");
    }
    if(btn.releasedEdge() && rec.active){
      WindowMeans probe = rec.stop();
      ledGreen(false);

      bool haveAny=false;
      for(uint8_t w=0;w<NUM_WORDS;w++) if(repDone[w]>0) { haveAny=true; break; }
      if(!haveAny || !rec.enough()){
        oledMsgSmall("REPRODUCAO", haveAny?"Muito curto":"Sem refs");
        ledRedBlink(2);
        return;
      }

      std::vector<Neighbor> pool; pool.reserve(NUM_WORDS*REPS_PER_WORD);
      float bestDFlex=1e9f;
      for(uint8_t w=0;w<NUM_WORDS;w++){
        for(uint8_t r=0;r<repDone[w];r++){
          WindowMeans ref{};
          for(int i=0;i<5;i++) ref.flex[i]=meanFlex[w][r][i];
          ref.gyroNorm=meanGyro[w][r];
          ref.accelNorm=meanAccel[w][r];
          DistParts parts=weightedDistanceParts(probe,ref);
          pool.push_back({w,r,parts.total,parts.dflex,parts.dg,parts.da});
          if(parts.dflex<bestDFlex) bestDFlex=parts.dflex;
        }
      }
      if(pool.empty()){ oledMsgSmall("REPRODUCAO","Sem refs"); ledRedBlink(2); return; }
      std::sort(pool.begin(),pool.end(),[](const Neighbor&a,const Neighbor&b){return a.d<b.d;});

      std::vector<Neighbor> gated; gated.reserve(pool.size());
      float gate = bestDFlex + FLEX_GATE_PLUS;
      for(auto& nb: pool) if(nb.dflex<=gate) gated.push_back(nb);
      if(gated.empty()) gated=pool;
      if((int)gated.size()>K_NEIGHBORS) gated.resize(K_NEIGHBORS);

      float score[NUM_WORDS]={0};
      const float KNN_EPS = 1e-6f;
      for(auto& nb: gated){ float wgt = 1.0f / (KNN_EPS + nb.d * nb.d); score[nb.w]+=wgt; }

      int win=-1, run=-1; float sWin=-1, sRun=-1;
      for(uint8_t w=0;w<NUM_WORDS;w++){
        if(score[w]>sWin){ sRun=sWin; run=win; sWin=score[w]; win=w; }
        else if(score[w]>sRun){ sRun=score[w]; run=w; }
      }
      int agree=0; for(auto& nb: gated) if(nb.w==win) agree++;

      if(win>=0 && agree>=2 && sWin >= MARGIN_RATIO*sRun){
        // só o essencial: palavra reconhecida
        dbgPrintf("[REC] %s\r\n", words[win]);
        if(dfOK) dfPlayer.playMp3Folder(trackOfWord[win]);
        String upper = String(words[win]); upper.toUpperCase();
        oledRecognizedBigAuto(upper);
        ledGreenBlink(2);
      }else{
        oledMsgSmall("Nao reconhecido","");
        ledRedBlink(2);
      }
    }
  }

  delay(1);
}
