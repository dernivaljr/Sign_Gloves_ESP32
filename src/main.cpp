#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DFRobotDFPlayerMini.h>
#include <HardwareSerial.h>

// ================== PINAGEM (ESP32 DevKit V1) ==================
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   32
#define OLED_RESET      -1
const uint8_t OLED_ADDR = 0x3C;

#define LED_GREEN_PIN   4
#define LED_RED_PIN    5

#define SWITCH_PIN     26   // BOTÃO de MODO (INPUT_PULLUP)
#define BUTTON_PIN     25   // BOTÃO de CAPTURA (INPUT_PULLUP)

#define DF_RX_PIN      16   // ESP32 RX1  <= TX do DFPlayer
#define DF_TX_PIN      17   // ESP32 TX1  => RX do DFPlayer

#define SD_CS_PIN      27
#define SD_MISO_PIN    19
#define SD_MOSI_PIN    23
#define SD_SCK_PIN     18

// ================== OBJETOS ==================
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
SPIClass spiSD(VSPI);
HardwareSerial dfSerial(1);
DFRobotDFPlayerMini dfPlayer;

// ================== ESTADO ==================
enum TestMode { TM_BASE=0, TM_SDWRITE=1, TM_MP3SEQ=2 };
volatile TestMode mode = TM_BASE;
uint8_t track = 1;                // 1..10
bool ledState = false;

uint32_t lastBlink = 0;
uint32_t lastOLED  = 0;
uint32_t lastInfo  = 0;

bool sdOK   = false;
bool oledOK = false;
bool dfOK   = false;
String lastOpMsg = "-";

// ======= debounce simples =======
bool readBtn(uint8_t pin) {
  static uint32_t tLast[2] = {0,0};
  static bool     last[2]  = {true,true};
  static bool     stable[2]= {true,true};
  uint8_t idx = (pin==SWITCH_PIN)?0:1;

  bool v = digitalRead(pin); // true=HIGH (solto), false=LOW (pressionado com pullup)
  if (v != last[idx]) {
    last[idx] = v;
    tLast[idx] = millis();
  }
  if (millis() - tLast[idx] > 20) {
    stable[idx] = v;
  }
  return !stable[idx]; // retorna true quando PRESS (LOW)
}

// ======= OLED helper =======
void oledPrintStatus() {
  if (!oledOK) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Linha 0: modo + faixa
  display.setCursor(0,0);
  display.print(F("Modo: "));
  if      (mode==TM_BASE)    display.print(F("TESTE"));
  else if (mode==TM_SDWRITE) display.print(F("SD-WRITE"));
  else                       display.print(F("MP3-SEQ"));
  display.setCursor(90,0);
  display.print(F("Trk:"));
  display.print(track);

  // Linha 1: botoes
  display.setCursor(0,10);
  display.print(F("MODE:"));
  display.print(digitalRead(SWITCH_PIN)==LOW ? F("ON ") : F("OFF"));
  display.print(F("  CAP:"));
  display.print(digitalRead(BUTTON_PIN)==LOW ? F("ON") : F("OFF"));

  // Linha 2: SD/DF
  display.setCursor(0,20);
  display.print(F("SD:"));
  display.print(sdOK?F("OK "):F("ERRO "));
  display.print(F("DFP:"));
  display.print(dfOK?F("OK "):F("ERRO"));

  display.display();
}

// ======= Inicializações individuais =======
bool initOLED() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) return false;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0); display.println(F("OLED OK"));
  display.setCursor(0,10); display.println(F("128x32 I2C 0x3C"));
  display.display();
  return true;
}

bool initSD() {
  spiSD.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN, spiSD)) return false;
  // teste simples: cria/ler arquivo
  File f = SD.open("/selftest.txt", FILE_WRITE);
  if (!f) return false;
  f.println("Selftest SD OK");
  f.close();
  f = SD.open("/selftest.txt", FILE_READ);
  if (!f) return false;
  String line = f.readStringUntil('\n');
  f.close();
  return line.indexOf("Selftest SD OK") >= 0;
}

bool initDFPlayer() {
  dfSerial.begin(9600, SERIAL_8N1, DF_RX_PIN, DF_TX_PIN);
  delay(300);
  for (int i=0;i<5;i++) {
    if (dfPlayer.begin(dfSerial)) {
      dfPlayer.volume(30);             // volume no MÁXIMO
      dfPlayer.EQ(DFPLAYER_EQ_NORMAL);
      return true;
    }
    delay(250);
  }
  return false;
}

// ======= Ações de teste =======
void toggleLeds() {
  ledState = !ledState;
  digitalWrite(LED_GREEN_PIN, ledState);
  digitalWrite(LED_RED_PIN,   !ledState);
}

void doSDWriteBurst() {
  if (!sdOK) { lastOpMsg = "SD NAO INICIALIZADO"; return; }
  File f = SD.open("/burst.csv", FILE_APPEND);
  if (!f) { lastOpMsg = "FALHA OPEN burst.csv"; return; }
  uint32_t now = millis();
  for (int i=0;i<10;i++) {
    f.print(now+i); f.print(',');
    f.print(analogRead(32)); f.print(',');
    f.print(analogRead(34)); f.print(',');
    f.print(analogRead(35)); f.println();
  }
  f.close();
  lastOpMsg = "SD burst.csv OK (+10 linhas)";
}

void playNextTrack() {
  if (!dfOK) { lastOpMsg = "DFP NAO INICIALIZADO"; return; }
  dfPlayer.volume(30); // garante máximo
  dfPlayer.play(track);
  lastOpMsg = String("Play ")+track;
  track++; if (track>10) track=1;
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  while(!Serial){}

  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN,   OUTPUT);
  pinMode(SWITCH_PIN,    INPUT_PULLUP);
  pinMode(BUTTON_PIN,    INPUT_PULLUP);

  Serial.println();
  Serial.println(F("===== AUTOTESTE COMPLETO ====="));

  oledOK = initOLED();
  Serial.print(F("[OLED] ")); Serial.println(oledOK?F("OK"):F("FALHA"));

  sdOK = initSD();
  Serial.print(F("[SD] ")); Serial.println(sdOK?F("OK"):F("FALHA"));

  dfOK = initDFPlayer();
  Serial.print(F("[DFP] ")); Serial.println(dfOK?F("OK"):F("FALHA (verifique TX/RX, VCC/GND, SD com 0001.mp3..0010.mp3)"));

  oledPrintStatus();
  lastOpMsg = "-";
}

// ================== LOOP ==================
void loop() {
  // Blink de vida a cada 500 ms
  if (millis() - lastBlink >= 500) {
    lastBlink = millis();
    toggleLeds();
  }

  // Botão de MODO: alterna o modo
  static bool lastModeState = HIGH;
  bool modePressed = readBtn(SWITCH_PIN);
  if (modePressed && lastModeState==HIGH) {
    mode = (TestMode)(((int)mode + 1) % 3);
    lastOpMsg = String("Modo=") + (mode==TM_BASE?"TESTE":(mode==TM_SDWRITE?"SD-WRITE":"MP3-SEQ"));
  }
  lastModeState = !modePressed ? HIGH : LOW;

  // Botão de CAPTURA: toca próxima faixa
  static bool lastCapState = HIGH;
  bool capPressed = readBtn(BUTTON_PIN);
  if (capPressed && lastCapState==HIGH) {
    playNextTrack();
  }
  lastCapState = !capPressed ? HIGH : LOW;

  // Executa ação do modo atual
  if (mode == TM_SDWRITE) {
    static uint32_t lastSd = 0;
    if (millis() - lastSd >= 1500) {
      lastSd = millis();
      doSDWriteBurst(); // escreve 10 linhas no burst.csv
    }
  } else if (mode == TM_MP3SEQ) {
    static uint32_t lastPlay = 0;
    if (millis() - lastPlay >= 2000) {
      lastPlay = millis();
      playNextTrack();
    }
  }

  // Atualiza OLED a cada 250 ms com estados + última operação
  if (oledOK && millis() - lastOLED >= 250) {
    lastOLED = millis();
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    display.setCursor(0,0);
    display.print(F("M:"));
    display.print(mode==TM_BASE?"TEST":(mode==TM_SDWRITE?"SDWR":"MP3"));
    display.setCursor(60,0);
    display.print(F("Trk:")); display.print(track);
    display.setCursor(100,0);
    display.print(F("V:30"));

    display.setCursor(0,10);
    display.print(F("MODE:"));
    display.print(digitalRead(SWITCH_PIN)==LOW?"ON ":"OFF");
    display.print(F(" CAP:"));
    display.print(digitalRead(BUTTON_PIN)==LOW?"ON":"OFF");

    display.setCursor(0,20);
    display.print(F("SD:"));  display.print(sdOK?"OK ":"ERRO");
    display.print(F(" DFP:"));display.print(dfOK?"OK ":"ERRO");

    display.display();
  }

  // Log periódico no Serial com última operação
  if (millis() - lastInfo >= 1500) {
    lastInfo = millis();
    Serial.print(F("[INFO] ")); Serial.println(lastOpMsg);
  }
}
