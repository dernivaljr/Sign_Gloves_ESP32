#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DFRobotDFPlayerMini.h>

// ======================= PINOS (ESP32 DevKit V1) =======================
#define OLED_SDA        21
#define OLED_SCL        22
#define DF_RX_PIN       16
#define DF_TX_PIN       17
#define BUTTON_PIN      25
#define MODE_SWITCH_PIN 26   // HIGH=CONFIGURACAO  LOW=EXPERIMENTO
#define LED_GREEN_PIN   4
#define LED_RED_PIN     5

// ======================= OLED / DFPlayer =======================
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   32
#define OLED_RESET      -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
HardwareSerial dfSerial(1);
DFRobotDFPlayerMini dfPlayer;

static bool oledOK = false;
static bool dfOK = false;

struct Scenario {
  const char* label;
  const char* result;
  uint8_t track;
};

static const Scenario scenarios[] = {
  {"1 BANCO", "CARTAO BLOQUEADO", 1},
  {"2 CLINICA", "CONSULTA ATRASADA", 2},
  {"3 FACULDADE", "MATRICULA", 3},
};

static const uint8_t SCENARIO_COUNT = sizeof(scenarios) / sizeof(scenarios[0]);
static uint8_t selectedScenario = 0;

enum class Mode {
  Config,
  Experiment,
};

enum class ExperimentState {
  Ready,
  Capturing,
  Interpreting,
  Result,
};

static Mode currentMode = Mode::Experiment;
static ExperimentState experimentState = ExperimentState::Ready;
static uint32_t stateStartedAt = 0;
static uint32_t lastAudioErrorBlinkAt = 0;
static bool audioErrorLedOn = false;

struct DebouncedButton {
  bool stable = false;
  bool previousStable = false;
  bool raw = false;
  uint32_t rawChangedAt = 0;
  static const uint16_t debounceMs = 30;

  void update() {
    const bool reading = (digitalRead(BUTTON_PIN) == LOW);
    const uint32_t now = millis();

    if (reading != raw) {
      raw = reading;
      rawChangedAt = now;
    }

    previousStable = stable;
    if (now - rawChangedAt >= debounceMs) {
      stable = raw;
    }
  }

  bool pressedEdge() const {
    return !previousStable && stable;
  }

  bool releasedEdge() const {
    return previousStable && !stable;
  }

  bool isPressed() const {
    return stable;
  }
};

static DebouncedButton button;

static void ledGreen(bool on) {
  digitalWrite(LED_GREEN_PIN, on ? HIGH : LOW);
}

static void ledRed(bool on) {
  digitalWrite(LED_RED_PIN, on ? HIGH : LOW);
}

static Mode readMode() {
  return (digitalRead(MODE_SWITCH_PIN) == HIGH) ? Mode::Config : Mode::Experiment;
}

static void showLines(const char* line1, const char* line2 = "") {
  if (!oledOK) {
    return;
  }

  display.clearDisplay();
  display.setTextWrap(false);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(line1);
  if (line2 && line2[0] != '\0') {
    display.println(line2);
  }
  display.display();
}

static void showConfig() {
  showLines("CONFIGURACAO", scenarios[selectedScenario].label);
  Serial.printf("[CONFIG] Cenario selecionado: %s\r\n", scenarios[selectedScenario].label);
}

static void showReady() {
  showLines("SIGN GLOVES", "PRONTO");
  Serial.printf("[EXPERIMENTO] Pronto. Cenario=%s Track=%u\r\n",
                scenarios[selectedScenario].label,
                scenarios[selectedScenario].track);
}

static void setExperimentReady() {
  ledGreen(false);
  experimentState = ExperimentState::Ready;
  stateStartedAt = millis();
  showReady();
}

static void showAudioErrorBriefly() {
  Serial.println("[DFP] FALHA");
  showLines("ERRO AUDIO");
  for (uint8_t i = 0; i < 6; i++) {
    ledRed(true);
    delay(120);
    ledRed(false);
    delay(120);
  }
}

static void keepAudioErrorVisible() {
  if (dfOK || experimentState != ExperimentState::Ready) {
    return;
  }

  const uint32_t now = millis();
  if (now - lastAudioErrorBlinkAt >= 500) {
    lastAudioErrorBlinkAt = now;
    audioErrorLedOn = !audioErrorLedOn;
    ledRed(audioErrorLedOn);
  }
}

static void startCapturing() {
  ledGreen(true);
  experimentState = ExperimentState::Capturing;
  stateStartedAt = millis();
  showLines("CAPTANDO...");
  Serial.printf("[BOTAO] Pressionado. Cenario=%s\r\n", scenarios[selectedScenario].label);
}

static void startInterpreting() {
  ledGreen(false);
  experimentState = ExperimentState::Interpreting;
  stateStartedAt = millis();
  showLines("INTERPRETANDO...");
  Serial.println("[BOTAO] Solto. Interpretando...");
}

static void showResultAndPlay() {
  const Scenario& scenario = scenarios[selectedScenario];

  experimentState = ExperimentState::Result;
  stateStartedAt = millis();
  showLines(scenario.result);

  Serial.printf("[RESULTADO] %s | Track=%u | Arquivo=/mp3/%04u.mp3\r\n",
                scenario.result,
                scenario.track,
                scenario.track);

  if (dfOK) {
    dfPlayer.playMp3Folder(scenario.track);
  } else {
    Serial.println("[DFP] Track nao tocada: DFPlayer indisponivel");
  }
}

static void handleModeChange(Mode nextMode) {
  currentMode = nextMode;
  ledGreen(false);
  experimentState = ExperimentState::Ready;
  stateStartedAt = millis();

  if (currentMode == Mode::Config) {
    Serial.println("[MODO] CONFIGURACAO");
    showConfig();
  } else {
    Serial.println("[MODO] EXPERIMENTO");
    setExperimentReady();
  }
}

void setup() {
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(MODE_SWITCH_PIN, INPUT);

  ledGreen(false);
  ledRed(false);

  Serial.begin(115200);
  delay(150);
  Serial.println();
  Serial.println("[BOOT] Sign Gloves Explorer 2026");

  Wire.begin(OLED_SDA, OLED_SCL);
  oledOK = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (oledOK) {
    display.setRotation(2);
    showLines("SIGN GLOVES", "EXPLORER 2026");
  } else {
    Serial.println("[OLED] FALHA");
  }

  dfSerial.begin(9600, SERIAL_8N1, DF_RX_PIN, DF_TX_PIN);
  dfOK = dfPlayer.begin(dfSerial);
  if (dfOK) {
    dfPlayer.volume(25);
    Serial.println("[DFP] OK");
  } else {
    showAudioErrorBriefly();
  }

  delay(900);
  handleModeChange(readMode());
}

void loop() {
  const Mode nextMode = readMode();
  if (nextMode != currentMode) {
    handleModeChange(nextMode);
  }

  button.update();

  if (currentMode == Mode::Config) {
    if (button.pressedEdge()) {
      selectedScenario = (selectedScenario + 1) % SCENARIO_COUNT;
      showConfig();
    }
    delay(5);
    return;
  }

  switch (experimentState) {
    case ExperimentState::Ready:
      keepAudioErrorVisible();
      if (button.pressedEdge()) {
        ledRed(false);
        startCapturing();
      }
      break;

    case ExperimentState::Capturing:
      if (button.releasedEdge()) {
        startInterpreting();
      }
      break;

    case ExperimentState::Interpreting:
      if (millis() - stateStartedAt >= 800) {
        showResultAndPlay();
      }
      break;

    case ExperimentState::Result:
      if (millis() - stateStartedAt >= 5000) {
        setExperimentReady();
      }
      break;
  }

  delay(5);
}
