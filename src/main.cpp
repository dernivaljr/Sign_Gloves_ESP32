#include <Arduino.h>
#include <Wire.h>

// === CONFIGURAÇÕES ===
// Ajuste a lista abaixo com os pinos ADC usados pelos sensores flex.
// No ESP32 DevKit V1 é recomendável usar apenas pinos ADC0/ADC1 (32-39).
struct AnalogSensorConfig {
  const char *name;
  uint8_t pin;
};

AnalogSensorConfig analogSensors[] = {
    {"Dedo polegar", 32},
    {"Dedo indicador", 33},
    {"Dedo médio", 34},
    {"Dedo anelar", 35},
    {"Dedo mínimo", 36}, // GPIO36 (VP) – somente entrada
};
constexpr size_t ANALOG_SENSOR_COUNT = sizeof(analogSensors) / sizeof(AnalogSensorConfig);

// Pinos padrão do barramento I2C no ESP32 DevKit V1
constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;

constexpr uint8_t MPU6050_ADDR = 0x68; // endereço padrão do MPU6050

// === FUNÇÕES AUXILIARES ===
void printDivider() {
  Serial.println(F("----------------------------------------"));
}

void configureAnalogInputs() {
  for (size_t i = 0; i < ANALOG_SENSOR_COUNT; ++i) {
    pinMode(analogSensors[i].pin, INPUT);
  }
}

void scanI2CBus() {
  Serial.println(F("Iniciando varredura I2C..."));
  uint8_t deviceCount = 0;

  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      Serial.print(F("Dispositivo encontrado no endereço 0x"));
      Serial.println(address, HEX);
      ++deviceCount;
    } else if (error == 4) {
      Serial.print(F("Erro desconhecido no endereço 0x"));
      Serial.println(address, HEX);
    }
  }

  if (deviceCount == 0) {
    Serial.println(F("Nenhum dispositivo I2C encontrado."));
  } else {
    Serial.print(deviceCount);
    Serial.println(F(" dispositivo(s) I2C detectado(s)."));
  }
  printDivider();
}

void readAnalogSensors() {
  Serial.println(F("=== Leituras dos Sensores Flex (ADC) ==="));
  for (size_t i = 0; i < ANALOG_SENSOR_COUNT; ++i) {
    int raw = analogRead(analogSensors[i].pin);
    float voltage = raw * (3.3f / 4095.0f); // Conversão aproximada

    Serial.print(analogSensors[i].name);
    Serial.print(F(" (GPIO "));
    Serial.print(analogSensors[i].pin);
    Serial.print(F("): valor bruto = "));
    Serial.print(raw);
    Serial.print(F(", tensão ≈ "));
    Serial.print(voltage, 2);
    Serial.println(F(" V"));
  }
  printDivider();
}

bool writeMPU6050Register(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readMPU6050Registers(uint8_t startReg, uint8_t *buffer, size_t length) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  size_t read = Wire.requestFrom(MPU6050_ADDR, static_cast<uint8_t>(length));
  if (read != length) {
    return false;
  }

  for (size_t i = 0; i < length; ++i) {
    buffer[i] = Wire.read();
  }
  return true;
}

void readMPU6050() {
  uint8_t rawData[14];
  if (!readMPU6050Registers(0x3B, rawData, sizeof(rawData))) {
    Serial.println(F("Falha ao ler dados do MPU6050."));
    printDivider();
    return;
  }

  auto toInt16 = [&](size_t index) {
    return static_cast<int16_t>((rawData[index] << 8) | rawData[index + 1]);
  };

  const float accelScale = 8192.0f; // sensibilidade configurada para ±4g
  const float gyroScale = 65.5f;    // sensibilidade configurada para ±500 °/s

  float accelX = toInt16(0) / accelScale * 9.80665f;
  float accelY = toInt16(2) / accelScale * 9.80665f;
  float accelZ = toInt16(4) / accelScale * 9.80665f;

  float tempC = toInt16(6) / 340.0f + 36.53f;

  float gyroX = toInt16(8) / gyroScale;
  float gyroY = toInt16(10) / gyroScale;
  float gyroZ = toInt16(12) / gyroScale;

  Serial.println(F("=== MPU6050: Acelerômetro (m/s²) ==="));
  Serial.print(F("X: "));
  Serial.print(accelX, 3);
  Serial.print(F(" | Y: "));
  Serial.print(accelY, 3);
  Serial.print(F(" | Z: "));
  Serial.println(accelZ, 3);

  Serial.println(F("=== MPU6050: Giroscópio (°/s) ==="));
  Serial.print(F("X: "));
  Serial.print(gyroX, 3);
  Serial.print(F(" | Y: "));
  Serial.print(gyroY, 3);
  Serial.print(F(" | Z: "));
  Serial.println(gyroZ, 3);

  Serial.println(F("=== MPU6050: Temperatura (°C) ==="));
  Serial.println(tempC, 2);
  printDivider();
}

bool configureMPU6050() {
  Serial.println(F("Inicializando MPU6050..."));

  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println(F("MPU6050 não respondeu no endereço 0x68. Verifique alimentação e fiação."));
    printDivider();
    return false;
  }

  // Acorda o sensor (reg. PWR_MGMT_1)
  if (!writeMPU6050Register(0x6B, 0x00)) {
    Serial.println(F("Falha ao acordar o MPU6050."));
    printDivider();
    return false;
  }

  // Configura o acelerômetro para ±4g (reg. ACCEL_CONFIG)
  if (!writeMPU6050Register(0x1C, 0x08)) {
    Serial.println(F("Falha ao configurar alcance do acelerômetro."));
    printDivider();
    return false;
  }

  // Configura o giroscópio para ±500 °/s (reg. GYRO_CONFIG)
  if (!writeMPU6050Register(0x1B, 0x08)) {
    Serial.println(F("Falha ao configurar alcance do giroscópio."));
    printDivider();
    return false;
  }

  Serial.println(F("MPU6050 configurado com sucesso!"));
  printDivider();
  return true;
}

// === FUNÇÕES PRINCIPAIS ===
void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println();
  Serial.println(F("===== Diagnóstico da Luva com ESP32 DevKit V1 ====="));
  printDivider();

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Serial.print(F("Barramento I2C iniciado (SDA="));
  Serial.print(I2C_SDA_PIN);
  Serial.print(F(", SCL="));
  Serial.print(I2C_SCL_PIN);
  Serial.println(F(")"));
  printDivider();

  configureAnalogInputs();
  Serial.println(F("Entradas analógicas configuradas."));
  printDivider();

  scanI2CBus();
  if (!configureMPU6050()) {
    Serial.println(F("Interrompendo leituras devido a erro na configuração do MPU6050."));
    while (true) {
      delay(1000);
    }
  }
}

void loop() {
  readAnalogSensors();
  readMPU6050();

  Serial.println(F("Aguarde 2 segundos para nova leitura..."));
  printDivider();
  delay(2000);
}
