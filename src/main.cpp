#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Definir pinos dos sensores analógicos
const int sensor1Pin = 32; // GPIO 32
const int sensor2Pin = 33; // GPIO 33
const int sensor3Pin = 34; // GPIO 34
const int sensor4Pin = 35; // GPIO 35
const int sensor5Pin = 36; // GPIO 36

// Objeto para o MPU6050
Adafruit_MPU6050 mpu;

void setup() {
  // Iniciar comunicação serial
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // Aguarda a conexão do Monitor Serial
  }

  // Inicializar I2C para o MPU6050
  Wire.begin(21, 22); // SDA = GPIO 21, SCL = GPIO 22

  // Inicializar o MPU6050
  if (!mpu.begin()) {
    Serial.println("Erro ao inicializar o MPU6050!");
    while (1);
  }
  Serial.println("MPU6050 inicializado com sucesso!");

  // Configurar o MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // Faixa de aceleração: ±8g
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);     // Faixa de giroscópio: ±500°/s
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);  // Filtro de largura de banda

  // Configurar pinos analógicos como entrada
  pinMode(sensor1Pin, INPUT);
  pinMode(sensor2Pin, INPUT);
  pinMode(sensor3Pin, INPUT);
  pinMode(sensor4Pin, INPUT);
  pinMode(sensor5Pin, INPUT);
}

void loop() {
  // Ler os sensores analógicos
  int sensor1Value = analogRead(sensor1Pin);
  int sensor2Value = analogRead(sensor2Pin);
  int sensor3Value = analogRead(sensor3Pin);
  int sensor4Value = analogRead(sensor4Pin);
  int sensor5Value = analogRead(sensor5Pin);

  // Exibir valores dos sensores analógicos
  Serial.println("=== Leituras dos Sensores Analógicos ===");
  Serial.print("Sensor 1: "); Serial.println(sensor1Value);
  Serial.print("Sensor 2: "); Serial.println(sensor2Value);
  Serial.print("Sensor 3: "); Serial.println(sensor3Value);
  Serial.print("Sensor 4: "); Serial.println(sensor4Value);
  Serial.print("Sensor 5: "); Serial.println(sensor5Value);

  // Ler dados do MPU6050
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Exibir dados de aceleração
  Serial.println("=== Acelerômetro (m/s²) ===");
  Serial.print("X: "); Serial.print(accel.acceleration.x); Serial.print(" ");
  Serial.print("Y: "); Serial.print(accel.acceleration.y); Serial.print(" ");
  Serial.print("Z: "); Serial.println(accel.acceleration.z);

  // Exibir dados de giroscópio
  Serial.println("=== Giroscópio (°/s) ===");
  Serial.print("X: "); Serial.print(gyro.gyro.x); Serial.print(" ");
  Serial.print("Y: "); Serial.print(gyro.gyro.y); Serial.print(" ");
  Serial.print("Z: "); Serial.println(gyro.gyro.z);

  // Exibir temperatura (opcional)
  Serial.println("=== Temperatura (°C) ===");
  Serial.print("Temperatura: "); Serial.println(temp.temperature);

  // Pequeno delay para não sobrecarregar o Monitor Serial
  delay(1000);
}