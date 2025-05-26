#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ADC pins
const int ADC_CS_PIN   = 5;
const int ADC_SCK_PIN  = 18;
const int ADC_MISO_PIN = 19;
const int ADC_MOSI_PIN = 23;

// Diagnostic pin for oscilloscope
const int TOGGLE_PIN = 32;

const int PRINT_INTERVAL = 500;
const int LOOP_INTERVAL  = 20;

const float kx  = 20.0;
const float VREF = 4.096;

// Global objects
Adafruit_MPU6050 mpu;  // Default I2C pins: SCL=22, SDA=21

// (Optional) ADC read helper
uint16_t readADC(uint8_t channel) {
  uint8_t TXByte0 = 0x06 | (channel >> 2);
  uint8_t TXByte1 = (channel & 0x03) << 6;

  digitalWrite(ADC_CS_PIN, LOW);
  SPI.transfer(TXByte0);
  uint8_t RXByte0 = SPI.transfer(TXByte1);
  uint8_t RXByte1 = SPI.transfer(0x00);
  digitalWrite(ADC_CS_PIN, HIGH);

  return ((RXByte0 & 0x0F) << 8) | RXByte1;
}

void setup() {
  Serial.begin(115200);
  pinMode(TOGGLE_PIN, OUTPUT);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  // Set up ADC and SPI
  pinMode(ADC_CS_PIN, OUTPUT);
  digitalWrite(ADC_CS_PIN, HIGH);
  SPI.begin(ADC_SCK_PIN, ADC_MISO_PIN, ADC_MOSI_PIN, ADC_CS_PIN);
}

void loop() {
  static unsigned long previous_time = millis();
  static unsigned long printTimer   = 0;
  static unsigned long loopTimer    = 0;
  static float theta_n               = 0.0;
  static float integral              = 0.0;
  static float tilt_angle_x          = 0.0;
  static float tilt_angle_y          = 0.0;
  static float tilt_angle_z          = 0.0;
  static float gyro_x                = 0.0;
  static float gyro_y                = 0.0;
  static float gyro_z                = 0.0;
  static float error                 = 0.0;
  const float c                       = 0.96;

  if (millis() - loopTimer >= LOOP_INTERVAL) {
    loopTimer += LOOP_INTERVAL;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    tilt_angle_x = a.acceleration.x / 9.67;
    tilt_angle_y = a.acceleration.y / 9.67;
    tilt_angle_z = a.acceleration.z / 9.67;
    gyro_x = g.gyro.x;
    gyro_y = g.gyro.y;
    gyro_z = g.gyro.z;

    unsigned long current_time = millis();
    float dt = (current_time - previous_time) / 1000.0;
    previous_time = current_time;

    theta_n = (1 - c) * tilt_angle_z + c * (gyro_y * dt + theta_n);

    float reference = 0.026;
    error = reference - theta_n;
  }

  if (millis() - printTimer >= PRINT_INTERVAL) {
    printTimer += PRINT_INTERVAL;
    Serial.print(tilt_angle_z, 4);
    Serial.print(' ');
    Serial.print(theta_n, 4);
    Serial.print(' ');
    Serial.println(error, 4);
  }
}
