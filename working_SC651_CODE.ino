#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>

// ESP32 I2C pins
#define SDA_PIN 21
#define SCL_PIN 22

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

struct __attribute__((packed)) IMUPacket {
  uint8_t start_byte;
  float ax, ay, az;
  float gx, gy, gz;
  uint8_t checksum;
};

// Function to calculate XOR checksum (excluding start_byte and checksum)
uint8_t calculateChecksum(uint8_t* data, size_t len) {
  uint8_t sum = 0;
  for (size_t i = 0; i < len; ++i) {
    sum ^= data[i];
  }
  return sum;
}

void setupSensor() {
  // Accelerometer: ±2g
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  // Gyroscope: ±245 dps
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  // Magnetometer: ±4 gauss (unused here)
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
}

void setup() {
  Serial.begin(115200);
  delay(1000);  // Wait for serial monitor to start

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!lsm.begin()) {
    Serial.println("Failed to find LSM9DS1 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("LSM9DS1 Found!");
  setupSensor();
}

void loop() {
  lsm.read();  // Must be called before getEvent()

  sensors_event_t accel_event, mag_event, gyro_event, temp_event;
  lsm.getEvent(&accel_event, &mag_event, &gyro_event, &temp_event);

  IMUPacket packet;
  packet.start_byte = 0xAA;

  // Fill packet with sensor data
  packet.ax = accel_event.acceleration.x;
  packet.ay = accel_event.acceleration.y;
  packet.az = accel_event.acceleration.z;

  packet.gx = gyro_event.gyro.x;
  packet.gy = gyro_event.gyro.y;
  packet.gz = gyro_event.gyro.z;

  // Compute checksum of all data fields (excluding start and checksum)
  packet.checksum = calculateChecksum(((uint8_t*)&packet) + 1, sizeof(IMUPacket) - 2);

  // Send binary packet
  Serial.write((uint8_t*)&packet, sizeof(packet));
  // Adjust data rate
}
