// Dependencies
#include <SPI.h>
#include <SD.h>
#include <MPU6050.h>
#include <Adafruit_MPL3115A2.h>
#include <I2Cdev.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// SD Card File Constants
File WRITE_FILE;

// Pin / I2C Address Assignment
MPU6050 mpu(0x68);
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2(); // Alternatively, define at 0x60
const int SD_PIN = 4;
const int STATUS_LED_PIN = LED_BUILTIN;

void setup() {
  // Status LED
  pinMode(STATUS_LED_PIN, OUTPUT);
  
  // Serial Write
  Serial.begin(38400);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("START");

  // Test SD Card
  if (!SD.begin(SD_PIN)) {
    Serial.println("SD CARD: Card Missing / Error");
    while(1);
  }
  Serial.println("SD CARD: READY");

  // Test Barometer & IMU
  Serial.println(baro.begin() ? "MPL3115A2: READY" : "MPL3115A2: ERROR");
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "MPU6050: READY" : "MPU6050: ERROR");

  // Initialize IMU DMP
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  mpu.dmpInitialize();
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.setDMPEnabled(true);
}

void loop() {

  long timeNow = millis();
  float altitudeMeters = baro.getAltitude();
  Serial.println(timeNow);
  Serial.println(altitudeMeters);

  // IMU Vectors
  uint8_t fifoBuffer[64]; // FIFO storage buffer
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorInt16 aa;         // [x, y, z]            accel sensor measurements
  VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
  VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
  VectorFloat gravity;    // [x, y, z]            gravity vector
  float euler[3];         // [psi, theta, phi]    Euler angle container
  float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

  // Readable Orientation (Yaw, Pitch, Roll)
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180/M_PI);
  
    // Readable Acceleration
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    Serial.print("areal\t");
    Serial.print(aaReal.x);
    Serial.print("\t");
    Serial.print(aaReal.y);
    Serial.print("\t");
    Serial.println(aaReal.z);
  }
}
