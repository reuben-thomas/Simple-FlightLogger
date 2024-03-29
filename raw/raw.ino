// Dependencies
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_Sensor.h>
#include <I2Cdev.h>

// SD Card File Constants
File MY_FILE;
String MY_FILE_NAME = "New.txt";

// Pin / I2C Address Assignment
Adafruit_MPU6050 mpu;
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2(); // Alternatively, define at 0x60
const int SD_PIN = 4;
const int STATUS_LED_PIN = LED_BUILTIN;

void setup() {
  // Status LED
  pinMode(STATUS_LED_PIN, OUTPUT);
  
  // Serial Write
  Serial.begin(9600);
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
  Serial.println(mpu.begin() ? "MPU6050: READY" : "MPU6050: ERROR");

  // IMU Settings
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}


void loop() {
  // Time Reading
  long timeNow = millis();
  
  // Barometer Readings
  float altitudeMeters = baro.getAltitude();
  float temperatureDegrees = baro.getTemperature();

  // IMU Readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;
  float wx = g.gyro.x;
  float wy = g.gyro.y;
  float wz = g.gyro.z;

  // Serial Print
  Serial.print("Time: ");
  Serial.print(timeNow);
  Serial.print("\t Altitude: ");
  Serial.print(altitudeMeters);
  Serial.print("\t Temperature: ");
  Serial.print(temperatureDegrees);
  Serial.print("\t Acceleration: [");
  Serial.print(ax);
  Serial.print(", ");
  Serial.print(ay);
  Serial.print(", ");
  Serial.print(az);
  Serial.print("]");
  Serial.print("\t Angular: [");
  Serial.print(wx);
  Serial.print(", ");
  Serial.print(wy);
  Serial.print(", ");
  Serial.print(wz);
  Serial.print("]");
  Serial.println("");

  // Write to File
  MY_FILE = SD.open("data.txt", FILE_WRITE);
  if (MY_FILE) {
    MY_FILE.print(timeNow); MY_FILE.print(",");
    MY_FILE.print(altitudeMeters); MY_FILE.print(",");
    MY_FILE.print(temperatureDegrees); MY_FILE.print(",");
    MY_FILE.print(ax); MY_FILE.print(",");
    MY_FILE.print(ay); MY_FILE.print(",");
    MY_FILE.print(az); MY_FILE.print(",");
    MY_FILE.print(wx); MY_FILE.print(",");
    MY_FILE.print(wy); MY_FILE.print(",");
    MY_FILE.print(wz); MY_FILE.print(",");
    MY_FILE.println("");
    MY_FILE.close();

  } else {
    
    Serial.println("Error opening data file!");
  }

  delay(1000);

 }
