#include <Wire.h>
#include <Adafruit_MPL3115A2.h>
#include <SPI.h>
#include <SD.h>

Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
File myFile;

void setup() {
  Serial.begin(9600);
  Serial.println("Adafruit_MPL3115A2 test!");

  Serial.print("Initializing SD card...");

  if (!SD.begin(4)) {
    Serial.println("Card failed, or not present");
    while (1);
  }
  Serial.println("card initialized.");

  myFile = SD.open("test100.txt", FILE_WRITE);

}


void loop() {
  if (! baro.begin()) {
    Serial.println("Couldnt find sensor");
    return;
  }
  
  float pascals = baro.getPressure();

  Serial.print(pascals/3377); Serial.print(" Inches (Hg)\t");

  float altm = baro.getAltitude();
  Serial.print(altm); Serial.print("   meters\t");

  float tempC = baro.getTemperature();
  Serial.print(tempC); Serial.println("*C");

  myFile = SD.open("test100.txt", FILE_WRITE);

  if (myFile) {

    myFile.print(pascals / 3377); myFile.print("\t");
    myFile.print(altm); myFile.print("\t");
    myFile.print(tempC); myFile.println();
    myFile.close();

  } else {
    
    Serial.println("Error opening data file!");
  }

  delay(1000);

 }
