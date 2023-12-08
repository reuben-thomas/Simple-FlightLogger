# Simple FlightLogger
Instructtions to build a simple flight logger to run on an Arduino or on a STM32 board. Logs barometric altitude, acceleration and orientation data. We use DMP data to create velocity and position estimates, which are vulnerable to long term drift error. Thus, this should be used for low power rockets only.

### Components
* Microcontroller: Any Arduino/STM32 with I2C Interface
* Barometer: MPL3115A2
* IMU: MPU6050
* SD Card Reader:
* GPS:
### Libraries
* I2C: [Wire.h](https://github.com/arduino/ArduinoCore-avr/blob/master/libraries/Wire/src/Wire.h)
* I2C: I2Cdev.h
* SPI: SPI.h
* SD Card: SD.h
* Barometer: Adafruit_MPL3115A2.h
* IMU: MPU6050.h
  
