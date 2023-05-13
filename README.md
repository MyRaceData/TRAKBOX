# TRAKBOX
Data acquisition system 

TrakBox is a data acquisition system built from inexpensive hobbyist level development boards. The system runs on a microcomputer. The microcomputer used is a LoLin style NodeMCU type esp8266. The system includes a GPS sensor, an inertial measurement unit and an environmental sensor. There is also a small OLED display and a microSD card module. The microSD card is used to write the output from the sensors to a comma delimited text file which can be import into speadsheet for viewing. TrakBox includes a web server that can run a dash display on a cell phone. The inertial measurement unit has a 3 axis accelerometer, a 3 axis Gyroscope, a 3 axis Magnetometer and an onboard co-processor to fuse the sensor data. The TrakBox is rigidly mounted inside a racecar and used to evaluate the performance of the car.

Components:

  Main Board:
    LoLin style NodeMCU type esp8266
    
  GPS:
    GT-U7
    
  MCU:
    BNO055 10 DOF MCU
    
  Environment:
    BME280
  
  microSD card module:
    Adafruit MicroSD Card Breakout Board
    
  Display:
    0.96 Inch Blue Yellow OLED Display Module I2C 
    
 Notes:
 
    TinyGPSPlus is used for GPS data. All the examples set the baud rate at 4800.
    I found none of the examples worked until I set the baud rate at 9600.
    
    In Adafruit BME280 examples, I found I had to set the address like so: status = bme.begin(0x76)

Version 000 is the first working version

Version 005 adds the ability to write the data to a microSD card. It connects to the internet using a hotspot on a cell phone and can display a speedometer on the connected phone


## Parts List
- ESP8266 CP2102 NodeMCU LUA ESP-12E WIFI Serial Wireless Module 
- Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055 
- GPS Module GPS NEO-6M
- BME280 Digital Temperature Humidity Sensor
