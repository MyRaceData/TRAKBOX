# TRAKBOX
Data acquisition system 

Components:

  Main Board:
    LoLin style NodeMCU type esp8266
    
  GPS:
    GT-U7
    
  MCU:
    BNO055 10 DOF MCU
    
  Environment:
    BME280
    
    
 Notes:
 
    TinyGPSPlus is used for GPS data. All the examples set the baud rate at 4800.
    I found none of the examples worked until I set the baud rate at 9600.
    
    In Adafruit BME280 examples, I found I had to set the address like so: status = bme.begin(0x76)

Version 000 is the first working version

Version 001 adds the ability to write the data to a Google sheets file. This method was copied from here

https://github.com/StorageB/Google-Sheets-Logging

It uses the HTTPSRedirect library found here:

https://github.com/electronicsguy/HTTPSRedirect

It connects to the internet using a hotspot on a cell phone


## Parts List
- ESP8266 CP2102 NodeMCU LUA ESP-12E WIFI Serial Wireless Module 
- Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055 
- GPS Module GPS NEO-6M
- BME280 Digital Temperature Humidity Sensor
