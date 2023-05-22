# TRAKBOX
Data acquisition system 

TrakBox is a data acquisition system built from inexpensive hobbyist level development boards. The TrakBox is rigidly mounted inside a competition vehicle and used to evaluate the performance of the vehicle. The system runs on a microcomputer. The microcomputer used is a LoLin style NodeMCU type esp8266. The system includes a GPS sensor, an inertial measurement unit and an environmental sensor. There is also a small OLED display and a microSD card module. The microSD card is used to write the output from the sensors to a comma delimited text file which can be import into speadsheet for viewing. TrakBox includes a web server that can run a dash display on a cell phone. The inertial measurement unit has a 3 axis accelerometer, a 3 axis Gyroscope, a 3 axis Magnetometer and an onboard co-processor to fuse the sensor data.
    
    
 Notes:
 
    TinyGPSPlus is used for GPS data. All the examples set the baud rate at 4800.
    I found none of the examples worked until I set the baud rate at 9600.
    
    In Adafruit BME280 examples, I found I had to set the address like so: status = bme.begin(0x76)

## Parts List
- ESP8266 CP2102 NodeMCU LUA ESP-12E WIFI Serial Wireless Module 
- Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055 
- GPS Module GPS NEO-6M
- BME280 Digital Temperature Humidity Sensor
- Adafruit MicroSD Card Breakout Board
- 0.96 Inch Blue Yellow OLED Display Module I2C
- ChrnoDot V2 with DS3231 and battery backup

Basic wiring diagram below
![alt text](https://github.com/MyRaceData/TRAKBOX/blob/main/trakboxwiring.png)

## Version History
### Version 006
Adds a hardware real time clock. The popular chronodot V2 is utilized. It has a DS3231 chip. The RTC sinks with an NTP server during setup. This version waits until a wifi connection to the hotspot is established before continuing and displays a warning on the OLED display until a connection is made. The dash display has been enhanced with a 24 hour clock and latitude/ longitude displays. 
### Version 005
Adds the ability to write the data to a microSD card. It connects to the internet using a hotspot on a cell phone and can display a speedometer on the connected phone using a asynchronous web server
### Version 001
First working version. All sensors running with no ability to record data
