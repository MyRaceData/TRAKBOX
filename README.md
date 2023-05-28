# TRAKBOX
Racing Data Acquisition system 

### Brief
TrakBox is a data acquisition system built from inexpensive hobbyist level development boards. The TrakBox is rigidly mounted inside a competition vehicle and used to evaluate the performance of the vehicle. The system runs on a microcontroller. The system includes a GPS sensor, an inertial measurement unit and an environmental sensor. There is also a real time clock, a small OLED display and a microSD card module. TrakBox includes a web server that can run a dash display on a cell phone.

Original prototype pictured below

![prototype](https://github.com/MyRaceData/TRAKBOX/blob/main/prototype.JPG)

### In Depth
 The original prototype (pictured above) was built on a hobbist breadboard. It was powered by a cell phone charging battery (brick) making it completely stand alone with no external inputs. The breadboard was stuck to the cell phone battery with the adhesive backing on the breadboard. Many different microcontrollers could be used to assemble the unit. The microcontroller used for the prototype was a LoLin style NodeMCU type esp8266. The inertial measurement unit has a 3 axis accelerometer, a 3 axis Gyroscope, a 3 axis Magnetometer and an onboard co-processor to fuse the sensor data. The IMU can output data in the form of Euler angles and Quaterion. The microSD card module is used to write the output from the sensors to a comma delimited text file which can be import into a speadsheet for viewing, analysis and visualizaion. The real time clock has a battery backup and syncs with a NTP server on start up. Because TrakBox writes over 90 file entries a second at full speed, milliseconds provided by the microcontroller crystal are also logged because the GPS and RTC only have one second resolution. The OLED display is attached to the unit to provide visual confirmation of proper operation. An asynchronous web server was chosen to keep main loop speed maximized recording data
    
    
 Notes:
 
    TinyGPSPlus is used for GPS data. All the examples set the baud rate at 4800.
    I found none of the examples worked until I set the baud rate at 9600.
    
    In Adafruit BME280 examples,the bme address defaulted to 0x77. I found most the
    chips I tried had an address of 0x76. I had to set the address like so: bme.begin(0x76)
    
    The chronodot needs a 5k resistor on the SDA and SDL lines or it will not run

## Parts List
- ESP8266 CP2102 NodeMCU LUA ESP-12E WIFI Serial Wireless Module 
- Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055 
- GPS Module GPS NEO-6M
- BME280 Digital Temperature Humidity Sensor
- Adafruit MicroSD Card Breakout Board
- 0.96 Inch Blue Yellow OLED Display Module I2C
- ChronoDot V2 with DS3231 and battery backup

Basic wiring diagram below
![Wiring](https://github.com/MyRaceData/TRAKBOX/blob/main/trakboxwiring.png)

## Version History
### Version 006
Adds a hardware real time clock. The popular chronodot V2 is utilized. It has a DS3231 chip. The RTC sinks with an NTP server during setup. This version waits until a wifi connection to the hotspot is established before continuing and displays a warning on the OLED display until a connection is made. The dash display has been enhanced with a 24 hour clock and latitude/ longitude displays. 
### Version 005
Adds the ability to write the data to a microSD card. It connects to the internet using a hotspot on a cell phone and can display a speedometer on the connected phone using a asynchronous web server
### Version 001
First working version. All sensors running with no ability to record data

## Schematic

This beautiful schematic was created using a free account on https://easyeda.com/

![EasyEDA Schematic](https://github.com/MyRaceData/TRAKBOX/blob/main/EasyEDA_Schematic_RacTrakBox.png)

## Libraries Used
- Wire.h                https://github.com/esp8266/Arduino                      GPL licence
- Adafruit_GFX.h        https://github.com/adafruit/Adafruit-GFX-Library        BSD licence
- Adafruit_SSD1306.h    https://github.com/adafruit/Adafruit_SSD1306            BSD licence
- Adafruit_Sensor.h     https://github.com/adafruit/Adafruit_Sensor             BSD licence
- Adafruit_BME280.h     https://github.com/adafruit/Adafruit_BME280_Library     BSD license
- TinyGPSPlus.h         https://github.com/mikalhart/TinyGPSPlus                GNU License
- SoftwareSerial.h      https://github.com/plerup/espsoftwareserial             GPL licence
- Adafruit_BNO055.h     https://github.com/adafruit/Adafruit_BNO055             BSD license
- utility/imumaths.h    https://github.com/adafruit/Adafruit_BNO055/tree/master/utility
- SdFat.h               https://github.com/greiman/SdFat                        MIT licence
- Arduino.h             https://github.com/esp8266/Arduino                      GNU licence
- ESP8266WiFi.h         https://github.com/esp8266/Arduino                      GPL licence
- ESPAsyncTCP.h         https://github.com/me-no-dev/ESPAsyncTCP                LGPL-3.0 licence
- ESPAsyncWebServer.h   https://github.com/me-no-dev/ESPAsyncWebServer
- RTClib.h              https://github.com/adafruit/RTClib                      MIT licence
- time.h        

## Getting Started

1. Assemble the circuit as shown in the [schematic](https://github.com/MyRaceData/TRAKBOX/edit/main/README.md#schematic)
2. Install [Aduino IDE](https://www.arduino.cc/en/Guide)
3. Choose a version from the [snapshots](https://github.com/MyRaceData/TRAKBOX/tree/main/snapshots) folder and copy/paste in the editor
4. Upload the firmware
5. Place sd chip in slot, plug into power source and start recording data!

