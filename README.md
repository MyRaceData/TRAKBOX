# TRAKBOX
Racing Data Acquisition system 

### Brief
TrakBox is a data acquisition system built from inexpensive hobbyist level development boards. The TrakBox is rigidly mounted inside a competition vehicle and used to evaluate the performance of the vehicle. The system runs on a microcontroller. The system includes a GPS sensor, an inertial measurement unit and an environmental sensor. There is also a real time clock, a small OLED display and a microSD card module. TrakBox includes a web server that can run a dash display on a cell phone.

![prototype](https://github.com/MyRaceData/TRAKBOX/blob/main/trakboxprod-proto.jpg)

### In Depth
 The Race TrakBox is a data acquisition system built for racing from inexpensive hobbyist level development boards. It is designed to be simple and cheap to build. If you can solder, you can build one yourself. It uses an IMU (Inertial Measurement Unit), which is basically a sensor used for discerning movement. It also has a GPS module which can locate the vehicle accurately anywhere on the planet as well as discern speed and bearing/heading once in motion. The IMU is the real rock star here though. By measuring lateral g-forces, it can be used to gauge outright acceleration, braking and cornering performance. Gyroscopic forces can be used to measure chassis and suspension performance. Most of the rest of the componets are included to support this two modules. There is an environmental sensor providing ambient air temperature, humidity and atmospheric air presure, all of which can be very useful for tuning purposes. There is also a real time clock with a battery backup which provides extremely accurate times and dates for record keeping. An on-board micro SD card module allows all the data to be recorded to disc for later analysis.

 Original prototype pictured below

![prototype](https://github.com/MyRaceData/TRAKBOX/blob/main/prototype.JPG)

 The original prototype (pictured above) was built on a hobbist breadboard. It was powered by a cell phone charging bank (brick) making it completely stand alone with no external inputs. The breadboard was stuck to the charging bank with the adhesive backing on the breadboard. Many different microcontrollers could be used to assemble the unit. The microcontroller used for the prototype was a LoLin style NodeMCU type esp8266. The inertial measurement unit used on the prototype was the Adafruit BNO055 9-DOF Absolute Orientation IMU Fusion Breakout. It has a 3 axis accelerometer, a 3 axis Gyroscope, a 3 axis Magnetometer and an onboard co-processor to fuse the sensor data. This IMU can output data in the form of Euler angles and Quaterion. The microSD card module is used to write the output from the sensors to a comma delimited text file which can be import into a speadsheet for viewing, analysis and visualizaion. The real time clock has a battery backup and syncs with a NTP server on start up. Because TrakBox writes over 90 file entries a second at full speed, milliseconds provided by the microcontroller crystal are also logged because the GPS and RTC only have one second resolution. The OLED display is attached to the unit to provide visual confirmation of proper operation. An asynchronous web server was chosen to provide the dash display feature to keep main loop speed maximized while recording data.

 As of Version 011 the file format has changed
 The file output is as follows:

millis,MM/DD/YYYY,hh:mm.ss,lat,long,gps speed,altitude,temperature,humitity,atmospheric pressure,linear accel x,y,z, orientation x,y,z

Scales of the readings in the above order

millis is as emitted by the crystal of the microcontroller and is not exactly accurate in milliseconds

month day year per RTC

hour minute second per RTC

Latitude per GPS

Longitude per GPS

speed in MPH per GPS

altitude in feet above sea level (calculated from BME180 atmospheric pressure)

temperature in Celsius per BME180

humitity in percent per BME180

atmospheric pressure in h/PA per BME180

linear acceleration in feet per second on the X axis per BNO055

linear acceleration in feet per second on the Y axis per BNO055

linear acceleration in feet per second on the Z axis per BNO055

orientation in Euler angles on the X axis (pitch) per BNO055

orientation in Euler angles on the Y axis (roll) per BNO055

orientation in Euler angles on the Z axis (yaw) per BNO055
    
    
 Notes:
 
    Development has switch from NodeMCU LUA ESP-12E style esp8266 to Wemos D1 mini style esp8266.
    Main reason being they are smaller - approximately 2/3rds the length.
    
    TinyGPSPlus is the library used for GPS data. All the examples set the baud rate at 4800.
    I found none of the examples worked until I set the baud rate at 9600.
    
    In Adafruit BME280 examples,the bme address defaulted to 0x77. I found most the
    chips I tried had an address of 0x76. I had to set the address like so: bme.begin(0x76)
    
    The chronodot needs a 5k resistor on the SDA and SDL lines or it will not run
    
    When using the ESP8266 LittleFS Filesystem Uploader Plugin, place all the files
    you would like tranfered in a folder called data in the same folder as your sketch
    for installation and usage https://github.com/earlephilhower/arduino-esp8266littlefs-plugin
    If the upload fails, shut down the serial monitor
    
    IMU notes
    Various IMUs were being tested current version uses Bosch BNO-055
    
    Bosch BNO-055      adafruit 9-DOF Absolute Orientation Sensor breakout $34.95 (June 2023)
    https://www.mouser.com/ProductDetail/Adafruit/2472?qs=N%2F3wi2MvZWDmk8dteqdybw%3D%3D
    adafruit 9-DOF Absolute Orientation IMU Fusion Breakout from Mouser $29.95 (STEMMA QT / Qwiic version)
    https://www.mouser.com/ProductDetail/Adafruit/4646?qs=W%2FMpXkg%252BdQ4Msqspav40iw%3D%3D
    
    InvenSense TDK ICM20948     adafruit breakout from Mouser $14.95 (STEMMA QT / Qwiic version)
    https://www.mouser.com/ProductDetail/Adafruit/4554?qs=DPoM0jnrROVsh%2FR5VsCmTg%3D%3D
    this chip has a I2c address of 0x69
    
    this chip is considered an upgrade to the MPU92XX below

    MPU9250 - I have not gotten good results from this chip
    MPU9250 are also sold by name GY-91 which includes a 9250 and a BMP280 barometric pressure sensor
    GY-9250    HiLetgo (and other brands) on amazon 14.99 (June 2023)
    
    MPU-6050 (AKA GY-521) are also being evaluated. This chip has accel & gyro, no magnetometer
    Library that includes DMP (digital motion processor) use https://github.com/jrowberg/i2cdevlib by Jeff Rowberg
    Specifically this library: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
    to use this library you must change line #define INTERRUPT_PIN 2 to another pin
    I used D8 on a nodemcu which is GPIO15 so change the above to: #define INTERRUPT_PIN 15
    Also change the line: void dmpDataReady() { to: void ICACHE_RAM_ATTR dmpDataReady() {
    

    To use sparkfun library for ICM20948
    * ** Important note: by default the DMP functionality is disabled in the library
    * ** as the DMP firmware takes up 14301 Bytes of program memory.
    * ** To use the DMP, you will need to:
    * ** Edit ICM_20948_C.h
    * ** Uncomment line 29: #define ICM_20948_USE_DMP
    * ** Save changes
    * ** you can find ICM_20948_C.h in:
    * ** /Home/Andy/Arduino/libraries/SparkFun_ICM-20948_ArduinoLibrary/src/util

    The DS3231 is the RTC used. A lot of the break out boards for this chip are bulky and expensive.
    One model, called the DS3231 for PI are cheap ($3 in lots of 5) and includes a battery.
    They are also tiny (about 1/2" square) Only problem is they have female headers which need desoldered and replaced
    DS3231 For PI notes
    Pin 1 (square) = 5V or 3.3V "+"
    Pin 2 = SDA "D" is data
    Pin 3 = SCL "C" is clock
    Pin 4 = not connected "NC" is Not Connected
    Pin 5 = GND "-" is GND

## Parts List
- ESP8266 CP2102 NodeMCU LUA ESP-12E WIFI Serial Wireless Module 
- Adafruit BNO055 9-DOF Absolute Orientation IMU Fusion Breakout
- GPS Module GPS NEO-6M
- BME280 Digital Temperature Humidity Sensor
- Adafruit MicroSD Card Breakout Board
- 0.96 Inch Blue Yellow OLED Display Module I2C
- ChronoDot V2 with DS3231 and battery backup

Basic wiring diagram below
![Wiring](https://github.com/MyRaceData/TRAKBOX/blob/main/trakboxwiring.png)

## Recent Updates
12.27.23
The first pre-production units are ready for testing. 

8.30.23
First PCB board sent for fabrication. The enclosure used is by Polycase model SN-27

## Version History
### Version 012
This version does not halt startup if no wifi connection can be made. Allowing startup and use without a wifi connection will allow data to be recorded normally. The phone dash display will not function without a wifi connection. The realtime clock will also not be able to be synced to a time server. The recorded times may not be accurate and should be verified. This version will be used on the first pre-production prototypes
### Version 011
This version changes the file format and file extension. Version 10 is the same as this version without the file format change which included a bugfix 
in v9 when no SD card was inserted. 
### Version 009
Version 8 is skipped. This version is the same as V7 except setup does not halt if there is no SD card
### Version 007
This version provides much more robust hardware error checking. The firmware will run even with none of the sensors attached allowing testing of just the microcontroller or any combination of sensors. The debug messages are also improved. Other portions of the firmware were refactored.
This version also introduces the use of storing the web page files for the dash display in flash memory. Flash memory is used because this allows the html to be removed from the instruction portion of non-volitile RAM used by the firmware. It also allows the web page files to be edited more easily. This feature uses the littleFS library to connect to the flash memory and the ESP8266 LittleFS Filesystem Uploader Plugin [found here](https://github.com/earlephilhower/arduino-esp8266littlefs-plugin) to upload the web files to flash memory. This version waits until a wifi connection to the hotspot is established before continuing and displays a warning on the OLED display until a connection is made. This version only logs data to the sd card if moving over two MPH.
### Version 006
Adds a hardware real time clock. The popular chronodot V2 is utilized. It has a DS3231 chip. The RTC sinks with an NTP server during setup. This version waits until a wifi connection to the hotspot is established before continuing and displays a warning on the OLED display until a connection is made. This version only logs data to the sd card if moving over two MPH. The dash display has been enhanced with a 24 hour clock and latitude/ longitude displays. 
### Version 005
Adds the ability to write the data to a microSD card. It connects to the internet using a hotspot on a cell phone and can display a speedometer on the connected phone using a asynchronous web server
### Version 001
First working version. All sensors running with no ability to record data

## Schematic

This beautiful schematic was created using a free account on https://easyeda.com/

Note this is the schematic for the Wemos style D1 mini V3~

![EasyEDA Schematic](https://github.com/MyRaceData/TRAKBOX/blob/main/Schematic_DiMiniTrakBox4_2023-09-30.png)

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
3. Install the ESP8266 LittleFS Filesystem Uploader Plugin [found here](https://github.com/earlephilhower/arduino-esp8266littlefs-plugin)
4. Choose a version from the [snapshots](https://github.com/MyRaceData/TRAKBOX/tree/main/snapshots) folder and copy/paste it into the [Aduino IDE](https://www.arduino.cc/en/Guide) editor
5. Create a folder in the same folder as the firmware file - name the new folder data
6. Place the contents of [data](https://github.com/MyRaceData/TRAKBOX/tree/main/data) in the folder from the previous step
7. Create a hotspot on your phone for the dash display
8. Edit the sketch to add your wifi credentials from the previous step
9. Write the firmware to the D1 Mini using the [Aduino IDE](https://www.arduino.cc/en/Guide)
10. Use [file upload plugin](https://github.com/earlephilhower/arduino-esp8266littlefs-plugin) to upload the contents of the data folder
11. Place sd chip in slot, plug into power source and start recording data!

