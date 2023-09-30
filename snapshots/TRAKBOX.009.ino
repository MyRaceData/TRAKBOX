#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "SdFat.h"
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "RTClib.h"
#include "time.h"
#include "LittleFS.h"

/*
   TrakBox V1.009 10th version
   Andrew Rowe Sept 9 2023 finalized Sept 30 2023
   
   version 8 is skipped, this version starts with V1.007
   Only change from V7 is the firmware no longer halts setup if there is no SD card

   This version uses the ESP8266 LittleFS Filesystem Uploader Plugin
   you must install the plugin to upload the web pages files to flash memory
   find the plugin and instalation instruction here:
   https://github.com/earlephilhower/arduino-esp8266littlefs-plugin

   This version includes all functionality from the V1.007 version
   including a dash function that runs a speedometer on a phone
   the web server's ip address for phone speedo displays on OLED diplay
   This version waits for a wifi connection and puts a warning on the OLED display

   Write to SD card notes:
   This version writes data to a file on the root of the SD card
   The file name is TrakBox.txt
   It is a comma delimited text file

   The file output is as follows:
   hh:mm.ss, millis, lat, long, gps speed, gps altitude in ft, bme altitude in ft, temperature in C, humitity%

   Then on the end of the same line it outputs one of the following:
   orientation angVelocity linearAccel magnetometer accelerometer gravity
   in xyz format
*/

// add your wifi credentials below
const char *ssid = "************";
const char *password = "************";
/*
   this is the sample rate for the IMU
   the speed is in milliseconds
   max tested speed 100
   this speed produced 90 lines in file per second
   30 minute drive produced 22mb file
*/
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
// no edits below

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C
#define SEALEVELPRESSURE_HPA (1013.25)
#define SPI_CLOCK SD_SCK_MHZ(50)
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
static const int RXPin = 0, TXPin = 2; //RX = D3 TX = D4
static const uint32_t GPSBaud = 9600;
uint16_t PRINT_DELAY_MS = 500; // how often to print the data
uint16_t printCount = 0; //counter to avoid printing every 10MS sample
const uint8_t SD_CS_PIN = SS;
const char* ntpServer = "pool.ntp.org";// NTP server to request time
const long  gmtOffset_sec = -14400;// offset from meridian in seconds
const int   daylightOffset_sec = 0;//offset for daylight saving in seconds

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_BME280 bme; // I2C
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
SdFs sd;
FsFile file;
AsyncWebServer server(80);
RTC_DS3231 rtc;

void setup()
{
  Serial.begin(115200);
  while (!Serial) delay(10);  // wait for serial port to open!
  delay(2000);
  Serial.println("");
  Serial.println("TrakBox V1.009 starting"); Serial.println("");

  /* initialize the OLED display*/
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);

  startWIFI();//pauses start and waits for wifi
  Serial.println("wifi connected!");

  if (!rtc.begin()) {
    ; //Start RCT
    Serial.println("RTC not detected ... Check your wiring or I2C ADDR!");
  } else {
    Serial.println("Real Time Clock running... ");
    Serial.println("checking ntp...");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);//get time from ntp server
    getTime();//set rtc with npt time
    Serial.println("RTC synced to ntp");
  }

  if (!sd.begin(SD_CONFIG)) {
    Serial.println("SD card not detected ... Insert a chip!");
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setCursor(0, 8);
    display.setTextSize(1);
    display.println("WARNING!!!");
    display.setTextSize(3);
    display.setCursor(0, 32);
    display.println("NO SD!");
    display.display();
    //sd.initErrorHalt(&Serial);
    delay(30000);
    //return;
  } else {
    Serial.println("SD card module running");
  }

  if (!LittleFS.begin()) {
    Serial.println("An Error has occurred while mounting LittleFS");
    return;
  }

  File lfsfile = LittleFS.open("/index.htm", "r");
  if (!lfsfile) {
    Serial.println("Failed to open flash memory file for reading");
    return;
  } else {
    Serial.println("Opening Dash display file from flash memory successful!");
    lfsfile.close();
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    //request->send(LittleFS, "/index.htm", "text/html");
    request->send(LittleFS, "/index.htm", String(), false, processor);
  });
  server.onNotFound(notFound);
  server.begin();
  Serial.println("Dash display web server started");

  ss.begin(GPSBaud);//start soft serial for GPS

  /* Initialise the accel/gyro */
  if (!bno.begin())
  {
    //delay(3000);
    Serial.print("BNO055 sensor not detected ... Check your wiring or I2C ADDR!"); Serial.println("");
  } else {
    Serial.println("BNO055 sensor running... "); Serial.println("");
  }

  /* initialize the BME180*/
  if (!bme.begin(0x76)) {
    //delay(3000);
    Serial.println("BME180 sensor not detected ... Check your wiring or I2C ADDR!"); Serial.println("");
  } else {
    Serial.println("BME sensor running... "); Serial.println("");
  }

  // Display Splash Text
  display.clearDisplay();  // Clear the buffer
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 8);
  display.setTextSize(1);
  display.println("Starting ");
  display.setTextSize(3);
  display.setCursor(0, 32);
  display.println("TRAKBOX");
  display.display();
  delay(2000);

}

void loop()
{
  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      displayGPSInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
  }
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  printEvent(&orientationData);
  printEvent(&angVelocityData);
  printEvent(&linearAccelData);
  printEvent(&magnetometerData);
  printEvent(&accelerometerData);
  printEvent(&gravityData);
}

void startWIFI() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("WiFi connecting "); Serial.println("");
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.print(".");
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setCursor(0, 8);
    display.setTextSize(1);
    display.println("WARNING!!!");
    display.setTextSize(3);
    display.setCursor(0, 32);
    display.println("NO WIFI");
    display.display();
    delay(500);
  }
  Serial.println("");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

void displayGPSInfo()
{
  // update the OLED screen
  // throttled so it doesn't flicker
  if (printCount * BNO055_SAMPLERATE_DELAY_MS >= PRINT_DELAY_MS) {
    //update screen
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.print(gps.speed.mph()); // Speed in miles per hour (double)
    display.println(" MPH");
    //
    display.setCursor(0, 16);
    display.setTextSize(1);
    display.print("Lat ");
    display.println(gps.location.lat(), 6);
    //
    display.setCursor(0, 26);
    display.setTextSize(1);
    display.print("Lon ");
    display.println(gps.location.lng(), 6);
    //
    display.setCursor(0, 36);
    display.setTextSize(1);
    display.print("Alt ");
    display.print(gps.altitude.feet());
    display.println(" ft");
    //
    DateTime now = rtc.now();
    display.setCursor(0, 46);
    display.setTextSize(1);
    display.print("Time ");
    display.print(now.hour());
    display.print(":");
    display.println(now.minute());
    //
    //float Fdeg;
    // Fdeg = 9.0 * degCin / 5.0 + 32.0;
    display.setCursor(0, 57);
    display.setTextSize(1);
    display.print("IP: ");
    display.println(WiFi.localIP());
    //
    display.setCursor(70, 20);
    display.setTextSize(3);
    //display.print("Temp: ");
    //
    display.display();

    printCount = 0;
  }
  else {
    printCount = printCount + 1;
  }
}

// Function that gets time from NTP Server and syncs RTC clock
void getTime() {

  int yr = 0;
  int mt = 0;
  int dy = 0;
  int hr = 0;
  int mi = 0;
  int se = 0;

  struct tm timeinfo;
  getLocalTime(&timeinfo);

  yr = timeinfo.tm_year + 1900;
  mt = timeinfo.tm_mon + 1;
  dy = timeinfo.tm_mday;
  hr = timeinfo.tm_hour;
  mi = timeinfo.tm_min;
  se = timeinfo.tm_sec;

  rtc.adjust(DateTime(yr, mt, dy, hr, mi, se));

}

// Function to process templates in web pages
String processor(const String& var)
{
  //DateTime now = rtc.now();
  String h = "00"; //Print(now.hour());
  if (var == "now_hour") return h;
  return String();
}

// Function to write the file to SD card
void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  double s = gps.speed.mph();
    //Serial.print("BME temp ");
    //Serial.println(bme.readTemperature());
  if (s < 2) {
    Serial.print("Not logging ");
    Serial.print("GPS Time: ");
    Serial.println(gps.time.value());
    //Serial.print("Lat ");
    //Serial.print(gps.location.lat(), 6);
    //Serial.print("Lon ");
    //Serial.println(gps.location.lng(), 6);
    return;
  }
  // open the file for write at end
  if (!file.open("TrakBox.txt", O_RDWR | O_CREAT | O_AT_END)) {
    sd.errorHalt("opening TrakBox.txt for write failed");
    Serial.println("No SD card!!!");
    return;
  }
  // if the file opened okay, write to it:
  Serial.println("Writing to TrakBox.txt...");
  //Serial.println(millis());
  DateTime now = rtc.now();
  file.print(now.hour());
  file.print(":");
  file.print(now.minute());
  file.print(":");
  file.print(now.second());
  file.print(",");
  file.print(millis());
  file.print(",");
  file.print(gps.location.lat(), 6);
  file.print(",");
  file.print(gps.location.lng(), 6);
  file.print(",");
  file.print(gps.speed.mph());
  file.print(" MPH,");
  file.print("gpsAlt ");
  file.print(gps.altitude.feet());
  file.print(" ft, ");
  file.print("bmeAlt ");
  file.print(bme.readAltitude(SEALEVELPRESSURE_HPA) * 3.281);
  file.print(" ft, ");
  file.print(bme.readTemperature());
  file.print(" °C,");
  file.print(bme.readHumidity());
  file.print("%,");
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    file.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    file.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    file.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    file.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    file.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    file.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    file.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    file.print("Unk:");
  }

  file.print("\tx= ");
  file.print(x);
  file.print(" |\ty= ");
  file.print(y);
  file.print(" |\tz= ");
  file.println(z);
  //file.println(); //uncomment to have a blank line

  // close the file:
  file.close();
}
