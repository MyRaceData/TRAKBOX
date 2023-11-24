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

/*
   TrakBox V1.006 7th version
   Andrew Rowe May 14 2023

   This version includes all functionality from the V1.005 version
   including a dash function that runs a speedometer on a phone
   using an async web server - ip address for phone speedo displays on OLED diplay
   This version waits for a wifi connection and puts a warning on the OLED display
   This version waits 30 seconds for the SD card module and puts a warning on the OLED display
   This version introduces a RTC (real time clock)
   The popular chronodot V2 is utilized. It has a DS3231 chip
   The RTC sinks with an NTP server during setup

   the compass function from version 1.3 has been removed

   Write to SD card notes
   This version writes data to a file on the root of the SD card
   The file name is TrakBox.txt
   It is a comma delimited file
   It outputs time from the RTC which syncs with ntp during set up

   The file output changed for this version
   the output is as follows:
   hh:mm.ss, millis, lat, long, gps speed, gps altitude in ft, bme altitude in ft, temperature in C, humitity%

   Then on the end of the same line it outputs one of the following:
   orientation angVelocity linearAccel magnetometer accelerometer gravity
   in xyz format
*/

// add your wifi credentials below
const char *ssid = "***************";
const char *password = "*************";
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
static const int RXPin = 0, TXPin = 2;
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
  delay(100);
  Serial.println("TrakBox starting"); Serial.println("");

  /* initialize the OLED display*/
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("Display not detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  startWIFI();//pauses start and waits for wifi
  Serial.println("wifi connected checking ntp");
  rtc.begin(); //Start RCT
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);//get time from ntp server
  getTime();//set rtc with npt time
  Serial.println("RTC synced to ntp");

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
    sd.initErrorHalt(&Serial);
    delay(30000);
    return;
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    DateTime now = rtc.now();
    AsyncResponseStream *response = request->beginResponseStream("text/html");
    response->addHeader("Server", "TrakBox Dash Server");
    response->printf("<!DOCTYPE html><html><head>", request->url().c_str());
    response->print("<title>TrakBox Dash</title>");
    response->print("<style>");
    response->print("html { font-family: sans-serif; display: inline-block; margin: 0px auto; text-align: center; background-color: black; color: green;}");
    response->print(".bigtxt { position: relative; left: 10px; font-size: 280px;}");
    response->print(".littletxt { position: relative; left: 10px; font-size: 80px;}");
    response->print(".clk { position: fixed; top: 20px; right: 20px; font-size: 60px;}");
    response->print(".ll { position: fixed; top: 40px; left: 10px; font-size: 25px;}");
    response->print("</style>");
    response->print("<meta http-equiv=\"refresh\" content=\"1\">");
    response->print("</head><body>");
    response->print("<span class=\"bigtxt\">");
    response->print(gps.speed.mph(), 0);
    response->print("</span>");
    response->print("<span class=\"littletxt\">");
    response->print("MPH");
    response->print("</span>");
    response->print("<span class=\"clk\">");
    response->print(now.hour());
    response->print(":");
    response->print(now.minute());
    response->print("</span>");
    response->print("<br><span class=\"ll\">Lat: ");
    response->print(gps.location.lat(),6);
    response->print("<br>Lng: ");
    response->print(gps.location.lng(),6);
    response->print("</span>");
    response->print("</body></html>");
    //send the response last
    request->send(response);
  });
  server.onNotFound(notFound);
  server.begin();

  ss.begin(GPSBaud);//start soft serial for GPS

  /* Initialise the accel/gyro */
  if (!bno.begin())
  {
    Serial.print("BNO055 not detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  Serial.println("BNO055 sensor running... "); Serial.println("");

  /* initialize the BME180*/
  if (!bme.begin(0x76)) {
    Serial.println("BME180 not detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  Serial.println("BME sensor running... "); Serial.println("");

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
    while (true);
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

// Function to write the file to SD card
void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  double s = gps.speed.mph();
  if (s < 2) {
    Serial.println("Not logging");
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
