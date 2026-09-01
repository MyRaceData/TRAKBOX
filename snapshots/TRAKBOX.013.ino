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
   TrakBox V1.013 14th version
   Andrew Rowe Aug 29 2026

   This version includes several improvements in code structure aimed at
   efficient code execution with the overall goal being increasing logging speed.

   This version does not halt startup if no wifi connection can be made.
   Allowing startup and use without a wifi connection will allow data to
   be recorded normally. The realtime clock will not be synced to a time server.
   The recorded times may not be accurate and should be verified

   This version uses the file extension .csv
   This version only records the actual data values with no scale
   All additional scale information is not written to file such as MPH
   For file output format see Write to SD card notes: below

   The web page used for the dash display uses the template processor feature of the
   Arduino async web server to insert live data into the web page. More info here
   https://github.com/me-no-dev/ESPAsyncWebServer#specifying-template-processor-callback

   This version uses the ESP8266 LittleFS Filesystem Uploader Plugin
   you must install the plugin to upload the web pages files to flash memory
   find the plugin and instalation instruction here:
   https://github.com/earlephilhower/arduino-esp8266littlefs-plugin

   This version of the firmware no longer halts setup if there is no SD card
   Start up is delayed for 30 seconds if there is no SD card inserted
   A warning is printed to the onboard OLED display 'NO SD'
   This version includes a dash function that runs a speedometer on a phone
   the web server's ip address for phone speedo displays on OLED diplay

   Write to SD card notes:
   This version writes data to a file on the root of the SD card
   The file name is TrakBox.csv
   It is a comma delimited text file

   The file output is as follows:
   millis,MM/DD/YYYY,hh:mm.ss,lat,long,gps speed,altitude,temperature,humitity,atmospheric pressure,linear accel x,y,z, angular velocity x,y,z

   Scales of the readings in the above order
   millis is as emitted by the crystal of the microcontroller and is not exactly accurate in milliseconds
   month day year per RTC
   hour minute secons per RTC
   Latitude per GPS
   Longitude per GPS
   speed in MPH per GPS
   altitude in feet above sea level (calculated from BME180 atmospheric pressure)
   temperature in Celsius per BME180
   humitity in percent per BME180
   atmospheric pressure in h/PA per BME180
   linear acceleration in meters per second squared (m/s^2) on the X axis per BNO055
   linear acceleration in meters per second squared (m/s^2) on the Y axis per BNO055
   linear acceleration in meters per second squared (m/s^2) on the Z axis per BNO055
   angular velocity in Radians per Second (rad/s) on the X axis per BNO055
   angular velocity in Radians per Second (rad/s) on the Y axis per BNO055
   angular velocity in Radians per Second (rad/s) on the Z axis per BNO055

*/

// wifi credentials below
 const char *ssid = "TrakBoxHotSpot";
const char *password = "trakbox1234";
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
double orx = -1000.000;
double ory = -1000.000;
double orz = -1000.000;
double lnx = -1000.000;
double lny = -1000.000;
double lnz = -1000.000;
bool iswifi = false;


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
  while (!Serial) delay(100);  // wait for serial port
  delay(5000);
  Serial.println("");
  Serial.println("TrakBox V1.013 starting"); Serial.println("");

  /* initialize the OLED display*/
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
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

  startWIFI();//pauses 30 seconds for wifi then continues

  if (!rtc.begin()) {
    ; //Start RCT
    Serial.println("RTC not detected ... Check your wiring or I2C ADDR!");
  } else {
    Serial.println("Real Time Clock running... ");
  }
  if (iswifi == true) {
    Serial.println("checking ntp...");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);//get time from ntp server
    getTime();//set rtc with npt time
    Serial.println("RTC synced to ntp");
  } else {
    Serial.println("RTC NOT synced to ntp");
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
    delay(30000);
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

  // Overclock the I2C bus speed to 400kHz Fast Mode
  Wire.setClock(400000);
  Serial.println("Overclocking I2C to 400kHz fast mode");

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
  printEvent();
  // Keep sample rate timing steady
  //delay(BNO055_SAMPLERATE_DELAY_MS);

}

// utility modules
//////////////////
void startWIFI() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("WiFi connecting "); Serial.println("");
  int cnt = 0;
  while (WiFi.waitForConnectResult() != WL_CONNECTED && cnt < 30) {
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
    cnt++;
  }
  if (WiFi.waitForConnectResult() == WL_CONNECTED) {
    iswifi = true;
    Serial.println("");
    Serial.println("wifi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    iswifi = false;
    Serial.println("");
    Serial.println("NO WIFI");
  }
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
  DateTime now = rtc.now();
  String h = "";
  String m = String(now.minute());
  if (now.minute() < 10) {
    m = "0" + m;
  }
  if (now.hour() > 12) {
    h = String(now.hour() - 12);
    m = m + " pm";
  } else {
    h = String(now.hour());
    m = m + " am";
  }
  String s = String(gps.speed.mph(), 0);

  if (var == "now_hour") return h;
  if (var == "now_minute") return m;
  if (var == "gps_speed_mph") return s;

  return String();
}

//////////////////////////////////////////
// Function to write the file to SD card
//////////////////////////////////////////
void printEvent() {
  //double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  
  // Returns fused, drift-corrected angular velocity in Radians per Second (rad/s)
  imu::Vector<3> gyroData = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  float gx = gyroData.x();
  float gy = gyroData.y();
  float gz = gyroData.z();
  
  // Returns fused, gravity-removed acceleration in Meters per Second Squared (m/s^2)
  imu::Vector<3> linearAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  
  double s = gps.speed.mph();
  //Serial.print("BME temp "); // for testing with serial monitor
  //Serial.println(bme.readTemperature()); // for testing with serial monitor
  if (s < 2) {
    Serial.print("Not logging --- ");
    Serial.print("GPS Time: ");
    Serial.println(gps.time.value());
    return;
  }
  // open the file for write at end
  if (!file.open("TrakBox.csv", O_RDWR | O_CREAT | O_AT_END)) {
    sd.errorHalt("opening TrakBox.csv for write failed");
    Serial.println("No SD card!!!");
    return;
  }
  // if the file opened okay, write to it:
  Serial.print("Writing to TrakBox.csv... millis:");
  Serial.println(millis());
  DateTime now = rtc.now();
  file.print(millis());
  file.print(",");
  file.print(now.month());
  file.print("/");
  file.print(now.day());
  file.print("/");
  file.print(now.year());
  file.print(",");
  file.print(now.hour());
  file.print(":");
  file.print(now.minute());
  file.print(":");
  file.print(now.second());
  file.print(",");

  file.print(gps.location.lat(), 6);
  file.print(",");
  file.print(gps.location.lng(), 6);
  file.print(",");
  file.print(gps.speed.mph());
  file.print(",");

  file.print(bme.readAltitude(SEALEVELPRESSURE_HPA) * 3.281);
  file.print(",");
  file.print(bme.readTemperature());
  file.print(",");
  file.print(bme.readHumidity());
  file.print(",");
  file.print(bme.readPressure() / 100.0F);
  file.print(",");

  file.print(linearAccel.x()); file.print(",");
  file.print(linearAccel.y()); file.print(",");
  file.print(linearAccel.z()); file.print(",");
  file.print(gx); file.print(",");
  file.print(gy); file.print(",");
  file.println(gz); // End row cleanly

  // close the file:
  file.close();
}
