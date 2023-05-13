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

/*
   TrakBox V1.005 6th version
   Andrew Rowe May 13 2023
   This version adds a dash function using an async web server

   the compass function from version 1.3 does not work
   the web server dash function from version 1.4 does not work
   this version starts with version 1.3 and adds async web server

   includes all changes from 1.2 below:
   Write to SD card
   This version writes data to a file on the root of the SD card
   The file name is TrakBox.txt
   It is a comma delimited file
   It outputs time at the meridian (no offset for time zones)

   the output is as follows:
   hh:mm.ss.cc, lat, long, gps speed, altitude, temperature,

   Then on the end of the same line it outputs one of the following:
   orientation angVelocity linearAccel magnetometer accelerometer gravity
   in xyz format
*/

// add your wifi credentials below
const char *ssid = "************";
const char *password = "****************";
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
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
uint16_t PRINT_DELAY_MS = 500; // how often to print the data
uint16_t printCount = 0; //counter to avoid printing every 10MS sample
const uint8_t SD_CS_PIN = SS;
String cardinal;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_BME280 bme; // I2C
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
SdFs sd;
FsFile file;
AsyncWebServer server(80);

void setup()
{
  Serial.begin(115200);
  while (!Serial) delay(10);  // wait for serial port to open!
  Serial.println("TrakBox starting"); Serial.println("");
  ss.begin(GPSBaud);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.printf("WiFi Failed!\n");
    return;
  }

  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    AsyncResponseStream *response = request->beginResponseStream("text/html");
    response->addHeader("Server", "TrakBox Dash Server");
    response->printf("<!DOCTYPE html><html><head>", request->url().c_str());
    response->print("<title>TrakBox Dash</title>");
    response->print("<style>");
    response->print("html { font-family: sans-serif; display: inline-block; margin: 0px auto; text-align: center; background-color: black;}");
    response->print(".bigtxt { font-size: 280px; color: green;}");
    response->print(".littletxt { font-size: 80px; color: green;}");
    response->print("</style>");
    response->print("<meta http-equiv=\"refresh\" content=\"1\">");
    response->print("</head><body>");
    response->print("<span class=\"bigtxt\">");
    response->print(gps.speed.mph());
    response->printf("</span>");
    response->print("<span class=\"littletxt\">");
    response->print("MPH");
    response->printf("</span>");
    response->print("</body></html>");
    //send the response last
    request->send(response);
  });
  server.onNotFound(notFound);
  server.begin();
  
  /* Initialise the accel/gyro */
  if (!bno.begin())
  {
    Serial.print("BNO055 not detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  Serial.println("BNO055 sensor running... "); Serial.println("");
  /* initialize the OLED display*/
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("Display not detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  /* initialize the BME180*/
  if (!bme.begin(0x76)) {
    Serial.println("BME180 not detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  Serial.println("BME sensor running... "); Serial.println("");

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
    return;
  }

  // Clear the buffer.
  display.clearDisplay();
  // Display Splash Text
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
    display.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    display.println(" m");
    //
    display.setCursor(0, 46);
    display.setTextSize(1);
    display.print("Time ");
    display.print(gps.time.hour());
    display.print(":");
    display.println(gps.time.minute());
    //
    //float Fdeg;
    // Fdeg = 9.0 * degCin / 5.0 + 32.0;
    display.setCursor(0, 57);
    display.setTextSize(1);
    display.print("Temp: ");
    display.println(bme.readTemperature());
    //
    display.setCursor(70, 40);
    display.setTextSize(3);
    //display.print("Temp: ");
    display.println(cardinal);
    //
    display.display();

    printCount = 0;
  }
  else {
    printCount = printCount + 1;
  }
}

void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  // open the file for write at end like the Native SD library
  if (!file.open("TrakBox.txt", O_RDWR | O_CREAT | O_AT_END)) {
    sd.errorHalt("opening TrakBox.txt for write failed");
  }
  // if the file opened okay, write to it:
  Serial.println("Writing to TrakBox.txt...");
  file.print(gps.time.hour());
  file.print(":");
  file.print(gps.time.minute());
  file.print(":");
  file.print(gps.time.second());
  file.print(".");
  file.print(gps.time.centisecond());
  file.print(",");
  file.print(gps.location.lat(), 6);
  file.print(",");
  file.print(gps.location.lng(), 6);
  file.print(",");
  file.print(gps.speed.mph());
  file.print(" MPH");
  file.print(",");
  file.print("Alt ");
  file.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  file.print(" m, ");
  file.print(bme.readTemperature());
  file.print(" °C");
  file.print(",");
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
    //String cardinal;
    float headingRadians = atan2(y, x);
    float headingDegrees = headingRadians * 180 / PI;
    float declinationAngle = -7.316666667;
    //float declinationAngle = 11.41666666666667;//sample

    headingDegrees += declinationAngle;

    if (headingDegrees < 0) {
      headingDegrees += 360;
    }

    if (headingDegrees > 338 || headingDegrees < 22) {
      cardinal = " N";
    }
    else if (headingDegrees > 23 && headingDegrees < 67) {
      cardinal = " NE";
    }
    else if (headingDegrees > 68 && headingDegrees < 112) {
      cardinal = " E";
    }
    else if (headingDegrees > 113 && headingDegrees < 157) {
      cardinal = " SE";
    }
    else if (headingDegrees > 158 && headingDegrees < 201) {
      cardinal = " S";
    }
    else if (headingDegrees > 202 && headingDegrees < 247) {
      cardinal = " SW";
    }
    else if (headingDegrees > 248 && headingDegrees < 292) {
      cardinal = " W";
    }
    else if (headingDegrees > 292 && headingDegrees < 337) {
      cardinal = " NW";
    }
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
