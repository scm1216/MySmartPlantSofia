/* 
 * Project myProject
 * Author: Sofia Cortes 
 * Date: 19- MArch -2025
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "Adafruit_BME280.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_GFX.h"
#include "neopixel.h"
#include "Colors.h"
#include "Air_Quality_Sensor.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "credentials.h"


//SYSTEM
SYSTEM_MODE(AUTOMATIC);

//ADAFRUIT.IO
TCPClient TheClient; 
// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 
/****************************** Feeds ***************************************/ 
// Setup Feeds to publish or subscribe 
Adafruit_MQTT_Subscribe buttonWater = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/waterplant"); 
Adafruit_MQTT_Publish TEMP = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");
Adafruit_MQTT_Publish HUMIDITY = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");
Adafruit_MQTT_Publish MOISTURE = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/moisture");
Adafruit_MQTT_Publish AIRQUALITY = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/airquality");
/************Declare Functions and Variables*************/
int buttonState;
unsigned long publishTime;
void MQTT_connect();
bool MQTT_ping();

//MOISTURE SENSOR
int soilMoist= D14;
int moistureReads;

//TIME
String dateTime, timeOnly;
unsigned int lastTime;

//BME SENSOR
Adafruit_BME280 bme;
bool status;
const int hexAddress = 0x76;
unsigned int currentTime;
unsigned int lastSecond;
float tempC;
float tempF;
float humidRH;
const byte PERCENT = 37;
const byte DEGREE  = 167;

//AIR QUALITY SENSOR
AirQualitySensor sensor (A0);
int quality;

//OLED DISPLAY
const int OLED_RESET =-1;
Adafruit_SSD1306 display(OLED_RESET);
#define SCREEN_HEIGHT 16
#define SCREEN_WIDTH  16

//WATER PUMP
const int WATER_PUMP = D16;
unsigned int currentTimeWater;
unsigned int lastSecondWater;

//NEOPIXEL
const int PIXELCOUNT = 61;
int brightness = 50;
int i;
int j;
Adafruit_NeoPixel pixel(PIXELCOUNT, SPI1, WS2812B);


void setup() {
//SERIAL MONITOR
  Serial.begin(9600);
  waitFor(Serial.isConnected,10000);
  
  Serial.begin(9600);
  delay (3000);

//ADAFRUIT.IO
 Serial.printf("Connecting to Internet \n");
 WiFi.connect();
 while(WiFi.connecting()) {
   Serial.printf(".");
 }
 Serial.printf("\n Connected!!!!!! \n");
  // Setup MQTT subscription
   mqtt.subscribe(&buttonWater);

//DATE 
Time.zone(-7);
Particle.syncTime();

//MOISTURE
  pinMode (soilMoist,INPUT);

//BME 
status = bme.begin (hexAddress); 
  if (status== false){
    Serial.printf("BME280 at address 0x%02x failed to start", hexAddress);
  }

//AIR QUALITY
Serial.println("Waiting sensor to init...");
if (sensor.init()) {
  Serial.println("Sensor ready.");
}
else {
    Serial.println("Sensor ERROR!");
}
//OLED
display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
display.clearDisplay();

//WATER PUMP
pinMode (WATER_PUMP, OUTPUT);

//NEOPIXELS
pixel.begin ();
pixel.setBrightness(brightness);
pixel.show();
}

void loop() {

  //TIME
dateTime = Time.timeStr ();
timeOnly = dateTime. substring (11,19);
if(millis()-lastTime>28800000){
  lastTime = millis();
  Serial.printf("Date and Time is %s\n",dateTime.c_str());
  Serial.printf("Time is %s\n",timeOnly.c_str());

//ADAFRUIT
 if( mqtt.Update() ){
// this is our 'wait for incoming subscription packets' busy subloop 
   Adafruit_MQTT_Subscribe *subscription;
   while ((subscription = mqtt.readSubscription(100))) {
     if (subscription == &buttonWater) {
       buttonState = atof((char *)buttonWater.lastread);
       Serial.printf("Button State: %d\n", buttonState);
 
       if (buttonState==1){
       digitalWrite(WATER_PUMP, HIGH);
       }else {
       digitalWrite(WATER_PUMP,LOW);
       }
     }
   if (mqtt.Update()) {
    TEMP.publish (tempF);
    HUMIDITY.publish (humidRH);
    MOISTURE.publish (moistureReads);
    AIRQUALITY.publish (quality); 
    }
  publishTime = millis();

  }
}
//MOISTURE
moistureReads = analogRead(soilMoist);
Serial.printf("Moisture is %i\n", moistureReads);

//BME
tempC = bme.readTemperature();
humidRH = bme.readHumidity();
tempF = (tempC*9/5)+32;
  Serial.printf("Temp: %.2f%c\n ", tempF,DEGREE); 
  Serial.printf("Humi: %.2f%c\n",humidRH,PERCENT);

//AIR QUALITY
int quality = sensor.slope();
Serial.print("Sensor value: ");
Serial.println(sensor.getValue());

if (quality == AirQualitySensor::FORCE_SIGNAL) {
  Serial.println("High pollution! Force signal active.");
  for(i=0;i<60;i++){
    pixel.setPixelColor(i,red);
    pixel.show();
      delay(50);
  }
  pixel.show();
  pixel.clear();  
}
else if (quality == AirQualitySensor::HIGH_POLLUTION) {
  Serial.println("High pollution!");
  for(i=0;i<60;i++){
    pixel.setPixelColor(i,orange);
    pixel.show();
    delay(50);
}
pixel.show();
pixel.clear();  
}
else if (quality == AirQualitySensor::LOW_POLLUTION) {
  Serial.println("Low pollution!");
  for(i=0;i<60;i++){
    pixel.setPixelColor(i,yellow);
    pixel.show();
    delay(50);
}
pixel.show();
pixel.clear();  
}
else if (quality == AirQualitySensor::FRESH_AIR) {
  Serial.println("Fresh air."); 
  for(i=0;i<60;i++){
    pixel.setPixelColor(i,green);
    pixel.show();
    delay(50);
}
pixel.show();
pixel.clear();
}  
//WATER PUMP
if (moistureReads>3000){
    digitalWrite(WATER_PUMP,HIGH);
    pixel.setPixelColor(60,blue);
    pixel.show();
    Serial.printf("Pump ON\n");
    delay(500); //approved by Brian
    digitalWrite(WATER_PUMP,LOW);
    Serial.printf("Pump OFF:\n");
      pixel.setPixelColor(60,black);
      pixel.show();
}
}
//OLED
display.clearDisplay();
display.setTextSize(1);
display.setTextColor(WHITE);
display.setCursor(0,5);
display.printf("Time: %s\n",timeOnly.c_str());
display.setCursor(0,15);
display.printf("Soil moisture: %i\n", moistureReads);
display.setCursor(0,25);
display.printf("Room Temp: %.2f%c\n ",tempF,DEGREE); 
display.setCursor(0,35);
display.printf("Humidity Room: %.2f%c\n",humidRH,PERCENT);
display.setCursor(0,50);
display.setTextSize(1.9);
display.print("  MY SMART PLANT");
display.display();
}

void MQTT_connect() {
  int8_t ret;
 
  // Return if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds...\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}

bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;

  if ((millis()-last)>6000) {
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
}