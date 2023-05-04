#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <WiFi.h>
#include "ThingSpeak.h"

//MPU Libraries
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
//I2C Library
#include <Wire.h>
//Max30100 Library
#include "MAX30100_PulseOximeter.h"
#define REPORTING_PERIOD_MS 1000
//MAX30100 Setting
PulseOximeter pox;
uint32_t tsLastReport = 0;
float SP02Reading;
int BPMReading;
//WIFI SETTING
#define WIFI_NETWORK "WE783624"
#define WIFI_PASSWORD "habebaahmed123"
#define WIFI_TIMEOUT 20000

//THINGSPEAK SETTING
#define CHANNELID 2071403
#define CHANNEL_API_WRITE_KEY "W43UNB33KUAVE3TW"
WiFiClient Client;
//MPU Object
Adafruit_MPU6050 mpu;

 //OximeterFunctions
void onBeatDetected()
{
    Serial.println("Beat!");
}
 void oxiemeterSetup(){
  if (!pox.begin()) {
        Serial.println("FAILED");
        for(;;);
    } else {
        Serial.println("SUCCESS");
    }
     pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
 
    // Register a callback for the beat detection
    pox.setOnBeatDetectedCallback(onBeatDetected);
 }
 void oxiemeterMeasuring(){
   pox.update();
    if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
        Serial.print("Heart rate:");
        BPMReading=pox.getHeartRate();
        Serial.print(pox.getHeartRate());
        Serial.print("bpm / SpO2:");
        SP02Reading=pox.getSpO2();
        Serial.print(pox.getSpO2());
        Serial.println("%");
 
        tsLastReport = millis();
    }
 }

//WIFI FUNCTION
void connectToWIFI(){
  Serial.print("connecting to network...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_NETWORK,WIFI_PASSWORD);
  unsigned long startAttemptTime=millis();
  while(WiFi.status()!=WL_CONNECTED
   && millis()-startAttemptTime<WIFI_TIMEOUT);{
    Serial.print(".");
    delay(100);
   }
   if(WiFi.status()!=WL_CONNECTED){
    Serial.println("Failed");
   }
   else{
    Serial.print("Connected");
    Serial.println(WiFi.localIP());
   }
  }
//ThingSpeak Write Functions
void ThingSpeakWIFI(){
  if(WiFi.status()==WL_CONNECTED){
    ThingSpeak.begin(Client);

  }
}
void ThingSpeakOxiemeter(){
  ThingSpeak.writeField(CHANNELID,2,BPMReading,CHANNEL_API_WRITE_KEY);
  delay(15000);
    ThingSpeak.writeField(CHANNELID,3,SP02Reading,CHANNEL_API_WRITE_KEY);
delay(15000);

}

void setup() {
  Serial.begin(9600);
  connectToWIFI();
  ThingSpeakWIFI();
  void oxiemeterSetup();
}

void loop() {
}






