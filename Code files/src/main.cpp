#include <Arduino.h>
#include "ThingSpeak.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <WiFi.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
//WIFI SETTING
#define WIFI_NETWORK "WE783624"
#define WIFI_PASSWORD "habebaahmed123"
#define WIFI_TIMEOUT 20000

//THINGSPEAK SETTING
#define CHANNELID 2071403
#define CHANNEL_API_WRITE_KEY "W43UNB33KUAVE3TW"
WiFiClient Client;

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
void ThingSpeakWIFI(){
  if(WiFi.status()==WL_CONNECTED){
    ThingSpeak.begin(Client);

  }
}

void ThingSpeakGYROSCOPE(){
  //ThingSpeak.writeField()
  delay(15000);
}

void setup() {
  Serial.begin(9600);
  connectToWIFI();
  ThingSpeakWIFI();
}

void loop() {
  void ThingSpeakGYROSCOPE();
}