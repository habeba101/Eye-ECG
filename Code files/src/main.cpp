#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

Adafruit_MPU6050 mpu;
MAX30105 particleSensor;

uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

long samplesTaken = 0; //Counter for calculating the Hz or read rate
long unblockedValue; //Average IR at power up
long startTime; //Used to calculate measurement rate

//Define UUID
#define SERVICE_UUID            "99e21b25-40e3-4392-94e6-832c95957d31"
#define CHARACTERSTIC_UUID_TX   "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// Create a BLE characteristic object
BLECharacteristic* pCharacteristic =NULL;
bool deviceConnected = false;
int txValue =0;

//fUNCTION TO CONNECT BLE
 
class MyServerCallbacks: public BLEServerCallbacks
{
   //this class contain two functions 1-connected 2-disconnected
   void onConnect(BLEServer* pServer) 
   {
       deviceConnected = true;
   };
   void onDisconnect(BLEServer* pServer)
   {
    deviceConnected = false;};

};


void max30102Setup(); 
void SPO2_BPM();
void gyroscopesetup();
void gyroscopeloop();
void ECGsetup();
void ECGloop();
void bluetoothsetup();
void bluetoothloop();
void setup() 
{
  Serial.begin(115200); // initialize serial communication at 115200 bits per second:
  bluetoothsetup();
  max30102Setup();
  gyroscopesetup();

}

void loop() 
{
  bool flag=false;
  bluetoothloop();
  samplesTaken++;
  long currentDelta = particleSensor.getIR() - unblockedValue;
  unsigned long now = millis();
  
  delay(3000);
if (currentDelta > (long)100)
  {
   SPO2_BPM();
  }
  else if(!flag){
  gyroscopeloop();
  flag=true;
  delay(500);
  }
   if(flag){
    ECGloop();
    flag=false;
    delay(500);
  }

}
void max30102Setup(){
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }
  
  particleSensor.setup(); //Configure sensor with these settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to  indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
  
  unblockedValue = 0;
  for (byte x = 0 ; x < 32 ; x++)
  {
    unblockedValue += particleSensor.getIR(); //Read the IR value
  }
  unblockedValue /= 32;
  startTime = millis();

}
void SPO2_BPM(){
    bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps
    for (byte i = 0 ; i < bufferLength ; i++) //read the first 100 samples, and determine the signal range
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample
    }
    //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
    
      //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
      //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
      for (byte i = 25; i < 100; i++)
      {
        redBuffer[i - 25] = redBuffer[i];
        irBuffer[i - 25] = irBuffer[i];
      }
     
      //take 25 sets of samples before calculating the heart rate.
      for (byte i = 75; i < 100; i++)
      {
        while (particleSensor.available() == false) //do we have new data?
          particleSensor.check(); //Check the sensor for new data

        redBuffer[i] = particleSensor.getRed();
        irBuffer[i] = particleSensor.getIR();
        particleSensor.nextSample(); //We're finished with this sample so move to next sample
        if((spo2!=-999)&& irBuffer[i]>50000)
        {
        //send samples and calculation result to terminal program through UART
        Serial.print(F("red="));
        Serial.print(redBuffer[i], DEC);
        Serial.print(F(", ir="));
        Serial.print(irBuffer[i], DEC);
        Serial.print(F(", SPO2="));
        Serial.print(spo2, DEC);
        Serial.println();
        delay(500);
        }
        maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
      
        long irValue = particleSensor.getIR();
        if (checkForBeat(irValue) != true)
        {
          //We sensed a beat!
          long delta = millis() - lastBeat;
          lastBeat = millis();

          beatsPerMinute = 60000.0 / (delta);
          if (beatsPerMinute < 255 && beatsPerMinute > 20)
            {
          rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
          rateSpot %= RATE_SIZE; //Wrap variable

          //Take average of readings
          beatAvg = 0;
          for (byte x = 0 ; x < RATE_SIZE ; x++)
            beatAvg += rates[x];

          beatAvg /= RATE_SIZE;
            }
           if(irBuffer[i]>50000){
            Serial.print(" BPM=");
            Serial.print(beatsPerMinute);
            Serial.print(", Avg BPM=");
            Serial.print(beatAvg);
            Serial.println();
            delay(500);
           }
         } 
      
     }
   }

   void gyroscopeloop(){
    sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        float acc_x = a.acceleration.x;
        float acc_y = a.acceleration.y;
        float acc_z = a.acceleration.z;
       
        if (acc_z < 9.8 && acc_z> 8 ) 
        {
          Serial.println("Sitting");
          delay(100);
        }
         else if (acc_y > 8.5 ) 
        {
          Serial.println("Standing");
          delay(100);
        }
         else if(acc_x >7 || acc_x <(-7) )
        {
          Serial.println("Sleeping");
          delay(100);
        }
        delay(300);
   }
   void gyroscopesetup(){
    if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
}
   }
   void ECGsetup() {
// initialize the serial communication:

pinMode(32, INPUT); // Setup for leads off detection LO +
pinMode(35, INPUT); // Setup for leads off detection LO -

}

void ECGloop() {
if((digitalRead(32) == 1)||(digitalRead(35) == 1))
{
//Serial.println('!');
}
else{
if( analogRead(A0)>0 && analogRead(A0) <1024)
{
//Serial.println(analogRead(A0));
}
}
//Wait for a bit to keep serial data from saturating
delay(1);
}
void bluetoothsetup(){
// Initialize the BLE device
 BLEDevice::init("ESP32 BLE Device");

  // Create a BLE server object
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());


  // Create a BLE service object
  BLEService *pService =  pServer->createService(SERVICE_UUID);

  // Create a BLE characteristic object
  pCharacteristic = pService->createCharacteristic(
  CHARACTERSTIC_UUID_TX ,
  //BLECharacteristic::PROPERTY_READ |
  BLECharacteristic::PROPERTY_NOTIFY
);

pCharacteristic->setValue("Connected");

  // BLE needed to notify 
  pCharacteristic->addDescriptor(new BLE2902());
  
  //Start the service 
  pService->start();

  // Start advertising the BLE service
 // BLEAdvertising *pAdvertis = pServer->getAdvertising();
  pServer->getAdvertising()->start();
  Serial.println("waiting for a client connection to notify");
  delay(100);
  Serial.println("Connected");
}
void bluetoothloop(){
if(deviceConnected)
   {
     txValue=random(-10,20);
     //convert tx
     char txString[8];
     dtostrf(txValue,1,2, txString);
     //setting the value to the charactersitic
     pCharacteristic->setValue(txString);
     //connect to the client
     pCharacteristic->notify();
     Serial.println("Sent value :"+ String(txString));
     //delay(500);}
}
}