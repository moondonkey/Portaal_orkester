#include <esp_now.h>
#include <Arduino.h>
#include <WiFi.h>
#include "FastAccelStepper.h"

#define dirPin 17
#define stepPin 16
#define dirPin2 19
#define stepPin2 18
#define dirPin3 22
#define stepPin3 1
#define hallSensor 33
#define hallSensor2 34
#define limit2 35
const int ledPin1 = 12;

int tuningRange = 53000;
int homingSpeed1 = 640;
int homingSpeed2 = 2400;
int homingSpeed3 = 240;
int startPos1 = 0;
int startPos2 = tuningRange + 50;
int startPos3 = 0;
int lastDiskState = 0;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
FastAccelStepper *stepper2 = NULL;
FastAccelStepper *stepper3 = NULL;

int dutyCycle1;
int positsioon = 2;
int positsioon2 = 65000;
int positsioon3 = 2;
int kiirus = 0;
int diskKiirendus = 24000;
int kiirus2 = 0;
int kiirus3 = 0;
int laeng = 0;

// setting PWM properties
const int freq = 5000;
const int ledChannel1 = 0;
const int resolution = 8;

int getData[7];

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&getData, incomingData, sizeof(getData));

  dutyCycle1 =getData[0]; // Light output
  kiirus = map(getData[1], 0, 255, 0, 55000); // speed of the disk
  positsioon =  map(getData[2], 0, 255, 0, 3); // direction of the disk
  positsioon2 =  map(getData[3], 0, 255, tuningRange, 1); // tuning
  kiirus2 =  map(getData[4], 0, 255, 0, 20000);  // tuning speed
  positsioon3 =  map(getData[5], 0, 255, 0, 3); // pick direction 
  kiirus3 = map(getData[6], 0, 255, 0, 62000); // pick speed
}

void homing1(){
  Serial.println("homing1");
  
  while(analogRead(hallSensor) >= 1050 && analogRead(hallSensor) <= 2525 ) {
    
    stepper->setSpeedInHz(homingSpeed1);
    stepper->runForward();
  }
  stepper->setSpeedInHz(homingSpeed1 * 4);
  stepper->move(homingSpeed1 * -2);
  delay(1000);
  while(analogRead(hallSensor) >= 1050 && analogRead(hallSensor) <= 2525) {
   
    stepper->setSpeedInHz(homingSpeed1/2);
    stepper->runForward();
  }
    stepper->stopMove();
    delay(200);
    stepper->setCurrentPosition(startPos1);
    Serial.print("homed ");
    Serial.println(stepper->getCurrentPosition());

}

void homing2(){ 
    while(analogRead(limit2) <= 2500 ){
      ledcWrite(ledChannel1, 60);
      stepper2->setSpeedInHz(homingSpeed2);
      stepper2->runForward();
    }
  ledcWrite(ledChannel1, 0);  
  stepper2->setSpeedInHz(homingSpeed2 * 2);
  stepper2->move(homingSpeed2 * -2);
  delay(1000);
  while(analogRead(limit2) <= 2500) {
    ledcWrite(ledChannel1, 60);
    stepper2->setSpeedInHz(homingSpeed2 /2);
    stepper2->runForward();
  }
    ledcWrite(ledChannel1, 0);
    stepper2->stopMove();
    delay(200);
    stepper2->setCurrentPosition(startPos2);
    Serial.print("homed2 ");
    Serial.println(stepper2->getCurrentPosition());
    kiirus2 = 0;
    stepper2->setSpeedInHz(0);
}

void homing3(){
   Serial.println("homing3");
  //  Serial.println(analogRead(hallSensor2));
    while(analogRead(hallSensor2) <= 2525){
      stepper3->setSpeedInHz(homingSpeed3);
      stepper3->runForward();
    }
    stepper3->setSpeedInHz(homingSpeed3 * 4);
  stepper3->move(homingSpeed3 * -1);
  delay(1000);
  while(analogRead(hallSensor2) <= 2525) {
   // Serial.println(analogRead(hallSensor2));
    stepper3->setSpeedInHz(homingSpeed3/2);
    stepper3->runForward();
  }
    stepper3->stopMove();
    delay(200);
    stepper3->setCurrentPosition(startPos3);
    Serial.print("homed3 ");
    Serial.println(stepper3->getCurrentPosition());
    kiirus3 = 0;
    stepper3->setSpeedInHz(0);
}

void setup() {

  pinMode(ledPin1, OUTPUT);
  pinMode(limit2, INPUT);
  pinMode(hallSensor, INPUT);
  pinMode(hallSensor2, INPUT);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_recv_cb(OnDataRecv);
  
  ledcSetup(ledChannel1, freq, resolution);
  ledcAttachPin(ledPin1, ledChannel1);

  engine.init();
   stepper = engine.stepperConnectToPin(stepPin);
   stepper2 = engine.stepperConnectToPin(stepPin2);
   stepper3 = engine.stepperConnectToPin(stepPin3);

  if (stepper) 
   {
      stepper->setDirectionPin(dirPin);
      stepper->setAutoEnable(false);
      stepper->setSpeedInHz(kiirus);
      stepper->setAcceleration(diskKiirendus); 
      
   }
     if (stepper2) 
   {
      stepper2->setDirectionPin(dirPin2);
      stepper2->setAutoEnable(false);
      stepper2->setSpeedInHz(kiirus2); 
      stepper2->setAcceleration(24000); 
      
   } 
   if (stepper3) 
   {
      stepper3->setDirectionPin(dirPin3);
      stepper3->setAutoEnable(false);
      stepper3->setSpeedInHz(kiirus3); 
      stepper3->setAcceleration(32000); 
      
   }
  
  homing1();
  homing2();
  homing3();
  }


void loop() {
  ledcWrite(ledChannel1, dutyCycle1);
  
  

      if (positsioon == 3 && lastDiskState != positsioon){
        lastDiskState = positsioon;
        stepper->setSpeedInHz(kiirus);
        stepper->runForward(); 
        
      }
      else if (positsioon == 3 && lastDiskState == positsioon){
        stepper->setSpeedInHz(kiirus);
        stepper->runForward(); 
        
      }

      else if ((positsioon == 0 || positsioon == 2) && lastDiskState != positsioon){
        lastDiskState = positsioon; 
        int peatumisVahemaa;
        long int asukoht = stepper->getCurrentPosition();
        int hetkKiirus = stepper->getCurrentSpeedInMilliHz() / 1000;
        peatumisVahemaa = sq(hetkKiirus) / (2 * diskKiirendus);

       if(stepper->getCurrentSpeedInMilliHz() > 0){  
        int minAbsSeiskumispunkt = (asukoht % 6400) + abs(peatumisVahemaa);
        Serial.println((6400 - (minAbsSeiskumispunkt % 6400) + abs(peatumisVahemaa) - (hetkKiirus / 50)) * -1);
        stepper->move(6400 - (minAbsSeiskumispunkt % 6400) + abs(peatumisVahemaa) - (hetkKiirus / 50));
       }
       else{
        int minAbsSeiskumispunkt = (-asukoht % 6400) - (peatumisVahemaa);
        Serial.println((6400 - (minAbsSeiskumispunkt % 6400) + abs(peatumisVahemaa) - (hetkKiirus / 50)) * -1);
        stepper->move((6400 - (minAbsSeiskumispunkt % 6400) - abs(peatumisVahemaa) + (hetkKiirus / 50)) * -1);
       }
      }

      else if (positsioon == 1 && lastDiskState != positsioon){
        lastDiskState = positsioon;
        stepper->setSpeedInHz(kiirus);
        stepper->runBackward();     
      }
       else if (positsioon == 1 && lastDiskState == positsioon){
        stepper->setSpeedInHz(kiirus);
        stepper->runBackward();     
      }
      else {
        
      }

       if (stepper2) 
   {
      
      stepper2->setAutoEnable(false);
      stepper2->setSpeedInHz(kiirus2);
      stepper2->moveTo(positsioon2);
   }

       if (positsioon3 == 3){
        stepper3->setSpeedInHz(kiirus3);
        stepper3->runForward();        
      }
      else if (positsioon3 == 2 || positsioon3 == 0){
        stepper3->stopMove();     
      }
      else if (positsioon3 == 1){
        stepper3->setSpeedInHz(kiirus3);
        stepper3->runBackward();
        
      }


}

/* 
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-web-server-websocket-sliders/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/