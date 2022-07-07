#include <esp_now.h>
#include <Arduino.h>
#include <WiFi.h>
#include "FastAccelStepper.h"

#define leadscrew 4 // 1 - 2mm pitch ;;;; 4 - 4mm pitch
#define hall_sensor 2 // 1- 49E, 2-digital
const int machine = 5;
#define dirPin 17
#define stepPin 16

#define dirPin2 19

#define stepPin2 18
#define dirPin3 23
#define stepPin3 22
#define dirPin4 14
#define stepPin4 13
#define hallSensor 33
#define hallSensor2 34
#define limit2 35
#define limit4 32
const int ledPin1 = 12;

bool homed4 = false;
bool homed3 = false;

int tuningRange = 60000; //53000
int homingSpeed1 = 640;
int homingSpeed2 = 2400;
int homingSpeed3 = 2400;
int homingSpeed4 = 2400 / leadscrew;
int startPos1 = 0;
int startPos2 = 0; //tuningRange + 50;
int startPos3 = 0;
int startPos4 = 0;
int lastDiskState = 0;
int lastDiskState2 = 0;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
FastAccelStepper *stepper2 = NULL;
FastAccelStepper *stepper3 = NULL;
FastAccelStepper *stepper4 = NULL;

int dutyCycle1;
int positsioon = 2;
int positsioon2 = 0;
int positsioon3 = 2;
int positsioon4 = 0;
int kiirus = 0; //speed of the disk
int diskKiirendus = 24000;
int diskKiirendus2 = 24000;
int kiirus2 = 0;
int kiirus3 = 0;
int laeng = 0;

// setting PWM properties
const int freq = 5000;
const int ledChannel1 = 0;
const int resolution = 8;

int getData[8];

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&getData, incomingData, sizeof(getData));

  dutyCycle1 =getData[0]; // Light output
  if(getData[1] == 2){
    kiirus = 200;
  }
  else{
    kiirus = map(getData[1], 0, 255, 0, 55000); // speed of the disk
  }
  
  if(getData[2] == 0){
    positsioon = 0;
  }
  else if(getData[2] > 0 && getData[2] < 128){
    positsioon = 1;
  }
  else if(getData[2] > 128){
    positsioon = 3;
  }
  positsioon2 =  map(getData[3], 0, 255, 0, -tuningRange); // tuning
  //kiirus2 =  map(getData[4], 0, 255, 0, 20000);  // tuning speed
  if(getData[4] == 0){
    positsioon3 = 0;
  }
  else if(getData[4] > 0 && getData[4] < 128){
    positsioon3 = 1;
  }
  else if(getData[4] > 128){
    positsioon3 = 3;
  }
  kiirus3 = map(getData[5], 0, 255, 0,80000); // pick speed
  switch(machine){
    case 1:
      positsioon4 = map(getData[6], 0, 255, 0, 28000/leadscrew); // string distance
      break;
    case 2:
      positsioon4 = map(getData[6], 0, 255, 19460, 35000/leadscrew); // string distance
      break;
    case 3:
      positsioon4 = map(getData[6], 0, 255, 0, 23625/leadscrew); // string distance
      break;
    case 4:
      positsioon4 = map(getData[6], 0, 255, 3000/leadscrew, 31000/leadscrew); // string distance
      break;
    case 5:
      positsioon4 = map(getData[6], 0, 255, 10900/leadscrew,  35000/leadscrew ); // string distance
      break;
    case 6:
      positsioon4 = map(getData[6], 0, 255, 0, 25375/leadscrew); // string distance
      break;
    case 7:
      positsioon4 = map(getData[6], 0, 255, 0, 40000/leadscrew); // string distance
      break;
  
  }

  // if(getData[7] == 1){
  //   ESP.restart();
  // } 
}

void homingKetas(){
  stepper->setSpeedInHz(homingSpeed1 * 2);
  stepper->runForward();
  for(int i = 1; i<9; i++){
    ledcWrite(ledChannel1, (i * 32 )- 1);
    delay(500);
  }
  stepper->stopMove();
  stepper->setSpeedInHz(0);
  ledcWrite(ledChannel1, 0);
}
void homing1(){
  Serial.println("homing1");
  Serial.println(analogRead(hallSensor));
  //while(!digitalRead(hallSensor)) {
  while(analogRead(hallSensor) >= 1200 && analogRead(hallSensor) <= 2400 ) {
    Serial.println(analogRead(hallSensor));
    stepper->setSpeedInHz(homingSpeed1);
    stepper->runForward();
  }
  stepper->setSpeedInHz(homingSpeed1 * 4);
  stepper->move(homingSpeed1 * -2);
  delay(1000);
  //while(!digitalRead(hallSensor)) {
  while(analogRead(hallSensor) >= 1200 && analogRead(hallSensor) <= 2400 ) {
    Serial.println(analogRead(hallSensor));
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
  unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
  unsigned long debounceDelay = 5
  0;
  bool homed2 = false;
  bool lastReading = 0;
  Serial.println("Homing 2");
  stepper2->move(-6 * homingSpeed2);
  ledcWrite(ledChannel1, 30);
  delay(1000);
  while(homed2 == false){
    bool reading = digitalRead(limit2);
    Serial.print("digitalRead");
    Serial.println(digitalRead(limit2) );
    ledcWrite(ledChannel1, 60);
    stepper2->setSpeedInHz(homingSpeed2/2);
    stepper2->runForward();

    if (reading != lastReading){

    }
        if (((millis() - lastDebounceTime) > debounceDelay) && reading == true){
          homed2 = true; 
          stepper2->setSpeedInHz(homingSpeed2 * 2);
          stepper2->move(homingSpeed2 * -1);
          delay(1000);
          stepper2->setCurrentPosition(0);
          stepper2->setSpeedInHz(0);
          Serial.println("homed 2");
        }

    lastReading = reading;

  }

}

void homing3(){

  unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
  unsigned long debounceDelay = 100;
  bool homed32 = false;
  homed3 = false;
  bool lastReading = 0;
  Serial.println("Homing 3");
  while(homed3 == false){
    bool reading = digitalRead(hallSensor);

    ledcWrite(ledChannel1, 60);
    stepper3->setSpeedInHz(homingSpeed3 / 2);
    stepper3->runForward();

    if (reading != lastReading){
      lastDebounceTime = millis();
    }

    if (((millis() - lastDebounceTime) > debounceDelay) && !reading){
      homed3 = true; 
      ledcWrite(ledChannel1, 0);
      stepper3->setSpeedInHz(homingSpeed3 * 2);
      stepper3->move(homingSpeed3 / -2);
      delay(1000);
    }

    lastReading = reading;
  }
  lastReading = 0;
  delay (200);
  while(homed32 == false && homed3 == true){
    bool reading = digitalRead(hallSensor);

    ledcWrite(ledChannel1, 30);
    stepper3->setSpeedInHz(homingSpeed3 / 6);
    stepper3->runForward();

    if (reading != lastReading){
      lastDebounceTime = millis();
    }

    if (((millis() - lastDebounceTime) > debounceDelay) && !reading){
      homed32 = true; 
      ledcWrite(ledChannel1, 0);
      stepper3->setSpeedInHz(homingSpeed3 * 2);
      stepper3->move(homingSpeed3 / -2);
      delay(1500);
      stepper3->setCurrentPosition(startPos3);
      Serial.print("current position: ");
      Serial.println(stepper3->getCurrentPosition());
      Serial.println("homed");
    }

    lastReading = reading;
  }
}
void testhoming(){
  Serial.println("testing");
  //stepper4->setSpeedInHz(homingSpeed4);
  //   stepper4->runForward();
  while(dirPin2 == 19){
    delay(250);
    Serial.print("limit 2: ");
    Serial.print(digitalRead(limit2));
    Serial.print("   hallSensor: ");
    Serial.println(analogRead(hallSensor));

   }

   Serial.println("end of test");
}

void homing4(){
  unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
  unsigned long debounceDelay = 50;
  homed4 = false;
  bool lastReading = 0;
  Serial.println("Homing 4");
  stepper4->setSpeedInHz(18000 / leadscrew);
  stepper4->move(-6 * homingSpeed4);
  ledcWrite(ledChannel1, 30);
  delay(1000);
  while(homed4 == false){
    bool reading = digitalRead(limit4);

    ledcWrite(ledChannel1, 60);
    stepper4->setSpeedInHz(homingSpeed4);
    stepper4->runForward();

    if (reading != lastReading){
      lastDebounceTime = millis();

    }
        if (((millis() - lastDebounceTime) > debounceDelay) && reading == true){
          homed4 = true; 
          stepper4->setSpeedInHz(homingSpeed4 * 2);
          //stepper4->move(homingSpeed4 * -2);
          delay(500);
          stepper4->stopMove();
          stepper4->setCurrentPosition(startPos4);
          Serial.println("homed 4");
        }

    lastReading = reading;
    }
}

void setup() {
  Serial.begin(115200);
  Serial.println(WiFi.macAddress());
  pinMode(ledPin1, OUTPUT);
  pinMode(limit2, INPUT_PULLDOWN);
  pinMode(limit4, INPUT_PULLDOWN);

  pinMode(hallSensor2, INPUT_PULLDOWN);
  pinMode(hallSensor, INPUT_PULLUP); 

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
  stepper4 = engine.stepperConnectToPin(stepPin4);

  if (stepper) 
   {
      stepper->setDirectionPin(dirPin);
      stepper->setAutoEnable(false);
      //stepper->setSpeedInHz(kiirus);
      stepper->setAcceleration(diskKiirendus); 
      
   }
     if (stepper2) 
   {
      stepper2->setDirectionPin(dirPin2);
      stepper2->setAutoEnable(false);
      //stepper2->setSpeedInHz(kiirus2); 
      stepper2->setAcceleration(24000); 
      
   } 
   if (stepper3) 
   {
      stepper3->setDirectionPin(dirPin3);
      stepper3->setAutoEnable(false);
      //stepper3->setSpeedInHz(kiirus3); 
      stepper3->setAcceleration(diskKiirendus2); 
      
   }
     if (stepper4) 
   {
      stepper4->setDirectionPin(dirPin4);
      stepper4->setAutoEnable(false);
      //stepper4->setSpeedInHz(18000 / leadscrew); 
      stepper4->setAcceleration(24000 / leadscrew); 
      
   }
  
  //testhoming();
  //homing1();
  homingKetas();
  homing2();
  homing4();
  homing3();
 
  
  Serial.println("end of homing");
  }


void loop() {

  ledcWrite(ledChannel1, dutyCycle1);
  /// DISK MOVEMENT
    if (positsioon == 3 && lastDiskState != positsioon){
      lastDiskState = positsioon;
      stepper->setSpeedInHz(kiirus);
      if (kiirus > 0){
        stepper->runForward();
      }
      else {
        stepper->stopMove();
      }
    }
    else if (positsioon == 3 && lastDiskState == positsioon){
      stepper->setSpeedInHz(kiirus);
      if (kiirus > 0){
        stepper->runForward();
      }
      else {
        stepper->stopMove();
      } 
    }

    else if ((positsioon == 0 || positsioon == 2) && lastDiskState != positsioon){
      stepper->stopMove();
    }

    else if (positsioon == 1 && lastDiskState != positsioon){
      lastDiskState = positsioon;
      stepper->setSpeedInHz(kiirus);
      if (kiirus > 0){
        stepper->runBackward();
      }
      else {
        stepper->stopMove();
      }    
      }
      else if (positsioon == 1 && lastDiskState == positsioon){
      stepper->setSpeedInHz(kiirus);
      if (kiirus > 0){
        stepper->runBackward();
      }
      else {
        stepper->stopMove();
      }       
      }
    else {
      
    }
    /// TUNING MOVEMENT
    if (stepper2) 
      {
      stepper2->setAutoEnable(false);
      stepper2->setSpeedInHz(20000);
      stepper2->moveTo(positsioon2);
      Serial.print("pos2: ");
      Serial.print(positsioon2);
      Serial.print(" Stepper2 pos: ");
      Serial.print(stepper2->getCurrentPosition());
      Serial.print(" kiirus: ");
      Serial.print(stepper2->getCurrentSpeedInMilliHz());
      Serial.print(" kiirendus ");
      Serial.println(stepper2->getCurrentAcceleration());
      
      }


    // SNARE MOVEMENT
    if (positsioon3 == 3 && lastDiskState2 != positsioon3){
      lastDiskState2 = positsioon3;
      stepper3->setSpeedInHz(kiirus3);
      if (kiirus3 > 0){
        stepper3->runForward();
      }
      else {
        stepper3->stopMove();
      }   
    }
    else if (positsioon3 == 3 && lastDiskState2 == positsioon3){
      stepper3->setSpeedInHz(kiirus3);
      if (kiirus3 > 0){
        stepper3->runForward();
      }
      else {
        stepper3->stopMove();
      }  ; 
    }

    else if ((positsioon3 == 0 || positsioon3 == 2) && lastDiskState2 != positsioon3){
      lastDiskState2 = positsioon3; 
      stepper3->stopMove();
      long int peatumisPaik = stepper3->getPositionAfterCommandsCompleted();

      if(stepper3->getCurrentSpeedInMilliHz() > 0){     
        int konstant = (stepper3->getCurrentSpeedInMilliHz()/1000)/2;
        int konstant2 = (konstant / 6400) * 3.4;
        stepper3->moveTo(peatumisPaik + (6400 - (peatumisPaik % 6400)) + konstant2 * 6400);    
      }
      if(stepper3->getCurrentSpeedInMilliHz() < 0){     
        int konstant = abs(stepper3->getCurrentSpeedInMilliHz()/1000)/2;
        int konstant2 = (konstant / 6400) * 3.4;
        stepper3->moveTo(peatumisPaik - (6400 + (peatumisPaik % 6400)) - konstant2 * 6400);    
      }  
    }  

    else if (positsioon3 == 1 && lastDiskState2 != positsioon3){
      lastDiskState2 = positsioon3;
      stepper3->setSpeedInHz(kiirus3);
 
      if (kiirus3 > 0){
      stepper3->runBackward();  
      }
      else {
        stepper3->stopMove();
      }    
    }
    else if (positsioon3 == 1 && lastDiskState2 == positsioon3){
      stepper3->setSpeedInHz(kiirus3);
      if (kiirus3 > 0){
      stepper3->runBackward();  
      }
      else {
        stepper3->stopMove();
      }      
    }

    //STRING DISTANCE
    if (stepper4)
      {
        stepper4->setAutoEnable(false);
        stepper4->setSpeedInHz(18000 / leadscrew);
        stepper4->moveTo(-positsioon4);
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