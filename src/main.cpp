/* 
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-web-server-websocket-sliders/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/
#include <esp_now.h>
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
//#include <AsyncElegantOTA.h>
#include "SPIFFS.h"
#include <Arduino_JSON.h>
#include "FastAccelStepper.h"
#include <WebSerial.h>

// Replace with your network credentials
const char* ssid = "ladu";
const char* password = "laduladu";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create a WebSocket object
AsyncWebSocket ws("/ws");

#define dirPin 17 //17
#define stepPin 16 //16
#define dirPin2 19
#define stepPin2 18
#define dirPin3 22 //19
#define stepPin3 21 //1 - muutsin ära, et saaks Seriali kasutada.
#define hallSensor 33
#define hallSensor2 34
#define limit2 35
const int ledPin1 = 12; //32

int tuningRange = 53000;
int homingSpeed1 = 640;
int homingSpeed2 = 2400;
int homingSpeed3 = 240;
int startPos1 = 0;
int startPos2 = tuningRange + 50;
int startPos3 = 0;

//staatus, et salvestada ketta olek
int lastDiskState = 0;


FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
FastAccelStepper *stepper2 = NULL;
FastAccelStepper *stepper3 = NULL;

String message = "";
String sliderValue1 = "0";
String sliderValue2 = "2";
String sliderValue3 = "0";
String sliderValue4 = "6000";
String sliderValue5 = "0";
String sliderValue6 = "1";
String sliderValue7 = "2";
String sliderValue8 = "0";
String sliderValue9 = "0";

int dutyCycle1;
int positsioon = 2;
int positsioon2 = 65000;
int positsioon3 = 2;
int kiirus = 0;
int diskKiirendus = 24000;
int kiirus2 = 0;
int kiirus3 = 0;
int laeng = 0;
int homed = false;

// setting PWM properties
const int freq = 5000;
const int ledChannel1 = 0;
const int resolution = 8;
int dutyCycle = 0;

int getData[7];

void recvMsg(uint8_t *data, size_t len){
  WebSerial.println("Received Data...");
  String d = "";
  for(int i=0; i < len; i++){
    d += char(data[i]);
  }
  WebSerial.println(d);
}

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

//Json Variable to Hold Slider Values
JSONVar sliderValues;

//Get Slider Values
String getSliderValues(){
  sliderValues["sliderValue1"] = String(sliderValue1);
  sliderValues["sliderValue2"] = String(sliderValue2);
  sliderValues["sliderValue3"] = String(sliderValue3);
  sliderValues["sliderValue4"] = String(sliderValue4);
  sliderValues["sliderValue5"] = String(sliderValue5);
  sliderValues["sliderValue6"] = String(sliderValue6);
  sliderValues["sliderValue7"] = String(sliderValue7);
  String jsonString = JSON.stringify(sliderValues);
  return jsonString;
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

// Initialize SPIFFS
void initFS() {
  if (!SPIFFS.begin()) {
    //Serial.println("An error has occurred while mounting SPIFFS");
  }
  else{
   //Serial.println("SPIFFS mounted successfully");
  }
}

// Initialize WiFi
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.macAddress());
}

void notifyClients(String sliderValues) {
  ws.textAll(sliderValues);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    message = (char*)data;
    Serial.println(message);
    if (message.indexOf("1s") >= 0) {
      sliderValue1 = message.substring(2);
      dutyCycle1 = map(sliderValue1.toInt(), 0, 100, 0, 255);
      Serial.print("lamp");
      notifyClients(getSliderValues());
    }
    if (message.indexOf("2s") >= 0) {
      sliderValue2 = message.substring(2);
      positsioon = map(sliderValue2.toInt(), 1, 5, 1, 5);
      Serial.print(getSliderValues());
      
      notifyClients(getSliderValues());
    }    
    if (message.indexOf("3s") >= 0) {
      sliderValue3 = message.substring(2);
      kiirus = map(sliderValue3.toInt(), 0, 8000, 0, 16000);
      Serial.print(getSliderValues());
      notifyClients(getSliderValues());
    }
     if (message.indexOf("4s") >= 0) {
      sliderValue4 = message.substring(2);
      positsioon2 = map(sliderValue4.toInt(),1, 6000, 1, startPos2-10);
      Serial.print(getSliderValues());
      notifyClients(getSliderValues());
    }
    if (message.indexOf("5s") >= 0) {
      sliderValue5 = message.substring(2);
      kiirus2 = map(sliderValue5.toInt(), 0, 8000, 0, 8000);
      Serial.print(getSliderValues());
      notifyClients(getSliderValues());
    }
     if (message.indexOf("6s") >= 0) {
      sliderValue6 = message.substring(2);
      
      Serial.print(getSliderValues());
      notifyClients(getSliderValues());
    }
    if (message.indexOf("7s") >= 0) {
      sliderValue7 = message.substring(2);
      positsioon3 = map(sliderValue7.toInt(), 1, 4, 1, 4);
      Serial.print(getSliderValues());
      notifyClients(getSliderValues());
      }
    if (message.indexOf("8s") >= 0) {
      sliderValue8 = message.substring(2);
      kiirus3 = map(sliderValue8.toInt(), 0, 900, 0, 8000);
      Serial.print(getSliderValues());
      notifyClients(getSliderValues());
      }  
    
   
    if (strcmp((char*)data, "getValues") == 0) {
      notifyClients(getSliderValues());
    }
  
}
}
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}


void setup() {
  // Start ElegantOTA
  //AsyncElegantOTA.begin(&server);    

  //Serial.begin(115200);
  pinMode(ledPin1, OUTPUT);
  pinMode(limit2, INPUT);
  pinMode(hallSensor, INPUT);
  pinMode(hallSensor2, INPUT);

  WiFi.mode(WIFI_STA);
  //Serial.println(WiFi.macAddress());
  // initFS();
  //initWiFi();

  //WebSerial.begin(&server);
  //WebSerial.msgCallback(recvMsg);
  
  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
 esp_now_register_recv_cb(OnDataRecv);
  
  // configure LED PWM functionalitites
  ledcSetup(ledChannel1, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ledPin1, ledChannel1);

  // initWebSocket();
  
  // //Web Server Root URL
  // server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
  //   request->send(SPIFFS, "/index.html", "text/html");
  // });
  
  // server.serveStatic("/", SPIFFS, "/");

  // //Start server
   //server.begin();

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
  //homing2();
  //homing3();
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
        // Serial.println(); 
        // Serial.print("minAbsSeiskumispunkt % 6400: ");
        // Serial.println(minAbsSeiskumispunkt % 6400);
        // Serial.print("abs(peatumisvahemaa): ");
        // Serial.println(abs(peatumisVahemaa));
        // Serial.print("hetkKiirus: ");
        // Serial.println(hetkKiirus);
        // Serial.print("move: ");
        Serial.println((6400 - (minAbsSeiskumispunkt % 6400) + abs(peatumisVahemaa) - (hetkKiirus / 50)) * -1);
        stepper->move(6400 - (minAbsSeiskumispunkt % 6400) + abs(peatumisVahemaa) - (hetkKiirus / 50));
       }
       else{
        int minAbsSeiskumispunkt = (-asukoht % 6400) - (peatumisVahemaa);
        // Serial.println(); 
        // Serial.print("minAbsSeiskumispunkt % 6400: ");
        // Serial.println(minAbsSeiskumispunkt % 6400);
        // Serial.print("abs(peatumisvahemaa): ");
        // Serial.println(abs(peatumisVahemaa));
        // Serial.print("hetkKiirus: ");
        // Serial.println(hetkKiirus);
        // Serial.print("move: ");
        Serial.println((6400 - (minAbsSeiskumispunkt % 6400) + abs(peatumisVahemaa) - (hetkKiirus / 50)) * -1);
        stepper->move((6400 - (minAbsSeiskumispunkt % 6400) - abs(peatumisVahemaa) + (hetkKiirus / 50)) * -1);
       }
        
      //  }
      //   else {
      //     stepper->move((6400 - (minAbsSeiskumispunkt % 6400) + (peatumisVahemaa) - (hetkKiirus / 50)) * -1);
      //   }
       

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
        //stepper->setSpeedInHz(0);
        stepper3->stopMove();     
      }
      else if (positsioon3 == 1){
        stepper3->setSpeedInHz(kiirus3);
        stepper3->runBackward();
        
      }


}