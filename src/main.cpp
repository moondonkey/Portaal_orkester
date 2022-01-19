/* 
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-web-server-websocket-sliders/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
//#include <AsyncElegantOTA.h>
#include "SPIFFS.h"
#include <Arduino_JSON.h>
#include "FastAccelStepper.h"
#include <ESP32Servo.h>

// Replace with your network credentials
const char* ssid = "ladu";
const char* password = "laduladu";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create a WebSocket object
AsyncWebSocket ws("/ws");

// Set LED GPIO
const int ledPin1 = 32;

Servo myservo;
int minUs = 1000;
int maxUs = 2000;
ESP32PWM pwm;

// #define dirPin 22
// #define stepPin 23
// #define dirPin2 19
// #define stepPin2 18
#define dirPin 17
#define stepPin 16
#define dirPin2 23
#define stepPin2 22
#define servoPin 9
#define hallSensor 33
#define limit2 25

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
FastAccelStepper *stepper2 = NULL;

String message = "";
String sliderValue1 = "0";
String sliderValue2 = "2";
String sliderValue3 = "0";
String sliderValue4 = "6000";
String sliderValue5 = "0";
String sliderValue6 = "0";
String sliderValue7 = "0";

int dutyCycle1;
int positsioon = 2;
int positsioon2 = 2;
int fookus = 0;
int kiirus = 0;
int kiirus2 = 0;
int laeng = 0;
int homed = false;


// setting PWM properties
const int freq = 5000;
const int ledChannel1 = 0;
const int resolution = 8;

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

void magnet(){
  pinMode(hallSensor, INPUT);
  laeng = digitalRead(hallSensor);
  Serial.println(laeng);
  
  }

void homing(){
  Serial.println("homing");
 
  // Move motor until home position reached;
  
  
  while(analogRead(hallSensor) >= 1050) {
    stepper->setSpeedInHz(40);
    stepper->runForward();
  }
    stepper->stopMove();
    delay(200);
    stepper->setCurrentPosition(0);
    Serial.print("homed ");
    Serial.println(stepper->getCurrentPosition());
    stepper->setSpeedInHz(0);
}

void homing2(){
   pinMode(limit2, INPUT_PULLUP);
   Serial.println("homing2");

    while(digitalRead(limit2)){
      stepper2->setSpeedInHz(80);
      stepper2->runForward();
    }
    stepper2->stopMove();
    delay(200);
    stepper2->setCurrentPosition(6010);
    Serial.print("homed2 ");
    Serial.println(stepper2->getCurrentPosition());
    stepper2->setSpeedInHz(0);

}
// Initialize SPIFFS
void initFS() {
  if (!SPIFFS.begin()) {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  else{
   Serial.println("SPIFFS mounted successfully");
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
      // Serial.println(dutyCycle1);
      // Serial.print(getSliderValues());
      notifyClients(getSliderValues());
    }
    if (message.indexOf("2s") >= 0) {
      sliderValue2 = message.substring(2);
      positsioon = map(sliderValue2.toInt(), 1, 5, 1, 5);
      // Serial.print("Suund");
      // Serial.println(positsioon);
      Serial.print(getSliderValues());
      
      notifyClients(getSliderValues());
    }    
    if (message.indexOf("3s") >= 0) {
      sliderValue3 = message.substring(2);
      kiirus = map(sliderValue3.toInt(), 0, 8000, 0, 8000);
      // Serial.print("kiirus1");
      // Serial.println(kiirus);
      Serial.print(getSliderValues());
      notifyClients(getSliderValues());
    }
     if (message.indexOf("4s") >= 0) {
      sliderValue4 = message.substring(2);
      positsioon2 = map(sliderValue4.toInt(),1, 6000, 1, 6000);
      // Serial.print("kiirus1");
      // Serial.println(kiirus);
      Serial.print(getSliderValues());
      notifyClients(getSliderValues());
    }
    if (message.indexOf("5s") >= 0) {
      sliderValue5 = message.substring(2);
      kiirus2 = map(sliderValue5.toInt(), 0, 8000, 0, 8000);
      // Serial.print("kiirus1");
      // Serial.println(kiirus);
      Serial.print(getSliderValues());
      notifyClients(getSliderValues());
    }
     if (message.indexOf("6s") >= 0) {
      sliderValue6 = message.substring(2);
      fookus = map(sliderValue6.toInt(), 1, 180, 0, 180);
      // Serial.print("kiirus1");
      // Serial.println(kiirus);
      Serial.print(getSliderValues());
      notifyClients(getSliderValues());
    }
    if (message.indexOf("7s") >= 0) {
      sliderValue7 = message.substring(2);
      Serial.println(sliderValue7);
      if (sliderValue7.toInt() == 1){
        Serial.println("nupp");
      homing();
      }
      
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

  Serial.begin(115200);
  pinMode(ledPin1, OUTPUT);
  pinMode(hallSensor, INPUT);

  initFS();
  initWiFi();
  

  //Servo
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);
  myservo.attach(servoPin, minUs, maxUs);
  
  // configure LED PWM functionalitites
  ledcSetup(ledChannel1, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ledPin1, ledChannel1);

  initWebSocket();
  
  // Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html");
  });
  
  server.serveStatic("/", SPIFFS, "/");

  // Start server
  server.begin();

  engine.init();
  

   stepper = engine.stepperConnectToPin(stepPin);
   stepper2 = engine.stepperConnectToPin(stepPin2);

  if (stepper) 
   {
      stepper->setDirectionPin(dirPin);
      stepper->setAutoEnable(false);
      stepper->setSpeedInHz(kiirus);       // 500 steps/s
      stepper->setAcceleration(200);    // 100 steps/s²
      
   }
     if (stepper2) 
   {
      stepper2->setDirectionPin(dirPin2);
      stepper2->setAutoEnable(false);
      stepper2->setSpeedInHz(kiirus2);       // 500 steps/s
      stepper2->setAcceleration(2000);    // 100 steps/s²
      
   }
  homing();
  homing2();
}

void loop() {
  ledcWrite(ledChannel1, dutyCycle1);
  

      if (positsioon == 3){
        stepper->setSpeedInHz(kiirus);
        stepper->runForward();   
      }
      else if (positsioon == 2){
        stepper->stopMove();
      }
      else if (positsioon == 1){
        stepper->setSpeedInHz(kiirus);
        stepper->runBackward();

      }
      else if (positsioon == 4){
        homing();

      }
            else if (positsioon == 5){
        stepper->setSpeedInHz(kiirus);
        int long asukoht = stepper->getCurrentPosition();
        stepper->moveTo(asukoht / 800);

      }
      else{
        stepper->stopMove();
      }
       if (stepper2) 
   {
      
      stepper2->setAutoEnable(false);
      stepper2->setSpeedInHz(kiirus2);       // 500 steps/s
      
      stepper2->moveTo(positsioon2);
   }
 
  //magnet();
  ws.cleanupClients();
}