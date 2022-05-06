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
#include <Preferences.h>

Preferences preferences;

// Replace with your network credentials
const char* ssid = "ladu";
const char* password = "laduladu";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create a WebSocket object
AsyncWebSocket ws("/ws");

#define dirPin 17
#define stepPin 16
#define button 34
#define limit 35
const int ledPin1 = 32;

int homingSpeed1 = 80;
int startPos1 = 6000;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

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
int positsioon2 = 6000;
int positsioon3 = 2;
int fookus = 9;
int kiirus = 0;
int kiirus2 = 0;
int kiirus3 = 0;
int laeng = 0;
int homed = false;

// setting PWM properties
const int freq = 5000;
const int ledChannel1 = 0;
const int resolution = 8;

const int PWMFreq2 = 50;
const int PWMChannel2 = 2;
const int PWMResolution2 = 8;
int dutyCycle = 0;

typedef struct test_struct{
  int pwm;
  int kiirus;
  int suund;
  int positsioon;
  int kiirus2;
  int suund3;
  int kiirus3;
} test_struct;

int getData[7];
test_struct myData;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&getData, incomingData, sizeof(getData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  
  dutyCycle1 =getData[0];
  kiirus = map(getData[1], 0, 255, 0, 16000);
  positsioon =  map(getData[2], 0, 255, 0, 3);
  positsioon2 =  map(getData[3], 0, 255, 6000, 1);
  kiirus2 =  map(getData[4], 0, 255, 0, 2000);
  positsioon3 =  map(getData[5], 0, 255, 0, 3);
  kiirus3 = map(getData[6], 0, 255, 0, 4000);
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

void homing(){
  Serial.println("homing");
  while(analogRead(limit) <= 2500) {
   
    stepper->setSpeedInHz(homingSpeed1);
    stepper->runForward();
  }
  stepper->setSpeedInHz(homingSpeed1 * 4);
  stepper->move(homingSpeed1 * -2);
  delay(1000);
  while(analogRead(hallSensor) <= 1050) {
   
    stepper->setSpeedInHz(homingSpeed1/2);
    stepper->runForward();
  }
    stepper->stopMove();
    delay(200);
    stepper->setCurrentPosition(startPos1);
    Serial.print("homed ");
    Serial.println(stepper->getCurrentPosition());
    kiirus = 0;
    stepper->setSpeedInHz(0);
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
      fookus = map(sliderValue6.toInt(), 1, 180, 9, 16);
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

  Serial.begin(115200);
  pinMode(ledPin1, OUTPUT);
  pinMode(limit, INPUT);
  pinMode(nupp, INPUT);
  

  preferences.begin("my-app", false);

  WiFi.mode(WIFI_STA);
   initFS();
   initWiFi();

  // configure LED PWM functionalitites
  ledcSetup(ledChannel1, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ledPin1, ledChannel1);

  ledcSetup(PWMChannel2, PWMFreq2, PWMResolution2);
  /* Attach the LED PWM Channel to the GPIO Pin */
  ledcAttachPin(servoPin, PWMChannel2);
  ledcWrite(PWMChannel2, 8);


  initWebSocket();
  
  //Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html");
  });
  
  server.serveStatic("/", SPIFFS, "/");

  //Start server
  server.begin();

  engine.init();
   stepper = engine.stepperConnectToPin(stepPin);

  if (stepper) 
   {
      stepper->setDirectionPin(dirPin);
      stepper->setAutoEnable(false);
      stepper->setSpeedInHz(kiirus);
      stepper->setAcceleration(1500); 
      
   }

  homing();

  }


void loop() {
  ledcWrite(ledChannel1, dutyCycle1);
 
       if (stepper) 
   {
      
      stepper->setAutoEnable(false);
      stepper->setSpeedInHz(kiirus2);
      stepper->moveTo(positsioon2);
   }


}