
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
#include <ESPmDNS.h>


Preferences preferences;

// Replace with your network credentials
const char* ssid = "valgus";

/* Put IP Address details */
IPAddress local_ip(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);


// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create a WebSocket object
AsyncWebSocket ws("/ws");


#define dirPin 17
#define stepPin 16
#define nupp 19
#define limit 18
#define tuli 35 //35 on input only!
const int ledPin1 = 32;

int homingSpeed1 = 80;
int startPos1 = 6000;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

String message = "";
String sliderValue1 = "0";
String sliderValue2 = "2";
String sliderValue3 = "0";
int kiirus;
int dutyCycle1;

// setting PWM properties
const int freq = 5000;
const int ledChannel1 = 0;
const int resolution = 8;

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

//Json Variable to Hold Slider Values
JSONVar sliderValues;

//Get Slider Values
String getSliderValues(){
  sliderValues["sliderValue1"] = String(sliderValue1);
  sliderValues["sliderValue2"] = String(sliderValue2);
  sliderValues["sliderValue3"] = String(sliderValue3);
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
  while(analogRead(limit) > 2500) {
   
    stepper->setSpeedInHz(homingSpeed1/2);
    stepper->runForward();
  }
    stepper->stopMove();
    delay(200);
    stepper->setCurrentPosition(startPos1);
    Serial.print("homed ");
    Serial.println(stepper->getCurrentPosition());
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

  WiFi.softAP(ssid);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  
  if(!MDNS.begin("valgus")) {
   return;
}
  Serial.print("Connecting to WiFi ..");
  // while (WiFi.status() != WL_CONNECTED) {
  //   Serial.print('.');
  //   delay(1000);
  // }
  Serial.println(WiFi.localIP());
  
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
      dutyCycle1 = map(sliderValue2.toInt(), 1, 100, 1, 255);
      Serial.print(getSliderValues());
      
      notifyClients(getSliderValues());
    }    
    if (message.indexOf("3s") >= 0) {
      sliderValue3 = message.substring(2);
      kiirus = map(sliderValue3.toInt(), 1, 8000, 1, 8000);
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

void liikumine(){

      ledcWrite(ledChannel1, dutyCycle1);
 
      stepper->setAutoEnable(false);
      stepper->setSpeedInHz(kiirus);
      stepper->moveTo(6000);
      delay(100);
      stepper->moveTo(0);

      ledcWrite(ledChannel1, 0);
   


}
void setup() {
  // Start ElegantOTA
  //AsyncElegantOTA.begin(&server);    

  Serial.begin(115200);
  pinMode(ledPin1, OUTPUT);
  pinMode(limit, INPUT);
  pinMode(nupp, INPUT);
  

  preferences.begin("my-app", false);

   initFS();
   initWiFi();

  // configure LED PWM functionalitites
  ledcSetup(ledChannel1, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ledPin1, ledChannel1);


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
  
      


}