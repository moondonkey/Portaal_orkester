
#include <esp_now.h>
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <AsyncElegantOTA.h>
#include <ESPAsyncWebServer.h>
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
#define nupp 35
#define limit 34
#define tuli 25 
const int ledPin1 = 32; 

int maxKiirus = 32000;
int homingSpeed1 = 400;
int startPos1 = 0;
int ulatus = -140000;
bool liikumineState = false;
bool valgusState = false;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
int kiirus = 1;
int dutyCycle1 = 1;
String message = "";
String sliderValue1 = "1";
String sliderValue2 = String(dutyCycle1);
String sliderValue3 = String(kiirus);
String sliderValue4 = "4";


// setting PWM properties
const int freq = 5000;
const int ledChannel1 = 0; //peavalgus
const int resolution = 8;
const int ledChannel2 = 1; //nupu valgus


int dutyCycle = 0;

//Json Variable to Hold Slider Values
JSONVar sliderValues;

//Get Slider Values

String getSliderValuesStart(){
  sliderValues["sliderValue1"] = String(sliderValue1);
  sliderValues["sliderValue2"] = String(preferences.getInt("kiirus", 1));
  sliderValues["sliderValue3"] = String(preferences.getInt("heledus", 1));
  sliderValues["sliderValue4"] = String(preferences.getInt("valgusState", 1));
  String jsonString = JSON.stringify(sliderValues);
  return jsonString;
}

String getSliderValues(){
  sliderValues["sliderValue1"] = String(sliderValue1);
  sliderValues["sliderValue2"] = String(sliderValue2);
  sliderValues["sliderValue3"] = String(sliderValue3);
  sliderValues["sliderValue4"] = String(sliderValue4);
  String jsonString = JSON.stringify(sliderValues);
  return jsonString;
}
void liikumine(void);

void homing(){
  Serial.println("homing");
  while(analogRead(limit) <= 4090) {
    ledcWrite(ledChannel1, 125);
    stepper->setSpeedInHz(homingSpeed1);
    stepper->runForward();
  }
  Serial.println("homing1");
  ledcWrite(ledChannel1, 0);
  ledcWrite(ledChannel2, 0);
  stepper->setSpeedInHz(homingSpeed1 * 8);
  stepper->move(homingSpeed1 * -2);
  delay(1000);
  while(analogRead(limit) <= 4090) {
   
    stepper->setSpeedInHz(homingSpeed1/2);
    stepper->runForward();
  }
    stepper->stopMove();
    delay(200);
    stepper->move(-400);
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
    if (message== "1s100") {
      //sliderValue1 = message.substring(2);
      liikumineState = true;
      //notifyClients(getSliderValues());
    }
    if (message.indexOf("2s") >= 0) {
      sliderValue2 = message.substring(2);
      dutyCycle1 = map(sliderValue2.toInt(), 1, 100, 1, 255);
      preferences.putInt("heledus", dutyCycle1);
      Serial.print(getSliderValues());
      
      notifyClients(getSliderValues());
    }    
    if (message.indexOf("3s") >= 0) {
      sliderValue3 = message.substring(2);
      kiirus = map(sliderValue3.toInt(), 1, 100, maxKiirus/100, maxKiirus);
      preferences.putInt("kiirus", kiirus);
      Serial.print(getSliderValues());
      notifyClients(getSliderValues());
    }
    if (message.indexOf("4s") >= 0) {
      sliderValue4 = message.substring(2);
      if(sliderValue4.toInt() == 1){
        valgusState = false;
        preferences.putInt("valgusState", 1);
      }
      else{
        valgusState = true;
        preferences.putInt("valgusState", 2);
      }
      
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
      ledcWrite(ledChannel2, 0);
      
      stepper->setAutoEnable(false);
      stepper->setSpeedInHz(kiirus);
      stepper->moveTo(ulatus);
      while(stepper->targetPos() != stepper->getCurrentPosition()){
        ledcWrite(ledChannel1, dutyCycle1);
        stepper->setSpeedInHz(kiirus);
      }
      stepper->moveTo(0);
      while(stepper->targetPos() != stepper->getCurrentPosition()){
        ledcWrite(ledChannel1, dutyCycle1);
        stepper->setSpeedInHz(kiirus);
      }
      
      ledcWrite(ledChannel1,0);
      liikumineState = false;
}
void setup() {
  Serial.begin(115200);
  pinMode(ledPin1, OUTPUT);
  pinMode(tuli, OUTPUT);
  pinMode(limit, INPUT);
  pinMode(nupp, INPUT);
  
  preferences.begin("my-app", false);
  kiirus = preferences.getInt("kiirus", 1);
  dutyCycle1 = preferences.getInt("heledus", 1);
  if (preferences.getInt("valgusState", 1) == 1){
    valgusState = false;
  }
  else if (preferences.getInt("valgusState", 1) == 2){
    valgusState = true;
  }
  notifyClients(getSliderValuesStart());

   initFS();
   initWiFi();
   AsyncElegantOTA.begin(&server);  

  // configure LED PWM functionalitites
  ledcSetup(ledChannel1, freq, resolution);
  ledcSetup(ledChannel2, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ledPin1, ledChannel1);
  ledcAttachPin(tuli,ledChannel2);

  ledcWrite(ledChannel2, 0);

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
      stepper->setAcceleration(8000); 
      
   }
  
  homing();

  }


void loop() {
    ledcWrite(ledChannel2, 255);
    if (valgusState == true){
      ledcWrite(ledChannel1, dutyCycle1);
    }
    else if(valgusState == false){
      ledcWrite(ledChannel1, 0);
    }



  if (digitalRead(nupp) == HIGH && liikumineState == false)
  {
    liikumineState = true;
  }
    

  if (liikumineState == true){
    liikumine();
  }
}