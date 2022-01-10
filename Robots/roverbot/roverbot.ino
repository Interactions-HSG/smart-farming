#include <M5StickC.h>
//#include <M5StickCPlus.h>
#include "RoverC.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <ESPAsyncWebServer.h>
#include "AsyncJson.h"
#include "ArduinoJson.h"
#include "TD.h"

constexpr char WIFI_SSID[] = "JANGIRI";
constexpr char WIFI_PASS[] = "desigowlan";

long totalDuration = 0;
float batvol = 11.1;
float temperature = 28.0;
boolean wheelMoving =false;
long startedMoving = 0;
int requestedDuration = 0;
long lastPmuRead = 0;

float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

float pitch = 0.0F;
float roll  = 0.0F;
float yaw   = 0.0F;
float batVoltage = 0.0F;

StaticJsonDocument<500> imuDataDoc;
StaticJsonDocument<500> pmuDataDoc;

AsyncWebServer server(80);

//-------------------------
void initWiFi();
void initHttpServer();
void doPeriodicTask();
void doWheelControl(const JsonVariant &input);

// the setup routine runs once when M5StickC starts up
void setup(){ 
  // Initialize the M5StickC object
  M5.begin();
  
  M5.Lcd.setRotation(3);
    // Lcd display
  M5.Lcd.fillScreen(WHITE);
  delay(500);
  M5.Lcd.fillScreen(RED);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setCursor(40, 10, 4);
  M5.Lcd.printf("Rover");

  RoverC_Init();
  Move_stop(0);
  Move_servo1(70);
  M5.IMU.Init();
  M5.Axp.EnableCoulombcounter();
  initWiFi();
  initHttpServer();
}

//-----------------------------------------------------------
void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}

void initWiFi() {
    WiFi.mode(WIFI_MODE_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    Serial.printf("Connecting to %s .", WIFI_SSID);
    while (WiFi.status() != WL_CONNECTED) { Serial.print("."); delay(200); }
    Serial.println(" ok");

    IPAddress ip = WiFi.localIP();

    M5.Lcd.setCursor(5, 40, 4);
    M5.Lcd.printf("%u.%u.%u.%u\n", ip & 0xff, (ip >> 8) & 0xff, (ip >> 16) & 0xff, ip >> 24);

    Serial.printf("SSID: %s\n", WIFI_SSID);
    Serial.printf("Channel: %u\n", WiFi.channel());
    Serial.printf("IP: %u.%u.%u.%u\n", ip & 0xff, (ip >> 8) & 0xff, (ip >> 16) & 0xff, ip >> 24); 
}

// the loop routine runs over and over again forever
void loop() {
/*
  Move_forward(100);
  delay(2000);
  Move_back(100);
  delay(2000);
  Move_turnleft(100);
  delay(2000);
  Move_turnright(100);
  
  delay(2000);
  Move_servo1(0);
  delay(2000);
  Move_servo1(90);*/
  doPeriodicTask();
  delay(10);
}

void readImu(){
  M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
  M5.IMU.getAccelData(&accX,&accY,&accZ);
  M5.IMU.getAhrsData(&pitch,&roll,&yaw);
  M5.IMU.getTempData(&temperature);
}

void readPmu(){
  batVoltage = M5.Axp.GetBatVoltage();
}

void doPeriodicTask(){
  batvol = batvol - 0.00001;

  if(wheelMoving && (millis() - startedMoving) > requestedDuration){
    Move_forward(0);
    wheelMoving = false;
  }

  readImu();
  readPmu();
}

void initHttpServer(){

  server.on("/td", HTTP_GET, [] (AsyncWebServerRequest *request) {
      request->send(200, "application/json", td);
  });
    
  server.on("/things/roverbot/properties/imu", HTTP_GET, [] (AsyncWebServerRequest *request) {
      imuDataDoc["accX"] = accX;
      imuDataDoc["accY"] = accY;
      imuDataDoc["accZ"] = accZ;
      String output;
      serializeJson(imuDataDoc, output);
      request->send(200, "application/json", output);
  });

  server.on("/things/roverbot/properties/pmu", HTTP_GET, [] (AsyncWebServerRequest *request) {
      //pmuDataDoc["temperature"] = M5.Axp.GetTempInAXP192();
      pmuDataDoc["batVoltage"] = batVoltage;
      //pmuDataDoc["batCurrent"] = M5.Axp.GetBatCurrent();
      //pmuDataDoc["vbusVoltage"] = M5.Axp.GetVBusVoltage();
      //pmuDataDoc["vbusCurrent"] = M5.Axp.GetVBusCurrent();
      //pmuDataDoc["vinVoltage"] = M5.Axp.GetVinVoltage();
      //pmuDataDoc["vinCurrent"] = M5.Axp.GetVinCurrent();      
      //pmuDataDoc["batPower"] = M5.Axp.GetBatPower();
      String output;
      serializeJson(pmuDataDoc, output);
      request->send(200, "application/json", output);
  });

  server.on("/things/roverbot/properties/batteryVoltage", HTTP_GET, [] (AsyncWebServerRequest *request) {
   
      request->send(200, "application/json", String(batvol));
  });

  server.on("/things/roverbot/properties/motorTemperature", HTTP_GET, [] (AsyncWebServerRequest *request) {
   
      request->send(200, "application/json", String(temperature));
  });  
  
  AsyncCallbackJsonWebHandler *wHandler = new AsyncCallbackJsonWebHandler("/things/roverbot/actions/wheelControl", [](AsyncWebServerRequest *request, JsonVariant &json) {
    StaticJsonDocument<200> data;
    if (json.is<JsonArray>())
    {
      data = json.as<JsonArray>();
    }
    else if (json.is<JsonObject>())
    {
      data = json.as<JsonObject>();
      doWheelControl(json);
    }
    String response;
    serializeJson(data, response);
    request->send(200, "application/json", response);
    Serial.println(response);
  });
  server.addHandler(wHandler);   

  AsyncCallbackJsonWebHandler *gHandler = new AsyncCallbackJsonWebHandler("/things/roverbot/actions/grabberControl", [](AsyncWebServerRequest *request, JsonVariant &json) {
    StaticJsonDocument<200> data;
    if (json.is<JsonObject>())
    {
      data = json.as<JsonObject>();
      doGrabberControl(json);
    }
    String response;
    serializeJson(data, response);
    request->send(200, "application/json", response);
    Serial.println(response);
  });
  
  server.addHandler(gHandler);     
  server.begin();  
}

void doGrabberControl(const JsonVariant &input) {
  JsonObject inputObj = input.as<JsonObject>();
  int angle = inputObj["angle"];
  Serial.printf("Grabber: %d\n", angle);
  Move_servo1(angle);
}

void doWheelControl(const JsonVariant &input) {
  JsonObject inputObj = input.as<JsonObject>();
  int axis = inputObj["axis"];
  int speed = inputObj["speed"];
  int duration = inputObj["duration"];
  Serial.print("ControlWheel(");Serial.print(axis);Serial.print(",");Serial.print(speed);Serial.print(",");Serial.print(duration);Serial.println(")");
  int wa = 0, wb = 0, wc = 0, wd = 0;
  if(axis == 0)
  { //X-axis
    Move_forward(speed);
  }else if(axis == 1)
  { //Y-axis
    Move_right(speed);
  }else if(axis == 2)
  { //Z-axis
    Move_turnright(speed);
  }

  if(duration > 0){
    requestedDuration = duration;
    wheelMoving = true;
    startedMoving = millis();
  }else{
    wheelMoving = false;
  }
  
  temperature = temperature + (duration/1000);
  batvol = batvol - (duration * .00001);
}
