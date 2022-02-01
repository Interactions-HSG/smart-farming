#define LARGE_JSON_BUFFERS 1
#define USE_COLOR_SENSOR true
#define ROBOT_NAME "SPOCK"
#define ROBOT_COLOR RED
#define UPDATE_INTERVAL_MS 10

#define PIN_BAT_VOLTS 35
#define PIN_BAT_AMPS 36
#define PIN_LIDAR_ENABLE 5
#define PIN_TEMPERATURE_SENSOR 19

#include<M5Stack.h>
//#include "sdk/include/angles.h"
#include "doProcess.h"
#include "mapData.h"
#include "X2driver.h"
//#include "lidarcar.h"
#include "lock.h"
//#include "espHttpServer.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <ESPAsyncWebServer.h>
#include "AsyncJson.h"
#include "ArduinoJson.h"
#include "TD.h"
#include "Robot.h"

#if USE_COLOR_SENSOR
#include "Adafruit_TCS34725.h"
#endif


constexpr char WIFI_SSID[] = "labnet";
constexpr char WIFI_PASS[] = "inthrustwetrust";

long totalDuration = 0;
float temperature = 28.0;
float lastLidarStash[720] = { 0.0 };
String lidarData = "[]";
long lastUpdate = 0;
String test = "[0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,3450.00,0.00,0.00,3428.00,0.00,0.00,0.00,3414.00,0.00,0.00,3403.00,0.00,0.00,3387.00,0.00,0.00,3382.00,0.00,0.00,0.00,3378.00,0.00,0.00,3371.00,0.00,0.00,3369.00,0.00,0.00,3368.00,0.00,0.00,0.00,3372.00,0.00,0.00,3377.00,0.00,0.00,3386.00,0.00,0.00,3397.00,0.00,0.00,3364.00,0.00,0.00,0.00,3402.00,0.00,0.00,3439.00,0.00,0.00,3476.00,0.00,0.00,3496.00,0.00,0.00,0.00,3517.00,0.00,0.00,3555.00,0.00,0.00,3593.00,0.00,0.00,3627.00,0.00,0.00,3679.00,0.00,0.00,0.00,3719.00,0.00,0.00,3777.00,0.00,0.00,3834.00,0.00,0.00,3897.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,522.00,0.00,726.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,625.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,577.00,580.00,0.00,585.00,597.00,0.00,625.00,636.00,0.00,664.00,676.00,0.00,718.00,741.00,0.00,0.00,3064.00,3046.00,0.00,3049.00,3006.00,0.00,3033.00,2966.00,0.00,2957.00,2970.00,0.00,0.00,2988.00,2982.00,0.00,2971.00,2951.00,0.00,2967.00,2950.00,0.00,2963.00,2950.00,0.00,2947.00,2907.00,0.00,0.00,2879.00,2870.00,0.00,2966.00,2952.00,0.00,2917.00,2935.00,0.00,2869.00,2918.00,0.00,0.00,2901.00,2963.00,0.00,2893.00,2904.00,0.00,2977.00,2980.00,0.00,2967.00,2981.00,0.00,2958.00,2982.00,0.00,0.00,2936.00,3011.00,0.00,3025.00,3021.00,0.00,3030.00,3023.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00]";
float batvol = 11.1;

boolean wheelMoving = false;
long startedMoving = 0;
int requestedDuration = 0;

boolean irrigating = false;
long startedIrrigation = 0;
int requestedIrrigationDuration = 0;
int waterLevel = 10;

//-------------------------- objects ------------------------
X2 lidar;
//LidarCar lidarcar;
Robot robot;
#if USE_COLOR_SENSOR
Adafruit_TCS34725 colorsensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
#endif
AsyncWebServer server(80);

//-------------------------- methods -----------------------
void initWiFi();
void initLcd();
void displayMap();
void initSerialPorts();
#if USE_COLOR_SENSOR
void initColorSensor();
#endif
void initServer();
//static void dis_task(void *arg);
void createLidarData();
void doWheelControl(const JsonVariant &input);
void doIrrigation(const JsonVariant &input);
//void wheelCommand(int wa, int wb, int wc, int wd);
//void ledCommand(int pos, int r, int g, int b);

//-----------------------------------------------------------
void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}

void setup() {
  M5.begin(true, false, false);
  robot.begin();
  //initSerialPorts();
  initWiFi();
  initLcd();
  initServer();
  //Create separate task to display map on LCD
  //xTaskCreatePinnedToCore(dis_task, "lidar", 10 * 1024, NULL, 1, NULL, 1); 
}

void loop() {
  /*
  while (Serial1.available()) {
    lidar.lidar_data_deal(Serial1.read());
  }
  */
 /* */

  if(wheelMoving && (millis() - startedMoving) > requestedDuration){
    robot.wheelCommand(0,0,0,0);
    wheelMoving = false;
  }

  if(irrigating && (millis() - startedIrrigation) > requestedIrrigationDuration){
    robot.ledCommand(0,0,0,0);
    irrigating = false;
  }
  batvol = analogRead(PIN_BAT_VOLTS); 
  delay(UPDATE_INTERVAL_MS);
}

//-------------------------------------------------------------
void initServer()
{
  server.on("/", HTTP_GET, [] (AsyncWebServerRequest *request) {
      request->send(200, "application/json", td);
  });
    
  //-------------------------------- Properties ----------------

  server.on("/properties/lidar", HTTP_GET, [] (AsyncWebServerRequest *request) {
      createLidarData();
      request->send(200, "application/json", lidarData);
  });  

  server.on("/properties/soilcondition", HTTP_GET, [](AsyncWebServerRequest *request){ 
      #if USE_COLOR_SENSOR
      uint16_t clear, red, green, blue; 
      colorsensor.getRawData(&red, &green, &blue, &clear);
      StaticJsonDocument<200> soil;
      String colorData = "";
      soil.clear();
      soil["ph"] = clear;
      soil["moisture"] = blue;
      soil["density"] = red;
      soil["nitrate"] = green;
      //colorData = "[" + String(clear) + "," + String(red) + "," + String(green) + "," + String(blue) + "]";
      serializeJson(soil, colorData);
      #endif
      request->send(200, "application/json", colorData); }
    );

  server.on("/properties/batteryvoltage", HTTP_GET, [](AsyncWebServerRequest *request){ request->send(200, "application/json", String(batvol)); });

  server.on("/properties/waterlevel", HTTP_GET, [](AsyncWebServerRequest *request){ request->send(200, "application/json", String(waterLevel)); });

  server.on("/properties/temperature", HTTP_GET, [](AsyncWebServerRequest *request){ request->send(200, "application/json", String(temperature)); });
  
  //------------------------------ Actions -------------------------------
  AsyncCallbackJsonWebHandler *wheelHandler = new AsyncCallbackJsonWebHandler("/actions/wheelcontrol", [](AsyncWebServerRequest *request, JsonVariant &json) {
    StaticJsonDocument<200> data;
    data = json.as<JsonObject>();
    doWheelControl(json);
    String response;
    serializeJson(data, response);
    request->send(200, "application/json", response);
    Serial.println(response);
  });
  server.addHandler(wheelHandler); 

  AsyncCallbackJsonWebHandler *waterHandler = new AsyncCallbackJsonWebHandler("/actions/irrigate", [](AsyncWebServerRequest *request, JsonVariant &json) {
      StaticJsonDocument<200> data;
      data = json.as<JsonObject>();
      int duration = data["duration"];
      if(duration <= 5 && waterLevel > duration)
      {
        doIrrigation(json);     
        request->send(200, "application/json", "true");
      }else{
        request->send(400, "application/json", "false");
      }
  });
  server.addHandler(waterHandler); 

  server.on("/actions/refillwater", HTTP_POST, [] (AsyncWebServerRequest *request) {
      waterLevel = 10;     
      request->send(200, "application/json", "true");
  });  

  server.begin(); 
}
/*
void initSerialPorts(){
  Serial.begin(115200);  
  Serial1.begin(115200, SERIAL_8N1, 16, 17);
  Serial2.begin(115200);                        
}
*/
#if USE_COLOR_SENSOR
void initColorSensor(){
  while(!colorsensor.begin()){
      Serial.println("No TCS34725 found ... check your connections");
      M5.Lcd.setTextFont(4);
      M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
      M5.Lcd.drawString("No Found sensor.",50, 100, 4);
      delay(1000);
  }
  colorsensor.setIntegrationTime(TCS34725_INTEGRATIONTIME_154MS);
  colorsensor.setGain(TCS34725_GAIN_4X);  
}
#endif

void initLcd() {
  M5.Lcd.clear(BLACK);
  M5.Lcd.fillScreen(ROBOT_COLOR);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(3, 10);
  M5.Lcd.println(ROBOT_NAME);
  M5.Lcd.setTextSize(2);
  M5.Lcd.println(WiFi.localIP());
  //M5.Lcd.drawCircle(160, 120 , 4, WHITE);
}

void initWiFi() {
    WiFi.mode(WIFI_MODE_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    Serial.printf("Connecting to %s .", WIFI_SSID);
    while (WiFi.status() != WL_CONNECTED) { Serial.print("."); delay(200); }
    Serial.println(" ok");

    IPAddress ip = WiFi.localIP();

    Serial.printf("SSID: %s\n", WIFI_SSID);
    Serial.printf("Channel: %u\n", WiFi.channel());
    Serial.printf("IP: %u.%u.%u.%u\n", ip & 0xff, (ip >> 8) & 0xff, (ip >> 16) & 0xff, ip >> 24); 
}

void createLidarData(){
  float map_data_stash[720] = { 0.0 };
  xSemaphoreTake( xSemaphore, portMAX_DELAY);
  memcpy(map_data_stash, lidar.dismap.mapdata, 720*sizeof(float));
  xSemaphoreGive( xSemaphore );
  String ldata = "[" ;
  for (int i = 0; i < 720; i++) {
      ldata = ldata + String((int)map_data_stash[i]);
      if(i < 719)
        ldata = ldata + ",";
  }
  lidarData = ldata + "]";
}

void displayMap(void) {
  float map_data_stash[720] = { 0.0 };
  xSemaphoreTake( xSemaphore, portMAX_DELAY);
  memcpy(map_data_stash, lidar.dismap.mapdata, 720*sizeof(float)); 
  xSemaphoreGive( xSemaphore );

  for (int i = 0; i < 720; i++) {
      float oldAng = from_degrees((i / 2.0));
      float Ang = from_degrees((i / 2.0));
      float oldX = (sin(oldAng) * lidar.oldmap.mapdata[i]/5);
      float oldY = (cos(oldAng) * lidar.oldmap.mapdata[i]/5);
      float X = (sin(Ang) * map_data_stash[i]/5);
      float Y = (cos(Ang) * map_data_stash[i]/5);

      if (lidar.oldmap.mapdata[i] > 0) {
        M5.Lcd.drawPixel(-oldX + 160, oldY + 120, BLACK);  
      }

      if (map_data_stash[i] > 0) {
        M5.Lcd.drawPixel(-X + 160, Y + 120, WHITE);  
      }
      lidar.oldmap.mapdata[i] = map_data_stash[i];
  }
}

/*
static void dis_task(void *arg) {
  while(1) {
    //Serial.printf("uart_task\r\n");
    if (lidar.disPlayFlag){
      displayMap();
      delay(5); 
      lidar.disPlayFlag = 0;
    }    
    delay(10);
  }
}
*/
void doWheelControl(const JsonVariant &input) {
  JsonObject inputObj = input.as<JsonObject>();
  int axis = inputObj["axis"];
  int spd = inputObj["speed"];
  int duration = inputObj["duration"];
  Serial.print("ControlWheel(");Serial.print(axis);Serial.print(",");Serial.print(spd);Serial.print(",");Serial.print(duration);Serial.println(")");
  
  int wa = 0, wb = 0, wc = 0, wd = 0;
  if(axis == 0)
  { //X-axis
    wa = spd;
    wb = spd;
    wc = spd;
    wd = spd;
  }else if(axis == 1)
  { //Y-axis
    wa = spd;
    wb = -1*spd;
    wc = -1*spd;
    wd = spd;
  }else if(axis == 2)
  { //Z-axis
    wa = spd;
    wb = -1*spd;
    wc = spd;
    wd = -1*spd;
  }
    
  if(duration > 0){
    robot.wheelCommand(wa, wb, wc, wd);
    requestedDuration = duration;
    wheelMoving = true;
    startedMoving = millis();
  }else{
    wheelMoving = false;
    robot.wheelCommand(0,0,0,0);
  }
  temperature = temperature + (duration/1000);
  //batvol = batvol - (duration * .00001);
  /**/
}

void doIrrigation(const JsonVariant &input) {
  JsonObject inputObj = input.as<JsonObject>();
  int duration = inputObj["duration"];
  if(duration > 0){
    robot.ledCommand(0,0,0,0xff);
    requestedIrrigationDuration = duration * 1000;
    irrigating = true;
    startedIrrigation = millis();
    waterLevel = waterLevel - duration;
  }else{
    irrigating = false;
    robot.ledCommand(0,0,0,0);
  }  
}
/*
void wheelCommand(int wa, int wb, int wc, int wd){
  Serial2.write(0xAA);
  Serial2.write(wa);
  Serial2.write(wb);
  Serial2.write(wc);
  Serial2.write(wd);
  Serial2.write(0x55);
  Serial.printf("wheelCommand(%d, %d, %d, %d)\n", wa, wb, wc, wd);
}

void ledCommand(int pos, int r, int g, int b){
  Serial2.write(0xAE);
  Serial2.write(r);
  Serial2.write(g);
  Serial2.write(b);
  Serial2.write(0x55);
  Serial.printf("ledCommand(%d, %d, %d, %d)\n", pos, r, g, b);
}
*/
