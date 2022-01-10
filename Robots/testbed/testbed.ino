#include <Arduino.h>
#include <M5Stack.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "bmm.h"


#define PIN_BAT_VOLTS 35
#define PIN_BAT_AMPS 36
#define PIN_LIDAR_ENABLE 5
#define PIN_TEMPERATURE_SENSOR 19

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(PIN_TEMPERATURE_SENSOR);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature tsensor(&oneWire);

BMM150 bmm;

// the setup routine runs once when M5Stack starts up
void setup(){
  Serial2.begin(115200);
  pinMode(PIN_BAT_VOLTS, INPUT);
  pinMode(PIN_BAT_AMPS, INPUT);
  pinMode(PIN_LIDAR_ENABLE, OUTPUT);
  
  digitalWrite(PIN_LIDAR_ENABLE, LOW);
  // Initialize the M5Stack object
  M5.begin();

  /*
    Power chip connected to gpio21, gpio22, I2C device
    Set battery charging voltage and current
    If used battery, please call this function in your project
  */
  M5.Power.begin();
    
  // LCD display
  M5.Lcd.setTextSize(2);
  M5.Lcd.print("IO Test");
  delay(2000);
  //pinMode(PIN_TEMPERATURE_SENSOR, INPUT_PULLUP);
  tsensor.begin();
  bmm.Initialize();
}



// the loop routine runs over and over again forever
void loop() {
  M5.Lcd.clear();
  int v = analogRead(PIN_BAT_VOLTS);
  int i = analogRead(PIN_BAT_AMPS);
  tsensor.requestTemperatures(); 
  float t = tsensor.getTempCByIndex(0);
  float h = bmm.ReadHeading();
  //float t = 0;
  Serial.println(t);
  M5.Lcd.setCursor(0, 20);
  M5.Lcd.printf("V=%d, I=%d, T=%d", v, i, (int)t);
  M5.Lcd.setCursor(0, 50);
  M5.Lcd.printf("Heading=%f",h);
  M5.update();

 
  // if you want to use Releasefor("was released for"), use .wasReleasefor(int time) below
  if (M5.BtnA.wasReleased() || M5.BtnA.pressedFor(1000, 200)) {
     M5.Lcd.setTextColor(GREEN);
     digitalWrite(PIN_LIDAR_ENABLE, LOW);
      Serial2.write(0xAA);
      Serial2.write(0);
      Serial2.write(0);
      Serial2.write(0);
      Serial2.write(0);
      Serial2.write(0x55);     
  }
  else if (M5.BtnB.wasReleased() || M5.BtnB.pressedFor(1000, 200)) {
     M5.Lcd.setTextColor(RED);
     digitalWrite(PIN_LIDAR_ENABLE, HIGH);
  }
  else if (/*M5.BtnC.wasReleased() ||*/ M5.BtnC.pressedFor(1000, 200)) {
    Serial2.write(0xAA);
    Serial2.write(-1);
    Serial2.write(1);
    Serial2.write(-1);
    Serial2.write(1);
    Serial2.write(0x55);
    bmm.Calibrate(10000);
    Serial2.write(0xAA);
    Serial2.write(0);
    Serial2.write(0);
    Serial2.write(0);
    Serial2.write(0);
    Serial2.write(0x55);   
  }
  else if (M5.BtnB.wasReleasefor(700)) {

  }
  delay(1000);
}
