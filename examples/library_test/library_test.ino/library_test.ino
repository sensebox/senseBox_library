#include "SenseBox.h"

HDC100X HDC1(0,0);
Ultrasonic Ultrasonic(3,4); //RX,TX Pins
TSL45315 luxsensor = TSL45315(TSL45315_TIME_M4);
VEML6070 uvsensor;


uint32_t lux;
uint16_t uv;

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
HDC1.begin(HDC100X_TEMP_HUMI,HDC100X_14BIT,HDC100X_14BIT,DISABLE);
luxsensor.begin();
uvsensor.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  long distance = Ultrasonic.getDistance();
  Serial.println(distance);
  Serial.print(" Humidity: ");
    Serial.print(HDC1.getHumi()); 
    Serial.print("%, Temperature: ");     
    Serial.print(HDC1.getTemp());
    Serial.println("C");

   lux = luxsensor.readLux();
   uv = uvsensor.getUV();
   
}
