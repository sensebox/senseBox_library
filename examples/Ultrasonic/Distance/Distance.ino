#include "SenseBox.h"


Ultrasonic Ultrasonic(3,4); //RX,TX Pins

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  long distance = Ultrasonic.getDistance();
  Serial.println(distance);
}
