#include "SenseBox.h"

OpenSenseMap osem = OpenSenseMap("boxID", true, "ingress.opensensemap.org", 80); // boxID, log enabled, osem api domain, osem api port

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial) { ; }

  Serial.println("init");
    
  osem.beginWiFi("SSID", "PASSWD");
  //osem.beginEthernet("192.168.2.125");
}

void loop() {
  delay(3000);   
  osem.uploadValue(24.3f, "sensorID");
}
