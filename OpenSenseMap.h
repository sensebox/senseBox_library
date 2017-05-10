#pragma once

#include <Ethernet.h>


class OpenSenseMap {

public:
  OpenSenseMap();
  OpenSenseMap(String ipAddress);
  void begin();
  void loop();
  void postObservation(float measurement, String sensorId, String boxId);

private:
  IPAddress myIp;
  byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
  char server[] = "ingress.opensemap.org";
  EthernetClient client;

  void waitForResponse();
}

