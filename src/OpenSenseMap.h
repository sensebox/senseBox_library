#pragma once

#include "./Ethernet.h"
#include "./WiFi101.h"

class OpenSenseMap {

public:
  OpenSenseMap();
  OpenSenseMap(bool enableLogging, const char* server);
  void beginEthernet();
  void beginEthernet(const char* ipAddress);
  void beginWiFi(char* ssid, char* passwd);
  void postFloatValue(float measurement, String sensorId, String boxId);

private:
  byte mac[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
  const char* server = "ingress.opensemap.org";
  bool enableLog = false;
  Client* client = NULL;
  uint8_t wifistatus = WL_IDLE_STATUS;

  void waitForResponse();
  bool log(const char data);
  bool log(const char* data);
};

