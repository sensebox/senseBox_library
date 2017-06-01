#pragma once

#include "./Ethernet.h"
#include "./WiFi101.h"

class OpenSenseMap {

public:
  OpenSenseMap(const char* boxId);
  OpenSenseMap(const char* boxId, bool enableLogging, const char* server, unsigned int port);
  void beginEthernet();
  void beginEthernet(const char* ipAddress);
  void beginWiFi(char* ssid, char* passwd);
  void uploadValue(float measurement, const char* sensorId);
  void uploadValue(int measurement, const char* sensorId);

private:
  byte mac[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
  const char* server = "ingress.opensensemap.org";
  const char* boxId = "";
  unsigned int port = 80;
  bool enableLog = true;
  Client* client = NULL;
  uint8_t wifistatus = WL_IDLE_STATUS;

  void postToOsem(char* obs, const char* sensorId);
  void waitForResponse();
  bool log(const char data);
  bool log(const char* data);
  bool log(String data);
};
