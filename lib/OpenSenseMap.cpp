#include "OpenSenseMap.h"

OpenSenseMap::OpenSenseMap()
{
  server = "ingress.opensemap.org";
  myIp = IPAddress(192, 168, 0, 155);
}

OpenSenseMap::OpenSenseMap(String serverDomain, String ipAddress)
{
  server = serverDomain;
  myIp = IPAddress();
  myIp.fromString(ipAddress);
}

void OpenSenseMap::begin()
{
  //Serial.print("Starting ethernet connection...");

  if (Ethernet.begin(mac) == 0) {
    //Serial.println("Failed to configure Ethernet using DHCP");
    Ethernet.begin(mac, myIp);
  } else {
    //Serial.println("done!");
  }
}

void OpenSenseMap::loop()
{
  if (client.available()) {
    char c = client.read();
    //Serial.print(c);
  }
}

void OpenSenseMap::postObservation(float measurement, String sensorId, String boxId)
{
  char obs[10];
  dtostrf(measurement, 5, 2, obs);
  //Serial.println(obs);
  //json must look like: {"value":"12.5"}
  //post observation to: /boxes/boxId/sensorId
  //Serial.println("connecting...");
  String value = "{\"value\":";
  value += obs;
  value += "}";
  if (client.connect(server, 80))
  {
    //Serial.println("connected");
    // Make a HTTP Post request:
    client.print("POST /boxes/");
    client.print(boxId);
    client.print("/");
    client.print(sensorId);
    client.println(" HTTP/1.1");
    // Send the required header parameters
    client.print("Host:");
    client.println(server);
    client.println("Content-Type: application/json");
    client.println("Connection: close");
    client.print("Content-Length: ");
    client.println(value.length());
    client.println();
    // Send the data
    client.print(value);
    client.println();
  }
  waitForResponse();
}

void OpenSenseMap::waitForResponse()
{
  // if there are incoming bytes available
  // from the server, read them and print them:
  boolean repeat = true;
  do {
    if (client.available())
    {
      char c = client.read();
      //Serial.print(c);
    }
    // if the servers disconnected, stop the client:
    if (!client.connected())
    {
      //Serial.println();
      //Serial.println("disconnecting.");
      client.stop();
      repeat = false;
    }
  }
  while (repeat);
}
