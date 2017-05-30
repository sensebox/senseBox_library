#include "OpenSenseMap.h"

OpenSenseMap::OpenSenseMap() { 
}

OpenSenseMap::OpenSenseMap(bool enableLogging, const char* serverDomain, unsigned int serverPort)
{
  enableLog = enableLogging;
  server = serverDomain;
  port = serverPort;
}

void OpenSenseMap::beginEthernet() {
  beginEthernet("192.168.1.123");
}

void OpenSenseMap::beginEthernet(const char* ipAddress)
{
  client = new EthernetClient();

  // give the ethernet module time to boot up
  delay(1000);

  log("Starting ethernet connection...");
  if (Ethernet.begin(mac) == 0)
  {
    log("Failed to configure Ethernet using DHCP, trying static IP\n");
    IPAddress myIp = IPAddress().fromString(ipAddress);
    Ethernet.begin(mac, myIp);
  }
  else
  {
    log("done! IP Address is: \n");
    log(Ethernet.localIP());
  }
}

void OpenSenseMap::beginWiFi(char* ssid, char* pass)
{
  client = new WiFiClient;

  //Enable Wifi Shield
  pinMode(4, INPUT);
  digitalWrite(4, HIGH);
  delay(2000);

  //Check WiFi Shield status
  if (WiFi.status() == WL_NO_SHIELD)
  {
    log("WiFi shield not present");
    while (true); // don't continue:
  }
  // attempt to connect to Wifi network:
  while (wifistatus != WL_CONNECTED)
  {
    log("Attempting to connect to SSID: ");
    log(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network
    wifistatus = WiFi.begin(ssid, pass);
    // wait 60 seconds for connection:
    log("\n\n\n");
    log("Waiting 3 seconds for connection...");
    delay(3000);
    log("done.\n");
  }

}

void OpenSenseMap::postFloatValue(float measurement, String sensorId, String boxId)
{
  char obs[10];
  dtostrf(measurement, 5, 2, obs);
  log(obs);
  //json must look like: {"value":"12.5"}
  //post observation to: /boxes/boxId/sensorId

  Serial.println("__________________________\n");
  if (client->connected()) {
    client->stop();
    delay(1000);
  }

  log("connecting...");
  log(server);
  if (client->connect(server, port))
  {
    log("connected\n");
    String value = "{\"value\":";
    value += obs;
    value += "}";
    // Make a HTTP Post request:
    client->print("POST /boxes/");
    client->print(boxId);
    client->print("/");
    client->print(sensorId);
    client->println(" HTTP/1.1");
    // Send the required header parameters
    client->print("Host:");
    client->println(server);
    client->println("Content-Type: application/json");
    client->println("Connection: close");
    client->print("Content-Length: ");
    client->println(value.length());
    client->println();
    // Send the data
    client->print(value);
    client->println();
  }
  waitForResponse();
}

void OpenSenseMap::waitForResponse()
{
  // if there are incoming bytes from the server, read and print them
  delay(100);
  String response = "";
  char c;
  boolean repeat = true;
  do {
    if (client->available()) c = client->read();
    else repeat = false;
    response += c;
    if (response == "HTTP/1.1 ") response = "";
    if (c == '\n') repeat = false;
  }
  while (repeat);

  log("Server Response: ");
  Serial.println(response);

  client->flush();
  client->stop();
}

bool OpenSenseMap::log(const char data)
{
  if (!enableLog || !Serial) return false;

  Serial.print(data);
  return true;
}

bool OpenSenseMap::log(const char* data)
{
  if (!enableLog || !Serial) return false;

  Serial.print(data);
  return true;
}
