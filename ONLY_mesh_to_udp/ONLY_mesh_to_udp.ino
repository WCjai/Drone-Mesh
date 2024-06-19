#define MODULE_NODEMCU_ESP32_WROOM32
#include <painlessMesh.h>
#include <WiFi.h>
#include "mlrs-wireless-bridge-boards.h"
#include "base64.hpp"

String ssid = "mLRS AP"; // Wifi name
String password = ""; // "thisisgreat"; // WiFi password, "" makes it an open AP
IPAddress ip(192, 168, 4, 55); // connect to this IP // MissionPlanner default is 127.0.0.1, so enter
int port_udp = 14550; // connect to this port per UDP // MissionPlanner default is 14550
int wifi_channel = 6;
#define WIFI_POWER  WIFI_POWER_2dBm // WIFI_POWER_MINUS_1dBm is the lowest possible, WIFI_POWER_19_5dBm is the max
#define MESH_PREFIX     "yourMeshNetwork"
#define MESH_PASSWORD   "yourMeshPassword"
#define MESH_PORT       5555


int baudrate = 115200;

IPAddress ip_udp(ip[0], ip[1], ip[2], ip[3]+1); // usually the client/MissionPlanner gets assigned +1
IPAddress ip_gateway(0, 0, 0, 0);
IPAddress netmask(255, 255, 255, 0);

WiFiUDP udp;



painlessMesh mesh;
void sendMessageToMesh(uint8_t* buf, int len) {
    uint8_t message[256 + sizeof(len)];
    memcpy(message, &len, sizeof(len)); 
    memcpy(message + sizeof(len), buf, len); 

    uint8_t encodedMessage[384]; 
    encode_base64(message, sizeof(len) + len, encodedMessage);

    mesh.sendBroadcast((char*)encodedMessage); 
}

void receivedCallback(uint32_t from, String &msg) {
    uint8_t decoded[256];
    uint8_t decodedLength = decode_base64((uint8_t*)msg.c_str(), msg.length(), decoded);

    int len;
    memcpy(&len, decoded, sizeof(len));

    uint8_t buf[len];
    memcpy(buf, decoded + sizeof(len), len);


    Serial.write(buf, len);
 


}

void serialFlushRx(void)
{
    while (Serial.available() > 0) { Serial.read(); }
}


void setup()
{

    delay(500);
    Serial.begin(115200);

    mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);  // set before init() so that you can see startup messages
    mesh.init(MESH_PREFIX, MESH_PASSWORD, MESH_PORT);
    mesh.onReceive(&receivedCallback);

    serialFlushRx();


}


void loop()
{ 
    
  mesh.update();
  uint8_t buf[256];
  int packetSize = Serial.available();
  if (packetSize > 0) {
        Serial.print("iam read");
        int len = Serial.readBytes(buf, packetSize); // Read the bytes into the buffer    
        sendMessageToMesh(buf, len);
    }
}
