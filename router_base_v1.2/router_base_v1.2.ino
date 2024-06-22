
#include <painlessMesh.h>
#include "base64.hpp"

#define MESH_PREFIX     "yourMeshNetwork"
#define MESH_PASSWORD   "yourMeshPassword"
#define MESH_PORT       5555


int baudrate = 921600;
painlessMesh mesh;


void sendMessageToMesh(uint8_t* buf, int len) {
    uint8_t message[512 + sizeof(len)];
    memcpy(message, &len, sizeof(len)); 
    memcpy(message + sizeof(len), buf, len); 
    uint8_t encodedMessage[768]; 
    encode_base64(message, sizeof(len) + len, encodedMessage);
    mesh.sendBroadcast((char*)encodedMessage); 
}

void receivedCallback(uint32_t from, String &msg) {
    uint8_t decoded[512];
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
    Serial.begin(baudrate);
    Serial.print("below");
    Serial.print(mesh.getNodeId());

    mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);  // set before init() so that you can see startup messages
    mesh.init(MESH_PREFIX, MESH_PASSWORD, MESH_PORT);
    mesh.onReceive(&receivedCallback);   // writes data from node
    serialFlushRx();

}


void loop()
{ 
    
  mesh.update();
  uint8_t buf[512];
  int packetSize = Serial.available();
  if (packetSize > 0) {
        Serial.print("iam read");
        int len = Serial.readBytes(buf, packetSize); // Read the bytes into the buffer    
        sendMessageToMesh(buf, len);   // sends gcs data to node
    }
}