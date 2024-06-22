#include <painlessMesh.h>
#include "base64.hpp"

#define MESH_PREFIX     "yourMeshNetwork"
#define MESH_PASSWORD   "yourMeshPassword"
#define MESH_PORT       5555

int baudrate = 57600; // Raised baudrate
painlessMesh mesh;

void sendMessageToMesh(uint8_t* buf, int len) {
    uint8_t encodedMessage[384]; // Larger buffer for base64 encoding
    encode_base64(buf, len, encodedMessage);
    String message = String((char*)encodedMessage);
    Serial.print("Broadcasting encoded message: ");
    Serial.println(message);
    mesh.sendBroadcast(message);
}

void receivedCallback(uint32_t from, String &msg) {
    Serial.print("Received message from mesh: ");
    Serial.println(msg);

    uint8_t encoded[384];
    uint8_t decoded[256];
    memcpy(encoded, msg.c_str(), msg.length());
    int decodedLength = decode_base64(encoded, msg.length(), decoded);

    Serial.print("Decoded message: ");
    for (int i = 0; i < decodedLength; i++) {
        Serial.print(decoded[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    Serial.write(decoded, decodedLength);
}

void serialFlushRx() {
    while (Serial.available() > 0) { Serial.read(); }
}

void setup() {
    delay(500);
    Serial.begin(baudrate);
    Serial.print("Router ID: ");
    Serial.println(mesh.getNodeId());

    mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);
    mesh.init(MESH_PREFIX, MESH_PASSWORD, MESH_PORT);
    mesh.onReceive(&receivedCallback);

    serialFlushRx();
}

void loop() { 
    mesh.update();
    uint8_t buf[256]; // Increased buffer size
    int packetSize = Serial.available();
    if (packetSize > 0) {
        Serial.print("Reading from Serial: ");
        int len = Serial.readBytes(buf, packetSize);
        Serial.print("Data: ");
        for (int i = 0; i < len; i++) {
            Serial.print(buf[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
        sendMessageToMesh(buf, len);
    }
}
