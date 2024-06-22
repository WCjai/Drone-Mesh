#include <painlessMesh.h>
#include "PixhawkArduinoMAVLink.h" //has mavlink.h
#include "base64.hpp"

#define MESH_PREFIX     "yourMeshNetwork"
#define MESH_PASSWORD   "yourMeshPassword"
#define MESH_PORT       5555

int baudrate = 921600;
painlessMesh mesh;

#define BUFFER_SIZE 1024  // You can change this value as needed
#define MESH_PACKET_SIZE 256

void sendMessageToMesh(uint8_t* buf, int len) {
    int offset = 0;

    while (offset < len) {
        int packetSize = min(MESH_PACKET_SIZE, len - offset);

        uint8_t message[MESH_PACKET_SIZE];
        memcpy(message, buf + offset, packetSize);

        uint8_t encodedMessage[MESH_PACKET_SIZE * 4 / 3 + 4];
        encode_base64(message, packetSize, encodedMessage);

        mesh.sendBroadcast((char*)encodedMessage);

        offset += packetSize;
    }
}

void receivedCallback(uint32_t from, String &msg) {
    static uint8_t reassemblyBuffer[BUFFER_SIZE];
    static int receivedLength = 0;

    uint8_t decoded[MESH_PACKET_SIZE * 4 / 3 + 4];
    int decodedLength = decode_base64((uint8_t*)msg.c_str(), msg.length(), decoded);

    memcpy(reassemblyBuffer + receivedLength, decoded, decodedLength);
    receivedLength += decodedLength;

    // Here, we assume the whole MAVLink message is received
    mavlink_message_t message;
    mavlink_status_t status;

    for (int i = 0; i < receivedLength; i++) {
        if (mavlink_parse_char(MAVLINK_COMM_0, reassemblyBuffer[i], &message, &status)) {
            // Successfully parsed a MAVLink message
            uint8_t send_buf[MAVLINK_MAX_PACKET_LEN];
            int send_len = mavlink_msg_to_send_buffer(send_buf, &message);
            Serial.write(send_buf, send_len);
            receivedLength = 0; // Reset buffer for next message
            break;
        }
    }
}

void serialFlushRx(void) {
    while (Serial.available() > 0) { Serial.read(); }
}

void setup() {
    delay(500);
    Serial.begin(baudrate);
    Serial.print("Node ID: ");
    Serial.println(mesh.getNodeId());

    mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);  // set before init() so that you can see startup messages
    mesh.init(MESH_PREFIX, MESH_PASSWORD, MESH_PORT);
    mesh.onReceive(&receivedCallback); // writes data from node
    serialFlushRx();
}

void loop() { 
    mesh.update();
    uint8_t buf[BUFFER_SIZE];
    int packetSize = Serial.available();
    if (packetSize > 0) {
        int len = Serial.readBytes(buf, packetSize); // Read the bytes into the buffer    

        mavlink_message_t msg;
        mavlink_status_t status;

        // Parse the received MAVLink messages
        for (int i = 0; i < len; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                // Successfully parsed a MAVLink message
                uint8_t send_buf[MAVLINK_MAX_PACKET_LEN];
                int send_len = mavlink_msg_to_send_buffer(send_buf, &msg);
                sendMessageToMesh(send_buf, send_len); // sends GCS data to node
            }
        }
    }
}
