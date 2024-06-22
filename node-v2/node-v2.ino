#include <painlessMesh.h>
#include "PixhawkArduinoMAVLink.h" //has mavlink.h
#include "base64.hpp"


bool is_connected;
unsigned long is_connected_tlast_ms;
unsigned long serial_data_received_tfirst_ms;
int baudrate = 921600;

#define MESH_PREFIX     "yourMeshNetwork"
#define MESH_PASSWORD   "yourMeshPassword"
#define MESH_PORT       5555

#define MAX_MESH_PACKET_SIZE 512
#define Drone Serial2

painlessMesh mesh;

void sendMessageToMesh(uint8_t* buf, int len) {
    uint8_t encodedMessage[768]; // Larger buffer for base64 encoding
    encode_base64(buf, len, encodedMessage);
    String message = String((char*)encodedMessage);

    // Debug: Print the encoded message
    Serial.print("Sending encoded message to mesh: ");
    Serial.println(message);

    mesh.sendSingle(3813146501, message);
}

void receivedCallback(uint32_t from, String &msg) {
    Serial.print("Received message from mesh: ");
    Serial.println(msg);

    uint8_t encoded[768];
    uint8_t decoded[512];
    memcpy(encoded, msg.c_str(), msg.length());
    int decodedLength = decode_base64(encoded, msg.length(), decoded);

    Serial.print("Decoded message: ");
    for (int i = 0; i < decodedLength; i++) {
        Serial.print(decoded[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    // Write to drone serial
    Drone.write(decoded, decodedLength);
}

void serialFlushRx() {
    while (Drone.available() > 0) { Drone.read(); }
}

void setup() {
    delay(500);
    Serial.begin(115200);

    mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);
    mesh.init(MESH_PREFIX, MESH_PASSWORD, MESH_PORT);
    Serial.print("Node ID: ");
    Serial.println(mesh.getNodeId());

    size_t rxbufsize = Drone.setRxBufferSize(4*1024); // Increased buffer size
    size_t txbufsize = Drone.setTxBufferSize(1024); // Increased buffer size
    Drone.begin(baudrate, SERIAL_8N1, 16, 17);

    mesh.onReceive(&receivedCallback);

    is_connected = false;
    is_connected_tlast_ms = 0;
    serial_data_received_tfirst_ms = 0;
    serialFlushRx();
}

void loop() { 
    mesh.update();

    unsigned long tnow_ms = millis();
    static uint8_t buf[MAX_MESH_PACKET_SIZE];
    static int buf_len = 0;

    if (is_connected && (tnow_ms - is_connected_tlast_ms > 2000)) {
        is_connected = false;
    }

    tnow_ms = millis();
    int avail = Drone.available();
    if (avail > 0) {
        int len = Drone.read(buf + buf_len, sizeof(buf) - buf_len);
        buf_len += len;

        // Debug: Print the raw data read from the drone
        Serial.print("Raw data read from drone: ");
        for (int i = 0; i < buf_len; i++) {
            Serial.print(buf[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }

    // Parse and pack MAVLink messages
    while (buf_len >= MAVLINK_NUM_HEADER_BYTES) {
        mavlink_message_t msg;
        mavlink_status_t status;
        int msg_len = mavlink_parse_char(MAVLINK_COMM_0, buf[0], &msg, &status);

        if (msg_len > 0) {
            // If a full MAVLink message has been received
            int mavlink_msg_len = MAVLINK_NUM_HEADER_BYTES + msg.len + MAVLINK_NUM_CHECKSUM_BYTES;
            if (mavlink_msg_len <= buf_len) {
                // Debug: Print the MAVLink message
                Serial.print("MAVLink message received: ");
                for (int i = 0; i < mavlink_msg_len; i++) {
                    Serial.print(buf[i], HEX);
                    Serial.print(" ");
                }
                Serial.println();
                sendMessageToMesh(buf, mavlink_msg_len);
                buf_len -= mavlink_msg_len;
                memmove(buf, buf + mavlink_msg_len, buf_len);
            } else {
                break; // Not enough data to parse a full message
            }
        } else {
            // No valid MAVLink message found, discard the first byte
            memmove(buf, buf + 1, --buf_len);
        }
    }
}
