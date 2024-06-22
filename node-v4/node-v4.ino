// node
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

#define Drone Serial2

#define BUFFER_SIZE 1024
#define MESH_PACKET_SIZE 256

painlessMesh mesh;

void sendMessageToMesh(uint8_t* buf, int len) {
    int offset = 0;

    while (offset < len) {
        int packetSize = min(MESH_PACKET_SIZE, len - offset);

        uint8_t message[MESH_PACKET_SIZE];
        memcpy(message, buf + offset, packetSize);

        uint8_t encodedMessage[MESH_PACKET_SIZE * 4 / 3 + 4];
        encode_base64(message, packetSize, encodedMessage);

        mesh.sendSingle(3813146501, (char*)encodedMessage);

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
            Drone.write(send_buf, send_len);
            receivedLength = 0; // Reset buffer for next message
            break;
        }
    }
}

void serialFlushRx(void) {
    while (Drone.available() > 0) { Drone.read(); }
}

void setup() {
    delay(500);
    Serial.begin(115200);

    mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);  // set before init() so that you can see startup messages
    mesh.init(MESH_PREFIX, MESH_PASSWORD, MESH_PORT);
    Serial.print(mesh.getNodeId());
    size_t rxbufsize = Drone.setRxBufferSize(2 * 1024); // must come before uart started, returns 0 if it fails
    size_t txbufsize = Drone.setTxBufferSize(BUFFER_SIZE); // must come before uart started, returns 0 if it fails
    Drone.begin(baudrate, SERIAL_8N1, 16, 17);
    mesh.onReceive(&receivedCallback); // writes data from router to drone
    is_connected = false;
    is_connected_tlast_ms = 0;
    serial_data_received_tfirst_ms = 0;
    serialFlushRx();
}

void loop() {
    mesh.update();

    unsigned long tnow_ms = millis();
    uint8_t buf[BUFFER_SIZE];

    if (is_connected && (tnow_ms - is_connected_tlast_ms > 2000)) { // nothing from GCS for 2 secs
        is_connected = false;
    }

    tnow_ms = millis(); // may not be relevant, but just update it
    int avail = Drone.available();
    if (avail <= 0) {
        serial_data_received_tfirst_ms = tnow_ms;
    } else if ((tnow_ms - serial_data_received_tfirst_ms) > 10 || avail > 256) {
        serial_data_received_tfirst_ms = tnow_ms;
        int len = Drone.read(buf, sizeof(buf));

        mavlink_message_t msg;
        mavlink_status_t status;

        // Parse the received MAVLink messages
        for (int i = 0; i < len; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                // Successfully parsed a MAVLink message
                uint8_t send_buf[MAVLINK_MAX_PACKET_LEN];
                int send_len = mavlink_msg_to_send_buffer(send_buf, &msg);
                sendMessageToMesh(send_buf, send_len);
            }
        }
    }
}
