//node 
#include <painlessMesh.h>
#include "base64.hpp"

bool is_connected;
unsigned long is_connected_tlast_ms;
unsigned long serial_data_received_tfirst_ms;
int baudrate = 921600;

#define MESH_PREFIX     "yourMeshNetwork"
#define MESH_PASSWORD   "yourMeshPassword"
#define MESH_PORT       5555

#define Drone Serial2

#define BUFFER_SIZE 1024  // You can change this value as needed
#define BATCH_SIZE 256

painlessMesh mesh;

void sendMessageToMesh(uint8_t* buf, int len) {
    int numBatches = (len + BATCH_SIZE - 1) / BATCH_SIZE; // Calculate number of batches

    for (int i = 0; i < numBatches; i++) {
        int offset = i * BATCH_SIZE;
        int batchSize = min(BATCH_SIZE, len - offset);

        uint8_t message[BATCH_SIZE + 2 * sizeof(int)];
        memcpy(message, &len, sizeof(len)); // Total length
        memcpy(message + sizeof(len), &i, sizeof(i)); // Batch number
        memcpy(message + 2 * sizeof(int), buf + offset, batchSize);

        uint8_t encodedMessage[(BATCH_SIZE + 2 * sizeof(int)) * 4 / 3 + 4]; 
        encode_base64(message, 2 * sizeof(int) + batchSize, encodedMessage);

        mesh.sendSingle(3813146501, (char*)encodedMessage); 
    }
}

void receivedCallback(uint32_t from, String &msg) {
    static uint8_t reassemblyBuffer[BUFFER_SIZE];
    static int totalLength = 0;
    static int receivedBatches = 0;

    uint8_t decoded[(BATCH_SIZE + 2 * sizeof(int)) * 4 / 3 + 4];
    int decodedLength = decode_base64((uint8_t*)msg.c_str(), msg.length(), decoded);

    int len, batchNumber;
    memcpy(&len, decoded, sizeof(len));
    memcpy(&batchNumber, decoded + sizeof(len), sizeof(batchNumber));

    int batchSize = decodedLength - 2 * sizeof(int);
    memcpy(reassemblyBuffer + batchNumber * BATCH_SIZE, decoded + 2 * sizeof(int), batchSize);

    receivedBatches++;
    if (receivedBatches * BATCH_SIZE >= len) {
        Drone.write(reassemblyBuffer, len);
        receivedBatches = 0; // Reset for next message
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
    } else if ((tnow_ms - serial_data_received_tfirst_ms) > 10 || avail > 256) { // 10 ms at 57600 bps corresponds to 57 bytes, no chance for 128 bytes
        serial_data_received_tfirst_ms = tnow_ms;
        int len = Drone.read(buf, sizeof(buf));
        sendMessageToMesh(buf, len); // sends data to router
    }
}

