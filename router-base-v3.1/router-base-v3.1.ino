// router
#include <painlessMesh.h>
#include "base64.hpp"

#define MESH_PREFIX     "yourMeshNetwork"
#define MESH_PASSWORD   "yourMeshPassword"
#define MESH_PORT       5555

int baudrate = 921600;
painlessMesh mesh;

#define BUFFER_SIZE 1024  // You can change this value as needed
#define BATCH_SIZE 256

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

        mesh.sendBroadcast((char*)encodedMessage); 
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
        Serial.write(reassemblyBuffer, len);
        receivedBatches = 0; // Reset for next message
    }
}

void serialFlushRx(void) {
    while (Serial.available() > 0) { Serial.read(); }
}

void setup() {
    delay(500);
    Serial.begin(baudrate);
    Serial.print("below");
    Serial.print(mesh.getNodeId());

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
        sendMessageToMesh(buf, len); // sends GCS data to node
    }
}

