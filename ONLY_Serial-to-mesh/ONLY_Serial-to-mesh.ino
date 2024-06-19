#define MODULE_NODEMCU_ESP32_WROOM32
#include <WiFi.h>
int baudrate = 115200;

#include "mlrs-wireless-bridge-boards.h"
#include "base64.hpp"

WiFiUDP udp;

bool is_connected;
unsigned long is_connected_tlast_ms;
unsigned long serial_data_received_tfirst_ms;

#include <painlessMesh.h>

#define MESH_PREFIX     "yourMeshNetwork"
#define MESH_PASSWORD   "yourMeshPassword"
#define MESH_PORT       5555


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

    SERIAL.write(buf, len);
 
}

void serialFlushRx(void)
{
    while (SERIAL.available() > 0) { SERIAL.read(); }
}


void setup()
{

    delay(500);

    mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);  // set before init() so that you can see startup messages
    mesh.init(MESH_PREFIX, MESH_PASSWORD, MESH_PORT);
    size_t rxbufsize = SERIAL.setRxBufferSize(2*1024); // must come before uart started, retuns 0 if it fails
    size_t txbufsize = SERIAL.setTxBufferSize(512); // must come before uart started, retuns 0 if it fails
    SERIAL.begin(baudrate, SERIAL_8N1, SERIAL_RXD, SERIAL_TXD);
    mesh.onReceive(&receivedCallback);
    is_connected = false;
    is_connected_tlast_ms = 0;
    serial_data_received_tfirst_ms = 0;
    serialFlushRx();
}


void loop()
{ 
    
    mesh.update();

    unsigned long tnow_ms = millis();

    if (is_connected && (tnow_ms - is_connected_tlast_ms > 2000)) { // nothing from GCS for 2 secs
        is_connected = false;
    }
    uint8_t buf[256]; // working buffer
    int packetSize = udp.parsePacket();
    if (packetSize) {
        int len = udp.read(buf, sizeof(buf));
        SERIAL.write(buf, len);
        is_connected = true;
        is_connected_tlast_ms = millis();
    }

    tnow_ms = millis(); // may not be relevant, but just update it
    int avail = SERIAL.available();
    if (avail <= 0) {
        serial_data_received_tfirst_ms = tnow_ms;
    } else
    if ((tnow_ms - serial_data_received_tfirst_ms) > 10 || avail > 128) { // 10 ms at 57600 bps corresponds to 57 bytes, no chance for 128 bytes
        serial_data_received_tfirst_ms = tnow_ms;
        int len = SERIAL.read(buf, sizeof(buf));
        sendMessageToMesh(buf, len);
    }
  
}
