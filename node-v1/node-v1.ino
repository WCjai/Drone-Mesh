#include <painlessMesh.h>
#include "base64.hpp"

bool is_connected;
unsigned long is_connected_tlast_ms;
unsigned long serial_data_received_tfirst_ms;
int baudrate = 115200;


#define MESH_PREFIX     "yourMeshNetwork"
#define MESH_PASSWORD   "yourMeshPassword"
#define MESH_PORT       5555

#define Drone Serial2

painlessMesh mesh;

void sendMessageToMesh(uint8_t* buf, int len) {
    uint8_t message[256 + sizeof(len)];
    memcpy(message, &len, sizeof(len)); 
    memcpy(message + sizeof(len), buf, len); 
    
    // encode the data to be sent to router
    uint8_t encodedMessage[384]; 
    encode_base64(message, sizeof(len) + len, encodedMessage);
    
    // broadcast this encoded message to all avaiable nobe ready to reacive 
    //mesh.sendBroadcast((char*)encodedMessage);
    mesh.sendSingle(3813146501 , (char*)encodedMessage); 
}

void receivedCallback(uint32_t from, String &msg) {
    // decode the on comming message 
    uint8_t decoded[256];
    uint8_t decodedLength = decode_base64((uint8_t*)msg.c_str(), msg.length(), decoded);

    int len;
    memcpy(&len, decoded, sizeof(len));

    uint8_t buf[len];
    memcpy(buf, decoded + sizeof(len), len);
    // write to drone serial 
    Drone.write(buf, len);
 
}

void serialFlushRx(void)
{
    while (Drone.available() > 0) { Drone.read(); }
}


void setup()
{

    delay(500);
    Serial.begin(115200);

    mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);  // set before init() so that you can see startup messages
    mesh.init(MESH_PREFIX, MESH_PASSWORD, MESH_PORT);
    Serial.print(mesh.getNodeId());
    size_t rxbufsize = Drone.setRxBufferSize(2*1024); // must come before uart started, retuns 0 if it fails
    size_t txbufsize = Drone.setTxBufferSize(512); // must come before uart started, retuns 0 if it fails
    Drone.begin(baudrate, SERIAL_8N1, 16, 17);
    mesh.onReceive(&receivedCallback);      // writes data from router to drone
    is_connected = false;
    is_connected_tlast_ms = 0;
    serial_data_received_tfirst_ms = 0;
    serialFlushRx();
}


void loop()
{ 
    mesh.update();

    unsigned long tnow_ms = millis();
    uint8_t buf[256];

    if (is_connected && (tnow_ms - is_connected_tlast_ms > 2000)) { // nothing from GCS for 2 secs
        is_connected = false;
    }

    tnow_ms = millis(); // may not be relevant, but just update it
    int avail = Drone.available();
    if (avail <= 0) {
        serial_data_received_tfirst_ms = tnow_ms;
    } else
    if ((tnow_ms - serial_data_received_tfirst_ms) > 10 || avail > 128) { // 10 ms at 57600 bps corresponds to 57 bytes, no chance for 128 bytes
        serial_data_received_tfirst_ms = tnow_ms;
        int len = Drone.read(buf, sizeof(buf));
        sendMessageToMesh(buf, len);           // sends data to router
    }
  
}
