//************************************************************
// this is a simple example that uses the easyMesh library
//
// 1. blinks led once for every node on the mesh
// 2. blink cycle repeats every BLINK_PERIOD
// 3. sends a silly message to every node on the mesh at a random time between 1
// and 5 seconds
// 4. prints anything it receives to Serial.print
//
//
//************************************************************
#include <painlessMesh.h>

#define MESH_SSID "whateverYouLike"
#define MESH_PASSWORD "somethingSneaky"
#define MESH_PORT 5555
#define MESH_CHAN 1
#define MESH_NAME "painlessMesh"

// Prototypes
void startMesh();
void stopMesh();
void receivedCallback(uint32_t from, String& msg);
unsigned int printChain();

Scheduler userScheduler;  // to control your personal task
painlessMesh mesh;

boolean meshUp = false;
uint32_t baseline = 10 * 1000;
uint32_t stop_timer = baseline;
uint32_t delayUp = 0;
uint32_t ct1 = 0;
uint32_t tnow;

void setup() {
  Serial.begin(115200);

  mesh.setDebugMsgTypes(ERROR | APPLICATION);

  startMesh();
  mesh.setContainsRoot(true);

  randomSeed(analogRead(A0));
}

void loop() {
  mesh.update();
  tnow = millis();
  if (!meshUp) {
    if (tnow > delayUp) {
      startMesh();
    }
  } else {
    if (tnow > stop_timer && tnow > baseline) {
      stopMesh();
    }
  }
  ct1++;
  if (ct1 % 50000 == 0) {
    Serial.printf("%1.3f,%u,%u,%d\n", millis() / 3600000.0, ESP.getFreeHeap(),
                  printChain(), meshUp);
    ct1 = 0;
  }
}

void startMesh() {
  Serial.println("Starting mesh");
  mesh.init(MESH_SSID, MESH_PASSWORD, &userScheduler, MESH_PORT, WIFI_STA);
  meshUp = true;
  stop_timer = millis() + 30000;  // keep up for 30s
  mesh.onReceive(&receivedCallback);
}

void stopMesh() {
  Serial.println("Stopping mesh");
  mesh.stop();
  meshUp = false;
  delayUp = millis() + 30000;  // keep stopped for 30s
}

void receivedCallback(uint32_t from, String& msg) {}

unsigned int printChain() {
  unsigned int n = 0;
  Task* f = userScheduler.getFirstTask();
  Serial.print("Task(s): ");
  while (f) {
    Serial.printf("%d ", f->getId());
    n++;
    f = f->getNextTask();
  }
  Serial.println("");
  return n;
}
