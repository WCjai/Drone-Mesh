#include "Arduino.h"
#ifndef _PAINLESS_MESH_CONFIGURATION_HPP_
#define _PAINLESS_MESH_CONFIGURATION_HPP_

#include <list>

#include "painlessTaskOptions.h"

#include <TaskSchedulerDeclarations.h>

#define ARDUINOJSON_USE_LONG_LONG 1
#undef ARDUINOJSON_ENABLE_STD_STRING
#include <ArduinoJson.h>
#undef ARDUINOJSON_ENABLE_STD_STRING

// Enable (arduino) wifi support
#define PAINLESSMESH_ENABLE_ARDUINO_WIFI

// Enable OTA support
#define PAINLESSMESH_ENABLE_OTA

// Minimum free memory, besides here all packets in queue are discarded.
#define MIN_FREE_MEMORY 4000
// MAX number of unsent messages in queue. Newer messages are discarded
#define MAX_MESSAGE_QUEUE 50

#define NODE_TIMEOUT 10 * TASK_SECOND
#define SCAN_INTERVAL 30 * TASK_SECOND  // AP scan period in ms

#ifdef ESP32
#include <AsyncTCP.h>
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#endif  // ESP32

typedef String TSTRING;

// backward compatibility
template <typename T>
using SimpleList = std::list<T>;

namespace painlessmesh {
namespace wifi {
class Mesh;
};
};  // namespace painlessmesh

/** A convenience typedef to access the mesh class*/
#ifdef PAINLESSMESH_ENABLE_ARDUINO_WIFI
using painlessMesh = painlessmesh::wifi::Mesh;
#endif

#ifdef ESP32
#define MAX_CONN 10
#else
#define MAX_CONN 4
#endif  // DEBUG

#endif
