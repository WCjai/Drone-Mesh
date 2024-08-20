#define CATCH_CONFIG_MAIN

#include "catch2/catch.hpp"

#include "Arduino.h"

#include "catch_utils.hpp"

#include "boost/asynctcp.hpp"

WiFiClass WiFi;
ESPClass ESP;

#include "painlessmesh/configuration.hpp"

#include "painlessmesh/connection.hpp"
#include "painlessmesh/logger.hpp"

using namespace painlessmesh;

logger::LogClass Log;

SCENARIO("We can send messages") {
  using namespace painlessmesh::logger;
  Log.setLogLevel(ERROR | COMMUNICATION);

  boost::asio::io_service io_service;

  auto pServer = std::make_shared<AsyncServer>(io_service, 6081);

  std::shared_ptr<tcp::BufferedConnection> conn1;
  pServer->onClient([&conn1](void *, AsyncClient *client) {
    conn1 = std::make_shared<tcp::BufferedConnection>(client);
  });
  pServer->begin();

  auto client = new AsyncClient(io_service);
  bool connected = false;
  std::shared_ptr<tcp::BufferedConnection> conn2;
  client->onConnect([&conn2, &connected](void *, AsyncClient *client) {
    conn2 = std::make_shared<tcp::BufferedConnection>(client);
    connected = true;
  });
  client->connect(boost::asio::ip::address::from_string("127.0.0.1"), 6081);

  while (!connected) {
    io_service.poll();
  }

  Scheduler scheduler;
  conn1->initialize(&scheduler);
  std::string last1;

  conn2->initialize(&scheduler);
  std::string last2;

  conn1->onReceive([&last1](std::string msg) { last1 += msg; });
  conn2->onReceive([&last2](std::string msg) { last2 += msg; });

  bool discon1 = false;
  bool discon2 = false;
  conn1->onDisconnect([&discon1]() { discon1 = true; });
  conn2->onDisconnect([&discon2]() { discon2 = true; });

  conn1->write("Blaat");
  conn2->write("Blaat1");
  conn2->write("Blaat3");
  for (auto i = 0; i < 1000; ++i) {
    scheduler.execute();
    io_service.poll();
  }

  REQUIRE(last2 == "Blaat");
  REQUIRE(last1 == "Blaat1Blaat3");
  conn1->close();
  conn2->close();
  REQUIRE(conn1.use_count() == 1);
  REQUIRE(conn2.use_count() == 1);

  for (auto i = 0; i < 1000; ++i) {
    scheduler.execute();
    io_service.poll();
  }

  REQUIRE(discon1);
  REQUIRE(discon2);
}
