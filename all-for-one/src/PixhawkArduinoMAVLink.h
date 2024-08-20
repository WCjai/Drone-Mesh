/*
  PixhawkArduinoMAVLink.h - Library for using Arduino to recieve Pixhawk sensors data.
  Created by Shashi Kant, June 23, 2018.
  Using MAVLink C headers files generated from the ardupilotmega.xml with the help of mavgenerator.
*/

#ifndef PixhawkArduinoMAVLink_h
#define PixhawkArduinoMAVLink_h

#include "ardupilotmega/mavlink.h"
#include <checksum.h>
#include <mavlink_types.h>
#include <protocol.h>
#include <Arduino.h>

#include <HardwareSerial.h>

class PixhawkArduinoMAVLink
{
  public:
    PixhawkArduinoMAVLink(HardwareSerial &hs);
    bool begin();
    void ReadAcceleration(float *xacc, float *yacc, float *zacc);
    void ReadBattery(int *xbatt, int *xid);
    void FakeGPS(double lat, double lon,float alt, uint32_t time_week, uint32_t time_week_ms, uint64_t time_usec);
    void HilGPS(int fixType, double lat, double lon, float alt, int satellitesVisible);
    void ReadLocalpos(uint32_t *xtime_boot_ms, float *x_x, float *x_y);
    void ChangeMODE();
    void GPSsetOrigin();
    void BatteryReset(int energy);
    void Stream();
    

  private:
    HardwareSerial* _MAVSerial;
    double MILLIG_TO_MS2;
    uint8_t system_id;
    uint8_t component_id;
    uint8_t type;
    uint8_t autopilot;
    uint8_t received_sysid; // Pixhawk sysid
    uint8_t received_compid; // Pixhawk compid
};

#endif
