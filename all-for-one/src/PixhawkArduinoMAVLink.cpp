/*
  PixhawkArduinoMAVLink.cpp - Library for using Arduino to recieve Pixhawk's sensor data as well as some other usefull data which you might need.
  Created by Shashi Kant, June 23, 2018.
  Using MAVLink C headers files generated from the ardupilotmega.xml with the help of mavgenerator.
*/

#include "PixhawkArduinoMAVLink.h"

PixhawkArduinoMAVLink::PixhawkArduinoMAVLink(HardwareSerial &hs){
  _MAVSerial = &hs;
  MILLIG_TO_MS2 = 9.80665 / 1000.0;
  system_id = 10; // Your i.e. Arduino sysid
  component_id = 161; // Your i.e. Arduino compid
  type = MAV_TYPE_QUADROTOR;
  autopilot = MAV_AUTOPILOT_GENERIC;
}

bool PixhawkArduinoMAVLink::begin(){
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  if(Serial2.available() >= 1){
    return 0;
  }else{
    return 1;
  }
}



// At first we will send some HeartBeats to Pixhawk to check whether it's available or not??
// After that we will check for whether we are recieving HeartBeats back from Pixhawk if Yes,
// We will note down its sysid and compid to send it a req to Stream Data.
void PixhawkArduinoMAVLink::Stream(){
  delay(2000);
  int flag=1;
  Serial.println("Sending Heartbeats...");
  mavlink_message_t msghb;
  mavlink_heartbeat_t heartbeat;
  uint8_t bufhb[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_heartbeat_pack(system_id, component_id, &msghb, type, autopilot, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0, MAV_STATE_STANDBY);
  uint16_t lenhb = mavlink_msg_to_send_buffer(bufhb, &msghb);
  delay(1000);
  //_MAVSerial->write(bufhb,lenhb);
  Serial2.write(bufhb,lenhb);
  Serial.println("Heartbeats sent! Now will check for recieved heartbeats to record sysid and compid...");

  // Looping untill we get the required data.
  while(flag==1){
    delay(1);
    //while(_MAVSerial->available()>0){
    while(Serial2.available() >= 1){
      mavlink_message_t msgpx;
      mavlink_status_t statuspx;
      // uint8_t ch = _MAVSerial->read();
      uint8_t ch = Serial2.read();
      if(mavlink_parse_char(MAVLINK_COMM_0, ch, &msgpx, &statuspx)){
        Serial.println("Message Parsing Done!");
        switch(msgpx.msgid){
          case MAVLINK_MSG_ID_HEARTBEAT:
          {
            mavlink_heartbeat_t packet;
            mavlink_msg_heartbeat_decode(&msgpx, &packet);
            received_sysid = msgpx.sysid; // Pixhawk sysid
            received_compid = msgpx.compid; // Pixhawk compid
            Serial.println(received_sysid);
            Serial.println(received_compid);
            Serial.println("sysid and compid successfully recorded");
            flag = 0;
            break;
          }
        }
      }
    }
  }

  // Sending request for data stream...
  Serial.println("Now sending request for data stream...");
  delay(2000);
  mavlink_message_t msgds;
  uint8_t bufds[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_request_data_stream_pack(system_id, component_id, &msgds, received_sysid, received_compid, MAV_DATA_STREAM_ALL , 0x05, 1);
  uint16_t lends = mavlink_msg_to_send_buffer(bufds, &msgds);
  delay(10);
  //_MAVSerial->write(bufds,lends);
  Serial2.write(bufds,lends);
  Serial.println("Request sent! Now you are ready to recieve datas...");


}

// uint64_t getTimestampMicros() {
//     return millis() * 1000ULL; // Convert milliseconds to microseconds
// }
void PixhawkArduinoMAVLink::FakeGPS(double lat, double lon, float alt, uint32_t time_week, uint32_t time_week_ms, uint64_t time_usec) {
    mavlink_message_t msggps;
    mavlink_gps_input_t gpsData;
    //long unsigned int time_to_boot = getTimestampMicros();
    gpsData.time_usec = time_usec;
    gpsData.time_week_ms = time_week_ms; /*< [ms] GPS time (from start of GPS week)*/
    gpsData.lat = lat * 1e7; /*< [degE7] Latitude (WGS84)*/
    gpsData.lon = lon * 1e7; /*< [degE7] Longitude (WGS84)*/
    gpsData.alt = alt; /*< [m] Altitude (MSL). Positive for up.*/
    gpsData.hdop = 0.6; /*<  GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX*/
    gpsData.vdop = 0.6; /*<  GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX*/
    gpsData.vn = 0; /*< [m/s] GPS velocity in north direction in earth-fixed NED frame*/
    gpsData.ve = 0; /*< [m/s] GPS velocity in east direction in earth-fixed NED frame*/
    gpsData.vd = 0; /*< [m/s] GPS velocity in down direction in earth-fixed NED frame*/
    gpsData.speed_accuracy = 0; /*< [m/s] GPS speed accuracy*/
    gpsData.horiz_accuracy = 0; /*< [m] GPS horizontal accuracy*/
    gpsData.vert_accuracy = 0; /*< [m] GPS vertical accuracy*/
    gpsData.ignore_flags = 248; /*<  Bitmap indicating which GPS input flags fields to ignore.  All other fields must be provided.*/
    gpsData.time_week = time_week; /*<  GPS week number*/
    gpsData.gps_id = 1; /*<  ID of the GPS for multiple GPS inputs*/
    gpsData.fix_type = 5; /*<  0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK*/
    gpsData.satellites_visible = 24;
    gpsData.yaw = 0;  /*< [cdeg] Yaw of vehicle relative to Earth's North, zero means not available, use 36000 for north*/

    uint8_t bufgps[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_gps_input_encode(received_sysid, received_compid, &msggps, &gpsData);
    uint16_t lengps = mavlink_msg_to_send_buffer(bufgps, &msggps);

    size_t bytes_written = Serial2.write(bufgps, lengps);

    // Error handling to ensure all bytes are written
    if (bytes_written != lengps) {
        Serial.println("Error: GPS data not fully sent.");
    } else {
        Serial.println("GPS data sent successfully.");
    }

    // Ensure all data is transmitted
    Serial2.flush();
}



 void PixhawkArduinoMAVLink::HilGPS(int fixType, double lat, double lon, float alt, int satellitesVisible) {

     //Serial.println("Sending FakeGPS...");
     mavlink_message_t msghilgps;
     mavlink_hil_gps_t gpshilData;
     // long unsigned int time_to_boot = getTimestampMicros();
     //gpshilData.time_usec = time_to_boot; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
     gpshilData.lat = (lat) * 1e7; /*< [degE7] Latitude (WGS84)*/
     gpshilData.lon = (lon) * 1e7; /*< [degE7] Longitude (WGS84)*/
     gpshilData.alt = alt; /*< [mm] Altitude (MSL). Positive for up.*/
     gpshilData.eph = 1; /*<  GPS HDOP horizontal dilution of position (unitless * 100). If unknown, set to: UINT16_MAX*/
     gpshilData.epv = 1; /*<  GPS VDOP vertical dilution of position (unitless * 100). If unknown, set to: UINT16_MAX*/
     gpshilData.vel = UINT16_MAX; /*< [cm/s] GPS ground speed. If unknown, set to: UINT16_MAX*/
     gpshilData.vn = 0; /*< [cm/s] GPS velocity in north direction in earth-fixed NED frame*/
     gpshilData.ve = 0; /*< [cm/s] GPS velocity in east direction in earth-fixed NED frame*/
     gpshilData.vd = 0; /*< [cm/s] GPS velocity in down direction in earth-fixed NED frame*/
     gpshilData.cog = UINT16_MAX; /*< [cdeg] Course over ground (NOT heading, but direction of movement), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
     gpshilData.fix_type = 3; /*<  0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.*/
     gpshilData.satellites_visible = UINT8_MAX; /*<  Number of satellites visible. If unknown, set to UINT8_MAX*/

     uint8_t bufhilgps[MAVLINK_MAX_PACKET_LEN];
     mavlink_msg_hil_gps_encode(received_sysid, received_compid, &msghilgps, &gpshilData);
     uint16_t lenhilgps = mavlink_msg_to_send_buffer(bufhilgps, &msghilgps);

     Serial2.write(bufhilgps, lenhilgps);
     //Serial.println("sented gps");

 }


void PixhawkArduinoMAVLink::ChangeMODE() {

    Serial.println("Sending mode change request");
    mavlink_message_t msgmode;
    mavlink_set_mode_t modeData;
    modeData.custom_mode = 4;
    modeData.base_mode =  5;
    modeData.target_system = 1;

    uint8_t bufmode[MAVLINK_MAX_PACKET_LEN];
   // mavlink_msg_set_gps_global_origin_encode(system_id, component_id, &msgmode, &modeData, &modeData)
    mavlink_msg_set_mode_encode(received_sysid, received_compid, &msgmode, &modeData );
    uint16_t lenmode = mavlink_msg_to_send_buffer(bufmode, &msgmode);

    //_MAVSerial->write(bufmode, lenmode);
    Serial2.write(bufmode,lenmode);
    Serial.println("sented mode change");

}

void PixhawkArduinoMAVLink::GPSsetOrigin() {

    Serial.println("Setting origin");
    mavlink_message_t msgorgin;
    mavlink_set_gps_global_origin_t gpsoriginData;

    gpsoriginData.latitude =  (-35.35472512) * 1e7;
    gpsoriginData.longitude = (149.15396143) * 1e7;
    gpsoriginData.altitude = 0;
    gpsoriginData.target_system = 1;
    //long unsigned int time_to_boot = getTimestampMicros();
    long unsigned int time_to_boot = 0;  // need rtc
    //gpsoriginData.time_usec = time_to_boot;

    uint8_t bufgpsorigin[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_set_gps_global_origin_encode(system_id, component_id, &msgorgin, &gpsoriginData);
    //mavlink_msg_set_mode_encode(received_sysid, received_compid, &msgmode, &modeData );
    uint16_t lengpsorigin = mavlink_msg_to_send_buffer(bufgpsorigin, &msgorgin);
  
    //_MAVSerial->write(bufmode, lenmode);
    Serial2.write(bufgpsorigin, lengpsorigin);
    Serial.println("sending origin set");

}


void PixhawkArduinoMAVLink::BatteryReset(int energy) {
    Serial.println("Sending battery change request");
    mavlink_message_t msgbattery;
    mavlink_command_long_t batteryData;
    batteryData.param1 = 2; /*<bit mask 1-reset battery 1, 2- reset battery 2, 3- reset both battery*/
    batteryData.param2 = energy; /*< persentage consumed*/
    batteryData.command = 42651;
    batteryData.target_system = received_sysid; /*<  System which should execute the command*/
    batteryData.target_component = 0; /*<  Component which should execute the command, 0 for all components*/
    batteryData.confirmation = 255; /*<  0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)*/

    uint8_t bufbattery[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_command_long_encode(received_sysid, received_compid, &msgbattery, &batteryData);
    uint16_t lenbattery = mavlink_msg_to_send_buffer(bufbattery, &msgbattery);
    Serial2.write(bufbattery, lenbattery);
    Serial.println("sented mode change");

}


void PixhawkArduinoMAVLink::ReadBattery(int *xbatt, int *xid){
  int flagI = 1;
  int flagA = 1;
  int battery_remaining;
  int id;

  while((flagI==1)||(flagA==1)){
    while(_MAVSerial->available() > 0){
      //Serial.println("decode");
      mavlink_message_t msg;
      mavlink_status_t status1;
      uint8_t ch = Serial2.read();
      if(mavlink_parse_char(MAVLINK_COMM_0, ch, &msg, &status1)){

        //Serial.println(msg.msgid);
        switch(msg.msgid){
          case MAVLINK_MSG_ID_BATTERY_STATUS:
           //case 163:
            {
              //Serial.println("Sending battery status");
              mavlink_battery_status_t data;
              mavlink_msg_battery_status_decode(&msg, &data);
              battery_remaining = (data.battery_remaining);
              id = (data.id);
              flagI = 0;
              flagA = 0;
              break;
            }
        }
      }
    }
  }

  *xbatt = battery_remaining;
  *xid = id;

  return;
}

void PixhawkArduinoMAVLink::ReadLocalpos(uint32_t *xtime_boot_ms, float *x_x, float *x_y){
  int flagI = 1;
  int flagA = 1;
  uint32_t time_boot_ms;
  float x;
  float y;

  while((flagI==1)||(flagA==1)){
    while(Serial2.available() > 0){
      mavlink_message_t msg;
      mavlink_status_t status1;
      uint8_t ch = Serial2.read();
      if(mavlink_parse_char(MAVLINK_COMM_0, ch, &msg, &status1)){

        //Serial.println(msg.msgid);
        switch(msg.msgid){
          case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
           //case 163:
            {
              //Serial.println("Sending battery status");
              mavlink_local_position_ned_t data;
              mavlink_msg_local_position_ned_decode(&msg, &data);
              time_boot_ms = (data.time_boot_ms);
              x = (data.x);
              y = (data.y);
              flagI = 0;
              flagA = 0;
              break;
            }
        }
      }
    }
  }

  *x_x = x;
  *xtime_boot_ms = time_boot_ms;
  *x_y = y;

  return;
}

void PixhawkArduinoMAVLink::ReadAcceleration(float *xacc, float *yacc, float *zacc){
  int flagI = 1;
  int flagA = 1;
  float xa, ya, za, q0, q1, q2, q3;

  while((flagI==1)||(flagA==1)){

    while(_MAVSerial->available() > 0){
      //Serial.println("decode");
      mavlink_message_t msg;
      mavlink_status_t status1;
      uint8_t ch = Serial2.read();
      if(mavlink_parse_char(MAVLINK_COMM_0, ch, &msg, &status1)){

        //Serial.println(msg.msgid);
        switch(msg.msgid){
          case MAVLINK_MSG_ID_ATTITUDE:
           //case 163:
            {
              //Serial.println("Sending Highres IMU Data");
              mavlink_attitude_t data;
              mavlink_msg_attitude_decode(&msg, &data);
              xa = (data.roll);
              ya = (data.pitch);
              za = (data.yaw);
              flagI = 0;
              flagA = 0;
              break;
            }
          case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
            {
              //Serial.println("Sending Quaternion Attitude");
              mavlink_attitude_quaternion_t data;
              mavlink_msg_attitude_quaternion_decode(&msg, &data);
              q0 = data.q1;
              q1 = data.q2;
              q2 = data.q3;
              q3 = data.q4;
              flagA = 0;
              break;
            }
        }
      }
    }
  }

  *xacc = xa;
  *yacc = ya;
  *zacc = za;

  return;
}