/*
 * This file is part of u360gts, aka amv-open360tracker 32bits:
 * https://github.com/raul-ortega/amv-open360tracker-32bits
 *
 * The code below is an adaptation by Ra˙l Ortega of the original code of OpenTX
 * https://github.com/opentx/opentx
 *
 * u360gts is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * u360gts is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with u360gts.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
//#define DEBUG


//#include "config.h"
//#include "telemetry.h"
#define ROM_CRC_H

#define BROADCAST_ADDRESS              0x00
#define RADIO_ADDRESS                  0xEA
#define GPS_ID                         0x02
// int32_t Latitude ( degree / 10`000`000 )
// int32_t Longitude (degree / 10`000`000 )
// #define DECIDEGREES_TO_RADIANS10000(angle) ((int16_t)(1000.0f * (angle) * RAD))
// uint16_t Groundspeed ( km/h / 100 )
// uint16_t GPS heading ( degree / 100 )
// uint16 Altitude ( meter - 1000m offset )
// uint8_t Satellites in use ( counter 
#define BAT_ID                         0x08 
// uint16_t Voltage ( mV * 100 )
// uint16_t Current ( mA * 100 )
// uint24_t Capacity ( mAh )
// uint8_t Battery remaining ( percent )
#define ATTI_ID                        0x1E
#define FMODE_ID                       0x21
//int16_t Pitch angle ( rad / 10000 )
//int16_t Roll angle ( rad / 10000 )
//int16_t Yaw angle ( rad / 10000 
//CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
#define BARO_ALT_ID                    0x09
#define LINK_STAT_ID                  0x14
 /* 0x14 Link statistics
 * Payload:
 *
 * uint8_t Uplink RSSI Ant. 1 ( dBm * -1 )
 * uint8_t Uplink RSSI Ant. 2 ( dBm * -1 )
 * uint8_t Uplink Package success rate / Link quality ( % )
 * int8_t Uplink SNR ( db )
 * uint8_t Diversity active antenna ( enum ant. 1 = 0, ant. 2 )
 * uint8_t RF Mode ( enum 4fps = 0 , 50fps, 150hz)
 * uint8_t Uplink TX Power ( enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000mW )
 * uint8_t Downlink RSSI ( dBm * -1 )
 * uint8_t Downlink package success rate / Link quality ( % )
 * int8_t Downlink SNR ( db )
 * Uplink is the connection from the ground to the UAV and downlink the opposite direction.
 */
#define TELEMETRY_RX_PACKET_SIZE       128

const char *crsfActiveAntenna[2] = {"ANTENNA 1", "ANTENNA 2"};
const char *crsrRfMode[3] = {"4 HZ", "50 HZ", "150_HZ"};
const char *csrfRfPower[7] = {"0 mW", "10 mW", "25 mW", "100 mW", "500 mW", "1000 mW", "2000 mW" };

uint8_t crc8(const uint8_t * ptr, uint32_t len);

uint8_t telemetryRxBuffer[TELEMETRY_RX_PACKET_SIZE];   // Receive buffer. 9 bytes (full packet), worst case 18 bytes with byte-stuffing (+1)
uint8_t telemetryRxBufferCount = 0;
uint8_t posCount = 0;

//VFAS sensor = voltage
uint16_t telemetry_voltage;
uint32_t telemetry_lat;
uint32_t telemetry_lon;
float telemetry_lat_f;
float telemetry_lon_f;
float telemetry_speed;
float telemetry_course;
uint16_t telemetry_alt;
uint16_t telemetry_sats;
int telemetry_failed_cs;
uint16_t telemetry_att_pitch;
uint16_t telemetry_att_roll;
uint16_t telemetry_att_yaw;
uint16_t telemetry_current;
uint32_t telemetry_bat_capacity;
uint16_t telemetry_bat_remain;

uint8_t telemetry_rssi_ant1;
uint8_t telemetry_rssi_ant2;
uint8_t telemetry_rssi_lq;
uint8_t telemetry_rssi_snr;
char* telemetry_rssi_divers;
char* telemetry_rssi_rfmode;
char* telemetry_rssi_rfpower;

char* telemetry_fmode;

int telemetry_fixtype;
bool gotAlt;
bool gotFix;

void CRSF_Decode(uint8_t *data, size_t length) {
  //Serial.println("csrf_d1");
  //uint8_t data[128] 
  //Serial.print("csrf_data1ength: ");
  //Serial.println((uint16_t) length);
  memset(telemetryRxBuffer, 0, sizeof(telemetryRxBuffer));
  
  if ((uint16_t) length > 6) {
    memcpy(telemetryRxBuffer, data, length);
    processCrossfireTelemetryFrame();
  } 
  return;
}

bool checkCrossfireTelemetryFrameCRC()
{
//  Serial.println("csrf_crc1");
  uint8_t len = telemetryRxBuffer[1];
//  Serial.print("csrf_crc1_1");
//  Serial.println((uint8_t) len);
  if ( len > 6 ) {
    uint8_t crc = crc8(&telemetryRxBuffer[2], len-1);

    //unit8_t crc = ~crc8_le(
//    Serial.println("csrf_crc1_2");
    return (crc == telemetryRxBuffer[len+1]);
  }
  return false;
}

bool getCrossfireTelemetryValue(uint8_t N, uint8_t index, int32_t * value)
{
  bool result = false;
  uint8_t byte = telemetryRxBuffer[index];
  *value = (byte & 0x80) ? -1 : 0;
  for (uint8_t i=0; i<N; i++) {
    *value = (*value << 8);
    if (byte != 0xff) {
      result = true;
    }
    *value += telemetryRxBuffer[index++];
  }
  return result;
}

void processCrossfireTelemetryFrame()
{
  //Serial.println("csrf_process1");
  if (!checkCrossfireTelemetryFrameCRC()) {
    telemetry_failed_cs++;
    Serial.println("csrf_crc_failed");
    return;
  }
        
  uint8_t id = telemetryRxBuffer[2];
  int32_t value;
  switch(id) {
    case GPS_ID:
      if (getCrossfireTelemetryValue(4, 3, &value)){
        //uav_lat = int32_t (value / 1E7);
        uav_lat = int32_t (value);
        if(posCount == 0) posCount++;
        #if defined(DEBUG) 
          Serial.printf("Lat: %d\n", uav_lat); 
        #endif
      }
      if (getCrossfireTelemetryValue(4, 7, &value)){
        //uav_lon  = int32_t (value / 1E7);
        uav_lon  = int32_t (value);
        if(posCount == 1) posCount++;
        #if defined(DEBUG) 
          Serial.printf("Lon: %d\n", uav_lon); 
        #endif        
      }
      if (getCrossfireTelemetryValue(2, 11, &value)) {
        uint16_t tmpspeed  = (uint16_t) (value);
        uav_groundspeed  = (int) tmpspeed;
        #if defined(DEBUG) 
          Serial.printf("Speed: %d\n", tmpspeed); 
        #endif
      }
        
      if (getCrossfireTelemetryValue(2, 13, &value)) {
        uav_gpsheading  = (int16_t) (value / 100);
        #if defined(DEBUG)
          Serial.printf("Heading: %d - \n", uav_gpsheading ); 
        #endif
      }
      if (getCrossfireTelemetryValue(2, 15, &value)){
        uav_alt  = (int32_t) ((value - 1000)* 100 );
        gotAlt = true;
        #if defined(DEBUG)
          Serial.printf("Höhe: %d \n", uav_alt  ); 
        #endif        
      }
      if (getCrossfireTelemetryValue(1, 17, &value))
        uav_satellites_visible  = (uint8_t) value;
        #if defined(DEBUG)
          //Serial.printf("Sats: %d \n", telemetry_alt ); 
          Serial.printf("Sats/Lat/Lon/Alt: %d %d %d %d\n", uav_satellites_visible , uav_lat, uav_lon, uav_alt);
        #endif        
      
      break;

    case BAT_ID:
      if (getCrossfireTelemetryValue(2, 3, &value))
        uav_bat  = (uint16_t) (value * 100);
      if (getCrossfireTelemetryValue(2, 5, &value))
        uav_current  = (uint16_t) (value * 100);
      if (getCrossfireTelemetryValue(3, 7, &value))
        uav_amp  = (uint16_t) value;
      if (getCrossfireTelemetryValue(1, 10, &value))
        telemetry_bat_remain = (uint16_t) value;
      #if defined(DEBUG)
        Serial.printf("Volt/Current/Capac/Remain: %d %d %d %d\n", uav_bat , uav_current , uav_amp, telemetry_bat_remain);
      #endif  

      break;

     case ATTI_ID:
       //Serial.printf("atti");
       if (getCrossfireTelemetryValue(2, 3, &value))
        uav_pitch  = (int16_t) (value / 10);
       if (getCrossfireTelemetryValue(2, 5, &value))
        uav_roll  = (int16_t) (value / 10);
       if (getCrossfireTelemetryValue(2, 7, &value))
        uav_heading  = (int16_t) (value / 10);
       #if defined(DEBUG)        
         Serial.printf("Pit/Rol/Yaw: %d %d %d\n", telemetry_att_pitch, telemetry_att_roll, telemetry_att_yaw);
       #endif
       break;
       

     case LINK_STAT_ID:
       //Serial.println("Link");
       if (getCrossfireTelemetryValue(1, 3, &value))
        //uint8_t rssinegdbm  = (uint8_t) (value - 1);
        //uint8_t rssitmp =  (100 - (rssinegdbm / 130))
        //uav_rssi  = (uint8_t) (value - 1);
        uav_rssi = (uint8_t) (100 - ((value - 1) / 130));
       if (getCrossfireTelemetryValue(1, 4, &value))
        telemetry_rssi_ant2 = (uint8_t) (value - 1);
       if (getCrossfireTelemetryValue(1, 5, &value))
        uav_linkquality  = (uint8_t) (value);
       if (getCrossfireTelemetryValue(1, 6, &value))
        telemetry_rssi_snr = (int8_t) (value);
       if (getCrossfireTelemetryValue(1, 7, &value)) {
          uint8_t rfantidx = (uint8_t) (value);
          telemetry_rssi_divers = (char*) crsfActiveAntenna[rfantidx];  
       }
       if (getCrossfireTelemetryValue(1, 8, &value)) {
          uint8_t rfidx = (uint8_t) (value);
          telemetry_rssi_rfmode = (char*) crsrRfMode[rfidx];  
       }
       if (getCrossfireTelemetryValue(1, 9, &value)) {
          uint8_t rfpowidx = (uint8_t) (value);
          telemetry_rssi_rfpower = (char*) csrfRfPower[rfpowidx];  
       }

       #if defined(DEBUG)        
         Serial.printf("Ant1/Ant2/LQ/SNR: %d %d %d  %d ", uav_rssi, telemetry_rssi_ant2, uav_linkquality, telemetry_rssi_snr);
         //Serial.println(telemetry_rssi_divers);
         Serial.printf("SNR/Dive/: %d %d %d\n", telemetry_rssi_snr, telemetry_rssi_divers);
         Serial.printf("RfMode: %s  RfPower: %s\n", telemetry_rssi_rfmode, telemetry_rssi_rfpower);
       #endif
      break;


     case FMODE_ID:
        if (getCrossfireTelemetryValue(2, 3, &value)) {
          auto textLength = min<int>(16, telemetryRxBuffer[1]);
          telemetryRxBuffer[textLength] = '\0';
          telemetry_fmode = (char*) telemetryRxBuffer + 3;
          #if defined(DEBUG)                
            Serial.print("fmode: ");
            Serial.println(telemetry_fmode);
          #endif
          
          if (strcmp(telemetry_fmode, "Manual") == 0) {
            uav_flightmode = 0;
          } else if (strcmp(telemetry_fmode, "Rate") == 1) {
            uav_flightmode = 1;
          } else if (strcmp(telemetry_fmode, "Attitude/Angle") == 2) {
            uav_flightmode = 2;
          } else if (strcmp(telemetry_fmode, "Horizon") == 3) {
            uav_flightmode = 3;
          } else if (strcmp(telemetry_fmode, "Acro") == 4) {
            uav_flightmode = 4;
          } else if (strcmp(telemetry_fmode, "Stabilized 1") == 5) {
            uav_flightmode = 5;
          } else if (strcmp(telemetry_fmode, "Stabilized 2") == 6) {
            uav_flightmode = 6;                        
          } else if (strcmp(telemetry_fmode, "Stabilized 3") == 7) {
            uav_flightmode = 7;
          } else if (strcmp(telemetry_fmode, "Altitude Hold") == 8) {
            uav_flightmode = 8;
          } else if (strcmp(telemetry_fmode, "Loiter/GPS Hold") == 9) {
            uav_flightmode = 9;
          } else if (strcmp(telemetry_fmode, "Auto/Waypoints") == 10) {
            uav_flightmode = 10;
          } else if (strcmp(telemetry_fmode, "Heading Hold/Head Free") == 11) {
            uav_flightmode = 11;
          } else if (strcmp(telemetry_fmode, "Circle") == 12) {
            uav_flightmode = 12;
          } else if (strcmp(telemetry_fmode, "RTH") == 13) {
            uav_flightmode = 13;
          } else if (strcmp(telemetry_fmode, "Follow Me") == 14) {
            uav_flightmode = 14;
          } else if (strcmp(telemetry_fmode, "Land") == 15) {
            uav_flightmode = 15;
          } else if (strcmp(telemetry_fmode, "FlybyWire A") == 16) {
            uav_flightmode = 16;   
          } else if (strcmp(telemetry_fmode, "FlybyWire B") == 17) {
            uav_flightmode = 17;
          } else if (strcmp(telemetry_fmode, "Cruise") == 18) {
            uav_flightmode = 18;
          } else if (strcmp(telemetry_fmode, "Unknown") == 19) {
            uav_flightmode = 19;
          } else {
            uav_flightmode = 19;
          }


          #if defined(DEBUG)                
            //Serial.printf("FMode/LTM-FMode: %s %d\n",telemetry_fmode, uav_flightmode);
          #endif
        }
        break;
    }

  if (uav_satellites_visible == 0) { 
    uav_fix_type = 0; 
  }
  else if (uav_satellites_visible < 5 && uav_satellites_visible > 0) {
     uav_fix_type = 2;
     }
  else {
     uav_fix_type = 3;
     } 
  
  if(posCount == 2 ) {
    posCount = 0;
    gotFix = true;
    //Serial.printf("Sats/Lat/Lon/Alt/Spd/Heading: %d %f %f %d %.2f %.2f\n", telemetry_sats, telemetry_lat_f, telemetry_lon_f, telemetry_alt, telemetry_speed, telemetry_course);
  }
}

void crossfire_encodeTargetData(uint8_t data)
{

  if (telemetryRxBufferCount == 0 && data != RADIO_ADDRESS) {
    Serial.printf("address 0x%02X error\n", data);
    return;
  }

  if (telemetryRxBufferCount == 1 && (data < 2 || data > TELEMETRY_RX_PACKET_SIZE-2)) {
    Serial.printf("length 0x%02X error\n", data);
    telemetryRxBufferCount = 0;
    return;
  }

  if (telemetryRxBufferCount < TELEMETRY_RX_PACKET_SIZE) {
    telemetryRxBuffer[telemetryRxBufferCount++] = data;
  }
  else {
    Serial.printf("array size %d error\n", telemetryRxBufferCount);
    telemetryRxBufferCount = 0;
  }

  if (telemetryRxBufferCount > 4) {
    uint8_t length = telemetryRxBuffer[1];
    if (length + 2 == telemetryRxBufferCount) {
      processCrossfireTelemetryFrame();
      telemetryRxBufferCount = 0;
    }
  }
}

// CRC8 implementation with polynom = x^8+x^7+x^6+x^4+x^2+1 (0xD5)
const unsigned char crc8tab[256] = {
  0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54,
  0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
  0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
  0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
  0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0,
  0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
  0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2,
  0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
  0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
  0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
  0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B,
  0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
  0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D,
  0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
  0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
  0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
  0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB,
  0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
  0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9,
  0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
  0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
  0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
  0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D,
  0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
  0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26,
  0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
  0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,
  0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
  0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82,
  0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
  0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0,
  0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};

uint8_t crc8(const uint8_t *ptr, uint32_t len)
{
  uint8_t crc = 0;
  for (uint32_t i=0; i<len; i++) {
    crc = crc8tab[crc ^ *ptr++];
  }
  return crc;
}


