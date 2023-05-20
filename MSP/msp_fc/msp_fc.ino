#include <stdint.h>
#include <stdbool.h>
#include <EEPROM.h>
#include "msp_protocol.h"
#include "variables.h"


#define RX PC7
#define TX PC6


// -------------------------------------
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#define FC_FIRMWARE_NAME            "Betaflight"
#define FC_VERSION_MAJOR            4  // increment when a major release is made (big new feature, etc)
#define FC_VERSION_MINOR            1  // increment when a minor release is made (small new feature, change etc)
#define FC_VERSION_PATCH_LEVEL      0  // increment when a bug is fixed

extern const char* const targetName;

#define GIT_SHORT_REVISION_LENGTH   7 // lower case hexadecimal digits.
extern const char* const shortGitRevision;

#define BUILD_DATE_LENGTH 11
extern const char* const buildDate;  // "MMM DD YYYY" MMM = Jan/Feb/...

#define BUILD_TIME_LENGTH 8
extern const char* const buildTime;  // "HH:MM:SS"

#define MSP_API_VERSION_STRING STR(API_VERSION_MAJOR) "." STR(API_VERSION_MINOR)

//----------------------------------------

void setup() {
  // // Serial.begin(115200);     // // Serial port for debugging
  Serial.begin(115200);
  pinMode(PC8 , OUTPUT);
}

// Function to calculate MSP checksum
uint8_t calculateChecksum(uint8_t *data, uint8_t length) {
  uint8_t checksum = 0;
  for (uint8_t i = 3; i < length; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

// Function to send MSP packet
void sendPacket(uint8_t *data, uint8_t length)
{
  for(uint8_t i = 0; i < length; i++)
  {
    Serial.write(data[i]);
  }
}

//------------------------------------------------------------

void send_MSP_API_VERSION()
{
  packet[0] = '$';
  packet[1] = 'M';
  packet[2] = '>';
  packet[3] = 3;
  packet[4] = MSP_API_VERSION;
  packet[5] = MSP_PROTOCOL_VERSION;
  packet[6] = API_VERSION_MAJOR;
  packet[7] = API_VERSION_MINOR;
  packet[8] = calculateChecksum(packet, 8);

  sendPacket(packet, 9);
}

void send_MSP_FC_VARIANT()
{
  packet[0] = '$';
  packet[1] = 'M';
  packet[2] = '>';
  packet[3] = 4;
  packet[4] = MSP_FC_VARIANT;
  packet[5] = 0x42;
  packet[6] = 0x54;
  packet[7] = 0x46;
  packet[8] = 0x4c;
  packet[9] = calculateChecksum(packet, 9);

  sendPacket(packet, 10);
}

void send_MSP_FC_VERSION()
{
  packet[0] = '$';
  packet[1] = 'M';
  packet[2] = '>';
  packet[3] = 3;
  packet[4] = MSP_FC_VERSION;
  packet[5] = FC_VERSION_MAJOR;
  packet[6] = FC_VERSION_MINOR;
  packet[7] = FC_VERSION_PATCH_LEVEL;
  packet[8] = calculateChecksum(packet, 8);

  sendPacket(packet, 9);
}

void send_MSP_BOARD_INFO()
{
  packet[0] = '$';
  packet[1] = 'M';
  packet[2] = '>';
  packet[3] = 0x4f;
  packet[4] = MSP_BOARD_INFO;
  packet[5] = 'B';
  packet[6] = 'P';
  packet[7] = 'M';
  packet[8] = 'C';
  packet[9] = 0x00;    // HW Revision
  packet[10]= 0x00;   // HW Revision
  packet[11]= 0x00;   // FC w/wo MAX7456
  packet[12]= 0x00;   // targetCapabilities
  packet[13]= 0x14;   // Name String length
  packet[14]= 'B';    // Name.....
  packet[15]= 'r';
  packet[16]= 'u';
  packet[17]= 's';
  packet[18]= 'h';
  packet[19]= 'l';
  packet[20]= 'e';
  packet[21]= 's';
  packet[22]= 's';
  packet[23]= 'P';
  packet[24]= 'o';
  packet[25]= 'w';
  packet[26]= 'e';
  packet[27]= 'r';
  packet[28]= ' ';
  packet[29]= 'M';
  packet[30]= 'C';
  packet[31]= 'T';
  packet[32]= 'R';
  packet[33]= 'L';
  packet[34]= 0x00;
  packet[35]= 0x00;
  packet[36]= 0x00;
  packet[37]= 0x00;
  packet[38]= 0x00;
  packet[39]= 0x00;
  packet[40]= 0x00;
  packet[41]= 0x00;
  packet[42]= 0x00;
  packet[43]= 0x00;
  packet[44]= 0x00;
  packet[45]= 0x00;
  packet[46]= 0x00;
  packet[47]= 0x00;
  packet[48]= 0x00;
  packet[49]= 0x00;
  packet[50]= 0x00;
  packet[51]= 0x00;
  packet[52]= 0x00;
  packet[53]= 0x00;
  packet[54]= 0x00;
  packet[55]= 0x00;
  packet[56]= 0x00;
  packet[57]= 0x00;
  packet[58]= 0x00;
  packet[59]= 0x00;
  packet[60]= 0x00;
  packet[61]= 0x00;
  packet[62]= 0x00;
  packet[63]= 0x00;
  packet[64]= 0x00;
  packet[65]= 0x00;
  packet[66]= 0x00;
  packet[67]= 0x00;
  packet[68]= 0x00;
  packet[69]= 0x00;
  packet[70]= 0x00;
  packet[71]= 0x00;
  packet[72]= 0x00;
  packet[73]= 0x00;
  packet[74]= 0x00;   // getMcuTypeId
  packet[75]= 0x00;   // configurationState
  packet[76]= 0x00;   // Gyro
  packet[77]= 0x00;   // Gyro
  packet[78]= 0x00;   // configurationProblems
  packet[79]= 0x00;   // configurationProblems
  packet[80]= 0x00;   // configurationProblems
  packet[81]= 0x00;   // configurationProblems
  packet[82]= 0x00;   // SPI
  packet[83]= 0x00;   // I2C
  packet[84] = calculateChecksum(packet, 84);

  sendPacket(packet, 85);
  // printpacket(packet,85);
}

void send_MSP_BUILD_INFO()
{
  packet[0] = '$';
  packet[1] = 'M';
  packet[2] = '>';
  packet[3] = 3;
  packet[4] = MSP_BUILD_INFO;
  packet[5] = 4;
  packet[6] = 1;
  packet[7] = 0;
  packet[8] = calculateChecksum(packet,8);

  sendPacket(packet, 9);
}

void send_MSP_ANALOG()
{
  packet[0] = '$';
  packet[1] = 'M';
  packet[2] = '>';    // Direction as in response. May be absent so needs to be removed in that case.
  packet[3] = sizeof(VBAT) + sizeof(INT_POWER_METER_SUM) + sizeof(RSSI) + sizeof(AMPERAGE);
  packet[4] = MSP_ANALOG;
  packet[5] = VBAT;

  tv = INT_POWER_METER_SUM;
  packet[6] = tv & 0xff;
  tv = tv >> 8;
  packet[7] = tv & 0xff;

  tv = RSSI;
  packet[8] = tv & 0xff;
  tv = tv >> 8;
  packet[9] = tv & 0xff;

  tv = AMPERAGE;
  packet[10] = tv & 0xff;
  tv = tv >> 8;
  packet[11] = tv & 0xff;

  packet[12] = calculateChecksum(packet, 12);

  sendPacket(packet, 13);
}

// Explicit function meant for later design
// Source Betaflight github repo
// void send_MSP_DEBUG()
// {
//   packet[0] = '$';
//   packet[1] = 'M';
//   packet[2] = '>';    // Direction as in response. May be absent so needs to be removed in that case.
//   packet[3] = DEBUG16_VALUE_COUNT;
//   packet[4] = MSP_DEBUG;
//
//   for (int i = 0; i < DEBUG16_VALUE_COUNT; i++)
//   {
//     packet[2*i+5] = debug[i] & 0xff;
//     tv = debug[i];
//     tv = tv >> 8;
//     packet[2*i+6] = tv & 0xff;
//   }
//
//   packet[2*DEBUG16_VALUE_COUNT + 5] = calculateChecksum(packet, 2*DEBUG16_VALUE_COUNT + 5);
//
//   sendPacket(packet, 2*DEBUG16_VALUE_COUNT + 6);
// }

void send_MSP_UID()
{
  packet[0] = '$';
  packet[1] = 'M';
  packet[2] = '>';
  packet[3] = 6;
  packet[4] = MSP_UID;
  

  tv = uint32_t(U_ID_0);
  packet[5] = tv & 0xff;
  tv = tv >> 8;
  packet[6] = tv & 0xff;
  tv = tv >> 8;
  packet[7] = tv & 0xff;
  tv = tv >> 8;
  packet[8] = tv & 0xff;

  tv = uint32_t(U_ID_1);
  packet[9] = tv & 0xff;
  tv = tv >> 8;
  packet[10] = tv & 0xff;
  tv = tv >> 8;
  packet[11] = tv & 0xff;
  tv = tv >> 8;
  packet[12] = tv & 0xff;

  tv = uint32_t(U_ID_2);
  packet[13] = tv & 0xff;
  tv = tv >> 8;
  packet[14] = tv & 0xff;
  tv = tv >> 8;
  packet[15] = tv & 0xff;
  tv = tv >> 8;
  packet[16] = tv & 0xff;

  packet[17] = calculateChecksum(packet, 17);

  sendPacket(packet, 18);
}

// below uptil line are inaccordance with betaflight

void send_MSP_FEATURE_CONFIG() // incomplete
{

}

void send_MSP_BEEPER_CONFIG() // incomplete
{}

void send_MSP_BATTERY_STATE() // incomplete
{}

void send_MSP_VOLTAGE_METERS() // incomplete
{}

void send_MSP_CURRENT_METERS() // incomplete
{}

void send_MSP_VOLTAGE_METER_CONFIG() // incomplete
{}

void send_MSP_CURRENT_METER_CONFIG() // incomplete
{}

void send_MSP_BATTERY_CONFIG() // incomplete
{

}

void send_MSP_TRANSPONDER_CONFIG() // incomplete
{}

// ---------------------------------

void send_MSP_SET_RX_MAP()
{
  packet[0] = '$';
  packet[1] = 'M';
  packet[2] = '<';
  packet[3] = 8;
  packet[4] = MSP_SET_RX_MAP;
  packet[5] = mapping[0];
  packet[6] = mapping[1];
  packet[7] = mapping[2];
  packet[8] = mapping[3];
  packet[9] = mapping[4];
  packet[10] = mapping[5];
  packet[11] = mapping[6];
  packet[12] = mapping[7];
  packet[13] = calculateChecksum(packet,13);
  sendPacket(packet, 14);
}

void send_MSP_STATUS()
{
  packet[0] = '$';
  packet[1] = 'M';
  packet[2] = '>';    // Direction as in response. May be absent so needs to be removed in that case.
  packet[3] = sizeof(CYCLETIME) + sizeof(I2C_ERRORS_COUNT) + sizeof(SENSOR) + sizeof(FLAG) + sizeof(GLOBAL_CONF_CURRENT_SET);
  packet[4] = MSP_STATUS;

  tv = CYCLETIME;
  packet[5] = tv & 0xff;
  tv = tv >> 8;
  packet[6] = tv & 0xff;

  tv = I2C_ERRORS_COUNT;
  packet[7] = tv & 0xff;
  tv = tv >> 8;
  packet[8] = tv & 0xff;

  tv = SENSOR;
  packet[9] = tv & 0xff;
  tv = tv >> 8;
  packet[10] = tv & 0xff;

  tv = FLAG;
  packet[11] = tv & 0xff;
  tv = tv >> 8;
  packet[12] = tv & 0xff;
  tv = tv >> 8;
  packet[13] = tv & 0xff;
  tv = tv >> 8;
  packet[14] = tv & 0xff;

  packet[15] = GLOBAL_CONF_CURRENT_SET;

  packet[16] = calculateChecksum(packet, 16);

  sendPacket(packet, 17);
}

void send_MSP_RAW_IMU()
{
  packet[0] = '$';
  packet[1] = 'M';
  packet[2] = '>';    // Direction as in response. May be absent so needs to be removed in that case.
  packet[3] = sizeof(ACCX) + sizeof(ACCY) + sizeof(ACCZ) + sizeof(GYRX) + sizeof(GYRY) + sizeof(GYRZ) + sizeof(MAGX)+ sizeof(MAGY) + sizeof(MAGZ);
  packet[4] = MSP_RAW_IMU;

  tv = ACCX;
  packet[5] = tv & 0xff;
  tv = tv >> 8;
  packet[6] = tv & 0xff;

  tv = ACCY;
  packet[7] = tv & 0xff;
  tv = tv >> 8;
  packet[8] = tv & 0xff;

  tv = ACCZ;
  packet[9] = tv & 0xff;
  tv = tv >> 8;
  packet[10] = tv & 0xff;

  tv = GYRX;
  packet[11] = tv & 0xff;
  tv = tv >> 8;
  packet[12] = tv & 0xff;

  tv = GYRY;
  packet[13] = tv & 0xff;
  tv = tv >> 8;
  packet[14] = tv & 0xff;

  tv = GYRZ;
  packet[15] = tv & 0xff;
  tv = tv >> 8;
  packet[16] = tv & 0xff;

  tv = MAGX;
  packet[17] = tv & 0xff;
  tv = tv >> 8;
  packet[18] = tv & 0xff;

  tv = MAGY;
  packet[19] = tv & 0xff;
  tv = tv >> 8;
  packet[20] = tv & 0xff;

  tv = MAGZ;
  packet[21] = tv & 0xff;
  tv = tv >> 8;
  packet[22] = tv & 0xff;  

  packet[23] = calculateChecksum(packet, 23);

  sendPacket(packet, 24);
}

void send_MSP_SERVO()
{
  packet[0] = '$';
  packet[1] = 'M';
  packet[2] = '>';    // Direction as in response. May be absent so needs to be removed in that case.
  packet[3] = sizeof(SERVO_X) + sizeof(SERVO_Y);
  packet[4] = MSP_SERVO;

  tv = SERVO_X[0];
  packet[5] = tv & 0xff;
  tv = tv >> 8;
  packet[6] = tv & 0xff; 

  tv = SERVO_X[1];
  packet[7] = tv & 0xff;
  tv = tv >> 8;
  packet[8] = tv & 0xff; 

  tv = SERVO_X[2];
  packet[9] = tv & 0xff;
  tv = tv >> 8;
  packet[10] = tv & 0xff; 

  tv = SERVO_X[3];
  packet[11] = tv & 0xff;
  tv = tv >> 8;
  packet[12] = tv & 0xff; 

  tv = SERVO_X[4];
  packet[13] = tv & 0xff;
  tv = tv >> 8;
  packet[14] = tv & 0xff; 

  tv = SERVO_X[5];
  packet[15] = tv & 0xff;
  tv = tv >> 8;
  packet[16] = tv & 0xff; 

  tv = SERVO_X[6];
  packet[17] = tv & 0xff;
  tv = tv >> 8;
  packet[18] = tv & 0xff; 

  tv = SERVO_X[7];
  packet[19] = tv & 0xff;
  tv = tv >> 8;
  packet[20] = tv & 0xff; 

  tv = SERVO_Y[0];
  packet[21] = tv & 0xff;
  tv = tv >> 8;
  packet[22] = tv & 0xff; 

  tv = SERVO_Y[1];
  packet[23] = tv & 0xff;
  tv = tv >> 8;
  packet[24] = tv & 0xff; 

  tv = SERVO_Y[2];
  packet[25] = tv & 0xff;
  tv = tv >> 8;
  packet[26] = tv & 0xff; 

  tv = SERVO_Y[3];
  packet[27] = tv & 0xff;
  tv = tv >> 8;
  packet[28] = tv & 0xff; 

  tv = SERVO_Y[4];
  packet[29] = tv & 0xff;
  tv = tv >> 8;
  packet[30] = tv & 0xff; 

  tv = SERVO_Y[5];
  packet[31] = tv & 0xff;
  tv = tv >> 8;
  packet[32] = tv & 0xff; 

  tv = SERVO_Y[6];
  packet[33] = tv & 0xff;
  tv = tv >> 8;
  packet[34] = tv & 0xff; 

  tv = SERVO_Y[7];
  packet[35] = tv & 0xff;
  tv = tv >> 8;
  packet[36] = tv & 0xff; 

  packet[37] = calculateChecksum(packet, 37);
  
  sendPacket(packet, 38);
}

void send_MSP_MOTOR()
{
  packet[0] = '$';
  packet[1] = 'M';
  packet[2] = '>';    // Direction as in response. May be absent so needs to be removed in that case.
  packet[3] = sizeof(MOTOR_X) + sizeof(MOTOR_Y);
  packet[4] = MSP_MOTOR;

  tv = MOTOR_X[0];
  packet[5] = tv & 0xff;
  tv = tv >> 8;
  packet[6] = tv & 0xff; 

  tv = MOTOR_X[1];
  packet[7] = tv & 0xff;
  tv = tv >> 8;
  packet[8] = tv & 0xff; 

  tv = MOTOR_X[2];
  packet[9] = tv & 0xff;
  tv = tv >> 8;
  packet[10] = tv & 0xff; 

  tv = MOTOR_X[3];
  packet[11] = tv & 0xff;
  tv = tv >> 8;
  packet[12] = tv & 0xff; 

  tv = MOTOR_X[4];
  packet[13] = tv & 0xff;
  tv = tv >> 8;
  packet[14] = tv & 0xff; 

  tv = MOTOR_X[5];
  packet[15] = tv & 0xff;
  tv = tv >> 8;
  packet[16] = tv & 0xff; 

  tv = MOTOR_X[6];
  packet[17] = tv & 0xff;
  tv = tv >> 8;
  packet[18] = tv & 0xff; 

  tv = MOTOR_X[7];
  packet[19] = tv & 0xff;
  tv = tv >> 8;
  packet[20] = tv & 0xff; 

  tv = MOTOR_Y[0];
  packet[21] = tv & 0xff;
  tv = tv >> 8;
  packet[22] = tv & 0xff; 

  tv = MOTOR_Y[1];
  packet[23] = tv & 0xff;
  tv = tv >> 8;
  packet[24] = tv & 0xff; 

  tv = MOTOR_Y[2];
  packet[25] = tv & 0xff;
  tv = tv >> 8;
  packet[26] = tv & 0xff; 

  tv = MOTOR_Y[3];
  packet[27] = tv & 0xff;
  tv = tv >> 8;
  packet[28] = tv & 0xff; 

  tv = MOTOR_Y[4];
  packet[29] = tv & 0xff;
  tv = tv >> 8;
  packet[30] = tv & 0xff; 

  tv = MOTOR_Y[5];
  packet[31] = tv & 0xff;
  tv = tv >> 8;
  packet[32] = tv & 0xff; 

  tv = MOTOR_Y[6];
  packet[33] = tv & 0xff;
  tv = tv >> 8;
  packet[34] = tv & 0xff; 

  tv = MOTOR_Y[7];
  packet[35] = tv & 0xff;
  tv = tv >> 8;
  packet[36] = tv & 0xff; 

  packet[37] = calculateChecksum(packet, 37);
  
  sendPacket(packet, 38);
}

void send_MSP_RC()
{
  packet[0] = '$';
  packet[1] = 'M';
  packet[2] = '>';    // Direction as in response. May be absent so needs to be removed in that case.
  packet[3] = 32;
  packet[4] = MSP_RC;

  for(int i = 0; i < CHANNEL_COUNT; i++)
  {
    tv = rcData[i];
    packet[2*i + 5] = tv & 0xff;
    tv = tv >> 8;
    packet[2*i + 6] = tv & 0xff;
  }

  packet[2*CHANNEL_COUNT + 5] = calculateChecksum(packet, 2*CHANNEL_COUNT + 5);

  sendPacket(packet, 2*CHANNEL_COUNT + 6);
}

void send_MSP_RAW_GPS()
{
  packet[0] = '$';
  packet[1] = 'M';
  packet[2] = '>';    // Direction as in response. May be absent so needs to be removed in that case.
  packet[3] = sizeof(GPS_FIX) + sizeof(GPS_NUMSAT) + sizeof(GPS_COORD_LAT) + sizeof(GPS_COORD_LON) + sizeof(GPS_ALTITUDE) + sizeof(GPS_SPEED) + sizeof(GPS_GROUND_COURSE);
  packet[4] = MSP_RAW_GPS;
  packet[5] = GPS_FIX;
  packet[6] = GPS_NUMSAT;

  tv = GPS_COORD_LAT;
  packet[7] = tv & 0xff;
  tv = tv >> 8;
  packet[8] = tv & 0xff;
  tv = tv >> 8;
  packet[9] = tv & 0xff;
  tv = tv >> 8;
  packet[10] = tv & 0xff;

  tv = GPS_COORD_LON;
  packet[11] = tv & 0xff;
  tv = tv >> 8;
  packet[12] = tv & 0xff;
  tv = tv >> 8;
  packet[13] = tv & 0xff;
  tv = tv >> 8;
  packet[14] = tv & 0xff;

  tv = GPS_ALTITUDE;
  packet[15] = tv & 0xff;
  tv = tv >> 8;
  packet[16] = tv & 0xff;

  tv = GPS_SPEED;
  packet[17] = tv & 0xff;
  tv = tv >> 8;
  packet[18] = tv & 0xff;

  tv = GPS_GROUND_COURSE;
  packet[19] = tv & 0xff;
  tv = tv >> 8;
  packet[20] = tv & 0xff;

  packet[21] = calculateChecksum(packet, 21);


  sendPacket(packet, 22);
}

void send_MSP_COMP_GPS()
{
  packet[0] = '$';
  packet[1] = 'M';
  packet[2] = '>';    // Direction as in response. May be absent so needs to be removed in that case.
  packet[3] = sizeof(GPS_DISTANCE_TO_HOME) + sizeof(GPS_DIRECTION_TO_HOME) + sizeof(GPS_UPDATE);
  packet[4] = MSP_COMP_GPS;

  tv = GPS_DISTANCE_TO_HOME;
  packet[5] = tv & 0xff;
  tv = tv >> 8;
  packet[6] = tv & 0xff;

  tv = GPS_DIRECTION_TO_HOME;
  packet[7] = tv & 0xff;
  tv = tv >> 8;
  packet[8] = tv & 0xff;

  packet[9] = GPS_UPDATE;

  packet[10] = calculateChecksum(packet, 10);

  sendPacket(packet, 11);
}

void send_MSP_ATTITUDE()
{
  packet[0] = '$';
  packet[1] = 'M';
  packet[2] = '>';    // Direction as in response. May be absent so needs to be removed in that case.
  packet[3] = sizeof(ANGX) + sizeof(ANGY) + sizeof(HEADING);
  packet[4] = MSP_ATTITUDE;

  tv = ANGX;
  packet[5] = tv & 0xff;
  tv = tv >> 8;
  packet[6] = tv & 0xff;

  tv = ANGY;
  packet[7] = tv & 0xff;
  tv = tv >> 8;
  packet[8] = tv & 0xff;

  tv = HEADING;
  packet[9] = tv & 0xff;
  tv = tv >> 8;
  packet[10] = tv & 0xff;

  packet[11] = calculateChecksum(packet, 11);

  sendPacket(packet, 12);
}

void send_MSP_ALTITUDE()
{
  packet[0] = '$';
  packet[1] = 'M';
  packet[2] = '>';    // Direction as in response. May be absent so needs to be removed in that case.
  packet[3] = sizeof(EST_ALT) + sizeof(VARIO);
  packet[4] = MSP_ALTITUDE;

  tv = EST_ALT;
  packet[5] = tv & 0xff;
  tv = tv >> 8;
  packet[6] = tv & 0xff;
  tv = tv >> 8;
  packet[7] = tv & 0xff;
  tv = tv >> 8;
  packet[8] = tv & 0xff;

  tv = VARIO;
  packet[9] = tv & 0xff;
  tv = tv >> 8;
  packet[10] = tv & 0xff;

  packet[11] = calculateChecksum(packet, 11);

  sendPacket(packet, 12);
}

void send_MSP_RC_TUNING()
{
  packet[0] = '$';
  packet[1] = 'M';
  packet[2] = '>';    // Direction as in response. May be absent so needs to be removed in that case.
  packet[3] = sizeof(BYTE_RC_RATE) + sizeof(BYTE_RC_EXPO) + sizeof(BYTE_ROLL_PITCH_RATE) + sizeof(BYTE_YAW_RATE) + sizeof(BYTE_DYN_THR_PID) + sizeof(BYTE_THROTTLE_EXPO) + sizeof(BYTE_THROTTLE_MID);
  packet[4] = MSP_RC_TUNING;
  packet[5] = BYTE_RC_RATE;
  packet[6] = BYTE_RC_EXPO;
  packet[7] = BYTE_ROLL_PITCH_RATE;
  packet[8] = BYTE_YAW_RATE;
  packet[9] = BYTE_DYN_THR_PID;
  packet[10] = BYTE_THROTTLE_MID;
  packet[11] = BYTE_THROTTLE_EXPO;
  packet[12] = calculateChecksum(packet, 12);

  sendPacket(packet, 13);
}

void send_MSP_PID()
{
  packet[0] = '$';
  packet[1] = 'M';
  packet[2] = '>';    // Direction as in response. May be absent so needs to be removed in that case.
  packet[3] = 30;
  packet[4] = MSP_PID;
  packet[5] = ROLL_X;
  packet[6] = PITCH_X;
  packet[7] = YAW_X;
  packet[8] = ALT_X;
  packet[9] = POS_X;
  packet[10] = POSR_X;
  packet[11] = NAVR_X;
  packet[12] = LEVEL_X;
  packet[13] = MAG_X;
  packet[14] = VEL_X;
  packet[15] = ROLL_Y;
  packet[16] = PITCH_Y;
  packet[17] = YAW_Y;
  packet[18] = ALT_Y;
  packet[19] = POS_Y;
  packet[20] = POSR_Y;
  packet[21] = NAVR_Y;
  packet[22] = LEVEL_Y;
  packet[23] = MAG_Y;
  packet[24] = VEL_Y;
  packet[25] = ROLL_Z;
  packet[26] = PITCH_Z;
  packet[27] = YAW_Z;
  packet[28] = ALT_Z;
  packet[29] = POS_Z;
  packet[30] = POSR_Z;
  packet[31] = NAVR_Z;
  packet[32] = LEVEL_Z;
  packet[33] = MAG_Z;
  packet[34] = VEL_Z;
  packet[35] = calculateChecksum(packet, 35);

  sendPacket(packet, 36);
}

void send_MSP_BOXNAMES()  // incomplete
{
  
}

void send_MSP_PIDNAMES()  // incomplete
{

}

void send_MSP_WP()
{
  packet[0] = '$';
  packet[1] = 'M';
  packet[2] = '>';    // Direction as in response. May be absent so needs to be removed in that case.
  packet[3] = sizeof(WP_NO) + sizeof(LAT) + sizeof(LON) + sizeof(ALT_HOLD) + sizeof(HEADING) + sizeof(TIME_TO_STAY) + sizeof(NAV_FLAG);
  packet[4] = MSP_WP;
  packet[5] = WP_NO;

  tv = LAT;
  packet[6] = tv & 0xff;
  tv = tv >> 8;
  packet[7] = tv & 0xff;
  tv = tv >> 8;
  packet[8] = tv & 0xff;
  tv = tv >> 8;
  packet[9] = tv & 0xff;

  tv = LON;
  packet[10] = tv & 0xff;
  tv = tv >> 8;
  packet[11] = tv & 0xff;
  tv = tv >> 8;
  packet[12] = tv & 0xff;
  tv = tv >> 8;
  packet[13] = tv & 0xff;  

  tv = ALT_HOLD;
  packet[14] = tv & 0xff;
  tv = tv >> 8;
  packet[15] = tv & 0xff;
  tv = tv >> 8;
  packet[16] = tv & 0xff;
  tv = tv >> 8;
  packet[17] = tv & 0xff;  

  tv = HEADING;
  packet[18] = tv & 0xff;
  tv = tv >> 8;
  packet[19] = tv & 0xff;

  tv = TIME_TO_STAY;
  packet[20] = tv & 0xff;
  tv = tv >> 8;
  packet[21] = tv & 0xff;

  packet[22] = NAV_FLAG;

  packet[23] = calculateChecksum(packet, 23);


  sendPacket(packet, 24);
}

void send_MSP_BOXIDS()    // incomplete
{

}

//------------------------------------------------------------

void call_msp()
{
  uint8_t response[64];
  uint8_t response_length = 0;
  digitalWrite(PC8 , HIGH);
  delay(50);
  digitalWrite(PC8 , LOW);
  delay(50);
if (Serial.available() > 0){
  
  while (Serial.available() > 0) {
    response[response_length++] = Serial.read();
    eeprom_buffered_write_byte(pos,response[response_length-1]);
    pos++;
  }
  
    eeprom_buffered_write_byte(pos,0xff);
    pos++;

  Serial.println();

  // 0 = '$'
  // 1 = 'M'
  // 2 = Direction
  // 3 = payload size
  // 4 = type
  // 5- N-1 = payload data
  // N-1 = checksum
  uint8_t code = response[4];

  eeprom_buffer_flush();

    // Once received the data, Based on the code obtained, one of the following switch statements will work.
    switch(code)
    {
      case MSP_API_VERSION:
      send_MSP_API_VERSION();
      break;

      case MSP_FC_VARIANT:
      send_MSP_FC_VARIANT();
      break;

      case MSP_FC_VERSION:
      send_MSP_FC_VERSION();
      break;

      case MSP_BOARD_INFO:
      send_MSP_BOARD_INFO();
      break;

      case MSP_BUILD_INFO:
      send_MSP_BUILD_INFO();
      break;

      case MSP_STATUS :
      send_MSP_STATUS();
      break;

      case MSP_RAW_IMU :
      send_MSP_RAW_IMU();
      break;
      
      case MSP_SERVO :
      send_MSP_SERVO();
      break;
      
      case MSP_MOTOR :
      send_MSP_MOTOR();
      break;
      
      case MSP_RC :
      send_MSP_RC();
      break;
      
      case MSP_RAW_GPS :
      send_MSP_RAW_GPS();
      break;
      
      case MSP_COMP_GPS :
      send_MSP_COMP_GPS();
      break;
      
      case MSP_ATTITUDE :
      send_MSP_ATTITUDE();
      break;
      
      case MSP_ALTITUDE :
      send_MSP_ALTITUDE();
      break;
      
      case MSP_ANALOG :
      send_MSP_ANALOG();
      break;
      
      case MSP_RC_TUNING :
      send_MSP_RC_TUNING();
      break;
      
      case MSP_PID :
      send_MSP_PID();
      break;
      
      case MSP_BOXNAMES :
      send_MSP_BOXNAMES();
      break;
      
      case MSP_PIDNAMES :
      send_MSP_PIDNAMES();
      break;
      
      case MSP_WP :
      send_MSP_WP();
      break;
      
      case MSP_BOXIDS :
      send_MSP_BOXIDS();
      break;
    }
}
}

void loop()
{
  call_msp();
  return;
}