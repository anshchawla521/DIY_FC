#include <SoftwareSerial.h>
#include <stdint.h>
#include <stdbool.h>

#define RX PC7
#define TX PC6

// ------------------------------------------------------

// The link used for the requests and commands
// https://web.archive.org/web/20200225114200/multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol
// defining the various msp command/request types and their codes
#define MSP_IDENT 100
#define MSP_STATUS 101
#define MSP_RAW_IMU 102
#define MSP_SERVO 103
#define MSP_MOTOR 104
#define MSP_RC 105
#define MSP_RAW_GPS 106
#define MSP_COMP_GPS 107
#define MSP_ATTITUDE 108
#define MSP_ALTITUDE 109
#define MSP_ANALOG 110
#define MSP_RC_TUNING 111
#define MSP_PID 112
#define MSP_BOX 113
#define MSP_MISC 114
#define MSP_MOTOR_PINS 115
#define MSP_BOXNAMES 116
#define MSP_PIDNAMES 117
#define MSP_WP 118
#define MSP_BOXIDS 119
#define MSP_SERVO_CONF 120
#define MSP_SET_RAW_RC 200
#define MSP_SET_RAW_GPS 201
#define MSP_SET_PID 202
#define MSP_SET_BOX 203
#define MSP_SET_RC_TUNING 204
#define MSP_ACC_CALIBRATION 205
#define MSP_MAG_CALIBRATION 206
#define MSP_SET_MISC 207
#define MSP_RESET_CONF 208
#define MSP_SET_WP 209
#define MSP_SELECT_SETTING 210
#define MSP_SET_HEAD 211
#define MSP_SET_SERVO_CONF 212
#define MSP_SET_MOTOR 214
#define MSP_BIND 240
#define MSP_EEPROM_WRITE 250

// SoftwareSerial Serial(RX, TX);

typedef struct {
  uint16_t min;
  uint16_t max;
  uint16_t middle;
  uint8_t rate;
} servo_conf_;


void setup() {
  // // Serial.begin(115200);     // // Serial port for debugging
  Serial.begin(115200);
  pinMode(PC8 , OUTPUT);
}

// ----------------------------------------------------------

// Sample Variables
uint8_t VERSION = 5;
uint8_t MULTITYPE = 4;
uint8_t MSP_VERSION = 6;
uint32_t CAPABILITY = 0x11223344;
uint16_t CYCLETIME = 0x1235;
uint16_t I2C_ERRORS_COUNT = 0X0000;
uint16_t SENSOR = 0x1;
uint32_t FLAG = 0x456;
uint8_t GLOBAL_CONF_CURRENT_SET = 0x5;
int16_t ACCX = 0x12;
int16_t ACCY = 0X13;
int16_t ACCZ = 0x14;
int16_t GYRX = 0x02;
int16_t GYRY = 0x03;
int16_t GYRZ = 0x04;
int16_t MAGX = 0x22;
int16_t MAGY = 0x23;
int16_t MAGZ = 0x24;
uint16_t SERVO_X[8] = {};
uint16_t SERVO_Y[8] = {};
uint16_t MOTOR_X[8] = {};
uint16_t MOTOR_Y[8] = {};
uint8_t GPS_FIX = 0x1;
uint8_t GPS_NUMSAT = 0x2;
uint32_t GPS_COORD_LAT = 0x98786;
uint32_t GPS_COORD_LON = 0X4672;
uint16_t GPS_ALTITUDE = 0x146;
uint16_t GPS_SPEED = 0x987;
uint16_t GPS_GROUND_COURSE = 0x123;
uint16_t GPS_DISTANCE_TO_HOME = 0x23;
uint16_t GPS_DIRECTION_TO_HOME = 0x56;
uint8_t GPS_UPDATE = 0x17;
int16_t ANGX = 0x5;
int16_t ANGY = 0x2;
int16_t HEADING = 0x7;
int32_t EST_ALT =  0x555;
int16_t VARIO = 0x444;
uint8_t VBAT = 0x10;
uint16_t INT_POWER_METER_SUM = 0x100;
uint16_t RSSI = 0x200;
uint16_t AMPERAGE = 0x300;
uint8_t BYTE_RC_RATE = 0x10;
uint8_t BYTE_RC_EXPO = 0x20;
uint8_t BYTE_ROLL_PITCH_RATE = 0x30;
uint8_t BYTE_YAW_RATE = 0x40;
uint8_t BYTE_DYN_THR_PID = 0x50;
uint8_t BYTE_THROTTLE_MID = 0x60;
uint8_t BYTE_THROTTLE_EXPO = 0x70;
uint8_t ROLL_X = 0x10;
uint8_t ROLL_Y = 0x11;
uint8_t ROLL_Z = 0x12;
uint8_t PITCH_X = 0x20;
uint8_t PITCH_Y = 0x21;
uint8_t PITCH_Z = 0x22;
uint8_t YAW_X = 0x30;
uint8_t YAW_Y = 0x31;
uint8_t YAW_Z = 0x32;
uint8_t ALT_X = 0x40;
uint8_t ALT_Y = 0x41;
uint8_t ALT_Z = 0x42;
uint8_t POS_X = 0x50;
uint8_t POS_Y = 0x51;
uint8_t POS_Z = 0x52;
uint8_t POSR_X = 0x60;
uint8_t POSR_Y = 0x61;
uint8_t POSR_Z = 0x62;
uint8_t NAVR_X = 0x70;
uint8_t NAVR_Y = 0x71;
uint8_t NAVR_Z = 0x72;
uint8_t LEVEL_X = 0x80;
uint8_t LEVEL_Y = 0x81;
uint8_t LEVEL_Z = 0x82;
uint8_t MAG_X = 0x90;
uint8_t MAG_Y = 0x91;
uint8_t MAG_Z = 0x92;
uint8_t VEL_X = 0xa0;
uint8_t VEL_Y = 0xa1;
uint8_t VEL_Z = 0xa2;
uint8_t BOXITEMS = 0x02;
uint16_t BOX_CONF = 0x13;
uint16_t INT_POWER_TRIGGER1 = 0x01;
uint16_t CONF_MIN_THROTTLE = 0x0400;
uint16_t MAXTHROTTLE = 0x0401;
uint16_t MINCOMMAND = 0x400;
uint16_t CONF_FAILSAFE_THROTTLE = 0x3ff;
uint16_t PLOG_ARM = 0x010;
uint32_t PLOG_LIFETIME;
uint16_t CONF_MAG_DECLINATION = 0x20;
uint8_t CONF_VBATSCALE = 0x50;
uint8_t CONF_VBATLEVEL_WARN1 = 0x60;
uint8_t CONF_VBATLEVEL_WARN2 = 0x70;
uint8_t CONF_VBATLEVEL_CRIT = 0x80;
uint8_t MOTOR1 = 0x01;
uint8_t MOTOR2 = 0x02;
uint8_t MOTOR3 = 0x03;
uint8_t MOTOR4 = 0x04;
uint8_t MOTOR5 = 0x05;
uint8_t MOTOR6 = 0x06;
uint8_t MOTOR7 = 0x07;
uint8_t MOTOR8 = 0x08;
uint8_t WP_NO = 0x0f;
uint32_t LAT;
uint32_t LON;
uint32_t ALT_HOLD;
uint16_t TIME_TO_STAY;
uint8_t NAV_FLAG;
servo_conf_ CONF_SERVO[8];


int64_t tv;
// ----------------------------------------------------------

// Function to send an MSP request
// Here header length is considered 6 to go with the conventional flow of the packet contents

void sendMSPRequest(uint8_t type) {
  uint8_t header[6] = { '$', 'M', '<', 0, type, 0 };    // $M is header, next is direction, size of payload, type of command and the termination is payload.
                                                    // actually payload is not to be presented but to maintain the integrity, it is shown. The code will not read payload once the command is interpreted
  uint8_t checksum = calculateChecksum(header,6);
  
  // Send the request
  Serial.write(header, 6);
  Serial.write(checksum);
}

uint8_t get_request()
{

}

// ----------------------------------------------------------

// Function to calculate MSP checksum
uint8_t calculateChecksum(uint8_t *data, uint8_t length) {
  uint8_t checksum = 0;
  for (uint8_t i = 3; i < length; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

void printPacket(uint8_t *data, uint8_t length)
{
  // // Serial.print("Received Data: ");
  //   for (uint8_t i = 0; i < length; i++) {
  //     // Serial.print(data[i], HEX);
  //     // Serial.print(" ");
  //   }
  //   // Serial.println();
}

void sendPacket(uint8_t *data, uint8_t length)
{
  for(uint8_t i = 0; i < length; i++)
  {
    Serial.write(data[i]);
  }
}

//------------------------------------------------------------

void send_MSP_IDENT()
{
  uint8_t packet[12];
  packet[0] = '$';
  packet[1] = 'M';
  packet[2] = '>';    // Direction as in response. May be absent so needs to be removed in that case.
  packet[3] = sizeof(VERSION) + sizeof(MULTITYPE) + sizeof(MSP_VERSION) + sizeof(CAPABILITY);
  packet[4] = MSP_IDENT;
  packet[5] = VERSION;
  packet[6] = MULTITYPE;
  packet[7] = MSP_VERSION;
  tv = CAPABILITY;
  packet[8] = tv & 0xff;
  tv = tv >> 8;
  packet[9] = tv & 0xff;
  tv = tv >> 8;
  packet[10] = tv & 0xff;
  tv = tv >> 8;
  packet[11] = tv & 0xff;

  sendPacket(packet, sizeof(packet));
  uint8_t cs = calculateChecksum(packet, sizeof(packet));
  Serial.write(cs);
  // Serial.write('$');
  // Serial.write('M');
  // Serial.write('>');   // Direction as in response. May be absent so needs to be removed in that case.
  // Serial.write(sizeof(VERSION) + sizeof(MULTITYPE) + sizeof(MSP_VERSION) + sizeof(CAPABILITY));
  // Serial.write(MSP_IDENT);
  // Serial.write(VERSION);
  // Serial.write(MULTITYPE);
  // Serial.write(MSP_VERSION);
  // Serial.write(CAPABILITY);

  printPacket(packet,sizeof(packet));
  // // Serial.print(cs);
  // // Serial.println();
}

void send_MSP_STATUS()
{
  uint8_t packet[16];
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

  sendPacket(packet, sizeof(packet));
  uint8_t cs = calculateChecksum(packet, sizeof(packet));
  Serial.write(cs);

  printPacket(packet,sizeof(packet));
  // Serial.print(cs);
  // Serial.println();
}

void send_MSP_RAW_IMU()
{
  uint8_t packet[23];
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


  sendPacket(packet, sizeof(packet));
  uint8_t cs = calculateChecksum(packet, sizeof(packet));
  Serial.write(cs);

  printPacket(packet,sizeof(packet));
  // Serial.print(cs);
  // Serial.println();
}

void send_MSP_SERVO()
{
  uint8_t packet[37];
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
  
  sendPacket(packet, sizeof(packet));
  uint8_t cs = calculateChecksum(packet, sizeof(packet));
  Serial.write(cs);

  printPacket(packet,sizeof(packet));
  // Serial.print(cs);
  // Serial.println();

}

void send_MSP_MOTOR()
{
  uint8_t packet[37];
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
  
  sendPacket(packet, sizeof(packet));
  uint8_t cs = calculateChecksum(packet, sizeof(packet));
  Serial.write(cs);

  printPacket(packet,sizeof(packet));
  // Serial.print(cs);
  // Serial.println();

}

void send_MSP_RC()  // incomplete
{
  uint8_t packet[21];
  packet[0] = '$';
  packet[1] = 'M';
  packet[2] = '>';    // Direction as in response. May be absent so needs to be removed in that case.
  packet[3] = sizeof(CYCLETIME) + sizeof(I2C_ERRORS_COUNT) + sizeof(SENSOR) + sizeof(FLAG) + sizeof(GLOBAL_CONF_CURRENT_SET);
  packet[4] = MSP_RC;

  sendPacket(packet, sizeof(packet));
  uint8_t cs = calculateChecksum(packet, sizeof(packet));
  Serial.write(cs);

  printPacket(packet,sizeof(packet));
  // Serial.print(cs);
  // Serial.println();

}


void send_MSP_RAW_GPS()
{
  uint8_t packet[21];
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



  sendPacket(packet, sizeof(packet));
  uint8_t cs = calculateChecksum(packet, sizeof(packet));
  Serial.write(cs);

  printPacket(packet,sizeof(packet));
  // Serial.print(cs);
  // Serial.println();

}

void send_MSP_COMP_GPS()
{
  uint8_t packet[10];
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

  sendPacket(packet, sizeof(packet));
  uint8_t cs = calculateChecksum(packet, sizeof(packet));
  Serial.write(cs);

  printPacket(packet,sizeof(packet));
  // Serial.print(cs);
  // Serial.println();

}

void send_MSP_ATTITUDE()
{
  uint8_t packet[11];
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

  sendPacket(packet, sizeof(packet));
  uint8_t cs = calculateChecksum(packet, sizeof(packet));
  Serial.write(cs);

  printPacket(packet,sizeof(packet));
  // Serial.print(cs);
  // Serial.println();

}

void send_MSP_ALTITUDE()
{
  uint8_t packet[11];
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

  sendPacket(packet, sizeof(packet));
  uint8_t cs = calculateChecksum(packet, sizeof(packet));
  Serial.write(cs);

  printPacket(packet,sizeof(packet));
  // Serial.print(cs);
  // Serial.println();

}

void send_MSP_ANALOG()
{
  uint8_t packet[12];
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

  sendPacket(packet, sizeof(packet));
  uint8_t cs = calculateChecksum(packet, sizeof(packet));
  Serial.write(cs);

  printPacket(packet,sizeof(packet));
  // Serial.print(cs);
  // Serial.println();
}

void send_MSP_RC_TUNING()
{
  uint8_t packet[12];
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

  sendPacket(packet, sizeof(packet));
  uint8_t cs = calculateChecksum(packet, sizeof(packet));
  Serial.write(cs);

  printPacket(packet,sizeof(packet));
  // Serial.print(cs);
  // Serial.println();
}

void send_MSP_PID()
{
  uint8_t packet[35];
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

  sendPacket(packet, sizeof(packet));
  uint8_t cs = calculateChecksum(packet, sizeof(packet));
  Serial.write(cs);

  printPacket(packet,sizeof(packet));
  // Serial.print(cs);
  // Serial.println();
}

void send_MSP_BOX()
{
  uint8_t packet[8];
  packet[0] = '$';
  packet[1] = 'M';
  packet[2] = '>';    // Direction as in response. May be absent so needs to be removed in that case.
  packet[3] = sizeof(BOXITEMS) + sizeof(BOX_CONF);
  packet[4] = MSP_BOX;
  packet[5] = BOXITEMS;   // select the mode by making bit 1.
  tv = BOX_CONF;          // set the values by making use of the table as in above link
  packet[6] = tv & 0xff;
  tv = tv >> 8;
  packet[7] = tv & 0xff;


  sendPacket(packet, sizeof(packet));
  uint8_t cs = calculateChecksum(packet, sizeof(packet));
  Serial.write(cs);

  printPacket(packet,sizeof(packet));
  // Serial.print(cs);
  // Serial.println();

}

void send_MSP_MISC()
{
  uint8_t packet[27];
  packet[0] = '$';
  packet[1] = 'M';
  packet[2] = '>';    // Direction as in response. May be absent so needs to be removed in that case.
  packet[3] = sizeof(INT_POWER_TRIGGER1) + sizeof(CONF_MIN_THROTTLE) + sizeof(MAXTHROTTLE) + sizeof(MINCOMMAND) + sizeof(CONF_FAILSAFE_THROTTLE) + sizeof(PLOG_ARM) + sizeof(PLOG_LIFETIME) + sizeof(CONF_MAG_DECLINATION) + sizeof(CONF_VBATSCALE) + sizeof(CONF_VBATLEVEL_WARN1) + sizeof(CONF_VBATLEVEL_WARN2) + sizeof(CONF_VBATLEVEL_CRIT);
  packet[4] = MSP_MISC;
  tv = INT_POWER_TRIGGER1;
  packet[5] = tv & 0xff;
  tv = tv >> 8;
  packet[6] = tv & 0xff;
  tv = CONF_MIN_THROTTLE;
  packet[7] = tv & 0xff;
  tv = tv >> 8;
  packet[8] = tv & 0xff;
  tv = MAXTHROTTLE;
  packet[9] = tv & 0xff;
  tv = tv >> 8;
  packet[10] = tv & 0xff;
  tv = MINCOMMAND;
  packet[11] = tv & 0xff;
  tv = tv >> 8;
  packet[12] = tv & 0xff;
  tv = CONF_FAILSAFE_THROTTLE;
  packet[13] = tv & 0xff;
  tv = tv >> 8;
  packet[14] = tv & 0xff;
  tv = PLOG_ARM;
  packet[15] = tv & 0xff;
  tv = tv >> 8;
  packet[16] = tv & 0xff;
  tv = PLOG_LIFETIME;
  packet[17] = tv & 0xff;
  tv = tv >> 8;
  packet[18] = tv & 0xff;
  tv = tv >> 8;
  packet[19] = tv & 0xff;
  tv = tv >> 8;
  packet[20] = tv & 0xff;
  tv = CONF_MAG_DECLINATION;
  packet[21] = tv & 0xff;
  tv = tv >> 8;
  packet[22] = tv & 0xff;
  packet[23] = CONF_VBATSCALE;
  packet[24] = CONF_VBATLEVEL_WARN1;
  packet[25] = CONF_VBATLEVEL_WARN2;
  packet[26] = CONF_VBATLEVEL_CRIT;

  sendPacket(packet, sizeof(packet));
  uint8_t cs = calculateChecksum(packet, sizeof(packet));
  Serial.write(cs);

  printPacket(packet,sizeof(packet));
  // Serial.print(cs);
  // Serial.println();

}

void send_MSP_MOTOR_PINS()
{
  uint8_t packet[13];
  packet[0] = '$';
  packet[1] = 'M';
  packet[2] = '>';    // Direction as in response. May be absent so needs to be removed in that case.
  packet[3] = 8;
  packet[4] = MSP_MOTOR_PINS;
  packet[5] = MOTOR1;
  packet[6] = MOTOR2;
  packet[7] = MOTOR3;
  packet[8] = MOTOR4;
  packet[9] = MOTOR5;
  packet[10] = MOTOR6;
  packet[11] = MOTOR7;
  packet[12] = MOTOR8;

  sendPacket(packet, sizeof(packet));
  uint8_t cs = calculateChecksum(packet, sizeof(packet));
  Serial.write(cs);

  printPacket(packet,sizeof(packet));
  // Serial.print(cs);
  // Serial.println();

}

void send_MSP_BOXNAMES()  // incomplete
{
  
}

void send_MSP_PIDNAMES()  // incomplete
{

}

void send_MSP_WP()
{
  uint8_t packet[23];
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

  sendPacket(packet, sizeof(packet));
  uint8_t cs = calculateChecksum(packet, sizeof(packet));
  Serial.write(cs);

  printPacket(packet,sizeof(packet));
  // Serial.print(cs);
  // Serial.println();
}

void send_MSP_BOXIDS()    // incomplete
{

}

void send_MSP_SERVO_CONF()
{
//  uint8_t packet[21];
//  packet[0] = '$';
//  packet[1] = 'M';
//  packet[2] = '>';    // Direction as in response. May be absent so needs to be removed in that case.
//  packet[3] = sizeof(CONF_SERVO);
//  packet[4] = MSP_SERVO_CONF;
//  for(uint8_t i = 0; i < sizeof(CONF_SERVO); i++)
//  {
//    tv = CONF_SERVO[i];
//    for(uint8_t j = 0; j < sizeof(servo_conf_); j++)
//    {
//      packet[5 + i*7 + j] = tv & 0xff;
//      tv = tv >> 8;
//    }
//  }
//
//  sendPacket(packet, sizeof(packet));
//  uint8_t cs = calculateChecksum(packet, sizeof(packet));
//  Serial.write(cs);
//
//  printPacket(packet,sizeof(packet));
//  // Serial.print(cs);
//  // Serial.println();
}

//------------------------------------------------------------

void loop()
{
  uint8_t response[64];
  uint8_t response_length = 0;
  digitalWrite(PC8 , HIGH);
  delay(100);
  digitalWrite(PC8 , LOW);
  delay(100);
if (Serial.available() > 0){
  
  while (Serial.available() > 0) {
    response[response_length++] = Serial.read();
  }
  digitalWrite(PC8 , HIGH);
  delay(1000);
  digitalWrite(PC8 , LOW);
  delay(1000);

  // For debugging
  // Serial.print("Response: ");
  for (int i = 0; i < response_length; i++) {
    // Serial.print(response[i], HEX);
    // Serial.print(" ");
  }
  // Serial.println();

  // 0 = '$'
  // 1 = 'M'
  // 2 = Direction
  // 3 = payload size
  // 4 = type
  // 5- N-1 = payload data
  // N-1 = checksum
     uint8_t code = response[4];
  //  uint8_t code = MSP_IDENT;

    // Once received the data, Based on the code obtained, one of the following switch statements will work.
    switch(code)
    {
      case MSP_IDENT :
      send_MSP_IDENT();
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
      
      case MSP_BOX :
      send_MSP_BOX();
      break;
      
      case MSP_MISC :
      send_MSP_MISC();
      break;
      
      case MSP_MOTOR_PINS :
      send_MSP_MOTOR_PINS();
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
      
      case MSP_SERVO_CONF :
      send_MSP_SERVO_CONF();
      break;
    }
}
}
