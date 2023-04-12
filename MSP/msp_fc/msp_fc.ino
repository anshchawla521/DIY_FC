#include <SoftwareSerial.h>
#include <stdint.h>
#include <stdbool.h>

#define RX 10
#define TX 11

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

SoftwareSerial mspSerial(RX, TX);

void setup() {
  Serial.begin(115200);     // Serial port for debugging
  mspSerial.begin(115200);
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



int64_t tv;
// ----------------------------------------------------------

// Function to send an MSP request
// Here header length is considered 6 to go with the conventional flow of the packet contents

void sendMSPRequest(uint8_t type) {
  uint8_t header[6] = { '$', 'M', '<', 0, type, 0 };    // $M is header, next is direction, size of payload, type of command and the termination is payload.
                                                    // actually payload is not to be presented but to maintain the integrity, it is shown. The code will not read payload once the command is interpreted
  uint8_t checksum = calculateChecksum(header,6);
  
  // Send the request
  mspSerial.write(header, 6);
  mspSerial.write(checksum);
}

uint8_t get_request()
{

}

// ----------------------------------------------------------

// Function to calculate MSP checksum
uint8_t calculateChecksum(uint8_t *data, uint8_t length) {
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < length; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

void printPacket(uint8_t *data, uint8_t length)
{
  Serial.print("Received Data: ");
    for (uint8_t i = 0; i < length; i++) {
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
}

void sendPacket(uint8_t *data, uint8_t length)
{
  for(uint8_t i = 0; i < length; i++)
  {
    mspSerial.write(data[i]);
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
  uint8_t cs = calculateChecksum(packet, sizeof(packet))
  mspSerial.write(cs);
  // mspSerial.write('$');
  // mspSerial.write('M');
  // mspSerial.write('>');   // Direction as in response. May be absent so needs to be removed in that case.
  // mspSerial.write(sizeof(VERSION) + sizeof(MULTITYPE) + sizeof(MSP_VERSION) + sizeof(CAPABILITY));
  // mspSerial.write(MSP_IDENT);
  // mspSerial.write(VERSION);
  // mspSerial.write(MULTITYPE);
  // mspSerial.write(MSP_VERSION);
  // mspSerial.write(CAPABILITY);

  printPacket(packet,sizeof(packet))
  Serial.print(cs);
  Serial.println();
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
  uint8_t cs = calculateChecksum(packet, sizeof(packet))
  mspSerial.write(cs);

  printPacket(packet,sizeof(packet))
  Serial.print(cs);
  Serial.println();
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
  uint8_t cs = calculateChecksum(packet, sizeof(packet))
  mspSerial.write(cs);

  printPacket(packet,sizeof(packet))
  Serial.print(cs);
  Serial.println();
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
  uint8_t cs = calculateChecksum(packet, sizeof(packet))
  mspSerial.write(cs);

  printPacket(packet,sizeof(packet))
  Serial.print(cs);
  Serial.println();

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
  uint8_t cs = calculateChecksum(packet, sizeof(packet))
  mspSerial.write(cs);

  printPacket(packet,sizeof(packet))
  Serial.print(cs);
  Serial.println();

}

void send_MSP_RC()
{
  uint8_t packet[21];
  packet[0] = '$';
  packet[1] = 'M';
  packet[2] = '>';    // Direction as in response. May be absent so needs to be removed in that case.
  packet[3] = sizeof(CYCLETIME) + sizeof(I2C_ERRORS_COUNT) + sizeof(SENSOR) + sizeof(FLAG) + sizeof(GLOBAL_CONF_CURRENT_SET);
  packet[4] = MSP_RC;
  packet[5]

  sendPacket(packet, sizeof(packet));
  uint8_t cs = calculateChecksum(packet, sizeof(packet))
  mspSerial.write(cs);

  printPacket(packet,sizeof(packet))
  Serial.print(cs);
  Serial.println();

}
// incomplete

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
  uint8_t cs = calculateChecksum(packet, sizeof(packet))
  mspSerial.write(cs);

  printPacket(packet,sizeof(packet))
  Serial.print(cs);
  Serial.println();

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
  uint8_t cs = calculateChecksum(packet, sizeof(packet))
  mspSerial.write(cs);

  printPacket(packet,sizeof(packet))
  Serial.print(cs);
  Serial.println();

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
  uint8_t cs = calculateChecksum(packet, sizeof(packet))
  mspSerial.write(cs);

  printPacket(packet,sizeof(packet))
  Serial.print(cs);
  Serial.println();

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
  uint8_t cs = calculateChecksum(packet, sizeof(packet))
  mspSerial.write(cs);

  printPacket(packet,sizeof(packet))
  Serial.print(cs);
  Serial.println();

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
  uint8_t cs = calculateChecksum(packet, sizeof(packet))
  mspSerial.write(cs);

  printPacket(packet,sizeof(packet))
  Serial.print(cs);
  Serial.println();
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
  uint8_t cs = calculateChecksum(packet, sizeof(packet))
  mspSerial.write(cs);

  printPacket(packet,sizeof(packet))
  Serial.print(cs);
  Serial.println();
}

//------------------------------------------------------------

void loop()
{
  uint8_t response[64];
  uint8_t response_length = 0;
  while (!mspSerial.available());   // wait until data comes

  while (mspSerial.available()) {
    response[response_length++] = mspSerial.read();
  }

  // For debugging
  Serial.print("Response: ");
  for (int i = 0; i < response_length; i++) {
    Serial.print(response[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Derive a way to separate the code from the received information and other details.

    uint8_t code;

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
      break;
      
      case MSP_PID :
      break;
      
      case MSP_BOX :
      break;
      
      case MSP_MISC :
      break;
      
      case MSP_MOTOR_PINS :
      break;
      
      case MSP_BOXNAMES :
      break;
      
      case MSP_PIDNAMES :
      break;
      
      case MSP_WP :
      break;
      
      case MSP_BOXIDS :
      break;
      
      case MSP_SERVO_CONF :
      break;
      
      case MSP_ACC_CALIBRATION :
      break;
      
      case MSP_MAG_CALIBRATION :
      break;
      
      case MSP_EEPROM_WRITE :
      break;
    }
}
