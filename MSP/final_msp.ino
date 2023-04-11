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


//------------------------------------------------------------

// Function to send MSP command
void sendMSPCommand(uint8_t command, uint8_t *data, uint8_t length) {
  // Calculate MSP checksum
  uint8_t checksum = calculateChecksum(data, length);

  // Incomplete
  // Here comes the code for sending command.
  // Use switch case for each type of command.
  // Don't do like did in requests.  
}

//---------------------------------------------------------------

char mode;

void loop() {
switch(mode)
{

  case 'R':   // Request Mode
  {
    
    sendMSPRequest( get_request()/* here goes the type of request */);
    // Can do hard coding for each type of request and leaving only to the switch case to execute. In that case, no need to use get_request function.
    // It is basically the get_request method only but extended form.
  }
  break;

  case 'G':   // Receiving mode
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
      break;

      case MSP_STATUS :
      break;

      case MSP_RAW_IMU :
      break;
      
      case MSP_SERVO :
      break;
      
      case MSP_MOTOR :
      break;
      
      case MSP_RC :
      break;
      
      case MSP_RAW_GPS :
      break;
      
      case MSP_COMP_GPS :
      break;
      
      case MSP_ATTITUDE :
      break;
      
      case MSP_ALTITUDE :
      break;
      
      case MSP_ANALOG :
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
  break;

  case 'C':   // Command Mode
  {
    sendMSPCommand(/* Here come the contents of the command*/);

    // Again the commands can be hard coded for each type of command
  }
  break;
}
}
