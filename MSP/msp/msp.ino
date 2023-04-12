#include <SoftwareSerial.h>

#define MSP_HEADER '$'
#define MSP_VERSION '0'
#define MSP_MAX_PAYLOAD_SIZE 64

typedef struct {
  uint8_t cmd;
  uint8_t data[MSP_MAX_PAYLOAD_SIZE];
  uint8_t dataSize;
  uint8_t checksum;
} MSPFrame_t;

SoftwareSerial mspSerial(10, 11); // RX, TX

bool msp_process_rx(MSPFrame_t* rxFrame) {
  static uint8_t rxBuffer[MSP_MAX_PAYLOAD_SIZE + 6];
  static uint8_t rxIndex = 0;
  static uint8_t rxChecksum = 0;

  while (mspSerial.available() > 0) {
    uint8_t byte = mspSerial.read();

    switch (rxIndex) {
      case 0:
        if (byte != MSP_HEADER) {
          break;
        }
        rxChecksum ^= byte;
        break;

      case 1:
        if (byte != MSP_VERSION) {
          rxIndex = 0;
          rxChecksum = 0;
          break;
        }
        rxChecksum ^= byte;
        break;

      case 2:
        rxFrame->dataSize = byte;
        rxChecksum ^= byte;
        break;

      case 3:
        rxFrame->cmd = byte;
        rxChecksum ^= byte;
        break;

      default:
        rxFrame->data[rxIndex - 4] = byte;
        rxChecksum ^= byte;
        if (rxIndex == rxFrame->dataSize + 3) {
          rxFrame->checksum = byte;
          if (rxChecksum == 0) {
            return true;
          } else {
            // Bad checksum
          }
          rxIndex = 0;
          rxChecksum = 0;
        }
        break;
    }

    rxBuffer[rxIndex++] = byte;
    if (rxIndex >= MSP_MAX_PAYLOAD_SIZE + 6) {
      // Buffer overflow
      rxIndex = 0;
      rxChecksum = 0;
    }
  }

  return false;
}

void msp_send(MSPFrame_t* txFrame) {
  uint8_t checksum = 0;
  uint8_t dataSize = txFrame->dataSize;

  mspSerial.write(MSP_HEADER);
  checksum ^= MSP_HEADER;

  mspSerial.write(MSP_VERSION);
  checksum ^= MSP_VERSION;

  mspSerial.write(dataSize);
  checksum ^= dataSize;

  mspSerial.write(txFrame->cmd);
  checksum ^= txFrame->cmd;

  for (uint8_t i = 0; i < dataSize; i++) {
    mspSerial.write(txFrame->data[i]);
    checksum ^= txFrame->data[i];
  }

  mspSerial.write(checksum);
}

void setup() {
  mspSerial.begin(115200);
}

void loop() {
  MSPFrame_t rxFrame;
  rxFrame.cmd = 0;
  rxFrame.dataSize = 0;
  rxFrame.checksum = 0;

  if (msp_process_rx(&rxFrame)) {
    // Handle the received command here
  }
}









-----------------------------------------------------
#include <stdint.h>
#include <stdbool.h>

// Function to calculate MSP checksum
uint8_t calculateChecksum(uint8_t *data, uint8_t length) {
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < length; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

// Function to send MSP command
void sendMSPCommand(uint8_t command, uint8_t *data, uint8_t length) {
  // Calculate MSP checksum
  uint8_t checksum = calculateChecksum(data, length);

  // Send MSP command header
  uint8_t header[] = { '$', 'M', '<', length, command, checksum };
  for (uint8_t i = 0; i < 6; i++) {
    // Send each byte of header
    // (Assuming UART is configured for 115200 baudrate, 8 data bits, no parity, 1 stop bit)
    USART_SendData(USART1, header[i]);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
  }

  // Send MSP command payload
  for (uint8_t i = 0; i < length; i++) {
    // Send each byte of payload
    USART_SendData(USART1, data[i]);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
  }
}

uint16_t rcData[8] = { 1500, 1500, 1500, 1500, 1000, 1000, 1000, 1000 };
uint8_t data[16];
for (uint8_t i = 0; i < 8; i++) {
  data[i*2] = rcData[i] & 0xFF;
  data[i*2+1] = rcData[i] >> 8;
}
sendMSPCommand(MSP_SET_RAW_RC, data, 16);












// --------------------------------------------------
#include <SoftwareSerial.h> // Include the software serial library
#define MSP_SET_RAW_RC 200   // Define the MSP code for setting raw RC values
#define MSP_HEADER     0x24  // Define the header for MSP commands
#define MSP_DIRECTION  0x10  // Define the direction (to the MultiWii board)
#define SERIAL_PORT    Serial // Define the serial port for sending MSP commands

// Set up the software serial port
SoftwareSerial MSP_SERIAL(10, 11); // RX, TX

void setup() {
  MSP_SERIAL.begin(115200); // Initialize the software serial port
  SERIAL_PORT.begin(115200); // Initialize the main serial port
}

void loop() {
  // Define the raw RC values to send to the MultiWii board
  uint16_t rc_values[8] = { 1000, 1500, 1500, 1000, 1000, 1000, 1000, 1000 };

  // Create an array to hold the MSP command data
  uint8_t msp_data[16];

  // Add the header and direction to the MSP command data
  msp_data[0] = MSP_HEADER;
  msp_data[1] = MSP_DIRECTION;
  msp_data[2] = sizeof(rc_values) + 2;
  msp_data[3] = MSP_SET_RAW_RC;
  

  // Add the raw RC values to the MSP command data
  for (int i = 0; i < 8; i++) {
    msp_data[i*2+4] = rc_values[i] & 0xFF;      // first 8 bits
    msp_data[i*2+5] = rc_values[i] >> 8;        // next 8 bits
  }

  // Calculate and add the checksum to the MSP command data
  uint8_t checksum = 0;
  for (int i = 0; i < msp_data[1]; i++) {
    checksum ^= msp_data[i+3];
  }
  msp_data[msp_data[1]+2] = checksum;

  // Send the MSP command data over the software serial port
  for (int i = 0; i < msp_data[1]+4; i++) {
    MSP_SERIAL.write(msp_data[i]);
  }

  // Wait for a response from the MultiWii board
  while (!SERIAL_PORT.available()) {}
  while (SERIAL_PORT.available()) {
    SERIAL_PORT.read();
  }
}
