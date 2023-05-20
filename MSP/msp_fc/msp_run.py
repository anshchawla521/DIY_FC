MSP_API_VERSION = 1
MSP_FC_VARIANT  = 2
MSP_FC_VERSION  = 3
MSP_BOARD_INFO  = 4
MSP_BUILD_INFO  = 5

MSP_STATUS    = 101 
MSP_RAW_IMU   = 102 
MSP_SERVO     = 103 
MSP_MOTOR     = 104 
MSP_RC        = 105 
MSP_RAW_GPS   = 106 
MSP_COMP_GPS  = 107 
MSP_ATTITUDE  = 108 
MSP_ALTITUDE  = 109 
MSP_ANALOG    = 110 
MSP_RC_TUNING = 111 
MSP_PID       = 112 

def send_Command(value):
    SerialObj.write('$')
    SerialObj.write('M')
    SerialObj.write('<')
    SerialObj.write(0)
    SerialObj.write(str(value))
    SerialObj.write(str(value))
    
def receive_Command():
    packet = list()
    packet.append(SerialObj.read(5))
    packet.append(SerialObj.read(packet[3]+1))
    return packet

import serial
import time

SerialObj = serial.Serial('COM8')

SerialObj.baudrate = 115200

time.sleep(3)

send_Command(MSP_ATTITUDE)

time.sleep(1)

received_packet = receive_Command()


SerialObj.close()