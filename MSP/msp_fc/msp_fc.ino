#include <stdint.h>
#include <stdbool.h>
#include <EEPROM.h>
#include "msp_protocol.h"


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

#define FC_VERSION_STRING STR(FC_VERSION_MAJOR) "." STR(FC_VERSION_MINOR) "." STR(FC_VERSION_PATCH_LEVEL)

extern const char* const targetName;

#define GIT_SHORT_REVISION_LENGTH   7 // lower case hexadecimal digits.
extern const char* const shortGitRevision;

#define BUILD_DATE_LENGTH 11
extern const char* const buildDate;  // "MMM DD YYYY" MMM = Jan/Feb/...

#define BUILD_TIME_LENGTH 8
extern const char* const buildTime;  // "HH:MM:SS"

#define MSP_API_VERSION_STRING STR(API_VERSION_MAJOR) "." STR(API_VERSION_MINOR)


/**
 * MSP Guidelines, emphasis is used to clarify.
 *
 * Each FlightController (FC, Server) MUST change the API version when any MSP command is added, deleted, or changed.
 *
 * If you fork the FC source code and release your own version, you MUST change the Flight Controller Identifier.
 *
 * NEVER release a modified copy of this code that shares the same Flight controller IDENT and API version
 * if the API doesn't match EXACTLY.
 *
 * Consumers of the API (API clients) SHOULD first attempt to get a response from the MSP_API_VERSION command.
 * If no response is obtained then client MAY try the legacy MSP_IDENT command.
 *
 * API consumers should ALWAYS handle communication failures gracefully and attempt to continue
 * without the information if possible.  Clients MAY log/display a suitable message.
 *
 * API clients should NOT attempt any communication if they can't handle the returned API MAJOR VERSION.
 *
 * API clients SHOULD attempt communication if the API MINOR VERSION has increased from the time
 * the API client was written and handle command failures gracefully.  Clients MAY disable
 * functionality that depends on the commands while still leaving other functionality intact.
 * that the newer API version may cause problems before using API commands that change FC state.
 *
 * It is for this reason that each MSP command should be specific as possible, such that changes
 * to commands break as little functionality as possible.
 *
 * API client authors MAY use a compatibility matrix/table when determining if they can support
 * a given command from a given flight controller at a given api version level.
 *
 * Developers MUST NOT create new MSP commands that do more than one thing.
 *
 * Failure to follow these guidelines will likely invoke the wrath of developers trying to write tools
 * that use the API and the users of those tools.
 */

#pragma once

/* Protocol numbers used both by the wire format, config system, and
   field setters.
*/

#define MSP_PROTOCOL_VERSION                0

#define API_VERSION_MAJOR                   1  // increment when major changes are made
#define API_VERSION_MINOR                   41 // increment after a release, to set the version for all changes to go into the following release (if no changes to MSP are made between the releases, this can be reverted before the release)

#define API_VERSION_LENGTH                  2

#define MULTIWII_IDENTIFIER "MWII";
#define BASEFLIGHT_IDENTIFIER "BAFL";
#define BETAFLIGHT_IDENTIFIER "BTFL"
#define CLEANFLIGHT_IDENTIFIER "CLFL"
#define INAV_IDENTIFIER "INAV"
#define RACEFLIGHT_IDENTIFIER "RCFL"

#define FLIGHT_CONTROLLER_IDENTIFIER_LENGTH 4
#define FLIGHT_CONTROLLER_VERSION_LENGTH    3
#define FLIGHT_CONTROLLER_VERSION_MASK      0xFFF

#define BOARD_IDENTIFIER_LENGTH             4 // 4 UPPER CASE alpha numeric characters that identify the board being used.
#define BOARD_HARDWARE_REVISION_LENGTH      2

// These are baseflight specific flags but they are useless now since MW 2.3 uses the upper 4 bits for the navigation version.
#define CAP_PLATFORM_32BIT          ((uint32_t)1 << 31)
#define CAP_BASEFLIGHT_CONFIG       ((uint32_t)1 << 30)

// MW 2.3 stores NAVI_VERSION in the top 4 bits of the capability mask.
#define CAP_NAVI_VERSION_BIT_4_MSB  ((uint32_t)1 << 31)
#define CAP_NAVI_VERSION_BIT_3      ((uint32_t)1 << 30)
#define CAP_NAVI_VERSION_BIT_2      ((uint32_t)1 << 29)
#define CAP_NAVI_VERSION_BIT_1_LSB  ((uint32_t)1 << 28)

#define CAP_DYNBALANCE              ((uint32_t)1 << 2)
#define CAP_FLAPS                   ((uint32_t)1 << 3)
#define CAP_NAVCAP                  ((uint32_t)1 << 4)
#define CAP_EXTAUX                  ((uint32_t)1 << 5)

#define MSP_API_VERSION                 1    //out message
#define MSP_FC_VARIANT                  2    //out message
#define MSP_FC_VERSION                  3    //out message
#define MSP_BOARD_INFO                  4    //out message
#define MSP_BUILD_INFO                  5    //out message

#define MSP_NAME                        10   //out message          Returns user set board name - betaflight
#define MSP_SET_NAME                    11   //in message           Sets board name - betaflight

//
// MSP commands for Cleanflight original features
//
#define MSP_BATTERY_CONFIG              32
#define MSP_SET_BATTERY_CONFIG          33

#define MSP_MODE_RANGES                 34    //out message         Returns all mode ranges
#define MSP_SET_MODE_RANGE              35    //in message          Sets a single mode range

#define MSP_FEATURE_CONFIG              36
#define MSP_SET_FEATURE_CONFIG          37

#define MSP_BOARD_ALIGNMENT_CONFIG      38
#define MSP_SET_BOARD_ALIGNMENT_CONFIG  39

#define MSP_CURRENT_METER_CONFIG        40
#define MSP_SET_CURRENT_METER_CONFIG    41

#define MSP_MIXER_CONFIG                42
#define MSP_SET_MIXER_CONFIG            43

#define MSP_RX_CONFIG                   44
#define MSP_SET_RX_CONFIG               45

#define MSP_LED_COLORS                  46
#define MSP_SET_LED_COLORS              47

#define MSP_LED_STRIP_CONFIG            48
#define MSP_SET_LED_STRIP_CONFIG        49

#define MSP_RSSI_CONFIG                 50
#define MSP_SET_RSSI_CONFIG             51

#define MSP_ADJUSTMENT_RANGES           52
#define MSP_SET_ADJUSTMENT_RANGE        53

// private - only to be used by the configurator, the commands are likely to change
#define MSP_CF_SERIAL_CONFIG            54
#define MSP_SET_CF_SERIAL_CONFIG        55

#define MSP_VOLTAGE_METER_CONFIG        56
#define MSP_SET_VOLTAGE_METER_CONFIG    57

#define MSP_SONAR_ALTITUDE              58 //out message get sonar altitude [cm]

#define MSP_PID_CONTROLLER              59
#define MSP_SET_PID_CONTROLLER          60

#define MSP_ARMING_CONFIG               61
#define MSP_SET_ARMING_CONFIG           62

//
// Baseflight MSP commands (if enabled they exist in Cleanflight)
//
#define MSP_RX_MAP                      64 //out message get channel map (also returns number of channels total)
#define MSP_SET_RX_MAP                  65 //in message set rx map, numchannels to set comes from MSP_RX_MAP

// DEPRECATED - DO NOT USE "MSP_BF_CONFIG" and MSP_SET_BF_CONFIG.  In Cleanflight, isolated commands already exist and should be used instead.
// DEPRECATED - #define MSP_BF_CONFIG                   66 //out message baseflight-specific settings that aren't covered elsewhere
// DEPRECATED - #define MSP_SET_BF_CONFIG               67 //in message baseflight-specific settings save

#define MSP_REBOOT                      68 //in message reboot settings

// Use MSP_BUILD_INFO instead
// DEPRECATED - #define MSP_BF_BUILD_INFO               69 //out message build date as well as some space for future expansion

#define MSP_DATAFLASH_SUMMARY           70 //out message - get description of dataflash chip
#define MSP_DATAFLASH_READ              71 //out message - get content of dataflash chip
#define MSP_DATAFLASH_ERASE             72 //in message - erase dataflash chip

// No-longer needed
// DEPRECATED - #define MSP_LOOP_TIME                   73 //out message         Returns FC cycle time i.e looptime parameter // DEPRECATED
// DEPRECATED - #define MSP_SET_LOOP_TIME               74 //in message          Sets FC cycle time i.e looptime parameter    // DEPRECATED

#define MSP_FAILSAFE_CONFIG             75 //out message         Returns FC Fail-Safe settings
#define MSP_SET_FAILSAFE_CONFIG         76 //in message          Sets FC Fail-Safe settings

#define MSP_RXFAIL_CONFIG               77 //out message         Returns RXFAIL settings
#define MSP_SET_RXFAIL_CONFIG           78 //in message          Sets RXFAIL settings

#define MSP_SDCARD_SUMMARY              79 //out message         Get the state of the SD card

#define MSP_BLACKBOX_CONFIG             80 //out message         Get blackbox settings
#define MSP_SET_BLACKBOX_CONFIG         81 //in message          Set blackbox settings

#define MSP_TRANSPONDER_CONFIG          82 //out message         Get transponder settings
#define MSP_SET_TRANSPONDER_CONFIG      83 //in message          Set transponder settings

#define MSP_OSD_CONFIG                  84 //out message         Get osd settings - betaflight
#define MSP_SET_OSD_CONFIG              85 //in message          Set osd settings - betaflight

#define MSP_OSD_CHAR_READ               86 //out message         Get osd settings - betaflight
#define MSP_OSD_CHAR_WRITE              87 //in message          Set osd settings - betaflight

#define MSP_VTX_CONFIG                  88 //out message         Get vtx settings - betaflight
#define MSP_SET_VTX_CONFIG              89 //in message          Set vtx settings - betaflight

// Betaflight Additional Commands
#define MSP_ADVANCED_CONFIG             90
#define MSP_SET_ADVANCED_CONFIG         91

#define MSP_FILTER_CONFIG               92
#define MSP_SET_FILTER_CONFIG           93

#define MSP_PID_ADVANCED                94
#define MSP_SET_PID_ADVANCED            95

#define MSP_SENSOR_CONFIG               96
#define MSP_SET_SENSOR_CONFIG           97

#define MSP_CAMERA_CONTROL              98

#define MSP_SET_ARMING_DISABLED         99

//
// OSD specific
//
#define MSP_OSD_VIDEO_CONFIG            180
#define MSP_SET_OSD_VIDEO_CONFIG        181

// External OSD displayport mode messages
#define MSP_DISPLAYPORT                 182

#define MSP_COPY_PROFILE                183

#define MSP_BEEPER_CONFIG               184
#define MSP_SET_BEEPER_CONFIG           185

#define MSP_SET_TX_INFO                 186 // in message           Used to send runtime information from TX lua scripts to the firmware
#define MSP_TX_INFO                     187 // out message          Used by TX lua scripts to read information from the firmware

//
// Multwii original MSP commands
//

// See MSP_API_VERSION and MSP_MIXER_CONFIG
//DEPRECATED - #define MSP_IDENT                100    //out message         mixerMode + multiwii version + protocol version + capability variable


#define MSP_STATUS               101    //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102    //out message         9 DOF
#define MSP_SERVO                103    //out message         servos
#define MSP_MOTOR                104    //out message         motors
#define MSP_RC                   105    //out message         rc channels and more
#define MSP_RAW_GPS              106    //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107    //out message         distance home, direction home
#define MSP_ATTITUDE             108    //out message         2 angles 1 heading
#define MSP_ALTITUDE             109    //out message         altitude, variometer
#define MSP_ANALOG               110    //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING            111    //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112    //out message         P I D coeff (9 are used currently)
// Legacy Multiicommand that was never used.
//DEPRECATED - #define MSP_BOX                  113    //out message         BOX setup (number is dependant of your setup)
// Legacy command that was under constant change due to the naming vagueness, avoid at all costs - use more specific commands instead.
//DEPRECATED - #define MSP_MISC                 114    //out message         powermeter trig
// Legacy Multiicommand that was never used and always wrong
//DEPRECATED - #define MSP_MOTOR_PINS           115    //out message         which pins are in use for motors & servos, for GUI
#define MSP_BOXNAMES             116    //out message         the aux switch names
#define MSP_PIDNAMES             117    //out message         the PID names
#define MSP_WP                   118    //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BOXIDS               119    //out message         get the permanent IDs associated to BOXes
#define MSP_SERVO_CONFIGURATIONS 120    //out message         All servo configurations.
#define MSP_NAV_STATUS           121    //out message         Returns navigation status
#define MSP_NAV_CONFIG           122    //out message         Returns navigation parameters
#define MSP_MOTOR_3D_CONFIG      124    //out message         Settings needed for reversible ESCs
#define MSP_RC_DEADBAND          125    //out message         deadbands for yaw alt pitch roll
#define MSP_SENSOR_ALIGNMENT     126    //out message         orientation of acc,gyro,mag
#define MSP_LED_STRIP_MODECOLOR  127    //out message         Get LED strip mode_color settings
#define MSP_VOLTAGE_METERS       128    //out message         Voltage (per meter)
#define MSP_CURRENT_METERS       129    //out message         Amperage (per meter)
#define MSP_BATTERY_STATE        130    //out message         Connected/Disconnected, Voltage, Current Used
#define MSP_MOTOR_CONFIG         131    //out message         Motor configuration (min/max throttle, etc)
#define MSP_GPS_CONFIG           132    //out message         GPS configuration
#define MSP_COMPASS_CONFIG       133    //out message         Compass configuration
#define MSP_ESC_SENSOR_DATA      134    //out message         Extra ESC data from 32-Bit ESCs (Temperature, RPM)
#define MSP_GPS_RESCUE           135    //out message         GPS Rescues's angle, initialAltitude, descentDistance, rescueGroundSpeed, sanityChecks and minSats
#define MSP_GPS_RESCUE_PIDS      136    //out message         GPS Rescues's throttleP and velocity PIDS + yaw P
#define MSP_VTXTABLE_BAND        137    //out message         vtxTable band/channel data
#define MSP_VTXTABLE_POWERLEVEL  138    //out message         vtxTable powerLevel data
#define MSP_MOTOR_TELEMETRY      139    //out message         Per-motor telemetry data (RPM, packet stats, ESC temp, etc.)

#define MSP_SET_RAW_RC           200    //in message          8 rc chan
#define MSP_SET_RAW_GPS          201    //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202    //in message          P I D coeff (9 are used currently)
// Legacy multiiwii command that was never used.
//DEPRECATED - #define MSP_SET_BOX              203    //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204    //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID, yaw expo
#define MSP_ACC_CALIBRATION      205    //in message          no param
#define MSP_MAG_CALIBRATION      206    //in message          no param
// Legacy command that was under constant change due to the naming vagueness, avoid at all costs - use more specific commands instead.
//DEPRECATED - #define MSP_SET_MISC             207    //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208    //in message          no param
#define MSP_SET_WP               209    //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210    //in message          Select Setting Number (0-2)
#define MSP_SET_HEADING          211    //in message          define a new heading hold direction
#define MSP_SET_SERVO_CONFIGURATION 212    //in message          Servo settings
#define MSP_SET_MOTOR            214    //in message          PropBalance function
#define MSP_SET_NAV_CONFIG       215    //in message          Sets nav config parameters - write to the eeprom
#define MSP_SET_MOTOR_3D_CONFIG  217    //in message          Settings needed for reversible ESCs
#define MSP_SET_RC_DEADBAND      218    //in message          deadbands for yaw alt pitch roll
#define MSP_SET_RESET_CURR_PID   219    //in message          resetting the current pid profile to defaults
#define MSP_SET_SENSOR_ALIGNMENT 220    //in message          set the orientation of the acc,gyro,mag
#define MSP_SET_LED_STRIP_MODECOLOR 221 //in  message         Set LED strip mode_color settings
#define MSP_SET_MOTOR_CONFIG     222    //out message         Motor configuration (min/max throttle, etc)
#define MSP_SET_GPS_CONFIG       223    //out message         GPS configuration
#define MSP_SET_COMPASS_CONFIG   224    //out message         Compass configuration
#define MSP_SET_GPS_RESCUE       225    //in message          GPS Rescues's angle, initialAltitude, descentDistance, rescueGroundSpeed, sanityChecks and minSats
#define MSP_SET_GPS_RESCUE_PIDS  226    //in message          GPS Rescues's throttleP and velocity PIDS + yaw P
#define MSP_SET_VTXTABLE_BAND    227    //in message          set vtxTable band/channel data (one band at a time)
#define MSP_SET_VTXTABLE_POWERLEVEL 228 //in message          set vtxTable powerLevel data (one powerLevel at a time)

// #define MSP_BIND                 240    //in message          no param
// #define MSP_ALARMS               242

#define MSP_EEPROM_WRITE         250    //in message          no param
#define MSP_RESERVE_1            251    //reserved for system usage
#define MSP_RESERVE_2            252    //reserved for system usage
#define MSP_DEBUGMSG             253    //out message         debug string buffer
#define MSP_DEBUG                254    //out message         debug1,debug2,debug3,debug4
#define MSP_V2_FRAME             255    //MSPv2 payload indicator

// Additional commands that are not compatible with MultiWii
#define MSP_STATUS_EX            150    //out message         cycletime, errors_count, CPU load, sensor present etc
#define MSP_UID                  160    //out message         Unique device ID
#define MSP_GPSSVINFO            164    //out message         get Signal Strength (only U-Blox)
#define MSP_GPSSTATISTICS        166    //out message         get GPS debugging data
#define MSP_MULTIPLE_MSP         230    //out message         request multiple MSPs in one request - limit is the TX buffer; returns each MSP in the order they were requested starting with length of MSP; MSPs with input arguments are not supported
#define MSP_MODE_RANGES_EXTRA    238    //out message         Reads the extra mode range data
#define MSP_ACC_TRIM             240    //out message         get acc angle trim values
#define MSP_SET_ACC_TRIM         239    //in message          set acc angle trim values
#define MSP_SERVO_MIX_RULES      241    //out message         Returns servo mixer configuration
#define MSP_SET_SERVO_MIX_RULE   242    //in message          Sets servo mixer configuration
#define MSP_SET_4WAY_IF          245    //in message          Sets 4way interface
#define MSP_SET_RTC              246    //in message          Sets the RTC clock
#define MSP_RTC                  247    //out message         Gets the RTC clock
#define MSP_SET_BOARD_INFO       248    //in message          Sets the board information for this board
#define MSP_SET_SIGNATURE        249    //in message          Sets the signature of the board and serial number


uint8_t MSP_Check(uint8_t MSP_buf[], uint8_t buf_size);
//----------------------------------------


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
uint8_t packet[300];
uint8_t VERSION = 5;
uint8_t MULTITYPE = 4;
uint8_t MSP_VERSION = 6;
//uint8_t MSP_PROTOCOL_VERSION = 4;
//uint8_t API_VERSION_MAJOR = 8;
//uint8_t API_VERSION_MINOR = 1;
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

// uint8_t get_request()
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
  printPacket(packet,9);
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
  printPacket(packet,10);
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
  printPacket(packet,9);
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
  printPacket(packet,85);
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

void send_MSP_IDENT()
{
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
  packet[12] = calculateChecksum(packet, 12);
  
  sendPacket(packet, 13);
  // Serial.write('$');
  // Serial.write('M');
  // Serial.write('>');   // Direction as in response. May be absent so needs to be removed in that case.
  // Serial.write(sizeof(VERSION) + sizeof(MULTITYPE) + sizeof(MSP_VERSION) + sizeof(CAPABILITY));
  // Serial.write(MSP_IDENT);
  // Serial.write(VERSION);
  // Serial.write(MULTITYPE);
  // Serial.write(MSP_VERSION);
  // Serial.write(CAPABILITY);

  printPacket(packet,13);
  // // Serial.print(cs);
  // // Serial.println();
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

  printPacket(packet,17);
  // Serial.print(cs);
  // Serial.println();
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
  printPacket(packet,24);
  // Serial.print(cs);
  // Serial.println();
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

  printPacket(packet,38);
  // Serial.print(cs);
  // Serial.println();

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

  printPacket(packet,38);
  // Serial.print(cs);
  // Serial.println();

}

void send_MSP_RC()  // incomplete
{
  packet[0] = '$';
  packet[1] = 'M';
  packet[2] = '>';    // Direction as in response. May be absent so needs to be removed in that case.
  packet[3] = sizeof(CYCLETIME) + sizeof(I2C_ERRORS_COUNT) + sizeof(SENSOR) + sizeof(FLAG) + sizeof(GLOBAL_CONF_CURRENT_SET);
  packet[4] = MSP_RC;

  sendPacket(packet, sizeof(packet));

  printPacket(packet,sizeof(packet));
  // Serial.print(cs);
  // Serial.println();

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

  printPacket(packet,22);
  // Serial.print(cs);
  // Serial.println();

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

  printPacket(packet,11);
  // Serial.print(cs);
  // Serial.println();

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

  printPacket(packet,12);
  // Serial.print(cs);
  // Serial.println();

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

  printPacket(packet,12);
  // Serial.print(cs);
  // Serial.println();

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

  printPacket(packet,13);
  // Serial.print(cs);
  // Serial.println();
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

  printPacket(packet,13);
  // Serial.print(cs);
  // Serial.println();
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

  printPacket(packet,sizeof(packet));
  // Serial.print(cs);
  // Serial.println();
}

void send_MSP_BOX()
{
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
  packet[8] = calculateChecksum(packet, 8);

  sendPacket(packet, 9);

  printPacket(packet,9);
  // Serial.print(cs);
  // Serial.println();

}

void send_MSP_MISC()
{
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
  packet[27] = calculateChecksum(packet, 27);

  sendPacket(packet, 28);

  printPacket(packet,28);
  // Serial.print(cs);
  // Serial.println();

}

void send_MSP_MOTOR_PINS()
{
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
  packet[13] = calculateChecksum(packet, 13);

  sendPacket(packet, 14);

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

  printPacket(packet,sizeof(packet));
  // Serial.print(cs);
  // Serial.println();
}

void send_MSP_BOXIDS()    // incomplete
{

}

void send_MSP_SERVO_CONF()// incomplete
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
// complete
//------------------------------------------------------------

uint32_t pos = 0;

void loop()
{
  uint8_t response[64];
  uint8_t response_length = 0;
  // digitalWrite(PC8 , HIGH);
  // delay(100);
  // digitalWrite(PC8 , LOW);
  // delay(100);
if (Serial.available() > 0){
  
  while (Serial.available() > 0) {
    response[response_length++] = Serial.read();
    eeprom_buffered_write_byte(pos,response[response_length-1]);
    pos++;
  }
  
    eeprom_buffered_write_byte(pos,0x11);
    pos++;
  digitalWrite(PC8 , !digitalRead(PC8));
  // delay(1000);
  // digitalWrite(PC8 , LOW);
  // delay(1000);

  // For debugging
  // // Serial.print("Response: ");
  // for (int i = 0; i < response_length; i++) {
  //   // Serial.print(response[i], HEX);
  //   // Serial.print(" ");
  // }
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
  // eeprom_buffered_write_byte(pos,code);
  // eeprom_buffered_write_byte(pos+1,pos);
  // pos = pos +2;
  eeprom_buffer_flush();
digitalWrite(PC8 , HIGH);
  delay(100);
  digitalWrite(PC8 , LOW);
  delay(100);
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
