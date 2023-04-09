#include <SPI.h>

#define BEEPER_1 PC5
#define MOTOR_1 PB6
#define MOTOR_2 PB7
#define MOTOR_3 PB8
#define MOTOR_4 PB9
#define MOTOR_5 PB0
#define MOTOR_6 PB1
#define MOTOR_7 PB5
#define MOTOR_8 PB4
#define SERVO_1 PA8
#define PPM_1 PA3
#define LED_STRIP_1 PC9
#define Serial_TX_1 PA9
#define Serial_TX_2 PA2
#define Serial_TX_3 PC10
#define Serial_TX_4 PA0
#define Serial_TX_5 PC12
#define Serial_TX_6 PC6
#define Serial_RX_1 PA10
#define Serial_RX_2 PA3
#define Serial_RX_3 PC11
#define Serial_RX_4 PA1
#define Serial_RX_5 PD2
#define Serial_RX_6 PC7
#define I2C_SCL_2 PB10
#define I2C_SDA_2 PB11
#define LED_1 PC8
#define SPI_SCK_1 PA5
#define SPI_SCK_2 PB13
#define SPI_MISO_1 PA6
#define SPI_MISO_2 PB14
#define SPI_MOSI_1 PA7
#define SPI_MOSI_2 PB15
#define CAMERA_CONTROL_1 PB3
#define ADC_BATT_1 PC0
#define ADC_RSSI_1 PC2
#define ADC_CURR_1 PC1
#define SDCARD_CS_1 PA15
#define PINIO_1 PC3 // used for disabling bluetooth module
#define OSD_CS_1 PB12
#define GYRO_EXTI_1 PC4
#define GYRO_CS_1 PA4

SPIClass SPI_1(SPI_MOSI_1, SPI_MISO_1, SPI_SCK_1);
// HardwareSerial Serial(Serial_RX_6, Serial_TX_6);

uint16_t accx = 0;
uint16_t accy = 0;
uint16_t accz = 0;
uint16_t gyrox = 0;
uint16_t gyroy = 0;
uint16_t gyroz = 0;
uint16_t magx = 0;
uint16_t magy = 0;
uint16_t magz = 0;
int altitude_from_baro = 0;

unsigned long prev_time, current_time;
float dt;

void setup()
{
  pinMode(GYRO_CS_1, OUTPUT);
  pinMode(LED_1, OUTPUT);

  // initialize bmi270 in spi mode
  digitalWrite(GYRO_CS_1, LOW);
  delay(10);
  digitalWrite(GYRO_CS_1, HIGH);
  delay(100);

  SPI_1.begin();
  SPI_1.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));
  Serial.begin(9600);
  delay(25);
}

void loop()
{
  prev_time = current_time;
  current_time = micros();
  dt = (current_time - prev_time) / 1000000.0;

  loopBlink(); // Indicate we are in main loop with short blink every 1.5 seconds

  // Print data at 100hz - SELECT ONE:
  // printRadioData();
  // printDesiredState();
  // printGyroData();
  // printAccelData();
  // printMagData();
  // printRollPitchYaw();
  // printPIDoutput();
  // printMotorCommands();

  get_imu_data();
  Madgwick();

  getDesState();

  controlAngle(); // PID loops goes inside this
  controlMixer();

  log_data();

  throttleCut();

  commandMotors();

  getCommands(); // Pulls current available radio commands
  failSafe();    // Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup

  // Regulate loop rate
  loopRate(2000); // Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
}
void loopRate(int freq)
{
  // DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until
   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to
   * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
   * and remain above 2kHz, without needing to retune all of our filtering parameters.
   */
  float invFreq = 1.0 / freq * 1000000.0;
  unsigned long checker = micros();

  // Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time))
  {
    checker = micros();
  }
}

void get_imu_data();