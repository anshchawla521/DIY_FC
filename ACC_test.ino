
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
//HardwareSerial Serial(Serial_RX_6, Serial_TX_6);

uint16_t accx = 0;

uint16_t read_values_from_sensor_spi(uint8_t address_of_reg , uint8_t salve_device_cs_pin , bool bytes_16 = false)
{


  digitalWrite(salve_device_cs_pin, LOW);
  //read





  SPI_1.transfer(0x80 | address_of_reg); // MSB
  SPI_1.transfer(0); // dummy
  uint16_t data_received = SPI_1.transfer(0); // actual data
  if (bytes_16)
  {

    data_received |= SPI_1.transfer(0) << 8; // other bytes
    // supports LSB first only
  }
  delayMicroseconds(10);
  digitalWrite(salve_device_cs_pin, HIGH);
  delayMicroseconds(10);

  return data_received;



}
void setup()
{
  pinMode(GYRO_CS_1, OUTPUT);
  pinMode(LED_1 , OUTPUT);
  digitalWrite(GYRO_CS_1, LOW);
  delay(10);
  digitalWrite(GYRO_CS_1, HIGH);
  delay(100);
  SPI_1.begin();
  Serial.begin(9600);
  delay(25);
  SPI_1.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));
  delay(100);
  write_values_to_sensor_spi(0x40 , GYRO_CS_1 , 0x28);

  write_values_to_sensor_spi(0x41 , GYRO_CS_1 , 0x02);

  write_values_to_sensor_spi(0x73 , GYRO_CS_1 , 0x45);

  write_values_to_sensor_spi(0x74 , GYRO_CS_1 , 0x67);

  write_values_to_sensor_spi(0x75 , GYRO_CS_1 , 0x23);

  write_values_to_sensor_spi(0x76 , GYRO_CS_1 , 0x22);


}

void loop()
{


  // get_imu_data();

  delay(100);
  Serial.print(read_values_from_sensor_spi(0x40, GYRO_CS_1) , HEX); // chip id
  Serial.print(" ");
  Serial.print(read_values_from_sensor_spi(0x41, GYRO_CS_1) , HEX); // chip id
  Serial.print(" ");
  Serial.print(read_values_from_sensor_spi(0x0C, GYRO_CS_1,true) , HEX); // chip id
  Serial.print(" ");
  Serial.print(read_values_from_sensor_spi(0x0E, GYRO_CS_1,true) , HEX); // chip id
  Serial.print(" ");
  Serial.print(read_values_from_sensor_spi(0x10, GYRO_CS_1,true) , HEX); // chip id
  Serial.print(" ");
  Serial.println(read_values_from_sensor_spi(0x03, GYRO_CS_1) , BIN); // status



}




void write_values_to_sensor_spi(uint8_t address_of_reg , uint8_t salve_device_cs_pin , byte data)
{


  digitalWrite(salve_device_cs_pin, LOW);

  SPI_1.transfer(0x7F & address_of_reg); // write address
  SPI_1.transfer(data);
  digitalWrite(salve_device_cs_pin, HIGH);
  delay(2);



}


void get_imu_data()
{
  Serial.println(read_values_from_sensor_spi(0x00, GYRO_CS_1) , HEX);// chip id
  Serial.println(read_values_from_sensor_spi(0x02, GYRO_CS_1) , BIN); // error reg
  Serial.println(read_values_from_sensor_spi(0x03, GYRO_CS_1) , BIN); // status reg

  Serial.println(read_values_from_sensor_spi(0x0C, GYRO_CS_1 , true)); // read accx
  Serial.println(read_values_from_sensor_spi(0x0E, GYRO_CS_1, true)); // read accy
  Serial.println(read_values_from_sensor_spi(0x10, GYRO_CS_1 , true)); // read temp
  Serial.println("  ");

}
