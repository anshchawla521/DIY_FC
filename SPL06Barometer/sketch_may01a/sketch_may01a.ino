#include <Wire.h>

#define SPL06_ADDRESS 0x76  // I2C address of SPL06 sensor

float seaLevelPressure = 101325.0;  // Default sea level pressure in Pascals (Pa)
float currentPressure;  // Current pressure in Pascals (Pa)

void setup() {
  Serial.begin(9600);
  Wire.setSDA(PB11); // using pin name PY_n
  Wire.setSCL(PB10); // using pin number PYn
  Wire.begin();


  delay(5000);
  Wire.beginTransmission(SPL06_ADDRESS);
  Wire.write(0x08);
  Wire.write(0x07);
  delay(10);
  Wire.endTransmission();

  Wire.beginTransmission(SPL06_ADDRESS);
  Wire.write(0x08);
  Wire.requestFrom(SPL06_ADDRESS, 1);
  Wire.endTransmission();

  while (Wire.available())
  {
    Serial.println(Wire.read() , BIN);
  }





}

void loop() {

  Wire.beginTransmission(SPL06_ADDRESS);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(SPL06_ADDRESS, 3);


  while (Wire.available() > 3);

  Serial.println(int32_t(Wire.read() << 16 | Wire.read() << 8 | Wire.read()) , DEC);

  delay(10);
  Wire.beginTransmission(SPL06_ADDRESS);
  Wire.write(0x08);
  Wire.endTransmission();
  Wire.requestFrom(SPL06_ADDRESS, 1);

  while (Wire.available())
  {
    //    Serial.println(Wire.read() , BIN);
    Wire.read();
  }

  delay(20);

}
