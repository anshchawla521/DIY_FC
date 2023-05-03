#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>


Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345); //defining variable for compass

void setup(void) 
{
  Wire.setSDA(PB11); //SDA
  Wire.setSCL(PB10); //SCL
  Serial.begin(9600);
  mag.begin();
}

void loop(void) 
{
  Wire.beginTransmission(118);// 3 addresses: 0x76,0x77,0x1E (0x76=118)
  sensors_event_t event; 
  mag.getEvent(&event); //get data
  Wire.endTransmission(); 
  // Hold the module so that Z is pointing up and you can measure the heading with x&y
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // We need to add declination angle, which is the error of the magnetic field in current location.
  float declinationAngle = 1.73;  //at chandigarh: 1degree 44minute
  heading += declinationAngle;
  
  // We need to keep heading angle between 0 and 2pi
  if(heading < 0)    heading += 2*PI;
  if(heading > 2*PI) heading -= 2*PI;
   
  // Convert radians to degrees
  float headingDegrees = heading*(180/3.1416) ; 
  Serial.println(headingDegrees);
}
