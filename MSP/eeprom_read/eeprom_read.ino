#include <EEPROM.h>

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
delay(5000);
  eeprom_buffer_fill();
  digitalWrite(PC8 , HIGH);
  delay(100);
  digitalWrite(PC8 , LOW);
  delay(100);
}

uint32_t pos = 0;

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(PC8 , HIGH);
  delay(100);
  digitalWrite(PC8 , LOW);
  delay(100);
  Serial.println(eeprom_buffered_read_byte(pos), HEX);
  pos++;
  if (pos == 400)
  exit(0);
}
