#include <EEPROM.h>

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
delay(5000);
  eeprom_buffer_fill();
}

uint32_t pos = 0;

void loop() {
  // put your main code here, to run repeatedly:

  Serial.println(eeprom_buffered_read_byte(pos), HEX);
  pos++;
  if (pos == 200)
  exit(0);
}
