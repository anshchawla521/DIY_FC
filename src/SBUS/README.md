Arduino & STM32Duino library for Futaba & FrSky SBUS
===============================

This library allows your arduino or STM32 to read SBUS channel values.

Source:
* https://mbed.org/users/Digixx/notebook/futaba-s-bus-controlled-by-mbed/

To connect this to your Arduino, you'll need an inverter cable such as http://www.hobbyking.com/hobbyking/store/__24523__ZYX_S_S_BUS_Connection_Cable.html, or you can build your own using a TTL inverter or the schematic in the included SBUS_inverter.jpeg file.

![correct inverter](https://github.com/zendes/SBUS/blob/master/SBUS_inverter.jpeg)

Basic Installation
------------------
```
$ cd <arduino-sketchbook>/libraries
$ git clone https://github.com/zendes/SBUS.git
```

Then restart your Arduino IDE and take a look at the examples.