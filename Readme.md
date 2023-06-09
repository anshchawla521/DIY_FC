# Diy FC

This project is inspired by drehmFlight and is supposed to be a code for a basic flight controller in arduino ide that runs on alredy existing Stm32 based flight controllers with little bit of modification.

## Documentation

Will be uploaded soon

## Tasks

### Ansh

- [x] Interafce and get ACC , gyro readings from the BMI 270 sensor
- [x] Find the pin connections for stm32f405 on the speedybee f405 v3 board
- [x] Make a skeleton code for people to work on
- [ ] Write documentation for getting started with stm32 programming using arduino ide
- [ ] Write a function to find the desired state from user
- [ ] Add battery capacity , voltage ,current meter
- [x] Write and test code for PID loop in stm32

### Ashok

- [ ] study and write code for MSP protocol
- [ ] test the code for POC(proof of concept)

### Shreyas

- [ ] Write and Test code to that dispalys error code using LED
- [ ] get altitude from the SPL06 sensor using SPEEDYBEE
- [ ] adapt the led code to run without delay
- [ ] write code for a status led that blinks every 1.5 seconds to show that the FC code is alive.
- [ ] implement pre-arm functionality on channel 5
- [ ] implement arm/disarm functionality on channel 6

### Ruhaan

- [ ] Write and test the code for Receiving values from an SBUS receiver
- [ ] Write and test the code for driving ESC's using PWM signal
- [ ] look into the DSHOT protocol
- [ ] implement failsafe Functionality
- [ ] write code for calibration of sticks

### Shubham

- [x] Write and test the code for interfacing M8n GPS with Speedybee
- [x] find the algorithm for waypoint navigation using GPS
- [x] take a look into the madgwick filter
- [x] write code for madgwick/kalamn filter after comparison
- [ ] Make the code for GPS interrupt based

### Peeyush

- [x] Write and test the code for interfacing Compass with Speedybee
- [x] find the algorithm for waypoint navigation using GPS
- [x] take a look into the kalman filter
- [x] write code for madgwick/kalamn filter after comparison

### Anubhav


- [ ] write code for interfacing SDcard slot present on speedybee
- [ ] Adapt the PID code to the main code

## Hardware Involved

- Speedybee f405v3 stack
- Pressure sensor DPS 310
- IMU - BMI270
- OSD chip - AT7456e

## Useful Resources

- https://github.com/betaflight/unified-targets
- https://www.youtube.com/watch?v=fNLxHWd0Bvg
- https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-415-stm32f407-417-stm32f427-437-and-stm32f429-439-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf
- https://www.youtube.com/watch?v=AjN58ceQaF4
- https://github.com/stm32duino/Arduino_Core_STM32/wiki/API

Typical SBUS packet looks like 0F E5 03 1F F8 C0 07 3E F0 81 0F 7C E0 03 06 F8 80 91 3D F0 81 0F 7C 00 00


- improve the SPI CODE AND I2C CODE TO GET ALL DATA AT ONCE

## Things to be implemented

- SD CARD save frequency
- all frequency parameters on top
- hardwrae timers for onshot so that it is accurate
- use the flush command
- set the temperature source to extenral for cofficients
- get spi and i2c data at once
- Implment a starting cycle paramter to increase randomness instead implment an array of pointers to non critical function through which we run the functions cyclically
- but first have to find the function because of which frequency drops to 100
