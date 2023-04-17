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
- [ ] write code for madgwick/kalamn filter after comparison
- [ ] Make the code for GPS interrupt based

### Peeyush

- [x] Write and test the code for interfacing Compass with Speedybee
- [x] find the algorithm for waypoint navigation using GPS
- [] write code for madgwick/kalamn filter after comparison
- [x] take a look into the kalman filter

### Anubhav

- [ ] Write and test code for PID loop in stm32
- [ ] write code for interfacing SDcard slot present on speedybee
- [ ] Adapt the PID code to the main code such that the channel values control the angle of drone
