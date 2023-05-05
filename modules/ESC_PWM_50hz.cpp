#include"ESC_PWM_50hz.h"

void calibrateESC()
{
  motor_1.writeMicroseconds(1200); // writing a 1200us pulse so motors dont initialize
  motor_2.writeMicroseconds(1200);
  motor_3.writeMicroseconds(1200);
  motor_4.writeMicroseconds(1200);
  while (!Serial)
  {
  }
  Serial.println("READY - PLEASE SEND INSTRUCTIONS AS FOLLOWING :");
  Serial.println("\t0 : Send min throttle");
  Serial.println("\t1 : Send max throttle");
  Serial.println("\t2 : Run test function");
  Serial.println("\t3 : Continue the FC code\n");

  while (true)
  {
    if (Serial.available())
    {
      data = Serial.read();

      switch (data)
      {
      // 0
      case 48:
        Serial.println("Sending minimum throttle");
        motor_1.writeMicroseconds(MIN_PULSE_LENGTH);
        motor_2.writeMicroseconds(MIN_PULSE_LENGTH);
        motor_3.writeMicroseconds(MIN_PULSE_LENGTH);
        motor_4.writeMicroseconds(MIN_PULSE_LENGTH);
        break;

      // 1
      case 49:
        Serial.println("Sending maximum throttle");
        motor_1.writeMicroseconds(MAX_PULSE_LENGTH);
        motor_2.writeMicroseconds(MAX_PULSE_LENGTH);
        motor_3.writeMicroseconds(MAX_PULSE_LENGTH);
        motor_4.writeMicroseconds(MAX_PULSE_LENGTH);
        break;

      // 2
      case 50:
        Serial.print("Running test in 3");
        delay(1000);
        Serial.print(" 2");
        delay(1000);
        Serial.println(" 1...");
        delay(1000);
        for (int i = MIN_PULSE_LENGTH; i <= MAX_PULSE_LENGTH; i += 5)
        {
          Serial.print("Pulse length = ");
          Serial.println(i);

          motor_1.writeMicroseconds(i);
          motor_2.writeMicroseconds(i);
          motor_3.writeMicroseconds(i);
          motor_4.writeMicroseconds(i);

          delay(200);
        }

        Serial.println("STOP");
        motor_1.writeMicroseconds(MIN_PULSE_LENGTH);
        motor_2.writeMicroseconds(MIN_PULSE_LENGTH);
        motor_3.writeMicroseconds(MIN_PULSE_LENGTH);
        motor_4.writeMicroseconds(MIN_PULSE_LENGTH);

        break;

      case 51:
        return;
        break; // unreacheable code
      }
    }
  }
}

void commandMotors()
{ /*DESCRIPTION : This function constraints and transmits the values to the ESC*/
  m1_command_PWM = m1_command_scaled * 1000 + 1000;
  m2_command_PWM = m2_command_scaled * 1000 + 1000;
  m3_command_PWM = m3_command_scaled * 1000 + 1000;
  m4_command_PWM = m4_command_scaled * 1000 + 1000;
  m5_command_PWM = m5_command_scaled * 1000 + 1000;
  m6_command_PWM = m6_command_scaled * 1000 + 1000;
  // Constrain commands to motors to the max and min limits
  m1_command_PWM = constrain(m1_command_PWM, 1000, 2000);
  m2_command_PWM = constrain(m2_command_PWM, 1000, 2000);
  m3_command_PWM = constrain(m3_command_PWM, 1000, 2000);
  m4_command_PWM = constrain(m4_command_PWM, 1000, 2000);
  m5_command_PWM = constrain(m5_command_PWM, 1000, 2000);
  m6_command_PWM = constrain(m6_command_PWM, 1000, 2000);

  // NOW generate PWM pulse for motors
  motor_1.writeMicroseconds(m1_command_PWM);
  motor_2.writeMicroseconds(m2_command_PWM);
  motor_3.writeMicroseconds(m3_command_PWM);
  motor_4.writeMicroseconds(m4_command_PWM);
  // motor_5.writeMicroseconds(m5_command_PWM);
  // motor_6.writeMicroseconds(m6_command_PWM);
}