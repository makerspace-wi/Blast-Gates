/*
 * based on Copyright (C)2015-2017 Laurentiu Badea
 *
 * This file may be redistributed under the terms of the MIT license.
 * A copy of this license has been included with this distribution in the file LICENSE.

 driving schieber with stepping motors and spindel
 and comunication over xBee
 */
#include <Arduino.h>

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 495
// Microstepping mode. If you hardwired it to save pins, set to the same value here.
#define MICROSTEPS 4
// enable all steppers
#define stepperENA 4

// this pin should connect NOT to Ground when want to stop the motor
#define STOPPER_S1 A5
#define STOPPER_S2 A4
#define STOPPER_S3 3

// Steps for the stepper
#define STEP_S1 10
#define STEP_S2 13
#define STEP_S3 11
// Direction of movement
#define DIR_S1 2
#define DIR_S2 5
#define DIR_S3 7

// Acceleration and deceleration values are always in FULL steps / s^2
#define MOTOR_ACCEL 2000
#define MOTOR_DECEL 2000

#define gateMax 2785  // max distance for the two switches
#define gateDIF 2532  // difference between open and close
#define gateCheck 70  // min. steps for check position (open | close)
#define gateOPEN   1  // direction for opening gate
#define gateCLOSE -1  // direction for closing gate

byte gatePOS = 0;  // Gate position control 1 == closed

// Test switch for controll
#define taster1 A2
#define taster2 A3

#include "BasicStepperDriver.h"
BasicStepperDriver stepper1(MOTOR_STEPS, DIR_S1, STEP_S1, stepperENA);
// BasicStepperDriver stepper2(MOTOR_STEPS, DIR_S2, STEP_S2, stepperENA);
// BasicStepperDriver stepper3(MOTOR_STEPS, DIR_S3, STEP_S3, stepperENA);

void setup() {
  Serial.begin(57600);

  pinMode(taster1, INPUT_PULLUP);

  // Configure stopper pin to read HIGH unless grounded
  pinMode(STOPPER_S3, INPUT_PULLUP);

  stepper1.begin(RPM, MICROSTEPS);
  // stepper2.begin(RPM, MICROSTEPS);
  // stepper3.begin(RPM, MICROSTEPS);

  stepper1.setSpeedProfile(stepper1.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);
  // stepper2.setSpeedProfile(stepper2.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);
  // stepper3.setSpeedProfile(stepper3.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);

  Serial.println("START");
  stepper1.enable();  // comment out to keep motor powered
  stepper1.startMove(gateCLOSE * gateDIF * MICROSTEPS);
}

void loop() {
  // motor control loop - send pulse and return how long to wait until next pulse
  if (digitalRead(STOPPER_S1) == HIGH){
    stepper1.stop();
    gatePOS =1;
  }

  unsigned wait_time_micros = stepper1.nextAction();

  if (wait_time_micros <= 0) {
    stepper1.disable(); // comment out to keep motor power off
  }

  if (gatePOS == 1 && digitalRead(taster1) == LOW) {
    Serial.println("G1");
    stepper1.enable();  // comment out to keep motor powered
    stepper1.move(gateOPEN * gateCheck * MICROSTEPS);
    gatePOS =2;
    stepper1.startMove(gateOPEN * (gateDIF-gateCheck) * MICROSTEPS);
  } else if (gatePOS == 2 && digitalRead(taster1) == LOW) {
    Serial.println("G2");
    stepper1.enable();  // comment out to keep motor powered
    stepper1.startMove(gateCLOSE * gateDIF * MICROSTEPS);
    gatePOS =3;
  }
}
