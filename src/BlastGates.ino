/* DESCRIPTION
  ====================
  driving 3 gates for vacuum cleaner for 3 machines

  the gate is driven by steppermotor and spindle
 and an xBee for comunication.
 -Parameter of our system-------------:
  Commands to Raspi
  'GATE678' - from xBee (=Ident)
  'POR'     - machine power on reset (Ident;por)
  'G6O'     - gate 6 is open (Formatkreissaege)
  'G7O'     - gate 7 is open (Bandsaege)
  'G8O'     - gate 8 is open (Dickenhobel)
  'G6C'     - gate 6 is closed
  'G7C'     - gate 7 is closed
  'G8C'     - gate 8 is closed

  Commands from Raspi
  'time'    - format time2018,09,12,12,22,00
  'OG6'     - open gate 6
  'OG7'     - open gate 7
  'OG8'     - open gate 8
  'CG6'     - close gate 6
  'CG7'     - close gate 7
  'CG8'     - close gate 8

  last change: 24.01.2019 by Michael Muehl
  changed: add gate control with tasker and xBee communication
 */
#define Version "0.1"

#include <Arduino.h>
#include <TaskScheduler.h>
// PIN Assignments
// this pin should connect NOT to Ground when want to stop the motor
#define STOPPER_S6 A5	// FSK
#define STOPPER_S7 A4	// BS
#define STOPPER_S8 3	// DH

#define BUSError      8 // Bus error
#define xbeError     13 // Bus error

// DEFINES
// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 495
// Microstepping mode. If you hardwired it to save pins, set to the same value here.
#define MICROSTEPS 4
// enable all steppers
#define stepperENA 4


// Steps for the stepper
#define STEP_S6 10
#define STEP_S7 13
#define STEP_S8 11
// Direction of movement
#define DIR_S6 2
#define DIR_S7 5
#define DIR_S8 7

// Acceleration and deceleration values are always in FULL steps / s^2
#define MOTOR_ACCEL 2000
#define MOTOR_DECEL 2000

#define gateMax 2785  // max distance for the two switches
#define gateDIF 2532  // difference between open and close
#define gateCheck 50  // min. steps for check position (open | close)
#define gateOPEN   1  // direction for opening gate
#define gateCLOSE -1  // direction for closing gate

byte gatePOS = 0;  // Gate position control 1 == closed

#include "BasicStepperDriver.h"
BasicStepperDriver stepper6(MOTOR_STEPS, DIR_S6, STEP_S6, stepperENA);
BasicStepperDriver stepper7(MOTOR_STEPS, DIR_S7, STEP_S7, stepperENA);
BasicStepperDriver stepper8(MOTOR_STEPS, DIR_S8, STEP_S8, stepperENA);

Scheduler runner;

// Callback methods prototypes
void checkXbee();        // Task connect to xBee Server
void pixelCallback();    // Task for clock time
void lightCallback();    // Task for switch light on
void poweroffCallback(); // Task to let LED blink - added by D. Haude 08.03.2017

// TASKS
Task tP(clockTime/10, TASK_FOREVER, &pixelCallback); // main task default clock mode
Task tL(clockTime, TASK_FOREVER, &checkXbee);        // task for check xBee
Task tD(5000, TASK_FOREVER, &retryPOR);              // task for debounce; added M. Muehl

// Test switch for controll
#define taster1 A2
#define taster2 A3

// ======>  SET UP AREA <=====
void setup() {
  //init Serial port
  Serial.begin(57600);  // Serial
  inStr.reserve(40);    // reserve for instr serial input
  IDENT.reserve(5);     // reserve for instr serial input

  // initalize steppers
  stepper6.begin(RPM, MICROSTEPS);
  stepper7.begin(RPM, MICROSTEPS);
  stepper8.begin(RPM, MICROSTEPS);

  stepper6.setSpeedProfile(stepper6.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);
  stepper7.setSpeedProfile(stepper7.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);
  stepper8.setSpeedProfile(stepper8.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);

  // PIN MODES
  // Configure stopper pin to read HIGH unless grounded
  pinMode(STOPPER_S6, INPUT_PULLUP);
  pinMode(STOPPER_S7, INPUT_PULLUP);
  pinMode(STOPPER_S8, INPUT_PULLUP);

  runner.init();
  runner.addTask(tP);
  runner.addTask(tL);
  runner.addTask(tD);

  stepper6.enable();  // comment out to keep motor powered
  stepper6.startMove(gateCLOSE * gateDIF * MICROSTEPS);
  stepper7.enable();  // comment out to keep motor powered
  stepper7.startMove(gateCLOSE * gateDIF * MICROSTEPS);
  stepper8.enable();  // comment out to keep motor powered
  stepper8.startMove(gateCLOSE * gateDIF * MICROSTEPS);

  pinMode(taster1, INPUT_PULLUP);  // only test

  Serial.print("+++"); //Starting the request of IDENT
  tP.enable();
  tL.enable();
}

// FUNCTIONS (Tasks) ----------------------------
void checkXbee() {
  if (IDENT.startsWith("GATE678") && plplpl == 2) {
    ++plplpl;
    tD.enable();
    digitalWrite(xbeError, LOW); // turn the LED off (Programm start)
  }
}

void retryPOR() {
  if (getTime < porTime * 5) {
    Serial.println(String(IDENT) + ";POR");
    ++getTime;
    tD.setInterval(getTime * SECONDS);
  }
  else if (getTime == 255)
  {
    tL.setCallback(lightCallback);
    // set tD to debounce
    tD.setCallback(debounceCallback);
    tD.setInterval(clockTime/10);
    tD.disable();
    digitalWrite(BUSError, LOW); // turn the LED off (Programm start)
  }

  // Only for Test!!! ---> ------------
  if (getTime == 8)
  {
    inStr = "time33.33.3333 33:33:33 ";
    evalSerialData();
  }
  // <--- Test ------------------------
}

  // motor control loop - send pulse and return how long to wait until next pulse
  if (digitalRead(STOPPER_S6) == HIGH){
    stepper6.stop();
    gatePOS =1;
  }

  unsigned wait_time_micros = stepper6.nextAction();

  if (wait_time_micros <= 0) {
    stepper6.disable(); // comment out to keep motor power off
  }
// END OF TASKS ---------------------------------

// FUNCTIONS ------------------------------------
// clock mode
void clockMode() {
  if (gatePOS == 1 && digitalRead(taster1) == LOW) {
    Serial.println("G1");
    stepper6.enable();  // comment out to keep motor powered
    stepper6.move(gateOPEN * gateCheck * MICROSTEPS);
    gatePOS =2;
    stepper6.startMove(gateOPEN * (gateDIF-gateCheck) * MICROSTEPS);
  } else if (gatePOS == 2 && digitalRead(taster1) == LOW) {
    Serial.println("G2");
    stepper6.enable();  // comment out to keep motor powered
    stepper6.startMove(gateCLOSE * gateDIF * MICROSTEPS);
    gatePOS =3;
  }
}
// End Funktions --------------------------------

// Funktions Serial Input (Event) ---------------
void evalSerialData() {
  inStr.toUpperCase();

  if (inStr.startsWith("OK")) {
    if (plplpl == 0) {
      ++plplpl;
      Serial.println("ATNI");
    } else {
      ++plplpl;
    }
  }

  if (inStr.startsWith("SENSOR")) {
    Serial.println("ATCN");
    IDENT = inStr;
  }

  if (inStr.startsWith("TIME")) { // adjust TIME
    setClock(inStr.substring(4));
    tL.setInterval(500);
    getTime = 255;
}

  if (inStr.startsWith("CM")) { // set to Clock Mode
    mode=0;
    tP.setInterval(clockTime/10);
  }

  if (inStr.startsWith("CC")) { // shows color (rgb) cycle
    mode=1;
    tP.setInterval(clockTime/10);
  }

  if (inStr.startsWith("BM")) { //bouncing mode (shows bouncing colors)
    mode=2;
    tP.setInterval(clockTime);
  }

  if (inStr.startsWith("AM")) { //ambient mode (clock shows defined color)
    mode=3;
    tP.setInterval(clockTime/10);
  }
  if (inStr.startsWith("GA")) { //alert mode - Green Alert (clock flashes orange)
    alert = 0;
  }

  if (inStr.startsWith("OA")) { //alert mode - orange alert (clock flashes orange)
    alert = 1;
  }

  if (inStr.startsWith("CO")) { // set Clock Option X minute dots
    if (coptionfivemin) {
      coptionfivemin = 0;
    } else {
      coptionfivemin = 1;
    }
  }
}

/* SerialEvent occurs whenever a new data comes in the
  hardware serial RX.  This routine is run between each
  time loop() runs, so using delay inside loop can delay
  response.  Multiple bytes of data may be available.
*/
void serialEvent() {
  char inChar = (char)Serial.read();
  if (inChar == '\x0d') {
    evalSerialData();
    inStr = "";
  } else {
    inStr += inChar;
  }
}
// End Funktions Serial Input -------------------

// PROGRAM LOOP AREA ----------------------------
void loop() {
  runner.execute();
}
