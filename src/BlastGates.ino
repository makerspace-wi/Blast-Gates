/* DESCRIPTION
  ====================
  driving 3 gates for vacuum cleaner for 3 machines

  the gate is driven by steppermotor and spindle
 and an xBee for comunication.
 -Parameter of our system-------------:
  Commands to Raspi
  'GATE123' - from xBee (=Ident)
  'POR'     - machine power on reset (Ident;por)
  'G1O'     - gate 1 is open
  'G2O'     - gate 2 is open
  'G3O'     - gate 3 is open
  'G1C'     - gate 1 is closed
  'G2C'     - gate 2 is closed
  'G3C'     - gate 3 is closed

  Commands from Raspi
  'time'    - format time2018,09,12,12,22,00
  'OG1'     - open gate 1
  'OG2'     - open gate 2
  'OG3'     - open gate 3
  'CG1'     - close gate 1
  'CG2'     - close gate 2
  'CG3'     - close gate 3
  last change: 20.01.2019 by Michael Muehl
  changed: add gate control with tasker and xBee communication
 */
#define Version "0.1"

#include <Arduino.h>
#include <TaskScheduler.h>
// PIN Assignments
// this pin should connect NOT to Ground when want to stop the motor
#define STOPPER_S1 A5
#define STOPPER_S2 A4
#define STOPPER_S3 3

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
#define gateCheck 50  // min. steps for check position (open | close)
#define gateOPEN   1  // direction for opening gate
#define gateCLOSE -1  // direction for closing gate

byte gatePOS = 0;  // Gate position control 1 == closed

#include "BasicStepperDriver.h"
BasicStepperDriver stepper1(MOTOR_STEPS, DIR_S1, STEP_S1, stepperENA);
BasicStepperDriver stepper2(MOTOR_STEPS, DIR_S2, STEP_S2, stepperENA);
BasicStepperDriver stepper3(MOTOR_STEPS, DIR_S3, STEP_S3, stepperENA);

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
  stepper1.begin(RPM, MICROSTEPS);
  stepper2.begin(RPM, MICROSTEPS);
  stepper3.begin(RPM, MICROSTEPS);

  stepper1.setSpeedProfile(stepper1.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);
  stepper2.setSpeedProfile(stepper2.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);
  stepper3.setSpeedProfile(stepper3.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);

  // PIN MODES
  pinMode(taster1, INPUT_PULLUP);

  // Configure stopper pin to read HIGH unless grounded
  pinMode(STOPPER_S1, INPUT_PULLUP);
  pinMode(STOPPER_S2, INPUT_PULLUP);
  pinMode(STOPPER_S3, INPUT_PULLUP);

  runner.init();
  runner.addTask(tP);
  runner.addTask(tL);
  runner.addTask(tD);

  Serial.println("START");    // only for test
  stepper1.enable();  // comment out to keep motor powered
  stepper1.startMove(gateCLOSE * gateDIF * MICROSTEPS);
  stepper2.enable();  // comment out to keep motor powered
  stepper2.startMove(gateCLOSE * gateDIF * MICROSTEPS);
  stepper3.enable();  // comment out to keep motor powered
  stepper3.startMove(gateCLOSE * gateDIF * MICROSTEPS);

  Serial.print("+++"); //Starting the request of IDENT
  tP.enable();
  tL.enable();
}

// FUNCTIONS (Tasks) ----------------------------
void checkXbee() {
  if (IDENT.startsWith("SENSOR") && plplpl == 2) {
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
  if (digitalRead(STOPPER_S1) == HIGH){
    stepper1.stop();
    gatePOS =1;
  }

  unsigned wait_time_micros = stepper1.nextAction();

  if (wait_time_micros <= 0) {
    stepper1.disable(); // comment out to keep motor power off
  }

// END OF TASKS ---------------------------------

// FUNCTIONS ------------------------------------
// clock mode
void clockMode() {
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
