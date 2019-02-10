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
  'time'    - format time33.33.33 33:33:33
  'OG6'     - open gate 6
  'OG7'     - open gate 7
  'OG8'     - open gate 8
  'CG6'     - close gate 6
  'CG7'     - close gate 7
  'CG8'     - close gate 8

  last change: 08.02.2019 by Michael Muehl
  changed: add gate control with tasker and xBee communication
					 for 3 gates with check if connected.
 */
#define Version "1.0"

#include <Arduino.h>
#define _TASK_MICRO_RES
#include <TaskScheduler.h>

// PIN Assignments
// Steps for the stepper
#define STEP_S6 10
#define STEP_S7 13
#define STEP_S8 11
// Direction of movement
#define DIR_S6 2
#define DIR_S7 5
#define DIR_S8 7
// this pin should connect NOT to Ground when want to stop the motor
#define STOP_S6  A5 // FSK
#define STOP_S7  A4 // BS
#define STOP_S8   3 // DH

#define tastG6   A0 // taster switch gate 6 open / close
#define tastG7   A1 // taster switch gate 7 open / close
#define tastG8   A2 // taster switch gate 8 open / close

#define BUSError  8 // Bus error
#define xbeError 13 // Bus error

// DEFINES
// Task speck
// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
// Microstepping mode. If you hardwired it to save pins, set to the same value here.
#define MICROSTEPS 4

#define maxSPEED  50  // [50] µs
#define minSPEED 500  // [500] µs
// Acceleration and deceleration values are always in FULL steps / s^2
#define MOTOR_ACCEL (minSPEED - maxSPEED)
#define MOTOR_DECEL (minSPEED - maxSPEED)

// enable all steppers
#define stepperENA 4

#define gateStepsMM 25 // steps to drive 1 mm
#define gateOPEN  HIGH // direction for opening gate

// DEFINES
#define CLOSE2END   15 // MINUTES before activation is off
#define porTime      5 // wait seconds for sending Ident + POR

Scheduler runner;

// Callback methods prototypes
void checkXbee();        // Task connect to xBee Server
void retryPOR();        // Task connect to xBee Server
void tastCheck();        // Task connect to xBee Server
void debounceCheckLOW();        // Task connect to xBee Server
void startPOSI();        // Task connect to xBee Server

void motorStepA6();     // Task for Accel step time
void gatePosCheck6();     // Task for Linear step time
void motorStepA7();     // Task for Accel step time
void gatePosCheck7();     // Task for Linear step time
void motorStepA8();     // Task for Accel step time
void gatePosCheck8();     // Task for Linear step time

// TASKS
Task tC(TASK_SECOND / 2, TASK_FOREVER, &checkXbee);
Task tB(TASK_SECOND * 5, TASK_FOREVER, &retryPOR);   // task for debounce; added M. Muehl
Task tTA(TASK_SECOND / 2, TASK_FOREVER, &tastCheck); // task for generating linear steps
Task tDC(1, TASK_FOREVER, &debounceCheckLOW);        // task for generating linear steps
Task tSP(TASK_SECOND, TASK_FOREVER, &startPOSI);     // task for generating linear steps

// --- Blast Gates ----------
Task tS6(minSPEED, TASK_FOREVER, &motorStepA6);   // task for generating linear steps
Task tC6(TASK_SECOND / 50, TASK_FOREVER, &gatePosCheck6);   // task for generating linear steps
Task tS7(minSPEED, TASK_FOREVER, &motorStepA7);   // task for generating linear steps
Task tC7(TASK_SECOND / 50, TASK_FOREVER, &gatePosCheck7);   // task for generating linear steps
Task tS8(minSPEED, TASK_FOREVER, &motorStepA8);   // task for generating linear steps
Task tC8(TASK_SECOND / 50, TASK_FOREVER, &gatePosCheck8);   // task for generating linear steps

// VARIABLES
unsigned long gateSTEPS6 = 0;   // MICROSTEPS * gateStepsMM * 120; // 120 mm
unsigned long StepCounter6 = 0; //
unsigned long StepSpeed6 = 0;   //
boolean noGAT6 = LOW; // bit no gate 6
boolean bitAL6 = LOW; // bit ACCEL with step up & down = HIGH
boolean posRY6 = LOW; // bit position ready = HIGH
boolean posSW6 = LOW; // bit position switch = off = HIGH
boolean offSW6 = LOW; // bit position switch surch for off again
boolean gate6O = LOW; // bit gate 6 open = HIGH
boolean gate6C = LOW; // bit gate 6 close = HIGH

unsigned long gateSTEPS7 = MICROSTEPS * gateStepsMM * 120; // 120 mm
unsigned long StepCounter7 = 0;
unsigned long StepSpeed7 = 0;
boolean noGAT7 = LOW; // bit no gate 7
boolean bitAL7 = LOW; // bit ACCEL with step up & down = HIGH
boolean posRY7 = LOW; // bit position ready = HIGH
boolean posSW7 = LOW; // bit position switch = off = HIGH
boolean offSW7 = LOW; // bit position switch surch for off again
boolean gate7O = LOW; // bit gate 7 open = HIGH
boolean gate7C = LOW; // bit gate 7 close = HIGH

unsigned long gateSTEPS8 = MICROSTEPS * gateStepsMM * 120; // 120 mm
unsigned long StepCounter8 = 0;
unsigned long StepSpeed8 = 0;
boolean noGAT8 = LOW; // bit no gate 8
boolean bitAL8 = LOW; // bit ACCEL with step up & down = HIGH
boolean posRY8 = LOW; // bit position ready = HIGH
boolean posSW8 = LOW; // bit position switch = off = HIGH
boolean offSW8 = LOW; // bit position switch surch for off again
boolean gate8O = LOW; // bit gate 8 open = HIGH
boolean gate8C = LOW; // bit gate 8 close = HIGH

byte stepSP = 0;  // steps for start position

// Serial with xBee
String inStr = "";  // a string to hold incoming data
String IDENT = "";  // Machine identifier for remote access control
byte plplpl = 0;    // send +++ control AT sequenz
byte getTime = porTime;

// ======>  SET UP AREA <=====
void setup() {
  //init Serial port
  Serial.begin(57600);  // Serial
  inStr.reserve(40);    // reserve for instr serial input
  IDENT.reserve(5);     // reserve for IDENT serial output

  // PIN MODES
  // Stepper -----------
  pinMode(STEP_S6, OUTPUT);
  pinMode(STEP_S7, OUTPUT);
  pinMode(STEP_S8, OUTPUT);

  pinMode(DIR_S6, OUTPUT);
  pinMode(DIR_S7, OUTPUT);
  pinMode(DIR_S8, OUTPUT);

  pinMode(stepperENA, OUTPUT);

  pinMode(STOP_S6, INPUT_PULLUP);
  pinMode(STOP_S7, INPUT_PULLUP);
  pinMode(STOP_S8, INPUT_PULLUP);

  pinMode(tastG6, INPUT_PULLUP);
  pinMode(tastG7, INPUT_PULLUP);
  pinMode(tastG8, INPUT_PULLUP);

  pinMode(BUSError, OUTPUT);
  pinMode(xbeError, OUTPUT);

  // Set default values
  digitalWrite(BUSError, HIGH);   // turn the LED ON (init start)
  digitalWrite(xbeError, HIGH);   // turn the LED ON (Pin 13)
  digitalWrite(stepperENA, HIGH); // turn the LED ON (Pin 13)

  runner.init();
  runner.addTask(tC);
  runner.addTask(tB);
  runner.addTask(tTA);
  runner.addTask(tDC);
  runner.addTask(tSP);

  runner.addTask(tS6);
  runner.addTask(tC6);
  runner.addTask(tS7);
  runner.addTask(tC7);
  runner.addTask(tS8);
  runner.addTask(tC8);

  Serial.print("+++"); //Starting the request of IDENT
  tC.enable();
}

// FUNCTIONS (Taesks) ----------------------------
void checkXbee() {
  if (IDENT.startsWith("BGATE") && plplpl == 2) {
    ++plplpl;
    tB.enable();
    digitalWrite(xbeError, LOW); // turn the LED off (Programm start)
  }
}

void retryPOR() {
  if (getTime < porTime * 5) {
    Serial.println(String(IDENT) + ";POR");
    ++getTime;
    tB.setInterval(TASK_SECOND * getTime);
  }
  else if (getTime == 255)
  {
    digitalWrite(BUSError, LOW); // turn the LED off (Programm start)
    tB.disable();
    tC.disable();

    stepSP = 0;
    tSP.enable();
  }
}

void startPOSI() {
  switch (stepSP) {
    case 0:   // set level values to min
      bitAL6 = LOW;
      gatesCLOSE6();
      stepSP = 1;
      break;
    case 1:
      if (noGAT6 || gate6C ) stepSP = 2;
      break;
    case 2:
      bitAL7 = LOW;
      gatesCLOSE7();
      stepSP = 3;
      break;
    case 3:
      if (noGAT7 || gate7C ) stepSP = 4;
      break;
    case 4:
      bitAL8 = LOW;
      gatesCLOSE8();
      stepSP = 5;
      break;
    case 5:
      if (noGAT8 || gate8C ) {
        tTA.enable();
        tSP.disable();
        stepSP = 6;
      }
      break;
  }
}

void tastCheck() {
  if (!digitalRead(tastG6) || !digitalRead(tastG7) || !digitalRead(tastG8)) {
    tTA.disable();
    tDC.enable();
    tDC.restartDelayed(50);
  }
}

void debounceCheckLOW() {
  if (!digitalRead(tastG6) && digitalRead(tastG7) && digitalRead(tastG8) && !noGAT6) {
    if (gate6O) {
      bitAL6 = HIGH;
      gatesCLOSE6();
    } else {
      gatesOPEN6();
    }
    tDC.setCallback(&debounceCheckHIGH);
    tDC.restartDelayed(TASK_SECOND *2);
  }
  if (digitalRead(tastG6) && !digitalRead(tastG7) && digitalRead(tastG8) && !noGAT7) {
    if (gate7O) {
      bitAL7 = HIGH;
      gatesCLOSE7();
    } else {
      gatesOPEN7();
    }
    tDC.setCallback(&debounceCheckHIGH);
    tDC.restartDelayed(TASK_SECOND *2);
  }
  if (digitalRead(tastG8) && digitalRead(tastG7) && !digitalRead(tastG8) && !noGAT8) {
    if (gate8O) {
      bitAL8 = HIGH;
      gatesCLOSE8();
    } else {
      gatesOPEN8();
    }
    tDC.setCallback(&debounceCheckHIGH);
    tDC.restartDelayed(TASK_SECOND *2);
  }
}

void debounceCheckHIGH() {
  if (digitalRead(tastG6) && digitalRead(tastG7) && digitalRead(tastG8)) {
    tTA.enable();
    tDC.disable();
    tDC.setCallback(&debounceCheckLOW);
  }
}

// Tasks for GATE6: ------------------------
void motorStepA6() {
  posSW6 = digitalRead(STOP_S6);
  if (StepCounter6 >= gateSTEPS6 - MOTOR_ACCEL) {
    if (bitAL6 && StepCounter6 % (MICROSTEPS)) {
      --StepSpeed6;
      tS6.setInterval(StepSpeed6);
    }
    --StepCounter6;
    digitalWrite(STEP_S6, HIGH);
    delayMicroseconds(5);
    digitalWrite(STEP_S6, LOW);
  } else {
    tS6.setCallback(&motorStepL6);
  }
}

void motorStepL6() {
  posSW6 = digitalRead(STOP_S6);
  if (StepCounter6 > MOTOR_DECEL) {
    --StepCounter6;
    digitalWrite(STEP_S6, HIGH);
    delayMicroseconds(5);
    digitalWrite(STEP_S6, LOW);
  } else {
    tS6.setCallback(&motorStepD6);
  }
}

void motorStepD6() {
  posSW6 = digitalRead(STOP_S6);
  if (StepCounter6 >  0) {
    if (bitAL6 && StepCounter6 % (MICROSTEPS)) {
      ++StepSpeed6;
      tS6.setInterval(StepSpeed6);
    }
    --StepCounter6;
    digitalWrite(STEP_S6, HIGH);
    delayMicroseconds(5);
    digitalWrite(STEP_S6, LOW);
  } else {
    tS6.disable();
    tS6.setCallback(&motorStepA6);
    bitAL6 = LOW;
    posRY6 = HIGH; // ready position
    digitalWrite(stepperENA, HIGH);
  }
}

void gatePosCheck6() {  // drive gate and check witch position is reached
  if (posRY6 && offSW6 && posSW6) {
    tS6.disable();
    tC6.disable();
    digitalWrite(stepperENA, HIGH);
    noGAT6 = HIGH;
    Serial.println("NG6");
  } else if ((posRY6 || offSW6) && !posSW6) {
    tS6.disable();
    tC6.disable();
    digitalWrite(stepperENA, HIGH);
    if (posRY6 && !offSW6 && !posSW6) {
      Serial.println("G6O");
      gate6O = HIGH;
    }
    if (!posRY6 && offSW6 && !posSW6) {
      Serial.println("G6C");
      gate6C = HIGH;
    }
  } else if (posSW6 && !offSW6) {
    offSW6 = HIGH;
    tC6.setCallback(&gate2SWoff6);
  }
}

void gate2SWoff6() {    // drive gate to switch off endposition detection
  gateSTEPS6 = MICROSTEPS * gateStepsMM * 10;
  StepCounter6 = gateSTEPS6;
  StepSpeed6 = minSPEED * 2;
  bitAL6 = LOW;
  posRY6 = LOW;
  digitalWrite(DIR_S6, gateOPEN);
  tS6.setCallback(&motorStepA6);
  tS6.enable();   // !start steps
  tC6.setCallback(&gatePosCheck6);
  tC6.enable();   // !start steps
}

// Tasks for GATE7: ------------------------
void motorStepA7() {
  posSW7 = digitalRead(STOP_S7);
  if (StepCounter7 >= gateSTEPS7 - MOTOR_ACCEL) {
    if (bitAL7 && StepCounter7 % (MICROSTEPS)) {
      --StepSpeed7;
      tS7.setInterval(StepSpeed7);
    }
    --StepCounter7;
    digitalWrite(STEP_S7, HIGH);
    delayMicroseconds(5);
    digitalWrite(STEP_S7, LOW);
  } else {
    tS7.setCallback(&motorStepL7);
  }
}

void motorStepL7() {
  posSW7 = digitalRead(STOP_S7);
  if (StepCounter7 > MOTOR_DECEL) {
    --StepCounter7;
    digitalWrite(STEP_S7, HIGH);
    delayMicroseconds(5);
    digitalWrite(STEP_S7, LOW);
  } else {
    tS7.setCallback(&motorStepD7);
  }
}

void motorStepD7() {
  posSW7 = digitalRead(STOP_S7);
  if (StepCounter7 >  0) {
    if (bitAL7 && StepCounter7 % (MICROSTEPS)) {
      ++StepSpeed7;
      tS7.setInterval(StepSpeed7);
    }
    --StepCounter7;
    digitalWrite(STEP_S7, HIGH);
    delayMicroseconds(5);
    digitalWrite(STEP_S7, LOW);
  } else {
    tS7.disable();
    tS7.setCallback(&motorStepA7);
    bitAL7 = LOW;
    posRY7 = HIGH; // ready position
    digitalWrite(stepperENA, HIGH);
  }
}

void gatePosCheck7() {  // drive gate and check witch position is reached
  if (posRY7 && offSW7 && posSW7) {
    noGAT7 = HIGH;
    tS7.disable();
    tC7.disable();
    digitalWrite(stepperENA, HIGH);
    Serial.println("NG7");
  } else if ((posRY7 || offSW7) && !posSW7) {
    tS7.disable();
    tC7.disable();
    digitalWrite(stepperENA, HIGH);
    if (posRY7 && !posSW7 && !offSW7) {
      Serial.println("G7O");
      gate7O = HIGH;
    }
    if (!posRY7 && !posSW7 && offSW7) {
      Serial.println("G7C");
      gate7C = HIGH;
    }
  } else if (posSW7 && !offSW7) { //!posRY &&
    offSW7 = HIGH;
    tC6.setCallback(&gate2SWoff7);
  }
}

void gate2SWoff7() {    // drive gate to switch off endposition detection
  gateSTEPS7 = MICROSTEPS * gateStepsMM * 10;
  StepCounter7 = gateSTEPS7;
  StepSpeed7 = minSPEED * 2;
  bitAL7 = LOW;
  posRY7 = LOW;
  digitalWrite(DIR_S7, gateOPEN);
  tS7.setCallback(&motorStepA7);
  tS7.enable();   // !start steps
  tC7.setCallback(&gatePosCheck7);
  tC7.enable();   // !start steps
}

// Tasks for GATE8: ------------------------
void motorStepA8() {
  posSW8 = digitalRead(STOP_S8);
  if (StepCounter8 >= gateSTEPS8 - MOTOR_ACCEL) {
    if (bitAL8 && StepCounter8 % (MICROSTEPS)) {
      --StepSpeed8;
      tS8.setInterval(StepSpeed8);
    }
    --StepCounter8;
    digitalWrite(STEP_S8, HIGH);
    delayMicroseconds(5);
    digitalWrite(STEP_S8, LOW);
  } else {
    tS8.setCallback(&motorStepL8);
  }
}

void motorStepL8() {
  posSW8 = digitalRead(STOP_S8);
  if (StepCounter8 > MOTOR_DECEL) {
    --StepCounter8;
    digitalWrite(STEP_S8, HIGH);
    delayMicroseconds(5);
    digitalWrite(STEP_S8, LOW);
  } else {
    tS8.setCallback(&motorStepD8);
  }
}

void motorStepD8() {
  posSW8 = digitalRead(STOP_S8);
  if (StepCounter8 >  0) {
    if (bitAL8 && StepCounter8 % (MICROSTEPS)) {
      ++StepSpeed8;
      tS8.setInterval(StepSpeed8);
    }
    --StepCounter8;
    digitalWrite(STEP_S8, HIGH);
    delayMicroseconds(5);
    digitalWrite(STEP_S8, LOW);
  } else {
    tS8.disable();
    tS8.setCallback(&motorStepA8);
    bitAL8 = LOW;
    posRY8 = HIGH; // ready position
    digitalWrite(stepperENA, HIGH);
  }
}

void gatePosCheck8() {  // drive gate and check witch position is reached
  // Serial.println("posRY8:" + String(posRY8) + " posSW8:" + String(posSW8) + " offSW8:" + String(offSW8));
  if (posRY8 && offSW8 && posSW8) {
    noGAT8 = HIGH;
    tS8.disable();
    tC8.disable();
    digitalWrite(stepperENA, HIGH);
    Serial.println("NG8");
  } else if ((posRY8 || offSW8) && !posSW8) {
    tS8.disable();
    tC8.disable();
    digitalWrite(stepperENA, HIGH);
    if (posRY8 && !posSW8 && !offSW8) {
      Serial.println("G8O");
      gate8O = HIGH;
    }
    if (!posRY8 && !posSW8 && offSW8) {
      Serial.println("G8C");
      gate8C = HIGH;
    }
  } else if (posSW8 && !offSW8) {
    offSW8 = HIGH;
    tC8.setCallback(&gate2SWoff8);
  }
}

void gate2SWoff8() {    // drive gate to switch off endposition detection
  gateSTEPS8 = MICROSTEPS * gateStepsMM * 10;
  StepCounter8 = gateSTEPS8;
  StepSpeed8 = minSPEED * 2;
  bitAL8 = LOW;
  posRY8 = LOW;
  digitalWrite(DIR_S8, gateOPEN);
  tS8.setCallback(&motorStepA8);
  tS8.enable();   // !start steps
  tC8.setCallback(&gatePosCheck8);
  tC8.enable();   // !start steps
}
// END OF TASKS ---------------------------------

// FUNCTIONS ------------------------------------
// Function GATE6 -------------------------------
void gatesOPEN6() {
  gateSTEPS6 = MICROSTEPS * gateStepsMM * 120;	// open gate 120 mm
  StepCounter6 = gateSTEPS6;
  StepSpeed6 = minSPEED;
  bitAL6 = HIGH;
  posRY6 = LOW;
  offSW6 = LOW;
  gate6C = LOW;
  digitalWrite(DIR_S6, gateOPEN);
  tS6.setCallback(&motorStepA6);
  tS6.enable();   // !start steps
  tC6.setCallback(&gatePosCheck6);
  tC6.enable();   // !start steps
  digitalWrite(stepperENA, LOW);
}

void gatesCLOSE6() {
  gateSTEPS6 = MICROSTEPS * gateStepsMM * 125;	// close gate 125 mm
  StepCounter6 = gateSTEPS6;
  StepSpeed6 = minSPEED;
  posRY6 = LOW;
  offSW6 = LOW;
  gate6O = LOW;
  digitalWrite(DIR_S6, !gateOPEN);
  tS6.setCallback(&motorStepA6);
  tS6.enable();   // !start steps
  tC6.setCallback(&gatePosCheck6);
  tC6.enable();   // !start steps
  digitalWrite(stepperENA, LOW);
}

// Function GATE7 -------------------------------
void gatesOPEN7() {
  gateSTEPS7 = MICROSTEPS * gateStepsMM * 120;	// open gate 120 mm
  StepCounter7 = gateSTEPS7;
  StepSpeed7 = minSPEED;
  bitAL7 = HIGH;
  posRY7 = LOW;
  offSW7 = LOW;
  gate7C = LOW;
  digitalWrite(DIR_S7, gateOPEN);
  tS7.setCallback(&motorStepA7);
  tS7.enable();   // !start steps
  tC7.setCallback(&gatePosCheck7);
  tC7.enable();   // !start steps
  digitalWrite(stepperENA, LOW);
}

void gatesCLOSE7() {
  // Serial.println("posRY7:" + String(posRY7) + " posSW7:" + String(posSW7) + " offSW7:" + String(offSW7));
  gateSTEPS7 = MICROSTEPS * gateStepsMM * 125;	// close gate 125 mm
  StepCounter7 = gateSTEPS7;
  StepSpeed7 = minSPEED;
  posRY7 = LOW;
  offSW7 = LOW;
  gate7O = LOW;
  digitalWrite(DIR_S7, !gateOPEN);
  tS7.setCallback(&motorStepA7);
  tS7.enable();   // !start steps
  tC7.setCallback(&gatePosCheck7);
  tC7.enable();   // !start steps
  digitalWrite(stepperENA, LOW);
}

// Function GATE8 --------------------------
void gatesOPEN8() {
  gateSTEPS8 = MICROSTEPS * gateStepsMM * 120;	// open gate 120 mm
  StepCounter8 = gateSTEPS8;
  StepSpeed8 = minSPEED;
  bitAL8 = HIGH;
  posRY8 = LOW;
  offSW8 = LOW;
  gate8C = LOW;
  digitalWrite(DIR_S8, gateOPEN);
  tS8.setCallback(&motorStepA8);
  tS8.enable();   // !start steps
  tC8.setCallback(&gatePosCheck8);
  tC8.enable();   // !start steps
  digitalWrite(stepperENA, LOW);
}

void gatesCLOSE8() {
  gateSTEPS8 = MICROSTEPS * gateStepsMM * 125;	// close gate 125 mm
  StepCounter8 = gateSTEPS8;
  StepSpeed8 = minSPEED;
  posRY8 = LOW;
  offSW8 = LOW;
  gate8O = LOW;
  digitalWrite(DIR_S8, !gateOPEN);
  tS8.setCallback(&motorStepA8);
  tS8.enable();   // !start steps
  tC8.setCallback(&gatePosCheck8);
  tC8.enable();   // !start steps
  digitalWrite(stepperENA, LOW);
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

  if (inStr.startsWith("BGATE")) {
    Serial.println("ATCN");
    IDENT = inStr;
  }

  if (inStr.startsWith("TIME")) { // return TIME
    getTime = 255;
  }

  if (inStr.startsWith("OG6") && !noGAT6 && !gate6O) { // return TIME
    gatesOPEN6();
  }

  if (inStr.startsWith("OG7") && !noGAT7 && !gate7O) { // return TIME
    gatesOPEN7();
  }

  if (inStr.startsWith("OG8") && !noGAT8 && !gate8O) { // return TIME
    gatesOPEN8();
  }

  if (inStr.startsWith("CG6") && !noGAT6) { // return TIME
    bitAL6 = HIGH;
    gatesCLOSE6();
  }

  if (inStr.startsWith("CG7") && !noGAT7) { // return TIME
    bitAL7 = HIGH;
    gatesCLOSE7();
  }

  if (inStr.startsWith("CG8") && !noGAT8) { // return TIME
    bitAL8 = HIGH;
    gatesCLOSE8();
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
