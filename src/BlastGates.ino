/* DESCRIPTION
  ====================
  control 4(1) blast gates for vacuum cleaner for 4 machines and manuell

  the gate is driven by hand, controller and xBee for
  comunication with machines (ID):
  Formatkreissaege (6), Bandsaege (7), Dickenhobel (8), Schleifer (9)
  and a gate for manuell cleaning (H)
 -Parameter of our system-------------:
  Commands to Raspi --->
  'BGATE' - from xBee (=Ident)
  'POR'   - machine power on reset (Ident;por)
  ---------- (_=6,7,8,9)
  'g_O'   - Gate is Open
  'gho'   - Gate By hand is open (Manuell)
  'g_c'   - Gate is Closed
  'ghc    - gate By hand is closed
  'g_x'   - Gate position not defined: X
  'ghx'   - Gate Hand position not defined: X
  'err'   - Error message is following ('ERR:G7O')
  'gok'   - all blast gates are ok, in write position
  '_ok'   - gate nr._ is ok

  Commands from Raspi <---
  'time'  - format time33.33.33 33:33:33
  ---------- (_=6,7,8,9)
  'li_'   - log in for machine
  'lo_'   - log out for machine
  'do_'   - Dust collector On for machine
  'df_'   - Dust collector oFf for machine
  'ng_'   - No Gate is available

  last change: 01.08.2019 by Michael Muehl
  changed: add gate control with tasker and xBee communication
					 for 4 + hand gates controlled manually.
 */
#define Version "0.x"
#define _TASK_MICRO_RES
// ---------------------
#include <Arduino.h>
#include <TaskScheduler.h>

// PIN Assignments
// Steps for the stepper
#define STEPS_G6 11	// stepper steps Gate 6
#define STEPS_G7 13 // stepper steps Gate 7
#define STEPS_G8  7	// stepper steps Gate 8
#define STEPS_G9  2	// stepper steps Gate 9
// Direction of movement
#define DIR_G6    4	// dir stepper Gate 6
#define DIR_G7    3	// dir stepper Gate 7
#define DIR_G8    9	// dir stepper Gate 8
#define DIR_G9    5	// dir stepper Gate 9
// enable 2 steppers
#define enaStGate 6	// enable stepper Gates

// this pin should connect NOT to Ground when want to stop the motor
#define EndPoG6C 10 // Endpostion Gate 6 close
#define EndPoG6O A5 // Endpostion Gate 6 open
#define EndPoG7C 12 // Endpostion Gate 7 close
#define EndPoG7O A4 // Endpostion Gate 7 open
#define EndPoG8C A7 // Endpostion Gate 8 close
#define EndPoG8O A6 // Endpostion Gate 8 open
#define EndPoG9C A1 // Endpostion Gate 9 close
#define EndPoG9O A0 // Endpostion Gate 9 open
#define EndPoGHC  5 // Endpostion Gate by hand close (STEPS_G8)
#define EndPoGHO  2 // Endpostion Gate by hand open  (DIR_G8)

#define SSR_Vac  A2 // SSR Dust on / off  (Dust Collector)
#define SIGError A3 // SIGError for error
#define BUSError  8 // Bus error
#define xbeError  8 // Bus error (13)

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

#define gateStepsMM 25 // steps to drive 1 mm
#define gateOPEN  HIGH // direction for opening gate

// DEFINES
#define porTime         5 // wait seconds for sending Ident + POR
#define intervalCLMn   30 // min clean time in seconds
#define intervalCLMx   10 * 60 // max clean time in seconds

Scheduler runner;

// Callback methods prototypes
void checkXbee();     // Task connect to xBee Server
void retryPOR();      // Task connect to xBee Server
void gateChange();    // Task connect to xBee Server

// TASKS
Task tC(TASK_SECOND / 2, TASK_FOREVER, &checkXbee);
Task tB(TASK_SECOND * 5, TASK_FOREVER, &retryPOR);   // task for debounce; added M. Muehl

// --- Blast Gates ----------
Task tGC(TASK_SECOND / 4, TASK_FOREVER, &gateChange);  // task for Gate Change position
Task tGS(TASK_SECOND, TASK_FOREVER, &gateStart);  // task for Gate start
Task tGD(TASK_SECOND / 2, TASK_FOREVER, &gateDustco);  // task for Gate Dust collector
Task tGL(TASK_SECOND, TASK_FOREVER, &gateLogin);  // task for Gate Dust collector

// VARIABLES
boolean noGAT6 = LOW; // bit no gate 6
boolean gate6O = LOW; // bit gate 6 open = HIGH
boolean gate6C = LOW; // bit gate 6 close = HIGH
boolean dustC6 = LOW; // bit dust Collector for machine 6 = HIGH
boolean logIM6 = LOW; // bit log in machine 6 = HIGH
boolean logOM6 = LOW; // bit log out machine 6 = HIGH

boolean noGAT7 = LOW; // bit no gate 7
boolean gate7O = LOW; // bit gate 7 open = HIGH
boolean gate7C = LOW; // bit gate 7 close = HIGH
boolean dustC7 = LOW; // bit dust Collector for machine 7 = HIGH
boolean logIM7 = LOW; // bit log in for machine 7 = HIGH
boolean logOM7 = LOW; // bit log in for machine 7 = HIGH

boolean noGAT8 = LOW; // bit no gate 8
boolean gate8O = LOW; // bit gate 8 open = HIGH
boolean gate8C = LOW; // bit gate 8 close = HIGH
boolean dustC8 = LOW; // bit dust Collector for machine 8 = HIGH
boolean logIM8 = LOW; // bit log in machine 8 = HIGH
boolean logOM8 = LOW; // bit log out machine 8 = HIGH

boolean noGAT9 = LOW; // bit no gate 9
boolean gate9O = LOW; // bit gate 9 open = HIGH
boolean gate9C = LOW; // bit gate 9 close = HIGH
boolean dustC9 = LOW; // bit dust Collector for machine 9 = HIGH
boolean logIM9 = LOW; // bit log in machine 9 = HIGH
boolean logOM9 = LOW; // bit log out machine 9 = HIGH

//boolean noGATH = LOW; // bit no gate Hand
boolean gateHO = LOW; // bit gate By hand open = HIGH
boolean gateHC = LOW; // bit gate By hand close = HIGH

byte stepSP = 0;  // steps for start position

unsigned int dustCount = 0; // counter how long dust collektor must be switch off

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
  pinMode(STEPS_G6, OUTPUT);
  pinMode(STEPS_G7, OUTPUT);
  pinMode(STEPS_G8, OUTPUT);
  pinMode(STEPS_G9, OUTPUT);

  pinMode(DIR_G6, OUTPUT);
  pinMode(DIR_G7, OUTPUT);
  pinMode(DIR_G8, OUTPUT);
  pinMode(DIR_G9, OUTPUT);

  pinMode(enaStGate, OUTPUT);

  pinMode(EndPoG6C, INPUT);
  pinMode(EndPoG6O, INPUT);
  pinMode(EndPoG7C, INPUT);
  pinMode(EndPoG7O, INPUT);

  //pinMode(EndPoG8C, analogRead(uint8_t);  // Input?
  //pinMode(EndPoG8O, analogRead(uint8_t);  // Input?

  pinMode(EndPoG9C, INPUT);
  pinMode(EndPoG9O, INPUT);

  pinMode(EndPoGHC, INPUT);
  pinMode(EndPoGHO, INPUT);

  pinMode(SSR_Vac, OUTPUT);

  pinMode(SIGError, OUTPUT);
  pinMode(BUSError, OUTPUT);
  pinMode(xbeError, OUTPUT);

  // Set default values
  digitalWrite(SSR_Vac, LOW);   // turn off SSR_VAC
  digitalWrite(SIGError, HIGH);  // turn Error SIGnal off

  digitalWrite(BUSError, HIGH); // turn the LED ON (init start)
  digitalWrite(xbeError, HIGH); // turn the LED ON (Pin 13)

  digitalWrite(STEPS_G7, LOW);   // turn off SSR_VAC
  // digitalWrite(EndPoG8C, LOW);   // turn off SSR_VAC
  // digitalWrite(EndPoG8O, LOW);   // turn off SSR_VAC

  runner.init();
  runner.addTask(tC);
  runner.addTask(tB);
  runner.addTask(tGC);
  runner.addTask(tGS);
  runner.addTask(tGD);
  runner.addTask(tGL);

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

    // Set value for first read
    gate6O = HIGH;
    gate6C = LOW;
    gate7O = HIGH;
    gate7C = LOW;
    gate8O = HIGH;
    gate8C = LOW;
    gate9O = HIGH;
    gate9C = LOW;
    gateHO = HIGH;
    gateHC = LOW;

    dustC6 = LOW;
    dustC7 = LOW;
    dustC8 = LOW;
    dustC9 = LOW;

    logIM6 = LOW;
    logIM7 = LOW;
    logIM8 = LOW;
    logIM9 = LOW;

    stepSP = 0;
    tGC.enable();
    tGS.enable();
  }
}

// Task Gates popsition: -------------------
void gateChange() {
  if (digitalRead(EndPoG6O) != gate6O || digitalRead(EndPoG6C) != gate6C) {
    gate6O = digitalRead(EndPoG6O);
    gate6C = digitalRead(EndPoG6C);
    if (gate6O == HIGH && gate6C == LOW) {
      Serial.println("G6O");
    } else if (gate6O == LOW && gate6C == HIGH) {
      Serial.println("G6C");
    } else {
      Serial.println("G6X");
    }
  }
  if (digitalRead(EndPoG7O) != gate7O || digitalRead(EndPoG7C) != gate7C) {
    gate7O = digitalRead(EndPoG7O);
    gate7C = digitalRead(EndPoG7C);
    if (gate7O == HIGH && gate7C == LOW) {
      Serial.println("G7O");
    } else if (gate7O == LOW && gate7C == HIGH) {
      Serial.println("G7C");
    } else {
      Serial.println("G7X");
    }
  }
  if (readDigital(EndPoG8O) != gate8O || readDigital(EndPoG8C) != gate8C) {
    gate8O = readDigital(EndPoG8O);
    gate8C = readDigital(EndPoG8C);
    if (gate8O == HIGH && gate8C == LOW) {
      Serial.println("G8O");
    } else if (gate8O == LOW && gate8C == HIGH) {
      Serial.println("G8C");
    } else {
      Serial.println("G8X");
    }
  }
  if (digitalRead(EndPoG9O) != gate9O || digitalRead(EndPoG9C) != gate9C) {
    gate9O = digitalRead(EndPoG9O);
    gate9C = digitalRead(EndPoG9C);
    if (gate9O == HIGH && gate9C == LOW) {
      Serial.println("G9O");
    } else if (gate9O == LOW && gate9C == HIGH) {
      Serial.println("G9C");
    } else {
      Serial.println("G9X");
    }
  }
  if (digitalRead(EndPoGHO) != gateHO || digitalRead(EndPoGHC) != gateHC) {
    gateHO = digitalRead(EndPoGHO);
    gateHC = digitalRead(EndPoGHC);
    if (gateHO == HIGH && gateHC == LOW) {
      Serial.println("GHO");
    } else if (gateHO == LOW && gateHC == HIGH) {
      Serial.println("GHC");
    } else {
      Serial.println("GHX");
    }
  }
}

// Task Gate start: ------------------------
void gateStart() {
  if (gate6O || gate7O || gate8O || gate9O || gateHO) {
    digitalWrite(SIGError, LOW);
    if (gate6O) Serial.println("G6O");
    if (gate7O) Serial.println("G7O");
    if (gate8O) Serial.println("G8O");
    if (gate9O) Serial.println("G9O");
    if (gateHO) Serial.println("GHO");
    delay(125);
    digitalWrite(SIGError, HIGH);
  } else if (gate6C && gate7C && gate8C && gate9C && gateHC) {
    digitalWrite(SIGError, LOW);
    Serial.println("GOK");  // gates ok, all in position
    tGS.disable();
    tGD.enable();
    tGL.enable();
  } else {
    digitalWrite(SIGError, LOW);
    if (!gate6C) Serial.println("G6X");
    if (!gate7C) Serial.println("G7X");
    if (!gate8C) Serial.println("G8X");
    if (!gate9C) Serial.println("G9X");
    if (!gateHC) Serial.println("GHX");
    delay(500);
    digitalWrite(SIGError, HIGH);
  }
}

// Task Gate Dust on?: ---------------------
void gateDustco() {
  if ((gate6O || gate7O || gate8O || gate9O) && gateHC) {
    // control dust collector over machine 6 - 9
    if (dustC6 || dustC7 || dustC8 || dustC9) {
      digitalWrite(SSR_Vac, HIGH);
    } else if (!dustC6 && !dustC7 && !dustC8 && !dustC9) {
      digitalWrite(SSR_Vac, LOW);
    }
    dustCount = 0;
  } else if (gate6C && gate7C && gate8C && gate9C && gateHO && !gateHC) { //&& (dustCount > intervalCLMn)
    digitalWrite(SSR_Vac, HIGH);
    dustCount = 0;
  } else if (gate6C && gate7C && gate8C && gate9C && (!gateHO || gateHC)) {
    digitalWrite(SSR_Vac, LOW);
    ++dustCount;
  } else {
    dustC6 = LOW;
    dustC7 = LOW;
    dustC8 = LOW;
    dustC9 = LOW;
    digitalWrite(SSR_Vac, LOW);
    // error blink
    // check error nr and send message
    // after wait time
  }
}

// Task Gate Dust on?: ---------------------
void gateLogin() {
  if (!gate6O && logIM6 && !logOM6) {
    Serial.println("ERR:G6O");
  } else if (!gate6C && logIM6 && logOM6) {
    Serial.println("ERR:G6C");
  } else if (gate6C && logIM6 && logOM6) {
    logIM6 = LOW;
    logOM6 = LOW;
  }
  if (!gate7O && logIM7 && !logOM7) {
    Serial.println("ERR:G7O");
  } else if (!gate7C && logIM7 && logOM7) {
    Serial.println("ERR:G7C");
  } else if (gate7C && logIM7 && logOM7) {
    logIM7 = LOW;
    logOM7 = LOW;
  }
  if (!gate8O && logIM8 && !logOM8) {
    Serial.println("ERR:G8O");
  } else if (!gate8C && logIM8 && logOM8) {
    Serial.println("ERR:G8C");
  } else if (gate8C && logIM8 && logOM8) {
    logIM8 = LOW;
    logOM8 = LOW;
  }
  if (!gate9O && logIM9 && !logOM9) {
    Serial.println("ERR:G9O");
  } else if (!gate9C && logIM9 && logOM9) {
    Serial.println("ERR:G9C");
  } else if (gate9C && logIM9 && logOM9) {
    logIM9 = LOW;
    logOM9 = LOW;
  }
}
// END OF TASKS ---------------------------------

// FUNCTIONS ------------------------------------
// Convert analog signal in digital boolean values
boolean readDigital(int inputPin) {
  if (analogRead(inputPin) > 512) {
    return 1;
  } else {
    return 0;
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

  if (inStr.startsWith("BGATE")) {
    Serial.println("ATCN");
    IDENT = inStr;
  }

  if (inStr.startsWith("TIME")) { // return TIME
    getTime = 255;
  }

  if (inStr.startsWith("LI6") && !logOM6) logIM6 = HIGH;

  if (inStr.startsWith("LI7") && !logOM7) logIM7 = HIGH;

  if (inStr.startsWith("LI8") && !logOM8) logIM8 = HIGH;

  if (inStr.startsWith("LI9") && !logOM9) logIM9 = HIGH;

  if (inStr.startsWith("LO6") && logIM6) logOM6 = HIGH;

  if (inStr.startsWith("LO7") && logIM7) logOM7 = HIGH;

  if (inStr.startsWith("LO8") && logIM8) logOM8 = HIGH;

  if (inStr.startsWith("LO9") && logIM9
) logOM9 = HIGH;

  if (inStr.startsWith("DO6") && !noGAT6) {
    if (gate6O && !gate6C && logIM6 && !logOM6) dustC6 = HIGH;
  }

  if (inStr.startsWith("DO7") && !noGAT7) {
    if (gate7O && !gate7C && logIM7 && !logOM7) dustC7 = HIGH;
  }

  if (inStr.startsWith("DO8") && !noGAT8) {
    if (gate8O && !gate8C && logIM8 && !logOM8) dustC8 = HIGH;
  }

  if (inStr.startsWith("DO9") && !noGAT9) {
    if (gate9O && !gate9C && logIM9 && !logOM9) dustC9 = HIGH;
  }

  if (inStr.startsWith("DF6") && !noGAT6) {
    if (gate6O && !gate6C && logIM6 && !logOM6) dustC6 = LOW;
  }

  if (inStr.startsWith("DF7") && !noGAT7) {
    if (gate7O && !gate7C && logIM7 && !logOM7) dustC7 = LOW;
  }

  if (inStr.startsWith("DF8") && !noGAT8) {
    if (gate8O && !gate8C && logIM8 && !logOM8) dustC8 = LOW;
  }

  if (inStr.startsWith("DF9") && !noGAT9) {
    if (gate9O && !gate9C && logIM9 && !logOM9) dustC9 = LOW;
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
