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
  'g_o'   - Gate is Open
  'gho'   - Gate By hand is open (Manuell)
  'g_c'   - Gate is Closed
  'ghc'   - gate By hand is closed
  'g_x'   - Gate position not defined: X
  'ghx'   - Gate Hand position not defined: X
  'err'   - Error message is following ('ERR:G7O')
  'gok'   - all blast gates are ok, in write position

  Commands from Raspi <---
  'time'  - format time33.33.33 33:33:33

  ---------- (_=6,7,8,9)
  'li_'   - log in for machine
  'lo_'   - log out for machine
  'do_'   - Dust collector On for machine
  'df_'   - Dust collector oFf for machine
  'ng_'   - No Gate is available

  last change: 25.08.2019 by Michael Muehl
  changed: add gate control with tasker and xBee communication
  for 4 + hand gates controlled manually.
*/
#define Version "1.0" // _ (Test)

//#define _TASK_MICRO_RES // only for SM
// ---------------------
#include <Arduino.h>
#include <TaskScheduler.h>

// PIN Assignments
// Steps for the stepper
#define STEPS_G6 11	// stepper steps Gate 6
#define STEPS_G7  3 // stepper steps Gate 7
#define STEPS_G8  7	// stepper steps Gate 8
#define STEPS_G9  2	// stepper steps Gate 9
// Direction of movement
#define DIR_G6   13	// dir stepper Gate 6
#define DIR_G7    4	// dir stepper Gate 7
#define DIR_G8    9	// dir stepper Gate 8
#define DIR_G9    5	// dir stepper Gate 9
// enable 2 steppers
#define enaStGate 6	// enable stepper Gates

// this pin should connect NOT to Ground when want to stop the motor
#define EndPoG6C 10 // Endpostion Gate 6 close
#define EndPoG6O A5 // Endpostion Gate 6 open (only with endswitch open)
#define EndPoG7C 12 // Endpostion Gate 7 close
#define EndPoG7O A4 // Endpostion Gate 7 open (only with endswitch open)
#define EndPoG8C A1 // Endpostion Gate 8 close
#define EndPoG8O  5 // Endpostion Gate 8 open (only with endswitch open)
#define EndPoG9C A0 // Endpostion Gate 9 close
#define EndPoG9O  2 // Endpostion Gate 9 open (only with endswitch open)
#define EndPoGHC A7 // Endpostion Gate by hand close (Only analog input)
#define EndPoGHO A6 // Endpostion Gate by hand open (Only analog input)

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
Task tB(TASK_SECOND * 5, TASK_FOREVER, &retryPOR);    // task for debounce; added M. Muehl

// --- Blast Gates ----------
Task tGS(TASK_SECOND, TASK_FOREVER, &gateStart);      // task for Gate start
Task tGC(TASK_SECOND / 4, TASK_FOREVER, &gateChange); // task for Gate Change position
Task tER(TASK_SECOND / 4, TASK_FOREVER, &gateERR);    // task for Gate ERRor

Task tGM(1, TASK_ONCE, &gateMA);  // task for Gate machines
Task tGH(1, TASK_ONCE, &gateHA);  // task for Gate hand

// VARIABLES
boolean noGAT6 = LOW; // bit no gate 6
boolean dustC6 = LOW; // bit dust Collector for machine 6 = HIGH
boolean logIM6 = LOW; // bit log in machine 6 = HIGH
boolean gate6O = LOW; // bit gate 6 open = HIGH
boolean gate6C = LOW; // bit gate 6 close = HIGH

boolean noGAT7 = LOW; // bit no gate 7
boolean dustC7 = LOW; // bit dust Collector for machine 7 = HIGH
boolean logIM7 = LOW; // bit log in for machine 7 = HIGH
boolean gate7O = LOW; // bit gate 7 open = HIGH
boolean gate7C = LOW; // bit gate 7 close = HIGH

boolean noGAT8 = LOW; // bit no gate 8
boolean dustC8 = LOW; // bit dust Collector for machine 8 = HIGH
boolean logIM8 = LOW; // bit log in machine 8 = HIGH
boolean gate8O = LOW; // bit gate 8 open = HIGH
boolean gate8C = LOW; // bit gate 8 close = HIGH

boolean noGAT9 = LOW; // bit no gate 9
boolean dustC9 = LOW; // bit dust Collector for machine 9 = HIGH
boolean logIM9 = LOW; // bit log in machine 9 = HIGH
boolean gate9O = LOW; // bit gate 9 open = HIGH
boolean gate9C = LOW; // bit gate 9 close = HIGH

//boolean noGATH = LOW; // bit no gate Hand
boolean gateHO = LOW; // bit gate By hand open = HIGH
boolean gateHC = LOW; // bit gate By hand close = HIGH

byte stepSP = 0;  // steps for start position

unsigned int dustWaitT = 0; // counter how long dust wait time
byte dustCount = 0; // how many dust are on
byte errCount = 0;  // how many errors are active
int gateNR = 0;     // "0" = no gates, 6,7,8,9 with gate

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
//  pinMode(STEPS_G9, OUTPUT); // not on ES

  pinMode(DIR_G6, OUTPUT);
  pinMode(DIR_G7, OUTPUT);
  pinMode(DIR_G8, OUTPUT);
//  pinMode(DIR_G9, OUTPUT); // not on ES

  pinMode(enaStGate, OUTPUT);

  pinMode(EndPoG6C, INPUT);
  pinMode(EndPoG6O, INPUT); // Switch input ES
  pinMode(EndPoG7C, INPUT);
  pinMode(EndPoG7O, INPUT); // Switch input ES

  pinMode(EndPoG8C, INPUT);
  pinMode(EndPoG8O, INPUT); // Switch input ES

  pinMode(EndPoG9C, INPUT);
  pinMode(EndPoG9O, INPUT); // Switch input ES

  pinMode(EndPoGHC, INPUT);
  pinMode(EndPoGHO, INPUT);

  pinMode(SSR_Vac, OUTPUT);

  pinMode(SIGError, OUTPUT);
  pinMode(BUSError, OUTPUT);
  pinMode(xbeError, OUTPUT);

  // Set default values
  digitalWrite(SSR_Vac,  LOW);  // turn off SSR_VAC
  digitalWrite(SIGError, LOW);  // turn Error SIGnal off

  digitalWrite(BUSError, HIGH); // turn the LED ON (init start)
  digitalWrite(xbeError, HIGH); // turn the LED ON (Pin 13)

  digitalWrite(STEPS_G6, LOW);  // turn off SM
  digitalWrite(STEPS_G7, LOW);  // turn off SM
  digitalWrite(STEPS_G8, LOW);  // turn off SM
  digitalWrite(STEPS_G9, LOW);  // not on ES & turn off SM

  digitalWrite(DIR_G6, LOW);    // turn off SM
  digitalWrite(DIR_G7, LOW);    // turn off SM
  digitalWrite(DIR_G8, LOW);    // turn off SM
  digitalWrite(DIR_G9, LOW);    // not on ES & turn off SM

  runner.init();
  runner.addTask(tC);
  runner.addTask(tB);
  runner.addTask(tGC);
  runner.addTask(tGS);
  runner.addTask(tER);
  runner.addTask(tGH);
  runner.addTask(tGM);

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
    tGS.enable();
    tGC.enable();
  }
}

// Task Gate start: ------------------------
void gateStart() {
  if (!gate6O && gate6C && !gate7O && gate7C && !gate8O && gate8C && !gate9O && gate9C && !gateHO && gateHC) {
    digitalWrite(SIGError, LOW);
    errCount = 0;
    Serial.println("GOK");  // gates ok, all in position
    tGS.disable();
    tER.enable();
  } else {
    errCount = 4;
    digitalWrite(SIGError, HIGH);
    if (!(!gate6O && gate6C)) Serial.println("ERR:G6C");
    if (!(!gate7O && gate7C)) Serial.println("ERR:G7C");
    if (!(!gate8O && gate8C)) Serial.println("ERR:G8C");
    if (!(!gate9O && gate9C)) Serial.println("ERR:G9C");
    if (!(!gateHO && gateHC)) Serial.println("ERR:GHC");
  }
}

// Task Gates popsition: -------------------
void gateChange() {
  if (digitalRead(EndPoG6O) != gate6O || digitalRead(EndPoG6C) != gate6C) {
    gate6O = digitalRead(EndPoG6O);
    gate6C = digitalRead(EndPoG6C);
    if (logIM6) {
      if (gate6O && !gate6C) {
        Serial.println("G6O");
        if (errCount > 0) --errCount;
        tGM.restartDelayed(50);
      } else if (!gate6O) {
        ++errCount;
      }
    } else {
      if (!gate6O && gate6C) {
        Serial.println("G6C");
        if (errCount > 0) --errCount;
        tGM.restartDelayed(50);
      }
    }
  }
  if (digitalRead(EndPoG7O) != gate7O || digitalRead(EndPoG7C) != gate7C) {
    gate7O = digitalRead(EndPoG7O);
    gate7C = digitalRead(EndPoG7C);
    if (logIM7) {
      if (gate7O && !gate7C) {
        Serial.println("G7O");
        if (errCount > 0) --errCount;
        tGM.restartDelayed(50);
      } else if (!gate7O) {
        ++errCount;
      }
    } else {
      if (!gate7O && gate7C) {
        Serial.println("G7C");
        if (errCount > 0) --errCount;
        tGM.restartDelayed(50);
      }
    }
  }
  if (digitalRead(EndPoG8O) != gate8O || digitalRead(EndPoG8C) != gate8C) {
    gate8O = digitalRead(EndPoG8O);
    gate8C = digitalRead(EndPoG8C);
    if (logIM8) {
      if (gate8O && !gate8C) {
        Serial.println("G8O");
        if (errCount > 0) --errCount;
        tGM.restartDelayed(50);
      } else if (!gate8O) {
        ++errCount;
      }
    } else {
      if (!gate8O && gate8C) {
        Serial.println("G8C");
        if (errCount > 0) --errCount;
        tGM.restartDelayed(50);
      }
    }
  }
  if (digitalRead(EndPoG9O) != gate9O || digitalRead(EndPoG9C) != gate9C) {
    gate9O = digitalRead(EndPoG9O);
    gate9C = digitalRead(EndPoG9C);
    if (logIM9) {
      if (gate9O && !gate9C) {
        Serial.println("G9O");
        if (errCount > 0) --errCount;
        tGM.restartDelayed(50);
      } else if (!gate9O) {
        ++errCount;
      }
    } else {
      if (!gate9O && gate9C) {
        Serial.println("G9C");
        if (errCount > 0) --errCount;
        tGM.restartDelayed(50);
      }
    }
  }
  if (digitalHand(EndPoGHO) != gateHO || digitalHand(EndPoGHC) != gateHC) {
    gateHO = digitalHand(EndPoGHO);
    gateHC = digitalHand(EndPoGHC);
    if (dustCount == 4) {
      ++errCount;
    } else if (gateHO && !gateHC) {
      Serial.println("GHO");
      if (errCount > 0) --errCount;
      tGH.restartDelayed(50);
    } else if (!gateHO && gateHC) {
      Serial.println("GHC");
      if (errCount > 0) --errCount;
      tGH.restartDelayed(50);
    }
  }
  if (dustWaitT > 0) --dustWaitT;
  if (errCount > 0) {
    digitalWrite(SIGError, HIGH);
  } else {
    digitalWrite(SIGError, LOW);
  }
}

// Task Gate HAnd: ---------------------
void gateHA() {
  if (dustWaitT > 0 && errCount == 0 && gateHO && !gateHC) {
    ++errCount;
  }
  if (dustWaitT == 0 && errCount == 0 && gateHO && !gateHC) {
    if (dustCount == 0) dustWaitT = 30 * 4; // *0,25 sec until next dust on
    digitalWrite(SSR_Vac, HIGH);
  } else if (dustCount == 0 && !gateHO && gateHC) {
    digitalWrite(SSR_Vac, LOW);
  }
//  Serial.println(IDENT + " HA:GO-GC-Count#gate#ERR:" + String(gateHO) + String(gateHC) + "#" + String(dustCount) + "#" + String(errCount));
}

// Task Gate MAchine: ---------------------
void gateMA() {
  // Serial.println("DustC: " + String(dustCount) + " Hand: " + String(gateHO) + String(gateHC));  // Test
  if (dustCount > 0 || (gateHO && !gateHC)) {
    digitalWrite(SSR_Vac, HIGH);
  } else {
    digitalWrite(SSR_Vac, LOW);
  }
}

// Task Gate log in and out: --------------------
void gateERR() {
  // Serial.println("lIM9:" + String(logIM9) + "lO9:" + "GO9:" + String(gate9O) + "GC9:" + String(gate9C));  // Test
  if (errCount > 0 && logIM6 && !gate6O) {
    Serial.println("ERR:G6O");
  } else if (errCount > 0 && !logIM6 && !gate6C) {
    Serial.println("ERR:G6C");
  }

  if (errCount > 0 && logIM7 && !gate7O) {
    Serial.println("ERR:G7O");
  } else if (errCount > 0 && !logIM7 && !gate7C) {
    Serial.println("ERR:G7C");
  }

  if (errCount > 0 && logIM8 && !gate8O) {
    Serial.println("ERR:G8O");
  } else if (errCount > 0 && !logIM8 && !gate8C) {
    Serial.println("ERR:G8C");
  }

  if (errCount > 0 && logIM9 && !gate9O) {
    Serial.println("ERR:G9O");
  } else if (errCount > 0 && !logIM9 && !gate9C) {
    Serial.println("ERR:G9C");
  }
}
// END OF TASKS ---------------------------------

// FUNCTIONS ------------------------------------
// Convert analog signal in digital boolean values
boolean digitalHand(int inputPin) {
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

  // Blast gate controll
  if (inStr.startsWith("LI")) {
    gateNR = inStr.substring(2, 3).toInt();
    switch (gateNR) {
      case 6:
        if (!logIM6) {
          logIM6 = HIGH;
          if (!gate6O) ++errCount;
          if (gate6O && !gate6C) Serial.println("G6O");
        }
        break;
      case 7:
        if (!logIM7) {
          logIM7 = HIGH;
          if (!gate7O) ++errCount;
          if (gate7O && !gate7C) Serial.println("G7O");
        }
        break;
      case 8:
        if (!logIM8) {
          logIM8 = HIGH;
          if (!gate8O) ++errCount;
          if (gate8O && !gate8C) Serial.println("G8O");
        }
        break;
      case 9:
        if (!logIM9) {
          logIM9 = HIGH;
          if (!gate9O) ++errCount;
          if (gate9O && !gate9C) Serial.println("G9O");
        }
        break;
    }
    tGM.restartDelayed(75);
    // Serial.println("LI:" + String(gateNR) +  + "Li:" + String(logIM9));  // Test
  }

  if (inStr.startsWith("LO")) {
    gateNR = inStr.substring(2, 3).toInt();
    switch (gateNR) {
      case 6:
        if (logIM6) {
          logIM6 = LOW;
          if (!gate6C) ++errCount;
        }
        break;
      case 7:
        if (logIM7) {
          logIM7 = LOW;
          if (!gate7C) ++errCount;
        }
        break;
      case 8:
        if (logIM8) {
          logIM8 = LOW;
          if (!gate8C) ++errCount;
        }
        break;
      case 9:
        if (logIM9) {
          logIM9 = LOW;
          if (!gate9C) ++errCount;
        }
        break;
    }
    tGM.restartDelayed(75);
  }

  if (inStr.startsWith("DO")) {
    gateNR = inStr.substring(2, 3).toInt();
    switch (gateNR) {
      case 6:
        if (gate6O && !gate6C && logIM6) {
          if (!dustC6) ++dustCount;
          dustC6 = HIGH;
        }
        break;
      case 7:
        if (gate7O && !gate7C && logIM7) {
          if (!dustC7) ++dustCount;
          dustC7 = HIGH;
        }
        break;
      case 8:
        if (gate8O && !gate8C && logIM8) {
          if (!dustC8) ++dustCount;
          dustC8 = HIGH;
        }
        break;
      case 9:
        if (gate9O && !gate9C && logIM9) {
          if (!dustC9) ++dustCount;
          dustC9 = HIGH;
        }
        break;
    }
    tGM.restartDelayed(75);
  }

  if (inStr.startsWith("DF")) {
    gateNR = inStr.substring(2, 3).toInt();
    switch (gateNR) {
      case 6:
        if (gate6O && !gate6C && logIM6) {
          dustC6 = LOW;
          if (dustCount > 0) --dustCount;
        }
        break;
      case 7:
        if (gate7O && !gate7C && logIM7) {
          if (dustCount > 0) --dustCount;
          dustC7 = LOW;
        }
        break;
      case 8:
        if (gate8O && !gate8C && logIM8) {
          if (dustCount > 0) --dustCount;
          dustC8 = LOW;
        }
        break;
      case 9:
        if (gate9O && !gate9C && logIM9) {
          if (dustCount > 0) --dustCount;
          dustC9 = LOW;
        }
        break;
    }
    tGM.restartDelayed(75);
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
