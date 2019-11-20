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
  'err'   - Error message is following ('ERR:G7O')
  'gok'   - all blast gates are ok, in write position

  Commands from Raspi <---
  'time'  - format time33.33.33 33:33:33
  'setrt'  - set RepeaT messages

  ---------- (_=6,7,8,9)
  'li_'   - log in for machine
  'lo_'   - log out for machine
  'do_'   - Dust collector On for machine
  'df_'   - Dust collector oFf for machine
  'ng_'   - No Gate is available

  last change: 20.11.2019 by Michael Muehl
  changed: changed comunucation blastgate with MA6-9: repeat messages
	*/
#define Version "1.1" // (Test =1.x ==> 1.1)

//#define _TASK_MICRO_RES // only for SM
// ---------------------
#include <Arduino.h>
#include <TaskScheduler.h>

// PIN Assignments
// Steps for the stepper
#define STEPS_G6  9	// stepper steps Gate 6
#define STEPS_G7 10 // stepper steps Gate 7
#define STEPS_G8 11	// stepper steps Gate 8
#define STEPS_G9 A6	// stepper steps Gate 9 [not for SM]
// Direction of movement
#define DIR_G6    7	// dir stepper Gate 6
#define DIR_G7   13	// dir stepper Gate 7
#define DIR_G8   12	// dir stepper Gate 8
#define DIR_G9   A7	// dir stepper Gate 9 [not for SM]
// enable all steppers
#define enaStGate 6	// enable stepper Gates

// this pin should connect NOT to Ground when want to stop the motor
#define EndPoG6C  2 // Endpostion Gate 6 close
#define EndPoG6O  5 // Endpostion Gate 6 open (only with endswitch open)
#define EndPoG7C  4 // Endpostion Gate 7 close
#define EndPoG7O A5 // Endpostion Gate 7 open (only with endswitch open)
#define EndPoG8C  3 // Endpostion Gate 8 close
#define EndPoG8O A4 // Endpostion Gate 8 open (only with endswitch open)
#define EndPoG9C A0 // Endpostion Gate 9 close
#define EndPoG9O A1 // Endpostion Gate 9 open (only with endswitch open)
#define EndPoGHC A6 // Endpostion Gate by hand close (Only analog input)
#define EndPoGHO A7 // Endpostion Gate by hand open (Only analog input)

#define REL_Vac  A2 // Relais Dust on / off  (Dust Collector)
#define SIGError A3 // SIGError for error
#define BUSError  8 // Bus error
#define xbeError  8 // Bus error (13)

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
#define repMES          1 // repeat commands

Scheduler runner;

// Callback methods prototypes
void checkXbee();        // Task connect to xBee Server
void repeatMES();        // Task to repeat messages
void retryPOR();         // Task connect to xBee Server

void gateStart();        // Task start with closed all
void gateChange();       // Task check changes
void gateMA();           // Task control maschines
void gateHA();           // Task control hand gate

// TASKS
Task tC(TASK_SECOND / 2, TASK_FOREVER, &checkXbee);     // 500ms
Task tB(TASK_SECOND * 5, TASK_FOREVER, &retryPOR);      // 100ms, task for debounce
Task tR(TASK_SECOND / 2, 0, &repeatMES);                // 500ms * repMES repeat messages

// --- Blast Gates ----------
Task tGS(TASK_SECOND, TASK_FOREVER, &gateStart);        // task for Gate start
Task tGC(TASK_SECOND / 4, TASK_FOREVER, &gateChange);   // task for Gate Change position

Task tGM(1, TASK_ONCE, &gateMA);                        // task for Gate machines
Task tGH(1, TASK_ONCE, &gateHA);                        // task for Gate hand

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

boolean gateHO = LOW; // bit gate By hand open = HIGH
boolean gateHC = LOW; // bit gate By hand close = HIGH

byte stepSP = 0;      // steps for start position

byte errCount = 0;      // how many errors are active
int gateNR = 0;         // "0" = no gates, 6,7,8,9 with gate
int dustWaitT = 0;      // counter how long dust wait time

// Serial with xBee
String inStr = "";      // a string to hold incoming data
String IDENT = "";      // Machine identifier for remote access control
String SFMes = "";      // String send for repeatMES
byte plplpl = 0;        // send +++ control AT sequenz
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

  pinMode(REL_Vac, OUTPUT);

  pinMode(SIGError, OUTPUT);
  pinMode(BUSError, OUTPUT);
  pinMode(xbeError, OUTPUT);

  // Set default values
  digitalWrite(REL_Vac,  LOW);  // turn off REL_Vac
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
  runner.addTask(tR);
  runner.addTask(tGC);
  runner.addTask(tGS);
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
  else if (getTime == 255) {
    tR.setIterations(repMES);
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

// Task repeatMES: ------------------------
void repeatMES() {
  // --repeat messages from machines
  Serial.println(String(SFMes));
}

// Task Gate start: ------------------------
void gateStart() {
  if (!gate6O && gate6C && !gate7O && gate7C && !gate8O && gate8C && !gate9O && gate9C && !gateHO && gateHC) {
    digitalWrite(SIGError, LOW);
    errCount = 0;
    Serial.println("GOK");  // gates ok, all in position
    tGS.disable();
    tR.disable();
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
        SFMes = "G6O";
        if (errCount > 0) --errCount;
      } else if (!gate6O) {
        if (errCount == 0) ++errCount;
        SFMes = "ERR:G6O";
      }
    } else {
      if (!gate6O && gate6C) {
        SFMes = "G6C";
        if (errCount > 0) --errCount;
      } else if (!gate6C) {
        if (errCount == 0) ++errCount;
				SFMes = "ERR:G6C";
      }
    }
    Serial.println(SFMes);
    tR.restart();
    tGM.restartDelayed(50);
  }
  if (digitalRead(EndPoG7O) != gate7O || digitalRead(EndPoG7C) != gate7C) {
    gate7O = digitalRead(EndPoG7O);
    gate7C = digitalRead(EndPoG7C);
    if (logIM7) {
      if (gate7O && !gate7C) {
        SFMes = "G7O";
       if (errCount > 0) --errCount;
      } else if (!gate7O) {
        if (errCount == 0) ++errCount;
        SFMes = "ERR:G7O";
      }
    } else {
      if (!gate7O && gate7C) {
        SFMes = "G7C";
        if (errCount > 0) --errCount;
      } else if (!gate7C) {
        if (errCount == 0) ++errCount;
				SFMes = "ERR:G7C";
      }
    }
    Serial.println(SFMes);
    tR.restart();
    tGM.restartDelayed(50);
  }
  if (digitalRead(EndPoG8O) != gate8O || digitalRead(EndPoG8C) != gate8C) {
    gate8O = digitalRead(EndPoG8O);
    gate8C = digitalRead(EndPoG8C);
    if (logIM8) {
      if (gate8O && !gate8C) {
        SFMes = "G8O";
        if (errCount > 0) --errCount;
      } else if (!gate8O) {
        if (errCount == 0) ++errCount;
        SFMes = "ERR:G8O";
      }
    } else {
      if (!gate8O && gate8C) {
        SFMes = "G8C";
       if (errCount > 0) --errCount;
      } else if (!gate8C) {
        if (errCount == 0) ++errCount;
				SFMes = "ERR:G8C";
      }
    }
    Serial.println(SFMes);
    tR.restart();
    tGM.restartDelayed(50);
  }
  if (digitalRead(EndPoG9O) != gate9O || digitalRead(EndPoG9C) != gate9C) {
    gate9O = digitalRead(EndPoG9O);
    gate9C = digitalRead(EndPoG9C);
    if (logIM9) {
      if (gate9O && !gate9C) {
        SFMes = "G9O";
        if (errCount > 0) --errCount;
      } else if (!gate9O) {
        if (errCount == 0) ++errCount;
        SFMes = "ERR:G9O";
      }
    } else {
      if (!gate9O && gate9C) {
        SFMes = "G9C";
        if (errCount > 0) --errCount;
      } else if (!gate9C) {
        if (errCount == 0) ++errCount;
				SFMes = "ERR:G9C";
      }
    }
    Serial.println(SFMes);
    tR.restart();
    tGM.restartDelayed(50);
  }
  if (digitalHand(EndPoGHO) != gateHO || digitalHand(EndPoGHC) != gateHC) {
    gateHO = digitalHand(EndPoGHO);
    gateHC = digitalHand(EndPoGHC);
    if (gateHO && !gateHC) {
      Serial.println("GHO");
      tGH.restartDelayed(50);
    } else if (!gateHO && gateHC) {
      Serial.println("GHC");
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
  if (!dustC6 && !dustC7 && !dustC8 && !dustC9) {
    if (errCount == 0 && dustWaitT > 0 && gateHO && !gateHC) ++errCount;
    if (errCount == 0 && dustWaitT <= 0) {
      if (gateHO && !gateHC) {
        digitalWrite(REL_Vac, HIGH);
        dustWaitT = -1;
      } else {
        digitalWrite(REL_Vac, LOW);
        if (dustWaitT == -1) dustWaitT = 30 * 4; // *0,25 sec until next dust on
      }
    } else if (errCount > 0 && !gateHO && gateHC) --errCount;
  }
}

// Task Gate MAchine: ---------------------
void gateMA() {
  if (dustC6 || dustC7 || dustC8 || dustC9) {
    digitalWrite(REL_Vac, HIGH);
  } else {
    digitalWrite(REL_Vac, LOW);
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

  if (inStr.startsWith("SETRT")) { // set repeat messages
    tR.setIterations(inStr.substring(5).toInt());
  }

  // Blast gate controll
  if (inStr.startsWith("LI")) {
    gateNR = inStr.substring(2, 3).toInt();
    switch (gateNR) {
      case 6:
        if (!logIM6) {
          logIM6 = HIGH;
          if (!gate6O) {
            if (errCount == 0) ++errCount;
            SFMes = "ERR:G6O";
          }
        }
        break;
      case 7:
        if (!logIM7) {
          logIM7 = HIGH;
          if (!gate7O) {
            if (errCount == 0) ++errCount;
            SFMes = "ERR:G7O";
          }
        }
        break;
      case 8:
        if (!logIM8) {
          logIM8 = HIGH;
          if (!gate8O) {
            if (errCount == 0) ++errCount;
            SFMes = "ERR:G8O";
          }
        }
        break;
      case 9:
        if (!logIM9) {
          logIM9 = HIGH;
          if (!gate9O) {
            if (errCount == 0) ++errCount;
            SFMes = "ERR:G9O";
          }
        }
        break;
      }
    if (errCount > 0) {
      Serial.println(SFMes);
      tR.restart();
    }
  }

  if (inStr.startsWith("LO")) {
    gateNR = inStr.substring(2, 3).toInt();
    switch (gateNR) {
      case 6:
        if (logIM6) {
          logIM6 = LOW;
          if (!gate6C) {
            if (errCount == 0) ++errCount;
            SFMes = "ERR:G6C";
          }
        }
        break;
      case 7:
        if (logIM7) {
          logIM7 = LOW;
          if (!gate7C) {
            if (errCount == 0) ++errCount;
            SFMes = "ERR:G7C";
          }
        }
        break;
      case 8:
        if (logIM8) {
          logIM8 = LOW;
          if (!gate8C) {
            if (errCount == 0) ++errCount;
            SFMes = "ERR:G8C";
          }
        }
        break;
      case 9:
        if (logIM9) {
          logIM9 = LOW;
          if (!gate9C) {
            if (errCount == 0) ++errCount;
            SFMes = "ERR:G9C";
          }
        }
        break;
    }
    if (errCount > 0) {
      Serial.println(SFMes);
      tR.restart();
    }
  }

  if (inStr.startsWith("DO")) {
    gateNR = inStr.substring(2, 3).toInt();
    switch (gateNR) {
      case 6:
        if (gate6O && !gate6C && logIM6 && !dustC6) dustC6 = HIGH;
        break;
      case 7:
        if (gate7O && !gate7C && logIM7 && !dustC7) dustC7 = HIGH;
        break;
      case 8:
        if (gate8O && !gate8C && logIM8 && !dustC8) dustC8 = HIGH;
        break;
      case 9:
        if (gate9O && !gate9C && logIM9 && !dustC9) dustC9 = HIGH;
        break;
    }
    tGM.restartDelayed(75);
  }

  if (inStr.startsWith("DF")) {
    gateNR = inStr.substring(2, 3).toInt();
    switch (gateNR) {
      case 6:
        if (gate6O && !gate6C && logIM6 && dustC6) dustC6 = LOW;
        break;
      case 7:
        if (gate7O && !gate7C && logIM7 && dustC7) dustC7 = LOW;
        break;
      case 8:
        if (gate8O && !gate8C && logIM8 && dustC8) dustC8 = LOW;
        break;
      case 9:
        if (gate9O && !gate9C && logIM9 && dustC9) dustC9 = LOW;
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
