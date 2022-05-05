#include <PS4BT.h>
#include <usbhub.h>
#include <math.h>
#include <SPI.h>
#include <Stepper.h>
#include <Wire.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

USB Usb;
BTD Btd(&Usb);

PS4BT PS4(&Btd, PAIR);  // Pairing
//PS4BT PS4(&Btd);        // Already Paired

// Driver Pins
#define driverPin1 5    // Left Driver Pin
#define driverPin2 6    // Right Driver Pin


// State functions variables
int state = 0;          // State Machine
int Motion = 0;         // Motion Direction
bool Change = true;     // Check if an interrupt is detected
bool runningRandomMotion = true;
bool doingAltTilt = true;

// Analog Position Variables
int LeftPos = 127;      // Analog Write Vals
int RightPos = 127;     // Analog Write Vals

// Timing & Angle/Speed Variables
int timing = 4;         // Timing [Delay] within the Pinout Function (Ideal is 3 & 4)
int delayer = 0;
int speedLevel = 5;
int timingIncrement = 5;  //how much we change "timing" variable when increasing/decreasing speed
int tiltLevel = 3; // corresponds to what level of intensity the max tilt angle OF PLATFORM is. Ranges from 1 to 3, 3 being the most intense
int reRockProb = 70;  //probability in % that robot will rebound and repeat rocking motion in randomMovement function

// Tilting Parameters
int maxBackTiltAngle = 255 - 64; // Max Angle when tilting backwards. Is to be fixed no matter the tilt amplitude bc robot can only tilt 10 deg backwards
int minForwardTiltAngle = 0;
int maxSideTiltAngle = 255;
int minSideTiltAngle = 0;

// Gamebar Buttons
int buttonMode = 0;
int movementCompleted = 1;


//testing
int speedd = timingIncrement;

void setup() {
  Serial.begin(115200);
  Serial.println("Startup Done");
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    while (1); // Halt
  }

  // Pin Definition
  pinMode(driverPin1, OUTPUT);
  pinMode(driverPin2, OUTPUT);
  pinMode(BouncePin,OUTPUT);
  analogWrite(driverPin1, LeftPos);
  analogWrite(driverPin2, RightPos);

  // Random Seed for randomizer
  randomSeed(millis());
  Serial.println("Startup Done");
}

/*
  States (COLORS?)
  0 - Joystick        (RED)     L Joystick
  1 - Circular Loop   (BLUE)    O Button
  2 - Left/Right Loop (GREEN)   X Button
  3 - Front/Back Loop (YELLOW)  □ Button
  4 - Bouncing Loop   (PURPLE)  △ Button
  5 - Homing Reset    ()        Touchpad/L3
  6 - Random Mode     (ORANGE)  R1/L1
  7 - Patient Mode    ()        R2/L2
*/

void loop() {

  delay(50); // slow down by 50 ms to be able to read serial monitor
  Serial.print("\r\nState: ");
  Serial.print(state);
  switch (state) {
    case 0:   // Joystick
      JoystickMovement();
      break;
    case 1:   // Cirlce
      CircularMovement();
      break;
    case 2:   // Cross
      LRMovement();
      break;
    case 3:   // Square
      FBMovement();
      break;
    case 4:   // Triangle
      BounceMovement();
      break;
    case 5:   // Touchpad / L3
      CheckForChange();
      break;
    case 6:   // L1 or R1
      RandomMovement();
      break;
    case 7:   // L2 or R2
      PatientMode();
      break;
    default:
      break;
  }
}


void TiltChange(int tiltVar) {
  tiltLevel += tiltVar;
  if (tiltLevel > 3) {
    tiltLevel = 3;
  }
  if (tiltLevel < 1) {
    tiltLevel = 1;
  }
  if (tiltLevel == 3) {
    minForwardTiltAngle = 0;
    maxSideTiltAngle = 255;
    minSideTiltAngle = 0;
  } else if (tiltLevel == 2) {
    minForwardTiltAngle = 30;
    maxSideTiltAngle = 225;
    minSideTiltAngle = 30;
  } else if (tiltLevel == 1) {
    minForwardTiltAngle = 60;
    maxSideTiltAngle = 200;
    minSideTiltAngle = 60;
  }
}

void OverflowCheck(char Direct) {
  switch (Direct) {
    case 'L':
      if (LeftPos < minSideTiltAngle) {
        LeftPos = minSideTiltAngle;
      }
      if (RightPos > maxSideTiltAngle) {
        RightPos = maxSideTiltAngle;
      }
      break;
    case 'R':
      if (LeftPos > maxSideTiltAngle) {
        LeftPos = maxSideTiltAngle;
      }
      if (RightPos < minSideTiltAngle) {
        RightPos = minSideTiltAngle;
      }
      break;
    case 'F':
      if (LeftPos < minForwardTiltAngle) {
        LeftPos = minForwardTiltAngle;
      }
      if (RightPos < minForwardTiltAngle) {
        RightPos = minForwardTiltAngle;
      }
      break;
    case 'B':
      if (LeftPos > maxBackTiltAngle) {
        LeftPos = maxBackTiltAngle;
      }
      if (RightPos > maxBackTiltAngle) {
        RightPos = maxBackTiltAngle;
      }
      break;
  }
}

void CheckForChange() {
  //Serial.println("Checking for Change");
  Usb.Task();
  if (PS4.connected()) {
    if (((abs(PS4.getAnalogHat(LeftHatX) - 127) > 15) && (abs(PS4.getAnalogHat(LeftHatY) - 127) > 15))) {
      if (state != 0) {
        state = 0;
        Reset();
      }
    } else if ((PS4.getButtonClick(CIRCLE))) {
      if (state != 1) {
        state = 1;
        Reset();
      }
    } else if ((PS4.getButtonClick(CROSS))) {
      if (state != 2) {
        state = 2;
        Reset();
      }
    } else if ((PS4.getButtonClick(SQUARE))) {
      if (state != 3) {
        state = 3;
        Reset();
      }
    } else if ((PS4.getButtonClick(TRIANGLE))) {
      if (state != 4) {
        state = 4;
        Reset();
      }
    } else if ((PS4.getButtonClick(TOUCHPAD))) {
      if (state != 5) {
        state = 5;
        Reset();
      }
    } else if (PS4.getButtonClick(L3)) {
      if (state != 5) {
        state = 5;
        Reset();
      }
    } else if ((PS4.getButtonClick(L1)) || (PS4.getButtonClick(R1))) {
      if (state != 6) {
        state = 6;
        Reset();
      }
    } else if ((PS4.getButtonClick(L2)) || (PS4.getButtonClick(R2))) {
      if (state != 7) {
        state = 7;
        Reset();
      }
    } else if (PS4.getButtonClick(UP)) {
      Reset();
      TiltChange(1);
    } else if (PS4.getButtonClick(DOWN)) {
      Reset();
      TiltChange(-1);
    } else if (PS4.getButtonClick(RIGHT)) {
      Serial.println("Speed increased");
    } else if (PS4.getButtonClick(LEFT)) {
      Serial.println("Speed decreased");
    }
  }
}

void DriverPinOut() {
  if (state != 0) {
    for (int i = 0; i <= delayer ; i++) {
      CheckForChange();
      if (i == delayer) {
        analogWrite(driverPin1, LeftPos);
        analogWrite(driverPin2, RightPos);
      }
      delay(timing);
    }
  } else {
    CheckForChange();
    analogWrite(driverPin1, LeftPos);
    analogWrite(driverPin2, RightPos);
    delay(timing);
  }
}

void Reset() {
  while ((LeftPos != 127) || (RightPos != 127)) {
    if (LeftPos < 127) {
      LeftPos++;
      if (LeftPos > 127) {
        LeftPos = 127;
      }
    } else if (LeftPos > 127) {
      LeftPos--;
      if (LeftPos < 127) {
        LeftPos = 127;
      }
    }
    if (RightPos < 127) {
      RightPos++;
      if (RightPos > 127) {
        RightPos = 127;
      }
    } else if (RightPos > 127) {
      RightPos--;
      if (RightPos < 127) {
        RightPos = 127;
      }
    }
    DriverPinOut();
  }
  Change = false;
}

void JoystickMovement() {
  while (Change) {
    if (PS4.getAnalogHat(LeftHatX) > 137) {
      if ((LeftPos + RightPos) < (185 * 2)) {
        LeftPos++;
      }
      RightPos--;
      //RightOverflowCheck();
      OverflowCheck('R');
    }
    if (PS4.getAnalogHat(LeftHatX) < 117) {
      if ((LeftPos + RightPos) < (185 * 2)) {
        RightPos++;
      }
      LeftPos--;
      //LeftOverflowCheck();
      OverflowCheck('L');
    }
    if (PS4.getAnalogHat(LeftHatY) > 137) {
      if ((LeftPos + RightPos) < (185 * 2)) {
        LeftPos++;
        RightPos++;
      }
      //BackOverflowCheck();
      OverflowCheck('B');
    }
    if (PS4.getAnalogHat(LeftHatY) < 117) {
      LeftPos--;
      RightPos--;
      //ForwardOverflowCheck();
      OverflowCheck('F');
    }
    CheckForChange();
    if (Change) {
      DriverPinOut();
    }
  }
}

void FBMovement() {
  Motion = 0;
  while (Change) {
    switch (Motion) {
      case 0:   // Nose Down
        LeftPos--;
        RightPos--;
        //ForwardOverflowCheck();
        OverflowCheck('F');
        if ((LeftPos == minForwardTiltAngle) && (RightPos == minForwardTiltAngle)) {
          Motion = 1;
        }
        break;
      case 1:   // Nose Up
        LeftPos++;
        RightPos++;
        //BackOverflowCheck();
        OverflowCheck('B');
        if ((LeftPos == maxBackTiltAngle) && (RightPos == maxBackTiltAngle)) {
          Motion = 0;
        }
        break;
    }
    CheckForChange();
    if (Change) {
      DriverPinOut();
    }
  }
  Change = true;
}

// implement making this run only for a set amount of time
void FBMovementTimed() {
  Motion = 0;
  while (Change) {
    switch (Motion) {
      case 0:   // Nose Down
        LeftPos--;
        RightPos--;
        //ForwardOverflowCheck();
        OverflowCheck('F');
        if ((LeftPos == minForwardTiltAngle) && (RightPos == minForwardTiltAngle)) {
          Motion = 1;
        }
        break;
      case 1:   // Nose Up
        LeftPos++;
        RightPos++;
        //BackOverflowCheck();
        OverflowCheck('B');
        if ((LeftPos == maxBackTiltAngle) && (RightPos == maxBackTiltAngle)) {
          Motion = 0;
        }
        break;
    }
    CheckForChange();
    if (Change) {
      DriverPinOut();
    }
  }
  Change = true;
}

void LRMovement() {
  Motion = 2;
  while (Change) {
    switch (Motion) {
      case 2:   // Left Orientated
        LeftPos++;
        RightPos--;
        //RightOverflowCheck();
        OverflowCheck('R');
        if ((LeftPos == maxSideTiltAngle) && (RightPos == minSideTiltAngle)) {
          Motion = 3;
        }
        break;
      case 3:   // Right Orientated
        LeftPos--;
        RightPos++;
        //LeftOverflowCheck();
        OverflowCheck('L');
        if ((LeftPos == minSideTiltAngle) && (RightPos == maxSideTiltAngle)) {
          Motion = 2;
        }
        break;
    }
    CheckForChange();
    if (Change) {
      DriverPinOut();
    }
  }
  Change = true;
}

// implement making this run only for a set amount of time
void LRMovementTimed() {
  Motion = 2;
  while (Change) {
    switch (Motion) {
      case 2:   // Left Orientated
        LeftPos++;
        RightPos--;
        //RightOverflowCheck();
        OverflowCheck('R');
        if ((LeftPos == maxSideTiltAngle) && (RightPos == minSideTiltAngle)) {
          Motion = 3;
        }
        break;
      case 3:   // Right Orientated
        LeftPos--;
        RightPos++;
        //LeftOverflowCheck();
        OverflowCheck('L');
        if ((LeftPos == minSideTiltAngle) && (RightPos == maxSideTiltAngle)) {
          Motion = 2;
        }
        break;
    }
    CheckForChange();
    if (Change) {
      DriverPinOut();
    }
  }
  Change = true;
}

void CircularMovement() {
  int counter = 0;        // For smooth transition of back motion
  Motion = 0;
  while (Change) {
    switch (Motion) {
      case 0:   // Nose Down
        LeftPos--;
        RightPos--;
        //ForwardOverflowCheck();
        OverflowCheck('F');
        if ((LeftPos == minForwardTiltAngle) && (RightPos == minForwardTiltAngle)) {
          Motion = 2;
        }
        break;
      case 1:   // Nose Up
        counter++;
        if (counter == 3) {
          LeftPos--;
          counter = 0;
        }
        RightPos++;
        if (LeftPos < maxBackTiltAngle) {
          LeftPos = maxBackTiltAngle;
        }
        if (RightPos > maxBackTiltAngle) {
          RightPos = maxBackTiltAngle;
        }
        if ((LeftPos == maxBackTiltAngle) && (RightPos == maxBackTiltAngle)) {
          Motion = 3;
          counter = 0;
        }
        break;
      case 2:   // Right Orientated
        LeftPos++;
        RightPos--;
        //RightOverflowCheck();
        OverflowCheck('R');
        if ((LeftPos == maxSideTiltAngle) && (RightPos == minSideTiltAngle)) {
          Motion = 1;
        }
        break;
      case 3:   // Left Orientated
        counter++;
        if (counter == 3) {
          RightPos++;
          counter = 0;
        }
        LeftPos--;
        //LeftOverflowCheck();
        OverflowCheck('L');
        if ((LeftPos == minSideTiltAngle) && (RightPos == maxSideTiltAngle)) {
          Motion = 0;
          counter = 0;
        }
        break;
    }
    CheckForChange();
    if (Change) {
      DriverPinOut();
    }
  }
  Change = true;
}

void PatientMode() {
  if (Serial.available()) {
    buttonMode = Serial.read(); // read one byte from serial buffer and save to buttonMode
  }

  while (Change) {
    // Detect Patient Button Press similiar to Joystick
    if (buttonMode == 0 && movementCompleted == 1) {
      //Run LR Movement
      movementCompleted = 0;
      LRMovementTimed();
      movementCompleted = 1;
    }

    if (buttonMode == 1 && movementCompleted == 1) {
      // Run FB Movement
      movementCompleted = 0;
      FBMovementTimed();
      movementCompleted = 1;
    }

    if (buttonMode == 2 && movementCompleted == 1) {
      // Run bounce code
      movementCompleted = 0;
      BounceMovement();
      movementCompleted = 1;
    }

    // right now, will run to infinity, not timed motion
    //    while (movementCompleted) {
    //      if (digitalRead(LRButtonPin) == HIGH) {
    //        Movement()
    //        if (reach to the end)
    //          movementCompleted = 0;
    //      }
    //    }

    CheckForChange();
    if (Change) {
      DriverPinOut();
    }
  }
  Change = true;
}


void BounceMovement() {
  //  while (Change) {
  //    CheckForChange();
  //    if (Change) {
  //      analogWrite(BouncePin, 90);     // Bounce ON
  //    }
  //  }
  //  analogWrite(BouncePin, 0);          // Bounce OFF
  //  Change = true;
}

void RandomMovement() {
//  randomSeed(millis());  // can move this line to setup
//  while (Change) {
//    if (random(2) == -1) { // has 50/50 chance of occuring. If satisfied, will do rocking motion
//
//    }
//    else { // will do a single tilt in random direction
//      Motion = random(4);  // motion = 0, 1, 2, or 3. # dictates which direction base will tilt
//      runningRandomMotion = true;
//      while (runningRandomMotion) {
//        switch (Motion) {
//          case 0:  //Tilt left
//            LeftPos -= speedd;
//            RightPos += speedd;
//            OverflowCheck();
//            if ((LeftPos == 0) && (RightPos == 255)) {
//              // get out of this case and restart RandomMovement
//              runningRandomMotion = false;
//            }
//            break;
//          case 1:  //Tilt right
//            LeftPos += speedd;
//            RightPos -= speedd;
//            OverflowCheck();
//            if ((LeftPos == 255) && (RightPos == 0)) {
//              // get out of this case and restart RandomMovement
//              runningRandomMotion = false;
//            }
//            break;
//          case 2:  //Tilt forward
//            LeftPos -= speedd;
//            RightPos -= speedd;
//             OverflowCheck();
//            if ((LeftPos == AngleMin) && (RightPos == AngleMin)) {
//              // get out of this case and restart RandomMovement
//              runningRandomMotion = false;
//            }
//            break;
//          case 3:  //Tilt back
//            LeftPos += speedd;
//            RightPos += speedd;
//            OverflowCheck();
//            if ((LeftPos == AngleMax) && (RightPos == AngleMax)) {
//              // get out of this case and restart RandomMovement
//              runningRandomMotion = false;
//            }
//            break;
//        }
//      }
//      CheckForChange();
//      if (Change) {
//        DriverPinOut();
//      }
//    }
//    Reset();
//  }
//  Change = true;
}
