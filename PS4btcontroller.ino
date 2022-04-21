#include <PS4BT.h>
#include <usbhub.h>
#include <math.h>
#include <SPI.h>
#include <Stepper.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

#define driverPin1 5
#define driverPin2 6

USB Usb;
BTD Btd(&Usb);

//PS4BT PS4(&Btd, PAIR);  // Pairing
PS4BT PS4(&Btd);        // Already Paired

/*
  States (COLORS?)

  0 - Joystick        (RED)     R Joystick
  1 - Circular Loop   (BLUE)    □ Button
  2 - Left/Right Loop (GREEN)   O Button
  3 - Front/Back Loop (YELLOW)  X Button
  4 - Bouncing Loop   (PURPLE)  △ Button
  5 - Patient Mode    (ORANGE)  Touchpad
*/

// Bouncing Variables
#define buttonPin 4
#define motorStepPin 11
#define motorDirPin 10

int btnState = 0;
int steps = 200;
int motorSpeed = 2000;
Stepper bbStepper(steps, 10, 11);

// Hall Effect Sensor
#define hallSensor 2

// Robotic
int state = 0;     // State Machine for mode method
int Motion = 0;    // State Machone for motion direction
bool Change = true; // Check if a state change occurs

int xPos = 127;     // Analog Write Vals
int yPos = 127;     // Analog Write Vals
int speedd = 1;     // Speed
int timing = 4;    // Timing [Delay] within the Action function (Not sure if we need it)
int SpeeddMin = 1;  // Speed Min
int SpeeddMax = 10; // Speed Max
int counter = 0;
int delayer = 0;
bool runningRandomMotion = true;
bool doingAltTilt = true;
int reRockProb = 70;  //probability in % that robot will rebound and repeat rocking motion in randomMovement function
int tiltLevel = 3 // corresponds to what level of intensity the max tilt angle OF PLATFORM is. Ranges from 1 to 3, 3 being the most intense
int maxBackTiltAngle = 255 - 64; // Max Angle when tilting backwards. Is to be fixed no matter the tilt amplitude bc robot can only tilt 10 deg backwards
int minForwardTiltAngle = 0;
int maxSideTiltAngle = 255;
int minSideTiltAngle = 0;
int speedLevel = 5;
int timingIncrement = 5;  //how much we change "timing" variable when increasing/decreasing speed

void increaseTilt() {
  tiltLevel += 1;
  if (tiltLevel == 3) {  //these PWM values r all good
    minForwardTiltAngle = 0;
    maxSideTiltAngle = 255;
    minSidetiltAngle = 0;
  }
  else {  //PWM values need to be calibrated
    minForwardTiltAngle = 30;
    maxSideTiltAngle = 225
    minSideTiltAngle = 30;
  }
}

void decreaseTilt() {
  tiltLevel -= 1;
  if (tiltLevel == 2) { //PWM values need to be calibrated
    minForwardTiltAngle = 30;
    maxSideTiltAngle = 225
                       minSideTiltAngle = 30;
  }
  else {  //PWM needs calibration
    minForwardTiltAngle = 60;
    maxSideTiltAngle = 200;
    minSideTiltAngle = 60;
  }
}

void LeftOverflowCheck() {
  if (xPos < minSideTiltAngle) {
    xPos = minSideTiltAngle;
  }
  if (yPos > maxSideTiltAngle) {
    yPos = maxSideTiltAngle;
  }
}

void RightOverflowCheck() {
  if (xPos > maxSideTiltAngle) {
    xPos = maxSideTiltAngle;
  }
  if (yPos < minSideTiltAngle) {
    yPos = minSideTiltAngle;
  }
}

void ForwardOverflowCheck() {
  if (xPos < minForwardTiltAngle) {
    xPos = minForwardTiltAngle;
  }
  if (yPos < minForwardTiltAngle) {
    yPos = minForwardTiltAngle;
  }
}

void BackOverflowCheck() {
  if (xPos > maxBackTiltAngle) {
    xPos = maxBackTiltAngle;
  }
  if (yPos > maxBackTiltAngle) {
    yPos = maxBackTiltAngle;
  }
}

void setup() {
  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    //Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }
  //Serial.print(F("\r\nPS4 USB Library Started"));

  pinMode(driverPin1, OUTPUT);
  pinMode(driverPin2, OUTPUT);
  analogWrite(driverPin1, xPos);
  analogWrite(driverPin2, yPos);
  bbStepper.setSpeed(motorSpeed);
  pinMode(buttonPin, INPUT);
  pinMode(motorStepPin, OUTPUT);
  pinMode(motorDirPin, OUTPUT);
  pinMode(hallSensor, INPUT);
  bbStepper.step(0);
  randomSeed(millis());  // can move this line to setup
}

void loop() {
  Usb.Task();
  ////Serial.print("\r\nLooping");
  switch (state) {
    case 0:  //joystick
      JoystickMovement();
      break;
    case 1:  //cirlce
      CircularMovement();
      break;
    case 2: //cross
      LRMovement();
      break;
    case 3:  //square
      FBMovement();
      break;
    case 4:  //triangle
      BounceMovement();
//      RandomMovement();
      break;
    case 5: //touchpad
      PatientMode();
      break;
    default:
      break;
    case 6: //L1 or R1
      RandomMovement();
  }
}

void CheckForChange() {
  //Serial.print("State: ");
  //Serial.println(state);
  Usb.Task();
  if (PS4.connected()) {
    if (((abs(PS4.getAnalogHat(LeftHatX) - 127) > 15) && (abs(PS4.getAnalogHat(LeftHatY) - 127) > 15)) && (state != 0)) {
      state = 0;
      //Serial.println("Joystick Mode Activated");
      //PS4.setLed(red, green, blue);
      Reset();
    }
    if ((PS4.getButtonClick(CIRLCE)) && (state != 1)) {
      state = 1;
      //PS4.setLed(red, green, blue);
      //Serial.println("Circular Mode Activated");
      Reset();
    } else if ((PS4.getButtonClick(CROSS))  && (state != 2)) {
      state = 2;
      //PS4.setLed(red, green, blue);
      //Serial.println("Left/Right Mode Activated");
      Reset();
    } else if ((PS4.getButtonClick(SQUARE))  && (state != 3)) {
      state = 3;
      //PS4.setLed(red, green, blue);
      //Serial.println("Front/Back Mode Activated");
      Reset();
    } else if ((PS4.getButtonClick(TRIANGLE))  && (state != 4)) {
      state = 4;
      //PS4.setLed(red, green, blue);
      //Serial.println("Bounce Mode Activated");
      Reset();
    } else if ((PS4.getButtonClick(TOUCHPAD))  && (state != 5)) {
      state = 5;
      //PS4.setLed(red, green, blue);
      //Serial.println("Patient Mode Activated");
      Reset();
    } else if ((PS4.getButtonClick(L1)) || (PS$.getButtonClick(R1))){
      state = 6;
    } else if ((PS4.getButtonClick(UP))  && (tiltLevel != 3)) {
      Reset();
      increaseTilt();
      state = 0;
      Serial.println("Max tilt angle increased");
    } else if ((PS4.getButtonClick(DOWN) && (tiltLevel != 1)) {
      Reset();
      decreaseTilt();
      state = 0;
      Serial.println("Max tilt angle decreased");
    } else if((PS5.getButtonClick(RIGHT) && (speedLevel != 5)) {
      Reset();
      timing += timingIncrement;
      Serial.println("Speed increased");
    } else if((PS5.getButtonClick(Left) && (speedLevel != 1)) {
      Reset();
      timing += timingIncrement;
      Serial.println("Speed increased");
    }
  }
}

void DriverPinOut() {
  if (state != 0) {

    for (int i = 0; i <= delayer ; i++) {
      if (i == delayer) {
        analogWrite(driverPin1, xPos);
        analogWrite(driverPin2, yPos);
        delay(timing);
      } else {
        delay(timing);
      }
    }

    //    could replace above chunk of code (FOR loop) with this maybe?
    //    delay(timing or something idk);
    //    analogWrite(driverPin1, xPos);
    //    analogWrite(driverPin2, yPos);

  } else {
    analogWrite(driverPin1, xPos);
    analogWrite(driverPin2, yPos);
    delay(timing);
  }
}

void Reset() {
  while ((xPos != 127) || (yPos != 127)) {
    if (xPos < 127) {
      xPos += speedd;
      if (xPos > 127) {
        xPos = 127;
      }
    } else if (xPos > 127) {
      xPos -= speedd;
      if (xPos < 127) {
        xPos = 127;
      }
    }
    if (yPos < 127) {
      yPos += speedd;
      if (yPos > 127) {
        yPos = 127;
      }
    } else if (yPos > 127) {
      yPos -= speedd;
      if (yPos < 127) {
        yPos = 127;
      }
    }
    DriverPinOut();
  }
  Serial.print("Final Xpos:");
  Serial.println(xPos);
  Serial.print("Final Ypos:");
  Serial.println(yPos);
  Change = false;
}

void JoystickMovement() {
  while (Change) {
    if (PS4.getAnalogHat(LeftHatX) > 137) {
      if ((xPos + yPos) < (185 * 2)) {
        xPos += speedd;
      }
      yPos -= speedd;
      RightOverflowCheck();
    }
    if (PS4.getAnalogHat(LeftHatX) < 117) {
      if ((xPos + yPos) < (185 * 2)) {
        yPos += speedd;
      }
      xPos -= speedd;
      LeftOverflowCheck();
    }
    if (PS4.getAnalogHat(LeftHatY) > 137) {
      if ((xPos + yPos) < (185 * 2)) {
        xPos += speedd;
        yPos += speedd;
      }
      ForwardOverflowCheck();
    }
    if (PS4.getAnalogHat(LeftHatY) < 117) {
      xPos -= speedd;
      yPos -= speedd;
    }
    if (PS4.getButtonClick(L3)) { // if L3 is pressed, home the babybot https://blog.artwolf.in/a?ID=ac8ea0a0-d501-4a85-89d7-ad8d3a7d6a62
      Reset();
      Change = false;
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
        xPos -= speedd;
        yPos -= speedd;
        ForwardOverflowCheck();
        if ((xPos == minForwardTiltAngle) && (yPos == minForwardTiltAngle)) {
          Motion = 1;
          delay(2000);
        }
        break;
      case 1:   // Nose Up
        xPos += speedd;
        yPos += speedd;
        BackOverflowCheck();
        if ((xPos == maxBackTiltAngle) && (yPos == maxBackTiltAngle)) {
          Motion = 0;
          delay(2000);
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
        xPos += speedd;
        yPos -= speedd;
        LeftOverflowCheck();
        if ((xPos == maxSideTiltAngle) && (yPos == minSideTiltAngle)) {
          Motion = 3;
          delay(2000);
        }
        break;
      case 3:   // Right Orientated
        xPos -= speedd;
        yPos += speedd;
        RightOverflowCheck();
        if ((xPos == minSideTiltAngle) && (yPos == maxSideTiltAngle)) {
          Motion = 2;
          delay(2000);
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
  Motion = 0;
  while (Change) {
    switch (Motion) {
      case 0:   // Nose Down
        xPos -= speedd;
        yPos -= speedd;
        ForwardOverflowCheck();
        if ((xPos == minForwardTiltAngle) && (yPos == minForwardTiltAngle)) {
          Motion = 2;
          delay(2000);
        }
        break;
      case 1:   // Nose Up
        counter++;
        if (counter == 3) {
          xPos -= speedd;
          counter = 0;
        }
        yPos += speedd;
        BackOverflowCheck();
        if (xPos < maxBackTiltAngle) {
          xPos = maxBackTiltAngle;
        }
        if (yPos > maxBackTiltAngle) {
          yPos = maxBackTiltAngle;
        }
        if ((xPos == maxBackTiltAngle) && (yPos == maxBackTiltAngle)) {
          Motion = 3;
          counter = 0;
          delay(2000);
        }
        break;
      case 2:   // Right Orientated
        xPos += speedd;
        yPos -= speedd;
        RightOverflowCheck();
        if ((xPos == maxSideTiltAngle) && (yPos == minSideTiltAngle)) {
          Motion = 1;
          delay(2000);
        }
        break;
      case 3:   // Left Orientated
        counter++;
        if (counter == 3) {
          yPos += speedd;
          counter = 0;
        }
        xPos -= speedd;
        LeftOverflowCheck();
        if ((xPos == minSideTiltAngle) && (yPos == maxSideTiltAngle)) {
          Motion = 0;
          counter = 0;
          delay(2000);
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
  while (Change) {
    // Detect Patient Button Press similiar to Joystick
    CheckForChange();
    if (Change) {
      DriverPinOut();
    }
  }
  Change = true;
}

void BounceMovement() {
  while (Change) {
    Serial.println("this is suppose to be bouncing");
    CheckForChange();
    if (Change) {
      bbStepper.step(steps);
    }
  }
  bbStepper.step(0);
  Change = true;
}

void RandomMovement() {
  Motion = 0;
  while (Change) {
    runningRandomMotion = true;
    if (random(2) == 1) { // has 50/50 chance of occuring. If satisfied, will do rocking motion
      Motion = random(2); // Motion = 0 or 1. # dictates which direction rocking motion will start with.
      if (Motion == 0) {  //rock the robot left and then right
        while (runningRandomMotion) {  //WHILE loop may rerun several times depending on random-ness for consecutive rocking motions

          //tilting left code
          xPos -= speedd;
          yPos += speedd;
          LeftOverflowCheck();
          CheckForChange();
          if (Change) {
            DriverPinOut();
          }
          else {
            runningRandomMotion = false;
          }
          if ((xPos == minSideTiltAngle) && (yPos == maxSideTiltAngle)) {
            // get robot back to neutral position and begin tilting to other side
            Reset();
            Change = true;
            doingAltTilt = true;
          }

          //tilting right code
          while (doingAltTilt) {
            xPos += speedd;
            yPos -= speedd;
            RightOverflowCheck();
            CheckForChange();
            if (Change) {
              DriverPinOut();
            }
            else {
              runningRandomMotion = false;
            }
            if ((xPos == maxSideTiltAngle) && (yPos == minSideTiltAngle)) {
              // get robot back to neutral position AND USE RANDOM-NESS TO DETERMINE IF ROBOT WILL MAKE ANOTHER ROCKING MOTION
              Reset();
              Change = true;
              doingAltTilt = false;
              if (random(100) > reRockProb - 1) {  // has 70% chance of rocking left then right again
                runningRandomMotion = false;  // exits rocking WHILE loop
              }
            }
          }
        }
      }
      else if (Motion == 1) {  //rock the robot forward and then back
        while (runningRandomMotion) {  //WHILE loop has a chance of rerunning several times

          //tilting forward code
          xPos -= speedd;
          yPos -= speedd;
          ForwardOverflowCheck();
          CheckForChange();
          if (Change) {
            DriverPinOut();
          }
          else {
            runningRandomMotion = false;
          }
          if ((xPos == minForwardTiltAngle) && (yPos == minForwardTiltAngle)) {
            // get robot back to neutral position and begin tilting to other side
            Reset();
            Change = true;
            doingAltTilt = true;
          }

          //tilting backwards code
          while (doingAltTilt) {
            xPos += speedd;
            yPos += speedd;
            BackOverflowCheck();
            CheckForChange();
            if (Change) {
              DriverPinOut();
            }
            else {
              runningRandomMotion = false;
            }
            if ((xPos == maxBackTiltAngle) && (yPos == maxBackTiltAngle)) {
              // get robot back to neutral position AND USE RANDOM-NESS TO DETERMINE IF ROBOT WILL MAKE ANOTHER ROCKING MOTION
              Reset();
              Change = true;
              doingAltTilt = false;
              if (random(100) > reRockProb - 1) {  // has 70% chance of rocking again in same direction
                runningRandomMotion = false;
              }
            }
          }
        }
      }
    }
    else { // will do a single tilt in random direction
      Motion = random(4);  // motion = 0, 1, 2, or 3. # dictates which direction base will tilt
      while (runningRandomMotion) {
        switch (Motion) {
          case 0:  //Tilt left
            xPos -= speedd;
            yPos += speedd;
            LeftOverflowCheck();
            if ((xPos == minSideTiltAngle) && (yPos == maxSideTiltAngle)) {
              // get out of this case and restart RandomMovement
              runningRandomMotion = false;
            }
            break;
          case 1:  //Tilt right
            xPos += speedd;
            yPos -= speedd;
            RightOverflowCheck();
            if ((xPos == maxSideTiltAngle) && (yPos == minSideTiltAngle)) {
              // get out of this case and restart RandomMovement
              runningRandomMotion = false;
            }
            break;
          case 2:  //Tilt forward
            xPos -= speedd;
            yPos -= speedd;
            ForwardOverflowCheck();
            if ((xPos == minForwardTiltAngle) && (yPos == minForwardTiltAngle)) {
              // get out of this case and restart RandomMovement
              runningRandomMotion = false;
            }
            break;
          case 3:  //Tilt back
            xPos += speedd;
            yPos += speedd;
            BackOverflowCheck();
            if ((xPos == maxBackTiltAngle) && (yPos == maxBackTiltAngle)) {
              // get out of this case and restart RandomMovement
              runningRandomMotion = false;
            }
            break;
        }
        CheckForChange();
        if (Change) {
          DriverPinOut();
        }
        else {
          runningRandomMotion = false;
        }
      }
    }
    Reset();
  }
  Change = true;
}
