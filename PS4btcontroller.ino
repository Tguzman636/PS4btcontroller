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

PS4BT PS4(&Btd, PAIR);  // Pairing
//PS4BT PS4(&Btd);        // Already Paired

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
int AngleMin = 0;
int AngleMax = 255-64; // Max Angle
int counter = 0;
int delayer = 0;

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
}

void loop() {
  Usb.Task();
  ////Serial.print("\r\nLooping");
      switch(state){
        case 0:
          JoystickMovement();
          break;
        case 1:
          CircularMovement();
          break;
        case 2:
          LRMovement();
          break;
        case 3:
          FRMovement();
          break;
        case 4:
          BounceMovement();
          break;
        case 5:
          PatientMode();
          break;
        default:
          break;
      }
}

void CheckForChange() {
  //Serial.print("State: ");
  //Serial.println(state);
  Usb.Task();
  if (PS4.connected()) {
  if(((abs(PS4.getAnalogHat(LeftHatX) - 127) > 15) && (abs(PS4.getAnalogHat(LeftHatY) - 127) > 15)) && (state != 0)){
    state = 0;
    //Serial.println("Joystick Mode Activated");
    //PS4.setLed(red, green, blue);
    Reset();
  }
  if((PS4.getButtonClick(SQUARE)) && (state != 1)){
    state = 1;
    //PS4.setLed(red, green, blue);
    //Serial.println("Circular Mode Activated");
    Reset();
  } else
  if((PS4.getButtonClick(CIRCLE))  && (state != 2)){
     state = 2;
     //PS4.setLed(red, green, blue);
     //Serial.println("Left/Right Mode Activated");
     Reset();
  } else
  if((PS4.getButtonClick(CROSS))  && (state != 3)){
     state = 3;
     //PS4.setLed(red, green, blue);
     //Serial.println("Front/Back Mode Activated");
     Reset();
  } else
  if((PS4.getButtonClick(TRIANGLE))  && (state != 4)){
     state = 4;
     //PS4.setLed(red, green, blue);
     //Serial.println("Bounce Mode Activated");
     Reset();
  } else
  if((PS4.getButtonClick(TOUCHPAD))  && (state != 5)){
    state = 5;
    //PS4.setLed(red, green, blue);
    //Serial.println("Patient Mode Activated");
    Reset();
  }
  /*
  if(PS4.checkDpad(UP)){
    speedd++;
    if(speedd > SpeeddMax) {
      speedd = SpeeddMax;
    }
  }
  if(PS4.checkDpad(DOWN)){
    speedd--;
    if(speedd < SpeeddMin) {
      speedd = SpeeddMin;
    }
  }
 */
  }
}

void OverflowCheck() {
    if (yPos < 0) {yPos = 0;}
    if (xPos < 0) {xPos = 0;}
    if (yPos > 255) {yPos = 255;}
    if (xPos > 255) {xPos = 255;} 
}

void DriverPinOut() {
  for (int i=0; i <= delayer ; i++) {
    if (i == delayer) {
      analogWrite(driverPin1, xPos);
      analogWrite(driverPin2, yPos);
      delay(timing);
    }
  }
}

void Reset() {
  while((xPos != 127) || (yPos != 127)) {
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
      if ((xPos + yPos) < (185*2)) {
        xPos += speedd;
      }
      yPos -= speedd;
    }
    if (PS4.getAnalogHat(LeftHatX) < 117) {
      if ((xPos + yPos) < (185*2)) {
        yPos += speedd;
      }
      xPos -= speedd;
    }
    if (PS4.getAnalogHat(LeftHatY) > 137) {
      if ((xPos + yPos) < (185*2)) {
        xPos += speedd;
        yPos += speedd;
      }
    }
    if (PS4.getAnalogHat(LeftHatY) < 117) {
      xPos -= speedd;
      yPos -= speedd;
    }
    OverflowCheck();
    CheckForChange();
    if (Change) {
      DriverPinOut();
    }
  }
}

void FRMovement() {
  Motion = 0;
  while (Change) {
    switch(Motion) {
      case 0:   // Nose Down
        xPos -= speedd;
        yPos -= speedd;
        OverflowCheck();
        if ((xPos == AngleMin) && (yPos == AngleMin)) {
          Motion = 1;
          delay(2000);
        }
        break;
      case 1:   // Nose Up
        xPos += speedd;
        yPos += speedd;
        OverflowCheck();
        if ((xPos == AngleMax) && (yPos == AngleMax)) {
          Motion = 0;
          delay(2000);
        }
        break;
      case 2:   // Left Orientated
        xPos += speedd;
        yPos -= speedd;
        OverflowCheck();
        if ((xPos == AngleMax) && (yPos == AngleMin)) {
          //Motion = #;
        }
        break;
      case 3:   // Right Orientated
        xPos -= speedd;
        yPos += speedd;
        OverflowCheck();
        if ((xPos == AngleMin) && (yPos == AngleMax)) {
          //Motion = #;
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
    switch(Motion) {
      case 0:   // Nose Down
        xPos -= speedd;
        yPos -= speedd;
        OverflowCheck();
        if ((xPos == 0) && (yPos == 0)) {
          //Motion = #;
        }
        break;
      case 1:   // Nose Up
        xPos += speedd;
        yPos += speedd;
        OverflowCheck();
        if ((xPos == 255) && (yPos == 255)) {
          //Motion = #;
        }
        break;
      case 2:   // Left Orientated
        xPos += speedd;
        yPos -= speedd;
        OverflowCheck();
        if ((xPos == 255) && (yPos == 0)) {
          Motion = 3;
          delay(2000);
        }
        break;
      case 3:   // Right Orientated
        xPos -= speedd;
        yPos += speedd;
        OverflowCheck();
        if ((xPos == 0) && (yPos == 255)) {
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
    Serial.print(" Xpos:");
  Serial.println(xPos);
  Serial.print(" Ypos:");
  Serial.println(yPos);
    switch(Motion) {
      case 0:   // Nose Down
        xPos -= speedd;
        yPos -= speedd;
        OverflowCheck();
        if ((xPos == 0) && (yPos == 0)) {
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
        OverflowCheck();
        if (xPos < AngleMax) {
          xPos = AngleMax;
        }
        if (yPos > AngleMax) {
          yPos = AngleMax;
        }
        if ((xPos == AngleMax) && (yPos == AngleMax)) {
          Motion = 3;
          counter = 0;
          delay(2000);
        }
        break;
      case 2:   // Right Orientated
        xPos += speedd;
        yPos -= speedd;
        OverflowCheck();
        if ((xPos == 255) && (yPos == 0)) {
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
        OverflowCheck();
        if ((xPos == 0) && (yPos == 255)) {
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
  while(Change) {
    // Detect Patient Button Press similiar to Joystick
    CheckForChange();
    if (Change) {
      DriverPinOut();
    }
  }
  Change = true;
}

void BounceMovement() {
  while(Change){
    Serial.println("this is suppose to be bouncing");
    CheckForChange();
    if (Change) {
      bbStepper.step(steps);
    }
  }
  bbStepper.step(0);
  Change = true;
}
