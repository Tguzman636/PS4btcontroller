#include <PS4BT.h>
#include <usbhub.h>
#include <math.h>

#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

USB Usb;

BTD Btd(&Usb);

PS4BT PS4(&Btd, PAIR);

char state = 0;
char Motion = 0;
bool Change = true; // Check if a state change occurs

void setup() {
  Serial.begin(19200);
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }
  Serial.print(F("\r\nPS4 Bluetooth Library Started"));
}

/*
States (COLORS?)

0 - Joystick        (RED)     R Joystick
1 - Circular Loop   (BLUE)    □ Button
2 - Left/Right Loop (GREEN)   O Button
3 - Front/Back Loop (YELLOW)  X Button
4 - Bouncing Loop   (PURPLE)  △ Button
5 - Patient Mode    (ORANGE)  Touchpad
*/

void loop() {
  Usb.Task();

  if (PS4.connected()) {
      //PS4.setLed(red, green, blue);
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
}

void CheckForChange() {
  if(((abs(PS4.getAnalogHat(LeftHatX)) > 5) && (abs(PS4.getAnalogHat(LeftHatY)) > 5)) && (state != 0)){
    state = 0;
    Reset();
  }
  if((PS4.getButtonClick(SQUARE)) && (state != 1)){
    state = 1;
    Reset();
  }
  if((PS4.getButtonClick(CIRCLE))  && (state != 2)){
     state = 2;
     Reset();
  }
  if((PS4.getButtonClick(CROSS))  && (state != 3)){
     state = 3;
     Reset();
  }
  if((PS4.getButtonClick(TRIANGLE))  && (state != 4)){
     state = 4;
     Reset();
  }
  if((PS4.getButtonClick(TOUCHPAD))  && (state != 5)){
    state = 5;
    Reset();
  }
  /*if(PS4.DPAD){
    speedd++;
  }
  if(PS4.DPAD){
    speedd--;
  }*/
}

void OverflowCheck() {
  if (yPos < 0) {yPos = 0;}
  if (xPos < 0) {xPos = 0;}
  if (yPos > 255) {yPos = 255;}
  if (xPos > 255) {xPos = 255;}
}

void DriverPinOut() {
  analogWrite(driverPin1, xPos);
  analogWrite(driverPin2, yPos);
  delay(10);
}

void Reset() {
  while((xPos != 127) && (yPos != 127)) {
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
  Change = false;
}

void JoystickMovement() {
  while (Change) {
    if (PS4.getAnalogHat(LeftHatX) > 137) {
      xPos += speedd;
    }
    if (PS4.getAnalogHat(LeftHatX) < 117) {
      xPos -= speedd;
    }
    if (PS4.getAnalogHat(LeftHatY) > 137) {
      yPos += speedd;
    }
    if (PS4.getAnalogHat(LeftHatY) < 117) {
      yPos -= speedd;
    }
    OverflowCheck();
    CheckForChange();
    if (Change) {
      DriverPinOut();
    }
  }
}

void FBMovement() {
  Motion = 0;
  while (Change) {
    switch(Motion) {
      case 0:   // Nose Down
        xPos -= speedd;
        yPos -= speedd;
        OverflowCheck();
        if ((xPos == 0) && (yPos == 0)) {
          Motion = 1;
        }
        break;
      case 1:   // Nose Up
        xPos += speedd;
        yPos += speedd;
        OverflowCheck();
        if ((xPos == 255) && (yPos == 255)) {
          Motion = 0;
        }
        break;
      case 2:   // Left Orientated
        xPos += speedd;
        yPos -= speedd;
        OverflowCheck();
        if ((xPos == 255) && (yPos == 0)) {
          //Motion = #;
        }
        break;
      case 3:   // Right Orientated
        xPos -= speedd;
        yPos += speedd;
        OverflowCheck();
        if ((xPos == 0) && (yPos == 255)) {
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
        }
        break;
      case 3:   // Right Orientated
        xPos -= speedd;
        yPos += speedd;
        OverflowCheck();
        if ((xPos == 0) && (yPos == 255)) {
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
  Motion = 0;
  while (Change) {
    switch(Motion) {
      case 0:   // Nose Down
        xPos -= speedd;
        yPos -= speedd;
        OverflowCheck();
        if ((xPos == 0) && (yPos == 0)) {
          Motion = 2;
        }
        break;
      case 1:   // Nose Up
        xPos += speedd;
        yPos += speedd;
        OverflowCheck();
        if ((xPos == 255) && (yPos == 255)) {
          Motion = 3;
        }
        break;
      case 2:   // Left Orientated
        xPos += speedd;
        yPos -= speedd;
        OverflowCheck();
        if ((xPos == 255) && (yPos == 0)) {
          Motion = 1;
        }
        break;
      case 3:   // Right Orientated
        xPos -= speedd;
        yPos += speedd;
        OverflowCheck();
        if ((xPos == 0) && (yPos == 255)) {
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
