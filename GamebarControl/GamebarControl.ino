//#include <Esplora.h>


// Gamebar Pins
#define LRButtonPin 6
#define FBButtonPin 7
#define BouncePin 8

// Key for now
// 0 is the white button and is the LR Button on Pin 6
// 1 is the red button and is the FB Button on Pin 7
// 2 is currently not working

// Audio
#include "SerialMP3Player.h"
#define TX 11
#define RX 10
SerialMP3Player mp3(RX, TX);
const int buttonPing = 4;
const int buttonPiny = 13;
int newbuttong = 0;
int oldbuttong = 0;
int mpstateg = 0;
int buttonStateg = 0;
int newbuttony = 0;
int oldbuttony = 0;
int mpstatey = 0;
int buttonStatey = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Startup Done");
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif

  // Movement Buttons
  pinMode(LRButtonPin, INPUT);
  pinMode(FBButtonPin, INPUT);
  pinMode(BouncePin, INPUT);

  // Audio Buttons
  pinMode(buttonPing, INPUT);             // wait for init
  pinMode(buttonPiny, INPUT);
  mp3.sendCommand(CMD_SEL_DEV, 0, 2);   //select sd-card
  delay(500);             // wait for init
}

void loop() {
  delay(50);

  // Movement
  if (digitalRead(LRButtonPin) == HIGH) {
    Serial.write(0);
    Serial.print("\n0 has been sent\n");
    delay(100);
  }

  if (digitalRead(FBButtonPin) == HIGH) {
    Serial.write(1);
    Serial.print("\n1 has been sent\n");
    delay(100);
  }

  //  if (digitalRead(BouncePin) == HIGH) {
  //    Serial.write('2'); //2 currently is not working
  //    Serial.print("\n2 has been sent\n");
  //  }

  buttonStateg = digitalRead(buttonPing);
  buttonStatey = digitalRead(buttonPiny);
  if (buttonStateg == HIGH) {
    newbuttong = 1;
    Serial.print("\nGreen button has been pressed");
  } else {
    newbuttong = 0;
  }
  if (buttonStatey == HIGH) {
    newbuttony = 1;
    Serial.print("\nYellow button has been pressed");
  } else {
    newbuttony = 0;
  }
  if (oldbuttong == 0 && newbuttong == 1) {
    if (mpstateg == 0) {
      mp3.play(2, 25);
      Serial.print("\nSupposed to be playing");

    }
    if (mpstateg == 1) {
      mp3.play(6, 25);

    }
    if (mpstateg == 2) {
      mp3.play(5, 25);

    }
    if (mpstateg == 3) {
      mp3.play(4, 25);
    }
    if (mpstateg == 4) {
      mp3.play(3, 25);
    }
    if (mpstateg == 5) {
      mp3.stop();
      mpstateg = -1;
    }
    mpstateg = mpstateg + 1;

  }


  // Audio
  if (oldbuttony == 0 && newbuttony == 1) {
    if (mpstatey == 0) {
      mp3.play(1, 18);

    }
    if (mpstatey == 1) {
      mp3.play(10, 18);

    }
    if (mpstatey == 2) {
      mp3.play(7, 15);

    }
    if (mpstatey == 3) {
      mp3.play(8, 18);

    }
    if (mpstatey == 4) {
      mp3.play(9, 20);

    }
    if (mpstatey == 5) {
      mp3.stop();
      mpstatey = -1;
    }
    mpstatey = mpstatey + 1;

  }
  oldbuttong = newbuttong;
  oldbuttony = newbuttony;
}
