//#include <Esplora.h>

// Gamebar Pins
#define LRButtonPin 6
#define FBButtonPin 7
#define BouncePin 8

// Key for now
// 0 is the white button and is the LR Button on Pin 6
// 1 is the red button and is the FB Button on Pin 7
// 2 is currently not working


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
}

void loop() {
  delay(50);
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
}
