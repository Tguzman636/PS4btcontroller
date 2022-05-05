// Gamebar Pins
#define LRButtonPin 6
#define FBButtonPin 7
#define BouncePin 8


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
  if (digitalRead(LRButtonPin) == HIGH) {
    Serial.write('0');
  }

  if (digitalRead(FBButtonPin) == HIGH) {
    Serial.write('1');
  }

  if (digitalRead(BouncePin) == HIGH) {
    Serial.write('2');
  }
}
