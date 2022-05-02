// UART Communication
#define remoteHasControl 8 // When this pin is high, PS4 remote will have motion control

// Priority Scheme Parameter
char data_rcvd == '1';

void setup() {
  Serial.begin(115200);
  Serial.println("Startup Done");
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    while (1); // Halt
  }


  // UART Communication - Slave
  pinMode(remoteHasControl, INPUT);
}

void loop() {
  if (Serial.available()) {
    data_rcvd = Serial.read(); // read one byte from serial buffer and save to data_rcvd

    if (data_rcvd == '1') digitalWrite(remoteHasControl, HIGH); // PS4 remote will have control (therapist mode)
    if (data_rcvd == '0') digitalWrite(remoteHasControl, LOW); // gamebar will have control (patient mode)


  }
  while (data_rcvd == '0') {
    // hook up buttons to motion here
  }
}
