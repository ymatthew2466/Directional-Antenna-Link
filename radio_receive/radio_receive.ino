#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Stepper.h>
#include <Servo.h>


// CE pin 9 and CSN pin 8
RF24 radio(9, 8);

// Address for communication 
const byte address[6] = "00001";

// Stepper
#define STEPS_PER_REV 100
Stepper pan(STEPS_PER_REV, 8, 9, 10, 11);  // pins

// Servo
Servo tilt;

// return # of N samples had RPD == 1 (greater than -64 dBm)
int measureStrength(int N=20) {
  int c = 0;
  for (int i = 0; i < N; i++) {
    if (radio.testRPD()){
      c += 1;
    }
    delay(1);
  }
  return c;
}

void setup() {
  Serial.begin(9600);
  if (!radio.begin()) {
    Serial.println("Radio hardware not responding!");
    while (1); // Halt if radio isn't found
  }

  radio.setPALevel(RF24_PA_LOW);          // RF Power level
  radio.setChannel(108);                  // Channel must match transmitter
  radio.setDataRate(RF24_250KBPS);        // Data rate must match transmitter

  radio.openReadingPipe(0, address);      // Open reading pipe on address
  radio.startListening();                 // Set module as receiver

  Serial.println("Receiver ready, waiting for messages...");
}

void loop() {
  if (radio.available()) {
    char text[32] = {0};                  // Buffer for incoming message
    radio.read(&text, sizeof(text));      // Read message into buffer
    // Serial.print("Received: ");
    // Serial.println(text);

    // RPD bit
    // 1 if greater than -64 dBm, 0 if less
    Serial.println("" + radio.testRPD());


    // XZ-axis (stepper motor)  
    int basePan = measureStrength();
    pan.step(1);
    int upPan = measureStrength();

    // if it's worse, then undo
    if (upPan < basePan){
      pan.step(-1);  
    } 

    // Y-axis (servo)
    static int angle = 90;
    int baseTilt = measureStrength();
    tilt.write(angle + 1);
    delay(150);
    int upTilt = measureStrength();
    if (upTilt >= baseTilt) {
      angle = min(angle + 1, 180);
    }
    else {
      tilt.write(angle - 1);
      delay(150);
    }

    delay(100);
  }
}