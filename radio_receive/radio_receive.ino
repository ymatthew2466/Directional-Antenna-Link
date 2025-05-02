#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Create RF24 object using CE pin 9 and CSN pin 8
RF24 radio(9, 8);

// Address for communication (must match on both devices)
const byte address[6] = "00001";

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
    Serial.print("Received: ");
    Serial.println(text);
  }
}