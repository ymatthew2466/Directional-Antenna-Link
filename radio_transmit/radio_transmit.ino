// #include <SPI.h>
// #include <nRF24L01.h>
// #include <RF24.h>

// // Create RF24 object using CE pin 9 and CSN pin 8
// RF24 radio(9, 8);

// // Address for communication (must match on both devices)
// const byte address[6] = "00001";

// void setup() {
//   Serial.begin(9600);
//   if (!radio.begin()) {
//     Serial.println("Radio hardware not responding!");
//     while (1); // Halt if radio isn't found
//   }

//   radio.setPALevel(RF24_PA_LOW);          // RF Power level: MIN, LOW, HIGH, MAX
//   radio.setChannel(108);                  // Channel 108, out of WiFi range
//   radio.setDataRate(RF24_250KBPS);        // Lower data rate = better range

//   radio.openWritingPipe(address);         // Set the writing pipe
//   radio.stopListening();                  // Set module as transmitter

//   Serial.println("Transmitter ready");
// }

// void loop() {
//   char text[] = "hello world";
//   bool success = radio.write(&text, sizeof(text));

//   if (success) {
//     Serial.println("Sent: hello world");
//   } else {
//     Serial.println("Send failed");
//   }

//   delay(1000); // Wait 1 second before next send
// }




#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 8);
const byte address[6] = "00001";

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(108);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(address);
  radio.setAutoAck(false);    // continuous stream, no ACK handshake
  radio.stopListening();
  Serial.println("TX: streaming continuously…");
}

void loop() {
  // send a 1-byte “beacon” as fast as possible
  const byte beacon = 0xAA;
  radio.write(&beacon, sizeof(beacon));
  // no delay!
}