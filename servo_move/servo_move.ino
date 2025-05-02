#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>
#include <printf.h>

#include <Servo.h>

// create objects
Servo s1;
Servo s2;

const int s1Pin = 6;
const int s2Pin = 7;


void setup() {

  s1.attach(s1Pin);  // attach servo on pin
  s2.attach(s2Pin);

  s1.write(0);  // set to 0 degrees
  s2.write(0);

  delay(1000);

}

void loop() {
  // put your main code here, to run repeatedly:

  // 0 to 180
  for (int i = 0; i<180; i+=1){
    s1.write(i);
    s2.write(i);

    delay(25);  // random move delay
  }

  // 180 to 0
  for (int i=180; i>0; i-=1){
    s1.write(i);
    s2.write(i);

    delay(25);
  }

}
