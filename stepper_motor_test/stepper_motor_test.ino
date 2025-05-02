#include <Stepper.h>

int STEPS = 100;
int pins[] = {8, 9, 10, 11};
Stepper stepper(STEPS, pins[0], pins[1], pins[2], pins[3]);

// prev reading from analog input
int prev = 0;

void setup() {

  stepper.setSpeed(30);

}

void loop() {
  // get sensor val
  int val = analogRead(0);

  stepper.step(val - prev);
  prev = val;
}
