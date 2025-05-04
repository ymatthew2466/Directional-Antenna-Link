#include <PID_v1.h>
#include <Servo.h>
#include <Stepper.h>

// Define pins for photoresistors
#define PHOTO_TOP_RIGHT A0
#define PHOTO_BOTTOM_RIGHT A1
#define PHOTO_BOTTOM_LEFT A2

// Define pins for stepper motor (horizontal movement)
#define STEPPER_STEPS_PER_REV 200
#define STEPPER_PIN1 2
#define STEPPER_PIN2 3
#define STEPPER_PIN3 4
#define STEPPER_PIN4 5

// Define pin for servo motor (vertical movement)
#define SERVO_VERTICAL_PIN 10

// Motor control constants
#define SERVO_MIN_POS 0
#define SERVO_MAX_POS 180
#define SERVO_CENTER 90
#define STEPPER_MIN_POS -100
#define STEPPER_MAX_POS 100
#define STEPPER_CENTER 0

// Movement control constants
#define SCANNING_SPEED 3
#define STEPPER_SCANNING_SPEED 2
#define MAX_SENSOR_VALUE 100
#define DEAD_ZONE 2

// State definitions
#define STATE_INITIAL_SCAN 0
#define STATE_TRACKING 1

// Create motor objects
Stepper stepperHorizontal(STEPPER_STEPS_PER_REV, STEPPER_PIN1, STEPPER_PIN2, STEPPER_PIN3, STEPPER_PIN4);
Servo servoVertical;

// Motor position tracking
int horizontalPos = STEPPER_CENTER;
int verticalPos = SERVO_CENTER;

// Sensor readings
int topRightVal = 0;
int bottomRightVal = 0;
int bottomLeftVal = 0;

// Directional differences
int horizontalDiff = 0;
int verticalDiff = 0;

// Scan state
int currentState = STATE_INITIAL_SCAN;
int scanPhase = 0;
unsigned long scanStartTime = 0;
int maxLightValue = 0;
int maxLightHPos = STEPPER_CENTER;
int maxLightVPos = SERVO_CENTER;

// PID variables
double horizontalSetpoint, horizontalInput, horizontalOutput;
double verticalSetpoint, verticalInput, verticalOutput;

// PID tuning parameters
double Kp = 0.5, Ki = 0.1, Kd = 0.2;

// PID controller objects
PID horizontalPID(&horizontalInput, &horizontalOutput, &horizontalSetpoint, Kp, Ki, Kd, DIRECT);
PID verticalPID(&verticalInput, &verticalOutput, &verticalSetpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);

  stepperHorizontal.setSpeed(60);
  servoVertical.attach(SERVO_VERTICAL_PIN);
  servoVertical.write(verticalPos);

  horizontalSetpoint = 0;
  verticalSetpoint = 0;

  horizontalPID.SetOutputLimits(-5, 5);
  verticalPID.SetOutputLimits(-10, 10);

  horizontalPID.SetMode(AUTOMATIC);
  verticalPID.SetMode(AUTOMATIC);

  delay(1000);

  scanStartTime = millis();
  Serial.println("Starting initial scan sequence...");
}

void loop() {
  // Read and map sensor values
  topRightVal = map(analogRead(PHOTO_TOP_RIGHT), 0, 1023, 0, MAX_SENSOR_VALUE);
  bottomRightVal = map(analogRead(PHOTO_BOTTOM_RIGHT), 0, 1023, 0, MAX_SENSOR_VALUE);
  bottomLeftVal = map(analogRead(PHOTO_BOTTOM_LEFT), 0, 1023, 0, MAX_SENSOR_VALUE);

  int totalLight = topRightVal + bottomRightVal + bottomLeftVal;

  horizontalDiff = bottomRightVal - bottomLeftVal;
  verticalDiff = topRightVal - bottomRightVal;

  Serial.print("Sensors - TR: "); Serial.print(topRightVal);
  Serial.print(", BR: "); Serial.print(bottomRightVal);
  Serial.print(", BL: "); Serial.print(bottomLeftVal);
  Serial.print(" | Diffs - H: "); Serial.print(horizontalDiff);
  Serial.print(", V: "); Serial.print(verticalDiff);
  Serial.print(" | State: "); Serial.println(currentState);

  switch (currentState) {
    case STATE_INITIAL_SCAN:
      performInitialScan(totalLight);
      break;
    case STATE_TRACKING:
      performTracking();
      break;
  }

  delay(20);
}

void performInitialScan(int lightValue) {
  unsigned long currentTime = millis();

  if (lightValue > maxLightValue) {
    maxLightValue = lightValue;
    maxLightHPos = horizontalPos;
    maxLightVPos = verticalPos;
  }

  switch (scanPhase) {
    case 0:
      horizontalPos += STEPPER_SCANNING_SPEED;
      if (horizontalPos >= STEPPER_MAX_POS) {
        horizontalPos = STEPPER_MAX_POS;
        scanPhase = 1;
      }
      stepperHorizontal.step(STEPPER_SCANNING_SPEED);
      break;

    case 1:
      verticalPos += SCANNING_SPEED;
      if (verticalPos >= SERVO_MAX_POS) {
        verticalPos = SERVO_MAX_POS;
        scanPhase = 2;
      }
      servoVertical.write(verticalPos);
      break;

    case 2:
      horizontalPos -= STEPPER_SCANNING_SPEED;
      if (horizontalPos <= STEPPER_MIN_POS) {
        horizontalPos = STEPPER_MIN_POS;
        scanPhase = 3;
      }
      stepperHorizontal.step(-STEPPER_SCANNING_SPEED);
      break;

    case 3:
      verticalPos -= SCANNING_SPEED;
      if (verticalPos <= SERVO_MIN_POS) {
        verticalPos = SERVO_MIN_POS;
        scanPhase = 4;
      }
      servoVertical.write(verticalPos);
      break;

    case 4:
      horizontalPos += STEPPER_SCANNING_SPEED;
      if (horizontalPos >= STEPPER_CENTER) {
        horizontalPos = STEPPER_CENTER;
        scanPhase = 5;
      }
      stepperHorizontal.step(STEPPER_SCANNING_SPEED);
      break;

    case 5:
      if (abs(horizontalPos - maxLightHPos) > 2 || abs(verticalPos - maxLightVPos) > 2) {
        if (horizontalPos < maxLightHPos) {
          horizontalPos++;
          stepperHorizontal.step(1);
        } else if (horizontalPos > maxLightHPos) {
          horizontalPos--;
          stepperHorizontal.step(-1);
        }

        if (verticalPos < maxLightVPos) {
          verticalPos++;
          servoVertical.write(verticalPos);
        } else if (verticalPos > maxLightVPos) {
          verticalPos--;
          servoVertical.write(verticalPos);
        }
      } else {
        Serial.print("Max light found at H:"); Serial.print(maxLightHPos);
        Serial.print(" V:"); Serial.println(maxLightVPos);
        Serial.println("Switching to tracking mode");
        currentState = STATE_TRACKING;
      }
      break;
  }
}

void performTracking() {
  horizontalInput = horizontalDiff;
  verticalInput = verticalDiff;

  horizontalPID.Compute();
  verticalPID.Compute();

  int horizontalSteps = 0;
  bool verticalMoved = false;

  if (abs(horizontalDiff) > DEAD_ZONE) {
    horizontalSteps = -round(horizontalOutput);
    horizontalPos += horizontalSteps;

    if (horizontalPos < STEPPER_MIN_POS) {
      horizontalSteps -= (horizontalPos - STEPPER_MIN_POS);
      horizontalPos = STEPPER_MIN_POS;
    } else if (horizontalPos > STEPPER_MAX_POS) {
      horizontalSteps -= (horizontalPos - STEPPER_MAX_POS);
      horizontalPos = STEPPER_MAX_POS;
    }

    if (horizontalSteps != 0) {
      stepperHorizontal.step(horizontalSteps);
      Serial.print("Horizontal steps: "); Serial.println(horizontalSteps);
    }
  }

  if (abs(verticalDiff) > DEAD_ZONE) {
    verticalPos -= verticalOutput;
    verticalPos = constrain(verticalPos, SERVO_MIN_POS, SERVO_MAX_POS);
    servoVertical.write(verticalPos);
    verticalMoved = true;
  }

  if (horizontalSteps != 0 || verticalMoved) {
    Serial.print("Tracking - Positions: H:"); Serial.print(horizontalPos);
    Serial.print(" V:"); Serial.println(verticalPos);
  }
}