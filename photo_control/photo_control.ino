// #include <Servo.h>
// #include <Stepper.h>

// //---- Pins ----
// #define PHOTO_TOP_RIGHT     A0
// #define PHOTO_BOTTOM_RIGHT  A1
// #define PHOTO_BOTTOM_LEFT   A2

// // For 28BYJ-48 stepper motor with ULN2003 driver
// #define STEPPER_STEPS_PER_REV 2038  // 28BYJ-48 has 2038 steps per revolution
// #define STEPPER_PIN1 2
// #define STEPPER_PIN2 4
// #define STEPPER_PIN3 3
// #define STEPPER_PIN4 5

// #define SERVO_VERTICAL_PIN 10

// //---- Limits & Constants ----
// #define SERVO_MIN_POS     0
// #define SERVO_MAX_POS     180
// #define SERVO_CENTER      90
// #define STEPPER_MIN_POS  -500
// #define STEPPER_MAX_POS   500
// #define STEPPER_CENTER     0

// #define MAX_SENSOR_VALUE  100
// #define DEAD_ZONE         3  // Increased tolerance zone for more stability

// //---- Hardware Objects ----
// Stepper stepperHorizontal(STEPPER_STEPS_PER_REV,
//                           STEPPER_PIN1, STEPPER_PIN2,
//                           STEPPER_PIN3, STEPPER_PIN4);
// Servo servoVertical;

// //---- Position Tracking ----
// int horizontalPos = STEPPER_CENTER;
// int verticalPos   = SERVO_CENTER;

// //---- Sensor & Error Variables ----
// int topRightVal    = 0;
// int bottomRightVal = 0;
// int bottomLeftVal  = 0;
// int horizontalDiff = 0;
// int verticalDiff   = 0;

// //---- Custom PID Variables ----
// // Horizontal PID
// float h_error = 0;
// float h_lastError = 0;
// float h_sumError = 0;
// float h_output = 0;

// // Vertical PID
// float v_error = 0;
// float v_lastError = 0;
// float v_sumError = 0;
// float v_output = 0;

// // PID constants
// float h_Kp = 0.5;
// float h_Ki = 0.05;
// float h_Kd = 0.2;

// float v_Kp = 0.3;
// float v_Ki = 0.01;
// float v_Kd = 0.1;

// // Maximum integral windup prevention
// #define MAX_SUM_ERROR 100

// // Function prototypes
// void readSensors();
// void calculateErrors();
// void calculatePID();
// void moveHorizontal();
// void moveVertical();
// void debugOutput();

// unsigned long lastMoveTime = 0;
// unsigned long servoMoveInterval = 200; // ms between servo moves

// void setup() {
//   Serial.begin(9600);
//   Serial.println("Photoresistor Gimbal System Starting - Custom PID Implementation");

//   stepperHorizontal.setSpeed(15);
//   servoVertical.attach(SERVO_VERTICAL_PIN);
//   servoVertical.write(verticalPos);

//   delay(1000);  // Allow time for motors to initialize
//   Serial.println("Continuous tracking mode active");
// }

// void loop() {
//   readSensors();
//   calculateErrors();
//   calculatePID();
//   moveHorizontal();
//   // moveVertical();
//   debugOutput();
//   delay(50);
// }

// /**
//  * Read all photoresistor values and map them to a standard range
//  */
// void readSensors() {
//   topRightVal    = map(analogRead(PHOTO_TOP_RIGHT),    0, 1023, 0, MAX_SENSOR_VALUE);
//   bottomRightVal = map(analogRead(PHOTO_BOTTOM_RIGHT), 0, 1023, 0, MAX_SENSOR_VALUE);
//   bottomLeftVal  = map(analogRead(PHOTO_BOTTOM_LEFT),  0, 1023, 0, MAX_SENSOR_VALUE);
// }

// /**
//  * Calculate horizontal and vertical error values based on sensor readings
//  */
// void calculateErrors() {
//   horizontalDiff = bottomRightVal - bottomLeftVal;
//   verticalDiff   = topRightVal - bottomRightVal;
// }

// /**
//  * Calculate PID values for both axes
//  */
// void calculatePID() {
//   // Horizontal PID
//   h_error = horizontalDiff;
//   h_sumError += h_error;
//   h_sumError = constrain(h_sumError, -MAX_SUM_ERROR, MAX_SUM_ERROR);
//   float h_proportional = h_error * h_Kp;
//   float h_integral     = h_sumError * h_Ki;
//   float h_derivative   = (h_error - h_lastError) * h_Kd;
//   h_output = h_proportional + h_integral + h_derivative;
//   h_lastError = h_error;

//   // Vertical PID
//   v_error = verticalDiff;
//   v_sumError += v_error;
//   v_sumError = constrain(v_sumError, -MAX_SUM_ERROR, MAX_SUM_ERROR);
//   float v_proportional = v_error * v_Kp;
//   float v_integral     = v_sumError * v_Ki;
//   float v_derivative   = (v_error - v_lastError) * v_Kd;
//   v_output = v_proportional + v_integral + v_derivative;
//   v_lastError = v_error;
// }

// /**
//  * Control the horizontal stepper motor based on PID output
//  */
// void moveHorizontal() {
//   if (abs(horizontalDiff) > DEAD_ZONE) {
//     int horizontalSteps = round(h_output * 5);
//     if (h_output != 0 && abs(horizontalSteps) < 3) {
//       horizontalSteps = (h_output > 0) ? 3 : -3;
//     }

//     horizontalPos += horizontalSteps;
//     if (horizontalPos < STEPPER_MIN_POS) {
//       horizontalSteps -= (horizontalPos - STEPPER_MIN_POS);
//       horizontalPos = STEPPER_MIN_POS;
//     } else if (horizontalPos > STEPPER_MAX_POS) {
//       horizontalSteps -= (horizontalPos - STEPPER_MAX_POS);
//       horizontalPos = STEPPER_MAX_POS;
//     }

//     if (horizontalSteps != 0) {
//       Serial.print("Horizontal steps: ");
//       Serial.println(horizontalSteps);
//       stepperHorizontal.step(horizontalSteps);
//     }
//   }
// }

// /**
//  * Control the vertical servo motor based on PID output
//  */
// void moveVertical() {
//   unsigned long currentMillis = millis();
//   if (abs(verticalDiff) > DEAD_ZONE && (currentMillis - lastMoveTime) >= servoMoveInterval) {
//     float scaledOutput = v_output * 0.5;
//     scaledOutput = constrain(scaledOutput, -2, 2);

//     if (abs(scaledOutput) >= 0.5) {
//       verticalPos -= 20*round(scaledOutput);
//       verticalPos = constrain(verticalPos, SERVO_MIN_POS, SERVO_MAX_POS);
//       Serial.print("Vertical position: ");
//       Serial.println(verticalPos);
//       servoVertical.write(verticalPos);
//       lastMoveTime = currentMillis;
//     }
//   }
// }

// /**
//  * Output debug information to Serial monitor
//  */
// void debugOutput() {
//   Serial.print("Sensors → TR: "); Serial.print(topRightVal);
//   Serial.print("  BR: "); Serial.print(bottomRightVal);
//   Serial.print("  BL: "); Serial.print(bottomLeftVal);
//   Serial.print(" | Diffs → H: "); Serial.print(horizontalDiff);
//   Serial.print("  V: "); Serial.print(verticalDiff);
//   Serial.print(" | PID → H: "); Serial.print(h_output);
//   Serial.print("  V: "); Serial.print(v_output);
//   Serial.print(" | Pos → H: "); Serial.print(horizontalPos);
//   Serial.print("  V: "); Serial.println(verticalPos);
// }




#include <Servo.h>
#include <Stepper.h>

//---- Pins ----
#define PHOTO_TOP_RIGHT    A0
#define PHOTO_BOTTOM_RIGHT A1
#define PHOTO_BOTTOM_LEFT  A2

// For 28BYJ-48 stepper motor with ULN2003 driver
#define STEPPER_STEPS_PER_REV 2038  // 28BYJ-48 has 2038 steps per revolution
#define STEPPER_PIN1 2
#define STEPPER_PIN2 4
#define STEPPER_PIN3 3
#define STEPPER_PIN4 5

#define SERVO_VERTICAL_PIN 10

//---- Limits & Constants ----
#define SERVO_MIN_POS     0
#define SERVO_MAX_POS     180
#define SERVO_CENTER      90
#define STEPPER_MIN_POS  -500
#define STEPPER_MAX_POS   500
#define STEPPER_CENTER      0

#define MAX_SENSOR_VALUE  100
#define DEAD_ZONE         5  // Increased tolerance zone for more stability

//---- Hardware Objects ----
Stepper stepperHorizontal(STEPPER_STEPS_PER_REV,
                          STEPPER_PIN1, STEPPER_PIN2,
                          STEPPER_PIN3, STEPPER_PIN4);
Servo servoVertical;

//---- Position Tracking ----
int horizontalPos = STEPPER_CENTER;
int verticalPos   = SERVO_CENTER;

//---- Sensor & Error Variables ----
int topRightVal    = 0;
int bottomRightVal = 0;
int bottomLeftVal  = 0;
int horizontalDiff = 0;
int verticalDiff   = 0;

//---- Custom PID Variables ----
// Horizontal PID
float h_error = 0;
float h_lastError = 0;
float h_sumError = 0;
float h_output = 0;

// Vertical PID
float v_error = 0;
float v_lastError = 0;
float v_sumError = 0;
float v_output = 0;

// PID constants
float h_Kp = 0.5;
float h_Ki = 0.05;
float h_Kd = 0.2;

float v_Kp = 0.3;
float v_Ki = 0.01;
float v_Kd = 0.1;

// Maximum integral windup prevention
#define MAX_SUM_ERROR 100

// Function prototypes
void readSensors();
void calculateErrors();
void calculatePID();
void moveHorizontal();
void moveVertical();
void debugOutput();

unsigned long lastMoveTime = 0;
unsigned long servoMoveInterval = 200;

void setup() {
  Serial.begin(9600);
  Serial.println("Photoresistor Gimbal System Starting - Custom PID Implementation");

  stepperHorizontal.setSpeed(15);
  servoVertical.attach(SERVO_VERTICAL_PIN);
  servoVertical.write(verticalPos);

  delay(1000);
  Serial.println("Continuous tracking mode active");
}

void loop() {
  readSensors();
  calculateErrors();
  calculatePID();
  moveHorizontal();
  moveVertical();
  debugOutput();

  if (abs(horizontalDiff) < DEAD_ZONE) {
    h_sumError = 0;
  }

  delay(50);
}

void readSensors() {
  topRightVal    = map(analogRead(PHOTO_TOP_RIGHT),    0, 1023, 10, MAX_SENSOR_VALUE + 10);
  bottomRightVal = map(analogRead(PHOTO_BOTTOM_RIGHT), 0, 1023, 10, MAX_SENSOR_VALUE + 10);
  bottomLeftVal  = map(analogRead(PHOTO_BOTTOM_LEFT),  0, 1023, 10, MAX_SENSOR_VALUE + 10);

  Serial.print("Raw sensor values - TR: ");
  Serial.print(topRightVal);
  Serial.print(" BR: ");
  Serial.print(bottomRightVal);
  Serial.print(" BL: ");
  Serial.println(bottomLeftVal);
}

void calculateErrors() {
  horizontalDiff = bottomLeftVal - topRightVal;
  verticalDiff   = topRightVal - bottomRightVal;

  Serial.print("Raw Horizontal Diff: ");
  Serial.println(horizontalDiff);
}

void calculatePID() {
  h_error = horizontalDiff;
  float h_proportional = h_error * h_Kp;
  h_sumError += h_error;
  h_sumError = constrain(h_sumError, -MAX_SUM_ERROR, MAX_SUM_ERROR);
  float h_integral = h_sumError * h_Ki;
  float h_derivative = (h_error - h_lastError) * h_Kd;
  h_lastError = h_error;
  h_output = h_proportional + h_integral + h_derivative;

  v_error = verticalDiff;
  float v_proportional = v_error * v_Kp;
  v_sumError += v_error;
  v_sumError = constrain(v_sumError, -MAX_SUM_ERROR, MAX_SUM_ERROR);
  float v_integral = v_sumError * v_Ki;
  float v_derivative = (v_error - v_lastError) * v_Kd;
  v_lastError = v_error;
  v_output = v_proportional + v_integral + v_derivative;
}

void moveHorizontal() {
  if (abs(horizontalDiff) > DEAD_ZONE) {
    int direction = (bottomLeftVal > topRightVal) ? 1 : -1;
    int stepSize = abs(round(h_output * 5));
    if (stepSize < 3) stepSize = 3;

    int horizontalSteps = direction * stepSize;

    Serial.print("Direction: ");
    Serial.print(direction);
    Serial.print(", Step Size: ");
    Serial.print(stepSize);
    Serial.print(", Horizontal Steps: ");
    Serial.println(horizontalSteps);

    horizontalPos += horizontalSteps;

    if (horizontalPos < STEPPER_MIN_POS) {
      horizontalSteps -= (horizontalPos - STEPPER_MIN_POS);
      horizontalPos = STEPPER_MIN_POS;
    } else if (horizontalPos > STEPPER_MAX_POS) {
      horizontalSteps -= (horizontalPos - STEPPER_MAX_POS);
      horizontalPos = STEPPER_MAX_POS;
    }

    if (horizontalSteps != 0) {
      Serial.print("Horizontal steps: ");
      Serial.println(horizontalSteps);
      stepperHorizontal.step(horizontalSteps);
    }
  }
}

void moveVertical() {
  unsigned long currentMillis = millis();

  if (abs(verticalDiff) > DEAD_ZONE && (currentMillis - lastMoveTime) >= servoMoveInterval) {
    float scaledOutput = v_output * 0.5;
    scaledOutput = constrain(scaledOutput, -2, 2);

    if (abs(scaledOutput) >= 0.5) {
      verticalPos -= 15*round(scaledOutput);
      verticalPos = constrain(verticalPos, SERVO_MIN_POS, SERVO_MAX_POS);

      Serial.print("Vertical position: ");
      Serial.println(verticalPos);
      servoVertical.write(verticalPos);

      lastMoveTime = currentMillis;
    }
  }
}

void debugOutput() {
  Serial.print("Sensors → TR: "); Serial.print(topRightVal);
  Serial.print("  BR: "); Serial.print(bottomRightVal);
  Serial.print("  BL: "); Serial.print(bottomLeftVal);
  Serial.print(" | Diffs → H: "); Serial.print(horizontalDiff);
  Serial.print("  V: "); Serial.print(verticalDiff);
  Serial.print(" | PID → H: "); Serial.print(h_output);
  Serial.print("  V: "); Serial.print(v_output);
  Serial.print(" | Pos → H: "); Serial.print(horizontalPos);
  Serial.print("  V: "); Serial.println(verticalPos);
}