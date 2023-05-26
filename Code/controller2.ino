#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <AccelStepper.h>

// Define stepper motor pins
#define X_STEP_PIN 2
#define X_DIR_PIN 5
#define Y_STEP_PIN 3
#define Y_DIR_PIN 6
#define Z_STEP_PIN 4
#define Z_DIR_PIN 7
#define ENABLE_PIN 8

// Create AccelStepper instances
AccelStepper stepperX(1, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(1, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper stepperZ(1, Z_STEP_PIN, Z_DIR_PIN);

float maxSpeed = 10000;
float maxAccel = 10000;
// Define target speeds (steps per second)
float speedX = maxSpeed;
float speedY = maxSpeed;
float speedZ = maxSpeed;

// Define acceleration (steps per second per second)
float accelerationX = maxAccel;
float accelerationY = maxAccel;
float accelerationZ = maxAccel;

// Create PCA9685 instance
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo settings
#define SERVOMIN 150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 600 // This is the 'maximum' pulse length count (out of 4096)

// Add global variables to store motor states
uint16_t motorCommands = 0;

void setup() {
  // Set initial speed and acceleration for each motor
  stepperX.setMaxSpeed(speedX);
  stepperX.setAcceleration(accelerationX);

  stepperY.setMaxSpeed(speedY);
  stepperY.setAcceleration(accelerationY);

  stepperZ.setMaxSpeed(speedZ);
  stepperZ.setAcceleration(accelerationZ);

  // Initialize PCA9685
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  Serial.begin(9600);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Read incoming data from Python script
  if (Serial.available() >= 2) {
    uint16_t receivedValue = Serial.read() | (Serial.read() << 8);
    bool check = (receivedValue >> 15) & 1;
    if (check & (motorCommands != receivedValue & 0b0011111111111111)) {
      motorCommands = receivedValue & 0b0011111111111111;
    }
  }
  // Control stepper motors
  controlStepper(stepperX, (motorCommands >> 0) & 0b11);
  controlStepper(stepperY, (motorCommands >> 2) & 0b11);
  controlStepper(stepperZ, (motorCommands >> 4) & 0b11);
  

  // Control servo motors
  controlServo(0, (motorCommands >> 6) & 0b11);
  controlServo(1, (motorCommands >> 8) & 0b11);
  controlServo(2, (motorCommands >> 10) & 0b11);
  controlServo(3, (motorCommands >> 12) & 0b11);
}

void controlStepper(AccelStepper& stepper, uint8_t command) {
  if (command == 1) {
    // Accelerate the motors to maxSpeed
    stepper.moveTo(stepper.currentPosition() + 100000); // Set a large target position to move in the positive direction
  } else if (command == 2) {
    // Accelerate the motors to -maxSpeed
    stepper.moveTo(stepper.currentPosition() - 100000); // Set a large target position to move in the negative direction
  } else {
    // Decelerate the motors to a stop
    int32_t stoppingDistance = (stepper.speed() * stepper.speed()) / (2 * stepper.acceleration()); // Calculate the stopping distance based on the current speed
    stepper.moveTo(stepper.currentPosition() + (stepper.speed() > 0 ? stoppingDistance : -stoppingDistance)); // Set the target position to the calculated stopping distance
  }
  stepper.run();
}

void setServoPosition(uint8_t servoIndex, uint8_t angle) {
  if (angle > 180) {
    angle = 180;
  }
  uint16_t pulseLength = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servoIndex, 0, pulseLength);
}

void controlServo(uint8_t servoIndex, uint8_t command) {
  if (command == 0) {
    setServoPosition(servoIndex, 90);
  } 
  else if (command == 1) {
    setServoPosition(servoIndex, 110);
  }
  else if (command == 2) {
    setServoPosition(servoIndex, 70);
  }
}
