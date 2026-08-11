#include <AccelStepper.h>

#define stepPin 3
#define dirPin 2
#define joyX A0

#define SOL1 8
#define BUTTON 4
#define PULSE_MS 120

// --- Motor setup ---
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);
const int maxSpeed = 1500;       // maximum motor speed
const int acceleration = 800;    // motor acceleration

// --- Joystick setup ---
const int deadzone = 50;         // dead zone around center
const int center = 512;          // joystick center value
int lastSpeed = 0;

void setup() {
  Serial.begin(9600);

  // motor setup
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(acceleration);

  // I/O setup
  pinMode(SOL1, OUTPUT);
  digitalWrite(SOL1, LOW);
  pinMode(BUTTON, INPUT_PULLUP);
}

void loop() {
  // read joystick
  int x = analogRead(joyX); // 0 - 1023
  int speed = 0;

  // dead zone to keep the motor still near center
  if (x > center + deadzone) {
    speed = map(x, center + deadzone, 1023, 0, maxSpeed);
  } else if (x < center - deadzone) {
    speed = map(x, center - deadzone, 0, 0, -maxSpeed);
  } else {
    speed = 0;
  }

  // update motor speed only when it changes
  if (speed != lastSpeed) {
    stepper.setSpeed(speed);
    lastSpeed = speed;
  }

  // run the motor smoothly
  stepper.runSpeed();

  // solenoid control
  if (digitalRead(BUTTON) == LOW) {
    digitalWrite(SOL1, HIGH);
    delay(PULSE_MS);
    digitalWrite(SOL1, LOW);
  }
}
