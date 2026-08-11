#include <AccelStepper.h>
#include "DFRobotDFPlayerMini.h"
#include <SoftwareSerial.h>
#include <TM1637Display.h>
#include "Arduino.h"

/* ===== DFPlayer & 7-Segment Display Setup ===== */
#define CLK 12
#define DIO 13

TM1637Display display(CLK, DIO);

SoftwareSerial softSerial(8, 9);
DFRobotDFPlayerMini df;

/* ===== Motor Pins ===== */
const int DIR_PIN   = 2;
const int STEP_PIN  = 3;
const int LIMIT_PIN = 4;

const int SOLENOID_PIN = 7;
const bool SOLENOID_ACTIVE_HIGH = true;

/* ===== Laser & Sensor Pins ===== */
int ldrpin = 5;
int laserpin = 11;
int ldrpin2 = 6;
int laserpin2 = 10;

/* ===== Game Variables ===== */
int roundCount = 0;
int c1 = 0, c2 = 0;
bool resultPlayed = false;
bool wasLdr1High = false;
bool wasLdr2High = false;

unsigned long currentMillis;
unsigned long player1time = 0;
unsigned long player2time = 0;

const long SOFT_MAX = 1475;
const float MAX_SPEED = 1400;
const float ACCEL     = 5000;

const int   HOMING_DIR    = +1;
const long  BACKOFF_STEPS = 50;
const float HOMING_SPEED  = 600;
const float HOMING_ACC    = 2000;

const unsigned KICK_PULSE_MS    = 50;
const unsigned KICK_COOLDOWN_MS = 200;
bool kickActive = false;
unsigned long kickEndMs  = 0;
unsigned long lastKickMs = 0;

String rx;

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

inline bool limitPressed() {
  return digitalRead(LIMIT_PIN) == HIGH;
}

long clampSteps(long s) {
  if (s < 0) return 0;
  if (s > SOFT_MAX) return SOFT_MAX;
  return s;
}

void solenoidOn()  { digitalWrite(SOLENOID_PIN, SOLENOID_ACTIVE_HIGH ? HIGH : LOW); }
void solenoidOff() { digitalWrite(SOLENOID_PIN, SOLENOID_ACTIVE_HIGH ? LOW  : HIGH); }

void requestKick() {
  unsigned long now = millis();
  if (now - lastKickMs < KICK_COOLDOWN_MS) return;

  lastKickMs = now;
  kickActive = true;
  kickEndMs  = now + KICK_PULSE_MS;
  solenoidOn();
}

void doHoming() {
  stepper.setMaxSpeed(HOMING_SPEED);
  stepper.setAcceleration(HOMING_ACC);

  if (limitPressed()) {
    stepper.setSpeed(-HOMING_DIR * HOMING_SPEED);
    while (limitPressed()) stepper.runSpeed();
    delay(20);
  }

  stepper.setSpeed(HOMING_DIR * HOMING_SPEED);
  while (!limitPressed()) stepper.runSpeed();
  delay(20);

  stepper.setSpeed(-HOMING_DIR * (HOMING_SPEED / 2));
  while (limitPressed()) stepper.runSpeed();
  delay(20);

  stepper.move(-HOMING_DIR * BACKOFF_STEPS);
  stepper.runToPosition();
  stepper.setCurrentPosition(0);
}

void handleLine(String line) {
  line.trim();
  if (!line.length()) return;

  if (line.startsWith("MOVE")) {
    int sp = line.indexOf(' ');
    if (sp > 0) {
      long tgt = line.substring(sp + 1).toInt();
      tgt = clampSteps(tgt);
      long hw = -tgt;
      stepper.moveTo(hw);

      static unsigned long lastAckMs = 0;
      if (millis() - lastAckMs >= 100) {
        Serial.print("ACK MOVE "); Serial.println(tgt);
        lastAckMs = millis();
      }
    }
    return;
  }

  if (line == "KICK") {
    requestKick();
    return;
  }
}

void setup() {
  pinMode(LIMIT_PIN, INPUT_PULLUP);
  pinMode(SOLENOID_PIN, OUTPUT);
  solenoidOff();

  pinMode(ldrpin, INPUT);
  pinMode(laserpin, OUTPUT);
  digitalWrite(laserpin, HIGH);

  pinMode(ldrpin2, INPUT);
  pinMode(laserpin2, OUTPUT);
  digitalWrite(laserpin2, HIGH);

  softSerial.begin(9600);
  Serial.begin(115200);

  if (!df.begin(softSerial)) {
    Serial.println("DFPlayer Mini not found!");
    while (true);
  }

  df.volume(25);

  display.setBrightness(0x0f);
  display.showNumberDec(0, true);

  stepper.setMinPulseWidth(2);
  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(ACCEL);

  doHoming();
  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(ACCEL);

  df.play(5); // game start sound
  delay(6000);

  // wait until both sensors are LOW (no ball blocking the beam)
  while (digitalRead(ldrpin) == HIGH || digitalRead(ldrpin2) == HIGH) {
    delay(10);
  }
}

void loop() {
  stepper.run();

  if (kickActive && millis() >= kickEndMs) {
    solenoidOff();
    kickActive = false;
  }

  if (limitPressed()) {
    stepper.stop();
    stepper.runToPosition();

    stepper.setSpeed(-HOMING_DIR * HOMING_SPEED);
    while (limitPressed()) stepper.runSpeed();
    delay(20);

    stepper.move(-HOMING_DIR * BACKOFF_STEPS);
    stepper.runToPosition();
    stepper.setCurrentPosition(0);
    stepper.setMaxSpeed(MAX_SPEED);
    stepper.setAcceleration(ACCEL);
  }

  if (roundCount < 6) {
    int ldrstatus = digitalRead(ldrpin);
    int ldrstatus2 = digitalRead(ldrpin2);
    currentMillis = millis();

    // player 1
    if (ldrstatus == HIGH && !wasLdr1High && (currentMillis - player1time > 3000)) {
      wasLdr1High = true;
      player1time = currentMillis;
      df.play(2);
      delay(2000);
      c1++;
      roundCount++;
      display.showNumberDec(c1, true, 2, 0);
    } else if (ldrstatus == LOW) {
      wasLdr1High = false;
    }

    // player 2
    if (ldrstatus2 == HIGH && !wasLdr2High && (currentMillis - player2time > 3000)) {
      wasLdr2High = true;
      player2time = currentMillis;
      df.play(4);
      delay(2000);
      c2++;
      roundCount++;
      display.showNumberDec(c2, true, 2, 2);
    } else if (ldrstatus2 == LOW) {
      wasLdr2High = false;
    }

  } else {
    if (!resultPlayed) {
      if (c1 > c2) {
        df.play(1); // player 1 wins
        delay(500);
      } else {
        df.play(3); // player 2 wins or tie
        delay(500);
      }
      resultPlayed = true;
    }

    solenoidOff();
    delay(20000);
    display.showNumberDec(0, true);
    roundCount = 0;
    c1 = 0;
    c2 = 0;
    resultPlayed = false;

    df.play(5);
    delay(6000);

    // make sure sensors are inactive before restarting
    while (digitalRead(ldrpin) == HIGH || digitalRead(ldrpin2) == HIGH) {
      delay(10);
    }

    // reset timers
    player1time = millis();
    player2time = millis();
    wasLdr1High = false;
    wasLdr2High = false;
  }

  // Serial input from Raspberry Pi
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (rx.length()) {
        handleLine(rx);
        rx = "";
      }
    } else {
      rx += ch;
      if (rx.length() > 80) rx = "";
    }
  }
}
