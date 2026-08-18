# Crash Bash Bot – Human vs Robot

An interactive table game where a robot competes against a human, inspired by the PlayStation game *Crash Bash*. A top-down OAK-D camera streams video to a Raspberry Pi, which tracks the ball in real time, predicts its trajectory, and sends motion commands to an Arduino for fast, precise actuation.

Graduation Project – Computer Engineering Department, An-Najah National University.

## Team
- Aya Tammam
- Aseel Abdo

Supervised by Dr. Saed Tarapiah and Dr. Ashraf Armoush.

## How It Works

- An **OAK-D camera** connected to a **Raspberry Pi** tracks the ball's position in real time.
- The Raspberry Pi predicts the ball's trajectory and calculates the target position for the robot's carriage.
- The Raspberry Pi sends `MOVE` and `KICK` commands over serial to the **robot-side Arduino**, which drives a **NEMA-17 stepper motor** (via a DRV8825 driver) to position the carriage on linear rails, and fires a **12V solenoid** to strike the ball.
- A separate **joystick-side Arduino** lets the human player control their own carriage and solenoid manually using a joystick and push button.
- Goals are detected using **LDR + laser sensor pairs** on each side of the table, with score shown on a 7-segment display and game sounds played through a DFPlayer module.

## Repository Structure

```
crash-bash-bot/
├── raspberry-pi/
│   └── ball_prediction.py       # Ball tracking, trajectory prediction, and serial commands to the robot Arduino
├── arduino/
│   ├── robot_arduino/
│   │   └── robot_arduino.ino    # Robot side: motor, solenoid, goal sensors, sound, score display, serial commands from the Pi
│   └── joystick_arduino/
│       └── joystick_arduino.ino # Human side: joystick + push button control of motor and solenoid
└── README.md
```

## Hardware Components

- Raspberry Pi 4
- OAK-D camera
- 2x Arduino Uno
- NEMA-17 stepper motor + DRV8825 driver
- 12V solenoid (x2)
- LDR + laser module pairs (x2, one per goal)
- DFPlayer Mini + speaker
- TM1637 7-segment display
- Joystick + push button
- Limit switch (homing)
- Linear rail, GT2 belt and pulleys, aluminum frame

## Notes

- The robot-side Arduino is connected to the Raspberry Pi via USB for serial communication (`MOVE`, `KICK`).
- The joystick-side Arduino connects to the Raspberry Pi only for 5V power over USB — it operates independently and does not receive commands from the Pi.
- At startup, the robot-side Arduino performs a homing routine using the limit switch to set its zero position.

## Setup & Running

### 1 Raspberry Pi (Python)

Install the required Python libraries:

```bash
pip install -r requirements.txt
```

Then run the ball tracking script:

```bash
python3 raspberry-pi/ball_prediction.py
```

### 2 Arduino boards

Open each `.ino` file in the Arduino IDE and install the required libraries via **Tools → Manage Libraries**:

**For `robot_arduino.ino`:**
- AccelStepper
- DFRobotDFPlayerMini
- TM1637Display
- SoftwareSerial *(built into the Arduino IDE)*

**For `joystick_arduino.ino`:**
- AccelStepper

Upload each sketch to its corresponding Arduino board.

### 3 Run order

1. Power on both Arduino boards and let the robot-side Arduino complete homing.
2. Connect the robot-side Arduino to the Raspberry Pi via USB.
3. Run `ball_prediction.py` on the Raspberry Pi.
4. The joystick-side Arduino works independently once powered — no extra setup needed on the Pi side.
