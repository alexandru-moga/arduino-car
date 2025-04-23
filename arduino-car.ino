#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <NewPing.h>

// Motor pins
#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 5

// IR sensors
#define FRONT_LEFT_IR 6
#define FRONT_RIGHT_IR 7
#define BACK_LEFT_IR 8
#define BACK_RIGHT_IR 9

// Ultrasonic
#define TRIG_PIN 11
#define ECHO_PIN 10
#define MAX_DISTANCE 200
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Joystick pins
#define VRX_PIN A0  // X-axis (left-right)
#define VRY_PIN A1  // Y-axis (up-down)

// Custom character ●
byte dotChar[8] = {B00100,B01110,B11111,B11111,B11111,B01110,B00100,B00000};

void setup() {
  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // IR sensors
  pinMode(FRONT_LEFT_IR, INPUT);
  pinMode(FRONT_RIGHT_IR, INPUT);
  pinMode(BACK_LEFT_IR, INPUT);
  pinMode(BACK_RIGHT_IR, INPUT);

  // LCD setup
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, dotChar);
  lcd.clear();
}

void loop() {
  updateDisplay();
  handleJoystick();
}

void updateDisplay() {
  // Centered distance
  int distance = sonar.ping_cm();
  lcd.setCursor(3, 0);
  lcd.print(distance <= 0 ? "  --cm   " : String("  ") + distance + "cm   ");

  // Sensor indicators (● appears when sensor reads LOW)
  lcd.setCursor(0, 0);
  lcd.write(digitalRead(FRONT_LEFT_IR) ? ' ' : 0);
  lcd.setCursor(15, 0);
  lcd.write(digitalRead(FRONT_RIGHT_IR) ? ' ' : 0);
  lcd.setCursor(0, 1);
  lcd.write(digitalRead(BACK_LEFT_IR) ? ' ' : 0);
  lcd.setCursor(15, 1);
  lcd.write(digitalRead(BACK_RIGHT_IR) ? ' ' : 0);
}

void handleJoystick() {
  int yValue = analogRead(VRY_PIN);  // Up-Down
  int xValue = analogRead(VRX_PIN);  // Left-Right

  // Deadzone (center position)
  if (abs(yValue - 512) < 50 && abs(xValue - 512) < 50) {
    stopMotors();
    return;
  }

  // Forward/Backward control
  if (yValue < 400) {  // UP (Forward)
    if (canMoveForward()) {
      moveForward();
    } else {
      stopMotors();
    }
  } 
  else if (yValue > 600) {  // DOWN (Backward)
    if (canMoveBackward()) {
      moveBackward();
    } else {
      stopMotors();
    }
  }

  // Left/Right rotation
  if (xValue < 400) {  // LEFT
    rotateLeft();
  } 
  else if (xValue > 600) {  // RIGHT
    rotateRight();
  }
}

bool canMoveForward() {
  bool frontClear = digitalRead(FRONT_LEFT_IR) && digitalRead(FRONT_RIGHT_IR);
  int distance = sonar.ping_cm();
  return frontClear && (distance > 10 || distance == 0);
}

bool canMoveBackward() {
  return digitalRead(BACK_LEFT_IR) && digitalRead(BACK_RIGHT_IR);
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void rotateLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void rotateRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
