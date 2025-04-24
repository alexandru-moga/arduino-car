#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// LCD Setup (ESP32-compatible library)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Ultrasonic Sensors
#define FRONT_TRIG 13
#define FRONT_ECHO 12
#define BACK_TRIG 14
#define BACK_ECHO 27

// Motor Control Pins
#define LEFT_IN1 17
#define LEFT_IN2 16
#define LEFT_ENA 5
#define RIGHT_IN3 18
#define RIGHT_IN4 19
#define RIGHT_ENB 4

// PWM Settings
const int PWM_FREQ = 1000;
const int PWM_RES = 8;
const int LEFT_CH = 0;
const int RIGHT_CH = 1;

// Global Variables
bool dangerMode = false;
float frontDistance = 0;
float backDistance = 0;
float speedMultiplier = 0.5;  // 50% speed (0.0 to 1.0)

void setup() {
  Serial.begin(115200);
  
  // LCD Initialization
  lcd.init();
  lcd.backlight();
  lcd.clear();
  
  // Ultrasonic Sensors
  pinMode(FRONT_TRIG, OUTPUT);
  pinMode(FRONT_ECHO, INPUT);
  pinMode(BACK_TRIG, OUTPUT);
  pinMode(BACK_ECHO, INPUT);

  // Motor Control Setup
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN3, OUTPUT);
  pinMode(RIGHT_IN4, OUTPUT);
  
  ledcSetup(LEFT_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(LEFT_ENA, LEFT_CH);
  ledcSetup(RIGHT_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(RIGHT_ENB, RIGHT_CH);

  Dabble.begin("ESP32-Car");
}

float measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;
}

void updateLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);

  // Front distance display
  lcd.print("F:");
  if (frontDistance > 200) {
    lcd.print("max");
  } else if (frontDistance < 5) {
    lcd.print("min");
  } else {
    lcd.print(frontDistance, 0); // 0 decimals
    lcd.print("cm");
  }

  lcd.print(" B:");

  // Back distance display
  if (backDistance > 200) {
    lcd.print("max");
  } else if (backDistance < 5) {
    lcd.print("min");
  } else {
    lcd.print(backDistance, 0); // 0 decimals
    lcd.print("cm");
  }

  lcd.setCursor(0, 1);
  lcd.print("Mode: ");
  lcd.print(dangerMode ? "DANGER" : "SAFE  ");
}

void setMotor(int in1, int in2, int pwmChannel, int speed) {
  int pwm = map(abs(speed), 0, 100, 0, 255);
  
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(pwmChannel, pwm);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(pwmChannel, pwm);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(pwmChannel, 0);
  }
}

void loop() {
  Dabble.processInput();
  
  // Measure distances
  frontDistance = measureDistance(FRONT_TRIG, FRONT_ECHO);
  backDistance = measureDistance(BACK_TRIG, BACK_ECHO);
  
  // Update LCD every 500ms
  static unsigned long lastLCDUpdate = 0;
  if (millis() - lastLCDUpdate > 500) {
    updateLCD();
    lastLCDUpdate = millis();
  }

  // Handle Danger Mode Toggle
  if (GamePad.isCrossPressed()) dangerMode = true;
  if (GamePad.isCirclePressed()) dangerMode = false;

  if (Dabble.isAppConnected()) {
    int x = GamePad.getXaxisData();
    int y = -GamePad.getYaxisData();  // Invert Y-axis
    
    // Safety checks
    bool blockFront = (backDistance < 50) && !dangerMode;
    bool blockBack = (frontDistance < 50) && !dangerMode;
    bool blockRotation = (frontDistance < 25 || backDistance < 25) && !dangerMode;

    // Apply movement restrictions
    if (blockFront && y > 0) y = 0;
    if (blockBack && y < 0) y = 0;
    if (blockRotation && x != 0) x = 0;

    // Calculate motor speeds
int leftSpeed = constrain((y + x) * speedMultiplier, -100, 100);
int rightSpeed = constrain((y - x) * speedMultiplier, -100, 100);

    // Set motors
    setMotor(LEFT_IN1, LEFT_IN2, LEFT_CH, leftSpeed);
    setMotor(RIGHT_IN3, RIGHT_IN4, RIGHT_CH, rightSpeed);
  } else {
    setMotor(LEFT_IN1, LEFT_IN2, LEFT_CH, 0);
    setMotor(RIGHT_IN3, RIGHT_IN4, RIGHT_CH, 0);
  }
}
