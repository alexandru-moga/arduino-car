#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Motor driver pins
#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 5

// Ultrasonic pins
#define FRONT_ECHO 6
#define FRONT_TRIG 7
#define BACK_ECHO 8
#define BACK_TRIG 9

// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Communication with ESP8266
#define BAUD_RATE 9600

// Variables for joystick and sensors
int joyX = 0, joyY = 0, joyBtn = 0;
long frontDist = 0, backDist = 0;

// Helper: measure distance
long measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 25000); // 25ms timeout
  long distance = duration * 0.034 / 2;
  return (distance == 0) ? 400 : distance; // If timeout, return max
}

// Helper: move motors
void move(int x, int y) {
  // Simple tank drive logic
  if (y > 100 && frontDist > 50) { // Forward
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    Serial.println("Moving Forward");
  } else if (y < -100 && backDist > 50) { // Backward
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    Serial.println("Moving Backward");
  } else if (x > 100) { // Right
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    Serial.println("Turning Right");
  } else if (x < -100) { // Left
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    Serial.println("Turning Left");
  } else { // Stop
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    Serial.println("Stopped");
  }
}

void setup() {
  Serial.begin(BAUD_RATE);

  // Motor pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // Ultrasonic pins
  pinMode(FRONT_TRIG, OUTPUT); pinMode(FRONT_ECHO, INPUT);
  pinMode(BACK_TRIG, OUTPUT); pinMode(BACK_ECHO, INPUT);

  // LCD
  lcd.init(); lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Initializing...");

  Serial.println("Setup complete");
}

void loop() {
  // Read distances
  frontDist = measureDistance(FRONT_TRIG, FRONT_ECHO);
  backDist = measureDistance(BACK_TRIG, BACK_ECHO);

  // Read joystick data from ESP8266 via Serial
  if (Serial.available() >= 7) { // Expecting: S,XH,XL,YH,YL,B,E
    if (Serial.read() == 'S') {
      int xh = Serial.read(); int xl = Serial.read();
      int yh = Serial.read(); int yl = Serial.read();
      joyBtn = Serial.read();
      if (Serial.read() == 'E') {
        joyX = (xh << 8) | xl;
        joyY = (yh << 8) | yl;
      }
    }
  }

  // Debug output
  Serial.print("Joystick: X="); Serial.print(joyX);
  Serial.print(" Y="); Serial.print(joyY);
  Serial.print(" Btn="); Serial.print(joyBtn);
  Serial.print(" | FrontDist="); Serial.print(frontDist);
  Serial.print(" BackDist="); Serial.println(backDist);

  // LCD update

sprintf(buffer, "X:%4d Y:%4d B:%d ", joyX, joyY, joyBtn);
lcd.print(buffer);

sprintf(buffer, "F:%3dcm B:%3dcm   ", frontDist, backDist);
lcd.print(buffer);

  // Motor control logic
  if (frontDist < 50 && joyY > 100) {
    move(0, 0); // Block forward
    Serial.println("Blocked: Front Obstacle");
  } else if (backDist < 50 && joyY < -100) {
    move(0, 0); // Block backward
    Serial.println("Blocked: Back Obstacle");
  } else {
    move(joyX, joyY);
  }

  delay(100);
}