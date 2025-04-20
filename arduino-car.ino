// Motor control pins
#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 5

// Front IR sensors
#define IR_LEFT 6
#define IR_RIGHT 7

void setup() {
  // Configure motor pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Configure IR sensors as inputs
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  // Enable motors (if ENA/ENB are connected to PWM pins)
  // analogWrite(ENA, 255); // Uncomment if ENA/ENB are used
  // analogWrite(ENB, 255);
}

void loop() {
  int leftSensor = digitalRead(IR_LEFT);
  int rightSensor = digitalRead(IR_RIGHT);

  // Stop motors if either front sensor detects an obstacle
  if (leftSensor == LOW || rightSensor == LOW) { // Adjust HIGH/LOW based on sensor behavior
    stopMotors();
  } else {
    moveForward();
  }
}

void moveForward() {
  // Motor A forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  // Motor B forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopMotors() {
  // Stop both motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}