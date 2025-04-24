#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// I2C LCD setup - address may need adjustment
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Motor control pins (4 pins only)
#define LEFT_MOTOR_IN1 5   // D1 (GPIO5)
#define LEFT_MOTOR_IN2 4   // D2 (GPIO4)
#define RIGHT_MOTOR_IN3 0  // D3 (GPIO0)
#define RIGHT_MOTOR_IN4 2  // D4 (GPIO2)

// Define the structure for receiving data
typedef struct control_message {
  int x;
  int y;
  bool button;
} control_message;

// Create an instance of the structure
control_message controlData;

// Flag for connection status
bool isConnected = false;
unsigned long lastReceivedTime = 0;
const unsigned long CONNECTION_TIMEOUT = 3000; // 3 seconds timeout

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C LCD
  Wire.begin(4, 5); // SDA=D2(GPIO4), SCL=D1(GPIO5)
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Waiting for");
  lcd.setCursor(0, 1);
  lcd.print("connection...");
  
  // Configure motor pins
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN3, OUTPUT);
  pinMode(RIGHT_MOTOR_IN4, OUTPUT);
  
  // Stop motors initially
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN3, LOW);
  digitalWrite(RIGHT_MOTOR_IN4, LOW);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Print MAC address for pairing
  Serial.print("Receiver MAC Address: ");
  Serial.println(WiFi.macAddress());

  // Initialize ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Set ESP-NOW Role
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  
  // Register callback function for incoming data
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // Check connection status
  if (millis() - lastReceivedTime > CONNECTION_TIMEOUT && isConnected) {
    isConnected = false;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Disconnected!");
    lcd.setCursor(0, 1);
    lcd.print("Waiting...");
    
    // Stop motors when connection lost
    stopMotors();
  }
}

// Callback when data is received
void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  lastReceivedTime = millis();
  
  if (!isConnected) {
    isConnected = true;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Connected!");
    lcd.setCursor(0, 1);
    lcd.print("Receiving data...");
  }
  
  // Copy incoming data to structure
  memcpy(&controlData, incomingData, sizeof(controlData));
  
  // Process the joystick data
  controlCar(controlData.x, controlData.y);
}

// Function to control car movement based on joystick input
void controlCar(int x, int y) {
  // PWM values for speed control
  int leftSpeed = y + x;   // Left = forward + turn
  int rightSpeed = y - x;  // Right = forward - turn
  
  // Constrain speeds to -100 to 100 range
  leftSpeed = constrain(leftSpeed, -100, 100);
  rightSpeed = constrain(rightSpeed, -100, 100);
  
  // Left motor direction control
  if (leftSpeed > 0) {
    analogWrite(LEFT_MOTOR_IN1, leftSpeed * 2.55); // Scale to 0-255
    digitalWrite(LEFT_MOTOR_IN2, LOW);
  } else if (leftSpeed < 0) {
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    analogWrite(LEFT_MOTOR_IN2, abs(leftSpeed) * 2.55);
  } else {
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
  }
  
  // Right motor direction control
  if (rightSpeed > 0) {
    analogWrite(RIGHT_MOTOR_IN3, rightSpeed * 2.55);
    digitalWrite(RIGHT_MOTOR_IN4, LOW);
  } else if (rightSpeed < 0) {
    digitalWrite(RIGHT_MOTOR_IN3, LOW);
    analogWrite(RIGHT_MOTOR_IN4, abs(rightSpeed) * 2.55);
  } else {
    digitalWrite(RIGHT_MOTOR_IN3, LOW);
    digitalWrite(RIGHT_MOTOR_IN4, LOW);
  }
}

void stopMotors() {
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN3, LOW);
  digitalWrite(RIGHT_MOTOR_IN4, LOW);
}
