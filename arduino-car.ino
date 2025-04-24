#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// I2C LCD setup
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Motor control pins
#define LEFT_MOTOR_IN1 14  // D5 (GPIO14)
#define LEFT_MOTOR_IN2 12  // D6 (GPIO12)
#define RIGHT_MOTOR_IN3 13 // D7 (GPIO13)
#define RIGHT_MOTOR_IN4 15 // D8 (GPIO15)

// Data structure
typedef struct control_message {
  int x;        // -100 to 100 (steering)
  int y;        // -100 to 100 (throttle)
  bool button;  // Button state
} control_message;

control_message controlData;

// Connection status variables
bool isConnected = false;
unsigned long lastReceivedTime = 0;
const unsigned long CONNECTION_TIMEOUT = 3000; // 3 seconds timeout
unsigned long packetCounter = 0;
unsigned long lastDebugTime = 0;
const int debugInterval = 2000; // Print debug info every 2 seconds

// WiFi channel setup for testing different channels
int currentChannel = 1;
unsigned long lastChannelSwitch = 0;
const unsigned long channelSwitchInterval = 5000; // 5 seconds between channel switches

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("\n\n----- ESP8266 CAR RECEIVER STARTING -----");
  Serial.println("Debugging ESP32 ↔ ESP8266 ESP-NOW connection");
  
  // Initialize I2C LCD
  Wire.begin(4, 5); // SDA=D2(GPIO4), SCL=D1(GPIO5)
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("ESP-NOW Debug");
  lcd.setCursor(0, 1); lcd.print("Waiting...");
  
  // Configure motor pins
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN3, OUTPUT);
  pinMode(RIGHT_MOTOR_IN4, OUTPUT);
  
  // Stop motors initially
  stopMotors();
  
  // Set device as WiFi Station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  // Set initial WiFi channel
  wifi_set_channel(currentChannel);
  
  // Print MAC and connection info
  Serial.print("ESP8266 Car MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Initial WiFi channel: ");
  Serial.println(currentChannel);
  Serial.println("Copy this MAC address into your transmitter code if needed!");
  
  // Initialize ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("ERROR: Failed to initialize ESP-NOW");
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("ESP-NOW Error!");
    lcd.setCursor(0, 1); lcd.print("Check serial log");
    
    Serial.println("Trying to restart ESP-NOW...");
    esp_now_deinit();
    delay(1000);
    
    if (esp_now_init() != 0) {
      Serial.println("ESP-NOW init failed again!");
      Serial.println("Restarting ESP8266...");
      delay(3000);
      ESP.restart();
      return;
    }
  }
  
  // Set ESP-NOW role
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  
  // Register callback function
  esp_now_register_recv_cb(OnDataRecv);
  
  Serial.println("ESP8266 Car initialized and listening for ESP-NOW messages");
  Serial.println("Will automatically try different WiFi channels if no connection");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Print debug info at regular intervals
  if (currentTime - lastDebugTime > debugInterval) {
    lastDebugTime = currentTime;
    
    Serial.print("Status: ");
    Serial.print(isConnected ? "CONNECTED" : "WAITING");
    Serial.print(" | Channel: ");
    Serial.print(currentChannel);
    Serial.print(" | Packets: ");
    Serial.println(packetCounter);
    
    if (!isConnected) {
      // Update LCD with current channel info
      lcd.clear();
      lcd.setCursor(0, 0); lcd.print("Waiting Ch: ");
      lcd.print(currentChannel);
      lcd.setCursor(0, 1); lcd.print("MAC on serial log");
    }
  }
  
  // Check connection status
  if (isConnected && (currentTime - lastReceivedTime > CONNECTION_TIMEOUT)) {
    isConnected = false;
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Disconnected!");
    lcd.setCursor(0, 1); lcd.print("Waiting Ch: ");
    lcd.print(currentChannel);
    
    Serial.println("CONNECTION LOST! Waiting for reconnection...");
    stopMotors();
  }
  
  // Auto-switch WiFi channels if not connected
  if (!isConnected && currentTime - lastChannelSwitch > channelSwitchInterval) {
    // Try the next channel (1-13)
    currentChannel = (currentChannel % 13) + 1;
    wifi_set_channel(currentChannel);
    
    lastChannelSwitch = currentTime;
    
    Serial.print("Switching to WiFi channel ");
    Serial.println(currentChannel);
    
    // Update LCD
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Trying channel:");
    lcd.setCursor(0, 1); lcd.print(currentChannel);
    
    // Reinitialize ESP-NOW when changing channels
    esp_now_deinit();
    delay(100);
    if (esp_now_init() == 0) {
      esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
      esp_now_register_recv_cb(OnDataRecv);
    }
  }
}

// Callback when data is received
void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  lastReceivedTime = millis();
  packetCounter++;
  
  // Display MAC of sender on first connection
  if (!isConnected) {
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
    Serial.print("CONNECTED! Receiving from: ");
    Serial.print(macStr);
    Serial.print(" on channel ");
    Serial.println(currentChannel);
    
    isConnected = true;
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Connected! Ch:");
    lcd.print(currentChannel);
    lcd.setCursor(0, 1); lcd.print("Recv packets:");
    lcd.print(packetCounter);
  }
  
  // Log packet details periodically (every 20 packets)
  if (packetCounter % 20 == 0) {
    Serial.print("Recv packet #");
    Serial.print(packetCounter);
    Serial.print(" | Ch: ");
    Serial.print(currentChannel);
    Serial.print(" | Len: ");
    Serial.println(len);
  }
  
  // Update LCD occasionally
  if (packetCounter % 10 == 0) {
    lcd.setCursor(0, 1);
    lcd.print("Recv packets:");
    lcd.print(packetCounter);
  }
  
  // Copy incoming data
  memcpy(&controlData, incomingData, sizeof(controlData));
  
  // Process joystick data
  controlCar(controlData.x, controlData.y);
}

void controlCar(int x, int y) {
  // Normalize to -1.0 ... 1.0
  float xNorm = constrain(x / 100.0, -1.0, 1.0);
  float yNorm = constrain(y / 100.0, -1.0, 1.0);
  
  // Deadzone for joystick
  const float DEADZONE = 0.07;
  if (abs(xNorm) < DEADZONE) xNorm = 0;
  if (abs(yNorm) < DEADZONE) yNorm = 0;

  // Linear mixing for smooth rotation
  float leftSpeed = yNorm + xNorm;
  float rightSpeed = yNorm - xNorm;
  
  // Normalize if exceeded limits
  float maxVal = max(abs(leftSpeed), abs(rightSpeed));
  if (maxVal > 1.0) {
    leftSpeed /= maxVal;
    rightSpeed /= maxVal;
  }
  
  // Map to PWM values (0-255)
  int leftPWM = abs(leftSpeed) * 255;
  int rightPWM = abs(rightSpeed) * 255;
  
  // Left motor direction
  if (leftSpeed > 0) {
    analogWrite(LEFT_MOTOR_IN1, leftPWM);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
  } else if (leftSpeed < 0) {
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    analogWrite(LEFT_MOTOR_IN2, leftPWM);
  } else {
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
  }
  
  // Right motor direction
  if (rightSpeed > 0) {
    analogWrite(RIGHT_MOTOR_IN3, rightPWM);
    digitalWrite(RIGHT_MOTOR_IN4, LOW);
  } else if (rightSpeed < 0) {
    digitalWrite(RIGHT_MOTOR_IN3, LOW);
    analogWrite(RIGHT_MOTOR_IN4, rightPWM);
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