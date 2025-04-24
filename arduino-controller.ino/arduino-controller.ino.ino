#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Joystick pins
#define JOYSTICK_X_PIN A0    // Analog pin
#define JOYSTICK_SW_PIN 13   // D7/GPIO13 for button

// I2C LCD setup
LiquidCrystal_I2C lcd(0x27, 16, 2);

// MAC Address of the receiver (replace with your receiver's MAC)
uint8_t receiverAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Define the structure for sending data
typedef struct control_message {
  int x;
  int y;
  bool button;
} control_message;

// Create an instance of the structure
control_message controlData;

// Variables for connection status
bool connected = false;
unsigned long lastSentTime = 0;

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C LCD
  Wire.begin(4, 5); // SDA=D2(GPIO4), SCL=D1(GPIO5)
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Controller Ready");
  lcd.setCursor(0, 1);
  lcd.print("Connecting...");
  
  // Configure pins
  pinMode(JOYSTICK_X_PIN, INPUT);
  pinMode(JOYSTICK_SW_PIN, INPUT_PULLUP);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Initialize ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Set ESP-NOW Role
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  
  // Register the send callback
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_add_peer(receiverAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  
  Serial.println("Transmitter initialized!");
}

void loop() {
  // Read joystick values
  int xRaw = analogRead(JOYSTICK_X_PIN);
  int y; // We'll simulate Y with X since ESP8266 only has one analog pin
  
  // Simulate Y-axis by using the X reading and some logic
  // If X is in middle range, use button presses to determine Y
  if (xRaw > 400 && xRaw < 600) {
    // Use button to determine if going forward or not
    y = digitalRead(JOYSTICK_SW_PIN) ? 0 : 100;
  } else {
    // Otherwise, no Y movement
    y = 0;
  }
  
  // Map analog X values to a suitable range for motor control
  int x = map(xRaw, 0, 1023, -100, 100);
  
  // Center deadzone for X axis
  if (x > -15 && x < 15) x = 0;
  
  // Update control data
  controlData.x = x;
  controlData.y = y;
  controlData.button = !digitalRead(JOYSTICK_SW_PIN); // Inverted because it's active LOW
  
  // Send data at regular intervals
  if (millis() - lastSentTime > 100) { // 10 times per second
    lastSentTime = millis();
    
    // Print values for debugging
    Serial.print("X: ");
    Serial.print(controlData.x);
    Serial.print(" | Y: ");
    Serial.print(controlData.y);
    Serial.print(" | Button: ");
    Serial.println(controlData.button ? "Pressed" : "Released");
    
    // Send message via ESP-NOW
    esp_now_send(receiverAddress, (uint8_t *) &controlData, sizeof(controlData));
  }
}

// Callback function when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  if (sendStatus == 0) {
    if (!connected) {
      connected = true;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Connected!");
      lcd.setCursor(0, 1);
      lcd.print("Controlling car...");
    }
  } else {
    if (connected) {
      connected = false;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Connection lost!");
      lcd.setCursor(0, 1);
      lcd.print("Retrying...");
    }
  }
}
