#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// Joystick pins for ESP32
#define JOYSTICK_X_PIN 34  // ADC pin
#define JOYSTICK_Y_PIN 35  // ADC pin
#define JOYSTICK_SW_PIN 32 // Button pin

// Try broadcast address first, which often works better between ESP32 and ESP8266
uint8_t broadcastAddress[] = {0x48, 0x55, 0x19, 0xEC, 0x36, 0xE3};

// Specific MAC address of your ESP8266 receiver - use this if broadcast doesn't work
uint8_t receiverAddress[] = {0x48, 0x55, 0x19, 0xEC, 0x36, 0xE3};

// Use broadcast or specific address (true = use broadcast)
bool useBroadcast = true;

// Define control message structure
typedef struct control_message {
  int x;
  int y;
  bool button;
} control_message;

control_message controlData;

// Joystick calibration
int xCenter = 2048;
int yCenter = 2048;
const int deadZone = 150;

// Status tracking
bool connectionEstablished = false;
unsigned long lastSendTime = 0;
const int sendInterval = 50; // 50ms between sends
int failCounter = 0;
int successCounter = 0;

// Define the WiFi channels to try (1-13)
const int numChannels = 13;
int currentChannel = 1;
unsigned long lastChannelSwitch = 0;
const unsigned long channelSwitchInterval = 5000; // 5 seconds between channel switches

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  
  Serial.print("Sent to: "); 
  Serial.print(macStr);
  Serial.print(" on channel "); 
  Serial.print(currentChannel);
  Serial.print(" - Status: ");
  
  if (status == ESP_NOW_SEND_SUCCESS) {
    successCounter++;
    Serial.println("SUCCESS ✓");
    connectionEstablished = true;
  } else {
    failCounter++;
    Serial.println("FAILED ✗");
  }
  
  Serial.print("Success/Fail: ");
  Serial.print(successCounter);
  Serial.print("/");
  Serial.println(failCounter);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("\n\n----- ESP32 CONTROLLER STARTING -----");
  Serial.println("Debugging ESP32 ↔ ESP8266 ESP-NOW connection");
  
  // Configure pins
  pinMode(JOYSTICK_X_PIN, INPUT);
  pinMode(JOYSTICK_Y_PIN, INPUT);
  pinMode(JOYSTICK_SW_PIN, INPUT_PULLUP);
  
  // Set up WiFi in Station mode
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Set the channel
  esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
  
  // Print MAC address
  Serial.print("ESP32 Controller MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Using ");
  Serial.print(useBroadcast ? "broadcast" : "specific");
  Serial.println(" address for communication");
  
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: Failed to initialize ESP-NOW");
    Serial.println("Restarting controller...");
    delay(3000);
    ESP.restart();
    return;
  }

  // Register callback
  esp_now_register_send_cb(OnDataSent);
  
  // Add peer
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  if (useBroadcast) {
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  } else {
    memcpy(peerInfo.peer_addr, receiverAddress, 6);
  }
  peerInfo.channel = currentChannel;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ERROR: Failed to add peer");
    return;
  }
  
  // Calibrate joystick
  Serial.println("Calibrating joystick...");
  int xSum = 0, ySum = 0;
  for (int i = 0; i < 10; i++) {
    xSum += analogRead(JOYSTICK_X_PIN);
    ySum += analogRead(JOYSTICK_Y_PIN);
    delay(50);
  }
  xCenter = xSum / 10;
  yCenter = ySum / 10;
  
  Serial.print("X center: "); Serial.println(xCenter);
  Serial.print("Y center: "); Serial.println(yCenter);
  Serial.println("ESP32 Controller ready! Starting on channel 1");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Check if it's time to switch channel
  if (!connectionEstablished && currentTime - lastChannelSwitch > channelSwitchInterval) {
    // Try the next channel
    currentChannel = (currentChannel % numChannels) + 1;
    
    Serial.print("Switching to channel ");
    Serial.println(currentChannel);
    
    // Delete existing peer
    if (useBroadcast) {
      esp_now_del_peer(broadcastAddress);
    } else {
      esp_now_del_peer(receiverAddress);
    }
    
    // Set new channel
    esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
    
    // Add peer with new channel
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    if (useBroadcast) {
      memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    } else {
      memcpy(peerInfo.peer_addr, receiverAddress, 6);
    }
    peerInfo.channel = currentChannel;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
    
    lastChannelSwitch = currentTime;
  }
  
  // Read joystick values
  int xRaw = analogRead(JOYSTICK_X_PIN);
  int yRaw = analogRead(JOYSTICK_Y_PIN);
  bool buttonPressed = !digitalRead(JOYSTICK_SW_PIN);
  
  // Calculate offset from center
  int xOffset = xRaw - xCenter;
  int yOffset = yRaw - yCenter;
  
  // Apply deadzone
  if (abs(xOffset) < deadZone) xOffset = 0;
  if (abs(yOffset) < deadZone) yOffset = 0;
  
  // Map to -100 to 100 range
  int xMapped = map(xOffset, -2048, 2048, -100, 100);
  int yMapped = map(yOffset, -2048, 2048, -100, 100);
  
  // Update control data
  controlData.x = xMapped;
  controlData.y = yMapped;
  controlData.button = buttonPressed;
  
  // Send data at regular intervals
  if (currentTime - lastSendTime > sendInterval) {
    lastSendTime = currentTime;
    
    // Only print joystick values once per second to reduce serial output
    if (currentTime % 1000 < 100) {
      Serial.print("X: ");
      Serial.print(controlData.x);
      Serial.print("\tY: ");
      Serial.print(controlData.y);
      Serial.print("\tButton: ");
      Serial.println(buttonPressed ? "Pressed" : "Released");
    }
    
    // Send message via ESP-NOW
    uint8_t* destAddr = useBroadcast ? broadcastAddress : receiverAddress;
    
    esp_err_t result = esp_now_send(destAddr, (uint8_t *)&controlData, sizeof(controlData));
    
    if (result != ESP_OK) {
      Serial.print("ERROR: Send failed with code ");
      Serial.println(result);
    }
  }
}