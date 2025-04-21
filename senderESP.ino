#include <esp_now.h>
#include <WiFi.h>

#define joyYPin       35   
#define leftButtonPin 22
#define rightButtonPin 23

#define joyXPin  34
#define forwardButtonPin 18
#define backwardButtonPin 5


int joyYState;
int leftButtonState;
int rightButtonState;

int joyXState;
int forwardButtonState;
int backwardButtonState;

uint8_t broadcastAddress[] = {0x38, 0x18, 0x2b, 0xe8, 0x73, 0x88};

typedef struct struct_message {
  int joyY;      
  int leftButton;   
  int rightButton; 
  int joyX;
  int forwardButton;
  int backwardButton;
} struct_message;

struct_message controlData;
esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nSend Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
  Serial.begin(115200);

  pinMode(joyYPin, INPUT);
  pinMode(leftButtonPin, INPUT_PULLUP);
  pinMode(rightButtonPin, INPUT_PULLUP);
  pinMode(joyXPin, INPUT);
  pinMode(forwardButtonPin, INPUT_PULLUP);
  pinMode(backwardButtonPin, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {

  joyYState = analogRead(joyYPin);  

  leftButtonState = digitalRead(leftButtonPin);
  rightButtonState = digitalRead(rightButtonPin);

  joyXState = analogRead(joyXPin);
  forwardButtonState = digitalRead(forwardButtonPin);
  backwardButtonState = digitalRead(backwardButtonPin);

  controlData.joyY = joyYState;
  controlData.leftButton = leftButtonState;
  controlData.rightButton = rightButtonState;

  controlData.joyX = joyXState;
  controlData.backwardButton = backwardButtonState;
  controlData.forwardButton = forwardButtonState;

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &controlData, sizeof(controlData));

  if (result == ESP_OK) {
    Serial.println("Sent!");
  } else {
    Serial.println("Send failed");
  }

  delay(50);
}
