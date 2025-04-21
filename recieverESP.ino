#include <esp_now.h>
#include <WiFi.h>

#define enA 26
#define in1 12
#define in2 13
#define in3 27
#define in4 14
#define enB 25

const int motorSpeed = 200;

#define R_S 2
#define L_S 4

typedef struct struct_message {
  int joyY;        
  int leftButton;  
  int rightButton;
  int joyX;
  int forwardButton;
  int backwardButton;
} struct_message;

struct_message incomingData;

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingDataPtr, int len) {
  memcpy(&incomingData, incomingDataPtr, sizeof(incomingData));

  Serial.println("Data Received:");
  Serial.print("Joy Y: "); Serial.println(incomingData.joyY);
  Serial.print("Left Button: "); Serial.println(incomingData.leftButton);
  Serial.print("Right Button: "); Serial.println(incomingData.rightButton);
  Serial.print("Joy X: "); Serial.println(incomingData.joyX);
  Serial.print("Forward Button: "); Serial.println(incomingData.forwardButton);
  Serial.print("Backward Button: "); Serial.println(incomingData.backwardButton);
  Serial.println("-----------------------------");

  handleMovement();
}

void setup() {
  Serial.begin(115200);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
 
}

void handleMovement() {
  int joyY = incomingData.joyY;
  int leftBtn = incomingData.leftButton;
  int rightBtn = incomingData.rightButton;

  int joyX = incomingData.joyX;
  int forwardBtn = incomingData.forwardButton;
  int backwardBtn = incomingData.backwardButton;


  if (leftBtn == 0 || joyX > 3000) {
    turnLeft();
  } else if (rightBtn == 0 || joyX < 1000) {
    turnRight();
  } else {
    // Joystick-based movement (forward/backward)
    if (joyY > 3000 || forwardBtn == 0) {
      moveForward(); 
    } else if (joyY < 1000 || backwardBtn == 0) {
      moveBackward(); 
    } else {
      stopMotors(); 
    }
  }
}

void moveForward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
}

void moveBackward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
}

void turnLeft() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, 0);
  analogWrite(enB, motorSpeed);
}

void turnRight() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, motorSpeed);
  analogWrite(enB, 0);
}

void stopMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}
