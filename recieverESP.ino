#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

#define enA 26
#define in1 16
#define in2 13
#define in3 27 
#define in4 14
#define enB 25

#define SERVO_PIN 18

Servo myservo;

const int motorSpeed = 255;


typedef struct struct_message {
  int joyY;
  int leftButton;
  int rightButton;
  int joyX;
  int forwardButton;
  int backwardButton;
  int pickButton;
  int releaseButton;
} struct_message;

struct_message incomingData;

void setup() {
  Serial.begin(115200);

  pinMode(in1, OUTPUT); 
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT); 
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT); 
  pinMode(enB, OUTPUT);

  // pinMode(R_S, INPUT);
  // pinMode(L_S, INPUT);

  myservo.attach(SERVO_PIN, 1000, 2000);  // wider range
  myservo.write(0);



  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {

}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  memcpy(&incomingData, data, sizeof(incomingData));
  // lastReceivedTime = millis();
  handleRemoteControl();
}

void handleRemoteControl() {
  int x = incomingData.joyX;
  int y = incomingData.joyY;
  int leftBtn = incomingData.leftButton;
  int rightBtn = incomingData.rightButton;
  int fwdBtn = incomingData.forwardButton;
  int backBtn = incomingData.backwardButton;
  int pickBtn = incomingData.pickButton;
  int releaseBtn = incomingData.releaseButton;

  if(pickBtn == 0){
    myservo.write(180);
    delay(10);
  };
  if(releaseBtn == 0){
    myservo.write(0);
    delay(10);

  };

  if (x > 3000 || leftBtn == 0) {
    turnLeft();
  } else if (x < 1000 || rightBtn == 0) {
    turnRight();
  } else if (y > 3000 || fwdBtn == 0) {
    moveForward();
  } else if (y < 1000 || backBtn == 0) {
    moveBackward();
  } else {
    stopMotors();
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


