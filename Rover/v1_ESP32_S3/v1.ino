#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Rover data structure
struct RoverData {
  uint8_t buttons[12];
  uint16_t pots[5];
};

volatile RoverData roverState;

// Motor pins
const int IN1 = 2, IN2 = 3, ENA = 4;
const int IN3 = 5, IN4 = 6, ENB = 7;

// Motor speed
const int MOTOR_SPEED = 255;

// Toggle pin for button 5
const int PIN5_TOGGLE = 8;
bool pin5State = false;

// Servo driver
#define MIN_PULSE 150  // Adjust if servos don't move
#define MAX_PULSE 600
#define SERVO_CHANNEL_START 2  // Channels 2,3,4,5,6 for pots
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Arm gripper servo
#define GRIPPER_CHANNEL 7
bool gripperState = false; // false = 0°, true = 120°

// Camera servo channels
#define CAM_SERVO1 0
#define CAM_SERVO2 1
int camServo1Pos = 90; // start middle
int camServo2Pos = 90;

// ---- Motor functions ----
void moveForward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(ENA, MOTOR_SPEED);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(ENB, MOTOR_SPEED);
}
void moveBackward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, MOTOR_SPEED);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, MOTOR_SPEED);
}
void turnLeft() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, MOTOR_SPEED);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(ENB, MOTOR_SPEED);
}
void turnRight() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(ENA, MOTOR_SPEED);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, MOTOR_SPEED);
}
void stopMotors() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// ---- Servo function ----
void setServoAngle(uint8_t channel, int angle) {
  angle = constrain(angle, 0, 180);
  uint16_t pulse = map(angle, 0, 180, MIN_PULSE, MAX_PULSE);
  pwm.setPWM(channel, 0, pulse);
}

// ---- ESP-NOW callback ----
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(RoverData)) return;
  memcpy((void*)&roverState, data, sizeof(RoverData));
}

void setup() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) return;
  esp_now_register_recv_cb(OnDataRecv);

  // Motor pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  stopMotors();

  // Toggle pin
  pinMode(PIN5_TOGGLE, OUTPUT);
  digitalWrite(PIN5_TOGGLE, LOW);

  // Servo init
  Wire.begin(16, 17); // SDA=16, SCL=17
  pwm.begin();
  pwm.setPWMFreq(50); // 50Hz for standard servos
}

void loop() {
  // Motor control
  if (roverState.buttons[0]) moveForward();
  else if (roverState.buttons[1]) turnLeft();
  else if (roverState.buttons[2]) turnRight();
  else if (roverState.buttons[3]) moveBackward();
  else stopMotors();

  // Servo control channels 2-6 from pots
  for (int i = 0; i < 5; i++) {
    int angle = map(roverState.pots[i], 0, 1023, 0, 180);

    // Reverse channel 5 (index 3)
    if (i == 3) angle = 180 - angle;

    setServoAngle(SERVO_CHANNEL_START + i, angle);
  }

  // ---- Button 5: toggle pin ----
  static bool lastButton5 = false;
  if (roverState.buttons[4] && !lastButton5) { // only toggle on rising edge
    pin5State = !pin5State;
    digitalWrite(PIN5_TOGGLE, pin5State ? HIGH : LOW);
  }
  lastButton5 = roverState.buttons[4];

  // ---- Button 6: gripper ----
  static bool lastButton6 = false;
  if (roverState.buttons[5] && !lastButton6) { // toggle on rising edge
    gripperState = !gripperState;
    setServoAngle(GRIPPER_CHANNEL, gripperState ? 140 : 80);
  }
  lastButton6 = roverState.buttons[5];

  // ---- Buttons 7-10: camera servo ----
  if (roverState.buttons[6]) camServo1Pos = constrain(camServo1Pos + 1, 0, 180);
  if (roverState.buttons[7]) camServo2Pos = constrain(camServo2Pos - 1, 0, 180);
  if (roverState.buttons[8]) camServo2Pos = constrain(camServo2Pos + 1, 0, 180);
  if (roverState.buttons[9]) camServo1Pos = constrain(camServo1Pos - 1, 0, 180);

  setServoAngle(CAM_SERVO1, camServo1Pos);
  setServoAngle(CAM_SERVO2, camServo2Pos);

  delay(20); // ~50Hz update
}
