
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_BME280.h>
#include <TinyGPS++.h>
#include <NewPing.h>
#include <QMC5883LCompass.h>
#include <MPU6050_light.h>

// ---------------- Rover data ----------------
struct RoverData {
  uint8_t buttons[12];       // Control buttons
  uint16_t pots[5];          // Potentiometers

  // Sensor data
  float temperature;
  float pressure;
  float humidity;
  float altitude;
  double latitude;
  double longitude;
  float distance_cm;
  int compass_azimuth;
  float mpu_temp;
  float mpu_accX;
  float mpu_accY;
};

volatile RoverData roverState;

// ---------------- Pins ----------------
const int IN1=2, IN2=3, ENA=4, IN3=5, IN4=6, ENB=7;
const int MOTOR_SPEED=255;
const int PIN5_TOGGLE=8;
bool pin5State=true;

// Servos
#define MIN_PULSE 150
#define MAX_PULSE 600
#define SERVO_CHANNEL_START 2
#define GRIPPER_CHANNEL 7
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Camera servos
#define CAM_SERVO1 0
#define CAM_SERVO2 1
int camServo1Pos=90;
int camServo2Pos=90;

// ---------------- Sensors ----------------
Adafruit_BME280 bme;
#define SEALEVEL_PRESSURE_HPA 1013.25

TinyGPSPlus gps;
HardwareSerial GPSSerial(1);

#define TRIG_PIN 9
#define ECHO_PIN 10
NewPing sonar(TRIG_PIN, ECHO_PIN, 200);

QMC5883LCompass compass;
MPU6050 mpu(Wire);

// ---------------- ESP-NOW ----------------
uint8_t broadcastAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// Callback for incoming control data
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len){
  if(len!=sizeof(RoverData)) return;
  memcpy((void*)&roverState, data, sizeof(RoverData));
}

// ---------------- Setup ----------------
void setup(){
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  if(esp_now_init()!=ESP_OK){
    Serial.println("ESP-NOW init failed!");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  // Add broadcast peer
  esp_now_peer_info_t peerInfo{};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if(esp_now_add_peer(&peerInfo)!=ESP_OK){
    Serial.println("Failed to add broadcast peer");
  }

  // Motor pins
  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT); pinMode(ENA,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT); pinMode(ENB,OUTPUT);
  analogWrite(ENA,0); analogWrite(ENB,0);

  pinMode(PIN5_TOGGLE, OUTPUT);
  digitalWrite(PIN5_TOGGLE,HIGH);

  // Servo init
  Wire.begin(16,17);
  pwm.begin();
  pwm.setPWMFreq(50);

  // Sensors init
  if(!bme.begin(0x76,&Wire)) Serial.println("BME280 not found");
  GPSSerial.begin(9600, SERIAL_8N1, 11, 12);
  compass.init();
  mpu.begin();
  mpu.calcOffsets(true,false);
}

// ---------------- Loop ----------------
void loop(){
  // --- Motor control ---
  if(roverState.buttons[0]) moveForward();
  else if(roverState.buttons[1]) turnLeft();
  else if(roverState.buttons[2]) turnRight();
  else if(roverState.buttons[3]) moveBackward();
  else stopMotors();

  // Servo control channels 2-6 from pots
  for(int i=0;i<5;i++){
    int angle=map(roverState.pots[i],0,1023,0,180);
    if(i==3) angle=180-angle;
    setServoAngle(SERVO_CHANNEL_START+i,angle);
  }

  // Button 5 toggle
  static bool lastButton5=false;
  if(roverState.buttons[4] && !lastButton5){
    pin5State=!pin5State;
    digitalWrite(PIN5_TOGGLE,pin5State?HIGH:LOW);
  }
  lastButton5=roverState.buttons[4];

  // Button 6 gripper
  static bool lastButton6=false;
  static bool gripperState=false;
  if(roverState.buttons[5] && !lastButton6){
    gripperState=!gripperState;
    setServoAngle(GRIPPER_CHANNEL,gripperState?140:80);
  }
  lastButton6=roverState.buttons[5];

  // Camera servos
  if(roverState.buttons[6]) camServo1Pos=constrain(camServo1Pos-1,0,180);
  if(roverState.buttons[7]) camServo2Pos=constrain(camServo2Pos+1,0,180);
  if(roverState.buttons[8]) camServo2Pos=constrain(camServo2Pos-1,0,180);
  if(roverState.buttons[9]) camServo1Pos=constrain(camServo1Pos+1,0,180);
  setServoAngle(CAM_SERVO1,camServo1Pos);
  setServoAngle(CAM_SERVO2,camServo2Pos);

  // --- Read sensors ---
  roverState.temperature = bme.readTemperature();
  roverState.pressure    = bme.readPressure()/100.0F;
  roverState.humidity    = bme.readHumidity();
  roverState.altitude    = bme.readAltitude(SEALEVEL_PRESSURE_HPA);

  while(GPSSerial.available()>0) gps.encode(GPSSerial.read());
  if(gps.location.isValid()){
    roverState.latitude=gps.location.lat();
    roverState.longitude=gps.location.lng();
  }

  roverState.distance_cm=sonar.ping_cm();

  compass.read();
  int az=compass.getAzimuth();
  if(az<0) az+=360;
  roverState.compass_azimuth=az;

  mpu.update();
  roverState.mpu_temp=mpu.getTemp();
  roverState.mpu_accX=mpu.getAccAngleX();
  roverState.mpu_accY=mpu.getAccAngleY();

  // --- Send sensor data to all receivers ---
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&roverState, sizeof(roverState));
  if(result != ESP_OK) Serial.println("ESP-NOW send failed");

  delay(100);
}

// ---- Motor functions ----
void moveForward(){digitalWrite(IN1,LOW);digitalWrite(IN2,HIGH);analogWrite(ENA,MOTOR_SPEED);digitalWrite(IN3,LOW);digitalWrite(IN4,HIGH);analogWrite(ENB,MOTOR_SPEED);}
void moveBackward(){digitalWrite(IN1,HIGH);digitalWrite(IN2,LOW);analogWrite(ENA,MOTOR_SPEED);digitalWrite(IN3,HIGH);digitalWrite(IN4,LOW);analogWrite(ENB,MOTOR_SPEED);}
void turnLeft(){digitalWrite(IN1,HIGH);digitalWrite(IN2,LOW);analogWrite(ENA,MOTOR_SPEED);digitalWrite(IN3,LOW);digitalWrite(IN4,HIGH);analogWrite(ENB,MOTOR_SPEED);}
void turnRight(){digitalWrite(IN1,LOW);digitalWrite(IN2,HIGH);analogWrite(ENA,MOTOR_SPEED);digitalWrite(IN3,HIGH);digitalWrite(IN4,LOW);analogWrite(ENB,MOTOR_SPEED);}
void stopMotors(){analogWrite(ENA,0);analogWrite(ENB,0);digitalWrite(IN1,LOW);digitalWrite(IN2,LOW);digitalWrite(IN3,LOW);digitalWrite(IN4,LOW);}
void setServoAngle(uint8_t channel,int angle){angle=constrain(angle,0,180);pwm.setPWM(channel,0,map(angle,0,180,MIN_PULSE,MAX_PULSE));}