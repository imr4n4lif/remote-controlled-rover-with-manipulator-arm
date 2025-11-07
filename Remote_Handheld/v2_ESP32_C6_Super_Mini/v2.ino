#include "SimpleESPNOW.h"
#include <Wire.h>
#include <Adafruit_SSD1306.h>

SimpleESPNOW espnow;

// ---------------- Rover data ----------------
struct RoverData {
  uint8_t buttons[12];
  uint16_t pots[5];
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

RoverData roverData;

// ---------------- Serial ----------------
HardwareSerial nanoSerial(1);
#define RX_PIN 4
#define TX_PIN -1

// ---------------- OLED ----------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

int currentPage = 0;
const int totalPages = 3;

// ---------------- Function declarations ----------------
void displayPage(int page);
bool parseLine(const String &line);

void setup() {
  Serial.begin(115200);
  nanoSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  // Initialize OLED
  Wire.begin(6, 7); // SDA = 6, SCL = 7
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)){
    Serial.println("SSD1306 allocation failed");
    while(true);
  }
  display.clearDisplay();
  display.display();

  // Initialize ESP-NOW
  if(!espnow.begin()){
    Serial.println("ESP-NOW init failed");
    while(true);
  }

  // Receive raw rover data from ESP-NOW
  esp_now_register_recv_cb([](const esp_now_recv_info_t *info, const uint8_t *data, int len){
    if(len == sizeof(RoverData)){
      memcpy(&roverData, data, sizeof(RoverData));
      displayPage(currentPage);
    }
  });

  Serial.print("MAC: "); Serial.println(WiFi.macAddress());
}

// ---------------- Loop ----------------
void loop() {
  static String line = "";

  // Read Nano for button/pot values
  while(nanoSerial.available()){
    char c = nanoSerial.read();
    if(c=='\n' || c=='\r'){
      line.trim();
      if(line.length()>0){
        parseLine(line);
        espnow.sendMessage((uint8_t*)&roverData, sizeof(roverData));
        line="";
      }
    } else line+=c;
  }

  // Page switch buttons
  static bool lastNext = false;
  static bool lastPrev = false;

  bool btnNext = roverData.buttons[10];
  bool btnPrev = roverData.buttons[11];

  if(btnNext && !lastNext){
    currentPage = (currentPage+1) % totalPages;
    displayPage(currentPage);
  }
  if(btnPrev && !lastPrev){
    currentPage = (currentPage-1+totalPages) % totalPages;
    displayPage(currentPage);
  }

  lastNext = btnNext;
  lastPrev = btnPrev;
}

// ---------------- Display function ----------------
void displayPage(int page){
  display.clearDisplay();

  display.setTextColor(SSD1306_WHITE);
  
  switch(page){
    case 0: // Environment sensors
      display.setTextSize(1);
      display.setCursor(0,0);
      display.println("ENVIRONMENT");
      display.setTextSize(1);
      display.println("----------------");
      display.println("T: " + String(roverData.temperature) + " C");
      display.println("P: " + String(roverData.pressure) + " hPa");
      display.println("H: " + String(roverData.humidity) + " %");
      display.println("Alt: " + String(roverData.altitude) + " m");
      break;

    case 1: // GPS / distance / compass
      display.setTextSize(1);
      display.setCursor(0,0);
      display.println("NAVIGATION");
      display.setTextSize(1);
      display.println("----------------");
      display.println("Lat: " + String(roverData.latitude,6));
      display.println("Lng: " + String(roverData.longitude,6));
      display.println("Dist: " + String(roverData.distance_cm) + " cm");
      display.println("Comp: " + String(roverData.compass_azimuth) + " deg");
      break;

    case 2: // MPU
      display.setTextSize(1);
      display.setCursor(0,0);
      display.println("MPU6050");
      display.setTextSize(1);
      display.println("----------------");
      display.println("Temp: " + String(roverData.mpu_temp) + " C");
      display.println("Acc X: " + String(roverData.mpu_accX));
      display.println("Acc Y: " + String(roverData.mpu_accY));
      break;
  }

  display.display();
}


// ---------------- Nano line parser ----------------
bool parseLine(const String &line){
  int vals[17];
  int count = sscanf(line.c_str(),
                     "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
                     &vals[0], &vals[1], &vals[2], &vals[3], &vals[4], &vals[5],
                     &vals[6], &vals[7], &vals[8], &vals[9], &vals[10], &vals[11],
                     &vals[12], &vals[13], &vals[14], &vals[15], &vals[16]);
  if(count != 17) return false;

  for(int i=0;i<12;i++) roverData.buttons[i] = vals[i];
  for(int i=0;i<5;i++) roverData.pots[i] = vals[12+i];

  return true;
}
