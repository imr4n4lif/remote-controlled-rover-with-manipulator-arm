#include "SimpleESPNOW.h"

SimpleESPNOW espnow;

// Define structure for your data
struct RoverData {
  uint8_t buttons[12];  // 12 digital inputs
  uint16_t pots[5];     // 5 analog values (0-1023)
};

RoverData roverData;

HardwareSerial nanoSerial(1);
#define RX_PIN 4
#define TX_PIN -1

void setup() {
  Serial.begin(460800); // High-speed debug serial
  if (!espnow.begin()) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  Serial.println("Sender ready...");

  nanoSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); // Nano serial
}

// Parse a line from Nano: "0 1 0 ... 500"
bool parseLine(const String &line) {
  int vals[17];
  int count = sscanf(line.c_str(),
                     "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
                     &vals[0], &vals[1], &vals[2], &vals[3], &vals[4], &vals[5],
                     &vals[6], &vals[7], &vals[8], &vals[9], &vals[10], &vals[11],
                     &vals[12], &vals[13], &vals[14], &vals[15], &vals[16]);
  if (count != 17) return false;

  for (int i = 0; i < 12; i++) roverData.buttons[i] = vals[i];
  for (int i = 0; i < 5; i++)  roverData.pots[i] = vals[12 + i];
  return true;
}

void loop() {
  static String line = "";
  while (nanoSerial.available()) {
    char c = nanoSerial.read();
    if (c == '\n' || c == '\r') {
      line.trim(); // remove whitespace
      if (line.length() > 0) {
        if (parseLine(line)) {
          // Send full valid packet only
          espnow.sendMessage((uint8_t *)&roverData, sizeof(roverData));
          Serial.println("Sent binary packet");
        }
        line = ""; // reset for next line
      }
    } else {
      line += c; // accumulate characters
    }
  }
}
