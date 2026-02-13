// Managing Display Modes
#define DISPLAY_BOOT 0
#define DISPLAY_APRS 1
#define DISPLAY_DIGI 2

int displayMode;

// Telemetry Data
float IAQ;
float CO2;
float VOC;
float GAS;
float VBAT;

// Wind Information
float windDir;
float windSpeed;
float windGust;

int aprsPacketCounter = 0;
unsigned long lastAprsTime;

void setup() {
  // Initialize variables
  displayMode = DISPLAY_BOOT;
  lastAprsTime = millis();
}

void loop() {
  // Your logic for alternating beacons and telemetry
  if (aprsPacketCounter % 2 == 0) {
    sendBeacon(); // Send beacon packet
  } else {
    sendTelemetry(); // Send telemetry packet
  }
  aprsPacketCounter++;
  updateDisplayTimeout(); // Manage display timeout
}

void updateDisplayTimeout() {
  // Function to handle display timeout management
}

void sendBeacon() {
  // Logic to send APRS beacon
}

void sendTelemetry() {
  // Logic to send APRS telemetry
}