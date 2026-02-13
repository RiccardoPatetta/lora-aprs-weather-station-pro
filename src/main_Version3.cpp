#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>

#include <bsec2.h>
#include "bsec_config.h"

#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AHTX0.h>

#include "config.h"
#include "website.h"
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ================= GLOBALI SENSORI =================

Bsec2 bsec;
Adafruit_BME280 bme280;
Adafruit_BMP280 bmp280;
Adafruit_AHTX0 aht;

bsec_virtual_sensor_t bsecVirtualSensorList[] = {
  BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
  BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  BSEC_OUTPUT_RAW_PRESSURE,
  BSEC_OUTPUT_IAQ,
  BSEC_OUTPUT_CO2_EQUIVALENT,
  BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
  BSEC_OUTPUT_RAW_GAS
};

// ================= DISPLAY MANAGEMENT =================

#define DISPLAY_BOOT_TIME 30000    // 30 sec at startup
#define DISPLAY_APRS_TIME 30000    // 30 sec after APRS send
#define DISPLAY_DIGI_TIME 10000    // 10 sec after digi received

bool displayOn = false;
unsigned long displayStartTime = 0;
bool bootScreenActive = true;

enum DisplayReason {
  DISPLAY_BOOT,
  DISPLAY_APRS,
  DISPLAY_DIGI,
  DISPLAY_OFF
};

DisplayReason currentDisplayReason = DISPLAY_BOOT;

void displayWake(DisplayReason reason) {
  display.ssd1306_command(SSD1306_DISPLAYON);
  displayOn = true;
  displayStartTime = millis();
  currentDisplayReason = reason;
}

void displaySleep() {
  display.ssd1306_command(SSD1306_DISPLAYOFF);
  displayOn = false;
  currentDisplayReason = DISPLAY_OFF;
}

void updateDisplayTimeout() {
  if (!displayOn) return;

  unsigned long elapsed = millis() - displayStartTime;
  unsigned long timeout = DISPLAY_BOOT_TIME;

  switch (currentDisplayReason) {
    case DISPLAY_BOOT:
      timeout = DISPLAY_BOOT_TIME;
      break;
    case DISPLAY_APRS:
      timeout = DISPLAY_APRS_TIME;
      break;
    case DISPLAY_DIGI:
      timeout = DISPLAY_DIGI_TIME;
      break;
    default:
      return;
  }

  if (elapsed > timeout) {
    displaySleep();
  }
}

// ================= DIGIPEATER =================

#define DIGI_ENABLED true
#define DIGI_WIDE1 true
#define DIGI_WIDE2 true
#define DIGI_HOLD_TIME 30000        // anti-loop 30 sec
#define DIGI_DUP_WINDOW 20000       // anti-duplicate same station

unsigned long lastDigiTime = 0;
String lastDigiPacket = "";
String lastDigiSource = "";
unsigned long lastDigiSourceTime = 0;

// ================= DATI =================

float T=0,H=0,P=0;
float IAQ=0,CO2=0,VOC=0,GAS=0;
float VBAT=0;

// ================= DATI TELEMETRIA APRS =================

float windDir = 0;      // 0-360 gradi
float windSpeed = 0;    // km/h media
float windGust = 0;     // km/h raffica

unsigned long lastSensorTime=0;
unsigned long lastAprsTime=0;

// ================= APRS TELEMETRY COUNTER =================

unsigned int aprsPacketCounter = 0;

bool wifiActive=false;
unsigned long wifiStartTime=0;

// ================= WEB =================

AsyncWebServer server(WEBSERVER_PORT);
Preferences prefs;

// ================= CONFIG STRUCT =================

struct Config {
  String callsign;
  int ssid;
  float latitude;
  float longitude;
  unsigned long aprsIntervalMs;
  float variationThreshold;
  bool beaconEnabled;
  String wifiSsid;
  String wifiPass;
  unsigned long wifiTimeoutMs;
  bool wifiAlwaysOn;
};

Config config;

// ================= SENSOR TYPE =================

enum SensorType {
  SENSOR_NONE,
  SENSOR_BME680,
  SENSOR_BME280,
  SENSOR_BMP_AHT
};

SensorType activeSensor = SENSOR_NONE;

// ================= LOAD CONFIG =================

void loadConfig() {

  prefs.begin("config", true);

  config.callsign = prefs.getString("callsign", DEFAULT_CALLSIGN);
  config.ssid = prefs.getInt("ssid", DEFAULT_SSID_M);
  config.latitude = prefs.getFloat("lat", DEFAULT_LAT);
  config.longitude = prefs.getFloat("lon", DEFAULT_LON);
  config.aprsIntervalMs = prefs.getULong("aprsInt", APRS_INTERVAL_MS);
  config.variationThreshold = prefs.getFloat("varTh", VARIATION_THRESHOLD);
  config.beaconEnabled = prefs.getBool("beacon", DEFAULT_BEACON_ON);
  config.wifiSsid = prefs.getString("wifiSsid", DEFAULT_WIFI_SSID);
  config.wifiPass = prefs.getString("wifiPass", DEFAULT_WIFI_PWD);
  config.wifiTimeoutMs = prefs.getULong("wifiTout", 120000UL);
  config.wifiAlwaysOn = prefs.getBool("wifiAlways", false);

  prefs.end();
}

// ================= BATTERIA =================

float readBattery() {
  uint16_t raw = analogRead(VBAT_ADC_PIN);
  return (raw / 4095.0f) * 3.3f * VBAT_DIVIDER;
}

// ================= SENSOR DETECT =================

void detectSensor() {

  Wire.begin();

  if (bsec.begin(BME68X_I2C_ADDR_HIGH, Wire) ||
      bsec.begin(BME68X_I2C_ADDR_LOW, Wire)) {

    if (bsec.status == BSEC_OK) {

      activeSensor = SENSOR_BME680;

      bsec.setConfig(bsec_config_iaq);
      bsec.updateSubscription(
        bsecVirtualSensorList,
        sizeof(bsecVirtualSensorList) / sizeof(bsec_virtual_sensor_t),
        BSEC_SAMPLE_RATE_LP
      );

      return;
    }
  }

  if (bme280.begin(0x76) || bme280.begin(0x77)) {
    activeSensor = SENSOR_BME280;
    return;
  }

  if (bmp280.begin(0x76) || bmp280.begin(0x77)) {
    if (aht.begin()) {
      activeSensor = SENSOR_BMP_AHT;
      return;
    }
  }

  activeSensor = SENSOR_NONE;
}

// ================= READ SENSOR =================

void readSensors() {

  VBAT = readBattery();

  switch (activeSensor) {

    case SENSOR_BME680:

  if (bsec.run()) {

    const bsecOutputs* outputs = bsec.getOutputs();

    for (uint8_t i = 0; i < outputs->nOutputs; i++) {

      switch (outputs->output[i].sensor_id) {

        case BSEC_OUTPUT_RAW_TEMPERATURE:
          T = outputs->output[i].signal;
          break;

        case BSEC_OUTPUT_RAW_HUMIDITY:
          H = outputs->output[i].signal;
          break;

        case BSEC_OUTPUT_RAW_PRESSURE:
          P = outputs->output[i].signal / 100.0;
          break;

        case BSEC_OUTPUT_IAQ:
          IAQ = outputs->output[i].signal;
          break;

        case BSEC_OUTPUT_CO2_EQUIVALENT:
          CO2 = outputs->output[i].signal;
          break;

        case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
          VOC = outputs->output[i].signal;
          break;

        case BSEC_OUTPUT_RAW_GAS:
          GAS = outputs->output[i].signal;
          break;
      }
    }
  }
  break;


    case SENSOR_BME280:
      T = bme280.readTemperature();
      H = bme280.readHumidity();
      P = bme280.readPressure() / 100.0;
      IAQ = CO2 = VOC = GAS = 0;
      break;

    case SENSOR_BMP_AHT:
      T = bmp280.readTemperature();
      P = bmp280.readPressure() / 100.0;
      sensors_event_t humidity, temp;
      aht.getEvent(&humidity, &temp);
      H = humidity.relative_humidity;
      IAQ = CO2 = VOC = GAS = 0;
      break;

    default:
      T = H = P = IAQ = CO2 = VOC = GAS = 0;
      break;
  }
}

// ================= LORA =================

void initLoRa() {

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(433775000)) {
    Serial.println("LoRa FAIL");
    while (1);
  }

  // Parametri rete LoRa APRS OK2DDS
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.setSyncWord(0x12);
  LoRa.enableCrc();
  LoRa.setTxPower(17);

  Serial.println("LoRa APRS 433.775 READY");
}

// ================= APRS BEACON =================

String buildAPRS() {

  char buffer[300];

  float tempF = (T * 9.0 / 5.0) + 32.0;

  int latDeg = abs((int)config.latitude);
  float latMin = (abs(config.latitude) - latDeg) * 60.0;

  int lonDeg = abs((int)config.longitude);
  float lonMin = (abs(config.longitude) - lonDeg) * 60.0;

  char latDir = config.latitude >= 0 ? 'N' : 'S';
  char lonDir = config.longitude >= 0 ? 'E' : 'W';

  int windDirInt = (int)windDir;
  int windSpeedMph = (int)(windSpeed * 0.621371);
  int windGustMph = (int)(windGust * 0.621371);
  int tempFInt = (int)tempF;
  int humidityInt = (int)H;
  int pressureInt = (int)(P * 10);

  sprintf(buffer,
    "%s-%d>APLHM0:!%02d%05.2f%c/%03d%05.2f%c_%03d/%03dg%03dt%03d"
    "h%02db%05d",
    config.callsign.c_str(),
    config.ssid,
    latDeg, latMin, latDir,
    lonDeg, lonMin, lonDir,
    windDirInt,
    windSpeedMph,
    windGustMph,
    tempFInt,
    humidityInt,
    pressureInt
  );

  return String(buffer);
}

// ================= APRS TELEMETRY =================

String buildAPRSTelemetry() {

  char buffer[350];

  int latDeg = abs((int)config.latitude);
  float latMin = (abs(config.latitude) - latDeg) * 60.0;

  int lonDeg = abs((int)config.longitude);
  float lonMin = (abs(config.longitude) - lonDeg) * 60.0;

  char latDir = config.latitude >= 0 ? 'N' : 'S';
  char lonDir = config.longitude >= 0 ? 'E' : 'W';

  // Normalizzazione valori telemetrici a range 0-255
  int telA1 = constrain((int)(IAQ / 500.0 * 255.0), 0, 255);
  int telA2 = constrain((int)(CO2 / 5000.0 * 255.0), 0, 255);
  int telA3 = constrain((int)(VOC / 500.0 * 255.0), 0, 255);
  int telA4 = constrain((int)(GAS / 65535.0 * 255.0), 0, 255);
  int telA5 = constrain((int)(VBAT / 30.0 * 255.0), 0, 255);

  sprintf(buffer,
    "%s-%d>APLHM1:!%02d%05.2f%c/%03d%05.2f%c>%03d,%03d,%03d,%03d,%03d,00000000"
    " IAQ=%.0f CO2=%.0f VOC=%.2f GAS=%.0f VBAT=%.2fV",
    config.callsign.c_str(),
    config.ssid,
    latDeg, latMin, latDir,
    lonDeg, lonMin, lonDir,
    telA1, telA2, telA3, telA4, telA5,
    IAQ, CO2, VOC, GAS, VBAT
  );

  return String(buffer);
}

void sendAPRS() {
  
  if(!config.beaconEnabled) return;

  aprsPacketCounter++;

  String pkt;

  // Ogni 2 pacchetti, invia telemetria invece di beacon standard
  if (aprsPacketCounter % 2 == 0) {
    pkt = buildAPRSTelemetry();
    Serial.println("TX APRS TELEMETRY");
  } else {
    pkt = buildAPRS();
    Serial.println("TX APRS BEACON");
  }

  LoRa.beginPacket();
  LoRa.write('<');
  LoRa.write(0xFF);
  LoRa.write(0x01);
  LoRa.print(pkt);
  LoRa.endPacket();

  lastAprsTime = millis();
  
  // Accende display per mostrare invio
  displayWake(DISPLAY_APRS);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);

  if (aprsPacketCounter % 2 == 0) {
    display.println("APRS TELEMETRY");
    display.println("");
    display.printf("IAQ: %.0f\n", IAQ);
    display.printf("CO2: %.0f ppm\n", CO2);
    display.printf("VOC: %.2f\n", VOC);
    display.printf("GAS: %.0f\n", GAS);
    display.printf("VBAT: %.2fV\n", VBAT);
  } else {
    display.println("APRS BEACON");
    display.println("");
    display.printf("T: %.1f C\n", T);
    display.printf("H: %.1f %%\n", H);
    display.printf("P: %.1f hPa\n", P);
    display.printf("Wind: %.1f km/h\n", windSpeed);
  }

  display.display();
  
  Serial.println("APRS: " + pkt);
}

// ================= WEB =================

void initWeb() {

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r) {
    r->send(200, "text/html", pageIndex);
  });

  server.on("/api", HTTP_GET, [](AsyncWebServerRequest *r) {

    DynamicJsonDocument doc(512);

    doc["t"]=T;
    doc["h"]=H;
    doc["p"]=P;
    doc["v"]=VBAT;
    doc["iaq"]=IAQ;
    doc["co2"]=CO2;
    doc["voc"]=VOC;
    doc["gas"]=GAS;
    doc["windDir"]=windDir;
    doc["windSpeed"]=windSpeed;
    doc["windGust"]=windGust;
    doc["rssi"]=LoRa.packetRssi();
    doc["uptime"]=millis()/1000;

    String json;
    serializeJson(doc,json);

    r->send(200,"application/json",json);
  });

server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request){

  DynamicJsonDocument doc(512);

  doc["callsign"] = config.callsign;
  doc["ssid"] = config.ssid;
  doc["latitude"] = config.latitude;
  doc["longitude"] = config.longitude;
  doc["aprsIntervalMs"] = config.aprsIntervalMs;
  doc["variationThreshold"] = config.variationThreshold;
  doc["beaconEnabled"] = config.beaconEnabled;
  doc["wifiSsid"] = config.wifiSsid;
  doc["wifiPass"] = config.wifiPass;
  doc["wifiTimeoutMs"] = config.wifiTimeoutMs;
  doc["wifiAlwaysOn"] = config.wifiAlwaysOn;

  String json;
  serializeJson(doc, json);

  request->send(200, "application/json", json);
});

server.on("/config", HTTP_POST,
  [](AsyncWebServerRequest *request){},
  NULL,
  [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){

    DynamicJsonDocument doc(1024);
    DeserializationError err = deserializeJson(doc, data);

    if(err) {
      request->send(400, "text/plain", "JSON Error");
      return;
    }

    config.callsign = doc["callsign"].as<String>();
    config.ssid = doc["ssid"];
    config.latitude = doc["latitude"];
    config.longitude = doc["longitude"];
    config.aprsIntervalMs = doc["aprsIntervalMs"];
    config.variationThreshold = doc["variationThreshold"];
    config.beaconEnabled = doc["beaconEnabled"];
    config.wifiSsid = doc["wifiSsid"].as<String>();
    config.wifiPass = doc["wifiPass"].as<String>();
    config.wifiTimeoutMs = doc["wifiTimeoutMs"];
    config.wifiAlwaysOn = doc["wifiAlwaysOn"];

    prefs.begin("config", false);

    prefs.putString("callsign", config.callsign);
    prefs.putInt("ssid", config.ssid);
    prefs.putFloat("lat", config.latitude);
    prefs.putFloat("lon", config.longitude);
    prefs.putULong("aprsInt", config.aprsIntervalMs);
    prefs.putFloat("varTh", config.variationThreshold);
    prefs.putBool("beacon", config.beaconEnabled);
    prefs.putString("wifiSsid", config.wifiSsid);
    prefs.putString("wifiPass", config.wifiPass);
    prefs.putULong("wifiTout", config.wifiTimeoutMs);
    prefs.putBool("wifiAlways", config.wifiAlwaysOn);

    prefs.end();

    request->send(200, "text/plain", "OK");
});

server.on("/reboot", HTTP_POST, [](AsyncWebServerRequest *request){
  request->send(200, "text/plain", "Rebooting");
  delay(500);
  ESP.restart();
});

  server.begin();
}

// ================= WIFI =================

void startWiFi() {

  WiFi.mode(WIFI_STA);
  WiFi.begin(config.wifiSsid.c_str(), config.wifiPass.c_str());

  unsigned long startAttempt = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 15000) {
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    initWeb();
    wifiActive=true;
    wifiStartTime=millis();
    Serial.println("WiFi connected");
  } else {
    WiFi.mode(WIFI_OFF);
    wifiActive=false;
    Serial.println("WiFi failed");
  }
}

// ================= SETUP =================

void setup() {

  Serial.begin(115200);

  loadConfig();
  initLoRa();
  detectSensor();
  readSensors();

  lastSensorTime = millis();
  lastAprsTime = millis();  // CORRETTO: Inizializza per evitare invio subito

  Wire.begin();

if(display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_I2C_ADDR)) {

  displayWake(DISPLAY_BOOT);  // CORRETTO: Passa il motivo

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("LoRa APRS");
  display.println("Weather Station");
  display.println("PRO");
  display.println("");

  display.print("Sensor: ");

  switch(activeSensor) {
    case SENSOR_BME680: display.println("BME680"); break;
    case SENSOR_BME280: display.println("BME280"); break;
    case SENSOR_BMP_AHT: display.println("BMP280+AHT20"); break;
    default: display.println("NONE"); break;
  }

  display.display();

} else {
  displayOn = false;
}

  startWiFi();
}

// ================= DIGIPEATER =================

void handleDigipeater() {

  int packetSize = LoRa.parsePacket();
  if (!packetSize) return;

  String rx = LoRa.readString();

  // Verifica header OK2DDS <FF01>
  if (rx.length() < 4) return;
  if (rx[0] != '<') return;

  rx = rx.substring(3);   // rimuove < FF 01
  rx.trim();

  Serial.println("RX CLEAN: " + rx);

  // Controlli minimi APRS
  if (rx.length() < 10) return;
  if (rx.indexOf('>') < 3) return;
  if (rx.indexOf(':') < rx.indexOf('>')) return;

  String source = rx.substring(0, rx.indexOf('>'));

  // Non ripetere se è nostro
  if (source.startsWith(config.callsign)) return;

  // Anti-duplicate stessa stazione
  if (source == lastDigiSource &&
      millis() - lastDigiSourceTime < DIGI_DUP_WINDOW) {
    Serial.println("Duplicate source blocked");
    return;
  }

  String header = rx.substring(0, rx.indexOf(':'));
  String payload = rx.substring(rx.indexOf(':') + 1);

  String path = header.substring(header.indexOf('>') + 1);

  if (path.indexOf(',') < 0) return;

  String dest = path.substring(0, path.indexOf(','));
  String digiPath = path.substring(path.indexOf(',') + 1);

  // Se già ripetuto
  if (digiPath.indexOf('*') != -1) return;

  bool shouldRepeat = false;

  // WIDE1
  if (DIGI_WIDE1 && digiPath.indexOf("WIDE1-1") >= 0) {
    digiPath.replace("WIDE1-1",
      config.callsign + "-" + String(config.ssid) + "*");
    shouldRepeat = true;
  }

  // WIDE2
  else if (DIGI_WIDE2 && digiPath.indexOf("WIDE2-1") >= 0) {
    digiPath.replace("WIDE2-1",
      config.callsign + "-" + String(config.ssid) + "*");
    shouldRepeat = true;
  }

  if (!shouldRepeat) return;

  // Anti-loop globale
  if (millis() - lastDigiTime < DIGI_HOLD_TIME &&
      rx == lastDigiPacket) {
    Serial.println("Loop blocked");
    return;
  }

  String newPacket =
    source + ">" + dest + "," + digiPath + ":" + payload;

  Serial.println("DIGI TX: " + newPacket);

  // Salva per anti-loop
  lastDigiPacket = rx;
  lastDigiTime = millis();
  lastDigiSource = source;
  lastDigiSourceTime = millis();

  // Accende display per mostrare ricezione digipeater
  displayWake(DISPLAY_DIGI);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("DIGI RX/TX");
  display.println(source);
  display.println("");
  display.println("Path: " + path);
  display.display();

  // Trasmissione compatibile OK2DDS
  LoRa.beginPacket();
  LoRa.write('<');
  LoRa.write(0xFF);
  LoRa.write(0x01);
  LoRa.print(newPacket);
  LoRa.endPacket();
}

// ================= MAIN LOOP =================

void loop() {

  // Lettura sensori
  if (millis() - lastSensorTime >= SENSOR_INTERVAL_MS) {
    lastSensorTime = millis();
    readSensors();
  }

  // Invio APRS
  if (millis() - lastAprsTime >= config.aprsIntervalMs) {
    sendAPRS();
  }

  // Gestione WiFi timeout
  if (!config.wifiAlwaysOn && wifiActive &&
      millis() - wifiStartTime > config.wifiTimeoutMs) {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    wifiActive = false;
    Serial.println("WiFi disconnected (timeout)");
  }

  // Gestione display timeout (centralizzato)
  updateDisplayTimeout();

  // Digipeater
  if (DIGI_ENABLED) {
    handleDigipeater();
  }

}