#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <LoRa.h>
#include <Preferences.h>
#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>

#include "bsec_config.h"
#include <bsec2.h>

#include <Adafruit_AHTX0.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_SSD1306.h>

#include "config.h"
#include "website.h"

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
    BSEC_OUTPUT_RAW_GAS};

// ================= DATI =================

float T = 0, H = 0, P = 0;
float IAQ = 0, CO2 = 0, VOC = 0, GAS = 0;
float VBAT = 0;

// ================= DATI TELEMETRIA APRS =================

float windDir = 0;   // 0-360 gradi
float windSpeed = 0; // km/h media
float windGust = 0;  // km/h raffica

unsigned long lastSensorTime = 0;
unsigned long lastAprsTime = 0;

// ================= APRS TELEMETRY COUNTER =================

unsigned int aprsPacketCounter = 0;

bool wifiActive = false;
unsigned long wifiStartTime = 0;

// ================= WEB =================

AsyncWebServer server(WEBSERVER_PORT);
Preferences prefs;

// ================= DISPLAY =================

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RST);
bool displayReady = false;

// Display state (rendered by updateDisplay in loop)
String dispLine1 = "";
String dispLine2 = "";
String dispLine3 = "";
String dispLine4 = "";
bool dispNeedsUpdate = false;

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

enum SensorType { SENSOR_NONE, SENSOR_BME680, SENSOR_BME280, SENSOR_BMP_AHT };

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
      bsec.updateSubscription(bsecVirtualSensorList,
                              sizeof(bsecVirtualSensorList) /
                                  sizeof(bsec_virtual_sensor_t),
                              BSEC_SAMPLE_RATE_LP);

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

  if (activeSensor == SENSOR_BME680) {
    // I dati vengono aggiornati direttamente nel loop tramite bsec.run()
    // Qui estraiamo solo i valori aggiornati nelle variabili globali
    const bsecOutputs *outputs = bsec.getOutputs();
    for (uint8_t i = 0; i < outputs->nOutputs; i++) {
      switch (outputs->output[i].sensor_id) {
      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
        T = outputs->output[i].signal;
        break;
      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
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
  } else if (activeSensor == SENSOR_BME280) {
    T = bme280.readTemperature();
    H = bme280.readHumidity();
    P = bme280.readPressure() / 100.0;
  } else if (activeSensor == SENSOR_BMP_AHT) {
    T = bmp280.readTemperature();
    P = bmp280.readPressure() / 100.0;
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);
    H = humidity.relative_humidity;
  }
}

// ================= DISPLAY FUNCTIONS =================

void initDisplay() {
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(50);
  digitalWrite(OLED_RST, HIGH);
  delay(50);

  if (display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_I2C_ADDR)) {
    displayReady = true;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.display();
  } else {
    displayReady = false;
    Serial.println("OLED init FAIL");
  }
}

void setDisplayContent(String l1, String l2, String l3, String l4) {
  dispLine1 = l1;
  dispLine2 = l2;
  dispLine3 = l3;
  dispLine4 = l4;
  dispNeedsUpdate = true;
}

void updateDisplay() {
  if (!displayReady || !dispNeedsUpdate)
    return;
  dispNeedsUpdate = false;

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(dispLine1);
  display.setCursor(0, 16);
  display.println(dispLine2);
  display.setCursor(0, 32);
  display.println(dispLine3);
  display.setCursor(0, 48);
  display.println(dispLine4);
  display.display();
}

void showSplash() {
  String sensorStr;
  switch (activeSensor) {
  case SENSOR_BME680:
    sensorStr = "BME680 OK";
    break;
  case SENSOR_BME280:
    sensorStr = "BME280 OK";
    break;
  case SENSOR_BMP_AHT:
    sensorStr = "BMP+AHT OK";
    break;
  default:
    sensorStr = "NO SENSOR";
    break;
  }

  setDisplayContent("LoRa APRS WX PRO",
                    "v" + String(RELEASE_VERSION) + "  " + String(BUILD_DATE),
                    "Sensor: " + sensorStr, "");
  updateDisplay();
}

// ================= LORA =================

void initLoRa() {

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(433775000)) {
    Serial.println("LoRa FAIL");
    while (1)
      ;
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
          "%s-%d>APZVDW:!%02d%05.2f%c/%03d%05.2f%c_%03d/%03dg%03dt%03d"
          "h%02db%05d",
          config.callsign.c_str(), config.ssid, latDeg, latMin, latDir, lonDeg,
          lonMin, lonDir, windDirInt, windSpeedMph, windGustMph, tempFInt,
          humidityInt, pressureInt);

  return String(buffer);
}

// ================= APRS TELEMETRY =================

String buildAPRSTelemetry() {
  char buffer[150];

  // Normalizzazione 0-255. Usiamo 5.0V come fondo scala per avere risoluzione.
  int telA1 = constrain((int)(IAQ / 500.0 * 255.0), 0, 255);
  int telA2 = constrain((int)(CO2 / 5000.0 * 255.0), 0, 255);
  int telA3 = constrain((int)(VOC / 500.0 * 255.0), 0, 255);
  int telA4 = constrain((int)(GAS / 65535.0 * 255.0), 0, 255);
  int telA5 = constrain((int)(VBAT / 5.0 * 255.0), 0, 255); // Calibrato su 5V

  // Formato Telemetria Standard: T#SEQ,VAL1,VAL2,VAL3,VAL4,VAL5,DIGITAL
  sprintf(buffer, "%s-%d>APZVDW:T#%03u,%03d,%03d,%03d,%03d,%03d,00000000",
          config.callsign.c_str(), config.ssid, aprsPacketCounter % 1000, telA1,
          telA2, telA3, telA4, telA5);

  return String(buffer);
}

// ================= DEFINIZIONI TELEMETRIA APRS =================

void sendLoRaRaw(String payload) {
  LoRa.beginPacket();
  LoRa.write('<');
  LoRa.write(0xFF);
  LoRa.write(0x01);
  LoRa.print(payload);
  LoRa.endPacket();
  Serial.println("TX RAW DEFINITION: " + payload);
}

void sendAPRSDefinitions() {
  char buffer[150];
  // Il destinatario dei messaggi di configurazione deve essere il proprio
  // callsign e il campo deve essere di 9 caratteri (standard APRS)
  String destCall = config.callsign + "-" + String(config.ssid);
  while (destCall.length() < 9)
    destCall += " ";

  // 1. Nomi dei parametri (PARM)
  sprintf(buffer, "%s-%d>APZVDW::%-9s:PARM.IAQ,CO2,VOC,GAS,Batt",
          config.callsign.c_str(), config.ssid, destCall.c_str());
  sendLoRaRaw(String(buffer));
  delay(1000); // Pausa per non saturare il gateway

  // 2. Unità di misura (UNIT)
  sprintf(buffer, "%s-%d>APZVDW::%-9s:UNIT.idx,ppm,mg/m,ohm,V",
          config.callsign.c_str(), config.ssid, destCall.c_str());
  sendLoRaRaw(String(buffer));
  delay(1000);

  // 3. Equazioni di conversione (EQNS)
  // Per la batteria (Param 5): Valore * 0.0196 = Volt
  sprintf(buffer, "%s-%d>APZVDW::%-9s:EQNS.0,2,0,0,1,0,0,1,0,0,1,0,0,0.0196,0",
          config.callsign.c_str(), config.ssid, destCall.c_str());
  sendLoRaRaw(String(buffer));
}

void sendAPRS() {

  if (!config.beaconEnabled)
    return;

  aprsPacketCounter++;

  String pkt;
  String txType;

  // Ogni 2 pacchetti, invia telemetria invece di beacon standard
  if (aprsPacketCounter % 2 == 0) {
    pkt = buildAPRSTelemetry();
    txType = "TELEMETRY";
    Serial.println("TX APRS TELEMETRY");
  } else {
    pkt = buildAPRS();
    txType = "BEACON";
    Serial.println("TX APRS BEACON");
  }

  LoRa.beginPacket();
  LoRa.write('<');
  LoRa.write(0xFF);
  LoRa.write(0x01);
  LoRa.print(pkt);
  int txResult = LoRa.endPacket();

  lastAprsTime = millis();

  String callsignFull = config.callsign + "-" + String(config.ssid);
  String statusStr =
      (txResult == 1) ? "TX " + txType + " OK" : "TX " + txType + " FAIL";
  String rssiStr = "RSSI: " + String(LoRa.packetRssi()) + " dBm";
  String infoStr =
      "Up:" + String(millis() / 1000) + "s " + String(VBAT, 1) + "V";

  setDisplayContent(callsignFull, statusStr, rssiStr, infoStr);

  Serial.println("APRS: " + pkt);
}

// ================= WEB =================

void initWeb() {

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r) {
    r->send(200, "text/html", pageIndex);
  });

  server.on("/api", HTTP_GET, [](AsyncWebServerRequest *r) {
    JsonDocument doc;

    doc["t"] = T;
    doc["h"] = H;
    doc["p"] = P;
    doc["v"] = VBAT;
    doc["iaq"] = IAQ;
    doc["co2"] = CO2;
    doc["voc"] = VOC;
    doc["gas"] = GAS;
    doc["windDir"] = windDir;
    doc["windSpeed"] = windSpeed;
    doc["windGust"] = windGust;
    doc["rssi"] = LoRa.packetRssi();
    doc["uptime"] = millis() / 1000;

    String json;
    serializeJson(doc, json);

    r->send(200, "application/json", json);
  });

  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {
    JsonDocument doc;

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

  server.on(
      "/config", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
      [](AsyncWebServerRequest *request, uint8_t *data, size_t len,
         size_t index, size_t total) {
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, data);

        if (err) {
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

  server.on("/reboot", HTTP_POST, [](AsyncWebServerRequest *request) {
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
    wifiActive = true;
    wifiStartTime = millis();
    Serial.println("WiFi connected");
  } else {
    WiFi.mode(WIFI_OFF);
    wifiActive = false;
    Serial.println("WiFi failed");
  }
}

// ================= SETUP =================

void setup() {

  Serial.begin(115200);

  loadConfig();
  Wire.begin(OLED_SDA, OLED_SCL);
  initDisplay();
  detectSensor();
  showSplash();
  initLoRa();
  readSensors();

  lastSensorTime = millis();
  lastAprsTime = millis() - config.aprsIntervalMs;

  startWiFi();

  // INVIO DEFINIZIONI (Una volta all'avvio)
  Serial.println("Invio definizioni telemetria...");
  sendAPRSDefinitions();
}

// ================= MAIN LOOP =================

void loop() {
  // 1. GESTIONE SENSORI (BSEC2 ULP o Standard)
  if (activeSensor == SENSOR_BME680) {
    // In modalità ULP, bsec.run() gestisce i suoi 300s interni.
    // Restituisce true solo quando ha un nuovo dato pronto.
    if (bsec.run()) {
      Serial.println("BME680: Nuovi dati IAQ pronti");
    }
  } else {
    // Timer standard per BME280 / BMP280
    if (millis() - lastSensorTime >= SENSOR_INTERVAL_MS) {
      lastSensorTime = millis();
      readSensors();
    }
  }

  // 2. INVIO APRS (Alterna Meteo e Telemetria ogni 20 min)
  if (millis() - lastAprsTime >= config.aprsIntervalMs) {
    sendAPRS();
  }

  // 3. GESTIONE WIFI TIMEOUT
  if (!config.wifiAlwaysOn && wifiActive) {
    if (millis() - wifiStartTime > config.wifiTimeoutMs) {
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      wifiActive = false;
      Serial.println("WiFi OFF: Risparmio energetico attivato.");
    }
  }

  // 4. LOGICA LIGHT SLEEP (Solo se WiFi è spento)
  if (!wifiActive) {
    // Calcoliamo quando deve avvenire il prossimo evento
    unsigned long nextSensorEvent = lastSensorTime + SENSOR_INTERVAL_MS;
    unsigned long nextAprsEvent = lastAprsTime + config.aprsIntervalMs;

    // La scadenza più vicina tra sensori e radio
    unsigned long nextEvent = min(nextSensorEvent, nextAprsEvent);

    long sleepTimeMs = nextEvent - millis();

    // Se mancano più di 2 secondi al prossimo evento, dormiamo
    if (sleepTimeMs > 2000) {
      Serial.print("Light Sleep per (ms): ");
      Serial.println(sleepTimeMs);
      Serial.flush(); // Svuota il buffer seriale prima di dormire

      // Configura il risveglio tramite timer
      esp_sleep_enable_timer_wakeup(sleepTimeMs * 1000ULL); // microsecondi
      esp_light_sleep_start();

      // Al risveglio il codice riparte da qui
      Serial.println("Sveglia!");
    }
  }

  // Aggiornamento display
  updateDisplay();

  // Piccolo delay per stabilità del sistema (RTOS)
  delay(10);
}