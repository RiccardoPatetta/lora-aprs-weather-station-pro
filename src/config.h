#pragma once

// ======================================================
// CONFIGURAZIONE GENERALE
// ======================================================

#define PROJECT_NAME    "LoRa_APRS_Station"
#define RELEASE_VERSION "4.00"
#define BUILD_DATE      "260211"

// ======================================================
// DEFAULT USER SETTINGS (modificabili via Web / NVS)
// ======================================================

#define DEFAULT_CALLSIGN     "NOCALL"
#define DEFAULT_SSID_M       13
#define DEFAULT_LAT          0.00000
#define DEFAULT_LON          0.00000
#define DEFAULT_BEACON_ON    true
#define DEFAULT_INTERVAL_MIN 20   // invio APRS ogni 20 minuti

#define DEFAULT_WIFI_SSID    "WIFI"
#define DEFAULT_WIFI_PWD     "PASSWORD WIFI"

// ======================================================
// LORA TTGO T3 v1.6.1
// ======================================================

#define LORA_SCK    5
#define LORA_MISO   19
#define LORA_MOSI   27
#define LORA_SS     18
#define LORA_RST    14
#define LORA_DIO0   26

#define LORA_FREQ_HZ   433775000UL
#define LORA_BW        125E3
#define LORA_SF        12
#define LORA_CR        5
#define LORA_PWR_DBM   14

// ======================================================
// SENSORE BME680 (BSEC2)
// ======================================================

#define BME680_I2C_ADDR       0x76
#define BSEC_SAMPLE_RATE      BSEC_SAMPLE_RATE_LP  // 300s outdoor

// Salvataggio stato IAQ ogni 6 ore
#define BSEC_STATE_SAVE_MS    21600000UL  // 6h

// ======================================================
// LIGHT SLEEP
// ======================================================

#define SENSOR_INTERVAL_MS    300000UL    // 5 minuti (300s BSEC)
#define APRS_INTERVAL_MS      1200000UL   // 20 minuti
#define VARIATION_THRESHOLD   0.02f       // 2%

// ======================================================
// SENSORI FALLBACK
// ======================================================

#define ENABLE_BME280   true
#define ENABLE_BMP280   true
#define ENABLE_AHT20    true
#define ENABLE_AS3935   true

// ======================================================
// BATTERIA
// ======================================================

#define VBAT_ADC_PIN    35
#define VBAT_DIVIDER    2.0f

// ======================================================
// DISPLAY
// ======================================================

#define DISPLAY_I2C_ADDR 0x3C
#define DISPLAY_ENABLED_DEFAULT false

// ======================================================
// WEB SERVER
// ======================================================

#define WEBSERVER_PORT 80

