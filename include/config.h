#pragma once

// ============================================================
//  Hardware pins (ESP-12F / ESP8266)
// ============================================================
#define RELAY_PIN           14
#define I2C_SDA             4
#define I2C_SCL             5
#define LED_PIN             2     // ESP-12F built-in LED (active LOW)
#define FACTORY_RESET_PIN   0     // hold LOW at boot ~5s to factory reset (flash button)

// ============================================================
//  Thermistor (raw Steinhart–Hart, then calibration table)
// ============================================================
#define FIXED_RESISTOR      10980.0f
#define NTC_R25             9380.0f
#define NTC_BETA            3950.0f
#define VCC                 3.3f

// Piecewise linear: NTC reading (°C) → calibrated probe (°C)
// From your comparison table
#define CAL_POINTS          15
// EMA filter: 0 = off, 0.15–0.35 typical (higher = more smoothing, slower response)
#define TEMP_EMA_ALPHA      0.20f
// Reject single samples that jump more than this vs filtered value (°C)
#define TEMP_JUMP_REJECT    6.0f

// ============================================================
//  Thermostat defaults
// ============================================================
#define DEFAULT_SETPOINT    55.0f
#define DEFAULT_HYSTERESIS  5.0f
#define DEFAULT_BOOST_TEMP  65.0f
#define DEFAULT_HOME_TEMP   55.0f
#define DEFAULT_AWAY_TEMP   40.0f
#define DEFAULT_ECO_TEMP    50.0f
#define DEFAULT_BOOST_MIN   0.0f
#define MIN_SETPOINT        30.0f
#define MAX_SETPOINT        100.0f
#define TEMP_VALID_MIN      5.0f
#define TEMP_VALID_MAX      110.0f
#define BOOST_SAFETY_MS     (90UL * 60UL * 1000UL)
#define SENSOR_FAULT_MS     (3UL * 60UL * 1000UL)

// ============================================================
//  Timing
// ============================================================
#define READ_INTERVAL_MS    2000
#define DEFAULT_MIN_ON_MS   30000
#define DEFAULT_MIN_OFF_MS  30000
#define MQTT_RECONNECT_MS   5000
#define WIFI_LED_UPDATE_MS  1000
#define HISTORY_POINTS      60
#define HISTORY_INTERVAL_MS 120000

// ============================================================
//  WiFi / MQTT / Web
// ============================================================
#define AP_NAME             "Geyser-Setup"
#define AP_PASSWORD         "12345678"
#define MQTT_HOST_DEFAULT   ""
#define MQTT_PORT_DEFAULT   1883
#define MQTT_USER_DEFAULT   ""
#define MQTT_PASS_DEFAULT   ""
#define MQTT_BASE_TOPIC     "geyser"
#define HA_DISCOVERY_PREFIX "homeassistant"
#define HTTP_PORT           80
#define FIRMWARE_VERSION    "3.1.2"
#define GITHUB_OWNER        ""
#define GITHUB_REPO         ""
#define GITHUB_ASSET_NAME   "firmware.bin"
