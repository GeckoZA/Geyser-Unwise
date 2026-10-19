/*
 * Geyser Controller v3.1 – ESP-12F + ADS1115 + NTC + Relay
 * Modes: OFF · AUTO · BOOST
 * Features: schedule, presets, runtime stats, temp graph, sensor fault,
 *           soft-min boost, factory reset, OTA, multi-page web UI
 */

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <LittleFS.h>
#include <time.h>
#include <math.h>
#include <ArduinoOTA.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include "config.h"

Adafruit_ADS1115 ads;
AsyncWebServer server(HTTP_PORT);
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

enum Mode : uint8_t { MODE_OFF = 0, MODE_AUTO, MODE_BOOST };
enum Preset : uint8_t { PRESET_CUSTOM = 0, PRESET_HOME, PRESET_AWAY, PRESET_ECO };
const char* MODE_NAMES[] = { "off", "auto", "boost" };
const char* PRESET_NAMES[] = { "custom", "home", "away", "eco" };

float currentTemp = NAN;
float setpoint = DEFAULT_SETPOINT;
float boostTemp = DEFAULT_BOOST_TEMP;
float hysteresis = DEFAULT_HYSTERESIS;
float homeTemp = DEFAULT_HOME_TEMP;
float awayTemp = DEFAULT_AWAY_TEMP;
float ecoTemp = DEFAULT_ECO_TEMP;
float boostMinTemp = DEFAULT_BOOST_MIN; // 0 = disabled
uint32_t minOnMs = DEFAULT_MIN_ON_MS;
uint32_t minOffMs = DEFAULT_MIN_OFF_MS;
Mode mode = MODE_AUTO;
Preset preset = PRESET_CUSTOM;
bool heaterOn = false;
bool sensorFault = false;
bool scheduleEnabled = false;
uint8_t scheduleOnH = 5, scheduleOnM = 0;
uint8_t scheduleOffH = 22, scheduleOffM = 0;

unsigned long lastRead = 0, heaterChangedAt = 0, boostStartedAt = 0;
unsigned long lastMqttAttempt = 0, lastDiscovery = 0, lastLedCheck = 0;
unsigned long badSensorSince = 0;
unsigned long lastHistoryAt = 0;
unsigned long heaterOnAccumMs = 0; // session segment
unsigned long lastHeaterTick = 0;

// Runtime (seconds)
uint32_t runtimeTotalSec = 0;
uint32_t runtimeTodaySec = 0;
uint32_t runtimeWeekSec = 0;
int lastDayOfYear = -1;
int lastWeekNum = -1;

// Temperature history ring buffer
float histTemp[HISTORY_POINTS];
uint32_t histEpoch[HISTORY_POINTS];
uint8_t histHead = 0, histCount = 0;

char mqtt_host[40] = MQTT_HOST_DEFAULT;
char mqtt_port[6] = "1883";
char mqtt_user[32] = MQTT_USER_DEFAULT;
char mqtt_pass[32] = MQTT_PASS_DEFAULT;
bool hideAp = false, apEnabled = true, mqttEnabled = true;
bool pendingMqttReconnect = false;
char apName[32] = AP_NAME;
char friendlyName[40] = "GeyserUnwise";
char apPass[32] = AP_PASSWORD;
uint8_t themeId = 0;
int8_t tzOffsetHours = 2;
String timeStr = "--:--";
char wifi_ssid[33] = "";
char wifi_pass[65] = "";
char githubOwner[40] = GITHUB_OWNER;
char githubRepo[40] = GITHUB_REPO;
bool otaAutoCheck = false;
String lastOtaMsg = "Idle";
String deviceId, topicBase, topicState, topicCmd, topicAvail;

struct Settings {
  float setpoint; uint8_t mode;
  char mqtt_host[40]; char mqtt_port[6]; char mqtt_user[32]; char mqtt_pass[32];
  char wifi_ssid[33]; char wifi_pass[65];
  uint8_t hide_ap, ap_enabled;
  char ap_name[32]; char ap_pass[32];
  uint8_t mqtt_enabled; int8_t tz_offset_hours; uint8_t theme;
  char friendly_name[40];
  float boost_temp, hysteresis;
  uint32_t min_on_ms, min_off_ms;
  char github_owner[40]; char github_repo[40]; uint8_t ota_auto;
  float home_temp, away_temp, eco_temp, boost_min_temp;
  uint8_t preset;
  uint8_t schedule_enabled;
  uint8_t sched_on_h, sched_on_m, sched_off_h, sched_off_m;
  uint32_t runtime_total;
  uint32_t magic;
};
#define EEPROM_MAGIC 0x47455958
#define EEPROM_SIZE sizeof(Settings)

void saveSettings();
void setHeater(bool on, bool force = false);
void setMode(Mode m);
void publishState();
void factoryReset();

bool tempValid(float t) {
  return !isnan(t) && t >= TEMP_VALID_MIN && t <= TEMP_VALID_MAX;
}

void loadSettings() {
  EEPROM.begin(EEPROM_SIZE);
  Settings s;
  EEPROM.get(0, s);
  if (s.magic != EEPROM_MAGIC) return;
  if (s.setpoint < MIN_SETPOINT || s.setpoint > MAX_SETPOINT || s.mode > MODE_BOOST) return;
  setpoint = s.setpoint;
  mode = (Mode)s.mode;
  strncpy(mqtt_host, s.mqtt_host, sizeof(mqtt_host)-1);
  strncpy(mqtt_port, s.mqtt_port, sizeof(mqtt_port)-1);
  strncpy(mqtt_user, s.mqtt_user, sizeof(mqtt_user)-1);
  strncpy(mqtt_pass, s.mqtt_pass, sizeof(mqtt_pass)-1);
  strncpy(wifi_ssid, s.wifi_ssid, sizeof(wifi_ssid)-1);
  strncpy(wifi_pass, s.wifi_pass, sizeof(wifi_pass)-1);
  hideAp = s.hide_ap != 0;
  apEnabled = s.ap_enabled != 0;
  if (s.ap_name[0]) strncpy(apName, s.ap_name, sizeof(apName)-1);
  if (s.ap_pass[0]) strncpy(apPass, s.ap_pass, sizeof(apPass)-1);
  mqttEnabled = s.mqtt_enabled != 0;
  tzOffsetHours = s.tz_offset_hours;
  themeId = s.theme;
  if (s.friendly_name[0]) strncpy(friendlyName, s.friendly_name, sizeof(friendlyName)-1);
  if (s.boost_temp >= MIN_SETPOINT && s.boost_temp <= MAX_SETPOINT) boostTemp = s.boost_temp;
  if (s.hysteresis >= 0.5f && s.hysteresis <= 20.0f) hysteresis = s.hysteresis;
  if (s.min_on_ms >= 5000 && s.min_on_ms <= 600000) minOnMs = s.min_on_ms;
  if (s.min_off_ms >= 5000 && s.min_off_ms <= 600000) minOffMs = s.min_off_ms;
  if (s.github_owner[0]) strncpy(githubOwner, s.github_owner, sizeof(githubOwner)-1);
  if (s.github_repo[0]) strncpy(githubRepo, s.github_repo, sizeof(githubRepo)-1);
  otaAutoCheck = s.ota_auto != 0;
  if (s.home_temp >= MIN_SETPOINT && s.home_temp <= MAX_SETPOINT) homeTemp = s.home_temp;
  if (s.away_temp >= MIN_SETPOINT && s.away_temp <= MAX_SETPOINT) awayTemp = s.away_temp;
  if (s.eco_temp >= MIN_SETPOINT && s.eco_temp <= MAX_SETPOINT) ecoTemp = s.eco_temp;
  if (s.boost_min_temp >= 0 && s.boost_min_temp <= MAX_SETPOINT) boostMinTemp = s.boost_min_temp;
  if (s.preset <= PRESET_ECO) preset = (Preset)s.preset;
  scheduleEnabled = s.schedule_enabled != 0;
  if (s.sched_on_h < 24) scheduleOnH = s.sched_on_h;
  if (s.sched_on_m < 60) scheduleOnM = s.sched_on_m;
  if (s.sched_off_h < 24) scheduleOffH = s.sched_off_h;
  if (s.sched_off_m < 60) scheduleOffM = s.sched_off_m;
  runtimeTotalSec = s.runtime_total;
  if (strlen(mqtt_port) == 0) strncpy(mqtt_port, "1883", sizeof(mqtt_port)-1);
  if (hideAp) apEnabled = false;
  Serial.printf("[EEPROM] set=%.1f boost=%.1f mode=%s sched=%d\n",
                setpoint, boostTemp, MODE_NAMES[mode], scheduleEnabled);
}

void saveSettings() {
  Settings s;
  memset(&s, 0, sizeof(s));
  s.setpoint = setpoint; s.mode = mode;
  strncpy(s.mqtt_host, mqtt_host, sizeof(s.mqtt_host)-1);
  strncpy(s.mqtt_port, mqtt_port, sizeof(s.mqtt_port)-1);
  strncpy(s.mqtt_user, mqtt_user, sizeof(s.mqtt_user)-1);
  strncpy(s.mqtt_pass, mqtt_pass, sizeof(s.mqtt_pass)-1);
  strncpy(s.wifi_ssid, wifi_ssid, sizeof(s.wifi_ssid)-1);
  strncpy(s.wifi_pass, wifi_pass, sizeof(s.wifi_pass)-1);
  s.hide_ap = hideAp ? 1 : 0; s.ap_enabled = apEnabled ? 1 : 0;
  strncpy(s.ap_name, apName, sizeof(s.ap_name)-1);
  strncpy(s.ap_pass, apPass, sizeof(s.ap_pass)-1);
  s.mqtt_enabled = mqttEnabled ? 1 : 0;
  s.tz_offset_hours = tzOffsetHours; s.theme = themeId;
  strncpy(s.friendly_name, friendlyName, sizeof(s.friendly_name)-1);
  s.boost_temp = boostTemp; s.hysteresis = hysteresis;
  s.min_on_ms = minOnMs; s.min_off_ms = minOffMs;
  strncpy(s.github_owner, githubOwner, sizeof(s.github_owner)-1);
  strncpy(s.github_repo, githubRepo, sizeof(s.github_repo)-1);
  s.ota_auto = otaAutoCheck ? 1 : 0;
  s.home_temp = homeTemp; s.away_temp = awayTemp; s.eco_temp = ecoTemp;
  s.boost_min_temp = boostMinTemp; s.preset = preset;
  s.schedule_enabled = scheduleEnabled ? 1 : 0;
  s.sched_on_h = scheduleOnH; s.sched_on_m = scheduleOnM;
  s.sched_off_h = scheduleOffH; s.sched_off_m = scheduleOffM;
  s.runtime_total = runtimeTotalSec;
  s.magic = EEPROM_MAGIC;
  EEPROM.put(0, s);
  EEPROM.commit();
}

void factoryReset() {
  Serial.println("[RESET] Factory reset");
  setHeater(false, true);
  Settings blank; memset(&blank, 0, sizeof(blank));
  EEPROM.put(0, blank);
  EEPROM.commit();
  WiFi.disconnect(true);
  delay(300);
  ESP.restart();
}

// Calibration table: raw NTC °C → true (calibrated probe) °C
// Interpolated from your probe comparison
// Measured probe pairs + high-end extension (40–100 °C focus)
// Above 75 °C: extrapolated from last measured segment — re-check with probe at 85/95 if possible
static const float CAL_NTC[CAL_POINTS] = {
  26.0f, 28.7f, 38.2f, 42.9f, 49.6f, 53.2f, 60.2f, 66.5f, 71.9f, 78.0f,
  85.0f, 92.0f, 99.0f, 106.0f, 113.0f
};
static const float CAL_TRUE[CAL_POINTS] = {
  26.8f, 30.0f, 40.0f, 45.0f, 50.0f, 55.0f, 60.0f, 65.0f, 70.0f, 75.0f,
  80.7f, 86.5f, 92.2f, 98.0f, 103.7f
};

float applyCalibration(float ntcC) {
  if (isnan(ntcC)) return NAN;
  // Below first point: linear extend using first segment
  if (ntcC <= CAL_NTC[0]) {
    float dx = CAL_NTC[1] - CAL_NTC[0];
    if (fabsf(dx) < 1e-6f) return CAL_TRUE[0];
    float t = (ntcC - CAL_NTC[0]) / dx;
    return CAL_TRUE[0] + t * (CAL_TRUE[1] - CAL_TRUE[0]);
  }
  for (int i = 0; i < CAL_POINTS - 1; i++) {
    if (ntcC <= CAL_NTC[i + 1]) {
      float dx = CAL_NTC[i + 1] - CAL_NTC[i];
      float t = (ntcC - CAL_NTC[i]) / dx;
      return CAL_TRUE[i] + t * (CAL_TRUE[i + 1] - CAL_TRUE[i]);
    }
  }
  // Above last point: extend last segment
  int i = CAL_POINTS - 2;
  float dx = CAL_NTC[i + 1] - CAL_NTC[i];
  float t = (ntcC - CAL_NTC[i]) / dx;
  return CAL_TRUE[i] + t * (CAL_TRUE[i + 1] - CAL_TRUE[i]);
}

float readTemperatureRaw() {
  // 8 samples — solar can push >75 °C; more averaging stabilises high end
  int32_t sum = 0;
  for (int i = 0; i < 8; i++) {
    sum += ads.readADC_SingleEnded(0);
    delay(2);
  }
  float adc = sum / 8.0f;
  float voltage = adc * 0.125f / 1000.0f;
  // Allow closer to 3.3 V so ~100 °C remains measurable
  if (voltage < 0.05f || voltage > 3.28f) return NAN;
  float rNtc = FIXED_RESISTOR * ((VCC - voltage) / voltage);
  float steinhart = logf(rNtc / NTC_R25) / NTC_BETA;
  steinhart += 1.0f / 298.15f;
  return (1.0f / steinhart) - 273.15f;
}

static float filteredTemp = NAN;

float readTemperature() {
  float raw = readTemperatureRaw();
  if (isnan(raw)) return NAN;
  float cal = applyCalibration(raw);

  // Spike reject: ignore one-off jumps far from filter (ADC glitch)
  if (!isnan(filteredTemp) && fabsf(cal - filteredTemp) > TEMP_JUMP_REJECT) {
    // Soften: pull filter halfway toward spike once, don't trust full jump
    cal = filteredTemp + 0.3f * (cal - filteredTemp);
  }

  // Exponential moving average
  if (isnan(filteredTemp)) {
    filteredTemp = cal;
  } else {
    filteredTemp = TEMP_EMA_ALPHA * cal + (1.0f - TEMP_EMA_ALPHA) * filteredTemp;
  }
  return filteredTemp;
}

void updateRuntimeTick() {
  unsigned long now = millis();
  if (heaterOn) {
    if (lastHeaterTick == 0) lastHeaterTick = now;
    unsigned long d = now - lastHeaterTick;
    lastHeaterTick = now;
    uint32_t sec = d / 1000;
    if (sec > 0) {
      runtimeTotalSec += sec;
      runtimeTodaySec += sec;
      runtimeWeekSec += sec;
      // persist total occasionally (~every 60s of heater time)
      static uint32_t sinceSave = 0;
      sinceSave += sec;
      if (sinceSave >= 60) { sinceSave = 0; saveSettings(); }
    }
  } else {
    lastHeaterTick = 0;
  }
}

void updateDayWeekCounters() {
  time_t now = time(nullptr);
  if (now < 100000) return;
  struct tm* t = localtime(&now);
  int doy = t->tm_yday;
  int w = t->tm_yday / 7;
  if (lastDayOfYear >= 0 && doy != lastDayOfYear) runtimeTodaySec = 0;
  if (lastWeekNum >= 0 && w != lastWeekNum) runtimeWeekSec = 0;
  lastDayOfYear = doy;
  lastWeekNum = w;
}

bool scheduleAllowsHeat() {
  if (!scheduleEnabled) return true;
  time_t now = time(nullptr);
  if (now < 100000) return true; // no time → allow
  struct tm* t = localtime(&now);
  int nowM = t->tm_hour * 60 + t->tm_min;
  int onM = scheduleOnH * 60 + scheduleOnM;
  int offM = scheduleOffH * 60 + scheduleOffM;
  if (onM == offM) return true; // zero-length = always
  if (onM < offM) return nowM >= onM && nowM < offM; // same day
  return nowM >= onM || nowM < offM; // overnight window
}

void setHeater(bool on, bool force) {
  if (on) {
    // Never turn on with bad sensor
    if (!tempValid(currentTemp) || sensorFault) {
      Serial.println("[HEATER] blocked – sensor fault/invalid");
      on = false;
      force = true;
    }
  }
  if (on == heaterOn) return;
  if (!force) {
    unsigned long elapsed = millis() - heaterChangedAt;
    if (on && elapsed < minOffMs) return;
    if (!on && elapsed < minOnMs) return;
  }
  digitalWrite(RELAY_PIN, on ? HIGH : LOW);
  heaterOn = on;
  heaterChangedAt = millis();
  lastHeaterTick = on ? millis() : 0;
  Serial.printf("[HEATER] %s%s\n", on ? "ON" : "OFF", force ? " (forced)" : "");
}

void updateThermostat() {
  // Sensor fault tracking
  if (!tempValid(currentTemp)) {
    if (badSensorSince == 0) badSensorSince = millis();
    if (millis() - badSensorSince >= SENSOR_FAULT_MS) {
      if (!sensorFault) {
        sensorFault = true;
        Serial.println("[SENSOR] FAULT – heater forced off");
        setHeater(false, true);
        publishState();
      }
    }
  } else {
    badSensorSince = 0;
    if (sensorFault) {
      sensorFault = false;
      Serial.println("[SENSOR] recovered");
    }
  }

  if (mode == MODE_BOOST) {
    if (millis() - boostStartedAt >= BOOST_SAFETY_MS) {
      mode = MODE_AUTO; saveSettings();
      Serial.println("[MODE] BOOST safety → AUTO");
    } else if (tempValid(currentTemp) && currentTemp >= boostTemp) {
      mode = MODE_AUTO; saveSettings();
      setHeater(false);
      Serial.println("[MODE] BOOST done → AUTO");
    }
  }

  switch (mode) {
    case MODE_OFF:
      setHeater(false, true);
      break;
    case MODE_AUTO:
    case MODE_BOOST: {
      if (!tempValid(currentTemp) || sensorFault) {
        setHeater(false, true);
        break;
      }
      // Schedule gates AUTO only; BOOST overrides schedule
      if (mode == MODE_AUTO && !scheduleAllowsHeat()) {
        setHeater(false, true);
        break;
      }
      float target = (mode == MODE_BOOST) ? boostTemp : setpoint;
      if (currentTemp < target - hysteresis) setHeater(true);
      else if (currentTemp >= target) setHeater(false);
      break;
    }
  }
}

void setMode(Mode m) {
  if (m == MODE_BOOST) {
    // Soft min boost: only if below threshold (when threshold > 0)
    if (boostMinTemp > 0.5f && tempValid(currentTemp) && currentTemp >= boostMinTemp) {
      Serial.printf("[MODE] Boost skipped – temp %.1f >= min %.1f\n", currentTemp, boostMinTemp);
      return;
    }
    if (!tempValid(currentTemp) || sensorFault) {
      Serial.println("[MODE] Boost blocked – sensor");
      return;
    }
  }
  if (m == mode) return;
  mode = m;
  if (m == MODE_BOOST) boostStartedAt = millis();
  saveSettings();
  Serial.printf("[MODE] → %s\n", MODE_NAMES[mode]);
  if (m == MODE_OFF) setHeater(false, true);
}

void applyPreset(Preset p) {
  preset = p;
  if (p == PRESET_HOME) setpoint = homeTemp;
  else if (p == PRESET_AWAY) setpoint = awayTemp;
  else if (p == PRESET_ECO) setpoint = ecoTemp;
  saveSettings();
  publishState();
}

void pushHistory(float t) {
  if (!tempValid(t)) return;
  time_t now = time(nullptr);
  histTemp[histHead] = t;
  histEpoch[histHead] = (now > 100000) ? (uint32_t)now : (millis() / 1000);
  histHead = (histHead + 1) % HISTORY_POINTS;
  if (histCount < HISTORY_POINTS) histCount++;
}

void updateWifiLed() {
  digitalWrite(LED_PIN, (WiFi.status() == WL_CONNECTED) ? LOW : HIGH);
}

void setupMDNS() {
  WiFi.hostname("Geyser-Unwise");
  if (MDNS.begin("Geyser-Unwise")) MDNS.addService("http", "tcp", 80);
}

void setupNTP() {
  configTime(tzOffsetHours * 3600, 0, "pool.ntp.org", "time.nist.gov");
}

void updateTimeString() {
  time_t now = time(nullptr);
  if (now < 100000) { timeStr = "--:--"; return; }
  struct tm* t = localtime(&now);
  char buf[16];
  snprintf(buf, sizeof(buf), "%02d:%02d", t->tm_hour, t->tm_min);
  timeStr = buf;
}

// ---- MQTT ----
void publishState() {
  if (!mqtt.connected()) return;
  JsonDocument doc;
  if (tempValid(currentTemp))
    doc["temperature"] = (float)(roundf(currentTemp * 10.0f) / 10.0f);
  else
    doc["temperature"] = nullptr;
  float shown = (mode == MODE_BOOST) ? boostTemp : setpoint;
  doc["target_temp"] = (float)(roundf(shown * 10.0f) / 10.0f);
  doc["setpoint"] = (float)(roundf(setpoint * 10.0f) / 10.0f);
  doc["boost_temp"] = (float)(roundf(boostTemp * 10.0f) / 10.0f);
  doc["hysteresis"] = (float)(roundf(hysteresis * 10.0f) / 10.0f);
  doc["mode"] = MODE_NAMES[mode];
  doc["preset"] = PRESET_NAMES[preset];
  doc["action"] = heaterOn ? "heating" : "idle";
  doc["heater"] = heaterOn;
  doc["sensor_fault"] = sensorFault;
  doc["schedule_active"] = scheduleEnabled && scheduleAllowsHeat();
  doc["runtime_total_h"] = (float)(roundf(runtimeTotalSec / 3600.0f * 10) / 10);
  doc["runtime_today_h"] = (float)(roundf(runtimeTodaySec / 3600.0f * 100) / 100);
  doc["runtime_week_h"] = (float)(roundf(runtimeWeekSec / 3600.0f * 100) / 100);
  char buf[512];
  serializeJson(doc, buf, sizeof(buf));
  mqtt.publish(topicState.c_str(), (const uint8_t*)buf, strlen(buf), true);
}

void publishDiscovery() {
  if (!mqtt.connected()) return;
  auto pub = [&](const String& topic, JsonDocument& doc) {
    char buf[1400]; size_t n = serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(topic.c_str(), (const uint8_t*)buf, n, true);
  };
  {
    String t = String(HA_DISCOVERY_PREFIX) + "/climate/" + deviceId + "/config";
    JsonDocument doc;
    doc["name"] = friendlyName;
    doc["unique_id"] = deviceId + "_climate_v5";
    doc["icon"] = "mdi:water-boiler";
    JsonArray modes = doc["modes"].to<JsonArray>();
    modes.add("off"); modes.add("auto");
    JsonArray presets = doc["preset_modes"].to<JsonArray>();
    presets.add("none"); presets.add("boost");
    doc["mode_command_topic"] = topicCmd + "/mode";
    doc["mode_state_topic"] = topicState;
    doc["mode_state_template"] = "{% if value_json.mode == 'off' %}off{% else %}auto{% endif %}";
    doc["preset_mode_command_topic"] = topicCmd + "/preset";
    doc["preset_mode_state_topic"] = topicState;
    doc["preset_mode_value_template"] = "{% if value_json.mode == 'boost' %}boost{% else %}none{% endif %}";
    doc["current_temperature_topic"] = topicState;
    doc["current_temperature_template"] = "{{ value_json.temperature }}";
    doc["temperature_command_topic"] = topicCmd + "/setpoint";
    doc["temperature_state_topic"] = topicState;
    doc["temperature_state_template"] = "{{ value_json.target_temp }}";
    doc["action_topic"] = topicState;
    doc["action_template"] = "{{ value_json.action }}";
    doc["min_temp"] = (int)MIN_SETPOINT; doc["max_temp"] = (int)MAX_SETPOINT;
    doc["temp_step"] = 1; doc["temperature_unit"] = "C";
    doc["availability_topic"] = topicAvail;
    doc["payload_available"] = "online"; doc["payload_not_available"] = "offline";
    doc["device"]["identifiers"][0] = String("geyser_") + deviceId;
    doc["device"]["name"] = friendlyName;
    doc["device"]["sw_version"] = FIRMWARE_VERSION;
    pub(t, doc);
  }
  {
    String t = String(HA_DISCOVERY_PREFIX) + "/binary_sensor/" + deviceId + "_fault/config";
    JsonDocument doc;
    doc["name"] = String(friendlyName) + " Sensor Fault";
    doc["unique_id"] = deviceId + "_fault_v5";
    doc["state_topic"] = topicState;
    doc["value_template"] = "{{ 'ON' if value_json.sensor_fault else 'OFF' }}";
    doc["device_class"] = "problem";
    doc["availability_topic"] = topicAvail;
    doc["payload_available"] = "online"; doc["payload_not_available"] = "offline";
    doc["device"]["identifiers"][0] = String("geyser_") + deviceId;
    pub(t, doc);
  }
  {
    String t = String(HA_DISCOVERY_PREFIX) + "/sensor/" + deviceId + "_runtime/config";
    JsonDocument doc;
    doc["name"] = String(friendlyName) + " Heater Hours Today";
    doc["unique_id"] = deviceId + "_rt_today_v5";
    doc["state_topic"] = topicState;
    doc["value_template"] = "{{ value_json.runtime_today_h }}";
    doc["unit_of_measurement"] = "h";
    doc["state_class"] = "total_increasing";
    doc["availability_topic"] = topicAvail;
    doc["payload_available"] = "online"; doc["payload_not_available"] = "offline";
    doc["device"]["identifiers"][0] = String("geyser_") + deviceId;
    pub(t, doc);
  }
  {
    String t = String(HA_DISCOVERY_PREFIX) + "/number/" + deviceId + "_boost/config";
    JsonDocument doc;
    doc["name"] = String(friendlyName) + " Boost Temp";
    doc["unique_id"] = deviceId + "_boost_v5";
    doc["command_topic"] = topicCmd + "/boost_temp";
    doc["state_topic"] = topicState;
    doc["value_template"] = "{{ value_json.boost_temp }}";
    doc["min"] = (int)MIN_SETPOINT; doc["max"] = (int)MAX_SETPOINT; doc["step"] = 1;
    doc["unit_of_measurement"] = "°C";
    doc["availability_topic"] = topicAvail;
    doc["payload_available"] = "online"; doc["payload_not_available"] = "offline";
    doc["device"]["identifiers"][0] = String("geyser_") + deviceId;
    pub(t, doc);
  }
  mqtt.publish(topicAvail.c_str(), "online", true);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String t(topic), msg;
  for (unsigned i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();
  Serial.printf("[MQTT] %s → %s\n", topic, msg.c_str());
  if (t.endsWith("/setpoint")) {
    float v = msg.toFloat();
    if (v >= MIN_SETPOINT && v <= MAX_SETPOINT) {
      setpoint = v; preset = PRESET_CUSTOM; saveSettings();
    }
  } else if (t.endsWith("/boost_temp")) {
    float v = msg.toFloat();
    if (v >= MIN_SETPOINT && v <= MAX_SETPOINT) { boostTemp = v; saveSettings(); }
  } else if (t.endsWith("/hysteresis")) {
    float v = msg.toFloat();
    if (v >= 0.5f && v <= 20.0f) { hysteresis = v; saveSettings(); }
  } else if (t.endsWith("/mode")) {
    if (msg == "off") setMode(MODE_OFF);
    else if (msg == "auto" || msg == "heat" || msg == "on") setMode(MODE_AUTO);
  } else if (t.endsWith("/preset")) {
    if (msg == "boost") setMode(MODE_BOOST);
    else if (msg == "none" && mode == MODE_BOOST) setMode(MODE_AUTO);
    else if (msg == "home") applyPreset(PRESET_HOME);
    else if (msg == "away") applyPreset(PRESET_AWAY);
    else if (msg == "eco") applyPreset(PRESET_ECO);
  } else if (t.endsWith("/heater")) {
    if (msg == "ON" || msg == "on" || msg == "1") setMode(MODE_AUTO);
    else setMode(MODE_OFF);
  }
  publishState();
}

bool mqttConnect() {
  if (strlen(mqtt_host) < 2) return false;
  mqtt.setServer(mqtt_host, atoi(mqtt_port));
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(2048);
  mqtt.setSocketTimeout(5);
  mqtt.setKeepAlive(60);
  String clientId = "geyser-" + deviceId;
  bool ok = (strlen(mqtt_user) > 0)
    ? mqtt.connect(clientId.c_str(), mqtt_user, mqtt_pass, topicAvail.c_str(), 0, true, "offline")
    : mqtt.connect(clientId.c_str(), topicAvail.c_str(), 0, true, "offline");
  if (ok) {
    mqtt.subscribe((topicCmd + "/#").c_str());
    publishDiscovery();
    publishState();
  }
  return ok;
}

void mqttLoop() {
  if (!mqttEnabled || strlen(mqtt_host) < 2) return;
  if (pendingMqttReconnect) { pendingMqttReconnect = false; lastMqttAttempt = 0; }
  if (!mqtt.connected()) {
    if (millis() - lastMqttAttempt > MQTT_RECONNECT_MS) {
      lastMqttAttempt = millis();
      mqttConnect();
    }
  } else {
    mqtt.loop();
    if (millis() - lastDiscovery > 60000UL) {
      lastDiscovery = millis();
      mqtt.publish(topicAvail.c_str(), "online", true);
    }
  }
}

void setupArduinoOTA() {
  ArduinoOTA.setHostname("Geyser-Unwise");
  ArduinoOTA.onStart([]() { setHeater(false, true); });
  ArduinoOTA.begin();
}

bool checkGitHubUpdate(bool install) {
  if (strlen(githubOwner) < 1 || strlen(githubRepo) < 1) {
    lastOtaMsg = "GitHub owner/repo not set"; return false;
  }
  if (WiFi.status() != WL_CONNECTED) { lastOtaMsg = "WiFi not connected"; return false; }
  String api = String("https://api.github.com/repos/") + githubOwner + "/" + githubRepo + "/releases/latest";
  WiFiClientSecure client; client.setInsecure();
  HTTPClient http; http.setTimeout(15000); http.setUserAgent("GeyserUnwise");
  if (!http.begin(client, api)) { lastOtaMsg = "HTTP begin failed"; return false; }
  int code = http.GET();
  if (code != 200) { lastOtaMsg = String("API HTTP ") + code; http.end(); return false; }
  String body = http.getString(); http.end();
  JsonDocument doc;
  if (deserializeJson(doc, body)) { lastOtaMsg = "JSON error"; return false; }
  const char* tag = doc["tag_name"];
  if (!tag) { lastOtaMsg = "No tag"; return false; }
  String remoteVer = tag;
  if (remoteVer.startsWith("v") || remoteVer.startsWith("V")) remoteVer = remoteVer.substring(1);
  if (remoteVer == FIRMWARE_VERSION) { lastOtaMsg = String("Up to date (") + FIRMWARE_VERSION + ")"; return false; }
  String binUrl;
  for (JsonObject a : doc["assets"].as<JsonArray>()) {
    const char* name = a["name"];
    if (name && String(name) == GITHUB_ASSET_NAME) { binUrl = a["browser_download_url"].as<String>(); break; }
  }
  if (!binUrl.length()) { lastOtaMsg = "No firmware.bin asset"; return false; }
  lastOtaMsg = String("Update available: ") + tag;
  if (!install) return true;
  setHeater(false, true);
  ESPhttpUpdate.rebootOnUpdate(true);
  t_httpUpdate_return ret = ESPhttpUpdate.update(client, binUrl);
  if (ret == HTTP_UPDATE_FAILED) lastOtaMsg = ESPhttpUpdate.getLastErrorString();
  return ret == HTTP_UPDATE_OK;
}

// ---- API helpers ----
void copyStr(char *dst, size_t n, JsonVariantConst v) {
  if (v.isNull()) return;
  const char *s = v.as<const char*>();
  if (!s) return;
  strncpy(dst, s, n - 1); dst[n - 1] = 0;
}

void bodyAccum(AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t index, size_t total) {
  if (index == 0) {
    if (req->_tempObject) delete reinterpret_cast<String*>(req->_tempObject);
    req->_tempObject = new String();
    reinterpret_cast<String*>(req->_tempObject)->reserve(total + 1);
  }
  String *b = reinterpret_cast<String*>(req->_tempObject);
  if (b) for (size_t i = 0; i < len; i++) (*b) += (char)data[i];
}

void applySettingsJson(JsonDocument& doc) {
  copyStr(mqtt_host, sizeof(mqtt_host), doc["mqtt_host"]);
  copyStr(mqtt_port, sizeof(mqtt_port), doc["mqtt_port"]);
  copyStr(mqtt_user, sizeof(mqtt_user), doc["mqtt_user"]);
  if (!doc["mqtt_pass"].isNull()) {
    const char *pw = doc["mqtt_pass"].as<const char*>();
    if (pw && strlen(pw) > 0) copyStr(mqtt_pass, sizeof(mqtt_pass), doc["mqtt_pass"]);
  }
  if (!doc["mqtt_enabled"].isNull()) mqttEnabled = doc["mqtt_enabled"].as<bool>();
  if (strlen(mqtt_host) > 0) mqttEnabled = true;
  if (strlen(mqtt_port) == 0) strncpy(mqtt_port, "1883", sizeof(mqtt_port)-1);
  if (!doc["ap_enabled"].isNull()) { apEnabled = doc["ap_enabled"].as<bool>(); hideAp = !apEnabled; }
  copyStr(apName, sizeof(apName), doc["ap_name"]);
  if (!doc["ap_pass"].isNull()) {
    const char *pw = doc["ap_pass"].as<const char*>();
    if (pw && (strlen(pw) == 0 || strlen(pw) >= 8)) copyStr(apPass, sizeof(apPass), doc["ap_pass"]);
  }
  if (!doc["tz"].isNull()) tzOffsetHours = (int8_t)doc["tz"].as<int>();
  if (!doc["theme"].isNull()) themeId = (uint8_t)constrain(doc["theme"].as<int>(), 0, 2);
  if (!doc["friendly_name"].isNull()) {
    copyStr(friendlyName, sizeof(friendlyName), doc["friendly_name"]);
    if (strlen(friendlyName) < 1) strncpy(friendlyName, "GeyserUnwise", sizeof(friendlyName)-1);
  }
  auto clampF = [](float v, float lo, float hi) { return constrain(v, lo, hi); };
  if (!doc["setpoint"].isNull()) { setpoint = clampF(doc["setpoint"].as<float>(), MIN_SETPOINT, MAX_SETPOINT); preset = PRESET_CUSTOM; }
  if (!doc["boost_temp"].isNull()) boostTemp = clampF(doc["boost_temp"].as<float>(), MIN_SETPOINT, MAX_SETPOINT);
  if (!doc["hysteresis"].isNull()) hysteresis = clampF(doc["hysteresis"].as<float>(), 0.5f, 20.0f);
  if (!doc["min_on_ms"].isNull()) { uint32_t v = doc["min_on_ms"].as<uint32_t>(); if (v >= 5000 && v <= 600000) minOnMs = v; }
  if (!doc["min_off_ms"].isNull()) { uint32_t v = doc["min_off_ms"].as<uint32_t>(); if (v >= 5000 && v <= 600000) minOffMs = v; }
  if (!doc["home_temp"].isNull()) homeTemp = clampF(doc["home_temp"].as<float>(), MIN_SETPOINT, MAX_SETPOINT);
  if (!doc["away_temp"].isNull()) awayTemp = clampF(doc["away_temp"].as<float>(), MIN_SETPOINT, MAX_SETPOINT);
  if (!doc["eco_temp"].isNull()) ecoTemp = clampF(doc["eco_temp"].as<float>(), MIN_SETPOINT, MAX_SETPOINT);
  if (!doc["boost_min_temp"].isNull()) boostMinTemp = clampF(doc["boost_min_temp"].as<float>(), 0.0f, MAX_SETPOINT);
  if (!doc["schedule_enabled"].isNull()) scheduleEnabled = doc["schedule_enabled"].as<bool>();
  if (!doc["sched_on_h"].isNull()) scheduleOnH = constrain(doc["sched_on_h"].as<int>(), 0, 23);
  if (!doc["sched_on_m"].isNull()) scheduleOnM = constrain(doc["sched_on_m"].as<int>(), 0, 59);
  if (!doc["sched_off_h"].isNull()) scheduleOffH = constrain(doc["sched_off_h"].as<int>(), 0, 23);
  if (!doc["sched_off_m"].isNull()) scheduleOffM = constrain(doc["sched_off_m"].as<int>(), 0, 59);
  copyStr(githubOwner, sizeof(githubOwner), doc["github_owner"]);
  copyStr(githubRepo, sizeof(githubRepo), doc["github_repo"]);
  if (!doc["ota_auto"].isNull()) otaAutoCheck = doc["ota_auto"].as<bool>();
  if (!doc["preset"].isNull()) {
    const char* p = doc["preset"].as<const char*>();
    if (p) {
      if (!strcmp(p, "home")) applyPreset(PRESET_HOME);
      else if (!strcmp(p, "away")) applyPreset(PRESET_AWAY);
      else if (!strcmp(p, "eco")) applyPreset(PRESET_ECO);
      else preset = PRESET_CUSTOM;
    }
  }
  saveSettings();
  if (!apEnabled) { WiFi.softAPdisconnect(true); WiFi.mode(WIFI_STA); }
  else if (WiFi.status() == WL_CONNECTED) {
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(apName, strlen(apPass) >= 8 ? apPass : AP_PASSWORD);
  }
  setupNTP();
  if (mqtt.connected()) mqtt.disconnect();
  pendingMqttReconnect = true;
}

void setupAPI() {
  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *req) {
    JsonDocument doc;
    if (tempValid(currentTemp)) doc["temperature"] = serialized(String(currentTemp, 1));
    else doc["temperature"] = nullptr;
    doc["setpoint"] = serialized(String(setpoint, 1));
    doc["boost_temp"] = serialized(String(boostTemp, 1));
    doc["hysteresis"] = serialized(String(hysteresis, 1));
    doc["home_temp"] = serialized(String(homeTemp, 1));
    doc["away_temp"] = serialized(String(awayTemp, 1));
    doc["eco_temp"] = serialized(String(ecoTemp, 1));
    doc["boost_min_temp"] = serialized(String(boostMinTemp, 1));
    doc["min_on_ms"] = minOnMs; doc["min_off_ms"] = minOffMs;
    doc["mode"] = MODE_NAMES[mode];
    doc["preset"] = PRESET_NAMES[preset];
    doc["heater"] = heaterOn;
    doc["action"] = heaterOn ? "heating" : "idle";
    doc["sensor_fault"] = sensorFault;
    doc["schedule_enabled"] = scheduleEnabled;
    doc["sched_on_h"] = scheduleOnH; doc["sched_on_m"] = scheduleOnM;
    doc["sched_off_h"] = scheduleOffH; doc["sched_off_m"] = scheduleOffM;
    doc["schedule_active"] = scheduleEnabled && scheduleAllowsHeat();
    doc["runtime_total_sec"] = runtimeTotalSec;
    doc["runtime_today_sec"] = runtimeTodaySec;
    doc["runtime_week_sec"] = runtimeWeekSec;
    doc["ip"] = WiFi.localIP().toString();
    doc["rssi"] = WiFi.RSSI();
    doc["uptime"] = millis() / 1000;
    doc["version"] = FIRMWARE_VERSION;
    if (mode == MODE_BOOST) {
      unsigned long left = BOOST_SAFETY_MS - (millis() - boostStartedAt);
      doc["boost_remaining"] = left / 1000;
    }
    doc["time"] = timeStr;
    doc["mqtt_host"] = mqtt_host; doc["mqtt_port"] = mqtt_port;
    doc["mqtt_user"] = mqtt_user; doc["mqtt_connected"] = mqtt.connected();
    doc["ap_enabled"] = apEnabled; doc["ap_name"] = apName;
    doc["mqtt_enabled"] = mqttEnabled; doc["tz"] = tzOffsetHours;
    doc["theme"] = themeId; doc["friendly_name"] = friendlyName;
    doc["mdns"] = "Geyser-Unwise.local";
    doc["github_owner"] = githubOwner; doc["github_repo"] = githubRepo;
    doc["ota_auto"] = otaAutoCheck; doc["ota_msg"] = lastOtaMsg;
    String json; serializeJson(doc, json);
    req->send(200, "application/json", json);
  });

  server.on("/api/history", HTTP_GET, [](AsyncWebServerRequest *req) {
    JsonDocument doc;
    JsonArray temps = doc["temps"].to<JsonArray>();
    JsonArray epochs = doc["epochs"].to<JsonArray>();
    for (uint8_t i = 0; i < histCount; i++) {
      uint8_t idx = (histHead + HISTORY_POINTS - histCount + i) % HISTORY_POINTS;
      temps.add(serialized(String(histTemp[idx], 1)));
      epochs.add(histEpoch[idx]);
    }
    String json; serializeJson(doc, json);
    req->send(200, "application/json", json);
  });

  server.on("/api/setpoint", HTTP_POST, [](AsyncWebServerRequest *req) {}, NULL,
    [](AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t, size_t) {
      JsonDocument doc;
      if (deserializeJson(doc, data, len) || !doc["setpoint"].is<float>()) {
        req->send(400, "application/json", "{\"error\":\"bad request\"}"); return;
      }
      setpoint = constrain(doc["setpoint"].as<float>(), MIN_SETPOINT, MAX_SETPOINT);
      preset = PRESET_CUSTOM; saveSettings(); publishState();
      req->send(200, "application/json", "{\"ok\":true}");
    });

  server.on("/api/mode", HTTP_POST, [](AsyncWebServerRequest *req) {}, NULL,
    [](AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t, size_t) {
      JsonDocument doc;
      if (deserializeJson(doc, data, len) || !doc["mode"].is<const char*>()) {
        req->send(400, "application/json", "{\"error\":\"bad request\"}"); return;
      }
      const char* m = doc["mode"];
      if (!strcmp(m, "off")) setMode(MODE_OFF);
      else if (!strcmp(m, "auto")) setMode(MODE_AUTO);
      else if (!strcmp(m, "boost")) setMode(MODE_BOOST);
      else { req->send(400, "application/json", "{\"error\":\"unknown mode\"}"); return; }
      publishState();
      req->send(200, "application/json", "{\"ok\":true}");
    });

  server.on("/api/preset", HTTP_POST, [](AsyncWebServerRequest *req) {}, NULL,
    [](AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t, size_t) {
      JsonDocument doc;
      if (deserializeJson(doc, data, len) || !doc["preset"].is<const char*>()) {
        req->send(400, "application/json", "{\"error\":\"bad request\"}"); return;
      }
      const char* p = doc["preset"];
      if (!strcmp(p, "home")) applyPreset(PRESET_HOME);
      else if (!strcmp(p, "away")) applyPreset(PRESET_AWAY);
      else if (!strcmp(p, "eco")) applyPreset(PRESET_ECO);
      else { req->send(400, "application/json", "{\"error\":\"unknown\"}"); return; }
      req->send(200, "application/json", "{\"ok\":true}");
    });

  server.on("/api/settings", HTTP_POST,
    [](AsyncWebServerRequest *req) {
      String *body = reinterpret_cast<String*>(req->_tempObject);
      if (!body) { req->send(400, "application/json", "{\"error\":\"no body\"}"); return; }
      JsonDocument doc;
      if (deserializeJson(doc, *body)) {
        delete body; req->_tempObject = nullptr;
        req->send(400, "application/json", "{\"error\":\"bad json\"}"); return;
      }
      delete body; req->_tempObject = nullptr;
      applySettingsJson(doc);
      publishState();
      req->send(200, "application/json", "{\"ok\":true}");
    }, NULL, bodyAccum);

  server.on("/api/ota/check", HTTP_GET, [](AsyncWebServerRequest *req) {
    checkGitHubUpdate(false);
    JsonDocument doc; doc["msg"] = lastOtaMsg; doc["version"] = FIRMWARE_VERSION;
    String json; serializeJson(doc, json);
    req->send(200, "application/json", json);
  });
  server.on("/api/ota/install", HTTP_POST, [](AsyncWebServerRequest *req) {
    req->send(200, "application/json", "{\"ok\":true}");
    delay(100); checkGitHubUpdate(true);
  });
  server.on("/api/factoryreset", HTTP_POST, [](AsyncWebServerRequest *req) {
    req->send(200, "text/plain", "Factory reset...");
    delay(300); factoryReset();
  });
  server.on("/api/resetwifi", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send(200, "text/plain", "Clearing WiFi...");
    delay(300); WiFi.disconnect(true); ESP.restart();
  });
}

// ---- Web UI ----
const char SHARED_HEAD[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="en"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>GeyserUnwise</title>
<style>
:root{--bg:#05080f;--card:#0a1628;--primary:#1e6bb8;--primary-bright:#3b9eff;--accent:#ff8c00;--accent-hot:#ff5a00;--text:#e8f0fa;--muted:#6b8aad;--track:#123050;--drawer:#0c1a2e}
body.theme-dark{--bg:#0a0a0a;--card:#161616;--primary:#3b82f6;--primary-bright:#60a5fa;--track:#2a2a2a;--drawer:#111;--muted:#888;--text:#f0f0f0}
body.theme-light{--bg:#e8eef5;--card:#fff;--primary:#1e6bb8;--primary-bright:#0284c7;--text:#0f172a;--muted:#64748b;--track:#cbd5e1;--drawer:#f1f5f9;--accent:#ea580c;--accent-hot:#dc2626}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,sans-serif;background:var(--bg);color:var(--text);min-height:100vh;padding:16px 12px 40px}
.topbar{display:flex;align-items:center;justify-content:space-between;max-width:440px;margin:0 auto 16px}
.brand{display:flex;align-items:center;gap:10px}
.brand img{width:48px;height:48px;border-radius:10px;object-fit:contain}
.brand span{font-weight:700;color:var(--primary-bright)}
.hamburger{width:42px;height:42px;border:none;border-radius:10px;background:var(--card);cursor:pointer;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:5px}
.hamburger span{display:block;width:20px;height:2px;background:var(--text)}
.overlay{position:fixed;inset:0;background:rgba(0,0,0,.45);z-index:40;opacity:0;pointer-events:none;transition:.25s}
.overlay.open{opacity:1;pointer-events:auto}
.drawer{position:fixed;top:0;right:0;width:min(300px,88vw);height:100%;background:var(--drawer);z-index:50;transform:translateX(100%);transition:.28s;overflow-y:auto;padding:20px 18px 40px}
.drawer.open{transform:translateX(0)}
.drawer h2{font-size:1rem;margin-bottom:16px;color:var(--primary-bright)}
.drawer nav a{display:block;padding:12px 14px;border-radius:10px;color:var(--text);text-decoration:none;margin-bottom:4px;font-weight:600}
.drawer nav a:hover,.drawer nav a.active{background:var(--primary);color:#fff}
.drawer nav .sub{padding-left:28px;font-size:.88rem;font-weight:500;color:var(--muted)}
.card{background:var(--card);border-radius:20px;padding:20px;max-width:440px;margin:0 auto;box-shadow:0 12px 40px rgba(0,0,0,.35)}
.card h1{font-size:1.1rem;margin-bottom:14px;color:var(--primary-bright)}
label{display:block;font-size:.72rem;text-transform:uppercase;letter-spacing:.06em;color:var(--muted);margin:12px 0 6px}
label.row{display:flex;align-items:center;justify-content:space-between;text-transform:none;font-size:.9rem;letter-spacing:0;color:var(--text)}
input,select{width:100%;padding:10px;border-radius:8px;border:1px solid var(--track);background:var(--bg);color:var(--text);font-size:.95rem}
button.save,button.primary{width:100%;padding:12px;border:none;border-radius:10px;background:var(--primary);color:#fff;font-weight:600;cursor:pointer;margin-top:12px}
button.danger{background:#b91c1c}
.hint{font-size:.75rem;color:var(--muted);margin-top:6px}
.footer{max-width:440px;margin:18px auto 0;text-align:center;font-size:.75rem;color:var(--muted)}
.footer #clock{font-size:1.1rem;font-weight:600;color:var(--primary-bright);margin-bottom:4px}
.badge{display:inline-block;padding:2px 8px;border-radius:6px;font-size:.7rem;font-weight:700;background:#b91c1c;color:#fff;margin-left:6px}
.dial-wrap{position:relative;width:260px;height:260px;margin:0 auto 12px;user-select:none;touch-action:none}
.dial-svg{width:100%;height:100%}
.dial-track{fill:none;stroke:var(--track);stroke-width:18}
.dial-progress{fill:none;stroke:var(--primary);stroke-width:18;stroke-linecap:round}
.dial-progress.heating{stroke:var(--accent)}.dial-progress.boost{stroke:var(--accent-hot)}
.dial-center{position:absolute;inset:48px;border-radius:50%;background:var(--bg);display:flex;flex-direction:column;align-items:center;justify-content:center}
.dial-temp{font-size:2.8rem;font-weight:700;line-height:1}
.dial-unit{font-size:.85rem;color:var(--muted)}
.dial-target{font-size:.8rem;color:var(--primary-bright);margin-top:8px;font-weight:600}
.dial-action{font-size:.7rem;text-transform:uppercase;letter-spacing:.1em;margin-top:4px;color:var(--muted)}
.dial-action.heating{color:var(--accent)}.dial-action.boost{color:var(--accent-hot)}
.modes{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-bottom:10px}
.presets{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.mode-btn,.preset-btn{background:var(--bg);border:1px solid transparent;color:var(--muted);border-radius:12px;padding:10px 4px;font-size:.72rem;font-weight:700;text-transform:uppercase;cursor:pointer}
.mode-btn.active{background:var(--primary);color:#fff}
.mode-btn.active[data-mode=off]{background:#475569}
.mode-btn.active[data-mode=boost]{background:var(--accent-hot);color:#fff}
.preset-btn.active{background:var(--primary);color:#fff}
canvas.chart{width:100%;height:160px;background:var(--bg);border-radius:12px;margin-top:8px}
.stats{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin-top:12px;text-align:center}
.stats div{background:var(--bg);border-radius:10px;padding:10px 4px}
.stats .n{font-size:1.1rem;font-weight:700;color:var(--primary-bright)}
.stats .l{font-size:.65rem;color:var(--muted);text-transform:uppercase}
</style></head><body>
)rawliteral";

const char NAV_SCRIPT[] PROGMEM = R"rawliteral(
<script>
function toggleMenu(){document.getElementById('drawer').classList.toggle('open');document.getElementById('overlay').classList.toggle('open')}
function applyTheme(id){document.body.classList.remove('theme-dark','theme-light');if(id==1)document.body.classList.add('theme-dark');if(id==2)document.body.classList.add('theme-light')}
async function loadTheme(){try{const r=await fetch('/api/status');const d=await r.json();applyTheme(Number(d.theme||0));const c=document.getElementById('clock');if(c)c.textContent=d.time||'--:--'}catch(e){}}
loadTheme();
</script>
)rawliteral";

String navHtml(const char* active) {
  // Use custom raw-string delimiter — plain R"( breaks on onclick="...()"
  String s = R"nav(
  <div class="topbar"><div class="brand"><img src="/logo.png" alt="" onerror="this.style.display='none'"><span>GeyserUnwise</span></div>
  <button class="hamburger" onclick="toggleMenu()"><span></span><span></span><span></span></button></div>
  <div class="overlay" id="overlay" onclick="toggleMenu()"></div>
  <aside class="drawer" id="drawer"><h2>Menu</h2><nav>
  <a href="/" class=")nav";
  if (!strcmp(active,"home")) s += "active";
  s += R"nav(">Home</a><a href="/connections" class=")nav";
  if (!strcmp(active,"connections")) s += "active";
  s += R"nav(">Connections</a><a href="/connections#ap" class="sub">AP</a><a href="/connections#mqtt" class="sub">MQTT</a>
  <a href="/time" class=")nav";
  if (!strcmp(active,"time")) s += "active";
  s += R"nav(">Time</a><a href="/temperature" class=")nav";
  if (!strcmp(active,"temperature")) s += "active";
  s += R"nav(">Temperature</a><a href="/schedule" class=")nav";
  if (!strcmp(active,"schedule")) s += "active";
  s += R"nav(">Schedule</a><a href="/stats" class=")nav";
  if (!strcmp(active,"stats")) s += "active";
  s += R"nav(">Stats / Graph</a><a href="/update" class=")nav";
  if (!strcmp(active,"update")) s += "active";
  s += R"nav(">Update</a></nav></aside>
)nav";
  return s;
}

const char INDEX_BODY[] PROGMEM = R"rawliteral(
<div class="card">
  <div id="fault" style="display:none" class="hint"><span class="badge">SENSOR FAULT</span> Heater locked off</div>
  <div class="dial-wrap" id="dial">
    <svg class="dial-svg" viewBox="0 0 260 260">
      <circle class="dial-track" cx="130" cy="130" r="105" stroke-dasharray="494" transform="rotate(135 130 130)"/>
      <circle class="dial-progress" id="arc" cx="130" cy="130" r="105" stroke-dasharray="494" stroke-dashoffset="494" transform="rotate(135 130 130)"/>
    </svg>
    <div class="dial-center">
      <div class="dial-temp" id="temp">--.-</div><div class="dial-unit">C</div>
      <div class="dial-target" id="target">Set 55.0</div>
      <div class="dial-action" id="action">idle</div>
    </div>
  </div>
  <div class="modes">
    <button class="mode-btn" data-mode="off" onclick="setMode('off')">Off</button>
    <button class="mode-btn" data-mode="auto" onclick="setMode('auto')">Auto</button>
    <button class="mode-btn" data-mode="boost" onclick="setMode('boost')">Boost</button>
  </div>
  <div class="presets">
    <button class="preset-btn" data-p="home" onclick="setPreset('home')">Home</button>
    <button class="preset-btn" data-p="away" onclick="setPreset('away')">Away</button>
    <button class="preset-btn" data-p="eco" onclick="setPreset('eco')">Eco</button>
  </div>
  <div class="hint" id="schedHint" style="margin-top:10px;text-align:center"></div>
</div>
<div class="footer"><div id="clock">--:--</div><div id="info"></div></div>
<script>
const MIN=30,MAX=100,SPAN=MAX-MIN,CIRC=494.8;let setpoint=55,dragging=false;
const arc=document.getElementById('arc'),dial=document.getElementById('dial');
function tempToOffset(t){return CIRC*(1-Math.max(0,Math.min(1,(t-MIN)/SPAN))*0.75)}
function setArc(t,h,b){arc.style.strokeDashoffset=tempToOffset(t);arc.className='dial-progress'+(b?' boost':h?' heating':'')}
function angleFromEvent(e){const r=dial.getBoundingClientRect(),cx=r.left+r.width/2,cy=r.top+r.height/2,pt=e.touches?e.touches[0]:e;let a=Math.atan2(pt.clientY-cy,pt.clientX-cx)*180/Math.PI;a=(a+360+135)%360;if(a>270)a=a>315?0:270;return a}
function onStart(e){dragging=true;onMove(e);e.preventDefault()}
function onMove(e){if(!dragging)return;const a=angleFromEvent(e);setpoint=Math.round((MIN+(a/270)*SPAN)*2)/2;document.getElementById('target').textContent='Set '+setpoint.toFixed(1);setArc(setpoint,false,false);e.preventDefault()}
function onEnd(){if(!dragging)return;dragging=false;fetch('/api/setpoint',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({setpoint})})}
dial.addEventListener('mousedown',onStart);dial.addEventListener('touchstart',onStart,{passive:false});
window.addEventListener('mousemove',onMove);window.addEventListener('touchmove',onMove,{passive:false});
window.addEventListener('mouseup',onEnd);window.addEventListener('touchend',onEnd);
async function setMode(m){await fetch('/api/mode',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({mode:m})});update()}
async function setPreset(p){await fetch('/api/preset',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({preset:p})});update()}
async function update(){try{
  const d=await(await fetch('/api/status')).json();
  const t=d.temperature!=null?Number(d.temperature):null;
  document.getElementById('temp').textContent=t!=null?t.toFixed(1):'--.-';
  setpoint=Number(d.setpoint);const tgt=d.mode==='boost'?Number(d.boost_temp):setpoint;
  document.getElementById('target').textContent=(d.mode==='boost'?'Boost ':'Set ')+tgt.toFixed(1);
  const act=document.getElementById('action');act.textContent=d.mode==='boost'?'boost':(d.heater?'heating':'idle');
  act.className='dial-action'+(d.mode==='boost'?' boost':d.heater?' heating':'');
  if(!dragging)setArc(tgt,d.heater,d.mode==='boost');
  document.querySelectorAll('.mode-btn').forEach(b=>b.classList.toggle('active',b.dataset.mode===d.mode));
  document.querySelectorAll('.preset-btn').forEach(b=>b.classList.toggle('active',b.dataset.p===d.preset));
  document.getElementById('fault').style.display=d.sensor_fault?'block':'none';
  document.getElementById('schedHint').textContent=d.schedule_enabled?(d.schedule_active?'Schedule: in window':'Schedule: outside window – heating blocked'):'';
  document.getElementById('clock').textContent=d.time||'--:--';
  document.getElementById('info').textContent=(d.ip||'')+' | v'+d.version;
  applyTheme(Number(d.theme||0));
}catch(e){}}
setInterval(update,2000);update();
</script></body></html>
)rawliteral";

const char CONNECTIONS_BODY[] PROGMEM = R"rawliteral(
<div class="card"><h1>Connections</h1>
<section id="ap"><h3 style="font-size:.8rem;color:var(--muted)">Access Point</h3>
<label class="row">Enable AP when online <input type="checkbox" id="ap_enabled"></label>
<label>AP Name</label><input type="text" id="ap_name" maxlength="31">
<label>AP Password</label><input type="password" id="ap_pass" placeholder="Leave blank to keep"></section>
<section id="mqtt" style="margin-top:16px"><h3 style="font-size:.8rem;color:var(--muted)">MQTT</h3>
<label class="row">Enable MQTT <input type="checkbox" id="mqtt_enabled"></label>
<label>Broker host</label><input type="text" id="mqtt_host">
<label>Port</label><input type="text" id="mqtt_port" value="1883">
<label>User</label><input type="text" id="mqtt_user">
<label>Password</label><input type="password" id="mqtt_pass" placeholder="Leave blank to keep">
<div class="hint" id="mqtt_status">-</div></section>
<label>Friendly name</label><input type="text" id="friendly_name" maxlength="39">
<label>Theme</label><select id="theme"><option value="0">Blue</option><option value="1">Dark</option><option value="2">Light</option></select>
<button class="save" onclick="save()">Save</button>
<div class="hint" style="margin-top:12px"><a href="/api/resetwifi" style="color:var(--primary-bright)" onclick="return confirm('Forget WiFi?')">Reset WiFi</a></div>
</div><div class="footer"><div id="clock">--:--</div></div>
<script>
async function load(){const d=await(await fetch('/api/status')).json();
document.getElementById('ap_enabled').checked=!!d.ap_enabled;document.getElementById('ap_name').value=d.ap_name||'';
document.getElementById('mqtt_enabled').checked=d.mqtt_enabled!==false;document.getElementById('mqtt_host').value=d.mqtt_host||'';
document.getElementById('mqtt_port').value=d.mqtt_port||'1883';document.getElementById('mqtt_user').value=d.mqtt_user||'';
document.getElementById('friendly_name').value=d.friendly_name||'';document.getElementById('theme').value=String(d.theme||0);
document.getElementById('mqtt_status').textContent=d.mqtt_enabled===false?'MQTT: disabled':(d.mqtt_host?('MQTT: '+(d.mqtt_connected?'ok':'down')):'not set');
document.getElementById('clock').textContent=d.time||'--:--';applyTheme(Number(d.theme||0))}
async function save(){await fetch('/api/settings',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({
ap_enabled:document.getElementById('ap_enabled').checked,ap_name:document.getElementById('ap_name').value,ap_pass:document.getElementById('ap_pass').value,
mqtt_enabled:document.getElementById('mqtt_enabled').checked,mqtt_host:document.getElementById('mqtt_host').value,mqtt_port:document.getElementById('mqtt_port').value,
mqtt_user:document.getElementById('mqtt_user').value,mqtt_pass:document.getElementById('mqtt_pass').value,
friendly_name:document.getElementById('friendly_name').value,theme:parseInt(document.getElementById('theme').value)||0})});
document.getElementById('mqtt_pass').value='';document.getElementById('ap_pass').value='';alert('Saved');load()}
load();
</script></body></html>
)rawliteral";

const char TIME_BODY[] PROGMEM = R"rawliteral(
<div class="card"><h1>Time</h1>
<label>Timezone offset (hours UTC)</label><input type="number" id="tz" min="-12" max="14" value="2">
<div class="hint">SAST = 2 | Current: <span id="clock2">--:--</span></div>
<button class="save" onclick="save()">Save</button></div>
<div class="footer"><div id="clock">--:--</div></div>
<script>
async function load(){const d=await(await fetch('/api/status')).json();
document.getElementById('tz').value=d.tz!=null?d.tz:2;document.getElementById('clock').textContent=d.time||'--:--';
document.getElementById('clock2').textContent=d.time||'--:--';applyTheme(Number(d.theme||0))}
async function save(){await fetch('/api/settings',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({tz:parseInt(document.getElementById('tz').value)||2})});alert('Saved');load()}
load();setInterval(load,30000);
</script></body></html>
)rawliteral";

const char TEMP_BODY[] PROGMEM = R"rawliteral(
<div class="card"><h1>Temperature</h1>
<label>Auto setpoint (°C)</label><input type="number" id="setpoint" min="30" max="100" step="0.5">
<label>Boost temperature (°C)</label><input type="number" id="boost_temp" min="30" max="100" step="0.5">
<label>Soft min for boost (°C, 0=off)</label><input type="number" id="boost_min" min="0" max="100" step="0.5">
<div class="hint">Boost only starts if current temp is below this (avoids useless boosts)</div>
<label>Hysteresis (°C under to re-on)</label><input type="number" id="hysteresis" min="0.5" max="20" step="0.5">
<label>Home preset (°C)</label><input type="number" id="home_temp" min="30" max="100" step="0.5">
<label>Away preset (°C)</label><input type="number" id="away_temp" min="30" max="100" step="0.5">
<label>Eco preset (°C)</label><input type="number" id="eco_temp" min="30" max="100" step="0.5">
<label>Min ON (s)</label><input type="number" id="min_on" min="5" max="600">
<label>Min OFF (s)</label><input type="number" id="min_off" min="5" max="600">
<button class="save" onclick="save()">Save</button></div>
<div class="footer"><div id="clock">--:--</div></div>
<script>
async function load(){const d=await(await fetch('/api/status')).json();
document.getElementById('setpoint').value=d.setpoint;document.getElementById('boost_temp').value=d.boost_temp;
document.getElementById('boost_min').value=d.boost_min_temp;document.getElementById('hysteresis').value=d.hysteresis;
document.getElementById('home_temp').value=d.home_temp;document.getElementById('away_temp').value=d.away_temp;
document.getElementById('eco_temp').value=d.eco_temp;
document.getElementById('min_on').value=Math.round((d.min_on_ms||30000)/1000);
document.getElementById('min_off').value=Math.round((d.min_off_ms||30000)/1000);
document.getElementById('clock').textContent=d.time||'--:--';applyTheme(Number(d.theme||0))}
async function save(){await fetch('/api/settings',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({
setpoint:+document.getElementById('setpoint').value,boost_temp:+document.getElementById('boost_temp').value,
boost_min_temp:+document.getElementById('boost_min').value,hysteresis:+document.getElementById('hysteresis').value,
home_temp:+document.getElementById('home_temp').value,away_temp:+document.getElementById('away_temp').value,
eco_temp:+document.getElementById('eco_temp').value,
min_on_ms:+document.getElementById('min_on').value*1000,min_off_ms:+document.getElementById('min_off').value*1000})});
alert('Saved');load()}
load();
</script></body></html>
)rawliteral";

const char SCHEDULE_BODY[] PROGMEM = R"rawliteral(
<div class="card"><h1>Schedule</h1>
<label class="row">Enable schedule <input type="checkbox" id="schedule_enabled"></label>
<div class="hint">When enabled, Auto only heats inside the window. Boost still works anytime. Outside window heater is forced off.</div>
<label>Heat allowed from (hour)</label><input type="number" id="on_h" min="0" max="23">
<label>From (minute)</label><input type="number" id="on_m" min="0" max="59">
<label>Until (hour)</label><input type="number" id="off_h" min="0" max="23">
<label>Until (minute)</label><input type="number" id="off_m" min="0" max="59">
<div class="hint">Overnight windows OK (e.g. 22:00 → 06:00)</div>
<button class="save" onclick="save()">Save</button>
<div class="hint" id="status" style="margin-top:12px"></div>
</div><div class="footer"><div id="clock">--:--</div></div>
<script>
async function load(){const d=await(await fetch('/api/status')).json();
document.getElementById('schedule_enabled').checked=!!d.schedule_enabled;
document.getElementById('on_h').value=d.sched_on_h;document.getElementById('on_m').value=d.sched_on_m;
document.getElementById('off_h').value=d.sched_off_h;document.getElementById('off_m').value=d.sched_off_m;
document.getElementById('status').textContent=d.schedule_enabled?(d.schedule_active?'Currently IN heating window':'Currently OUTSIDE heating window'):'Schedule off';
document.getElementById('clock').textContent=d.time||'--:--';applyTheme(Number(d.theme||0))}
async function save(){await fetch('/api/settings',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({
schedule_enabled:document.getElementById('schedule_enabled').checked,
sched_on_h:+document.getElementById('on_h').value,sched_on_m:+document.getElementById('on_m').value,
sched_off_h:+document.getElementById('off_h').value,sched_off_m:+document.getElementById('off_m').value})});
alert('Saved');load()}
load();setInterval(load,15000);
</script></body></html>
)rawliteral";

const char STATS_BODY[] PROGMEM = R"rawliteral(
<div class="card"><h1>Stats & temperature</h1>
<div class="stats">
  <div><div class="n" id="rtoday">-</div><div class="l">Today (h)</div></div>
  <div><div class="n" id="rweek">-</div><div class="l">Week (h)</div></div>
  <div><div class="n" id="rtotal">-</div><div class="l">Total (h)</div></div>
</div>
<canvas class="chart" id="chart" width="400" height="160"></canvas>
<div class="hint" id="chartHint">Loading history…</div>
</div><div class="footer"><div id="clock">--:--</div></div>
<script>
function drawChart(temps){
  const c=document.getElementById('chart'),ctx=c.getContext('2d');
  const w=c.width,h=c.height;ctx.clearRect(0,0,w,h);
  if(!temps||temps.length<2){document.getElementById('chartHint').textContent='Not enough samples yet';return}
  let mn=Math.min(...temps),mx=Math.max(...temps);if(mx-mn<2){mn-=1;mx+=1}
  ctx.strokeStyle=getComputedStyle(document.body).getPropertyValue('--track')||'#123050';
  ctx.beginPath();ctx.moveTo(0,h-1);ctx.lineTo(w,h-1);ctx.stroke();
  ctx.strokeStyle=getComputedStyle(document.body).getPropertyValue('--primary-bright')||'#3b9eff';
  ctx.lineWidth=2;ctx.beginPath();
  temps.forEach((t,i)=>{const x=i/(temps.length-1)*w,y=h-4-((t-mn)/(mx-mn))*(h-12);
    if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y)});
  ctx.stroke();
  document.getElementById('chartHint').textContent=temps.length+' samples · min '+mn.toFixed(1)+' · max '+mx.toFixed(1)+' °C';
}
async function load(){
  const d=await(await fetch('/api/status')).json();
  document.getElementById('rtoday').textContent=(d.runtime_today_sec/3600).toFixed(2);
  document.getElementById('rweek').textContent=(d.runtime_week_sec/3600).toFixed(2);
  document.getElementById('rtotal').textContent=(d.runtime_total_sec/3600).toFixed(1);
  document.getElementById('clock').textContent=d.time||'--:--';applyTheme(Number(d.theme||0));
  const h=await(await fetch('/api/history')).json();
  drawChart((h.temps||[]).map(Number));
}
load();setInterval(load,10000);
</script></body></html>
)rawliteral";

const char UPDATE_BODY[] PROGMEM = R"rawliteral(
<div class="card"><h1>Update & reset</h1>
<p class="hint">Version: <strong id="ver">-</strong></p>
<p class="hint" id="ota_msg">-</p>
<label>GitHub owner</label><input type="text" id="github_owner">
<label>GitHub repo</label><input type="text" id="github_repo">
<div class="hint">Release asset must be <code>firmware.bin</code></div>
<label class="row">Auto-check on boot <input type="checkbox" id="ota_auto"></label>
<button class="save" onclick="saveGh()">Save GitHub</button>
<button class="save" style="background:#0d9488" onclick="check()">Check update</button>
<button class="save danger" onclick="install()">Install update</button>
<hr style="border:none;border-top:1px solid var(--track);margin:20px 0">
<button class="save danger" onclick="factory()">Factory reset</button>
<div class="hint">Clears all settings + WiFi. Or hold FLASH (GPIO0) for 5s at boot.</div>
<div class="hint" style="margin-top:12px">Local OTA: PlatformIO → Geyser-Unwise.local</div>
</div><div class="footer"><div id="clock">--:--</div></div>
<script>
async function load(){const d=await(await fetch('/api/status')).json();
document.getElementById('ver').textContent=d.version;document.getElementById('ota_msg').textContent=d.ota_msg||'Idle';
document.getElementById('github_owner').value=d.github_owner||'';document.getElementById('github_repo').value=d.github_repo||'';
document.getElementById('ota_auto').checked=!!d.ota_auto;document.getElementById('clock').textContent=d.time||'--:--';
applyTheme(Number(d.theme||0))}
async function saveGh(){await fetch('/api/settings',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({
github_owner:document.getElementById('github_owner').value,github_repo:document.getElementById('github_repo').value,
ota_auto:document.getElementById('ota_auto').checked})});alert('Saved');load()}
async function check(){document.getElementById('ota_msg').textContent='Checking...';
const d=await(await fetch('/api/ota/check')).json();document.getElementById('ota_msg').textContent=d.msg}
async function install(){if(!confirm('Install from GitHub and reboot?'))return;
document.getElementById('ota_msg').textContent='Installing...';await fetch('/api/ota/install',{method:'POST'});setTimeout(load,5000)}
async function factory(){if(!confirm('FACTORY RESET all settings and WiFi?'))return;
await fetch('/api/factoryreset',{method:'POST'})}
load();
</script></body></html>
)rawliteral";

void sendPage(AsyncWebServerRequest *req, const char* active, const char* body) {
  String html;
  html.reserve(14000);
  html += FPSTR(SHARED_HEAD);
  html += FPSTR(NAV_SCRIPT);
  html += navHtml(active);
  html += FPSTR(body);
  req->send(200, "text/html", html);
}

void setupWebUI() {
  server.on("/logo.png", HTTP_GET, [](AsyncWebServerRequest *req) {
    if (LittleFS.exists("/logo.png")) {
      AsyncWebServerResponse *res = req->beginResponse(LittleFS, "/logo.png", "image/png");
      res->addHeader("Cache-Control", "public, max-age=86400");
      req->send(res);
    } else req->send(404);
  });
  server.on("/connections", HTTP_GET, [](AsyncWebServerRequest *req) { sendPage(req, "connections", CONNECTIONS_BODY); });
  server.on("/time", HTTP_GET, [](AsyncWebServerRequest *req) { sendPage(req, "time", TIME_BODY); });
  server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *req) { sendPage(req, "temperature", TEMP_BODY); });
  server.on("/schedule", HTTP_GET, [](AsyncWebServerRequest *req) { sendPage(req, "schedule", SCHEDULE_BODY); });
  server.on("/stats", HTTP_GET, [](AsyncWebServerRequest *req) { sendPage(req, "stats", STATS_BODY); });
  server.on("/update", HTTP_GET, [](AsyncWebServerRequest *req) { sendPage(req, "update", UPDATE_BODY); });
}

// ---- Portal ----
DNSServer dnsServer;
bool portalActive = false;
String scannedNetworksJson = "[]";
bool pendingConnect = false;
String pendingSsid, pendingPass;
unsigned long pendingConnectAt = 0;

void startScan() {
  int n = WiFi.scanNetworks(false, false);
  struct Net { String ssid; int32_t rssi; bool secure; };
  Net nets[32]; int count = 0;
  for (int i = 0; i < n && count < 32; i++) {
    String ssid = WiFi.SSID(i), clean;
    for (unsigned j = 0; j < ssid.length(); j++) if (ssid[j] >= 32 && ssid[j] < 127) clean += ssid[j];
    ssid = clean; ssid.trim(); if (!ssid.length()) continue;
    int32_t rssi = WiFi.RSSI(i); bool secure = WiFi.encryptionType(i) != ENC_TYPE_NONE;
    int found = -1;
    for (int j = 0; j < count; j++) if (nets[j].ssid == ssid) { found = j; break; }
    if (found >= 0) { if (rssi > nets[found].rssi) { nets[found].rssi = rssi; nets[found].secure = secure; } }
    else nets[count++] = {ssid, rssi, secure};
  }
  for (int i = 0; i < count; i++) for (int j = i + 1; j < count; j++)
    if (nets[j].rssi > nets[i].rssi) { Net t = nets[i]; nets[i] = nets[j]; nets[j] = t; }
  JsonDocument doc; JsonArray arr = doc.to<JsonArray>();
  for (int i = 0; i < count; i++) {
    if (nets[i].rssi < -90) continue;
    JsonObject o = arr.add<JsonObject>();
    o["ssid"] = nets[i].ssid; o["rssi"] = nets[i].rssi; o["secure"] = nets[i].secure;
  }
  scannedNetworksJson = ""; serializeJson(doc, scannedNetworksJson);
}

const char PORTAL_HTML[] PROGMEM = R"portal(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Setup</title><style>
body{font-family:system-ui;background:#05080f;color:#e8f0fa;margin:0;padding:20px}
.card{background:#0a1628;border-radius:16px;padding:24px;max-width:400px;margin:0 auto}
h1{color:#3b9eff;text-align:center}label{display:block;font-size:.72rem;color:#6b8aad;margin:14px 0 6px}
input,select{width:100%;padding:12px;border-radius:10px;border:1px solid #123050;background:#0e243f;color:#e8f0fa;box-sizing:border-box}
button{width:100%;margin-top:20px;padding:14px;border:none;border-radius:12px;background:#1e6bb8;color:#fff;font-weight:700}
.hint{font-size:.75rem;color:#6b8aad;text-align:center;margin-top:8px}
</style></head><body><div class="card"><h1>GeyserUnwise</h1>
<label>Network</label><select id="ssid"><option>Scanning...</option></select>
<label>Password</label><input type="password" id="pass">
<button id="btn" onclick="save()">Connect</button><p class="hint" id="hint"></p></div>
<script>
async function load(){try{const arr=await(await fetch('/scan')).json();const s=document.getElementById('ssid');s.innerHTML='';
arr.forEach(n=>{const o=document.createElement('option');o.value=n.ssid;o.textContent=n.ssid;s.appendChild(o)});
document.getElementById('hint').textContent=arr.length+' networks'}catch(e){}}
async function save(){const ssid=document.getElementById('ssid').value,pass=document.getElementById('pass').value;
document.getElementById('btn').disabled=true;await fetch('/connect?ssid='+encodeURIComponent(ssid)+'&pass='+encodeURIComponent(pass));
document.body.innerHTML='<div class="card"><h1>Connecting...</h1></div>'}
load();setInterval(load,15000);
</script></body></html>
)portal";

void setupPortalRoutes() {
  server.on("/scan", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send(200, "application/json", scannedNetworksJson);
  });
  server.on("/connect", HTTP_GET, [](AsyncWebServerRequest *req) {
    if (!req->hasParam("ssid")) { req->send(400); return; }
    pendingSsid = req->getParam("ssid")->value();
    pendingPass = req->hasParam("pass") ? req->getParam("pass")->value() : "";
    strncpy(wifi_ssid, pendingSsid.c_str(), sizeof(wifi_ssid)-1);
    strncpy(wifi_pass, pendingPass.c_str(), sizeof(wifi_pass)-1);
    saveSettings();
    pendingConnect = true; pendingConnectAt = millis();
    req->send(200, "text/plain", "ok");
  });
  server.onNotFound([](AsyncWebServerRequest *req) {
    if (portalActive) req->redirect("http://192.168.4.1/");
    else req->send(404);
  });
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    if (portalActive) req->send_P(200, "text/html", PORTAL_HTML);
    else sendPage(req, "home", INDEX_BODY);
  });
}

bool tryConnectSaved(unsigned long timeoutMs = 15000) {
  if (!strlen(wifi_ssid)) return false;
  WiFi.persistent(true); WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_pass);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) { delay(200); yield(); }
  return WiFi.status() == WL_CONNECTED;
}

void startCaptivePortal() {
  portalActive = true; WiFi.mode(WIFI_AP);
  if (strlen(apName) < 1) strncpy(apName, AP_NAME, sizeof(apName)-1);
  if (strlen(apPass) < 8) strncpy(apPass, AP_PASSWORD, sizeof(apPass)-1);
  WiFi.softAP(apName, apPass); delay(200);
  dnsServer.start(53, "*", WiFi.softAPIP());
  startScan();
}

void setupWiFi() {
  if (tryConnectSaved()) {
    portalActive = false;
    if (!apEnabled || hideAp) WiFi.mode(WIFI_STA);
    else { WiFi.mode(WIFI_AP_STA); WiFi.softAP(apName, apPass); }
    setupNTP(); setupMDNS(); updateWifiLed();
  } else {
    startCaptivePortal(); updateWifiLed();
  }
}

void checkFactoryResetPin() {
  pinMode(FACTORY_RESET_PIN, INPUT_PULLUP);
  if (digitalRead(FACTORY_RESET_PIN) != LOW) return;
  Serial.println("[RESET] Hold FLASH 5s for factory reset...");
  unsigned long t0 = millis();
  while (digitalRead(FACTORY_RESET_PIN) == LOW) {
    if (millis() - t0 > 5000) { factoryReset(); return; }
    delay(50);
  }
}

void setup() {
  Serial.begin(115200); delay(200);
  Serial.println("\n=== Geyser Controller v3.1.2 ===");
  pinMode(RELAY_PIN, OUTPUT); digitalWrite(RELAY_PIN, LOW);
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, HIGH);
  checkFactoryResetPin();

  if (!LittleFS.begin()) Serial.println("[FS] fail");
  Wire.begin(I2C_SDA, I2C_SCL);
  if (ads.begin(0x48)) { ads.setGain(GAIN_ONE); Serial.println("[ADS] ok"); }

  deviceId = String(ESP.getChipId(), HEX);
  topicBase = String(MQTT_BASE_TOPIC) + "/" + deviceId;
  topicState = topicBase + "/state";
  topicCmd = topicBase + "/cmd";
  topicAvail = topicBase + "/status";

  loadSettings();
  setupWiFi();
  setupAPI();
  setupWebUI();
  setupPortalRoutes();
  server.begin();

  if (!portalActive && WiFi.status() == WL_CONNECTED) {
    setupArduinoOTA();
    if (otaAutoCheck) checkGitHubUpdate(true);
  }
}

void loop() {
  if (portalActive) {
    dnsServer.processNextRequest();
    if (pendingConnect && millis() - pendingConnectAt > 300) {
      pendingConnect = false;
      WiFi.mode(WIFI_AP_STA);
      WiFi.begin(pendingSsid.c_str(), pendingPass.c_str());
      unsigned long start = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - start < 25000) {
        delay(300); yield(); dnsServer.processNextRequest();
      }
      if (WiFi.status() == WL_CONNECTED) {
        strncpy(wifi_ssid, pendingSsid.c_str(), sizeof(wifi_ssid)-1);
        strncpy(wifi_pass, pendingPass.c_str(), sizeof(wifi_pass)-1);
        saveSettings(); WiFi.persistent(true); WiFi.begin(wifi_ssid, wifi_pass);
        delay(500); ESP.restart();
      } else startScan();
    }
    updateWifiLed();
    return;
  }

  ArduinoOTA.handle();
  mqttLoop();
  MDNS.update();
  updateRuntimeTick();

  if (millis() - lastLedCheck >= WIFI_LED_UPDATE_MS) {
    lastLedCheck = millis();
    updateWifiLed();
  }

  if (millis() - lastRead >= READ_INTERVAL_MS) {
    lastRead = millis();
    currentTemp = readTemperature();
    updateThermostat();
    updateTimeString();
    updateDayWeekCounters();
    if (millis() - lastHistoryAt >= HISTORY_INTERVAL_MS) {
      lastHistoryAt = millis();
      pushHistory(currentTemp);
    }
    publishState();
    Serial.printf("T=%.1f Set=%.1f Mode=%s H=%s Fault=%d\n",
                  currentTemp, setpoint, MODE_NAMES[mode], heaterOn ? "ON" : "OFF", sensorFault);
  }
}
