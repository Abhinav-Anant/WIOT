/*
 * WIOT -- municipal water auto-fill controller for Motor 2
 * ESP32-WROOM-32 / Arduino framework
 *
 * The municipal line is dead until the pump pulls on it, so arrival can only be
 * detected by briefly running the pump and watching the flow sensor. If water
 * is there, keep pumping until the supply dies or the tank fills.
 *
 * All control logic is local. WiFi/MQTT is telemetry and override only -- if
 * the router is down at 3am the pump still fills the tank.
 *
 * See README.md for why each threshold is what it is.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <time.h>

#include "config.h"
#include "secrets.h"
#include "levelmath.h"

// ---------------------------------------------------------------- settings

struct Settings {
  uint16_t pollIntervalMin;
  uint16_t testDurationS;
  uint16_t cooldownHours;
  float    flowThresholdLpm;
  uint16_t lowFlowWindowS;
  uint16_t maxRuntimeMin;
  float    pulsesPerLitre;
  uint16_t tankEmptyCm;
  uint16_t tankFullCm;
  uint32_t tankFullLitres;
  uint16_t minSessionLitres;
  bool     autoEnabled;
  bool     stopOnSensorFail;
};

static Settings cfg;
static Preferences nvs;

static void loadSettings() {
  nvs.begin("wiot", false);
  cfg.pollIntervalMin   = nvs.getUShort("poll",     DEF_POLL_MIN);
  cfg.testDurationS     = nvs.getUShort("test",     DEF_TEST_S);
  cfg.cooldownHours     = nvs.getUShort("cool",     DEF_COOLDOWN_H);
  cfg.flowThresholdLpm  = nvs.getFloat ("thresh",   DEF_FLOW_THRESH_LPM);
  cfg.lowFlowWindowS    = nvs.getUShort("lfwin",    DEF_LOWFLOW_WIN_S);
  cfg.maxRuntimeMin     = nvs.getUShort("maxrun",   DEF_MAX_RUNTIME_MIN);
  cfg.pulsesPerLitre    = nvs.getFloat ("ppl",      DEF_PULSES_PER_L);
  cfg.tankEmptyCm       = nvs.getUShort("empty",    DEF_TANK_EMPTY_CM);
  cfg.tankFullCm        = nvs.getUShort("full",     DEF_TANK_FULL_CM);
  cfg.tankFullLitres    = nvs.getULong ("tankl",    DEF_TANK_FULL_L);
  cfg.minSessionLitres  = nvs.getUShort("minsess",  DEF_MIN_SESSION_L);
  cfg.autoEnabled       = nvs.getBool  ("auto",     true);
  cfg.stopOnSensorFail  = nvs.getBool  ("sfstop",   false);
}

static void saveSettings() {
  nvs.putUShort("poll",    cfg.pollIntervalMin);
  nvs.putUShort("test",    cfg.testDurationS);
  nvs.putUShort("cool",    cfg.cooldownHours);
  nvs.putFloat ("thresh",  cfg.flowThresholdLpm);
  nvs.putUShort("lfwin",   cfg.lowFlowWindowS);
  nvs.putUShort("maxrun",  cfg.maxRuntimeMin);
  nvs.putFloat ("ppl",     cfg.pulsesPerLitre);
  nvs.putUShort("empty",   cfg.tankEmptyCm);
  nvs.putUShort("full",    cfg.tankFullCm);
  nvs.putULong ("tankl",   cfg.tankFullLitres);
  nvs.putUShort("minsess", cfg.minSessionLitres);
  nvs.putBool  ("auto",    cfg.autoEnabled);
  nvs.putBool  ("sfstop",  cfg.stopOnSensorFail);
}

// ------------------------------------------------------------- global state

enum State : uint8_t { ST_IDLE, ST_TESTING, ST_PUMPING, ST_COOLDOWN, ST_FAULT };
static const char* STATE_NAME[] = { "idle", "testing", "pumping", "cooldown", "fault" };

static State    state         = ST_IDLE;
static uint32_t stateSinceMs  = 0;
static bool     relayOn       = false;

static float    flowLpm       = 0.0f;
static float    sessionLitres = 0.0f;
static uint32_t sessionStartMs = 0;

static uint16_t distanceCm    = 0;      // 0 = no valid echo
static float    tankPercent   = 0.0f;
static uint32_t tankLitres    = 0;
static bool     floatFull     = false;
static bool     sensorOk      = true;
static uint32_t lastGoodUsMs  = 0;

static uint32_t todayLitres     = 0;
static uint32_t todayRuntimeSec = 0;
static uint16_t todayDryStarts  = 0;
static int      lastYday        = -1;
static time_t   lastArrival     = 0;

static uint32_t lastTestMs      = 0;
static uint32_t lowFlowSinceMs  = 0;
static const char* lastStopReason = "none";
static const char* faultReason    = "";

static bool forceTestReq = false;
static bool forceStopReq = false;

// ------------------------------------------------------------- flow sensor

static volatile uint32_t pulseCount = 0;
static portMUX_TYPE pulseMux = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR flowISR() {
  portENTER_CRITICAL_ISR(&pulseMux);
  pulseCount++;
  portEXIT_CRITICAL_ISR(&pulseMux);
}

// Called once per second.
static void updateFlow() {
  portENTER_CRITICAL(&pulseMux);
  uint32_t p = pulseCount;
  pulseCount = 0;
  portEXIT_CRITICAL(&pulseMux);

  flowLpm = wiot::lpmFromPulses(p, cfg.pulsesPerLitre);
  float litres = flowLpm / 60.0f;

  if (relayOn) {
    sessionLitres += litres;
    todayLitres   += (uint32_t)litres;
  }
}

// -------------------------------------------------------------- ultrasonic

static uint16_t readDistanceOnce() {
  digitalWrite(PIN_US_TRIG, LOW);
  delayMicroseconds(4);
  digitalWrite(PIN_US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_US_TRIG, LOW);

  unsigned long us = pulseIn(PIN_US_ECHO, HIGH, 40000UL);
  if (us == 0) return 0;

  uint32_t cm = us / 58;
  if (cm > 0xFFFF) return 0;
  if (!wiot::validDistance((uint16_t)cm, US_MIN_CM, US_MAX_CM)) return 0;  // blind zone / out of range
  return (uint16_t)cm;
}

static uint16_t readDistance() {
  uint16_t s[5] = {0};
  for (int i = 0; i < 5; i++) {
    s[i] = readDistanceOnce();
    delay(60);
  }
  return wiot::medianOf5(s);
}

static void updateLevel() {
  uint16_t d = readDistance();
  if (d > 0) {
    distanceCm   = d;
    lastGoodUsMs = millis();
    sensorOk     = true;

    tankPercent = wiot::levelPercent(d, cfg.tankEmptyCm, cfg.tankFullCm);
    tankLitres  = wiot::litresFromPercent(tankPercent, cfg.tankFullLitres);
  } else if (millis() - lastGoodUsMs > US_FAIL_ALERT_MS) {
    sensorOk = false;
  }
}

// ------------------------------------------------------------------ outputs

static void setRelay(bool on) {
  digitalWrite(PIN_RELAY, on ? RELAY_ON : RELAY_OFF);
  relayOn = on;
}

static void setState(State s) {
  state = s;
  stateSinceMs = millis();
}

// ------------------------------------------------------------ state machine

static bool tankIsFull() {
  // Float is the authority -- it is also the hardware interlock on the coil.
  // The ultrasonic is a second opinion and may be blind near the top.
  return floatFull || (sensorOk && tankPercent >= 99.0f);
}

static void startTest() {
  sessionLitres  = 0.0f;
  sessionStartMs = millis();
  lowFlowSinceMs = 0;
  setRelay(true);
  setState(ST_TESTING);
  Serial.println(F("[fsm] TESTING -- pulsing pump to probe the line"));
}

static void stopPump(const char* reason) {
  setRelay(false);
  lastStopReason = reason;
  todayRuntimeSec += (millis() - sessionStartMs) / 1000;
  lastTestMs = millis();

  // A stop after only a trickle was a sputter, not a delivery. Going into a
  // 6-hour cooldown on that would make us miss the real supply half an hour
  // later, so only a meaningful session earns the long rest.
  if (sessionLitres >= cfg.minSessionLitres) {
    setState(ST_COOLDOWN);
    Serial.printf("[fsm] COOLDOWN after %.0f L (%s)\n", sessionLitres, reason);
  } else {
    setState(ST_IDLE);
    Serial.printf("[fsm] IDLE after %.0f L (%s)\n", sessionLitres, reason);
  }

  nvs.putULong("tdl",  todayLitres);
  nvs.putULong("tdr",  todayRuntimeSec);
  nvs.putUShort("tds", todayDryStarts);
  nvs.putULong("arr",  (uint32_t)lastArrival);
}

static void enterFault(const char* why) {
  setRelay(false);
  faultReason = why;
  setState(ST_FAULT);
  Serial.printf("[fsm] FAULT: %s\n", why);
}

static void tickStateMachine() {
  uint32_t now = millis();

  // Disagreement between the two level sensors means one of them is lying and
  // we no longer know how full the tank is. Refuse to pump until a human looks.
  if (floatFull && sensorOk && tankPercent < 50.0f && state != ST_FAULT) {
    enterFault("float says full but level reads low");
    return;
  }

  if (forceStopReq) {
    forceStopReq = false;
    if (relayOn) stopPump("manual stop");
    return;
  }

  switch (state) {

    case ST_IDLE: {
      if (forceTestReq) { forceTestReq = false; startTest(); break; }
      if (!cfg.autoEnabled) break;
      if (tankIsFull()) { lastTestMs = now; break; }   // no point probing
      if (now - lastTestMs >= (uint32_t)cfg.pollIntervalMin * 60000UL) startTest();
      break;
    }

    case ST_TESTING: {
      uint32_t elapsed = now - stateSinceMs;
      // First 4s is priming -- the pump has to pull the column up before any
      // flow appears, so judging it earlier would report a false dry line.
      if (elapsed >= 4000 && flowLpm >= cfg.flowThresholdLpm) {
        lastArrival = time(nullptr);
        setState(ST_PUMPING);
        Serial.printf("[fsm] PUMPING -- water detected at %.1f L/min\n", flowLpm);
        break;
      }
      if (elapsed >= (uint32_t)cfg.testDurationS * 1000UL) {
        todayDryStarts++;
        stopPump("dry line");
      }
      break;
    }

    case ST_PUMPING: {
      if (forceTestReq) forceTestReq = false;   // already running

      if (tankIsFull())      { stopPump("tank full"); break; }
      if (!sensorOk && cfg.stopOnSensorFail) { stopPump("level sensor failed"); break; }

      if (now - sessionStartMs >= (uint32_t)cfg.maxRuntimeMin * 60000UL) {
        stopPump("max runtime");
        break;
      }

      // Municipal supply dies gradually and sputters on the way out. Require a
      // sustained low reading so a momentary dip doesn't chatter the contactor.
      if (flowLpm < cfg.flowThresholdLpm) {
        if (lowFlowSinceMs == 0) lowFlowSinceMs = now;
        else if (now - lowFlowSinceMs >= (uint32_t)cfg.lowFlowWindowS * 1000UL) {
          stopPump("supply ended");
        }
      } else {
        lowFlowSinceMs = 0;
      }
      break;
    }

    case ST_COOLDOWN: {
      if (forceTestReq) { forceTestReq = false; startTest(); break; }
      if (now - stateSinceMs >= (uint32_t)cfg.cooldownHours * 3600000UL) {
        setState(ST_IDLE);
        lastTestMs = now - (uint32_t)cfg.pollIntervalMin * 60000UL;  // probe now
      }
      break;
    }

    case ST_FAULT:
      forceTestReq = false;
      if (relayOn) setRelay(false);
      break;
  }
}

// ----------------------------------------------------------------- LED / UI

static void tickLed() {
  uint32_t t = millis();
  bool on;
  switch (state) {
    case ST_PUMPING:  on = true;                    break;   // solid
    case ST_TESTING:  on = (t % 200)  < 100;        break;   // fast blink
    case ST_FAULT:    on = (t % 400)  < 100;        break;   // stutter
    case ST_COOLDOWN: on = (t % 3000) < 100;        break;   // rare wink
    default:          on = (t % 2000) < 100;        break;   // slow blink
  }
  digitalWrite(PIN_LED, on ? HIGH : LOW);
}

static void tickButton() {
  static bool     down = false;
  static uint32_t downAt = 0;

  bool pressed = (digitalRead(PIN_BUTTON) == LOW);

  if (pressed && !down) { down = true; downAt = millis(); }

  if (!pressed && down) {
    down = false;
    uint32_t held = millis() - downAt;
    if (held < 50) return;                       // debounce
    if (held >= 3000) {                          // long press: clear fault
      if (state == ST_FAULT) { faultReason = ""; setState(ST_IDLE); }
    } else {                                     // short press: test / stop
      if (relayOn) forceStopReq = true;
      else         forceTestReq = true;
    }
  }
}

// --------------------------------------------------------------- networking

static WiFiClient   net;
static PubSubClient mqtt(net);
static WebServer    web(80);

static void buildStateJson(JsonDocument& d) {
  d["state"]             = STATE_NAME[state];
  d["pump_on"]           = relayOn ? 1 : 0;
  d["tank_percent"]      = roundf(tankPercent * 10) / 10.0f;
  d["tank_litres"]       = tankLitres;
  d["distance_cm"]       = distanceCm;
  d["flow_lpm"]          = roundf(flowLpm * 10) / 10.0f;
  d["session_litres"]    = (uint32_t)sessionLitres;
  d["today_litres"]      = todayLitres;
  d["runtime_today_min"] = todayRuntimeSec / 60;
  d["dry_starts_today"]  = todayDryStarts;
  d["float_full"]        = floatFull ? 1 : 0;
  d["sensor_ok"]         = sensorOk ? 1 : 0;
  d["last_stop"]         = lastStopReason;
  d["fault"]             = faultReason;
  d["rssi"]              = WiFi.RSSI();
  d["uptime_min"]        = millis() / 60000UL;

  d["poll_interval_min"] = cfg.pollIntervalMin;
  d["test_duration_s"]   = cfg.testDurationS;
  d["cooldown_hours"]    = cfg.cooldownHours;
  d["flow_threshold"]    = cfg.flowThresholdLpm;
  d["lowflow_window_s"]  = cfg.lowFlowWindowS;
  d["max_runtime_min"]   = cfg.maxRuntimeMin;
  d["pulses_per_litre"]  = cfg.pulsesPerLitre;
  d["tank_empty_cm"]     = cfg.tankEmptyCm;
  d["tank_full_cm"]      = cfg.tankFullCm;
  d["auto_enabled"]      = cfg.autoEnabled ? 1 : 0;

  if (lastArrival > 0) {
    char buf[26];
    struct tm tmv;
    localtime_r(&lastArrival, &tmv);
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M", &tmv);
    d["last_arrival"] = buf;
  } else {
    d["last_arrival"] = "never";
  }
}

static void publishState() {
  if (!mqtt.connected()) return;
  JsonDocument d;
  buildStateJson(d);
  char buf[900];
  size_t n = serializeJson(d, buf, sizeof(buf));
  mqtt.publish(MQTT_BASE "/state", (const uint8_t*)buf, n, true);
}

// ---- Home Assistant discovery -------------------------------------------

static void devBlock(JsonDocument& d) {
  JsonObject dev = d["dev"].to<JsonObject>();
  dev["ids"].to<JsonArray>().add(DEVICE_ID);
  dev["name"] = DEVICE_NAME;
  dev["mf"]   = "DIY";
  dev["mdl"]  = "WIOT ESP32";
}

static void discSensor(const char* key, const char* name,
                       const char* unit, const char* devcla, const char* icon) {
  JsonDocument d;
  d["name"]     = name;
  d["uniq_id"]  = String(DEVICE_ID) + "_" + key;
  d["stat_t"]   = MQTT_BASE "/state";
  d["val_tpl"]  = String("{{ value_json.") + key + " }}";
  d["avty_t"]   = MQTT_BASE "/availability";
  if (unit)   { d["unit_of_meas"] = unit; d["stat_cla"] = "measurement"; }
  if (devcla) d["dev_cla"] = devcla;
  if (icon)   d["ic"] = icon;
  devBlock(d);

  char topic[128];
  snprintf(topic, sizeof(topic), MQTT_HA_PREFIX "/sensor/" DEVICE_ID "/%s/config", key);
  char buf[700];
  size_t n = serializeJson(d, buf, sizeof(buf));
  mqtt.publish(topic, (const uint8_t*)buf, n, true);
}

static void discBinary(const char* key, const char* name, const char* devcla) {
  JsonDocument d;
  d["name"]    = name;
  d["uniq_id"] = String(DEVICE_ID) + "_" + key;
  d["stat_t"]  = MQTT_BASE "/state";
  d["val_tpl"] = String("{{ value_json.") + key + " }}";
  d["avty_t"]  = MQTT_BASE "/availability";
  d["pl_on"]   = "1";
  d["pl_off"]  = "0";
  if (devcla) d["dev_cla"] = devcla;
  devBlock(d);

  char topic[128];
  snprintf(topic, sizeof(topic), MQTT_HA_PREFIX "/binary_sensor/" DEVICE_ID "/%s/config", key);
  char buf[700];
  size_t n = serializeJson(d, buf, sizeof(buf));
  mqtt.publish(topic, (const uint8_t*)buf, n, true);
}

static void discNumber(const char* key, const char* name,
                       float mn, float mx, float step, const char* unit) {
  JsonDocument d;
  d["name"]    = name;
  d["uniq_id"] = String(DEVICE_ID) + "_" + key;
  d["stat_t"]  = MQTT_BASE "/state";
  d["val_tpl"] = String("{{ value_json.") + key + " }}";
  d["cmd_t"]   = String(MQTT_BASE "/set/") + key;
  d["avty_t"]  = MQTT_BASE "/availability";
  d["min"] = mn; d["max"] = mx; d["step"] = step;
  d["mode"] = "box";
  d["ent_cat"] = "config";
  if (unit) d["unit_of_meas"] = unit;
  devBlock(d);

  char topic[128];
  snprintf(topic, sizeof(topic), MQTT_HA_PREFIX "/number/" DEVICE_ID "/%s/config", key);
  char buf[700];
  size_t n = serializeJson(d, buf, sizeof(buf));
  mqtt.publish(topic, (const uint8_t*)buf, n, true);
}

static void discButton(const char* cmd, const char* name, const char* icon) {
  JsonDocument d;
  d["name"]    = name;
  d["uniq_id"] = String(DEVICE_ID) + "_" + cmd;
  d["cmd_t"]   = MQTT_BASE "/cmd";
  d["pl_prs"]  = cmd;
  d["avty_t"]  = MQTT_BASE "/availability";
  if (icon) d["ic"] = icon;
  devBlock(d);

  char topic[128];
  snprintf(topic, sizeof(topic), MQTT_HA_PREFIX "/button/" DEVICE_ID "/%s/config", cmd);
  char buf[700];
  size_t n = serializeJson(d, buf, sizeof(buf));
  mqtt.publish(topic, (const uint8_t*)buf, n, true);
}

static void publishDiscovery() {
  discSensor("state",             "Pump State",        nullptr, nullptr, "mdi:state-machine");
  discSensor("tank_percent",      "Tank Level",        "%",     nullptr, "mdi:car-coolant-level");
  discSensor("tank_litres",       "Tank Volume",       "L",     "water", nullptr);
  discSensor("flow_lpm",          "Flow Rate",         "L/min", nullptr, "mdi:water-pump");
  discSensor("session_litres",    "This Fill",         "L",     "water", nullptr);
  discSensor("today_litres",      "Today Delivered",   "L",     "water", nullptr);
  discSensor("runtime_today_min", "Pump Runtime Today","min",   nullptr, "mdi:timer");
  discSensor("dry_starts_today",  "Dry Starts Today",  nullptr, nullptr, "mdi:water-off");
  discSensor("last_arrival",      "Last Water Arrival",nullptr, nullptr, "mdi:clock-check");
  discSensor("last_stop",         "Last Stop Reason",  nullptr, nullptr, "mdi:information");
  discSensor("rssi",              "WiFi Signal",       "dBm", "signal_strength", nullptr);

  discBinary("pump_on",    "Pump Running",       "running");
  discBinary("float_full", "Tank Full Float",    nullptr);
  discBinary("sensor_ok",  "Level Sensor OK",    nullptr);

  discNumber("poll_interval_min", "Poll Interval",       1,   240,  1,   "min");
  discNumber("test_duration_s",   "Test Pulse Length",   3,    60,  1,   "s");
  discNumber("cooldown_hours",    "Cooldown After Fill", 0,    24,  1,   "h");
  discNumber("flow_threshold",    "Flow Threshold",      0.5f, 50,  0.5f,"L/min");
  discNumber("lowflow_window_s",  "Low Flow Window",     10,  600,  5,   "s");
  discNumber("max_runtime_min",   "Max Runtime",        10,  720,  10,   "min");
  discNumber("pulses_per_litre",  "Flow K-Factor",       1,  1000,  0.1f,"p/L");
  discNumber("tank_empty_cm",     "Distance When Empty",30,   600,  1,   "cm");
  discNumber("tank_full_cm",      "Distance When Full", 25,   600,  1,   "cm");

  discButton("force_test",  "Check For Water Now", "mdi:water-check");
  discButton("force_stop",  "Stop Pump",           "mdi:stop");
  discButton("clear_fault", "Clear Fault",         "mdi:restart-alert");

  // auto_enabled as a switch
  {
    JsonDocument d;
    d["name"]    = "Automatic Mode";
    d["uniq_id"] = String(DEVICE_ID) + "_auto_enabled";
    d["stat_t"]  = MQTT_BASE "/state";
    d["val_tpl"] = "{{ value_json.auto_enabled }}";
    d["cmd_t"]   = MQTT_BASE "/set/auto_enabled";
    d["avty_t"]  = MQTT_BASE "/availability";
    d["stat_on"] = "1"; d["stat_off"] = "0";
    d["pl_on"]   = "1"; d["pl_off"]   = "0";
    d["ic"]      = "mdi:robot";
    devBlock(d);
    char buf[700];
    size_t n = serializeJson(d, buf, sizeof(buf));
    mqtt.publish(MQTT_HA_PREFIX "/switch/" DEVICE_ID "/auto_enabled/config",
                 (const uint8_t*)buf, n, true);
  }
}

// ---- MQTT command handling ----------------------------------------------

static void applySetting(const String& key, const String& val) {
  bool ok = true;
  if      (key == "poll_interval_min") cfg.pollIntervalMin  = val.toInt();
  else if (key == "test_duration_s")   cfg.testDurationS    = val.toInt();
  else if (key == "cooldown_hours")    cfg.cooldownHours    = val.toInt();
  else if (key == "flow_threshold")    cfg.flowThresholdLpm = val.toFloat();
  else if (key == "lowflow_window_s")  cfg.lowFlowWindowS   = val.toInt();
  else if (key == "max_runtime_min")   cfg.maxRuntimeMin    = val.toInt();
  else if (key == "pulses_per_litre")  cfg.pulsesPerLitre   = val.toFloat();
  else if (key == "tank_empty_cm")     cfg.tankEmptyCm      = val.toInt();
  else if (key == "tank_full_cm")      cfg.tankFullCm       = val.toInt();
  else if (key == "auto_enabled")      cfg.autoEnabled      = (val == "1" || val == "ON" || val == "true");
  else ok = false;

  if (cfg.pulsesPerLitre < 0.1f) cfg.pulsesPerLitre = DEF_PULSES_PER_L;  // never divide by ~0
  if (ok) { saveSettings(); publishState(); }
}

static void applyCommand(const String& cmd) {
  if      (cmd == "force_test")  forceTestReq = true;
  else if (cmd == "force_stop")  forceStopReq = true;
  else if (cmd == "clear_fault") { if (state == ST_FAULT) { faultReason = ""; setState(ST_IDLE); } }
  publishState();
}

static void mqttCallback(char* topic, byte* payload, unsigned int len) {
  String t(topic);
  String v; v.reserve(len);
  for (unsigned int i = 0; i < len; i++) v += (char)payload[i];

  const String setPrefix = MQTT_BASE "/set/";
  if (t.startsWith(setPrefix))      applySetting(t.substring(setPrefix.length()), v);
  else if (t == MQTT_BASE "/cmd")   applyCommand(v);
}

static void mqttEnsure() {
  static uint32_t nextTry = 0;
  static uint16_t backoff = 2;

  if (mqtt.connected()) { mqtt.loop(); return; }
  if (WiFi.status() != WL_CONNECTED) return;
  if (millis() < nextTry) return;

  if (mqtt.connect(DEVICE_ID, MQTT_USER, MQTT_PASSWORD,
                   MQTT_BASE "/availability", 0, true, "offline")) {
    mqtt.publish(MQTT_BASE "/availability", "online", true);
    mqtt.subscribe(MQTT_BASE "/set/#");
    mqtt.subscribe(MQTT_BASE "/cmd");
    publishDiscovery();
    publishState();
    backoff = 2;
    Serial.println(F("[mqtt] connected"));
  } else {
    backoff = min<uint16_t>(backoff * 2, 60);
    nextTry = millis() + backoff * 1000UL;
  }
}

// ---- built-in web page ---------------------------------------------------

static const char PAGE[] PROGMEM = R"HTML(<!doctype html><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>Water Tank</title><style>
:root{color-scheme:light dark}
body{font:16px/1.5 system-ui,sans-serif;margin:0;padding:20px;max-width:560px;margin:auto}
h1{font-size:1.2rem;margin:0 0 16px}
.bar{height:26px;background:#8883;border-radius:6px;overflow:hidden;margin:10px 0}
.bar>i{display:block;height:100%;background:#2b8a3e;transition:width .6s}
table{width:100%;border-collapse:collapse}
td{padding:6px 0;border-bottom:1px solid #8883}
td:last-child{text-align:right;font-variant-numeric:tabular-nums}
button{font:inherit;padding:10px 16px;margin:4px 4px 0 0;border:1px solid #8886;
border-radius:8px;background:#8881;cursor:pointer}
#st{font-weight:600}
</style>
<h1>Water Tank &mdash; Motor 2</h1>
<div>State: <span id=st>...</span></div>
<div class=bar><i id=lv style=width:0%></i></div>
<table id=t></table>
<p><button onclick=go('force_test')>Check for water now</button>
<button onclick=go('force_stop')>Stop pump</button>
<button onclick=go('clear_fault')>Clear fault</button></p>
<script>
const F=[['tank_percent','Level','%'],['tank_litres','Volume','L'],
['flow_lpm','Flow','L/min'],['session_litres','This fill','L'],
['today_litres','Today','L'],['runtime_today_min','Runtime today','min'],
['dry_starts_today','Dry starts today',''],['last_arrival','Last arrival',''],
['last_stop','Last stop',''],['distance_cm','Sensor distance','cm'],
['float_full','Float full',''],['sensor_ok','Level sensor OK',''],
['rssi','WiFi','dBm'],['uptime_min','Uptime','min']];
async function go(c){await fetch('/cmd?c='+c,{method:'POST'});tick()}
async function tick(){
 const d=await(await fetch('/api/status')).json();
 st.textContent=d.state+(d.fault?' — '+d.fault:'');
 lv.style.width=d.tank_percent+'%';
 t.innerHTML=F.map(([k,l,u])=>`<tr><td>${l}</td><td>${d[k]}${u?' '+u:''}</td></tr>`).join('');
}
tick();setInterval(tick,2000);
</script>)HTML";

static bool webAuthOk() {
  if (strlen(WEB_USER) == 0) return true;
  if (web.authenticate(WEB_USER, WEB_PASSWORD)) return true;
  web.requestAuthentication();
  return false;
}

static void setupWeb() {
  web.on("/", []() {
    if (!webAuthOk()) return;
    web.send_P(200, "text/html", PAGE);
  });
  web.on("/api/status", []() {
    if (!webAuthOk()) return;
    JsonDocument d;
    buildStateJson(d);
    String out;
    serializeJson(d, out);
    web.send(200, "application/json", out);
  });
  web.on("/cmd", HTTP_POST, []() {
    if (!webAuthOk()) return;
    applyCommand(web.arg("c"));
    web.send(200, "text/plain", "ok");
  });
  web.begin();
}

// ------------------------------------------------------------------ daily

static void tickDailyRollover() {
  time_t now = time(nullptr);
  if (now < 1700000000) return;            // NTP hasn't landed yet
  struct tm tmv;
  localtime_r(&now, &tmv);
  if (lastYday == -1) { lastYday = tmv.tm_yday; return; }
  if (tmv.tm_yday != lastYday) {
    lastYday = tmv.tm_yday;
    todayLitres = 0;
    todayRuntimeSec = 0;
    todayDryStarts = 0;
    nvs.putULong("tdl", 0);
    nvs.putULong("tdr", 0);
    nvs.putUShort("tds", 0);
  }
}

// ------------------------------------------------------------------- setup

void setup() {
  // FIRST. Before serial, before anything. A floating relay pin during boot
  // would kick the contactor and start a 1HP pump with nobody watching.
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, RELAY_OFF);

  Serial.begin(115200);
  delay(100);
  Serial.println(F("\n[boot] WIOT motor2 controller"));

  pinMode(PIN_LED,    OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_FLOAT,  INPUT_PULLUP);
  pinMode(PIN_US_TRIG, OUTPUT);
  pinMode(PIN_US_ECHO, INPUT);
  pinMode(PIN_FLOW,   INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW), flowISR, RISING);

  loadSettings();
  todayLitres     = nvs.getULong ("tdl", 0);
  todayRuntimeSec = nvs.getULong ("tdr", 0);
  todayDryStarts  = nvs.getUShort("tds", 0);
  lastArrival     = (time_t)nvs.getULong("arr", 0);

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(DEVICE_ID);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  configTzTime(TZ_INFO, "pool.ntp.org", "time.nist.gov");

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(1024);          // discovery payloads blow past the 256 default

  setupWeb();

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t wdt = { .timeout_ms = 30000, .idle_core_mask = 0, .trigger_panic = true };
  esp_task_wdt_reconfigure(&wdt);
#else
  esp_task_wdt_init(30, true);
#endif
  esp_task_wdt_add(NULL);

  lastGoodUsMs = millis();
  lastTestMs   = millis() - (uint32_t)cfg.pollIntervalMin * 60000UL;  // probe on boot
  stateSinceMs = millis();
}

// -------------------------------------------------------------------- loop

void loop() {
  esp_task_wdt_reset();

  uint32_t now = millis();
  static uint32_t lastFlow = 0, lastUs = 0, lastPub = 0, lastDay = 0;

  if (now - lastFlow >= 1000)  { lastFlow = now; updateFlow(); }

  uint32_t usPeriod = (state == ST_PUMPING || state == ST_TESTING)
                        ? US_PERIOD_PUMPING_MS : US_PERIOD_IDLE_MS;
  if (now - lastUs >= usPeriod) { lastUs = now; updateLevel(); }

  floatFull = FLOAT_IS_FULL(digitalRead(PIN_FLOAT));

  tickStateMachine();
  tickButton();
  tickLed();

  mqttEnsure();
  web.handleClient();

  if (now - lastPub >= 5000) { lastPub = now; publishState(); }
  if (now - lastDay >= 60000) { lastDay = now; tickDailyRollover(); }
}
