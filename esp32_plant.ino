
#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <time.h>
#include <stdint.h>
#include <ArduinoJson.h>
#include <Preferences.h>

// ====== Network & Firebase config ======
const char* WIFI_SSID = "";
const char* WIFI_PASSWORD = "";

const char* FIREBASE_HOST = "";
const char* FIREBASE_AUTH = ""; // 

// ====== Device identity ======
const char* DEVICE_ID = "ESP32_01";
const char* DEVICE_KEY = "1234567890";

// ====== Pins and sensors ======
const int soilSensorPin = 34;
const int lightSensorPin = 32;
const int relayPin = 26;
const bool relayActiveLow = true;

Adafruit_SHT31 sht31 = Adafruit_SHT31();

// ====== ADC & sampling ======
const int analogResolutionBits = 12;
const int sampleCount = 6;
const int sampleDelayMs = 25;

// ====== calibration for soil sensor ======
int dryValue = 3800;
int wetValue = 800;

float computeSoilPercent(int raw) {
  if (dryValue == wetValue) return 0.0f;
  if (dryValue > wetValue) {
    float p = (float)(dryValue - raw) * 100.0f / (float)(dryValue - wetValue);
    p = constrain(p, 0.0f, 100.0f);
    return p;
  } else {
    float p = (float)(raw - dryValue) * 100.0f / (float)(wetValue - dryValue);
    p = constrain(p, 0.0f, 100.0f);
    return p;
  }
}

// ====== control params & thresholds ======
const int thresholdPercent = 40;
const int hysteresisPercent = 6;
const float skipHumidityAbove = 85.0;

// measurement timing
const unsigned long measureInterval = 60000UL; // 1 minute
unsigned long lastMeasureMillis = 0;
bool pumpOn = false;

// watering request local rate-limiting (extra local safeguard)
uint64_t lastWaterReqTs = 0;
const uint64_t WATER_REQ_MIN_INTERVAL_MS = 5UL * 60UL * 1000UL; // 5 minutes (local safeguard)

// ====== Command poll / execution ======
const unsigned long COMMAND_POLL_INTERVAL_MS = 5000UL; // poll every 5s
unsigned long lastCommandPollMillis = 0;

// safe clamp for duration
const uint32_t SAFE_MAX_DURATION_MS = 180000; // 3 minutes

// command execution state (simple blocking execution used here)
String currentCommandKey = "";
bool executingCommand = false;

// ====== Pump calibration (replace with your measured flow) ======
const float PUMP_FLOW_ML_PER_S = 12.0f; // <-- 校準後改為實測值 (ml/s)
const uint32_t PUMP_SAFE_MAX_MS = 120000; // safety ceiling for pump run (2 minutes)

// pulse settings
const uint32_t PULSE_MS = 5000;
const uint32_t INTERVAL_MS = 2000;

// ====== AI ask rate-limiting and decision state ======
Preferences prefs;
const char* PREF_NAMESPACE = "ai_req";
const char* PREF_KEY_LAST_ASK = "lastAskTs";

// debounce / trend
const int CONSECUTIVE_BELOW = 3;
const float SOIL_CRITICAL_PCT = 20.0f; // 臨界 soil_pct
const uint64_t EMERGENCY_MIN_INTERVAL_MS = 1UL * 60UL * 60UL * 1000UL; // 1 hour
const uint64_t MIN_ASK_INTERVAL_MS_FLOOR = 6UL * 60UL * 60UL * 1000UL; // 6 hours floor
const uint32_t DEFAULT_BASE_MIN_DAYS = 7;
const uint64_t DEFAULT_BASE_MIN_INTERVAL_MS = (uint64_t)DEFAULT_BASE_MIN_DAYS * 24UL * 3600UL * 1000UL;

// RH / trend adjustments
const float RH_LOW_THRESHOLD = 40.0f;
const float RH_LOW_FACTOR = 0.5f;
const float SOIL_TREND_FACTOR = 0.6f;
const float SOIL_TREND_THRESHOLD_PCT_PER_MIN = 2.0f;

int consecBelowCnt = 0;
const int TREND_BUFFER_SIZE = 6;
float soilBuffer[TREND_BUFFER_SIZE];
uint8_t soilBufIdx = 0;
bool soilBufFull = false;

// ====== Device-side request limiting (daily limits etc) ======
const uint8_t MAX_RUNS_PER_DAY = 3; // 每日允許發出的 watering_request 次數
const uint64_t MIN_REQUEST_INTERVAL_MS = 4UL * 3600UL * 1000UL; // 4 小時（device 端最小間隔）
const uint64_t EMERGENCY_MIN_INTERVAL_LOCAL_MS = 1UL * 3600UL * 1000UL; // 臨界情況下本地最小間隔 1 小時

const char* PREF_KEY_RUNS_TODAY = "runsToday";
const char* PREF_KEY_DAY_START = "runsDayStart";
const char* PREF_KEY_LAST_REQ_ID = "lastReqId";

// ====== Local cooldown for commands (prevent re-exec on 401) ======
const char* PREF_LAST_CMD = "lastCmd";       // key 名稱
const char* PREF_LAST_CMD_TS = "lastCmdTs";
const uint64_t CMD_RETRY_AVOID_MS = 10UL * 60UL * 1000UL; // 10 分鐘 cooldown

// ====== Helpers: time ======
String iso8601UtcNow(bool &synced) {
  time_t now = time(NULL);
  if (now <= 1000) { synced = false; return String("time_not_synced"); }
  synced = true;
  struct tm tm_utc;
  gmtime_r(&now, &tm_utc);
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm_utc);
  return String(buf);
}
uint64_t epochMillisNow(bool &synced) {
  time_t now = time(NULL);
  if (now <= 1000) { synced = false; return 0ULL; }
  synced = true;
  uint64_t ms = (uint64_t)now * 1000ULL;
  ms += (uint64_t)(millis() % 1000);
  return ms;
}

// ====== HTTP helpers ======
bool ensureWiFiConnected(unsigned long timeoutMs = 20000) {
  if (WiFi.status() == WL_CONNECTED) return true;
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
    if (millis() - start > timeoutMs) {
      Serial.println("\nWiFi connection timeout.");
      return false;
    }
  }
  Serial.println("\nWiFi connected.");
  return true;
}

String httpGet(const String &url) {
  if (!ensureWiFiConnected()) return String("");
  HTTPClient http;
  http.begin(url);
  int code = http.GET();
  String payload = "";
  if (code > 0) {
    payload = http.getString();
  } else {
    Serial.printf("GET failed: %s\n", http.errorToString(code).c_str());
  }
  http.end();
  return payload;
}

bool postJsonToPath(const String &path, const String &json, String &outPushKey) {
  if (!ensureWiFiConnected()) { Serial.println("WiFi not connected"); return false; }
  String host = String(FIREBASE_HOST);
  while (host.length() && host.charAt(host.length()-1) == '/') host.remove(host.length()-1);
  String url = "https://" + host + "/" + path + ".json";
  if (strlen(FIREBASE_AUTH) > 0) { url += "?auth="; url += FIREBASE_AUTH; }
  Serial.println("POST URL: " + url);
  Serial.println("JSON: " + json);
  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(json);
  if (code > 0) {
    String payload = http.getString();
    Serial.printf("HTTP %d: %s\n", code, payload.c_str());
    http.end();
    if (code == 200 || code == 201) {
      int iName = payload.indexOf("\"name\"");
      if (iName >= 0) {
        int iColon = payload.indexOf(':', iName);
        int iQ1 = payload.indexOf('"', iColon);
        int iQ2 = payload.indexOf('"', iQ1 + 1);
        if (iQ1 >= 0 && iQ2 > iQ1) {
          outPushKey = payload.substring(iQ1 + 1, iQ2);
        }
      }
      return true;
    } else {
      Serial.printf("POST returned code %d\n", code);
      return false;
    }
  } else {
    Serial.printf("POST failed: %s\n", http.errorToString(code).c_str());
    http.end();
  }
  return false;
}

// putJsonToPath: accepts either path (no .json) or full URL; appends auth if configured
bool putJsonToPath(const String &pathOrUrl, const String &json) {
  if (!ensureWiFiConnected()) { Serial.println("WiFi not connected"); return false; }

  String url = pathOrUrl;
  if (!url.startsWith("http://") && !url.startsWith("https://")) {
    String host = String(FIREBASE_HOST);
    while (host.length() && host.charAt(host.length()-1) == '/') host.remove(host.length()-1);
    String path = pathOrUrl;
    if (path.startsWith("/")) path = path.substring(1);
    url = "https://" + host + "/" + path;
    if (!url.endsWith(".json")) url += ".json";
    if (strlen(FIREBASE_AUTH) > 0) {
      if (url.indexOf("auth=") < 0) {
        url += "?auth=";
        url += FIREBASE_AUTH;
      }
    }
  } else {
    // full URL: ensure auth appended if set
    if (strlen(FIREBASE_AUTH) > 0 && url.indexOf("auth=") < 0) {
      url += (url.indexOf('?') >= 0) ? "&auth=" : "?auth=";
      url += FIREBASE_AUTH;
    }
  }

  Serial.println("PUT URL: " + url);
  Serial.println("JSON: " + json);

  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  int code = http.PUT(json);
  if (code > 0) {
    String payload = http.getString();
    Serial.printf("PUT HTTP %d: %s\n", code, payload.c_str());
    http.end();
    return (code == 200 || code == 201 || code == 204);
  } else {
    Serial.printf("PUT failed: %s\n", http.errorToString(code).c_str());
    http.end();
    return false;
  }
}

// ====== Build JSON payloads ======
String buildJsonPayload(uint64_t epochMs, const String &datetime, int soilRaw, float soilPct, float T, float RH, int lightRaw, bool pump, const String &mode) {
  char buf[700];
  int n = snprintf(buf, sizeof(buf),
    "{\"id\":\"%s\",\"device_key\":\"%s\",\"datetime\":\"%s\",\"timestamp\":%llu,\"soil_raw\":%d,\"soil_pct\":%.2f,\"T\":%.2f,\"RH\":%.2f,\"light_raw\":%d,\"pump\":%s,\"mode\":\"%s\"}",
    DEVICE_ID,
    DEVICE_KEY,
    datetime.c_str(),
    (unsigned long long)epochMs,
    soilRaw,
    soilPct,
    T,
    RH,
    lightRaw,
    pump ? "true" : "false",
    mode.c_str()
  );
  if (n < 0) return String("");
  return String(buf);
}

String buildWateringRequestJson(const String &requestId, uint64_t epochMs, int soilRaw, float soilPct, float T, float RH, int lightRaw, bool pump) {
  char buf[600];
  int n = snprintf(buf, sizeof(buf),
    "{\"request_id\":\"%s\",\"device_key\":\"%s\",\"timestamp\":%llu,\"snapshot\":{\"soil_pct\":%.2f,\"soil_raw\":%d,\"T\":%.2f,\"RH\":%.2f,\"light_raw\":%d,\"pump\":%s}}",
    requestId.c_str(),
    DEVICE_KEY,
    (unsigned long long)epochMs,
    soilPct,
    soilRaw,
    T,
    RH,
    lightRaw,
    pump ? "true" : "false"
  );
  if (n < 0) return String("");
  return String(buf);
}

// ====== Sensors & aux ======
int readSoilAverage() {
  long sum = 0;
  for (int i = 0; i < sampleCount; ++i) {
    sum += analogRead(soilSensorPin);
    delay(sampleDelayMs);
  }
  return (int)(sum / sampleCount);
}

void setRelay(bool on) {
  if (relayActiveLow) digitalWrite(relayPin, on ? LOW : HIGH);
  else digitalWrite(relayPin, on ? HIGH : LOW);
  pumpOn = on;
}

// fetch latest_mode
String fetchLatestMode() {
  String host = String(FIREBASE_HOST);
  while (host.length() && host.charAt(host.length()-1) == '/') host.remove(host.length()-1);
  String url = "https://" + host + "/devices/" + String(DEVICE_ID) + "/latest_mode.json";
  if (strlen(FIREBASE_AUTH) > 0) { url += "?auth="; url += FIREBASE_AUTH; }
  String resp = httpGet(url);
  if (resp.length() == 0 || resp == "null") {
    return String("unknown");
  }
  int i = resp.indexOf("\"mode\"");
  if (i < 0) {
    return String("unknown");
  }
  int colon = resp.indexOf(':', i);
  if (colon < 0) return String("unknown");
  int q1 = resp.indexOf('"', colon);
  if (q1 < 0) return String("unknown");
  int q2 = resp.indexOf('"', q1 + 1);
  if (q2 < 0) return String("unknown");
  String mode = resp.substring(q1 + 1, q2);
  mode.trim();
  return mode.length() ? mode : String("unknown");
}

// ====== AI ask persistence (store seconds to avoid overflow) ======
void saveLastAskTs(uint64_t ts_ms) {
  unsigned long ts_s = (unsigned long)(ts_ms / 1000ULL);
  prefs.putULong(PREF_KEY_LAST_ASK, ts_s);
}
uint64_t loadLastAskTs() {
  if (!prefs.isKey(PREF_KEY_LAST_ASK)) return 0;
  unsigned long v = prefs.getULong(PREF_KEY_LAST_ASK, 0);
  return (uint64_t)v * 1000ULL;
}

// ====== daily runs persistence helpers (store seconds) ======
uint32_t loadRunsToday() {
  return (uint32_t) prefs.getUInt(PREF_KEY_RUNS_TODAY, 0);
}
uint64_t loadRunsDayStart() {
  unsigned long v = prefs.getULong(PREF_KEY_DAY_START, 0);
  return (uint64_t)v * 1000ULL;
}
void saveRunsToday(uint32_t v) {
  prefs.putUInt(PREF_KEY_RUNS_TODAY, v);
}
void saveRunsDayStart(uint64_t ts_ms) {
  unsigned long ts_s = (unsigned long)(ts_ms / 1000ULL);
  prefs.putULong(PREF_KEY_DAY_START, ts_s);
}
void saveLastReqId(const String &id) {
  prefs.putString(PREF_KEY_LAST_REQ_ID, id);
}

// compute UTC day start ms
uint64_t utcDayStartMs(uint64_t epochMs) {
  uint64_t days = epochMs / (24ULL * 3600ULL * 1000ULL);
  return days * 24ULL * 3600ULL * 1000ULL;
}
void resetIfNewDay(uint64_t epochMs) {
  uint64_t currentDayStart = utcDayStartMs(epochMs);
  uint64_t stored = loadRunsDayStart();
  if (stored == 0 || stored != currentDayStart) {
    saveRunsDayStart(currentDayStart);
    saveRunsToday(0);
    Serial.println("New UTC day detected, runsToday reset to 0");
  }
}

// soil trend buffer
void updateSoilBuffer(float soilPct) {
  soilBuffer[soilBufIdx++] = soilPct;
  if (soilBufIdx >= TREND_BUFFER_SIZE) {
    soilBufIdx = 0;
    soilBufFull = true;
  }
}
float computeSoilTrendPctPerMin() {
  int count = soilBufFull ? TREND_BUFFER_SIZE : soilBufIdx;
  if (count < 2) return 0.0f;
  int startIdx = (soilBufIdx + TREND_BUFFER_SIZE - count) % TREND_BUFFER_SIZE;
  float first = soilBuffer[startIdx];
  int lastIdx = (startIdx + count - 1) % TREND_BUFFER_SIZE;
  float last = soilBuffer[lastIdx];
  float minutes = (count * (float)measureInterval) / 60000.0f;
  if (minutes <= 0.0f) return 0.0f;
  float slope = (first - last) / minutes; // %/min downward positive
  return slope;
}

// decision: should ask AI?
bool shouldAskAI(uint64_t epochMs, float soilPct, float RH) {
  int lowerThreshold = thresholdPercent - hysteresisPercent / 2;
  if (soilPct < lowerThreshold) {
    consecBelowCnt++;
  } else {
    consecBelowCnt = 0;
  }
  if (consecBelowCnt < CONSECUTIVE_BELOW) return false;

  uint64_t lastAsk = loadLastAskTs();
  // emergency
  if (soilPct <= SOIL_CRITICAL_PCT) {
    if (lastAsk == 0 || (epochMs - lastAsk) >= EMERGENCY_MIN_INTERVAL_MS) return true;
    return false;
  }

  uint64_t baseMinInterval = DEFAULT_BASE_MIN_INTERVAL_MS; // could be read from DB config
  float effectiveFactor = 1.0f;
  if (RH < RH_LOW_THRESHOLD) effectiveFactor *= RH_LOW_FACTOR;
  float slope = computeSoilTrendPctPerMin();
  if (slope >= SOIL_TREND_THRESHOLD_PCT_PER_MIN) effectiveFactor *= SOIL_TREND_FACTOR;

  uint64_t effectiveInterval = (uint64_t)((float)baseMinInterval * effectiveFactor);
  if (effectiveInterval < MIN_ASK_INTERVAL_MS_FLOOR) effectiveInterval = MIN_ASK_INTERVAL_MS_FLOOR;

  if (lastAsk == 0 || (epochMs - lastAsk) >= effectiveInterval) return true;
  return false;
}

// ====== Device-side control: canSendRequest and recordSuccessfulRequest ======
bool canSendRequest(uint64_t epochMs, float soilPct, float RH) {
  resetIfNewDay(epochMs);
  uint64_t lastAsk = loadLastAskTs();
  uint32_t runsToday = loadRunsToday();

  // emergency handling
  if (soilPct <= SOIL_CRITICAL_PCT) {
    if (lastAsk == 0 || (epochMs - lastAsk) >= EMERGENCY_MIN_INTERVAL_LOCAL_MS) {
      if (runsToday >= MAX_RUNS_PER_DAY) {
        Serial.println("Emergency but runsToday reached MAX_RUNS_PER_DAY; skipping.");
        return false;
      }
      return true;
    } else {
      Serial.println("Emergency but emergency min interval not passed; skipping.");
      return false;
    }
  }

  // normal checks
  if (runsToday >= MAX_RUNS_PER_DAY) {
    Serial.println("runsToday >= MAX_RUNS_PER_DAY; skipping request.");
    return false;
  }
  if (lastAsk != 0 && (epochMs - lastAsk) < MIN_REQUEST_INTERVAL_MS) {
    Serial.println("Min request interval not passed; skipping.");
    return false;
  }
  return true;
}

void recordSuccessfulRequest(uint64_t epochMs, const String &requestId) {
  resetIfNewDay(epochMs);
  uint32_t runsToday = loadRunsToday();
  runsToday++;
  saveRunsToday(runsToday);
  saveLastAskTs(epochMs);
  saveLastReqId(requestId);
  Serial.printf("Recorded request: id=%s, runsToday=%u\n", requestId.c_str(), runsToday);
}

// ====== Pump helpers ======
uint32_t volumeToDurationMs(float volume_ml) {
  if (PUMP_FLOW_ML_PER_S <= 0.001f) return 0;
  float seconds = volume_ml / PUMP_FLOW_ML_PER_S;
  uint32_t ms = (uint32_t)(seconds * 1000.0f + 0.5f);
  if (ms > PUMP_SAFE_MAX_MS) ms = PUMP_SAFE_MAX_MS;
  return ms;
}

void runPumpForMsBlocking(uint32_t ms) {
  setRelay(true);
  unsigned long start = millis();
  while (millis() - start < ms) {
    delay(10);
  }
  setRelay(false);
}

void runPumpVolumeWithPulse(float volume_ml) {
  uint32_t total_ms = volumeToDurationMs(volume_ml);
  if (total_ms == 0) return;
  uint32_t remaining = total_ms;
  while (remaining > 0) {
    uint32_t seg = (remaining > PULSE_MS) ? PULSE_MS : remaining;
    runPumpForMsBlocking(seg);
    remaining -= seg;
    if (remaining > 0) delay(INTERVAL_MS);
  }
}

// ====== Local attempt helpers (store seconds to avoid overflow) ======
void saveLastAttemptCmdLocal(const String &cmdId, uint64_t ts_ms) {
  prefs.putString(PREF_LAST_CMD, cmdId);
  unsigned long ts_s = (unsigned long)(ts_ms / 1000ULL);
  prefs.putULong(PREF_LAST_CMD_TS, ts_s);
}
String loadLastAttemptCmdLocal() {
  return prefs.getString(PREF_LAST_CMD, "");
}
uint64_t loadLastAttemptCmdTsLocal() {
  unsigned long ts_s = prefs.getULong(PREF_LAST_CMD_TS, 0);
  return (uint64_t)ts_s * 1000ULL;
}

// ====== Commands polling & execution (uses ArduinoJson) ======
void pollAndHandleCommands() {
  if (executingCommand) return;

  String host = String(FIREBASE_HOST);
  while (host.length() && host.charAt(host.length()-1) == '/') host.remove(host.length()-1);
  String url = "https://" + host + "/devices/" + String(DEVICE_ID) + "/commands.json";
  if (strlen(FIREBASE_AUTH) > 0) { url += "?auth="; url += FIREBASE_AUTH; }

  String resp = httpGet(url);
  if (resp.length() == 0 || resp == "null") return;

  StaticJsonDocument<8 * 1024> doc;
  DeserializationError err = deserializeJson(doc, resp);
  if (err) {
    Serial.println("parse commands json failed");
    return;
  }
  JsonObject root = doc.as<JsonObject>();

  // iterate commands
  for (JsonPair kv : root) {
    const char* cmdKeyC = kv.key().c_str();
    String cmdKey = String(cmdKeyC);
    JsonObject cmd = kv.value().as<JsonObject>();
    const char* statusC = cmd["status"] | "";
    String status = String(statusC);

    if (status != "pending") continue; // only handle pending

    // local cooldown: if we tried this cmd recently, skip it
    uint64_t nowMs; bool synced;
    nowMs = epochMillisNow(synced);
    String lastCmd = loadLastAttemptCmdLocal();
    uint64_t lastCmdTs = loadLastAttemptCmdTsLocal();
    if (lastCmd == cmdKey && lastCmdTs != 0 && (nowMs - lastCmdTs) < CMD_RETRY_AVOID_MS) {
      Serial.printf("Skipping cmd %s: attempted %.0f sec ago (cooldown)\n", cmdKey.c_str(), (nowMs - lastCmdTs) / 1000.0);
      continue;
    }

    // get duration (safety)
    uint32_t dur = cmd["duration_ms"] | 0;
    if (dur == 0) {
      // try to mark failed (may 401 if rules disallow) but still skip executing
      // Build a full payload for failure marking similar to ack/done to satisfy rules
      const char* command_id = cmd["command_id"] | "";
      const char* type = cmd["type"] | "run";
      const char* request_id = cmd["request_id"] | "";
      char failBuf[512];
      unsigned long now_s = (unsigned long)(nowMs / 1000ULL);
      int nf = snprintf(failBuf, sizeof(failBuf),
        "{\"command_id\":\"%s\",\"device_key\":\"%s\",\"duration_ms\":%u,\"request_id\":\"%s\",\"type\":\"%s\",\"status\":\"failed\",\"executed_at\":%lu}",
        command_id, DEVICE_KEY, dur, request_id, type, now_s);
      String pathFail = String("devices/") + DEVICE_ID + "/commands/" + cmdKey;
      bool okFail = putJsonToPath(pathFail, String(failBuf));
      Serial.printf("Invalid duration for %s, marked failed: %s\n", cmdKey.c_str(), okFail ? "ok" : "put_failed");
      // record attempt locally so we don't repeatedly try invalid commands
      saveLastAttemptCmdLocal(cmdKey, nowMs);
      continue;
    }
    if (dur > SAFE_MAX_DURATION_MS) dur = SAFE_MAX_DURATION_MS;

    // record attempt BEFORE executing so we avoid immediate re-exec even if PUT fails
    saveLastAttemptCmdLocal(cmdKey, nowMs);

    // Build and send ACK payload that includes immutable fields (so rules accept it)
    {
      const char* command_id = cmd["command_id"] | "";
      const char* type = cmd["type"] | "run";
      uint32_t duration = cmd["duration_ms"] | 0;
      const char* request_id = cmd["request_id"] | "";
      char ackBuf[512];
      uint64_t nowAckMs; bool syncedAck;
      nowAckMs = epochMillisNow(syncedAck);
      int n = snprintf(ackBuf, sizeof(ackBuf),
        "{\"command_id\":\"%s\",\"device_key\":\"%s\",\"duration_ms\":%u,\"request_id\":\"%s\",\"type\":\"%s\",\"status\":\"ack\",\"ack_at\":%llu}",
        command_id, DEVICE_KEY, duration, request_id, type, (unsigned long long)nowAckMs);
      if (n > 0) {
        String pathAck = String("devices/") + DEVICE_ID + "/commands/" + cmdKey;
        bool okAck = putJsonToPath(pathAck, String(ackBuf));
        if (!okAck) {
          Serial.println("PUT ack failed (permission or rule mismatch). Will still execute but won't re-exec soon due to local record.");
        } else {
          Serial.println("ACK updated on server.");
        }
      }
    }

    // Execute command (blocking)
    Serial.printf("Executing command %s duration %u ms\n", cmdKey.c_str(), dur);
    executingCommand = true;
    runPumpForMsBlocking(dur);
    executingCommand = false;

    // After execution, build and send DONE payload including immutable fields
    {
      const char* command_id = cmd["command_id"] | "";
      const char* type = cmd["type"] | "run";
      uint32_t duration = cmd["duration_ms"] | 0;
      const char* request_id = cmd["request_id"] | "";
      char doneBuf[512];
      uint64_t execTs; bool synced2;
      execTs = epochMillisNow(synced2);
      int m = snprintf(doneBuf, sizeof(doneBuf),
        "{\"command_id\":\"%s\",\"device_key\":\"%s\",\"duration_ms\":%u,\"request_id\":\"%s\",\"type\":\"%s\",\"status\":\"done\",\"executed_at\":%llu}",
        command_id, DEVICE_KEY, duration, request_id, type, (unsigned long long)execTs);
      if (m > 0) {
        String pathDone = String("devices/") + DEVICE_ID + "/commands/" + cmdKey;
        bool okDone = putJsonToPath(pathDone, String(doneBuf));
        if (!okDone) {
          Serial.println("PUT done failed (permission or rule mismatch). Command may remain pending on server.");
        } else {
          // success: clear local attempt to allow future same-id commands if server updates
          prefs.remove(PREF_LAST_CMD);
          prefs.remove(PREF_LAST_CMD_TS);
          Serial.println("Server marked command done; cleared local attempt record.");
        }
      }
    }

    // Stop after handling one pending command this poll (avoid bulk)
    break;
  } // end for
}

// ====== main measurement + upload + maybe request AI ======
void maybePostWateringRequest(uint64_t epochMs, int soilRaw, float soilPct, float temperature, float humidity, int lightRaw, bool pumpOnLocal) {
  updateSoilBuffer(soilPct);

  String latestMode = fetchLatestMode();
  if (latestMode != "ai") return;

  // Device-side rate limiting (first gate)
  if (!canSendRequest(epochMs, soilPct, humidity)) {
    Serial.println("canSendRequest returned false; not creating watering_request.");
    return;
  }

  // Second gate: shouldAskAI (debounce, trend, RH)
  if (!shouldAskAI(epochMs, soilPct, humidity)) {
    Serial.println("shouldAskAI returned false; skip.");
    return;
  }

  // local safeguard (extra)
  uint64_t lastAsk = loadLastAskTs();
  if (lastAsk != 0 && (epochMs - lastAsk) < WATER_REQ_MIN_INTERVAL_MS) {
    Serial.println("Local min interval prevents creating new request.");
    return;
  }

  // build request
  String reqId = "req-" + String((unsigned long long)epochMs) + "-" + String(random(1000,9999));
  String reqJson = buildWateringRequestJson(reqId, epochMs, soilRaw, soilPct,
                                           isnan(temperature)?0.0f:temperature,
                                           isnan(humidity)?0.0f:humidity,
                                           lightRaw, pumpOnLocal);
  String pathReq = String("devices/") + DEVICE_ID + "/watering_requests";
  String pushKey;
  bool ok = postJsonToPath(pathReq, reqJson, pushKey);
  if (ok) {
    recordSuccessfulRequest(epochMs, reqId);
    Serial.println("AI mode: watering_request posted. reqId=" + reqId + " pushKey=" + pushKey);
    // optional: write last_request small flag (may be restricted by rules)
    char buf[400];
    int n = snprintf(buf, sizeof(buf), "{\"request_id\":\"%s\",\"device_key\":\"%s\",\"status\":\"pending\",\"timestamp\":%llu}", reqId.c_str(), DEVICE_KEY, (unsigned long long)epochMs);
    if (n > 0) {
      putJsonToPath(String("devices/") + DEVICE_ID + "/last_request", String(buf)); // may fail if rules disallow
    }
  } else {
    Serial.println("AI mode: failed to post watering_request.");
  }
}

void doMeasureAndUpload() {
  int soilRaw = readSoilAverage();
  float soilPct = computeSoilPercent(soilRaw);
  float temperature = sht31.readTemperature();
  float humidity = sht31.readHumidity();
  int lightRaw = analogRead(lightSensorPin);

  int lowerThreshold = thresholdPercent - hysteresisPercent / 2;
  int upperThreshold = thresholdPercent + hysteresisPercent / 2;

  String latestMode = fetchLatestMode();

  bool timeSynced = false;
  String datetime = iso8601UtcNow(timeSynced);
  uint64_t epochMs = epochMillisNow(timeSynced);

  if (latestMode == "ai") {
    if (!isnan(humidity) && humidity > skipHumidityAbove) {
      if (pumpOn) { setRelay(false); Serial.println("AI mode: Skip watering due to high ambient humidity."); }
    } else {
      if (!pumpOn && soilPct < lowerThreshold) {
        maybePostWateringRequest(epochMs, soilRaw, soilPct, temperature, humidity, lightRaw, pumpOn);
      }
      // device waits for command; do not auto-start pump in ai mode
    }
  } else {
    // local auto control
    if (!isnan(humidity) && humidity > skipHumidityAbove) {
      if (pumpOn) { setRelay(false); Serial.println("Skip watering due to high ambient humidity."); }
    } else {
      if (!pumpOn && soilPct < lowerThreshold) {
        setRelay(true);
        Serial.println("Auto: Pump ON (soil dry)");
      } else if (pumpOn && soilPct > upperThreshold) {
        setRelay(false);
        Serial.println("Auto: Pump OFF (soil wet)");
      }
    }
  }

  Serial.printf("[%s] soil_raw=%d soil=%.2f T=%.2f RH=%.2f light=%d pump=%s mode=%s\n",
                datetime.c_str(), soilRaw, soilPct,
                isnan(temperature)?0.0f:temperature,
                isnan(humidity)?0.0f:humidity,
                lightRaw, pumpOn?"ON":"OFF", latestMode.c_str());

  // upload history/latest/meta
  String json = buildJsonPayload(epochMs, datetime, soilRaw, soilPct,
                                isnan(temperature)?0.0f:temperature,
                                isnan(humidity)?0.0f:humidity,
                                lightRaw, pumpOn,
                                latestMode);

  String pathHistory = String("devices/") + DEVICE_ID + "/history";
  String pushKey;
  bool okPost = postJsonToPath(pathHistory, json, pushKey);
  if (okPost) {
    Serial.println("Posted to history. pushKey: " + pushKey);
    String pathLatest = String("devices/") + DEVICE_ID + "/latest";
    bool okPut = putJsonToPath(pathLatest, json);
    if (okPut) Serial.println("Updated latest.");
    char metaBuf[200];
    int mn = snprintf(metaBuf, sizeof(metaBuf),
                      "{\"last_push_key\":\"%s\",\"last_seen\":%llu}",
                      pushKey.c_str(), (unsigned long long)epochMs);
    if (mn > 0) {
      String pathMeta = String("devices/") + DEVICE_ID + "/meta";
      putJsonToPath(pathMeta, String(metaBuf));
    }
  } else {
    Serial.println("Failed to post to history.");
  }
}

// ====== Setup & loop ======
void setup() {
  Serial.begin(115200);
  delay(200);

  prefs.begin(PREF_NAMESPACE, false);

  // WiFi
  if (ensureWiFiConnected()) {
    Serial.println("WiFi connected. IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("WiFi not connected (will attempt later).");
  }

  // time sync
  configTime(0, 0, "pool.ntp.org", "time.google.com");
  Serial.print("Waiting for time sync");
  struct tm timeinfo;
  unsigned long start = millis();
  while (!getLocalTime(&timeinfo)) {
    Serial.print(".");
    delay(500);
    if (millis() - start > 10000) {
      Serial.println("\nTime sync timeout");
      break;
    }
  }
  if (getLocalTime(&timeinfo)) Serial.println("\nTime synced.");
  else Serial.println("\nProceeding without real time.");

  // ADC config
  analogReadResolution(analogResolutionBits);
  analogSetPinAttenuation(soilSensorPin, ADC_11db);
  analogSetPinAttenuation(lightSensorPin, ADC_11db);

  // SHT31
  Wire.begin();
  if (!sht31.begin(0x44)) {
    Serial.println("SHT31 not found at 0x44");
  } else {
    Serial.println("SHT31 found");
  }

  // relay
  pinMode(relayPin, OUTPUT);
  if (relayActiveLow) digitalWrite(relayPin, HIGH); else digitalWrite(relayPin, LOW);

  // initial measurement
  doMeasureAndUpload();
  lastMeasureMillis = millis();
  lastCommandPollMillis = millis();
}

void loop() {
  unsigned long now = millis();

  // measurement interval
  if (now - lastMeasureMillis >= measureInterval) {
    doMeasureAndUpload();
    lastMeasureMillis = now;
  }

  // poll commands periodically
  if (now - lastCommandPollMillis >= COMMAND_POLL_INTERVAL_MS) {
    lastCommandPollMillis = now;
    pollAndHandleCommands();
  }

  delay(10);
}