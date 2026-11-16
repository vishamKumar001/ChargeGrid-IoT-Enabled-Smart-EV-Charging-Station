// SIMULATION ONLY: no GPIO relay switching or physical charging.
// Implements the normalized time-plus-wait priority formula from the
// EV Charging Priority Logic Guide. Do not use as a hardware charging controller.
/*
  ============================================================
  CHARGEGRID - DUAL PORT EV CHARGING STATION
  ESP32 DevKit V1
  ============================================================
  Sensors:
    Battery 1 : GPIO34, 100k/100k divider
    Battery 2 : GPIO32, 100k/100k divider
    ACS712 1  : GPIO35
    ACS712 2  : GPIO33

  Relays:
    Port 1: GPIO25
    Port 2: GPIO26
    Active LOW by default

  HTTP endpoints:
    GET  /                    Dashboard
    GET  /data                Live JSON telemetry
    GET  /control?port=1&enable=1
    GET  /control?port=1&enable=0
    GET  /control?port=2&enable=1
    GET  /control?port=2&enable=0

  Substation:
    GET  /budget
    POST /telemetry

  Required hardware safety: battery protection, suitable fuses,
  correctly rated relay contacts, and verified sensor wiring.
  Dashboard measurements are not certified protection.
  ============================================================
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <math.h>

// ---------------- WIFI ----------------

const char* WIFI_SSID = "Athulya's Galaxy";
const char* WIFI_PASSWORD = "12345678";

// REQUIRED: after uploading the substation sketch, replace this placeholder
// with the IP printed by ESP32 #2 in Serial Monitor, then upload this sketch.
const char* SUBSTATION_IP = "10.18.92.165";

WebServer server(80);
WiFiClient mqttNet;
PubSubClient mqtt(mqttNet);
const char* MQTT_HOST = "broker.hivemq.com";
const int MQTT_PORT = 1883;
const char* MQTT_TOPIC = "vk/ev/grid_status";
unsigned long lastMqttTry = 0, lastGridMqttMs = 0;
const unsigned long MQTT_TIMEOUT_MS = 10000;
const unsigned long ALLOCATION_INTERVAL_MS = 500;

// ---------------- PRIORITY ALGORITHM (SIMULATION ONLY) ----------------
// These values follow the EV Charging Priority Logic Guide.
const float PRIORITY_ALPHA = 0.70f;
const float PRIORITY_BETA = 0.30f;
const float TARGET_SOC_PERCENT = 100.0f;
const float CELL_CAPACITY_AH = 2.2f;       // prototype cell nominal capacity
const float CELL_NOMINAL_V = 3.7f;
const float CELL_NOMINAL_ENERGY_WH = CELL_CAPACITY_AH * CELL_NOMINAL_V; // 8.14 Wh
const float PRIORITY_AVERAGE_POWER_W = 3.0f; // illustrative guide assumption; tune later
const float TIME_SCALE_MIN = 30.0f;
const float WAIT_SCALE_MIN = 30.0f;
const float REMAINING_TIME_EPSILON_MIN = 1.0f;
const float SIM_MAX_TARGET_CURRENT_A = 2.2f; // simulation cap only; hardware limit must be validated separately
const float MIN_COMMAND_CURRENT_A = 0.05f; // below this simulation threshold, keep the channel OFF

// Synthetic battery inputs; no physical sensing or relay control in this build.
float simBattery1V = 3.80f, simBattery2V = 3.70f;
float simSOC1 = 50.0f, simSOC2 = 35.0f;
float simWait1Min = 0.0f, simWait2Min = 0.0f;
float simCurrent1A = 0.0f, simCurrent2A = 0.0f;
bool requested1 = true, requested2 = true;
String mode1 = "OFF", mode2 = "OFF";
float target1A = 0, target2A = 0;
float target1W = 0, target2W = 0;
float priorityScore1 = 0, priorityScore2 = 0;
float remainingTime1Min = 0, remainingTime2Min = 0;
String lastAllocationGridStatus = "";

float jsonNumber(const String& body, const char* key, float fallback);
String jsonString(const String& body, const char* key, const String& fallback);


// ---------------- GPIO ----------------

const int BAT1_PIN = 34;
const int BAT2_PIN = 32;
const int ACS1_PIN = 35;
const int ACS2_PIN = 33;

const int RELAY1_PIN = 25;
const int RELAY2_PIN = 26;

const bool RELAY_ACTIVE_LOW = true;

// ---------------- SENSOR CALIBRATION ----------------

// 100k upper and 100k lower battery divider.
const float BATTERY_DIVIDER_RATIO = 2.0f;

// ACS712 output conditioner: 10k series and 20k to GND.
// Correct the attenuated ADC reading by approximately 1.5.
const float ACS_OUTPUT_SCALE = 1.5f;

// ACS712-05B nominal sensitivity.
const float ACS_SENSITIVITY = 0.185f;

// Calibrated zero offset in ADC-side voltage domain.
float acs1ZeroAdcV = 1.65f;
float acs2ZeroAdcV = 1.65f;

// Provisional only: measure actual voltage at each sensor's
// branch measurement point before treating calculated watts
// as actual electrical power.
const float PROVISIONAL_BRANCH_VOLTAGE = 5.0f;

// ---------------- TIMING ----------------

const unsigned long SENSOR_INTERVAL_MS = 500;
const unsigned long BUDGET_INTERVAL_MS = 2500;
const unsigned long TELEMETRY_INTERVAL_MS = 2000;
const unsigned long BUDGET_TIMEOUT_MS = 10000;

// ---------------- LIVE DATA ----------------

float battery1V = 0.0f;
float battery2V = 0.0f;

float branch1A = 0.0f;
float branch2A = 0.0f;

float branch1W = 0.0f;
float branch2W = 0.0f;

float allowedPowerW = 0.0f;
float receivedBudgetW = 0.0f;
float gridVoltageFromMQTT = 0.0f;
float gridCurrentFromMQTT = 0.0f;
float gridPowerFromMQTT = 0.0f;

bool port1CommandedOn = false;
bool port2CommandedOn = false;

bool substationOnline = false;
bool budgetReceived = false;

String gridStatus = "UNKNOWN";
String demandResponse = "UNKNOWN";
String lastCommand = "Waiting for substation budget";

unsigned long lastSample = 0;
unsigned long lastBudgetPoll = 0;
unsigned long lastBudgetSuccess = 0;
unsigned long lastTelemetryPost = 0;

// ============================================================
// SENSOR FUNCTIONS
// ============================================================

float readAdcVolts(int pin) {
  uint32_t sumMilliVolts = 0;
  const int samples = 24;

  for (int i = 0; i < samples; i++) {
    sumMilliVolts += analogReadMilliVolts(pin);
    delayMicroseconds(250);
  }

  return (sumMilliVolts / (float)samples) / 1000.0f;
}

float readBatteryVoltage(int pin) {
  float adcV = readAdcVolts(pin);
  return adcV * BATTERY_DIVIDER_RATIO;
}

float calibrateACSZero(int pin) {
  uint32_t sumMilliVolts = 0;
  const int samples = 250;

  for (int i = 0; i < samples; i++) {
    sumMilliVolts += analogReadMilliVolts(pin);
    delay(2);
  }

  // Keep the zero offset in the ADC-side voltage domain.
  return (sumMilliVolts / (float)samples) / 1000.0f;
}

float readACSCurrent(int pin, float zeroAdcV) {
  float adcV = readAdcVolts(pin);

  // Apply the same scale to the measured value and zero offset.
  float sensorV = adcV * ACS_OUTPUT_SCALE;
  float zeroV = zeroAdcV * ACS_OUTPUT_SCALE;

  float currentA = (sensorV - zeroV) / ACS_SENSITIVITY;

  // This prototype displays current magnitude only.
  currentA = fabsf(currentA);

  // Small zero-current noise suppression.
  if (currentA < 0.08f) {
    currentA = 0.0f;
  }

  return currentA;
}

// Approximate voltage-based SOC for a single Li-ion cell.
// Not accurate under load or during charging.
float estimateSOC(float v) {
  if (v >= 4.20f) return 100.0f;
  if (v >= 4.15f) return 95.0f;
  if (v >= 4.10f) return 90.0f;
  if (v >= 4.00f) return 80.0f;
  if (v >= 3.90f) return 65.0f;
  if (v >= 3.80f) return 50.0f;
  if (v >= 3.70f) return 35.0f;
  if (v >= 3.60f) return 20.0f;
  if (v >= 3.50f) return 10.0f;
  if (v >= 3.30f) return 5.0f;
  if (v >= 3.00f) return 0.0f;
  return 0.0f;
}

void sampleSensors() {
  battery1V = simBattery1V;
  battery2V = simBattery2V;
  // These user-entered values model synthetic current-sensor feedback sent
  // to the Substation. Allocation targets are reported separately below.
  branch1A = simCurrent1A;
  branch2A = simCurrent2A;
  branch1W = branch1A * battery1V;
  branch2W = branch2A * battery2V;
}

float clampFloat(float value, float low, float high) {
  return fminf(high, fmaxf(low, value));
}

float estimatedRemainingEnergyWh(float soc) {
  float remainingFraction = fmaxf(0.0f, (TARGET_SOC_PERCENT - clampFloat(soc, 0.0f, 100.0f)) / 100.0f);
  return CELL_NOMINAL_ENERGY_WH * remainingFraction;
}

float estimatedRemainingTimeMin(float soc) {
  float energyWh = estimatedRemainingEnergyWh(soc);
  if (energyWh <= 0.0f) return 0.0f;
  return 60.0f * energyWh / PRIORITY_AVERAGE_POWER_W;
}

float priorityScore(float soc, float waitMinutes) {
  // Guide's normalized score: alpha*(Tscale/Tremaining) + beta*min(wait/Wscale,1).
  if (soc >= 99.5f) return 0.0f; // target reached / essentially complete
  float remainingMin = fmaxf(estimatedRemainingTimeMin(soc), REMAINING_TIME_EPSILON_MIN);
  float waitTerm = fminf(fmaxf(waitMinutes, 0.0f) / WAIT_SCALE_MIN, 1.0f);
  return PRIORITY_ALPHA * (TIME_SCALE_MIN / remainingMin) + PRIORITY_BETA * waitTerm;
}

bool gridFresh() {
  return mqtt.connected() && lastGridMqttMs != 0 && millis() - lastGridMqttMs <= MQTT_TIMEOUT_MS;
}

String currentLabel(float currentA) {
  if (currentA < MIN_COMMAND_CURRENT_A) return "OFF";
  return String(currentA, 2) + " A";
}

// Score-weighted water-filling allocation. If a port reaches its power cap,
// remove it from the active set and redistribute the remaining budget.
void allocateWeightedPower(float budgetW,
                           const bool eligible[2],
                           const float scores[2],
                           const float maxPowerW[2],
                           float allocatedW[2]) {
  allocatedW[0] = 0.0f;
  allocatedW[1] = 0.0f;
  bool active[2] = {eligible[0] && maxPowerW[0] > 0.0f,
                    eligible[1] && maxPowerW[1] > 0.0f};
  float remainingBudgetW = fmaxf(0.0f, budgetW);

  // There are only two ports, so at most two clipping passes are needed.
  for (int pass = 0; pass < 2; ++pass) {
    float totalScore = 0.0f;
    for (int i = 0; i < 2; ++i) if (active[i]) totalScore += fmaxf(0.0f, scores[i]);
    if (totalScore <= 0.0f || remainingBudgetW <= 0.0f) break;

    float proposedW[2] = {0.0f, 0.0f};
    for (int i = 0; i < 2; ++i) {
      if (active[i]) proposedW[i] = remainingBudgetW * fmaxf(0.0f, scores[i]) / totalScore;
    }

    bool clippedAny = false;
    for (int i = 0; i < 2; ++i) {
      if (active[i] && proposedW[i] >= maxPowerW[i]) {
        allocatedW[i] = maxPowerW[i];
        remainingBudgetW = fmaxf(0.0f, remainingBudgetW - allocatedW[i]);
        active[i] = false;
        clippedAny = true;
      }
    }

    if (!clippedAny) {
      for (int i = 0; i < 2; ++i) if (active[i]) allocatedW[i] = proposedW[i];
      remainingBudgetW = 0.0f;
      break;
    }
  }
}

void applyAllocation() {
  sampleSensors();

  priorityScore1 = priorityScore(simSOC1, simWait1Min);
  priorityScore2 = priorityScore(simSOC2, simWait2Min);
  remainingTime1Min = estimatedRemainingTimeMin(simSOC1);
  remainingTime2Min = estimatedRemainingTimeMin(simSOC2);

  bool eligible[2] = {
    requested1 && simSOC1 < 99.5f && simBattery1V >= 3.0f && simBattery1V <= 4.3f,
    requested2 && simSOC2 < 99.5f && simBattery2V >= 3.0f && simBattery2V <= 4.3f
  };

  float maxPowerW[2] = {
    simBattery1V * SIM_MAX_TARGET_CURRENT_A,
    simBattery2V * SIM_MAX_TARGET_CURRENT_A
  };
  float scores[2] = {priorityScore1, priorityScore2};
  float allocatedW[2] = {0.0f, 0.0f};

  // Fail closed when MQTT is stale, status is not known, or grid power is unsafe.
  bool gridAllowsCharging = gridFresh() &&
      (gridStatus == "NORMAL" || gridStatus == "CONGESTION") &&
      allowedPowerW > 0.0f;
  bool allocationEligible[2] = {eligible[0], eligible[1]};
  if (gridAllowsCharging) {
    // First allocation. If either share is below the minimum usable current,
    // suspend that channel and redistribute the budget to the remaining one.
    for (int pass = 0; pass < 2; ++pass) {
      allocatedW[0] = allocatedW[1] = 0.0f;
      allocateWeightedPower(allowedPowerW, allocationEligible, scores, maxPowerW, allocatedW);
      bool removedUnderMinimum = false;
      if (allocationEligible[0] && allocatedW[0] > 0.0f &&
          allocatedW[0] < simBattery1V * MIN_COMMAND_CURRENT_A) {
        allocationEligible[0] = false;
        removedUnderMinimum = true;
      }
      if (allocationEligible[1] && allocatedW[1] > 0.0f &&
          allocatedW[1] < simBattery2V * MIN_COMMAND_CURRENT_A) {
        allocationEligible[1] = false;
        removedUnderMinimum = true;
      }
      if (!removedUnderMinimum) break;
    }
  }

  float newTarget1A = allocationEligible[0] && simBattery1V > 0.0f
      ? clampFloat(allocatedW[0] / simBattery1V, 0.0f, SIM_MAX_TARGET_CURRENT_A) : 0.0f;
  float newTarget2A = allocationEligible[1] && simBattery2V > 0.0f
      ? clampFloat(allocatedW[1] / simBattery2V, 0.0f, SIM_MAX_TARGET_CURRENT_A) : 0.0f;

  if (!gridAllowsCharging) {
    newTarget1A = 0.0f;
    newTarget2A = 0.0f;
  }

  // Small deadband applies only to tiny increases. Reductions, OFF commands,
  // and grid/status safety transitions are applied immediately so the station
  // never keeps a previous target that could exceed a reduced power budget.
  if (gridAllowsCharging && lastAllocationGridStatus == gridStatus) {
    if (newTarget1A > target1A && target1A >= MIN_COMMAND_CURRENT_A &&
        fabsf(newTarget1A - target1A) < 0.03f) newTarget1A = target1A;
    if (newTarget2A > target2A && target2A >= MIN_COMMAND_CURRENT_A &&
        fabsf(newTarget2A - target2A) < 0.03f) newTarget2A = target2A;
  }

  String newMode1 = currentLabel(newTarget1A);
  String newMode2 = currentLabel(newTarget2A);
  bool allocationChanged = fabsf(newTarget1A - target1A) >= 0.03f ||
                           fabsf(newTarget2A - target2A) >= 0.03f ||
                           lastAllocationGridStatus != gridStatus;

  target1A = newTarget1A;
  target2A = newTarget2A;
  target1W = target1A * simBattery1V;
  target2W = target2A * simBattery2V;
  mode1 = newMode1;
  mode2 = newMode2;
  port1CommandedOn = target1A >= MIN_COMMAND_CURRENT_A;
  port2CommandedOn = target2A >= MIN_COMMAND_CURRENT_A;

  substationOnline = gridFresh();
  budgetReceived = gridFresh();
  if (!gridFresh()) allowedPowerW = 0.0f;
  receivedBudgetW = allowedPowerW;

  if (!gridFresh()) demandResponse = "MQTT STALE / BOTH PORTS OFF";
  else if (gridStatus == "NORMAL") demandResponse = "NORMAL / SCORE-WEIGHTED ALLOCATION";
  else if (gridStatus == "CONGESTION") demandResponse = "CONGESTION / SCORE-WEIGHTED ALLOCATION";
  else demandResponse = "GRID SAFETY STATE / BOTH PORTS OFF";

  if (allocationChanged) {
    lastCommand = "alpha=0.70 beta=0.30; S1=" + String(priorityScore1, 3) +
      " S2=" + String(priorityScore2, 3) + "; P1=" + String(target1W, 2) +
      "W/" + String(target1A, 2) + "A P2=" + String(target2W, 2) +
      "W/" + String(target2A, 2) + "A; budget=" + String(allowedPowerW, 2) + "W";
    Serial.println(lastCommand);
    lastAllocationGridStatus = gridStatus;
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, MQTT_TOPIC) != 0) return;
  String message;
  message.reserve(length + 1);
  for (unsigned int i=0; i<length; i++) message += (char)payload[i];
  message.trim();

  // Current substation sketch sends JSON. Accept plain status text too,
  // so manual MQTTX testing with NORMAL/CONGESTION/OVERLOAD still works.
  String status;
  if (message.startsWith("{")) {
    status = jsonString(message, "grid_status", "UNKNOWN");
    gridVoltageFromMQTT = jsonNumber(message, "grid_voltage_v", 0.0f);
    gridCurrentFromMQTT = jsonNumber(message, "grid_current_a", 0.0f);
    gridPowerFromMQTT = jsonNumber(message, "grid_power_w", 0.0f);
    allowedPowerW = jsonNumber(message, "allowed_branch_power_w", 0.0f);
  } else {
    status = message;
    // A text-only MQTTX test has no grid power fields/budget.
    allowedPowerW = 0.0f;
  }

  if (status != "NORMAL" && status != "CONGESTION" && status != "OVERLOAD" && status != "FAULT") return;
  gridStatus = status;
  if (status == "OVERLOAD" || status == "FAULT") allowedPowerW = 0.0f;
  lastGridMqttMs = millis();
  applyAllocation();
}

void connectMQTT() {
  if (WiFi.status()!=WL_CONNECTED || mqtt.connected()) return;
  if (millis()-lastMqttTry < 3000) return;
  lastMqttTry=millis();
  String id="ChargeGrid-Station-"+String((uint32_t)ESP.getEfuseMac(),HEX);
  if (mqtt.connect(id.c_str())) {
    lastGridMqttMs=0;
    gridStatus="UNKNOWN";
    mqtt.subscribe(MQTT_TOPIC);
    Serial.println("[MQTT] Station subscribed");
  }
}

void handleSimulateStation() {
  if (!server.hasArg("v1") || !server.hasArg("v2") || !server.hasArg("soc1") || !server.hasArg("soc2") || !server.hasArg("wait1") || !server.hasArg("wait2") || !server.hasArg("i1") || !server.hasArg("i2")) {
    server.send(400,"application/json","{\"error\":\"Missing simulation fields\"}"); return;
  }
  float v1=server.arg("v1").toFloat(), v2=server.arg("v2").toFloat();
  float soc1=server.arg("soc1").toFloat(), soc2=server.arg("soc2").toFloat();
  float w1=server.arg("wait1").toFloat(), w2=server.arg("wait2").toFloat();
  float i1=server.arg("i1").toFloat(), i2=server.arg("i2").toFloat();
  if (v1<3||v1>4.3f||v2<3||v2>4.3f||soc1<0||soc1>100||soc2<0||soc2>100||w1<0||w1>1440||w2<0||w2>1440||i1<0||i1>5||i2<0||i2>5) {
    server.send(400,"application/json","{\"error\":\"Out of range\"}"); return;
  }
  simBattery1V=v1; simBattery2V=v2; simSOC1=soc1; simSOC2=soc2;
  simWait1Min=w1; simWait2Min=w2; simCurrent1A=i1; simCurrent2A=i2;
  applyAllocation();
  server.send(200,"application/json","{\"ok\":true}");
}

// ============================================================
// RELAY CONTROL
// ============================================================

void writeRelay(int pin, bool enabled) {
  // SIMULATION ONLY: intentionally no digitalWrite().
  (void)pin; (void)enabled;
}
void setPort(int port, bool enabled, const String& source) {
  if (port==1) requested1=enabled;
  else if (port==2) requested2=enabled;
  else return;
  lastCommand="Simulation: port "+String(port)+(enabled?" requested ON by ":" requested OFF by ")+source;
  applyAllocation();
}
bool budgetIsFresh() { return gridFresh(); }
void disableAllPorts(const String& reason) {
  mode1="OFF"; mode2="OFF";
  target1A=target2A=target1W=target2W=0.0f;
  port1CommandedOn=port2CommandedOn=false;
  lastCommand=reason;
  sampleSensors();
}

// ============================================================
// JSON HELPERS
// ============================================================

// Extract a numeric value from a simple JSON response.
float jsonNumber(
    const String& body,
    const char* key,
    float fallback) {

  String search = String("\"") + key + "\"";
  int pos = body.indexOf(search);

  if (pos < 0) return fallback;

  int colon = body.indexOf(':', pos + search.length());
  if (colon < 0) return fallback;

  int start = colon + 1;

  while (start < (int)body.length() &&
         (body[start] == ' ' || body[start] == '\t')) {
    start++;
  }

  int end = start;

  while (end < (int)body.length()) {
    char c = body[end];

    if ((c >= '0' && c <= '9') ||
        c == '-' || c == '+' ||
        c == '.' || c == 'e' || c == 'E') {
      end++;
    } else {
      break;
    }
  }

  if (end == start) return fallback;

  return body.substring(start, end).toFloat();
}

String jsonString(
    const String& body,
    const char* key,
    const String& fallback) {

  String search = String("\"") + key + "\"";
  int pos = body.indexOf(search);

  if (pos < 0) return fallback;

  int colon = body.indexOf(':', pos + search.length());
  if (colon < 0) return fallback;

  int quote1 = body.indexOf('"', colon + 1);
  if (quote1 < 0) return fallback;

  int quote2 = body.indexOf('"', quote1 + 1);
  if (quote2 < 0) return fallback;

  return body.substring(quote1 + 1, quote2);
}

// ============================================================
// STATION TELEMETRY JSON
// ============================================================

String makeTelemetryJSON() {
  String s;
  s.reserve(900);

  s = "{";

  s += "\"device\":\"EVCS_01\",";
  s += "\"simulation_mode\":true,";
  s += "\"grid_status\":\"" + gridStatus + "\",";
  s += "\"grid_voltage_v\":" + String(gridVoltageFromMQTT, 3) + ",";
  s += "\"grid_current_a\":" + String(gridCurrentFromMQTT, 3) + ",";
  s += "\"grid_power_w\":" + String(gridPowerFromMQTT, 3) + ",";
  s += "\"port1_mode\":\"" + mode1 + "\",";
  s += "\"port2_mode\":\"" + mode2 + "\",";
  s += "\"port1_target_current_a\":" + String(target1A,2) + ",";
  s += "\"port2_target_current_a\":" + String(target2A,2) + ",";
  s += "\"port1_target_power_w\":" + String(target1W,2) + ",";
  s += "\"port2_target_power_w\":" + String(target2W,2) + ",";
  s += "\"priority_score_1\":" + String(priorityScore1,3) + ",";
  s += "\"priority_score_2\":" + String(priorityScore2,3) + ",";
  s += "\"remaining_time_min_1\":" + String(remainingTime1Min,2) + ",";
  s += "\"remaining_time_min_2\":" + String(remainingTime2Min,2) + ",";
  s += "\"priority_alpha\":" + String(PRIORITY_ALPHA,2) + ",";
  s += "\"priority_beta\":" + String(PRIORITY_BETA,2) + ",";
  s += "\"port1_wait_min\":" + String(simWait1Min,1) + ",";
  s += "\"port2_wait_min\":" + String(simWait2Min,1) + ",";
  s += "\"battery1_v\":" + String(battery1V, 3) + ",";
  s += "\"battery2_v\":" + String(battery2V, 3) + ",";

  s += "\"battery1_soc_est\":" +
       String(simSOC1, 1) + ",";

  s += "\"battery2_soc_est\":" +
       String(simSOC2, 1) + ",";

  s += "\"port1_branch_current_a_measured\":" +
       String(branch1A, 3) + ",";

  s += "\"port2_branch_current_a_measured\":" +
       String(branch2A, 3) + ",";

  s += "\"port1_branch_input_power_w_calculated\":" +
       String(branch1W, 2) + ",";

  s += "\"port2_branch_input_power_w_calculated\":" +
       String(branch2W, 2) + ",";

  s += "\"port1_relay_commanded\":";
  s += port1CommandedOn ? "true," : "false,";

  s += "\"port2_relay_commanded\":";
  s += port2CommandedOn ? "true," : "false,";

  s += "\"substation_online\":";
  s += budgetIsFresh() ? "true," : "false,";

  s += "\"allowed_branch_power_w\":" +
       String(allowedPowerW, 1) + ",";
  s += "\"grid_power_w\":" + String(gridPowerFromMQTT, 2) + ",";

  s += "\"budget_w_received\":" +
       String(receivedBudgetW, 1) + ",";

  s += "\"last_command\":\"";
  s += lastCommand;
  s += "\"}";

  return s;
}

// ============================================================
// SUBSTATION COMMUNICATION
// ============================================================

// Grid status now arrives through MQTT, not HTTP /budget polling.

void postStationTelemetry() {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;

  String url = String("http://") + SUBSTATION_IP + "/telemetry";

  http.setTimeout(1000);

  if (!http.begin(url)) return;

  http.addHeader("Content-Type", "application/json");

  String payload = makeTelemetryJSON();
  int code = http.POST(payload);

  Serial.print("[TELEMETRY] HTTP status: ");
  Serial.println(code);

  if (code < 0) {
    Serial.println("[TELEMETRY] POST failed");
  }

  http.end();
}

// ============================================================
// PREMIUM DASHBOARD
// ============================================================

const char PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="theme-color" content="#06172f">
<title>ChargeGrid EV Station</title>
<style>
:root{
--bg:#041329;--cyan:#00d9ff;--cyan2:#64eaff;
--green:#00e6a8;--amber:#ffbd50;--red:#ff5575;
--muted:#9bb9d9;--border:#075b86
}
*{box-sizing:border-box}
body{
margin:0;color:#f1f8ff;font-family:Arial,sans-serif;
background:radial-gradient(ellipse at 10% 0%,#0a2b52,transparent 42%),var(--bg)
}
.topbar{
display:flex;justify-content:space-between;align-items:center;gap:12px;
padding:20px clamp(14px,4vw,42px);
background:#041329ed;border-bottom:1px solid #174365
}
.logo{
width:44px;height:44px;display:grid;place-items:center;border-radius:13px;
background:#063452;border:1px solid #087ba7;color:var(--cyan);font-size:27px
}
.brand{display:flex;align-items:center;gap:12px}
.brand h1{font-size:clamp(17px,3vw,23px);margin:0}
.brand p{font-size:12px;color:var(--muted);margin:5px 0 0}
.connection{font-size:12px;color:var(--muted);text-align:right}
.dot{display:inline-block;width:9px;height:9px;border-radius:50%;background:var(--amber);margin-right:7px}
.dot.online{background:var(--green)}
.dot.offline{background:var(--red)}
main{max-width:1400px;margin:auto;padding:25px clamp(12px,3vw,38px)}
.heading{display:flex;justify-content:space-between;align-items:end;gap:12px;flex-wrap:wrap;margin-bottom:18px}
.heading h2{font-size:27px;margin:0}
.heading p{font-size:13px;color:var(--muted);margin:7px 0 0}
.updated{font-size:12px;color:var(--muted)}
.overview{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:14px;margin-bottom:28px}
.overview-card{
background:linear-gradient(145deg,#0a2345,#06162e);
border:1px solid #124e76;border-radius:16px;padding:18px;min-width:0
}
.label{font-size:12px;color:var(--muted);margin-bottom:12px}
.big{font-size:clamp(22px,3vw,30px);font-weight:850;color:var(--cyan2);overflow-wrap:anywhere}
.sub{font-size:11px;color:#789bbf;margin-top:7px}
.ports{display:grid;grid-template-columns:1fr;gap:20px}
.port{
border:1px solid #0877a7;border-radius:20px;
padding:clamp(15px,2vw,25px);
background:radial-gradient(ellipse at 0% 100%,#06325265,transparent 45%),linear-gradient(145deg,#071d3b,#04162e);
box-shadow:0 12px 40px #0002
}
.port-head{display:flex;justify-content:space-between;align-items:start;gap:12px;margin-bottom:22px}
.port-title{font-size:29px;font-weight:900;margin:0}
.port-sub{font-size:13px;color:var(--muted);margin-top:6px}
.status{
padding:11px 14px;border-radius:30px;text-align:center;min-width:130px;
font-size:11px;font-weight:850;color:#ffd58b;background:#332714;border:1px solid #8b632b
}
.status.charging{color:#8dffe0;background:#063a34;border-color:#087f70}
.status.requested{color:#8deaff;background:#073248;border-color:#087b9c}
.status.off{color:#c0ccdc;background:#14243a;border-color:#31445d}
.status.fault{color:#ff9fb0;background:#3a1624;border-color:#8e2945}
.port-main{display:grid;grid-template-columns:125px minmax(0,1fr);gap:22px;align-items:center}
.battery-column{display:flex;justify-content:center;align-items:center;min-height:225px}
.battery{
position:relative;width:115px;height:215px;border:6px solid var(--cyan);
border-radius:25px;padding:8px;background:#04152b;
box-shadow:0 0 13px #00d9ff8c,0 0 32px #00d9ff32
}
.cap{position:absolute;top:-21px;left:30px;width:44px;height:15px;background:var(--cyan);border-radius:8px 8px 0 0}
.inner{position:relative;height:100%;overflow:hidden;border-radius:14px;background:#091c34}
.fill{
position:absolute;bottom:0;width:100%;height:0%;
background:linear-gradient(0deg,#087bff,#00d9ff);
transition:height .8s;border-radius:10px;box-shadow:0 0 23px #00d9ff64
}
.fill:before{content:"";position:absolute;top:-5px;left:-10%;width:120%;height:10px;border-radius:50%;background:#5cf3ff}
.fill.low{background:linear-gradient(0deg,#ff7b42,#ffc451)}
.fill.full{background:linear-gradient(0deg,#00a879,#00ffd0)}
.fill.charging{animation:glow 1.5s ease-in-out infinite alternate}
@keyframes glow{from{filter:brightness(.85)}to{filter:brightness(1.3)}}
.percent{position:absolute;top:50%;width:100%;transform:translateY(-50%);text-align:center;font-size:27px;font-weight:900;text-shadow:0 2px 5px #00152c}
.bolt{position:absolute;top:40%;width:100%;text-align:center;color:#ffffff75;font-size:22px}
.metrics{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:11px;min-width:0}
.metric{background:#021125df;border:1px solid #075d8b;border-radius:15px;padding:15px;min-height:91px;min-width:0}
.metric .label{color:#c0d9f1;margin-bottom:12px}
.metric .big{font-size:clamp(19px,2.5vw,30px);white-space:nowrap}
.unit{font-size:.65em;color:#9bdfff}
.progress-row{display:grid;grid-template-columns:1fr 48px;align-items:center;gap:14px;margin-top:22px}
.track{height:15px;border:1px solid #0078a6;background:#031126;border-radius:20px;overflow:hidden}
.progress{height:100%;width:0;background:linear-gradient(90deg,#00bce9,#00e8ff);transition:width .8s}
.pct{text-align:right;font-size:19px;font-weight:900}
.controls{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:20px}
button{border:0;border-radius:12px;min-height:53px;padding:13px;font-size:14px;font-weight:900;cursor:pointer}
button:disabled{opacity:.5;cursor:wait}
.on{background:var(--green);color:#001c24}
.offbtn{background:var(--amber);color:#261600}
.bottom{display:grid;grid-template-columns:1.2fr 1fr;gap:15px;margin-top:25px}
.panel{border:1px solid #124e76;border-radius:18px;padding:20px;background:#071a33;min-width:0}
.panel h3{font-size:16px;margin:0 0 17px}
.barline{display:flex;justify-content:space-between;gap:10px;color:var(--muted);font-size:12px;margin:14px 0 8px}
.bar{height:10px;border:1px solid #124e76;background:#031126;border-radius:20px;overflow:hidden}
.barfill{height:100%;width:0;background:linear-gradient(90deg,#00bce9,#00e6a8);transition:width .6s}
.notice{border:1px solid #17466b;background:#071a32;border-radius:12px;padding:13px;font-size:12px;line-height:1.6;color:#b8d0e9;overflow-wrap:anywhere}
.good{border-color:#087f70}.warn{border-color:#8b632b}.bad{border-color:#8e2945}
.footer{text-align:center;font-size:11px;color:#6687a8;padding:25px 10px;line-height:1.8}
.toast{position:fixed;bottom:20px;left:50%;transform:translate(-50%,20px);opacity:0;pointer-events:none;z-index:10;padding:13px 18px;border-radius:12px;background:#102d4b;border:1px solid #1678a1;transition:.25s;max-width:90vw;font-size:13px}
.toast.show{opacity:1;transform:translate(-50%,0)}
@media(max-width:720px){
.topbar{padding:15px}.overview{grid-template-columns:1fr}
.port-main{grid-template-columns:1fr;gap:20px}.battery-column{min-height:205px}
.battery{height:190px;width:105px}.bottom{grid-template-columns:1fr}
.metric{padding:12px}.metric .big{font-size:clamp(18px,5vw,25px)}
}
@media(max-width:390px){
.metrics{grid-template-columns:1fr}.controls{grid-template-columns:1fr}
.status{min-width:0;font-size:10px}
}
</style>
</head>
<body>
<header class="topbar">
<div class="brand">
<div class="logo">ϟ</div>
<div><h1>CHARGE<span style="color:var(--cyan)">GRID</span></h1>
<p>EV Charging Station Control Center</p></div>
</div>
<div class="connection"><span id="dot" class="dot"></span><span id="conn">CONNECTING</span><br>
<span style="display:inline-block;margin-top:8px">ESP32 · LIVE</span></div>
</header>

<main>
<div class="heading"><div><h2>Charging Overview</h2>
<p>Live battery telemetry and demand-response control</p></div>
<div class="updated">Last update: <span id="updated">Waiting...</span></div></div>

<div class="panel" style="margin-bottom:20px;border-color:var(--cyan)">
 <h3>Simulation inputs · Charging station ESP32</h3>
 <p class="sub">Enter synthetic battery voltage, SOC and accumulated wait. Branch-current fields model separate synthetic sensor feedback to the Substation; allocated current/power below come from the priority algorithm.</p>
 <form id="simStationForm" onsubmit="submitStation(event)" style="display:flex;flex-wrap:wrap;gap:12px;align-items:end">
 <label>Battery 1 V <input id="sv1" type="number" step="0.01" min="3" max="4.3" value="3.8" required></label>
 <label>Battery 1 SOC % <input id="ss1" type="number" step="1" min="0" max="100" value="50" required></label>
 <label>Wait 1 min <input id="sw1" type="number" step="1" min="0" max="1440" value="0" required></label>
 <label>Battery 2 V <input id="sv2" type="number" step="0.01" min="3" max="4.3" value="3.7" required></label>
 <label>Battery 2 SOC % <input id="ss2" type="number" step="1" min="0" max="100" value="35" required></label>
 <label>Wait 2 min <input id="sw2" type="number" step="1" min="0" max="1440" value="0" required></label>
 <label>Branch 1 current (A) <input id="si1" type="number" step="0.01" min="0" max="5" value="0" required></label>
 <label>Branch 2 current (A) <input id="si2" type="number" step="0.01" min="0" max="5" value="0" required></label>
 <button class="on" type="submit">Apply synthetic inputs</button>
 </form><p id="simStationResult" class="sub"></p>
 <p class="sub">MQTT grid: <strong id="mqttGrid">UNKNOWN</strong> · Port 1 target: <strong id="mode1">OFF</strong> · Port 2 target: <strong id="mode2">OFF</strong></p>
 <p class="sub">Priority weights: α = 0.70, β = 0.30 · Scores: Port 1 <strong id="score1">--</strong> (remaining time <strong id="remaining1">--</strong>), Port 2 <strong id="score2">--</strong> (remaining time <strong id="remaining2">--</strong>)</p>
 </div>
 <section class="overview">
<div class="overview-card"><div class="label">ALLOWED BRANCH POWER</div>
<div class="big"><span id="budget">--</span> <span class="unit">W</span></div>
<div class="sub">Budget reported by substation</div></div>
<div class="overview-card"><div class="label">TOTAL ALLOCATED CURRENT</div>
<div class="big"><span id="totalCurrent">--</span> <span class="unit">A</span></div>
<div class="sub">Sum of both branch sensors</div></div>
<div class="overview-card"><div class="label">SUBSTATION LINK</div>
<div class="big" id="substation" style="font-size:25px">--</div>
<div class="sub">Grid budget communication status</div></div>
</section>

<section class="ports">

<!-- PORT 1 -->
<article class="port">
<div class="port-head"><div><h2 class="port-title">PORT 01</h2>
<div class="port-sub">Li-ion Charging Port · Channel A</div></div>
<div class="status off" id="status1">WAITING</div></div>

<div class="port-main">
<div class="battery-column"><div class="battery"><div class="cap"></div>
<div class="inner"><div class="fill" id="fill1"></div><div class="bolt">ϟ</div>
<div class="percent" id="soc1">--%</div></div></div></div>
<div class="metrics">
<div class="metric"><div class="label">Battery Voltage</div><div class="big"><span id="v1">--</span> <span class="unit">V</span></div></div>
<div class="metric"><div class="label">Allocated Current (Target)</div><div class="big"><span id="i1">--</span> <span class="unit">A</span></div></div>
<div class="metric"><div class="label">Allocated Power (Target)</div><div class="big"><span id="p1">--</span> <span class="unit">W</span></div></div>
<div class="metric"><div class="label">Time Remaining (Estimate)</div><div class="big" id="t1">--</div></div>
</div></div>

<div class="progress-row"><div class="track"><div class="progress" id="bar1"></div></div><div class="pct" id="pct1">--%</div></div>
<div class="controls"><button class="on" id="on1" onclick="controlPort(1,1)">REQUEST ON</button>
<button class="offbtn" id="off1" onclick="controlPort(1,0)">REQUEST OFF</button></div>
</article>

<!-- PORT 2 -->
<article class="port">
<div class="port-head"><div><h2 class="port-title">PORT 02</h2>
<div class="port-sub">Li-ion Charging Port · Channel B</div></div>
<div class="status off" id="status2">WAITING</div></div>

<div class="port-main">
<div class="battery-column"><div class="battery"><div class="cap"></div>
<div class="inner"><div class="fill" id="fill2"></div><div class="bolt">ϟ</div>
<div class="percent" id="soc2">--%</div></div></div></div>
<div class="metrics">
<div class="metric"><div class="label">Battery Voltage</div><div class="big"><span id="v2">--</span> <span class="unit">V</span></div></div>
<div class="metric"><div class="label">Allocated Current (Target)</div><div class="big"><span id="i2">--</span> <span class="unit">A</span></div></div>
<div class="metric"><div class="label">Allocated Power (Target)</div><div class="big"><span id="p2">--</span> <span class="unit">W</span></div></div>
<div class="metric"><div class="label">Time Remaining (Estimate)</div><div class="big" id="t2">--</div></div>
</div></div>

<div class="progress-row"><div class="track"><div class="progress" id="bar2"></div></div><div class="pct" id="pct2">--%</div></div>
<div class="controls"><button class="on" id="on2" onclick="controlPort(2,1)">REQUEST ON</button>
<button class="offbtn" id="off2" onclick="controlPort(2,0)">REQUEST OFF</button></div>
</article>
</section>

<section class="bottom">
<div class="panel"><h3>Branch Power Allocation</h3>
<div class="barline"><span>Port 01 allocated power</span><strong id="bp1">-- W</strong></div>
<div class="bar"><div class="barfill" id="bbar1"></div></div>
<div class="barline"><span>Port 02 allocated power</span><strong id="bp2">-- W</strong></div>
<div class="bar"><div class="barfill" id="bbar2"></div></div>
<div class="barline"><span>Total allocated power</span><strong id="btotal">-- W</strong></div>
<div class="bar"><div class="barfill" id="bbarT"></div></div>
</div>
<div class="panel"><h3>System & Demand Response</h3>
<div class="notice" id="systemNotice">Waiting for telemetry...</div><div style="height:10px"></div>
<div class="notice" id="controlNotice">Waiting for controller state...</div>
</div>
</section>

<div class="footer">CHARGEGRID · ESP32 ENERGY MANAGEMENT<br>
Voltage-based SOC and time remaining are estimates. Branch power is provisional until branch voltage is measured.</div>
</main>
<div class="toast" id="toast"></div>

<script>
"use strict";

let busy = {1:false,2:false};
let toastTimer;

function text(id,value){
  document.getElementById(id).textContent = value;
}

function numberField(d,keys,fallback=null){
  for(const k of keys){
    if(d[k] !== undefined && d[k] !== null &&
       d[k] !== "" && Number.isFinite(Number(d[k]))){
      return Number(d[k]);
    }
  }
  return fallback;
}

function boolField(d,keys,fallback=false){
  for(const k of keys){
    if(d[k] === undefined || d[k] === null) continue;
    const v = d[k];
    if(typeof v === "boolean") return v;
    if(typeof v === "number") return v !== 0;
    return ["true","1","on","enabled","active"]
      .includes(String(v).toLowerCase());
  }
  return fallback;
}

function fmt(v,d=2){
  return v === null || !Number.isFinite(v) ? "--" : v.toFixed(d);
}

function toast(message){
  const el = document.getElementById("toast");
  el.textContent = message;
  el.classList.add("show");
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => el.classList.remove("show"),2800);
}

function timeRemaining(soc,v){
  // Matches the priority score's guide assumption: 8.14 Wh nominal battery, 3 W average effective power.
  // Illustrative only; later calibration should account for measured power and CC/CV taper.
  if(soc === null || v === null || soc >= 99.5 || v < 2.5) return "--";

  let minutes = 60 * 8.14 * (100 - soc) / 100 / 3.0;
  if(minutes > 1440) return ">24 h";
  if(minutes < 1) return "<1 min";

  const total = Math.ceil(minutes);
  const h = Math.floor(total/60);
  const m = total%60;

  if(h === 0) return m + " min";
  if(m === 0) return h + " h";
  return h + " h " + m + " min";
}

function setStatus(port,label,cls){
  const e = document.getElementById("status"+port);
  e.textContent = label;
  e.className = "status " + cls;
}

function renderPort(n,d,budget,online){
  const v = numberField(d,["battery"+n+"_v"]);
  const rawSOC = numberField(d,["battery"+n+"_soc_est"]);
  const soc = rawSOC === null ? null : Math.max(0,Math.min(100,rawSOC));

  const current = numberField(d,[
    "port"+n+"_target_current_a",
    "port"+n+"_branch_current_a_measured"
  ]);

  const power = numberField(d,[
    "port"+n+"_target_power_w",
    "port"+n+"_branch_input_power_w_calculated"
  ]);

  const relay = boolField(d,["port"+n+"_relay_commanded"]);
  const requested = boolField(d,["port"+n+"_requested"],relay);

  text("v"+n,fmt(v));
  text("i"+n,fmt(current));
  text("p"+n,fmt(power,1));
  text("t"+n,timeRemaining(soc,v));

  const percent = soc === null ? "--%" : Math.round(soc)+"%";
  text("soc"+n,percent);
  text("pct"+n,percent);

  const fill = document.getElementById("fill"+n);
  fill.style.height = (soc === null ? 0 : soc)+"%";
  fill.classList.toggle("low",soc !== null && soc < 20);
  fill.classList.toggle("full",soc !== null && soc >= 90);
  fill.classList.toggle("charging",relay && current !== null && current > 0.05);

  document.getElementById("bar"+n).style.width =
    (soc === null ? 0 : soc)+"%";

  if(!online || budget === null || budget <= 0){
    setStatus(n,"GRID UNAVAILABLE / OFF","fault");
  }else if(soc !== null && soc >= 99){
    setStatus(n,"TARGET SOC REACHED","off");
  }else if(relay && current !== null && current > 0.05){
    setStatus(n,"SIMULATED CHARGING","charging");
  }else if(relay){
    setStatus(n,"SIMULATED ON","requested");
  }else if(requested){
    setStatus(n,"REQUESTED · NOT ACTIVE","requested");
  }else{
    setStatus(n,"STANDBY","off");
  }

  document.getElementById("on"+n).disabled = busy[n];
  document.getElementById("off"+n).disabled = busy[n];

  text("bp"+n,fmt(power,1)+" W");
  document.getElementById("bbar"+n).style.width =
    Math.min(100,Math.max(0,(power||0)/Math.max(1,budget||0)*100))+"%";

  return {current:current,power:power};
}

function render(d){
  const budget = numberField(d,["allowed_branch_power_w"]);
  const online = boolField(d,["substation_online"]);

  const p1 = renderPort(1,d,budget,online);
  const p2 = renderPort(2,d,budget,online);

  text("budget",fmt(budget,1));
  text("substation",online ? "ONLINE" : "OFFLINE");
  document.getElementById("substation").style.color =
    online ? "var(--green)" : "var(--amber)";

  document.getElementById("dot").className =
    "dot " + (online ? "online" : "offline");
  text("conn",online ? "SUBSTATION ONLINE" : "SUBSTATION OFFLINE");
  text("updated",new Date().toLocaleTimeString());

  text("totalCurrent",fmt((p1.current||0)+(p2.current||0)));
  text("score1",fmt(numberField(d,["priority_score_1"]),3));
  text("score2",fmt(numberField(d,["priority_score_2"]),3));
  const rt1 = numberField(d,["remaining_time_min_1"]);
  const rt2 = numberField(d,["remaining_time_min_2"]);
  text("remaining1",rt1 === null ? "--" : Math.ceil(rt1)+" min");
  text("remaining2",rt2 === null ? "--" : Math.ceil(rt2)+" min");

  const totalPower = (p1.power||0)+(p2.power||0);
  text("btotal",fmt(totalPower,1)+" W");
  document.getElementById("bbarT").style.width =
    Math.min(100,totalPower/Math.max(1,budget||0)*100)+"%";

  const system = document.getElementById("systemNotice");
  const control = document.getElementById("controlNotice");

  if(!online){
    system.className = "notice bad";
    system.textContent =
      "Substation link unavailable. Check Wi-Fi, IP address and /budget endpoint.";
  }else if(budget === null || budget <= 0){
    system.className = "notice bad";
    system.textContent =
      "No charging power budget is available. Charging should remain disabled.";
  }else{
    system.className = "notice good";
    system.textContent = "Substation link available. Reported budget: "+
      fmt(budget,1)+" W. Grid status: "+String(d.grid_status||"UNKNOWN")+
      ". Demand response: "+String(d.demand_response||"UNKNOWN")+".";
  }

  control.className = "notice "+(!online || budget === null || budget <= 0 ? "warn":"good");
  control.textContent = "Controller: "+String(d.last_command||"No recent command")+".";

  if(budget !== null && totalPower > budget){
    control.className = "notice warn";
    control.textContent +=
      " Estimated branch power exceeds the displayed budget. Verify sensor placement and power calculations.";
  }
}

async function submitStation(e){
 e.preventDefault();
 const names=['v1','soc1','wait1','v2','soc2','wait2','i1','i2'];
 const ids=['sv1','ss1','sw1','sv2','ss2','sw2','si1','si2'];
 const params=new URLSearchParams();
 names.forEach((n,i)=>params.set(n,document.getElementById(ids[i]).value));
 try {const r=await fetch('/simulate?'+params.toString());
 document.getElementById('simStationResult').textContent=r.ok?'Synthetic readings updated':'Invalid input'; await refresh();}
 catch(err){document.getElementById('simStationResult').textContent='Request failed';}
}
async function refresh(){
  try{
    const response = await fetch("/data?t="+Date.now(),{cache:"no-store"});
    if(!response.ok) throw new Error("HTTP "+response.status);
    const d = await response.json();
    render(d);
    text("mqttGrid",d.grid_status||"UNKNOWN");
    text("mode1",d.port1_mode||"OFF");
    text("mode2",d.port2_mode||"OFF");
  }catch(error){
    document.getElementById("dot").className = "dot offline";
    text("conn","STATION DATA UNAVAILABLE");
    text("updated","Connection lost");

    const n = document.getElementById("systemNotice");
    n.className = "notice bad";
    n.textContent = "Cannot read /data. Check the ESP32 and its Wi-Fi connection.";
  }
}

async function controlPort(port,enable){
  if(busy[port]) return;

  busy[port] = true;
  document.getElementById("on"+port).disabled = true;
  document.getElementById("off"+port).disabled = true;
  toast("Sending Port "+port+" request...");

  try{
    const response = await fetch(
      "/control?port="+port+"&enable="+enable,
      {cache:"no-store"}
    );

    const message = await response.text();

    if(!response.ok){
      toast("Request rejected: "+message);
    }else{
      toast("Port "+port+" request accepted");
    }

    await refresh();
  }catch(error){
    toast("Control request failed. Check station connection.");
  }finally{
    busy[port] = false;
    document.getElementById("on"+port).disabled = false;
    document.getElementById("off"+port).disabled = false;
  }
}

refresh();
setInterval(refresh,2000);
</script>
</body>
</html>
)HTML";

// ============================================================
// HTTP HANDLERS
// ============================================================

void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", PAGE);
}

void handleData() {
  server.sendHeader("Cache-Control","no-store");
  server.send(200,"application/json",makeTelemetryJSON());
}

void handleControl() {
  if (!server.hasArg("port") || !server.hasArg("enable")) {
    server.send(400,"application/json",
                "{\"error\":\"Missing port or enable parameter\"}");
    return;
  }

  int port = server.arg("port").toInt();
  int enableValue = server.arg("enable").toInt();

  if ((port != 1 && port != 2) ||
      (enableValue != 0 && enableValue != 1)) {
    server.send(400,"application/json",
                "{\"error\":\"Use port=1|2 and enable=0|1\"}");
    return;
  }

  bool enable = enableValue == 1;

  // Simulation request only. Grid/MQTT policy can still force OFF.
  setPort(port,enable,"dashboard");

  server.send(200,"application/json",
              "{\"accepted\":true,\"relay_feedback\":\"not fitted\"}");
}

// ============================================================
// WIFI
// ============================================================

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID,WIFI_PASSWORD);

  Serial.print("Connecting to Wi-Fi");

  unsigned long start = millis();

  while (WiFi.status() != WL_CONNECTED &&
         millis()-start < 20000) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Station dashboard IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Wi-Fi unavailable. Relays remain OFF.");
  }
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(300);

  // SIMULATION: GPIOs deliberately NOT configured as relay outputs.
  // pinMode(RELAY1_PIN,OUTPUT);
  // pinMode(RELAY2_PIN,OUTPUT);

  writeRelay(RELAY1_PIN,false);
  writeRelay(RELAY2_PIN,false);

  port1CommandedOn = false;
  port2CommandedOn = false;

  analogReadResolution(12);

  analogSetPinAttenuation(BAT1_PIN,ADC_11db);
  analogSetPinAttenuation(BAT2_PIN,ADC_11db);
  analogSetPinAttenuation(ACS1_PIN,ADC_11db);
  analogSetPinAttenuation(ACS2_PIN,ADC_11db);

  Serial.println();
  Serial.println("==================================");
  Serial.println(" CHARGEGRID EV CHARGING STATION");
  Serial.println("==================================");

  Serial.println("SIMULATION: ACS712 and battery ADC reads disabled.");
  sampleSensors();

  connectWiFi();

  server.on("/",HTTP_GET,handleRoot);
  server.on("/data",HTTP_GET,handleData);
  server.on("/control",HTTP_GET,handleControl);
  server.on("/simulate",HTTP_GET,handleSimulateStation);

  server.onNotFound([](){
    server.send(404,"text/plain","Endpoint not found");
  });

  mqtt.setServer(MQTT_HOST,MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  server.begin();

  Serial.println("HTTP dashboard server started.");

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Open http://");
    Serial.println(WiFi.localIP());
  }

  lastCommand = "Startup complete; both ports OFF";
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  server.handleClient();
  connectMQTT();
  mqtt.loop();
  unsigned long now=millis();
  static unsigned long lastAllocationUpdate = 0;
  if (now - lastAllocationUpdate >= ALLOCATION_INTERVAL_MS) {
    lastAllocationUpdate = now;
    applyAllocation();
  }
  if (now-lastTelemetryPost >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryPost=now;
    postStationTelemetry(); // Existing HTTP substation dashboard telemetry remains.
  }
  static unsigned long lastPrint=0;
  if (now-lastPrint>=2000) {
    lastPrint=now;
    Serial.printf("[SIM] Grid=%s fresh=%d | P1=%s %.1fA | P2=%s %.1fA\n",
      gridStatus.c_str(),gridFresh(),mode1.c_str(),target1A,mode2.c_str(),target2A);
  }
  delay(2);
}
