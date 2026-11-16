// SIMULATION ONLY: no GPIO relay switching or physical charging.
// Synthetic inputs entered on HTTP dashboards; MQTT uses public test broker.
#include <WiFi.h>
#include <PubSubClient.h>
#include <WebServer.h>
#include <math.h>

// =====================================================
// 1. WI-FI CONFIGURATION
// =====================================================
const char* WIFI_SSID = "Athulya's Galaxy";
const char* WIFI_PASSWORD = "12345678";

// =====================================================
// 2. GRID SENSOR PINS
// =====================================================
const int GRID_VOLT_PIN = 34;
const int GRID_ACS_PIN  = 35;

// Voltage divider: 100k upper + 10k lower
const float GRID_DIVIDER_RATIO = 11.0f;

// ACS712-5A configuration
const float ACS_OUTPUT_SCALE = 1.5f;
const float ACS_SENSITIVITY  = 0.185f;

// Nominal supply
const float SOURCE_LIMIT_V = 30.0f;
const float SOURCE_LIMIT_A = 2.0f;

// Prototype thresholds
const float WARN_CURRENT_A = 1.70f;
const float TRIP_CURRENT_A = 1.90f;

// Dynamic branch power allowance: 75% of entered/calculated grid power.
const float BRANCH_POWER_FRACTION = 0.75f;
const float OVERLOAD_BUDGET_W = 0.0f;

// =====================================================
// 3. SERVER AND SENSOR VARIABLES
// =====================================================
WebServer server(80);
WiFiClient mqttNet;
PubSubClient mqtt(mqttNet);
const char* MQTT_HOST = "broker.hivemq.com";
const int MQTT_PORT = 1883;
const char* MQTT_TOPIC = "vk/ev/grid_status";
unsigned long lastMqttTry = 0, lastMqttPublish = 0;
const unsigned long MQTT_PUBLISH_INTERVAL_MS = 1000;
// Synthetic grid measurements; modify using /simulate (dashboard form).
float simulatedGridV = 30.0f;
float simulatedGridA = 1.00f;


float acsZeroAdcV = 1.65f;

float gridVoltageV = 0.0f;
float gridCurrentA = 0.0f;
float gridPowerW = 0.0f;
float gridLoadingPercent = 0.0f;
float allowedBranchPowerW = 0.0f;
float totalBranchPowerW = 0.0f;

bool gridVoltageValid = false;

String gridStatus = "STARTING";
String demandResponse = "INITIALIZING";

// Station telemetry
float battery1V = 0.0f;
float battery2V = 0.0f;
float battery1SOC = 0.0f;
float battery2SOC = 0.0f;

float branch1CurrentA = 0.0f;
float branch2CurrentA = 0.0f;
float branch1PowerW = 0.0f;
float branch2PowerW = 0.0f;

bool relay1Commanded = false;
bool relay2Commanded = false;
bool stationTelemetryReceived = false;

String stationDevice = "EV Charging Station";
String lastCommand = "No telemetry received";

unsigned long lastGridSampleMs = 0;
unsigned long lastTelemetryMs = 0;

const unsigned long GRID_SAMPLE_INTERVAL_MS = 500;
const unsigned long TELEMETRY_TIMEOUT_MS = 10000;

// =====================================================
// 4. DASHBOARD HTML
// The PAGE declaration fixes the original compile error.
// =====================================================
const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Grid Substation Monitor</title>

<style>
:root {
  --bg:#07111f;
  --panel:#101e32;
  --panel2:#142640;
  --border:#233b58;
  --cyan:#38d9ff;
  --blue:#478bff;
  --green:#3ce6a0;
  --yellow:#ffd166;
  --red:#ff647c;
  --text:#edf5ff;
  --muted:#91a6c2;
}
* { box-sizing:border-box; }
body {
  margin:0;
  font-family:Arial,Helvetica,sans-serif;
  background:linear-gradient(140deg,#06101d,#0b1930 55%,#07111f);
  color:var(--text);
}
header {
  padding:24px 5%;
  border-bottom:1px solid var(--border);
  background:#091628;
}
.headerrow {
  display:flex;
  justify-content:space-between;
  align-items:center;
  flex-wrap:wrap;
  gap:12px;
}
h1 { margin:0;font-size:25px; }
.subtitle { color:var(--muted);margin-top:7px;font-size:13px; }
.pill {
  display:inline-block;
  padding:8px 13px;
  border:1px solid var(--border);
  border-radius:30px;
  font-size:12px;
  color:var(--cyan);
  background:#10243a;
}
main { max-width:1250px;margin:auto;padding:24px 5%; }
.section-title { font-size:17px;margin:25px 0 14px; }
.grid {
  display:grid;
  grid-template-columns:repeat(auto-fit,minmax(205px,1fr));
  gap:15px;
}
.card {
  background:linear-gradient(145deg,var(--panel),#0c192a);
  border:1px solid var(--border);
  border-radius:16px;
  padding:19px;
  box-shadow:0 8px 28px #0002;
}
.label {
  color:var(--muted);
  font-size:12px;
  letter-spacing:.6px;
  text-transform:uppercase;
}
.value {
  font-size:30px;
  font-weight:700;
  margin:13px 0 6px;
  overflow-wrap:anywhere;
}
.unit { font-size:14px;color:var(--muted); }
.small { color:var(--muted);font-size:12px;line-height:1.6; }
.status {
  display:inline-block;
  border-radius:20px;
  padding:7px 11px;
  font-size:12px;
  font-weight:bold;
  background:#23364c;
}
.good { color:var(--green); }
.warn { color:var(--yellow); }
.bad { color:var(--red); }
.cyan { color:var(--cyan); }
.bar {
  height:9px;
  border-radius:10px;
  background:#24374d;
  margin-top:13px;
  overflow:hidden;
}
.fill {
  height:100%;
  width:0%;
  border-radius:10px;
  background:linear-gradient(90deg,var(--blue),var(--cyan));
  transition:width .3s;
}
.battery {
  height:72px;
  border:3px solid #8098b5;
  border-radius:9px;
  padding:5px;
  position:relative;
  margin:18px 8px 14px 0;
}
.battery:after {
  content:"";
  position:absolute;
  right:-9px;
  top:21px;
  height:23px;
  width:6px;
  background:#8098b5;
  border-radius:0 3px 3px 0;
}
.battery-fill {
  height:100%;
  width:0%;
  background:linear-gradient(90deg,#178be9,var(--cyan));
  border-radius:4px;
  transition:width .4s;
}
.batteryrow {
  display:grid;
  grid-template-columns:repeat(auto-fit,minmax(230px,1fr));
  gap:15px;
}
.kv {
  display:flex;
  justify-content:space-between;
  gap:12px;
  padding:9px 0;
  border-bottom:1px solid #20334a;
  font-size:13px;
}
.kv span:first-child { color:var(--muted); }
footer {
  color:var(--muted);
  font-size:12px;
  text-align:center;
  padding:22px;
}
@media(max-width:480px) {
  h1 { font-size:21px; }
  .value { font-size:26px; }
  main { padding:18px 4%; }
}
</style>
</head>

<body>
<header>
  <div class="headerrow">
    <div>
      <h1>Grid Substation Monitor</h1>
      <div class="subtitle">
        ESP32 grid telemetry and charging-station power management
      </div>
    </div>
    <span class="pill" id="connection">CONNECTING...</span>
  </div>
</header>

<main>
  <div class="card" style="margin-bottom:18px;border-color:#38d9ff">
 <div class="section-title" style="margin-top:0">Simulation input · Grid ESP32</div>
 <p class="small">No physical sensors. Enter synthetic grid readings. Current thresholds: NORMAL below 1.70 A; CONGESTION 1.70–1.89 A; OVERLOAD from 1.90 A.</p>
 <form id="simGridForm" onsubmit="submitGrid(event)">
 <label>Grid voltage (V) <input id="simGridV" type="number" step="0.1" min="0" max="40" value="30" required></label>
 <label style="margin-left:12px">Grid current (A) <input id="simGridA" type="number" step="0.01" min="0" max="5" value="1.00" required></label>
 <p class="small">Branch currents are entered on the charging-station dashboard. Branch power = battery voltage × branch current; total branch power is compared with the 75% grid-power allowance.</p>
 <button type="submit">Apply synthetic readings</button>
 </form><p id="simGridResult" class="small"></p>
 </div>
 <div class="section-title">Grid Overview</div>

  <div class="grid">
    <div class="card">
      <div class="label">Grid Voltage</div>
      <div class="value cyan" id="voltage">--</div>
      <div class="unit">Volts DC</div>
    </div>

    <div class="card">
      <div class="label">Grid Current</div>
      <div class="value" id="current">--</div>
      <div class="unit">Amperes</div>
    </div>

    <div class="card">
      <div class="label">Grid Power</div>
      <div class="value" id="power">--</div>
      <div class="unit">Watts</div>
    </div>

    <div class="card">
      <div class="label">Grid Status</div>
      <div class="value" id="statusValue" style="font-size:23px">--</div>
      <div class="small" id="demand">Waiting for measurements</div>
    </div>
  </div>

  <div class="section-title">Grid Loading and Power Allocation</div>

  <div class="grid">
    <div class="card">
      <div class="label">Source Current Loading</div>
      <div class="value" id="loading">--%</div>
      <div class="bar"><div class="fill" id="loadingBar"></div></div>
      <div class="small">Based on the configured 2 A source limit.</div>
    </div>

    <div class="card">
      <div class="label">Allowed Branch Power</div>
      <div class="value cyan" id="budget">-- W</div>
      <div class="bar"><div class="fill" id="budgetBar"></div></div>
      <div class="small">Configured allocation for the charging station.</div>
    </div>

    <div class="card">
      <div class="label">Measured Station Branch Power</div>
      <div class="value" id="stationPower">-- W</div>
      <div class="small">Sum of station-reported branch power estimates.</div>
    </div>

    <div class="card">
      <div class="label">Station Telemetry</div>
      <div style="margin-top:14px">
        <span class="status" id="stationStatus">OFFLINE</span>
      </div>
      <div class="small" id="stationTime" style="margin-top:12px">
        No recent telemetry
      </div>
    </div>
  </div>

  <div class="section-title">EV Charging Station</div>

  <div class="batteryrow">
    <div class="card">
      <div class="label">Battery / Port 1</div>
      <div class="battery">
        <div class="battery-fill" id="b1bar"></div>
      </div>
      <div class="value cyan" id="soc1" style="font-size:27px">--%</div>
      <div class="kv"><span>Battery Voltage</span><strong id="bv1">-- V</strong></div>
      <div class="kv"><span>Branch Current</span><strong id="bc1">-- A</strong></div>
      <div class="kv"><span>Branch Power</span><strong id="bp1">-- W</strong></div>
      <div class="kv"><span>Relay Command</span><strong id="r1">--</strong></div>
    </div>

    <div class="card">
      <div class="label">Battery / Port 2</div>
      <div class="battery">
        <div class="battery-fill" id="b2bar"></div>
      </div>
      <div class="value cyan" id="soc2" style="font-size:27px">--%</div>
      <div class="kv"><span>Battery Voltage</span><strong id="bv2">-- V</strong></div>
      <div class="kv"><span>Branch Current</span><strong id="bc2">-- A</strong></div>
      <div class="kv"><span>Branch Power</span><strong id="bp2">-- W</strong></div>
      <div class="kv"><span>Relay Command</span><strong id="r2">--</strong></div>
    </div>
  </div>

  <div class="section-title">System Information</div>
  <div class="card">
    <div class="kv"><span>Demand Response Mode</span><strong id="mode">--</strong></div>
    <div class="kv"><span>Last Station Command</span><strong id="command">--</strong></div>
    <div class="kv"><span>Station Device</span><strong id="device">--</strong></div>
    <div class="kv"><span>Last Dashboard Update</span><strong id="updated">--</strong></div>
  </div>

  <div class="small" style="margin-top:18px">
    Prototype monitoring only. Battery SOC is voltage-estimated, branch
    measurements depend on sensor calibration, and relay states are commanded
    states rather than confirmed contact feedback.
  </div>
</main>

<footer>ESP32 Smart Grid and IoT Project · Local Network Dashboard</footer>

<script>
async function submitGrid(e) {
 e.preventDefault();
 const v=document.getElementById('simGridV').value, a=document.getElementById('simGridA').value;
 try { const r=await fetch('/simulate?voltage='+encodeURIComponent(v)+'&current='+encodeURIComponent(a));
 const j=await r.json(); document.getElementById('simGridResult').textContent=r.ok?'Applied: '+j.grid_status:JSON.stringify(j); updateDashboard(); }
 catch(err){document.getElementById('simGridResult').textContent='Request failed';}
}
function put(id, value) {
  document.getElementById(id).textContent = value;
}
function number(v, digits) {
  const n = Number(v);
  return Number.isFinite(n) ? n.toFixed(digits) : "--";
}
function clamp(v, lo, hi) {
  return Math.max(lo, Math.min(hi, Number(v) || 0));
}
function setBar(id, percent) {
  document.getElementById(id).style.width =
    clamp(percent, 0, 100) + "%";
}
function setStatusClass(id, status) {
  const el = document.getElementById(id);
  el.className = "";
  if (status === "NORMAL" || status === "ONLINE") el.classList.add("good");
  else if (status === "CONGESTION" || status === "WARNING") el.classList.add("warn");
  else el.classList.add("bad");
}
async function updateDashboard() {
  try {
    const response = await fetch("/data", {cache:"no-store"});
    if (!response.ok) throw new Error("HTTP " + response.status);
    const d = await response.json();

    put("connection", "ESP32 ONLINE");
    put("voltage", number(d.grid_voltage_v, 2));
    put("current", number(d.grid_current_a, 2));
    put("power", number(d.grid_power_w, 2));

    put("statusValue", d.grid_status);
    setStatusClass("statusValue", d.grid_status);
    put("demand", d.demand_response);

    put("loading", number(d.grid_loading_percent, 1) + "%");
    setBar("loadingBar", d.grid_loading_percent);

    put("budget", number(d.allowed_branch_power_w, 1) + " W");
    setBar("budgetBar", Number(d.grid_power_w) > 0 ? Number(d.allowed_branch_power_w) / (Number(d.grid_power_w) * 0.75) * 100 : 0);

    put("stationPower", number(d.station_branch_power_w, 2) + " W");

    const online = !!d.station_online;
    put("stationStatus", online ? "ONLINE" : "OFFLINE");
    document.getElementById("stationStatus").className =
      "status " + (online ? "good" : "bad");
    put("stationTime", online ? "Receiving recent station telemetry" :
      "No telemetry received in the last 10 seconds");

    put("bv1", number(d.battery1_v, 3) + " V");
    put("bv2", number(d.battery2_v, 3) + " V");

    const s1 = clamp(d.battery1_soc_est, 0, 100);
    const s2 = clamp(d.battery2_soc_est, 0, 100);
    put("soc1", number(s1, 0) + "%");
    put("soc2", number(s2, 0) + "%");
    setBar("b1bar", s1);
    setBar("b2bar", s2);

    put("bc1", number(d.port1_branch_current_a_measured, 2) + " A");
    put("bc2", number(d.port2_branch_current_a_measured, 2) + " A");
    put("bp1", number(d.port1_branch_input_power_w_calculated, 2) + " W");
    put("bp2", number(d.port2_branch_input_power_w_calculated, 2) + " W");

    put("r1", d.port1_relay_commanded ? "ON command" : "OFF command");
    put("r2", d.port2_relay_commanded ? "ON command" : "OFF command");

    put("mode", d.demand_response);
    put("command", d.last_command || "--");
    put("device", d.station_device || "EV Charging Station");
    put("updated", new Date().toLocaleTimeString());
  } catch (e) {
    put("connection", "CONNECTION ERROR");
  }
}
updateDashboard();
setInterval(updateDashboard, 2000);
</script>
</body>
</html>
)rawliteral";

// =====================================================
// 5. ADC HELPERS
// =====================================================
float readAdcVolts(int pin) {
  uint32_t totalMilliVolts = 0;
  const int samples = 24;

  for (int i = 0; i < samples; i++) {
    totalMilliVolts += analogReadMilliVolts(pin);
    delayMicroseconds(200);
  }

  return (totalMilliVolts / (float)samples) / 1000.0f;
}

float calibrateACSZero(int pin) {
  uint32_t totalMilliVolts = 0;
  const int samples = 250;

  for (int i = 0; i < samples; i++) {
    totalMilliVolts += analogReadMilliVolts(pin);
    delay(2);
  }

  return (totalMilliVolts / (float)samples) / 1000.0f;
}

float readACSCurrent(int pin, float zeroAdcVoltage) {
  float measuredAdcV = readAdcVolts(pin);

  // Restore the sensor-side voltage after the external
  // 10k series / 20k ground conditioning network.
  float sensorVoltage = measuredAdcV * ACS_OUTPUT_SCALE;
  float sensorZero = zeroAdcVoltage * ACS_OUTPUT_SCALE;

  float current = (sensorVoltage - sensorZero) / ACS_SENSITIVITY;

  // This installation measures current magnitude only.
  current = fabsf(current);

  if (current < 0.08f) current = 0.0f;
  return current;
}

// =====================================================
// 6. JSON HELPERS
// Simple parser for the station's flat JSON telemetry.
// =====================================================
String jsonValue(String json, const char* key) {
  String searchKey = String("\"") + key + "\"";
  int keyPos = json.indexOf(searchKey);
  if (keyPos < 0) return "";

  int colonPos = json.indexOf(':', keyPos + searchKey.length());
  if (colonPos < 0) return "";

  int start = colonPos + 1;
  while (start < (int)json.length() &&
         (json[start] == ' ' || json[start] == '\n' ||
          json[start] == '\r' || json[start] == '\t')) {
    start++;
  }

  if (start >= (int)json.length()) return "";

  if (json[start] == '"') {
    int end = json.indexOf('"', start + 1);
    if (end < 0) return "";
    return json.substring(start + 1, end);
  }

  int end = start;
  while (end < (int)json.length() &&
         json[end] != ',' && json[end] != '}' &&
         json[end] != '\n' && json[end] != '\r') {
    end++;
  }

  String result = json.substring(start, end);
  result.trim();
  return result;
}

float jsonFloat(String json, const char* key, float fallback = 0.0f) {
  String value = jsonValue(json, key);
  if (value.length() == 0) return fallback;

  char* endPtr = nullptr;
  float parsed = strtof(value.c_str(), &endPtr);
  if (endPtr == value.c_str() || !isfinite(parsed)) return fallback;
  return parsed;
}

bool jsonBool(String json, const char* key, bool fallback = false) {
  String value = jsonValue(json, key);
  value.toLowerCase();

  if (value == "true" || value == "1") return true;
  if (value == "false" || value == "0") return false;

  return fallback;
}

String jsonEscape(String value) {
  value.replace("\\", "\\\\");
  value.replace("\"", "\\\"");
  value.replace("\n", "\\n");
  value.replace("\r", "\\r");
  return value;
}

// =====================================================
// 7. GRID SAMPLING AND BUDGET POLICY
// =====================================================
void sampleGridSensors() {
  gridVoltageV = simulatedGridV;
  gridCurrentA = simulatedGridA;
  gridVoltageValid = isfinite(gridVoltageV) && gridVoltageV >= 1.0f && gridVoltageV <= 40.0f;
  if (!gridVoltageValid || !isfinite(gridCurrentA) || gridCurrentA < 0) {
    gridPowerW = 0; gridLoadingPercent = 0; allowedBranchPowerW = 0;
    gridStatus = "VOLTAGE FAULT";
    demandResponse = "SIMULATED INPUT FAULT";
    return;
  }
  gridPowerW = gridVoltageV * gridCurrentA;
  gridLoadingPercent = gridCurrentA / SOURCE_LIMIT_A * 100.0f;
  // Calculate the branch allowance from the synthetic grid inputs.
  // Overload/fault remains a safety override and sets the allowance to zero.
  allowedBranchPowerW = gridPowerW * BRANCH_POWER_FRACTION;
  // Branch powers are calculated from the station's synthetic battery
  // voltages and branch-current inputs received through HTTP telemetry.
  branch1PowerW = battery1V * branch1CurrentA;
  branch2PowerW = battery2V * branch2CurrentA;
  totalBranchPowerW = branch1PowerW + branch2PowerW;

  if (gridCurrentA >= TRIP_CURRENT_A) {
    allowedBranchPowerW = OVERLOAD_BUDGET_W;
    gridStatus = "OVERLOAD";
    demandResponse = "SOURCE OVERLOAD: BOTH PORTS OFF";
  } else if (totalBranchPowerW > allowedBranchPowerW) {
    gridStatus = "CONGESTION";
    demandResponse = "BRANCH POWER EXCEEDS 75% GRID BUDGET";
  } else if (gridCurrentA >= WARN_CURRENT_A) {
    gridStatus = "CONGESTION";
    demandResponse = "SOURCE CURRENT WARNING: PRIORITY ALLOCATION";
  } else {
    gridStatus = "NORMAL";
    demandResponse = "BRANCH POWER WITHIN 75% GRID BUDGET";
  }
}

void connectMQTT() {
  if (WiFi.status() != WL_CONNECTED || mqtt.connected()) return;
  if (millis() - lastMqttTry < 3000) return;
  lastMqttTry = millis();
  String id = "ChargeGrid-Grid-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  if (mqtt.connect(id.c_str())) {
    Serial.println("[MQTT] Grid publisher connected");
    lastMqttPublish = 0;
  }
}

void publishGridStatus() {
  if (!mqtt.connected()) return;
  // Send the grid condition and its dynamically calculated 75% power budget.
  String status = (gridStatus == "NORMAL" || gridStatus == "CONGESTION" || gridStatus == "OVERLOAD")
                    ? gridStatus : "FAULT";
  String payload = "{";
  payload += "\"grid_status\":\"" + status + "\",";
  payload += "\"grid_voltage_v\":" + String(gridVoltageV, 3) + ",";
  payload += "\"grid_current_a\":" + String(gridCurrentA, 3) + ",";
  payload += "\"grid_power_w\":" + String(gridPowerW, 3) + ",";
  payload += "\"allowed_branch_power_w\":" + String(allowedBranchPowerW, 2) + ",";
  payload += "\"branch1_power_w\":" + String(branch1PowerW, 2) + ",";
  payload += "\"branch2_power_w\":" + String(branch2PowerW, 2) + ",";
  payload += "\"total_branch_power_w\":" + String(totalBranchPowerW, 2);
  payload += "}";
  mqtt.publish(MQTT_TOPIC, payload.c_str(), false);
}

void handleSimulateGrid() {
  if (!server.hasArg("voltage") || !server.hasArg("current")) {
    server.send(400, "application/json", "{\"error\":\"Need voltage and current\"}"); return;
  }
  float v = server.arg("voltage").toFloat();
  float a = server.arg("current").toFloat();
  if (!isfinite(v) || !isfinite(a) || v < 0 || v > 40 || a < 0 || a > 5) {
    server.send(400, "application/json", "{\"error\":\"Voltage 0-40V, current 0-5A\"}"); return;
  }
  simulatedGridV = v; simulatedGridA = a;
  sampleGridSensors(); publishGridStatus();
  server.send(200, "application/json", "{\"ok\":true,\"grid_status\":\"" + gridStatus + "\",\"grid_power_w\":" + String(gridPowerW, 2) + ",\"allowed_branch_power_w\":" + String(allowedBranchPowerW, 2) + "}");
}

// =====================================================
// 8. STATION ONLINE CHECK
// =====================================================
bool stationOnline() {
  return stationTelemetryReceived &&
         (millis() - lastTelemetryMs <= TELEMETRY_TIMEOUT_MS);
}

// =====================================================
// 9. ROOT DASHBOARD
// =====================================================
void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", PAGE);
}

// =====================================================
// 10. /BUDGET ENDPOINT
// The charging-station ESP32 polls this endpoint.
// =====================================================
void handleBudget() {
  String json = "{";

  json += "\"simulation_mode\":true,";
  json += "\"grid_voltage_v\":" + String(gridVoltageV, 3) + ",";
  json += "\"grid_current_a\":" + String(gridCurrentA, 3) + ",";
  json += "\"grid_power_w\":" + String(gridPowerW, 3) + ",";
  json += "\"grid_loading_percent\":" + String(gridLoadingPercent, 2) + ",";
  json += "\"grid_status\":\"" + jsonEscape(gridStatus) + "\",";
  json += "\"demand_response\":\"" + jsonEscape(demandResponse) + "\",";
  json += "\"allowed_branch_power_w\":" + String(allowedBranchPowerW, 2) + ",";
  json += "\"branch1_power_w\":" + String(branch1PowerW, 2) + ",";
  json += "\"branch2_power_w\":" + String(branch2PowerW, 2) + ",";
  json += "\"total_branch_power_w\":" + String(totalBranchPowerW, 2) + ",";
  json += "\"station_online\":" + String(stationOnline() ? "true" : "false");
  json += "}";

  server.send(200, "application/json", json);
}

// =====================================================
// 11. POST /TELEMETRY
// Receives the station's JSON payload.
// =====================================================
void handleTelemetry() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json",
                "{\"ok\":false,\"error\":\"Missing JSON body\"}");
    return;
  }

  String body = server.arg("plain");

  // Check for a few expected keys before accepting the payload.
  if (body.indexOf("\"battery1_v\"") < 0 ||
      body.indexOf("\"battery2_v\"") < 0) {
    server.send(400, "application/json",
                "{\"ok\":false,\"error\":\"Invalid telemetry payload\"}");
    return;
  }

  battery1V = jsonFloat(body, "battery1_v", battery1V);
  battery2V = jsonFloat(body, "battery2_v", battery2V);

  battery1SOC = jsonFloat(body, "battery1_soc_est", battery1SOC);
  battery2SOC = jsonFloat(body, "battery2_soc_est", battery2SOC);

  branch1CurrentA =
      jsonFloat(body, "port1_branch_current_a_measured", branch1CurrentA);
  branch2CurrentA =
      jsonFloat(body, "port2_branch_current_a_measured", branch2CurrentA);

  branch1PowerW =
      jsonFloat(body, "port1_branch_input_power_w_calculated", branch1PowerW);
  branch2PowerW =
      jsonFloat(body, "port2_branch_input_power_w_calculated", branch2PowerW);

  relay1Commanded =
      jsonBool(body, "port1_relay_commanded", relay1Commanded);
  relay2Commanded =
      jsonBool(body, "port2_relay_commanded", relay2Commanded);

  String receivedDevice = jsonValue(body, "device");
  if (receivedDevice.length() > 0) stationDevice = receivedDevice;

  String receivedCommand = jsonValue(body, "last_command");
  if (receivedCommand.length() > 0) lastCommand = receivedCommand;

  stationTelemetryReceived = true;
  lastTelemetryMs = millis();

  // Re-evaluate status whenever new synthetic branch currents/voltages arrive.
  sampleGridSensors();

  server.send(200, "application/json", "{\"ok\":true}");
}

// =====================================================
// 12. /DATA ENDPOINT FOR DASHBOARD
// =====================================================
void handleData() {
  String json = "{";

  json += "\"simulation_mode\":true,";
  json += "\"grid_voltage_v\":" + String(gridVoltageV, 3) + ",";
  json += "\"grid_current_a\":" + String(gridCurrentA, 3) + ",";
  json += "\"grid_power_w\":" + String(gridPowerW, 3) + ",";
  json += "\"grid_loading_percent\":" + String(gridLoadingPercent, 2) + ",";
  json += "\"grid_status\":\"" + jsonEscape(gridStatus) + "\",";
  json += "\"demand_response\":\"" + jsonEscape(demandResponse) + "\",";
  json += "\"allowed_branch_power_w\":" + String(allowedBranchPowerW, 2) + ",";
  json += "\"branch1_power_w\":" + String(branch1PowerW, 2) + ",";
  json += "\"branch2_power_w\":" + String(branch2PowerW, 2) + ",";
  json += "\"total_branch_power_w\":" + String(totalBranchPowerW, 2) + ",";

  json += "\"station_online\":" + String(stationOnline() ? "true" : "false") + ",";
  json += "\"station_branch_power_w\":" +
          String(branch1PowerW + branch2PowerW, 2) + ",";

  json += "\"battery1_v\":" + String(battery1V, 3) + ",";
  json += "\"battery2_v\":" + String(battery2V, 3) + ",";
  json += "\"battery1_soc_est\":" + String(battery1SOC, 1) + ",";
  json += "\"battery2_soc_est\":" + String(battery2SOC, 1) + ",";

  json += "\"port1_branch_current_a_measured\":" +
          String(branch1CurrentA, 3) + ",";
  json += "\"port2_branch_current_a_measured\":" +
          String(branch2CurrentA, 3) + ",";

  json += "\"port1_branch_input_power_w_calculated\":" +
          String(branch1PowerW, 3) + ",";
  json += "\"port2_branch_input_power_w_calculated\":" +
          String(branch2PowerW, 3) + ",";

  json += "\"port1_relay_commanded\":" +
          String(relay1Commanded ? "true" : "false") + ",";
  json += "\"port2_relay_commanded\":" +
          String(relay2Commanded ? "true" : "false") + ",";

  json += "\"station_device\":\"" + jsonEscape(stationDevice) + "\",";
  json += "\"last_command\":\"" + jsonEscape(lastCommand) + "\"";

  json += "}";

  server.send(200, "application/json", json);
}

// =====================================================
// 13. SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("SIMULATION ONLY: ADC calibration disabled");

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  // DHCP: avoid a hard-coded IP/gateway that may not match this Wi-Fi network.
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to Wi-Fi");

  unsigned long startAttempt = millis();

  while (WiFi.status() != WL_CONNECTED &&
         millis() - startAttempt < 20000) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Wi-Fi connected.");
    Serial.print("Substation IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Wi-Fi connection failed.");
    Serial.println("Check SSID/password and ensure this network permits device-to-device access.");
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);
  server.on("/budget", HTTP_GET, handleBudget);
  server.on("/simulate", HTTP_GET, handleSimulateGrid);
  server.on("/telemetry", HTTP_POST, handleTelemetry);

  server.onNotFound([]() {
    server.send(404, "text/plain", "Endpoint not found");
  });

  server.begin();
  Serial.println("HTTP server started.");

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  sampleGridSensors();

  Serial.println("Available endpoints:");
  Serial.println("  /");
  Serial.println("  /data");
  Serial.println("  /budget");
  Serial.println("  POST /telemetry");
}

// =====================================================
// 14. MAIN LOOP
// =====================================================
void loop() {
  server.handleClient();
  connectMQTT();
  mqtt.loop();
  if (millis() - lastMqttPublish >= MQTT_PUBLISH_INTERVAL_MS) {
    lastMqttPublish = millis(); publishGridStatus();
  }

  if (millis() - lastGridSampleMs >= GRID_SAMPLE_INTERVAL_MS) {
    lastGridSampleMs = millis();
    sampleGridSensors();
  }

  // Reconnect to the same configured Wi-Fi network if needed.
  static unsigned long lastReconnectAttempt = 0;

  if (WiFi.status() != WL_CONNECTED &&
      millis() - lastReconnectAttempt >= 5000) {
    lastReconnectAttempt = millis();
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.println("Attempting Wi-Fi reconnection...");
  }
}