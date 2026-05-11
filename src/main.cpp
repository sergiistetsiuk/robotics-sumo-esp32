#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_VL53L0X.h>
#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>

// Replace these with your Wi-Fi network credentials.
const char *WIFI_SSID = "stemlab118";
const char *WIFI_PASSWORD = "123118118";

WebServer server(80);

const uint8_t BTN_RUN_PIN = 4;
const unsigned long BUTTON_DEBOUNCE_MS = 50;
const unsigned long SENSOR_LOG_INTERVAL_MS = 1000;
const unsigned long START_SEQUENCE_INTERVAL_MS = 1000;
const unsigned long START_SEQUENCE_DURATION_MS = 5000;
const uint8_t WS2812B_PIN = 33;
const uint8_t WS2812B_LED_COUNT = 3;
const uint8_t I2C_SDA = 21;
const uint8_t I2C_SCL = 22;
const uint8_t XSHUT_RIGHT = 25;
const uint8_t XSHUT_FRONT = 26;
const uint8_t XSHUT_LEFT = 27;
const uint8_t ADDR_RIGHT = 0x30;
const uint8_t ADDR_FRONT = 0x31;
const uint8_t ADDR_LEFT = 0x32;
const uint8_t MOTOR_LEFT_PWM_PIN = 17;
const uint8_t MOTOR_LEFT_DIR_PIN = 16;
const uint8_t MOTOR_RIGHT_PWM_PIN = 19;
const uint8_t MOTOR_RIGHT_DIR_PIN = 18;
const uint8_t MOTOR_LEFT_FORWARD_STATE = LOW;
const uint8_t MOTOR_RIGHT_FORWARD_STATE = HIGH;
const uint32_t pwmFreq = 5000;
const uint8_t pwmResolution = 8;
const uint8_t MOTOR_MAX_DUTY = 255;
const uint8_t MOTOR_STOP_DUTY = 255;

constexpr int16_t speedPercent(uint8_t percent)
{
  return (static_cast<int16_t>(percent > 100 ? 100 : percent) * MOTOR_MAX_DUTY) / 100;
}

const int16_t SPEED_ATTACK = speedPercent(10);
const int16_t SPEED_SEARCH_TURN = speedPercent(7);
const uint16_t OPPONENT_DETECT_DISTANCE_MM = 400;
const uint16_t DISTANCE_CLOSE_THRESHOLD_MM = 80;
const uint16_t DISTANCE_INVALID_MM = 8191;
const uint16_t DISTANCE_SENSOR_PERIOD_MS = 20;
const uint32_t DISTANCE_SENSOR_TIMING_BUDGET_US = 20000;
const uint8_t MOTOR_LEFT_PWM_CHANNEL = 0;
const uint8_t MOTOR_RIGHT_PWM_CHANNEL = 1;
const uint8_t LINE_SENSOR_PIN = 23;
const uint8_t LINE_SENSOR_ACTIVE_STATE = HIGH;

Adafruit_NeoPixel ws2812b(WS2812B_LED_COUNT, WS2812B_PIN, NEO_BRG + NEO_KHZ800);
Adafruit_VL53L0X sensorRight;
Adafruit_VL53L0X sensorFront;
Adafruit_VL53L0X sensorLeft;

const size_t MAX_LOG_LINES = 80;
String logLines[MAX_LOG_LINES];
size_t logLineCount = 0;
size_t logWriteIndex = 0;

bool otaSuccess = false;
String otaMessage;

bool lastRunButtonReading = HIGH;
bool stableRunButtonState = HIGH;
unsigned long lastRunButtonChangeMs = 0;
bool projectRunning = false;
bool projectStarting = false;
unsigned long startSequenceStartedMs = 0;
int8_t lastStartSequenceStep = -1;
unsigned long lastSensorLogMs = 0;
bool sensorRightReady = false;
bool sensorFrontReady = false;
bool sensorLeftReady = false;
uint16_t rightDistanceMm = DISTANCE_INVALID_MM;
uint16_t frontDistanceMm = DISTANCE_INVALID_MM;
uint16_t leftDistanceMm = DISTANCE_INVALID_MM;

void setupWiFi();
void setupWebOTA();
void addLog(const String &message);
String getUptime();
void handleRoot();
void handleLogs();
void handleStatus();
void handleClearLogs();
void handleReset();
void handleFirmwareUpload();
void setWs2812bColor(uint8_t red, uint8_t green, uint8_t blue);
void setupDistanceSensors();
bool setupDistanceSensor(Adafruit_VL53L0X &sensor, const String &name, uint8_t xshutPin, uint8_t address);
void updateDistanceSensors();
void updateDistanceSensor(Adafruit_VL53L0X &sensor, bool ready, uint16_t &distanceMm);
void logDistanceSensors();
String formatDistanceSensor(uint16_t distanceMm, bool ready);
void setupMotors();
void attachMotorPwm(uint8_t pin, uint8_t channel);
void writeMotorPwm(uint8_t pin, uint8_t channel, uint8_t duty);
uint8_t speedPercentToDuty(uint8_t percent);
void setMotorSpeed(uint8_t pwmPin, uint8_t dirPin, uint8_t channel, uint8_t forwardState, int16_t speed);
void setMotorSpeeds(int16_t leftSpeed, int16_t rightSpeed);
void setMotorsSpeed(int16_t duty);
void setMotorsSpeedPercent(uint8_t percent);
void runCombatAlgorithm();
void setupLineSensor();
String readLineSensorStatus();
void setProjectRunning(bool running);
void beginStartSequence();
void updateStartSequence();
void setupProject();
void loopProject();

String htmlEscape(const String &text)
{
  String escaped;
  escaped.reserve(text.length());

  for (size_t i = 0; i < text.length(); i++)
  {
    const char c = text.charAt(i);
    switch (c)
    {
    case '&':
      escaped += F("&amp;");
      break;
    case '<':
      escaped += F("&lt;");
      break;
    case '>':
      escaped += F("&gt;");
      break;
    case '"':
      escaped += F("&quot;");
      break;
    case '\'':
      escaped += F("&#39;");
      break;
    default:
      escaped += c;
      break;
    }
  }

  return escaped;
}

String jsonEscape(const String &text)
{
  String escaped;
  escaped.reserve(text.length());

  for (size_t i = 0; i < text.length(); i++)
  {
    const char c = text.charAt(i);
    switch (c)
    {
    case '\\':
      escaped += F("\\\\");
      break;
    case '"':
      escaped += F("\\\"");
      break;
    case '\n':
      escaped += F("\\n");
      break;
    case '\r':
      escaped += F("\\r");
      break;
    case '\t':
      escaped += F("\\t");
      break;
    default:
      if (static_cast<uint8_t>(c) < 0x20)
      {
        escaped += ' ';
      }
      else
      {
        escaped += c;
      }
      break;
    }
  }

  return escaped;
}

String getLogText()
{
  String output;
  const size_t start = (logLineCount == MAX_LOG_LINES) ? logWriteIndex : 0;

  for (size_t i = 0; i < logLineCount; i++)
  {
    const size_t index = (start + i) % MAX_LOG_LINES;
    output += logLines[index];
    output += '\n';
  }

  return output;
}

void setupWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print(F("Connecting to Wi-Fi"));

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print('.');
  }

  Serial.println();
  addLog("Wi-Fi connected");
  addLog("IP address: " + WiFi.localIP().toString());
}

void setupWebOTA()
{
  server.on("/", HTTP_GET, handleRoot);
  server.on("/logs", HTTP_GET, handleLogs);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/clear-logs", HTTP_POST, handleClearLogs);
  server.on("/reset", HTTP_POST, handleReset);
  server.on(
      "/update", HTTP_POST,
      []()
      {
        const int statusCode = otaSuccess ? 200 : 500;
        const String statusClass = otaSuccess ? "success" : "error";
        const String title = otaSuccess ? "Firmware uploaded successfully" : "Firmware upload failed";
        const String details = otaSuccess ? "The ESP32 will reboot now. This page will return to the main interface automatically."
                                          : otaMessage;

        String page = F("<!doctype html><html><head><meta charset='utf-8'>"
                        "<meta name='viewport' content='width=device-width,initial-scale=1'>");

        page += F("<title>ESP32 OTA</title><style>"
                  "body{margin:0;min-height:100vh;display:grid;place-items:center;background:#f3f6fb;color:#172033;font-family:Arial,sans-serif}"
                  ".box{width:min(92vw,560px);background:#fff;border:1px solid #d9e1ef;border-radius:8px;padding:28px;box-shadow:0 18px 45px rgba(20,34,58,.12)}"
                  "h1{margin:0 0 12px;font-size:24px}.success{color:#147a45}.error{color:#b42318}"
                  "p{line-height:1.5}.btn{display:inline-block;margin-top:14px;padding:11px 16px;border-radius:6px;background:#1f6feb;color:#fff;text-decoration:none}"
                  "</style></head><body><main class='box'>");
        page += "<h1 class='" + statusClass + "'>" + htmlEscape(title) + "</h1>";
        page += "<p>" + htmlEscape(details) + "</p>";
        page += F("<a class='btn' href='/'>Back to dashboard</a></main>");

        if (otaSuccess)
        {
          page += F("<script>"
                    "let attempts=0;"
                    "function goHomeWhenReady(){"
                    "attempts++;"
                    "fetch('/',{cache:'no-store'})"
                    ".then(function(response){if(response.ok){location.href='/';return;}throw new Error('not ready');})"
                    ".catch(function(){setTimeout(goHomeWhenReady,1500);});"
                    "}"
                    "setTimeout(goHomeWhenReady,5000);"
                    "</script>");
        }

        page += F("</body></html>");
        server.send(statusCode, "text/html", page);

        if (otaSuccess)
        {
          addLog("OTA upload completed successfully; rebooting");
          delay(1000);
          ESP.restart();
        }
      },
      handleFirmwareUpload);

  server.begin();
  addLog("Web OTA server started on http://" + WiFi.localIP().toString() + "/");
  addLog("Sketch size: " + String(ESP.getSketchSize()) + " bytes");
  addLog("Free OTA space: " + String(ESP.getFreeSketchSpace()) + " bytes");
}

void addLog(const String &message)
{
  const String line = "[" + getUptime() + "] " + message;

  Serial.println(line);
  logLines[logWriteIndex] = line;
  logWriteIndex = (logWriteIndex + 1) % MAX_LOG_LINES;

  if (logLineCount < MAX_LOG_LINES)
  {
    logLineCount++;
  }
}

void clearLogs()
{
  for (size_t i = 0; i < MAX_LOG_LINES; i++)
  {
    logLines[i] = "";
  }

  logLineCount = 0;
  logWriteIndex = 0;
}

String getUptime()
{
  const uint64_t totalSeconds = millis() / 1000;
  const uint32_t days = totalSeconds / 86400;
  const uint8_t hours = (totalSeconds % 86400) / 3600;
  const uint8_t minutes = (totalSeconds % 3600) / 60;
  const uint8_t seconds = totalSeconds % 60;

  char buffer[32];

  if (days > 0)
  {
    snprintf(buffer, sizeof(buffer), "%lud %02u:%02u:%02u", static_cast<unsigned long>(days), hours, minutes, seconds);
  }
  else
  {
    snprintf(buffer, sizeof(buffer), "%02u:%02u:%02u", hours, minutes, seconds);
  }

  return String(buffer);
}

void handleRoot()
{
  const String ipAddress = WiFi.localIP().toString();

  String page = F(R"rawliteral(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP32 Web OTA</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #f4f7fb;
      --panel: #ffffff;
      --text: #172033;
      --muted: #5f6f86;
      --line: #dbe3ef;
      --primary: #1f6feb;
      --primary-dark: #174ea6;
      --danger: #b42318;
      --success: #147a45;
      --log-bg: #101828;
      --log-text: #d6e4ff;
    }

    * {
      box-sizing: border-box;
    }

    body {
      margin: 0;
      min-height: 100vh;
      background: var(--bg);
      color: var(--text);
      font-family: Arial, Helvetica, sans-serif;
    }

    .shell {
      width: min(1120px, calc(100% - 28px));
      margin: 0 auto;
      padding: 28px 0;
    }

    header {
      display: flex;
      align-items: flex-start;
      justify-content: space-between;
      gap: 18px;
      margin-bottom: 18px;
    }

    h1 {
      margin: 0 0 6px;
      font-size: clamp(28px, 5vw, 42px);
      line-height: 1.05;
      letter-spacing: 0;
    }

    .subtitle {
      margin: 0;
      color: var(--muted);
      font-size: 15px;
    }

    .status-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(130px, 1fr));
      gap: 10px;
      min-width: min(100%, 330px);
    }

    .status-card {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 13px 14px;
    }

    .status-card span {
      display: block;
      color: var(--muted);
      font-size: 12px;
      text-transform: uppercase;
      letter-spacing: .06em;
      margin-bottom: 7px;
    }

    .status-card strong {
      display: block;
      overflow-wrap: anywhere;
      font-size: 17px;
    }

    .panel {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      overflow: hidden;
      box-shadow: 0 18px 45px rgba(20, 34, 58, .10);
    }

    .tabs {
      display: flex;
      gap: 0;
      border-bottom: 1px solid var(--line);
      background: #eef3fa;
    }

    .tab-button {
      appearance: none;
      border: 0;
      border-right: 1px solid var(--line);
      background: transparent;
      color: var(--muted);
      cursor: pointer;
      font: inherit;
      font-weight: 700;
      padding: 15px 18px;
      min-width: 150px;
    }

    .tab-button.active {
      background: var(--panel);
      color: var(--primary-dark);
    }

    .tab-content {
      display: none;
      padding: 18px;
    }

    .tab-content.active {
      display: block;
    }

    .log-toolbar {
      display: flex;
      justify-content: space-between;
      gap: 12px;
      align-items: center;
      margin-bottom: 12px;
    }

    .log-toolbar p {
      color: var(--muted);
      margin: 0;
      font-size: 14px;
    }

    .actions {
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
      justify-content: flex-end;
    }

    .action-button {
      appearance: none;
      border: 1px solid var(--line);
      border-radius: 6px;
      background: #fff;
      color: var(--text);
      cursor: pointer;
      font: inherit;
      font-weight: 700;
      padding: 10px 13px;
    }

    .action-button:hover {
      border-color: #9fb0c8;
      background: #f8fbff;
    }

    .action-button:disabled {
      cursor: wait;
      opacity: .65;
    }

    .action-button.danger {
      border-color: #f0b8b2;
      color: var(--danger);
    }

    .action-button.danger:hover {
      background: #fff5f4;
    }

    pre {
      width: 100%;
      min-height: 420px;
      max-height: 62vh;
      overflow: auto;
      margin: 0;
      padding: 16px;
      border-radius: 8px;
      background: var(--log-bg);
      color: var(--log-text);
      font: 14px/1.45 "SFMono-Regular", Consolas, "Liberation Mono", monospace;
      white-space: pre-wrap;
      word-break: break-word;
    }

    form {
      display: grid;
      gap: 14px;
      max-width: 680px;
    }

    label {
      display: grid;
      gap: 8px;
      color: var(--muted);
      font-weight: 700;
    }

    input[type="file"] {
      width: 100%;
      border: 1px dashed #aab8cc;
      border-radius: 8px;
      padding: 18px;
      background: #f8fbff;
      color: var(--text);
    }

    button[type="submit"] {
      justify-self: start;
      min-width: 170px;
      border: 0;
      border-radius: 6px;
      background: var(--primary);
      color: #fff;
      cursor: pointer;
      font: inherit;
      font-weight: 700;
      padding: 12px 18px;
    }

    button[type="submit"]:hover {
      background: var(--primary-dark);
    }

    .notice {
      margin: 0;
      color: var(--muted);
      line-height: 1.5;
    }

    .message {
      min-height: 24px;
      font-weight: 700;
    }

    .message.error {
      color: var(--danger);
    }

    .message.success {
      color: var(--success);
    }

    @media (max-width: 720px) {
      header {
        display: grid;
      }

      .status-grid {
        grid-template-columns: 1fr;
      }

      .tabs {
        display: grid;
        grid-template-columns: 1fr 1fr;
      }

      .tab-button {
        min-width: 0;
        padding-inline: 10px;
      }

      .log-toolbar {
        display: grid;
      }

      pre {
        min-height: 360px;
      }
    }
  </style>
</head>
<body>
  <main class="shell">
    <header>
      <div>
        <h1>ESP32 Web OTA</h1>
        <p class="subtitle">Wireless firmware updates and live board diagnostics.</p>
      </div>
      <section class="status-grid" aria-label="Device status">
        <div class="status-card">
          <span>IP Address</span>
          <strong id="ipAddress">)rawliteral");

  page += htmlEscape(ipAddress);

  page += F(R"rawliteral(</strong>
        </div>
        <div class="status-card">
          <span>Uptime</span>
          <strong id="uptime">)rawliteral");

  page += htmlEscape(getUptime());

  page += F(R"rawliteral(</strong>
        </div>
      </section>
    </header>

    <section class="panel">
      <nav class="tabs" aria-label="Main tabs">
        <button class="tab-button active" type="button" data-tab="logs">Board Logs</button>
        <button class="tab-button" type="button" data-tab="upload">Firmware Upload</button>
      </nav>

      <section id="tab-logs" class="tab-content active">
        <div class="log-toolbar">
          <p>Live board logs update automatically.</p>
          <div class="actions">
            <button id="clearLogsButton" class="action-button" type="button">Clear Logs</button>
            <button id="resetButton" class="action-button danger" type="button">Reset ESP32</button>
          </div>
        </div>
        <pre id="logs">Loading logs...</pre>
      </section>

      <section id="tab-upload" class="tab-content">
        <form id="uploadForm" method="POST" action="/update" enctype="multipart/form-data">
          <label>
            Firmware .bin file
            <input id="firmwareFile" type="file" name="firmware" accept=".bin,application/octet-stream" required>
          </label>
          <p class="notice">Upload a PlatformIO firmware binary. The board will reboot automatically after a successful update.</p>
          <button type="submit">Upload Firmware</button>
          <div id="uploadMessage" class="message" role="status"></div>
        </form>
      </section>
    </section>
  </main>

  <script>
    const buttons = document.querySelectorAll('.tab-button');
    const logsEl = document.getElementById('logs');
    const uptimeEl = document.getElementById('uptime');
    const ipAddressEl = document.getElementById('ipAddress');
    const uploadForm = document.getElementById('uploadForm');
    const firmwareFile = document.getElementById('firmwareFile');
    const uploadMessage = document.getElementById('uploadMessage');
    const clearLogsButton = document.getElementById('clearLogsButton');
    const resetButton = document.getElementById('resetButton');

    buttons.forEach((button) => {
      button.addEventListener('click', () => {
        const tab = button.dataset.tab;

        buttons.forEach((item) => item.classList.toggle('active', item === button));
        document.querySelectorAll('.tab-content').forEach((content) => {
          content.classList.toggle('active', content.id === `tab-${tab}`);
        });
      });
    });

    async function refreshLogs() {
      try {
        const response = await fetch('/logs', { cache: 'no-store' });
        const text = await response.text();
        const shouldStickToBottom = logsEl.scrollTop + logsEl.clientHeight >= logsEl.scrollHeight - 24;
        logsEl.textContent = text || 'No logs yet.';

        if (shouldStickToBottom) {
          logsEl.scrollTop = logsEl.scrollHeight;
        }
      } catch (error) {
        logsEl.textContent = 'Unable to load logs.';
      }
    }

    async function refreshStatus() {
      try {
        const response = await fetch('/status', { cache: 'no-store' });
        const status = await response.json();
        uptimeEl.textContent = status.uptime;
        ipAddressEl.textContent = status.ip;
      } catch (error) {
        uptimeEl.textContent = 'Unavailable';
      }
    }

    uploadForm.addEventListener('submit', (event) => {
      const file = firmwareFile.files[0];
      uploadMessage.className = 'message';
      uploadMessage.textContent = '';

      if (!file || !file.name.toLowerCase().endsWith('.bin')) {
        event.preventDefault();
        uploadMessage.classList.add('error');
        uploadMessage.textContent = 'Please choose a .bin firmware file.';
        return;
      }

      uploadMessage.classList.add('success');
      uploadMessage.textContent = 'Uploading firmware. Keep this page open.';
    });

    clearLogsButton.addEventListener('click', async () => {
      clearLogsButton.disabled = true;

      try {
        await fetch('/clear-logs', { method: 'POST', cache: 'no-store' });
        await refreshLogs();
      } finally {
        clearLogsButton.disabled = false;
      }
    });

    resetButton.addEventListener('click', async () => {
      if (!confirm('Reset the ESP32 now?')) {
        return;
      }

      resetButton.disabled = true;

      try {
        await fetch('/reset', { method: 'POST', cache: 'no-store' });
        logsEl.textContent = 'Reset requested. Reconnecting...';
        setTimeout(() => location.reload(), 8000);
      } catch (error) {
        setTimeout(() => location.reload(), 8000);
      }
    });

    refreshLogs();
    refreshStatus();
    setInterval(refreshLogs, 1000);
    setInterval(refreshStatus, 1000);
  </script>
</body>
</html>
)rawliteral");

  server.send(200, "text/html", page);
}

void handleLogs()
{
  server.send(200, "text/plain", getLogText());
}

void handleClearLogs()
{
  clearLogs();
  addLog("Logs cleared from web interface");
  server.send(200, "text/plain", "Logs cleared");
}

void handleReset()
{
  addLog("Reset requested from web interface");
  server.send(200, "text/plain", "Reset requested");
  delay(500);
  ESP.restart();
}

void handleStatus()
{
  String response = "{";
  response += "\"ip\":\"" + jsonEscape(WiFi.localIP().toString()) + "\",";
  response += "\"uptime\":\"" + jsonEscape(getUptime()) + "\",";
  response += "\"rssi\":" + String(WiFi.RSSI()) + ",";
  response += "\"freeHeap\":" + String(ESP.getFreeHeap());
  response += "}";

  server.send(200, "application/json", response);
}

void handleFirmwareUpload()
{
  HTTPUpload &upload = server.upload();

  if (upload.status == UPLOAD_FILE_START)
  {
    otaSuccess = false;
    otaMessage = "";

    const String filename = upload.filename;

    if (!filename.endsWith(".bin"))
    {
      otaMessage = "Only .bin firmware files are accepted.";
      addLog("OTA rejected: " + otaMessage);
      return;
    }

    addLog("OTA upload started: " + filename);
    addLog("Free OTA space before upload: " + String(ESP.getFreeSketchSpace()) + " bytes");

    const size_t freeSketchSpace = ESP.getFreeSketchSpace();
    const size_t updateSpace = freeSketchSpace > 0x1000 ? (freeSketchSpace - 0x1000) & 0xFFFFF000 : 0;

    if (updateSpace == 0)
    {
      otaMessage = "No OTA update partition is available. Flash once by USB with an OTA partition table.";
      addLog("OTA begin failed: " + otaMessage);
    }
    else if (!Update.begin(updateSpace))
    {
      otaMessage = Update.errorString();
      addLog("OTA begin failed: " + otaMessage);
    }
  }
  else if (upload.status == UPLOAD_FILE_WRITE)
  {
    if (otaMessage.length() == 0)
    {
      const size_t written = Update.write(upload.buf, upload.currentSize);

      if (written != upload.currentSize)
      {
        otaMessage = Update.errorString();
        addLog("OTA write failed: " + otaMessage);
      }
    }
  }
  else if (upload.status == UPLOAD_FILE_END)
  {
    if (otaMessage.length() == 0 && Update.end(true))
    {
      otaSuccess = true;
      otaMessage = "Upload complete.";
      addLog("OTA upload size: " + String(upload.totalSize) + " bytes");
    }
    else
    {
      otaSuccess = false;

      if (otaMessage.length() == 0)
      {
        otaMessage = Update.errorString();
      }

      addLog("OTA end failed: " + otaMessage);
    }
  }
  else if (upload.status == UPLOAD_FILE_ABORTED)
  {
    Update.abort();
    otaSuccess = false;
    otaMessage = "Upload was aborted.";
    addLog("OTA aborted");
  }
}

void setWs2812bColor(uint8_t red, uint8_t green, uint8_t blue)
{
  const uint32_t color = ws2812b.Color(red, green, blue);

  for (uint8_t i = 0; i < WS2812B_LED_COUNT; i++)
  {
    ws2812b.setPixelColor(i, color);
  }

  ws2812b.show();
}

void setupDistanceSensors()
{
  Wire.begin(I2C_SDA, I2C_SCL);

  pinMode(XSHUT_RIGHT, OUTPUT);
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_LEFT, OUTPUT);

  digitalWrite(XSHUT_RIGHT, LOW);
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_LEFT, LOW);
  delay(10);

  addLog("I2C initialized on SDA " + String(I2C_SDA) + ", SCL " + String(I2C_SCL));

  sensorRightReady = setupDistanceSensor(sensorRight, "right", XSHUT_RIGHT, ADDR_RIGHT);
  sensorFrontReady = setupDistanceSensor(sensorFront, "front", XSHUT_FRONT, ADDR_FRONT);
  sensorLeftReady = setupDistanceSensor(sensorLeft, "left", XSHUT_LEFT, ADDR_LEFT);
}

bool setupDistanceSensor(Adafruit_VL53L0X &sensor, const String &name, uint8_t xshutPin, uint8_t address)
{
  digitalWrite(xshutPin, HIGH);
  delay(10);

  if (!sensor.begin(address, false, &Wire))
  {
    addLog("VL53L0X " + name + " init failed on XSHUT GPIO " + String(xshutPin));
    return false;
  }

  sensor.setMeasurementTimingBudgetMicroSeconds(DISTANCE_SENSOR_TIMING_BUDGET_US);
  sensor.startRangeContinuous(DISTANCE_SENSOR_PERIOD_MS);

  addLog("VL53L0X " + name + " initialized at address 0x" + String(address, HEX));
  return true;
}

void updateDistanceSensor(Adafruit_VL53L0X &sensor, bool ready, uint16_t &distanceMm)
{
  if (!ready)
  {
    distanceMm = DISTANCE_INVALID_MM;
    return;
  }

  if (!sensor.isRangeComplete())
  {
    return;
  }

  const uint16_t measuredDistanceMm = sensor.readRangeResult();
  const uint8_t rangeStatus = sensor.readRangeStatus();

  if (rangeStatus == 4)
  {
    distanceMm = DISTANCE_INVALID_MM;
    return;
  }

  distanceMm = measuredDistanceMm;
}

void updateDistanceSensors()
{
  updateDistanceSensor(sensorFront, sensorFrontReady, frontDistanceMm);
  updateDistanceSensor(sensorLeft, sensorLeftReady, leftDistanceMm);
  updateDistanceSensor(sensorRight, sensorRightReady, rightDistanceMm);
}

String formatDistanceSensor(uint16_t distanceMm, bool ready)
{
  if (!ready)
  {
    return "not ready";
  }

  if (distanceMm == DISTANCE_INVALID_MM)
  {
    return "out of range";
  }

  return String(distanceMm) + " mm";
}

void logDistanceSensors()
{
  updateDistanceSensors();

  const String rightValue = formatDistanceSensor(rightDistanceMm, sensorRightReady);
  const String frontValue = formatDistanceSensor(frontDistanceMm, sensorFrontReady);
  const String leftValue = formatDistanceSensor(leftDistanceMm, sensorLeftReady);

  addLog("VL53L0X distances - left: " + leftValue + ", front: " + frontValue + ", right: " + rightValue);
  addLog("Line sensor status: " + readLineSensorStatus());
}

void setupMotors()
{
  pinMode(MOTOR_LEFT_PWM_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_DIR_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR_PIN, OUTPUT);

  digitalWrite(MOTOR_LEFT_PWM_PIN, HIGH);
  digitalWrite(MOTOR_RIGHT_PWM_PIN, HIGH);
  digitalWrite(MOTOR_LEFT_DIR_PIN, LOW);
  digitalWrite(MOTOR_RIGHT_DIR_PIN, LOW);

  attachMotorPwm(MOTOR_LEFT_PWM_PIN, MOTOR_LEFT_PWM_CHANNEL);
  attachMotorPwm(MOTOR_RIGHT_PWM_PIN, MOTOR_RIGHT_PWM_CHANNEL);
  setMotorsSpeed(0);

  addLog("Motors initialized: left PWM GPIO " + String(MOTOR_LEFT_PWM_PIN) + ", left DIR GPIO " + String(MOTOR_LEFT_DIR_PIN));
  addLog("Motors initialized: right PWM GPIO " + String(MOTOR_RIGHT_PWM_PIN) + ", right DIR GPIO " + String(MOTOR_RIGHT_DIR_PIN));
}

void attachMotorPwm(uint8_t pin, uint8_t channel)
{
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, pwmFreq, pwmResolution);
#else
  ledcSetup(channel, pwmFreq, pwmResolution);
  ledcAttachPin(pin, channel);
#endif
}

void writeMotorPwm(uint8_t pin, uint8_t channel, uint8_t duty)
{
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(channel, duty);
#endif
}

uint8_t speedPercentToDuty(uint8_t percent)
{
  return speedPercent(percent);
}

void setMotorsSpeed(int16_t duty)
{
  setMotorSpeeds(duty, duty);
}

void setMotorsSpeedPercent(uint8_t percent)
{
  setMotorsSpeed(speedPercentToDuty(percent));
}

void setMotorSpeed(uint8_t pwmPin, uint8_t dirPin, uint8_t channel, uint8_t forwardState, int16_t speed)
{
  const int16_t limitedSpeed = speed < -MOTOR_MAX_DUTY ? -MOTOR_MAX_DUTY : speed > MOTOR_MAX_DUTY ? MOTOR_MAX_DUTY
                                                                                                  : speed;
  const uint8_t duty = abs(limitedSpeed);
  const uint8_t invertedDuty = MOTOR_STOP_DUTY - duty;
  const uint8_t reverseState = forwardState == HIGH ? LOW : HIGH;

  digitalWrite(dirPin, limitedSpeed >= 0 ? forwardState : reverseState);
  writeMotorPwm(pwmPin, channel, invertedDuty);
}

void setMotorSpeeds(int16_t leftSpeed, int16_t rightSpeed)
{
  setMotorSpeed(MOTOR_LEFT_PWM_PIN, MOTOR_LEFT_DIR_PIN, MOTOR_LEFT_PWM_CHANNEL, MOTOR_LEFT_FORWARD_STATE, leftSpeed);
  setMotorSpeed(MOTOR_RIGHT_PWM_PIN, MOTOR_RIGHT_DIR_PIN, MOTOR_RIGHT_PWM_CHANNEL, MOTOR_RIGHT_FORWARD_STATE, rightSpeed);
}

void runCombatAlgorithm()
{
  updateDistanceSensors();

  const bool frontDetected = frontDistanceMm <= OPPONENT_DETECT_DISTANCE_MM;
  const bool leftDetected = leftDistanceMm <= OPPONENT_DETECT_DISTANCE_MM;
  const bool rightDetected = rightDistanceMm <= OPPONENT_DETECT_DISTANCE_MM;

  if (frontDetected)
  {
    setMotorSpeeds(SPEED_ATTACK, SPEED_ATTACK);
  }
  else
  {
    setMotorSpeeds(0, SPEED_SEARCH_TURN);
  }

  /*
    if (!frontDetected && !leftDetected && !rightDetected)
    {
      setMotorSpeeds(-SPEED_SEARCH_TURN, SPEED_SEARCH_TURN);
      return;
    }

    const uint16_t smallestDistance = min(frontDistanceMm, min(leftDistanceMm, rightDistanceMm));

    if (frontDistanceMm <= leftDistanceMm + DISTANCE_CLOSE_THRESHOLD_MM &&
        frontDistanceMm <= rightDistanceMm + DISTANCE_CLOSE_THRESHOLD_MM)
    {
      setMotorSpeeds(SPEED_ATTACK, SPEED_ATTACK);
    }
    else if (leftDistanceMm < rightDistanceMm)
    {
      setMotorSpeeds(0, SPEED_SEARCH_TURN);
    }
    else
    {
      setMotorSpeeds(SPEED_SEARCH_TURN, 0);
    }
      */
}

void setupLineSensor()
{
  pinMode(LINE_SENSOR_PIN, INPUT);
  addLog("Line sensor initialized on GPIO " + String(LINE_SENSOR_PIN));
  addLog("Line sensor status: " + readLineSensorStatus());
}

String readLineSensorStatus()
{
  const uint8_t rawState = digitalRead(LINE_SENSOR_PIN);
  const String stateText = rawState == HIGH ? "HIGH" : "LOW";
  const String lineText = rawState == LINE_SENSOR_ACTIVE_STATE ? "line detected" : "no line";

  return lineText + " (raw " + stateText + ")";
}

void setProjectRunning(bool running)
{
  projectStarting = false;
  projectRunning = running;

  if (projectRunning)
  {
    setWs2812bColor(255, 0, 0);
    setMotorSpeeds(-SPEED_SEARCH_TURN, SPEED_SEARCH_TURN);
    lastSensorLogMs = 0;
    addLog("Project status: running");
    addLog("WS2812B color: red");
    addLog("Combat algorithm active");
  }
  else
  {
    setMotorsSpeed(0);
    digitalWrite(MOTOR_LEFT_DIR_PIN, LOW);
    digitalWrite(MOTOR_RIGHT_DIR_PIN, LOW);
    setWs2812bColor(0, 255, 0);
    addLog("Project status: stopped");
    addLog("WS2812B color: green");
    addLog("Motors stopped");
  }
}

void beginStartSequence()
{
  setProjectRunning(false);
  projectStarting = true;
  startSequenceStartedMs = millis();
  lastStartSequenceStep = -1;
  addLog("Start sequence: 5 second red/green countdown");
  updateStartSequence();
}

void updateStartSequence()
{
  if (!projectStarting)
  {
    return;
  }

  const unsigned long elapsedMs = millis() - startSequenceStartedMs;

  if (elapsedMs >= START_SEQUENCE_DURATION_MS)
  {
    addLog("Start sequence complete");
    setProjectRunning(true);
    return;
  }

  const int8_t currentStep = elapsedMs / START_SEQUENCE_INTERVAL_MS;

  if (currentStep == lastStartSequenceStep)
  {
    return;
  }

  lastStartSequenceStep = currentStep;

  if (currentStep % 2 == 0)
  {
    setWs2812bColor(255, 0, 0);
    addLog("Start sequence color: red");
  }
  else
  {
    setWs2812bColor(0, 255, 0);
    addLog("Start sequence color: green");
  }
}

void setupProject()
{
  pinMode(BTN_RUN_PIN, INPUT_PULLUP);
  lastRunButtonReading = digitalRead(BTN_RUN_PIN);
  stableRunButtonState = lastRunButtonReading;

  ws2812b.begin();
  ws2812b.setBrightness(60);

  setupLineSensor();
  setupDistanceSensors();
  setProjectRunning(false);

  // Add your own project initialization code here.
  addLog("BTN_RUN_PIN initialized on GPIO " + String(BTN_RUN_PIN));
  addLog("WS2812B initialized on GPIO " + String(WS2812B_PIN) + " with " + String(WS2812B_LED_COUNT) + " LEDs");
  addLog("Project setup complete");
}

void loopProject()
{
  const bool currentReading = digitalRead(BTN_RUN_PIN);

  if (currentReading != lastRunButtonReading)
  {
    lastRunButtonChangeMs = millis();
    lastRunButtonReading = currentReading;
  }

  if ((millis() - lastRunButtonChangeMs) > BUTTON_DEBOUNCE_MS && currentReading != stableRunButtonState)
  {
    stableRunButtonState = currentReading;

    if (stableRunButtonState == HIGH)
    {
      addLog("BTN_RUN_PIN released");

      if (projectRunning || projectStarting)
      {
        setProjectRunning(false);
      }
      else
      {
        beginStartSequence();
      }
    }
  }

  updateStartSequence();

  if (projectRunning)
  {
    runCombatAlgorithm();
  }

  if (projectRunning && millis() - lastSensorLogMs >= SENSOR_LOG_INTERVAL_MS)
  {
    lastSensorLogMs = millis();
    logDistanceSensors();
  }

  // Add your own repeated project logic here.
}

void setup()
{
  Serial.begin(115200);
  setupMotors();
  setupWiFi();
  setupWebOTA();
  setupProject();
}

void loop()
{
  server.handleClient();
  loopProject();
}
