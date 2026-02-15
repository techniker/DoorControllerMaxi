// GarageController (Controllino Maxi + MQTT + HA discovery)
// <tec ( att ) sixtopia.net>

#include <Arduino.h>
#include <Controllino.h>
#include <Bounce2.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>

//## USER CONFIG

// Network (static)
static byte MAC_ADDR[]    = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFF, 0xEF };
static IPAddress IP_ADDR(10, 22, 5, 10);
static IPAddress SUBNET(255, 255, 255, 0);
static IPAddress GATEWAY(10, 22, 5, 1);
static IPAddress DNS_ADDR(10, 22, 5, 1);

// MQTT broker
static IPAddress MQTT_HOST(10, 22, 5, 5);
static const uint16_t MQTT_PORT = 1883;
static const char* MQTT_USER = "mqtt";
static const char* MQTT_PASS = "";

// HA device identification
static const char* NODE_ID     = "garagecontroller01";
static const char* DEVICE_NAME = "Garage Door Controller";
static const char* SW_VERSION  = "1.1";

// Sensors enable/disable
static const bool DISABLE_EXTERNAL_SENSORS      = false; // If you don't have any external sensors, set to true

// Timings
static const uint32_t DOOR_MAX_MOVEMENT_MS      = 30000;
static const uint16_t RELAY_START_DELAY_MS      = 500;
static const uint16_t DEBOUNCE_MS               = 50;
static const uint16_t LOCAL_TOGGLE_GUARD_MS     = 1000;
static const uint16_t BLINK_MS                  = 300;

// MQTT Publishing
static const uint32_t STATE_REFRESH_MS          = 60000;
static const uint32_t MQTT_RECONNECT_MIN_MS     = 5000;

// After endstop triggers, keep running for a short time (mechanical slack)
static const uint16_t CLOSED_ENDSTOP_OVERRUN_MS = 1000; // adjust (e.g. 300..1500 ms)

static bool closedOverrunActive = false;
static uint32_t closedOverrunUntilMs = 0;

//## I/O mapping (Controllino Maxi)

// Local toggle input
static const int PIN_LOCAL_BUTTON     = CONTROLLINO_A9;

// Relays
static const int PIN_DOOR_POWER_RELAY = CONTROLLINO_RELAY_08;
static const int PIN_DOOR_DIR_RELAY   = CONTROLLINO_RELAY_09;

// Status outputs
static const int PIN_STATUS_LIGHT     = CONTROLLINO_D11; // blinking while moving, ON idle
static const int PIN_STATUS_LOCKED    = CONTROLLINO_D10; // ON when closed endstop ACTIVE

// Sensors
static const int PIN_ENDSTOP_CLOSED   = CONTROLLINO_A8; // THIS IST REQUIRED for input binary sensor
static const int PIN_ENDSTOP_OPEN     = -1;             // FIXME: optional
static const int PIN_LIGHT_CURTAIN    = -1;             // FIXME: optional

// Sensor polarity
static const int CLOSED_ENDSTOP_ACTIVE_LEVEL = LOW;
static const int OPEN_ENDSTOP_ACTIVE_LEVEL   = LOW;
static const int LIGHT_CURTAIN_ACTIVE_LEVEL  = LOW;

static bool lastToggleWantedOpen = true;
static bool lastStartWasOpen = false; // last START direction



enum DoorState : uint8_t {
  IDLE        = 0,
  OPENING     = 1,
  CLOSING     = 2,
  STOPPED     = 3,
  START_OPEN  = 4,
  START_CLOSE = 5
};

static DoorState doorState = IDLE;
static bool doorMoving = false;
static uint32_t doorStartMs = 0;
static uint32_t lastDoorStateChangeMs = 0;

static uint32_t relayArmMs = 0;

static bool closedEndstopActive = false;
static bool openEndstopActive   = false;
static bool obstructionActive   = false;

static Bounce debouncer;

static uint32_t lastBlinkMs = 0;
static bool blinkOn = false;

static EthernetServer httpServer(80);
static EthernetClient netClient;
static PubSubClient mqtt(netClient);

static uint32_t lastMqttAttemptMs = 0;
static bool wantDiscoveryPublish = true;
static bool connectedOnce = false;

// Topics
static char topicBase[96];
static char topicCmd[128];
static char topicState[128];
static char topicAvail[128];
static char topicClosedBin[128];
static char topicOpenBin[128];
static char topicObstBin[128];

// Publish tracking
static char lastCoverState[16] = {0};
static bool lastClosedBin = false;
static bool lastOpenBin   = false;
static bool lastObstBin   = false;
static uint32_t lastStatePublishMs = 0;

// little helpers

static bool hasClosedSensor() { return (!DISABLE_EXTERNAL_SENSORS) && (PIN_ENDSTOP_CLOSED >= 0); }
static bool hasOpenSensor()   { return (!DISABLE_EXTERNAL_SENSORS) && (PIN_ENDSTOP_OPEN   >= 0); }
static bool hasCurtain()      { return (!DISABLE_EXTERNAL_SENSORS) && (PIN_LIGHT_CURTAIN  >= 0); }

static void startClosedOverrun() {
  closedOverrunActive = true;
  closedOverrunUntilMs = millis() + CLOSED_ENDSTOP_OVERRUN_MS;
}

static void cancelClosedOverrun() {
  closedOverrunActive = false;
  closedOverrunUntilMs = 0;
}


static void buildTopics() {
  snprintf(topicBase, sizeof(topicBase), "homea/%s", NODE_ID);
  snprintf(topicCmd, sizeof(topicCmd), "%s/cmd", topicBase);
  snprintf(topicState, sizeof(topicState), "%s/state", topicBase);
  snprintf(topicAvail, sizeof(topicAvail), "%s/availability", topicBase);

  snprintf(topicClosedBin, sizeof(topicClosedBin), "%s/bin/closed", topicBase);
  snprintf(topicOpenBin,   sizeof(topicOpenBin),   "%s/bin/open", topicBase);
  snprintf(topicObstBin,   sizeof(topicObstBin),   "%s/bin/obstruction", topicBase);
}

static void mqttPublish(const char* topic, const char* payload, bool retain=true) {
  if (!mqtt.connected()) return;
  mqtt.publish(topic, payload, retain);
}

static void powerOff() {
  digitalWrite(PIN_DOOR_POWER_RELAY, LOW);
}

static void allRelaysOff() {
  digitalWrite(PIN_DOOR_POWER_RELAY, LOW);
  digitalWrite(PIN_DOOR_DIR_RELAY, LOW);   // de-energise direction relay only when NOT moving
}

static void setDirectionOpen() {
  digitalWrite(PIN_DOOR_DIR_RELAY, HIGH); // HIGH=up
}

static void setDirectionClose() {
  digitalWrite(PIN_DOOR_DIR_RELAY, LOW);  // LOW=down
}

static const char* coverState() {
  if (doorMoving) {
    if (doorState == OPENING || doorState == START_OPEN)  return "opening";
    if (doorState == CLOSING || doorState == START_CLOSE) return "closing";
  }
  if (hasClosedSensor() && closedEndstopActive) return "closed";
  if (hasOpenSensor() && openEndstopActive)     return "open";
  if (doorState == STOPPED)                     return "stopped";
  // report "open" when unknown/intermediate
  return "open";
}


// SENSORS


static void readSensors() {
  if (hasClosedSensor()) {
    int v = digitalRead(PIN_ENDSTOP_CLOSED);
    closedEndstopActive = (v == CLOSED_ENDSTOP_ACTIVE_LEVEL);
  } else closedEndstopActive = false;

  if (hasOpenSensor()) {
    int v = digitalRead(PIN_ENDSTOP_OPEN);
    openEndstopActive = (v == OPEN_ENDSTOP_ACTIVE_LEVEL);
  } else openEndstopActive = false;

  if (hasCurtain()) {
    int v = digitalRead(PIN_LIGHT_CURTAIN);
    obstructionActive = (v == LIGHT_CURTAIN_ACTIVE_LEVEL);
  } else obstructionActive = false;
}


// The DOOR CONTROL Logic

static void stopMotion() {
  allRelaysOff();              // <- to save energy: BOTH relays off when not moving
  doorMoving = false;
  doorState = STOPPED;
  lastDoorStateChangeMs = millis();
  cancelClosedOverrun();
}


static void armStart(DoorState startState) {
  // startState is START_OPEN or START_CLOSE
  cancelClosedOverrun();
  doorState = startState;
  doorMoving = true;
  doorStartMs = millis();
  lastDoorStateChangeMs = doorStartMs;
  // keep direction asserted for the whole movement
  if (startState == START_OPEN)  setDirectionOpen();
  else                          setDirectionClose();
  // but keep POWER off until delay elapsed
  powerOff();
  relayArmMs = doorStartMs + RELAY_START_DELAY_MS;
}



static void commandOpen() {
  lastStartWasOpen = true;
  if (hasOpenSensor() && openEndstopActive) return;
  armStart(START_OPEN);
}

static void commandClose() {
  lastStartWasOpen = false;
  if (hasClosedSensor() && closedEndstopActive) return;
  if (hasCurtain() && obstructionActive) return;
  armStart(START_CLOSE);
}



static void commandToggle() {
  // 1) If currently moving -> STOP
  if (doorMoving) {
    stopMotion();
    return;
  }

  // 2) If not moving -> start in opposite direction of the last START
  // Prefer the endstop logic if available -> otherwise alternate direction.
  if (hasClosedSensor() && closedEndstopActive) {
    // physically closed -> open
    lastStartWasOpen = true;
    commandOpen();
    return;
  }

  if (hasOpenSensor() && openEndstopActive) {
    // physically open -> close
    lastStartWasOpen = false;
    commandClose();
    return;
  }

  // No sensors (or intermediate position): alternate direction
  if (lastStartWasOpen) {
    lastStartWasOpen = false;
    commandClose();
  } else {
    lastStartWasOpen = true;
    commandOpen();
  }
}



static void tickRelayStart() {
  if (!doorMoving) return;

  if (doorState == START_OPEN || doorState == START_CLOSE) {
    uint32_t now = millis();
    // keep MQTT responsive during wait
    mqtt.loop();

    if ((int32_t)(now - relayArmMs) >= 0) {
      digitalWrite(PIN_DOOR_POWER_RELAY, HIGH);
      doorState = (doorState == START_OPEN) ? OPENING : CLOSING;
    }
  }
}

static void enforceSafety() {
  if (!doorMoving) return;

  // stop if closing and light curtain triggers
  if ((doorState == CLOSING || doorState == START_CLOSE) && hasCurtain() && obstructionActive) {
    stopMotion();
    return;
  }

 // stop if closing and closed endstop hits -> but allow a short run-on
if ((doorState == CLOSING || doorState == START_CLOSE) && hasClosedSensor() && closedEndstopActive) {
  if (!closedOverrunActive && CLOSED_ENDSTOP_OVERRUN_MS > 0) {
    startClosedOverrun();         // first time we see it: arm the run-on
    return;                       // keep moving for now
  }

  // if run-on time elapsed -> stop
  if (closedOverrunActive) {
    uint32_t now = millis();
    if ((int32_t)(now - closedOverrunUntilMs) >= 0) {
      stopMotion();
      doorState = IDLE;
      return;
    } else {
      return; // still within run-on window, keep going
    }
  }

  // fallback (if overrun disabled)
  stopMotion();
  doorState = IDLE;
  return;
}


  // stop if opening and open endstop hits
  if ((doorState == OPENING || doorState == START_OPEN) && hasOpenSensor() && openEndstopActive) {
    stopMotion();
    doorState = IDLE;
    return;
  }
}

static void enforceTimeout() {
  if (!doorMoving) return;
  uint32_t now = millis();
  if (now - doorStartMs >= DOOR_MAX_MOVEMENT_MS) {
    stopMotion();
  }
}


// HOME ASSISTANT DISCOVERY


static void publishDiscovery() {
  // device fragment
  char dev[256];
  snprintf(dev, sizeof(dev),
    "{\"identifiers\":[\"%s\"],\"name\":\"%s\",\"manufacturer\":\"Sixtopia.net\",\"model\":\"GarageDoorController01\",\"sw_version\":\"%s\"}",
    NODE_ID, DEVICE_NAME, SW_VERSION
  );

  // Cover
  {
    char t[160];
    snprintf(t, sizeof(t), "homeassistant/cover/%s/garage_door/config", NODE_ID);

    char payload[860];
    snprintf(payload, sizeof(payload),
      "{"
        "\"name\":\"Garage Door01\","
        "\"unique_id\":\"%s_cover\","
        "\"availability_topic\":\"%s\","
        "\"state_topic\":\"%s\","
        "\"command_topic\":\"%s\","
        "\"payload_open\":\"OPEN\","
        "\"payload_close\":\"CLOSE\","
        "\"payload_stop\":\"STOP\","
        "\"state_open\":\"open\","
        "\"state_closed\":\"closed\","
        "\"state_opening\":\"opening\","
        "\"state_closing\":\"closing\","
        "\"state_stopped\":\"stopped\","
        "\"optimistic\":false,"
        "\"qos\":1,"
        "\"retain\":true,"
        "\"device\":%s"
      "}",
      NODE_ID, topicAvail, topicState, topicCmd, dev
    );
    mqttPublish(t, payload, true);
  }

  // Buttons
  auto publishButton = [&](const char* objectId, const char* name, const char* unique, const char* pressPayload) {
    char t[180];
    snprintf(t, sizeof(t), "homeassistant/button/%s/%s/config", NODE_ID, objectId);

    char payload[700];
    snprintf(payload, sizeof(payload),
      "{"
        "\"name\":\"%s\","
        "\"unique_id\":\"%s\","
        "\"availability_topic\":\"%s\","
        "\"command_topic\":\"%s\","
        "\"payload_press\":\"%s\","
        "\"qos\":1,"
        "\"device\":%s"
      "}",
      name, unique, topicAvail, topicCmd, pressPayload, dev
    );
    mqttPublish(t, payload, true);
  };

  char uidOpen[64];   snprintf(uidOpen,   sizeof(uidOpen),   "%s_btn_open",   NODE_ID);
  char uidClose[64];  snprintf(uidClose,  sizeof(uidClose),  "%s_btn_close",  NODE_ID);
  char uidStop[64];   snprintf(uidStop,   sizeof(uidStop),   "%s_btn_stop",   NODE_ID);
  char uidToggle[64]; snprintf(uidToggle, sizeof(uidToggle), "%s_btn_toggle", NODE_ID);

  publishButton("open",   "Garage Open",   uidOpen,   "OPEN");
  publishButton("close",  "Garage Close",  uidClose,  "CLOSE");
  publishButton("stop",   "Garage Stop",   uidStop,   "STOP");
  publishButton("toggle", "Garage Toggle", uidToggle, "TOGGLE");

  // Binary sensors
  auto publishBinary = [&](const char* objectId, const char* name, const char* unique, const char* stateTopic) {
    char t[220];
    snprintf(t, sizeof(t), "homeassistant/binary_sensor/%s/%s/config", NODE_ID, objectId);

    char payload[720];
    snprintf(payload, sizeof(payload),
      "{"
        "\"name\":\"%s\","
        "\"unique_id\":\"%s\","
        "\"availability_topic\":\"%s\","
        "\"state_topic\":\"%s\","
        "\"payload_on\":\"ON\","
        "\"payload_off\":\"OFF\","
        "\"qos\":1,"
        "\"device\":%s"
      "}",
      name, unique, topicAvail, stateTopic, dev
    );
    mqttPublish(t, payload, true);
  };

  if (hasClosedSensor()) {
    char uid[64]; snprintf(uid, sizeof(uid), "%s_bin_closed", NODE_ID);
    publishBinary("closed", "Garage Closed", uid, topicClosedBin);
  }
  if (hasOpenSensor()) {
    char uid[64]; snprintf(uid, sizeof(uid), "%s_bin_open", NODE_ID);
    publishBinary("open", "Garage Open", uid, topicOpenBin);
  }
  if (hasCurtain()) {
    char uid[64]; snprintf(uid, sizeof(uid), "%s_bin_obstruction", NODE_ID);
    publishBinary("obstruction", "Garage Obstruction", uid, topicObstBin);
  }

  mqttPublish(topicAvail, "online", true);
}

// STATE PUBLISH (on-change + 60s refresh)


static void publishStates(bool force=false) {
  const char* st = coverState();

  bool stChanged = (strncmp(lastCoverState, st, sizeof(lastCoverState)) != 0);
  bool clChanged = (lastClosedBin != closedEndstopActive);
  bool opChanged = (lastOpenBin   != openEndstopActive);
  bool obChanged = (lastObstBin   != obstructionActive);

  uint32_t now = millis();
  bool timeRefresh = (now - lastStatePublishMs >= STATE_REFRESH_MS);

  if (!force && !timeRefresh && !stChanged && !clChanged && !opChanged && !obChanged) return;

  mqttPublish(topicState, st, true);

  if (hasClosedSensor()) mqttPublish(topicClosedBin, closedEndstopActive ? "ON" : "OFF", true);
  if (hasOpenSensor())   mqttPublish(topicOpenBin,   openEndstopActive   ? "ON" : "OFF", true);
  if (hasCurtain())      mqttPublish(topicObstBin,   obstructionActive   ? "ON" : "OFF", true);

  // update cache
  strncpy(lastCoverState, st, sizeof(lastCoverState));
  lastCoverState[sizeof(lastCoverState)-1] = 0;
  lastClosedBin = closedEndstopActive;
  lastOpenBin   = openEndstopActive;
  lastObstBin   = obstructionActive;
  lastStatePublishMs = now;
}

// MQTT CALLBACK / CONNECT


static void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, topicCmd) != 0) return;

  char cmd[16];
  unsigned int n = (length < sizeof(cmd)-1) ? length : (sizeof(cmd)-1);
  memcpy(cmd, payload, n);
  cmd[n] = '\0';

  for (unsigned int i = 0; i < n; i++) {
    if (cmd[i] >= 'a' && cmd[i] <= 'z') cmd[i] = char(cmd[i] - 32);
  }

  if      (strcmp(cmd, "OPEN")   == 0) commandOpen();
  else if (strcmp(cmd, "CLOSE")  == 0) commandClose();
  else if (strcmp(cmd, "STOP")   == 0) stopMotion();
  else if (strcmp(cmd, "TOGGLE") == 0) commandToggle();

  publishStates(true);
}

static void mqttEnsureConnected() {
  if (mqtt.connected()) return;

  uint32_t now = millis();
  if (now - lastMqttAttemptMs < MQTT_RECONNECT_MIN_MS) return;
  lastMqttAttemptMs = now;

  char clientId[64];
  snprintf(clientId, sizeof(clientId), "%s_%lu", NODE_ID, (unsigned long)now);

  bool ok = mqtt.connect(
    clientId,
    MQTT_USER, MQTT_PASS,
    topicAvail, 1, true, "offline"
  );

  if (ok) {
    mqtt.subscribe(topicCmd, 1);
    mqttPublish(topicAvail, "online", true);

    // publish discovery once on boot + once on each successful MQTT connect
    if (wantDiscoveryPublish || !connectedOnce) {
      publishDiscovery();
      wantDiscoveryPublish = false;
    } else {
      publishDiscovery();
    }
    connectedOnce = true;

    publishStates(true);
  }
}

// WEB UI


static void httpSend(EthernetClient& c) {
  c.println("HTTP/1.1 200 OK");
  c.println("Content-Type: text/html; charset=utf-8");
  c.println("Connection: close");
  c.println();

  const char* st = coverState();

  c.println("<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>");
  c.println("<title>Garage Door</title><style>");
  c.println("body{margin:0;font-family:Arial;background:#222;color:#fff;display:flex;align-items:center;justify-content:center;min-height:100vh;}");
  c.println(".wrap{width:min(520px,92vw);text-align:center;}");
  c.println("h1{font-size:22px;margin:8px 0 18px 0;font-weight:600;}");
  c.println(".btn{width:100%;padding:18px 14px;margin:8px 0;font-size:20px;border:0;border-radius:14px;background:#444;color:#fff;}");
  c.println(".row{display:flex;gap:10px;}");
  c.println(".row .btn{width:100%;}");
  c.println(".card{background:#2d2d2d;border-radius:14px;padding:14px;margin-top:14px;text-align:left;}");
  c.println(".dot{display:inline-block;width:14px;height:14px;border-radius:50%;margin-right:10px;vertical-align:middle;}");
  c.println(".ok{background:#00c853}.bad{background:#ff1744}.warn{background:#ffab00}");
  c.println("</style></head><body><div class='wrap'>");

  c.println("<h1>Garage Door Controller</h1>");
  c.println("<div class='row'>");
  c.println("<button class='btn' onclick=\"location.href='/open'\">Open</button>");
  c.println("<button class='btn' onclick=\"location.href='/close'\">Close</button>");
  c.println("</div>");
  c.println("<button class='btn' onclick=\"location.href='/stop'\">Stop</button>");
  c.println("<button class='btn' onclick=\"location.href='/toggle'\">Toggle</button>");

  c.println("<div class='card'>");

  c.print("<div><span class='dot ");
  if (doorMoving) c.print("warn");
  else if (hasClosedSensor() && closedEndstopActive) c.print("ok");
  else c.print("bad");
  c.print("'></span>Status: ");
  c.print(st);
  c.println("</div>");

  c.print("<div style='margin-top:8px;'>Closed endstop: ");
  c.print(hasClosedSensor() ? (closedEndstopActive ? "ACTIVE" : "inactive") : "disabled");
  c.println("</div>");

  c.print("<div style='margin-top:8px;'>Light curtain: ");
  c.print(hasCurtain() ? (obstructionActive ? "OBSTRUCTION" : "clear") : "disabled");
  c.println("</div>");

  c.println("</div></div></body></html>");
}

static void httpHandle() {
  EthernetClient client = httpServer.available();
  if (!client) return;

  // Read only the first request line with a tight timeout.
  char reqLine[96];
  uint8_t idx = 0;

  uint32_t start = millis();
  while ((millis() - start) < 30) {  // ~30ms window
    mqtt.loop();
    if (!client.available()) continue;

    char ch = client.read();
    if (ch == '\r') continue;
    if (ch == '\n') break;

    if (idx < sizeof(reqLine) - 1) reqLine[idx++] = ch;
  }
  reqLine[idx] = 0;

  if      (strncmp(reqLine, "GET /open",   9) == 0) commandOpen();
  else if (strncmp(reqLine, "GET /close", 10) == 0) commandClose();
  else if (strncmp(reqLine, "GET /stop",   9) == 0) stopMotion();
  else if (strncmp(reqLine, "GET /toggle",11) == 0) commandToggle();

  httpSend(client);
  client.stop();

  publishStates(true);
}


// STATUS LIGHTS


static void updateStatusLights() {
  uint32_t now = millis();

  if (doorMoving) {
    if (now - lastBlinkMs >= BLINK_MS) {
      lastBlinkMs = now;
      blinkOn = !blinkOn;
      digitalWrite(PIN_STATUS_LIGHT, blinkOn ? HIGH : LOW);
    }
  } else {
    digitalWrite(PIN_STATUS_LIGHT, HIGH);
  }

  if (hasClosedSensor()) digitalWrite(PIN_STATUS_LOCKED, closedEndstopActive ? HIGH : LOW);
  else digitalWrite(PIN_STATUS_LOCKED, LOW);
}

// SETUP


void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println("GarageController v3.0 (Controllino Maxi W5100 Ethernet)");

  // Mega2560 stays SPI master
  pinMode(53, OUTPUT);
  digitalWrite(53, HIGH);

  // Outputs
  pinMode(PIN_DOOR_POWER_RELAY, OUTPUT);
  pinMode(PIN_DOOR_DIR_RELAY, OUTPUT);
  pinMode(PIN_STATUS_LIGHT, OUTPUT);
  pinMode(PIN_STATUS_LOCKED, OUTPUT);

  allRelaysOff(); // <- both off at boot
  digitalWrite(PIN_STATUS_LIGHT, LOW);
  digitalWrite(PIN_STATUS_LOCKED, LOW);

  // Local button
  pinMode(PIN_LOCAL_BUTTON, INPUT);
  debouncer.attach(PIN_LOCAL_BUTTON);
  debouncer.interval(DEBOUNCE_MS);

  // Sensors
  if (!DISABLE_EXTERNAL_SENSORS) {
    if (PIN_ENDSTOP_CLOSED >= 0) pinMode(PIN_ENDSTOP_CLOSED, INPUT);
    if (PIN_ENDSTOP_OPEN   >= 0) pinMode(PIN_ENDSTOP_OPEN,   INPUT);
    if (PIN_LIGHT_CURTAIN  >= 0) pinMode(PIN_LIGHT_CURTAIN,  INPUT);
  }

  buildTopics();

  // Ethernet
  Ethernet.begin(MAC_ADDR, IP_ADDR, DNS_ADDR, GATEWAY, SUBNET);
  delay(150);

  Serial.print("IP: "); Serial.println(Ethernet.localIP());

  httpServer.begin();

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setKeepAlive(30);

  readSensors();

  wantDiscoveryPublish = true;

  lastCoverState[0] = 0;
  lastStatePublishMs = 0;
}

void loop() {
  // Local toggle button
  debouncer.update();
  int btn = debouncer.read();

  static int lastBtn = HIGH;
  if (btn == LOW && lastBtn == HIGH) {
    uint32_t now = millis();
    if (now - lastDoorStateChangeMs >= LOCAL_TOGGLE_GUARD_MS) {
      commandToggle();
      publishStates(true);
    }
  }
  lastBtn = btn;

  // Sensors + safety
  readSensors();
  tickRelayStart();
  enforceSafety();
  enforceTimeout();

  // MQTT
  mqttEnsureConnected();
  mqtt.loop();

  // Web
  httpHandle();

  // Lights
  updateStatusLights();

  // Publish only on change, but refresh every 60s
  publishStates(false);
}
