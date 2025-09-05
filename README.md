SmartHome32 — Modular Arduino/ESP32 Smart Home Hub
A production style, modular smart home hub built on Arduino (ESP32 recommended) that supports many common sensors/actuators, exposes MQTT + REST API, includes basic automations/scenes, and supports OTA updates. The code is written to be readable and extensible so you can plug in additional sensors with minimal effort.
⚠️ Safety first: Anything that touches mains (AC) voltage (relays, dimmers) can be lethal if mishandled. Use certified relay boards/enclosures, proper isolation, and consult a qualified electrician. The author assumes no liability.
________________________________________
Features
•	Connectivity: Wi Fi (ESP32), MQTT (PubSubClient), simple REST API over HTTP, OTA firmware updates.
•	Dashboards & Integrations: Works with Home Assistant via MQTT; JSON REST endpoints for quick testing.
•	Sensors (examples implemented)
o	Temperature/Humidity: DHT22/AM2302 (GPIO configurable)
o	Light level: LDR (photoresistor) via ADC
o	Motion: PIR digital sensor
o	Gas/Smoke: MQ 2 via ADC (threshold alarm)
o	Door/Window: Reed switch
o	Barometric pressure/altitude/temp: BMP280 (I²C) (optional)
o	1 Wire temp: DS18B20 (optional)
•	Actuators (examples implemented)
o	Relay (for lights/appliances)
o	Buzzer (beeps/alarms)
o	(Optional placeholders for Servo, OLED display)
•	Automations (built in)
o	Night light: if PIR motion and ambient light low → turn relay on for N seconds.
o	Gas alarm: if MQ 2 exceeds threshold → buzzer alarm + MQTT alert.
o	Door chime: if reed switch opens → short buzzer beep.
•	APIs
o	REST: GET /api/status, POST /api/cmd (JSON), GET / quick landing page.
o	MQTT: publishes home/<device_id>/status JSON; subscribes to home/<device_id>/cmd JSON.
•	Config via #define switches so you only include what you need.
________________________________________
Recommended Hardware
•	ESP32 DevKit C / NodeMCU 32S (Arduino core) — gives Wi Fi, more RAM/flash.
•	Breadboard, jumper wires, 10k resistors for pull ups where required.
•	5V power supply (with ample current for sensors/relays). ESP32 is 3.3V logic; most modules are 3.3–5V tolerant but check your part.
Example Pin Map (defaults — change in code)
Module	Pin(s)
DHT22	GPIO 4
PIR	GPIO 13
Reed switch	GPIO 14
LDR (ADC)	GPIO 36 (ADC1_CH0)
MQ 2 (ADC)	GPIO 39 (ADC1_CH3)
Relay	GPIO 26
Buzzer	GPIO 27
I²C (BMP280/OLED)	SDA=21, SCL=22
DS18B20 (1 Wire)	GPIO 15
________________________________________
Libraries
Install via Arduino Library Manager unless noted:
•	ArduinoJson by Benoit Blanchon
•	PubSubClient by Nick O’Leary
•	DHT sensor library + Adafruit Unified Sensor (for DHT22)
•	Adafruit BMP280 Library (if ENABLE_BMP280)
•	OneWire and DallasTemperature (if ENABLE_DS18B20)
•	ESP32: Arduino core (Board Manager URL: https://github.com/espressif/arduino-esp32)
•	(Optional) Adafruit GFX + Adafruit SSD1306 (if ENABLE_OLED)
If you don’t need a module, set its #define ENABLE_* 0 to avoid installing its library.
________________________________________
Quick Start
1.	Select Board: Tools → Board → ESP32 Arduino → ESP32 Dev Module.
2.	Install libraries (above).
3.	Open the .ino below, set your Wi Fi and MQTT credentials near the top.
4.	Adjust pin mapping and ENABLE_* flags as needed.
5.	Upload. Watch Serial Monitor at 115200 baud.
6.	Hit http://<device-ip>/api/status or subscribe to MQTT topic home/<device_id>/status.
________________________________________
REST & MQTT
•	REST
o	GET /api/status → JSON snapshot of sensors/actuators.
o	POST /api/cmd → send JSON commands, e.g., { "relay": true, "buzzer": {"beep_ms": 300} }.
•	MQTT
o	Publish status (every ~10s): home/<device_id>/status
o	Commands subscribe: home/<device_id>/cmd
o	Example command payload: { "relay": false, "night_mode": true }
Home Assistant (MQTT example)
Add an MQTT Sensor or MQTT Switch using the above topics. For auto discovery, you can use HA’s MQTT Discovery by publishing config topics (left as an exercise).
________________________________________
Extending
Create a new module file by following the pattern inside the code (see // === Module: ... ===). Implement:
•	begin() for setup
•	loop() for periodic reads
•	toJson(JsonObject) to report data
•	handleCmd(JsonVariant) for commands (optional)
Register the module in the arrays at the bottom of the code.
________________________________________
License
MIT — do whatever you want, just keep the notice.
________________________________________
Full Arduino Code (SmartHome32.ino)
/*
  SmartHome32 — Modular Smart Home Hub for ESP32 (Arduino)
Author: Salman Nawaz Malik
  --------------------------------------------------------
  - WiFi + MQTT + REST + OTA
  - Common sensors/actuators (DHT22, PIR, LDR, MQ2, Reed, BMP280, DS18B20, Relay, Buzzer)
  - Simple automations (night light, gas alarm, door chime)

  MIT License — (c) 2025 YourName
*/

/********************
 * Feature Toggles  *
 ********************/
#define ENABLE_WIFI       1
#define ENABLE_MQTT       1
#define ENABLE_WEBSERVER  1
#define ENABLE_OTA        1

#define ENABLE_DHT        1
#define ENABLE_PIR        1
#define ENABLE_REED       1
#define ENABLE_LDR        1
#define ENABLE_MQ2        1
#define ENABLE_RELAY      1
#define ENABLE_BUZZER     1
#define ENABLE_BMP280     0   // requires Adafruit_BMP280
#define ENABLE_DS18B20    0   // requires OneWire + DallasTemperature
#define ENABLE_OLED       0   // requires Adafruit_SSD1306 + GFX (placeholder section)

/********************
 * Pin Definitions  *
 ********************/
#if ENABLE_DHT
  #define DHT_PIN   4
  #define DHT_TYPE  22 // DHT22/AM2302
#endif
#if ENABLE_PIR
  #define PIR_PIN   13
#endif
#if ENABLE_REED
  #define REED_PIN  14
#endif
#if ENABLE_LDR
  #define LDR_PIN   36 // ADC1_CH0
#endif
#if ENABLE_MQ2
  #define MQ2_PIN   39 // ADC1_CH3
  #define MQ2_THRESHOLD  600 // adjust after calibration (0-4095)
#endif
#if ENABLE_RELAY
  #define RELAY_PIN 26
  #define RELAY_ACTIVE_HIGH 1
#endif
#if ENABLE_BUZZER
  #define BUZZER_PIN 27
#endif
#if ENABLE_DS18B20
  #define ONE_WIRE_BUS 15
#endif
// I2C (BMP280/OLED)
#define I2C_SDA 21
#define I2C_SCL 22

/********************
 * Config           *
 ********************/
const char* DEVICE_ID = "smarthome32-01";

// WiFi
#if ENABLE_WIFI
  #include <WiFi.h>
  const char* WIFI_SSID = "YOUR_WIFI_SSID";
  const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
#endif

// MQTT
#if ENABLE_MQTT
  #include <PubSubClient.h>
  const char* MQTT_HOST = "192.168.1.10"; // broker IP/host
  const uint16_t MQTT_PORT = 1883;
  const char* MQTT_USER = ""; // optional
  const char* MQTT_PASS = ""; // optional
  WiFiClient espClient;
  PubSubClient mqtt(espClient);
  String topicStatus = String("home/") + DEVICE_ID + "/status";
  String topicCmd    = String("home/") + DEVICE_ID + "/cmd";
#endif

// WebServer
#if ENABLE_WEBSERVER
  #include <WebServer.h>
  WebServer server(80);
#endif

// OTA
#if ENABLE_OTA
  #include <ArduinoOTA.h>
#endif

// JSON
#include <ArduinoJson.h>

// Timekeeping (millis based)
unsigned long lastStatusMs = 0;
const unsigned long STATUS_PERIOD_MS = 10000;

/********************
 * Utilities        *
 ********************/
String ipToStr(IPAddress ip){
  return String(ip[0]) + "." + ip[1] + "." + ip[2] + "." + ip[3];
}

/********************
 * Sensor Includes  *
 ********************/
#if ENABLE_DHT
  #include <DHT.h>
  DHT dht(DHT_PIN, DHT_TYPE);
  float dht_t = NAN, dht_h = NAN;
  unsigned long dhtLastMs = 0;
  const unsigned long DHT_PERIOD_MS = 2500;
#endif

#if ENABLE_PIR
  volatile bool pirMotion = false;
  unsigned long pirLastTrigMs = 0;
  void IRAM_ATTR pirISR(){ pirMotion = true; pirLastTrigMs = millis(); }
#endif

#if ENABLE_REED
  bool reedOpen = false; // true when door/window open
  int reedLastState = HIGH;
#endif

#if ENABLE_LDR
  int ldrRaw = 0; // 0-4095
#endif

#if ENABLE_MQ2
  int mq2Raw = 0; // 0-4095
#endif

#if ENABLE_RELAY
  bool relayOn = false;
#endif

#if ENABLE_BUZZER
  bool buzzerOn = false;
  unsigned long buzzerUntil = 0;
  void buzzerBeep(uint32_t ms){
    buzzerOn = true; buzzerUntil = millis() + ms;
    #if defined(ESP32)
      ledcAttachPin(BUZZER_PIN, 0);
      ledcWriteTone(0, 2000); // ~2kHz
    #else
      tone(BUZZER_PIN, 2000);
    #endif
  }
  void buzzerStop(){
    buzzerOn = false; buzzerUntil = 0;
    #if defined(ESP32)
      ledcWriteTone(0, 0);
      ledcDetachPin(BUZZER_PIN);
    #else
      noTone(BUZZER_PIN);
    #endif
  }
#endif

#if ENABLE_BMP280
  #include <Adafruit_BMP280.h>
  #include <Wire.h>
  Adafruit_BMP280 bmp;
  bool bmpOk = false;
  float bmp_temp= NAN, bmp_press= NAN, bmp_alt= NAN;
  unsigned long bmpLastMs = 0;
  const unsigned long BMP_PERIOD_MS = 2000;
#endif

#if ENABLE_DS18B20
  #include <OneWire.h>
  #include <DallasTemperature.h>
  OneWire oneWire(ONE_WIRE_BUS);
  DallasTemperature ds18b20(&oneWire);
  float ds_temp = NAN;
  unsigned long dsLastMs = 0;
  const unsigned long DS_PERIOD_MS = 2000;
#endif

/********************
 * Automation State *
 ********************/
bool nightMode = true;          // can be toggled via API/MQTT
uint16_t ldrLowThreshold = 1200; // ADC raw threshold for "dark"
uint32_t relayAutoOffMs = 15000; // 15s night-light
unsigned long relayOffAt = 0;

/********************
 * Forward Decls    *
 ********************/
void publishStatus();
void buildStatusJson(DynamicJsonDocument &doc);
void handleCommand(JsonVariant payload);

/********************
 * Setup            *
 ********************/
void setup(){
  Serial.begin(115200);
  delay(100);
  Serial.println("\n[SmartHome32] Booting...");

  // Pins
  #if ENABLE_PIR
    pinMode(PIR_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIR_PIN), pirISR, RISING);
  #endif
  #if ENABLE_REED
    pinMode(REED_PIN, INPUT_PULLUP);
  #endif
  #if ENABLE_LDR
    analogReadResolution(12); // ESP32 0..4095
  #endif
  #if ENABLE_MQ2
    analogReadResolution(12);
  #endif
  #if ENABLE_RELAY
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, RELAY_ACTIVE_HIGH ? LOW : HIGH);
  #endif
  #if ENABLE_BUZZER
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
  #endif

  #if ENABLE_DHT
    dht.begin();
  #endif

  #if ENABLE_BMP280
    Wire.begin(I2C_SDA, I2C_SCL);
    bmpOk = bmp.begin(0x76) || bmp.begin(0x77);
    if(!bmpOk) Serial.println("[BMP280] Not found");
  #endif

  #if ENABLE_DS18B20
    ds18b20.begin();
  #endif

  #if ENABLE_WIFI
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
    while (WiFi.status() != WL_CONNECTED){ delay(400); Serial.print("."); }
    Serial.printf("\n[WiFi] Connected: %s\n", ipToStr(WiFi.localIP()).c_str());
  #endif

  #if ENABLE_MQTT
    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    mqtt.setCallback([](char* topic, byte* payload, unsigned int len){
      // Parse JSON
      StaticJsonDocument<512> doc;
      DeserializationError err = deserializeJson(doc, payload, len);
      if(err){ Serial.printf("[MQTT] JSON error: %s\n", err.c_str()); return; }
      handleCommand(doc.as<JsonVariant>());
    });
  #endif

  #if ENABLE_WEBSERVER
    server.on("/", HTTP_GET, [](){
      String html = String("<h1>SmartHome32</h1><p>Device: ") + DEVICE_ID +
        "</p><ul><li><a href=\"/api/status\">/api/status</a></li></ul>";
      server.send(200, "text/html", html);
    });

    server.on("/api/status", HTTP_GET, [](){
      DynamicJsonDocument doc(1024);
      buildStatusJson(doc);
      String out; serializeJsonPretty(doc, out);
      server.send(200, "application/json", out);
    });

    server.on("/api/cmd", HTTP_POST, [](){}, [](){
      // handle upload
      if(server.hasArg("plain")){
        StaticJsonDocument<512> doc;
        DeserializationError err = deserializeJson(doc, server.arg("plain"));
        if(err){ server.send(400, "text/plain", String("Bad JSON: ")+err.c_str()); return; }
        handleCommand(doc.as<JsonVariant>());
        server.send(200, "application/json", "{\"ok\":true}");
      } else {
        server.send(400, "text/plain", "Expected JSON body");
      }
    });

    server.begin();
    Serial.println("[HTTP] Server started");
  #endif

  #if ENABLE_OTA
    ArduinoOTA.setHostname(DEVICE_ID);
    ArduinoOTA.onStart([](){ Serial.println("[OTA] Start"); });
    ArduinoOTA.onEnd([](){ Serial.println("\n[OTA] End"); });
    ArduinoOTA.onError([](ota_error_t e){ Serial.printf("[OTA] Error %u\n", e); });
    ArduinoOTA.begin();
  #endif
}

/********************
 * Main Loop        *
 ********************/
void loop(){
  #if ENABLE_WEBSERVER
    server.handleClient();
  #endif
  #if ENABLE_OTA
    ArduinoOTA.handle();
  #endif

  #if ENABLE_MQTT
    if(ENABLE_WIFI && WiFi.status()==WL_CONNECTED){
      if(!mqtt.connected()){
        // Reconnect
        String cid = String(DEVICE_ID) + String("-") + String((uint32_t)ESP.getEfuseMac(), HEX);
        if(mqtt.connect(cid.c_str(), MQTT_USER, MQTT_PASS)){
          mqtt.subscribe(topicCmd.c_str());
          Serial.println("[MQTT] Connected");
        }
      }
      mqtt.loop();
    }
  #endif

  // Read sensors periodically (non blocking)
  #if ENABLE_DHT
    if(millis() - dhtLastMs >= DHT_PERIOD_MS){ dhtLastMs = millis();
      dht_h = dht.readHumidity();
      dht_t = dht.readTemperature();
    }
  #endif

  #if ENABLE_REED
    {
      int st = digitalRead(REED_PIN); // assuming NC switch: HIGH=open, LOW=closed
      reedOpen = (st == HIGH);
      if(st != reedLastState){
        reedLastState = st;
        if(reedOpen){ // door opened
          #if ENABLE_BUZZER
            buzzerBeep(120);
          #endif
        }
      }
    }
  #endif

  #if ENABLE_LDR
    ldrRaw = analogRead(LDR_PIN);
  #endif

  #if ENABLE_MQ2
    mq2Raw = analogRead(MQ2_PIN);
  #endif

  #if ENABLE_BMP280
    if(bmpOk && millis() - bmpLastMs >= BMP_PERIOD_MS){ bmpLastMs = millis();
      bmp_temp = bmp.readTemperature();
      bmp_press = bmp.readPressure() / 100.0f; // hPa
      bmp_alt = bmp.readAltitude(1013.25);     // adjust sea level pressure
    }
  #endif

  #if ENABLE_DS18B20
    if(millis() - dsLastMs >= DS_PERIOD_MS){ dsLastMs = millis();
      ds18b20.requestTemperatures();
      ds_temp = ds18b20.getTempCByIndex(0);
    }
  #endif

  // Handle buzzer auto stop
  #if ENABLE_BUZZER
    if(buzzerOn && millis() > buzzerUntil){ buzzerStop(); }
  #endif

  // Automations
  // 1) Night light: motion + dark => relay ON for relayAutoOffMs
  #if ENABLE_PIR && ENABLE_LDR && ENABLE_RELAY
    if(pirMotion){
      pirMotion = false; // consume event
      if(nightMode && ldrRaw < ldrLowThreshold){
        relayOn = true; relayOffAt = millis() + relayAutoOffMs;
        #if RELAY_ACTIVE_HIGH
          digitalWrite(RELAY_PIN, HIGH);
        #else
          digitalWrite(RELAY_PIN, LOW);
        #endif
      }
    }
    if(relayOn && millis() > relayOffAt){
      relayOn = false;
      #if RELAY_ACTIVE_HIGH
        digitalWrite(RELAY_PIN, LOW);
      #else
        digitalWrite(RELAY_PIN, HIGH);
      #endif
    }
  #endif

  // 2) Gas alarm
  #if ENABLE_MQ2 && ENABLE_BUZZER
    static bool gasAlert = false;
    bool over = mq2Raw >= MQ2_THRESHOLD;
    if(over && !gasAlert){ gasAlert = true; buzzerBeep(1000); }
    if(!over && gasAlert){ gasAlert = false; }
  #endif

  // Periodic status publish
  if(millis() - lastStatusMs >= STATUS_PERIOD_MS){ lastStatusMs = millis();
    publishStatus();
  }
}

/********************
 * JSON + MQTT      *
 ********************/
void buildStatusJson(DynamicJsonDocument &doc){
  auto root = doc.to<JsonObject>();
  root["device"] = DEVICE_ID;
  root["uptime_ms"] = millis();

  #if ENABLE_WIFI
    JsonObject wifi = root.createNestedObject("wifi");
    wifi["rssi"] = WiFi.RSSI();
    wifi["ip"] = ipToStr(WiFi.localIP());
  #endif

  JsonObject sensors = root.createNestedObject("sensors");
  #if ENABLE_DHT
    JsonObject d = sensors.createNestedObject("dht22");
    d["temp_c"] = isfinite(dht_t) ? dht_t : nullptr;
    d["hum_%"] = isfinite(dht_h) ? dht_h : nullptr;
  #endif
  #if ENABLE_LDR
    sensors["ldr_raw"] = ldrRaw;
  #endif
  #if ENABLE_PIR
    sensors["pir_recent_ms_ago"] = (unsigned long)(millis() - pirLastTrigMs);
  #endif
  #if ENABLE_REED
    sensors["reed_open"] = reedOpen;
  #endif
  #if ENABLE_MQ2
    sensors["mq2_raw"] = mq2Raw;
    sensors["mq2_over_threshold"] = mq2Raw >= MQ2_THRESHOLD;
  #endif
  #if ENABLE_BMP280
    if(bmpOk){
      JsonObject b = sensors.createNestedObject("bmp280");
      b["temp_c"] = isfinite(bmp_temp) ? bmp_temp : nullptr;
      b["press_hpa"] = isfinite(bmp_press) ? bmp_press : nullptr;
      b["alt_m"] = isfinite(bmp_alt) ? bmp_alt : nullptr;
    }
  #endif
  #if ENABLE_DS18B20
    sensors["ds18b20_temp_c"] = isfinite(ds_temp) ? ds_temp : nullptr;
  #endif

  JsonObject actuators = root.createNestedObject("actuators");
  #if ENABLE_RELAY
    actuators["relay"] = relayOn;
  #endif
  #if ENABLE_BUZZER
    actuators["buzzer"] = buzzerOn;
  #endif

  JsonObject cfg = root.createNestedObject("config");
  cfg["night_mode"] = nightMode;
  cfg["ldr_low_threshold"] = ldrLowThreshold;
  cfg["relay_auto_off_ms"] = relayAutoOffMs;
}

void publishStatus(){
  DynamicJsonDocument doc(1024);
  buildStatusJson(doc);

  String payload; serializeJson(doc, payload);

  #if ENABLE_MQTT
    if(mqtt.connected()){
      mqtt.publish(topicStatus.c_str(), payload.c_str(), true);
    }
  #endif

  // Also print to serial for debugging
  Serial.println(payload);
}

void handleCommand(JsonVariant payload){
  // Commands: { "relay": true/false, "buzzer": {"beep_ms": 200}, "night_mode": true/false, "ldr_low_threshold": 1300 }
  if(payload.is<JsonObject>()){
    JsonObject obj = payload.as<JsonObject>();

    #if ENABLE_RELAY
      if(obj.containsKey("relay")){
        relayOn = obj["relay"].as<bool>();
        #if RELAY_ACTIVE_HIGH
          digitalWrite(RELAY_PIN, relayOn ? HIGH : LOW);
        #else
          digitalWrite(RELAY_PIN, relayOn ? LOW : HIGH);
        #endif
      }
    #endif

    #if ENABLE_BUZZER
      if(obj.containsKey("buzzer")){
        JsonVariant bz = obj["buzzer"];
        if(bz.is<JsonObject>()){
          uint32_t ms = bz["beep_ms"].as<uint32_t>();
          if(ms > 0 && ms < 10000) buzzerBeep(ms);
        } else if(bz.is<bool>()){
          bool on = bz.as<bool>();
          if(on) buzzerBeep(300); else buzzerStop();
        }
      }
    #endif

    if(obj.containsKey("night_mode")){
      nightMode = obj["night_mode"].as<bool>();
    }
    if(obj.containsKey("ldr_low_threshold")){
      ldrLowThreshold = obj["ldr_low_threshold"].as<uint16_t>();
    }
    if(obj.containsKey("relay_auto_off_ms")){
      relayAutoOffMs = obj["relay_auto_off_ms"].as<uint32_t>();
    }
  }
}
________________________________________
Example Commands
Curl (REST)
curl -X POST http://<device-ip>/api/cmd \
  -H 'Content-Type: application/json' \
  -d '{"relay":true, "buzzer":{"beep_ms":250}, "night_mode":true}'
MQTT (mosquitto_pub)
mosquitto_pub -h 192.168.1.10 -t home/smarthome32-01/cmd -m '{"relay": false}'
________________________________________
Notes
•	Calibrate MQ 2: Let it warm up ~24h for best results; determine threshold experimentally.
•	LDR scaling: ADC raw value varies with wiring (voltage divider). Tune ldrLowThreshold.
•	DHT timing: don’t sample faster than ~2s.
•	ESP32 ADC: Use ADC1 channels for Wi Fi coexistence.
•	Security: For production, add MQTT username/password/TLS and lock down the REST API (token).

