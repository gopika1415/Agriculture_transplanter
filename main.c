#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <TinyGPSPlus.h>

// ---------- WiFi ----------
const char* ssid = "Realme P4 5G x777";
const char* password = "ycgf8824";

// ---------- MQTT ----------
const char* mqtt_server = "93482c172cb3465898cde100a0b26b0d.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_user = "agribot";
const char* mqtt_pass = "Agribot123";

WiFiClientSecure espClient;
PubSubClient client(espClient);

// ---------- GPS ----------
HardwareSerial gpsSerial(1);
#define GPS_RX 16
#define GPS_TX 17
TinyGPSPlus gps;

// ---------- STM32 UART ----------
HardwareSerial STM32(2); 
#define STM_TX 25
#define STM_RX 26

// ---------- POWER SWITCH ----------
#define POWER_SWITCH 4   // 🔥 define pin

// ---------- Variables ----------
double gpsLat = 0.0;
double gpsLng = 0.0;

bool powerState = false;
bool lastPowerState = false;

// ---------- MQTT Callback ----------
void callback(char* topic, byte* payload, unsigned int length) {

  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  if (String(topic) == "agriBot/mode") {

    Serial.println("Mode Received: " + message);

    if(message == "1"){
      STM32.write('A');
      Serial.println("Tomato Mode → A");
    }

    else if(message == "2"){
      STM32.write('B');
      Serial.println("Brinjal Mode → B");
    }

    else if(message == "3"){
      STM32.write('C');
      Serial.println("Chilli Mode → C");
    }
  }
}

// ---------- MQTT Reconnect ----------
void reconnect() {

  while (!client.connected()) {

    Serial.print("Connecting MQTT...");

    if (client.connect("ESP32Client", mqtt_user, mqtt_pass)) {
      Serial.println("Connected");
      client.subscribe("agriBot/mode");
    } else {
      Serial.println("Failed → retrying...");
      delay(5000);
    }
  }
}

// ---------- SETUP ----------
void setup() {

  Serial.begin(115200);

  pinMode(POWER_SWITCH, INPUT_PULLUP);

  // GPS
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  // STM32 UART
  STM32.begin(115200, SERIAL_8N1, STM_RX, STM_TX);

  // WiFi
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi Connected");

  espClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

// ---------- LOOP ----------
void loop() {

  if (!client.connected()) reconnect();
  client.loop();

  // ===== READ SWITCH =====
  powerState = (digitalRead(POWER_SWITCH) == LOW);

  // ===== HANDLE POWER CHANGE =====
  if(powerState != lastPowerState){

    delay(50); // debounce
    powerState = (digitalRead(POWER_SWITCH) == LOW);

    if(powerState != lastPowerState){

      lastPowerState = powerState;

      String powerMsg = powerState ? "ON" : "OFF";

      String json = "{";
      json += "\"power\":\"" + powerMsg + "\"";
      json += "}";

      client.publish("agriBot/status", json.c_str());

      Serial.println("Power: " + powerMsg);

      // 🚨 SYSTEM SHUTDOWN ACTION
      if(!powerState){
        STM32.write('S');  // STOP command (define in STM32)
        Serial.println("System STOPPED");
      }
    }
  }

  // ❌ STOP EVERYTHING IF POWER OFF
  if(!powerState){
    return;   // 🔥 THIS LINE IS KEY
  }

  // ===============================
  // ===== RUN ONLY IF POWER ON ====
  // ===============================

  // ===== GPS =====
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isUpdated()) {

    gpsLat = gps.location.lat();
    gpsLng = gps.location.lng();

    String gpsJson = "{";
    gpsJson += "\"lat\":" + String(gpsLat, 6) + ",";
    gpsJson += "\"lng\":" + String(gpsLng, 6);
    gpsJson += "}";

    client.publish("agriBot/gps", gpsJson.c_str());

    Serial.println("GPS Sent");
  }

  // ===== PERIODIC STATUS =====
  static unsigned long lastPublish = 0;

  if (millis() - lastPublish > 5000) {

    lastPublish = millis();

    String statusJson = "{";
    statusJson += "\"power\":\"ON\"";
    statusJson += "}";

    client.publish("agriBot/status", statusJson.c_str());

    Serial.println("Status Updated");
  }
}
