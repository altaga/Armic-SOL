#include "creds.h"
#include "MQTTManager.h"
#include <ArduinoJson.h>
#include <SPI.h>
#include "protocentral_max30003.h"

// --- Configuration ---
#define PACKAGE_SIZE 20
#define AWS_IOT_TOPIC "/armic/ecg-1"
#define HEARTBEAT_TOPIC "/armic/HB"

// Set to 'true' to use the Serial Plotter (mutes text logs)
// Set to 'false' to debug the network and initialization
#define PLOTTER_MODE true 

// Adafruit Feather ESP32 Pin Mapping
#define MAX30003_CS_PIN 26  // Feather pin A0

String unique_client_id;
MAX30003 max30003(MAX30003_CS_PIN);

// --- Data Buffers ---
int32_t bufferToSend[PACKAGE_SIZE]; 
int counter = 0;

// --- Software Timers ---
unsigned long lastHeartbeatTime = 0;
const unsigned long HEARTBEAT_INTERVAL = 10000; // 10 seconds

unsigned long lastSensorReadTime = 0;
const unsigned long SENSOR_INTERVAL = 8; // 8 milliseconds (~125 Hz)

void setup() {
  Serial.begin(115200);
  
  // 1. Setup Unique ID
  uint64_t chipid = ESP.getEfuseMac();
  uint32_t lower32 = (uint32_t)chipid;
  uint16_t upper16 = (uint16_t)(chipid >> 32);
  char idBuff[30];
  snprintf(idBuff, sizeof(idBuff), "%s%04X%08X", client_base_name, upper16, lower32);
  unique_client_id = String(idBuff);

  // 2. Connect to Network
  wifiConnect(ssid, password);
  mqttConnect(host, mqtt_user, mqtt_pass, unique_client_id.c_str());

  // 3. Initialize SPI and MAX30003
  pinMode(MAX30003_CS_PIN, OUTPUT);
  digitalWrite(MAX30003_CS_PIN, HIGH);
  SPI.begin();
  
  // Verify chip connection
  bool ret = max30003.readDeviceID();
  while (!ret) {
    if (!PLOTTER_MODE) {
      Serial.println("Failed to read MAX30003 ID. Check SPI wiring!");
    } else {
      Serial.println(0); 
    }
    delay(1000);
    ret = max30003.readDeviceID();
  }
  
  if (!PLOTTER_MODE) Serial.println("MAX30003 initialized.");
  max30003.begin(); 
  
  if (!PLOTTER_MODE) Serial.println("\n[SYSTEM] Ready. Starting Software Polling...");
}

void loop() {
  
  // --- Task 1: Check and Send 10-Second Heartbeat ---
  if (millis() - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
    lastHeartbeatTime = millis();
    
    if (current_state == STATE_CONNECTED) {
      JsonDocument hbDoc;
      hbDoc["device"] = unique_client_id;
      hbDoc["status"] = "online";
      hbDoc["uptime_sec"] = millis() / 1000;
      
      if (!PLOTTER_MODE) {
        Serial.println("💓 Sending Heartbeat...");
      }
      
      mqttPublish(HEARTBEAT_TOPIC, hbDoc);
    }
  }

  // --- Task 2: High-Speed Sensor Polling (Every 8ms) ---
  if (millis() - lastSensorReadTime >= SENSOR_INTERVAL) {
    lastSensorReadTime = millis();

    // Grab the newest sample
    int32_t latestSample = 0;
    max30003.readEcgSample(latestSample); 
    
    // Add to our package buffer
    bufferToSend[counter] = latestSample;
    counter++;

    // When our buffer is full, process the payload
    if (counter >= PACKAGE_SIZE) {
      counter = 0; // Reset for the next batch

      // 1. Send to Serial Plotter if mode is active
      if (PLOTTER_MODE) {
        for (int i = 0; i < PACKAGE_SIZE; i++) {
          Serial.println(bufferToSend[i]); 
        }
      }

      // 2. Package and send over MQTT (Non-blocking)
      if (current_state == STATE_CONNECTED) {
        JsonDocument doc;
        JsonArray data = doc["data"].to<JsonArray>();
        
        for (int i = 0; i < PACKAGE_SIZE; i++) {
          data.add(bufferToSend[i]);
        }
        doc["pat"] = "1";
        
        if (!PLOTTER_MODE) {
          String output;
          serializeJson(doc, output);
          Serial.println("📤 Published: " + output);
        }
        
        mqttPublish(AWS_IOT_TOPIC, doc);
      }
    }
  }

  // Allow the ESP-IDF MQTT FreeRTOS task to process background network traffic
  vTaskDelay(pdMS_TO_TICKS(1)); 
}