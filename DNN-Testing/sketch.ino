#include "creds.h"
#include "MQTTManager.h"
#include <ArduinoJson.h>
#include <Armic_inferencing.h>

// --- Configuration ---
#define DATA_SIZE 900
const char* PUBLISH_TOPIC = "/armic/imu-1/output";
const char* SUBSCRIBE_TOPIC = "/armic/imu-1/input";

String unique_client_id;

// --- Memory & Buffers ---
float features[DATA_SIZE];
float featuresMemory[DATA_SIZE];
volatile int counter = 0;
volatile bool flag = false;

// --- Prototypes ---
int raw_feature_get_data(size_t offset, size_t length, float* out_ptr);

// --- Incoming MQTT Message Handler ---
void onMessageReceived(String topic, String payload) {
  if (topic == SUBSCRIBE_TOPIC) {
    Serial.println("Message arrived [" + topic + "]: " + payload);
    // Add any logic here you want to trigger when you receive a message
  }
}

// --- Subscription Handler ---
void onMqttConnected() {
  mqttSubscribe(SUBSCRIBE_TOPIC);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Armic Multi-Thread ML over Secure WebSockets");

  // 1. Setup Unique ID
  uint64_t chipid = ESP.getEfuseMac();
  uint32_t lower32 = (uint32_t)chipid;
  uint16_t upper16 = (uint16_t)(chipid >> 32);
  char idBuff[30];
  snprintf(idBuff, sizeof(idBuff), "%s%04X%08X", client_base_name, upper16, lower32);
  unique_client_id = String(idBuff);

  // 2. Configure MQTT Callbacks & Connect
  setMqttCallback(onMessageReceived);
  setMqttConnectCallback(onMqttConnected);
  
  wifiConnect(ssid, password);
  mqttConnect(host, mqtt_user, mqtt_pass, unique_client_id.c_str());

  // 3. Start the Hardware Collection Task on Core 1
  xTaskCreatePinnedToCore(
    TaskAccelerator,    // Function to implement the task
    "TaskAccelerator",  // Name of the task
    4096,               // Stack size in words
    NULL,               // Task input parameter
    1,                  // Priority of the task
    NULL,               // Task handle
    1                   // Core where the task should run
  );
}

void loop() {
  // The main loop handles the heavy ML processing whenever data is ready
  if (flag) {
    ei_printf("Edge Impulse standalone inferencing (Arduino)\n");
    
    ei_impulse_result_t result = { 0 };
    signal_t features_signal;
    features_signal.total_length = sizeof(features) / sizeof(features[0]);
    features_signal.get_data = &raw_feature_get_data;
    
    // Run the classifier
    EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false);
    if (res != EI_IMPULSE_OK) {
      ei_printf("ERR: Failed to run classifier (%d)\n", res);
      flag = false; // Reset flag to try again next time
      return;
    }
    
    // Find the highest confidence prediction
    float max_v = 0;
    int max_i = 0;
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
      if (result.classification[i].value > max_v) {
        max_v = result.classification[i].value;
        max_i = i;
      }
      ei_printf("  %s: %.5f\r\n", ei_classifier_inferencing_categories[i], result.classification[i].value);
    }
    
    // Package and send the best result via MQTT if connected
    if (current_state == STATE_CONNECTED) {
      JsonDocument doc;
      
      // Creates a clean JSON output like: {"normal": 0.98}
      doc[ei_classifier_inferencing_categories[max_i]] = max_v;
      
      mqttPublish(PUBLISH_TOPIC, doc);
    }

    flag = false;
  }

  // Yield to FreeRTOS background tasks (like MQTT!)
  vTaskDelay(pdMS_TO_TICKS(10));
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskAccelerator(void* pvParameters) {
  // This FreeRTOS function ensures the loop executes exactly every 5ms
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(5); 

  for (;;) {
    // Wait until exactly 5ms have passed since the last cycle
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Only collect if the main loop has finished processing the last batch
    if (!flag) {
      featuresMemory[counter++] = (float)analogRead(A0);
      featuresMemory[counter++] = (float)analogRead(A1);
      featuresMemory[counter++] = (float)analogRead(A2);

      if (counter >= DATA_SIZE) {
        // Safe array copy using memcpy for speed
        memcpy(features, featuresMemory, sizeof(featuresMemory));
        counter = 0;
        flag = true;
      }
    }
  }
}

// Functions
int raw_feature_get_data(size_t offset, size_t length, float* out_ptr) {
  memcpy(out_ptr, features + offset, length * sizeof(float));
  return 0;
}
