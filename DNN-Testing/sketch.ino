/*
 * Armic Edge Impulse inference on M5Stack Core2, with MQTT publishing
 * of the classification result.
 *
 * The deployed Edge Impulse model (Armic) is a sensor-fusion classifier
 * over 6 axes (accX + accY + accZ + gyrX + gyrY + gyrZ) sampled at
 * 50 Hz. It outputs 4 classes: Baseline / Bicepcurl / Elbowflexion /
 * Lateralraise. This sketch streams samples from the onboard MPU6886
 * IMU into the Armic inference library, prints the predictions, and
 * publishes a JSON payload with the top label and per-class scores
 * to the same broker used by the MotionCapture project.
 */

#include <Arduino.h>
#include <M5Core2.h>
#include <WiFi.h>
#include "mqtt_client.h"
#include <Armic_inferencing.h>

/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2     9.80665f
#define MAX_ACCEPTED_RANGE   2.0f    // Model was trained on accel clamped to +-2g
#define EI_SENSOR_AIXS_COUNT 6

/* MQTT topic on the local broker. MotionCapture uses armic/kinematics/stream
 * for raw samples; this device publishes classification results on a
 * separate topic so a downstream consumer can subscribe to either.
 */
#define MQTT_TOPIC_RESULT    "armic/inference/result"

/* --- Network configuration (matches MotionCapture) --------------------- */
static const char* WIFI_SSID     = "";
static const char* WIFI_PASSWORD = "";
static const char* MQTT_BROKER   = ""; 
static const char* MQTT_CLIENT_ID = "ArmicInferenceNode";

/* --- MQTT state --------------------------------------------------------- */
static esp_mqtt_client_handle_t mqttClient;
static volatile bool mqtt_connected = false;

/* --- 6-Axis Sensor Variables -------------------------------------------- */
static float data[EI_SENSOR_AIXS_COUNT]; // [accX, accY, accZ, gyrX, gyrY, gyrZ]
static const bool debug_nn = false;

/* --- Publish payload buffer (pre-allocated, no heap in hot path) -------- */
static char mqttPayload[512];

/* Forward declarations --------------------------------------------------- */
static bool read_imu(float *out);
static void run_inference(void);
static void setup_wifi(void);
static void mqtt_event_handler(void* handler_args, esp_event_base_t base,
                               int32_t event_id, void* event_data);
static void publish_inference_result(const ei_impulse_result_t *result);

/* ------------------------------------------------------------------------ */
void setup() {
    M5.begin();
    M5.Lcd.setTextSize(2);

    if (M5.IMU.Init() != 0) {
        M5.Lcd.setCursor(10, 10);
        M5.Lcd.setTextColor(RED, BLACK);
        M5.Lcd.println("IMU INIT FAIL");
        while (1) { delay(1000); }
    }

    // The Edge Impulse "Armic" model was trained on accel data clamped to
    // +-2 g, so configure the MPU6886 in that range for best fidelity.
    M5.IMU.SetAccelFsr(MPU6886::AFS_2G);

    Serial.begin(115200);
    Serial.println();
    Serial.println("=== Armic Edge Impulse + MQTT ===");

    ei_printf("Inferencing settings:\n");
    ei_printf("\tInterval: %.2f ms.\n", (float)EI_CLASSIFIER_INTERVAL_MS);
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
    ei_printf("\tNo. of classes: %d\n",
              sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0]));

    // Throw away the first few samples so the IMU settles.
    for (int i = 0; i < 10; i++) {
        read_imu(data);
        delay(10);
    }

    setup_wifi();

    // Plain TCP, no TLS — broker is on the local LAN at 192.168.1.168:1883.
    char uri[64];
    snprintf(uri, sizeof(uri), "mqtt://%s:1883", MQTT_BROKER);
    Serial.printf("MQTT: starting client -> %s\n", uri);

    esp_mqtt_client_config_t mqtt_cfg = {};
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    mqtt_cfg.broker.address.uri    = uri;
    mqtt_cfg.credentials.client_id = MQTT_CLIENT_ID;
    mqtt_cfg.session.keepalive     = 30;
    mqtt_cfg.buffer.size           = sizeof(mqttPayload);
#else
    mqtt_cfg.uri         = uri;
    mqtt_cfg.client_id   = MQTT_CLIENT_ID;
    mqtt_cfg.keepalive   = 30;
    mqtt_cfg.buffer_size = sizeof(mqttPayload);
#endif

    mqttClient = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqttClient, MQTT_EVENT_ANY, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqttClient);
}

/* ------------------------------------------------------------------------ */
void loop() {
    // Inference runs whether or not MQTT is up. The publish step no-ops
    // while the client is disconnected; ESP-IDF MQTT auto-reconnects in
    // its own task.
    run_inference();
}

/**
 * Read a fresh 6-axis sample from the MPU6886.
 *
 * Accelerometer: reported in g by the MPU6886 -> convert to m/s^2 for the
 * Edge Impulse model. Saturate to +-2g before scaling.
 *
 * Gyroscope: already reported in degrees/s, which matches the model.
 */
static bool read_imu(float *out) {
    float ax, ay, az;
    float gx, gy, gz;

    M5.IMU.getAccelData(&ax, &ay, &az);
    M5.IMU.getGyroData(&gx, &gy, &gz);

    // Belt-and-braces clamp: the MPU6886 is already in +-2g mode.
    if (ax >  MAX_ACCEPTED_RANGE) ax =  MAX_ACCEPTED_RANGE;
    if (ax < -MAX_ACCEPTED_RANGE) ax = -MAX_ACCEPTED_RANGE;
    if (ay >  MAX_ACCEPTED_RANGE) ay =  MAX_ACCEPTED_RANGE;
    if (ay < -MAX_ACCEPTED_RANGE) ay = -MAX_ACCEPTED_RANGE;
    if (az >  MAX_ACCEPTED_RANGE) az =  MAX_ACCEPTED_RANGE;
    if (az < -MAX_ACCEPTED_RANGE) az = -MAX_ACCEPTED_RANGE;

    out[0] = ax * CONVERT_G_TO_MS2; // accX (m/s^2)
    out[1] = ay * CONVERT_G_TO_MS2; // accY (m/s^2)
    out[2] = az * CONVERT_G_TO_MS2; // accZ (m/s^2)
    out[3] = gx;                    // gyrX (deg/s)
    out[4] = gy;                    // gyrY (deg/s)
    out[5] = gz;                    // gyrZ (deg/s)
    return true;
}

/**
 * Collect EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE raw samples at
 * EI_CLASSIFIER_INTERVAL_MS intervals and run the classifier.
 */
static void run_inference(void) {
    float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

    for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += EI_SENSOR_AIXS_COUNT) {
        int64_t next_tick = (int64_t)micros() + ((int64_t)(1000.0f / EI_CLASSIFIER_FREQUENCY) * 1000);

        if (!read_imu(data)) {
            ei_printf("ERR: failed to read IMU, skipping inference\n");
            return;
        }
        for (int i = 0; i < EI_SENSOR_AIXS_COUNT; i++) {
            buffer[ix + i] = data[i];
        }

        int64_t wait_time = next_tick - (int64_t)micros();
        if (wait_time > 0) {
            delayMicroseconds(wait_time);
        }
    }

    signal_t signal;
    int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
        ei_printf("ERR: signal_from_buffer failed (%d)\n", err);
        return;
    }

    ei_impulse_result_t result = { 0 };
    err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: run_classifier failed (%d)\n", err);
        return;
    }

    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.):\n",
              result.timing.dsp, result.timing.classification, result.timing.anomaly);
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: ", result.classification[ix].label);
        ei_printf_float(result.classification[ix].value);
        ei_printf("\n");
    }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    anomaly score: ");
    ei_printf_float(result.anomaly);
    ei_printf("\n");
#endif

    publish_inference_result(&result);
}

/* ------------------------------------------------------------------------ */
/* WiFi + MQTT                                                              */
/* ------------------------------------------------------------------------ */
static void setup_wifi(void) {
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.print("Connecting WiFi...");

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // 30s hard timeout — without this, a wrong password hangs the boot
    // silently and you have no idea why MQTT never comes up.
    const unsigned long WIFI_TIMEOUT_MS = 30000;
    unsigned long start_ms = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - start_ms > WIFI_TIMEOUT_MS) {
            M5.Lcd.println("\nWiFi FAILED (30s)");
            return;
        }
        delay(500);
        M5.Lcd.print(".");
    }
    M5.Lcd.println(" OK");
}

static void mqtt_event_handler(void* handler_args, esp_event_base_t base,
                               int32_t event_id, void* event_data) {
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_ERROR: {
            M5.Lcd.fillScreen(BLACK);
            M5.Lcd.setCursor(10, 10);
            M5.Lcd.setTextColor(RED, BLACK);
            M5.Lcd.println("MQTT ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                M5.Lcd.setCursor(10, 30);
                M5.Lcd.printf("TCP sock_errno=%d", event->error_handle->esp_transport_sock_errno);
            } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
                M5.Lcd.setCursor(10, 30);
                M5.Lcd.printf("Refused code=%d", (int)event->error_handle->connect_return_code);
            }
            break;
        }
        case MQTT_EVENT_CONNECTED:
            mqtt_connected = true;
            M5.Lcd.fillScreen(BLACK);
            M5.Lcd.setCursor(10, 10);
            M5.Lcd.setTextColor(GREEN, BLACK);
            M5.Lcd.println("INFERENCING -> MQTT");
            break;
        case MQTT_EVENT_DISCONNECTED:
            mqtt_connected = false;
            M5.Lcd.fillScreen(BLACK);
            M5.Lcd.setCursor(10, 10);
            M5.Lcd.setTextColor(YELLOW, BLACK);
            M5.Lcd.println("MQTT DISCONNECTED");
            M5.Lcd.setCursor(10, 30);
            M5.Lcd.print("Reconnecting...");
            break;
        default:
            break;
    }
}

/**
 * Format the classification result as a small JSON document and publish
 * it on MQTT_TOPIC_RESULT. No-op while the client is disconnected.
 *
 * Shape:
 *   {"label":"Bicepcurl","score":0.85,"scores":{...},"timing":{...}}
 */
static void publish_inference_result(const ei_impulse_result_t *result) {
    if (!mqtt_connected) {
        return;
    }

    // Find the top class. EI doesn't guarantee order, so scan.
    size_t top_ix = 0;
    float top_val = result->classification[0].value;
    for (size_t ix = 1; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        if (result->classification[ix].value > top_val) {
            top_val = result->classification[ix].value;
            top_ix = ix;
        }
    }
    const char *top_label = result->classification[top_ix].label;

    // Build the inner scores object first, then prepend the wrapper.
    char scores_buf[384] = {0};
    int  scores_len = 0;
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        int n = snprintf(scores_buf + scores_len,
                         sizeof(scores_buf) - scores_len,
                         "%s\"%s\":%.4f",
                         ix == 0 ? "" : ",",
                         result->classification[ix].label,
                         result->classification[ix].value);
        if (n <= 0 || (size_t)n >= sizeof(scores_buf) - scores_len) {
            return; // would-be-truncated; skip publish this round
        }
        scores_len += n;
    }

    int len = snprintf(mqttPayload, sizeof(mqttPayload),
                       "{\"label\":\"%s\",\"score\":%.4f,"
                       "\"scores\":{%s},"
                       "\"timing\":{\"dsp\":%d,\"classification\":%d,\"anomaly\":%d}"
#if EI_CLASSIFIER_HAS_ANOMALY == 1
                       ",\"anomaly\":%.4f"
#endif
                       "}",
                       top_label, top_val,
                       scores_buf,
                       result->timing.dsp,
                       result->timing.classification,
                       result->timing.anomaly
#if EI_CLASSIFIER_HAS_ANOMALY == 1
                       , result->anomaly
#endif
                       );

    if (len <= 0 || (size_t)len >= sizeof(mqttPayload)) {
        Serial.printf("WARN: MQTT payload truncated (len=%d, buf=%u)\n", len, (unsigned)sizeof(mqttPayload));
        return;
    }

    int msg_id = esp_mqtt_client_publish(mqttClient, MQTT_TOPIC_RESULT,
                                         mqttPayload, len, 0, 0);
    if (msg_id < 0) {
        Serial.println("WARN: MQTT publish returned error");
    } else {
        Serial.printf("MQTT -> %s (%d bytes): %s\n", MQTT_TOPIC_RESULT, len, mqttPayload);
    }
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_FUSION
#error "Invalid model for current sensor. This sketch expects the FUSION model."
#endif