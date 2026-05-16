/*
 * =====================================================================
 * IoT-Based Predictive Maintenance System
 * Using Vibration Analysis on Electric Motors
 * ---------------------------------------------------------------------
 * Module  : EN2160 — Electronic Design Realization
 * Dept    : Electrical & Electronic Engineering
 * Uni     : University of Peradeniya, Sri Lanka
 * Team    : E22215, E22028, E22177, E22287, E22199
 * Date    : May 2026
 * =====================================================================
 *
 * Description:
 *   Main entry point. Initialises hardware peripherals, connects to
 *   Wi-Fi and MQTT broker, then launches two FreeRTOS tasks:
 *     Task 1 (Core 0) — MPU6050 data acquisition at 1 kHz
 *     Task 2 (Core 1) — RMS computation, threshold check, MQTT publish
 *
 * Pin Assignments (ESP32 Dev Kit v1):
 *   GPIO21  SDA   -> MPU6050 SDA
 *   GPIO22  SCL   -> MPU6050 SCL
 *   GPIO25        -> Green LED  (Normal)
 *   GPIO26        -> Yellow LED (Warning)
 *   GPIO27        -> Red LED    (Fault)
 *   GPIO32        -> Buzzer (via 2N2222 transistor)
 *   GPIO34  ADC   -> Potentiometer wiper (threshold adjust)
 *   GPIO35        -> Push button 1 (calibrate baseline)
 *   GPIO36        -> Push button 2 (reset alert)
 * =====================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "config.h"
#include "mpu6050_handler.h"
#include "mqtt_client.h"

// ── FreeRTOS handles ──────────────────────────────────────────────
SemaphoreHandle_t xDataReady;
SemaphoreHandle_t xSerialMutex;

// ── Shared vibration buffer ───────────────────────────────────────
volatile float ax_buf[WINDOW_SIZE];
volatile float ay_buf[WINDOW_SIZE];
volatile float az_buf[WINDOW_SIZE];
volatile uint16_t buf_index = 0;

// ── Global state ──────────────────────────────────────────────────
float threshold_g  = DEFAULT_THRESHOLD;
float baseline_ax  = 0.0f;
float baseline_ay  = 0.0f;
float baseline_az  = 0.0f;
bool  calibrated   = false;
bool  fault_active = false;
unsigned long uptime_s = 0;

// ── Timer ─────────────────────────────────────────────────────────
hw_timer_t   *sampleTimer = NULL;
portMUX_TYPE  timerMux    = portMUX_INITIALIZER_UNLOCKED;

// ── Network clients ───────────────────────────────────────────────
WiFiClient   espClient;
PubSubClient mqttClient(espClient);

// ─────────────────────────────────────────────────────────────────
// ISR — fires at 1 kHz, reads one MPU6050 sample into buffer
// ─────────────────────────────────────────────────────────────────
void IRAM_ATTR onSampleTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  if (buf_index < WINDOW_SIZE) {
    float ax, ay, az;
    mpu_read_accel(&ax, &ay, &az);
    ax_buf[buf_index] = ax;
    ay_buf[buf_index] = ay;
    az_buf[buf_index] = az;
    buf_index++;
    if (buf_index >= WINDOW_SIZE) {
      BaseType_t xHPTW = pdFALSE;
      xSemaphoreGiveFromISR(xDataReady, &xHPTW);
      buf_index = 0;
      portYIELD_FROM_ISR(xHPTW);
    }
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

// ─────────────────────────────────────────────────────────────────
// LED helper
// ─────────────────────────────────────────────────────────────────
void set_leds(bool red, bool yellow, bool green) {
  digitalWrite(PIN_LED_RED,    red    ? HIGH : LOW);
  digitalWrite(PIN_LED_YELLOW, yellow ? HIGH : LOW);
  digitalWrite(PIN_LED_GREEN,  green  ? HIGH : LOW);
}

// ─────────────────────────────────────────────────────────────────
// Calibrate baseline over 5 windows
// ─────────────────────────────────────────────────────────────────
void calibrate_baseline() {
  Serial.println("[CAL] Starting baseline calibration...");
  float sum_ax = 0, sum_ay = 0, sum_az = 0;
  int samples = 5;
  for (int s = 0; s < samples; s++) {
    if (xSemaphoreTake(xDataReady, pdMS_TO_TICKS(2000)) == pdTRUE) {
      double sa = 0, sb = 0, sc = 0;
      for (int i = 0; i < WINDOW_SIZE; i++) {
        sa += (double)ax_buf[i] * ax_buf[i];
        sb += (double)ay_buf[i] * ay_buf[i];
        sc += (double)az_buf[i] * az_buf[i];
      }
      sum_ax += sqrtf(sa / WINDOW_SIZE);
      sum_ay += sqrtf(sb / WINDOW_SIZE);
      sum_az += sqrtf(sc / WINDOW_SIZE);
    }
  }
  baseline_ax = sum_ax / samples;
  baseline_ay = sum_ay / samples;
  baseline_az = sum_az / samples;
  threshold_g = 1.5f * fmaxf(baseline_ax, fmaxf(baseline_ay, baseline_az));
  calibrated  = true;
  Serial.printf("[CAL] Baseline: AX=%.3f AY=%.3f AZ=%.3f | Threshold=%.3f g\n",
    baseline_ax, baseline_ay, baseline_az, threshold_g);
}

// ─────────────────────────────────────────────────────────────────
// Task 1: Button/Pot monitoring — Core 0
// ─────────────────────────────────────────────────────────────────
void taskAcquisition(void *pvParameters) {
  for (;;) {
    int pot_raw = analogRead(PIN_POT);
    threshold_g = 0.10f + ((float)pot_raw / 4095.0f) * 1.90f;

    if (digitalRead(PIN_BTN1) == LOW) {
      delay(50);
      if (digitalRead(PIN_BTN1) == LOW) {
        calibrate_baseline();
        while (digitalRead(PIN_BTN1) == LOW) delay(10);
      }
    }
    if (digitalRead(PIN_BTN2) == LOW) {
      delay(50);
      if (digitalRead(PIN_BTN2) == LOW) {
        fault_active = false;
        set_leds(false, false, true);
        noTone(PIN_BUZZER);
        while (digitalRead(PIN_BTN2) == LOW) delay(10);
      }
    }
    uptime_s = millis() / 1000;
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ─────────────────────────────────────────────────────────────────
// Task 2: RMS computation + MQTT publish — Core 1
// ─────────────────────────────────────────────────────────────────
void taskProcessing(void *pvParameters) {
  for (;;) {
    if (xSemaphoreTake(xDataReady, portMAX_DELAY) == pdTRUE) {

      // Compute RMS
      double sum_ax = 0, sum_ay = 0, sum_az = 0;
      for (int i = 0; i < WINDOW_SIZE; i++) {
        sum_ax += (double)ax_buf[i] * ax_buf[i];
        sum_ay += (double)ay_buf[i] * ay_buf[i];
        sum_az += (double)az_buf[i] * az_buf[i];
      }
      float rms_ax = sqrtf(sum_ax / WINDOW_SIZE);
      float rms_ay = sqrtf(sum_ay / WINDOW_SIZE);
      float rms_az = sqrtf(sum_az / WINDOW_SIZE);

      // Subtract calibrated baseline
      if (calibrated) {
        rms_ax = fmaxf(0.0f, rms_ax - baseline_ax);
        rms_ay = fmaxf(0.0f, rms_ay - baseline_ay);
        rms_az = fmaxf(0.0f, rms_az - baseline_az);
      }

      // Fault check
      float max_rms = fmaxf(rms_ax, fmaxf(rms_ay, rms_az));
      fault_active  = (max_rms >= threshold_g);

      // Indicators
      if (fault_active) {
        set_leds(true, false, false);
        tone(PIN_BUZZER, 2000, 200);
      } else if (max_rms >= threshold_g * 0.75f) {
        set_leds(false, true, false);
        noTone(PIN_BUZZER);
      } else {
        set_leds(false, false, true);
        noTone(PIN_BUZZER);
      }

      // Build JSON
      StaticJsonDocument<256> doc;
      doc["ax_rms"]    = roundf(rms_ax * 1000.0f) / 1000.0f;
      doc["ay_rms"]    = roundf(rms_ay * 1000.0f) / 1000.0f;
      doc["az_rms"]    = roundf(rms_az * 1000.0f) / 1000.0f;
      doc["threshold"] = roundf(threshold_g * 1000.0f) / 1000.0f;
      doc["fault"]     = fault_active;
      doc["uptime_s"]  = uptime_s;
      doc["ts"]        = millis();

      char payload[256];
      serializeJson(doc, payload);
      mqtt_publish(MQTT_TOPIC_VIBRATION, payload);

      // Serial debug
      if (xSemaphoreTake(xSerialMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        Serial.printf("[%lu] AX:%.3f AY:%.3f AZ:%.3f g | THR:%.2f | %s\n",
          millis(), rms_ax, rms_ay, rms_az, threshold_g,
          fault_active ? "FAULT" : "OK");
        xSemaphoreGive(xSerialMutex);
      }

      vTaskDelay(pdMS_TO_TICKS(250));
    }
  }
}

// ─────────────────────────────────────────────────────────────────
// setup()
// ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== IoT Predictive Maintenance System ===");
  Serial.println("    Dept : Electrical & Electronic Engineering");
  Serial.println("    Uni  : University of Peradeniya, Sri Lanka");
  Serial.println("    Team : E22215 E22028 E22177 E22287 E22199\n");

  pinMode(PIN_LED_RED,    OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);
  pinMode(PIN_LED_GREEN,  OUTPUT);
  pinMode(PIN_BUZZER,     OUTPUT);
  pinMode(PIN_BTN1,       INPUT_PULLUP);
  pinMode(PIN_BTN2,       INPUT_PULLUP);
  analogReadResolution(12);

  for (int i = 0; i < 3; i++) {
    set_leds(true,true,true);   delay(150);
    set_leds(false,false,false);delay(150);
  }
  set_leds(false, false, true);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  if (!mpu_init()) {
    Serial.println("[ERROR] MPU6050 not found. Check wiring.");
    while (true) { set_leds(true,false,false); delay(300); set_leds(false,false,false); delay(300); }
  }
  Serial.println("[OK] MPU6050 ready.");

  wifi_connect(WIFI_SSID, WIFI_PASSWORD);
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setKeepAlive(30);
  mqtt_connect(&mqttClient, MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS);

  xDataReady   = xSemaphoreCreateBinary();
  xSerialMutex = xSemaphoreCreateMutex();

  sampleTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(sampleTimer, &onSampleTimer, true);
  timerAlarmWrite(sampleTimer, 1000, true);   // 1000 µs = 1 kHz
  timerAlarmEnable(sampleTimer);
  Serial.println("[OK] 1 kHz sampling timer started.");

  xTaskCreatePinnedToCore(taskAcquisition, "Acquisition", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskProcessing,  "Processing",  8192, NULL, 2, NULL, 1);
  Serial.println("[OK] FreeRTOS tasks launched. System running.\n");
}

// loop() — only handles MQTT keepalive; tasks run everything else
void loop() {
  if (!mqttClient.connected()) {
    mqtt_connect(&mqttClient, MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS);
  }
  mqttClient.loop();
  delay(10);
}
