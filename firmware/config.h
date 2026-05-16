/*
 * config.h — User configuration
 * IoT Predictive Maintenance | University of Peradeniya
 * Dept: Electrical & Electronic Engineering
 * Team: E22215, E22028, E22177, E22287, E22199
 *
 * EDIT THIS FILE with your Wi-Fi and MQTT credentials before flashing.
 */

#pragma once

// ── Wi-Fi ──────────────────────────────────────────────────────────
#define WIFI_SSID       "YOUR_WIFI_SSID"
#define WIFI_PASSWORD   "YOUR_WIFI_PASSWORD"
#define WIFI_TIMEOUT_MS  10000

// ── MQTT Broker ────────────────────────────────────────────────────
// Run Mosquitto locally: sudo apt install mosquitto mosquitto-clients
// Then: mosquitto -v
#define MQTT_BROKER     "192.168.1.100"   // Change to your laptop IP
#define MQTT_PORT       1883
#define MQTT_CLIENT_ID  "esp32_pdm_node_01"
#define MQTT_USER       ""                // Leave blank if no auth
#define MQTT_PASS       ""
#define MQTT_TOPIC_VIBRATION  "motor/vibration"
#define MQTT_TOPIC_STATUS     "motor/status"

// ── Pin Definitions ────────────────────────────────────────────────
#define PIN_SDA         21
#define PIN_SCL         22
#define PIN_LED_GREEN   25
#define PIN_LED_YELLOW  26
#define PIN_LED_RED     27
#define PIN_BUZZER      32
#define PIN_POT         34    // ADC1 — potentiometer wiper
#define PIN_BTN1        35    // Calibrate baseline
#define PIN_BTN2        36    // Reset alert

// ── Signal Processing ──────────────────────────────────────────────
#define WINDOW_SIZE         256       // Samples per RMS window
#define SAMPLE_RATE_HZ      1000      // 1 kHz
#define DEFAULT_THRESHOLD   0.20f     // g — initial fault threshold
#define THRESHOLD_MIN       0.10f     // g — pot minimum
#define THRESHOLD_MAX       2.00f     // g — pot maximum
#define WARNING_FRACTION    0.75f     // fraction of threshold for yellow LED

// ── MPU6050 ────────────────────────────────────────────────────────
#define MPU_I2C_ADDR    0x68          // AD0 pin tied to GND
#define MPU_ACCEL_RANGE 2             // 0=±2g 1=±4g 2=±8g 3=±16g
#define MPU_DLPF_BW     2             // DLPF config 2 = 94 Hz bandwidth
