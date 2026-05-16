/*
 * mqtt_client.h — Wi-Fi connection and MQTT publish helpers
 * IoT Predictive Maintenance | University of Peradeniya
 * Dept: Electrical & Electronic Engineering
 * Team: E22215, E22028, E22177, E22287, E22199
 */

#pragma once
#include <WiFi.h>
#include <PubSubClient.h>
#include "config.h"

extern PubSubClient mqttClient;

// ── Connect to Wi-Fi ───────────────────────────────────────────────
static void wifi_connect(const char *ssid, const char *password) {
  Serial.printf("[WiFi] Connecting to %s", ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - t0 > WIFI_TIMEOUT_MS) {
      Serial.println("\n[WiFi] Timeout. Check SSID/password in config.h");
      return;
    }
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\n[WiFi] Connected. IP: %s\n", WiFi.localIP().toString().c_str());
}

// ── Connect / reconnect to MQTT broker ────────────────────────────
static void mqtt_connect(PubSubClient *client, const char *clientId,
                         const char *user, const char *pass) {
  int retries = 0;
  while (!client->connected() && retries < 5) {
    Serial.printf("[MQTT] Connecting to broker %s:%d ...\n", MQTT_BROKER, MQTT_PORT);
    bool ok = (strlen(user) > 0)
              ? client->connect(clientId, user, pass)
              : client->connect(clientId);
    if (ok) {
      Serial.println("[MQTT] Connected.");
      // Publish online status
      client->publish(MQTT_TOPIC_STATUS,
        "{\"status\":\"online\",\"node\":\"esp32_pdm_01\","
        "\"dept\":\"EEE\",\"uni\":\"UoP\"}", true);
    } else {
      Serial.printf("[MQTT] Failed (rc=%d). Retry in 3s...\n", client->state());
      delay(3000);
      retries++;
    }
  }
}

// ── Publish payload to topic ───────────────────────────────────────
static void mqtt_publish(const char *topic, const char *payload) {
  if (!mqttClient.connected()) {
    mqtt_connect(&mqttClient, MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS);
  }
  mqttClient.publish(topic, payload);
}
