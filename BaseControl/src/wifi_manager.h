#pragma once

#include <WiFi.h>
#include "esp_wifi.h"

void scanWiFiNetworks() {
  Serial.println("\n--- Scanning WiFi Networks ---");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(200);

  int n = WiFi.scanNetworks(false, true);

  if (n == 0) {
    Serial.println("No networks found.");
    return;
  }

  Serial.printf("Found %d networks:\n\n", n);

  for (int i = 0; i < n; ++i) {
    String enc;
    switch (WiFi.encryptionType(i)) {
      case WIFI_AUTH_OPEN:            enc = "OPEN"; break;
      case WIFI_AUTH_WEP:             enc = "WEP"; break;
      case WIFI_AUTH_WPA_PSK:         enc = "WPA_PSK"; break;
      case WIFI_AUTH_WPA2_PSK:        enc = "WPA2_PSK"; break;
      case WIFI_AUTH_WPA_WPA2_PSK:    enc = "WPA_WPA2_PSK"; break;
      case WIFI_AUTH_WPA2_ENTERPRISE: enc = "WPA2_ENTERPRISE"; break;
      case WIFI_AUTH_WPA3_PSK:        enc = "WPA3_PSK"; break;
      case WIFI_AUTH_WPA2_WPA3_PSK:   enc = "WPA2_WPA3_PSK"; break;
      default:                        enc = "UNKNOWN";
    }
    Serial.printf(
      "%2d) SSID: %-25s  RSSI: %4d dBm  CH: %2d  ENC: %s  BSSID: %s\n",
      i + 1,
      WiFi.SSID(i).c_str(),
      WiFi.RSSI(i),
      WiFi.channel(i),
      enc.c_str(),
      WiFi.BSSIDstr(i).c_str()
    );
    delay(10);
  }

  Serial.println("\n--- Scan Complete ---\n");
}

bool beginWithBestMatch(const char* targetSSID, const char* pass) {
  Serial.println("Scanning for target to lock BSSID+channel...");
  int n = WiFi.scanNetworks(false, true);
  if (n <= 0) return false;

  int best = -1, bestRssi = -999;
  for (int i = 0; i < n; i++) {
    if (WiFi.SSID(i) == targetSSID) {
      int rssi = WiFi.RSSI(i);
      if (rssi > bestRssi) { bestRssi = rssi; best = i; }
    }
  }
  if (best < 0) return false;

  int ch = WiFi.channel(best);
  uint8_t* bssid = WiFi.BSSID(best);
  Serial.printf("Locking to SSID=%s CH=%d BSSID=%s RSSI=%d\n",
                targetSSID, ch, WiFi.BSSIDstr(best).c_str(), WiFi.RSSI(best));
  WiFi.begin(targetSSID, pass, ch, bssid, true);
  return true;
}

void WiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    Serial.printf("DISCONNECTED. reason=%d\n", info.wifi_sta_disconnected.reason);
  } else if (event == ARDUINO_EVENT_WIFI_STA_CONNECTED) {
    Serial.println("CONNECTED to AP");
  } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
    Serial.print("GOT IP: ");
    Serial.println(WiFi.localIP());
  }
}

bool connectWpa2Compat(const char* ssid, const char* pass) {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.persistent(false);

  WiFi.begin(ssid, pass);
  delay(200);
  WiFi.disconnect(true);
  delay(200);

  wifi_config_t cfg = {};
  strlcpy((char*)cfg.sta.ssid,     ssid, sizeof(cfg.sta.ssid));
  strlcpy((char*)cfg.sta.password, pass, sizeof(cfg.sta.password));
  cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
  cfg.sta.pmf_cfg.capable  = true;
  cfg.sta.pmf_cfg.required = false;

  esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &cfg);
  if (err != ESP_OK) {
    Serial.printf("esp_wifi_set_config failed: %d\n", (int)err);
    return false;
  }

  err = esp_wifi_connect();
  if (err != ESP_OK) {
    Serial.printf("esp_wifi_connect failed: %d\n", (int)err);
    return false;
  }
  return true;
}
