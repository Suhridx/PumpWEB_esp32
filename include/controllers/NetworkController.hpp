#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <vector>

// Struct to hold backup network credentials
struct WiFiCredential {
    String ssid;
    String password;
};

class NetworkController {
public:
    NetworkController();

    void begin();

    // Saves a new network to LittleFS (via FileManager) and adds to active memory
    bool save_config(const char* ssid, const char* password);

    // Instantly attempt connection to a specific network
    void connect_to(const char* ssid, const char* password);

    String scan_networks();
    bool is_connected();
    void disconnect();

private:
    enum class WiFiState {
        DISCONNECTED,
        CONNECTING,
        CONNECTED,
        SCANNING_FOR_KNOWN,
        WAITING_FOR_NETWORK
    };

    WiFiState current_state = WiFiState::DISCONNECTED;

    String current_ssid = "";
    String current_password = "";

    uint8_t retry_count = 0;
    std::vector<WiFiCredential> saved_networks;

    TaskHandle_t wifi_task_handle = NULL;

    bool load_config();
    static void wifi_background_task(void* parameter);
};