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

    /**
     * @brief Moves the state machine and mirrors the result into hw_status.
     *
     * Every transition goes through here so hw_status.bits.wifi_connected can never
     * drift away from what the state machine believes the link is doing. CONNECTED is
     * the only state that counts as an up link, every other one clears the bit.
     *
     * @param new_state The state to move into.
     */
    void set_state(WiFiState new_state);
};