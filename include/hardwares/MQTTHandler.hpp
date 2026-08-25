#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <map>
#include <functional>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ==============================================================================
// 2. Explicit MQTT Connection Status Variables
// ==============================================================================
enum class MQTTState {
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    CONNECTION_LOST,
    AUTH_FAILED
};

// ==============================================================================
// 3. Explicit MQTT Connection Constants (Populated from LittleFS)
// ==============================================================================
struct MQTTConfig {
    String broker_ip;
    uint16_t port;
    String username;
    String password;
    String client_id;
};

// Callback signature: passes raw topic and payload directly to main.cpp
typedef std::function<void(String topic, String payload)> MQTTMessageCallback;
typedef std::function<void(String payload)> TopicRoutedCallback;

class MQTTHandler {
public:
    MQTTHandler();

    // Starts the FreeRTOS background task (Core 0) to manage the loop() keep-alive
    void begin();

    // --- Configuration ---
    // Reads /mqttConfig.json using FileManager and populates the MQTTConfig struct
    bool load_config_from_fs();

    // --- 1. Explicit MQTT Methods ---
    void connect();                     // Force a connection attempt using loaded config
    void disconnect();                  // Force a graceful disconnect
    bool is_connected();                // Standard check

    // Bind the master callback from main.cpp (Where you handle the JSON to Pico translation)
    void on_message(MQTTMessageCallback callback);
    void on_topic(const char* exact_topic, TopicRoutedCallback callback);

    // Standard IO
    bool publish(const char* topic, const char* payload, bool retain = false);
    bool subscribe(const char* topic);

    // Get explicit current state for your UI or status LEDs
    MQTTState get_state();

private:
    MQTTConfig config;
    MQTTState current_state = MQTTState::DISCONNECTED;

    WiFiClientSecure wifi_client;
    PubSubClient mqtt_client;

    // The single master callback function provided by main.cpp
    MQTTMessageCallback user_callback = nullptr;

    // Internal Static Callback required by the PubSubClient C-library
    static void _internal_callback(char* topic, byte* payload, unsigned int length);

    std::map<String, TopicRoutedCallback> route_map;

    // FreeRTOS Background Task
    TaskHandle_t mqtt_task_handle = NULL;
    static void mqtt_background_task(void* parameter);
};