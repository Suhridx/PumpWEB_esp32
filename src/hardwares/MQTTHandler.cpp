#include "hardwares/MQTTHandler.hpp"
#include "configs/FileManager.hpp" // Utilizing your LittleFS static file manager
#include <SystemConstants.h>


constexpr const char* MQTT_CONFIG_FILE = "/mqttConfig.json";
constexpr const char* DEFAULT_MQTT_JSON = R"({
    "broker_ip": "af727d946e1d464a87eab96386068473.s1.eu.hivemq.cloud",
    "port": 8883,
    "username": "espclientlocalserver",
    "password": "qD4nhjfmjSig5Tx",
    "client_id": "esp32s3_client"
})";

// Global pointer needed strictly for the PubSubClient C-style static callback
static MQTTHandler* global_mqtt_instance = nullptr;

MQTTHandler::MQTTHandler() {
    global_mqtt_instance = this;
    wifi_client.setInsecure();
    wifi_client.setTimeout(3);

    mqtt_client.setClient(wifi_client);
    mqtt_client.setBufferSize(512);

    // Bind the static callback
    mqtt_client.setCallback(MQTTHandler::_internal_callback);
}

void MQTTHandler::begin() {
    Serial.println("[MQTT] Initializing Autonomous MQTT Controller...");

    // 1. Load config autonomously inside begin()
    if(!load_config_from_fs()) {
        Serial.println("[MQTT-ERR] Failed to load config. MQTT will idle.");
    }

    // 2. Start the self-managing background task
    xTaskCreatePinnedToCore(
        MQTTHandler::mqtt_background_task,
        "MQTT_Task",
        16384,
        this,
        1,
        &mqtt_task_handle,
        0
    );
}

bool MQTTHandler::load_config_from_fs() {
    // 1. The Magic One-Liner: Reads the file OR creates it with our default JSON template
    String json_data = FileManager::read_or_default(MQTT_CONFIG_FILE, DEFAULT_MQTT_JSON);

    if(json_data == "") {
        Serial.println("[MQTT-ERR] Config file empty and default recreation failed.");
        return false;
    }

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, json_data);

    if(error) {
        Serial.printf("[MQTT-ERR] JSON Parse Failed: %s\n", error.c_str());
        return false;
    }

    // Explicitly map JSON to the C++ Struct
    config.broker_ip = doc["broker_ip"].as<String>();
    config.port = doc["port"].as<uint16_t>();
    config.username = doc["username"].as<String>();
    config.password = doc["password"].as<String>();
    config.client_id = doc["client_id"].as<String>();

    // Lock in the server address
    mqtt_client.setServer(config.broker_ip.c_str(), config.port);

    Serial.println("[MQTT] Config loaded successfully from LittleFS.");
    return true;
}

void MQTTHandler::connect() {
    // Thread-Safety Check: By setting the state to CONNECTING, 
    // the FreeRTOS task on Core 0 will execute the actual TCP connection safely.
    if(current_state != MQTTState::CONNECTED) {
        Serial.println("[MQTT] Connection explicitly requested by main loop.");
        current_state = MQTTState::CONNECTING;
    }
}

void MQTTHandler::disconnect() {
    Serial.println("[MQTT] Disconnect explicitly requested.");
    mqtt_client.disconnect();
    current_state = MQTTState::DISCONNECTED;
}

bool MQTTHandler::is_connected() {
    return mqtt_client.connected();
}

MQTTState MQTTHandler::get_state() {
    return current_state;
}

void MQTTHandler::on_message(MQTTMessageCallback callback) {
    // Bind the main.cpp lambda function to our internal trigger
    this->user_callback = callback;
}

bool MQTTHandler::publish(const char* topic, const char* payload, bool retain) {
    if(current_state != MQTTState::CONNECTED) return false;
    return mqtt_client.publish(topic, payload, retain);
}

bool MQTTHandler::subscribe(const char* topic) {
    if(current_state != MQTTState::CONNECTED) return false;
    return mqtt_client.subscribe(topic);
}

void MQTTHandler::on_topic(const char* exact_topic, TopicRoutedCallback callback) {
    // Save the function into our dictionary
    route_map[String(exact_topic)] = callback;
}

// ==============================================================================
// Pure Bridge Callback (No JSON parsing, no Pico logic)
// ==============================================================================
void MQTTHandler::_internal_callback(char* topic, byte* payload, unsigned int length) {
    if(!global_mqtt_instance || !global_mqtt_instance->user_callback) return;

    // Convert raw bytes to standard C++ Strings
    String topic_str = String(topic);
    String payload_str = "";
    payload_str.reserve(length);
    for(unsigned int i = 0; i < length; i++) {
        payload_str += (char)payload[i];
    }

    // Explicitly hand the raw strings straight up to main.cpp
    if(global_mqtt_instance->route_map.count(topic_str) > 0) {
        // Match found! Trigger the specific separate callback
        global_mqtt_instance->route_map[topic_str](payload_str);
    }
    // Otherwise, fall back to the generic master callback (if you still want it)
    else if(global_mqtt_instance->user_callback) {
        global_mqtt_instance->user_callback(topic_str, payload_str);
    }
}



// ==============================================================================
// FreeRTOS Task (Running purely as a state machine on Core 0)
// ==============================================================================
void MQTTHandler::mqtt_background_task(void* parameter) {
    MQTTHandler* instance = static_cast<MQTTHandler*>(parameter);

    for(;;) {
        // Condition 1: Wi-Fi is physically connected
        if(WiFi.status() == WL_CONNECTED) {

            switch(instance->current_state) {

                // If disconnected, automatically transition to connecting
                case MQTTState::DISCONNECTED:
                case MQTTState::CONNECTION_LOST:
                    instance->current_state = MQTTState::CONNECTING;
                    break;

                case MQTTState::CONNECTING:
                    Serial.println("[MQTT] Attempting connection to Broker...");
                    if(instance->mqtt_client.connect(
                        instance->config.client_id.c_str(),
                        instance->config.username.c_str(),
                        instance->config.password.c_str()))
                    {
                        Serial.println("[MQTT] Broker Connected Successfully!");
                        instance->current_state = MQTTState::CONNECTED;

                        instance->mqtt_client.subscribe(MQTTConstants::SUB_COMMANDS);
                        instance->mqtt_client.subscribe(MQTTConstants::SUB_OTA);
                    }
                    else {
                        int err = instance->mqtt_client.state();
                        if(err == MQTT_CONNECT_BAD_CREDENTIALS || err == MQTT_CONNECT_UNAUTHORIZED) {
                            Serial.println("[MQTT-ERR] Auth Failed. Idling.");
                            instance->current_state = MQTTState::AUTH_FAILED;
                        }
                        else {
                            Serial.printf("[MQTT-ERR] Connection Failed (Code: %d). Retrying in 5s...\n", err);
                            instance->current_state = MQTTState::CONNECTION_LOST;

                            // Task manages its OWN retry interval without blocking other tasks
                            vTaskDelay(5000 / portTICK_PERIOD_MS);
                        }
                    }
                    break;

                case MQTTState::CONNECTED:
                    if(instance->mqtt_client.connected()) {
                        instance->mqtt_client.loop(); // Keep-alive and process messages
                    }
                    else {
                        Serial.println("[MQTT-ERR] Dropped from broker.");
                        instance->current_state = MQTTState::CONNECTION_LOST;
                    }
                    break;

                case MQTTState::AUTH_FAILED:
                    // Requires manual intervention/new config, do nothing
                    break;
            }
        }
        // Condition 2: Wi-Fi is disconnected
        else {
            if(instance->current_state == MQTTState::CONNECTED) {
                Serial.println("[MQTT] Wi-Fi lost. Halting MQTT loop safely.");
                instance->mqtt_client.disconnect();
                instance->current_state = MQTTState::DISCONNECTED;
            }
        }

        // CRITICAL: Yield to the Watchdog timer
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}