#include "hardwares/MQTTHandler.hpp"
#include "configs/FileManager.hpp" // Utilizing your LittleFS static file manager
#include <SystemConstants.h>
#include "DataConstants.h"
#include "hardwares/WS2812Handler.hpp"
#include "Log.h"

// Defined in main.cpp. network_blink() only raises a flag for update_leds() to act on
// later, so the broker task on core 0 can call it directly.
extern WS2812Handler status_leds;


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
    event_queue = xQueueCreate(10, sizeof(MqttAction_t));

    wifi_client.setInsecure();
    wifi_client.setTimeout(3);

    mqtt_client.setClient(wifi_client);
    mqtt_client.setBufferSize(512);

    // Bind the static callback
    mqtt_client.setCallback(MQTTHandler::_internal_callback);
}

void MQTTHandler::begin() {
    LOGLN("[MQTT] Initializing Autonomous MQTT Controller...");

    // 1. Load config autonomously inside begin()
    if(!load_config_from_fs()) {
        LOGLN("[MQTT-ERR] Failed to load config. MQTT will idle.");
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
        LOGLN("[MQTT-ERR] Config file empty and default recreation failed.");
        return false;
    }

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, json_data);

    if(error) {
        LOGF("[MQTT-ERR] JSON Parse Failed: %s\n", error.c_str());
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

    LOGLN("[MQTT] Config loaded successfully from LittleFS.");
    return true;
}

void MQTTHandler::connect() {
    // Thread-Safety Check: By setting the state to CONNECTING, 
    // the FreeRTOS task on Core 0 will execute the actual TCP connection safely.
    if(current_state != MQTTState::CONNECTED) {
        LOGLN("[MQTT] Connection explicitly requested by main loop.");
        set_state(MQTTState::CONNECTING);
    }
}

void MQTTHandler::disconnect() {
    LOGLN("[MQTT] Disconnect explicitly requested.");
    mqtt_client.disconnect();
    set_state(MQTTState::DISCONNECTED);
}

void MQTTHandler::set_state(MQTTState new_state) {
    current_state = new_state;

    // The status panel and the Pico HW_SYNC handshake both read this bit, so it moves
    // in lock step with the state machine instead of being polled. Only CONNECTED
    // means there is a live broker session behind it.
    //
    // Guarded because this task runs on core 0 while the main loop writes neighbouring
    // bits of the same byte from core 1.
    {
        HwStatusLock lock;
        hw_status.bits.mqtt_connected = (new_state == MQTTState::CONNECTED) ? 1 : 0;
    }
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

    // Raised before dispatch so it counts whether or not a handler is bound for this
    // topic. This only sets a flag; core 1 plays the pulse out in update_leds().
    status_leds.network_blink();

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


void MQTTHandler::enqueue_action(MqttAction_t action) {
    if(event_queue != nullptr) {
        // 0 ticks = non-blocking push from the fast hardware loop
        xQueueSend(event_queue, &action, 0);
    }
}

void MQTTHandler::process_mqtt_queue() {
    if(event_queue == nullptr) return;

    MqttAction_t action;
    if(xQueueReceive(event_queue, &action, 0) == pdTRUE) {
        switch(action) {
            case MqttAction_t::PUB_CONFIG:
                publish_sync_event("config", "success");
                break;
            case MqttAction_t::PUB_SCHEDULE:
                publish_sync_event("schedule", "success");
                break;
            case MqttAction_t::ERR_CONFIG_NACK:
                publish_sync_event("config", "error");
                break;
            case MqttAction_t::ERR_SCHEDULE_NACK:
                publish_sync_event("schedule", "error");
                break;
            default:
                break;
        }
    }
}

void MQTTHandler::publish_sync_event(const char* target, const char* status) {
    LOGF("[MQTT] Building sync event: Target=%s, Status=%s\n", target, status);

    // Adjust size based on your max expected JSON string
    StaticJsonDocument<1024> doc; // Or JsonDocument doc; if using ArduinoJson v7

    // 1. Build the Standard Envelope Header
    doc["status"] = status;
    doc["target"] = target;

    // 2. Attach Data on Success, or Message on Error
    if(strcmp(status, "success") == 0) {

        // ArduinoJson automatically detects the struct type and calls 
        // the corresponding convertToJson() function we wrote in DataConstants.h!
        if(strcmp(target, "config") == 0) {
            doc["data"] = shadow_config;
        }
        else if(strcmp(target, "schedule") == 0) {
            doc["data"] = shadow_schedules;
        }
        else if(strcmp(target, "runtime") == 0) {
            doc["data"] = shadow_runtime;
        }

    }
    else {
        doc["message"] = "hardware_rejected";
    }

    // 3. Serialize and Publish
    String payload;
    serializeJson(doc, payload);

    // Publish to the unified status topic
    publish(MQTTConstants::PUB_STATUS, payload.c_str(), false);
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
                    instance->set_state(MQTTState::CONNECTING);
                    break;

                case MQTTState::CONNECTING:
                    LOGLN("[MQTT] Attempting connection to Broker...");
                    if(instance->mqtt_client.connect(
                        instance->config.client_id.c_str(),
                        instance->config.username.c_str(),
                        instance->config.password.c_str()))
                    {
                        LOGLN("[MQTT] Broker Connected Successfully!");
                        instance->set_state(MQTTState::CONNECTED);

                        instance->mqtt_client.subscribe(MQTTConstants::SUB_COMMANDS);
                        instance->mqtt_client.subscribe(MQTTConstants::SUB_OTA);
                    }
                    else {
                        int err = instance->mqtt_client.state();
                        if(err == MQTT_CONNECT_BAD_CREDENTIALS || err == MQTT_CONNECT_UNAUTHORIZED) {
                            LOGLN("[MQTT-ERR] Auth Failed. Idling.");
                            instance->set_state(MQTTState::AUTH_FAILED);
                        }
                        else {
                            LOGF("[MQTT-ERR] Connection Failed (Code: %d). Retrying in 5s...\n", err);
                            instance->set_state(MQTTState::CONNECTION_LOST);

                            // Task manages its OWN retry interval without blocking other tasks
                            vTaskDelay(5000 / portTICK_PERIOD_MS);
                        }
                    }
                    break;

                case MQTTState::CONNECTED:
                    if(instance->mqtt_client.connected()) {
                        instance->mqtt_client.loop(); // Keep-alive and process messages
                        instance->process_mqtt_queue();
                    }
                    else {
                        LOGLN("[MQTT-ERR] Dropped from broker.");
                        instance->set_state(MQTTState::CONNECTION_LOST);
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
                LOGLN("[MQTT] Wi-Fi lost. Halting MQTT loop safely.");
                instance->mqtt_client.disconnect();
                instance->set_state(MQTTState::DISCONNECTED);
            }
        }

        // CRITICAL: Yield to the Watchdog timer
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}