#include "controllers/NetworkController.hpp"
#include "util/FlashFileManager.hpp" // Config storage now lives on the external SPI flash
#include "DataConstants.h"           // Shared hw_status word, read by the LED panel and the Pico sync
#include "Log.h"

// Defined in main.cpp, and mounted in boot_sequence() before this controller starts, so
// the config read below always runs against a live file system.
extern FlashFileManager flash_fs;

constexpr const char* WIFI_CONFIG_FILE = "/wifiConfig.json";
constexpr const char* DEFAULT_WIFI_JSON = R"({
        "networks": [
            {
                "ssid": "NetBus_Home 6🅴",
                "password": "apsecure_aes2.x"
            }
        ]
    })";
constexpr uint8_t MAX_RETRIES = 5;

NetworkController::NetworkController() {}

void NetworkController::begin() {
    LOGLN("[WIFI] Initializing Robust Network Controller...");

    if(!load_config()) {
        LOGLN("[WIFI] No valid saved config found. Awaiting manual input.");
    }

    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true);
    delay(100);

    xTaskCreatePinnedToCore(
        NetworkController::wifi_background_task,
        "WiFi_Task",
        4096,
        this,
        1,
        &wifi_task_handle,
        0
    );
}

void NetworkController::connect_to(const char* ssid, const char* password) {
    LOGF("[WIFI] Manual override. Connecting to: %s\n", ssid);
    current_ssid = ssid;
    current_password = password;
    retry_count = 0;

    WiFi.disconnect();
    set_state(WiFiState::DISCONNECTED);
}

bool NetworkController::load_config() {
    // 1. The Magic One-Liner: Reads the file OR creates it with our default JSON template
    String json_data = flash_fs.read_or_default(WIFI_CONFIG_FILE, DEFAULT_WIFI_JSON);

    if(json_data == "") return false;

    // 2. Parse the string directly
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, json_data);

    if(error) {
        LOGF("[WIFI-ERR] Failed to parse wificonfig.json: %s\n", error.c_str());
        return false;
    }

    saved_networks.clear();

    JsonArray networks = doc["networks"].as<JsonArray>();
    for(JsonObject net : networks) {
        WiFiCredential cred;
        cred.ssid = net["ssid"].as<String>();
        cred.password = net["password"].as<String>();
        saved_networks.push_back(cred);
    }

    if(saved_networks.size() > 0) {
        current_ssid = saved_networks[0].ssid;
        current_password = saved_networks[0].password;
        LOGF("[WIFI] Loaded %d saved networks. Primary: %s\n", saved_networks.size(), current_ssid.c_str());
        return true;
    }

    return false;
}

bool NetworkController::save_config(const char* ssid, const char* password) {
    // 1. Add to active memory list (pushing to the front so it's prioritized)
    WiFiCredential new_cred = {String(ssid), String(password)};
    saved_networks.insert(saved_networks.begin(), new_cred);

    // 2. Generate updated JSON document
    JsonDocument doc;
    JsonArray networks = doc["networks"].to<JsonArray>();

    for(const auto& cred : saved_networks) {
        JsonObject net = networks.add<JsonObject>();
        net["ssid"] = cred.ssid;
        net["password"] = cred.password;
    }

    // 3. Serialize to a standard C++ String
    String output_json;
    serializeJson(doc, output_json);

    // 4. Let the FileManager handle the disk write
    if(!flash_fs.write_file(WIFI_CONFIG_FILE, output_json.c_str())) {
        LOGLN("[WIFI-ERR] Failed to save updated config.");
        return false;
    }

    LOGLN("[WIFI] Successfully saved new network config.");

    // Trigger immediate connection attempt
    connect_to(ssid, password);
    return true;
}

String NetworkController::scan_networks() {
    int n = WiFi.scanNetworks();
    JsonDocument doc;
    JsonArray networks = doc["networks"].to<JsonArray>();

    for(int i = 0; i < n; ++i) {
        JsonObject network = networks.add<JsonObject>();
        network["ssid"] = WiFi.SSID(i);
        network["rssi"] = WiFi.RSSI(i);
        network["encryption"] = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "Open" : "Secured";
    }

    String output;
    serializeJson(doc, output);
    return output;
}

bool NetworkController::is_connected() {
    return WiFi.status() == WL_CONNECTED;
}

void NetworkController::disconnect() {
    WiFi.disconnect();
    set_state(WiFiState::DISCONNECTED);
}

void NetworkController::set_state(WiFiState new_state) {
    current_state = new_state;

    // The status panel and the Pico HW_SYNC handshake both read this bit, so it moves
    // in lock step with the state machine instead of being polled. CONNECTED is the
    // only state in which the station actually holds an IP address.
    //
    // Guarded because this task runs on core 0 while the main loop writes neighbouring
    // bits of the same byte from core 1.
    {
        HwStatusLock lock;
        hw_status.bits.wifi_connected = (new_state == WiFiState::CONNECTED) ? 1 : 0;
    }
}

// ============================================================================
// FreeRTOS Background Task (Running permanently on Core 0)
// ============================================================================
void NetworkController::wifi_background_task(void* parameter) {
    NetworkController* instance = static_cast<NetworkController*>(parameter);

    for(;;) {
        switch(instance->current_state) {

            case WiFiState::DISCONNECTED:
                if(instance->current_ssid != "") {
                    LOGF("[WIFI] Attempting connection to %s (Try %d/%d)\n",
                        instance->current_ssid.c_str(), instance->retry_count + 1, MAX_RETRIES);
                    WiFi.begin(instance->current_ssid.c_str(), instance->current_password.c_str());
                    instance->set_state(WiFiState::CONNECTING);
                }
                else {
                    instance->set_state(WiFiState::WAITING_FOR_NETWORK);
                }
                break;

            case WiFiState::CONNECTING:
                if(WiFi.status() == WL_CONNECTED) {
                    LOGF("[WIFI] Connected! IP Address: %s\n", WiFi.localIP().toString().c_str());
                    instance->retry_count = 0;
                    instance->set_state(WiFiState::CONNECTED);
                }
                else if(WiFi.status() == WL_CONNECT_FAILED || WiFi.status() == WL_NO_SSID_AVAIL || WiFi.status() == WL_DISCONNECTED) {
                    instance->retry_count++;

                    if(instance->retry_count >= MAX_RETRIES) {
                        LOGLN("[WIFI-ERR] Max retries reached. Switching to network scan mode.");
                        instance->set_state(WiFiState::SCANNING_FOR_KNOWN);
                    }
                    else {
                        instance->set_state(WiFiState::DISCONNECTED);
                        vTaskDelay(5000 / portTICK_PERIOD_MS);
                    }
                }
                break;

            case WiFiState::CONNECTED:
                if(WiFi.status() != WL_CONNECTED) {
                    LOGLN("[WIFI] Connection lost!");
                    instance->set_state(WiFiState::DISCONNECTED);
                }
                break;

            case WiFiState::SCANNING_FOR_KNOWN: {
                LOGLN("[WIFI] Scanning airwaves for known networks...");
                int n = WiFi.scanNetworks();
                bool found_network = false;

                for(int i = 0; i < n; ++i) {
                    String scanned_ssid = WiFi.SSID(i);

                    for(const auto& cred : instance->saved_networks) {
                        if(scanned_ssid == cred.ssid) {
                            LOGF("[WIFI] Found backup network: %s\n", cred.ssid.c_str());
                            instance->current_ssid = cred.ssid;
                            instance->current_password = cred.password;
                            instance->retry_count = 0;
                            instance->set_state(WiFiState::DISCONNECTED);
                            found_network = true;
                            break;
                        }
                    }
                    if(found_network) break;
                }

                if(!found_network) {
                    LOGLN("[WIFI] No known networks found in range. Going to sleep.");
                    instance->set_state(WiFiState::WAITING_FOR_NETWORK);
                }
                break;
            }

            case WiFiState::WAITING_FOR_NETWORK:
                vTaskDelay(30000 / portTICK_PERIOD_MS);
                LOGLN("[WIFI] Waking up to check for networks again...");
                instance->set_state(WiFiState::SCANNING_FOR_KNOWN);
                break;
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}