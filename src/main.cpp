#include <Arduino.h>
#include "SystemConstants.h"
#include "controllers/UARTController.hpp"
#include "hardwares/PicoHandler.hpp"
#include <configs/FileManager.hpp>
#include "controllers/NetworkController.hpp"
#include <hardwares/MQTTHandler.hpp>
#include "DataConstants.h"

// Define the hardware status union for memory-efficient flag tracking
typedef union
{
  struct
  {
    uint8_t eeprom_ready : 1; // Bit 0: 1 if I2C EEPROM responds
    uint8_t rtc_ready : 1;    // Bit 1: 1 if RTC initializes
    uint8_t reservoir_sensor_ready
      : 1;                       // Bit 2: 1 if upper float switch is active
    uint8_t tank_sensor_ready : 1; // Bit 3: 1 if lower float switch is active
    uint8_t relay_ready : 1;       // Bit 4: 1 if relay driver fails
    uint8_t bridge_connected : 1;     // Bit 5: 1 if Pico 2W is sending UART heartbeats
    uint8_t wifi_connected : 1;         // Bits 6-7: Reserved for future use
    uint8_t mqtt_connected : 1;         // Bit 8: 1 if MQTT is connected
  } bits;

  uint8_t all; // Access all bits simultaneously
} HardwareStatus_t;
// ==================================================================
// 1. Instantiate the Hardware Drivers
// ==================================================================
// UART1 (Pico Bridge) initialized at 115200 baud
UARTController pico_uart(1, UARTConfig::UART1_BAUD);
PicoHandler pico_handler(pico_uart);
NetworkController network_controller;
MQTTHandler mqtt;

DeviceConfigurationParams_t shadow_config;
ScheduleConfigTable shadow_schedules;
PumpRuntime_t shadow_runtime;
HardwareStatus_t hw_status;
HardwareStatus_t esp_local_status;
bool hw_sync_state = false;


void handle_incoming_mqtt(String topic, String payload);
void handle_settings_update(String payload);
void handle_schedule_update(String payload);
void handle_relay_command(String payload);
void handle_ota_command(String payload);
void handle_service_command(String payload);

void process_pico_packet(const PicoPacket& packet, PicoHandler& pico_handler);


void boot_sequence() {

  Serial.begin(115200);
  Serial.println("\n=========================================");
  Serial.println("  ESP32-S3 Network Bridge Initializing   ");
  Serial.println("=========================================");

  FileManager::begin();

  network_controller.begin();

  // Hook up the hardware interrupt using a clean inline lambda!
  pico_uart.set_rx_interrupt([](uint8_t byte) {
    // We keep the null check just in case the hardware UART 
    // triggers an interrupt before the C++ objects are fully constructed
    if(PicoHandler::instance != nullptr) {
      PicoHandler::instance->process_rx_byte(byte);
    }
    });


  mqtt.on_topic(MQTTConstants::CMD_SETTINGS, handle_settings_update);
  mqtt.on_topic(MQTTConstants::CMD_SCHEDULE, handle_schedule_update);
  mqtt.on_topic(MQTTConstants::CMD_RELAY, handle_relay_command);
  mqtt.on_topic(MQTTConstants::OTA_UPDATE, handle_ota_command);
  mqtt.on_topic(MQTTConstants::OTA_SERVICE, handle_service_command);

  mqtt.on_message(handle_incoming_mqtt);
  mqtt.begin();
}

void system_setup() {
  hw_status.all = 0;

  Serial.println("[BOOT] Pinging Pico and waiting for response...");
  pico_handler.send_ping();

  PicoPacket packet;
  uint32_t start_time = millis();

  // Block for up to 5 seconds
  while((millis() - start_time) < 5000) {

    if(pico_handler.pop_packet(packet)) {

      if(packet.msg_type == PicoProtocol::MSG_TYPE_ACK &&
        packet.length == 1 &&
        packet.payload[0] == PicoProtocol::MSG_TYPE_PING) {

        hw_status.bits.bridge_connected = 1;
        Serial.println("[BOOT] Pico successfully responded to PING.");
        break;
      }
    }
    delay(1);
  }


}



// ==================================================================
// SETUP
// ==================================================================
void setup() {
  delay(2000);

  boot_sequence();
  system_setup();
  trigger_hardware_sync();

  Serial.println("[SYSTEM] Started...");
}

// ==================================================================
// MAIN LOOP
// ==================================================================
void loop() {
  // Create a blank packet struct to hold incoming data

  PicoPacket packet;
  while(pico_handler.pop_packet(packet)) {
    process_pico_packet(packet, pico_handler);
  }

  pico_handler.process_tx_queue();
}


void trigger_hardware_sync() {

  esp_local_status.all = hw_status.all;

  Serial.printf("[SYNC] Pushing ESP32 Sensor State (0x%02X) to Pico...\n", esp_local_status.all);
  pico_handler.send_packet(PicoProtocol::MSG_TYPE_HW_SYNC, &esp_local_status.all, sizeof(HardwareStatus_t));
}



void process_pico_packet(const PicoPacket& packet, PicoHandler& pico_handler) {

  Serial.printf("\n[ESP32 RX] Valid Packet! Type: 0x%02X, Length: %d bytes\n",
    packet.msg_type, packet.length);

  switch(packet.msg_type) {

    // -------------------------------------------------------------------
    // 1. BASIC COMMS & LOGS (Your flawless code)
    // -------------------------------------------------------------------
    case PicoProtocol::MSG_TYPE_PING:
      Serial.println("-> Pico sent a PING. Sending ACK...");
      pico_handler.send_ack(PicoProtocol::MSG_TYPE_PING);
      break;

    case PicoProtocol::MSG_TYPE_LOG: {
      char log_buffer[PicoProtocol::MAX_PAYLOAD_SIZE + 1];
      memcpy(log_buffer, packet.payload, packet.length);
      log_buffer[packet.length] = '\0';
      Serial.printf("[PICO LOG]: %s\n", log_buffer);
      break;
    }

    case PicoProtocol::MSG_TYPE_RUN_REPORT: {
      if(packet.length == sizeof(PicoProtocol::PumpRunReport_t)) {
        PicoProtocol::PumpRunReport_t report;
        memcpy(&report, packet.payload, sizeof(PicoProtocol::PumpRunReport_t));
        Serial.println("--- 📊 PUMP RUN REPORT ---");
        Serial.printf("Mode: %d, Duration: %d mins\n", report.mode, report.run_duration_mins);
        Serial.println("--------------------------");
        // TODO: Push to MQTT / Google Drive
      }
      break;
    }

    case PicoProtocol::MSG_TYPE_ROUTINE_TIME: {
      if(packet.length == sizeof(PicoProtocol::RoutinePayload_t)) {
        PicoProtocol::RoutinePayload_t time_data;
        memcpy(&time_data, packet.payload, sizeof(PicoProtocol::RoutinePayload_t));
        Serial.printf("-> Pico Uptime: %d seconds\n", time_data.uptime_secs);
      }
      break;
    }

    case PicoProtocol::MSG_TYPE_CONFIG: {
      if(packet.length == sizeof(DeviceConfigurationParams_t)) {
        memcpy(&shadow_config, packet.payload, sizeof(DeviceConfigurationParams_t));
        Serial.println("[SYNC] Shadow Memory: Device Config Updated!");
        // TODO: Publish shadow_config to Cloud so dashboard updates!
      }
      break;
    }

    case PicoProtocol::MSG_TYPE_SCHEDULE: {
      if(packet.length == sizeof(ScheduleConfigTable)) {
        memcpy(&shadow_schedules, packet.payload, sizeof(ScheduleConfigTable));
        Serial.println("[SYNC] Shadow Memory: Schedules Updated!");
      }
      break;
    }

    case PicoProtocol::MSG_TYPE_STATE: {
      if(packet.length == sizeof(PumpRuntime_t)) {
        memcpy(&shadow_runtime, packet.payload, sizeof(PumpRuntime_t));
        Serial.println("[SYNC] Shadow Memory: Pump Runtime/Telemetry Updated!");
        // TODO: Instantly publish to MQTT so dashboard feels responsive!
      }
      break;
    }

    case PicoProtocol::MSG_TYPE_HW_SYNC: {

      if(packet.length == sizeof(HardwareStatus_t)) {

        HardwareStatus_t pico_merged_copy;
        memcpy(&pico_merged_copy, packet.payload, sizeof(HardwareStatus_t));

        if(pico_merged_copy.bits.tank_sensor_ready == esp_local_status.bits.tank_sensor_ready &&
          pico_merged_copy.bits.reservoir_sensor_ready == esp_local_status.bits.reservoir_sensor_ready)
        {
          hw_status.all = pico_merged_copy.all;
          Serial.printf("[SYNC] Consensus Reached! Master State: 0x%02X\n", hw_status.all);

          pico_handler.send_ack(PicoProtocol::MSG_TYPE_HW_SYNC);
        }
        else
        {
          Serial.println("[SYNC-ERR] Hardware consensus failed! Dropping packet.");
        }
      }
      break;
    }

    case PicoProtocol::MSG_TYPE_ACK: {
      if(packet.length == 1) {
        uint8_t acked_msg = packet.payload[0];
        Serial.printf("[PICO] Hardware Confirmed Success for: 0x%02X\n", acked_msg);

        // If Cloud changed settings, and Pico just ACK'd it, tell the Cloud!
        if(acked_msg == PicoProtocol::MSG_TYPE_CONFIG) {
          // publish_settings_to_cloud(); 
        }
      }
      break;
    }

    case PicoProtocol::MSG_TYPE_NACK: {
      if(packet.length == 1) {
        uint8_t nacked_msg = packet.payload[0];
        Serial.printf("[PICO-ERR] Hardware REJECTED message: 0x%02X\n", nacked_msg);

        // The hardware rejected our cloud command! Our shadow memory is corrupted.
        if(nacked_msg == PicoProtocol::MSG_TYPE_CONFIG ||
          nacked_msg == PicoProtocol::MSG_TYPE_SCHEDULE) {

          Serial.println("[SYNC] Requesting full EEPROM sync to heal shadow RAM...");
          pico_handler.send_packet(PicoProtocol::MSG_TYPE_REQUEST, nullptr, 0);
        }
      }
      break;
    }

    default:
      Serial.printf("[WARNING] Unhandled message type: 0x%02X\n", packet.msg_type);
      break;
  }
}

void handle_incoming_mqtt(String topic, String payload) {
  Serial.printf("\n[MQTT RX] Topic: %s, Payload: %s\n", topic.c_str(), payload.c_str());
  // ... (Your JSON parsing and Pico struct packing logic stays exactly the same) ...
}

void handle_settings_update(String payload) {
  Serial.println("[MQTT] Processing Device Settings Update...");

  // We use a larger JsonDocument (1024) because this struct has 25 variables
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload);

  if(error) {
    Serial.printf("[MQTT-ERR] Failed to parse Settings JSON: %s\n", error.c_str());
    return;
  }

  // --- 16-bit Values ---
  shadow_config.default_timer = doc["default_timer"] | shadow_config.default_timer;
  shadow_config.max_pump_runtime_sec = doc["max_pump_runtime_sec"] | shadow_config.max_pump_runtime_sec;
  shadow_config.cooldown_period = doc["cooldown_period"] | shadow_config.cooldown_period;
  shadow_config.restart_delay_sec = doc["restart_delay_sec"] | shadow_config.restart_delay_sec;
  shadow_config.sensor_timeout_sec = doc["sensor_timeout_sec"] | shadow_config.sensor_timeout_sec;
  shadow_config.timezone_minutes = doc["timezone_minutes"] | shadow_config.timezone_minutes;

  // --- 8-bit / Bool Values ---
  shadow_config.tank_thl = doc["tank_thl"] | shadow_config.tank_thl;
  shadow_config.res_thl = doc["res_thl"] | shadow_config.res_thl;
  shadow_config.lck_man_ctrl = doc["lck_man_ctrl"] | shadow_config.lck_man_ctrl;
  shadow_config.sch_turn_on = doc["sch_turn_on"] | shadow_config.sch_turn_on;
  shadow_config.apcd = doc["apcd"] | shadow_config.apcd;
  shadow_config.ovf_ctrl = doc["ovf_ctrl"] | shadow_config.ovf_ctrl;
  shadow_config.wl_ctrl = doc["wl_ctrl"] | shadow_config.wl_ctrl;

  shadow_config.schedule_master_enable = doc["schedule_master_enable"] | shadow_config.schedule_master_enable;
  shadow_config.skip_schedule_if_manual = doc["skip_schedule_if_manual"] | shadow_config.skip_schedule_if_manual;
  shadow_config.resume_schedule_after_manual = doc["resume_schedule_after_manual"] | shadow_config.resume_schedule_after_manual;
  shadow_config.dry_run_protection = doc["dry_run_protection"] | shadow_config.dry_run_protection;

  shadow_config.dst_enabled = doc["dst_enabled"] | shadow_config.dst_enabled;
  shadow_config.log_enable = doc["log_enable"] | shadow_config.log_enable;
  shadow_config.remote_control_enable = doc["remote_control_enable"] | shadow_config.remote_control_enable;
  shadow_config.cloud_sync_enable = doc["cloud_sync_enable"] | shadow_config.cloud_sync_enable;
  shadow_config.daily_reset_enabled = doc["daily_reset_enabled"] | shadow_config.daily_reset_enabled;

  // Notice we DO NOT allow the Cloud to overwrite magic, version, device_id, or crc!
  // The hardware handles those internally.

  // Transmit the updated 72-byte block to the Pico
  pico_handler.send_packet(PicoProtocol::MSG_TYPE_CONFIG,
    (uint8_t*)&shadow_config,
    sizeof(shadow_config));

  Serial.println("[MQTT] Sent updated Config to Pico.");
}

void handle_schedule_update(String payload) {
  Serial.println("[MQTT] Processing Schedule Update...");

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload);

  if(error) {
    Serial.printf("[MQTT-ERR] Failed to parse Schedule JSON: %s\n", error.c_str());
    return;
  }

  // 1. We MUST have an ID to know which schedule the Cloud wants to edit
  if(!doc.containsKey("id")) {
    Serial.println("[MQTT-ERR] Schedule payload is missing the 'id' key! Aborting.");
    return;
  }

  // Grab the target schedule ID
  uint8_t target_id = doc["id"];

  // 2. Validate the ID to prevent array out-of-bounds crashes
  if(target_id >= ScheduleConfig::MAX_SCHEDULES) {
    Serial.printf("[MQTT-ERR] Schedule ID %d is out of bounds!\n", target_id);
    return;
  }

  // 3. Apply partial updates to ONLY the targeted schedule entry in our shadow RAM
  shadow_schedules.entries[target_id].active = doc["active"] | shadow_schedules.entries[target_id].active;
  shadow_schedules.entries[target_id].start_time = doc["start_time"] | shadow_schedules.entries[target_id].start_time;
  shadow_schedules.entries[target_id].duration = doc["duration"] | shadow_schedules.entries[target_id].duration;
  shadow_schedules.entries[target_id].days_mask = doc["days_mask"] | shadow_schedules.entries[target_id].days_mask;
  shadow_schedules.entries[target_id].action_type = doc["action_type"] | shadow_schedules.entries[target_id].action_type;

  // (Again, we don't touch the magic, version, or crc bytes at the table level)

  // 4. Transmit the ENTIRE Schedule Table down to the Pico. 
  // The Pico expects the full struct via MSG_TYPE_SCHEDULE.
  pico_handler.send_packet(PicoProtocol::MSG_TYPE_SCHEDULE,
    (uint8_t*)&shadow_schedules,
    sizeof(shadow_schedules));

  Serial.printf("[MQTT] Sent updated Schedule Table (ID: %d modified) to Pico.\n", target_id);
}

void handle_relay_command(String payload) {
  Serial.println("[MAIN] Manual Relay Command Triggered...");
  if(payload == "ON") {
    // ... turn on pump ...
  }
}

void handle_ota_command(String payload) {
  Serial.println("[MAIN] OTA Command Triggered...");
  // ... handle OTA update logic ...
}

void handle_service_command(String payload) {
  Serial.println("[MAIN] Service Command Triggered...");
  // ... handle service command logic ...
}