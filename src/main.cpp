#include <Arduino.h>
#include "SystemConstants.h"
#include "controllers/UARTController.hpp"
#include "hardwares/PicoHandler.hpp"
#include "controllers/NetworkController.hpp"
#include <hardwares/MQTTHandler.hpp>
#include "controllers/SPIController.hpp"
#include "hardwares/W25Q64Handler.hpp"
#include "hardwares/WS2812Handler.hpp"
#include "util/FlashFileManager.hpp"
#include "util/FlashLog.hpp"
#include "DataConstants.h"
#include "Log.h"


// ==================================================================
// 1. Instantiate the Hardware Drivers
// ==================================================================
// UART1 (Pico Bridge) initialized at 115200 baud
UARTController pico_uart(1, UARTConfig::UART1_BAUD);
PicoHandler pico_handler(pico_uart);
NetworkController network_controller;
MQTTHandler mqtt;

// SPI flash chain: raw bus -> W25Q64 chip driver
SPIController spi_bus;
W25Q64Handler flash(spi_bus);

// ==================================================================
// External flash layout, 2048 sectors of 4 KB
// ==================================================================
// The file system takes the bottom 5 MB and keeps its mirrored directory in sectors 0 and
// 1, exactly where it already expects them. The two log rings take the tail. The asserts
// below are the guard against the three regions ever drifting into each other.
constexpr uint32_t FS_SECTOR_COUNT = 1280;  // 5 MB: html, Pico uf2, json configs
constexpr uint32_t PICO_LOG_START = 1280;
constexpr uint32_t PICO_LOG_SECTORS = 256;  // 1 MB
constexpr uint32_t ESP_LOG_START = 1536;
constexpr uint32_t ESP_LOG_SECTORS = 512;   // 2 MB

static_assert(PICO_LOG_START == FS_SECTOR_COUNT, "Logs must start where the file system ends");
static_assert(ESP_LOG_START == PICO_LOG_START + PICO_LOG_SECTORS, "Log regions must be adjacent");
static_assert(ESP_LOG_START + ESP_LOG_SECTORS == W25Q64::SECTOR_COUNT, "Regions must tile the chip");

FlashFileManager flash_fs(flash, FS_SECTOR_COUNT);
FlashLog pico_log(flash, PICO_LOG_START, PICO_LOG_SECTORS);
FlashLog esp_log(flash, ESP_LOG_START, ESP_LOG_SECTORS);

// WS2812 status panel, mirrors hw_status
WS2812Handler status_leds;

DeviceConfigurationParams_t shadow_config;
ScheduleConfigTable shadow_schedules;
PumpRuntime_t shadow_runtime;
HardwareStatus_t hw_status;
// Serialises the read-modify-store that every hw_status bit write compiles down to.
// The WiFi and MQTT tasks write it from core 0 while this file writes it from core 1.
portMUX_TYPE hw_status_mux = portMUX_INITIALIZER_UNLOCKED;
MemorySyncStatus_t memory_sync_status;

// How often loop() refreshes and republishes the fields this side owns in the shared
// status struct. This is a periodic state refresh, not a liveness probe, so it does not
// PING first the way the boot handshake does: it simply checks the local hardware and
// pushes the result. bridge_connected is maintained separately by any received traffic.
constexpr uint32_t HW_SYNC_INTERVAL_MS = 30000;

// millis() of the most recent packet received from the Pico, whatever its type. Only ever
// written from core 1 (the boot handshake and the main loop), so it needs no lock.
uint32_t last_pico_rx = 0;

// How long the Pico may stay silent before the bridge is declared down. Both ends push a
// HW_SYNC every 30 s and each answers the other, so while the Pico is alive traffic is
// expected comfortably inside one interval. The margin over 30 s is deliberate: a timeout
// equal to a single period would trip on ordinary jitter or one dropped packet and make
// the flag flap between up and down.
constexpr uint32_t BRIDGE_TIMEOUT_MS = 45000;



void handle_incoming_mqtt(String topic, String payload);
void handle_settings_update(String payload);
void handle_schedule_update(String payload);
void handle_relay_command(String payload);
void handle_ota_command(String payload);
void handle_service_command(String payload);

void process_pico_packet(const PicoPacket& packet, PicoHandler& pico_handler);
void trigger_hardware_sync();


void boot_sequence() {

  Serial.begin(115200);
  LOGLN("\n=========================================");
  LOGLN("  ESP32-S3 Network Bridge Initializing   ");
  LOGLN("=========================================");

  // Cleared here rather than in system_setup(), because the WiFi and MQTT background
  // tasks are started at the bottom of this function and immediately start writing
  // their own bits. Zeroing the word after that point would wipe a live connection flag.
  //
  // Deliberately unguarded: this is the one hw_status write that runs before either
  // background task exists, so there is no second writer to race against yet.
  hw_status.all = 0;
  memory_sync_status.all = 0;

  spi_bus.begin();
  flash.begin();
  flash_fs.begin();
  pico_log.begin();
  esp_log.begin();

  status_leds.begin();

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

  const bool wifi_up = network_controller.is_connected();
  const bool mqtt_up = (mqtt.get_state() == MQTTState::CONNECTED);
  const bool flash_up = flash.is_detected();
  {
    HwStatusLock lock;
    hw_status.bits.wifi_connected = wifi_up ? 1 : 0;
    hw_status.bits.mqtt_connected = mqtt_up ? 1 : 0;
    hw_status.bits.flash_ready = flash_up ? 1 : 0;
  }

  // --- Phase 2: send the refreshed word ---

  LOGLN("[BOOT] Pinging Pico and waiting for response...");
  pico_handler.send_ping();

  PicoPacket packet;
  uint32_t start_time = millis();
  bool ping_acked = false;

  // Block for up to 5 seconds
  while(!ping_acked && (millis() - start_time) < 5000) {
    pico_handler.process_tx_queue();

    if(pico_handler.pop_packet(packet)) {
      // Routed through the normal handler, so bridge_connected, the liveness clock and the
      // bridge pixel are all maintained exactly as they are once loop() is running, and
      // anything else the Pico sends meanwhile is processed rather than discarded.
      process_pico_packet(packet, pico_handler);

      ping_acked = (packet.msg_type == PicoProtocol::MSG_TYPE_ACK &&
        packet.length == 1 &&
        packet.payload[0] == PicoProtocol::MSG_TYPE_PING);
    }

    delay(1);
  }

  if(ping_acked) {
    LOGLN("[BOOT] Pico successfully responded to PING.");
  }
  else {
    LOGLN("[BOOT-WARN] Pico did not answer the PING within the timeout.");
  }

  LOGF("[SYNC] Pushing ESP32 Sensor State (0x%04X) to Pico...\n", hw_status.all);
  pico_handler.send_packet(PicoProtocol::MSG_TYPE_HW_SYNC, (uint8_t*)&hw_status, sizeof(HardwareStatus_t));
  pico_handler.process_tx_queue();


  start_time = millis();
  bool sync_done = false;

  while(!sync_done && (millis() - start_time) < 5000) {
    // Nothing else drains the queue yet, so this both pushes the HW_SYNC out and gets the
    // resulting ACK or NACK back onto the wire.
    pico_handler.process_tx_queue();

    if(pico_handler.pop_packet(packet)) {

      process_pico_packet(packet, pico_handler);
      sync_done = (packet.msg_type == PicoProtocol::MSG_TYPE_HW_SYNC_RESP);
    }

    delay(1);
  }

  if(!sync_done) {
    LOGLN("[BOOT-WARN] No HW_SYNC_RESP from the Pico. Consensus left to the main loop.");
  }


  LOGLN("[BOOT] Requesting initial memory sync from Pico...");
  uint8_t req_target = PicoProtocol::TARGET_ALL_DATA;
  pico_handler.send_packet(PicoProtocol::MSG_TYPE_REQUEST, &req_target, 1);

  LOGLN("[BOOT] System setup complete. Entering main loop.");

}


// ==================================================================
// SETUP
// ==================================================================
void setup() {
  delay(2000);

  boot_sequence();
  system_setup();

  LOGLN("[SYSTEM] Started...");
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

  // Traffic from the Pico is the only thing that raises bridge_connected, so this is the
  // only thing that can lower it again. Without it a Pico that stops talking stays latched
  // connected forever. The flag guard keeps this to a single log line per outage.
  if(hw_status.bits.bridge_connected && (millis() - last_pico_rx) >= BRIDGE_TIMEOUT_MS) {
    {
      HwStatusLock lock;
      hw_status.bits.bridge_connected = 0;
    }
    LOGF("[BRIDGE] Silent for %lu ms. Marking bridge down.\n", millis() - last_pico_rx);
  }


  static uint32_t last_hw_sync = 0;
  if(millis() - last_hw_sync >= HW_SYNC_INTERVAL_MS) {
    last_hw_sync = millis();
    trigger_hardware_sync();
  }

  pico_handler.process_tx_queue();
  status_leds.update_leds(hw_status);
}


/**
 * @brief Pushes the ESP32 owned half of the shared status struct to the Pico.
 *
 * HW_SYNC carries a struct that both firmwares share, with each side owning a subset of
 * the fields. This runs in two phases: it first checks the hardware this side is
 * responsible for so the packet carries measured state rather than whatever the
 * background tasks last wrote, then it transmits the refreshed word. The Pico replies
 * with MSG_TYPE_HW_SYNC_RESP, which is verified and merged in process_pico_packet().
 */
void trigger_hardware_sync() {

  // --- Phase 1: Check the fields this side owns ---
  const bool wifi_up = network_controller.is_connected();
  const bool mqtt_up = (mqtt.get_state() == MQTTState::CONNECTED);
  const bool flash_up = flash.is_detected();
  {
    HwStatusLock lock;
    hw_status.bits.wifi_connected = wifi_up ? 1 : 0;
    hw_status.bits.mqtt_connected = mqtt_up ? 1 : 0;
    hw_status.bits.flash_ready = flash_up ? 1 : 0;
  }

  // --- Phase 2: send the refreshed word ---
  LOGF("[SYNC] Pushing ESP32 Sensor State (0x%04X) to Pico...\n", hw_status.all);
  pico_handler.send_packet(PicoProtocol::MSG_TYPE_HW_SYNC, (uint8_t*)&hw_status, sizeof(HardwareStatus_t));
}



void process_pico_packet(const PicoPacket& packet, PicoHandler& pico_handler) {

  // Any valid packet at all proves the bridge is alive, whatever it carries. Recorded here
  // rather than in the individual handlers so that message types with no acknowledgement
  // of their own, a LOG or a STATE update, still count as evidence of life and still push
  // the timeout out.
  last_pico_rx = millis();
  {
    HwStatusLock lock;
    hw_status.bits.bridge_connected = 1;
  }

  // Visible confirmation that the link is carrying traffic, not merely flagged up.
  status_leds.bridge_blink();

  // LOGF("\n[ESP32 RX] Valid Packet! Type: 0x%02X, Length: %d bytes\n",packet.msg_type, packet.length);

  switch(packet.msg_type) {

    // -------------------------------------------------------------------
    // 1. BASIC COMMS & LOGS (Your flawless code)
    // -------------------------------------------------------------------
    case PicoProtocol::MSG_TYPE_PING:
      LOGLN("-> Pico sent a PING. Sending ACK...");
      pico_handler.send_ack(PicoProtocol::MSG_TYPE_PING);
      break;

    case PicoProtocol::MSG_TYPE_LOG: {
      char log_buffer[PicoProtocol::MAX_PAYLOAD_SIZE + 1];
      memcpy(log_buffer, packet.payload, packet.length);
      log_buffer[packet.length] = '\0';
      LOGF("[PICO LOG]: %s\n", log_buffer);

      // Straight into the ring: one page program, no metadata rewrite, no allocation.
      pico_log.append(packet.payload, packet.length);
      break;
    }

    case PicoProtocol::MSG_TYPE_RUN_REPORT: {
      if(packet.length == sizeof(PicoProtocol::PumpRunReport_t)) {
        PicoProtocol::PumpRunReport_t report;
        memcpy(&report, packet.payload, sizeof(PicoProtocol::PumpRunReport_t));
        LOGLN("--- 📊 PUMP RUN REPORT ---");
        LOGF("Mode: %d, Duration: %d mins\n", report.mode, report.run_duration_mins);
        LOGLN("--------------------------");
        // TODO: Push to MQTT / Google Drive
      }
      break;
    }

    case PicoProtocol::MSG_TYPE_ROUTINE_TIME: {
      if(packet.length == sizeof(PicoProtocol::RoutinePayload_t)) {
        PicoProtocol::RoutinePayload_t time_data;
        memcpy(&time_data, packet.payload, sizeof(PicoProtocol::RoutinePayload_t));
        LOGF("-> Pico time: %u mins from midnight, up %lu seconds\n",
          (unsigned)time_data.mins_from_midnight, (unsigned long)time_data.uptime_secs);

        // The only real clock either side has. Stamped into every record from here on.
        pico_log.set_time(time_data.mins_from_midnight);
        esp_log.set_time(time_data.mins_from_midnight);
      }
      break;
    }

    case PicoProtocol::MSG_TYPE_CONFIG: {
      if(packet.length == sizeof(DeviceConfigurationParams_t)) {
        memcpy(&shadow_config, packet.payload, sizeof(DeviceConfigurationParams_t));
        memory_sync_status.bits.config_synced = 1;
        LOGLN("[SYNC] Shadow Memory: Device Config Updated & Unlocked!");
        // TODO: Publish shadow_config to Cloud so dashboard updates!
      }
      break;
    }

    case PicoProtocol::MSG_TYPE_SCHEDULE: {
      if(packet.length == sizeof(ScheduleConfigTable)) {
        memcpy(&shadow_schedules, packet.payload, sizeof(ScheduleConfigTable));
        memory_sync_status.bits.schedules_synced = 1;
        LOGLN("[SYNC] Shadow Memory: Schedules Updated & Unlocked!");
      }
      break;
    }

    case PicoProtocol::MSG_TYPE_STATE: {
      if(packet.length == sizeof(PumpRuntime_t)) {
        memcpy(&shadow_runtime, packet.payload, sizeof(PumpRuntime_t));
        memory_sync_status.bits.runtime_synced = 1;
        LOGLN("[SYNC] Shadow Memory: Pump Runtime/Telemetry Updated!");
        // TODO: Instantly publish to MQTT so dashboard feels responsive!
      }
      break;
    }

    case PicoProtocol::MSG_TYPE_HW_SYNC: {

      if(packet.length == sizeof(HardwareStatus_t)) {
        HardwareStatus_t incoming_pico;
        memcpy(&incoming_pico, packet.payload, sizeof(HardwareStatus_t));
        {
          HwStatusLock lock;
          hw_status.bits.eeprom_ready = incoming_pico.bits.eeprom_ready;
          hw_status.bits.rtc_ready = incoming_pico.bits.rtc_ready;
          hw_status.bits.relay_ready = incoming_pico.bits.relay_ready;
          hw_status.bits.pump_state = incoming_pico.bits.pump_state;
        }

        // 2. Respond with the newly merged Master Byte
        pico_handler.send_packet(PicoProtocol::MSG_TYPE_HW_SYNC_RESP, (uint8_t*)&hw_status, sizeof(HardwareStatus_t));
      }
      break;
    }

    case PicoProtocol::MSG_TYPE_HW_SYNC_RESP: {

      if(packet.length == sizeof(HardwareStatus_t)) {
        HardwareStatus_t incoming_pico;
        memcpy(&incoming_pico, packet.payload, sizeof(HardwareStatus_t));

        // 1. Verify the Pico correctly echoed the fields the ESP32 owns
        bool verification_passed = (
          incoming_pico.bits.wifi_connected == hw_status.bits.wifi_connected &&
          incoming_pico.bits.mqtt_connected == hw_status.bits.mqtt_connected &&
          incoming_pico.bits.tank_sensor_ready == hw_status.bits.tank_sensor_ready &&
          incoming_pico.bits.reservoir_sensor_ready == hw_status.bits.reservoir_sensor_ready &&
          incoming_pico.bits.flash_ready == hw_status.bits.flash_ready
          );

        if(verification_passed) {
          // One critical section for the whole merge, as above.
          {
            HwStatusLock lock;
            hw_status.bits.eeprom_ready = incoming_pico.bits.eeprom_ready;
            hw_status.bits.rtc_ready = incoming_pico.bits.rtc_ready;
            hw_status.bits.relay_ready = incoming_pico.bits.relay_ready;
            hw_status.bits.pump_state = incoming_pico.bits.pump_state;
          }

          pico_handler.send_ack(PicoProtocol::MSG_TYPE_HW_SYNC_RESP);
          LOGF("[SYNC] HW Consensus Reached: 0x%04X\n", hw_status.all);
        }
        else {
          LOGLN("[SYNC-ERR] HW Sync Verification FAILED! Sending NACK and re-syncing.");
          pico_handler.send_nack(PicoProtocol::MSG_TYPE_HW_SYNC_RESP);

          // This exchange was started by this side, so this side owns the retry. The word
          // was just read for the verification above and is still current, so it goes back
          // out as is rather than re-running the phase 1 hardware checks.
          pico_handler.send_packet(PicoProtocol::MSG_TYPE_HW_SYNC, (uint8_t*)&hw_status, sizeof(HardwareStatus_t));
        }
      }
      break;
    }

    case PicoProtocol::MSG_TYPE_ACK: {
      if(packet.length == 1) {
        uint8_t acked_msg = packet.payload[0];
        LOGF("[PICO] Hardware Confirmed Success for: 0x%02X\n", acked_msg);

        if(acked_msg == PicoProtocol::MSG_TYPE_CONFIG ||
          acked_msg == PicoProtocol::MSG_TYPE_SCHEDULE) {
          status_leds.memory_blink(StatusLed::COLOR_GREEN);
        }
        if(acked_msg == PicoProtocol::MSG_TYPE_CONFIG) {
          memory_sync_status.bits.config_synced = 1;
        }
        else if(acked_msg == PicoProtocol::MSG_TYPE_SCHEDULE) {
          memory_sync_status.bits.schedules_synced = 1;
        }
      }
      break;
    }

    case PicoProtocol::MSG_TYPE_NACK: {
      if(packet.length == 1) {
        uint8_t nacked_msg = packet.payload[0];
        LOGF("[PICO-ERR] Hardware REJECTED message: 0x%02X\n", nacked_msg);

        // The red counterpart to the ACK pulse above, scoped to the same two transactions.
        if(nacked_msg == PicoProtocol::MSG_TYPE_CONFIG ||
          nacked_msg == PicoProtocol::MSG_TYPE_SCHEDULE) {
          status_leds.memory_blink(StatusLed::COLOR_RED);
        }

        // 1. Config Rollback
        if(nacked_msg == PicoProtocol::MSG_TYPE_CONFIG) {
          LOGLN("[SYNC] Config transaction failed. Requesting targeted Config rollback...");
          mqtt.enqueue_action(MqttAction_t::ERR_CONFIG_NACK);
          uint8_t req_target = PicoProtocol::TARGET_CONFIG;
          pico_handler.send_packet(PicoProtocol::MSG_TYPE_REQUEST, &req_target, 1);
        }
        // 2. Schedule Rollback
        else if(nacked_msg == PicoProtocol::MSG_TYPE_SCHEDULE) {
          LOGLN("[SYNC] Schedule transaction failed. Requesting targeted Schedule rollback...");
          mqtt.enqueue_action(MqttAction_t::ERR_SCHEDULE_NACK);
          uint8_t req_target = PicoProtocol::TARGET_SCHEDULE;
          pico_handler.send_packet(PicoProtocol::MSG_TYPE_REQUEST, &req_target, 1);
        }
        else if(nacked_msg == PicoProtocol::MSG_TYPE_HW_SYNC_RESP) {
          // Receiving this NACK means the Pico initiated that exchange and rejected the
          // response this side sent back. Re-driving a handshake is the initiator's job, so
          // the retry belongs on the Pico. This end only records that consensus was lost.
          LOGLN("[SYNC-ERR] Hardware Sync consensus failed. Retry belongs to the Pico.");
          memory_sync_status.bits.hw_state_synced = 0;
        }
      }
      break;
    }

    default:
      LOGF("[WARNING] Unhandled message type: 0x%02X\n", packet.msg_type);
      break;
  }
}

void handle_incoming_mqtt(String topic, String payload) {
  LOGF("\n[MQTT RX] Topic: %s, Payload: %s\n", topic.c_str(), payload.c_str());
  // ... (Your JSON parsing and Pico struct packing logic stays exactly the same) ...
}

void handle_settings_update(String payload) {
  LOGLN("[MQTT] Processing Device Settings Update...");

  if(!memory_sync_status.bits.config_synced) {
    LOGLN("[MQTT-WARN] Config locked or syncing! Dropping update.");
    return;
  }

  // We use a larger JsonDocument (1024) because this struct has 25 variables
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload);

  if(error) {
    LOGF("[MQTT-ERR] Failed to parse Settings JSON: %s\n", error.c_str());
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

  memory_sync_status.bits.config_synced = 0;
  LOGLN("[MQTT] Processing Device Settings Update. (Lock Engaged)");

  pico_handler.send_packet(PicoProtocol::MSG_TYPE_CONFIG,
    (uint8_t*)&shadow_config,
    sizeof(shadow_config));

  LOGLN("[MQTT] Sent updated Config to Pico.");
}

void handle_schedule_update(String payload) {
  LOGLN("[MQTT] Processing Schedule Update...");
  if(!memory_sync_status.bits.schedules_synced) {
    LOGLN("[MQTT-WARN] Schedules locked or syncing! Dropping update.");
    return;
  }

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload);

  if(error) {
    LOGF("[MQTT-ERR] Failed to parse Schedule JSON: %s\n", error.c_str());
    return;
  }

  // 1. We MUST have an ID to know which schedule the Cloud wants to edit
  if(!doc.containsKey("id")) {
    LOGLN("[MQTT-ERR] Schedule payload is missing the 'id' key! Aborting.");
    return;
  }

  // Grab the target schedule ID
  uint8_t target_id = doc["id"];

  // 2. Validate the ID to prevent array out-of-bounds crashes
  if(target_id >= ScheduleConfig::MAX_SCHEDULES) {
    LOGF("[MQTT-ERR] Schedule ID %d is out of bounds!\n", target_id);
    return;
  }

  // 3. Apply partial updates to ONLY the targeted schedule entry in our shadow RAM
  shadow_schedules.entries[target_id].active = doc["active"] | shadow_schedules.entries[target_id].active;
  shadow_schedules.entries[target_id].start_time = doc["start_time"] | shadow_schedules.entries[target_id].start_time;
  shadow_schedules.entries[target_id].duration = doc["duration"] | shadow_schedules.entries[target_id].duration;
  shadow_schedules.entries[target_id].days_mask = doc["days_mask"] | shadow_schedules.entries[target_id].days_mask;
  shadow_schedules.entries[target_id].action_type = doc["action_type"] | shadow_schedules.entries[target_id].action_type;

  memory_sync_status.bits.schedules_synced = 0;
  LOGLN("[MQTT] Processing Schedule Update. (Lock Engaged)");

  pico_handler.send_packet(PicoProtocol::MSG_TYPE_SCHEDULE,
    (uint8_t*)&shadow_schedules,
    sizeof(shadow_schedules));

  LOGF("[MQTT] Sent updated Schedule Table (ID: %d modified) to Pico.\n", target_id);
}

void handle_relay_command(String payload) {
  LOGLN("[MAIN] Manual Relay Command Triggered...");
  if(payload == "ON") {
    // ... turn on pump ...
  }
}

void handle_ota_command(String payload) {
  LOGLN("[MAIN] OTA Command Triggered...");
  // ... handle OTA update logic ...
}

void handle_service_command(String payload) {
  LOGLN("[MAIN] Service Command Triggered...");
  // ... handle service command logic ...
}