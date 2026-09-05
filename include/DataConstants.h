// include/DataConstants.h (ON THE ESP32)
#pragma once
#include <cstdint>
#include <ArduinoJson.h> // NEW: Required for custom converters
#include <freertos/FreeRTOS.h> // portMUX_TYPE and the critical section macros
#include "SystemConstants.h"
// ---------------------------------------------------------
// Struct 1: Copied from Pico's DeviceManager.hpp
// ---------------------------------------------------------
struct __attribute__((packed)) DeviceConfigurationParams_t {
    uint16_t magic;
    uint16_t version;
    uint32_t device_id;
    uint16_t default_timer;
    uint16_t max_pump_runtime_sec;
    uint16_t cooldown_period;
    uint16_t restart_delay_sec;
    uint16_t sensor_timeout_sec;
    int16_t  timezone_minutes;
    uint8_t tank_thl;
    uint8_t res_thl;
    uint8_t lck_man_ctrl;
    uint8_t sch_turn_on;
    uint8_t apcd;
    uint8_t ovf_ctrl;
    uint8_t wl_ctrl;
    uint8_t schedule_master_enable;
    uint8_t skip_schedule_if_manual;
    uint8_t resume_schedule_after_manual;
    uint8_t dry_run_protection;
    uint8_t dst_enabled;
    uint8_t log_enable;
    uint8_t remote_control_enable;
    uint8_t cloud_sync_enable;
    uint8_t daily_reset_enabled;
    uint16_t crc;
};

// ---------------------------------------------------------
// Struct 2: Copied from Pico's ScheduleManager.hpp
// ---------------------------------------------------------
struct __attribute__((packed)) ScheduleConfigurationParams_t {
    uint8_t id;
    uint8_t active;
    uint16_t start_time;
    uint16_t duration;
    uint8_t days_mask;
    uint8_t action_type;
};

struct __attribute__((packed)) ScheduleConfigTable {
    uint16_t magic;
    uint16_t version;
    ScheduleConfigurationParams_t entries[ScheduleConfig::MAX_SCHEDULES];
    uint16_t crc;
};

// ---------------------------------------------------------
// Struct 3: Copied from Pico's PumpRuntimeManager.hpp
// ---------------------------------------------------------
struct __attribute__((packed)) PumpRuntime_t {
    uint16_t magic;
    uint16_t version;
    uint8_t pump_state;
    uint8_t active_schedule_id;
    uint16_t last_start_time;
    uint16_t last_end_time;
    uint16_t next_schedule_time;
    uint16_t last_run_duration;
    uint16_t total_run_time_today;
    uint8_t schedule_completed_today;
    uint8_t missed_schedule_count;
    uint8_t current_date;
    uint8_t current_mode;
    uint16_t crc;
};

// Define the hardware status union for memory-efficient flag tracking.
// NOTE: this layout is shared byte for byte with the Pico firmware. Any change here
// must be mirrored there, otherwise sizeof(HardwareStatus_t) stops matching and every
// HW_SYNC packet gets dropped by the length check on one side of the link.
typedef union
{
    struct
    {
        // --- Byte 0: I2C and local hardware, owned by the Pico ---
        uint16_t eeprom_ready : 1;           // Bit 0: 1 if I2C EEPROM responds
        uint16_t rtc_ready : 1;              // Bit 1: 1 if RTC initializes
        uint16_t reservoir_sensor_ready : 1; // Bit 2: 1 if upper float switch is active
        uint16_t tank_sensor_ready : 1;      // Bit 3: 1 if lower float switch is active
        uint16_t relay_ready : 1;            // Bit 4: 1 if relay driver is healthy
        uint16_t bridge_connected : 1;       // Bit 5: 1 if Pico 2W is sending UART heartbeats
        uint16_t wifi_connected : 1;         // Bit 6: 1 if the ESP32 holds an IP address
        uint16_t mqtt_connected : 1;         // Bit 7: 1 if the MQTT broker session is up

        // --- Byte 1: ESP32 local peripherals and pump state ---
        uint16_t flash_ready : 1;            // Bit 8: 1 if the W25Q64 answered its JEDEC ID
        uint16_t pump_state : 1;             // Bit 9: 1 while the pump is running
        uint16_t reserved : 6;               // Bits 10-15: free for future flags
    } bits;

    uint16_t all; // Access all bits simultaneously
} HardwareStatus_t;

// The Pico shares this exact type. Catch a divergence at compile time rather than
// silently dropping every HW_SYNC packet at runtime.
static_assert(sizeof(HardwareStatus_t) == 2, "HardwareStatus_t must stay 2 bytes wide");

typedef union {
    struct {
        uint8_t config_synced : 1; // Bit 0
        uint8_t schedules_synced : 1; // Bit 1
        uint8_t runtime_synced : 1; // Bit 2
        uint8_t hw_state_synced : 1; // Bit 3
        uint8_t reserved : 4; // Bits 4-7
    } bits;
    uint8_t all;
} MemorySyncStatus_t;

// Latest reading from one water level sensor node, as the rest of the system sees
// it. Application data rather than a wire packet: whatever ends up decoding the
// radio payload lands the result here.
struct SensorData_t
{
    // char name[32];
    uint8_t device_id;
    float wLevel;
    float distance;
};


// ==============================================================================
// ArduinoJson Custom Converters
// ==============================================================================

// 1. Converter for Device Configuration
inline bool convertToJson(const DeviceConfigurationParams_t& src, JsonVariant dst) {
    JsonObject obj = dst.to<JsonObject>();

    // Map your specific config variables[cite: 3]
    obj["default_timer"] = src.default_timer;
    obj["max_pump_runtime_sec"] = src.max_pump_runtime_sec;
    obj["cooldown_period"] = src.cooldown_period;
    obj["restart_delay_sec"] = src.restart_delay_sec;
    obj["sensor_timeout_sec"] = src.sensor_timeout_sec;
    obj["timezone_minutes"] = src.timezone_minutes;
    obj["tank_thl"] = src.tank_thl;
    obj["res_thl"] = src.res_thl;
    obj["lck_man_ctrl"] = src.lck_man_ctrl;
    obj["sch_turn_on"] = src.sch_turn_on;
    obj["apcd"] = src.apcd;
    obj["ovf_ctrl"] = src.ovf_ctrl;
    obj["wl_ctrl"] = src.wl_ctrl;
    obj["schedule_master_enable"] = src.schedule_master_enable;
    obj["skip_schedule_if_manual"] = src.skip_schedule_if_manual;
    obj["resume_schedule_after_manual"] = src.resume_schedule_after_manual;
    obj["dry_run_protection"] = src.dry_run_protection;
    obj["dst_enabled"] = src.dst_enabled;
    obj["log_enable"] = src.log_enable;
    obj["remote_control_enable"] = src.remote_control_enable;
    obj["cloud_sync_enable"] = src.cloud_sync_enable;
    obj["daily_reset_enabled"] = src.daily_reset_enabled;

    return true;
}

// 2. Converter for Schedule Table
inline bool convertToJson(const ScheduleConfigTable& src, JsonVariant dst) {
    JsonArray array = dst.to<JsonArray>();

    // Loop through based on MAX_SCHEDULES[cite: 3]
    for(int i = 0; i < ScheduleConfig::MAX_SCHEDULES; i++) {
        if(src.entries[i].active) { // Check the 'active' flag[cite: 3]
            JsonObject entry = array.add<JsonObject>();
            entry["id"] = src.entries[i].id;
            entry["start_time"] = src.entries[i].start_time;
            entry["duration"] = src.entries[i].duration;
            entry["days_mask"] = src.entries[i].days_mask;
            entry["action_type"] = src.entries[i].action_type;
        }
    }

    return true;
}

// 3. Converter for Pump Runtime
inline bool convertToJson(const PumpRuntime_t& src, JsonVariant dst) {
    JsonObject obj = dst.to<JsonObject>();

    // Map runtime variables[cite: 3]
    obj["pump_state"] = src.pump_state;
    obj["active_schedule_id"] = src.active_schedule_id;
    obj["last_start_time"] = src.last_start_time;
    obj["last_end_time"] = src.last_end_time;
    obj["next_schedule_time"] = src.next_schedule_time;
    obj["last_run_duration"] = src.last_run_duration;
    obj["total_run_time_today"] = src.total_run_time_today;
    obj["schedule_completed_today"] = src.schedule_completed_today;
    obj["missed_schedule_count"] = src.missed_schedule_count;
    obj["current_date"] = src.current_date;
    obj["current_mode"] = src.current_mode;

    return true;
}

// Ensure the shadow memory is globally accessible to MQTTHandler
extern DeviceConfigurationParams_t shadow_config;
extern ScheduleConfigTable shadow_schedules;
extern PumpRuntime_t shadow_runtime;

// Live hardware status word, owned by main.cpp. The network layers own their own
// bits in it: NetworkController drives wifi_connected and MQTTHandler drives
// mqtt_connected, so the WS2812 panel and the Pico HW_SYNC handshake always read
// the real link state instead of a stale boot time value.
extern HardwareStatus_t hw_status;

// Guards hw_status. Defined in main.cpp alongside the status word itself.
extern portMUX_TYPE hw_status_mux;

/**
 * @brief RAII critical section held across every write to hw_status.
 *
 * hw_status is a bitfield union, so each write is a read-modify-store of the containing
 * byte, and every contended bit shares byte 0: the main loop on core 1 writes
 * bridge_connected and relay_ready while the WiFi and MQTT tasks on core 0 write
 * wifi_connected and mqtt_connected. Two cores inside that sequence at once lose one of
 * the updates, and because the flags only move on transitions the loss never repairs
 * itself: the WiFi task sits in CONNECTED and does not write the bit again until the
 * link actually drops, so the panel can blink a fault at a perfectly healthy link.
 *
 * Declare one in the narrowest scope around the write, and keep the body to the status
 * word alone. A critical section stalls both cores, so no logging, no SPI and nothing
 * blocking belongs inside it. Nothing writes hw_status from an ISR, so the plain macros
 * are the correct ones here rather than the _ISR variants.
 */
class HwStatusLock {
public:
    HwStatusLock() { portENTER_CRITICAL(&hw_status_mux); }
    ~HwStatusLock() { portEXIT_CRITICAL(&hw_status_mux); }

    // Copying a guard would leave the critical section twice
    HwStatusLock(const HwStatusLock&) = delete;
    HwStatusLock& operator=(const HwStatusLock&) = delete;
};