// include/DataConstants.h (ON THE ESP32)
#pragma once
#include <cstdint>
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