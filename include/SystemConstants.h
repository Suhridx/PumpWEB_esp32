#pragma once

#include <cstdint>

// ==============================================================================
// System-Wide Constants Configuration (ESP32-S3 Mirror)
// Group constants by module using namespaces to prevent naming collisions.
// ==============================================================================

namespace GPIOConfig {
    // --- I2C Pins (Local Sensors / IO Expanders) ---
    constexpr uint8_t I2C0_SDA_PIN = 8;
    constexpr uint8_t I2C0_SCL_PIN = 9;

    // --- Hardware SPI Pins (SD Card - FSPI Default) ---
    constexpr uint8_t SPI_CS_PIN = 10;
    constexpr uint8_t SPI_MOSI_PIN = 11;
    constexpr uint8_t SPI_SCK_PIN = 12;
    constexpr uint8_t SPI_MISO_PIN = 13;

    // --- High-Speed Hardware UART Pins ---
    // UART1 (Communication with Raspberry Pi Pico)
    constexpr uint8_t UART1_TX_PIN = 17;
    constexpr uint8_t UART1_RX_PIN = 18;

    // UART2 (Nextion HMI Display)
    constexpr uint8_t UART2_TX_PIN = 15;
    constexpr uint8_t UART2_RX_PIN = 16;

    // --- Low-Speed Software Serial Pins ---
    // HC-12 Water Level Sensor Radio
    constexpr uint8_t HC12_TX_PIN = 4;
    constexpr uint8_t HC12_RX_PIN = 5;

    // --- Status LED Indicator ---
    constexpr uint8_t LED_WIFI_STATUS = 6; // Your master RGB or standard Wi-Fi LED

    // --- General Purpose / Reserve Pins ---
    // Note: Pins 1 and 2 support Analog Reads (ADC)
    constexpr uint8_t RESERVE_PIN_1 = 1;
    constexpr uint8_t RESERVE_PIN_2 = 2;
    constexpr uint8_t RESERVE_PIN_3 = 7; // Freed up from MQTT!
}

namespace UARTConfig {
    // UART 1 (Raspberry Pi Pico Bridge)
    constexpr uint32_t UART1_BAUD = 115200; // Must match ESP Protocol baud
    constexpr uint8_t UART1_TX_PIN = GPIOConfig::UART1_TX_PIN;
    constexpr uint8_t UART1_RX_PIN = GPIOConfig::UART1_RX_PIN;

    // UART 2 (Nextion HMI Display)
    constexpr uint32_t UART2_BAUD = 9600;
    constexpr uint8_t UART2_TX_PIN = GPIOConfig::UART2_TX_PIN;
    constexpr uint8_t UART2_RX_PIN = GPIOConfig::UART2_RX_PIN;

    // HC-12 Radio Link to the Water Level Sensors (Software Serial)
    // All three hardware UARTs are taken (0 debug, 1 Pico, 2 Nextion), so the radio
    // is bit-banged. These used to alias the UART2 pins, which are the Nextion HMI
    // lines, so they now point at the radio pair the GPIO map already reserved.
    constexpr uint32_t SENSOR_BAUD = 9600;
    constexpr uint8_t SENSOR_TX_PIN = GPIOConfig::HC12_TX_PIN; // ESP32 TX -> HC-12 RX
    constexpr uint8_t SENSOR_RX_PIN = GPIOConfig::HC12_RX_PIN; // ESP32 RX <- HC-12 TX
}

namespace SPIConfig {
    // --- Bus Pins (shared by every device hanging off the bus) ---
    constexpr uint8_t SCK_PIN = GPIOConfig::SPI_SCK_PIN;
    constexpr uint8_t MOSI_PIN = GPIOConfig::SPI_MOSI_PIN;
    constexpr uint8_t MISO_PIN = GPIOConfig::SPI_MISO_PIN;

    // --- Chip Select (one per device on the bus) ---
    constexpr uint8_t FLASH_CS_PIN = GPIOConfig::SPI_CS_PIN;

    // --- Clock Speeds ---
    constexpr uint32_t DEFAULT_CLOCK_HZ = 40 * 1000 * 1000; // 40 MHz
    constexpr uint32_t SAFE_CLOCK_HZ = 1 * 1000 * 1000;     // 1 MHz for long wires / debugging

    // Device geometry and timeouts are NOT here on purpose. They belong to the
    // chip driver in hardwares/, so this bus layer stays device agnostic.
}

namespace BAUD {
    constexpr uint32_t lowspeed = 100 * 1000; // 100 kHz (Standard Mode)
    constexpr uint32_t hispeed = 400 * 1000; // 400 kHz (Fast Mode)
    constexpr uint32_t esp32 = 115200; // ESP32 AT Commands / Custom Bridge
    constexpr uint32_t sensor = 9600;   // General sensors, GPS, etc.
}

// --- Data Structures (Identical to Pico for memory alignment) ---

namespace DeviceConfigSettings {
    constexpr uint16_t EEPROM_START_ADDR = 0x0000;
    constexpr uint16_t VALID_MAGIC = 0x55AA;
    constexpr uint16_t CURRENT_VERSION = 0x0001;
}

namespace ScheduleConfig {
    constexpr uint16_t MAX_SCHEDULES = 6;

    constexpr uint16_t EEPROM_START_ADDR = 0x0040;
    constexpr uint16_t VALID_MAGIC = 0xA66A;
    constexpr uint16_t CURRENT_VERSION = 0x0001;

    constexpr uint8_t DAY_SUN = 1 << 0; // 0x01
    constexpr uint8_t DAY_MON = 1 << 1; // 0x02
    constexpr uint8_t DAY_TUE = 1 << 2; // 0x04
    constexpr uint8_t DAY_WED = 1 << 3; // 0x08
    constexpr uint8_t DAY_THU = 1 << 4; // 0x10
    constexpr uint8_t DAY_FRI = 1 << 5; // 0x20
    constexpr uint8_t DAY_SAT = 1 << 6; // 0x40
    constexpr uint8_t DAYS_EVERYDAY = 0x7F; // 0111 1111
}

enum DeviceType : uint8_t {
    DEV_MASTER = 0,
    DEV_TANK = 1,
    DEV_RES = 2,
    DEV_NANO = 3,
    DEV_ALL = 255 // Broadcast address
};

namespace PumpConfig {
    constexpr uint16_t EEPROM_START_ADDR = 0x0080;

    // Unique Magic Number and Version for the Runtime block
    constexpr uint16_t VALID_MAGIC = 0xB77B;
    constexpr uint16_t CURRENT_VERSION = 0x0003;

    // Explicitly sized enum for strict memory mapping
    enum PumpMode_t : uint8_t {
        MODE_AUTO = 0,           // Normal: Follows RTC daily schedules automatically
        MODE_MANUAL = 1,         // Manual: Ignores schedules, waits for physical/app button
        MODE_TIMER = 2,          // Override: Running a one-off countdown 
        MODE_OFFLINE = 3,        // Safety/Maintenance: System completely disabled
        MODE_POWER_FAILURE = 4,  // Power failure boot
        MODE_TURNED_OFF = 5      // Explicitly turned off
    };
}

// --- Network & Web Server Configurations ---
namespace NetworkConfig {
    constexpr uint16_t SERVER_PORT = 80;
    // Wi-Fi credentials will be injected via PlatformIO environment variables or SPIFFS later
}

namespace MQTTConstants {
    // --- Subscribe Topics (Data coming IN from Cloud) ---
    constexpr const char* SUB_COMMANDS = "pump/cmd/#";
    constexpr const char* SUB_OTA = "pump/ota/#";
    constexpr const char* SUB_REQ = "pump/req/#";

    constexpr const char* CMD_SETTINGS = "pump/cmd/settings";
    constexpr const char* CMD_SCHEDULE = "pump/cmd/schedule";
    constexpr const char* CMD_RELAY = "pump/cmd/run";

    constexpr const char* OTA_UPDATE = "pump/ota/update";
    constexpr const char* OTA_SERVICE = "pump/ota/service";

    // --- Publish Topics (Data going OUT to Cloud) ---
    constexpr const char* PUB_TELEMETRY = "pump/state/telemetry";
    constexpr const char* PUB_LOGS = "pump/state/logs";
    constexpr const char* PUB_STATUS = "pump/state/sync";
}