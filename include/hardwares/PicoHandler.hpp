#pragma once

#include "controllers/UARTController.hpp"
#include <cstdint>

// --- Protocol Definitions ---
namespace PicoProtocol {
    constexpr uint8_t SYNC1 = 0xAA;
    constexpr uint8_t SYNC2 = 0x55;

    // Message Types
    constexpr uint8_t MSG_TYPE_STATE = 0x01;
    constexpr uint8_t MSG_TYPE_CONFIG = 0x02;
    constexpr uint8_t MSG_TYPE_LOG = 0x03;
    constexpr uint8_t MSG_TYPE_REQUEST = 0x04;
    constexpr uint8_t MSG_TYPE_HW_SYNC = 0x0D;
    constexpr uint8_t MSG_TYPE_SENSOR = 0x05;
    constexpr uint8_t MSG_TYPE_SCHEDULE = 0x09;

    constexpr uint8_t MSG_TYPE_REMOTE_CMD = 0x0A; // Remote Turn ON/OFF command

    constexpr uint8_t MSG_TYPE_RUN_REPORT = 0x0C; // Cloud reporting
    constexpr uint8_t MSG_TYPE_ROUTINE_TIME = 0x0E; // Routine time update

    // The ultra-compact 8-byte payload received from the Pico
    struct __attribute__((packed)) PumpRunReport_t {
        uint16_t start_time_mins;
        uint16_t end_time_mins;
        uint16_t run_duration_mins;
        uint8_t  mode;
        uint8_t  schedule_id;
    };

    struct __attribute__((packed)) RoutinePayload_t {
        uint16_t mins_from_midnight; // 2 bytes
        uint32_t uptime_secs;        // 4 bytes
    };

    constexpr uint8_t MSG_TYPE_PING = 0x06;
    constexpr uint8_t MSG_TYPE_ACK = 0x07;
    constexpr uint8_t MSG_TYPE_NACK = 0x08;

    constexpr uint16_t MAX_PAYLOAD_SIZE = 256;
} // namespace PicoProtocol


// Struct to hold a complete packet in the queue
struct PicoPacket {
    uint8_t msg_type;
    uint16_t length;
    uint8_t payload[PicoProtocol::MAX_PAYLOAD_SIZE];
};

class PicoHandler {
public:
    PicoHandler(UARTController& uart_ref);

    // Wraps the payload in headers/CRC and sends it over UART
    void send_packet(uint8_t msg_type, const uint8_t* payload, uint16_t length);

    // State machine to process incoming bytes (Called from ISR)
    void process_rx_byte(uint8_t byte);

    void send_ping();
    void send_ack(uint8_t ack_msg_type);
    void send_nack(uint8_t nack_msg_type);

    // Processes all waiting packets in the queue (Called from Main Loop)
    bool pop_packet(PicoPacket& out_packet);

    // Drains the TX queue and physically sends data over UART
    void process_tx_queue();

    // Public instance pointer so static callbacks can access it directly
    static PicoHandler* instance;

private:
    UARTController& uart;

    // --- RX State Machine Variables ---
    enum class RXState {
        WAIT_SYNC1,
        WAIT_SYNC2,
        WAIT_TYPE,
        WAIT_LEN_L,
        WAIT_LEN_H,
        RECEIVE_PAYLOAD,
        WAIT_CRC_L,
        WAIT_CRC_H
    };

    RXState rx_state;
    uint8_t rx_msg_type;
    uint16_t rx_length;
    uint16_t rx_index;
    uint16_t rx_crc_received;
    uint8_t rx_buffer[PicoProtocol::MAX_PAYLOAD_SIZE];

    // --- Ring Buffer Variables ---
    static constexpr uint8_t QUEUE_SIZE = 5; // Can hold 5 packets at once
    PicoPacket rx_queue[QUEUE_SIZE];
    QueueHandle_t tx_queue;

    // NEW: The actual hardware write function (hidden from the rest of the system)

    // Volatile is critical here so the compiler knows these change inside an ISR
    volatile uint8_t queue_head = 0; // Where the ISR writes
    volatile uint8_t queue_tail = 0; // Where the main loop reads

    // Helper to calculate data integrity
    uint16_t calculate_crc(uint8_t msg_type, uint16_t length, const uint8_t* payload);

    void physical_send_packet(uint8_t msg_type, const uint8_t* payload, uint16_t length);
};