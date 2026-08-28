#include "hardwares/PicoHandler.hpp"
#include <Arduino.h> // Included for memcpy

// Initialize static instance pointer
PicoHandler* PicoHandler::instance = nullptr;

PicoHandler::PicoHandler(UARTController& uart_ref)
    : uart(uart_ref), rx_state(RXState::WAIT_SYNC1) {
    // Store the active instance so static callbacks can find it
    instance = this;

    tx_queue = xQueueCreate(10, sizeof(PicoPacket));
}

void PicoHandler::send_packet(uint8_t msg_type, const uint8_t* payload, uint16_t length) {
    if(length > PicoProtocol::MAX_PAYLOAD_SIZE)
        return; // Prevent buffer overflow

    PicoPacket tx_packet;
    tx_packet.msg_type = msg_type;
    tx_packet.length = length;

    if(length > 0 && payload != nullptr) {
        memcpy(tx_packet.payload, payload, length);
    }

    // Auto-detect if we are inside the UART hardware interrupt (ISR)
    if(xPortInIsrContext()) {
        BaseType_t high_task_woken = pdFALSE;
        // ISR-Safe queue push
        xQueueSendFromISR(tx_queue, &tx_packet, &high_task_woken);
    }
    else {
        // Normal Task queue push (Wait max 10 ticks if queue is temporarily full)
        xQueueSend(tx_queue, &tx_packet, 10);
    }


}

void PicoHandler::physical_send_packet(uint8_t msg_type, const uint8_t* payload, uint16_t length) {
    uint16_t crc = calculate_crc(msg_type, length, payload);

    uint8_t header[5];
    header[0] = PicoProtocol::SYNC1;
    header[1] = PicoProtocol::SYNC2;
    header[2] = msg_type;
    header[3] = length & 0xFF;
    header[4] = (length >> 8) & 0xFF;
    uart.send_bytes(header, 5);

    if(length > 0 && payload != nullptr) {
        uart.send_bytes(payload, length);
    }

    uint8_t crc_buf[2];
    crc_buf[0] = crc & 0xFF;
    crc_buf[1] = (crc >> 8) & 0xFF;
    uart.send_bytes(crc_buf, 2);
}

void PicoHandler::process_tx_queue() {
    PicoPacket tx_packet;

    // Attempt to pop a packet from the queue. 
    // 0 = Do not block! If empty, immediately return pdFALSE and move on.
    if(xQueueReceive(tx_queue, &tx_packet, 0) == pdTRUE) {
        // A packet was waiting! Physically send it.
        physical_send_packet(tx_packet.msg_type, tx_packet.payload, tx_packet.length);
    }
}

void PicoHandler::send_ping() {
    send_packet(PicoProtocol::MSG_TYPE_PING, nullptr, 0);
}

void PicoHandler::send_ack(uint8_t ack_msg_type) {
    // Send an ACK (0x07) with a 1-byte payload containing the ID of the command we are acknowledging
    send_packet(PicoProtocol::MSG_TYPE_ACK, &ack_msg_type, 1);
}

void PicoHandler::send_nack(uint8_t nack_msg_type) {
    // Send a NACK (0x08) with a 1-byte payload containing the ID of the failed command
    send_packet(PicoProtocol::MSG_TYPE_NACK, &nack_msg_type, 1);
}

void PicoHandler::process_rx_byte(uint8_t byte) {
    switch(rx_state) {
        case RXState::WAIT_SYNC1:
            if(byte == PicoProtocol::SYNC1)
                rx_state = RXState::WAIT_SYNC2;
            break;

        case RXState::WAIT_SYNC2:
            if(byte == PicoProtocol::SYNC2)
                rx_state = RXState::WAIT_TYPE;
            else
                rx_state = RXState::WAIT_SYNC1; // Reset if false positive
            break;

        case RXState::WAIT_TYPE:
            rx_msg_type = byte;
            rx_state = RXState::WAIT_LEN_L;
            break;

        case RXState::WAIT_LEN_L:
            rx_length = byte;
            rx_state = RXState::WAIT_LEN_H;
            break;

        case RXState::WAIT_LEN_H:
            rx_length |= (byte << 8);
            if(rx_length > PicoProtocol::MAX_PAYLOAD_SIZE) {
                rx_state = RXState::WAIT_SYNC1;
            }
            else if(rx_length == 0) {
                rx_state = RXState::WAIT_CRC_L;
            }
            else {
                rx_index = 0;
                rx_state = RXState::RECEIVE_PAYLOAD;
            }
            break;

        case RXState::RECEIVE_PAYLOAD:
            rx_buffer[rx_index++] = byte;
            if(rx_index >= rx_length) {
                rx_state = RXState::WAIT_CRC_L;
            }
            break;

        case RXState::WAIT_CRC_L:
            rx_crc_received = byte;
            rx_state = RXState::WAIT_CRC_H;
            break;

        case RXState::WAIT_CRC_H:
            rx_crc_received |= (byte << 8);

            // Validate the entire packet
            uint16_t calculated_crc = calculate_crc(rx_msg_type, rx_length, rx_buffer);

            if(rx_crc_received == calculated_crc) {
                // SUCCESS: Push to Ring Buffer!
                // Calculate where the next head will be
                uint8_t next_head = (queue_head + 1) % QUEUE_SIZE;

                if(next_head != queue_tail) {
                    // The queue is NOT full. Copy the data in.
                    rx_queue[queue_head].msg_type = rx_msg_type;
                    rx_queue[queue_head].length = rx_length;
                    if(rx_length > 0) {
                        memcpy(rx_queue[queue_head].payload, rx_buffer, rx_length);
                    }
                    // Move the head forward (this is atomic and safe)
                    queue_head = next_head;
                }
                else {
                    // Buffer overflow! The main loop is too slow. 
                    // We silently drop the packet and send a NACK. 
                    send_nack(rx_msg_type);
                }
            }
            else {
                // CRC Error!
                send_nack(rx_msg_type);
            }

            // Reset for the next packet
            rx_state = RXState::WAIT_SYNC1;
            break;
    }
}

bool PicoHandler::pop_packet(PicoPacket& out_packet) {
    // If head equals tail, the queue is empty
    if(queue_tail == queue_head) {
        return false;
    }

    // Copy the packet data into the provided struct
    out_packet = rx_queue[queue_tail];

    // Move the tail forward to free up the slot for the ISR
    queue_tail = (queue_tail + 1) % QUEUE_SIZE;

    return true;
}

// Internal Packet Processor

uint16_t PicoHandler::calculate_crc(uint8_t msg_type, uint16_t length,
    const uint8_t* payload) {
    uint16_t checksum = msg_type + (length & 0xFF) + ((length >> 8) & 0xFF);
    for(uint16_t i = 0; i < length; i++) {
        checksum += payload[i];
    }
    return checksum;
}