#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <cstdint>
#include <string>

class UARTController {
public:
    /**
     * @brief Initializes the hardware UART bus and assigns pins.
     * @param uart_num Hardware UART port (0, 1, or 2 for ESP32)
     * @param rx_pin GPIO pin for RX
     * @param tx_pin GPIO pin for TX
     * @param baudrate Communication speed in Hz (e.g., 115200)
     */
    UARTController(uint8_t uart_bus, uint32_t baudrate);

    /**
     * @brief Sends a null-terminated string over the UART bus.
     * @param str The string to send.
     */
    void send_string(const char* str);

    /**
     * @brief Sends raw byte data over the UART bus.
     * @param data Pointer to the byte array.
     * @param len Number of bytes to send.
     */
    void send_bytes(const uint8_t* data, size_t len);

    /**
     * @brief Checks if there is incoming data waiting in the RX FIFO.
     * @return true if data is available to read.
     */
    bool is_readable();

    /**
     * @brief Reads a single byte from the RX FIFO.
     * @note Make sure to check is_readable() before calling this, otherwise it will block.
     * @return The byte read from the UART.
     */
    uint8_t read_byte();

    /**
     * @brief Returns the initialized UART hardware instance.
     */
    HardwareSerial* get_bus();

    // --- Interrupt Features ---
    /**
     * @brief Enables the RX interrupt and assigns a callback function.
     * @param callback Function to execute when a byte is received.
     */
    void set_rx_interrupt(void (*callback)(uint8_t));

    // Internal handler meant to be called by the hardware IRQ (Do not call manually)
    void _handle_rx_irq();

private:
    HardwareSerial uart;
    uint8_t uart_num;
    uint8_t rx_pin;
    uint8_t tx_pin;
    uint32_t baud;

    // Pointer to the user-defined callback function
    void (*rx_callback)(uint8_t) = nullptr;

    // Static array to map hardware IRQs to the correct class instance (0, 1, 2 for ESP32)
    static UARTController* instances[3];

    // Static IRQ wrappers required by the ESP32 Arduino hardware vectors
    static void uart0_isr_wrapper();
    static void uart1_isr_wrapper();
    static void uart2_isr_wrapper();
};