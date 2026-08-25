#include "controllers/UARTController.hpp"
#include "SystemConstants.h"
// #include "utils/Logger.hpp" // Assuming you ported your Logger class!

// Initialize static instance pointers to nullptr
UARTController* UARTController::instances[3] = {nullptr, nullptr, nullptr};

UARTController::UARTController(uint8_t uart_bus, uint32_t baudrate)
    : uart(uart_bus), uart_num(uart_bus), baud(baudrate) {

    // Automatically determine the pins based on the chosen hardware bus
    if(uart_num == 1) {
        tx_pin = UARTConfig::UART1_TX_PIN;
        rx_pin = UARTConfig::UART1_RX_PIN;
        instances[1] = this;
    }
    else if(uart_num == 2) {
        tx_pin = UARTConfig::UART2_TX_PIN;
        rx_pin = UARTConfig::UART2_RX_PIN;
        instances[2] = this;
    }
    else {
        // UART 0 (Default USB Debug Pins - usually 43/44 on ESP32-S3)
        tx_pin = 43;
        rx_pin = 44;
        instances[0] = this;
    }

    // Initialize the hardware UART (8 data bits, 1 stop bit, no parity)
    uart.setRxBufferSize(256);
    uart.begin(baud, SERIAL_8N1, rx_pin, tx_pin);

    // ESP32 allows expanding the hardware FIFO buffer - helpful for heavy traffic

    // Logger::info("UART", "Initialized uart%d at %u baud (TX: %d, RX: %d)",uart_num, baud, tx_pin, rx_pin);
}

void UARTController::send_string(const char* str) {
    uart.print(str);
}

void UARTController::send_bytes(const uint8_t* data, size_t len) {
    if(data == nullptr || len == 0) return;
    uart.write(data, len);
}

bool UARTController::is_readable() {
    return uart.available() > 0;
}

uint8_t UARTController::read_byte() {
    return uart.read();
}

HardwareSerial* UARTController::get_bus() {
    return &uart;
}

void UARTController::set_rx_interrupt(void (*callback)(uint8_t)) {
    this->rx_callback = callback;

    // The ESP32 Arduino core uses onReceive() to attach an interrupt function to the UART FIFO
    if(uart_num == 0) {
        uart.onReceive(uart0_isr_wrapper);
    }
    else if(uart_num == 1) {
        uart.onReceive(uart1_isr_wrapper);
    }
    else if(uart_num == 2) {
        uart.onReceive(uart2_isr_wrapper);
    }

    // Logger::info("UART", "RX Interrupt enabled for uart%d", uart_num);
}

void UARTController::_handle_rx_irq() {
    // Drain the hardware FIFO
    while(uart.available() > 0) {
        uint8_t ch = uart.read();
        if(rx_callback != nullptr) {
            rx_callback(ch); // Trigger the user's callback function
        }
    }
}

// ---------------------------------------------------------
// Static IRQ Wrappers
// ---------------------------------------------------------
void UARTController::uart0_isr_wrapper() {
    if(instances[0] != nullptr) {
        instances[0]->_handle_rx_irq();
    }
}

void UARTController::uart1_isr_wrapper() {
    if(instances[1] != nullptr) {
        instances[1]->_handle_rx_irq();
    }
}

void UARTController::uart2_isr_wrapper() {
    if(instances[2] != nullptr) {
        instances[2]->_handle_rx_irq();
    }
}