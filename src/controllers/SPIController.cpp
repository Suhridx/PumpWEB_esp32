#include "controllers/SPIController.hpp"
#include "Log.h"

// ============================================================================
// Construction / Initialization
// ============================================================================

SPIController::SPIController(uint8_t cs_pin, uint32_t clock_hz, uint8_t spi_bus, uint8_t spi_mode)
    : spi(spi_bus), cs_pin(cs_pin), clock_hz(clock_hz), spi_bus(spi_bus), spi_mode(spi_mode),
      settings(clock_hz, MSBFIRST, spi_mode) {
    // Hardware is left untouched here so the object is safe to create globally,
    // before the Arduino core has finished starting up. begin() does the real work.
}

bool SPIController::begin() {
    if(started) return true;

    // CS idles high. Set it before the peripheral starts so no device on the bus
    // ever sees a half-asserted select line during initialization.
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);

    // -1 as the SS pin: chip select is driven manually so several devices can share
    // the bus, and so multi-phase commands can hold CS low across their phases.
    spi.begin(SPIConfig::SCK_PIN, SPIConfig::MISO_PIN, SPIConfig::MOSI_PIN, -1);

    started = true;
    LOGF("[SPI] Bus up. SCK: %d, MISO: %d, MOSI: %d, CS: %d @ %u Hz\n",
        SPIConfig::SCK_PIN, SPIConfig::MISO_PIN, SPIConfig::MOSI_PIN, cs_pin, clock_hz);

    return true;
}

void SPIController::end() {
    if(!started) return;

    spi.end();
    digitalWrite(cs_pin, HIGH);
    started = false;
}

// ============================================================================
// Transaction Control
// ============================================================================

void SPIController::select() {
    spi.beginTransaction(settings);
    digitalWrite(cs_pin, LOW);
}

void SPIController::deselect() {
    digitalWrite(cs_pin, HIGH);
    spi.endTransaction();
}

// ============================================================================
// Raw Transfers
// ============================================================================

uint8_t SPIController::transfer(uint8_t data) {
    return spi.transfer(data);
}

uint16_t SPIController::transfer16(uint16_t data) {
    return spi.transfer16(data);
}

void SPIController::transfer_bytes(const uint8_t* tx, uint8_t* rx, size_t len) {
    if(len == 0) return;
    // The driver clocks out dummy bytes when tx is null, and drops the
    // incoming bytes when rx is null.
    spi.transferBytes(tx, rx, len);
}

void SPIController::send_bytes(const uint8_t* data, size_t len) {
    if(data == nullptr || len == 0) return;
    spi.writeBytes(data, len);
}

void SPIController::get_bytes(uint8_t* buffer, size_t len) {
    if(buffer == nullptr || len == 0) return;
    spi.transferBytes(nullptr, buffer, len);
}

// ============================================================================
// Complete Transactions
// ============================================================================

void SPIController::send_command(uint8_t opcode) {
    select();
    spi.transfer(opcode);
    deselect();
}

void SPIController::send_then_get(const uint8_t* tx, size_t tx_len, uint8_t* rx, size_t rx_len) {
    select();
    send_bytes(tx, tx_len);
    get_bytes(rx, rx_len);
    deselect();
}

void SPIController::send_then_send(const uint8_t* header, size_t header_len,
                                   const uint8_t* payload, size_t payload_len) {
    select();
    send_bytes(header, header_len);
    send_bytes(payload, payload_len);
    deselect();
}

// ============================================================================
// Bus Configuration
// ============================================================================

void SPIController::set_clock(uint32_t new_clock_hz) {
    clock_hz = new_clock_hz;
    settings = SPISettings(clock_hz, MSBFIRST, spi_mode);
}

void SPIController::set_mode(uint8_t new_spi_mode) {
    spi_mode = new_spi_mode;
    settings = SPISettings(clock_hz, MSBFIRST, spi_mode);
}

uint32_t SPIController::get_clock() const {
    return clock_hz;
}

uint8_t SPIController::get_cs_pin() const {
    return cs_pin;
}

bool SPIController::is_started() const {
    return started;
}

SPIClass* SPIController::get_bus() {
    return &spi;
}
