#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <cstdint>
#include "SystemConstants.h"

/**
 * @brief Low level hardware SPI bus controller.
 *
 * This layer knows nothing about flash chips, displays or sensors. It only moves
 * bytes and owns the chip select line and the transaction settings for one device.
 * Anything device specific (opcodes, addressing, timing, geometry) belongs one
 * level up in hardwares/, the same way PicoHandler sits on top of UARTController.
 *
 * Transaction discipline: select() opens a transaction and pulls CS low, deselect()
 * releases it. Every multi-phase command must stay inside a single select/deselect
 * pair, otherwise the target device sees the command as aborted.
 */
class SPIController {
public:
    /**
     * @brief Stores the bus configuration. No hardware is touched until begin().
     * @param cs_pin GPIO used as chip select for this device.
     * @param clock_hz SPI clock in Hz.
     * @param spi_bus Hardware SPI peripheral (FSPI or HSPI on the ESP32-S3).
     * @param spi_mode Clock polarity and phase, normally SPI_MODE0.
     */
    SPIController(uint8_t cs_pin = SPIConfig::FLASH_CS_PIN,
                  uint32_t clock_hz = SPIConfig::DEFAULT_CLOCK_HZ,
                  uint8_t spi_bus = FSPI,
                  uint8_t spi_mode = SPI_MODE0);

    /**
     * @brief Configures the CS pin and starts the SPI peripheral on the configured pins.
     * @return true once the bus is up.
     */
    bool begin();

    /**
     * @brief Shuts the SPI peripheral down and releases its pins.
     */
    void end();

    // =====================================================================
    // Transaction control
    // =====================================================================

    /**
     * @brief Opens an SPI transaction and pulls CS low.
     */
    void select();

    /**
     * @brief Releases CS and closes the SPI transaction.
     */
    void deselect();

    // =====================================================================
    // Raw transfers (must sit between select() and deselect())
    // =====================================================================

    /**
     * @brief Clocks out one byte and returns whatever arrived on MISO.
     */
    uint8_t transfer(uint8_t data);

    /**
     * @brief Clocks out two bytes, MSB first, and returns the 16-bit response.
     */
    uint16_t transfer16(uint16_t data);

    /**
     * @brief Full duplex transfer. Sends tx while capturing the response into rx.
     * @param tx Bytes to send, or nullptr to clock out dummy bytes.
     * @param rx Buffer for the response, or nullptr to discard it.
     * @param len Number of bytes to clock.
     */
    void transfer_bytes(const uint8_t* tx, uint8_t* rx, size_t len);

    /**
     * @brief Clocks out a buffer and discards everything coming back.
     */
    void send_bytes(const uint8_t* data, size_t len);

    /**
     * @brief Clocks in a buffer, sending dummy bytes on MOSI.
     */
    void get_bytes(uint8_t* buffer, size_t len);

    // =====================================================================
    // Complete transactions (handle select/deselect internally)
    // =====================================================================

    /**
     * @brief Sends a single byte as a self-contained transaction.
     */
    void send_command(uint8_t opcode);

    /**
     * @brief Sends a header, then reads a response, all inside one CS assertion.
     * @param tx Bytes to send first, normally an opcode plus an address. May be nullptr if tx_len is 0.
     * @param tx_len Number of bytes to send.
     * @param rx Buffer for the response. May be nullptr if rx_len is 0.
     * @param rx_len Number of bytes to read back.
     */
    void send_then_get(const uint8_t* tx, size_t tx_len, uint8_t* rx, size_t rx_len);

    /**
     * @brief Sends a header followed by a payload inside one CS assertion.
     * @note This is the shape every write style command takes: opcode and address
     *       first, then the data, with CS held low across both halves.
     */
    void send_then_send(const uint8_t* header, size_t header_len,
                        const uint8_t* payload, size_t payload_len);

    // =====================================================================
    // Bus configuration
    // =====================================================================

    /**
     * @brief Changes the clock used by subsequent transactions.
     */
    void set_clock(uint32_t new_clock_hz);

    /**
     * @brief Changes the clock polarity and phase used by subsequent transactions.
     */
    void set_mode(uint8_t new_spi_mode);

    /**
     * @brief Current bus clock in Hz.
     */
    uint32_t get_clock() const;

    /**
     * @brief The chip select GPIO this instance drives.
     */
    uint8_t get_cs_pin() const;

    /**
     * @brief True once begin() has brought the peripheral up.
     */
    bool is_started() const;

    /**
     * @brief Returns the underlying SPI instance for advanced or shared use.
     */
    SPIClass* get_bus();

private:
    SPIClass spi;
    uint8_t cs_pin;
    uint32_t clock_hz;
    uint8_t spi_bus;
    uint8_t spi_mode;

    SPISettings settings;
    bool started = false;
};
