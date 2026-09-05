#pragma once

#include "controllers/SPIController.hpp"
#include <cstdint>

// --- W25Q64 Device Definitions (Winbond 64 Mbit / 8 MB SPI NOR Flash) ---
namespace W25Q64 {
    // --- Identity ---
    constexpr uint8_t MANUFACTURER_WINBOND = 0xEF;
    constexpr uint8_t MEMORY_TYPE_Q = 0x40;    // W25Q family in standard SPI mode
    constexpr uint8_t CAPACITY_64MBIT = 0x17;  // 2^0x17 = 8 MB
    constexpr uint32_t EXPECTED_JEDEC_ID = 0xEF4017;

    // --- Geometry ---
    // Nominal figures for the expected part. The driver reads the real size off the
    // chip via detect_capacity(), so these are documentation, not the source of truth.
    constexpr uint32_t CAPACITY_BYTES = 8UL * 1024 * 1024; // 8 MB
    constexpr uint16_t PAGE_SIZE = 256;                    // Smallest programmable unit
    constexpr uint32_t SECTOR_SIZE = 4096;                 // Smallest erasable unit
    constexpr uint32_t BLOCK_SIZE = 65536;                 // 64 KB erase block
    constexpr uint32_t PAGE_COUNT = 32768;
    constexpr uint32_t SECTOR_COUNT = 2048;
    constexpr uint32_t BLOCK_COUNT = 128;

    // An erased NOR cell always reads back as 0xFF
    constexpr uint8_t ERASED_BYTE = 0xFF;

    // --- Opcodes ---
    constexpr uint8_t CMD_JEDEC_ID = 0x9F;
    constexpr uint8_t CMD_UNIQUE_ID = 0x4B;

    constexpr uint8_t CMD_READ_DATA = 0x03; // Capped near 50 MHz
    constexpr uint8_t CMD_FAST_READ = 0x0B; // Any clock, costs one dummy byte

    constexpr uint8_t CMD_WRITE_ENABLE = 0x06;
    constexpr uint8_t CMD_WRITE_DISABLE = 0x04;
    constexpr uint8_t CMD_VOLATILE_SR_WRITE_ENABLE = 0x50;
    constexpr uint8_t CMD_PAGE_PROGRAM = 0x02;

    constexpr uint8_t CMD_SECTOR_ERASE = 0x20; // 4 KB
    constexpr uint8_t CMD_BLOCK_ERASE_32K = 0x52;
    constexpr uint8_t CMD_BLOCK_ERASE_64K = 0xD8;
    constexpr uint8_t CMD_CHIP_ERASE = 0xC7;

    constexpr uint8_t CMD_READ_STATUS1 = 0x05;
    constexpr uint8_t CMD_READ_STATUS2 = 0x35;
    constexpr uint8_t CMD_WRITE_STATUS1 = 0x01;

    constexpr uint8_t CMD_POWER_DOWN = 0xB9;
    constexpr uint8_t CMD_RELEASE_POWER_DOWN = 0xAB;
    constexpr uint8_t CMD_ENABLE_RESET = 0x66;
    constexpr uint8_t CMD_RESET_DEVICE = 0x99;

    // --- Status Register 1 bit masks ---
    constexpr uint8_t SR1_BUSY = 0x01; // 1 = erase or program still running
    constexpr uint8_t SR1_WEL = 0x02;  // 1 = write enable latch is set
    constexpr uint8_t SR1_BP_MASK = 0x1C; // Block protect bits BP0..BP2
    constexpr uint8_t SR1_SRP0 = 0x80;

    // --- Datasheet worst-case timings, in milliseconds ---
    constexpr uint32_t TIMEOUT_PAGE_MS = 10;
    constexpr uint32_t TIMEOUT_SECTOR_MS = 500;
    constexpr uint32_t TIMEOUT_BLOCK_MS = 2000;
    constexpr uint32_t TIMEOUT_CHIP_MS = 120000; // Up to 100 s on a W25Q64
} // namespace W25Q64


/**
 * @brief Middle layer driver for the W25Q64 SPI NOR flash chip.
 *
 * Owns everything device specific: opcodes, 24-bit addressing, page boundary
 * splitting, the write enable latch, erase granularity and busy polling. It talks
 * to the outside world purely through SPIController, exactly the way PicoHandler
 * sits on top of UARTController.
 *
 * The one rule this layer cannot hide: NOR flash programming can only clear bits,
 * turning 1 into 0. A region must be erased before it is written, otherwise the
 * result is the bitwise AND of the old and the new data. Use erase_sector(),
 * erase_range() or the file manager one level up to handle that.
 */
class W25Q64Handler {
public:
    W25Q64Handler(SPIController& spi_ref);

    /**
     * @brief Brings up the bus if needed, wakes the chip and verifies its JEDEC ID.
     * @return true if a W25Q64 answered.
     */
    bool begin();

    /**
     * @brief True once begin() has positively identified the chip.
     */
    bool is_detected() const;

    // =====================================================================
    // Identification and status
    // =====================================================================

    /**
     * @brief Issues 0x9F and returns the raw 24-bit JEDEC ID (0xEF4017 for a W25Q64).
     */
    uint32_t read_jedec_id();

    /**
     * @brief Works out the real size of the connected chip from its JEDEC density code.
     * @note Reads the size off the silicon instead of trusting the W25Q64 constants, so
     *       a W25Q32 or W25Q128 fitted by mistake is reported at its true size. Also
     *       refreshes the value get_capacity() hands back.
     * @return Size in bytes, or 0 if the chip did not answer or reported a density code
     *         outside the range a 32-bit byte count can express.
     */
    uint32_t detect_capacity();

    /**
     * @brief Reads the factory programmed 64-bit unique serial number.
     */
    uint64_t read_unique_id();

    /**
     * @brief Reads Status Register 1 (busy, write enable latch, block protect).
     */
    uint8_t read_status1();

    /**
     * @brief Reads Status Register 2 (quad enable, security register locks).
     */
    uint8_t read_status2();

    /**
     * @brief True while an erase or program cycle is still running.
     */
    bool is_busy();

    /**
     * @brief Blocks until the chip clears its BUSY flag, yielding to FreeRTOS meanwhile.
     * @param timeout_ms Give up after this many milliseconds.
     * @return false on timeout.
     */
    bool wait_ready(uint32_t timeout_ms);

    /**
     * @brief Clears the block protect bits so the whole chip is writable.
     * @note Uses the volatile status register write, so the change is lost on power
     *       cycle and the non-volatile register is never worn out. Only needed if a
     *       chip arrives with protection already enabled.
     */
    bool unlock();

    // =====================================================================
    // Read
    // =====================================================================

    /**
     * @brief Reads any number of bytes starting at a flash address.
     * @return false if the chip is missing or the range runs past the end.
     */
    bool read(uint32_t addr, uint8_t* buffer, size_t len);

    /**
     * @brief Convenience single byte read. Returns 0xFF on failure.
     */
    uint8_t read_byte(uint32_t addr);

    /**
     * @brief Checks that a range reads back as 0xFF, meaning it is ready to program.
     */
    bool is_erased(uint32_t addr, size_t len);

    // =====================================================================
    // Write
    // =====================================================================

    /**
     * @brief Writes any number of bytes, splitting the data across page boundaries.
     * @note The target range must already be erased. See the class note above.
     * @return false on a missing chip, an out of range address, or a timeout.
     */
    bool write(uint32_t addr, const uint8_t* data, size_t len);

    /**
     * @brief Convenience single byte write.
     */
    bool write_byte(uint32_t addr, uint8_t value);

    // =====================================================================
    // Erase
    // =====================================================================

    /**
     * @brief Erases the 4 KB sector containing addr.
     */
    bool erase_sector(uint32_t addr);

    /**
     * @brief Erases the 64 KB block containing addr.
     */
    bool erase_block(uint32_t addr);

    /**
     * @brief Erases every 4 KB sector touched by the range addr to addr + len.
     * @note Erasing is sector granular, so neighbouring data inside those sectors is lost too.
     */
    bool erase_range(uint32_t addr, size_t len);

    /**
     * @brief Erases the whole chip. Blocking, and can take up to a minute or two.
     */
    bool erase_chip();

    // =====================================================================
    // Power management
    // =====================================================================

    /**
     * @brief Puts the chip into deep power down, roughly 1 uA.
     */
    void power_down();

    /**
     * @brief Wakes the chip from deep power down.
     */
    void wake_up();

    /**
     * @brief Issues the 0x66 / 0x99 software reset sequence.
     */
    void reset_device();

    // =====================================================================
    // Geometry
    // =====================================================================

    /**
     * @brief Size of the chip that is actually connected, in bytes.
     * @note Comes from detect_capacity(), which begin() runs, so this reflects the
     *       silicon on the board rather than the W25Q64 constants in the header.
     *       Returns 0 if the chip never answered or reported a density this driver
     *       cannot express.
     */
    uint32_t get_capacity() const;

    /**
     * @brief Number of 4 KB sectors on the connected chip.
     */
    uint32_t sector_count() const;

    // --- Address arithmetic. Sector size is 4 KB across the whole W25Q family,
    // --- so these stay compile time constants and are safe to call anywhere.

    /** @brief Index of the sector an address falls in. */
    static uint32_t sector_of(uint32_t addr) { return addr / W25Q64::SECTOR_SIZE; }

    /** @brief First address of the sector an address falls in. */
    static uint32_t sector_base(uint32_t addr) { return addr & ~(W25Q64::SECTOR_SIZE - 1); }

    /** @brief First address of a sector, by index. */
    static uint32_t sector_address(uint32_t sector_index) { return sector_index * W25Q64::SECTOR_SIZE; }

    /** @brief Number of whole sectors needed to hold len bytes. */
    static uint32_t sectors_needed(size_t len) {
        return (len + W25Q64::SECTOR_SIZE - 1) / W25Q64::SECTOR_SIZE;
    }

private:
    SPIController& spi;
    bool detected = false;
    uint32_t detected_capacity = 0; // Filled in by detect_capacity(), not a compile time value

    // Builds an opcode plus 24-bit big endian address into a 4 byte header
    void build_header(uint8_t* header, uint8_t opcode, uint32_t addr);

    // Sets the write enable latch and verifies it actually took
    bool write_enable();

    // Programs up to one 256 byte page without crossing its boundary
    bool write_page(uint32_t addr, const uint8_t* data, size_t len);

    // Issues an erase opcode against one address and waits for completion
    bool erase_at(uint8_t opcode, uint32_t addr, uint32_t timeout_ms);

    // Rejects ranges that fall outside the chip, and calls out a missing chip
    bool check_range(uint32_t addr, size_t len);
};
