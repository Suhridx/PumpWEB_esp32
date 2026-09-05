#include "hardwares/W25Q64Handler.hpp"
#include "Log.h"

// ============================================================================
// Construction / Initialization
// ============================================================================

W25Q64Handler::W25Q64Handler(SPIController& spi_ref)
    : spi(spi_ref) {
}

bool W25Q64Handler::begin() {
    // Harmless if the bus is already up, so the caller does not have to care
    // about the order in which the drivers on this bus are started.
    if(!spi.is_started()) {
        spi.begin();
    }

    // A chip coming out of deep power down ignores everything until it is released
    wake_up();

    uint32_t jedec = read_jedec_id();

    // An absent or mis-wired chip reads back as all zeros or all ones
    if(jedec == 0x000000 || jedec == 0xFFFFFF) {
        detected = false;
        LOGF("[FLASH-ERR] No chip responding on CS %d (JEDEC: 0x%06X). Check wiring.\n",
            spi.get_cs_pin(), jedec);
        return false;
    }

    if(jedec != W25Q64::EXPECTED_JEDEC_ID) {
        // Still usable if it is a compatible part, so warn instead of refusing.
        // The real size is read back from the chip a few lines below either way.
        LOGF("[FLASH-WARN] Expected W25Q64 (0x%06X) but found 0x%06X. Continuing.\n",
            W25Q64::EXPECTED_JEDEC_ID, jedec);
    }

    detected = true;

    // Ask the chip how big it is rather than assuming the part on the schematic
    if(detect_capacity() == 0) {
        detected = false;
        LOGLN("[FLASH-ERR] Could not work out the chip size. Refusing to use it, "
            "since every bounds check depends on knowing the real capacity.");
        return false;
    }

    uint8_t status = read_status1();
    LOGF("[FLASH] Online. JEDEC: 0x%06X, %u MB (%u sectors), Status: 0x%02X\n",
        jedec, detected_capacity / (1024 * 1024), sector_count(), status);

    if(status & W25Q64::SR1_BP_MASK) {
        LOGLN("[FLASH-WARN] Block protect bits are set. Call unlock() before writing.");
    }

    return true;
}

uint32_t W25Q64Handler::detect_capacity() {
    uint32_t jedec = read_jedec_id();

    if(jedec == 0x000000 || jedec == 0xFFFFFF) {
        detected_capacity = 0;
        return 0;
    }

    // Third ID byte is the density code. Across the JEDEC SPI NOR families it is a
    // power of two exponent for the size in bytes: 0x16 is 4 MB, 0x17 is 8 MB (the
    // W25Q64), 0x18 is 16 MB. Below 0x10 it is not a density, and 0x20 upwards would
    // overflow a 32-bit byte count.
    uint8_t density_code = jedec & 0xFF;

    if(density_code < 0x10 || density_code > 0x1F) {
        LOGF("[FLASH-ERR] Density code 0x%02X is not one this driver can decode.\n",
            density_code);
        detected_capacity = 0;
        return 0;
    }

    detected_capacity = (uint32_t)1 << density_code;

    // build_header() only emits 24-bit addresses, so anything past 16 MB is out of
    // reach without the 4-byte address mode a W25Q64 never needs.
    if(detected_capacity > 16UL * 1024 * 1024) {
        LOGF("[FLASH-WARN] Chip reports %u MB but this driver addresses 24 bits. "
            "Limiting use to the first 16 MB.\n", detected_capacity / (1024 * 1024));
        detected_capacity = 16UL * 1024 * 1024;
    }

    return detected_capacity;
}

uint32_t W25Q64Handler::get_capacity() const {
    return detected_capacity;
}

uint32_t W25Q64Handler::sector_count() const {
    return detected_capacity / W25Q64::SECTOR_SIZE;
}

bool W25Q64Handler::is_detected() const {
    return detected;
}

void W25Q64Handler::build_header(uint8_t* header, uint8_t opcode, uint32_t addr) {
    // 8 MB fits comfortably inside 24 bits, so the W25Q64 never needs 4 byte addressing
    header[0] = opcode;
    header[1] = (addr >> 16) & 0xFF;
    header[2] = (addr >> 8) & 0xFF;
    header[3] = addr & 0xFF;
}

// ============================================================================
// Identification and Status
// ============================================================================

uint32_t W25Q64Handler::read_jedec_id() {
    uint8_t cmd = W25Q64::CMD_JEDEC_ID;
    uint8_t id[3] = {0, 0, 0};

    spi.send_then_get(&cmd, 1, id, 3);

    return ((uint32_t)id[0] << 16) | ((uint32_t)id[1] << 8) | id[2];
}

uint64_t W25Q64Handler::read_unique_id() {
    // 0x4B takes four dummy bytes before the 8 byte serial number starts
    uint8_t cmd[5] = {W25Q64::CMD_UNIQUE_ID, 0x00, 0x00, 0x00, 0x00};
    uint8_t id[8] = {0};

    spi.send_then_get(cmd, sizeof(cmd), id, sizeof(id));

    uint64_t unique = 0;
    for(uint8_t i = 0; i < 8; i++) {
        unique = (unique << 8) | id[i];
    }
    return unique;
}

uint8_t W25Q64Handler::read_status1() {
    uint8_t cmd = W25Q64::CMD_READ_STATUS1;
    uint8_t status = 0;

    spi.send_then_get(&cmd, 1, &status, 1);
    return status;
}

uint8_t W25Q64Handler::read_status2() {
    uint8_t cmd = W25Q64::CMD_READ_STATUS2;
    uint8_t status = 0;

    spi.send_then_get(&cmd, 1, &status, 1);
    return status;
}

bool W25Q64Handler::is_busy() {
    return (read_status1() & W25Q64::SR1_BUSY) != 0;
}

bool W25Q64Handler::wait_ready(uint32_t timeout_ms) {
    uint32_t start = millis();

    while((millis() - start) < timeout_ms) {
        if(!is_busy()) return true;
        delay(1); // Yields to FreeRTOS so the WiFi and MQTT tasks keep breathing
    }

    LOGF("[FLASH-ERR] Timed out after %u ms waiting for the chip to go idle\n", timeout_ms);
    return false;
}

bool W25Q64Handler::unlock() {
    // The volatile variant leaves the non-volatile status register untouched, so
    // this costs no write endurance and simply reverts on the next power cycle.
    spi.send_command(W25Q64::CMD_VOLATILE_SR_WRITE_ENABLE);

    uint8_t cmd[2] = {W25Q64::CMD_WRITE_STATUS1, 0x00};
    spi.send_then_send(cmd, sizeof(cmd), nullptr, 0);

    if(!wait_ready(W25Q64::TIMEOUT_PAGE_MS)) return false;

    uint8_t status = read_status1();
    if(status & W25Q64::SR1_BP_MASK) {
        LOGF("[FLASH-ERR] Block protect bits still set (Status: 0x%02X). "
            "The WP pin may be held low.\n", status);
        return false;
    }

    LOGLN("[FLASH] Block protection cleared. Chip is writable.");
    return true;
}

bool W25Q64Handler::write_enable() {
    spi.send_command(W25Q64::CMD_WRITE_ENABLE);

    // Confirm the latch took, otherwise the program or erase that follows is a silent no-op
    if((read_status1() & W25Q64::SR1_WEL) == 0) {
        LOGLN("[FLASH-ERR] Write enable latch did not set. Chip may be write protected.");
        return false;
    }
    return true;
}

bool W25Q64Handler::check_range(uint32_t addr, size_t len) {
    if(!detected) {
        LOGLN("[FLASH-ERR] No chip detected. Call begin() first.");
        return false;
    }
    if(len == 0) return false;

    if((uint64_t)addr + len > detected_capacity) {
        LOGF("[FLASH-ERR] Range 0x%06X + %u runs past the end of the chip (%u bytes)\n",
            addr, (uint32_t)len, detected_capacity);
        return false;
    }
    return true;
}

// ============================================================================
// Read
// ============================================================================

bool W25Q64Handler::read(uint32_t addr, uint8_t* buffer, size_t len) {
    if(buffer == nullptr || !check_range(addr, len)) return false;

    // Fast Read works at any clock, unlike 0x03 which caps out near 50 MHz.
    // Byte 4 is the dummy the chip needs to turn its output driver around.
    uint8_t header[5];
    build_header(header, W25Q64::CMD_FAST_READ, addr);
    header[4] = 0x00;

    spi.send_then_get(header, sizeof(header), buffer, len);
    return true;
}

uint8_t W25Q64Handler::read_byte(uint32_t addr) {
    uint8_t value = W25Q64::ERASED_BYTE;
    read(addr, &value, 1);
    return value;
}

bool W25Q64Handler::is_erased(uint32_t addr, size_t len) {
    if(!check_range(addr, len)) return false;

    uint8_t chunk[64];
    size_t remaining = len;
    uint32_t cursor = addr;

    while(remaining > 0) {
        size_t to_read = (remaining > sizeof(chunk)) ? sizeof(chunk) : remaining;
        if(!read(cursor, chunk, to_read)) return false;

        for(size_t i = 0; i < to_read; i++) {
            if(chunk[i] != W25Q64::ERASED_BYTE) return false;
        }

        cursor += to_read;
        remaining -= to_read;
    }
    return true;
}

// ============================================================================
// Write
// ============================================================================

bool W25Q64Handler::write_page(uint32_t addr, const uint8_t* data, size_t len) {
    if(!write_enable()) return false;

    uint8_t header[4];
    build_header(header, W25Q64::CMD_PAGE_PROGRAM, addr);

    spi.send_then_send(header, sizeof(header), data, len);

    return wait_ready(W25Q64::TIMEOUT_PAGE_MS);
}

bool W25Q64Handler::write(uint32_t addr, const uint8_t* data, size_t len) {
    if(data == nullptr || !check_range(addr, len)) return false;

    size_t written = 0;

    while(written < len) {
        uint32_t cursor = addr + written;

        // A page program wraps around inside its own page instead of spilling into
        // the next one, so every chunk has to stop at the page boundary.
        uint32_t page_space = W25Q64::PAGE_SIZE - (cursor % W25Q64::PAGE_SIZE);
        size_t chunk = (len - written < page_space) ? (len - written) : page_space;

        if(!write_page(cursor, data + written, chunk)) {
            LOGF("[FLASH-ERR] Page program failed at 0x%06X\n", cursor);
            return false;
        }

        written += chunk;
    }

    return true;
}

bool W25Q64Handler::write_byte(uint32_t addr, uint8_t value) {
    return write(addr, &value, 1);
}

// ============================================================================
// Erase
// ============================================================================

bool W25Q64Handler::erase_at(uint8_t opcode, uint32_t addr, uint32_t timeout_ms) {
    if(!write_enable()) return false;

    uint8_t header[4];
    build_header(header, opcode, addr);

    spi.send_then_send(header, sizeof(header), nullptr, 0);

    return wait_ready(timeout_ms);
}

bool W25Q64Handler::erase_sector(uint32_t addr) {
    if(!check_range(addr, 1)) return false;

    // Snap down to the first address of the sector this address lives in
    return erase_at(W25Q64::CMD_SECTOR_ERASE, sector_base(addr), W25Q64::TIMEOUT_SECTOR_MS);
}

bool W25Q64Handler::erase_block(uint32_t addr) {
    if(!check_range(addr, 1)) return false;

    uint32_t block_addr = addr & ~(W25Q64::BLOCK_SIZE - 1);
    return erase_at(W25Q64::CMD_BLOCK_ERASE_64K, block_addr, W25Q64::TIMEOUT_BLOCK_MS);
}

bool W25Q64Handler::erase_range(uint32_t addr, size_t len) {
    if(!check_range(addr, len)) return false;

    uint32_t first = sector_base(addr);
    uint32_t last = sector_base(addr + len - 1);

    for(uint32_t sector = first; sector <= last; sector += W25Q64::SECTOR_SIZE) {
        if(!erase_at(W25Q64::CMD_SECTOR_ERASE, sector, W25Q64::TIMEOUT_SECTOR_MS)) {
            LOGF("[FLASH-ERR] Sector erase failed at 0x%06X\n", sector);
            return false;
        }
    }

    return true;
}

bool W25Q64Handler::erase_chip() {
    if(!detected) {
        LOGLN("[FLASH-ERR] No chip detected. Call begin() first.");
        return false;
    }

    LOGLN("[FLASH] Full chip erase started. This can take up to a minute...");

    if(!write_enable()) return false;
    spi.send_command(W25Q64::CMD_CHIP_ERASE);

    if(!wait_ready(W25Q64::TIMEOUT_CHIP_MS)) return false;

    LOGLN("[FLASH] Chip erase complete.");
    return true;
}

// ============================================================================
// Power Management
// ============================================================================

void W25Q64Handler::power_down() {
    spi.send_command(W25Q64::CMD_POWER_DOWN);
    delayMicroseconds(5); // tDP
}

void W25Q64Handler::wake_up() {
    spi.send_command(W25Q64::CMD_RELEASE_POWER_DOWN);
    delayMicroseconds(20); // tRES1, the chip ignores everything until this passes
}

void W25Q64Handler::reset_device() {
    spi.send_command(W25Q64::CMD_ENABLE_RESET);
    spi.send_command(W25Q64::CMD_RESET_DEVICE);
    delayMicroseconds(50); // tRST recovery time
}
