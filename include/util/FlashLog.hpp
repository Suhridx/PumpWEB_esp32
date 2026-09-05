// include/util/FlashLog.hpp
#pragma once

#include <Arduino.h>
#include <functional>
#include "hardwares/W25Q64Handler.hpp"

// --- On-Flash Layout ---
//
// A log is a fixed span of sectors used as a ring. Records are appended in order, the
// ring wraps at the end, and the oldest sector is erased just ahead of the write head.
// There is no directory and no metadata to rewrite, which is what makes this both fast
// and robust: an append is a single page program, and a power cut can only ever damage
// the record being written, never anything already stored.
namespace FlashLogFS {
    constexpr uint32_t SECTOR_MAGIC = 0x50574C47; // "PWLG"
    constexpr uint16_t RECORD_MAGIC = 0xA55A;     // Erased flash reads 0xFFFF, so anything else marks a used slot

    // One slot is exactly one flash page, so every append is a single page program that
    // can never straddle a page boundary. A W25Q64 wraps a program within its page
    // rather than spilling into the next one, so this removes a whole class of bug.
    constexpr uint32_t SLOT_SIZE = 256;

    // The first page of every sector holds that sector's header, the rest are slots.
    constexpr uint32_t SECTOR_HEADER_PAGE = 256;
    constexpr uint8_t SLOTS_PER_SECTOR = (W25Q64::SECTOR_SIZE - SECTOR_HEADER_PAGE) / SLOT_SIZE; // 15

    constexpr uint16_t RECORD_HEADER_SIZE = 12;
    constexpr uint16_t MAX_PAYLOAD = SLOT_SIZE - RECORD_HEADER_SIZE; // 244

    constexpr uint16_t TIME_UNKNOWN = 0xFFFF;
} // namespace FlashLogFS

// Written to the first bytes of a sector when it is claimed. The highest sequence found
// across the region identifies the head sector at mount time.
struct __attribute__((packed)) FlashLogSectorHeader {
    uint32_t magic;
    uint32_t sequence;
    uint32_t reserved0;
    uint32_t reserved1;
};

// Precedes the payload in every slot.
struct __attribute__((packed)) FlashLogRecord {
    uint16_t magic;
    uint16_t mins_from_midnight; // From the Pico RTC, TIME_UNKNOWN until the first sync
    uint32_t uptime_ms;          // millis() at the append, orders records within a minute
    uint16_t length;             // Payload bytes following this header
    uint16_t crc;                // CRC16-CCITT over the payload
};

static_assert(sizeof(FlashLogRecord) == FlashLogFS::RECORD_HEADER_SIZE,
    "FlashLogRecord must stay 12 bytes so a slot holds exactly one page");

// One stored entry, as handed to a visitor
struct FlashLogEntry {
    uint16_t mins_from_midnight;
    uint32_t uptime_ms;
    const uint8_t* data;
    uint16_t length;
};

using FlashLogVisitor = std::function<void(const FlashLogEntry&)>;

/**
 * @brief Append only circular log on a span of W25Q64 sectors.
 *
 * Built for the one thing a filesystem is bad at: frequent appends to raw NOR flash. A
 * general file system rewrites directory or metadata blocks on every append, wearing the
 * same few sectors and risking the whole volume on a power cut. A ring writes each record
 * once, spreads wear evenly across the region by construction, and has no metadata that a
 * torn write could corrupt.
 *
 * Two instances are expected, one per log stream, over separate sector spans.
 *
 * Timing: an append is one page program, roughly 0.7 ms. Once every SLOTS_PER_SECTOR
 * records the ring rolls into a fresh sector and that append also pays a sector erase,
 * roughly 45 ms. Nothing else is required of the caller: append() is the whole API.
 */
class FlashLog {
public:
    /**
     * @brief Binds the log to a sector span. Nothing touches the chip until begin().
     * @param flash_ref    The chip driver.
     * @param start_sector First sector of the region, absolute on the chip.
     * @param sector_count How many sectors the ring spans.
     */
    FlashLog(W25Q64Handler& flash_ref, uint32_t start_sector, uint32_t sector_count);

    /**
     * @brief Finds the write head, formatting the region if it is blank.
     *
     * Reads one header per sector to locate the highest sequence number, then scans that
     * sector for the first free slot. Costs a few milliseconds.
     */
    bool begin();

    bool is_mounted() const;

    /**
     * @brief Appends one record. Payloads longer than MAX_PAYLOAD are truncated.
     * @return true if the record reached the chip.
     */
    bool append(const uint8_t* data, size_t len);
    bool append(const char* text);

    /**
     * @brief Supplies the timestamp stamped into subsequent records.
     *
     * Call this whenever a MSG_TYPE_ROUTINE_TIME packet arrives from the Pico. Until the
     * first call, records carry TIME_UNKNOWN and are ordered by uptime alone.
     */
    void set_time(uint16_t mins_from_midnight);

    /**
     * @brief Walks every stored record, oldest surviving first.
     * @return How many valid records were visited.
     */
    size_t for_each(FlashLogVisitor visitor) const;

    /**
     * @brief Erases the whole region and restarts the ring.
     * @note Blocking, and slow: this erases the entire span, seconds rather than
     *       milliseconds. Intended for maintenance, not normal operation.
     */
    bool clear();

    uint32_t capacity_records() const;

private:
    W25Q64Handler& flash;
    uint32_t start_sector;
    uint32_t sector_count;

    bool mounted = false;
    uint32_t head_index = 0;    // Sector holding the write head, relative to start_sector
    uint32_t head_sequence = 0; // Sequence stamped on that sector
    uint8_t head_slot = 0;      // Next free slot within it

    uint16_t current_mins = FlashLogFS::TIME_UNKNOWN;

    uint32_t sector_address(uint32_t index) const;
    uint32_t slot_address(uint32_t index, uint8_t slot) const;

    bool read_sector_header(uint32_t index, FlashLogSectorHeader& out) const;
    bool claim_sector(uint32_t index, uint32_t sequence);
    bool advance_head();

    static uint16_t crc16(const uint8_t* data, size_t len);
};
