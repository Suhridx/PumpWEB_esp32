#include "util/FlashLog.hpp"
#include "Log.h"

FlashLog::FlashLog(W25Q64Handler& flash_ref, uint32_t start_sector, uint32_t sector_count)
    : flash(flash_ref), start_sector(start_sector), sector_count(sector_count) {
    // Nothing touches the chip here. begin() does the mounting.
}

// ============================================================================
// Addressing
// ============================================================================

uint32_t FlashLog::sector_address(uint32_t index) const {
    return (start_sector + index) * W25Q64::SECTOR_SIZE;
}

uint32_t FlashLog::slot_address(uint32_t index, uint8_t slot) const {
    return sector_address(index) + FlashLogFS::SECTOR_HEADER_PAGE +
        ((uint32_t)slot * FlashLogFS::SLOT_SIZE);
}

// ============================================================================
// Mounting
// ============================================================================

bool FlashLog::read_sector_header(uint32_t index, FlashLogSectorHeader& out) const {
    if(!flash.read(sector_address(index), (uint8_t*)&out, sizeof(out))) return false;
    return out.magic == FlashLogFS::SECTOR_MAGIC;
}

bool FlashLog::claim_sector(uint32_t index, uint32_t sequence) {
    if(!flash.erase_sector(sector_address(index))) return false;

    FlashLogSectorHeader hdr;
    hdr.magic = FlashLogFS::SECTOR_MAGIC;
    hdr.sequence = sequence;
    hdr.reserved0 = 0xFFFFFFFF;
    hdr.reserved1 = 0xFFFFFFFF;

    if(!flash.write(sector_address(index), (const uint8_t*)&hdr, sizeof(hdr))) return false;

    head_index = index;
    head_sequence = sequence;
    head_slot = 0;
    return true;
}

bool FlashLog::begin() {
    mounted = false;

    if(!flash.is_detected()) {
        LOGLN("[FLASHLOG-ERR] Chip not detected. Log unavailable.");
        return false;
    }
    if(sector_count < 2) {
        LOGLN("[FLASHLOG-ERR] A ring needs at least two sectors.");
        return false;
    }

    // 1. The sector carrying the highest sequence number is the head. One small read per
    //    sector, so even a 512 sector region costs only a few milliseconds.
    bool found = false;
    uint32_t best_index = 0;
    uint32_t best_sequence = 0;

    for(uint32_t i = 0; i < sector_count; i++) {
        FlashLogSectorHeader hdr;
        if(!read_sector_header(i, hdr)) continue;

        if(!found || hdr.sequence > best_sequence) {
            found = true;
            best_index = i;
            best_sequence = hdr.sequence;
        }
    }

    // 2. A blank region just gets its first sector claimed.
    if(!found) {
        if(!claim_sector(0, 1)) {
            LOGLN("[FLASHLOG-ERR] Failed to format the region.");
            return false;
        }
        mounted = true;
        LOGF("[FLASHLOG] Formatted %lu sectors at %lu. Empty.\n",
            (unsigned long)sector_count, (unsigned long)start_sector);
        return true;
    }

    head_index = best_index;
    head_sequence = best_sequence;

    // 3. Records fill a sector in order, so the first slot still erased is the write head.
    //    A torn record left by a power cut fails its CRC when read back but still occupies
    //    its slot, which is why this looks at the magic rather than the CRC.
    head_slot = FlashLogFS::SLOTS_PER_SECTOR;
    for(uint8_t s = 0; s < FlashLogFS::SLOTS_PER_SECTOR; s++) {
        uint16_t magic = 0;
        if(!flash.read(slot_address(head_index, s), (uint8_t*)&magic, sizeof(magic))) break;

        if(magic != FlashLogFS::RECORD_MAGIC) {
            head_slot = s;
            break;
        }
    }

    mounted = true;
    LOGF("[FLASHLOG] Mounted %lu sectors at %lu. Head sector %lu, slot %u, seq %lu.\n",
        (unsigned long)sector_count, (unsigned long)start_sector,
        (unsigned long)head_index, (unsigned)head_slot, (unsigned long)head_sequence);
    return true;
}

bool FlashLog::is_mounted() const {
    return mounted;
}

// ============================================================================
// Writing
// ============================================================================

bool FlashLog::advance_head() {
    // The only slow path: this append also pays a sector erase, roughly 45 ms, once every
    // SLOTS_PER_SECTOR records. Deliberately left inline rather than hoisted into a
    // housekeeping call, so append() remains the entire API.
    uint32_t next = (head_index + 1) % sector_count;
    return claim_sector(next, head_sequence + 1);
}

bool FlashLog::append(const uint8_t* data, size_t len) {
    if(!mounted || data == nullptr || len == 0) return false;

    if(len > FlashLogFS::MAX_PAYLOAD) len = FlashLogFS::MAX_PAYLOAD;

    if(head_slot >= FlashLogFS::SLOTS_PER_SECTOR) {
        if(!advance_head()) return false;
    }

    // Header and payload go out as one buffer, so the whole record is a single page
    // program. A partial write can therefore only ever truncate this record, never
    // disturb one already stored.
    uint8_t buffer[FlashLogFS::SLOT_SIZE];
    FlashLogRecord* rec = (FlashLogRecord*)buffer;

    rec->magic = FlashLogFS::RECORD_MAGIC;
    rec->mins_from_midnight = current_mins;
    rec->uptime_ms = millis();
    rec->length = (uint16_t)len;
    rec->crc = crc16(data, len);

    memcpy(buffer + FlashLogFS::RECORD_HEADER_SIZE, data, len);

    if(!flash.write(slot_address(head_index, head_slot), buffer,
        FlashLogFS::RECORD_HEADER_SIZE + len)) {
        return false;
    }

    head_slot++;
    return true;
}

bool FlashLog::append(const char* text) {
    if(text == nullptr) return false;
    return append((const uint8_t*)text, strlen(text));
}

void FlashLog::set_time(uint16_t mins_from_midnight) {
    current_mins = mins_from_midnight;
}

// ============================================================================
// Reading
// ============================================================================

size_t FlashLog::for_each(FlashLogVisitor visitor) const {
    if(!mounted || !visitor) return 0;

    size_t visited = 0;
    uint8_t buffer[FlashLogFS::SLOT_SIZE];

    // Sequence numbers rise by one per claim, so the sector after the head is the oldest
    // surviving one. Walking the ring from there yields records in chronological order.
    for(uint32_t n = 1; n <= sector_count; n++) {
        uint32_t index = (head_index + n) % sector_count;

        FlashLogSectorHeader hdr;
        if(!flash.read(sector_address(index), (uint8_t*)&hdr, sizeof(hdr))) continue;
        if(hdr.magic != FlashLogFS::SECTOR_MAGIC) continue; // Erased, or never claimed

        for(uint8_t s = 0; s < FlashLogFS::SLOTS_PER_SECTOR; s++) {
            if(!flash.read(slot_address(index, s), buffer, FlashLogFS::SLOT_SIZE)) break;

            const FlashLogRecord* rec = (const FlashLogRecord*)buffer;
            if(rec->magic != FlashLogFS::RECORD_MAGIC) break; // First free slot ends the sector
            if(rec->length == 0 || rec->length > FlashLogFS::MAX_PAYLOAD) continue;

            const uint8_t* payload = buffer + FlashLogFS::RECORD_HEADER_SIZE;
            if(crc16(payload, rec->length) != rec->crc) continue; // Torn or corrupted, skip

            FlashLogEntry entry;
            entry.mins_from_midnight = rec->mins_from_midnight;
            entry.uptime_ms = rec->uptime_ms;
            entry.data = payload;
            entry.length = rec->length;

            visitor(entry);
            visited++;
        }
    }

    return visited;
}

// ============================================================================
// Maintenance
// ============================================================================

bool FlashLog::clear() {
    if(!flash.is_detected()) return false;

    LOGF("[FLASHLOG] Erasing %lu sectors at %lu. This blocks for several seconds.\n",
        (unsigned long)sector_count, (unsigned long)start_sector);

    if(!flash.erase_range(sector_address(0), (size_t)sector_count * W25Q64::SECTOR_SIZE)) {
        LOGLN("[FLASHLOG-ERR] Region erase failed.");
        return false;
    }

    mounted = false;
    head_sequence = 0;
    return begin();
}

uint32_t FlashLog::capacity_records() const {
    return sector_count * FlashLogFS::SLOTS_PER_SECTOR;
}

// ============================================================================
// CRC16-CCITT, over the payload only
// ============================================================================

uint16_t FlashLog::crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;

    for(size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;

        for(uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }

    return crc;
}
