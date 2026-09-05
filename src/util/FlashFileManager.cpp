#include "util/FlashFileManager.hpp"
#include <ArduinoJson.h>
#include "Log.h"

// ============================================================================
// Construction / Mounting
// ============================================================================

FlashFileManager::FlashFileManager(W25Q64Handler& flash_ref, uint32_t sector_limit)
    : flash(flash_ref), sector_limit(sector_limit) {
    memset(&header, 0, sizeof(header));
    memset(entries, 0, sizeof(entries));
}

bool FlashFileManager::begin() {
    if(!flash.is_detected()) {
        LOGLN("[FS-ERR] Flash chip is not ready. Call W25Q64Handler::begin() first.");
        return false;
    }

    if(load_directory()) {
        mounted = true;
        LOGF("[FS] Mounted. %d files, %u KB used of %u KB.\n",
            file_count(), used_space() / 1024, total_space() / 1024);
        return true;
    }

    // No valid directory in either mirror, so this is a blank or foreign chip
    LOGLN("[FS] No valid file system found. Formatting...");
    return format();
}

bool FlashFileManager::is_mounted() const {
    return mounted;
}

uint32_t FlashFileManager::slot_address(uint8_t slot) {
    uint32_t sector = (slot == 0) ? FlashFS::DIR_SECTOR_A : FlashFS::DIR_SECTOR_B;
    return sector * W25Q64::SECTOR_SIZE;
}

bool FlashFileManager::format() {
    if(!flash.is_detected()) return false;

    // Both mirrors have to go, otherwise a stale copy with a higher sequence
    // number would win the next time we mount.
    if(!flash.erase_sector(slot_address(0))) return false;
    if(!flash.erase_sector(slot_address(1))) return false;

    memset(entries, 0, sizeof(entries));

    header.magic = FlashFS::MAGIC;
    header.version = FlashFS::VERSION;
    header.max_files = FlashFS::MAX_FILES;
    header.sequence = 0;
    header.data_start_sector = FlashFS::DATA_START_SECTOR;
    header.total_sectors = usable_sectors();

    // flush_directory() writes to the opposite slot, so aim it at slot 0
    active_slot = 1;
    mounted = true;

    if(!flush_directory()) {
        mounted = false;
        LOGLN("[FS-ERR] Format failed while writing the directory.");
        return false;
    }

    LOGF("[FS] Formatted. %u KB available for files.\n", total_space() / 1024);
    return true;
}

// ============================================================================
// Directory Persistence
// ============================================================================

uint32_t FlashFileManager::compute_checksum() const {
    // FNV-1a over the raw table. Catches the torn writes a plain sum would miss.
    const uint8_t* raw = (const uint8_t*)entries;
    uint32_t hash = 2166136261u;

    for(size_t i = 0; i < sizeof(entries); i++) {
        hash ^= raw[i];
        hash *= 16777619u;
    }
    return hash;
}

bool FlashFileManager::read_slot_header(uint8_t slot, FlashFSHeader& out) const {
    if(!flash.read(slot_address(slot), (uint8_t*)&out, sizeof(out))) return false;

    return out.magic == FlashFS::MAGIC
        && out.version == FlashFS::VERSION
        && out.max_files == FlashFS::MAX_FILES;
}

bool FlashFileManager::load_slot(uint8_t slot) {
    FlashFSHeader candidate;
    if(!read_slot_header(slot, candidate)) return false;

    if(!flash.read(slot_address(slot) + FlashFS::TABLE_OFFSET,
                   (uint8_t*)entries, sizeof(entries))) {
        return false;
    }

    if(compute_checksum() != candidate.checksum) {
        LOGF("[FS-WARN] Directory mirror %d failed its checksum. Ignoring it.\n", slot);
        return false;
    }

    header = candidate;
    active_slot = slot;
    return true;
}

bool FlashFileManager::load_directory() {
    FlashFSHeader head_a, head_b;
    bool ok_a = read_slot_header(0, head_a);
    bool ok_b = read_slot_header(1, head_b);

    if(!ok_a && !ok_b) return false;

    // Try the newest valid mirror first, then fall back to the older one if its
    // table turns out to be corrupt.
    uint8_t first, second;
    if(ok_a && ok_b) {
        first = (head_a.sequence >= head_b.sequence) ? 0 : 1;
        second = first ^ 1;
    }
    else {
        first = ok_a ? 0 : 1;
        second = first;
    }

    if(load_slot(first)) return true;

    if(second != first && load_slot(second)) {
        LOGLN("[FS-WARN] Recovered the file system from the backup directory mirror.");
        return true;
    }

    return false;
}

bool FlashFileManager::flush_directory() {
    // Write to the mirror we are not currently using, so the live copy survives a
    // power cut right through this function.
    uint8_t target = active_slot ^ 1;
    uint32_t base = slot_address(target);

    header.sequence++;
    header.checksum = compute_checksum();

    if(!flash.erase_sector(base)) {
        LOGLN("[FS-ERR] Could not erase the directory sector.");
        return false;
    }

    // Table first, header last. The header carries the magic number, so a write
    // interrupted before it lands leaves the slot invalid rather than half valid.
    if(!flash.write(base + FlashFS::TABLE_OFFSET, (const uint8_t*)entries, sizeof(entries))) {
        LOGLN("[FS-ERR] Could not write the file table.");
        return false;
    }

    if(!flash.write(base, (const uint8_t*)&header, sizeof(header))) {
        LOGLN("[FS-ERR] Could not commit the directory header.");
        return false;
    }

    active_slot = target;
    return true;
}

// ============================================================================
// Lookup and Allocation
// ============================================================================

const char* FlashFileManager::normalize(const char* name) {
    // Accept LittleFS style paths like "/wifiConfig.json" so the same string
    // works against either storage backend.
    if(name != nullptr && name[0] == '/') return name + 1;
    return name;
}

bool FlashFileManager::valid_name(const char* name) {
    const char* clean = normalize(name);
    if(clean == nullptr) return false;

    size_t len = strlen(clean);
    if(len == 0 || len > FlashFS::MAX_NAME_LEN) {
        LOGF("[FS-ERR] File name must be 1 to %d characters.\n", FlashFS::MAX_NAME_LEN);
        return false;
    }
    return true;
}

int8_t FlashFileManager::find_entry(const char* name) const {
    const char* clean = normalize(name);
    if(clean == nullptr) return -1;

    for(uint8_t i = 0; i < FlashFS::MAX_FILES; i++) {
        if(entries[i].in_use && strncmp(entries[i].name, clean, FlashFS::MAX_NAME_LEN) == 0) {
            return (int8_t)i;
        }
    }
    return -1;
}

int8_t FlashFileManager::find_free_entry() const {
    for(uint8_t i = 0; i < FlashFS::MAX_FILES; i++) {
        if(!entries[i].in_use) return (int8_t)i;
    }
    return -1;
}

int32_t FlashFileManager::allocate_run(uint32_t sectors_needed, int8_t ignore_entry) const {
    // One bit per sector. Sized for the largest chip this layer supports, which is
    // why usable_sectors() caps the scan below rather than trusting the chip alone.
    uint8_t bitmap[W25Q64::SECTOR_COUNT / 8];
    memset(bitmap, 0, sizeof(bitmap));

    const uint32_t limit = usable_sectors();

    for(uint8_t i = 0; i < FlashFS::MAX_FILES; i++) {
        if(!entries[i].in_use || (int8_t)i == ignore_entry) continue;

        for(uint32_t s = 0; s < entries[i].sector_count; s++) {
            uint32_t sector = entries[i].start_sector + s;
            if(sector < limit) {
                bitmap[sector / 8] |= (1 << (sector % 8));
            }
        }
    }

    // First fit over the data area. Fragmentation is possible but files here are
    // few and long lived, so a smarter allocator would not earn its complexity.
    uint32_t run_start = FlashFS::DATA_START_SECTOR;
    uint32_t run_len = 0;

    for(uint32_t sector = FlashFS::DATA_START_SECTOR; sector < limit; sector++) {
        bool used = (bitmap[sector / 8] & (1 << (sector % 8))) != 0;

        if(used) {
            run_start = sector + 1;
            run_len = 0;
            continue;
        }

        run_len++;
        if(run_len >= sectors_needed) return (int32_t)run_start;
    }

    return -1;
}

void FlashFileManager::fill_handle(const FlashFileEntry& entry, FlashFile& out) const {
    strncpy(out.name, entry.name, FlashFS::MAX_NAME_LEN);
    out.name[FlashFS::MAX_NAME_LEN] = '\0';
    out.address = entry.start_sector * W25Q64::SECTOR_SIZE;
    out.size = entry.size_bytes;
    out.capacity = entry.sector_count * W25Q64::SECTOR_SIZE;
    out.valid = true;
}

// ============================================================================
// Writing
// ============================================================================

bool FlashFileManager::save_file(const char* name, const uint8_t* data, size_t len) {
    if(!mounted) {
        LOGLN("[FS-ERR] File system is not mounted.");
        return false;
    }
    if(!valid_name(name)) return false;
    if(len > 0 && data == nullptr) return false;

    const char* clean = normalize(name);

    uint32_t needed = W25Q64Handler::sectors_needed(len);
    if(needed == 0) needed = 1; // An empty file still owns one sector

    int8_t idx = find_entry(clean);
    uint32_t start_sector;
    uint32_t run_sectors;

    if(idx >= 0 && entries[idx].sector_count >= needed) {
        // The current allocation is big enough, so overwrite in place. Keeping the
        // full run also preserves the headroom that makes appends cheap.
        start_sector = entries[idx].start_sector;
        run_sectors = entries[idx].sector_count;
    }
    else {
        // Ignore our own sectors while searching so a growing file can absorb the
        // free space sitting right after it instead of moving across the chip.
        int32_t run = allocate_run(needed, idx);
        if(run < 0) {
            LOGF("[FS-ERR] No contiguous space for '%s' (%u sectors needed, %u KB free).\n",
                clean, needed, free_space() / 1024);
            return false;
        }

        if(idx < 0) {
            idx = find_free_entry();
            if(idx < 0) {
                LOGF("[FS-ERR] Directory is full (%d files max).\n", FlashFS::MAX_FILES);
                return false;
            }
        }

        start_sector = (uint32_t)run;
        run_sectors = needed;
    }

    uint32_t addr = start_sector * W25Q64::SECTOR_SIZE;

    // NOR flash cannot overwrite in place, so the whole run is erased first. That
    // is also what leaves the tail blank for later appends.
    if(!flash.erase_range(addr, run_sectors * W25Q64::SECTOR_SIZE)) {
        LOGF("[FS-ERR] Erase failed while saving '%s'.\n", clean);
        return false;
    }

    if(len > 0 && !flash.write(addr, data, len)) {
        LOGF("[FS-ERR] Write failed while saving '%s'.\n", clean);
        return false;
    }

    FlashFileEntry& entry = entries[idx];
    memset(entry.name, 0, sizeof(entry.name));
    strncpy(entry.name, clean, FlashFS::MAX_NAME_LEN);
    entry.start_sector = start_sector;
    entry.size_bytes = len;
    entry.sector_count = (uint16_t)run_sectors;
    entry.in_use = 1;
    entry.reserved = 0;

    return flush_directory();
}

bool FlashFileManager::save_file(const char* name, const char* content) {
    if(content == nullptr) return false;
    return save_file(name, (const uint8_t*)content, strlen(content));
}

bool FlashFileManager::write_file(const char* name, const char* content) {
    return save_file(name, content);
}

bool FlashFileManager::append_file(const char* name, const uint8_t* data, size_t len) {
    if(!mounted || len == 0 || data == nullptr) return false;
    if(!valid_name(name)) return false;

    const char* clean = normalize(name);
    int8_t idx = find_entry(clean);

    if(idx < 0) return save_file(clean, data, len);

    FlashFileEntry& entry = entries[idx];
    uint32_t capacity = entry.sector_count * W25Q64::SECTOR_SIZE;

    // The tail of the allocation is still erased, so short appends are a plain
    // page program with no erase cycle at all.
    if((uint64_t)entry.size_bytes + len <= capacity) {
        uint32_t tail = entry.start_sector * W25Q64::SECTOR_SIZE + entry.size_bytes;

        if(!flash.write(tail, data, len)) {
            LOGF("[FS-ERR] Append failed for '%s'.\n", clean);
            return false;
        }

        entry.size_bytes += len;
        return flush_directory();
    }

    // Outgrew its sectors, so the file has to be rebuilt somewhere with more room
    size_t new_size = entry.size_bytes + len;
    if(new_size > FlashFS::MAX_RAM_FILE) {
        LOGF("[FS-ERR] '%s' would grow to %u bytes, too large to relocate through RAM.\n",
            clean, (uint32_t)new_size);
        return false;
    }

    uint8_t* buffer = (uint8_t*)malloc(new_size);
    if(buffer == nullptr) {
        LOGF("[FS-ERR] Out of heap relocating '%s' (%u bytes).\n", clean, (uint32_t)new_size);
        return false;
    }

    bool ok = false;
    if(read_file(clean, buffer, entry.size_bytes) == entry.size_bytes) {
        memcpy(buffer + entry.size_bytes, data, len);
        ok = save_file(clean, buffer, new_size);
    }
    else {
        LOGF("[FS-ERR] Could not read '%s' back while relocating it.\n", clean);
    }

    free(buffer);
    return ok;
}

bool FlashFileManager::append_file(const char* name, const char* content) {
    if(content == nullptr) return false;
    return append_file(name, (const uint8_t*)content, strlen(content));
}

// ============================================================================
// Reading
// ============================================================================

bool FlashFileManager::open_file(const char* name, FlashFile& out) {
    out.valid = false;
    if(!mounted) return false;

    int8_t idx = find_entry(name);
    if(idx < 0) return false;

    fill_handle(entries[idx], out);
    return true;
}

size_t FlashFileManager::read_at(const FlashFile& file, uint32_t offset, uint8_t* buffer, size_t len) {
    if(!file.valid || buffer == nullptr || len == 0) return 0;
    if(offset >= file.size) return 0;

    // Never read past what was actually stored, even though the sectors extend further
    size_t available = file.size - offset;
    size_t to_read = (len < available) ? len : available;

    if(!flash.read(file.address + offset, buffer, to_read)) return 0;
    return to_read;
}

size_t FlashFileManager::read_file(const char* name, uint8_t* buffer, size_t max_len) {
    FlashFile file;
    if(!open_file(name, file)) return 0;

    return read_at(file, 0, buffer, max_len);
}

String FlashFileManager::read_file(const char* name) {
    FlashFile file;
    if(!open_file(name, file)) return String();

    if(file.size == 0) return String();

    if(file.size > FlashFS::MAX_RAM_FILE) {
        LOGF("[FS-ERR] '%s' is %u bytes, too large to read into a String. "
            "Use open_file() with read_at().\n", file.name, file.size);
        return String();
    }

    String out;
    if(!out.reserve(file.size + 1)) {
        LOGF("[FS-ERR] Out of heap reading '%s' (%u bytes).\n", file.name, file.size);
        return String();
    }

    uint8_t chunk[128];
    uint32_t offset = 0;

    while(offset < file.size) {
        size_t got = read_at(file, offset, chunk, sizeof(chunk));
        if(got == 0) break;

        // The length-aware overload keeps embedded null bytes intact
        out.concat(chunk, got);
        offset += got;
    }

    return out;
}

String FlashFileManager::read_or_default(const char* name, const char* default_content) {
    if(!exists(name)) {
        LOGF("[FS] '%s' not found. Creating it from the default template.\n", normalize(name));
        if(!save_file(name, default_content)) return String();
    }

    return read_file(name);
}

// ============================================================================
// Management
// ============================================================================

bool FlashFileManager::exists(const char* name) {
    return mounted && find_entry(name) >= 0;
}

size_t FlashFileManager::file_size(const char* name) {
    if(!mounted) return 0;

    int8_t idx = find_entry(name);
    return (idx >= 0) ? entries[idx].size_bytes : 0;
}

bool FlashFileManager::delete_file(const char* name) {
    if(!mounted) return false;

    int8_t idx = find_entry(name);
    if(idx < 0) return false;

    // Clearing the entry is enough. The sectors get erased when they are handed
    // out again, which avoids a pointless erase cycle now.
    memset(&entries[idx], 0, sizeof(FlashFileEntry));

    return flush_directory();
}

bool FlashFileManager::rename_file(const char* old_name, const char* new_name) {
    if(!mounted || !valid_name(new_name)) return false;

    int8_t idx = find_entry(old_name);
    if(idx < 0) return false;

    if(find_entry(new_name) >= 0) {
        LOGF("[FS-ERR] '%s' already exists.\n", normalize(new_name));
        return false;
    }

    memset(entries[idx].name, 0, sizeof(entries[idx].name));
    strncpy(entries[idx].name, normalize(new_name), FlashFS::MAX_NAME_LEN);

    return flush_directory();
}

// ============================================================================
// Introspection
// ============================================================================

uint8_t FlashFileManager::file_count() const {
    uint8_t count = 0;
    for(uint8_t i = 0; i < FlashFS::MAX_FILES; i++) {
        if(entries[i].in_use) count++;
    }
    return count;
}

bool FlashFileManager::get_file_at(uint8_t index, FlashFile& out) {
    out.valid = false;
    if(!mounted) return false;

    uint8_t seen = 0;
    for(uint8_t i = 0; i < FlashFS::MAX_FILES; i++) {
        if(!entries[i].in_use) continue;

        if(seen == index) {
            fill_handle(entries[i], out);
            return true;
        }
        seen++;
    }
    return false;
}

String FlashFileManager::list_files() {
    JsonDocument doc;
    JsonArray files = doc["files"].to<JsonArray>();

    for(uint8_t i = 0; i < FlashFS::MAX_FILES; i++) {
        if(!entries[i].in_use) continue;

        JsonObject file = files.add<JsonObject>();
        file["name"] = entries[i].name;
        file["size"] = entries[i].size_bytes;
        file["reserved"] = entries[i].sector_count * W25Q64::SECTOR_SIZE;
    }

    doc["used"] = used_space();
    doc["free"] = free_space();
    doc["total"] = total_space();

    String output;
    serializeJson(doc, output);
    return output;
}

uint32_t FlashFileManager::usable_sectors() const {
    // Follow the chip that is actually fitted, but never walk past the allocation
    // bitmap, which is built for a W25Q64.
    uint32_t sectors = flash.sector_count();
    if(sectors > W25Q64::SECTOR_COUNT) sectors = W25Q64::SECTOR_COUNT;

    // Honour the caller's ceiling, so the sectors above it can be handed to the log rings.
    if(sector_limit > 0 && sector_limit < sectors) sectors = sector_limit;

    return sectors;
}

uint32_t FlashFileManager::total_space() const {
    uint32_t sectors = usable_sectors();
    if(sectors <= FlashFS::DATA_START_SECTOR) return 0;

    return (sectors - FlashFS::DATA_START_SECTOR) * W25Q64::SECTOR_SIZE;
}

uint32_t FlashFileManager::used_space() const {
    uint32_t sectors = 0;
    for(uint8_t i = 0; i < FlashFS::MAX_FILES; i++) {
        if(entries[i].in_use) sectors += entries[i].sector_count;
    }
    return sectors * W25Q64::SECTOR_SIZE;
}

uint32_t FlashFileManager::free_space() const {
    return total_space() - used_space();
}
