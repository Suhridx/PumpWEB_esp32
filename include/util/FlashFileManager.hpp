#pragma once

#include <Arduino.h>
#include "hardwares/W25Q64Handler.hpp"

// --- On-Flash Layout Definitions ---
namespace FlashFS {
    constexpr uint32_t MAGIC = 0x50574653; // "PWFS", Pump Web File System
    constexpr uint16_t VERSION = 0x0001;

    constexpr uint8_t MAX_FILES = 32;
    constexpr uint8_t MAX_NAME_LEN = 27; // Plus the null terminator, so 28 bytes stored

    // The directory is mirrored across two sectors. Every flush alternates between
    // them, so a power cut mid-write always leaves the previous copy intact.
    constexpr uint32_t DIR_SECTOR_A = 0;
    constexpr uint32_t DIR_SECTOR_B = 1;
    constexpr uint32_t DATA_START_SECTOR = 2;

    // Where the file table starts inside a directory sector, leaving the first
    // 64 bytes for the header.
    constexpr uint32_t TABLE_OFFSET = 64;

    // Whole-file reads land in RAM, so refuse anything that would blow the heap
    constexpr uint32_t MAX_RAM_FILE = 64 * 1024;
} // namespace FlashFS


// Directory header, written last on every flush so it doubles as a commit marker
struct __attribute__((packed)) FlashFSHeader {
    uint32_t magic;
    uint16_t version;
    uint16_t max_files;
    uint32_t sequence;   // Higher value wins when both mirrors are valid
    uint32_t checksum;   // FNV-1a over the file table
    uint32_t data_start_sector;
    uint32_t total_sectors;
};

// One directory entry. Files occupy a contiguous run of whole 4 KB sectors.
struct __attribute__((packed)) FlashFileEntry {
    char name[FlashFS::MAX_NAME_LEN + 1];
    uint32_t start_sector;
    uint32_t size_bytes;
    uint16_t sector_count;
    uint8_t in_use;
    uint8_t reserved;
};

static_assert(sizeof(FlashFSHeader) <= FlashFS::TABLE_OFFSET,
    "Directory header must fit before the file table");
static_assert(FlashFS::TABLE_OFFSET + sizeof(FlashFileEntry) * FlashFS::MAX_FILES <= W25Q64::SECTOR_SIZE,
    "Directory must fit inside a single 4 KB sector");

// Lightweight handle returned by open_file(), for streaming reads of large files
struct FlashFile {
    char name[FlashFS::MAX_NAME_LEN + 1] = {0};
    uint32_t address = 0;  // Absolute byte address of the first byte in flash
    uint32_t size = 0;     // Bytes currently stored
    uint32_t capacity = 0; // Bytes reserved, always a multiple of the sector size
    bool valid = false;
};


/**
 * @brief High level file storage on top of the W25Q64 flash chip.
 *
 * Mirrors the API of configs/FileManager, which does the same job on LittleFS, so
 * moving a config or a log between internal and external storage is a one line change.
 * The whole point of this layer is that callers never touch addresses, pages, erase
 * granularity or the write-enable latch. They deal in names and content.
 *
 * Layout: sectors 0 and 1 hold two mirrored copies of the directory, data starts at
 * sector 2. Each file gets a contiguous run of whole 4 KB sectors, which keeps reads
 * and appends to a single flash operation at the cost of rounding every file up to
 * 4 KB. Deleting a file releases its run for reuse.
 *
 * Wear note: every operation that changes the directory erases one sector, alternating
 * between the two mirrors. That is fine for configs and occasional logs. If you append
 * to a log many times per second, batch the writes in RAM first.
 */
class FlashFileManager {
public:
    /**
     * @brief Binds to the chip. Nothing is touched until begin().
     * @param flash_ref    The chip driver.
     * @param sector_limit Stop the file system at this sector, leaving everything above it
     *                     free for other users such as the log rings. 0 means the whole
     *                     chip. The limit is recorded in the on-flash header, so shrinking
     *                     it later does not silently strand files that sit above the line.
     */
    FlashFileManager(W25Q64Handler& flash_ref, uint32_t sector_limit = 0);

    /**
     * @brief Mounts the file system, formatting the chip automatically if it is blank.
     * @return true once the directory is loaded and usable.
     */
    bool begin();

    /**
     * @brief Wipes the directory, releasing every file. Old data is left in place
     *        and gets erased when its sectors are handed out again.
     */
    bool format();

    /**
     * @brief True once begin() has loaded a valid directory.
     */
    bool is_mounted() const;

    // =====================================================================
    // Writing
    // =====================================================================

    /**
     * @brief Stores raw bytes under a name, replacing any existing file with that name.
     * @note Handles the erase for you. This is the one to reach for by default.
     */
    bool save_file(const char* name, const uint8_t* data, size_t len);

    /**
     * @brief Stores a null-terminated string, without the terminator.
     */
    bool save_file(const char* name, const char* content);

    /**
     * @brief Alias of save_file, matching the name used by configs/FileManager.
     */
    bool write_file(const char* name, const char* content);

    /**
     * @brief Adds bytes to the end of a file, creating it if it does not exist.
     * @note Appends land straight in the already-erased tail of the allocation, so
     *       they stay cheap until the file outgrows its sectors and has to be moved.
     */
    bool append_file(const char* name, const uint8_t* data, size_t len);

    /**
     * @brief Adds a null-terminated string to the end of a file.
     */
    bool append_file(const char* name, const char* content);

    // =====================================================================
    // Reading
    // =====================================================================

    /**
     * @brief Looks a file up and fills in a handle describing where it lives.
     * @return false if no such file exists.
     */
    bool open_file(const char* name, FlashFile& out);

    /**
     * @brief Reads a whole file into a String. Empty String on failure.
     * @note Refuses files larger than FlashFS::MAX_RAM_FILE. Use open_file plus
     *       read_at for anything bigger.
     */
    String read_file(const char* name);

    /**
     * @brief Reads a whole file into a caller-supplied buffer.
     * @return Number of bytes actually read, 0 on failure.
     */
    size_t read_file(const char* name, uint8_t* buffer, size_t max_len);

    /**
     * @brief Reads part of an opened file, clamped to its stored size.
     * @return Number of bytes actually read.
     */
    size_t read_at(const FlashFile& file, uint32_t offset, uint8_t* buffer, size_t len);

    /**
     * @brief Reads a file, creating it from default_content first if it is missing.
     * @note The robust config pattern, same as configs/FileManager::read_or_default.
     */
    String read_or_default(const char* name, const char* default_content);

    // =====================================================================
    // Management
    // =====================================================================

    bool exists(const char* name);
    size_t file_size(const char* name);
    bool delete_file(const char* name);
    bool rename_file(const char* old_name, const char* new_name);

    // =====================================================================
    // Introspection
    // =====================================================================

    /**
     * @brief How many files are currently stored.
     */
    uint8_t file_count() const;

    /**
     * @brief Fetches the Nth stored file, for iterating the directory.
     * @param index Counts stored files only, skipping empty directory slots.
     */
    bool get_file_at(uint8_t index, FlashFile& out);

    /**
     * @brief Directory listing as a JSON string, ready to publish or serve.
     */
    String list_files();

    uint32_t total_space() const;
    uint32_t used_space() const;
    uint32_t free_space() const;

private:
    W25Q64Handler& flash;
    uint32_t sector_limit; // 0 means use the whole chip

    FlashFSHeader header;
    FlashFileEntry entries[FlashFS::MAX_FILES];
    uint8_t active_slot = 0;
    bool mounted = false;

    // --- Directory persistence ---
    bool load_directory();
    bool read_slot_header(uint8_t slot, FlashFSHeader& out) const;
    bool load_slot(uint8_t slot);
    bool flush_directory();
    uint32_t compute_checksum() const;
    static uint32_t slot_address(uint8_t slot);

    // --- Lookup and allocation ---
    static const char* normalize(const char* name);
    static bool valid_name(const char* name);
    int8_t find_entry(const char* name) const;
    int8_t find_free_entry() const;
    int32_t allocate_run(uint32_t sectors_needed, int8_t ignore_entry) const;
    void fill_handle(const FlashFileEntry& entry, FlashFile& out) const;

    // Sectors this file system may use: the detected chip size, capped by the size
    // the allocation bitmap was built for.
    uint32_t usable_sectors() const;
};
