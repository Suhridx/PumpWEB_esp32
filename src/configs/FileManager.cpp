#include "configs/FileManager.hpp"
#include "Log.h"

bool FileManager::begin() {
    // The 'true' parameter tells the ESP32 to automatically format the 
    // flash memory if the file system is corrupt or completely blank (first boot)
    if(!LittleFS.begin(true)) {
        LOGLN("[FILE-ERR] LittleFS Mount Failed!");
        return false;
    }
    LOGLN("[FILE] LittleFS Mounted Successfully.");
    return true;
}

String FileManager::read_file(const char* path) {
    if(!LittleFS.exists(path)) {
        LOGF("[FILE-WARN] %s does not exist.\n", path);
        return "";
    }

    File file = LittleFS.open(path, "r");
    if(!file) {
        LOGF("[FILE-ERR] Failed to open %s for reading.\n", path);
        return "";
    }

    String content = file.readString();
    file.close();
    return content;
}

bool FileManager::write_file(const char* path, const char* content) {
    File file = LittleFS.open(path, "w");
    if(!file) {
        LOGF("[FILE-ERR] Failed to open %s for writing.\n", path);
        return false;
    }

    if(file.print(content)) {
        file.close();
        return true;
    }
    else {
        LOGF("[FILE-ERR] Write failed to %s.\n", path);
        file.close();
        return false;
    }
}

bool FileManager::append_file(const char* path, const char* content) {
    // "a" mode opens the file for appending (writing to the very end)
    File file = LittleFS.open(path, "a");
    if(!file) {
        LOGF("[FILE-ERR] Failed to open %s for appending.\n", path);
        return false;
    }

    if(file.print(content)) {
        file.close();
        return true;
    }
    else {
        LOGF("[FILE-ERR] Append failed to %s.\n", path);
        file.close();
        return false;
    }
}

String FileManager::read_or_default(const char* path, const char* default_content) {
    // If the file does not exist, physically create it with the default payload
    if(!LittleFS.exists(path)) {
        LOGF("[FILE] %s missing. Creating with default template...\n", path);
        write_file(path, default_content);
    }

    // Now, return the contents (which will either be the user's saved data, 
    // or the fresh default template we just generated).
    return read_file(path);
}

bool FileManager::exists(const char* path) {
    return LittleFS.exists(path);
}

bool FileManager::delete_file(const char* path) {
    if(!LittleFS.exists(path)) {
        return false;
    }

    if(LittleFS.remove(path)) {
        LOGF("[FILE] Deleted %s\n", path);
        return true;
    }
    else {
        LOGF("[FILE-ERR] Failed to delete %s\n", path);
        return false;
    }
}