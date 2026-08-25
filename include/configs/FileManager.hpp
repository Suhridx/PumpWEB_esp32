#pragma once

#include <Arduino.h>
#include <LittleFS.h>

class FileManager {
public:
    // Initialize the LittleFS file system (Formats on first boot if necessary)
    static bool begin();

    // Read the entire contents of a file into a String
    static String read_file(const char* path);

    // Write completely new content to a file (Overwrites existing)
    static bool write_file(const char* path, const char* content);

    // Add content to the end of an existing file (Great for offline logging)
    static bool append_file(const char* path, const char* content);

    // The robust fallback: If file doesn't exist, create it with default_content, then read it.
    static String read_or_default(const char* path, const char* default_content);

    // Check if a file exists
    static bool exists(const char* path);

    // Delete a file
    static bool delete_file(const char* path);
};