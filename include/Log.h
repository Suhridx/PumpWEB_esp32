// include/Log.h
#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>

// ==============================================================================
// Serial logging with a core tag
// ==============================================================================
//
// Work in this firmware is split across both cores. The Arduino loop, the Pico packet
// handling and the status panel all run on core 1, while the WiFi and MQTT state
// machines run on tasks pinned to core 0. An interleaved serial capture is hard to read
// without knowing which side produced each line, and a line appearing on a core it has
// no business running on is usually the first sign of a threading mistake.
//
// Every line is prefixed with [C0] or [C1]. xPortGetCoreID() is a cheap register read,
// so tagging costs effectively nothing even in the hot paths.
//
//   LOGF  replaces Serial.printf.  Takes a literal format string plus its arguments.
//   LOGLN replaces Serial.println. Takes a bare string and appends the newline itself.

#define LOGF(fmt, ...) Serial.printf("[C%d] " fmt, xPortGetCoreID(), ##__VA_ARGS__)
#define LOGLN(msg)     Serial.printf("[C%d] %s\n", xPortGetCoreID(), msg)
