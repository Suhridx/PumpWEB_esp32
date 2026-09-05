#pragma once

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "SystemConstants.h"
#include "DataConstants.h"

// --- WS2812 Status Panel Definitions ---
namespace StatusLed {
    constexpr uint8_t LED_COUNT = 8;
    constexpr uint8_t DATA_PIN = GPIOConfig::LED_WIFI_STATUS;

    // --- Pixel assignment, counting from the end the data line enters ---
    constexpr uint8_t IDX_MEMORY = 0;   // EEPROM and RTC together
    constexpr uint8_t IDX_SENSORS = 1;  // Tank and reservoir float switches together
    constexpr uint8_t IDX_RELAY = 2;    // Relay driver
    constexpr uint8_t IDX_BRIDGE = 3;   // UART link to the Pico
    constexpr uint8_t IDX_NETWORK = 4;  // WiFi and MQTT together
    constexpr uint8_t IDX_OTA = 5;      // OTA update activity, pulse only, no fault state
    constexpr uint8_t IDX_SPARE = 6;    // Unassigned, stays dark
    constexpr uint8_t IDX_ACTIVITY = 7; // Driven only by blue_blink()

    // --- Colors, packed 0xRRGGBB. Output is scaled by the panel brightness setting. ---
    constexpr uint32_t COLOR_OFF = 0x000000;
    constexpr uint32_t COLOR_RED = 0xFF0000;
    constexpr uint32_t COLOR_GREEN = 0x00FF00;
    constexpr uint32_t COLOR_YELLOW = 0xFF8000; // Shifted towards amber, which reads better than pure 0xFFFF00
    constexpr uint32_t COLOR_WHITE = 0xFFFFFF;
    constexpr uint32_t COLOR_CYAN = 0x00FF80; // Soft aqua, currently unassigned
    constexpr uint32_t COLOR_BLUE = 0x0000FF;

    // --- Brightness, 0 to 255 ---
    // A WS2812 at full scale is blinding on a panel and pulls 60 mA per pixel, so the
    // fault indicators sit very low. They only need to be noticed, not read.
    constexpr uint8_t DEFAULT_BRIGHTNESS = 10;

    // --- Timing ---
    constexpr uint16_t BLINK_HALF_PERIOD_MS = 400; // Fault blink, on for this long then off
    constexpr uint16_t BLUE_HALF_PERIOD_MS = 150;  // Activity pulse, deliberately snappier
    constexpr uint16_t TRAFFIC_FLASH_MS = 80;      // Single pulse per packet, on the bridge and network pixels
    constexpr uint16_t MEMORY_FLASH_MS = 400;      // Transaction result on the memory pixel, held long enough to read
} // namespace StatusLed


/**
 * @brief Driver for the 8 pixel WS2812 status panel.
 *
 * Turns the HardwareStatus_t bits into a row of indicator lights. The panel reads
 * dark when everything is healthy, so any lit pixel is a problem worth looking at.
 * The one exception is the activity pixel, which flashes blue on demand.
 *
 * Nothing here blocks. update_leds() is driven from the main loop and drives the
 * blink phases off millis(), and the strip is only pushed when something actually
 * changed, since a WS2812 refresh is expensive to do on every loop iteration.
 */
class WS2812Handler {
public:
    WS2812Handler(uint8_t data_pin = StatusLed::DATA_PIN,
        uint8_t led_count = StatusLed::LED_COUNT);

    /**
     * @brief Starts the strip and clears every pixel.
     */
    bool begin();

    /**
     * @brief Refreshes the panel from the current hardware status.
     *
     * Call this on every pass of the main loop. It maps the status bits onto the
     * pixels, advances the blink timers, and pushes to the strip only when the
     * visible state has actually changed.
     *
     * @param status The live hardware status word.
     */
    void update_leds(const HardwareStatus_t& status);

    /**
     * @brief Flashes the activity pixel blue the requested number of times.
     *
     * Returns immediately. The pulses are played out by update_leds() off a millis
     * timer, so this is safe to call from anywhere that must not block, including
     * an MQTT or packet handler. Calling it while a burst is already running adds
     * the new pulses to the queue rather than cutting the burst short.
     *
     * @param count Number of blinks to play.
     */
    void blue_blink(uint8_t count);

    /**
     * @brief True while an activity burst is still playing.
     */
    bool is_blinking() const;

    /**
     * @brief Pulses the bridge pixel white once, to show traffic from the Pico.
     *
     * Returns immediately and is played out by update_leds() off a millis timer, so it is
     * safe to call from a packet handler. The pulse overrides whatever the bridge pixel is
     * showing for its duration, then the pixel reverts to the link state. Calling it again
     * while a pulse is running restarts it, so a burst of packets holds the pixel white
     * rather than producing a flicker too short to see.
     */
    void bridge_blink();

    /**
     * @brief Pulses the network pixel yellow once, to show traffic from the broker.
     *
     * The counterpart to bridge_blink() for the MQTT side, with the same semantics: it
     * returns immediately, is played out by update_leds() off a millis timer, overrides the
     * pixel for its duration, and restarts rather than stacking if called again mid pulse.
     *
     * Safe to call from either core. It writes three plain variables and never touches the
     * strip, so the broker task on core 0 can raise the flag directly and leave core 1 to
     * play the pulse out on its next pass through update_leds().
     */
    void network_blink();

    /**
     * @brief Pulses the memory pixel once in the given colour, to report the outcome of a
     *        config or schedule transaction.
     *
     * Green for an ACK from the Pico, red for a NACK. Held for MEMORY_FLASH_MS, which is
     * deliberately far longer than the traffic pulses: this reports the result of something
     * the user asked for, so it has to be readable rather than merely noticeable.
     *
     * Unlike the traffic pulses, a result here says nothing about whether the EEPROM and
     * RTC are healthy, so it can briefly paint over a genuine memory fault. That is
     * intended, since the fault returns the moment the pulse expires.
     *
     * @param color Packed 0xRRGGBB colour to hold for the duration of the pulse.
     */
    void memory_blink(uint32_t color);

    /**
     * @brief Pulses the sensor pixel blue once, to show a water level reading arriving.
     *
     * Same 80 ms one shot as the traffic pulses. The pixel keeps its own red fault blink
     * for the case where the sensors are not reporting; this pulse rides over it, so a
     * reading is visible even while the pair is still considered unhealthy.
     *
     * Not yet called from anywhere: it waits on the sensor decode path.
     */
    void sensor_blink();

    /**
     * @brief Pulses the OTA pixel green once, to show OTA update activity.
     *
     * That pixel has no fault state of its own and is otherwise dark, so this is the only
     * thing that ever lights it. Fixed green, same 80 ms as the traffic pulses.
     *
     * Not yet called from anywhere: it waits on the OTA mode implementation.
     */
    void ota_blink();

    /**
     * @brief Scales the whole panel, 0 to 255.
     */
    void set_brightness(uint8_t brightness);

    /**
     * @brief Clears every pixel immediately.
     */
    void all_off();

private:
    // How one pixel is currently being driven
    enum class LedMode : uint8_t {
        OFF,
        SOLID, // Held at colour
        BLINK  // Alternates colour and off, in step with the other blinking pixels
    };

    struct PixelState {
        LedMode mode = LedMode::OFF;
        uint32_t color = StatusLed::COLOR_OFF;
    };

    Adafruit_NeoPixel strip;
    uint8_t data_pin;
    uint8_t led_count;
    bool started = false;

    PixelState pixels[StatusLed::LED_COUNT];

    // --- Fault blink phase, shared by every blinking pixel so they pulse together ---
    bool blink_on = true;
    uint32_t blink_last_toggle = 0;

    // --- Activity burst state ---
    uint8_t blue_remaining = 0;
    bool blue_on = false;
    uint32_t blue_last_toggle = 0;

    // --- One shot pixel pulses, as opposed to the repeating blink phases above ---
    // A pulse overrides one pixel's normal state for a fixed time, then lets it revert.
    // Each slot is bound to its pixel and duration once in the constructor; only the
    // trigger time, and for the memory slot the colour, change at runtime.
    enum PulseSlot : uint8_t {
        PULSE_BRIDGE,  // White, on any packet from the Pico
        PULSE_NETWORK, // Yellow, on any message from the broker
        PULSE_MEMORY,  // Green or red, on a config or schedule transaction result
        PULSE_SENSORS, // Blue, on a water level sensor reading
        PULSE_OTA,     // Green, on OTA update activity
        PULSE_COUNT
    };

    struct PixelPulse {
        uint8_t index = 0;                     // Pixel this slot drives
        uint16_t duration = 0;                 // How long it is held, in ms
        uint32_t color = StatusLed::COLOR_OFF; // Colour shown while active
        uint32_t start = 0;                    // millis() at the last trigger
        bool on = false;
    };

    PixelPulse pulses[PULSE_COUNT];

    // Starts, or restarts, one slot
    void trigger_pulse(PulseSlot slot);

    // --- Change tracking, so the strip is only pushed when it needs to be ---
    uint16_t last_status = 0xFFFF; // Deliberately impossible, forces the first render
    bool dirty = true;

    // Applies the status word to the pixel table
    void map_status(const HardwareStatus_t& status);

    // Advances the shared blink phase, the one shot pulses and the activity burst
    void advance_timers(uint32_t now);

    // Writes the pixel table out to the strip
    void render();

    void set_pixel(uint8_t index, LedMode mode, uint32_t color);
};
