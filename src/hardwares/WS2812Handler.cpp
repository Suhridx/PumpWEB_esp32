#include "hardwares/WS2812Handler.hpp"
#include "Log.h"

// ============================================================================
// Construction / Initialization
// ============================================================================

WS2812Handler::WS2812Handler(uint8_t data_pin, uint8_t led_count)
    : strip(led_count, data_pin, NEO_GRB + NEO_KHZ800),
      data_pin(data_pin), led_count(led_count) {
    // Only the pixel buffer is allocated here. The GPIO is not touched until begin().

    // Bind each pulse slot to the pixel it drives and how long it is held. Assigned field
    // by field rather than braced, so this does not depend on the toolchain's C++ level.
    pulses[PULSE_BRIDGE].index = StatusLed::IDX_BRIDGE;
    pulses[PULSE_BRIDGE].duration = StatusLed::TRAFFIC_FLASH_MS;
    pulses[PULSE_BRIDGE].color = StatusLed::COLOR_WHITE;

    pulses[PULSE_NETWORK].index = StatusLed::IDX_NETWORK;
    pulses[PULSE_NETWORK].duration = StatusLed::TRAFFIC_FLASH_MS;
    pulses[PULSE_NETWORK].color = StatusLed::COLOR_YELLOW;

    pulses[PULSE_MEMORY].index = StatusLed::IDX_MEMORY;
    pulses[PULSE_MEMORY].duration = StatusLed::MEMORY_FLASH_MS;
    // The memory colour is set per call, since that slot reports an outcome not an event.
}

bool WS2812Handler::begin() {
    if(started) return true;

    strip.begin();
    strip.setBrightness(StatusLed::DEFAULT_BRIGHTNESS);
    strip.clear();
    strip.show();

    blink_last_toggle = millis();
    blue_last_toggle = blink_last_toggle;
    started = true;

    LOGF("[LED] WS2812 panel up. %d pixels on GPIO %d.\n", led_count, data_pin);
    return true;
}

void WS2812Handler::set_brightness(uint8_t brightness) {
    strip.setBrightness(brightness);
    dirty = true;
}

void WS2812Handler::all_off() {
    for(uint8_t i = 0; i < StatusLed::LED_COUNT; i++) {
        pixels[i].mode = LedMode::OFF;
        pixels[i].color = StatusLed::COLOR_OFF;
    }

    blue_remaining = 0;
    blue_on = false;

    for(uint8_t i = 0; i < PULSE_COUNT; i++) {
        pulses[i].on = false;
    }

    if(started) {
        strip.clear();
        strip.show();
    }
    dirty = false;
}

void WS2812Handler::set_pixel(uint8_t index, LedMode mode, uint32_t color) {
    if(index >= led_count || index >= StatusLed::LED_COUNT) return;

    if(pixels[index].mode != mode || pixels[index].color != color) {
        pixels[index].mode = mode;
        pixels[index].color = color;
        dirty = true;
    }
}

// ============================================================================
// Status Mapping
// ============================================================================

void WS2812Handler::map_status(const HardwareStatus_t& status) {
    // The panel reads dark when the system is healthy, so a lit pixel always means
    // something needs attention. Held solid is a hard local fault, blinking means it may
    // still come good on its own: red for the water level sensors and the Pico bridge,
    // amber for the network. On top of that the bridge and network pixels take a brief
    // traffic pulse when a packet lands, white and yellow respectively, which only ever
    // shows against the dark healthy state since traffic means the link is up.

    // Paired flags are ANDed: the pixel only goes dark once both halves are good.
    bool memory_ok = status.bits.eeprom_ready && status.bits.rtc_ready;
    bool sensors_ok = status.bits.tank_sensor_ready && status.bits.reservoir_sensor_ready;
    bool network_ok = status.bits.wifi_connected && status.bits.mqtt_connected;

    set_pixel(StatusLed::IDX_MEMORY,
        memory_ok ? LedMode::OFF : LedMode::SOLID,
        memory_ok ? StatusLed::COLOR_OFF : StatusLed::COLOR_RED);

    set_pixel(StatusLed::IDX_SENSORS,
        sensors_ok ? LedMode::OFF : LedMode::BLINK,
        sensors_ok ? StatusLed::COLOR_OFF : StatusLed::COLOR_RED);

    set_pixel(StatusLed::IDX_RELAY,
        status.bits.relay_ready ? LedMode::OFF : LedMode::SOLID,
        status.bits.relay_ready ? StatusLed::COLOR_OFF : StatusLed::COLOR_RED);

    set_pixel(StatusLed::IDX_BRIDGE,
        status.bits.bridge_connected ? LedMode::OFF : LedMode::BLINK,
        status.bits.bridge_connected ? StatusLed::COLOR_OFF : StatusLed::COLOR_RED);

    set_pixel(StatusLed::IDX_NETWORK,
        network_ok ? LedMode::OFF : LedMode::BLINK,
        network_ok ? StatusLed::COLOR_OFF : StatusLed::COLOR_YELLOW);
}

// ============================================================================
// Timing
// ============================================================================

void WS2812Handler::advance_timers(uint32_t now) {
    // --- Shared fault blink phase ---
    // Subtraction on unsigned millis stays correct across the 49 day rollover.
    if((now - blink_last_toggle) >= StatusLed::BLINK_HALF_PERIOD_MS) {
        blink_last_toggle = now;
        blink_on = !blink_on;

        // Only worth a refresh if something is actually blinking right now
        for(uint8_t i = 0; i < StatusLed::LED_COUNT; i++) {
            if(pixels[i].mode == LedMode::BLINK) {
                dirty = true;
                break;
            }
        }
    }

    // --- One shot pulses ---
    for(uint8_t i = 0; i < PULSE_COUNT; i++) {
        if(pulses[i].on && (now - pulses[i].start) >= pulses[i].duration) {
            pulses[i].on = false;
            dirty = true;
        }
    }

    // --- Activity burst ---
    if(blue_remaining > 0 || blue_on) {
        if((now - blue_last_toggle) >= StatusLed::BLUE_HALF_PERIOD_MS) {
            blue_last_toggle = now;

            if(blue_on) {
                // End of a pulse. One blink is a complete on then off pair.
                blue_on = false;
                if(blue_remaining > 0) blue_remaining--;
            }
            else if(blue_remaining > 0) {
                blue_on = true;
            }

            dirty = true;
        }
    }
}

void WS2812Handler::blue_blink(uint8_t count) {
    if(count == 0) return;

    // Adding rather than replacing, so a second request during a burst queues up
    // instead of cutting the first one short.
    uint16_t total = (uint16_t)blue_remaining + count;
    blue_remaining = (total > 255) ? 255 : (uint8_t)total;

    if(!blue_on) {
        blue_on = true;
        blue_last_toggle = millis();
        dirty = true;
    }
}

void WS2812Handler::trigger_pulse(PulseSlot slot) {
    // Restarting rather than ignoring a repeat call, so back to back events hold the pixel
    // lit instead of chopping the pulse into something too brief to register.
    pulses[slot].start = millis();
    pulses[slot].on = true;
    dirty = true;
}

void WS2812Handler::bridge_blink() {
    trigger_pulse(PULSE_BRIDGE);
}

void WS2812Handler::network_blink() {
    trigger_pulse(PULSE_NETWORK);
}

void WS2812Handler::memory_blink(uint32_t color) {
    // The one slot whose colour varies, because it reports an outcome rather than simply
    // that something happened.
    pulses[PULSE_MEMORY].color = color;
    trigger_pulse(PULSE_MEMORY);
}

bool WS2812Handler::is_blinking() const {
    return blue_remaining > 0 || blue_on;
}

// ============================================================================
// Rendering
// ============================================================================

void WS2812Handler::render() {
    for(uint8_t i = 0; i < led_count && i < StatusLed::LED_COUNT; i++) {
        uint32_t color = StatusLed::COLOR_OFF;

        switch(pixels[i].mode) {
            case LedMode::SOLID:
                color = pixels[i].color;
                break;

            case LedMode::BLINK:
                color = blink_on ? pixels[i].color : StatusLed::COLOR_OFF;
                break;

            case LedMode::OFF:
            default:
                color = StatusLed::COLOR_OFF;
                break;
        }

        strip.setPixelColor(i, color);
    }

    // Active pulses override the status table for their duration. Written after the main
    // loop so they win, and before the activity pixel, which is not part of the table at
    // all. For the two traffic slots this only ever paints over the dark healthy state,
    // since traffic implies the link is up; the memory slot can briefly cover a real fault,
    // which is intended and reverts when the pulse expires.
    for(uint8_t i = 0; i < PULSE_COUNT; i++) {
        if(pulses[i].on) {
            strip.setPixelColor(pulses[i].index, pulses[i].color);
        }
    }

    // The activity pixel is not part of the status table, it is driven purely by
    // the burst state, so it is written last and overrides whatever sits there.
    strip.setPixelColor(StatusLed::IDX_ACTIVITY,
        blue_on ? StatusLed::COLOR_BLUE : StatusLed::COLOR_OFF);

    strip.show();
}

void WS2812Handler::update_leds(const HardwareStatus_t& status) {
    if(!started) return;

    // Re-mapping every pass would be wasted work, since the status word only moves
    // when the Pico syncs or the network changes.
    if(status.all != last_status) {
        last_status = status.all;
        map_status(status);
    }

    advance_timers(millis());

    // A WS2812 refresh is far too expensive to run on every loop iteration, so it
    // only happens when the visible state actually moved.
    if(dirty) {
        // Cleared before the repaint rather than after. network_blink() may be raised from
        // core 0 at any moment, and clearing afterwards could wipe a flag set while
        // render() was still running, swallowing that pulse entirely. Clearing first turns
        // the same race into a harmless extra repaint on the following pass.
        dirty = false;
        render();
    }
}
