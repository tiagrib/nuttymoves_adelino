#ifndef ADELINO_LED_MATRIX_H
#define ADELINO_LED_MATRIX_H

#include "config.h"

// ============================================================
// Keyes 2812 8x4 LED Matrix (WS2812B) Driver
// ============================================================
//
// Hardware: 32 WS2812B LEDs in an 8-column x 4-row serpentine grid.
// Wiring:   DIN -> LED_MATRIX_PIN, VCC -> 5V, GND -> GND.
//           300-500 ohm resistor on data line recommended.
//           100-1000 uF cap across VCC/GND for power smoothing.
//
// The Rust side owns all drawing/rendering logic and sends a flat
// array of 32 RGB triplets in row-major logical order.  This driver
// handles the serpentine re-mapping to physical LED indices.

#if LED_MATRIX_ENABLED

#include <Adafruit_NeoPixel.h>

static Adafruit_NeoPixel led_strip(
    LED_MATRIX_COUNT, LED_MATRIX_PIN, NEO_GRB + NEO_KHZ800);

/// Map logical (col, row) in row-major order to the physical LED index.
/// Even rows run left-to-right, odd rows run right-to-left (serpentine).
inline uint16_t led_xy_to_index(uint8_t col, uint8_t row) {
    if (row & 1) {
        return row * LED_MATRIX_COLS + (LED_MATRIX_COLS - 1 - col);
    }
    return row * LED_MATRIX_COLS + col;
}

/// Initialize the LED strip.  Call once from setup().
/// Flashes first 3 LEDs (R, G, B) briefly as a self-test.
inline void led_matrix_init() {
    led_strip.begin();
    led_strip.setBrightness(30);

    // Boot self-test: flash R/G/B on first 3 LEDs for 400ms
    led_strip.setPixelColor(0, 255, 0, 0);
    led_strip.setPixelColor(1, 0, 255, 0);
    led_strip.setPixelColor(2, 0, 0, 255);
    led_strip.show();
    delay(400);

    led_strip.clear();
    led_strip.show();
}

/// Update the entire matrix from a flat RGB buffer (32 pixels x 3 bytes,
/// row-major logical order) and a global brightness value.
///
/// `rgb` must point to at least LED_MATRIX_COUNT * 3 bytes.
inline void led_matrix_update(const uint8_t* rgb, uint8_t brightness) {
    led_strip.setBrightness(brightness);
    for (uint8_t row = 0; row < LED_MATRIX_ROWS; row++) {
        for (uint8_t col = 0; col < LED_MATRIX_COLS; col++) {
            uint16_t logical = row * LED_MATRIX_COLS + col;
            uint16_t physical = led_xy_to_index(col, row);
            led_strip.setPixelColor(physical,
                rgb[logical * 3 + 0],   // R
                rgb[logical * 3 + 1],   // G
                rgb[logical * 3 + 2]);  // B
        }
    }
    led_strip.show();
}

/// Turn all LEDs off.
inline void led_matrix_clear() {
    led_strip.clear();
    led_strip.show();
}

#else  // LED_MATRIX_ENABLED == 0

inline void led_matrix_init() {}
inline void led_matrix_update(const uint8_t* rgb, uint8_t brightness) {
    (void)rgb; (void)brightness;
}
inline void led_matrix_clear() {}

#endif // LED_MATRIX_ENABLED

#endif // ADELINO_LED_MATRIX_H
