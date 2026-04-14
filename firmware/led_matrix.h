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

/// Write a partial range of pixels into the buffer (no show).
/// `offset` is the starting logical pixel index.
/// `rgb` points to `count * 3` bytes of R,G,B data.
inline void led_matrix_set_pixels(uint8_t offset, uint8_t count, const uint8_t* rgb) {
    for (uint8_t i = 0; i < count; i++) {
        uint8_t logical = offset + i;
        if (logical >= LED_MATRIX_COUNT) break;
        uint8_t row = logical / LED_MATRIX_COLS;
        uint8_t col = logical % LED_MATRIX_COLS;
        uint16_t physical = led_xy_to_index(col, row);
        led_strip.setPixelColor(physical,
            rgb[i * 3 + 0],   // R
            rgb[i * 3 + 1],   // G
            rgb[i * 3 + 2]);  // B
    }
}

/// Set brightness and push the pixel buffer to the strip.
inline void led_matrix_show(uint8_t brightness) {
    led_strip.setBrightness(brightness);
    led_strip.show();
}

/// Turn all LEDs off.
inline void led_matrix_clear() {
    led_strip.clear();
    led_strip.show();
}

/// Initialize the LED strip.  Call once from setup().
/// Flashes first 3 LEDs (R, G, B) briefly as a self-test.
inline void led_matrix_init() {
    led_strip.begin();
    led_strip.setBrightness(30);

    // Boot self-test: flash R/G/B on first 3 LEDs for 400ms
    const uint8_t test_rgb[] = {255,0,0, 0,255,0, 0,0,255};
    led_matrix_set_pixels(0, 3, test_rgb);
    led_matrix_show(30);
    delay(400);

    led_strip.clear();
    led_matrix_show(30);
}

#else  // LED_MATRIX_ENABLED == 0

inline void led_matrix_init() {}
inline void led_matrix_set_pixels(uint8_t offset, uint8_t count, const uint8_t* rgb) {
    (void)offset; (void)count; (void)rgb;
}
inline void led_matrix_show(uint8_t brightness) { (void)brightness; }
inline void led_matrix_clear() {}

#endif // LED_MATRIX_ENABLED

#endif // ADELINO_LED_MATRIX_H
