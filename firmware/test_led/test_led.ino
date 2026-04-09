// Bare-minimum NeoPixel test. No servos, no serial, nothing else.
#include <Adafruit_NeoPixel.h>

#define PIN 7
#define NUM_LEDS 32

Adafruit_NeoPixel strip(NUM_LEDS, PIN, NEO_GRB + NEO_KHZ800);

void setup() {
    pinMode(13, OUTPUT);

    strip.begin();
    strip.setBrightness(30);
    strip.setPixelColor(0, 255, 0, 0);  // red
    strip.setPixelColor(1, 0, 255, 0);  // green
    strip.setPixelColor(2, 0, 0, 255);  // blue
    strip.show();

    // Blink pin 13 to confirm setup() ran
    digitalWrite(13, HIGH);
    delay(500);
    digitalWrite(13, LOW);
}

void loop() {
    // Cycle all LEDs through red/green/blue every 2 seconds
    uint32_t colors[] = {
        strip.Color(255, 0, 0),
        strip.Color(0, 255, 0),
        strip.Color(0, 0, 255),
    };

    for (int c = 0; c < 3; c++) {
        strip.fill(colors[c]);
        strip.show();
        digitalWrite(13, HIGH);
        delay(2000);
        digitalWrite(13, LOW);
        delay(100);
    }
}
