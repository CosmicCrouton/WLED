#pragma once
#ifndef WLED_COLORS_H
#define WLED_COLORS_H

/*
 * Color structs and color utility functions
 */
#include <vector>
#include "FastLED.h"

// CRGBW can be used to manipulate 32bit colors faster. However: if it is passed to functions, it adds overhead compared to a uint32_t color
// use with caution and pay attention to flash size. Usually converting a uint32_t to CRGBW to extract r, g, b, w values is slower than using bitshifts
// it can be useful to avoid back and forth conversions between uint32_t and fastled CRGB
struct CRGBW {
    union {
        uint32_t color32; // Access as a 32-bit value (0xWWRRGGBB)
        struct {
            uint8_t b;
            uint8_t g;
            uint8_t r;
            uint8_t w;
        };
        uint8_t raw[4];   // Access as an array in the order B, G, R, W
    };

    // Default constructor
    inline CRGBW() __attribute__((always_inline)) = default;

    // Constructor from a 32-bit color (0xWWRRGGBB)
    constexpr CRGBW(uint32_t color) __attribute__((always_inline)) : color32(color) {}

    // Constructor with r, g, b, w values
    constexpr CRGBW(uint8_t red, uint8_t green, uint8_t blue, uint8_t white = 0) __attribute__((always_inline)) : b(blue), g(green), r(red), w(white) {}

    // Constructor from CRGB
    constexpr CRGBW(CRGB rgb) __attribute__((always_inline)) : b(rgb.b), g(rgb.g), r(rgb.r), w(0) {}

    // Access as an array
    inline const uint8_t& operator[] (uint8_t x) const __attribute__((always_inline)) { return raw[x]; }

    // Assignment from 32-bit color
    inline CRGBW& operator=(uint32_t color) __attribute__((always_inline)) { color32 = color; return *this; }

    // Assignment from r, g, b, w
    inline CRGBW& operator=(const CRGB& rgb) __attribute__((always_inline)) { b = rgb.b; g = rgb.g; r = rgb.r; w = 0; return *this; }

    // Conversion operator to uint32_t
    inline operator uint32_t() const __attribute__((always_inline)) {
      return color32;
    }
    /*
    // Conversion operator to CRGB
    inline operator CRGB() const __attribute__((always_inline)) {
      return CRGB(r, g, b);
    }

    CRGBW& scale32 (uint8_t scaledown) // 32bit math
    {
      if (color32 == 0) return *this; // 2 extra instructions, worth it if called a lot on black (which probably is true) adding check if scaledown is zero adds much more overhead as its 8bit
      uint32_t scale = scaledown + 1;
      uint32_t rb = (((color32 & 0x00FF00FF) * scale) >> 8) & 0x00FF00FF; // scale red and blue
      uint32_t wg = (((color32 & 0xFF00FF00) >> 8) * scale) & 0xFF00FF00; // scale white and green
          color32 =  rb | wg;
      return *this;
    }*/

};

struct CHSV32 { // 32bit HSV color with 16bit hue for more accurate conversions
  union {
    struct {
        uint16_t h;  // hue
        uint8_t s;   // saturation
        uint8_t v;   // value
    };
    uint32_t raw;    // 32bit access
  };
  inline CHSV32() __attribute__((always_inline)) = default; // default constructor

    /// Allow construction from hue, saturation, and value
    /// @param ih input hue
    /// @param is input saturation
    /// @param iv input value
  inline CHSV32(uint16_t ih, uint8_t is, uint8_t iv) __attribute__((always_inline)) // constructor from 16bit h, s, v
        : h(ih), s(is), v(iv) {}
  inline CHSV32(uint8_t ih, uint8_t is, uint8_t iv) __attribute__((always_inline)) // constructor from 8bit h, s, v
        : h((uint16_t)ih << 8), s(is), v(iv) {}
  inline CHSV32(const CHSV& chsv) __attribute__((always_inline))  // constructor from CHSV
    : h((uint16_t)chsv.h << 8), s(chsv.s), v(chsv.v) {}
  inline operator CHSV() const { return CHSV((uint8_t)(h >> 8), s, v); } // typecast to CHSV
};

void colorXYtosRGB(float x, float y, byte* rgb);
void hsv2rgb(const CHSV32& hsv, uint32_t& rgb); // convert HSV (16bit hue) to RGB (32bit with white = 0)
void rgb2hsv(const uint32_t rgb, CHSV32& hsv); // convert RGB to HSV (16bit hue), much more accurate and faster than fastled version
#endif

