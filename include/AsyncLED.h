// AsyncLED.h
#ifndef ASYNCLED_H
#define ASYNCLED_H

#ifndef HAL_TIM_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#endif

#include <Arduino.h>
#include "stm32g0xx_hal.h"
#include "stm32g0xx_hal_tim.h"

// bring in a standard 8×8 font table, ASCII 0x00–0x7F:
#include <font8x8_basic.h>

class AsyncLED
{
public:
    AsyncLED() : brightnessLimit(255) {}

    void begin();

    // single-pixel
    void setPixelRGBW(uint16_t idx, uint8_t r, uint8_t g, uint8_t b, uint8_t w);
    void setPixelRGB(uint16_t idx, uint8_t r, uint8_t g, uint8_t b);

    // matrix-mode
    void drawPixelMatrix(uint8_t x, uint8_t y,
                         uint8_t r, uint8_t g, uint8_t b, uint8_t w);

    // fill / clear
    void fillRGBW(uint8_t r, uint8_t g, uint8_t b, uint8_t w);
    void clearRGBW() { fillRGBW(0, 0, 0, 0); }
    void fillRGB(uint8_t r, uint8_t g, uint8_t b);
    void clearRGB() { fillRGB(0, 0, 0); }

    // brightness limit
    void setBrightness(uint8_t b) { brightnessLimit = b; }
    uint8_t getBrightness() const { return brightnessLimit; }

    // **8×8** single-char & scrolling
    void showChar8x8(char c,
                     uint8_t r, uint8_t g, uint8_t b, uint8_t w);
    void show(); // non-blocking DMA + PWM

    static DMA_HandleTypeDef hdma_ch1;

private:
    static const uint16_t NUM_RGBW = 64;
    static const uint16_t NUM_RGB = 3;
    static const uint16_t BITS_PER_RGBW = 32;
    static const uint16_t BITS_PER_RGB = 24;
    static const uint16_t RESET_SLOTS = 52;
    static const uint16_t TOTAL_SLOTS =
        NUM_RGBW * BITS_PER_RGBW + NUM_RGB * BITS_PER_RGB + RESET_SLOTS;

    uint8_t colorsRGBW[NUM_RGBW][4] = {};
    uint8_t colorsRGB[NUM_RGB][3] = {};
    uint8_t brightnessLimit;

    void fillBuffer();
    uint16_t indexFromXY(uint8_t x, uint8_t y) const;

    static TIM_HandleTypeDef htim16;
    static uint16_t bitBuffer[TOTAL_SLOTS];
};

#endif // ASYNCLED_H
