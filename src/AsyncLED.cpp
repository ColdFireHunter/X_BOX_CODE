// AsyncLED.cpp
#include "AsyncLED.h"

// static storage for timer, DMA, and bit buffer
TIM_HandleTypeDef AsyncLED::htim16;
DMA_HandleTypeDef AsyncLED::hdma_ch1;
uint16_t AsyncLED::bitBuffer[AsyncLED::TOTAL_SLOTS];

void AsyncLED::begin()
{
    // 1) enable clocks
    __HAL_RCC_TIM16_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    // 2) configure PD0 = AF2 = TIM16_CH1
    GPIO_InitTypeDef gpio = {};
    gpio.Pin = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF2_TIM16;
    HAL_GPIO_Init(GPIOD, &gpio);

    // 3) configure TIM16 for 800 kHz PWM (48 MHz / 60)
    htim16.Instance = TIM16;
    htim16.Init.Prescaler = 0;
    htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim16.Init.Period = 59; // ARR = 59 → 60 ticks
    htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim16);
    HAL_TIM_PWM_Init(&htim16);

    TIM_OC_InitTypeDef oc = {};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim16, &oc, TIM_CHANNEL_1);

    // 4) configure DMA1_Channel1 → TIM16_CH1
    hdma_ch1.Instance = DMA1_Channel1;
    hdma_ch1.Init.Request = DMA_REQUEST_TIM16_CH1;
    hdma_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_ch1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_ch1.Init.Mode = DMA_NORMAL;
    hdma_ch1.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_ch1);

    __HAL_LINKDMA(&htim16, hdma[TIM_DMA_ID_CC1], hdma_ch1);

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

void AsyncLED::setPixelRGBW(uint16_t idx, uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
    if (idx >= NUM_RGBW)
        return;
    colorsRGBW[idx][0] = g;
    colorsRGBW[idx][1] = r;
    colorsRGBW[idx][2] = b;
    colorsRGBW[idx][3] = w;
}

void AsyncLED::setPixelRGB(uint16_t idx, uint8_t r, uint8_t g, uint8_t b)
{
    if (idx >= NUM_RGB)
        return;
    colorsRGB[idx][0] = g;
    colorsRGB[idx][1] = r;
    colorsRGB[idx][2] = b;
}

/// map (x,y) where (0,0) is top‑right, x increases leftwards, y increases downwards
uint16_t AsyncLED::indexFromXY(uint8_t x, uint8_t y) const
{
    // first pixel idx=0 at (0,0)=top‑right,
    // each row continues leftwards: idx++
    return y * 8 + x;
}

void AsyncLED::drawPixelMatrix(uint8_t x, uint8_t y,
                               uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
    if (x < 8 && y < 8)
    {
        uint16_t idx = indexFromXY(x, y);
        setPixelRGBW(idx, r, g, b, w);
    }
}

void AsyncLED::fillRGBW(uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
    for (uint16_t i = 0; i < NUM_RGBW; i++)
    {
        colorsRGBW[i][0] = g;
        colorsRGBW[i][1] = r;
        colorsRGBW[i][2] = b;
        colorsRGBW[i][3] = w;
    }
}

void AsyncLED::fillRGB(uint8_t r, uint8_t g, uint8_t b)
{
    for (uint16_t i = 0; i < NUM_RGB; i++)
    {
        colorsRGB[i][0] = g;
        colorsRGB[i][1] = r;
        colorsRGB[i][2] = b;
    }
}

void AsyncLED::fillBuffer()
{
    uint32_t ptr = 0;
    constexpr uint16_t T1 = 36, T0 = 20;
    auto applyB = [&](uint8_t v)
    {
        return (uint16_t(v) * brightnessLimit) / 255;
    };

    // encode 64 SKC6812 RGBW LEDs
    for (uint16_t i = 0; i < NUM_RGBW; i++)
    {
        for (uint8_t ch = 0; ch < 4; ch++)
        {
            uint8_t val = applyB(colorsRGBW[i][ch]);
            for (int8_t b = 7; b >= 0; b--)
            {
                bitBuffer[ptr++] = (val & (1 << b)) ? T1 : T0;
            }
        }
    }
    // encode 3 WS2812B RGB LEDs
    for (uint16_t i = 0; i < NUM_RGB; i++)
    {
        for (uint8_t ch = 0; ch < 3; ch++)
        {
            uint8_t val = applyB(colorsRGB[i][ch]);
            for (int8_t b = 7; b >= 0; b--)
            {
                bitBuffer[ptr++] = (val & (1 << b)) ? T1 : T0;
            }
        }
    }
    // reset / latch
    for (uint16_t i = 0; i < RESET_SLOTS; i++)
    {
        bitBuffer[ptr++] = 0;
    }
}

void AsyncLED::show()
{
    fillBuffer();
    HAL_TIM_PWM_Start_DMA(&htim16,
                          TIM_CHANNEL_1,
                          (uint32_t *)bitBuffer,
                          TOTAL_SLOTS);
}

// DMA IRQ handler
extern "C" void DMA1_Channel1_IRQHandler()
{
    HAL_DMA_IRQHandler(&AsyncLED::hdma_ch1);
}

// — show one 8×8 char, rotated 270° clockwise (i.e. 90° CCW) —
void AsyncLED::showChar8x8(char c,
                           uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
    clearRGBW();
    if (c < 0x20 || c > 0x7F)
        c = '?';
    // original font8x8_basic[c][col] gives 8 bits for rows 0..7
    for (uint8_t col = 0; col < 8; col++)
    {
        uint8_t bits = font8x8_basic[(uint8_t)c][col];
        for (uint8_t row = 0; row < 8; row++)
        {
            if (bits & (1 << row))
            {
                // rotate 270° cw: (col,row) → ( newX = 7-row, newY = col )
                drawPixelMatrix(7 - row, col, r, g, b, w);
            }
        }
    }
    show();
}