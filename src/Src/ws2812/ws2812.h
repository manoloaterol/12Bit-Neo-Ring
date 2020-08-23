#include "stm32f0xx_hal.h"

typedef struct RGB {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
} RGB;

/**
 * NZR communication protocol at 800KHz.
 * For a MCU working at 48MHz, timer is
 * configured with prescaler=0 and period=60
 */
// Bit 0 = 1/3 HIGH and 2/3 low, so 60/3 = 20
#define WS2812_0 20	
// Bit 1 = 2/3 HIGH and 1/3 low, so (60*2)/3 = 40
#define WS2812_1 40	

// Reset = stay LOW for at least 280us, so we
// need 280us / 1,25us(800KHz) = 224 -> 230
#define WS2812_RESET (0)
#define RESET_FRAMES 230

// Number of leds
#define LED_COUNT 12

// Data array length. Each led needs 24bits to store RGB values (8bits*3)
#define LED_BUFFER_SIZE ((LED_COUNT*24) + RESET_FRAMES)

volatile static uint8_t LEDbuffer[LED_BUFFER_SIZE] = {0};
volatile static RGB LEDdata[LED_COUNT] = {0};
TIM_HandleTypeDef *ws_timer;

void ws2812_fillBlack(void);
void ws2812_fillBufferBlack(void);
void ws2812_setLEDcolor(uint32_t LEDnumber, uint8_t RED, uint8_t GREEN, uint8_t BLUE);
void ws2812_setLEDhue(uint32_t LEDnumber, uint8_t hue, uint8_t sat, uint8_t val);
void ws2812_setLEDfade (uint32_t LEDnumber, float fade);
void ws2812_setWHOLEcolor(uint8_t RED, uint8_t GREEN, uint8_t BLUE);
void ws2812_update(void);
void ws2812_generateBuffer(void);
void ws2812_init(TIM_HandleTypeDef *timer);

void ws2812_shift(uint8_t steps);
void ws2812_mirrorFirstQuarter();
