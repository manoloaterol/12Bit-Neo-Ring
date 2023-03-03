#include "ws2812.h"

#include "math.h"

TIM_HandleTypeDef *ws_timer;

void ws2812_fillBlack(void)
{
	for (uint8_t i = 0; i < LED_COUNT; i++)
	{
		LEDdata[i].red = 0;
		LEDdata[i].green = 0;
		LEDdata[i].blue = 0;
	}
}

void ws2812_fillBufferBlack(void)
{
	uint32_t index, buffIndex;
	buffIndex = 0;

	for (index = 0; index < RESET_FRAMES; index++)
	{
		LEDbuffer[buffIndex] = WS2812_RESET;
		buffIndex++;
	}
	for (index = 0; index < (LED_COUNT * 24); index++)
	{
		LEDbuffer[buffIndex] = WS2812_0;
		buffIndex++;
	}
	LEDbuffer[buffIndex] = WS2812_0;
	buffIndex++;
}

void ws2812_setLEDcolor(uint32_t LEDnumber, uint8_t RED, uint8_t GREEN, uint8_t BLUE)
{

	LEDdata[LEDnumber].red = RED;
	LEDdata[LEDnumber].green = GREEN;
	LEDdata[LEDnumber].blue = BLUE;
}

void ws2812_setLEDfade(uint32_t LEDnumber, float fade)
{
	LEDdata[LEDnumber].red /= fade;
	LEDdata[LEDnumber].green /= fade;
	LEDdata[LEDnumber].blue /= fade;
}

/** HUE Code credits goes to FastLed library
 * https://github.com/FastLED/FastLED
 */
void ws2812_setLEDhue(uint32_t LEDnumber, uint8_t hue, uint8_t sat, uint8_t val, uint8_t fadeIfLower, float fade)
{
	// use Spectrum colors instead of Rainbow implementation of FastLED
	hue =  (((uint16_t)hue) * (1+(uint16_t)(191))) >> 8;

	uint8_t r, g, b = 0;
	// Convert hue, saturation and brightness ( HSV/HSB ) to RGB
	// "Dimming" is used on saturation and brightness to make
	// the output more visually linear.

	// The brightness floor is minimum number that all of
	// R, G, and B will be set to.
	uint8_t invsat = 255 - sat;
	uint8_t brightness_floor = (val * invsat) / 256;

	// The color amplitude is the maximum amount of R, G, and B
	// that will be added on top of the brightness_floor to
	// create the specific hue desired.
	uint8_t color_amplitude = val - brightness_floor;

	// Figure out which section of the hue wheel we're in,
	// and how far offset we are withing that section
	uint8_t section = hue / 0x40; // 0..2
	uint8_t offset = hue % 0x40;  // 0..63

	uint8_t rampup = offset;				// 0..63
	uint8_t rampdown = (0x40 - 1) - offset; // 63..0

	// We now scale rampup and rampdown to a 0-255 range -- at least
	// in theory, but here's where architecture-specific decsions
	// come in to play:
	// To scale them up to 0-255, we'd want to multiply by 4.
	// But in the very next step, we multiply the ramps by other
	// values and then divide the resulting product by 256.
	// So which is faster?
	//   ((ramp * 4) * othervalue) / 256
	// or
	//   ((ramp    ) * othervalue) /  64
	// It depends on your processor architecture.
	// On 8-bit AVR, the "/ 256" is just a one-cycle register move,
	// but the "/ 64" might be a multicycle shift process. So on AVR
	// it's faster do multiply the ramp values by four, and then
	// divide by 256.
	// On ARM, the "/ 256" and "/ 64" are one cycle each, so it's
	// faster to NOT multiply the ramp values by four, and just to
	// divide the resulting product by 64 (instead of 256).
	// Moral of the story: trust your profiler, not your insticts.

	// Since there's an AVR assembly version elsewhere, we'll
	// assume what we're on an architecture where any number of
	// bit shifts has roughly the same cost, and we'll remove the
	// redundant math at the source level:

	//  // scale up to 255 range
	//  //rampup *= 4; // 0..252
	//  //rampdown *= 4; // 0..252

	// compute color-amplitude-scaled-down versions of rampup and rampdown
	uint8_t rampup_amp_adj = (rampup * color_amplitude) / (256 / 4);
	uint8_t rampdown_amp_adj = (rampdown * color_amplitude) / (256 / 4);

	// add brightness_floor offset to everything
	uint8_t rampup_adj_with_floor = rampup_amp_adj + brightness_floor;
	uint8_t rampdown_adj_with_floor = rampdown_amp_adj + brightness_floor;

	if (section)
	{
		if (section == 1)
		{
			// section 1: 0x40..0x7F
			r = brightness_floor;
			g = rampdown_adj_with_floor;
			b = rampup_adj_with_floor;
		}
		else
		{
			// section 2; 0x80..0xBF
			r = rampup_adj_with_floor;
			g = brightness_floor;
			b = rampdown_adj_with_floor;
		}
	}
	else
	{
		// section 0: 0x00..0x3F
		r = rampdown_adj_with_floor;
		g = rampup_adj_with_floor;
		b = brightness_floor;
	}

	if(fadeIfLower == 1 && LEDdata[LEDnumber].red > r && LEDdata[LEDnumber].green > g && LEDdata[LEDnumber].blue > b ) {
		ws2812_setLEDfade(LEDnumber, fade);
	} else {
	ws2812_setLEDcolor(LEDnumber, r, g, b);
	}
}

void ws2812_setWHOLEcolor(uint8_t RED, uint8_t GREEN, uint8_t BLUE)
{

	for (uint8_t i = 0; i < LED_COUNT; i++)
	{
		LEDdata[i].red = RED;
		LEDdata[i].green = GREEN;
		LEDdata[i].blue = BLUE;
	}
}

void ws2812_update(void)
{

	ws2812_generateBuffer();

	HAL_TIM_PWM_Start_DMA(ws_timer, TIM_CHANNEL_1, (uint32_t *)&LEDbuffer, LED_BUFFER_SIZE);
}

void ws2812_generateBuffer(void)
{
	uint32_t buffIndex = 0;

	for (uint8_t i = 0; i < RESET_FRAMES; i++)
	{
		LEDbuffer[buffIndex] = WS2812_RESET;
		buffIndex++;
	}

	for (uint8_t i = 0; i < LED_COUNT; i++)
	{
		for (uint8_t g = 0; g < 8; g++)
		{ // GREEN data
			LEDbuffer[buffIndex] = ((LEDdata[i].green << g) & 0x80) ? WS2812_1 : WS2812_0;
			buffIndex++;
		}
		for (uint8_t r = 0; r < 8; r++)
		{ // RED
			LEDbuffer[buffIndex] = ((LEDdata[i].red << r) & 0x80) ? WS2812_1 : WS2812_0;
			buffIndex++;
		}
		for (uint8_t b = 0; b < 8; b++)
		{ // BLUE
			LEDbuffer[buffIndex] = ((LEDdata[i].blue << b) & 0x80) ? WS2812_1 : WS2812_0;
			buffIndex++;
		}
	}
}

void ws2812_init(TIM_HandleTypeDef *timer)
{
	ws_timer = timer;
	int i;
	for (i = RESET_FRAMES; i < LED_BUFFER_SIZE; i++)
		LEDbuffer[i] = WS2812_0;
}

void ws2812_shift(uint8_t steps)
{

	RGB LEDdataRotated[LED_COUNT] = {0};

	for (uint8_t i = 0; i < LED_COUNT; i++)
	{
		LEDdataRotated[i] = LEDdata[i];
	}

	RGB first;
	if (steps > 0)
	{
		for (uint8_t step = 0; step < steps; step++)
		{
			first = LEDdataRotated[0];
			for (uint8_t i = 0; i < LED_COUNT - 1; i++)
			{
				LEDdataRotated[i] = LEDdataRotated[i + 1];
			}
			LEDdataRotated[LED_COUNT - 1] = first;
		}
	}

	// Buffer
	uint32_t buffIndex = 0;

	for (uint8_t i = 0; i < RESET_FRAMES; i++)
	{
		LEDbuffer[buffIndex] = WS2812_RESET;
		buffIndex++;
	}

	for (uint8_t i = 0; i < LED_COUNT; i++)
	{
		for (uint8_t g = 0; g < 8; g++)
		{ // GREEN data
			LEDbuffer[buffIndex] = ((LEDdataRotated[i].green << g) & 0x80) ? WS2812_1 : WS2812_0;
			buffIndex++;
		}
		for (uint8_t r = 0; r < 8; r++)
		{ // RED
			LEDbuffer[buffIndex] = ((LEDdataRotated[i].red << r) & 0x80) ? WS2812_1 : WS2812_0;
			buffIndex++;
		}
		for (uint8_t b = 0; b < 8; b++)
		{ // BLUE
			LEDbuffer[buffIndex] = ((LEDdataRotated[i].blue << b) & 0x80) ? WS2812_1 : WS2812_0;
			buffIndex++;
		}
	}

	HAL_TIM_PWM_Start_DMA(ws_timer, TIM_CHANNEL_1, (uint32_t *)&LEDbuffer, LED_BUFFER_SIZE);
}

void ws2812_mirrorFirstQuarter()
{
	for (int i = 11; i > 7; i--)
	{
		int mirrorLed = 0 + (12 - i);
		LEDdata[i] = LEDdata[mirrorLed];
	}

	for (int i = 4; i < 9; i++)
	{
		int mirrorLed = i - (2 * (i - 3));
		if (mirrorLed < 0)
		{
			mirrorLed += 12;
		}
		LEDdata[i] = LEDdata[mirrorLed];
	}
}

void ws2812_mirrorHalf() {
	for (uint8_t i = (LED_COUNT/2); i < LED_COUNT ; i++){
		LEDdata[i] = LEDdata[i-(LED_COUNT/2)];
	}
}
