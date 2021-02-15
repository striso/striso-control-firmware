#ifndef _LED_H_
#define _LED_H_
#include <hal.h>

void led_init(void);

void led_rgb(uint32_t rgb);
void led_rgb3(int r, int g, int b);

void led_updown(uint32_t state);

#endif