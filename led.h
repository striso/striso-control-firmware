#ifndef _LED_H_
#define _LED_H_
#include <hal.h>

void led_init(void);

void led_rgb(uint32_t rgb);
void led_rgb3(int r, int g, int b);
/* Set light color and return to previous color after given time */
void led_rgb3_blink(int r, int g, int b, sysinterval_t time);

void led_tick(void);

void led_updown(uint32_t state);
void led_updown_dial(int angle);

#endif
