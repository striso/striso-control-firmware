#include <ch.h>
#include <hal.h>

static PWMConfig pwmcfg = {
  8000000,                                 /* PWM clock frequency.   */
  2048,                                    /* Initial PWM period.    */
  NULL,
  {
   {PWM_OUTPUT_ACTIVE_LOW, NULL},
   {PWM_OUTPUT_ACTIVE_LOW, NULL},
   {PWM_OUTPUT_ACTIVE_LOW, NULL},
   {PWM_OUTPUT_DISABLED, NULL}
  },
  0,
  0
};

int gamma8(int x) {
  if (x < 33) return x;
  else return ((x+1)*(x+1))/32-1;
}

void led_init(void) {
  pwmStart(&PWMD4, &pwmcfg);
  palSetLineMode(LINE_LED_R, PAL_MODE_ALTERNATE(2));
  palSetLineMode(LINE_LED_G, PAL_MODE_ALTERNATE(2));
  palSetLineMode(LINE_LED_B, PAL_MODE_ALTERNATE(2));
}

void led_rgb(uint32_t rgb) {
  int r = rgb >> 16 & 0xff;
  int g = rgb >>  8 & 0xff;
  int b = rgb >>  0 & 0xff;
  pwmEnableChannel(&PWMD4, 0, gamma8(r));
  pwmEnableChannel(&PWMD4, 2, gamma8(g));
  pwmEnableChannel(&PWMD4, 1, gamma8(b));
}

void led_rgb3(int r, int g, int b) {
  pwmEnableChannel(&PWMD4, 0, gamma8(r));
  pwmEnableChannel(&PWMD4, 2, gamma8(g));
  pwmEnableChannel(&PWMD4, 1, gamma8(b));
}

void led_updown(uint32_t state) {
  if (state       & 0xf) palSetLine(LINE_LED_UP2);   else palClearLine(LINE_LED_UP2);
  if (state >>  4 & 0xf) palSetLine(LINE_LED_UP);    else palClearLine(LINE_LED_UP);
  if (state >>  8 & 0xf) palSetLine(LINE_LED_DOWN);  else palClearLine(LINE_LED_DOWN);
  if (state >> 12 & 0xf) palSetLine(LINE_LED_DOWN2); else palClearLine(LINE_LED_DOWN2);
}
