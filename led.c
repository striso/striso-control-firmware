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

ioline_t led_dimmed = LINE_LED_UP;

static void pwm_dimmed_on(PWMDriver *pwmp) {
  (void)pwmp;
  palSetLine(led_dimmed);
}

static void pwm_dimmed_off(PWMDriver *pwmp) {
  (void)pwmp;
  palClearLine(led_dimmed);
}

static PWMConfig pwmcfg_updown = {
  125000,                                 /* PWM clock frequency.   */
  2048,                                    /* Initial PWM period.    */
  pwm_dimmed_on,
  {
   {PWM_OUTPUT_ACTIVE_HIGH, pwm_dimmed_off},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
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
  pwmStart(&PWMD3, &pwmcfg_updown);
  pwmEnableChannel(&PWMD3, 0, 768); // duty cycle of dimmed up/down leds
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

inline void led_updown_state(ioline_t line, uint32_t state) {
  if (state) {
    palSetLine(line);
    if (state != 0xf) {
      led_dimmed = line;
      pwmEnablePeriodicNotification(&PWMD3);
      pwmEnableChannelNotification(&PWMD3, 0);
    }
  } else {
    palClearLine(line);
  }
}

/**
 * Set up/down leds state
 *
 * state is 0xabcd with a=up2, b=up, c=down, d=down2
 * 0 = off, f = on, anything else = dimmed.
 * Only one led can be dimmed at the same time.
 */
void led_updown(uint32_t state) {
  pwmDisableChannelNotification(&PWMD3, 0);
  pwmDisablePeriodicNotification(&PWMD3);
  led_updown_state(LINE_LED_UP2,   state       & 0xf);
  led_updown_state(LINE_LED_UP,    state >>  4 & 0xf);
  led_updown_state(LINE_LED_DOWN,  state >>  8 & 0xf);
  led_updown_state(LINE_LED_DOWN2, state >> 12 & 0xf);
}

void led_updown_dial(int angle) {
  angle = ((angle + 1024) % 16) - 8; // +1024 to make % work as modulo for negative numbers
  if (angle >= 0) {
    led_updown(0xffff >> 2*angle);
  } else {
    led_updown(0xffff << 2*-angle);
  }
}
