#ifndef __LED_GPIO_CORELYNX_H__
#define __LED_GPIO_CORELYNX_H__

extern struct k_timer led_timer;
void led_state_init(void);
void led_state_set(int state);

#endif
