#ifndef PWM_C_
#define PWM_C_

#include <stdint.h>

int pwm_init(uint8_t gptimer, uint8_t gpsubtimer, uint32_t freq, uint8_t port, uint8_t pin);
void pwm_start(uint8_t gptimer, uint8_t gpsubtimer);
void pwm_stop(uint8_t gptimer, uint8_t gpsubtimer);

#endif