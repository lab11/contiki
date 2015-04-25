
#include "contiki.h"
#include "dev/ioc.h"
#include "dev/gpio.h"
#include "pwm.h"
#include "gptimer.h"

static uint32_t cycles;

int
pwm_init(uint8_t gptimer, uint8_t gpsubtimer, uint32_t freq, uint8_t port, uint8_t pin)
{
  cycles = 16000000 / freq;

  ungate_gpt(gptimer);
  gpt_configure_timer(gptimer, GPTIMER_CFG_GPTMCFG_16BIT_TIMER);
  gpt_set_mode(gptimer, gpsubtimer, GPTIMER_TnMR_TnMR_PERIODIC);
  gpt_set_alternate_mode(gptimer, gpsubtimer, GPTIMER_TnMR_TnAMS_PWM_MODE);
  gpt_set_interval_value(gptimer, gpsubtimer, cycles);
  gpt_set_match_value(gptimer, gpsubtimer, cycles/2);

  uint32_t ioc_sel = IOC_PXX_SEL_GPT0_ICP1 + (gptimer*2) + gpsubtimer;
  ioc_set_sel(port, pin, ioc_sel);
  ioc_set_over(port, pin, IOC_OVERRIDE_OE);

  GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(port), GPIO_PIN_MASK(pin));

  return 0;
}

void
pwm_start(uint8_t gptimer, uint8_t gpsubtimer)
{
  gpt_enable_event(gptimer, gpsubtimer);
}

void
pwm_stop(uint8_t gptimer, uint8_t gpsubtimer)
{
  gpt_disable_event(gptimer, gpsubtimer);
}

int pwm_set_dutycycle(uint8_t gptimer, uint8_t gpsubtimer, uint32_t dc) {
  uint32_t new_cycles = ((100-dc) * cycles) / 100;
  gpt_set_match_value(gptimer, gpsubtimer, new_cycles);
}




