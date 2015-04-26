
#include "contiki.h"
#include "dev/ioc.h"
#include "dev/gpio.h"
#include "pwm.h"
#include "gptimer.h"

static uint32_t cycles;
static uint32_t saved_dc;

int
pwm_init(uint8_t gptimer, uint8_t gpsubtimer, uint32_t freq, uint32_t dc,
         uint8_t port, uint8_t pin)
{
  ungate_gpt(gptimer);
  gpt_configure_timer(gptimer, GPTIMER_CFG_GPTMCFG_16BIT_TIMER);
  gpt_set_mode(gptimer, gpsubtimer, GPTIMER_TnMR_TnMR_PERIODIC);
  gpt_set_alternate_mode(gptimer, gpsubtimer, GPTIMER_TnMR_TnAMS_PWM_MODE);

  cycles = 16000000 / freq;
  gpt_set_interval_value(gptimer, gpsubtimer, cycles);

  saved_dc = dc;
  uint32_t dc_cycles = ((10000-dc) * cycles) / 10000;
  gpt_set_match_value(gptimer, gpsubtimer, dc_cycles);

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

int pwm_set_frequency(uint8_t gptimer, uint8_t gpsubtimer, uint32_t freq) {
  cycles = 16000000 / freq;
  uint32_t dc_cycles = ((10000-saved_dc) * cycles) / 10000;
  gpt_set_match_value(gptimer, gpsubtimer, dc_cycles);
  gpt_set_interval_value(gptimer, gpsubtimer, cycles);
  return 0;
}

int pwm_set_dutycycle(uint8_t gptimer, uint8_t gpsubtimer, uint32_t dc) {
  uint32_t dc_cycles;
  saved_dc = dc;
  if (dc == 0) {
    gpt_disable_event(gptimer, gpsubtimer);
  } else {
    if (dc == 10000) {
      dc_cycles = cycles;
    } else {
      dc_cycles = ((10000-dc) * cycles) / 10000;
    }
    gpt_set_match_value(gptimer, gpsubtimer, dc_cycles);
    gpt_enable_event(gptimer, gpsubtimer);
  }
  return 0;
}




