#include "gptimer.h"
#include "reg.h"
#include "sys/energest.h"
#include "dev/sys-ctrl.h"
#include "lpm.h"
#include "dev/leds.h"
#include <stdint.h>
#include <stdio.h>

static gptimer_callback_t gptimer_callbacks[32] = {NULL};
// gptimer_callback_t testgptf = NULL;
// static uint8_t subtimer_a_masks[4] = {0,2,4,16};
// static uint8_t subtimer_b_masks[4] = {0,2,4,8};
static uint8_t TIMER_TO_SYS_CTL[4] = {SYS_CTRL_RCGCGPT_GPT0, SYS_CTRL_RCGCGPT_GPT1, SYS_CTRL_RCGCGPT_GPT2, SYS_CTRL_RCGCGPT_GPT3};
static uint32_t TIMER_TO_BASE[4] = {GPTIMER_0_BASE, GPTIMER_1_BASE, GPTIMER_2_BASE, GPTIMER_3_BASE};
// static uint32_t interrupt_masks[2][4] = {{0, 2, 4, 16}, {256, 512, 1024, 2048}};



int
ungate_gpt(uint8_t timer)
{
  GPTIMER_CHECK_VALID_TIMER(timer);

  ungate_gpt_running(timer);
  ungate_gpt_sleeping(timer);
  ungate_gpt_pm0(timer);

  return 0;
}

int
gate_gpt(uint8_t timer)
{
  GPTIMER_CHECK_VALID_TIMER(timer);

  gate_gpt_running(timer);
  gate_gpt_sleeping(timer);
  gate_gpt_pm0(timer);

  return 0;
}

int
ungate_gpt_running(uint8_t timer) {
  GPTIMER_CHECK_VALID_TIMER(timer);
  REG(SYS_CTRL_RCGCGPT) |= TIMER_TO_SYS_CTL[timer];
  return 0;
}

int
ungate_gpt_sleeping(uint8_t timer)
{
  GPTIMER_CHECK_VALID_TIMER(timer);
  REG(SYS_CTRL_SCGCGPT) |= TIMER_TO_SYS_CTL[timer];
  return 0;
}

int
ungate_gpt_pm0(uint8_t timer)
{
  GPTIMER_CHECK_VALID_TIMER(timer);
  REG(SYS_CTRL_DCGCGPT) |= TIMER_TO_SYS_CTL[timer];
  return 0;
}

int
gate_gpt_running(uint8_t timer)
{
  GPTIMER_CHECK_VALID_TIMER(timer);
  REG(SYS_CTRL_RCGCGPT) &= ~TIMER_TO_SYS_CTL[timer];
  return 0;
}

int
gate_gpt_sleeping(uint8_t timer)
{
  GPTIMER_CHECK_VALID_TIMER(timer);
  REG(SYS_CTRL_SCGCGPT) &= ~TIMER_TO_SYS_CTL[timer];
  return 0;
}

int
gate_gpt_pm0(uint8_t timer)
{
  GPTIMER_CHECK_VALID_TIMER(timer);
  REG(SYS_CTRL_DCGCGPT) &= ~TIMER_TO_SYS_CTL[timer];
  return 0;
}

static uint32_t
get_event_time(uint8_t timer, uint8_t subtimer)
{
  uint32_t timer_base = TIMER_TO_BASE[timer];
  uint32_t gpt_time = 0;

  if(subtimer == GPTIMER_SUBTIMER_A) {
    gpt_time = REG(timer_base | GPTIMER_TAR);
  } else if (subtimer == GPTIMER_SUBTIMER_B) {
    gpt_time = REG(timer_base | GPTIMER_TBR);
  }

  // Check for 16 bit timer
  if(REG(timer_base | GPTIMER_CFG) == 0x04) {
    gpt_time &= 0x0000FFFF;
  }

  return gpt_time;
}

/*
  Returns true if function successfully added to callback array
  Returns false if input parameters are invalid
*/
int
gpt_register_callback(gptimer_callback_t f, uint8_t timer,
                      uint8_t subtimer, uint8_t interrupt_type)
{
  GPTIMER_CHECK_VALID_TIMER(timer);
  GPTIMER_CHECK_VALID_SUBTIMER(subtimer);
  GPTIMER_CHECK_VALID_INTERRUPT_TYPE(interrupt_type);

  uint8_t timer_index = timer * 8;
  uint8_t subtimer_index = subtimer * 4;
  printf("callindex: %u\n", timer_index + subtimer_index + interrupt_type);
  gptimer_callbacks[timer_index + subtimer_index + interrupt_type] = f;

  return 0;
}

static void run_callbacks(uint8_t timer, uint8_t subtimer,
                          uint32_t interrupt_mask)
{

  uint8_t findex = timer*8 + subtimer*4;
  uint32_t gpt_time = get_event_time(timer, subtimer);

  /*
   * Check for each of the possible interrupts and call the correct
   * callbacks if they exist.
   */
  if(subtimer == GPTIMER_SUBTIMER_A) {
    if((interrupt_mask & GPTIMER_MIS_TATOMIS) &&
       gptimer_callbacks[findex+GPTIMER_TIMEOUT_INT]) {
      (*gptimer_callbacks[findex+GPTIMER_TIMEOUT_INT])(gpt_time);
    }
    if((interrupt_mask & GPTIMER_MIS_CAMMIS) &&
       gptimer_callbacks[findex+GPTIMER_CAPTURE_MATCH_INT]) {
      (*gptimer_callbacks[findex+GPTIMER_CAPTURE_MATCH_INT])(gpt_time);
    }
    if((interrupt_mask & GPTIMER_MIS_CAEMIS) &&
       gptimer_callbacks[findex+GPTIMER_CAPTURE_EVENT_INT]) {
      (*gptimer_callbacks[findex+GPTIMER_CAPTURE_EVENT_INT])(gpt_time);
    }
    if((interrupt_mask & GPTIMER_MIS_TAMRIS) &&
       gptimer_callbacks[findex+GPTIMER_MATCH_INT]) {
      (*gptimer_callbacks[findex+GPTIMER_MATCH_INT])(gpt_time);
    }
  } else if(subtimer == GPTIMER_SUBTIMER_B) {
    if((interrupt_mask & GPTIMER_MIS_TBTOMIS) &&
       gptimer_callbacks[findex+GPTIMER_TIMEOUT_INT]) {
      (*gptimer_callbacks[findex+GPTIMER_TIMEOUT_INT])(gpt_time);
    }
    if((interrupt_mask & GPTIMER_MIS_CBMMIS) &&
       gptimer_callbacks[findex+GPTIMER_CAPTURE_MATCH_INT]) {
      (*gptimer_callbacks[findex+GPTIMER_CAPTURE_MATCH_INT])(gpt_time);
    }
    if((interrupt_mask & GPTIMER_MIS_CBEMIS) &&
       gptimer_callbacks[findex+GPTIMER_CAPTURE_EVENT_INT]) {
      (*gptimer_callbacks[findex+GPTIMER_CAPTURE_EVENT_INT])(gpt_time);
    }
    if((interrupt_mask & GPTIMER_MIS_TBMMIS) &&
       gptimer_callbacks[findex+GPTIMER_MATCH_INT]) {
      (*gptimer_callbacks[findex+GPTIMER_MATCH_INT])(gpt_time);
    }
  }
}

inline void
clear_gpt_interrupt(uint32_t timer_base, uint32_t icr_mask)
{
  REG(timer_base | GPTIMER_ICR) &= icr_mask;
}




int
gpt_configure_timer(uint8_t timer, uint32_t config)
{
  GPTIMER_CHECK_VALID_TIMER(timer);
  uint32_t timer_base = TIMER_TO_BASE[timer];
  REG(timer_base | GPTIMER_CFG) = config;
  return 0;
}

static int
gpt_set_mode_register(uint8_t timer, uint8_t subtimer, uint32_t mask,
                      uint32_t value)
{
  GPTIMER_CHECK_VALID_TIMER(timer);
  GPTIMER_CHECK_VALID_SUBTIMER(subtimer);

  uint32_t timer_base = TIMER_TO_BASE[timer];
  uint32_t subtimer_reg;

  if(subtimer == GPTIMER_SUBTIMER_A) {
    subtimer_reg = GPTIMER_TAMR;
  } else if(subtimer == GPTIMER_SUBTIMER_B) {
    subtimer_reg = GPTIMER_TBMR;
  }

  REG(timer_base | subtimer_reg) = (REG(timer_base | subtimer_reg) & ~mask) | value;

  return 0;
}

int
gpt_set_mode(uint8_t timer, uint8_t subtimer, uint32_t mode)
{
  if(mode > 3) return -1;
  return gpt_set_mode_register(timer, subtimer, GPTIMER_TAMR_TAMR, mode);
}

int
gpt_set_capture_mode(uint8_t timer, uint8_t subtimer, uint32_t cap_mode)
{
  if((cap_mode != GPTIMER_TnMR_TnCMR_EDGE_COUNT) &&
     (cap_mode != GPTIMER_TnMR_TnCMR_EDGE_TIME)) {
    return -1;
  }
  return gpt_set_mode_register(timer, subtimer, GPTIMER_TAMR_TACMR, cap_mode);
}

int
gpt_set_alternate_mode(uint8_t timer, uint8_t subtimer, uint32_t alt_mode)
{
  if((alt_mode != GPTIMER_TnMR_TnAMS_CAPTURE_MODE) &&
     (alt_mode != GPTIMER_TnMR_TnAMS_PWM_MODE)) {
    return -1;
  }
  return gpt_set_mode_register(timer, subtimer, GPTIMER_TAMR_TAAMS, alt_mode);
}

int
gpt_set_count_dir(uint8_t timer, uint8_t subtimer, uint32_t count_dir)
{
  if((count_dir != GPTIMER_TnMR_TnCDIR_COUNT_DOWN) &&
     (count_dir != GPTIMER_TnMR_TnCDIR_COUNT_UP)) {
    return -1;
  }
  return gpt_set_mode_register(timer, subtimer, GPTIMER_TAMR_TACDIR, count_dir);
}

static int
gpt_set_control_register(uint8_t timer, uint8_t subtimer, uint32_t mask,
                         uint32_t value)
{
  GPTIMER_CHECK_VALID_TIMER(timer);
  GPTIMER_CHECK_VALID_SUBTIMER(subtimer);

  uint32_t timer_base = TIMER_TO_BASE[timer];

  if(subtimer == GPTIMER_SUBTIMER_B) {
    mask = mask << 8;
    value = value << 8;
  }

  REG(timer_base | GPTIMER_CTL) = (REG(timer_base | GPTIMER_CTL) & ~mask) | value;

  return 0;

}

int
gpt_set_event_mode(uint8_t timer, uint8_t subtimer, uint32_t event_mode) {
  if((event_mode != GPTIMER_CTL_TnEVENT_POSITIVE_EDGE) &&
     (event_mode != GPTIMER_CTL_TnEVENT_NEGATIVE_EDGE) &&
     (event_mode != GPTIMER_CTL_TnEVENT_BOTH_EDGES)) {
    return -1;
  }
  return gpt_set_control_register(timer, subtimer, GPTIMER_CTL_TAEVENT, event_mode);
}

int
gpt_enable_event(uint8_t timer, uint8_t subtimer)
{
  return gpt_set_control_register(timer, subtimer, GPTIMER_CTL_TAEN, GPTIMER_CTL_TAEN);
}

int
gpt_disable_event(uint8_t timer, uint8_t subtimer)
{
  return gpt_set_control_register(timer, subtimer, GPTIMER_CTL_TAEN, 0);
}

static uint32_t
gpt_get_interrupt_mask(uint8_t subtimer, uint8_t interrupt_type)
{
  uint32_t int_mask = 0;

  /*
   * Have to split these up because the bits in the register are not
   * symmetrical.
   */
  if(subtimer == GPTIMER_SUBTIMER_A) {
    switch(interrupt_type) {
      case GPTIMER_TIMEOUT_INT:       int_mask = GPTIMER_IMR_TATOIM; break;
      case GPTIMER_CAPTURE_MATCH_INT: int_mask = GPTIMER_IMR_CAMIM; break;
      case GPTIMER_CAPTURE_EVENT_INT: int_mask = GPTIMER_IMR_CAEIM; break;
      case GPTIMER_MATCH_INT:         int_mask = GPTIMER_IMR_TAMIM; break;
    }
  } else if(subtimer == GPTIMER_SUBTIMER_B) {
    switch(interrupt_type) {
      case GPTIMER_TIMEOUT_INT:       int_mask = GPTIMER_IMR_TBTOIM; break;
      case GPTIMER_CAPTURE_MATCH_INT: int_mask = GPTIMER_IMR_CBMIM; break;
      case GPTIMER_CAPTURE_EVENT_INT: int_mask = GPTIMER_IMR_CBEIM; break;
      case GPTIMER_MATCH_INT:         int_mask = GPTIMER_IMR_TBMIM; break;
    }
  }
  return int_mask;
}

int
gpt_enable_interrupt(uint8_t timer, uint8_t subtimer, uint8_t interrupt_type)
{
  GPTIMER_CHECK_VALID_TIMER(timer);
  GPTIMER_CHECK_VALID_SUBTIMER(subtimer);
  if (interrupt_type > 3) return -1;

  uint32_t timer_base = TIMER_TO_BASE[timer];
  uint32_t int_mask = gpt_get_interrupt_mask(subtimer, interrupt_type);
  REG(timer_base | GPTIMER_IMR) |= int_mask;

  return 0;
}

int
gpt_disable_interrupt(uint8_t timer, uint8_t subtimer, uint8_t interrupt_type)
{
  GPTIMER_CHECK_VALID_TIMER(timer);
  GPTIMER_CHECK_VALID_SUBTIMER(subtimer);
  if (interrupt_type > 3) return -1;

  uint32_t timer_base = TIMER_TO_BASE[timer];
  uint32_t int_mask = gpt_get_interrupt_mask(subtimer, interrupt_type);
  REG(timer_base | GPTIMER_IMR) &= ~int_mask;

  return 0;
}

int
gpt_set_interval_value(uint8_t timer, uint8_t subtimer, uint32_t interval_value)
{
  GPTIMER_CHECK_VALID_TIMER(timer);
  GPTIMER_CHECK_VALID_SUBTIMER(subtimer);

  uint32_t timer_base = TIMER_TO_BASE[timer];
  uint32_t subtimer_mask;

  if(subtimer == GPTIMER_SUBTIMER_A) {
    subtimer_mask = GPTIMER_TAILR;
  } else if(subtimer == GPTIMER_SUBTIMER_B) {
    subtimer_mask = GPTIMER_TBILR;
  }

  REG(timer_base | subtimer_mask) = interval_value;

  return 0;
}

int
gpt_set_match_value(uint8_t timer, uint8_t subtimer, uint32_t match_value)
{
  GPTIMER_CHECK_VALID_TIMER(timer);
  GPTIMER_CHECK_VALID_SUBTIMER(subtimer);

  uint32_t timer_base = TIMER_TO_BASE[timer];
  uint32_t subtimer_mask;

  if(subtimer == GPTIMER_SUBTIMER_A) {
    subtimer_mask = GPTIMER_TAMATCHR;
  } else if(subtimer == GPTIMER_SUBTIMER_B) {
    subtimer_mask = GPTIMER_TBMATCHR;
  }

  REG(timer_base | subtimer_mask) = match_value;

  return 0;
}

void gpt_0_a_isr(void) {
  lpm_exit();
  ENERGEST_ON(ENERGEST_TYPE_IRQ);

  run_callbacks(GPTIMER_0, GPTIMER_SUBTIMER_A, REG(GPT_0_BASE | GPTIMER_MIS));
  //clear_gpt_interrupt(GPTIMER_0_BASE, GPTIMER_ICR_A_MASK);

  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}

void gpt_0_b_isr(void) {
  lpm_exit();
  ENERGEST_ON(ENERGEST_TYPE_IRQ);

  run_callbacks(GPTIMER_0, GPTIMER_SUBTIMER_B, REG(GPT_0_BASE | GPTIMER_MIS));
  //clear_gpt_interrupt(GPTIMER_0_BASE, GPTIMER_ICR_B_MASK);

  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}

void gpt_1_a_isr(void) {
  lpm_exit();
  nvic_interrupt_disable(NVIC_INT_GPTIMER_1A);
  leds_on(LEDS_RED);
  ENERGEST_ON(ENERGEST_TYPE_IRQ);
  printf("GPT: %lu\n", REG(GPT_1_BASE | GPTIMER_MIS));
  run_callbacks(GPTIMER_1, GPTIMER_SUBTIMER_A, REG(GPT_1_BASE | GPTIMER_MIS));
  clear_gpt_interrupt(GPTIMER_1_BASE, GPTIMER_ICR_A_MASK);

  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}

void gpt_1_b_isr(void) {
  lpm_exit();
    ENERGEST_ON(ENERGEST_TYPE_IRQ);

    run_callbacks(GPTIMER_0, GPTIMER_SUBTIMER_B, REG(GPT_0_BASE | GPTIMER_MIS));
    //clear_gpt_interrupt(GPTIMER_1_BASE, GPTIMER_ICR_B_MASK);

    ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}

void gpt_2_a_isr(void) {
  lpm_exit();
    ENERGEST_ON(ENERGEST_TYPE_IRQ);

    run_callbacks(GPTIMER_2, GPTIMER_SUBTIMER_A, REG(GPT_2_BASE | GPTIMER_MIS));
    //clear_gpt_interrupt(GPTIMER_2_BASE, GPTIMER_ICR_A_MASK);

    ENERGEST_OFF(ENERGEST_TYPE_IRQ);

}

void gpt_2_b_isr(void) {
  lpm_exit();
    ENERGEST_ON(ENERGEST_TYPE_IRQ);

    run_callbacks(GPTIMER_0, GPTIMER_SUBTIMER_B, REG(GPT_0_BASE | GPTIMER_MIS));
    //clear_gpt_interrupt(GPTIMER_2_BASE, GPTIMER_ICR_B_MASK);

    ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}

void gpt_3_a_isr(void) {
  lpm_exit();
    ENERGEST_ON(ENERGEST_TYPE_IRQ);

    run_callbacks(GPTIMER_3, GPTIMER_SUBTIMER_A, REG(GPT_3_BASE | GPTIMER_MIS));
    //clear_gpt_interrupt(GPTIMER_3_BASE, GPTIMER_ICR_A_MASK);

    ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}

void gpt_3_b_isr(void) {
  lpm_exit();
    ENERGEST_ON(ENERGEST_TYPE_IRQ);

    run_callbacks(GPTIMER_0, GPTIMER_SUBTIMER_B, REG(GPT_0_BASE | GPTIMER_MIS));
    //clear_gpt_interrupt(GPTIMER_3_BASE, GPTIMER_ICR_B_MASK);

    ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}
