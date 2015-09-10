/**
 * \addtogroup cc2538-rtimer
 * @{
 *
 * \file
 * Implementation of the arch-specific rtimer functions for the cc2538
 *
 */
#include "contiki.h"
#include "sys/energest.h"
#include "dev/nvic.h"
#include "dev/smwdthrosc.h"
#include "cpu.h"
#include "lpm.h"
#include "vtimer-arch.h"
#include "vtimer.h"
#include "dev/leds.h"

#include <stdint.h>
/*---------------------------------------------------------------------------*/
static volatile uint32_t next_trigger;
static bool debug = false;
/*---------------------------------------------------------------------------*/

void vtimer_arch_init(void) {
  next_trigger = 0;
  return;
}

void vtimer_arch_clear() {
    nvic_interrupt_unpend(NVIC_INT_SM_TIMER);
}

void vtimer_arch_cancel() {
    nvic_interrupt_disable(NVIC_INT_SM_TIMER);
    nvic_interrupt_unpend(NVIC_INT_SM_TIMER);
}


/*---------------------------------------------------------------------------*/
/**
 * \brief Schedules an rtimer task to be triggered at time t
 * \param t The time when the task will need executed. This is an absolute
 *          time, in other words the task will be executed AT time \e t,
 *          not IN \e t ticks
 */
void vtimer_arch_schedule(uint32_t t) {
  INTERRUPTS_DISABLE();
  uint32_t now;
  uint32_t st = t;

  if(debug) {

  }
  nvic_interrupt_unpend(NVIC_INT_SM_TIMER); // clear
  /* STLOAD must be 1 */
  while((REG(SMWDTHROSC_STLOAD) & SMWDTHROSC_STLOAD_STLOAD) != 1);

  /*
   * New value must be 5 ticks in the future. The ST may tick once while we're
   * writing the registers. We play it safe here and  add a bit of leeway.
   * UINT math should wrap around the MAX, which should make our arithmetic always work.
   */

  while((REG(SMWDTHROSC_STLOAD) & SMWDTHROSC_STLOAD_STLOAD) != 1) {};
  now = vtimer_arch_now();
  if((uint32_t)(t-now) < 30){
      st = vtimer_arch_now() + 30;
  }
  
  /* ST0 latches ST[1:3] and must be written last */
  REG(SMWDTHROSC_ST3) = (st >> 24) & 0x000000FF;
  REG(SMWDTHROSC_ST2) = (st >> 16) & 0x000000FF;
  REG(SMWDTHROSC_ST1) = (st >> 8) & 0x000000FF;
  REG(SMWDTHROSC_ST0) = st & 0x000000FF;
  nvic_interrupt_unpend(NVIC_INT_SM_TIMER); // clear timer just in case
  /* Store the value. The LPM module will query us for it */
  next_trigger = st;
  INTERRUPTS_ENABLE();
  nvic_interrupt_enable(NVIC_INT_SM_TIMER);
}
/*---------------------------------------------------------------------------*/
uint32_t vtimer_arch_next_trigger() {
  return next_trigger;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Returns the current real-time clock time
 * \return The current rtimer time in ticks
 */
inline uint32_t vtimer_arch_now() {
  uint32_t rv;

  /* SMWDTHROSC_ST0 latches ST[1:3] and must be read first */
  rv = REG(SMWDTHROSC_ST0);
  rv |= (REG(SMWDTHROSC_ST1) << 8);
  rv |= (REG(SMWDTHROSC_ST2) << 16);
  rv |= (REG(SMWDTHROSC_ST3) << 24);

  return rv;
}

void vtimer_arch_debug() {
  debug = true;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief The rtimer ISR
 *
 *        Interrupts are only turned on when we have an rtimer task to schedule
 *        Once the interrupt fires, the task is called and then interrupts no
 *        longer get acknowledged until the next task needs scheduled.
 */
void vtimer_isr() {
  /*
   * If we were in PM1+, call the wake-up sequence first. This will make sure
   * that the 32MHz OSC is selected as the clock source. We need to do this
   * before calling the next rtimer_task, since the task may need the RF.
   */
  lpm_exit();

  next_trigger = 0;
  nvic_interrupt_disable(NVIC_INT_SM_TIMER);
  nvic_interrupt_unpend(NVIC_INT_SM_TIMER);
  vtimer_run_next();

}
/*---------------------------------------------------------------------------*/
/** @} */
