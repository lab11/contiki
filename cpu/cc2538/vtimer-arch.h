/**
 * \addtogroup cc2538
 * @{
 *
 * \defgroup cc2538-vtimer cc2538 vtimer
 *
 * Implementation of the vtimer module for the cc2538
 *
 * The vtimer runs on the Sleep Timer. This is a design choice, as many parts
 * of Contiki like vtimers with a value of vtimer_ARCH_SECOND being a power of
 * two. The ST runs on the 32kHz clock, which can provide us with an excellent
 * value of 32768 for vtimer_ARCH_SECOND.
 *
 * Additionally, since the ST keeps running in PM2, we can do things like drop
 * to PM2 and schedule a wake-up time through the vtimer API.
 *
 * \note If the 32kHz clock is running on the 32kHz RC OSC, the vtimer is
 * not 100% accurate (the RC OSC does not run at exactly 32.768 kHz). For
 * applications requiring higher accuracy, the 32kHz clock should be changed to
 * use the XOSC as its source. To see which low-frequency OSC the 32kHz clock
 * is running on, see cpu/cc2538/clock.c.
 *
 * \sa cpu/cc2538/clock.c
 * @{
 */
/**
 * \file
 * Header file for the cc2538 vtimer driver
 */
#ifndef VTIMER_ARCH_H_
#define VTIMER_ARCH_H_

#include "contiki.h"
#include "dev/gptimer.h"

#define VTIMER_ARCH_SECOND 32768

/** \sa vtimer_NOW() */
uint32_t vtimer_arch_now(void);

/**
 * \brief Get the time of the next scheduled vtimer trigger
 * \return The time next vtimer ISR is scheduled for
 */
uint32_t vtimer_arch_next_trigger(void);

void vtimer_arch_schedule(uint32_t t);
void vtimer_arch_init();
void vtimer_isr();
void vtimer_arch_debug();
void vtimer_arch_clear(); // clear hardware interrupt.
void vtimer_arch_cancel();

#endif /* VTIMER_ARCH_H_ */

/**
 * @}
 * @}
 */
