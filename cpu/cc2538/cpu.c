/*
 * Copyright (c) 2012, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \addtogroup cc2538-cpu
 * @{
 *
 * \file
 * Implementations of interrupt control on the cc2538 Cortex-M3 micro
 */
/*---------------------------------------------------------------------------*/

/*
 * Modfiied version of the original file that supports nesting INTERRUPT_DISABLES and INTERRUPT_ENABLES
 * @author WIlliam Huang <wwhuang@umich.edu>
*/

#include "dev/leds.h"
#include <stdint.h>
static uint32_t interrupt_counter = 0;

/* enable interrupts */
unsigned long cpu_cpsie(void) {
  if(interrupt_counter > 0) {interrupt_counter--;}

  if(interrupt_counter == 0) {
      /* enable interrupts */
      leds_off(LEDS_GREEN);
      __asm("cpsie   i\n");
      return 1;
  }
  return 2;
}
/*---------------------------------------------------------------------------*/
/* disable interrupts */
unsigned long cpu_cpsid(void) {
    if(interrupt_counter < UINT32_MAX) {interrupt_counter++;}
    else {return 0;}

  if(interrupt_counter == 1) {
      /* disable interrupts */
      leds_on(LEDS_GREEN);
      __asm("cpsid   i\n");
      return 1;
  }
  return 2;
}

/*---------------------------------------------------------------------------*/
/** @} */
