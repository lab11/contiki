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
 * \addtogroup cc2538
 * @{
 *
 * \defgroup cc2538-gptimer cc2538 General-Purpose Timers
 *
 * Driver for the cc2538 General Purpose Timers
 * @{
 *
 * \file
 * Header file for the cc2538 General Purpose Timers
 */
#include <stdint.h>

#ifndef GPTIMER_H_
#define GPTIMER_H_

#define CC2538_NUM_GPTIMERS 4
#define CC2538_NUM_SUBGPTIMERS 2
#define CC2538_NUM_GPTIMER_INTERRUPT_TYPES 4
#define GPTIMER_CHECK_VALID_TIMER(timer_index) \
  do { if (timer_index >= CC2538_NUM_GPTIMERS) return -1; } while(0)
#define GPTIMER_CHECK_VALID_SUBTIMER(subtimer_index) \
  do { if (subtimer_index >= CC2538_NUM_SUBGPTIMERS) return -1; } while(0)
#define GPTIMER_CHECK_VALID_INTERRUPT_TYPE(event) \
  do { if (event >= CC2538_NUM_GPTIMER_INTERRUPT_TYPES) return -1; } while(0)



/*---------------------------------------------------------------------------*/
/** \name prototype for callbacks invoked by gptimer interrupts
 * Subtimer definitions used for indexing interrupt functions
 * @{
 */
#define GPTIMER_0 			   0
#define GPTIMER_1              1
#define GPTIMER_2              2
#define GPTIMER_3              3
#define GPTIMER_SUBTIMER_A	   0
#define GPTIMER_SUBTIMER_B	   1

// Functions
// #define GPTIMER_TIMEOUT_EVENT          0
// #define GPTIMER_CAPTURE_MATCH_EVENT    1
// #define GPTIMER_CAPTURE_EVENT          2
// #define GPTIMER_MATCH_EVENT            3

// #define GPTIMER_ONE_SHOT_MODE	       0x00000001
// #define GPTIMER_PERIODIC_MODE	       0x00000002
// #define GPTIMER_CAPTURE_MODE		   0x00000003

// #define GPTIMER_CAPTURE_MODE_EDGE_COUNT	0x00000000
// #define GPTIMER_CAPTURE_MODE_EDGE_TIME	GPTIMER_TAMR_TACMR

// #define GPTIMER_EVENT_MODE_POSITIVE_EDGE 0x00000000
// #define GPTIMER_EVENT_MODE_NEGATIVE_EDGE 0x00000004
// #define GPTIMER_EVENT_MODE_BOTH_EDGES    0x0000000C

// #define GPTIMER_ALTERNATE_MODE_CAPTURE  0x00000000
// #define GPTIMER_ALTERNATE_MODE_PWM		GPTIMER_TAMR_TAAMS

// #define GPTIMER_COUNT_DIR_DOWN		    0x00000000
// #define GPTIMER_COUNT_DIR_UP			GPTIMER_TAMR_TACDIR

#define GPTIMER_TIMEOUT_INT				0
#define GPTIMER_CAPTURE_MATCH_INT		1
#define GPTIMER_CAPTURE_EVENT_INT		2
#define GPTIMER_MATCH_INT 				3

#define GPTIMER_ICR_A_MASK			    0xFFFEFF00
#define GPTIMER_ICR_B_MASK			    0xFFFE00FF

typedef void (*gptimer_callback_t)(uint32_t gpt_time);

/*---------------------------------------------------------------------------*/
/** \name Base addresses for the GPT register instances
 * @{
 */
#define GPT_0_BASE             0x40030000 /**< GPTIMER0 */
#define GPT_1_BASE             0x40031000 /**< GPTIMER1 */
#define GPT_2_BASE             0x40032000 /**< GPTIMER2 */
#define GPT_3_BASE             0x40033000 /**< GPTIMER3 */
#define GPTIMER_0_BASE		   GPT_0_BASE
#define GPTIMER_1_BASE         GPT_1_BASE
#define GPTIMER_2_BASE		   GPT_2_BASE
#define GPTIMER_3_BASE		   GPT_3_BASE
/** @} */
/*---------------------------------------------------------------------------*/
/** \name GPTIMER Register offset declarations
 * @{
 */
#define GPTIMER_CFG           0x00000000 /**< GPTM configuration */
#define GPTIMER_TAMR          0x00000004 /**< GPTM Timer A mode */
#define GPTIMER_TBMR          0x00000008 /**< GPTM Timer B mode */
#define GPTIMER_CTL           0x0000000C /**< GPTM control */
#define GPTIMER_SYNC          0x00000010 /**< GPTM synchronize (0 only) */
#define GPTIMER_IMR           0x00000018 /**< GPTM interrupt mask */
#define GPTIMER_RIS           0x0000001C /**< GPTM raw interrupt status */
#define GPTIMER_MIS           0x00000020 /**< GPTM masked interrupt status */
#define GPTIMER_ICR           0x00000024 /**< GPTM interrupt clear */
#define GPTIMER_TAILR         0x00000028 /**< GPTM Timer A interval load */
#define GPTIMER_TBILR         0x0000002C /**< GPTM Timer B interval load */
#define GPTIMER_TAMATCHR      0x00000030 /**< GPTM Timer A match */
#define GPTIMER_TBMATCHR      0x00000034 /**< GPTM Timer B match */
#define GPTIMER_TAPR          0x00000038 /**< GPTM Timer A prescale */
#define GPTIMER_TBPR          0x0000003C /**< GPTM Timer B prescale */
#define GPTIMER_TAPMR         0x00000040 /**< GPTM Timer A prescale match */
#define GPTIMER_TBPMR         0x00000044 /**< GPTM Timer B prescale match */
#define GPTIMER_TAR           0x00000048 /**< GPTM Timer A  */
#define GPTIMER_TBR           0x0000004C /**< GPTM Timer B  */
#define GPTIMER_TAV           0x00000050 /**< GPTM Timer A value  */
#define GPTIMER_TBV           0x00000054 /**< GPTM Timer B value */
#define GPTIMER_RTCPD         0x00000058 /**< GPTM RTC predivide  */
#define GPTIMER_TAPS          0x0000005C /**< GPTM Timer A prescale snapshot */
#define GPTIMER_TBPS          0x00000060 /**< GPTM Timer B prescale snapshot */
#define GPTIMER_TAPV          0x00000064 /**< GPTM Timer A prescale value  */
#define GPTIMER_TBPV          0x00000068 /**< GPTM Timer B prescale value  */
#define GPTIMER_PP            0x00000FC0 /**< GPTM peripheral properties  */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name  GPTIMER_CFG register bit values
 * @{
 */
#define GPTIMER_CFG_GPTMCFG_32BIT_TIMER 0x00000000
#define GPTIMER_CFG_GPTMCFG_32BIT_RTC   0x00000001
#define GPTIMER_CFG_GPTMCFG_16BIT_TIMER 0x00000004
/** @} */
/*---------------------------------------------------------------------------*/
/** \name  GPTIMER_CFG register bit masks
 * @{
 */
#define GPTIMER_CFG_GPTMCFG     0x00000007  /**< configuration */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name GPTIMER_TnMR bit values
 * @{
 */
#define GPTIMER_TAMR_TAMR_ONE_SHOT 0x00000001
#define GPTIMER_TAMR_TAMR_PERIODIC 0x00000002
#define GPTIMER_TAMR_TAMR_CAPTURE  0x00000003
#define GPTIMER_TnMR_TnMR_ONE_SHOT 0x00000001
#define GPTIMER_TnMR_TnMR_PERIODIC 0x00000002
#define GPTIMER_TnMR_TnMR_CAPTURE  0x00000003
/** @} */
/*---------------------------------------------------------------------------*/
/** \name GPTIMER_TnMR register bit values
 * @{
 */
#define GPTIMER_TnMR_TnCMR_EDGE_COUNT   0x00000000
#define GPTIMER_TnMR_TnCMR_EDGE_TIME    0x00000004
#define GPTIMER_TnMR_TnAMS_CAPTURE_MODE 0x00000000
#define GPTIMER_TnMR_TnAMS_PWM_MODE     0x00000008
#define GPTIMER_TnMR_TnCDIR_COUNT_DOWN  0x00000000
#define GPTIMER_TnMR_TnCDIR_COUNT_UP    0x00000010
/** @} */
/*---------------------------------------------------------------------------*/
/** \name GPTIMER_TAMR register bit masks
 * @{
 */
#define GPTIMER_TAMR_TAPLO      0x00000800 /**< Legacy PWM operation */
#define GPTIMER_TAMR_TAMRSU     0x00000400 /**< Timer A match register update mode */
#define GPTIMER_TAMR_TAPWMIE    0x00000200 /**< Timer A PWM interrupt enable */
#define GPTIMER_TAMR_TAILD      0x00000100 /**< Timer A PWM interval load write */
#define GPTIMER_TAMR_TASNAPS    0x00000080 /**< Timer A snap-shot mode */
#define GPTIMER_TAMR_TAWOT      0x00000040 /**< Timer A wait-on-trigger */
#define GPTIMER_TAMR_TAMIE      0x00000020 /**< Timer A match interrupt enable */
#define GPTIMER_TAMR_TACDIR     0x00000010 /**< Timer A count direction */
#define GPTIMER_TAMR_TAAMS      0x00000008 /**< Timer A alternate mode */
#define GPTIMER_TAMR_TACMR      0x00000004 /**< Timer A capture mode */
#define GPTIMER_TAMR_TAMR       0x00000003 /**< Timer A mode */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name GPTIMER_TBMR register bit masks
 * @{
 */
#define GPTIMER_TBMR_TBPLO      0x00000800 /**< Legacy PWM operation */
#define GPTIMER_TBMR_TBMRSU     0x00000400 /**< Timer B match register update mode */
#define GPTIMER_TBMR_TBPWMIE    0x00000200 /**< Timer B PWM interrupt enable */
#define GPTIMER_TBMR_TBILD      0x00000100 /**< Timer B PWM interval load write */
#define GPTIMER_TBMR_TBSNAPS    0x00000080 /**< Timer B snap-shot mode */
#define GPTIMER_TBMR_TBWOT      0x00000040 /**< Timer B wait-on-trigger */
#define GPTIMER_TBMR_TBMIE      0x00000020 /**< Timer B match interrupt enable */
#define GPTIMER_TBMR_TBCDIR     0x00000010 /**< Timer B count direction */
#define GPTIMER_TBMR_TBAMS      0x00000008 /**< Timer B alternate mode */
#define GPTIMER_TBMR_TBCMR      0x00000004 /**< Timer B capture mode */
#define GPTIMER_TBMR_TBMR       0x00000003 /**< Timer B mode */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name GPTIMER_CTL register bit values
 * @{
 */
#define GPTIMER_CTL_TnEVENT_POSITIVE_EDGE 0x00000000
#define GPTIMER_CTL_TnEVENT_NEGATIVE_EDGE 0x00000004
#define GPTIMER_CTL_TnEVENT_BOTH_EDGES    0x0000000C
/** @} */
/*---------------------------------------------------------------------------*/
/** \name GPTIMER_CTL register bit masks
 * @{
 */
#define GPTIMER_CTL_TBPWML      0x00004000 /**< Timer B PWM output level */
#define GPTIMER_CTL_TBOTE       0x00002000 /**< Timer B output trigger enable */
#define GPTIMER_CTL_TBEVENT     0x00000C00 /**< Timer B event mode */
#define GPTIMER_CTL_TBSTALL     0x00000200 /**< Timer B stall enable */
#define GPTIMER_CTL_TBEN        0x00000100 /**< Timer B enable */
#define GPTIMER_CTL_TAPWML      0x00000040 /**< Timer A PWM output level */
#define GPTIMER_CTL_TAOTE       0x00000020 /**< Timer A output trigger enable */
#define GPTIMER_CTL_RTCEN       0x00000010 /**< RTC enable */
#define GPTIMER_CTL_TAEVENT     0x0000000C /**< Timer A event mode */
#define GPTIMER_CTL_TASTALL     0x00000002 /**< Timer A stall enable */
#define GPTIMER_CTL_TAEN        0x00000001 /**< Timer A enable */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name GPTIMER_SYNC register bit masks
 * @{
 */
#define GPTIMER_SYNC_SYNC3      0x000000C0 /**< Synchronize GPTM3 */
#define GPTIMER_SYNC_SYNC2      0x00000030 /**< Synchronize GPTM2 */
#define GPTIMER_SYNC_SYNC1      0x0000000C /**< Synchronize GPTM1 */
#define GPTIMER_SYNC_SYNC0      0x00000003 /**< Synchronize GPTM0 */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name GPTIMER_IMR register bit masks
 * @{
 */
#define GPTIMER_IMR_TBMIM       0x00000800 /**< Timer B match int mask */
#define GPTIMER_IMR_CBEIM       0x00000400 /**< Timer B capture event int mask */
#define GPTIMER_IMR_CBMIM       0x00000200 /**< Timer B capture match int mask */
#define GPTIMER_IMR_TBTOIM      0x00000100 /**< Timer B time-out int mask */
#define GPTIMER_IMR_TAMIM       0x00000010 /**< Timer A match int mask */
#define GPTIMER_IMR_RTCIM       0x00000008 /**< RTC int mask */
#define GPTIMER_IMR_CAEIM       0x00000004 /**< Timer A capture event int mask */
#define GPTIMER_IMR_CAMIM       0x00000002 /**< Timer A capture match int mask */
#define GPTIMER_IMR_TATOIM      0x00000001 /**< Timer A time-out int mask */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name GPTIMER_RIS register bit masks
 * @{
 */
#define GPTIMER_RIS_TBMRIS      0x00000800 /**< Timer B match raw status */
#define GPTIMER_RIS_CBERIS      0x00000400 /**< Timer B capture event raw status */
#define GPTIMER_RIS_CBMRIS      0x00000200 /**< Timer B capture match raw status */
#define GPTIMER_RIS_TBTORIS     0x00000100 /**< Timer B time-out raw status */
#define GPTIMER_RIS_TAMRIS      0x00000010 /**< Timer A match raw status */
#define GPTIMER_RIS_RTCRIS      0x00000008 /**< RTC raw status */
#define GPTIMER_RIS_CAERIS      0x00000004 /**< Timer A capture event raw status */
#define GPTIMER_RIS_CAMRIS      0x00000002 /**< Timer A capture match raw status */
#define GPTIMER_RIS_TATORIS     0x00000001 /**< Timer A time-out raw status */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name GPTIMER_MIS register bit masks
 * @{
 */
#define GPTIMER_MIS_TBMMIS      0x00000800 /**< Timer B match masked status */
#define GPTIMER_MIS_CBEMIS      0x00000400 /**< Timer B capture event masked status */
#define GPTIMER_MIS_CBMMIS      0x00000200 /**< Timer B capture match masked status */
#define GPTIMER_MIS_TBTOMIS     0x00000100 /**< Timer B time-out masked status */
#define GPTIMER_MIS_TAMRIS      0x00000010 /**< Timer A match masked status */
#define GPTIMER_MIS_RTCMIS      0x00000008 /**< RTC masked status */
#define GPTIMER_MIS_CAEMIS      0x00000004 /**< Timer A capture event masked status */
#define GPTIMER_MIS_CAMMIS      0x00000002 /**< Timer A capture match masked status */
#define GPTIMER_MIS_TATOMIS     0x00000001 /**< Timer A time-out masked status */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name GPTIMER_ICR register bit masks
 * @{
 */
#define GPTIMER_ICR_WUECINT     0x00010000 /**< write update error int clear */
#define GPTIMER_ICR_TBMCINT     0x00000800 /**< Timer B match int clear */
#define GPTIMER_ICR_CBECINT     0x00000400 /**< Timer B capture event int clear */
#define GPTIMER_ICR_CBMCINT     0x00000200 /**< Timer B capture match int clear */
#define GPTIMER_ICR_TBTOCINT    0x00000100 /**< Timer B time-out int clear */
#define GPTIMER_ICR_TAMCINT     0x00000010 /**< Timer A match int clear */
#define GPTIMER_ICR_RTCCINT     0x00000008 /**< RTC interrupt clear */
#define GPTIMER_ICR_CAECINT     0x00000004 /**< Timer A capture event int clear */
#define GPTIMER_ICR_CAMCINT     0x00000002 /**< Timer A capture match int clear */
#define GPTIMER_ICR_TATOCINT    0x00000001 /**< Timer A time-out int clear */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name GPTIMER_TAILR register bit masks
 * @{
 */
#define GPTIMER_TAILR_TAILR     0xFFFFFFFF /**< A interval load register */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name GPTIMER_TBILR register bit masks
 * @{
 */
#define GPTIMER_TBILR_TBILR     0x0000FFFF /**< B interval load register */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name GPTIMER_TAMATCHR register bit masks
 * @{
 */
#define GPTIMER_TAMATCHR_TAMR   0xFFFFFFFF /**< Timer A match register */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name GPTIMER_TBMATCHR register bit masks
 * @{
 */
#define GPTIMER_TBMATCHR_TBMR   0x0000FFFF /**< Timer B match register */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name GPTIMER_TAPR register bit masks
 * @{
 */
#define GPTIMER_TAPR_TAPSR      0x000000FF /**< Timer A prescale */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name GPTIMER_TBPR register bit masks
 * @{
 */
#define GPTIMER_TBPR_TBPSR      0x000000FF /**< Timer B prescale */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name GPTIMER_TAPMR register bit masks
 * @{
 */
#define GPTIMER_TAPMR_TAPSR     0x000000FF /**< Timer A prescale match */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name GPTIMER_TBPMR register bit masks
 * @{
 */
#define GPTIMER_TBPMR_TBPSR     0x000000FF /**< Timer B prescale match */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name GPTIMER_TAR register bit masks
 * @{
 */
#define GPTIMER_TAR_TAR         0xFFFFFFFF /**< Timer A register */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name GPTIMER_TBR register bit masks
 * @{
 */
#define GPTIMER_TBR_TBR         0x0000FFFF /**< Timer B register */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name GPTIMER_TAV register bit masks
 * @{
 */
#define GPTIMER_TAV_TAV         0xFFFFFFFF /**< Timer A register */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name GPTIMER_TBV register bit masks
 * @{
 */
#define GPTIMER_TBV_PRE         0x00FF0000 /**< Timer B prescale register */
#define GPTIMER_TBV_TBV         0x0000FFFF /**< Timer B register */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name GPTIMER_RTCPD register bit masks
 * @{
 */
#define GPTIMER_RTCPD_RTCPD     0x0000FFFF /**< RTC predivider */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name GPTIMER_TAPS register bit masks
 * @{
 */
#define GPTIMER_TAPS_PSS        0x0000FFFF /**< Timer A prescaler */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name GPTIMER_TBPS register bit masks
 * @{
 */
#define GPTIMER_TBPS_PSS        0x0000FFFF /**< Timer B prescaler */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name GPTIMER_TAPV register bit masks
 * @{
 */
#define GPTIMER_TAPV_PSV        0x0000FFFF /**< Timer A prescaler value */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name GPTIMER_TBPV register bit masks
 * @{
 */
#define GPTIMER_TBPV_PSV        0x0000FFFF /**< Timer B prescaler value */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name GPTIMER_PP register bit masks
 * @{
 */
#define GPTIMER_PP_ALTCLK       0x00000040 /**< Alternate clock source */
#define GPTIMER_PP_SYNCNT       0x00000020 /**< Synchronized start */
#define GPTIMER_PP_CHAIN        0x00000010 /**< Chain with other timers */
#define GPTIMER_PP_SIZE         0x0000000F /**< Timer size */
/** @} */




int ungate_gpt(uint8_t timer);

int gate_gpt(uint8_t timer);

int ungate_gpt_running(uint8_t timer);

int ungate_gpt_sleeping(uint8_t timer);

int ungate_gpt_pm0(uint8_t timer);

int gate_gpt_running(uint8_t timer);

int gate_gpt_sleeping(uint8_t timer);

int gate_gpt_pm0(uint8_t timer);



/*
  Returns true if function successfully added to callback array
  Returns false if input parameters are invalid
*/
int gpt_register_callback(gptimer_callback_t f, uint8_t timer,
                      uint8_t subtimer, uint8_t interrupt_type);
void clear_gpt_interrupt(uint32_t timer_base, uint32_t icr_mask);

int gpt_configure_timer(uint8_t timer, uint32_t config);

int gpt_set_mode(uint8_t timer, uint8_t subtimer, uint32_t mode);

int gpt_set_capture_mode(uint8_t timer, uint8_t subtimer, uint32_t cap_mode);

int gpt_set_alternate_mode(uint8_t timer, uint8_t subtimer, uint32_t alt_mode);

int gpt_set_count_dir(uint8_t timer, uint8_t subtimer, uint32_t count_dir);



int gpt_set_event_mode(uint8_t timer, uint8_t subtimer, uint32_t event_mode);

int gpt_enable_event(uint8_t timer, uint8_t subtimer);
int gpt_disable_event(uint8_t timer, uint8_t subtimer);



int gpt_enable_interrupt(uint8_t timer, uint8_t subtimer, uint8_t interrupt_type);

int gpt_disable_interrupt(uint8_t timer, uint8_t subtimer, uint8_t int_type);

int gpt_set_interval_value(uint8_t timer, uint8_t subtimer, uint32_t interval_value);

int gpt_set_match_value(uint8_t timer, uint8_t subtimer, uint32_t match_value);


void gpt_0_a_isr();

void gpt_0_b_isr();

void gpt_1_a_isr();

void gpt_1_b_isr();

void gpt_2_a_isr();

void gpt_2_b_isr();

void gpt_3_a_isr();

void gpt_3_b_isr();

uint32_t get_event_time(uint8_t timer, uint8_t subtimer);
#endif /* GPTIMER_H_ */

/**
 * @}
 * @}
 */
