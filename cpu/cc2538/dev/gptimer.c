#include "gptimer.h"
#include "gpio.h"
#include "reg.h"
#include "sys/energest.h"
#include "dev/sys-ctrl.h"
#include "lpm.h"
#include "dev/leds.h"
#include <stdint.h>
#include <stdio.h>
#include "net/netstack.h"
#include "net/packetbuf.h"

static gptimer_callback_t gptimer_callbacks[32] = {0};
static void (*testgptf)(void) = NULL;
static uint8_t subtimer_a_masks[4] = {1,2,4,16};
static uint8_t subtimer_b_masks[4] = {1,2,4,8};
static uint8_t timer_to_sys_ctl[4] = {SYS_CTRL_RCGCGPT_GPT0, SYS_CTRL_RCGCGPT_GPT1, SYS_CTRL_RCGCGPT_GPT2, SYS_CTRL_RCGCGPT_GPT3};
static uint32_t timer_to_base[4] = {GPTIMER_0_BASE, GPTIMER_1_BASE, GPTIMER_2_BASE, GPTIMER_3_BASE};

//Interrupt masks for enabling, clearing, masking, and checking GPT interrupts
//Sorted by subtimer (A and B)
//In order: time-out, capture match, capture event, match
static uint32_t interrupt_masks[2][4] = {{GPTIMER_IMR_TATOIM, GPTIMER_IMR_CAMIM, GPTIMER_IMR_CAEIM, GPTIMER_IMR_TAMIM},
										 {GPTIMER_IMR_TBTOIM, GPTIMER_IMR_CBMIM, GPTIMER_IMR_CBEIM, GPTIMER_IMR_TBMIM}};

uint8_t ungate_gpt(uint8_t timer) {
	if(timer < 4) {
		ungate_gpt_running(timer);
		ungate_gpt_sleeping(timer);
		ungate_gpt_pm0(timer);
		return 1;
	}
	else {
		return 0;
	}
}

uint8_t gate_gpt(uint8_t timer) {
	if(timer < 4) {
		gate_gpt_running(timer);
		gate_gpt_sleeping(timer);
		gate_gpt_pm0(timer);
		return 1;
	}
	else {
		return 0;
	}
}

uint8_t ungate_gpt_running(uint8_t timer) {
	if(timer < 4) {
		uint32_t t = timer_to_sys_ctl[timer];
		REG(SYS_CTRL_RCGCGPT) |= t;
		return 1;
	}
	else {
		return 0;
	}
}

uint8_t ungate_gpt_sleeping(uint8_t timer) {
	if(timer < 4) {
		uint32_t t = timer_to_sys_ctl[timer];
		REG(SYS_CTRL_SCGCGPT) |= t;
		return 1;
	}
	else {
		return 0;
	}
}

uint8_t ungate_gpt_pm0(uint8_t timer) {
	if(timer < 4) {
		uint32_t t = timer_to_sys_ctl[timer];
		REG(SYS_CTRL_DCGCGPT) |= t;
		return 1;
	}
	else {
		return 0;
	}
}

uint8_t gate_gpt_running(uint8_t timer) {
	if(timer < 4) {
		uint32_t t = timer_to_sys_ctl[timer];
		REG(SYS_CTRL_RCGCGPT) &= ~t;
		return 1;
	}
	else {
		return 0;
	}
}

uint8_t gate_gpt_sleeping(uint8_t timer) {
	if(timer < 4) {
		uint32_t t = timer_to_sys_ctl[timer];
		REG(SYS_CTRL_SCGCGPT) &= ~t;
		return 1;
	}
	else {
		return 0;
	}
}

uint8_t gate_gpt_pm0(uint8_t timer) {
	if(timer < 4) {
		uint32_t t = timer_to_sys_ctl[timer];
		REG(SYS_CTRL_DCGCGPT) &= ~t;
		return 1;
	}
	else {
		return 0;
	}
}

void gpt_register_test_callback(void *f) {
	testgptf = f;
}

/*
	Returns true if function successfully added to callback array
	Returns false if input parameters are invalid
*/
uint8_t gpt_register_callback(gptimer_callback_t f, uint8_t timer,
							   uint8_t subtimer, uint8_t function) {

	if(timer <= 3 && subtimer < 2 && function <= 3) {
		uint8_t timer_index = timer * 8;
		uint8_t subtimer_index = subtimer * 4;
		printf("callindex: %u\n", timer_index + subtimer_index + function);
		gptimer_callbacks[timer_index + subtimer_index + function] = f;
		return 1;
	}
	return 0;
}

static void run_callbacks(uint8_t timer,
				   	      uint8_t subtimer,
				          uint8_t interrupt_mask,
				          uint8_t *subtimer_masks) {

	uint8_t findex = timer*8 + subtimer*4;
	gptimer_callback_t f;
	uint8_t i = 0;

	for(i=0;i<4;i++) {
		if(subtimer_masks[i] & interrupt_mask) {
			f = gptimer_callbacks[findex+i];
			if(f != NULL) {
				uint32_t gpt_time = get_event_time(timer, subtimer);
        		(f)(timer, subtimer, i, gpt_time);
      		}
		}
	}
}

void gpt_clear_interrupt(uint8_t timer, uint32_t icr_mask) {
	uint32_t timer_base = timer_to_base[timer];
	REG(timer_base | GPTIMER_ICR) |= icr_mask;
}

uint32_t get_event_time(uint8_t timer, uint8_t subtimer) {
	uint32_t timer_base = timer_to_base[timer];
	uint32_t subtimer_mask = GPTIMER_TAR;
	uint32_t gpt_time = 0;

	if(subtimer == GPTIMER_SUBTIMER_B) {
		subtimer_mask = GPTIMER_TBR;
	}

	gpt_time = REG(timer_base | subtimer_mask);

	// Check for 16 bit timer
	if(REG(timer_base | GPTIMER_CFG) == 0x04) {
		gpt_time &= 0x0000FFFF;
	}

	return gpt_time;
}


uint8_t gpt_set_16_bit_timer(uint8_t timer) {
	if(timer < 4) {
		uint32_t timer_base = timer_to_base[timer];
		REG(timer_base) = 0x04;
		return 1;
	}
	return 0;
}

uint8_t gpt_set_32_bit_timer(uint8_t timer) {
	if(timer < 4) {
		uint32_t timer_base = timer_to_base[timer];
		REG(timer_base) = 0x00;
		return 1;
	}
	return 0;
}

uint8_t gpt_set_32_bit_rtc(uint8_t timer) {
	if(timer < 4) {
		uint32_t timer_base = timer_to_base[timer];
		REG(timer_base) = 0x01;
		return 1;
	}
	return 0;
}

/*

*/
uint8_t gpt_set_mode(uint8_t timer, uint8_t subtimer, uint8_t mode) {
	if(timer < 4 && subtimer < 2 && mode < 4 && mode > 0) {
		uint32_t timer_base = timer_to_base[timer];
		uint32_t subtimer_reg = GPTIMER_TAMR;

		if(subtimer == GPTIMER_SUBTIMER_B) {
			subtimer_reg = GPTIMER_TBMR;
		}

		REG(timer_base | subtimer_reg) &= ~GPTIMER_TAMR_TAMR; // clear bitfield
		REG(timer_base | subtimer_reg) |= mode;
		return 1;
	}
	return 0;
}

uint8_t gpt_set_capture_mode(uint8_t timer, uint8_t subtimer, uint32_t cap_mode) {
	if(timer < 4 &&
	   subtimer < 2 &&
	   (cap_mode == GPTIMER_CAPTURE_MODE_EDGE_COUNT || cap_mode == GPTIMER_CAPTURE_MODE_EDGE_TIME)) {

		uint32_t timer_base = timer_to_base[timer];
		uint32_t subtimer_reg = GPTIMER_TAMR;

		if(subtimer == GPTIMER_SUBTIMER_B) {
			subtimer_reg = GPTIMER_TBMR;
		}

		REG(timer_base | subtimer_reg) &= ~GPTIMER_TAMR_TACMR; // clear bitfield
		REG(timer_base | subtimer_reg) |= cap_mode;

		return 1;
	}
	return 0;
}

uint8_t gpt_set_alternate_mode(uint8_t timer, uint8_t subtimer, uint32_t alt_mode) {
	if(timer < 4 &&
	   subtimer < 2 &&
	   (alt_mode == GPTIMER_ALTERNATE_MODE_CAPTURE || alt_mode == GPTIMER_ALTERNATE_MODE_PWM)) {

		uint32_t timer_base = timer_to_base[timer];
		uint32_t subtimer_reg = GPTIMER_TAMR;

		if(subtimer == GPTIMER_SUBTIMER_B) {
			subtimer_reg = GPTIMER_TBMR;
		}

		REG(timer_base | subtimer_reg) &= ~GPTIMER_TAMR_TAAMS; // clear bitfield
		REG(timer_base | subtimer_reg) |= alt_mode;

		return 1;
	}
	return 0;
}

uint8_t gpt_set_count_dir(uint8_t timer, uint8_t subtimer, uint32_t count_dir) {
	if(timer < 4 &&
	   subtimer < 2 &&
	   (count_dir == GPTIMER_COUNT_DIR_DOWN || count_dir == GPTIMER_COUNT_DIR_UP)) {

		uint32_t timer_base = timer_to_base[timer];
		uint32_t subtimer_reg = GPTIMER_TAMR;

		if(subtimer == GPTIMER_SUBTIMER_B) {
			subtimer_reg = GPTIMER_TBMR;
		}

		REG(timer_base | subtimer_reg) &= ~GPTIMER_TAMR_TACDIR; // clear bitfield
		REG(timer_base | subtimer_reg) |= count_dir;

		return 1;
	}
	return 0;
}

uint8_t gpt_set_event_mode(uint8_t timer, uint8_t subtimer, uint32_t event_mode) {
	if(timer < 4 &&
	   subtimer < 2 &&
	   (event_mode == GPTIMER_EVENT_MODE_POSITIVE_EDGE || event_mode == GPTIMER_EVENT_MODE_NEGATIVE_EDGE || event_mode == GPTIMER_EVENT_MODE_BOTH_EDGES)) {

		uint32_t timer_base = timer_to_base[timer];
		uint32_t subtimer_mask = GPTIMER_CTL_TAEVENT;
		uint32_t event_mode_mask = event_mode;

		if(subtimer == GPTIMER_SUBTIMER_B) {
			subtimer_mask = GPTIMER_CTL_TBEVENT;
			event_mode_mask = event_mode << 7;
		}

		REG(timer_base | GPTIMER_CTL) &= ~subtimer_mask; // clear bitfield
		REG(timer_base | GPTIMER_CTL) |= event_mode_mask;

		return 1;
	}
	return 0;
}

uint8_t gpt_enable_event(uint8_t timer, uint8_t subtimer) {
	if(timer < 4 && subtimer < 2) {
		uint32_t timer_base = timer_to_base[timer];
		uint32_t subtimer_mask = GPTIMER_CTL_TAEN;

		if(subtimer == GPTIMER_SUBTIMER_B) {
			subtimer_mask = GPTIMER_CTL_TBEN;
		}

		REG(timer_base | GPTIMER_CTL) |= subtimer_mask;

		return 1;
	}
	return 0;
}

uint8_t gpt_disable_event(uint8_t timer, uint8_t subtimer) {
	if(timer < 4 && subtimer < 2) {
		uint32_t timer_base = timer_to_base[timer];
		uint32_t subtimer_mask = GPTIMER_CTL_TAEN;

		if(subtimer == GPTIMER_SUBTIMER_B) {
			subtimer_mask = GPTIMER_CTL_TBEN;
		}

		REG(timer_base | GPTIMER_CTL) &= ~subtimer_mask;

		return 1;
	}
	return 0;
}

uint8_t gpt_enable_interrupt(uint8_t timer, uint8_t subtimer, uint8_t int_type) {
	if(timer < 4 && subtimer < 2 && int_type < 4) {
		uint32_t timer_base = timer_to_base[timer];
		uint32_t int_mask = interrupt_masks[subtimer][int_type];

		REG(timer_base | GPTIMER_IMR) |= int_mask;

		return 1;
	}
	return 0;
}

uint8_t gpt_disable_interrupt(uint8_t timer, uint8_t subtimer, uint8_t int_type) {
	if(timer < 4 && subtimer < 2 && int_type < 4) {
		uint32_t timer_base = timer_to_base[timer];
		uint32_t int_mask = interrupt_masks[subtimer][int_type];

		REG(timer_base | GPTIMER_IMR) &= ~int_mask;

		return 1;
	}
	return 0;
}

uint8_t gpt_set_interval_value(uint8_t timer, uint8_t subtimer, uint32_t interval_value) {
	if(timer < 4 && subtimer < 2) {
		uint32_t timer_base = timer_to_base[timer];
		uint32_t subtimer_mask = GPTIMER_TAILR;

		if(subtimer == GPTIMER_SUBTIMER_B) {
			subtimer_mask = GPTIMER_TBILR;
		}

		REG(timer_base | subtimer_mask) = 0;
		REG(timer_base | subtimer_mask) = interval_value;

		return 1;
	}
	return 0;
}

uint8_t gpt_set_match_value(uint8_t timer, uint8_t subtimer, uint32_t match_value) {
	if(timer < 4 && subtimer < 2) {
		uint32_t timer_base = timer_to_base[timer];
		uint32_t subtimer_mask = GPTIMER_TAMATCHR;

		if(subtimer == GPTIMER_SUBTIMER_B) {
			subtimer_mask = GPTIMER_TBMATCHR;
		}

		REG(timer_base | subtimer_mask) = 0;
		REG(timer_base | subtimer_mask) = match_value;

		return 1;
	}
	return 0;
}

void gpt_0_a_isr(void) {
	lpm_exit();
  	ENERGEST_ON(ENERGEST_TYPE_IRQ);

  	run_callbacks(GPTIMER_0, GPTIMER_SUBTIMER_A, REG(GPT_0_BASE | GPTIMER_MIS), &subtimer_a_masks[0]);

  	ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}

void gpt_0_b_isr(void) {
	lpm_exit();
  	ENERGEST_ON(ENERGEST_TYPE_IRQ);

  	run_callbacks(GPTIMER_0, GPTIMER_SUBTIMER_B, REG(GPT_0_BASE | GPTIMER_MIS) >> 8, &subtimer_b_masks[0]);

  	ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}

void gpt_1_a_isr(void) {
	lpm_exit();

  	ENERGEST_ON(ENERGEST_TYPE_IRQ);

  	run_callbacks(GPTIMER_1, GPTIMER_SUBTIMER_A, REG(GPT_1_BASE | GPTIMER_MIS), &subtimer_a_masks[0]);

  	ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}

void gpt_1_b_isr(void) {
	lpm_exit();
  	ENERGEST_ON(ENERGEST_TYPE_IRQ);
  	//GPIO_SET_PIN(GPIO_C_BASE, 0x20);

  	run_callbacks(GPTIMER_1, GPTIMER_SUBTIMER_B, REG(GPT_1_BASE | GPTIMER_MIS) >> 8, &subtimer_b_masks[0]);

  	ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}

void gpt_2_a_isr(void) {
	lpm_exit();
  	ENERGEST_ON(ENERGEST_TYPE_IRQ);

  	run_callbacks(GPTIMER_2, GPTIMER_SUBTIMER_A, REG(GPT_2_BASE | GPTIMER_MIS), &subtimer_a_masks[0]);

  	ENERGEST_OFF(ENERGEST_TYPE_IRQ);

}

void gpt_2_b_isr(void) {
	lpm_exit();
  	ENERGEST_ON(ENERGEST_TYPE_IRQ);

  	run_callbacks(GPTIMER_2, GPTIMER_SUBTIMER_B, REG(GPT_2_BASE | GPTIMER_MIS) >> 8, &subtimer_b_masks[0]);

  	ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}

void gpt_3_a_isr(void) {
	lpm_exit();
  	ENERGEST_ON(ENERGEST_TYPE_IRQ);

  	run_callbacks(GPTIMER_3, GPTIMER_SUBTIMER_A, REG(GPT_3_BASE | GPTIMER_MIS), &subtimer_a_masks[0]);

  	ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}

void gpt_3_b_isr(void) {
	lpm_exit();
  	ENERGEST_ON(ENERGEST_TYPE_IRQ);

  	run_callbacks(GPTIMER_3, GPTIMER_SUBTIMER_B, REG(GPT_3_BASE | GPTIMER_MIS) >> 8, &subtimer_b_masks[0]);

  	ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}