/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/zephyr.h>
#include <zephyr/drivers/gpio.h>
#include <nrfx_pwm.h>
#include <nrfx_dppi.h>
#include <nrfx_egu.h>
#include <nrfx_timer.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

#define PWM_COUNTERTOP 100

#define PWM_INVERTED(a) ((a) | 0x8000) 

#define PWM_STEP_SUBDIV 6
#define PWM_STEP_COUNT  6
#define PWM_BUF_LENGTH (PWM_STEP_COUNT*PWM_STEP_SUBDIV)

static nrfx_pwm_t m_pwm_ab = NRFX_PWM_INSTANCE(1);
static nrfx_pwm_t m_pwm_c  = NRFX_PWM_INSTANCE(2);
static nrfx_egu_t m_egu = NRFX_EGU_INSTANCE(2);
static nrfx_timer_t m_timer_pwm_step = NRFX_TIMER_INSTANCE(1);

K_SEM_DEFINE(m_sem_update_pwm_buf, 0, 1);
static int m_pwm_update_buf_index;

void nrfx_pwm_handler(nrfx_pwm_evt_type_t event_type, void * p_context)
{
	int pwm_index = (int)p_context;
	switch(event_type) {
		case NRFX_PWM_EVT_END_SEQ0:
			//printk("ENDSEQ0-C%i\n", pwm_index);
			if(pwm_index == 0) {
				m_pwm_update_buf_index = 0;
				k_sem_give(&m_sem_update_pwm_buf);
			}
			break;
		case NRFX_PWM_EVT_END_SEQ1:
			//printk("ENDSEQ1-C%i\n", pwm_index);
			if(pwm_index == 0) {
				m_pwm_update_buf_index = 1;
				k_sem_give(&m_sem_update_pwm_buf);
			}
			break;
		default:
			break;
	}
}

static int pwm_init(void)
{
	static nrfx_pwm_config_t const config0 =
    {
        .output_pins =
        {
            28,
            29,	
            30,	
            31	
        },
        .irq_priority = 5,
        .base_clock   = NRF_PWM_CLK_1MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = PWM_COUNTERTOP,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_TRIGGERED
    };
	static nrfx_pwm_config_t const config1 =
    {
        .output_pins =
        {
            11,
            12,
            NRFX_PWM_PIN_NOT_USED,
            NRFX_PWM_PIN_NOT_USED
        },
        .irq_priority = 5,
        .base_clock   = NRF_PWM_CLK_1MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = PWM_COUNTERTOP,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_TRIGGERED
    };

    nrfx_pwm_init(&m_pwm_ab, &config0, nrfx_pwm_handler, (void*)0);
    nrfx_pwm_init(&m_pwm_c,  &config1, nrfx_pwm_handler, (void*)1);

	// If PWM callbacks are to be used, remember to configure the interrupts correctly
	IRQ_DIRECT_CONNECT(PWM1_IRQn, 0, nrfx_pwm_1_irq_handler, 0);
	irq_enable(PWM1_IRQn);
	IRQ_DIRECT_CONNECT(PWM2_IRQn, 0, nrfx_pwm_2_irq_handler, 0);
	irq_enable(PWM2_IRQn);
}

static nrf_pwm_values_individual_t seq_values_ab_1[PWM_BUF_LENGTH], seq_values_ab_2[PWM_BUF_LENGTH];
static nrf_pwm_values_individual_t seq_values_c_1[PWM_BUF_LENGTH], seq_values_c_2[PWM_BUF_LENGTH];
static nrf_pwm_sequence_t seq_ab_1, seq_ab_2, seq_c_1, seq_c_2;

static void pwm_start(void) 
{
    seq_ab_1.values.p_individual = (const nrf_pwm_values_individual_t *)&seq_values_ab_1;
    seq_ab_1.length          = PWM_BUF_LENGTH*4;
    seq_ab_1.repeats         = 1;
    seq_ab_1.end_delay       = 0;
    seq_ab_2.values.p_individual = (const nrf_pwm_values_individual_t *)&seq_values_ab_2;
    seq_ab_2.length          = PWM_BUF_LENGTH*4;
    seq_ab_2.repeats         = 1;
    seq_ab_2.end_delay       = 0;
    seq_c_1.values.p_individual = (const nrf_pwm_values_individual_t *)&seq_values_c_1;
    seq_c_1.length          = PWM_BUF_LENGTH*4;
    seq_c_1.repeats         = 1;
    seq_c_1.end_delay       = 0;
	seq_c_2.values.p_individual = (const nrf_pwm_values_individual_t *)&seq_values_c_2;
    seq_c_2.length          = PWM_BUF_LENGTH*4;
    seq_c_2.repeats         = 1;
    seq_c_2.end_delay       = 0;

	nrfx_pwm_complex_playback(&m_pwm_ab, &seq_ab_1, &seq_ab_2, 1, NRFX_PWM_FLAG_SIGNAL_END_SEQ0 | NRFX_PWM_FLAG_SIGNAL_END_SEQ1 | NRFX_PWM_FLAG_LOOP);
	nrfx_pwm_complex_playback(&m_pwm_c, &seq_c_1, &seq_c_2, 1, NRFX_PWM_FLAG_SIGNAL_END_SEQ0 | NRFX_PWM_FLAG_SIGNAL_END_SEQ1 | NRFX_PWM_FLAG_LOOP);
}

#define IDLE_VALUE PWM_COUNTERTOP // TODO: Figure out why the max value gives low output....
static void set_pwm_sequence(uint32_t seq_index, uint32_t duty_cycle)
{
	nrf_pwm_values_individual_t *values_ab = (seq_index == 0) ? seq_values_ab_1 : seq_values_ab_2;
	nrf_pwm_values_individual_t *values_c = (seq_index == 0) ? seq_values_c_1 : seq_values_c_2;
	
	for(int i = 0; i < PWM_BUF_LENGTH; i++) {
		switch(i / PWM_STEP_SUBDIV) {
			case 0:
				values_ab[i].channel_0 = duty_cycle;
				values_ab[i].channel_1 = PWM_INVERTED(duty_cycle);
				values_ab[i].channel_2 = PWM_INVERTED(duty_cycle);
				values_ab[i].channel_3 = duty_cycle;
				values_c[i].channel_0 = IDLE_VALUE;
				values_c[i].channel_1 = IDLE_VALUE;
				break;
			case 1:
				values_ab[i].channel_0 = duty_cycle;
				values_ab[i].channel_1 = PWM_INVERTED(duty_cycle);
				values_ab[i].channel_2 = IDLE_VALUE;
				values_ab[i].channel_3 = IDLE_VALUE;
				values_c[i].channel_0 = PWM_INVERTED(duty_cycle);
				values_c[i].channel_1 = duty_cycle;
				break;
			case 2:
				values_ab[i].channel_0 = IDLE_VALUE;
				values_ab[i].channel_1 = IDLE_VALUE;
				values_ab[i].channel_2 = duty_cycle;
				values_ab[i].channel_3 = PWM_INVERTED(duty_cycle);
				values_c[i].channel_0 = PWM_INVERTED(duty_cycle);
				values_c[i].channel_1 = duty_cycle;
				break;
			case 3:
				values_ab[i].channel_0 = PWM_INVERTED(duty_cycle);
				values_ab[i].channel_1 = duty_cycle;
				values_ab[i].channel_2 = duty_cycle;
				values_ab[i].channel_3 = PWM_INVERTED(duty_cycle);
				values_c[i].channel_0 = IDLE_VALUE;
				values_c[i].channel_1 = IDLE_VALUE;
				break;
			case 4:
				values_ab[i].channel_0 = PWM_INVERTED(duty_cycle);
				values_ab[i].channel_1 = duty_cycle;
				values_ab[i].channel_2 = IDLE_VALUE;
				values_ab[i].channel_3 = IDLE_VALUE;
				values_c[i].channel_0 = duty_cycle;
				values_c[i].channel_1 = PWM_INVERTED(duty_cycle);
				break;
			case 5:
				values_ab[i].channel_0 = 100;
				values_ab[i].channel_1 = 100;
				values_ab[i].channel_2 = PWM_INVERTED(duty_cycle);
				values_ab[i].channel_3 = duty_cycle;
				values_c[i].channel_0 = duty_cycle;
				values_c[i].channel_1 = PWM_INVERTED(duty_cycle);
				break;
			default:
				break;
		}
	}
}

static uint8_t dppi_ch_pwm_step; 
#define EGU_CH_TRIGGER_PWM 0
#define SUBSCRIBE_ENABLE 0x80000000
#define PUBLISH_ENABLE   0x80000000
static void ppi_egu_init(void)
{
	nrfx_dppi_channel_alloc(&dppi_ch_pwm_step);
	nrfx_egu_init(&m_egu, 5, 0, 0);
	NRF_PWM1->SUBSCRIBE_NEXTSTEP = SUBSCRIBE_ENABLE | dppi_ch_pwm_step;
	NRF_PWM2->SUBSCRIBE_NEXTSTEP = SUBSCRIBE_ENABLE | dppi_ch_pwm_step;
	NRF_EGU2->PUBLISH_TRIGGERED[EGU_CH_TRIGGER_PWM] = PUBLISH_ENABLE | dppi_ch_pwm_step;
	NRF_TIMER1->PUBLISH_COMPARE[0] = PUBLISH_ENABLE | dppi_ch_pwm_step;
	nrfx_dppi_channel_enable(dppi_ch_pwm_step);
}

static void timer_pwm_step_init(uint32_t reload_time_us)
{
	nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG;
	timer_config.bit_width = NRF_TIMER_BIT_WIDTH_32;
	timer_config.frequency = NRF_TIMER_FREQ_1MHz;

	int err = nrfx_timer_init(&m_timer_pwm_step, &timer_config, 0);
	if (err != NRFX_SUCCESS) {
		printk("Error initializing timer: %x\n", err);
	}

	//IRQ_DIRECT_CONNECT(TIMER1_IRQn, 0, nrfx_timer_1_irq_handler, 0);
	//irq_enable(TIMER1_IRQn);

	nrfx_timer_extended_compare(&m_timer_pwm_step, NRF_TIMER_CC_CHANNEL0, reload_time_us, 
                                NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
}

static void timer_pwm_step_start(void)
{
	nrfx_timer_enable(&m_timer_pwm_step);
}

void main(void)
{
	int ret;

	printk("PWM 3-phase Demo started\n");

	ret = pwm_init();
	if(ret < 0) printk("PWM init error!!\n");
	
	set_pwm_sequence(0, 0);
	set_pwm_sequence(1, 0);

	timer_pwm_step_init(200);

	ppi_egu_init();

	pwm_start();

	timer_pwm_step_start();

	uint32_t counter = 12;
	while (1) {
		k_msleep(100);
		//NRF_PWM1->TASKS_NEXTSTEP = 1;
		//NRF_PWM2->TASKS_NEXTSTEP = 1;
		//nrfx_egu_trigger(&m_egu, EGU_CH_TRIGGER_PWM);
		if(k_sem_take(&m_sem_update_pwm_buf, K_NO_WAIT) == 0) {
			if(m_pwm_update_buf_index == 0) {
				// Safe to update sequence 0
				set_pwm_sequence(0, counter);
			} else {
				// Safe to update sequence 1
				set_pwm_sequence(1, counter);
			}
			counter += 10;
			if(counter > PWM_COUNTERTOP) counter = 12;
		}
	}
}
