#ifndef HAL_H
#define HAL_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

typedef void(*hal_callback_t)(void);

typedef enum{
	V_OUT,
	I_OUT
} HalSignals;

void hal_init();

void hal_set_pwm_period(float period);
void hal_set_pwm_duty_cycle(float duty);
void hal_start_pwm(void);
void hal_set_timer_period(float period);
void hal_set_timer_callback(hal_callback_t);
void hal_start_timer(void);
bool hal_has_eth_msg();
size_t hal_read_eth_msg(uint8_t* msg);

float hal_read_signal(HalSignals signal);

void hal_log(char* msg);

#endif
