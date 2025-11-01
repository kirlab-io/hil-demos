#include "application.h"
#include "hal.h"
#include <stdio.h>

float PI_step(PI_data_t* pi, float error){
	//compute P+I
	float P = pi->Kp * error;
	pi->I += pi->Ki * error;
	float out = P + pi->I;

	//anti-windup
	if(out > pi->max_out){
		out = pi->max_out;
		pi->I = pi->max_out - P;
	} else if(out < pi->min_out){
		out = pi->min_out;
		pi->I = pi->min_out - P;
	}

	return out;
}

//Control data
static const float Kp = 0.05;
static const float Ki = 0.005;
static const float MAX_DUTY = 0.9;
static const float MIN_DUTY = 0.1;
static const float TIMER_PERIOD = 100e-6;//100us
static const float PWM_PERIOD = 5e-6;//200kHz

//Local variables
float pwm_duty_cycle;
float v_out_meas;
float i_out_meas;
uint32_t interrupt_cnt;
float setpoint;
PI_data_t pi;
uint8_t eth_buffer[1500];

void application_init(void){
	pwm_duty_cycle = 0.0;
	setpoint = 3.3;
	interrupt_cnt = 0;

	pi = (PI_data_t){
		.Kp = Kp,
		.Ki = Ki,
		.I = 0.0,
		.max_out = MAX_DUTY,
		.min_out = MIN_DUTY
	};

	//timer config
	hal_set_timer_period(TIMER_PERIOD);
	hal_set_timer_callback(application_control_task);
	hal_start_timer();

	//pwm config
	hal_set_pwm_period(PWM_PERIOD);
	hal_set_pwm_duty_cycle(MIN_DUTY);
	hal_start_pwm();


}

void application_control_task(void){
	//PI controller
	v_out_meas = hal_read_signal(V_OUT);
	i_out_meas = hal_read_signal(I_OUT);
	pwm_duty_cycle = PI_step(&pi, (setpoint - v_out_meas));
	hal_set_pwm_duty_cycle(pwm_duty_cycle);
	interrupt_cnt++;
}

void update_setpoint(void){
	if(hal_has_eth_msg() > 0){
		size_t size = hal_read_eth_msg(eth_buffer);
		if(size == sizeof(setpoint)){
			setpoint = *(float*)(&eth_buffer);

			//log the reception of a new setpoint
			char msg[64];
			sprintf(msg, "New setpoint received: %f", setpoint);
			hal_log(msg);
		}
	}
}

void log_status(void){
	if(interrupt_cnt >= 100){
		char msg[64];
		sprintf(msg, " Setpoint: %f Vout: %f Iout: %f\n", setpoint, v_out_meas, i_out_meas);
		hal_log(msg);
		interrupt_cnt = 0;//atomic
	}
}

void application_background_loop(void){
	update_setpoint();
	log_status();
}
