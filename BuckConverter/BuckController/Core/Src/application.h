#ifndef APPLICATION_H
#define APPLICATION_H

//exported signals
extern float pwm_duty_cycle;

//callbacks
void application_init(void);

void application_background_loop(void);

void application_control_task(void);

//PI Type
typedef struct{
	float Kp;
	float Ki;
	float I;
	float max_out;
	float min_out;
} PI_data_t;

#endif
