#include "hal.h"
#include "stm32f3xx_hal.h"
#include <string.h>

#define HS_CLK_FREQ (64e6) //64Mhz

static float pwm_period;
static hal_callback_t timer_callback;
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;

typedef struct{
	float gain;
	float offset;
}AdcConvFactors;

static const AdcConvFactors CONV_FACTORS[] = {//Vref = 3.3V
		[V_OUT] = {.gain=0.00488*(1/0.93), .offset=0.0}, //0-20V range, gain=(20/4096)
		[I_OUT] = {.gain=0.00488*(1/0.93), .offset=0.0}};//0-20A range, gain=(20/4096)

static const int ADC_CHANNEL_MAP[] = {
		[V_OUT] = 2,
		[I_OUT] = 3,
};

/* PRIVATE FUNCTIONS */
uint32_t Read_ADC_Value(ADC_HandleTypeDef* hadc, uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5; // Adjust as needed
    HAL_ADC_ConfigChannel(hadc, &sConfig);

    // Start ADC conversion
    HAL_ADC_Start(hadc);

    // Wait for conversion to be complete
    HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);

    // Get the converted value
    uint32_t adcValue = HAL_ADC_GetValue(hadc);

    return adcValue;
}

void PWM_Init(void)
{
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_PWM_Init(&htim3);

	TIM_OC_InitTypeDef sConfigOC = {0};
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
}

void TIM2_IRQHandler(void)
{
    // Check if update interrupt flag is set
    if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);

            // Actions to be taken at the end of each PWM period
            if(timer_callback != NULL)
            	(*timer_callback)();
        }
    }
}

/* PUBLIC FUNCTIONS */

void hal_init(void){
	PWM_Init();
	HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void hal_set_timer_period(float period){
	__HAL_TIM_SET_AUTORELOAD(&htim3, period*HS_CLK_FREQ);
}

void hal_set_timer_callback(hal_callback_t callback){
	timer_callback = callback;
	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
}

void hal_start_timer(void){
	HAL_TIM_Base_Start_IT(&htim2);
}

void hal_set_pwm_period(float period){
    //period = (1/20e3);
	__HAL_TIM_SET_AUTORELOAD(&htim3, period*HS_CLK_FREQ);
	pwm_period = period;
}

void hal_set_pwm_duty_cycle(float duty){
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty*pwm_period*HS_CLK_FREQ);
    //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0.3*pwm_period*HS_CLK_FREQ);
}

void hal_start_pwm(void){
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}


bool hal_has_eth_msg(void){return false;}

size_t hal_read_eth_msg(uint8_t* msg){
	return 0;
}

float hal_read_signal(HalSignals id){
	uint32_t adc_counts  = Read_ADC_Value(&hadc2, ADC_CHANNEL_MAP[id]);
	float signal = (CONV_FACTORS[id].gain * adc_counts) + CONV_FACTORS[id].offset;
	return signal;
}

void hal_log(char* msg){
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);
}


