// ultrasonic anemometer
#include "anemometer.h"

#include <stdio.h>
#include "main.h"

#define ADC_BUFF_SZ 500
#define SAMP_PER_WORD 2
#define SAMP1_MASK 0x00000FFF
#define SAMP2_MASK 0x0FFF0000
// output TIM1 on PC2 (1/2 actual frequency)
#define TIM1_PC2
// log the waveform using printf and default serial studio format
#define DUMP_WAVEFORM
uint32_t adc_buffer[ADC_BUFF_SZ];

// offset into BSRR register to either set or clear the GPIO
const uint32_t PIN_ON = 0;
const uint32_t PIN_OFF = 16;
// pulse compression with barker codes
// see https://en.wikipedia.org/wiki/Barker_code
// barker 7: −16.9 dB sidelobe ratio
// TODO: dyn assign by barker code rather than hard-coded:
// const uint32_t barker_code_p[] = {+1, +1, +1, -1, -1, +1, -1};
// NOTE: the arrays have an extra element at the end to reset the pins back to 0
const uint32_t barker_code_p[] = {
		GPIO_PIN_10 << PIN_ON,
	    GPIO_PIN_10 << PIN_ON,
	    GPIO_PIN_10 << PIN_ON,
	    GPIO_PIN_10 << PIN_OFF,
	    GPIO_PIN_10 << PIN_OFF,
	    GPIO_PIN_10 << PIN_ON,
	    GPIO_PIN_10 << PIN_OFF,
	    GPIO_PIN_10 << PIN_OFF,
	    GPIO_PIN_10 << PIN_OFF,
};
const uint32_t barker_code_n[] = {
		// extra padding sample at beginning because this will be driven by
		// tim2 PWM CH1 , which will fire once before the first update event
		GPIO_PIN_3 << PIN_OFF,
		GPIO_PIN_3 << PIN_OFF,
		GPIO_PIN_3 << PIN_OFF,
		GPIO_PIN_3 << PIN_OFF,
		GPIO_PIN_3 << PIN_ON,
		GPIO_PIN_3 << PIN_ON,
		GPIO_PIN_3 << PIN_OFF,
		GPIO_PIN_3 << PIN_ON,
		GPIO_PIN_3 << PIN_OFF,
};
const int barker_sz = sizeof(barker_code_p)/sizeof(barker_code_p[0]);
const uint32_t scratch_addr;

anemometer_state_t _state = ANEMOMETER_UNINIT;

typedef struct io_pin_t {
	GPIO_TypeDef* bank;
	uint16_t ch;
} io_pin_t;

io_pin_t pulse_p, pulse_n;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_tim2_up;
extern DMA_HandleTypeDef hdma_tim2_ch1;

int _edge_cnt;

anemometer_status_t anemometer_init()
{
	if(_state == ANEMOMETER_BUSY) return ANEMOMETER_RESULT_ERROR;
	for(int i = 0; i < ADC_BUFF_SZ; i++) adc_buffer[i] = 0xDEADBEEF;
	_state = ANEMOMETER_IDLE;
	return ANEMOMETER_RESULT_OK;
}

anemometer_status_t anemometer_issue_pulse(const tx_node node)
{
	if(_state != ANEMOMETER_IDLE) return ANEMOMETER_RESULT_ERROR;
	printf("issue_pulse\n");

	_state = ANEMOMETER_BUSY;
	_edge_cnt = 0;

	// TODO assign ch based on node argument
	pulse_p.bank = GPIOA; pulse_p.ch = GPIO_PIN_10;
	pulse_n.bank = GPIOB; pulse_n.ch = GPIO_PIN_3;

    // stop the timers if they were running already
	// they shouldn't be running if this task is being
	// invoked, but leave it in for now
	if(HAL_TIM_Base_Stop(&htim1) != HAL_OK) Error_Handler();
	if(HAL_TIM_Base_Stop(&htim2) != HAL_OK) Error_Handler();

	HAL_GPIO_WritePin(pulse_p.bank, pulse_p.ch, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(pulse_n.bank, pulse_n.ch, GPIO_PIN_RESET);

	__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);
	__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_UPDATE);
	if(HAL_DMA_Start_IT(&hdma_tim2_up, (uint32_t)barker_code_p,
			(uint32_t)&(pulse_p.bank->BSRR), barker_sz) != HAL_OK)
		Error_Handler();
	if(HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)barker_code_n,
			(uint32_t)&(pulse_n.bank->BSRR), barker_sz) != HAL_OK)
		Error_Handler();

	// reset counters
	htim2.Instance->CNT = 0;
	htim1.Instance->CNT = 0;
	// since two 12-bit samples are packed into each 32-bit word, we can
	// sample 2x the buffer size (in 32-bit words)
	// this command starts the ADC, it will wait until triggered by TIM1 (I think? TODO)
	if(HAL_ADC_Start_DMA(&hadc1, adc_buffer, ADC_BUFF_SZ * SAMP_PER_WORD) != HAL_OK)
		Error_Handler();
	// start time base on TIM2, the update event will will trigger DMA->GPIO on pulse_p output
	if(HAL_TIM_Base_Start(&htim2) != HAL_OK) Error_Handler();
	// start no-output PWM on TIM2 CH1 which will trigger DMA->GPIO on pulse_n output
	if(HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
	// this triggers the ADC and TIM2
	if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
	// view TIM1 transitions on PC2
#ifdef TIM1_PC2
	if(HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
#endif
	// wait for ADC conversions to complete
	//printf("sampling\r\n");
	while(HAL_DMA_GetState(&hdma_adc1) != HAL_DMA_STATE_READY);
	while(HAL_DMA_GetState(&hdma_tim2_ch1) != HAL_DMA_STATE_READY);
	while(HAL_DMA_GetState(&hdma_tim2_up) != HAL_DMA_STATE_READY);
	//printf("done sampling\r\n");
	// stop the timer channels
	if(HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
	if(HAL_TIM_Base_Stop(&htim2) != HAL_OK) Error_Handler();
	if(HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
#ifdef TIM1_PC2
	if(HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
#endif
#ifdef DUMP_WAVEFORM
	for(int i = 0; i < ADC_BUFF_SZ; i ++)
	{
	  int word = adc_buffer[i];
	  printf("/*%i*/", (word & SAMP1_MASK));
	  printf("/*%i*/", (word & SAMP2_MASK) >> (32 / 2));
	}
	while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY) {}
#endif

	// blocking mode for now
	_state = ANEMOMETER_IDLE;

    return ANEMOMETER_RESULT_OK;
}

anemometer_state_t anemometer_get_state()
{
    return _state;
}

// toggle pins in interrupt using software rather than output compare
// because we need to
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
#if 0
	if(htim == &htim2)
	{
		if(_edge_cnt < barker_sz)
		{
			switch(barker_code[_edge_cnt])
			{
			case +1:
				pulse_n.bank->BRR = (uint32_t)pulse_n.ch;
				pulse_p.bank->BSRR = (uint32_t)pulse_p.ch;
				break;
			case -1:
				HAL_GPIO_WritePin(pulse_p.bank, pulse_p.ch, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(pulse_n.bank, pulse_n.ch, GPIO_PIN_SET);
				break;
			default:
				_state = ANEMOMETER_ERROR;
			}
			_edge_cnt++;
		}
		else
		{
			HAL_GPIO_WritePin(pulse_p.bank, pulse_p.ch, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(pulse_n.bank, pulse_n.ch, GPIO_PIN_RESET);
		}
	}
#endif
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//if(htim == &htim2) HAL_GPIO_TogglePin(pulse_p.bank, pulse_p.ch);
}

//void HAL_DMA()
