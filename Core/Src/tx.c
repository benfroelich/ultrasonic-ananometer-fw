// ultrasonic transmitter
#include "tx.h"
#include <stdio.h>
#include "main.h"

pulse_state _pulse_state = PULSE_IDLE;

typedef struct io_pin_t {
	GPIO_TypeDef* bank;
	uint16_t ch;
} io_pin_t;

io_pin_t pulse_p, pulse_n;

extern TIM_HandleTypeDef htim1;

int _pulse_cnt;
const int num_edges = 10;

pulse_state issue_pulse(const tx_node node)
{
	printf("issue_pulse\n");
	if(_pulse_state != PULSE_IDLE) return PULSE_ERROR;
	_pulse_state = PULSE_BUSY;
	_pulse_cnt = 0;
	// start the timer

	// TODO assign ch based on node argument
	pulse_p.bank = GPIOA; pulse_p.ch = GPIO_PIN_8;
	pulse_n.bank = GPIOB; pulse_n.ch = GPIO_PIN_10;

	HAL_GPIO_WritePin(pulse_p.bank, pulse_p.ch, GPIO_PIN_SET);
	HAL_GPIO_WritePin(pulse_n.bank, pulse_n.ch, GPIO_PIN_RESET);
	HAL_TIM_Base_Start_IT(&htim1);

    return PULSE_OK;
}

pulse_state get_pulse_state()
{
    return _pulse_state;
}
/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim17 && _pulse_state == PULSE_BUSY)
	{
		if(_pulse_cnt++ < num_edges)
		{
			HAL_GPIO_TogglePin(pulse_p.bank, pulse_p.ch);
			HAL_GPIO_TogglePin(pulse_n.bank, pulse_n.ch);
		}
		else
		{
			HAL_GPIO_WritePin(pulse_p.bank, pulse_p.ch, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(pulse_n.bank, pulse_n.ch, GPIO_PIN_RESET);
			_pulse_state = PULSE_IDLE;
		}
	}
	if(htim == &htim1)
	{
        asm("nop");
	}
}
*/

