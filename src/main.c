#include <stdint.h>
#include <stdbool.h>

#include "main.h"

ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim15;
UART_HandleTypeDef huart1;

enum switch_e { SW_FAN_OFF, SW_FAN_L, SW_FAN_M, SW_FAN_H, SW_FAN_AUTO, SW_FILTER, SW_DELAY, SW_LIGHTS };
enum output_e { OUT_NONE=-1, OUT_FAN_L, OUT_FAN_M, OUT_FAN_H, OUT_LIGHTS };
enum func_e { FUNC_FAN_OFF, FUNC_FAN_L, FUNC_FAN_M, FUNC_FAN_H, FUNC_LIGHTS_OFF, FUNC_LIGHTS_TOGGLE };

typedef struct {
	enum switch_e sw;
	enum func_e func;

	uint32_t debounce_time;

	GPIO_TypeDef *sw_port;
	uint32_t sw_pin;
	GPIO_TypeDef *led_port;
	uint32_t led_pin;
} sw_t;

typedef struct {
	enum output_e out;
	bool state;

	GPIO_TypeDef *pp_port;
	uint32_t pp_pin;
	GPIO_TypeDef *oe_port;
	uint32_t oe_pin;
} output_t;

static sw_t switches[] = {
	{ .sw = SW_FAN_OFF, .func = FUNC_FAN_OFF, .sw_port = FAN_OFF_GPIO_Port, .sw_pin = FAN_OFF_Pin, .led_port = LED_OFF_GPIO_Port, .led_pin = LED_OFF_Pin },
	{ .sw = SW_FAN_L, .func = FUNC_FAN_L, .sw_port = FAN_L_GPIO_Port, .sw_pin = FAN_L_Pin, .led_port = LED_L_GPIO_Port, .led_pin = LED_L_Pin },
	{ .sw = SW_FAN_M, .func = FUNC_FAN_M, .sw_port = FAN_M_GPIO_Port, .sw_pin = FAN_M_Pin, .led_port = LED_M_GPIO_Port, .led_pin = LED_M_Pin },
	{ .sw = SW_FAN_H, .func = FUNC_FAN_H, .sw_port = FAN_H_GPIO_Port, .sw_pin = FAN_H_Pin, .led_port = LED_H_GPIO_Port, .led_pin = LED_H_Pin },
	{ .sw = SW_LIGHTS, .func = FUNC_LIGHTS_TOGGLE, .sw_port = LIGHTS_GPIO_Port, .sw_pin = LIGHTS_Pin, .led_port = LED_LIGHTS_GPIO_Port, .led_pin = LED_LIGHTS_Pin },
	{ .sw_port = NULL }
};

static output_t outputs[] = {
	{ .out = OUT_FAN_L, .pp_port = OUT1_PP_GPIO_Port, .pp_pin = OUT1_PP_Pin, .oe_port = OUT1_OE_GPIO_Port, .oe_pin = OUT1_OE_Pin },
	{ .out = OUT_FAN_M, .pp_port = OUT2_PP_GPIO_Port, .pp_pin = OUT2_PP_Pin, .oe_port = OUT2_OE_GPIO_Port, .oe_pin = OUT2_OE_Pin },
	{ .out = OUT_FAN_H, .pp_port = OUT3_PP_GPIO_Port, .pp_pin = OUT3_PP_Pin, .oe_port = OUT3_OE_GPIO_Port, .oe_pin = OUT3_OE_Pin },
	{ .out = OUT_LIGHTS, .pp_port = OUT4_PP_GPIO_Port, .pp_pin = OUT4_PP_Pin, .oe_port = OUT4_OE_GPIO_Port, .oe_pin = OUT4_OE_Pin },
	{ .pp_port = NULL }
};


void Error_Handler(void)
{
	__disable_irq();
	while (1) ;
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
	Error_Handler();
}
#endif


static void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}


static void MX_ADC1_Init(void)
{
	ADC_MultiModeTypeDef multimode = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.GainCompensation = 0;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR_ADC1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}


static void MX_TIM2_Init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = -1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim2);
}


static void MX_TIM15_Init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	htim15.Instance = TIM15;
	htim15.Init.Prescaler = 0;
	htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim15.Init.Period = 4000;
	htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim15.Init.RepetitionCounter = 0;
	htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim15) != HAL_OK) {
		Error_Handler();
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_TIM_PWM_Init(&htim15) != HAL_OK) {
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK) {
		Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim15);
}


static void MX_USART1_UART_Init(void)
{
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
		Error_Handler();
	}
}


static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

	HAL_GPIO_WritePin(GPIOA, OUT3_PP_Pin | OUT4_PP_Pin | OUT4_OE_Pin | OUT5_OE_Pin | OUT5_PP_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, LED_M_Pin | LED_H_Pin | LED_AUTO_Pin | GPIO_PIN_10 | LED_LIGHTS_Pin | LED_DELAY_Pin | LED_FILTER_Pin | LED_OFF_Pin | OUT1_PP_Pin | OUT1_OE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, OUT2_OE_Pin | OUT2_PP_Pin | LED_L_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUT3_OE_GPIO_Port, OUT3_OE_Pin, GPIO_PIN_RESET);

	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pin = OUT3_PP_Pin | OUT4_PP_Pin | OUT4_OE_Pin | OUT5_OE_Pin | OUT5_PP_Pin;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LED_M_Pin | LED_H_Pin | LED_AUTO_Pin | GPIO_PIN_10 | LED_LIGHTS_Pin | LED_DELAY_Pin | LED_FILTER_Pin | LED_OFF_Pin | OUT1_PP_Pin | OUT1_OE_Pin;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = OUT2_OE_Pin | OUT2_PP_Pin | LED_L_Pin;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = OUT3_OE_Pin;
	HAL_GPIO_Init(OUT3_OE_GPIO_Port, &GPIO_InitStruct);

	HAL_GPIO_WritePin(BEEP_N_GPIO_Port, BEEP_N_Pin, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = BEEP_N_Pin;
	HAL_GPIO_Init(BEEP_N_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = FAN_L_Pin | FAN_M_Pin | FAN_AUTO_Pin | FAN_H_Pin | FAN_OFF_Pin | FILTER_Pin | LIGHTS_Pin | DELAY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_8;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_10 | GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_10;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
}


/* finds the sw_t entry for a given switch */
static sw_t *_find_sw(enum switch_e which)
{
	sw_t *s;

	for (s = &switches[0]; s->sw_port && s->sw != which; ++s) ;

	if (s->sw != which) {
		s = NULL;
	}

	return s;
}


/* finds the sw_t entry for a given switch */
static output_t *_find_output(enum output_e which)
{
	output_t *o;

	for (o = &outputs[0]; o->pp_port && o->out != which; ++o) ;

	if (o->out != which) {
		o = NULL;
	}

	return o;
}


static void set_output(enum output_e which, bool state)
{
	output_t *o;

	if ((o = _find_output(which))) {
/* push-pull */
#if 1
		HAL_GPIO_WritePin(o->pp_port, o->pp_pin, (state) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(o->oe_port, o->oe_pin, GPIO_PIN_SET);

/* open-drain */
#else
		HAL_GPIO_WritePin(o->pp_port, o->pp_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(o->oe_port, o->oe_pin, (state) ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
		o->state = state;
	}
}


static void set_fan(enum func_e func)
{
	enum output_e out;

	/* turn off all the FAN LEDs */
	HAL_GPIO_WritePin(LED_OFF_GPIO_Port, LED_OFF_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_L_GPIO_Port, LED_L_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_M_GPIO_Port, LED_M_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_H_GPIO_Port, LED_H_Pin, GPIO_PIN_RESET);

	/* turn off all the fan outputs */
	HAL_GPIO_WritePin(OUT1_OE_GPIO_Port, OUT1_OE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUT2_OE_GPIO_Port, OUT2_OE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUT3_OE_GPIO_Port, OUT3_OE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUT4_OE_GPIO_Port, OUT4_OE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUT5_OE_GPIO_Port, OUT5_OE_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(OUT1_PP_GPIO_Port, OUT1_PP_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUT2_PP_GPIO_Port, OUT2_PP_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUT3_PP_GPIO_Port, OUT3_PP_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUT4_PP_GPIO_Port, OUT4_PP_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUT5_PP_GPIO_Port, OUT5_PP_Pin, GPIO_PIN_RESET);

	switch (func) {
	default:		out = OUT_NONE; break;
	case FUNC_FAN_OFF:	out = OUT_NONE; break;
	case FUNC_FAN_L:	out = OUT_FAN_L; break;
	case FUNC_FAN_M:	out = OUT_FAN_M; break;
	case FUNC_FAN_H:	out = OUT_FAN_H; break;
	};

	set_output(out, true);
}


static void do_switch(sw_t *s)
{
	output_t *out;

	switch (s->func) {
	case FUNC_FAN_OFF:
	case FUNC_FAN_L:
	case FUNC_FAN_M:
	case FUNC_FAN_H:
		set_fan(s->func);
		HAL_GPIO_WritePin(s->led_port, s->led_pin, GPIO_PIN_SET);
		break;

	case FUNC_LIGHTS_OFF:
		set_output(OUT_LIGHTS, false);
		HAL_GPIO_WritePin(s->led_port, s->led_pin, GPIO_PIN_RESET);
		break;

	case FUNC_LIGHTS_TOGGLE:
		out = _find_output(OUT_LIGHTS);
		set_output(OUT_LIGHTS, !out->state);
		HAL_GPIO_WritePin(s->led_port, s->led_pin, out->state);
		break;

	default:
		/* do nothing */
		break;
	};
}


static void check_switch(enum switch_e sw)
{
	sw_t *s;

	if ((s = _find_sw(sw))) {
		uint32_t now;

		/* if we're not debouncing, see if the switch is pressed */
		now = HAL_GetTick();
		if (s->debounce_time == 0) {
			if (HAL_GPIO_ReadPin(s->sw_port, s->sw_pin) == GPIO_PIN_RESET) {
				s->debounce_time = 1;
				do_switch(s);
			}

		/* we're debouncing. Reset the debounce time if the button isn't released. */
		} else {
			if (HAL_GPIO_ReadPin(s->sw_port, s->sw_pin) == GPIO_PIN_SET) {
				if (now > s->debounce_time) {
					s->debounce_time = 0;
				}

			} else {
				s->debounce_time = now + 50;
			}
		}
	}
}


int main(void)
{
	uint32_t now, next_blink;

	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_TIM2_Init();
	MX_TIM15_Init();
	MX_USART1_UART_Init();

	next_blink = 0;
	while (1) {
		now = HAL_GetTick();

		if (now > next_blink) {
			check_switch(SW_FAN_OFF);
			check_switch(SW_FAN_L);
			check_switch(SW_FAN_M);
			check_switch(SW_FAN_H);
			check_switch(SW_FAN_AUTO);
			check_switch(SW_FILTER);
			check_switch(SW_DELAY);
			check_switch(SW_LIGHTS);

			next_blink = now + 5;
		}
	};
}
