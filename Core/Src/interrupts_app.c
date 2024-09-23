/*
 * interrupts_app.c
 *
 *  Created on: Jun 9, 2024
 *      Author: Hamza
 */

#include "interrupts_app.h"
#include "gateway_config.h"
#include "usart.h"
#include "esp32_command.h"
#include "tim.h"

uint8_t esp32_command_available = 0;
uint8_t _timerTimeout = false;

volatile uint8_t sendToMQTTCounter = 0;
uint8_t sendToMQTT = true;

void start_UART_ITs(void)
{
	HAL_UART_Receive_IT(&huart1, esp32_command_data, ESP32_COMMAND_LEN);
	HAL_UART_Receive_IT(&huart2, gateway_config_data, CONFIG_DATA_LEN);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
	{
		esp32_command_available = true;
		while (HAL_UART_Receive_IT(&huart1, esp32_command_data,
				ESP32_COMMAND_LEN) != HAL_OK)
		{
		}
	}
	if (huart == &huart2)
	{
		config_data_available = 1;
//		HAL_UART_Receive_IT(&huart2, gateway_config_data, CONFIG_DATA_LEN); //REINIT THE UART INTERRUPT NOW
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1)
	{
		HAL_IncTick();
	}
	if (htim == &htim2)
	{
//		_timerTimeout = true;
//
//		sendToMQTTCounter++;
//		if (sendToMQTTCounter == 10)
//		{
//			sendToMQTTCounter = 0;
//			sendToMQTT = true;
//		}
	}
}

