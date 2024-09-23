/*
 * interrupts_app.h
 *
 *  Created on: Jun 9, 2024
 *      Author: Hamza
 */

#ifndef INC_INTERRUPTS_APP_H_
#define INC_INTERRUPTS_APP_H_

#include "main.h"

extern uint8_t esp32_command_available;
extern uint8_t _timerTimeout;
extern uint8_t sendToMQTT;

void start_UART_ITs(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif /* INC_INTERRUPTS_APP_H_ */
