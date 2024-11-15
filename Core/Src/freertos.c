/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nRF24l01P.h"
#include "nRF24_app.h"
#include "gateway_config.h"
#include "interrupts_app.h"
#include "ee.h"
#include "esp32_command.h"
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = { .name = "defaultTask",
		.stack_size = 256 * 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for getNodeData */
osThreadId_t getNodeDataHandle;
const osThreadAttr_t getNodeData_attributes = { .name = "getNodeData",
		.stack_size = 256 * 4, .priority = (osPriority_t) osPriorityLow2, };
/* Definitions for sendDataToESP */
osThreadId_t sendDataToESPHandle;
const osThreadAttr_t sendDataToESP_attributes = { .name = "sendDataToESP",
		.stack_size = 256 * 4, .priority = (osPriority_t) osPriorityLow1, };
/* Definitions for hConfigData */
osThreadId_t hConfigDataHandle;
const osThreadAttr_t hConfigData_attributes = { .name = "hConfigData",
		.stack_size = 256 * 4, .priority = (osPriority_t) osPriorityLow3, };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void configDataHandler(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,
			&defaultTask_attributes);

	/* creation of getNodeData */
	getNodeDataHandle = osThreadNew(StartTask02, NULL, &getNodeData_attributes);

	/* creation of sendDataToESP */
	sendDataToESPHandle = osThreadNew(StartTask03, NULL,
			&sendDataToESP_attributes);

	/* creation of hConfigData */
	hConfigDataHandle = osThreadNew(configDataHandler, NULL,
			&hConfigData_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
	/* USER CODE BEGIN StartDefaultTask */
//    UBaseType_t uxHighWaterMarkdefault;
//    UBaseType_t uxHighWaterMarkgetnode;
//    UBaseType_t uxHighWaterMarksendtoesp;
//    UBaseType_t uxHighWaterMarkhconfig;
	/* Inspect our own high water mark on entering the task. */
//	uxHighWaterMarkdefault = uxTaskGetStackHighWaterMark( NULL );
//	uxHighWaterMarkgetnode = uxTaskGetStackHighWaterMark( getNodeDataHandle );
//	uxHighWaterMarksendtoesp = uxTaskGetStackHighWaterMark( sendDataToESPHandle );
//	uxHighWaterMarkhconfig = uxTaskGetStackHighWaterMark( defaultTaskHandle );
//	uint8_t tx_buff[40];
//	uint8_t len;
	/* Infinite loop */
	for (;;) {

//		uxHighWaterMarkdefault = uxTaskGetStackHighWaterMark( NULL );
//		len =sprintf((char*)tx_buff, "defaultTaskHandle: %lu\r\n", uxHighWaterMarkdefault);
//		HAL_UART_Transmit(&huart2, tx_buff, len, 10);
//		uxHighWaterMarkgetnode = uxTaskGetStackHighWaterMark( getNodeDataHandle );
//		len =sprintf((char*)tx_buff, "getNodeDataHandle: %lu\r\n", uxHighWaterMarkgetnode);
//		HAL_UART_Transmit(&huart2, tx_buff, len, 10);
//		uxHighWaterMarksendtoesp = uxTaskGetStackHighWaterMark( sendDataToESPHandle );
//		len =sprintf((char*)tx_buff, "sendDataToESPHandle: %lu\r\n", uxHighWaterMarksendtoesp);
//		HAL_UART_Transmit(&huart2, tx_buff, len, 10);
//		uxHighWaterMarkhconfig = uxTaskGetStackHighWaterMark( hConfigDataHandle );
//		len =sprintf((char*)tx_buff, "hConfigDataHandle: %lu\r\n", uxHighWaterMarkhconfig);
//		HAL_UART_Transmit(&huart2, tx_buff, len, 10);
		osDelay(10000);
	}
	/* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * @brief Function implementing the getNodeData thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument) {
	/* USER CODE BEGIN StartTask02 */
	/* Infinite loop */
	uint8_t ret = 0;
	for (;;) {
		for (int i = 0; i < gateway.num_nodes; i++) {
			if (node[i].ConfigReady == false) {
				node[i].tx_packet.read = 0x01;
			} else {
				node[i].tx_packet.read = 0x02;
			}

			while (node[i].tx_packet.txDone != true
					&& node[i].tx_packet.maxRT != true) {
				ret = nRF24_transmit_data(&node[i].tx_packet, &gateway.node[i]);
				if (ret == 2 || ret == 3) {
					if (node[i].deadCounter < 250) {
						node[i].deadCounter++;
					} else if (node[i].deadCounter >= 250) {
						node[i].deadCounter = 10; //if the counter rolls back, then we dont want to seee the device alive when its actually not.
					}
				}
			}
			uint32_t rxStartTime = HAL_GetTick();
			if (ret == 3) {
				while ((node[i].tx_packet.txDone == true
						&& node[i].tx_packet.rxDone != true)
						&& ((HAL_GetTick() - rxStartTime) < 100)) {
					ret = nRF24_receive_data(&node[i]);
					if (ret == 3 || ret == 4) {
						node[i].deadCounter = 0; //make it zero to inform that the node is alive
					}
					ret = 0;
				}
			}

//			if (node[i].tx_packet.txDone == true
//					&& node[i].tx_packet.rxDone == true)
//			{
//				sendToESP32(&node[i], i + 1);
//			}

			if (node[i].deadCounter <= 10) {
				node[i].alive = true;
			} else {
				node[i].alive = false;
			}

			node[i].tx_packet.txDone = false;
			node[i].tx_packet.rxDone = false;
			node[i].tx_packet.maxRT = false;
			osDelay(20);
		}

		if (_timerTimeout) {
			for (int i = 0; i < gateway.num_nodes; i++) {
				node[i].getConfigCounter++;
				if (node[i].getConfigCounter == 2) {
					node[i].getConfigCounter = 0;
					node[i].ConfigReady = false;
				}
			}
//			printRadioSettings();
//			printConfigReg();
//			printFIFOstatus();
//			printStatusReg();

			_timerTimeout = false;
		}
		if (HAL_GPIO_ReadPin(STM_EN_PIN_GPIO_Port, STM_EN_PIN_Pin) == 0) //Perform this if STM_EN is LOW
				{
			HAL_NVIC_SystemReset();
		}
		nrf24FailureCheckAndResolve();
		osDelay(1000);
	}
	/* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
 * @brief Function implementing the sendDataToESP thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument) {
	/* USER CODE BEGIN StartTask03 */
	int i = 0;
	/* Infinite loop */
	for (;;) {
		if (HAL_GPIO_ReadPin(STM_EN_PIN_GPIO_Port, STM_EN_PIN_Pin)) //perform this if STM_EN is HIGH
				{
			osDelay(5000);
			if (i > 9)
				i = 0;
			sendToESP32(&node[i], i);
			i++;
		}
	}
	/* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_configDataHandler */
/**
 * @brief Function implementing the hConfigData thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_configDataHandler */
void configDataHandler(void *argument) {
	/* USER CODE BEGIN configDataHandler */
	/* Infinite loop */
	for (;;) {
		update_config();
		process_command();
		//check the STM_UART_RST_PIN and reset if HIGH, this is controlled by the ESP32
		if (HAL_GPIO_ReadPin(STM_UART_RST_PIN_GPIO_Port,
				STM_UART_RST_PIN_Pin)) {
			char tx_buff[50];
			uint8_t length = sprintf(tx_buff, "Restarting UART\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*) tx_buff, length, 10);

			HAL_UART_DeInit(&huart1);
			MX_USART1_UART_Init();
			start_UART_ITs();
		}
		//Check for any runtime errors in UART1 and fix TODO check if this really works
		if (huart1.ErrorCode != HAL_UART_ERROR_NONE) {
			if (HAL_UART_Receive_IT(&huart1, esp32_command_data,
					ESP32_COMMAND_LEN) != HAL_OK) {
				// Check and handle specific UART errors
				if (huart1.ErrorCode & HAL_UART_ERROR_ORE)  // Overrun error
				{
					__HAL_UART_CLEAR_OREFLAG(&huart1); // Clear overrun error flag
					// Debug message for overrun error
				}
				if (huart1.ErrorCode & HAL_UART_ERROR_FE)  // Framing error
				{
					__HAL_UART_CLEAR_FEFLAG(&huart1); // Clear framing error flag
					// Debug message for framing error
				}
				if (huart1.ErrorCode & HAL_UART_ERROR_NE)  // Noise error
				{
					__HAL_UART_CLEAR_NEFLAG(&huart1);  // Clear noise error flag
					// Debug message for noise error
				}
				if (huart1.ErrorCode & HAL_UART_ERROR_PE)  // Parity error
				{
					__HAL_UART_CLEAR_PEFLAG(&huart1); // Clear parity error flag
					// Debug message for parity error
				}
				// Attempt to restart UART reception in interrupt mode
				if (HAL_UART_Receive_IT(&huart1, esp32_command_data,
						ESP32_COMMAND_LEN) != HAL_OK) {
					// If restart fails, then reinitialize the UART
					HAL_UART_DeInit(&huart1);
					MX_USART1_UART_Init();
				}
			}
		}
		osDelay(10);
	}
	/* USER CODE END configDataHandler */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

