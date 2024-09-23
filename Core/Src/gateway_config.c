/*
 * gateway_config.c
 *
 *  Created on: Jun 13, 2024
 *      Author: Hamza
 */

#include <stdio.h>
#include "gateway_config.h"
#include "ee.h"
#include "usart.h"

gateway_config_t gateway;

node_info_t node[10];
//node_info_t node1;
//node_info_t node2;
//node_info_t node3;
//node_info_t node4;
//node_info_t node5;
//node_info_t node6;
//node_info_t node7;
//node_info_t node8;
//node_info_t node9;


uint8_t gateway_config_data[CONFIG_DATA_LEN] = {0};
uint8_t config_data_available = false;

static uint16_t crc16(uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;

        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x8005;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void update_config(void)
{
	if(config_data_available == true)
	{
		config_data_available = false;
		//check CRC byte first move on if match, return if mismatch
		uint16_t recv_crc = ((uint16_t)gateway_config_data[CONFIG_DATA_LEN - 2]) << 8 | gateway_config_data[CONFIG_DATA_LEN - 1];
		uint16_t calc_crc = crc16(gateway_config_data, CONFIG_DATA_LEN - 2);
		if (recv_crc != calc_crc) {
			char uart2_tx_buff[60];
            char TempBuffer;
            HAL_StatusTypeDef hal_status;
            do {
                hal_status = HAL_UART_Receive(&huart2, (uint8_t*)&TempBuffer, 1, 10);
            } while(hal_status != HAL_TIMEOUT);

			uint8_t len = sprintf(uart2_tx_buff, "CRC mismatch, received: %X calculated %X\r\n", recv_crc, calc_crc);
			HAL_UART_Transmit(&huart2, (uint8_t*)uart2_tx_buff, len, 100);
			for (int i = 0; i < CONFIG_DATA_LEN; i++) {
				gateway_config_data[i] = 0;
			}
			HAL_UART_Receive_IT(&huart2, gateway_config_data, CONFIG_DATA_LEN); //REINIT THE UART INTERRUPT NOW
			return;
		}

		for (int i = 0; i < 10; i++) {
			for (int j = 0; j < 5; j++) {
				gateway.node[i].rx_pipe_address[j] = gateway_config_data[2+j+i*5];
			}
		}

		for (int i = 0; i < 5; i++) {
			gateway.nrf24_config.rx_pipe0_addr[i] = gateway_config_data[52+i];
		}

		for (int i = 0; i < 5; i++) {
			gateway.nrf24_config.rx_pipe1_addr[i] = gateway_config_data[57+i];
		}

		gateway.nrf24_config.channel = gateway_config_data[62];
		gateway.nrf24_config.drate = gateway_config_data[63];
		gateway.nrf24_config.pwr = gateway_config_data[64];
		gateway.num_nodes = gateway_config_data[65];

		if (EE_Write()) {
			memset(gateway_config_data, 0, sizeof(gateway_config_data));
			HAL_NVIC_SystemReset();
			return;
		} else {
			char uart2_tx_buff[30];
			uint8_t len = sprintf(uart2_tx_buff, "Write to flash failed!\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*)uart2_tx_buff, len, 100);
		}
		//clear buffer before leaving.
		memset(gateway_config_data, 0, sizeof(gateway_config_data));
		HAL_UART_Receive_IT(&huart2, gateway_config_data, CONFIG_DATA_LEN); //REINIT THE UART INTERRUPT NOW
	}
}

void print_config(void)
{
	uint8_t len;
	char uart2_tx_buff[60] = {0};
	for (int i = 0; i < 10; i++) {
		len = sprintf(uart2_tx_buff, "node %d pipe address 0x%X,0x%X,0x%X,0x%X,0x%X\r\n", i+1,
								gateway.node[i].rx_pipe_address[0],
								gateway.node[i].rx_pipe_address[1],
								gateway.node[i].rx_pipe_address[2],
								gateway.node[i].rx_pipe_address[3],
								gateway.node[i].rx_pipe_address[4]);
		HAL_UART_Transmit(&huart2, (uint8_t*)uart2_tx_buff, len, 100);
	}

	len = sprintf(uart2_tx_buff, "rx pipe0 address 0x%X,0x%X,0x%X,0x%X,0x%X\r\n",
							gateway.nrf24_config.rx_pipe0_addr[0],
							gateway.nrf24_config.rx_pipe0_addr[1],
							gateway.nrf24_config.rx_pipe0_addr[2],
							gateway.nrf24_config.rx_pipe0_addr[3],
							gateway.nrf24_config.rx_pipe0_addr[4]);
	HAL_UART_Transmit(&huart2, (uint8_t*)uart2_tx_buff, len, 100);

	len = sprintf(uart2_tx_buff, "rx pipe1 address 0x%X,0x%X,0x%X,0x%X,0x%X\r\n",
							gateway.nrf24_config.rx_pipe1_addr[0],
							gateway.nrf24_config.rx_pipe1_addr[1],
							gateway.nrf24_config.rx_pipe1_addr[2],
							gateway.nrf24_config.rx_pipe1_addr[3],
							gateway.nrf24_config.rx_pipe1_addr[4]);
	HAL_UART_Transmit(&huart2, (uint8_t*)uart2_tx_buff, len, 100);

	len = sprintf(uart2_tx_buff, "Channel: %d\r\n", gateway.nrf24_config.channel);
	HAL_UART_Transmit(&huart2, (uint8_t*)uart2_tx_buff, len, 100);

	len = sprintf(uart2_tx_buff, "Data Rate: %d\r\n", gateway.nrf24_config.drate);
	HAL_UART_Transmit(&huart2, (uint8_t*)uart2_tx_buff, len, 100);

	len = sprintf(uart2_tx_buff, "Tx Power: %d\r\n", gateway.nrf24_config.pwr);
	HAL_UART_Transmit(&huart2, (uint8_t*)uart2_tx_buff, len, 100);

	len = sprintf(uart2_tx_buff, "Number of nodes: %d\r\n", gateway.num_nodes);
	HAL_UART_Transmit(&huart2, (uint8_t*)uart2_tx_buff, len, 100);
}

