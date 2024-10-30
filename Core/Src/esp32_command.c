/*
 * esp32_command_data.c
 *
 *  Created on: Jun 16, 2024
 *      Author: Hamza
 */
#include "esp32_command.h"

#include "interrupts_app.h"
#include "usart.h"

uint8_t esp32_command_data[ESP32_COMMAND_LEN] = {0};

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

void sendToESP32(node_info_t *_node, uint8_t nodeNumber)
{
#if COMMAND_DEBUG == 1
		char tx_buff[50];
		uint8_t lenth = sprintf(tx_buff, "Check if data ready to publish\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff, lenth, 100);
#endif
        if (_node->ConfigReady == true) {
            char reply[50]; // Ensure this is large enough to hold all data and CRC
            int offset = 0;

            // Start with nodeNumber
            reply[offset++] = nodeNumber;
            reply[offset++] = _node->alive;
            // Copy device name
            memcpy(&reply[offset], _node->name, sizeof(_node->name));
            offset += sizeof(_node->name);

            // Copy device type
            reply[offset++] = _node->device_type;

            // Copy EUI
            memcpy(&reply[offset], _node->EUI, sizeof(_node->EUI));
            offset += sizeof(_node->EUI);

            // Copy channel
            reply[offset++] = _node->nrf24_config.channel;

            // Copy rx_pipe1_addr
            memcpy(&reply[offset], _node->nrf24_config.rx_pipe1_addr, sizeof(_node->nrf24_config.rx_pipe1_addr));
            offset += sizeof(_node->nrf24_config.rx_pipe1_addr);

            // Copy drate
            reply[offset++] = _node->nrf24_config.drate;

            // Copy pwr
            reply[offset++] = _node->nrf24_config.pwr;

            // Copy thermostat attributes
            switch (_node->device_type) {
            	case 1:
					reply[offset++] = _node->thermostatAttr.status;
					reply[offset++] = _node->thermostatAttr.mode;
					reply[offset++] = _node->thermostatAttr.fan_speed;

					reply[offset++] = _node->thermostatAttr.set_temperature;
					reply[offset++] = _node->thermostatAttr.room_humidity;
					reply[offset++] = _node->thermostatAttr.room_temperature;
					break;
            	case 2:
            		reply[offset++] = 0;
					reply[offset++] = 0;
					reply[offset++] = 0;
					reply[offset++] = _node->ductSensorAttr.co2;
					reply[offset++] = _node->ductSensorAttr.humidity;
					reply[offset++] = _node->ductSensorAttr.temperature;
					break;
            	case 3:
            		reply[offset++] = 0;
					reply[offset++] = 0;
					reply[offset++] = 0;
					reply[offset++] = 0;
					reply[offset++] = 0;
					reply[offset++] = 22;
					break;
            	case 5:
            		reply[offset++] = 0;
					reply[offset++] = 0;
					reply[offset++] = 0;
					reply[offset++] = 0;
					reply[offset++] = 0;
					reply[offset++] = _node->fireAlarmAttr.alarm;
					break;
            	default:
            		//all data zero
            		reply[offset++] = 0;
					reply[offset++] = 0;
					reply[offset++] = 0;
					reply[offset++] = 0;
					reply[offset++] = 0;
					reply[offset++] = 0;
            		break;
            }
            // Calculate CRC
            uint16_t crc = crc16((uint8_t*)reply, offset);

            // Append CRC to the reply array
            reply[offset++] = (crc >> 8) & 0xFF; // CRC high byte
            reply[offset++] = crc & 0xFF;        // CRC low byte

            // Transmit the data
#if COMMAND_DEBUG == 1
            char tx_buff[50];
            uint8_t lenth = sprintf(tx_buff, "Sending data to MQTT\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff, lenth, 100);
#endif
            HAL_UART_Transmit(&huart1, (uint8_t*)reply, offset, 100);
        }
}

void process_command(void)
{
	if (esp32_command_available != true) {
		return;
	}

	// Extract the received CRC from esp32_command_data[7] and esp32_command_data[8]
	uint16_t received_crc = (esp32_command_data[7] << 8) | esp32_command_data[8];

	// Calculate CRC of the first 7 bytes of esp32_command_data
	uint16_t calculated_crc = crc16(esp32_command_data, 7);

	if (calculated_crc == received_crc) {
	    // CRC is valid; proceed with processing data
	    gateway.node_num = esp32_command_data[0];
	    node[gateway.node_num].tx_packet.write = esp32_command_data[1];
	    node[gateway.node_num].tx_packet.thermostat_state = esp32_command_data[2];
	    node[gateway.node_num].tx_packet.thermostat_mode = esp32_command_data[3];
	    node[gateway.node_num].tx_packet.fan_speed = esp32_command_data[4];
	    node[gateway.node_num].tx_packet.set_temperature = esp32_command_data[5];
	    node[gateway.node_num].tx_packet.read = esp32_command_data[6];

	    char tx_buff[150];
		uint8_t length = sprintf(tx_buff,
		    "MQTT Command:\nNode: %d\nWrite: %d\nThermostat State: %d\nThermostat Mode: %d\nFan Speed: %d\nSet Temp: %d\nRead: %d\r\n\r\n",
		    esp32_command_data[0] + 1, esp32_command_data[1], esp32_command_data[2],
		    esp32_command_data[3], esp32_command_data[4], esp32_command_data[5],
		    esp32_command_data[6]
		);
		HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff, length, 100);
	}

	char tx_buff[50];
	sprintf(tx_buff, "done\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)tx_buff, 5, 100);

	esp32_command_available = false;

	memset(esp32_command_data, 0, sizeof(esp32_command_data));
	memset(tx_buff, 0, sizeof(tx_buff));
	// Debug buffer

	if (HAL_UART_Receive_IT(&huart1, esp32_command_data, ESP32_COMMAND_LEN) != HAL_OK) {
	    // Check and handle specific UART errors
	    if (huart1.ErrorCode & HAL_UART_ERROR_ORE)  // Overrun error
	    {
	        __HAL_UART_CLEAR_OREFLAG(&huart1);  // Clear overrun error flag
	        // Debug message for overrun error
#if COMMAND_DEBUG == 1
	        sprintf(tx_buff, "UART Error: Overrun detected, cleared ORE flag\r\n");
	        HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff, strlen(tx_buff), 100);
#endif
	    }
	    if (huart1.ErrorCode & HAL_UART_ERROR_FE)  // Framing error
	    {
	        __HAL_UART_CLEAR_FEFLAG(&huart1);  // Clear framing error flag
	        // Debug message for framing error
#if COMMAND_DEBUG == 1
	        sprintf(tx_buff, "UART Error: Framing error detected, cleared FE flag\r\n");
	        HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff, strlen(tx_buff), 100);
#endif
	    }
	    if (huart1.ErrorCode & HAL_UART_ERROR_NE)  // Noise error
	    {
	        __HAL_UART_CLEAR_NEFLAG(&huart1);  // Clear noise error flag
	        // Debug message for noise error
#if COMMAND_DEBUG == 1
	        sprintf(tx_buff, "UART Error: Noise error detected, cleared NE flag\r\n");
	        HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff, strlen(tx_buff), 100);
#endif
	    }
	    if (huart1.ErrorCode & HAL_UART_ERROR_PE)  // Parity error
	    {
	        __HAL_UART_CLEAR_PEFLAG(&huart1);  // Clear parity error flag
	        // Debug message for parity error
#if COMMAND_DEBUG == 1
	        sprintf(tx_buff, "UART Error: Parity error detected, cleared PE flag\r\n");
	        HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff, strlen(tx_buff), 100);
#endif
	    }

	    // Attempt to restart UART reception in interrupt mode
	    if (HAL_UART_Receive_IT(&huart1, esp32_command_data, ESP32_COMMAND_LEN) != HAL_OK) {
	        // If restart fails, then reinitialize the UART
	        HAL_UART_DeInit(&huart1);
	        MX_USART1_UART_Init();

	        // Debug message for reinitialization
#if COMMAND_DEBUG == 1
	        sprintf(tx_buff, "UART reinitialization: Reception restart failed, UART reinitialized\r\n");
	        HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff, strlen(tx_buff), 100);
#endif
	    } else {
	        // Debug message for successful restart
#if COMMAND_DEBUG == 1
	    	sprintf(tx_buff, "UART Reception restarted successfully after clearing errors\r\n");
	        HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff, strlen(tx_buff), 100);
#endif
	    }
	}
}


