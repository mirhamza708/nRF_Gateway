/*
 * esp32_command.h
 *
 *  Created on: Jun 16, 2024
 *      Author: Hamza
 */

#ifndef INC_ESP32_COMMAND_H_
#define INC_ESP32_COMMAND_H_
#include "main.h"
#include "gateway_config.h"

#define ESP32_COMMAND_LEN 17

extern uint8_t esp32_command_data[ESP32_COMMAND_LEN];

void sendToESP32(node_info_t *_node, uint8_t nodeNumber);
void process_command(void);

#endif /* INC_ESP32_COMMAND_H_ */
