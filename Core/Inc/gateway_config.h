/*
 * gateway_config.h
 *
 *  Created on: Jun 13, 2024
 *      Author: Hamza
 */

#ifndef INC_GATEWAY_CONFIG_H_
#define INC_GATEWAY_CONFIG_H_

#include "nRF24L01P.h"

#define CONFIG_DATA_LEN 68
#define MAX_NODES 10

typedef enum
{
	fan_speed_low = 0,
	fan_speed_medium,
	fan_speed_high,
	fan_off
}fan_speed_t;

typedef enum
{
	mode_ventilation = 0,
	mode_manual,
	mode_auto
}thermostat_mode_t;

typedef struct
{
	uint8_t status;
	uint8_t mode;
	uint8_t fan_speed;
	uint8_t set_temperature;
	uint8_t room_temperature;
	uint8_t room_humidity;
}thermostat_attr_t;

typedef struct
{
	uint8_t temperature;
	uint8_t humidity;
	uint8_t co2;
}ductSensor_attr_t;

typedef struct
{
    uint8_t alarm;
}fireAlarm_attr_t;

typedef struct
{
	uint8_t pressure;
}dpSensor_attr_t;

typedef struct
{
	bool write;
	uint8_t set_temperature;
	bool thermostat_state;
	fan_speed_t fan_speed;
	thermostat_mode_t thermostat_mode;
	uint8_t read;

	bool maxRT;
	bool txDone;
	bool rxDone;
}nRF24_transmit_data_t;

typedef struct
{
	char name[6];
	uint8_t device_type;
	uint8_t EUI[8];
	uint8_t applicationId;
	L01_config_t nrf24_config;
	nRF24_transmit_data_t tx_packet;
	thermostat_attr_t thermostatAttr;
	ductSensor_attr_t ductSensorAttr;
	dpSensor_attr_t dpSensorAttr;
	fireAlarm_attr_t fireAlarmAttr;
	bool ConfigReady;
	uint8_t getConfigCounter;
	uint8_t deadCounter;
	bool alive;
}node_info_t;

//structures to hold data of the gateway device right now only pipe address is given the
//rest of information will come inside the packed received from the node device.
typedef struct
{
	uint8_t rx_pipe_address[5];
}node_t;

typedef struct
{
	node_t node[MAX_NODES];
	L01_config_t nrf24_config;
	uint8_t node_num;
	uint8_t num_nodes;
}gateway_config_t;

extern gateway_config_t gateway;

extern node_info_t node[10];
//extern node_info_t node1;
//extern node_info_t node2;
//extern node_info_t node3;
//extern node_info_t node4;
//extern node_info_t node5;
//extern node_info_t node6;
//extern node_info_t node7;
//extern node_info_t node8;
//extern node_info_t node9;

extern uint8_t gateway_config_data[CONFIG_DATA_LEN];
extern uint8_t config_data_available;

void update_config(void);
void print_config(void);

#endif /* INC_GATEWAY_CONFIG_H_ */
