/*
 * nrf24_app.h
 *
 *  Created on: Jun 7, 2024
 *      Author: Hamza
 */

#ifndef INC_NRF24_APP_H_
#define INC_NRF24_APP_H_

#include "main.h"
#include "MyTypeDef.h"
#include "nRF24L01P.h"
#include "gateway_config.h"

//#define NRF_DEBUG 1

#define _BOOL(x) (((x)>0) ? 1:0)

extern uint8_t nrf24_failure_check_time;
extern uint8_t nrf24_failure_check_time_counter;
extern INT8U nRF24_data_recvd;
extern uint8_t maxrtCounter;

void APP_SwitchToTx(uint8_t *tx_pipe_addr);
void APP_SwitchToRx(uint8_t *rx_pipe0_addr, uint8_t *rx_pipe1_addr);
uint8_t nRF24_receive_data(node_info_t *_node);
uint8_t nRF24_transmit_data(nRF24_transmit_data_t *tx_data, node_t *_node);
void nrf24FailureCheckAndResolve(void);
void printRadioSettings(void);
void printStatusReg(void);
void printConfigReg(void);
void printFIFOstatus(void);

#endif /* INC_NRF24_APP_H_ */
