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

extern uint8_t nrf24MsgId;

typedef enum {
    NRF_TX_DONE,
    NRF_TX_MAX_RT,
    NRF_TX_INT_FAIL,
	NRF_TX_UNKNOWN_ERROR
} nrfTxStatus;

typedef enum {
	NRF_RX_WAITING,
    NRF_RX_EMPTY,
    NRF_RX_CONFIG,
    NRF_RX_DATA,
	NRF_RX_CRC_ERROR,
	NRF_RX_MID_ERROR,
	NRF_RX_LEN_ERROR,
	NRF_RX_UNKNOWN_ERROR
} nrfRxStatus;


void APP_SwitchToTx(uint8_t *tx_pipe_addr);
void APP_SwitchToRx(uint8_t *rx_pipe0_addr, uint8_t *rx_pipe1_addr);
nrfRxStatus nRF24_receive_data(node_info_t *_node, uint8_t msgId);
nrfTxStatus nRF24_transmit_data(nRF24_transmit_data_t *tx_data, node_t *_node, uint8_t msgId);
void nrf24FailureCheckAndResolve(void);
void printRadioSettings(void);
void printStatusReg(void);
void printConfigReg(void);
void printFIFOstatus(void);

#endif /* INC_NRF24_APP_H_ */
