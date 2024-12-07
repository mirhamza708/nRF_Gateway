/*
 * nrf24_app.c
 *
 *  Created on: Jun 7, 2024
 *      Author: Hamza
 */
#include <stdio.h>
#include "nRF24_app.h"
#include "usart.h"
#include "cmsis_os.h"
//static INT8U RF_SendBuffer[32] = {'1','H','E','L','L','O'};

INT8U nRF24_data_recvd = false;

uint8_t nrf24MsgId = 0;

uint8_t nrf24_failure_check_time = false;
uint8_t nrf24_failure_check_time_counter = 0;
uint8_t maxrtCounter = 0;
bool radioFailureDetected = false;

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

/*!
 *  @brief        Switch the current RF mode to RX
 *  @param        None
 *  @return       None
 *  @note
 */
void APP_SwitchToRx(uint8_t *rx_pipe0_addr, uint8_t *rx_pipe1_addr)
{
	L01_SetCE(CE_LOW);
	L01_SetPowerUp();
	HAL_Delay(3);
//	osDelay(5);
	L01_SetRXAddr(0, rx_pipe0_addr, 5); //Set RX pipe 0 address
//    L01_SetRXAddr(1,rx_pipe1_addr,5);//Set RX pipe 1 address
	L01_SetTRMode(RX_MODE);
	HAL_Delay(3);
//	osDelay(5);
	L01_FlushRX();
	L01_FlushTX();
	L01_ClearIRQ(IRQ_ALL);
	L01_SetCE(CE_HIGH);
}
/*!
 *  @brief        Switch the current RF mode to TX
 *  @param        None
 *  @return       None
 *  @note
 */
void APP_SwitchToTx(uint8_t *tx_pipe_addr)
{
	L01_FlushTX();
	L01_FlushRX();
	L01_ClearIRQ(IRQ_ALL);

	L01_SetCE(CE_LOW);
	L01_SetPowerUp();
	HAL_Delay(3);
//	osDelay(5);
	L01_SetTXAddr(tx_pipe_addr, 5); //Set TX address
	L01_SetRXAddr(0, tx_pipe_addr, 5); //Set RX pipe 0 address to tx address to receive the ACK automatically
	L01_SetTRMode(TX_MODE);
	HAL_Delay(3);
//	osDelay(5);
	L01_FlushRX();
	L01_FlushTX();
	L01_ClearIRQ(IRQ_ALL);
}

/*!
 *  @brief        Receive data from nRF24
 *  @param        None
 *  @return       None
 *  @note
 */
nrfRxStatus nRF24_receive_data(node_info_t *_node, uint8_t msgId)//TODO bring the msgId as a local variable to this file only to be used inside here the application doesnot have to worry about msgId
{
	INT8U len, rcv_buffer[32] =
	{ 0 };
	_node->tx_packet.rxDone = false;
	//check interrupt
	if (GET_L01_IRQ() == 1)
	{
		return NRF_RX_WAITING; // nothing received
	}
	//detect RF module receive interrupt
	if (!(L01_ReadIRQSource() & (1 << RX_DR)))
	{
		return NRF_RX_WAITING; // nothing received
	}

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	len = L01_ReadRXPayload(rcv_buffer);

//	char tx_buff[100];
//	uint8_t lenth = sprintf(tx_buff, "length: %d\r\n", len);
//	HAL_UART_Transmit(&huart2, (uint8_t*) tx_buff, lenth, 100);

    // Verify CRC if the length is at least 2 (to include CRC bytes)
    if (len >= 2) {
        // Calculate CRC on received data (excluding the last two bytes)
        uint16_t calculated_crc = crc16(rcv_buffer, len - 2);

        // Extract received CRC from the last two bytes
        uint16_t received_crc = (rcv_buffer[len - 2] << 8) | rcv_buffer[len - 1];

        // Check if CRC matches
        if (calculated_crc != received_crc) {
            return NRF_RX_CRC_ERROR;  // CRC check failed
        }

        // Adjust length to exclude the CRC bytes for further processing
        len -= 2;
    } else {
        return NRF_RX_LEN_ERROR; // Insufficient data for CRC check
    }

	if (len != 0)
	{
		if (rcv_buffer[0] != msgId) {
			return NRF_RX_MID_ERROR;
		}

		if (len == 8) //means data packet
		{
			_node->device_type = rcv_buffer[1];
			switch (_node->device_type)
			{
			case 1:
				_node->thermostatAttr.status = rcv_buffer[2];
				_node->thermostatAttr.mode = rcv_buffer[3];
				_node->thermostatAttr.fan_speed = rcv_buffer[4];
				_node->thermostatAttr.set_temperature = rcv_buffer[5];
				_node->thermostatAttr.room_humidity = rcv_buffer[6];
				_node->thermostatAttr.room_temperature = rcv_buffer[7];
				break;
			case 2:
				_node->ductSensorAttr.co2 = rcv_buffer[5];
				_node->ductSensorAttr.humidity = rcv_buffer[6];
				_node->ductSensorAttr.temperature = rcv_buffer[7];
				break;
			case 3:

				break;
			case 4:

				break;
			case 5:
				_node->fireAlarmAttr.alarm = rcv_buffer[7];
				break;
			default:
				//do nothing.
				break;
			}
#if NRF_DEBUG == 1
			char tx_buff[100];
			uint8_t lenth = sprintf(tx_buff, "message received >> ");
			HAL_UART_Transmit(&huart2, (uint8_t*) tx_buff, lenth, 100);
			lenth = sprintf(tx_buff, "Params: 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\r\n", rcv_buffer[0],
					rcv_buffer[1], rcv_buffer[2], rcv_buffer[3], rcv_buffer[4],
					rcv_buffer[5], rcv_buffer[6]);
			HAL_UART_Transmit(&huart2, (uint8_t*) tx_buff, lenth, 100);
#endif
			_node->tx_packet.rxDone = true;
			L01_FlushRX();
			L01_ClearIRQ(IRQ_ALL);
			return NRF_RX_DATA; // data length 8
		}
		else if (len == 25) //means configuration packet
		{
			// Copy name
			memcpy(_node->name, rcv_buffer + 1, 6);

			// Copy device_type
			_node->device_type = rcv_buffer[7];

			// Copy EUI
			memcpy(_node->EUI, rcv_buffer + 8, 8);

			_node->applicationId = rcv_buffer[16];
			// Copy channel
			_node->nrf24_config.channel = rcv_buffer[17];

			// Copy rx_pipe1_addr
			memcpy(_node->nrf24_config.rx_pipe1_addr, rcv_buffer + 18, 5);

			// Copy drate and pwr
			_node->nrf24_config.drate = rcv_buffer[23];
			_node->nrf24_config.pwr = rcv_buffer[24];
#if NRF_DEBUG == 1
			char tx_buff[200];  // Increased buffer size to hold the entire formatted string
			uint8_t length =
			    snprintf(tx_buff, sizeof(tx_buff),
			             "Config: 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X,"
			             " 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X,"
			             " 0x%02X, 0x%02X,\r\n",
			             rcv_buffer[0], rcv_buffer[1], rcv_buffer[2],
			             rcv_buffer[3], rcv_buffer[4], rcv_buffer[5],
			             rcv_buffer[6], rcv_buffer[7], rcv_buffer[8],
			             rcv_buffer[9], rcv_buffer[10], rcv_buffer[11],
			             rcv_buffer[12], rcv_buffer[13], rcv_buffer[14],
			             rcv_buffer[15], rcv_buffer[16], rcv_buffer[17],
			             rcv_buffer[18], rcv_buffer[19], rcv_buffer[20],
			             rcv_buffer[21], rcv_buffer[22]);

			// Transmit the message over UART
			HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff, length, 1000);
#endif
			_node->ConfigReady = true;
			_node->tx_packet.rxDone = true;
			L01_FlushRX();
			L01_ClearIRQ(IRQ_ALL);
			return NRF_RX_CONFIG; // data length 24
		}

		L01_FlushRX();
		L01_ClearIRQ(IRQ_ALL);
		return NRF_RX_LEN_ERROR; // data length neither 8 nor 24
	}

	L01_FlushRX();
	L01_ClearIRQ(IRQ_ALL);
	return NRF_RX_EMPTY; // data length 0
}

/*!
 *  @brief        Transmit data through nRF24
 *  @param        None
 *  @return       None
 *  @note
 */
//INT8U fill_rf_tx_buffer(INT8U *tx_buffer)
//{
//	if (nrf_rcv_data.read == 1) {//reply with configuration if read is 1
//		int i;
//		for (i = 0; i < sizeof(thermostat.name); i++) {
//			tx_buffer[i] = thermostat.name[i];
//		}
//
//		for (i = 0; i < sizeof(thermostat.EUI); i++) {
//			tx_buffer[i+sizeof(thermostat.name)] = thermostat.EUI[i];
//		}
//
//		tx_buffer[14] = thermostat.nrf24_config.channel;
//
//		for (i = 0; i < sizeof(thermostat.nrf24_config.rx_pipe0_addr); i++) {
//			tx_buffer[i+15] = thermostat.nrf24_config.rx_pipe0_addr[i];
//		}
//
//		for (i = 0; i < sizeof(thermostat.nrf24_config.rx_pipe1_addr); i++) {
//			tx_buffer[i+20] = thermostat.nrf24_config.rx_pipe1_addr[i];
//		}
//
//		tx_buffer[25] = thermostat.nrf24_config.pwr;
//		tx_buffer[26] = thermostat.nrf24_config.pwr;
//
//		return 27; //crude length of the data to be sent
//	} else if(nrf_rcv_data.read == 2) {
//		tx_buffer[0] = thermostat.attr.status;
//		tx_buffer[1] = thermostat.attr.mode;
//		tx_buffer[2] = thermostat.attr.fan_speed;
//		tx_buffer[3] = thermostat.attr.set_temperature;
//		tx_buffer[4] = thermostat.attr.room_humidity;
//		tx_buffer[5] = thermostat.attr.room_temperature;
//		return 6;
//	} else {
//		// do nothing
//		return 0;
//	}
//}
/*!
 *  @brief        Transmit data through nRF24
 *  @param        None
 *  @return       None
 *  @note
 */
nrfTxStatus nRF24_transmit_data(nRF24_transmit_data_t *tx_data, node_t *_node, uint8_t msgId)
{
	INT8U tx_buffer[32] =
	{ 0 };
	uint32_t txStartTime = 0;

//		INT8U len = fill_rf_tx_buffer(tx_buffer);
	tx_buffer[0] = msgId;
	tx_buffer[1] = tx_data->write;
	tx_buffer[2] = tx_data->thermostat_state;
	tx_buffer[3] = tx_data->thermostat_mode;
	tx_buffer[4] = tx_data->fan_speed;
	tx_buffer[5] = tx_data->set_temperature;
	tx_buffer[6] = tx_data->read;

	HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, 0); //turn on LED and keep on until message sent
	APP_SwitchToTx(_node->rx_pipe_address);
	L01_WriteTXPayload_Ack(tx_buffer, 7);
	L01_SetCE(CE_HIGH);
	txStartTime = HAL_GetTick();
	while (GET_L01_IRQ() != 0 && (HAL_GetTick() - txStartTime) < 95)
		;
	if (GET_L01_IRQ() != 0 && (HAL_GetTick() - txStartTime) >= 95)
	{
		radioFailureDetected = true;
#if NRF_APP_DEBUG == 1
					char tx_buff[50];
					uint8_t lenth = sprintf(tx_buff, "Interrupt not firing\r\n");
					HAL_UART_Transmit(&huart2, (uint8_t*) tx_buff, lenth, 100);
	#endif
		L01_FlushTX();
		L01_FlushRX();
		L01_ClearIRQ(IRQ_ALL);
		APP_SwitchToRx(gateway.nrf24_config.rx_pipe0_addr,
				gateway.nrf24_config.rx_pipe1_addr);
		return NRF_TX_INT_FAIL;
	}
	INT8U irqSrc = L01_ReadIRQSource();
	if (irqSrc & (1 << TX_DS))
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, 1); //turn off led if message sent else keep it on
		tx_data->txDone = true;
		tx_data->write = 0x00; //clear write so no repeated commands are sent if it is sent once
		maxrtCounter = 0;
#if NRF_DEBUG == 1
		char tx_buff[50];
		uint8_t lenth = sprintf(tx_buff,
				"Msg send to: %02X%02X%02X%02X%02X\r\n",
				_node->rx_pipe_address[0], _node->rx_pipe_address[1],
				_node->rx_pipe_address[2], _node->rx_pipe_address[3],
				_node->rx_pipe_address[4]);
		HAL_UART_Transmit(&huart2, (uint8_t*) tx_buff, lenth, 100);
#endif
		L01_FlushTX();
		L01_FlushRX();
		L01_ClearIRQ(IRQ_ALL);
		APP_SwitchToRx(gateway.nrf24_config.rx_pipe0_addr,
				gateway.nrf24_config.rx_pipe1_addr);
		return NRF_TX_DONE;
	}
	else if (irqSrc & (1 << MAX_RT))
	{
		tx_data->maxRT = true;
		tx_data->txDone = false;
#if NRF_DEBUG == 1
		char tx_buff[50];
		uint8_t lenth = sprintf(tx_buff, "MAX RT: %02X%02X%02X%02X%02X\r\n",
				_node->rx_pipe_address[0], _node->rx_pipe_address[1],
				_node->rx_pipe_address[2], _node->rx_pipe_address[3],
				_node->rx_pipe_address[4]);
		HAL_UART_Transmit(&huart2, (uint8_t*) tx_buff, lenth, 100);
#endif
//		HAL_Delay(5);
		maxrtCounter++;
		L01_FlushTX();
		L01_FlushRX();
		L01_ClearIRQ(IRQ_ALL);
		APP_SwitchToRx(gateway.nrf24_config.rx_pipe0_addr,
				gateway.nrf24_config.rx_pipe1_addr);
		return NRF_TX_MAX_RT;
	}
	L01_FlushTX();
	L01_FlushRX();
	L01_ClearIRQ(IRQ_ALL);
	APP_SwitchToRx(gateway.nrf24_config.rx_pipe0_addr,
			gateway.nrf24_config.rx_pipe1_addr);
	return NRF_TX_UNKNOWN_ERROR;
}

void nrf24FailureCheckAndResolve(void)
{
	//check to find if radio hardware failed. if yes then do a reset.

	//if the address returns correct we can atleast say that the SPI communication with the nrf24 is working
	uint8_t pipeAddrs[5];
	L01_ReadMultiReg(L01REG_RX_ADDR_P0, pipeAddrs, 5);
	for (int i = 0; i < 5; i++)
	{
		if (pipeAddrs[i] != gateway.nrf24_config.rx_pipe0_addr[i])
		{
			radioFailureDetected = true;
		}
	}
	//if there is no communication to any node for 100 loops then we can reset the nrf24
	if (maxrtCounter >= 100) {
		radioFailureDetected = true;
		maxrtCounter = 0;
	}

	uint8_t rxData = L01_ReadSingleReg(L01REG_CONFIG);
	if ((rxData & (1 << PWR_UP)) == 0)
	{
		radioFailureDetected = true;
	}

	if (radioFailureDetected == true)
	{
#if NRF_APP_DEBUG == 1
		char *errorSettings = "\r\nRadio settings at failure:\r\n\0";
		HAL_UART_Transmit(&huart2, (uint8_t*) errorSettings,
				strlen(errorSettings), 100);
		printRadioSettings();
		printConfigReg();
		printFIFOstatus();
		printStatusReg();
#endif
		uint8_t ret_code = L01_Init(gateway.nrf24_config);
		if (ret_code == 0)
		{
			radioFailureDetected = false;
		}
		APP_SwitchToRx(gateway.nrf24_config.rx_pipe0_addr,
				gateway.nrf24_config.rx_pipe1_addr);
#if NRF_APP_DEBUG == 1
		printRadioSettings();
		printConfigReg();
		printFIFOstatus();
		printStatusReg();
		char *resetSettings = "\r\nRadio settings after init at failure.\r\n\0";
		HAL_UART_Transmit(&huart2, (uint8_t*) resetSettings,
				strlen(resetSettings), 100);
#endif
	}
}

void printRadioSettings(void)
{
	uint8_t reg8Val;
	char uartTxBuf[100];
	sprintf(uartTxBuf,
			"\r\n**********************************************\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);
	//a) Get CRC settings - Config Register
	reg8Val = L01_ReadSingleReg(0x00);
	if (reg8Val & (1 << 3))
	{
		if (reg8Val & (1 << 2))
			sprintf(uartTxBuf, "CRC:\r\n    Enabled, 2 Bytes \r\n");
		else
			sprintf(uartTxBuf, "CRC:\r\n    Enabled, 1 Byte \r\n");
	}
	else
	{
		sprintf(uartTxBuf, "CRC:\r\n    Disabled \r\n");
	}
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);
	//b) AutoAck on pipes
	reg8Val = L01_ReadSingleReg(0x01);
	sprintf(uartTxBuf,
			"ENAA:\r\n    P0: %d\r\n    P1: %d\r\n    P2: %d\r\n    P3: %d\r\n    P4: %d\r\n    P5: %d\r\n",
			_BOOL(reg8Val & (1 << 0)), _BOOL(reg8Val & (1 << 1)),
			_BOOL(reg8Val & (1 << 2)), _BOOL(reg8Val & (1 << 3)),
			_BOOL(reg8Val & (1 << 4)), _BOOL(reg8Val & (1 << 5)));
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);
	//c) Enabled Rx addresses
	reg8Val = L01_ReadSingleReg(0x02);
	sprintf(uartTxBuf,
			"EN_RXADDR:\r\n    P0: %d\r\n    P1: %d\r\n    P2: %d\r\n    P3: %d\r\n    P4: %d\r\n    P5: %d\r\n",
			_BOOL(reg8Val & (1 << 0)), _BOOL(reg8Val & (1 << 1)),
			_BOOL(reg8Val & (1 << 2)), _BOOL(reg8Val & (1 << 3)),
			_BOOL(reg8Val & (1 << 4)), _BOOL(reg8Val & (1 << 5)));
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);
	//d) Address width
	reg8Val = L01_ReadSingleReg(0x03) & 0x03;
	reg8Val += 2;
	sprintf(uartTxBuf, "SETUP_AW:\r\n    %d bytes \r\n", reg8Val);
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);
	//e) RF channel
	reg8Val = L01_ReadSingleReg(0x05);
	sprintf(uartTxBuf, "RF_CH:\r\n    %d CH \r\n", reg8Val & 0x7F);
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);
	//f) Data rate & RF_PWR
	reg8Val = L01_ReadSingleReg(0x06);
	if (reg8Val & (1 << 3))
		sprintf(uartTxBuf, "Data Rate:\r\n    2Mbps \r\n");
	else
		sprintf(uartTxBuf, "Data Rate:\r\n    1Mbps \r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);
	reg8Val &= (3 << 1);
	reg8Val = (reg8Val >> 1);
	if (reg8Val == 0)
		sprintf(uartTxBuf, "RF_PWR:\r\n    -18dB \r\n");
	else if (reg8Val == 1)
		sprintf(uartTxBuf, "RF_PWR:\r\n    -12dB \r\n");
	else if (reg8Val == 2)
		sprintf(uartTxBuf, "RF_PWR:\r\n    -6dB \r\n");
	else if (reg8Val == 3)
		sprintf(uartTxBuf, "RF_PWR:\r\n    0dB \r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);
	//g) RX pipes addresses
	uint8_t pipeAddrs[6];
	L01_ReadMultiReg(0x0A, pipeAddrs, 5);
	sprintf(uartTxBuf, "RX_Pipe0 Addrs:\r\n    %02X,%02X,%02X,%02X,%02X  \r\n",
			pipeAddrs[4], pipeAddrs[3], pipeAddrs[2], pipeAddrs[1],
			pipeAddrs[0]);
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);

	L01_ReadMultiReg(0x0A + 1, pipeAddrs, 5);
	sprintf(uartTxBuf, "RX_Pipe1 Addrs:\r\n    %02X,%02X,%02X,%02X,%02X  \r\n",
			pipeAddrs[4], pipeAddrs[3], pipeAddrs[2], pipeAddrs[1],
			pipeAddrs[0]);
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);

	L01_ReadMultiReg(0x0A + 2, pipeAddrs, 1);
	sprintf(uartTxBuf, "RX_Pipe2 Addrs:\r\n    xx,xx,xx,xx,%02X  \r\n",
			pipeAddrs[0]);
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);

	L01_ReadMultiReg(0x0A + 3, pipeAddrs, 1);
	sprintf(uartTxBuf, "RX_Pipe3 Addrs:\r\n    xx,xx,xx,xx,%02X  \r\n",
			pipeAddrs[0]);
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);

	L01_ReadMultiReg(0x0A + 4, pipeAddrs, 1);
	sprintf(uartTxBuf, "RX_Pipe4 Addrs:\r\n    xx,xx,xx,xx,%02X  \r\n",
			pipeAddrs[0]);
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);

	L01_ReadMultiReg(0x0A + 5, pipeAddrs, 1);
	sprintf(uartTxBuf, "RX_Pipe5 Addrs:\r\n    xx,xx,xx,xx,%02X  \r\n",
			pipeAddrs[0]);
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);

	L01_ReadMultiReg(0x0A + 6, pipeAddrs, 5);
	sprintf(uartTxBuf, "TX Addrs:\r\n    %02X,%02X,%02X,%02X,%02X  \r\n",
			pipeAddrs[4], pipeAddrs[3], pipeAddrs[2], pipeAddrs[1],
			pipeAddrs[0]);
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);

	//h) RX PW (Payload Width 0 - 32)
	reg8Val = L01_ReadSingleReg(0x11);
	sprintf(uartTxBuf, "RX_PW_P0:\r\n    %d bytes \r\n", reg8Val & 0x3F);
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);

	reg8Val = L01_ReadSingleReg(0x11 + 1);
	sprintf(uartTxBuf, "RX_PW_P1:\r\n    %d bytes \r\n", reg8Val & 0x3F);
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);

	reg8Val = L01_ReadSingleReg(0x11 + 2);
	sprintf(uartTxBuf, "RX_PW_P2:\r\n    %d bytes \r\n", reg8Val & 0x3F);
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);

	reg8Val = L01_ReadSingleReg(0x11 + 3);
	sprintf(uartTxBuf, "RX_PW_P3:\r\n    %d bytes \r\n", reg8Val & 0x3F);
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);

	reg8Val = L01_ReadSingleReg(0x11 + 4);
	sprintf(uartTxBuf, "RX_PW_P4:\r\n    %d bytes \r\n", reg8Val & 0x3F);
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);

	reg8Val = L01_ReadSingleReg(0x11 + 5);
	sprintf(uartTxBuf, "RX_PW_P5:\r\n    %d bytes \r\n", reg8Val & 0x3F);
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);

	//i) Dynamic payload enable for each pipe
	reg8Val = L01_ReadSingleReg(0x1c);
	sprintf(uartTxBuf,
			"DYNPD_pipe:\r\n    P0: %d\r\n    P1: %d\r\n    P2: %d\r\n    P3: %d\r\n    P4: %d\r\n    P5: %d\r\n",
			_BOOL(reg8Val & (1 << 0)), _BOOL(reg8Val & (1 << 1)),
			_BOOL(reg8Val & (1 << 2)), _BOOL(reg8Val & (1 << 3)),
			_BOOL(reg8Val & (1 << 4)), _BOOL(reg8Val & (1 << 5)));
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);

	//j) EN_DPL (is Dynamic payload feature enabled ?)
	reg8Val = L01_ReadSingleReg(0x1d);
	if (reg8Val & (1 << 2))
		sprintf(uartTxBuf, "EN_DPL:\r\n    Enabled \r\n");
	else
		sprintf(uartTxBuf, "EN_DPL:\r\n    Disabled \r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);

	//k) EN_ACK_PAY
	if (reg8Val & (1 << 1))
		sprintf(uartTxBuf, "EN_ACK_PAY:\r\n    Enabled \r\n");
	else
		sprintf(uartTxBuf, "EN_ACK_PAY:\r\n    Disabled \r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);

	sprintf(uartTxBuf,
			"\r\n**********************************************\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);
}

//2. Print Status
void printStatusReg(void)
{
	uint8_t reg8Val;
	char uartTxBuf[100];
	sprintf(uartTxBuf, "\r\n-------------------------\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);

	reg8Val = L01_ReadSingleReg(0x07);
	sprintf(uartTxBuf,
			"STATUS reg:\r\n    RX_DR: %d\r\n    TX_DS: %d\r\n    MAX_RT: %d\r\n    RX_P_NO: %d\r\n    TX_FULL: %d\r\n",
			_BOOL(reg8Val & (1 << 6)), _BOOL(reg8Val & (1 << 5)),
			_BOOL(reg8Val & (1 << 4)), _BOOL(reg8Val & (3 << 1)),
			_BOOL(reg8Val & (1 << 0)));
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);

	sprintf(uartTxBuf, "\r\n-------------------------\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);
}
//3. Print Config
void printConfigReg(void)
{
	uint8_t reg8Val;
	char uartTxBuf[100];

	sprintf(uartTxBuf, "\r\n-------------------------\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);

	reg8Val = L01_ReadSingleReg(0x00);
	sprintf(uartTxBuf, "CONFIG reg:\r\n    PWR_UP: %d\r\n    PRIM_RX: %d\r\n",
			_BOOL(reg8Val & (1 << 1)), _BOOL(reg8Val & (1 << 0)));
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);

	sprintf(uartTxBuf, "\r\n-------------------------\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);
}

//5. FIFO Status
void printFIFOstatus(void)
{
	uint8_t reg8Val;
	char uartTxBuf[100];
	sprintf(uartTxBuf, "\r\n-------------------------\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);

	reg8Val = L01_ReadSingleReg(0x17);
	sprintf(uartTxBuf,
			"FIFO Status reg:\r\n    TX_FULL: %d\r\n    TX_EMPTY: %d\r\n    RX_FULL: %d\r\n    RX_EMPTY: %d\r\n",
			_BOOL(reg8Val & (1 << 5)), _BOOL(reg8Val & (1 << 4)),
			_BOOL(reg8Val & (1 << 1)), _BOOL(reg8Val & (1 << 0)));
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);

	sprintf(uartTxBuf, "\r\n-------------------------\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) uartTxBuf, strlen(uartTxBuf), 10);

}

