/**@file  	    nRF24L01P.h
* @brief            nRF24L01+ low level operations and configurations.
* @author           hyh
* @date             2021.6.9
* @version          1.0
* @copyright        Chengdu Ebyte Electronic Technology Co.Ltd
**********************************************************************************
*/
#ifndef nRF24L01P_H
#define nRF24L01P_H

#include "spi.h"
#include "MyTypeDef.h"
#include "nRF24L01P_REG.h"

#define IRQ_ALL ((1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT))
/*Data Rate selection*/
typedef enum {DRATE_250K,DRATE_1M,DRATE_2M}L01_DRATE;
/*Power selection*/
typedef enum {POWER_N_0,POWER_N_6,POWER_N_12,POWER_N_18}L01_PWR;
/*Mode selection*/
typedef enum {TX_MODE,RX_MODE}L01_MODE;
/*CE pin level selection*/
typedef enum {CE_LOW,CE_HIGH}CE_STATUS;
/*!
 *  @brief      Enum to represent LNA enable/disable states.
 */
typedef enum {
    LNA_DISABLE = 0,  /*!< Disable Low Noise Amplifier (LNA) */
    LNA_ENABLE  = 1   /*!< Enable Low Noise Amplifier (LNA) */
} L01_LNAState;
typedef struct
{
	uint8_t rx_pipe0_addr[5];
	uint8_t rx_pipe1_addr[5];
	uint8_t tx_pipe_addr[5];
	uint8_t channel;
	L01_DRATE drate;
	L01_PWR pwr;
}L01_config_t;

/*
================================================================================
============================Configurations and Options==========================
================================================================================
*/
#define DYNAMIC_PACKET      1 //1:DYNAMIC packet length, 0:fixed
#define FIXED_PACKET_LEN    32//Packet size in fixed size mode
#define INIT_ADDR           1,2,3,4,5
/*
================================================================================
==========================List of externally provided functions ================
================================================================================
*/
#define L01_CSN_LOW()      			HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, 0)//Pull down the SPI chip select
#define L01_CSN_HIGH()     			HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, 1)//Pull up the SPI chip select
#define L01_CE_LOW()       			HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, 0)//Set CE low level
#define L01_CE_HIGH()      			HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, 1)//Set CE high level
#define GET_L01_IRQ()      			HAL_GPIO_ReadPin(NRF_IRQ_GPIO_Port, NRF_IRQ_Pin)//Get the IRQ pin status
#define SPI_ExchangeByte(data)      HAL_SPI_ExchangeByte(data) //Exchange data by the SPI

/*
================================================================================
-------------------------------------Exported APIs------------------------------
================================================================================
*/
/*Set the level status of the CE pin low or high*/
void L01_SetCE(CE_STATUS status);
/*Read the value from the specified register */
INT8U L01_ReadSingleReg(INT8U addr);
/*Read the values of the specified registers and store them in buffer*/
void L01_ReadMultiReg(INT8U start_addr,INT8U *buffer,INT8U size);
/*Write a value to the specified register*/
void L01_WriteSingleReg(INT8U addr,INT8U value);
/*Write buffer to the specified registers */
void L01_WriteMultiReg(INT8U start_addr,INT8U *buffer,INT8U size);
/*Set the nRF24L01 into PowerDown mode */
void L01_SetPowerDown(void);
/*Set the nRF24L01 into PowerUp mode*/
void L01_SetPowerUp(void);
/*Flush the TX buffer*/
void L01_FlushTX(void);
/*Flush the RX buffer*/
void L01_FlushRX(void);
/*Reuse the last transmitted payload*/
void L01_ReuseTXPayload(void);
/*Read the status register of the nRF24L01*/
INT8U L01_ReadStatusReg(void);
/*Clear the IRQ caused by the nRF24L01+*/
void L01_ClearIRQ(INT8U irqMask);
/*Read the IRQ status of the nRF24L01+*/
INT8U L01_ReadIRQSource(void);
/*Read the payload width of the top buffer of the FIFO */
INT8U L01_ReadTopFIFOWidth(void);
/*Read the RX payload from the FIFO and store them in buffer*/
INT8U L01_ReadRXPayload(INT8U *buffer);
/*Write TX Payload to a data pipe,and PRX will return ACK back*/
void L01_WriteTXPayload_Ack(INT8U *buffer,INT8U size);
/*Write TX Payload to a data pipe,and PRX won't return ACK back*/
void L01_WriteTXPayload_NoAck(INT8U *buffer,INT8U size);
/*Write TX Payload to a data pipe when RX mode*/
void L01_WriteRXPayload_InAck(INT8U *buffer,INT8U size);
/*Write Transmit address into TX_ADDR register */
void L01_SetTXAddr(INT8U *Addrbuffer,INT8U Addr_size);
/*Write address for the RX pipe*/
void L01_SetRXAddr(INT8U pipeNum,INT8U *addrBuffer,INT8U addr_size);
/*Set the data rate of the nRF24L01+ */
void L01_SetDataRate(L01_DRATE drate);
/*Set the power of the nRF24L01+ */
void L01_SetPower(L01_PWR power);
/*Set the lnastate of the nRF24L01+ */
void L01_SetLNAState(L01_LNAState state);
/*Set the frequency of the nRF24L01+*/
void L01_WriteHoppingPoint(INT8U freq);
/*Set the nRF24L01+ as TX/RX mode*/
void L01_SetTRMode(L01_MODE mode);
/*Initialize the nRF24L01+ */
uint8_t L01_Init(L01_config_t L01);
#endif
