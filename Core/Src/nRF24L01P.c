/**@file  	    nRF24L01P.c
* @brief            nRF24L01+ low level operations and configurations.
* @author           hyh
* @date             2021.9.17
* @version          1.0
* @copyright        Chengdu Ebyte Electronic Technology Co.Ltd
**********************************************************************************
*/
#include "nRF24L01P.h"
#include "main.h"
#include "usart.h"
/*the CE pin level status*/
static INT8U CE_Status = 0;
/*!
================================================================================
------------------------------------Functions-----------------------------------
================================================================================
*/
/*!
 *  @brief          Get the level status of the CE pin
 *  @param          None     
 *  @return         CE pin level status: 0:low; 1:high
 *  @note          
*/
INT8U L01_GetCEStatus(void)
{
    return CE_Status;
}
/*!
 *  @brief          Set the level status of the CE pin low or high
 *  @param          status:CE pin level status    
 *  @return         None
 *  @note          
*/
void L01_SetCE(CE_STATUS status)
{
    CE_Status = status;
    if (status == CE_LOW)    { L01_CE_LOW(); }
    else                     { L01_CE_HIGH(); }
}
/*!
 *  @brief        Read the value from the specified register   
 *  @param        addr:the address of the register
 *  @return       value:the value read from the register  
 *  @note          
*/
INT8U L01_ReadSingleReg(INT8U addr)
{
    INT8U value;
    L01_CSN_LOW();
    SPI_ExchangeByte( R_REGISTER | addr);
    value = SPI_ExchangeByte(NOP);
    L01_CSN_HIGH();
    return value;
}
/*!
 *  @brief        Read the values of the specified registers and store them in buffer
 *  @param        start_addr:the start address of the registers
 *  @param        buffer:the buffer stores the read values
*  @param         size:the size to be read
 *  @return       None  
 *  @note          
*/
void L01_ReadMultiReg(INT8U start_addr,INT8U *buffer,INT8U size)
{
    INT8U i;
    L01_CSN_LOW();
    SPI_ExchangeByte(R_REGISTER | start_addr);
    for (i = 0; i < size; i++)
    {
        *(buffer + i) = SPI_ExchangeByte(NOP);
    }
    L01_CSN_HIGH();
}
/*!
 *  @brief        Write a value to the specified register   
 *  @param        addr:the address of the register
 *  @param        value:the value to be written  
 *  @return       None
 *  @note          
*/
void L01_WriteSingleReg(INT8U addr,INT8U value)
{
    L01_CSN_LOW();
    SPI_ExchangeByte(W_REGISTER | addr);
    SPI_ExchangeByte(value);
    L01_CSN_HIGH();
}
/*!
 *  @brief        Write buffer to the specified registers  
 *  @param        start_addr:the start address of the registers
 *  @param        buffer:the buffer to be written
 *  @param        size:the size to be written  
 *  @return       None
 *  @note          
*/
void L01_WriteMultiReg(INT8U start_addr,INT8U *buffer,INT8U size)
{
    INT8U i;
    L01_CSN_LOW();
    SPI_ExchangeByte(W_REGISTER | start_addr);
    for ( i = 0; i < size; i++)
    {
        SPI_ExchangeByte(*(buffer + i));
    }
    L01_CSN_HIGH();
}
/*!
 *  @brief        Set the nRF24L01 into PowerDown mode          
 *  @param        None
 *  @return       None  
 *  @note          
*/
void L01_SetPowerDown(void)
{
    INT8U controlreg = L01_ReadSingleReg(L01REG_CONFIG);
    L01_WriteSingleReg(L01REG_CONFIG,controlreg & (~(1 << PWR_UP)));
}
/*!
 *  @brief        Set the nRF24L01 into PowerUp mode       
 *  @param        None
 *  @return       None  
 *  @note          
*/
void L01_SetPowerUp(void)
{
    INT8U controlreg = L01_ReadSingleReg(L01REG_CONFIG);
    L01_WriteSingleReg(L01REG_CONFIG,controlreg | (1 << PWR_UP));
}
/*!
 *  @brief        Flush the TX buffer             
 *  @param        None 
 *  @return       None  
 *  @note          
*/
void L01_FlushTX(void)
{
    L01_CSN_LOW();
    SPI_ExchangeByte(FLUSH_TX);
    L01_CSN_HIGH();
}
/*!
 *  @brief        Flush the RX buffer           
 *  @param        None
 *  @return       None  
 *  @note          
*/
void L01_FlushRX(void)
{
    L01_CSN_LOW();
    SPI_ExchangeByte(FLUSH_RX);
    L01_CSN_HIGH();
}
/*!
 *  @brief        Reuse the last transmitted payload           
 *  @param        None
 *  @return       None  
 *  @note          
*/
void L01_ReuseTXPayload(void)
{
    L01_CSN_LOW();
    SPI_ExchangeByte(REUSE_TX_PL);
    L01_CSN_HIGH();
}
/*!
 *  @brief        NOP operation about the nRF24L01+           
 *  @param        None
 *  @return       None  
 *  @note          
*/
void L01_Nop(void)
{
    L01_CSN_LOW();
    SPI_ExchangeByte(NOP);
    L01_CSN_HIGH();
}
/*!
 *  @brief        Read the status register of the nRF24L01+           
 *  @param        None
 *  @return       the value of the status register  
 *  @note          
*/
INT8U L01_ReadStatusReg(void)
{
    INT8U status;
    L01_CSN_LOW();
    status = SPI_ExchangeByte(R_REGISTER + L01REG_STATUS);
    L01_CSN_HIGH();
    return status;
}
/*!
 *  @brief        Clear the IRQ caused by the nRF24L01+           
 *  @param        irqMask:RX_DR(bit[6]),TX_DS(bit[5]),MAX_RT(bit[4])
 *  @return       None  
 *  @note          
*/
void L01_ClearIRQ(INT8U irqMask)
{
    INT8U status = 0;
    irqMask &= IRQ_ALL;
    status = L01_ReadStatusReg();
    L01_CSN_LOW();
    L01_WriteSingleReg(L01REG_STATUS,irqMask | status);
    L01_CSN_HIGH();
    L01_ReadStatusReg();
}
/*!
 *  @brief        Read the IRQ status of the nRF24L01+           
 *  @param        None
 *  @return       irqMask:RX_DR(bit[6]),TX_DS(bit[5]),MAX_RT(bit[4]) 
 *  @note          
*/
INT8U L01_ReadIRQSource(void)
{
    INT8U status = 0;
    status = L01_ReadStatusReg();
    return (status & IRQ_ALL);
}
/*!
 *  @brief        Read the payload width of the top buffer of the FIFO           
 *  @param        None
 *  @return       width:the width of the pipe buffer  
 *  @note          
*/
INT8U L01_ReadTopFIFOWidth(void)
{
    INT8U width;
    L01_CSN_LOW();
    SPI_ExchangeByte(R_RX_PL_WID);
    width = SPI_ExchangeByte(NOP);
    L01_CSN_HIGH();
    return width;
}
/*!
 *  @brief        Read the RX payload from the FIFO and store them in buffer            
 *  @param        buffer:the buffer to store the data
 *  @return       the length to be read
 *  @note          
*/
INT8U L01_ReadRXPayload(INT8U *buffer)
{
    INT8U width,i;
    width = L01_ReadTopFIFOWidth();
    if(width > 32)
    {
        L01_CSN_HIGH();
        L01_FlushRX();
        return 0;
    }
    L01_CSN_LOW();
    SPI_ExchangeByte(R_RX_PAYLOAD);
    for (i = 0; i < width; i++)
    {
        *(buffer + i) = SPI_ExchangeByte(NOP);
    }
    L01_CSN_HIGH();
    L01_FlushRX();
    return width;
}
/*!
 *  @brief        Write TX Payload to a data pipe,and PRX will return ACK back         
 *  @param        buffer:the buffer stores the data
 *  @param        size:the size to be written  
 *  @return       None  
 *  @note          
*/
void L01_WriteTXPayload_Ack(INT8U *buffer,INT8U size)
{
    INT8U i;
    INT8U w_size = (size > 32) ? 32 : size;
    L01_FlushTX();
    L01_CSN_LOW();
    SPI_ExchangeByte(W_TX_PAYLOAD);
    for (i = 0; i < w_size; i++)
    {
        SPI_ExchangeByte(*(buffer + i));
    }
    L01_CSN_HIGH();
}
/*!
 *  @brief        Write TX Payload to a data pipe,and PRX won't return ACK back         
 *  @param        buffer:the buffer stores the data
 *  @param        size:the size to be written  
 *  @return       None  
 *  @note          
*/
void L01_WriteTXPayload_NoAck(INT8U *buffer,INT8U size)
{
    if (size > 32 || size == 0)
    {
        return;
    }
    L01_CSN_LOW();
    SPI_ExchangeByte(W_TX_PAYLOAD_NOACK);
    while (size --)
    {
        SPI_ExchangeByte(*buffer++);
    }
    L01_CSN_HIGH();
}
/*!
 *  @brief        Write TX Payload to a data pipe when RX mode         
 *  @param        buffer:the buffer stores the data
 *  @param        size:the size to be written  
 *  @return       None  
 *  @note          
*/
void L01_WriteRXPayload_InAck(INT8U *buffer,INT8U size)
{
    INT8U i;
    INT8U w_size = (size > 32) ? 32 : size;
    L01_CSN_LOW();
    SPI_ExchangeByte(W_ACK_PAYLOAD);
    for (i = 0; i < w_size; i++)
    {
        SPI_ExchangeByte(*(buffer + i));
    }
    L01_CSN_HIGH();
}
/*!
 *  @brief        Write Transmit address into TX_ADDR register          
 *  @param        addrBuffer:the buffer stores the address
 *  @param        addr_size:the address byte num
 *  @return       None  
 *  @note         Used for a PTX device only 
*/
void L01_SetTXAddr(INT8U *addrBuffer,INT8U addr_size)
{
    INT8U size = (addr_size > 5) ? 5 : addr_size;
    L01_WriteMultiReg(L01REG_TX_ADDR,addrBuffer,size);
}
/*!
 *  @brief        Write address for the RX pipe 
 *  @param        pipeNum:the number of the data pipe         
 *  @param        addrBuffer:the buffer stores the address
 *  @param        addr_size:the address byte num
 *  @return       None  
 *  @note          
*/
void L01_SetRXAddr(INT8U pipeNum,INT8U *addrBuffer,INT8U addr_size)
{
    INT8U size = (addr_size > 5) ? 5 : addr_size;
    INT8U num = (pipeNum > 5) ? 5 : pipeNum;
    L01_WriteMultiReg(L01REG_RX_ADDR_P0 + num,addrBuffer,size);
}
/*!
 *  @brief        Set the data rate of the nRF24L01+          
 *  @param        drate:250K,1M,2M
 *  @return       None  
 *  @note          
*/
void L01_SetDataRate(L01_DRATE drate)
{
    INT8U mask = L01_ReadSingleReg(L01REG_RF_SETUP);
    mask &= ~((1 << RF_DR_LOW) | (1 << RF_DR_HIGH));
    if(drate == DRATE_250K)
    {
        mask |= (1 << RF_DR_LOW);
    }
    else if(drate == DRATE_1M)
    {
        mask &= ~((1 << RF_DR_LOW) | (1 << RF_DR_HIGH));
    }
    else if(drate == DRATE_2M)
    {
        mask |= (1 << RF_DR_HIGH);
    }
    L01_WriteSingleReg(L01REG_RF_SETUP,mask);
}
/*!
 *  @brief        Set the power of the nRF24L01+          
 *  @param        power:18dB,12dB,6dB,0dB
 *  @return       None  
 *  @note          
*/
void L01_SetPower(L01_PWR power)
{
    INT8U mask = L01_ReadSingleReg(L01REG_RF_SETUP) & ~0x07;
    switch (power)
    {
    case POWER_N_18:
        mask |= PWR_N_18DB;
        break;
    case POWER_N_12:
        mask |= PWR_N_12DB;
        break;
    case POWER_N_6:
        mask |= PWR_N_6DB;
        break;
    case POWER_N_0:
        mask |= PWR_N_0DB;
        break;
    default:
        break;
    }
    L01_WriteSingleReg(L01REG_RF_SETUP,mask);
}
/*!
 *  @brief        Set the frequency of the nRF24L01+          
 *  @param        freq:the hopping frequency point,range:0-125,2400Mhz-2525Mhz
 *  @return       None  
 *  @note          
*/
void L01_WriteHoppingPoint(INT8U freq)
{
    L01_WriteSingleReg(L01REG_RF_CH,freq <= 125 ? freq : 125);
}
/*!
 *  @brief        Set the nRF24L01+ as TX/RX mode         
 *  @param        mode:TX/RX
 *  @return       None  
 *  @note          
*/
void L01_SetTRMode(L01_MODE mode)
{
    INT8U mask = L01_ReadSingleReg(L01REG_CONFIG);
    if (mode == TX_MODE)
    {
        mask &= ~(1 << PRIM_RX);
    }
    else if (mode == RX_MODE)
    {
        mask |= (1 << PRIM_RX);
    }
    L01_WriteSingleReg(L01REG_CONFIG,mask);
}

/*!
 *  @brief        Resets the nRF24L01+ to default settings
 *  @param        None
 *  @return       None
 *  @note
*/
void L01_Reset(void)
{
    // Reset pins
	L01_CSN_HIGH();
	L01_CE_LOW();

	HAL_Delay(100);
    // Reset registers
    L01_WriteSingleReg(L01REG_CONFIG, 0x08);
    L01_WriteSingleReg(L01REG_ENAA, 0x3F);
    L01_WriteSingleReg(L01REG_EN_RXADDR, 0x03);
    L01_WriteSingleReg(L01REG_SETUP_AW, 0x03);
    L01_WriteSingleReg(L01REG_SETUP_RETR, 0x03);
    L01_WriteSingleReg(L01REG_RF_CH, 0x02);
    L01_WriteSingleReg(L01REG_RF_SETUP, 0x07);
    L01_WriteSingleReg(L01REG_STATUS, 0x7E);
//    L01_WriteSingleReg(L01REG_OBSERVE_TX, 0x00); not sure if this reg is writable
    L01_WriteSingleReg(L01REG_RX_PW_P0, 0x00);
    L01_WriteSingleReg(L01REG_RX_PW_P0, 0x00);
    L01_WriteSingleReg(L01REG_RX_PW_P1, 0x00);
    L01_WriteSingleReg(L01REG_RX_PW_P2, 0x00);
    L01_WriteSingleReg(L01REG_RX_PW_P3, 0x00);
    L01_WriteSingleReg(L01REG_RX_PW_P4, 0x00);
    L01_WriteSingleReg(L01REG_RX_PW_P5, 0x00);
    L01_WriteSingleReg(L01REG_FIFO_STATUS, 0x11);
    L01_WriteSingleReg(L01REG_DYNPD, 0x00);
    L01_WriteSingleReg(L01REG_FEATURE, 0x00);

    // Reset FIFO
    L01_FlushRX();
    L01_FlushTX();
}

/*!
 *  @brief        Initialize the nRF24L01+         
 *  @param        None
 *  @return       None  
 *  @note          
*/
uint8_t L01_Init(L01_config_t L01)
{

    L01_Reset();
    HAL_Delay(5);
    L01_SetCE(CE_LOW);
    L01_ClearIRQ(IRQ_ALL);
#if DYNAMIC_PACKET == 1
    //dynamic payload length
    L01_WriteSingleReg(L01REG_DYNPD,(1 << DPL_P0) /*| (1 << DPL_P1)*/);//Enable pipe 0 and pipe 1 dynamic payload length
    L01_WriteSingleReg(L01REG_FEATURE,(1 << EN_DPL) | (1 << EN_ACK_PAY)); //enable Dynamic payload and enable ack payload
#elif DYNAMIC_PACKET == 0
    //fixed payload length
    L01_WriteSingleReg(L01REG_RX_PW_P0,FIXED_PACKET_LEN);
#endif
    L01_WriteSingleReg(L01REG_CONFIG,(1 << EN_CRC | (1 << CRCO)));//Enable CRC,2 bytes
    L01_WriteSingleReg(L01REG_ENAA,(1 << ENAA_P0) /*| (1 << ENAA_P1)*/);//Auto Ack in pipe0 and pipe 1
    L01_WriteSingleReg(L01REG_EN_RXADDR,(1 << ERX_P0) /*| (1 << ERX_P1)*/);//Enable RX pipe 0
    L01_WriteSingleReg(L01REG_SETUP_AW,AW_5BYTES);//Address width:5bytes
    L01_WriteSingleReg(L01REG_SETUP_RETR,ARD_4000US|ARC_15);//ARD:4000us,repeat time:15
    L01_SetDataRate(L01.drate);
    L01_WriteHoppingPoint(L01.channel);
    L01_SetPower(L01.pwr);
    L01_SetTXAddr(L01.tx_pipe_addr,5);//Set TX address
    L01_SetRXAddr(0,L01.rx_pipe0_addr,5);//Set RX pipe 0 address
//    L01_SetRXAddr(1,L01.rx_pipe1_addr,5);//Set RX pipe 1 address
    L01_ClearIRQ(IRQ_ALL);
    L01_FlushRX();
    L01_FlushTX();
    uint8_t config = L01_ReadSingleReg(L01REG_CONFIG);//Read the config register and check if there is any issue should return 0x0C only EN_CRC and CRC0 bits set
    char txbuff[20];
    uint16_t len = sprintf(txbuff, "Config = %d\r\n", config);
    HAL_UART_Transmit(&huart2, (uint8_t*)txbuff, len, 100);
    if (config != 0x0C) {
    	return 1;
    }
    return 0;
}
