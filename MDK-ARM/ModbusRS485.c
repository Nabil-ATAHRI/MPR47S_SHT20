#include "ModbusRS485.h"

static UART_HandleTypeDef *huart;
static uint8_t rx[256];

/* ================= INIT ================= */
void Modbus_Init(UART_HandleTypeDef *uart)
{
    huart = uart;
}

/* ================= CRC16 ================= */
static uint16_t CRC16(uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for(int i=0;i<len;i++)
    {
        crc ^= buf[i];
        for(int j=0;j<8;j++)
            crc = (crc & 1) ? (crc>>1)^0xA001 : (crc>>1);
    }
    return crc;
}

/* ================= LOW LEVEL READ ================= */
static ModbusStatus_t Modbus_Read(uint8_t id,
                                  uint16_t reg,
                                  uint16_t size,
                                  uint8_t func,
                                  uint16_t *data)
{
    uint8_t f[8];

    f[0]=id;
    f[1]=func;
    f[2]=reg>>8;
    f[3]=reg;
    f[4]=size>>8;
    f[5]=size;

    uint16_t crc = CRC16(f,6);
    f[6]=crc;
    f[7]=crc>>8;

    HAL_UART_Transmit(huart,f,8,100);

    uint16_t len = 5 + 2*size;

    if(HAL_UART_Receive(huart,rx,len,800)!=HAL_OK)
        return MODBUS_ERR_TIMEOUT;

    for(int i=0;i<size;i++)
        data[i]=(rx[3+i*2]<<8)|rx[4+i*2];

    return MODBUS_OK;
}

/* ================= RETRY WRAPPER ================= */
static ModbusStatus_t Modbus_ReadRetry(uint8_t id,
                                        uint16_t reg,
                                        uint16_t size,
                                        uint8_t func,
                                        uint16_t *data)
{
    for(int i=0;i<3;i++)
    {
        if(Modbus_Read(id,reg,size,func,data)==MODBUS_OK)
            return MODBUS_OK;

        HAL_Delay(30);
    }
    return MODBUS_ERR_TIMEOUT;
}

/* ================= PUBLIC ================= */
ModbusStatus_t Modbus_ReadHoldingRegisters(uint8_t s,uint16_t r,uint16_t n,uint16_t *d)
{
    return Modbus_ReadRetry(s,r,n,0x03,d);
}

ModbusStatus_t Modbus_ReadInputRegisters(uint8_t s,uint16_t r,uint16_t n,uint16_t *d)
{
    return Modbus_ReadRetry(s,r,n,0x04,d);
}