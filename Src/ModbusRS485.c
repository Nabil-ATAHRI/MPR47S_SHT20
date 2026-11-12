#include "ModbusRS485.h"
#include <string.h>

/* ================= UART HANDLE ================= */
static UART_HandleTypeDef *ModbusUart;

/* ================= RX BUFFER ================= */
static uint8_t rx_buffer[256];

/* ================= INIT ================= */
void Modbus_Init(UART_HandleTypeDef *huart)
{
    ModbusUart = huart;
}

/* ================= CRC16 ================= */
static uint16_t Modbus_CRC16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for(uint16_t i = 0; i < len; i++)
    {
        crc ^= buf[i];

        for(uint8_t j = 0; j < 8; j++)
        {
            if(crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/* ================= SEND FRAME ================= */
/* ================= GENERIC READ ================= */
static ModbusStatus_t Modbus_Read(uint8_t slave,
                                   uint16_t reg,
                                   uint16_t size,
                                   uint8_t func,
                                   uint16_t *data)
{
    uint8_t frame[8];
    if (ModbusUart == NULL || data == NULL || slave == 0 || slave > 247 ||
        size == 0 || size > 125)
        return MODBUS_ERR_FRAME;

    frame[0] = slave;
    frame[1] = func;
    frame[2] = reg >> 8;
    frame[3] = reg & 0xFF;
    frame[4] = size >> 8;
    frame[5] = size & 0xFF;

    uint16_t crc = Modbus_CRC16(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = crc >> 8;

    __HAL_UART_FLUSH_DRREGISTER(ModbusUart);
    if (HAL_UART_Transmit(ModbusUart, frame, 8, 100) != HAL_OK)
        return MODBUS_ERR_TIMEOUT;
    if(HAL_UART_Receive(ModbusUart, rx_buffer, 3, 500) != HAL_OK)
        return MODBUS_ERR_TIMEOUT;

    if(rx_buffer[0] != slave)
        return MODBUS_ERR_SLAVE;
    if (rx_buffer[1] == (uint8_t)(func | 0x80))
    {
        if (HAL_UART_Receive(ModbusUart, &rx_buffer[3], 2, 500) != HAL_OK)
            return MODBUS_ERR_TIMEOUT;
        uint16_t exception_crc = (rx_buffer[4] << 8) | rx_buffer[3];
        return Modbus_CRC16(rx_buffer, 3) == exception_crc ?
            MODBUS_ERR_EXCEPTION : MODBUS_ERR_CRC;
    }
    if (rx_buffer[1] != func || rx_buffer[2] != size * 2)
        return MODBUS_ERR_FRAME;
    uint16_t expected = 5 + 2 * size;
    if (HAL_UART_Receive(ModbusUart, &rx_buffer[3], expected - 3, 500) != HAL_OK)
        return MODBUS_ERR_TIMEOUT;

    uint16_t crc_rx = (rx_buffer[expected-1] << 8) | rx_buffer[expected-2];
    uint16_t crc_calc = Modbus_CRC16(rx_buffer, expected-2);

    if(crc_rx != crc_calc)
        return MODBUS_ERR_CRC;

    for(uint16_t i = 0; i < size; i++)
    {
        data[i] = (rx_buffer[3 + i*2] << 8) | rx_buffer[4 + i*2];
    }

    return MODBUS_OK;
}

/* ================= PUBLIC FUNCTIONS ================= */

ModbusStatus_t Modbus_ReadHoldingRegisters(uint8_t slave, uint16_t reg, uint16_t size, uint16_t *data)
{
    return Modbus_Read(slave, reg, size, 0x03, data);
}

ModbusStatus_t Modbus_ReadInputRegisters(uint8_t slave, uint16_t reg, uint16_t size, uint16_t *data)
{
    return Modbus_Read(slave, reg, size, 0x04, data);
}