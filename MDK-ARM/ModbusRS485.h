#ifndef MODBUS_RS485_H
#define MODBUS_RS485_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef enum {
    MODBUS_OK = 0,
    MODBUS_ERR_TIMEOUT = -1,
    MODBUS_ERR_SLAVE = -2,
    MODBUS_ERR_CRC = -3
} ModbusStatus_t;

void Modbus_Init(UART_HandleTypeDef *huart);

ModbusStatus_t Modbus_ReadHoldingRegisters(uint8_t slave, uint16_t reg, uint16_t size, uint16_t *data);
ModbusStatus_t Modbus_ReadInputRegisters(uint8_t slave, uint16_t reg, uint16_t size, uint16_t *data);

#endif