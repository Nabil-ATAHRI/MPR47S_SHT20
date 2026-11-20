#include "sht20.h"
#include "ModbusRS485.h"

static ModbusStatus_t SHT20_ReadReg(uint8_t id, uint16_t addr, uint16_t *buf)
{
    ModbusStatus_t status = MODBUS_ERR_TIMEOUT;

    for(int i=0;i<3;i++)
    {
        status = Modbus_ReadInputRegisters(id, addr, 2, buf);
        if(status == MODBUS_OK)
            return status;

        HAL_Delay(20);
    }
    return status;
}

void SHT20_Init(void)
{
    // rien
}

uint8_t SHT20_Read(SHT20_t *dev)
{
    uint16_t reg[2];
    ModbusStatus_t status = SHT20_ReadReg(dev->addr, 0x0001, reg);
    dev->last_status = (int)status;

    if(status != MODBUS_OK)
    {
        dev->ok = 0;
        return 0;
    }

    /* conversion s�curis�e */
    dev->temp = (int16_t)reg[0] / 10.0f;
    dev->hum  = (int16_t)reg[1] / 10.0f;

    if (dev->temp < -40.0f || dev->temp > 125.0f ||
        dev->hum < 0.0f || dev->hum > 100.0f)
    {
        dev->ok = 0;
        return 0;
    }
    dev->ok = 1;
    return 1;
}