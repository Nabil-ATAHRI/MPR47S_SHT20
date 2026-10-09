#include "SHT20.h"
#include "ModbusRS485.h"

static uint8_t SHT20_ReadReg(uint8_t id, uint16_t addr, uint16_t *buf)
{
    for(int i=0;i<3;i++)
    {
        if(Modbus_ReadInputRegisters(id, addr, 2, buf) == MODBUS_OK)
            return 1;

        HAL_Delay(20);
    }
    return 0;
}

void SHT20_Init(void)
{
    // rien
}

uint8_t SHT20_Read(SHT20_t *dev)
{
    uint16_t reg[2];

    if(!SHT20_ReadReg(dev->addr, 0x0001, reg))
    {
        dev->ok = 0;
        return 0;
    }

    /* conversion sécurisée */
    dev->temp = (int16_t)reg[0] / 10.0f;
    dev->hum  = (int16_t)reg[1] / 10.0f;

    dev->ok = 1;
    return 1;
}