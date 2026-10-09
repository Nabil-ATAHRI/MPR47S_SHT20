#include "EEM375.h"
#include "ModbusRS485.h"
#include <string.h>

/* ================= PRIVATE HANDLES ================= */
static UART_HandleTypeDef *modbus_uart;
static UART_HandleTypeDef *debug_uart;

/* ================= REGISTERS ================= */
static uint16_t regs_main[80];
static uint16_t regs_pf[20];
static uint16_t regs_thd[20];
static uint16_t regs_freq[2];

/* ================= INIT ================= */
void EEM375_Init(UART_HandleTypeDef *huart_modbus,
                 UART_HandleTypeDef *huart_debug)
{
    modbus_uart = huart_modbus;
    debug_uart  = huart_debug;

    Modbus_Init(modbus_uart);
}

/* ================= FLOAT CONVERSION ================= */
float EEM375_ToFloat(uint16_t *reg)
{
    union {
        uint32_t u;
        float f;
    } conv;

    conv.u = ((uint32_t)reg[0] << 16) | reg[1];
    return conv.f;
}

/* ================= MODBUS RETRY ================= */
static uint8_t Modbus_ReadRetry(uint8_t id,
                                uint16_t addr,
                                uint16_t size,
                                uint16_t *buf)
{
    for(int i = 0; i < 3; i++)
    {
        if(Modbus_ReadInputRegisters(id, addr, size, buf) == MODBUS_OK)
            return 1;

        HAL_Delay(50);
    }
    return 0;
}

/* ================= MAIN READ FUNCTION ================= */
uint8_t EEM375_Read(EEM375_Data_t *d)
{
    /* READ BLOCKS */
    if(!(Modbus_ReadRetry(1, 0x0000, 60, regs_main) &&
         Modbus_ReadRetry(1, 0x001E, 10, regs_pf) &&
         Modbus_ReadRetry(1, 0x00EA, 10, regs_thd)))
    {
        return 0;
    }

    /* ===== VOLTAGES ===== */
    d->V[0] = EEM375_ToFloat(&regs_main[0]);
    d->V[1] = EEM375_ToFloat(&regs_main[2]);
    d->V[2] = EEM375_ToFloat(&regs_main[4]);

    /* ===== CURRENTS ===== */
    d->I[0] = EEM375_ToFloat(&regs_main[6]);
    d->I[1] = EEM375_ToFloat(&regs_main[8]);
    d->I[2] = EEM375_ToFloat(&regs_main[10]);

    /* ===== POWER ===== */
    d->P[0] = EEM375_ToFloat(&regs_main[12]);
    d->P[1] = EEM375_ToFloat(&regs_main[14]);
    d->P[2] = EEM375_ToFloat(&regs_main[16]);

    d->Q[0] = EEM375_ToFloat(&regs_main[24]);
    d->Q[1] = EEM375_ToFloat(&regs_main[26]);
    d->Q[2] = EEM375_ToFloat(&regs_main[28]);

    d->S[0] = EEM375_ToFloat(&regs_main[18]);
    d->S[1] = EEM375_ToFloat(&regs_main[20]);
    d->S[2] = EEM375_ToFloat(&regs_main[22]);

    /* ===== POWER FACTOR ===== */
    d->PF[0] = EEM375_ToFloat(&regs_pf[0]);
    d->PF[1] = EEM375_ToFloat(&regs_pf[2]);
    d->PF[2] = EEM375_ToFloat(&regs_pf[4]);

    d->PF_avg = (d->PF[0] + d->PF[1] + d->PF[2]) / 3.0f;

    /* ===== FREQUENCY ===== */
    if(Modbus_ReadRetry(1, 0x0046, 2, regs_freq))
        d->freq = EEM375_ToFloat(&regs_freq[0]);
    else
        d->freq = 0;

    /* ===== THD ===== */
    d->THD_V[0] = EEM375_ToFloat(&regs_thd[0]);
    d->THD_V[1] = EEM375_ToFloat(&regs_thd[2]);
    d->THD_V[2] = EEM375_ToFloat(&regs_thd[4]);

    /* ===== TOTAL POWER ===== */
    d->P_total = d->P[0] + d->P[1] + d->P[2];

    return 1;
}