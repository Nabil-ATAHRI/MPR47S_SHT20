#include "EEM375.h"
#include "ModbusRS485.h"
#include <string.h>

/* ================= REGISTERS ================= */
static uint16_t regs_main[80];
static uint16_t regs_pf[20];
static uint16_t regs_thd[20];
static uint16_t regs_freq[2];
static uint16_t regs_vl[10];

/* ================= INIT ================= */
void EEM375_Init(UART_HandleTypeDef *huart)
{
    Modbus_Init(huart);
}

/* ================= FLOAT CONVERT ================= */
static float ToFloat(uint16_t *reg)
{
    union {
        uint32_t u;
        float f;
    } conv;

    conv.u = ((uint32_t)reg[0] << 16) | reg[1];
    return conv.f;
}

/* ================= RETRY FUNCTION ================= */
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

/* ================= MAIN READ ================= */
uint8_t EEM375_Read(EEM375_Data_t *d)
{
    if(!d) return 0;

    uint8_t id = d->id;

    /* ================= MAIN BLOCK ================= */
    if(!(Modbus_ReadRetry(id, 0x0000, 60, regs_main) &&
         Modbus_ReadRetry(id, 0x001E, 10, regs_pf) &&
         Modbus_ReadRetry(id, 0x00EA, 10, regs_thd)))
    {
        return 0;
    }

    /* ================= VOLTAGES ================= */
    d->V[0] = ToFloat(&regs_main[0]);
    d->V[1] = ToFloat(&regs_main[2]);
    d->V[2] = ToFloat(&regs_main[4]);

    /* ================= CURRENTS ================= */
    d->I[0] = ToFloat(&regs_main[6]);
    d->I[1] = ToFloat(&regs_main[8]);
    d->I[2] = ToFloat(&regs_main[10]);

    /* ================= ACTIVE POWER ================= */
    d->P[0] = ToFloat(&regs_main[12]);
    d->P[1] = ToFloat(&regs_main[14]);
    d->P[2] = ToFloat(&regs_main[16]);

    /* ================= APPARENT POWER ================= */
    d->S[0] = ToFloat(&regs_main[18]);
    d->S[1] = ToFloat(&regs_main[20]);
    d->S[2] = ToFloat(&regs_main[22]);

    /* ================= REACTIVE POWER ================= */
    d->Q[0] = ToFloat(&regs_main[24]);
    d->Q[1] = ToFloat(&regs_main[26]);
    d->Q[2] = ToFloat(&regs_main[28]);

    /* ================= LINE VOLTAGE (CORRECT ADDRESSES) ================= */
    if(Modbus_ReadRetry(id, 0x00C8, 8, regs_vl))
    {
        d->VL[0] = ToFloat(&regs_vl[0]); // L1-L2 (30201)
        d->VL[1] = ToFloat(&regs_vl[2]); // L2-L3 (30203)
        d->VL[2] = ToFloat(&regs_vl[4]); // L3-L1 (30205)
        d->VL_avg = ToFloat(&regs_vl[6]); // Avg L-L (30207)
    }
    else
    {
        d->VL[0] = d->VL[1] = d->VL[2] = 0;
        d->VL_avg = 0;
    }

    /* ================= POWER FACTOR ================= */
    d->PF[0] = ToFloat(&regs_pf[0]);
    d->PF[1] = ToFloat(&regs_pf[2]);
    d->PF[2] = ToFloat(&regs_pf[4]);

    d->PF_avg = (d->PF[0] + d->PF[1] + d->PF[2]) / 3.0f;
    d->PF_total_sys = d->PF_avg;

    /* ================= THD ================= */
    d->THD_V[0] = ToFloat(&regs_thd[0]);
    d->THD_V[1] = ToFloat(&regs_thd[2]);
    d->THD_V[2] = ToFloat(&regs_thd[4]);

    d->THD_I[0] = ToFloat(&regs_thd[6]);
    d->THD_I[1] = ToFloat(&regs_thd[8]);
    d->THD_I[2] = ToFloat(&regs_thd[10]);

    /* ================= FREQUENCY ================= */
    if(Modbus_ReadRetry(id, 0x0046, 2, regs_freq))
        d->freq = ToFloat(&regs_freq[0]);
    else
        d->freq = 0;

    /* ================= TOTAL POWER ================= */
    d->P_total_sys = d->P[0] + d->P[1] + d->P[2];
    d->S_total_sys = d->S[0] + d->S[1] + d->S[2];
    d->Q_total_sys = d->Q[0] + d->Q[1] + d->Q[2];

    return 1;
}