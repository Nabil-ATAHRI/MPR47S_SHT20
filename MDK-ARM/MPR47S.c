#include "MPR47S.h"
#include "ModbusRS485.h"

/* ========================================================= */
/* INIT */
/* ========================================================= */

void MPR47S_Init(UART_HandleTypeDef *huart)
{
    Modbus_Init(huart);
}

/* ========================================================= */
/* HELPERS */
/* ========================================================= */

static float U32_To_Float(uint16_t *r, float scale)
{
    uint32_t value;

    value = ((uint32_t)r[0] << 16) | r[1];

    return ((float)value * scale);
}

static float S32_To_Float(uint16_t *r, float scale)
{
    int32_t value;

    value = (int32_t)(((uint32_t)r[0] << 16) | r[1]);

    return ((float)value * scale);
}

static float Float32_From_Reg(uint16_t *r)
{
    union
    {
        uint32_t u32;
        float f32;
    } data;

    data.u32 = ((uint32_t)r[0] << 16) | r[1];

    if(data.f32 != data.f32)
        return 0.0f;

    return data.f32;
}

/* ========================================================= */
/* READ */
/* ========================================================= */

uint8_t MPR47S_Read(MPR47S_t *m)
{
    uint16_t reg[100];

    if(Modbus_ReadHoldingRegisters(m->id,
                                   0x0000,
                                   100,
                                   reg) != MODBUS_OK)
    {
        return 0;
    }

    /* ---------------- Voltage ---------------- */

    m->V1 = U32_To_Float(&reg[0x00], 0.1f);
    m->V2 = U32_To_Float(&reg[0x02], 0.1f);
    m->V3 = U32_To_Float(&reg[0x04], 0.1f);
    m->VN = U32_To_Float(&reg[0x06], 0.1f);

    m->VL12 = U32_To_Float(&reg[0x08], 0.1f);
    m->VL23 = U32_To_Float(&reg[0x0A], 0.1f);
    m->VL31 = U32_To_Float(&reg[0x0C], 0.1f);

    /* ---------------- Current ---------------- */

    m->I1 = U32_To_Float(&reg[0x0E], 0.001f);
    m->I2 = U32_To_Float(&reg[0x10], 0.001f);
    m->I3 = U32_To_Float(&reg[0x12], 0.001f);

    m->IN = U32_To_Float(&reg[0x16], 0.001f);

    /* ---------------- Frequency ---------------- */

    m->Freq = U32_To_Float(&reg[0x18], 0.01f);

    /* ---------------- Active Power ---------------- */

    m->P1 = Float32_From_Reg(&reg[0x1A]);
    m->P2 = Float32_From_Reg(&reg[0x1C]);
    m->P3 = Float32_From_Reg(&reg[0x1E]);

    m->P_Total = Float32_From_Reg(&reg[0x26]);

    /* ---------------- Reactive Power ---------------- */

    m->Q1 = Float32_From_Reg(&reg[0x28]);
    m->Q2 = Float32_From_Reg(&reg[0x2A]);
    m->Q3 = Float32_From_Reg(&reg[0x2C]);

    m->Q_Total = Float32_From_Reg(&reg[0x38]);

    /* ---------------- Apparent Power ---------------- */

    m->S1 = Float32_From_Reg(&reg[0x3A]);
    m->S2 = Float32_From_Reg(&reg[0x3C]);
    m->S3 = Float32_From_Reg(&reg[0x3E]);

    m->S_Total = Float32_From_Reg(&reg[0x42]);

    /* ---------------- Power Factor ---------------- */

    m->PF1 = S32_To_Float(&reg[0x48], 0.001f);
    m->PF2 = S32_To_Float(&reg[0x4A], 0.001f);
    m->PF3 = S32_To_Float(&reg[0x4C], 0.001f);

    return 1;
}