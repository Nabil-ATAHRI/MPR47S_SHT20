#include "MPR47S.h"
#include "ModbusRS485.h"

/* ================= INIT ================= */
void MPR47S_Init(UART_HandleTypeDef *huart)
{
    Modbus_Init(huart);
}

/* ================= HELPERS ================= */

static float U32_TO_F(uint16_t *r, float scale)
{
    uint32_t v = ((uint32_t)r[0] << 16) | r[1];
    return (float)v * scale;
}

static float S32_TO_F(uint16_t *r, float scale)
{
    int32_t v = (int32_t)(((uint32_t)r[0] << 16) | r[1]);
    return (float)v * scale;
}

static float F32(uint16_t *r)
{
    union {
        uint32_t u;
        float f;
    } v;

    v.u = ((uint32_t)r[0] << 16) | r[1];

    if (v.f != v.f) return 0.0f; // NaN protection

    return v.f;
}

/* ================= READ ================= */

uint8_t MPR47S_Read(MPR47S_t *m)
{
    uint16_t r[170];

    if(Modbus_ReadHoldingRegisters(m->id, 0x0000, 80, r) != MODBUS_OK)
        return 0;

    /* ===== VOLTAGE ===== */
    m->V1 = U32_TO_F(&r[0x0000], 0.1f);
    m->V2 = U32_TO_F(&r[0x0002], 0.1f);
    m->V3 = U32_TO_F(&r[0x0004], 0.1f);
    m->V4 = U32_TO_F(&r[0x0006], 0.1f);

    m->VL12 = U32_TO_F(&r[0x0008], 0.1f);
    m->VL23 = U32_TO_F(&r[0x000A], 0.1f);
    m->VL31 = U32_TO_F(&r[0x000C], 0.1f);

    /* ===== CURRENT ===== */
    m->I1 = U32_TO_F(&r[0x000E], 0.001f);
    m->I2 = U32_TO_F(&r[0x0010], 0.001f);
    m->I3 = U32_TO_F(&r[0x0012], 0.001f);
    m->IN = U32_TO_F(&r[0x0016], 0.001f);

    /* ===== FREQUENCY ===== */
    m->Freq = U32_TO_F(&r[0x0018], 0.01f);

    /* ===== ACTIVE POWER ===== */
    m->P1 = F32(&r[0x001A]);
    m->P2 = F32(&r[0x001C]);
    m->P3 = F32(&r[0x001E]);
    m->P_Total = F32(&r[0x0026]);

    /* ===== REACTIVE POWER ===== */
    m->Q1 = F32(&r[0x0028]);
    m->Q2 = F32(&r[0x002A]);
    m->Q3 = F32(&r[0x002C]);
    m->Q_Total = F32(&r[0x0038]);

    /* ===== APPARENT POWER ===== */
    m->S1 = F32(&r[0x003A]);
    m->S2 = F32(&r[0x003C]);
    m->S3 = F32(&r[0x003E]);
    m->S_Total = F32(&r[0x0042]);

    /* ===== POWER FACTOR ===== */
    m->PF1 = S32_TO_F(&r[0x0048], 0.001f);
    m->PF2 = S32_TO_F(&r[0x004A], 0.001f);
    m->PF3 = S32_TO_F(&r[0x004C], 0.001f);

    return 1;
}