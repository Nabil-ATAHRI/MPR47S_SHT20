#ifndef EEM375_H
#define EEM375_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ================= DATA STRUCT ================= */
typedef struct {
    float V[3];
    float I[3];
    float P[3];
    float Q[3];
    float S[3];
    float PF[3];
    float PF_avg;
    float THD_V[3];
    float freq;
    float P_total;
} EEM375_Data_t;

/* ================= API ================= */
void EEM375_Init(UART_HandleTypeDef *huart_modbus,
                 UART_HandleTypeDef *huart_debug);

uint8_t EEM375_Read(EEM375_Data_t *data);

float EEM375_ToFloat(uint16_t *reg);

#endif