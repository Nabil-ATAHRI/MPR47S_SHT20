#ifndef MPR47S_H
#define MPR47S_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef struct
{
    uint8_t id;

    float V1, V2, V3, V4;
    float VL12, VL23, VL31;

    float I1, I2, I3, IN;

    float Freq;

    float P1, P2, P3, P_Total;
    float Q1, Q2, Q3, Q_Total;
    float S1, S2, S3, S_Total;

    float PF1, PF2, PF3;

} MPR47S_t;

void MPR47S_Init(UART_HandleTypeDef *huart);
uint8_t MPR47S_Read(MPR47S_t *m);

#endif