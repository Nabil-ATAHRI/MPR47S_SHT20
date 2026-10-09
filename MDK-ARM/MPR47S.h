#ifndef MPR47S_H
#define MPR47S_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef struct
{
    uint8_t id;

    /* Voltages */
    float V1;
    float V2;
    float V3;
    float VN;

    float VL12;
    float VL23;
    float VL31;

    /* Currents */
    float I1;
    float I2;
    float I3;
    float IN;

    /* Frequency */
    float Freq;

    /* Active Power */
    float P1;
    float P2;
    float P3;
    float P_Total;

    /* Reactive Power */
    float Q1;
    float Q2;
    float Q3;
    float Q_Total;

    /* Apparent Power */
    float S1;
    float S2;
    float S3;
    float S_Total;

    /* Power Factor */
    float PF1;
    float PF2;
    float PF3;

} MPR47S_t;

void MPR47S_Init(UART_HandleTypeDef *huart);
uint8_t MPR47S_Read(MPR47S_t *m);

#endif