#ifndef EEM375_H
#define EEM375_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef struct
{
    uint8_t id;

    /* Voltages */
    float V[3];

    /* Line Voltage */
    float VL[3];
    float VL_avg;

    /* Currents */
    float I[3];

    /* Power */
    float P[3];
    float P_total_sys;

    float Q[3];
    float Q_total_sys;

    float S[3];
    float S_total_sys;

    /* Power Factor */
    float PF[3];
    float PF_avg;
    float PF_total_sys;

    /* THD */
    float THD_V[3];
    float THD_I[3];
    float THD_VLL_avg;
    float THD_I_avg;

    /* Frequency */
    float freq;

} EEM375_Data_t;

/* API */
void EEM375_Init(UART_HandleTypeDef *huart);
uint8_t EEM375_Read(EEM375_Data_t *d);

#endif