#ifndef SHT20_H
#define SHT20_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef struct
{
    uint8_t addr;
    float temp;
    float hum;
    uint8_t ok;
} SHT20_t;

void SHT20_Init(void);
uint8_t SHT20_Read(SHT20_t *dev);

#endif