#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "main.h"

typedef enum
{
    ENC_PM1 = 0,
    ENC_PM2 = 1,
    ENC_MAX_NUM
} Encoder_ID_TypeDef;

void Encoder_Init(Encoder_ID_TypeDef enc_id, TIM_HandleTypeDef *htim);
void Encoder_Update(void);
void Encoder_ReadPulse(Encoder_ID_TypeDef enc_id, int *pulse);

#endif /* __ENCODER_H__ */
