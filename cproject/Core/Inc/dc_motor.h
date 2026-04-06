#ifndef __DC_MOTOR_H__
#define __DC_MOTOR_H__

#include "main.h"

typedef enum
{
    MOTOR_PM1 = 0,
    MOTOR_PM2 = 1
} DC_Motor_ID_TypeDef;

/* Keep this in sync with TIM1 ARR. Current project uses 2599. */
#define DC_MOTOR_MAX_DUTY 2599

void DC_Motor_Init(DC_Motor_ID_TypeDef motor_id, TIM_HandleTypeDef *htim, uint32_t channel);
void DC_Motor_SetDuty(DC_Motor_ID_TypeDef motor_id, int16_t duty);
void DC_Motor_Brake(DC_Motor_ID_TypeDef motor_id);
void DC_Motor_Stop(DC_Motor_ID_TypeDef motor_id);
void DC_Motor_GetDuty(DC_Motor_ID_TypeDef motor_id, float *duty_percent);

#endif /* __DC_MOTOR_H__ */
