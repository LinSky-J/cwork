#include "dc_motor.h"

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    GPIO_TypeDef *in1_port;
    uint16_t in1_pin;
    GPIO_TypeDef *in2_port;
    uint16_t in2_pin;
    int16_t current_duty;
} DC_Motor_Info_t;

static DC_Motor_Info_t g_motor_resources[2];

static void DC_Motor_MapPins(DC_Motor_ID_TypeDef motor_id, DC_Motor_Info_t *motor)
{
    if (motor_id == MOTOR_PM1)
    {
        motor->in1_port = PM1_AIN1_GPIO_Port;
        motor->in1_pin = PM1_AIN1_Pin;
        motor->in2_port = GPIOB;
        motor->in2_pin = GPIO_PIN_13;
    }
    else
    {
        motor->in1_port = PM1_BIN1_GPIO_Port;
        motor->in1_pin = PM1_BIN1_Pin;
        motor->in2_port = GPIOB;
        motor->in2_pin = GPIO_PIN_14;
    }
}

void DC_Motor_Init(DC_Motor_ID_TypeDef motor_id, TIM_HandleTypeDef *htim, uint32_t channel)
{
    DC_Motor_Info_t *motor = &g_motor_resources[motor_id];

    motor->htim = htim;
    motor->channel = channel;
    motor->current_duty = 0;
    DC_Motor_MapPins(motor_id, motor);

    HAL_TIM_PWM_Stop(motor->htim, motor->channel);
    HAL_TIMEx_PWMN_Stop(motor->htim, motor->channel);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0);
    DC_Motor_Brake(motor_id);
}

void DC_Motor_Brake(DC_Motor_ID_TypeDef motor_id)
{
    DC_Motor_Info_t *motor = &g_motor_resources[motor_id];

    HAL_TIM_PWM_Stop(motor->htim, motor->channel);
    HAL_TIMEx_PWMN_Stop(motor->htim, motor->channel);
    HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_SET);
}

void DC_Motor_Stop(DC_Motor_ID_TypeDef motor_id)
{
    DC_Motor_SetDuty(motor_id, 0);
}

void DC_Motor_SetDuty(DC_Motor_ID_TypeDef motor_id, int16_t duty)
{
    DC_Motor_Info_t *motor = &g_motor_resources[motor_id];

    if (duty > DC_MOTOR_MAX_DUTY)
    {
        duty = DC_MOTOR_MAX_DUTY;
    }
    if (duty < -DC_MOTOR_MAX_DUTY)
    {
        duty = -DC_MOTOR_MAX_DUTY;
    }

    motor->current_duty = duty;

    if (duty > 0)
    {
        HAL_TIMEx_PWMN_Stop(motor->htim, motor->channel);
        HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, duty);
        HAL_TIM_PWM_Start(motor->htim, motor->channel);
    }
    else if (duty < 0)
    {
        HAL_TIM_PWM_Stop(motor->htim, motor->channel);
        HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, -duty);
        HAL_TIMEx_PWMN_Start(motor->htim, motor->channel);
    }
    else
    {
        DC_Motor_Brake(motor_id);
    }
}

void DC_Motor_GetDuty(DC_Motor_ID_TypeDef motor_id, float *duty_percent)
{
    if (duty_percent == 0)
    {
        return;
    }

    *duty_percent = (g_motor_resources[motor_id].current_duty * 100.0f) / (float)DC_MOTOR_MAX_DUTY;
}
