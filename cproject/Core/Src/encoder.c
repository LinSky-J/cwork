#include "encoder.h"

typedef struct
{
    TIM_HandleTypeDef *htim;
    int current_speed;
    uint8_t is_initialized;
} Encoder_Info_t;

static Encoder_Info_t g_encoder_resources[ENC_MAX_NUM];
static uint32_t g_encoder_tick = 0;

void Encoder_Init(Encoder_ID_TypeDef enc_id, TIM_HandleTypeDef *htim)
{
    Encoder_Info_t *enc;

    if (enc_id >= ENC_MAX_NUM || htim == 0)
    {
        return;
    }

    enc = &g_encoder_resources[enc_id];
    enc->htim = htim;
    enc->current_speed = 0;
    enc->is_initialized = 1;

    HAL_TIM_Encoder_Start(enc->htim, TIM_CHANNEL_ALL);
    __HAL_TIM_SetCounter(enc->htim, 0);
}

void Encoder_Update(void)
{
    uint8_t i;

    if ((HAL_GetTick() - g_encoder_tick) < 10U)
    {
        return;
    }

    g_encoder_tick = HAL_GetTick();

    for (i = 0; i < ENC_MAX_NUM; i++)
    {
        Encoder_Info_t *enc = &g_encoder_resources[i];

        if (enc->is_initialized != 0U)
        {
            enc->current_speed = (short)__HAL_TIM_GetCounter(enc->htim);
            __HAL_TIM_SetCounter(enc->htim, 0);
        }
    }
}

void Encoder_ReadPulse(Encoder_ID_TypeDef enc_id, int *pulse)
{
    if (pulse == 0 || enc_id >= ENC_MAX_NUM)
    {
        return;
    }

    *pulse = g_encoder_resources[enc_id].current_speed;
}
