/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint32_t heartbeat;
  uint8_t error_flag;
  uint8_t uart_online;
  uint8_t oled_online_flag;
  uint8_t motor_mode;
  uint8_t gray_l2;
  uint8_t gray_l1;
  uint8_t gray_m;
  uint8_t gray_r1;
  uint8_t gray_r2;
  uint8_t gray_pattern;
  int8_t track_error;
  int16_t left_target;
  int16_t right_target;
  int16_t left_output;
  int16_t right_output;
  int16_t enc_left_delta;
  int16_t enc_right_delta;
} DebugStatus_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG_MODE_NAME "TRACE"
#define MOTOR_PWM_PERIOD 2599U
#define TRACE_BASE_SPEED 380U
#define TRACE_SEARCH_SPEED 220U
#define TRACE_RIGHT_BIAS 10U
#define CTRL_PERIOD_MS 10U
#define DEBUG_TX_DIV 10U
#define VOFA_TX_DIV 5U
#define OLED_TX_DIV 20U
#define UART_TX_TIMEOUT_MS 20U
#define POINT_NOTIFY_MS 120U
#define STOP_ON_LINE_TIMEOUT_MS 3000U
#define STOP_ON_LINE_STABLE_COUNT 8U
#define STRAIGHT_HOLD_MS 450U
#define STRAIGHT_HOLD_KP 2
#define STRAIGHT_HOLD_KI 1
#define STRAIGHT_HOLD_I_LIMIT 80
#define STRAIGHT_HOLD_CORR_LIMIT 25
#define ENCODER_LEFT_SIGN (-1)
#define ENCODER_RIGHT_SIGN (1)
#define ENC_MODEL_SHIFT 4U
#define ENC_PI_KP 3
#define ENC_PI_KI 1
#define ENC_PI_I_LIMIT 400
#define ENC_PI_COMP_LIMIT 300

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t uart_heartbeat = 0;
char uart_msg[256];
uint8_t oled_buffer[8][128];
uint8_t oled_i2c_addr_write = 0x78;
bool oled_online = false;
DebugStatus_t g_debug_status = {0};
int32_t motor_pi_i_left = 0;
int32_t motor_pi_i_right = 0;
int8_t motor_pi_sign_left = 0;
int8_t motor_pi_sign_right = 0;
int8_t motor_channel_state[3] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void OLED_Init(void);
static void OLED_DrawStatus(const DebugStatus_t *status);
static void DEBUG_SendBootFrame(void);
static void DEBUG_SendStatusFrame(const DebugStatus_t *status);
static void VOFA_SendFireWaterFrame(const DebugStatus_t *status);
static void MOTOR_SetChannelPair(uint32_t channel, int16_t duty);
static void MOTOR_SetPM1(int16_t duty);
static void MOTOR_SetPM2(int16_t duty);
static void MOTOR_StopAll(void);
static void chassis_stop(void);
static void chassis_set_open_loop(int16_t left_target, int16_t right_target);
static const char *MOTOR_GetModeName(uint8_t mode);
static void line_follow_slow(uint8_t pattern, int8_t error);
static void go_straight_open_loop(void);
static void search_line_left(void);
static void search_line_right(void);
static void point_notify(void);
static void stop_on_line(void);
static void go_straight_with_hold(void);
static uint8_t READ_GRAY_L2(void);
static uint8_t READ_GRAY_L1(void);
static uint8_t READ_GRAY_M(void);
static uint8_t READ_GRAY_R1(void);
static uint8_t READ_GRAY_R2(void);
static uint8_t GRAY_GetPattern(void);
static int8_t GRAY_CalcError(uint8_t pattern, int8_t *last_error);
static void line_follow_basic(uint8_t pattern, int8_t error);
static int16_t ENCODER_ReadDeltaAndReset(TIM_HandleTypeDef *htim);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define OLED_I2C_ADDR_3C_WRITE 0x78
#define OLED_I2C_ADDR_3D_WRITE 0x7A
#define OLED_COLUMN_OFFSET 2U

enum
{
  MOTOR_MODE_STOP = 0,
  MOTOR_MODE_TRACE,
  MOTOR_MODE_TRACE_SLOW,
  MOTOR_MODE_STRAIGHT,
  MOTOR_MODE_SEARCH_LEFT,
  MOTOR_MODE_SEARCH_RIGHT,
  MOTOR_MODE_LOST_STRAIGHT
};

static const uint8_t oled_digit_segments[10] = {
  0x3F, /* 0 */
  0x06, /* 1 */
  0x5B, /* 2 */
  0x4F, /* 3 */
  0x66, /* 4 */
  0x6D, /* 5 */
  0x7D, /* 6 */
  0x07, /* 7 */
  0x7F, /* 8 */
  0x6F  /* 9 */
};

static int16_t CLAMP_I16(int32_t value, int16_t min_value, int16_t max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return (int16_t)value;
}

static int32_t CLAMP_I32(int32_t value, int32_t min_value, int32_t max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

static int8_t SIGN_I16(int16_t value)
{
  if (value > 0)
  {
    return 1;
  }
  if (value < 0)
  {
    return -1;
  }
  return 0;
}

static void UART2_SendString(const char *str)
{
  if (HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), UART_TX_TIMEOUT_MS) == HAL_OK)
  {
    g_debug_status.uart_online = 1U;
  }
  else
  {
    g_debug_status.uart_online = 0U;
    g_debug_status.error_flag = 1U;
  }
}

static const char *MOTOR_GetModeName(uint8_t mode)
{
  switch (mode)
  {
    case MOTOR_MODE_TRACE: return "TRACE";
    case MOTOR_MODE_TRACE_SLOW: return "TRACE_SLOW";
    case MOTOR_MODE_STRAIGHT: return "STRAIGHT";
    case MOTOR_MODE_SEARCH_LEFT: return "SEARCH_L";
    case MOTOR_MODE_SEARCH_RIGHT: return "SEARCH_R";
    case MOTOR_MODE_LOST_STRAIGHT: return "LOST_STRAIGHT";
    default: return "STOP";
  }
}

static void MOTOR_SetChannelPair(uint32_t channel, int16_t duty)
{
  uint8_t channel_idx = 0;
  int8_t target_state = 0;
  uint16_t compare;

  if (channel == TIM_CHANNEL_1)
  {
    channel_idx = 1U;
  }
  else if (channel == TIM_CHANNEL_2)
  {
    channel_idx = 2U;
  }

  if (duty > (int16_t)MOTOR_PWM_PERIOD)
  {
    duty = (int16_t)MOTOR_PWM_PERIOD;
  }
  if (duty < -(int16_t)MOTOR_PWM_PERIOD)
  {
    duty = -(int16_t)MOTOR_PWM_PERIOD;
  }

  target_state = SIGN_I16(duty);

  if (duty == 0)
  {
    if (motor_channel_state[channel_idx] != 0)
    {
      HAL_TIM_PWM_Stop(&htim1, channel);
      HAL_TIMEx_PWMN_Stop(&htim1, channel);
      motor_channel_state[channel_idx] = 0;
    }
    __HAL_TIM_SET_COMPARE(&htim1, channel, 0);
    return;
  }

  compare = (uint16_t)((duty > 0) ? duty : -duty);

  if (motor_channel_state[channel_idx] != target_state)
  {
    if (target_state > 0)
    {
      HAL_TIMEx_PWMN_Stop(&htim1, channel);
      HAL_TIM_PWM_Start(&htim1, channel);
    }
    else
    {
      HAL_TIM_PWM_Stop(&htim1, channel);
      HAL_TIMEx_PWMN_Start(&htim1, channel);
    }
    motor_channel_state[channel_idx] = target_state;
  }

  __HAL_TIM_SET_COMPARE(&htim1, channel, compare);
}

static void MOTOR_SetPM1(int16_t duty)
{
  /* 左轮方向取反后，正值统一表示前进 */
  MOTOR_SetChannelPair(TIM_CHANNEL_1, (int16_t)(-duty));
}

static void MOTOR_SetPM2(int16_t duty)
{
  MOTOR_SetChannelPair(TIM_CHANNEL_2, duty);
}

static void MOTOR_StopAll(void)
{
  MOTOR_SetPM1(0);
  MOTOR_SetPM2(0);
}

static void chassis_stop(void)
{
  g_debug_status.left_target = 0;
  g_debug_status.right_target = 0;
  MOTOR_StopAll();
}

static void chassis_set_open_loop(int16_t left_target, int16_t right_target)
{
  g_debug_status.left_target = left_target;
  g_debug_status.right_target = right_target;
  g_debug_status.left_output = left_target;
  g_debug_status.right_output = right_target;
  MOTOR_SetPM1(left_target);
  MOTOR_SetPM2(right_target);
}

static void go_straight_open_loop(void)
{
  chassis_set_open_loop(
      (int16_t)(TRACE_BASE_SPEED + TRACE_RIGHT_BIAS),
      (int16_t)TRACE_BASE_SPEED);
}

static void search_line_left(void)
{
  g_debug_status.motor_mode = MOTOR_MODE_SEARCH_LEFT;
  chassis_set_open_loop(
      0,
      (int16_t)TRACE_SEARCH_SPEED);
}

static void search_line_right(void)
{
  g_debug_status.motor_mode = MOTOR_MODE_SEARCH_RIGHT;
  chassis_set_open_loop(
      (int16_t)TRACE_SEARCH_SPEED,
      0);
}

static void point_notify(void)
{
  HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
  HAL_GPIO_TogglePin(BEEP_GPIO_Port, BEEP_Pin);
  HAL_Delay(POINT_NOTIFY_MS);
  HAL_GPIO_TogglePin(BEEP_GPIO_Port, BEEP_Pin);
  HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}

static void go_straight_with_hold(void)
{
  uint32_t start_tick;
  int32_t hold_i = 0;
  int16_t base_left;
  int16_t base_right;

  base_left = (int16_t)(TRACE_BASE_SPEED + TRACE_RIGHT_BIAS);
  base_right = (int16_t)TRACE_BASE_SPEED;
  g_debug_status.motor_mode = MOTOR_MODE_STRAIGHT;
  start_tick = HAL_GetTick();

  while ((HAL_GetTick() - start_tick) < STRAIGHT_HOLD_MS)
    {
      int16_t diff;
      int16_t correction;

      g_debug_status.enc_left_delta = (int16_t)(ENCODER_LEFT_SIGN * ENCODER_ReadDeltaAndReset(&htim3));
      g_debug_status.enc_right_delta = (int16_t)(ENCODER_RIGHT_SIGN * ENCODER_ReadDeltaAndReset(&htim4));

      diff = (int16_t)(g_debug_status.enc_left_delta - g_debug_status.enc_right_delta);
    hold_i = CLAMP_I32(hold_i + diff, -(int32_t)STRAIGHT_HOLD_I_LIMIT, (int32_t)STRAIGHT_HOLD_I_LIMIT);
    correction = CLAMP_I16(
        (int32_t)(STRAIGHT_HOLD_KP * diff) + (int32_t)(STRAIGHT_HOLD_KI * hold_i),
        -(int16_t)STRAIGHT_HOLD_CORR_LIMIT,
        (int16_t)STRAIGHT_HOLD_CORR_LIMIT);

    chassis_set_open_loop(
        (int16_t)(base_left - correction),
        (int16_t)(base_right + correction));
    HAL_Delay(CTRL_PERIOD_MS);
  }
}

static void stop_on_line(void)
{
  uint16_t stable_count = 0;
  int8_t local_last_error = g_debug_status.track_error;
  uint32_t start_tick = HAL_GetTick();

  while ((HAL_GetTick() - start_tick) < STOP_ON_LINE_TIMEOUT_MS)
  {
    uint8_t pattern;

    g_debug_status.enc_left_delta = (int16_t)(ENCODER_LEFT_SIGN * ENCODER_ReadDeltaAndReset(&htim3));
    g_debug_status.enc_right_delta = (int16_t)(ENCODER_RIGHT_SIGN * ENCODER_ReadDeltaAndReset(&htim4));
    g_debug_status.gray_l2 = READ_GRAY_L2();
    g_debug_status.gray_l1 = READ_GRAY_L1();
    g_debug_status.gray_m = READ_GRAY_M();
    g_debug_status.gray_r1 = READ_GRAY_R1();
    g_debug_status.gray_r2 = READ_GRAY_R2();
    pattern = GRAY_GetPattern();
    g_debug_status.gray_pattern = pattern;
    g_debug_status.track_error = GRAY_CalcError(pattern, &local_last_error);

    line_follow_slow(pattern, g_debug_status.track_error);

    if ((pattern & 0x04U) != 0U)
    {
      stable_count++;
    }
    else
    {
      stable_count = 0;
    }

    if (stable_count >= STOP_ON_LINE_STABLE_COUNT)
    {
      break;
    }

    HAL_Delay(CTRL_PERIOD_MS);
  }

  g_debug_status.motor_mode = MOTOR_MODE_STOP;
  chassis_stop();
  point_notify();
}

static uint8_t READ_GRAY_L2(void)
{
  return (uint8_t)HAL_GPIO_ReadPin(L2_GPIO_Port, L2_Pin);
}

static uint8_t READ_GRAY_L1(void)
{
  return (uint8_t)HAL_GPIO_ReadPin(L1_GPIO_Port, L1_Pin);
}

static uint8_t READ_GRAY_M(void)
{
  return (uint8_t)HAL_GPIO_ReadPin(MI_GPIO_Port, MI_Pin);
}

static uint8_t READ_GRAY_R1(void)
{
  return (uint8_t)HAL_GPIO_ReadPin(R1_GPIO_Port, R1_Pin);
}

static uint8_t READ_GRAY_R2(void)
{
  return (uint8_t)HAL_GPIO_ReadPin(R2_GPIO_Port, R2_Pin);
}

static uint8_t GRAY_GetPattern(void)
{
  uint8_t pattern = 0;

  pattern |= (uint8_t)(READ_GRAY_L2() ? 0x10U : 0x00U);
  pattern |= (uint8_t)(READ_GRAY_L1() ? 0x08U : 0x00U);
  pattern |= (uint8_t)(READ_GRAY_M()  ? 0x04U : 0x00U);
  pattern |= (uint8_t)(READ_GRAY_R1() ? 0x02U : 0x00U);
  pattern |= (uint8_t)(READ_GRAY_R2() ? 0x01U : 0x00U);

  return pattern;
}

static int8_t GRAY_CalcError(uint8_t pattern, int8_t *last_error)
{
  int16_t numerator = 0;
  int16_t denominator = 0;
  uint8_t inner_pattern = (uint8_t)(pattern & 0x0EU);
  uint8_t edge_left  = (uint8_t)(pattern & 0x10U);
  uint8_t edge_right = (uint8_t)(pattern & 0x01U);

  /* L2/R2 单独压线（内三路全灭）时直接给最大误差，加快大偏差修正 */
  if ((edge_left != 0U) && (inner_pattern == 0U) && (edge_right == 0U))
  {
    *last_error = -4;
    return *last_error;
  }
  if ((edge_right != 0U) && (inner_pattern == 0U) && (edge_left == 0U))
  {
    *last_error = 4;
    return *last_error;
  }

  /* 正常修正使用 L1 / M / R1 */
  if ((inner_pattern & 0x08U) != 0U)
  {
    numerator += -2;
    denominator++;
  }
  if ((inner_pattern & 0x04U) != 0U)
  {
    denominator++;
  }
  if ((inner_pattern & 0x02U) != 0U)
  {
    numerator += 2;
    denominator++;
  }

  if (denominator == 0)
  {
    return *last_error;
  }

  *last_error = (int8_t)(numerator / denominator);
  return *last_error;
}

static void line_follow_basic(uint8_t pattern, int8_t error)
{
  int16_t left_spd  = (int16_t)(TRACE_BASE_SPEED + TRACE_RIGHT_BIAS);
  int16_t right_spd = (int16_t)TRACE_BASE_SPEED;
  int16_t correction = 0;
  uint8_t inner_pattern = (uint8_t)(pattern & 0x0EU);
  uint8_t edge_left  = (uint8_t)(pattern & 0x10U);
  uint8_t edge_right = (uint8_t)(pattern & 0x01U);

  /* 优先处理边缘兜底：L2 / R2 看到线时，不做正常修正 */
  if ((edge_left != 0U) && (inner_pattern == 0U))
  {
    search_line_left();
    return;
  }
  else if ((edge_right != 0U) && (inner_pattern == 0U))
  {
    search_line_right();
    return;
  }
  else if (inner_pattern == 0U)
  {
    g_debug_status.motor_mode = MOTOR_MODE_LOST_STRAIGHT;
    go_straight_open_loop();
    return;
  }
  else
  {
    g_debug_status.motor_mode = MOTOR_MODE_TRACE;

    /* 仅中间探头压线时，保持直行，不做修正 */
    if (inner_pattern == 0x04U)
    {
      correction = 0;
    }
    else if (error <= -3)
    {
      correction = 20;
    }
    else if (error == -2)
    {
      correction = 12;
    }
    else if (error == -1)
    {
      correction = 5;
    }
    else if (error >= 3)
    {
      correction = -20;
    }
    else if (error == 2)
    {
      correction = -12;
    }
    else if (error == 1)
    {
      correction = -5;
    }
    else
    {
      correction = 0;
    }

    left_spd  -= correction;
    right_spd += correction;
  }

  chassis_set_open_loop(left_spd, right_spd);
}

static void line_follow_slow(uint8_t pattern, int8_t error)
{
  int16_t left_spd  = (int16_t)(TRACE_BASE_SPEED / 2U + TRACE_RIGHT_BIAS / 2U);
  int16_t right_spd = (int16_t)(TRACE_BASE_SPEED / 2U);
  int16_t correction = 0;
  uint8_t inner_pattern = (uint8_t)(pattern & 0x0EU);
  uint8_t edge_left  = (uint8_t)(pattern & 0x10U);
  uint8_t edge_right = (uint8_t)(pattern & 0x01U);

  if ((edge_left != 0U) && (inner_pattern == 0U))
  {
    search_line_left();
    return;
  }
  else if ((edge_right != 0U) && (inner_pattern == 0U))
  {
    search_line_right();
    return;
  }
  else if (inner_pattern == 0U)
  {
    g_debug_status.motor_mode = MOTOR_MODE_LOST_STRAIGHT;
    chassis_set_open_loop(
        (int16_t)(TRACE_SEARCH_SPEED / 2U),
        (int16_t)(TRACE_SEARCH_SPEED / 2U));
    return;
  }

  g_debug_status.motor_mode = MOTOR_MODE_TRACE_SLOW;

  if (inner_pattern == 0x04U)
  {
    correction = 0;
  }
  else if (error <= -3)
  {
    correction = 8;
  }
  else if (error == -2)
  {
    correction = 5;
  }
  else if (error == -1)
  {
    correction = 2;
  }
  else if (error >= 3)
  {
    correction = -8;
  }
  else if (error == 2)
  {
    correction = -5;
  }
  else if (error == 1)
  {
    correction = -2;
  }

  left_spd  -= correction;
  right_spd += correction;
  chassis_set_open_loop(left_spd, right_spd);
}

static int16_t ENCODER_ReadDeltaAndReset(TIM_HandleTypeDef *htim)
{
  int16_t delta;

  delta = (int16_t)__HAL_TIM_GET_COUNTER(htim);
  __HAL_TIM_SET_COUNTER(htim, 0);

  return delta;
}

static void OLED_I2C_Delay(void)
{
  volatile uint16_t i;
  for (i = 0; i < 150; i++)
  {
    __NOP();
  }
}

static void OLED_SCL(GPIO_PinState state)
{
  HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, state);
}

static void OLED_SDA(GPIO_PinState state)
{
  HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin, state);
}

static void OLED_I2C_Start(void)
{
  OLED_SDA(GPIO_PIN_SET);
  OLED_SCL(GPIO_PIN_SET);
  OLED_I2C_Delay();
  OLED_SDA(GPIO_PIN_RESET);
  OLED_I2C_Delay();
  OLED_SCL(GPIO_PIN_RESET);
}

static void OLED_I2C_Stop(void)
{
  OLED_SDA(GPIO_PIN_RESET);
  OLED_SCL(GPIO_PIN_SET);
  OLED_I2C_Delay();
  OLED_SDA(GPIO_PIN_SET);
  OLED_I2C_Delay();
}

static bool OLED_I2C_WriteByteAck(uint8_t value)
{
  uint8_t bit;
  GPIO_PinState ack_state;

  for (bit = 0; bit < 8; bit++)
  {
    OLED_SCL(GPIO_PIN_RESET);
    OLED_SDA((value & 0x80U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    OLED_I2C_Delay();
    OLED_SCL(GPIO_PIN_SET);
    OLED_I2C_Delay();
    value <<= 1;
  }

  /* Release SDA and sample ACK (ACK = pulled low by slave). */
  OLED_SCL(GPIO_PIN_RESET);
  OLED_SDA(GPIO_PIN_SET);
  OLED_I2C_Delay();
  OLED_SCL(GPIO_PIN_SET);
  OLED_I2C_Delay();
  ack_state = HAL_GPIO_ReadPin(OLED_SDA_GPIO_Port, OLED_SDA_Pin);
  OLED_SCL(GPIO_PIN_RESET);
  OLED_I2C_Delay();

  return (ack_state == GPIO_PIN_RESET);
}

static bool OLED_ProbeAddress(uint8_t addr_write)
{
  bool ack;

  OLED_I2C_Start();
  ack = OLED_I2C_WriteByteAck(addr_write);
  OLED_I2C_Stop();

  return ack;
}

static bool OLED_DetectAddress(void)
{
  uint8_t tries;

  for (tries = 0; tries < 3; tries++)
  {
    if (OLED_ProbeAddress(OLED_I2C_ADDR_3C_WRITE))
    {
      oled_i2c_addr_write = OLED_I2C_ADDR_3C_WRITE;
      return true;
    }

    if (OLED_ProbeAddress(OLED_I2C_ADDR_3D_WRITE))
    {
      oled_i2c_addr_write = OLED_I2C_ADDR_3D_WRITE;
      return true;
    }

    HAL_Delay(2);
  }

  oled_i2c_addr_write = OLED_I2C_ADDR_3C_WRITE;
  return false;
}

static void OLED_WriteCommand(uint8_t cmd)
{
  OLED_I2C_Start();
  OLED_I2C_WriteByteAck(oled_i2c_addr_write);
  OLED_I2C_WriteByteAck(0x00);
  OLED_I2C_WriteByteAck(cmd);
  OLED_I2C_Stop();
}

static void OLED_WriteDataBurst(const uint8_t *data, uint16_t len)
{
  uint16_t i;

  OLED_I2C_Start();
  OLED_I2C_WriteByteAck(oled_i2c_addr_write);
  OLED_I2C_WriteByteAck(0x40);
  for (i = 0; i < len; i++)
  {
    OLED_I2C_WriteByteAck(data[i]);
  }
  OLED_I2C_Stop();
}

static void OLED_BufferClear(void)
{
  uint8_t page;
  uint8_t col;

  for (page = 0; page < 8; page++)
  {
    for (col = 0; col < 128; col++)
    {
      oled_buffer[page][col] = 0x00;
    }
  }
}

static void OLED_Refresh(void)
{
  uint8_t page;
  uint8_t column_low;
  uint8_t column_high;

  for (page = 0; page < 8; page++)
  {
    column_low = (uint8_t)(OLED_COLUMN_OFFSET & 0x0F);
    column_high = (uint8_t)(0x10 + ((OLED_COLUMN_OFFSET >> 4) & 0x0F));
    OLED_WriteCommand(0xB0 + page);
    OLED_WriteCommand(column_low);
    OLED_WriteCommand(column_high);
    OLED_WriteDataBurst(oled_buffer[page], 128);
  }
}

static void OLED_DisplayAllOn(bool enable)
{
  OLED_WriteCommand(enable ? 0xA5 : 0xA4);
}

static void OLED_DrawPixel(uint8_t x, uint8_t y, bool on)
{
  uint8_t page;
  uint8_t bit_mask;

  if (x >= 128 || y >= 64)
  {
    return;
  }

  page = y / 8;
  bit_mask = (uint8_t)(1U << (y % 8U));

  if (on)
  {
    oled_buffer[page][x] |= bit_mask;
  }
  else
  {
    oled_buffer[page][x] &= (uint8_t)(~bit_mask);
  }
}

static void OLED_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool on)
{
  uint8_t px;
  uint8_t py;

  for (py = y; py < (uint8_t)(y + h) && py < 64; py++)
  {
    for (px = x; px < (uint8_t)(x + w) && px < 128; px++)
    {
      OLED_DrawPixel(px, py, on);
    }
  }
}

static void OLED_DrawDigit7Seg(uint8_t x, uint8_t y, uint8_t digit)
{
  uint8_t seg;

  if (digit > 9U)
  {
    return;
  }

  seg = oled_digit_segments[digit];

  if (seg & 0x01U) OLED_FillRect((uint8_t)(x + 3), y, 14, 3, true);               /* a */
  if (seg & 0x02U) OLED_FillRect((uint8_t)(x + 17), (uint8_t)(y + 3), 3, 13, true);/* b */
  if (seg & 0x04U) OLED_FillRect((uint8_t)(x + 17), (uint8_t)(y + 20), 3, 13, true);/* c */
  if (seg & 0x08U) OLED_FillRect((uint8_t)(x + 3), (uint8_t)(y + 33), 14, 3, true);/* d */
  if (seg & 0x10U) OLED_FillRect(x, (uint8_t)(y + 20), 3, 13, true);                /* e */
  if (seg & 0x20U) OLED_FillRect(x, (uint8_t)(y + 3), 3, 13, true);                 /* f */
  if (seg & 0x40U) OLED_FillRect((uint8_t)(x + 3), (uint8_t)(y + 16), 14, 3, true);/* g */
}

static void OLED_DrawStatus(const DebugStatus_t *status)
{
  uint8_t d0;
  uint8_t d1;
  uint8_t d2;
  uint8_t d3;
  uint8_t d4;
  uint8_t progress_width;
  uint16_t shown_left;
  uint16_t shown_right;
  uint8_t t0;
  uint8_t t1;
  uint8_t t2;
  uint8_t t3;

  if (!oled_online)
  {
    return;
  }

  progress_width = (uint8_t)(status->heartbeat % 120U);

  OLED_BufferClear();

  OLED_FillRect(4, 4, progress_width, 2, true);
  OLED_FillRect(4, 58, progress_width, 2, true);

  if ((status->heartbeat & 0x01U) == 0U)
  {
    d0 = status->gray_l2 ? 1U : 0U;
    d1 = status->gray_l1 ? 1U : 0U;
    d2 = status->gray_m ? 1U : 0U;
    d3 = status->gray_r1 ? 1U : 0U;
    d4 = status->gray_r2 ? 1U : 0U;

    OLED_DrawDigit7Seg(0, 14, d0);
    OLED_DrawDigit7Seg(24, 14, d1);
    OLED_DrawDigit7Seg(48, 14, d2);
    OLED_DrawDigit7Seg(72, 14, d3);
    OLED_DrawDigit7Seg(96, 14, d4);
  }
  else
  {
    int16_t abs_left = (status->left_target >= 0) ? status->left_target : -status->left_target;
    int16_t abs_right = (status->right_target >= 0) ? status->right_target : -status->right_target;
    shown_left = (uint16_t)abs_left;
    shown_right = (uint16_t)abs_right;
    shown_left /= 10U;
    shown_right /= 10U;
    shown_left %= 100U;
    shown_right %= 100U;
    t0 = (uint8_t)((shown_left / 10U) % 10U);
    t1 = (uint8_t)(shown_left % 10U);
    t2 = (uint8_t)((shown_right / 10U) % 10U);
    t3 = (uint8_t)(shown_right % 10U);

    OLED_DrawDigit7Seg(12, 14, t0);
    OLED_DrawDigit7Seg(40, 14, t1);
    OLED_DrawDigit7Seg(68, 14, t2);
    OLED_DrawDigit7Seg(96, 14, t3);

    if (status->left_target < 0)
    {
      OLED_FillRect(0, 30, 8, 2, true);
    }
    if (status->right_target < 0)
    {
      OLED_FillRect(56, 30, 8, 2, true);
    }
  }

  /* 右下角心跳点 */
  if ((status->heartbeat & 0x01U) != 0U)
  {
    OLED_FillRect(62, 52, 4, 4, true);
  }

  /* 左上角 3 个状态点：UART / OLED / ERR */
  if (status->uart_online != 0U)
  {
    OLED_FillRect(118, 2, 3, 3, true);
  }

  if (status->oled_online_flag != 0U)
  {
    OLED_FillRect(122, 2, 3, 3, true);
  }

  if (status->error_flag != 0U)
  {
    OLED_FillRect(126, 2, 2, 3, true);
  }

  if (status->motor_mode != 0U)
  {
    OLED_FillRect(110, 2, 4, 4, true);
  }

  OLED_Refresh();
}

static void DEBUG_SendBootFrame(void)
{
  snprintf(
      uart_msg,
      sizeof(uart_msg),
      "BOOT|mode=%s|step=%s|left=%d|right=%d|l2=%u|l1=%u|m=%u|r1=%u|r2=%u|uart=%u|oled=%u|addr=0x%02X|err=%u\r\n",
      DEBUG_MODE_NAME,
      MOTOR_GetModeName(g_debug_status.motor_mode),
      g_debug_status.left_target,
      g_debug_status.right_target,
      g_debug_status.gray_l2,
      g_debug_status.gray_l1,
      g_debug_status.gray_m,
      g_debug_status.gray_r1,
      g_debug_status.gray_r2,
      g_debug_status.uart_online,
      g_debug_status.oled_online_flag,
      (unsigned int)(oled_i2c_addr_write >> 1),
      g_debug_status.error_flag);
  UART2_SendString(uart_msg);
}

static void DEBUG_SendStatusFrame(const DebugStatus_t *status)
{
  snprintf(
      uart_msg,
      sizeof(uart_msg),
      "OPEN|mode=%s|hb=%lu|step=%s|left=%d|right=%d|outL=%d|outR=%d|encL=%d|encR=%d|l2=%u|l1=%u|m=%u|r1=%u|r2=%u|uart=%u|oled=%u|err=%u\r\n",
      DEBUG_MODE_NAME,
      status->heartbeat,
      MOTOR_GetModeName(status->motor_mode),
      status->left_target,
      status->right_target,
      status->left_output,
      status->right_output,
      status->enc_left_delta,
      status->enc_right_delta,
      status->gray_l2,
      status->gray_l1,
      status->gray_m,
      status->gray_r1,
      status->gray_r2,
      status->uart_online,
      status->oled_online_flag,
      status->error_flag);
  UART2_SendString(uart_msg);
}

static void VOFA_SendFireWaterFrame(const DebugStatus_t *status)
{
  snprintf(
      uart_msg,
      sizeof(uart_msg),
      "obs:%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,%u\n",
      status->left_target,
      status->right_target,
      status->left_output,
      status->right_output,
      status->enc_left_delta,
      status->enc_right_delta,
      status->gray_l2,
      status->gray_l1,
      status->gray_m,
      status->gray_r1,
      status->gray_r2);
  UART2_SendString(uart_msg);
}

static void OLED_Init(void)
{
  HAL_Delay(50);
  OLED_SDA(GPIO_PIN_SET);
  OLED_SCL(GPIO_PIN_SET);
  oled_online = OLED_DetectAddress();

  if (!oled_online)
  {
    return;
  }

  OLED_WriteCommand(0xAE);
  OLED_WriteCommand(0xD5);
  OLED_WriteCommand(0x80);
  OLED_WriteCommand(0xA8);
  OLED_WriteCommand(0x3F);
  OLED_WriteCommand(0xD3);
  OLED_WriteCommand(0x00);
  OLED_WriteCommand(0x40);
  OLED_WriteCommand(0x8D);
  OLED_WriteCommand(0x14);
  OLED_WriteCommand(0x20);
  OLED_WriteCommand(0x02);
  OLED_WriteCommand(0xA1);
  OLED_WriteCommand(0xC8);
  OLED_WriteCommand(0xDA);
  OLED_WriteCommand(0x12);
  OLED_WriteCommand(0x81);
  OLED_WriteCommand(0xCF);
  OLED_WriteCommand(0xD9);
  OLED_WriteCommand(0xF1);
  OLED_WriteCommand(0xDB);
  OLED_WriteCommand(0x40);
  OLED_WriteCommand(0xA4);
  OLED_WriteCommand(0xA6);
  OLED_WriteCommand(0xAF);

  /* 上电自检：整屏点亮一下，方便区分“初始化失败”和“绘图失败” */
  OLED_DisplayAllOn(true);
  HAL_Delay(300);
  OLED_DisplayAllOn(false);

  OLED_BufferClear();
  OLED_Refresh();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  /* USER CODE BEGIN 2 */
  g_debug_status.heartbeat = 0;
  g_debug_status.error_flag = 0;
  g_debug_status.uart_online = 1;
  g_debug_status.motor_mode = 0;
  g_debug_status.gray_l2 = 0;
  g_debug_status.gray_l1 = 0;
  g_debug_status.gray_m = 0;
  g_debug_status.gray_r1 = 0;
  g_debug_status.gray_r2 = 0;
  g_debug_status.gray_pattern = 0;
  g_debug_status.track_error = 0;
  g_debug_status.left_target = 0;
  g_debug_status.right_target = 0;
  g_debug_status.left_output = 0;
  g_debug_status.right_output = 0;
  g_debug_status.enc_left_delta = 0;
  g_debug_status.enc_right_delta = 0;

  UART2_SendString("System boot OK\r\n");
  UART2_SendString("USART2 ready, baud=115200\r\n");
  OLED_Init();
  g_debug_status.oled_online_flag = oled_online ? 1U : 0U;

  if (oled_online)
  {
    UART2_SendString("OLED ready\r\n");
  }
  else
  {
    g_debug_status.error_flag = 1U;
    UART2_SendString("OLED not found (0x3C/0x3D)\r\n");
  }

  g_debug_status.gray_l2 = READ_GRAY_L2();
  g_debug_status.gray_l1 = READ_GRAY_L1();
  g_debug_status.gray_m = READ_GRAY_M();
  g_debug_status.gray_r1 = READ_GRAY_R1();
  g_debug_status.gray_r2 = READ_GRAY_R2();
  MOTOR_StopAll();
  OLED_DrawStatus(&g_debug_status);
  DEBUG_SendBootFrame();
  VOFA_SendFireWaterFrame(&g_debug_status);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    static int8_t last_error = 0;
    static uint8_t debug_tx_div = 0;
    static uint8_t vofa_tx_div = 0;
    static uint8_t oled_tx_div = 0;
    uint8_t gray_pattern;
    int8_t track_error;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    g_debug_status.heartbeat = uart_heartbeat++;
  g_debug_status.heartbeat = uart_heartbeat++;
  g_debug_status.enc_left_delta = (int16_t)(ENCODER_LEFT_SIGN * ENCODER_ReadDeltaAndReset(&htim3));
  g_debug_status.enc_right_delta = (int16_t)(ENCODER_RIGHT_SIGN * ENCODER_ReadDeltaAndReset(&htim4));
  g_debug_status.gray_l2 = READ_GRAY_L2();
  g_debug_status.gray_l1 = READ_GRAY_L1();
  g_debug_status.gray_m = READ_GRAY_M();
  g_debug_status.gray_r1 = READ_GRAY_R1();
  g_debug_status.gray_r2 = READ_GRAY_R2();
  gray_pattern = GRAY_GetPattern();
  g_debug_status.gray_pattern = gray_pattern;
  track_error = GRAY_CalcError(gray_pattern, &last_error);
  g_debug_status.track_error = track_error;
  line_follow_basic(
      gray_pattern,
      track_error);

    debug_tx_div++;
    vofa_tx_div++;
    oled_tx_div++;

    if (debug_tx_div >= DEBUG_TX_DIV)
    {
      debug_tx_div = 0;
      DEBUG_SendStatusFrame(&g_debug_status);
    }
    if (vofa_tx_div >= VOFA_TX_DIV)
    {
      vofa_tx_div = 0;
      VOFA_SendFireWaterFrame(&g_debug_status);
    }
    if (oled_tx_div >= OLED_TX_DIV)
    {
      oled_tx_div = 0;
      OLED_DrawStatus(&g_debug_status);
    }
    HAL_Delay(CTRL_PERIOD_MS);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2599;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0x0F;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OLED_SCL_Pin|OLED_SDA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED0_Pin */
  GPIO_InitStruct.Pin = LED0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY_IN_1_Pin KEY_IN_3_Pin KEY_IN_4_Pin KEY_IN_2_Pin */
  GPIO_InitStruct.Pin = KEY_IN_1_Pin|KEY_IN_3_Pin|KEY_IN_4_Pin|KEY_IN_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin BEEP_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|BEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MI_Pin L1_Pin L2_Pin R1_Pin
                           R2_Pin */
  GPIO_InitStruct.Pin = MI_Pin|L1_Pin|L2_Pin|R1_Pin
                          |R2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_EXTI1_Pin */
  GPIO_InitStruct.Pin = GPIO_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_EXTI1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
