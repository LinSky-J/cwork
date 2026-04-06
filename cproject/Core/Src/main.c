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
  int16_t encoder_pm1;
} DebugStatus_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG_MODE_NAME "ENC1"

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
char uart_msg[64];
uint8_t oled_buffer[8][128];
uint8_t oled_i2c_addr_write = 0x78;
bool oled_online = false;
DebugStatus_t g_debug_status = {0};

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
static int16_t ENCODER_GetPM1Count(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define OLED_I2C_ADDR_3C_WRITE 0x78
#define OLED_I2C_ADDR_3D_WRITE 0x7A
#define OLED_COLUMN_OFFSET 2U

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

static void UART2_SendString(const char *str)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

static int16_t ENCODER_GetPM1Count(void)
{
  return (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
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
  uint16_t shown;
  uint8_t d0;
  uint8_t d1;
  uint8_t d2;
  uint8_t d3;
  uint8_t progress_width;
  int16_t enc_value;

  if (!oled_online)
  {
    return;
  }

  enc_value = status->encoder_pm1;
  shown = (uint16_t)((enc_value >= 0) ? enc_value : -enc_value);
  shown %= 10000U;
  d0 = (uint8_t)((shown / 1000U) % 10U);
  d1 = (uint8_t)((shown / 100U) % 10U);
  d2 = (uint8_t)((shown / 10U) % 10U);
  d3 = (uint8_t)(shown % 10U);
  progress_width = (uint8_t)(status->heartbeat % 120U);

  OLED_BufferClear();

  OLED_FillRect(4, 4, progress_width, 2, true);
  OLED_FillRect(4, 58, progress_width, 2, true);

  OLED_DrawDigit7Seg(4, 14, d0);
  OLED_DrawDigit7Seg(32, 14, d1);
  OLED_DrawDigit7Seg(60, 14, d2);
  OLED_DrawDigit7Seg(88, 14, d3);

  /* 编码器为负时，在左侧画一个减号 */
  if (enc_value < 0)
  {
    OLED_FillRect(0, 30, 8, 2, true);
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

  OLED_Refresh();
}

static void DEBUG_SendBootFrame(void)
{
  snprintf(
      uart_msg,
      sizeof(uart_msg),
      "BOOT|mode=%s|enc1=%d|uart=%u|oled=%u|addr=0x%02X|err=%u\r\n",
      DEBUG_MODE_NAME,
      g_debug_status.encoder_pm1,
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
      "STAT|mode=%s|hb=%lu|enc1=%d|uart=%u|oled=%u|err=%u\r\n",
      DEBUG_MODE_NAME,
      status->heartbeat,
      status->encoder_pm1,
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
      "obs:%lu,%d,%u,%u,%u\n",
      status->heartbeat,
      status->encoder_pm1,
      status->uart_online,
      status->oled_online_flag,
      status->error_flag);
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
  /* USER CODE BEGIN 2 */
  g_debug_status.heartbeat = 0;
  g_debug_status.error_flag = 0;
  g_debug_status.uart_online = 1;
  g_debug_status.encoder_pm1 = 0;

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

  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  OLED_DrawStatus(&g_debug_status);
  DEBUG_SendBootFrame();
  VOFA_SendFireWaterFrame(&g_debug_status);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    g_debug_status.heartbeat = uart_heartbeat;
    g_debug_status.encoder_pm1 = ENCODER_GetPM1Count();
    DEBUG_SendStatusFrame(&g_debug_status);
    VOFA_SendFireWaterFrame(&g_debug_status);
    OLED_DrawStatus(&g_debug_status);
    uart_heartbeat++;
    HAL_Delay(1000);
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
