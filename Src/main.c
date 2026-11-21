/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Smart Energy Monitor - STM32F407VGT6
  *
  * USART2 : Modbus RS485 sensors, 9600 8N1
  * USART3 : Diagnostic terminal, 9600 8N1
  * UART4 : JSON telemetry to Raspberry Pi, 115200 8N1
  *
  * IMPORTANT:
  * UART4 TX status OK means the DMA transmission completed successfully.
  * It does NOT prove that the Raspberry Pi received or parsed the frame.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ModbusRS485.h"
#include "MPR47S.h"
#include "EEM375.h"
#include "sht20.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TXBUF_SIZE       2048U
#define SHT_COUNT        2U
#define MPR_COUNT        1U
#define UART3_TX_TIMEOUT 1000U
#define UART4_TX_TIMEOUT  1000U
#define UART4_RX_BUFFER_SIZE 256U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;

/* USER CODE BEGIN PV */
static char txbuf[TXBUF_SIZE];
static char framebuf[TXBUF_SIZE];
static uint8_t uart4_rx_buffer[UART4_RX_BUFFER_SIZE];

static uint32_t telemetry_seq = 0U;

MPR47S_t mpr[MPR_COUNT];
uint8_t mpr_id[MPR_COUNT] = {5U};
uint8_t mpr_alive[MPR_COUNT] = {0U};

SHT20_t sht[SHT_COUNT];
uint8_t sht_id[SHT_COUNT] = {1U, 3U};
uint8_t sht_alive[SHT_COUNT] = {0U};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_UART4_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
static void UART_VPrintf(UART_HandleTypeDef *uart,
                         const char *fmt,
                         va_list args);
static HAL_StatusTypeDef UART_TransmitDMA(UART_HandleTypeDef *uart,
                                          uint8_t *data,
                                          uint16_t size,
                                          uint32_t timeout);
void UART_Printf(const char *fmt, ...);
static void Frame_Printf(const char *fmt, ...);
static const char *Modbus_StatusName(int status);

static void Raspberry_SendBootFrame(void);
static void Raspberry_SendErrorFrame(const char *sensor, uint8_t id);
static void Raspberry_SendSHTFrame(const SHT20_t *sensor, uint8_t read_ok);
static void Raspberry_SendMPRFrame(const MPR47S_t *sensor, uint8_t read_ok);

static void LED_On(uint16_t pin);
static void LED_Off(uint16_t pin);
static void LED_Blink(uint16_t pin, uint8_t n, uint16_t delay_ms);

static void Sensors_Init(void);
static void SHT_ReadAll(void);
static void MPR_ReadAll(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static const char *Modbus_StatusName(int status)
{
    switch (status)
    {
    case 0:
        return "OK";
    case -1:
        return "TIMEOUT";
    case -2:
        return "SLAVE";
    case -3:
        return "CRC";
    case -4:
        return "FRAME";
    case -5:
        return "EXCEPTION";
    case -6:
        return "TX_ERROR";
    default:
        return "UNKNOWN";
    }
}

static void UART_VPrintf(UART_HandleTypeDef *uart,
                         const char *fmt,
                         va_list args)
{
    int len = vsnprintf(txbuf, sizeof(txbuf), fmt, args);

    if ((len < 0) || (len >= (int)sizeof(txbuf)))
    {
        Error_Handler();
        return;
    }

    if ((len > 0) &&
        (HAL_UART_Transmit(uart, (uint8_t *)txbuf, (uint16_t)len,
                           UART3_TX_TIMEOUT) != HAL_OK))
    {
        Error_Handler();
    }
}

void UART_Printf(const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    UART_VPrintf(&huart3, fmt, args);
    va_end(args);
}

static HAL_StatusTypeDef UART_TransmitDMA(UART_HandleTypeDef *uart,
                                          uint8_t *data,
                                          uint16_t size,
                                          uint32_t timeout)
{
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(uart, data, size);
    uint32_t start_tick;

    if (status != HAL_OK)
    {
        return status;
    }

    start_tick = HAL_GetTick();
    while (uart->gState != HAL_UART_STATE_READY)
    {
        if ((HAL_GetTick() - start_tick) >= timeout)
        {
            (void)HAL_UART_AbortTransmit(uart);
            return HAL_TIMEOUT;
        }
    }

    return HAL_OK;
}

/* ================= UART4 JSON TRANSMISSION ================= */

static void Frame_Printf(const char *fmt, ...)
{
    va_list args;
    int len;
    uint16_t terminal_len;

    va_start(args, fmt);
    len = vsnprintf(framebuf, sizeof(framebuf), fmt, args);
    va_end(args);

    if ((len < 0) || (len >= (int)sizeof(framebuf)))
    {
        Error_Handler();
        return;
    }

    UART_Printf("UART4 TX (%d bytes):\r\n", len);
    terminal_len = (uint16_t)len;
    if ((terminal_len > 0U) && (framebuf[terminal_len - 1U] == '\n'))
    {
        terminal_len--;
    }
    if (terminal_len > 0U)
    {
        if (HAL_UART_Transmit(&huart3, (uint8_t *)framebuf,
                              terminal_len, UART3_TX_TIMEOUT) != HAL_OK)
        {
            Error_Handler();
        }
    }
    UART_Printf("\r\n");

    HAL_StatusTypeDef status =
        UART_TransmitDMA(&huart4,
                         (uint8_t *)framebuf,
                         (uint16_t)len,
                         UART4_TX_TIMEOUT);

    if (status != HAL_OK)
    {
        UART_Printf("UART4 DMA TX failed (HAL status=%d)\r\n", (int)status);
        Error_Handler();
    }
}

/* ================= JSON BOOT / ERROR FRAMES ================= */

static void Raspberry_SendBootFrame(void)
{
    Frame_Printf("{\"v\":1,\"event\":\"boot\"}\n");
}

static void Raspberry_SendErrorFrame(const char *sensor, uint8_t id)
{
    Frame_Printf(
        "{\"v\":1,\"sensor\":\"%s\",\"id\":%u,"
        "\"seq\":%lu,\"uptime_ms\":%lu,"
        "\"status\":\"error\","
        "\"error\":\"acquisition_failed\",\"data\":null}\n",
        sensor,
        (unsigned int)id,
        (unsigned long)telemetry_seq++,
        (unsigned long)HAL_GetTick());
}

/* ================= JSON SHT20 FRAME ================= */

static void Raspberry_SendSHTFrame(const SHT20_t *sensor,
                                   uint8_t read_ok)
{
    if (read_ok == 0U)
    {
        Raspberry_SendErrorFrame("sht20", sensor->addr);
        return;
    }

    Frame_Printf(
        "{\"v\":1,\"sensor\":\"sht20\",\"id\":%u,"
        "\"seq\":%lu,\"uptime_ms\":%lu,"
        "\"status\":\"ok\",\"error\":\"\","
        "\"data\":{\"temperature_c\":%.1f,"
        "\"humidity_pct\":%.1f}}\n",
        (unsigned int)sensor->addr,
        (unsigned long)telemetry_seq++,
        (unsigned long)HAL_GetTick(),
        (double)sensor->temp,
        (double)sensor->hum);
}

/* ================= JSON MPR47S FRAME ================= */

static void Raspberry_SendMPRFrame(const MPR47S_t *m,
                                   uint8_t read_ok)
{
    if (read_ok == 0U)
    {
        Raspberry_SendErrorFrame("mpr47s", m->id);
        return;
    }

    Frame_Printf(
        "{\"v\":1,\"sensor\":\"mpr47s\",\"id\":%u,"
        "\"seq\":%lu,\"uptime_ms\":%lu,"
        "\"status\":\"ok\",\"error\":\"\","
        "\"data\":{"
        "\"voltage_v\":[%.1f,%.1f,%.1f,%.1f],"
        "\"line_voltage_v\":[%.1f,%.1f,%.1f],"
        "\"current_a\":[%.3f,%.3f,%.3f,%.3f],"
        "\"active_power_w\":[%.3f,%.3f,%.3f,%.3f],"
        "\"reactive_power_var\":[%.3f,%.3f,%.3f,%.3f],"
        "\"apparent_power_va\":[%.3f,%.3f,%.3f,%.3f],"
        "\"power_factor\":[%.3f,%.3f,%.3f],"
        "\"frequency_hz\":%.2f}}\n",
        (unsigned int)m->id,
        (unsigned long)telemetry_seq++,
        (unsigned long)HAL_GetTick(),
        (double)m->V1, (double)m->V2,
        (double)m->V3, (double)m->V4,
        (double)m->VL12, (double)m->VL23, (double)m->VL31,
        (double)m->I1, (double)m->I2,
        (double)m->I3, (double)m->IN,
        (double)m->P1, (double)m->P2,
        (double)m->P3, (double)m->P_Total,
        (double)m->Q1, (double)m->Q2,
        (double)m->Q3, (double)m->Q_Total,
        (double)m->S1, (double)m->S2,
        (double)m->S3, (double)m->S_Total,
        (double)m->PF1, (double)m->PF2,
        (double)m->PF3, (double)m->Freq);
}

/* ================= LED CONTROL ================= */

#define LED_PORT     GPIOD
#define LED_SHT_PIN  GPIO_PIN_13
#define LED_MPR_PIN  GPIO_PIN_14

static void LED_On(uint16_t pin)
{
    HAL_GPIO_WritePin(LED_PORT, pin, GPIO_PIN_SET);
}

static void LED_Off(uint16_t pin)
{
    HAL_GPIO_WritePin(LED_PORT, pin, GPIO_PIN_RESET);
}

static void LED_Blink(uint16_t pin, uint8_t n, uint16_t delay_ms)
{
    uint8_t i;

    for (i = 0U; i < n; i++)
    {
        LED_On(pin);
        HAL_Delay(delay_ms);
        LED_Off(pin);
        HAL_Delay(delay_ms);
    }
}

/* ================= SENSOR INITIALIZATION ================= */

static void Sensors_Init(void)
{
    uint32_t i;

    for (i = 0U; i < MPR_COUNT; i++)
    {
        mpr[i].id = mpr_id[i];
        mpr_alive[i] = 0U;
    }

    for (i = 0U; i < SHT_COUNT; i++)
    {
        sht[i].addr = sht_id[i];
        sht_alive[i] = 0U;
    }
}

/* ================= SENSOR ACQUISITION ================= */

static void SHT_ReadAll(void)
{
    uint32_t i;

    for (i = 0U; i < SHT_COUNT; i++)
    {
        uint8_t read_ok = SHT20_Read(&sht[i]);

        sht_alive[i] = read_ok;
        Raspberry_SendSHTFrame(&sht[i], read_ok);

        if (read_ok != 0U)
        {
            LED_Blink(LED_SHT_PIN, 1U, 50U);
        }
        else
        {
            UART_Printf("SHT20 id=%u Modbus=%s (%d)\r\n",
                        (unsigned int)sht[i].addr,
                        Modbus_StatusName(sht[i].last_status),
                        sht[i].last_status);
            LED_Blink(LED_SHT_PIN, 2U, 150U);
        }

        /* Give the shared RS485 bus time between sensor transactions. */
        HAL_Delay(1000U);
    }
}

static void MPR_ReadAll(void)
{
    uint32_t i;

    for (i = 0U; i < MPR_COUNT; i++)
    {
        uint8_t read_ok = MPR47S_Read(&mpr[i]);

        mpr_alive[i] = read_ok;
        Raspberry_SendMPRFrame(&mpr[i], read_ok);

        if (read_ok != 0U)
        {
            LED_Blink(LED_MPR_PIN, 1U, 50U);
        }
        else
        {
            UART_Printf("MPR47S id=%u Modbus=%s (%d)\r\n",
                        (unsigned int)mpr[i].id,
                        Modbus_StatusName(mpr[i].last_status),
                        mpr[i].last_status);
            LED_Blink(LED_MPR_PIN, 2U, 150U);
        }

        HAL_Delay(500U);
    }
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  Modbus_Init(&huart2);
  Sensors_Init();

  UART_Printf("\r\n========================================\r\n");
  UART_Printf(" SMART ENERGY MONITOR - STM32F407\r\n");
  UART_Printf(" USART2: Modbus RS485, 9600 8N1\r\n");
  UART_Printf(" USART3: Diagnostic terminal, 9600 8N1\r\n");
  UART_Printf(" UART4: Raspberry Pi, 115200 8N1 (DMA)\r\n");
  UART_Printf("========================================\r\n");

  /* Send a JSON boot frame to the Raspberry Pi. */
  Raspberry_SendBootFrame();

  HAL_Delay(1000U);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      SHT_ReadAll();
      HAL_Delay(2000U);

      MPR_ReadAll();
      HAL_Delay(3000U);
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 15999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 4999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* Add project-specific callbacks here if required. */
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
