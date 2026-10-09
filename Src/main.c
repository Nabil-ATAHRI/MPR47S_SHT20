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
#include "ModbusRS485.h"
#include "MPR47S.h"
#include "EEM375.h"
#include "SHT20.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ================= UART PRINTF ================= */
char txbuf[256];

void UART_Printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf(txbuf, sizeof(txbuf), fmt, args);
    va_end(args);

    HAL_UART_Transmit(&huart3, (uint8_t*)txbuf, strlen(txbuf), 50);
}

/* ================= DEVICES ================= */
#define MPR_COUNT 5

MPR47S_t mpr[MPR_COUNT];

uint8_t mpr_id[MPR_COUNT] = {1,2,3,4,5};
uint8_t mpr_alive[MPR_COUNT];

/* ================= INIT ================= */

void MPR_Init(void)
{
    for(int i=0; i<MPR_COUNT; i++)
    {
        mpr[i].id = mpr_id[i];
        mpr_alive[i] = 1;
    }
}

#define LED_PORT GPIOD
#define LED_PIN  GPIO_PIN_14

static inline void LED_On(void)
{
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
}

static inline void LED_Off(void)
{
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
}

void LED_Blink(uint8_t n, uint16_t t)
{
    for(uint8_t i=0;i<n;i++)
    {
        LED_On();
        HAL_Delay(t);
        LED_Off();
        HAL_Delay(t);
    }
}

uint8_t Modbus_Ping(uint8_t addr)
{
    uint16_t dummy;

    if(Modbus_ReadHoldingRegisters(addr, 0x0001, 1, &dummy) == MODBUS_OK)
        return 1;

    if(Modbus_ReadInputRegisters(addr, 0x0001, 1, &dummy) == MODBUS_OK)
        return 1;

    return 0;
}

void Bus_Test_MPR(void)
{
    UART_Printf("\r\n===== MPR BUS TEST =====\r\n");

    for(int i=0; i<MPR_COUNT; i++)
    {
        uint8_t addr = mpr_id[i];

        UART_Printf("MPR %d -> ", addr);

        LED_Blink(addr,600);

        if(Modbus_Ping(addr))
        {
            UART_Printf("OK\r\n");

            LED_Blink(15,50);

            mpr_alive[i] = 1;
        }
        else
        {
            UART_Printf("FAIL\r\n");

            LED_Blink(1,1500);

            mpr_alive[i] = 0;
        }

        HAL_Delay(1500);
    }

    UART_Printf("===== TEST END =====\r\n");
}

/* ================= PRINT ================= */
	void MPR_ReadAll(void)
{
    for(int i=0; i<MPR_COUNT; i++)
    {
        if(!mpr_alive[i])
            continue;

        if(MPR47S_Read(&mpr[i]))
        {
            UART_Printf("\r\n[MPR %d]\r\n", mpr[i].id);

            UART_Printf("\r\nPHASE VOLTAGE\r\n");
            UART_Printf("V1 : %.1f V\r\n", mpr[i].V1);
            UART_Printf("V2 : %.1f V\r\n", mpr[i].V2);
            UART_Printf("V3 : %.1f V\r\n", mpr[i].V3);
            UART_Printf("VN : %.1f V\r\n", mpr[i].V4);

            UART_Printf("\r\nLINE VOLTAGE\r\n");
            UART_Printf("VL12 : %.1f V\r\n", mpr[i].VL12);
            UART_Printf("VL23 : %.1f V\r\n", mpr[i].VL23);
            UART_Printf("VL31 : %.1f V\r\n", mpr[i].VL31);

            UART_Printf("\r\nCURRENT\r\n");
            UART_Printf("I1 : %.3f A\r\n", mpr[i].I1);
            UART_Printf("I2 : %.3f A\r\n", mpr[i].I2);
            UART_Printf("I3 : %.3f A\r\n", mpr[i].I3);
            UART_Printf("IN : %.3f A\r\n", mpr[i].IN);

            UART_Printf("\r\nACTIVE POWER\r\n");
            UART_Printf("P1 : %.1f W\r\n", mpr[i].P1);
            UART_Printf("P2 : %.1f W\r\n", mpr[i].P2);
            UART_Printf("P3 : %.1f W\r\n", mpr[i].P3);
            UART_Printf("PT : %.1f W\r\n", mpr[i].P_Total);

            UART_Printf("\r\nREACTIVE POWER\r\n");
            UART_Printf("Q1 : %.1f VAR\r\n", mpr[i].Q1);
            UART_Printf("Q2 : %.1f VAR\r\n", mpr[i].Q2);
            UART_Printf("Q3 : %.1f VAR\r\n", mpr[i].Q3);
            UART_Printf("QT : %.1f VAR\r\n", mpr[i].Q_Total);

            UART_Printf("\r\nAPPARENT POWER\r\n");
            UART_Printf("S1 : %.1f VA\r\n", mpr[i].S1);
            UART_Printf("S2 : %.1f VA\r\n", mpr[i].S2);
            UART_Printf("S3 : %.1f VA\r\n", mpr[i].S3);
            UART_Printf("ST : %.1f VA\r\n", mpr[i].S_Total);

            UART_Printf("\r\nPOWER FACTOR\r\n");
            UART_Printf("PF1 : %.3f\r\n", mpr[i].PF1);
            UART_Printf("PF2 : %.3f\r\n", mpr[i].PF2);
            UART_Printf("PF3 : %.3f\r\n", mpr[i].PF3);

            UART_Printf("\r\nFREQUENCY\r\n");
            UART_Printf("F : %.2f Hz\r\n", mpr[i].Freq);
        }
        else
        {
            UART_Printf("MPR %d COMM ERROR\r\n", mpr[i].id);
        }

        HAL_Delay(2000);
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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  	Modbus_Init(&huart2);

		MPR_Init();

		UART_Printf("\r\nSYSTEM READY\r\n");

		HAL_Delay(2000);

		Bus_Test_MPR();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		UART_Printf("\r\n===== NEW MPR CYCLE =====\r\n");

    MPR_ReadAll();

    HAL_Delay(3000);

    HAL_Delay(200);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
