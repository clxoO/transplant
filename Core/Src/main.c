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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include <stdio.h>
#include "yaw_pid.h"
#include "chassis.h"
#include "odometry.h"
#include "pos_pid.h"
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

/* USER CODE BEGIN PV */
uint8_t RxTemp_2;           // 存放每次中断收到的1个字节
uint8_t sbus_start = 0;     // 状态机标志位：是否找到了0x0F
uint8_t sbus_buf_index = 0; // 当前存到了第几个字节
uint8_t sbus_buffer[25];    // 存放拼接好的25字节一帧数据
uint8_t sbus_new_cmd = 0;   // 标志位：是否收满了一整帧
uint16_t rc_channel[16];    // 存放解析后的通道数值

int32_t enc2_debug = 0;
int32_t enc3_debug = 0;
int32_t enc4_debug = 0;
int32_t enc5_debug = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SBUS_Parse(uint8_t *buff);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


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
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  Chassis_Init(&hi2c2);
  Motor_Init();
  Encoder_Init();

  int i;
  for (i = 0; i < 4; i++)
  {
    PID_Init(&motor_pid[i], 8.0f, 2.5f, 2500, 8400);
  }

  PosCtrl_Init(500.0f, 700.0f, 400.0f, 700.0f, 0.02f);
  YawCtrl_Init(50.0f, 0.0f, 0.0f, 100.0f, 700.0f);
  
  HAL_UART_Receive_IT(&huart2, &RxTemp_2, 1);
  HAL_TIM_Base_Start_IT(&htim6);
  uint8_t move_started = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (move_started == 0)
    {
        move_started = 1;
        Chassis_StartMoveWorld(1.6f, 0.0f, 0.0f);
    }

    if (HAL_GetTick() - chassis_last_tick >= 20) 
    {
          chassis_last_tick = HAL_GetTick(); 
          
          Chassis_Update(); 
    }
    
    /*enc2_debug = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    enc3_debug = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
    enc4_debug = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
    enc5_debug = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);*/
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) 
    {
        if (sbus_start == 0 && RxTemp_2 == 0x0F) 
        {
            sbus_start = 1;
            sbus_buf_index = 0;
            sbus_buffer[sbus_buf_index] = RxTemp_2;
        } 
        else if (sbus_start == 1) 
        {
            sbus_buf_index++;
            sbus_buffer[sbus_buf_index] = RxTemp_2;
            if (sbus_buf_index >= 24) 
            {
                sbus_start = 0;
                
                uint8_t stop_byte = sbus_buffer[24];
                if (stop_byte == 0x00 || (stop_byte & 0x0F) == 0x04) 
                {
                    sbus_new_cmd = 1; 
                }
                
            }
        }
        HAL_UART_Receive_IT(&huart2, &RxTemp_2, 1);
    }
}

void SBUS_Parse(uint8_t *buff)
{
    rc_channel[0]  = ((buff[1]      | buff[2] << 8)  & 0x07FF); // CH1
    rc_channel[1]  = ((buff[2] >> 3 | buff[3] << 5)  & 0x07FF); // CH2
    rc_channel[2]  = ((buff[3] >> 6 | buff[4] << 2 | buff[5] << 10) & 0x07FF); // CH3
    rc_channel[3]  = ((buff[5] >> 1 | buff[6] << 7)  & 0x07FF); // CH4
}

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6) 
    {
        Control(); 
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == MPU_INT_Pin)
    {
        Chassis_OnImuReady();
    }
}
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
#ifdef USE_FULL_ASSERT
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
