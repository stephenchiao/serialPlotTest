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
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "zdtCan.h"
#include "zdtEmm.h"
#include "serialPlot.h"
#include "zdtUart.h"
#include <stdio.h>
#include <math.h>
#include "mecanum_chassis.h"
#include "ops9.h"
#include "pid.h"
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
extern ZDT_Motor_t motors[4];

uint32_t last_odom_tick = 0;
uint8_t move_state = 0; // 0:向前，1:停，2:向后，3:停

// 4个电机速度缓存
float motor_target_speed[4] = {0};  // 目标速度
float motor_actual_speed[4] = {0};  // 实际速度
PID_Controller pid_x;
PID_Controller pid_y;
PID_Controller pid_yaw;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // 声明外部的接收缓存变量
  extern uint8_t ops9_rx_byte;
  // 开启 USART2 单字节中断接收
  HAL_UART_Receive_IT(&huart2, &ops9_rx_byte, 1);
  // 1. 初始化 CAN 和过滤器
  ZDT_CAN_ConfigFilter();

  // 2. 注册回调
  ZDT_CAN_RegisterCallback(ZDT_Emm_RxHandler);

  // 3. 初始化 4 个电机
  ZDT_Emm_InitAll();

  // 4. 使能所有电机（必须使能才能响应速度命令）
  // 依据：P48 5.3.2 电机使能控制
  HAL_Delay(100);
  ZDT_Emm_EnableByID(1);
  HAL_Delay(10);
  ZDT_Emm_EnableByID(2);
  HAL_Delay(10);
  ZDT_Emm_EnableByID(3);
  HAL_Delay(10);
  ZDT_Emm_EnableByID(4);
  HAL_Delay(100);  // 等待使能完成

  // 5.启动定时器（用于定时读取速度）
  HAL_TIM_Base_Start_IT(&htim3);

  // 6. 初始化里程计计时器
  last_odom_tick = HAL_GetTick();

  //7.初始化PID参数
  // 注意：坐标单位是 mm，误差 1000mm 时，乘以 Kp=0.001，算出的速度正好是 1.0 m/s
    PID_Init(&pid_x,   0.002f, 0.0f, 0.0f, 0.3f, 0.1f);  // X轴纠偏：限速 0.3 m/s
    PID_Init(&pid_y,   0.001f, 0.0f, 0.0f, 0.3f, 0.2f);  // Y轴主干：限速 0.5 m/s (比较安全的测试速度)
    PID_Init(&pid_yaw, 0.02f,  0.0f, 0.0f, 0.5f, 0.2f);  // 角度纠偏：限速 0.5 rad/s

    // 设定小车的首个演示目标
    PID_SetTarget(&pid_x, 0.0f);     // 目标 X 坐标 = 0
    PID_SetTarget(&pid_y, 300.0f);  // 目标 Y 坐标 = 1000 (向前直行 1米)
    PID_SetTarget(&pid_yaw, 0.0f);   // 目标角度 = 0 (车头保持朝前不变)

    for(int i = 5; i > 0; i--) {
          printf("Counting down: %d\r\n", i);
          HAL_Delay(1000); // 延时 1000ms (1秒)
      }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // 1. 根据当前 OPS-9 的实时绝对坐标，计算出三个维度的期望速度
	      float out_x   = PID_Calc(&pid_x, robot_x);
	      float out_y   = PID_Calc(&pid_y, robot_y);
	      float out_yaw = PID_Calc(&pid_yaw, robot_yaw);

	      // 2. 将 PID 算出的速度喂给麦克纳姆轮底盘进行逆解算
	      float V1, V2, V3, V4;
	      // (坐标系已对齐：out_x对应向右Vx，out_y对应向前Vy，out_yaw对应逆时针Vz)
	      Mecanum_Kinematics(out_x, out_y, out_yaw, &V1, &V2, &V3, &V4);

	      // 3. 把解算出来的四个轮子转速发送给 CAN 电机驱动
	      SetAllMotorsSpeed(V1, V2, V3, V4);

	      // 4. 打印实时状态，方便通过串口助手观察小车位姿和收敛情况
	      printf("Pos(%.0f, %.0f) Yaw:%.1f | Out[X:%.2f Y:%.2f Yaw:%.2f]\r\n",
	             robot_x, robot_y, robot_yaw, out_x, out_y, out_yaw);

	      // 5. PID 控制周期定为 20ms (50Hz 控制频率)
	      HAL_Delay(20);
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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
// CAN 接收中断回调
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    ZDT_CAN_RxFIFO0_Handler(hcan);
}
int _write(int file, char *ptr, int len)
{
    // 注意：假设你连接电脑的串口是 USART1。如果是其他串口，请修改 &huart1
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}
// 定时器中断回调 (10ms 一次)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        // 如果需要绘图，可以在这里置标志位
        // flag_plot_10ms = 1;
    }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // 调用 OPS-9 解析函数
    OPS9_UART_RxCpltCallback(huart);
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
