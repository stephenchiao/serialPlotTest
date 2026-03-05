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
#define WHEEL_DIAMETER  0.075f      // 轮子直径 100mm (需根据实际修改)
#define ROBOT_W         0.229f      // 轮距 (左右轮距离)
#define ROBOT_H         0.255f      // 轴距 (前后轮距离)
#define TARGET_DIST     2.0f       // 目标移动距离 2 米
#define MOVE_SPEED      0.3f       // 移动速度 0.3 m/s

extern ZDT_Motor_t motors[4];
// 里程计变量
float current_distance = 0.0f;
float last_distance = 0.0f;
uint32_t last_odom_tick = 0;
uint8_t move_state = 0; // 0:向前，1:停，2:向后，3:停

// 4个电机速度缓存
float motor_speeds[4] = {0};  // 0:ID1左后，1:ID2左前，2:ID3右前，3:ID4右后
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// ✅ 添加函数声明（解决隐式声明警告）
void Mecanum_Kinematics(float Vx, float Vy, float Vz, float *V_bl, float *V_fl, float *V_fr, float *V_br);
float MsToRpm(float v_ms);
void SetAllMotorsSpeed(float V_bl, float V_fl, float V_fr, float V_br);
void ReadAllMotorsSpeed(void);
void StopAllMotors(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Mecanum_Kinematics(float Vx, float Vy, float Vz, float *V_bl, float *V_fl, float *V_fr, float *V_br) {
    float L = (ROBOT_H / 2.0f) + (ROBOT_W / 2.0f);
    // 依据用户提供的公式
    *V_bl = Vx + Vy - Vz * L;  // ID 1 左后
    *V_fl = Vx - Vy - Vz * L;  // ID 2 左前
    *V_fr = -(Vx + Vy + Vz * L);  // ID 3 右前
    *V_br = -(Vx - Vy + Vz * L);  // ID 4 右后
}

// ✅ 线速度转 RPM 函数
float MsToRpm(float v_ms) {
    if (WHEEL_DIAMETER <= 0) return 0;
    // V = RPM * π * D / 60  =>  RPM = V * 60 / (π * D)
    return v_ms * 60.0f / (3.1415926f * WHEEL_DIAMETER);
}

// ✅ 设置 4 个轮子速度
void SetAllMotorsSpeed(float V_bl, float V_fl, float V_fr, float V_br) {
    ZDT_Emm_SetSpeedByID(1, MsToRpm(V_bl));  // ID 1: 左后
    ZDT_Emm_SetSpeedByID(2, MsToRpm(V_fl));  // ID 2: 左前
    ZDT_Emm_SetSpeedByID(3, MsToRpm(V_fr));  // ID 3: 右前
    ZDT_Emm_SetSpeedByID(4, MsToRpm(V_br));  // ID 4: 右后
}

// ✅ 读取 4 个轮子速度 (用于里程计)
void ReadAllMotorsSpeed(void) {
    ZDT_Emm_ReadSpeedByID(1);
    ZDT_Emm_ReadSpeedByID(2);
    ZDT_Emm_ReadSpeedByID(3);
    ZDT_Emm_ReadSpeedByID(4);
}

// ✅ 计算里程计（速度积分）
void UpdateOdometry(void) {
    // 计算4轮平均线速度 (m/s)
    float avg_rpm = (motors[0].actual_speed + motors[1].actual_speed +
                     motors[2].actual_speed + motors[3].actual_speed) / 4.0f;
    float avg_ms = avg_rpm * 3.1415926f * WHEEL_DIAMETER / 60.0f;

    // 计算时间间隔 (秒)
    uint32_t current_tick = HAL_GetTick();
    float dt = (current_tick - last_odom_tick) / 1000.0f;
    last_odom_tick = current_tick;

    // 速度积分计算距离
    float delta_dist = avg_ms * dt;
    current_distance += delta_dist;
}

// ✅ 停止所有电机
void StopAllMotors(void) {
    SetAllMotorsSpeed(0, 0, 0, 0);
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
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
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

  // 5. 启动定时器（用于定时读取速度）
  HAL_TIM_Base_Start_IT(&htim3);

  // 6. 初始化里程计计时器
  last_odom_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  float V1, V2, V3, V4;
	     Mecanum_Kinematics(MOVE_SPEED, 0, 0, &V1, &V2, &V3, &V4);
	     SetAllMotorsSpeed(V1, V2, V3, V4);

	     // 小延时，避免CPU占用过高
	     HAL_Delay(10);
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

// 定时器中断回调 (10ms 一次)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        // 如果需要绘图，可以在这里置标志位
        // flag_plot_10ms = 1;
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
