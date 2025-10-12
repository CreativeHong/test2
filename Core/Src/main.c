/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmi.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void bmi088_init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void acc_calculate(){
  // 1. 设置/读取acc0x41寄存器中的量程range参数，并换算为量程系数
  bmi088_accel_read_reg(0x41, &raw_range, 1);
  float accel_range_factor;
  switch(raw_range & 0x03) {  // 只考虑最低两位
    case 0x00:  // ±3g
      accel_range_factor = 3.0f * 9.81f / 32768.0f;
      break;
    case 0x01:  // ±6g
      accel_range_factor = 6.0f * 9.81f / 32768.0f;
      break;
    case 0x02:  // ±12g
      accel_range_factor = 12.0f * 9.81f / 32768.0f;
      break;
    case 0x03:  // ±24g
      accel_range_factor = 24.0f * 9.81f / 32768.0f;
      break;
  }

  // 2. 读取acc0x12寄存器中的6位acc数据
  bmi088_accel_read_reg(0x12, rx_acc_data, 7);

  // 3. 用量程系数将原始数据转换为常用单位
  int16_t raw_acc_x = (int16_t)((rx_acc_data[2] << 8) | rx_acc_data[1]);
  int16_t raw_acc_y = (int16_t)((rx_acc_data[4] << 8) | rx_acc_data[3]);
  int16_t raw_acc_z = (int16_t)((rx_acc_data[6] << 8) | rx_acc_data[5]);

  float acc_x = raw_acc_x * accel_range_factor;
  float acc_y = raw_acc_y * accel_range_factor;
  float acc_z = raw_acc_z * accel_range_factor;
  // 数据转换后添加溢出处理
  if (raw_acc_x > 32767) raw_acc_x -= 65536;
  if (raw_acc_y > 32767) raw_acc_y -= 65536;
  if (raw_acc_z > 32767) raw_acc_z -= 65536;


  imu.acc_x = acc_x;
  imu.acc_y = acc_y;
  imu.acc_z = acc_z;
}

void gyro_calculate(){
  // 1. 设置/读取gyro0x0F寄存器中的量程range参数，并换算为量程系数
  uint8_t gyro_range;
  bmi088_gyro_read_reg(0x0F, &gyro_range, 1);
  float gyro_range_factor;
  switch(gyro_range & 0x03) {  // 只考虑最低两位
    case 0x00:  // ±2000°/s
      gyro_range_factor = 2000.0f / 32768.0f;
      break;
    case 0x01:  // ±1000°/s
      gyro_range_factor = 1000.0f / 32768.0f;
      break;
    case 0x02:  // ±500°/s
      gyro_range_factor = 500.0f / 32768.0f;
      break;
    case 0x03:  // ±250°/s
      gyro_range_factor = 250.0f / 32768.0f;
      break;
  }

  // 2. 读取gyro0x02寄存器中的6位gyro数据
  bmi088_gyro_read_reg(0x02, rx_gyro_data, 6);

  // 3. 用量程系数将原始数据转换为常用单位
  int16_t raw_gyro_x = (int16_t)((rx_gyro_data[1] << 8) | rx_gyro_data[0]);
  int16_t raw_gyro_y = (int16_t)((rx_gyro_data[3] << 8) | rx_gyro_data[2]);
  int16_t raw_gyro_z = (int16_t)((rx_gyro_data[5] << 8) | rx_gyro_data[4]);

  float gyro_x = raw_gyro_x * gyro_range_factor;
  float gyro_y = raw_gyro_y * gyro_range_factor;
  float gyro_z = raw_gyro_z * gyro_range_factor;

  imu.angle_x = gyro_x;
  imu.angle_y = gyro_y;
  imu.angle_z = gyro_z;
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
  MX_SPI1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  bmi088_init();
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    // 在此处调用读取函数
    acc_calculate();   // 读取加速度数据
    gyro_calculate();  // 读取陀螺仪数据
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
void bmi088_init (void) {
  // Soft Reset ACCEL
  BMI088_ACCEL_NS_L();
  bmi088_write_reg(0x7E, 0xB6); // Write 0xB6 to ACC_SOFTRESET(0x7E)
  HAL_Delay(1);
  BMI088_ACCEL_NS_H();

  // Soft Reset GYRO
  BMI088_GYRO_NS_L();
  bmi088_write_reg(0x14, 0xB6); // Write 0xB6 to GYRO_SOFTRESET(0x14)
  HAL_Delay(30);
  BMI088_GYRO_NS_H();

  // Switch ACCEL to Normal Mode
  BMI088_ACCEL_NS_L();
  HAL_Delay(1);
  bmi088_write_reg(0x7D, 0x04); // Write 0x04 to ACC_PWR_CTRL(0x7D)
  HAL_Delay(1);
  BMI088_ACCEL_NS_H();
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
