/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "motor_can.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define pi 3.1415
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int16_t led_cnt;
int16_t text_speed = 0;
int16_t target_yaw_speed;
float target_yaw_angle = 90;	//设定电机角度0~360°，90°对应串口发送0x00,0x08（2048）
float now_yaw_angle1, now_yaw_angle2 = 0;		//当前电机反馈角度
extern moto_info_t motor_yaw_info1, motor_yaw_info2;		//电机结构体
extern pid_struct_t gimbal_yaw_speed_pid1, gimbal_yaw_speed_pid2;//电机速度PID
extern pid_struct_t gimbal_yaw_angle_pid1, gimbal_yaw_angle_pid2;//电机角度PID

char message[] = "Hello World";
uint8_t receiveData = 0;

//测试CAN接收功能使用
CAN_RxHeaderTypeDef test_header;
uint8_t             test_data[8] = {0};

//uint16_t can_cnt;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* 函数：单位映射函数
 * 作用：将编码器的值转换为弧度制的角度值
 * 参数x：需要转换的值
 * 参数in：编码器的角度范围，参考0~8191
 * 参数out：计算的角度范围，参考0~360°
 */
double msp(double x, double in_min, double in_max, double out_min, double out_max)
{
	return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
    	target_yaw_angle = receiveData;	//串口通信数据类型是uint，数据只能是正整数

    	HAL_UART_Transmit(&huart1, &receiveData, 3, 100);	//接收后反馈发送
        HAL_UART_Receive_IT(&huart1, &receiveData, 3);		//再次开启接收中断
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
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
//  HAL_UART_Receive_IT(&huart1, &receiveData, 3);	//开启串口中断接收，接收两位正数数据作为设定角度
  can_filter_init();//can初始化
  gimbal_PID_init();//PID初始化，在此函数里预设PID的参数

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	//0、闪烁小灯，检测程序是否正常循环
	led_cnt ++;
	if (led_cnt == 50)
	{
	  	led_cnt = 0;
	    HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_10);
	}

	//PID控制
	//1、计算当前的编码器角度值，运用msp函数将编码器的值映射为弧度制
	now_yaw_angle1=msp(motor_yaw_info1.rotor_angle,0,8191,0,360);
	now_yaw_angle2=msp(motor_yaw_info2.rotor_angle,0,8191,0,360);
	//2、角度环，设定角度target_yaw_angle，通过角度差计算出需要的pid速度
	pid_calc(&gimbal_yaw_angle_pid1, target_yaw_angle, now_yaw_angle1);
	pid_calc(&gimbal_yaw_angle_pid2, target_yaw_angle, now_yaw_angle2);
	//3、速度环，设定速度为角度环计算的速度，通过速度差计算更新输出pid值
	pid_calc(&gimbal_yaw_speed_pid1, gimbal_yaw_angle_pid1.output, motor_yaw_info1.rotor_speed);
	pid_calc(&gimbal_yaw_speed_pid2, gimbal_yaw_angle_pid2.output, motor_yaw_info2.rotor_speed);
	//4、向发送PID计算后更新的电压值
	set_GM6020_motor_voltage(&hcan1,gimbal_yaw_speed_pid1.output,gimbal_yaw_speed_pid2.output);
	//5、串口发送电机反馈的机械角度
	HAL_UART_Transmit(&huart1, &motor_yaw_info1.rotor_angle, 2, 100);
	HAL_UART_Transmit(&huart1, &motor_yaw_info2.rotor_angle, 2, 100);
	//6、延时
	HAL_Delay(200);


	 	 //CAN接收功能测试
//		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &test_header, test_data);	//自带接收函数，接受fifo0的数据拼接入test_header中
//
//		switch(test_header.StdId)
//				{
//			  	  	//ID6的电机反馈报文标识符为0x206
//					case 0x206:
//					{	//拼接电机参数
//						motor_yaw_info2.rotor_angle    = ((test_data[0] << 8) | test_data[1]);//角度，角度值范围：0 ~ 8191
//						motor_yaw_info2.rotor_speed    = ((test_data[2] << 8) | test_data[3]);//速度，转速值单位：rpm
//						motor_yaw_info2.torque_current = ((test_data[4] << 8) | test_data[5]);//转矩电流
//						motor_yaw_info2.temperate      =   test_data[6];					 //温度
//						break;
//					}
//				}
//
//		HAL_UART_Transmit(&huart1, &motor_yaw_info1.rotor_angle, 2, 100);
//		HAL_UART_Transmit(&huart1, &motor_yaw_info2.rotor_angle, 2, 100);
//		HAL_Delay(500);

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
