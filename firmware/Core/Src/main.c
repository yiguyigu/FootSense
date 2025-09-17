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
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "IIC.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t Buffer1[1];
uint16_t g_usart1_rx_sta;
uint8_t g_usart1_rx_buffer[200];
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

#include "NanoEdgeAI.h"
#include "knowledge.h"
/* Private define --------------------------------------------------------------*/
/* Private variables defined by user -------------------------------------------*/
float input_user_buffer[DATA_INPUT_USER * AXIS_NUMBER]; // Buffer of input values
float output_class_buffer[CLASS_NUMBER]; // Buffer of class probabilities

void fill_buffer(float sample_buffer[])
{
	/* USER BEGIN */
	/* USER END */
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint16_t ADC_Buffer[8];
uint32_t cnt;
uint8_t led_flag;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
		float pitch,roll,yaw; 		    //欧拉角
    short aacx,aacy,aacz;			//加速度传感器原始数据
    short gyrox,gyroy,gyroz;		//陀螺仪原始数据
//    float temp;					    //温度
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	
	enum neai_state error_code = neai_classification_init(knowledge);
	if (error_code != NEAI_OK) {
		/* This happens if the knowledge does not correspond to the library or if the library works into a not supported board. */
	}

	/* Classification ------------------------------------------------------------*/
	uint16_t id_class = 0;
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
		MPU_Init();			//MPU6050初始化
    mpu_dmp_init();		//dmp初始化

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//	printf("%d\r\n",mpu_dmp_init());
	printf("init is ok\r\n");
  while (1)
  {
		printf("\r\n");
		
				HAL_Delay(200);
        while(mpu_dmp_get_data(&pitch, &roll, &yaw));	//必须要用while等待，才能读取成功
        MPU_Get_Accelerometer(&aacx,&aacy, &aacz);		//得到加速度传感器数据
        MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);		//得到陀螺仪数据
//        temp=MPU_Get_Temperature();						//得到温度信息
//		printf("X:%.1f, Y:%.1f, Z:%.1f,tem:%.2f\r\n",roll,pitch,yaw,temp/100);//串口1输出采集信息
		printf("%.1f,%.1f,%.1f,%hd,%hd,%hd,%hd,%hd,%hd\r\n",roll,pitch,yaw,aacx,aacy,aacz,gyrox,gyroy,gyroz);//串口1输出采集信息

//		HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_Buffer,8);
		
		input_user_buffer[0]=roll;
		input_user_buffer[1]=pitch;
		input_user_buffer[2]=yaw;
		input_user_buffer[3]=aacx;
		input_user_buffer[4]=aacy;
		input_user_buffer[5]=aacz;
		input_user_buffer[6]=gyrox;
		input_user_buffer[7]=gyroy;
		input_user_buffer[8]=gyroz;
		
		fill_buffer(input_user_buffer);
		neai_classification(input_user_buffer, output_class_buffer, &id_class);
		for(int i=0;i<3;i++)
		{
			printf ("oput:%f\r\n",output_class_buffer[i]);
		}
		printf("id_class:%d",id_class);
			  HAL_Delay(100);


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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 240;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
