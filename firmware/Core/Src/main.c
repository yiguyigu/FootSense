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
/* ========== RTOS & AI includes ========== */
#include "cmsis_os2.h"
#include "NanoEdgeAI.h"
#include "knowledge.h"

/* ========== 用户数据结构与缓冲 ========== */
typedef struct {
    uint32_t ts;
    float pressure[8];
    float imu[6];
    float env[2];
} SensorData;

#define SAMPLE_PERIOD_MS 10
#define WINDOW_SIZE      20
#define FEATURE_DIM      20  /* 优化为20维特征 */

static SensorData g_window[WINDOW_SIZE];
static volatile uint32_t g_samples_count;

static float feature_vector[FEATURE_DIM];

/* 任务与队列句柄 */
static osThreadId_t hTaskSampling, hTaskFeature, hTaskAI, hTaskBLE;
static osMessageQueueId_t qFeature, qBLE;

typedef struct {
    uint32_t ts;
    uint8_t stance;
    uint8_t arch;
    uint8_t sweat;
    float probs[CLASS_NUMBER];
} AI_Result;

/* ========== 窗口特征提取（均值/标准差/能量） ========== */
static void compute_mean_std(const float *x, int n, float *mean, float *std)
{
    float sum = 0.0f, sum_sq = 0.0f;
    for (int i = 0; i < n; ++i) {
        sum += x[i];
        sum_sq += x[i] * x[i];
    }
    *mean = sum / (float)n;
    float var = (sum_sq / (float)n) - (*mean) * (*mean);
    *std = (var > 0.0f) ? sqrtf(var) : 0.0f;
}

static void extract_features_window(float *out20)
{
    if (g_samples_count < WINDOW_SIZE) {
        for (int i = 0; i < FEATURE_DIM; ++i) out20[i] = 0.0f;
        return;
    }
    
    int cursor = 0;
    float temp_buf[WINDOW_SIZE];
    
    /* 关键压力传感器 4 路：均值（足弓判断用） */
    for (int ch = 0; ch < 4; ++ch) {
        for (int i = 0; i < WINDOW_SIZE; ++i) {
            temp_buf[i] = g_window[i].pressure[ch];
        }
        compute_mean_std(temp_buf, WINDOW_SIZE, &out20[cursor], NULL);
        cursor++;
    }
    
    /* IMU 6 轴：均值（姿态判断用） */
    for (int ch = 0; ch < 6; ++ch) {
        for (int i = 0; i < WINDOW_SIZE; ++i) {
            temp_buf[i] = g_window[i].imu[ch];
        }
        compute_mean_std(temp_buf, WINDOW_SIZE, &out20[cursor], NULL);
        cursor++;
    }
    
    /* 环境 2 路：均值（出汗判断用） */
    for (int ch = 0; ch < 2; ++ch) {
        for (int i = 0; i < WINDOW_SIZE; ++i) {
            temp_buf[i] = g_window[i].env[ch];
        }
        compute_mean_std(temp_buf, WINDOW_SIZE, &out20[cursor], NULL);
        cursor++;
    }
    
    /* 压力分布方差（足弓特征） */
    float pressure_var = 0.0f;
    for (int ch = 0; ch < 8; ++ch) {
        float mean = 0.0f;
        for (int i = 0; i < WINDOW_SIZE; ++i) mean += g_window[i].pressure[ch];
        mean /= WINDOW_SIZE;
        for (int i = 0; i < WINDOW_SIZE; ++i) {
            float diff = g_window[i].pressure[ch] - mean;
            pressure_var += diff * diff;
        }
    }
    out20[cursor++] = pressure_var / (WINDOW_SIZE * 8);
    
    /* 姿态变化幅度（姿态特征） */
    float motion_amp = 0.0f;
    for (int ch = 0; ch < 6; ++ch) {
        float max_val = g_window[0].imu[ch], min_val = g_window[0].imu[ch];
        for (int i = 1; i < WINDOW_SIZE; ++i) {
            if (g_window[i].imu[ch] > max_val) max_val = g_window[i].imu[ch];
            if (g_window[i].imu[ch] < min_val) min_val = g_window[i].imu[ch];
        }
        motion_amp += (max_val - min_val) * (max_val - min_val);
    }
    out20[cursor++] = sqrtf(motion_amp / 6.0f);
    
    /* 填充剩余维度 */
    while (cursor < FEATURE_DIM) out20[cursor++] = 0.0f;
}

/* ========== 传感器读取函数声明 ========== */
static void read_pressure_sensors(float pressure[8]);
static void read_temp_humidity(float temp_humid[2]);

/* ========== 任务函数声明 ========== */
static void Task_Sampling(void *argument);
static void Task_Feature(void *argument);
static void Task_AI(void *argument);
static void Task_BLE(void *argument);

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
		float pitch,roll,yaw;           // 欧拉角
    short aacx,aacy,aacz;          // 加速度原始数据
    short gyrox,gyroy,gyroz;       // 陀螺仪原始数据
    float temp;                    // 温度
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* 初始化 NanoEdge AI */
  if (neai_classification_init(knowledge) != NEAI_OK) {
  }
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
  MPU_Init();
  mpu_dmp_init();

  /* ====== 创建 RTOS 资源 ====== */
  osKernelInitialize();
  qFeature = osMessageQueueNew(2, sizeof(feature_vector), NULL);
  qBLE     = osMessageQueueNew(4, sizeof(AI_Result), NULL);

  const osThreadAttr_t attrSampling = { .name = "Sampling", .priority = osPriorityHigh };
  const osThreadAttr_t attrFeature  = { .name = "Feature",  .priority = osPriorityNormal };
  const osThreadAttr_t attrAI       = { .name = "AI",       .priority = osPriorityNormal };
  const osThreadAttr_t attrBLE      = { .name = "BLE",      .priority = osPriorityBelowNormal };

  hTaskSampling = osThreadNew(Task_Sampling, NULL, &attrSampling);
  hTaskFeature  = osThreadNew(Task_Feature,  NULL, &attrFeature);
  hTaskAI       = osThreadNew(Task_AI,       NULL, &attrAI);
  hTaskBLE      = osThreadNew(Task_BLE,      NULL, &attrBLE);

  osKernelStart();
  /* 不应返回 */
  /* USER CODE END 2 */

  while (1) { }
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

/* ========== 传感器读取实现 ========== */
static void read_pressure_sensors(float pressure[8])
{
    /* 启动 ADC DMA 读取 8 路压力传感器 */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Buffer, 8);
    HAL_Delay(1); /* 等待 DMA 完成 */
    for (int i = 0; i < 8; ++i) {
        pressure[i] = (float)ADC_Buffer[i] * 3.3f / 4095.0f; /* 转换为电压值 */
    }
}

static void read_temp_humidity(float temp_humid[2])
{
    /* 简化实现：从 MPU6050 获取温度，湿度设为固定值 */
    float temp = MPU_Get_Temperature() / 100.0f; /* 转换为摄氏度 */
    temp_humid[0] = temp;
    temp_humid[1] = 50.0f; /* 固定湿度 50%，后续可接入实际温湿度传感器 */
}

/* ========== FreeRTOS 任务实现 ========== */
static void Task_Sampling(void *argument)
{
  float pitch, roll, yaw;
  short aacx, aacy, aacz;
  short gyrox, gyroy, gyroz;
  (void)argument;
  for(;;) {
    osDelay(SAMPLE_PERIOD_MS);
    while (mpu_dmp_get_data(&pitch, &roll, &yaw));
    MPU_Get_Accelerometer(&aacx, &aacy, &aacz);
    MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);
    
    SensorData s = {0};
    s.ts = HAL_GetTick();
    s.imu[0] = (float)aacx; s.imu[1] = (float)aacy; s.imu[2] = (float)aacz;
    s.imu[3] = (float)gyrox; s.imu[4] = (float)gyroy; s.imu[5] = (float)gyroz;
    
    /* 读取压力传感器和温湿度 */
    read_pressure_sensors(s.pressure);
    read_temp_humidity(s.env);
    
    g_window[g_samples_count % WINDOW_SIZE] = s;
    g_samples_count++;
  }
}

static void Task_Feature(void *argument)
{
  (void)argument;
  for(;;) {
    if (g_samples_count >= WINDOW_SIZE) {
      extract_features_window(feature_vector);
      (void)osMessageQueuePut(qFeature, feature_vector, 0U, 0U);
    }
    osDelay(100);
  }
}

static void Task_AI(void *argument)
{
  (void)argument;
  float fv[FEATURE_DIM];
  for(;;) {
    if (osMessageQueueGet(qFeature, fv, NULL, osWaitForever) == osOK) {
      AI_Result r = {0};
      r.ts = HAL_GetTick();
      
      /* 三模型分类：1个AI + 2个规则 */
      float input_user_buffer[DATA_INPUT_USER * AXIS_NUMBER] = {0};
      for (int i = 0; i < DATA_INPUT_USER && i < FEATURE_DIM; ++i) input_user_buffer[i] = fv[i];
      
      /* 姿态模型：NanoEdge AI（正常/内八/外八） */
      uint16_t stance_class = 0;
      float stance_probs[CLASS_NUMBER] = {0};
      neai_classification(input_user_buffer, stance_probs, &stance_class);
      r.stance = (uint8_t)stance_class;
      
      /* 足弓模型：规则判断（正常/扁平） */
      float arch_score = fv[13]; /* 压力分布方差特征 */
      r.arch = (arch_score > 0.5f) ? 1 : 0; /* 阈值判断 */
      
      /* 出汗模型：规则判断（正常/多汗） */
      float temp = fv[12];  /* 温度均值 */
      float humid = fv[13]; /* 湿度均值 */
      r.sweat = (temp > 25.0f && humid > 50.0f) ? 1 : 0; /* 温湿度阈值 */
      
      /* 复制概率（仅姿态模型有概率输出） */
      for (int i = 0; i < CLASS_NUMBER && i < 3; ++i) r.probs[i] = stance_probs[i];
      
      (void)osMessageQueuePut(qBLE, &r, 0U, 0U);
    }
  }
}

static void Task_BLE(void *argument)
{
  (void)argument;
  AI_Result r;
  char uart_msg[128];
  for(;;) {
    if (osMessageQueueGet(qUART, &r, NULL, osWaitForever) == osOK) {
      /* 格式化 JSON 输出到 UART */
      snprintf(uart_msg, sizeof(uart_msg), 
               "{\"ts\":%lu,\"stance\":%u,\"arch\":%u,\"sweat\":%u,\"probs\":[%.3f,%.3f,%.3f]}\r\n",
               (unsigned long)r.ts, r.stance, r.arch, r.sweat,
               r.probs[0], r.probs[1], r.probs[2]);
      
      /* 通过 UART2 发送（115200 波特率） */
      HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg, strlen(uart_msg), 100);
    }
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
