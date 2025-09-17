/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * FootSense - STM32-based wearable device for foot health monitoring
  * Copyright (c) 2025 Yigu
  *
  ******************************************************************************
  */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Private includes ----------------------------------------------------------*/
#include "sensor_manager.h"
#include "data_fusion.h"
#include "ai_engine.h"
#include "communication.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
#define FOOTSENSE_VERSION_MAJOR    1
#define FOOTSENSE_VERSION_MINOR    0
#define FOOTSENSE_VERSION_PATCH    0

/* System Configuration */
#define MAIN_TASK_PRIORITY         (tskIDLE_PRIORITY + 3)
#define SENSOR_TASK_PRIORITY       (tskIDLE_PRIORITY + 4)
#define AI_TASK_PRIORITY           (tskIDLE_PRIORITY + 2)
#define COMM_TASK_PRIORITY         (tskIDLE_PRIORITY + 1)

#define MAIN_TASK_STACK_SIZE       (256)
#define SENSOR_TASK_STACK_SIZE     (512)
#define AI_TASK_STACK_SIZE         (1024)
#define COMM_TASK_STACK_SIZE       (512)

/* GPIO Pin Definitions */
#define LED_STATUS_Pin             GPIO_PIN_13
#define LED_STATUS_GPIO_Port       GPIOC

#define SENSOR_POWER_Pin           GPIO_PIN_0
#define SENSOR_POWER_GPIO_Port     GPIOA

/* I2C Configuration */
#define SENSOR_I2C                 I2C1
#define SENSOR_I2C_SCL_Pin         GPIO_PIN_6
#define SENSOR_I2C_SCL_GPIO_Port   GPIOB
#define SENSOR_I2C_SDA_Pin         GPIO_PIN_7
#define SENSOR_I2C_SDA_GPIO_Port   GPIOB

/* SPI Configuration */
#define SENSOR_SPI                 SPI1
#define SENSOR_SPI_SCK_Pin         GPIO_PIN_5
#define SENSOR_SPI_SCK_GPIO_Port   GPIOA
#define SENSOR_SPI_MISO_Pin        GPIO_PIN_6
#define SENSOR_SPI_MISO_GPIO_Port  GPIOA
#define SENSOR_SPI_MOSI_Pin        GPIO_PIN_7
#define SENSOR_SPI_MOSI_GPIO_Port  GPIOA

/* UART Configuration */
#define DEBUG_UART                 USART2
#define BLE_UART                   USART1

/* Exported macro ------------------------------------------------------------*/
#define FOOTSENSE_DEBUG 1

#if FOOTSENSE_DEBUG
#define FS_DEBUG_PRINTF(fmt, ...) printf("[FootSense] " fmt "\r\n", ##__VA_ARGS__)
#else
#define FS_DEBUG_PRINTF(fmt, ...)
#endif

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_I2C1_Init(void);
void MX_SPI1_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);

/* FreeRTOS Task Functions */
void MainTask(void *argument);
void SensorTask(void *argument);
void AITask(void *argument);
void CommunicationTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */