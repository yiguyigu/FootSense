/**
  ******************************************************************************
  * @file    sensor_manager.h
  * @brief   Header file for sensor management system
  ******************************************************************************
  * @attention
  *
  * FootSense - Multi-sensor fusion for foot health monitoring
  * Copyright (c) 2025 Yigu
  *
  ******************************************************************************
  */

#ifndef __SENSOR_MANAGER_H
#define __SENSOR_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Sensor Types */
typedef enum {
    SENSOR_PRESSURE = 0,
    SENSOR_HUMIDITY,
    SENSOR_TEMPERATURE,
    SENSOR_ACCELEROMETER,
    SENSOR_GYROSCOPE,
    SENSOR_MAGNETOMETER,
    SENSOR_COUNT
} SensorType_t;

/* Sensor Status */
typedef enum {
    SENSOR_STATUS_OK = 0,
    SENSOR_STATUS_ERROR,
    SENSOR_STATUS_NOT_READY,
    SENSOR_STATUS_CALIBRATING
} SensorStatus_t;

/* FootSense Status */
typedef enum {
    FS_OK = 0,
    FS_ERROR,
    FS_BUSY,
    FS_TIMEOUT
} FS_Status_t;

/* Sensor Data Structure */
typedef struct {
    SensorType_t type;
    uint32_t timestamp;
    union {
        struct {
            float arch_pressure[8];     // 8 pressure points across foot arch
            float heel_pressure[4];     // 4 pressure points on heel
            float toe_pressure[4];      // 4 pressure points on toes
        } pressure;
        
        struct {
            float humidity;             // Relative humidity (%)
            float temperature;          // Temperature (°C)
        } environment;
        
        struct {
            float x, y, z;             // Acceleration in m/s²
        } accelerometer;
        
        struct {
            float x, y, z;             // Angular velocity in rad/s
        } gyroscope;
        
        struct {
            float x, y, z;             // Magnetic field in µT
        } magnetometer;
    } data;
    
    SensorStatus_t status;
    float confidence;                   // Data confidence (0.0-1.0)
} SensorData_t;

/* Sensor Configuration */
typedef struct {
    uint16_t sampling_rate;             // Hz
    uint8_t filter_enable;
    uint8_t calibration_enable;
    float sensitivity;
    float offset[3];                    // X, Y, Z offsets
} SensorConfig_t;

/* Function Prototypes */
FS_Status_t SensorManager_Init(void);
FS_Status_t SensorManager_DeInit(void);
FS_Status_t SensorManager_Start(void);
FS_Status_t SensorManager_Stop(void);

FS_Status_t SensorManager_ReadSensor(SensorType_t sensor, SensorData_t *data);
FS_Status_t SensorManager_ConfigureSensor(SensorType_t sensor, SensorConfig_t *config);
FS_Status_t SensorManager_CalibrateSensor(SensorType_t sensor);

SensorStatus_t SensorManager_GetSensorStatus(SensorType_t sensor);
uint32_t SensorManager_GetSampleCount(SensorType_t sensor);
float SensorManager_GetSensorHealth(SensorType_t sensor);

/* Task Function */
void SensorTask(void *argument);

/* Sensor-specific functions */
FS_Status_t PressureSensor_Init(void);
FS_Status_t PressureSensor_Read(SensorData_t *data);
FS_Status_t PressureSensor_Calibrate(void);

FS_Status_t EnvironmentSensor_Init(void);
FS_Status_t EnvironmentSensor_Read(SensorData_t *data);

FS_Status_t IMUSensor_Init(void);
FS_Status_t IMUSensor_ReadAccel(SensorData_t *data);
FS_Status_t IMUSensor_ReadGyro(SensorData_t *data);
FS_Status_t IMUSensor_ReadMag(SensorData_t *data);
FS_Status_t IMUSensor_Calibrate(void);

/* Utility Functions */
bool SensorManager_IsDataValid(SensorData_t *data);
void SensorManager_FilterData(SensorData_t *data);
void SensorManager_ApplyCalibration(SensorData_t *data, SensorType_t sensor);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_MANAGER_H */