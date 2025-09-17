/**
  ******************************************************************************
  * @file    sensor_manager.c
  * @brief   Sensor management system implementation
  ******************************************************************************
  * @attention
  *
  * FootSense - Multi-sensor fusion for foot health monitoring
  * Copyright (c) 2025 Yigu
  *
  ******************************************************************************
  */

#include "sensor_manager.h"
#include "main.h"
#include <string.h>
#include <math.h>

/* Private defines */
#define SENSOR_TASK_PERIOD_MS          20      // 50Hz sampling rate
#define PRESSURE_SENSOR_ADDR           0x48    // Example I2C address
#define ENVIRONMENT_SENSOR_ADDR        0x77    // Example I2C address  
#define IMU_SENSOR_ADDR               0x68    // Example I2C address

#define PRESSURE_POINTS_TOTAL         16      // Total pressure sensors
#define CALIBRATION_SAMPLES           100     // Samples for calibration

/* Private variables */
static SensorConfig_t sensor_configs[SENSOR_COUNT];
static SensorStatus_t sensor_status[SENSOR_COUNT];
static uint32_t sensor_sample_count[SENSOR_COUNT];
static float sensor_health[SENSOR_COUNT];
static bool manager_initialized = false;

/* Calibration data */
static float pressure_baseline[PRESSURE_POINTS_TOTAL];
static float accel_offset[3];
static float gyro_offset[3];
static float mag_offset[3];

/* External variables */
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern QueueHandle_t SensorDataQueue;
extern SemaphoreHandle_t I2CMutex;
extern SemaphoreHandle_t SPIMutex;

/* Private function prototypes */
static FS_Status_t InitializeSensorHardware(void);
static FS_Status_t ReadI2CSensor(uint16_t addr, uint8_t reg, uint8_t *data, uint16_t len);
static FS_Status_t WriteI2CSensor(uint16_t addr, uint8_t reg, uint8_t *data, uint16_t len);
static void ProcessSensorData(SensorData_t *data);
static float CalculateChecksum(SensorData_t *data);

/**
  * @brief  Initialize the sensor manager
  * @retval FS_Status_t
  */
FS_Status_t SensorManager_Init(void)
{
    if (manager_initialized) {
        return FS_OK;
    }

    FS_DEBUG_PRINTF("Initializing Sensor Manager...");

    /* Initialize sensor configurations */
    for (int i = 0; i < SENSOR_COUNT; i++) {
        sensor_configs[i].sampling_rate = 50;  // 50Hz default
        sensor_configs[i].filter_enable = 1;
        sensor_configs[i].calibration_enable = 1;
        sensor_configs[i].sensitivity = 1.0f;
        memset(sensor_configs[i].offset, 0, sizeof(sensor_configs[i].offset));
        
        sensor_status[i] = SENSOR_STATUS_NOT_READY;
        sensor_sample_count[i] = 0;
        sensor_health[i] = 1.0f;
    }

    /* Initialize hardware */
    if (InitializeSensorHardware() != FS_OK) {
        FS_DEBUG_PRINTF("Failed to initialize sensor hardware");
        return FS_ERROR;
    }

    /* Initialize individual sensors */
    if (PressureSensor_Init() != FS_OK) {
        FS_DEBUG_PRINTF("Pressure sensor initialization failed");
        return FS_ERROR;
    }

    if (EnvironmentSensor_Init() != FS_OK) {
        FS_DEBUG_PRINTF("Environment sensor initialization failed");
        return FS_ERROR;
    }

    if (IMUSensor_Init() != FS_OK) {
        FS_DEBUG_PRINTF("IMU sensor initialization failed");
        return FS_ERROR;
    }

    manager_initialized = true;
    FS_DEBUG_PRINTF("Sensor Manager initialized successfully");
    
    return FS_OK;
}

/**
  * @brief  Start sensor data acquisition
  * @retval FS_Status_t
  */
FS_Status_t SensorManager_Start(void)
{
    if (!manager_initialized) {
        return FS_ERROR;
    }

    FS_DEBUG_PRINTF("Starting sensor data acquisition");
    
    /* Set all sensors to OK status */
    for (int i = 0; i < SENSOR_COUNT; i++) {
        sensor_status[i] = SENSOR_STATUS_OK;
    }

    return FS_OK;
}

/**
  * @brief  Sensor task implementation
  * @param  argument: Not used
  * @retval None
  */
void SensorTask(void *argument)
{
    SensorData_t sensor_data;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS);

    FS_DEBUG_PRINTF("Sensor Task started");

    /* Initialize the xLastWakeTime variable with the current time */
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        /* Read pressure sensors */
        if (PressureSensor_Read(&sensor_data) == FS_OK) {
            ProcessSensorData(&sensor_data);
            xQueueSend(SensorDataQueue, &sensor_data, 0);
            sensor_sample_count[SENSOR_PRESSURE]++;
        }

        /* Read environment sensors */
        if (EnvironmentSensor_Read(&sensor_data) == FS_OK) {
            ProcessSensorData(&sensor_data);
            xQueueSend(SensorDataQueue, &sensor_data, 0);
            sensor_sample_count[SENSOR_HUMIDITY]++;
        }

        /* Read accelerometer */
        if (IMUSensor_ReadAccel(&sensor_data) == FS_OK) {
            ProcessSensorData(&sensor_data);
            xQueueSend(SensorDataQueue, &sensor_data, 0);
            sensor_sample_count[SENSOR_ACCELEROMETER]++;
        }

        /* Read gyroscope */
        if (IMUSensor_ReadGyro(&sensor_data) == FS_OK) {
            ProcessSensorData(&sensor_data);
            xQueueSend(SensorDataQueue, &sensor_data, 0);
            sensor_sample_count[SENSOR_GYROSCOPE]++;
        }

        /* Wait for the next cycle */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
  * @brief  Initialize pressure sensor system
  * @retval FS_Status_t
  */
FS_Status_t PressureSensor_Init(void)
{
    uint8_t config_data[2];
    
    FS_DEBUG_PRINTF("Initializing pressure sensors");

    /* Configure pressure sensor */
    config_data[0] = 0x01;  // Configuration register
    config_data[1] = 0x84;  // Continuous mode, high resolution
    
    if (WriteI2CSensor(PRESSURE_SENSOR_ADDR, 0x00, config_data, 2) != FS_OK) {
        return FS_ERROR;
    }

    /* Initialize baseline values */
    memset(pressure_baseline, 0, sizeof(pressure_baseline));
    
    sensor_status[SENSOR_PRESSURE] = SENSOR_STATUS_OK;
    return FS_OK;
}

/**
  * @brief  Read pressure sensor data
  * @param  data: Pointer to sensor data structure
  * @retval FS_Status_t
  */
FS_Status_t PressureSensor_Read(SensorData_t *data)
{
    uint8_t raw_data[32];  // 16 sensors * 2 bytes each
    
    if (ReadI2CSensor(PRESSURE_SENSOR_ADDR, 0x01, raw_data, sizeof(raw_data)) != FS_OK) {
        return FS_ERROR;
    }

    data->type = SENSOR_PRESSURE;
    data->timestamp = HAL_GetTick();
    data->status = SENSOR_STATUS_OK;
    
    /* Convert raw data to pressure values */
    for (int i = 0; i < 8; i++) {
        uint16_t raw = (raw_data[i*2] << 8) | raw_data[i*2+1];
        data->data.pressure.arch_pressure[i] = (float)raw * 0.1f - pressure_baseline[i];
    }
    
    for (int i = 0; i < 4; i++) {
        uint16_t raw = (raw_data[(i+8)*2] << 8) | raw_data[(i+8)*2+1];
        data->data.pressure.heel_pressure[i] = (float)raw * 0.1f - pressure_baseline[i+8];
    }
    
    for (int i = 0; i < 4; i++) {
        uint16_t raw = (raw_data[(i+12)*2] << 8) | raw_data[(i+12)*2+1];
        data->data.pressure.toe_pressure[i] = (float)raw * 0.1f - pressure_baseline[i+12];
    }

    data->confidence = 0.95f;  // High confidence for calibrated sensors
    
    return FS_OK;
}

/**
  * @brief  Initialize environment sensor (humidity/temperature)
  * @retval FS_Status_t
  */
FS_Status_t EnvironmentSensor_Init(void)
{
    uint8_t config_data = 0x27;  // Normal mode, temp and humidity enabled
    
    FS_DEBUG_PRINTF("Initializing environment sensor");
    
    if (WriteI2CSensor(ENVIRONMENT_SENSOR_ADDR, 0xF2, &config_data, 1) != FS_OK) {
        return FS_ERROR;
    }

    sensor_status[SENSOR_HUMIDITY] = SENSOR_STATUS_OK;
    sensor_status[SENSOR_TEMPERATURE] = SENSOR_STATUS_OK;
    
    return FS_OK;
}

/**
  * @brief  Read environment sensor data
  * @param  data: Pointer to sensor data structure
  * @retval FS_Status_t
  */
FS_Status_t EnvironmentSensor_Read(SensorData_t *data)
{
    uint8_t raw_data[6];
    
    if (ReadI2CSensor(ENVIRONMENT_SENSOR_ADDR, 0xF7, raw_data, 6) != FS_OK) {
        return FS_ERROR;
    }

    data->type = SENSOR_HUMIDITY;
    data->timestamp = HAL_GetTick();
    data->status = SENSOR_STATUS_OK;
    
    /* Convert raw data to physical values */
    uint32_t adc_T = (raw_data[3] << 12) | (raw_data[4] << 4) | (raw_data[5] >> 4);
    uint32_t adc_H = (raw_data[0] << 8) | raw_data[1];
    
    /* Simplified conversion (normally would use calibration coefficients) */
    data->data.environment.temperature = (float)adc_T / 100.0f - 40.0f;
    data->data.environment.humidity = (float)adc_H / 1024.0f * 100.0f;
    
    data->confidence = 0.90f;
    
    return FS_OK;
}

/**
  * @brief  Initialize IMU sensor
  * @retval FS_Status_t
  */
FS_Status_t IMUSensor_Init(void)
{
    uint8_t config_data;
    
    FS_DEBUG_PRINTF("Initializing IMU sensor");
    
    /* Wake up the sensor */
    config_data = 0x00;
    if (WriteI2CSensor(IMU_SENSOR_ADDR, 0x6B, &config_data, 1) != FS_OK) {
        return FS_ERROR;
    }
    
    /* Configure accelerometer (±8g, 100Hz) */
    config_data = 0x10;
    if (WriteI2CSensor(IMU_SENSOR_ADDR, 0x1C, &config_data, 1) != FS_OK) {
        return FS_ERROR;
    }
    
    /* Configure gyroscope (±1000°/s, 100Hz) */
    config_data = 0x10;
    if (WriteI2CSensor(IMU_SENSOR_ADDR, 0x1B, &config_data, 1) != FS_OK) {
        return FS_ERROR;
    }

    sensor_status[SENSOR_ACCELEROMETER] = SENSOR_STATUS_OK;
    sensor_status[SENSOR_GYROSCOPE] = SENSOR_STATUS_OK;
    sensor_status[SENSOR_MAGNETOMETER] = SENSOR_STATUS_OK;
    
    return FS_OK;
}

/**
  * @brief  Read accelerometer data
  * @param  data: Pointer to sensor data structure
  * @retval FS_Status_t
  */
FS_Status_t IMUSensor_ReadAccel(SensorData_t *data)
{
    uint8_t raw_data[6];
    
    if (ReadI2CSensor(IMU_SENSOR_ADDR, 0x3B, raw_data, 6) != FS_OK) {
        return FS_ERROR;
    }

    data->type = SENSOR_ACCELEROMETER;
    data->timestamp = HAL_GetTick();
    data->status = SENSOR_STATUS_OK;
    
    /* Convert raw data to m/s² */
    int16_t ax = (raw_data[0] << 8) | raw_data[1];
    int16_t ay = (raw_data[2] << 8) | raw_data[3];
    int16_t az = (raw_data[4] << 8) | raw_data[5];
    
    data->data.accelerometer.x = (float)ax / 4096.0f * 9.81f - accel_offset[0];
    data->data.accelerometer.y = (float)ay / 4096.0f * 9.81f - accel_offset[1];
    data->data.accelerometer.z = (float)az / 4096.0f * 9.81f - accel_offset[2];
    
    data->confidence = 0.92f;
    
    return FS_OK;
}

/**
  * @brief  Read gyroscope data
  * @param  data: Pointer to sensor data structure
  * @retval FS_Status_t
  */
FS_Status_t IMUSensor_ReadGyro(SensorData_t *data)
{
    uint8_t raw_data[6];
    
    if (ReadI2CSensor(IMU_SENSOR_ADDR, 0x43, raw_data, 6) != FS_OK) {
        return FS_ERROR;
    }

    data->type = SENSOR_GYROSCOPE;
    data->timestamp = HAL_GetTick();
    data->status = SENSOR_STATUS_OK;
    
    /* Convert raw data to rad/s */
    int16_t gx = (raw_data[0] << 8) | raw_data[1];
    int16_t gy = (raw_data[2] << 8) | raw_data[3];
    int16_t gz = (raw_data[4] << 8) | raw_data[5];
    
    data->data.gyroscope.x = (float)gx / 32.8f * M_PI / 180.0f - gyro_offset[0];
    data->data.gyroscope.y = (float)gy / 32.8f * M_PI / 180.0f - gyro_offset[1];
    data->data.gyroscope.z = (float)gz / 32.8f * M_PI / 180.0f - gyro_offset[2];
    
    data->confidence = 0.88f;
    
    return FS_OK;
}

/**
  * @brief  Initialize sensor hardware
  * @retval FS_Status_t
  */
static FS_Status_t InitializeSensorHardware(void)
{
    /* Power on sensors */
    HAL_GPIO_WritePin(SENSOR_POWER_GPIO_Port, SENSOR_POWER_Pin, GPIO_PIN_SET);
    HAL_Delay(100);  // Allow sensors to stabilize
    
    return FS_OK;
}

/**
  * @brief  Read data from I2C sensor
  * @param  addr: I2C device address
  * @param  reg: Register address
  * @param  data: Data buffer
  * @param  len: Data length
  * @retval FS_Status_t
  */
static FS_Status_t ReadI2CSensor(uint16_t addr, uint8_t reg, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;
    
    if (xSemaphoreTake(I2CMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return FS_TIMEOUT;
    }
    
    status = HAL_I2C_Mem_Read(&hi2c1, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
    
    xSemaphoreGive(I2CMutex);
    
    return (status == HAL_OK) ? FS_OK : FS_ERROR;
}

/**
  * @brief  Write data to I2C sensor
  * @param  addr: I2C device address  
  * @param  reg: Register address
  * @param  data: Data buffer
  * @param  len: Data length
  * @retval FS_Status_t
  */
static FS_Status_t WriteI2CSensor(uint16_t addr, uint8_t reg, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;
    
    if (xSemaphoreTake(I2CMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return FS_TIMEOUT;
    }
    
    status = HAL_I2C_Mem_Write(&hi2c1, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
    
    xSemaphoreGive(I2CMutex);
    
    return (status == HAL_OK) ? FS_OK : FS_ERROR;
}

/**
  * @brief  Process sensor data (filtering, validation)
  * @param  data: Pointer to sensor data
  * @retval None
  */
static void ProcessSensorData(SensorData_t *data)
{
    /* Apply digital filter if enabled */
    if (sensor_configs[data->type].filter_enable) {
        SensorManager_FilterData(data);
    }
    
    /* Apply calibration if enabled */
    if (sensor_configs[data->type].calibration_enable) {
        SensorManager_ApplyCalibration(data, data->type);
    }
    
    /* Validate data integrity */
    if (!SensorManager_IsDataValid(data)) {
        data->status = SENSOR_STATUS_ERROR;
        data->confidence = 0.0f;
    }
}

/**
  * @brief  Check if sensor data is valid
  * @param  data: Pointer to sensor data
  * @retval bool: true if valid, false otherwise
  */
bool SensorManager_IsDataValid(SensorData_t *data)
{
    /* Basic sanity checks */
    if (data == NULL) return false;
    if (data->confidence < 0.0f || data->confidence > 1.0f) return false;
    
    /* Type-specific validation */
    switch (data->type) {
        case SENSOR_PRESSURE:
            /* Check for reasonable pressure values */
            for (int i = 0; i < 8; i++) {
                if (data->data.pressure.arch_pressure[i] < -100.0f || 
                    data->data.pressure.arch_pressure[i] > 1000.0f) {
                    return false;
                }
            }
            break;
            
        case SENSOR_HUMIDITY:
            if (data->data.environment.humidity < 0.0f || 
                data->data.environment.humidity > 100.0f) {
                return false;
            }
            if (data->data.environment.temperature < -40.0f || 
                data->data.environment.temperature > 85.0f) {
                return false;
            }
            break;
            
        case SENSOR_ACCELEROMETER:
            /* Check for reasonable acceleration values (up to 8g) */
            float mag = sqrtf(data->data.accelerometer.x * data->data.accelerometer.x +
                             data->data.accelerometer.y * data->data.accelerometer.y +
                             data->data.accelerometer.z * data->data.accelerometer.z);
            if (mag > 80.0f) return false;  // 8g * 9.81 m/s²
            break;
            
        default:
            break;
    }
    
    return true;
}

/**
  * @brief  Apply digital filter to sensor data
  * @param  data: Pointer to sensor data
  * @retval None
  */
void SensorManager_FilterData(SensorData_t *data)
{
    /* Simple low-pass filter implementation */
    static float alpha = 0.8f;  // Filter coefficient
    static SensorData_t prev_data[SENSOR_COUNT];
    
    if (data->type < SENSOR_COUNT) {
        switch (data->type) {
            case SENSOR_ACCELEROMETER:
                data->data.accelerometer.x = alpha * prev_data[data->type].data.accelerometer.x + 
                                            (1.0f - alpha) * data->data.accelerometer.x;
                data->data.accelerometer.y = alpha * prev_data[data->type].data.accelerometer.y + 
                                            (1.0f - alpha) * data->data.accelerometer.y;
                data->data.accelerometer.z = alpha * prev_data[data->type].data.accelerometer.z + 
                                            (1.0f - alpha) * data->data.accelerometer.z;
                break;
                
            case SENSOR_GYROSCOPE:
                data->data.gyroscope.x = alpha * prev_data[data->type].data.gyroscope.x + 
                                        (1.0f - alpha) * data->data.gyroscope.x;
                data->data.gyroscope.y = alpha * prev_data[data->type].data.gyroscope.y + 
                                        (1.0f - alpha) * data->data.gyroscope.y;
                data->data.gyroscope.z = alpha * prev_data[data->type].data.gyroscope.z + 
                                        (1.0f - alpha) * data->data.gyroscope.z;
                break;
                
            default:
                /* No filtering for other sensor types */
                break;
        }
        
        /* Store current data for next iteration */
        prev_data[data->type] = *data;
    }
}

/**
  * @brief  Apply calibration to sensor data
  * @param  data: Pointer to sensor data
  * @param  sensor: Sensor type
  * @retval None
  */
void SensorManager_ApplyCalibration(SensorData_t *data, SensorType_t sensor)
{
    /* Apply stored calibration offsets */
    switch (sensor) {
        case SENSOR_ACCELEROMETER:
            data->data.accelerometer.x -= accel_offset[0];
            data->data.accelerometer.y -= accel_offset[1];
            data->data.accelerometer.z -= accel_offset[2];
            break;
            
        case SENSOR_GYROSCOPE:
            data->data.gyroscope.x -= gyro_offset[0];
            data->data.gyroscope.y -= gyro_offset[1];
            data->data.gyroscope.z -= gyro_offset[2];
            break;
            
        default:
            break;
    }
}

/**
  * @brief  Get sensor status
  * @param  sensor: Sensor type
  * @retval SensorStatus_t
  */
SensorStatus_t SensorManager_GetSensorStatus(SensorType_t sensor)
{
    if (sensor < SENSOR_COUNT) {
        return sensor_status[sensor];
    }
    return SENSOR_STATUS_ERROR;
}

/**
  * @brief  Get sensor sample count
  * @param  sensor: Sensor type
  * @retval uint32_t: Sample count
  */
uint32_t SensorManager_GetSampleCount(SensorType_t sensor)
{
    if (sensor < SENSOR_COUNT) {
        return sensor_sample_count[sensor];
    }
    return 0;
}

/**
  * @brief  Get sensor health metric
  * @param  sensor: Sensor type
  * @retval float: Health metric (0.0-1.0)
  */
float SensorManager_GetSensorHealth(SensorType_t sensor)
{
    if (sensor < SENSOR_COUNT) {
        return sensor_health[sensor];
    }
    return 0.0f;
}