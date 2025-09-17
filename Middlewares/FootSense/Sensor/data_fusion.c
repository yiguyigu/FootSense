/**
  ******************************************************************************
  * @file    data_fusion.c
  * @brief   Multi-sensor data fusion system implementation
  ******************************************************************************
  * @attention
  *
  * FootSense - Multi-sensor fusion for foot health monitoring
  * Copyright (c) 2025 Yigu
  *
  ******************************************************************************
  */

#include "data_fusion.h"
#include "main.h"
#include <string.h>
#include <math.h>

/* Private defines */
#define MAX_SENSOR_BUFFER_SIZE    50
#define FUSION_UPDATE_RATE_HZ     25
#define PRESSURE_THRESHOLD        50.0f   // Newtons
#define GAIT_CYCLE_THRESHOLD      0.5f    // seconds
#define ARCH_COLLAPSE_THRESHOLD   0.3f    // ratio

/* Private structures */
typedef struct {
    SensorData_t buffer[MAX_SENSOR_BUFFER_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} SensorBuffer_t;

/* Private variables */
static FusionConfig_t fusion_config;
static FusedData_t current_fused_data;
static SensorBuffer_t sensor_buffers[SENSOR_COUNT];
static bool fusion_initialized = false;
static uint32_t processed_sample_count = 0;

/* Kalman filter state variables */
static float kalman_state[9];        // [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z]
static float kalman_covariance[9][9];
static float kalman_process_noise[9][9];
static float kalman_measurement_noise[3][3];

/* Gait cycle detection */
static float prev_acceleration_magnitude = 0.0f;
static uint32_t last_step_time = 0;
static uint8_t step_count = 0;
static float step_times[20];  // Store last 20 step times

/* Private function prototypes */
static void InitializeKalmanFilter(void);
static void UpdateKalmanFilter(float measurements[3], float dt);
static void InitializeSensorBuffers(void);
static bool AddToSensorBuffer(SensorType_t type, SensorData_t *data);
static bool GetLatestSensorData(SensorType_t type, SensorData_t *data);
static void ProcessPressureData(SensorData_t *pressure_data);
static void ProcessIMUData(SensorData_t *imu_data);
static void ProcessEnvironmentData(SensorData_t *env_data);
static float CalculateStepFrequency(void);
static void UpdateGaitMetrics(float acceleration_magnitude);

/**
  * @brief  Initialize the data fusion system
  * @retval FS_Status_t
  */
FS_Status_t DataFusion_Init(void)
{
    if (fusion_initialized) {
        return FS_OK;
    }

    FS_DEBUG_PRINTF("Initializing Data Fusion System...");

    /* Initialize default configuration */
    fusion_config.algorithm = FUSION_KALMAN;
    fusion_config.update_rate = FUSION_UPDATE_RATE_HZ;
    fusion_config.adaptive_weighting = true;
    fusion_config.confidence_threshold = 0.7f;

    /* Set default sensor weights */
    fusion_config.sensor_weights[SENSOR_PRESSURE] = 0.3f;
    fusion_config.sensor_weights[SENSOR_ACCELEROMETER] = 0.25f;
    fusion_config.sensor_weights[SENSOR_GYROSCOPE] = 0.2f;
    fusion_config.sensor_weights[SENSOR_HUMIDITY] = 0.15f;
    fusion_config.sensor_weights[SENSOR_TEMPERATURE] = 0.1f;

    /* Initialize sensor buffers */
    InitializeSensorBuffers();

    /* Initialize Kalman filter */
    InitializeKalmanFilter();

    /* Initialize fused data structure */
    memset(&current_fused_data, 0, sizeof(FusedData_t));
    current_fused_data.fusion_confidence = 0.0f;
    current_fused_data.valid_sensors = 0;

    /* Initialize step detection variables */
    memset(step_times, 0, sizeof(step_times));
    step_count = 0;
    last_step_time = 0;

    fusion_initialized = true;
    processed_sample_count = 0;

    FS_DEBUG_PRINTF("Data Fusion System initialized successfully");
    return FS_OK;
}

/**
  * @brief  Process incoming sensor data
  * @param  sensor_data: Pointer to sensor data
  * @retval FS_Status_t
  */
FS_Status_t DataFusion_ProcessSensorData(SensorData_t *sensor_data)
{
    if (!fusion_initialized || sensor_data == NULL) {
        return FS_ERROR;
    }

    /* Add data to appropriate sensor buffer */
    if (!AddToSensorBuffer(sensor_data->type, sensor_data)) {
        return FS_ERROR;
    }

    /* Process data based on sensor type */
    switch (sensor_data->type) {
        case SENSOR_PRESSURE:
            ProcessPressureData(sensor_data);
            break;
            
        case SENSOR_ACCELEROMETER:
        case SENSOR_GYROSCOPE:
        case SENSOR_MAGNETOMETER:
            ProcessIMUData(sensor_data);
            break;
            
        case SENSOR_HUMIDITY:
        case SENSOR_TEMPERATURE:
            ProcessEnvironmentData(sensor_data);
            break;
            
        default:
            return FS_ERROR;
    }

    /* Update valid sensors bitmask */
    current_fused_data.valid_sensors |= (1 << sensor_data->type);
    current_fused_data.timestamp = sensor_data->timestamp;
    processed_sample_count++;

    /* Perform fusion if we have sufficient data */
    if (DataFusion_IsDataReady()) {
        /* Update Kalman filter with IMU data */
        SensorData_t accel_data, gyro_data;
        if (GetLatestSensorData(SENSOR_ACCELEROMETER, &accel_data) &&
            GetLatestSensorData(SENSOR_GYROSCOPE, &gyro_data)) {
            
            float measurements[3] = {
                accel_data.data.accelerometer.x,
                accel_data.data.accelerometer.y,
                accel_data.data.accelerometer.z
            };
            
            float dt = 1.0f / fusion_config.update_rate;
            UpdateKalmanFilter(measurements, dt);
        }

        /* Calculate health metrics */
        CalculateFootHealthMetrics(&current_fused_data, &current_fused_data.health_metrics);
        
        /* Update fusion confidence */
        current_fused_data.fusion_confidence = DataFusion_GetDataQuality();
    }

    return FS_OK;
}

/**
  * @brief  Get the latest fused data
  * @param  fused_data: Pointer to fused data structure
  * @retval FS_Status_t
  */
FS_Status_t DataFusion_GetFusedData(FusedData_t *fused_data)
{
    if (!fusion_initialized || fused_data == NULL) {
        return FS_ERROR;
    }

    /* Copy current fused data */
    memcpy(fused_data, &current_fused_data, sizeof(FusedData_t));
    
    return FS_OK;
}

/**
  * @brief  Analyze pressure distribution
  * @param  pressure_data: Pointer to pressure sensor data
  * @param  pressure_map: Pointer to pressure map structure
  * @retval FS_Status_t
  */
FS_Status_t AnalyzePressureDistribution(SensorData_t *pressure_data, PressureMap_t *pressure_map)
{
    if (pressure_data == NULL || pressure_map == NULL) {
        return FS_ERROR;
    }

    /* Clear pressure map */
    memset(pressure_map, 0, sizeof(PressureMap_t));
    pressure_map->timestamp = pressure_data->timestamp;

    /* Calculate total force and center of pressure */
    float total_force = 0.0f;
    float weighted_x = 0.0f;
    float weighted_y = 0.0f;
    float contact_area = 0.0f;

    /* Process arch pressure sensors */
    for (int i = 0; i < 8; i++) {
        float pressure = pressure_data->data.pressure.arch_pressure[i];
        if (pressure > PRESSURE_THRESHOLD) {
            total_force += pressure;
            contact_area += 1.0f;  // Simplified area calculation
            
            /* Map to pressure grid (arch region) */
            int x = 10 + (i % 4) * 3;  // Middle section of foot
            int y = 8 + (i / 4) * 4;
            pressure_map->pressure_map[x][y] = pressure;
            
            weighted_x += x * pressure;
            weighted_y += y * pressure;
        }
    }

    /* Process heel pressure sensors */
    for (int i = 0; i < 4; i++) {
        float pressure = pressure_data->data.pressure.heel_pressure[i];
        if (pressure > PRESSURE_THRESHOLD) {
            total_force += pressure;
            contact_area += 1.0f;
            
            /* Map to pressure grid (heel region) */
            int x = 2 + (i % 2) * 4;   // Back section of foot
            int y = 10 + (i / 2) * 4;
            pressure_map->pressure_map[x][y] = pressure;
            
            weighted_x += x * pressure;
            weighted_y += y * pressure;
        }
    }

    /* Process toe pressure sensors */
    for (int i = 0; i < 4; i++) {
        float pressure = pressure_data->data.pressure.toe_pressure[i];
        if (pressure > PRESSURE_THRESHOLD) {
            total_force += pressure;
            contact_area += 1.0f;
            
            /* Map to pressure grid (toe region) */
            int x = 25 + (i % 2) * 3;  // Front section of foot
            int y = 8 + (i / 2) * 8;
            pressure_map->pressure_map[x][y] = pressure;
            
            weighted_x += x * pressure;
            weighted_y += y * pressure;
        }
    }

    /* Calculate center of pressure */
    if (total_force > 0.0f) {
        pressure_map->center_of_pressure_x = weighted_x / total_force;
        pressure_map->center_of_pressure_y = weighted_y / total_force;
    }

    pressure_map->total_force = total_force;
    pressure_map->contact_area = contact_area * 2.0f;  // Convert to cm² (approximate)

    return FS_OK;
}

/**
  * @brief  Analyze gait pattern
  * @param  imu_data: Pointer to IMU sensor data
  * @param  gait_analysis: Pointer to gait analysis structure
  * @retval FS_Status_t
  */
FS_Status_t AnalyzeGaitPattern(SensorData_t *imu_data, GaitAnalysis_t *gait_analysis)
{
    if (imu_data == NULL || gait_analysis == NULL) {
        return FS_ERROR;
    }

    /* Calculate acceleration magnitude for step detection */
    float accel_magnitude = 0.0f;
    
    if (imu_data->type == SENSOR_ACCELEROMETER) {
        accel_magnitude = sqrtf(
            imu_data->data.accelerometer.x * imu_data->data.accelerometer.x +
            imu_data->data.accelerometer.y * imu_data->data.accelerometer.y +
            imu_data->data.accelerometer.z * imu_data->data.accelerometer.z
        );
        
        UpdateGaitMetrics(accel_magnitude);
    }

    /* Calculate gait parameters */
    gait_analysis->step_frequency = CalculateStepFrequency();
    gait_analysis->cadence = gait_analysis->step_frequency * 60.0f;  // steps per minute
    
    if (gait_analysis->step_frequency > 0.0f) {
        gait_analysis->step_length = current_fused_data.linear_acceleration[0] / 
                                   (gait_analysis->step_frequency * gait_analysis->step_frequency);
        gait_analysis->velocity = gait_analysis->step_length * gait_analysis->step_frequency;
    }

    /* Estimate stance and swing times (simplified) */
    if (gait_analysis->step_frequency > 0.0f) {
        float cycle_time = 1.0f / gait_analysis->step_frequency;
        gait_analysis->stance_time = cycle_time * 0.6f;    // Typically 60% of cycle
        gait_analysis->swing_time = cycle_time * 0.4f;     // Typically 40% of cycle
        gait_analysis->double_support_time = cycle_time * 0.1f;  // Typically 10% of cycle
    }

    return FS_OK;
}

/**
  * @brief  Calculate foot health metrics
  * @param  fused_data: Pointer to fused data
  * @param  metrics: Pointer to health metrics structure
  * @retval FS_Status_t
  */
FS_Status_t CalculateFootHealthMetrics(FusedData_t *fused_data, FootHealthMetrics_t *metrics)
{
    if (fused_data == NULL || metrics == NULL) {
        return FS_ERROR;
    }

    /* Analyze pressure distribution for arch support */
    float arch_pressure_total = 0.0f;
    float total_pressure = 0.0f;
    
    for (int x = 8; x < 20; x++) {  // Arch region
        for (int y = 6; y < 18; y++) {
            arch_pressure_total += fused_data->pressure_map.pressure_map[x][y];
        }
    }
    
    for (int x = 0; x < 32; x++) {  // Total foot
        for (int y = 0; y < 24; y++) {
            total_pressure += fused_data->pressure_map.pressure_map[x][y];
        }
    }
    
    if (total_pressure > 0.0f) {
        metrics->arch_support_index = 1.0f - (arch_pressure_total / total_pressure);
        metrics->arch_support_index = fmaxf(0.0f, fminf(1.0f, metrics->arch_support_index));
    }

    /* Calculate pressure distribution uniformity */
    float pressure_variance = 0.0f;
    float pressure_mean = total_pressure / (32.0f * 24.0f);
    int active_cells = 0;
    
    for (int x = 0; x < 32; x++) {
        for (int y = 0; y < 24; y++) {
            if (fused_data->pressure_map.pressure_map[x][y] > 0.0f) {
                float diff = fused_data->pressure_map.pressure_map[x][y] - pressure_mean;
                pressure_variance += diff * diff;
                active_cells++;
            }
        }
    }
    
    if (active_cells > 0) {
        pressure_variance /= active_cells;
        metrics->pressure_distribution = 1.0f / (1.0f + pressure_variance / 100.0f);
    }

    /* Calculate gait symmetry based on step timing consistency */
    float step_time_variance = 0.0f;
    float step_time_mean = 0.0f;
    int valid_steps = 0;
    
    for (int i = 1; i < 20 && step_times[i] > 0; i++) {
        float step_duration = step_times[i] - step_times[i-1];
        if (step_duration > 0.1f && step_duration < 2.0f) {  // Reasonable step duration
            step_time_mean += step_duration;
            valid_steps++;
        }
    }
    
    if (valid_steps > 0) {
        step_time_mean /= valid_steps;
        
        for (int i = 1; i < 20 && step_times[i] > 0; i++) {
            float step_duration = step_times[i] - step_times[i-1];
            if (step_duration > 0.1f && step_duration < 2.0f) {
                float diff = step_duration - step_time_mean;
                step_time_variance += diff * diff;
            }
        }
        
        step_time_variance /= valid_steps;
        metrics->gait_symmetry = 1.0f / (1.0f + step_time_variance * 10.0f);
    }

    /* Calculate balance stability from gyroscope data */
    float angular_velocity_magnitude = sqrtf(
        fused_data->angular_velocity[0] * fused_data->angular_velocity[0] +
        fused_data->angular_velocity[1] * fused_data->angular_velocity[1] +
        fused_data->angular_velocity[2] * fused_data->angular_velocity[2]
    );
    
    metrics->balance_stability = 1.0f / (1.0f + angular_velocity_magnitude);

    /* Calculate perspiration level from humidity */
    metrics->perspiration_level = fminf(1.0f, fmaxf(0.0f, (fused_data->humidity - 40.0f) / 40.0f));

    /* Calculate activity intensity from acceleration */
    float accel_magnitude = sqrtf(
        fused_data->linear_acceleration[0] * fused_data->linear_acceleration[0] +
        fused_data->linear_acceleration[1] * fused_data->linear_acceleration[1] +
        fused_data->linear_acceleration[2] * fused_data->linear_acceleration[2]
    );
    
    metrics->activity_intensity = fminf(1.0f, accel_magnitude / 20.0f);  // Normalize to 20 m/s²

    /* Set health status flags */
    fused_data->arch_collapse_risk = DetectArchCollapse(&fused_data->pressure_map);
    fused_data->excessive_pronation = DetectExcessivePronation(&fused_data->gait_analysis);
    fused_data->pressure_hotspots = DetectPressureHotspots(&fused_data->pressure_map);
    fused_data->gait_abnormality = DetectGaitAbnormality(&fused_data->gait_analysis);

    return FS_OK;
}

/**
  * @brief  Initialize Kalman filter
  * @retval None
  */
static void InitializeKalmanFilter(void)
{
    /* Initialize state vector to zero */
    memset(kalman_state, 0, sizeof(kalman_state));
    
    /* Initialize covariance matrix */
    memset(kalman_covariance, 0, sizeof(kalman_covariance));
    for (int i = 0; i < 9; i++) {
        kalman_covariance[i][i] = 1.0f;  // Unit diagonal
    }
    
    /* Initialize process noise matrix */
    memset(kalman_process_noise, 0, sizeof(kalman_process_noise));
    for (int i = 0; i < 9; i++) {
        kalman_process_noise[i][i] = 0.01f;  // Small process noise
    }
    
    /* Initialize measurement noise matrix */
    memset(kalman_measurement_noise, 0, sizeof(kalman_measurement_noise));
    for (int i = 0; i < 3; i++) {
        kalman_measurement_noise[i][i] = 0.1f;  // Measurement noise
    }
}

/**
  * @brief  Update Kalman filter with new measurements
  * @param  measurements: Array of 3 acceleration measurements
  * @param  dt: Time delta in seconds
  * @retval None
  */
static void UpdateKalmanFilter(float measurements[3], float dt)
{
    /* Simplified Kalman filter update for position, velocity, and acceleration */
    /* This is a basic implementation - a full implementation would include */
    /* proper matrix operations for state transition and covariance updates */
    
    /* Predict step */
    for (int i = 0; i < 3; i++) {
        /* Update position: pos += vel * dt + 0.5 * acc * dt^2 */
        kalman_state[i] += kalman_state[i+3] * dt + 0.5f * kalman_state[i+6] * dt * dt;
        
        /* Update velocity: vel += acc * dt */
        kalman_state[i+3] += kalman_state[i+6] * dt;
        
        /* Acceleration remains the same in predict step */
    }
    
    /* Update step with measurements */
    for (int i = 0; i < 3; i++) {
        /* Simple update: blend measurement with predicted acceleration */
        float innovation = measurements[i] - kalman_state[i+6];
        kalman_state[i+6] += 0.1f * innovation;  // Kalman gain approximation
    }
    
    /* Store filtered values in fused data */
    memcpy(current_fused_data.linear_acceleration, &kalman_state[6], 3 * sizeof(float));
}

/**
  * @brief  Initialize sensor buffers
  * @retval None
  */
static void InitializeSensorBuffers(void)
{
    for (int i = 0; i < SENSOR_COUNT; i++) {
        sensor_buffers[i].head = 0;
        sensor_buffers[i].tail = 0;
        sensor_buffers[i].count = 0;
        memset(sensor_buffers[i].buffer, 0, sizeof(sensor_buffers[i].buffer));
    }
}

/**
  * @brief  Add sensor data to buffer
  * @param  type: Sensor type
  * @param  data: Sensor data
  * @retval bool: true if successful
  */
static bool AddToSensorBuffer(SensorType_t type, SensorData_t *data)
{
    if (type >= SENSOR_COUNT || data == NULL) {
        return false;
    }
    
    SensorBuffer_t *buffer = &sensor_buffers[type];
    
    /* Add data to buffer */
    buffer->buffer[buffer->head] = *data;
    buffer->head = (buffer->head + 1) % MAX_SENSOR_BUFFER_SIZE;
    
    if (buffer->count < MAX_SENSOR_BUFFER_SIZE) {
        buffer->count++;
    } else {
        /* Buffer full, advance tail */
        buffer->tail = (buffer->tail + 1) % MAX_SENSOR_BUFFER_SIZE;
    }
    
    return true;
}

/**
  * @brief  Get latest sensor data from buffer
  * @param  type: Sensor type
  * @param  data: Pointer to store sensor data
  * @retval bool: true if data available
  */
static bool GetLatestSensorData(SensorType_t type, SensorData_t *data)
{
    if (type >= SENSOR_COUNT || data == NULL) {
        return false;
    }
    
    SensorBuffer_t *buffer = &sensor_buffers[type];
    
    if (buffer->count == 0) {
        return false;
    }
    
    /* Get most recent data */
    uint8_t latest_index = (buffer->head == 0) ? (MAX_SENSOR_BUFFER_SIZE - 1) : (buffer->head - 1);
    *data = buffer->buffer[latest_index];
    
    return true;
}

/**
  * @brief  Process pressure sensor data
  * @param  pressure_data: Pressure sensor data
  * @retval None
  */
static void ProcessPressureData(SensorData_t *pressure_data)
{
    /* Analyze pressure distribution and update pressure map */
    AnalyzePressureDistribution(pressure_data, &current_fused_data.pressure_map);
}

/**
  * @brief  Process IMU sensor data
  * @param  imu_data: IMU sensor data
  * @retval None
  */
static void ProcessIMUData(SensorData_t *imu_data)
{
    /* Update orientation and motion data */
    if (imu_data->type == SENSOR_ACCELEROMETER) {
        /* Calculate tilt angles from accelerometer */
        float ax = imu_data->data.accelerometer.x;
        float ay = imu_data->data.accelerometer.y;
        float az = imu_data->data.accelerometer.z;
        
        current_fused_data.orientation[0] = atan2f(ay, az);  // Roll
        current_fused_data.orientation[1] = atan2f(-ax, sqrtf(ay*ay + az*az));  // Pitch
        
        /* Analyze gait pattern */
        AnalyzeGaitPattern(imu_data, &current_fused_data.gait_analysis);
    } else if (imu_data->type == SENSOR_GYROSCOPE) {
        /* Update angular velocity */
        current_fused_data.angular_velocity[0] = imu_data->data.gyroscope.x;
        current_fused_data.angular_velocity[1] = imu_data->data.gyroscope.y;
        current_fused_data.angular_velocity[2] = imu_data->data.gyroscope.z;
    }
}

/**
  * @brief  Process environment sensor data
  * @param  env_data: Environment sensor data
  * @retval None
  */
static void ProcessEnvironmentData(SensorData_t *env_data)
{
    if (env_data->type == SENSOR_HUMIDITY) {
        current_fused_data.humidity = env_data->data.environment.humidity;
        current_fused_data.temperature = env_data->data.environment.temperature;
    }
}

/**
  * @brief  Calculate step frequency
  * @retval float: Step frequency in Hz
  */
static float CalculateStepFrequency(void)
{
    float total_time = 0.0f;
    int valid_intervals = 0;
    
    for (int i = 1; i < 20 && step_times[i] > 0; i++) {
        float interval = step_times[i] - step_times[i-1];
        if (interval > 0.1f && interval < 2.0f) {  // Reasonable step interval
            total_time += interval;
            valid_intervals++;
        }
    }
    
    if (valid_intervals > 0) {
        float avg_interval = total_time / valid_intervals;
        return 1.0f / avg_interval;  // Convert to frequency
    }
    
    return 0.0f;
}

/**
  * @brief  Update gait metrics with new acceleration data
  * @param  acceleration_magnitude: Current acceleration magnitude
  * @retval None
  */
static void UpdateGaitMetrics(float acceleration_magnitude)
{
    static float threshold = 10.0f;  // m/s² threshold for step detection
    static bool step_detected = false;
    
    /* Simple step detection using acceleration peaks */
    if (!step_detected && acceleration_magnitude > threshold && 
        prev_acceleration_magnitude <= threshold) {
        
        uint32_t current_time = HAL_GetTick();
        
        if (last_step_time > 0 && (current_time - last_step_time) > 200) {  // Minimum 200ms between steps
            /* Store step time */
            for (int i = 19; i > 0; i--) {
                step_times[i] = step_times[i-1];
            }
            step_times[0] = current_time / 1000.0f;  // Convert to seconds
            
            last_step_time = current_time;
            step_count++;
        }
        
        step_detected = true;
    } else if (step_detected && acceleration_magnitude <= threshold) {
        step_detected = false;
    }
    
    prev_acceleration_magnitude = acceleration_magnitude;
}

/**
  * @brief  Check if sufficient data is available for fusion
  * @retval bool: true if data is ready
  */
bool DataFusion_IsDataReady(void)
{
    /* Check if we have recent data from key sensors */
    uint32_t required_sensors = (1 << SENSOR_PRESSURE) | (1 << SENSOR_ACCELEROMETER);
    return (current_fused_data.valid_sensors & required_sensors) == required_sensors;
}

/**
  * @brief  Get overall data quality metric
  * @retval float: Data quality (0.0-1.0)
  */
float DataFusion_GetDataQuality(void)
{
    float quality = 0.0f;
    int active_sensors = 0;
    
    /* Count active sensors and average their confidence */
    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (current_fused_data.valid_sensors & (1 << i)) {
            SensorData_t data;
            if (GetLatestSensorData(i, &data)) {
                quality += data.confidence * fusion_config.sensor_weights[i];
                active_sensors++;
            }
        }
    }
    
    if (active_sensors > 0) {
        return quality;
    }
    
    return 0.0f;
}

/**
  * @brief  Get processed sample count
  * @retval uint32_t: Number of processed samples
  */
uint32_t DataFusion_GetProcessedSampleCount(void)
{
    return processed_sample_count;
}

/**
  * @brief  Detect arch collapse risk
  * @param  pressure_map: Pressure map data
  * @retval bool: true if arch collapse risk detected
  */
bool DetectArchCollapse(PressureMap_t *pressure_map)
{
    float arch_pressure = 0.0f;
    float total_pressure = 0.0f;
    
    /* Check pressure in arch region vs total pressure */
    for (int x = 8; x < 20; x++) {  // Arch region
        for (int y = 6; y < 18; y++) {
            arch_pressure += pressure_map->pressure_map[x][y];
        }
    }
    
    total_pressure = pressure_map->total_force;
    
    if (total_pressure > 0.0f) {
        float arch_ratio = arch_pressure / total_pressure;
        return arch_ratio > ARCH_COLLAPSE_THRESHOLD;
    }
    
    return false;
}

/**
  * @brief  Detect excessive pronation
  * @param  gait_analysis: Gait analysis data
  * @retval bool: true if excessive pronation detected
  */
bool DetectExcessivePronation(GaitAnalysis_t *gait_analysis)
{
    /* Simplified detection based on stance time asymmetry */
    return gait_analysis->stance_time > 0.8f;  // More than 80% of cycle
}

/**
  * @brief  Detect pressure hotspots
  * @param  pressure_map: Pressure map data
  * @retval bool: true if pressure hotspots detected
  */
bool DetectPressureHotspots(PressureMap_t *pressure_map)
{
    float max_pressure = 0.0f;
    float avg_pressure = 0.0f;
    int active_cells = 0;
    
    /* Find maximum and average pressure */
    for (int x = 0; x < 32; x++) {
        for (int y = 0; y < 24; y++) {
            float pressure = pressure_map->pressure_map[x][y];
            if (pressure > 0.0f) {
                max_pressure = fmaxf(max_pressure, pressure);
                avg_pressure += pressure;
                active_cells++;
            }
        }
    }
    
    if (active_cells > 0) {
        avg_pressure /= active_cells;
        return max_pressure > (avg_pressure * 3.0f);  // Hotspot if >3x average
    }
    
    return false;
}

/**
  * @brief  Detect gait abnormality
  * @param  gait_analysis: Gait analysis data
  * @retval bool: true if gait abnormality detected
  */
bool DetectGaitAbnormality(GaitAnalysis_t *gait_analysis)
{
    /* Check for abnormal cadence */
    if (gait_analysis->cadence < 60.0f || gait_analysis->cadence > 200.0f) {
        return true;
    }
    
    /* Check for abnormal step length */
    if (gait_analysis->step_length < 0.3f || gait_analysis->step_length > 1.5f) {
        return true;
    }
    
    return false;
}