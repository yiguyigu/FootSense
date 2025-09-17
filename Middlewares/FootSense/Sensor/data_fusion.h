/**
  ******************************************************************************
  * @file    data_fusion.h
  * @brief   Header file for multi-sensor data fusion system
  ******************************************************************************
  * @attention
  *
  * FootSense - Multi-sensor fusion for foot health monitoring
  * Copyright (c) 2025 Yigu
  *
  ******************************************************************************
  */

#ifndef __DATA_FUSION_H
#define __DATA_FUSION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sensor_manager.h"
#include <stdint.h>
#include <stdbool.h>

/* Fusion Algorithm Types */
typedef enum {
    FUSION_KALMAN = 0,
    FUSION_COMPLEMENTARY,
    FUSION_WEIGHTED_AVERAGE,
    FUSION_NEURAL_NETWORK
} FusionAlgorithm_t;

/* Foot Health Metrics */
typedef struct {
    float arch_support_index;      // 0.0-1.0 (1.0 = perfect arch support)
    float pressure_distribution;   // 0.0-1.0 (1.0 = optimal distribution)
    float gait_symmetry;          // 0.0-1.0 (1.0 = perfect symmetry)
    float balance_stability;      // 0.0-1.0 (1.0 = excellent balance)
    float perspiration_level;     // 0.0-1.0 (1.0 = excessive perspiration)
    float activity_intensity;     // 0.0-1.0 (1.0 = high intensity)
} FootHealthMetrics_t;

/* Gait Analysis Data */
typedef struct {
    float step_length;            // meters
    float step_frequency;         // steps per minute
    float stance_time;            // seconds
    float swing_time;             // seconds
    float double_support_time;    // seconds
    float velocity;              // m/s
    float cadence;               // steps/min
} GaitAnalysis_t;

/* Foot Pressure Map */
typedef struct {
    float pressure_map[32][24];   // 32x24 grid representing foot pressure
    float total_force;            // Total force in Newtons
    float center_of_pressure_x;   // CoP X coordinate
    float center_of_pressure_y;   // CoP Y coordinate
    float contact_area;           // Contact area in cm²
    uint32_t timestamp;
} PressureMap_t;

/* Fused Sensor Data */
typedef struct {
    FootHealthMetrics_t health_metrics;
    GaitAnalysis_t gait_analysis;
    PressureMap_t pressure_map;
    
    /* Environmental data */
    float temperature;
    float humidity;
    
    /* Motion data */
    float orientation[3];         // Roll, Pitch, Yaw in radians
    float linear_acceleration[3]; // m/s² in world frame
    float angular_velocity[3];    // rad/s
    
    /* Quality indicators */
    float fusion_confidence;      // Overall confidence in fused data
    uint32_t valid_sensors;       // Bitmask of active sensors
    uint32_t timestamp;
    
    /* Health status indicators */
    bool arch_collapse_risk;
    bool excessive_pronation;
    bool pressure_hotspots;
    bool gait_abnormality;
} FusedData_t;

/* Fusion Configuration */
typedef struct {
    FusionAlgorithm_t algorithm;
    float sensor_weights[SENSOR_COUNT];
    float update_rate;            // Hz
    bool adaptive_weighting;
    float confidence_threshold;
} FusionConfig_t;

/* Function Prototypes */
FS_Status_t DataFusion_Init(void);
FS_Status_t DataFusion_DeInit(void);
FS_Status_t DataFusion_Configure(FusionConfig_t *config);

FS_Status_t DataFusion_ProcessSensorData(SensorData_t *sensor_data);
FS_Status_t DataFusion_GetFusedData(FusedData_t *fused_data);
FS_Status_t DataFusion_UpdateCalibration(void);

/* Specific fusion algorithms */
FS_Status_t DataFusion_KalmanFilter(SensorData_t inputs[], uint8_t count, FusedData_t *output);
FS_Status_t DataFusion_ComplementaryFilter(SensorData_t inputs[], uint8_t count, FusedData_t *output);
FS_Status_t DataFusion_WeightedAverage(SensorData_t inputs[], uint8_t count, FusedData_t *output);

/* Analysis functions */
FS_Status_t AnalyzePressureDistribution(SensorData_t *pressure_data, PressureMap_t *pressure_map);
FS_Status_t AnalyzeGaitPattern(SensorData_t *imu_data, GaitAnalysis_t *gait_analysis);
FS_Status_t CalculateFootHealthMetrics(FusedData_t *fused_data, FootHealthMetrics_t *metrics);

/* Utility functions */
bool DataFusion_IsDataReady(void);
float DataFusion_GetDataQuality(void);
uint32_t DataFusion_GetProcessedSampleCount(void);

/* Health assessment functions */
bool DetectArchCollapse(PressureMap_t *pressure_map);
bool DetectExcessivePronation(GaitAnalysis_t *gait_analysis);
bool DetectPressureHotspots(PressureMap_t *pressure_map);
bool DetectGaitAbnormality(GaitAnalysis_t *gait_analysis);

#ifdef __cplusplus
}
#endif

#endif /* __DATA_FUSION_H */