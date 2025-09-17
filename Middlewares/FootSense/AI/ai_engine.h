/**
  ******************************************************************************
  * @file    ai_engine.h
  * @brief   Header file for NanoEdge AI engine
  ******************************************************************************
  * @attention
  *
  * FootSense - NanoEdge AI integration for foot health monitoring
  * Copyright (c) 2025 Yigu
  *
  ******************************************************************************
  */

#ifndef __AI_ENGINE_H
#define __AI_ENGINE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "data_fusion.h"
#include <stdint.h>
#include <stdbool.h>

/* AI Model Types */
typedef enum {
    AI_MODEL_CLASSIFICATION = 0,
    AI_MODEL_ANOMALY_DETECTION,
    AI_MODEL_REGRESSION,
    AI_MODEL_COUNT
} AIModelType_t;

/* AI Classification Results */
typedef enum {
    AI_CLASS_NORMAL_GAIT = 0,
    AI_CLASS_ABNORMAL_GAIT,
    AI_CLASS_FLAT_FOOT,
    AI_CLASS_HIGH_ARCH,
    AI_CLASS_PRONATION,
    AI_CLASS_SUPINATION,
    AI_CLASS_PRESSURE_HOTSPOT,
    AI_CLASS_FATIGUE,
    AI_CLASS_COUNT
} AIClassification_t;

/* AI Confidence Levels */
typedef enum {
    AI_CONFIDENCE_LOW = 0,
    AI_CONFIDENCE_MEDIUM,
    AI_CONFIDENCE_HIGH,
    AI_CONFIDENCE_VERY_HIGH
} AIConfidenceLevel_t;

/* AI Result Structure */
typedef struct {
    AIClassification_t classification;
    float confidence;                    // 0.0-1.0
    AIConfidenceLevel_t confidence_level;
    float anomaly_score;                // 0.0-1.0 (higher = more anomalous)
    float regression_output[8];         // Multi-output regression results
    uint32_t inference_time_us;         // Inference time in microseconds
    uint32_t timestamp;
    bool is_valid;
} AIResult_t;

/* AI Input Feature Vector */
typedef struct {
    /* Pressure features (16 values) */
    float pressure_features[16];        // Normalized pressure values
    
    /* Motion features (12 values) */
    float accel_features[3];           // Normalized accelerometer data
    float gyro_features[3];            // Normalized gyroscope data
    float orientation_features[3];      // Roll, pitch, yaw
    float motion_magnitude;            // Overall motion intensity
    float step_frequency;              // Steps per second
    float gait_symmetry;               // Gait symmetry metric
    
    /* Environmental features (2 values) */
    float temperature_norm;            // Normalized temperature
    float humidity_norm;               // Normalized humidity
    
    /* Derived features (8 values) */
    float center_of_pressure_x;        // Normalized CoP X
    float center_of_pressure_y;        // Normalized CoP Y
    float total_force_norm;            // Normalized total force
    float contact_area_norm;           // Normalized contact area
    float arch_support_index;          // Arch support metric
    float pressure_distribution;       // Pressure uniformity
    float balance_stability;           // Balance metric
    float activity_intensity;          // Activity level
    
    uint32_t timestamp;
    bool is_valid;
} AIFeatureVector_t;

/* AI Model Configuration */
typedef struct {
    AIModelType_t model_type;
    uint16_t input_size;               // Number of input features
    uint16_t output_size;              // Number of output classes/values
    float inference_threshold;         // Minimum confidence for valid inference
    bool auto_learning_enabled;       // Enable online learning
    uint32_t learning_rate_fixed;     // Fixed-point learning rate
} AIModelConfig_t;

/* AI Statistics */
typedef struct {
    uint32_t total_inferences;
    uint32_t valid_inferences;
    uint32_t anomalies_detected;
    uint32_t classifications[AI_CLASS_COUNT];
    float average_confidence;
    float average_inference_time_us;
    uint32_t last_learning_update;
} AIStatistics_t;

/* Function Prototypes */
FS_Status_t AIEngine_Init(void);
FS_Status_t AIEngine_DeInit(void);
FS_Status_t AIEngine_LoadModel(AIModelType_t model_type, const uint8_t *model_data, uint32_t model_size);

FS_Status_t AIEngine_ProcessFusedData(FusedData_t *fused_data, AIResult_t *result);
FS_Status_t AIEngine_RunInference(AIFeatureVector_t *features, AIResult_t *result);
FS_Status_t AIEngine_UpdateModel(AIFeatureVector_t *features, AIClassification_t true_label);

/* Feature extraction and preprocessing */
FS_Status_t AIEngine_ExtractFeatures(FusedData_t *fused_data, AIFeatureVector_t *features);
FS_Status_t AIEngine_NormalizeFeatures(AIFeatureVector_t *features);
FS_Status_t AIEngine_ValidateFeatures(AIFeatureVector_t *features);

/* Model management */
FS_Status_t AIEngine_SetModelConfig(AIModelConfig_t *config);
FS_Status_t AIEngine_GetModelConfig(AIModelConfig_t *config);
FS_Status_t AIEngine_GetStatistics(AIStatistics_t *stats);
FS_Status_t AIEngine_ResetStatistics(void);

/* Utility functions */
bool AIEngine_IsModelReady(AIModelType_t model_type);
float AIEngine_GetModelAccuracy(AIModelType_t model_type);
uint32_t AIEngine_GetModelVersion(AIModelType_t model_type);

/* Task function */
void AITask(void *argument);

/* NanoEdge AI Interface */
FS_Status_t NanoEdge_Init(void);
int NanoEdge_Classify(float *input_buffer, uint16_t input_size);
float NanoEdge_GetConfidence(void);
float NanoEdge_DetectAnomalies(float *input_buffer, uint16_t input_size);
FS_Status_t NanoEdge_Learn(float *input_buffer, uint16_t input_size, uint16_t class_id);

/* Post-processing functions */
FS_Status_t PostProcess_Classification(int raw_output, float confidence, AIResult_t *result);
FS_Status_t PostProcess_AnomalyDetection(float anomaly_score, AIResult_t *result);
FS_Status_t PostProcess_Regression(float *raw_outputs, uint16_t output_size, AIResult_t *result);

/* Health assessment based on AI results */
typedef struct {
    bool immediate_attention_required;
    bool long_term_monitoring_advised;
    AIClassification_t primary_concern;
    float severity_score;              // 0.0-1.0
    char recommendation[256];
} HealthAssessment_t;

FS_Status_t GenerateHealthAssessment(AIResult_t *ai_result, FusedData_t *fused_data, HealthAssessment_t *assessment);

#ifdef __cplusplus
}
#endif

#endif /* __AI_ENGINE_H */