/**
  ******************************************************************************
  * @file    ai_engine.c
  * @brief   NanoEdge AI engine implementation
  ******************************************************************************
  * @attention
  *
  * FootSense - NanoEdge AI integration for foot health monitoring
  * Copyright (c) 2025 Yigu
  *
  ******************************************************************************
  */

#include "ai_engine.h"
#include "main.h"
#include <string.h>
#include <math.h>

/* Private defines */
#define AI_FEATURE_VECTOR_SIZE        38    // Total number of input features
#define AI_INFERENCE_TIMEOUT_MS       100   // Maximum inference time
#define AI_ANOMALY_THRESHOLD          0.8f  // Anomaly detection threshold
#define AI_CONFIDENCE_THRESHOLD       0.6f  // Minimum confidence for valid result
#define AI_LEARNING_BUFFER_SIZE       100   // Buffer for online learning samples

/* NanoEdge AI specific defines (placeholder - would be from generated code) */
#define NEAI_ID                       "FootSense_v1_0"
#define NEAI_INPUT_SIZE               38
#define NEAI_OUTPUT_SIZE              8

/* Private structures */
typedef struct {
    AIFeatureVector_t buffer[AI_LEARNING_BUFFER_SIZE];
    AIClassification_t labels[AI_LEARNING_BUFFER_SIZE];
    uint16_t head;
    uint16_t count;
} LearningBuffer_t;

/* Private variables */
static AIModelConfig_t model_configs[AI_MODEL_COUNT];
static AIStatistics_t ai_statistics;
static LearningBuffer_t learning_buffer;
static bool ai_engine_initialized = false;

/* Feature normalization parameters */
static float feature_min[AI_FEATURE_VECTOR_SIZE];
static float feature_max[AI_FEATURE_VECTOR_SIZE];
static float feature_mean[AI_FEATURE_VECTOR_SIZE];
static float feature_std[AI_FEATURE_VECTOR_SIZE];

/* NanoEdge AI model data (placeholder - would be from generated code) */
static uint8_t neai_model_data[32768];  // Example model size
static bool neai_model_loaded = false;

/* External variables */
extern QueueHandle_t SensorDataQueue;
extern QueueHandle_t AIResultQueue;

/* Private function prototypes */
static void InitializeFeatureNormalization(void);
static void UpdateFeatureStatistics(AIFeatureVector_t *features);
static float CalculateFeatureZ_Score(float value, uint16_t feature_index);
static void AddToLearningBuffer(AIFeatureVector_t *features, AIClassification_t label);
static FS_Status_t PerformOnlineLearning(void);
static AIConfidenceLevel_t MapConfidenceToLevel(float confidence);
static void UpdateAIStatistics(AIResult_t *result);

/**
  * @brief  Initialize the AI engine
  * @retval FS_Status_t
  */
FS_Status_t AIEngine_Init(void)
{
    if (ai_engine_initialized) {
        return FS_OK;
    }

    FS_DEBUG_PRINTF("Initializing AI Engine...");

    /* Initialize model configurations */
    for (int i = 0; i < AI_MODEL_COUNT; i++) {
        model_configs[i].model_type = i;
        model_configs[i].input_size = AI_FEATURE_VECTOR_SIZE;
        model_configs[i].output_size = (i == AI_MODEL_CLASSIFICATION) ? AI_CLASS_COUNT : 1;
        model_configs[i].inference_threshold = AI_CONFIDENCE_THRESHOLD;
        model_configs[i].auto_learning_enabled = true;
        model_configs[i].learning_rate_fixed = 1000;  // Fixed-point representation
    }

    /* Initialize statistics */
    memset(&ai_statistics, 0, sizeof(AIStatistics_t));

    /* Initialize learning buffer */
    learning_buffer.head = 0;
    learning_buffer.count = 0;
    memset(learning_buffer.buffer, 0, sizeof(learning_buffer.buffer));
    memset(learning_buffer.labels, 0, sizeof(learning_buffer.labels));

    /* Initialize feature normalization */
    InitializeFeatureNormalization();

    /* Initialize NanoEdge AI */
    if (NanoEdge_Init() != FS_OK) {
        FS_DEBUG_PRINTF("NanoEdge AI initialization failed");
        return FS_ERROR;
    }

    ai_engine_initialized = true;
    FS_DEBUG_PRINTF("AI Engine initialized successfully");

    return FS_OK;
}

/**
  * @brief  AI processing task
  * @param  argument: Not used
  * @retval None
  */
void AITask(void *argument)
{
    SensorData_t sensor_data;
    FusedData_t fused_data;
    AIResult_t ai_result;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10Hz AI processing

    FS_DEBUG_PRINTF("AI Task started");

    /* Initialize the xLastWakeTime variable with the current time */
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        /* Check if we have fused data available */
        if (DataFusion_GetFusedData(&fused_data) == FS_OK) {
            
            /* Process fused data through AI engine */
            if (AIEngine_ProcessFusedData(&fused_data, &ai_result) == FS_OK) {
                
                /* Send AI result to communication task */
                if (xQueueSend(AIResultQueue, &ai_result, 0) != pdTRUE) {
                    FS_DEBUG_PRINTF("AI result queue full");
                }
                
                /* Update statistics */
                UpdateAIStatistics(&ai_result);
                
                /* Perform online learning if enabled */
                if (model_configs[AI_MODEL_CLASSIFICATION].auto_learning_enabled) {
                    PerformOnlineLearning();
                }
                
                /* Log significant findings */
                if (ai_result.confidence > 0.9f && ai_result.anomaly_score > AI_ANOMALY_THRESHOLD) {
                    FS_DEBUG_PRINTF("High confidence anomaly detected: %.2f", ai_result.anomaly_score);
                }
            }
        }

        /* Wait for the next cycle */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
  * @brief  Process fused data through AI engine
  * @param  fused_data: Pointer to fused sensor data
  * @param  result: Pointer to AI result structure
  * @retval FS_Status_t
  */
FS_Status_t AIEngine_ProcessFusedData(FusedData_t *fused_data, AIResult_t *result)
{
    if (!ai_engine_initialized || fused_data == NULL || result == NULL) {
        return FS_ERROR;
    }

    AIFeatureVector_t features;
    
    /* Extract features from fused data */
    if (AIEngine_ExtractFeatures(fused_data, &features) != FS_OK) {
        return FS_ERROR;
    }

    /* Validate and normalize features */
    if (AIEngine_ValidateFeatures(&features) != FS_OK) {
        return FS_ERROR;
    }

    if (AIEngine_NormalizeFeatures(&features) != FS_OK) {
        return FS_ERROR;
    }

    /* Run AI inference */
    return AIEngine_RunInference(&features, result);
}

/**
  * @brief  Extract features from fused data
  * @param  fused_data: Pointer to fused sensor data
  * @param  features: Pointer to feature vector
  * @retval FS_Status_t
  */
FS_Status_t AIEngine_ExtractFeatures(FusedData_t *fused_data, AIFeatureVector_t *features)
{
    if (fused_data == NULL || features == NULL) {
        return FS_ERROR;
    }

    memset(features, 0, sizeof(AIFeatureVector_t));
    features->timestamp = fused_data->timestamp;

    /* Extract pressure features (16 values) */
    int feature_idx = 0;
    
    /* Arch pressure features */
    for (int i = 0; i < 8; i++) {
        // Extract representative pressure values from pressure map
        float sum = 0.0f;
        int count = 0;
        
        int start_x = 8 + (i % 4) * 3;
        int start_y = 6 + (i / 4) * 4;
        
        for (int x = start_x; x < start_x + 3 && x < 32; x++) {
            for (int y = start_y; y < start_y + 4 && y < 24; y++) {
                sum += fused_data->pressure_map.pressure_map[x][y];
                count++;
            }
        }
        
        features->pressure_features[feature_idx++] = (count > 0) ? sum / count : 0.0f;
    }
    
    /* Heel and toe pressure features */
    for (int i = 8; i < 16; i++) {
        features->pressure_features[feature_idx++] = 0.0f;  // Simplified - would extract from pressure map
    }

    /* Motion features (12 values) */
    features->accel_features[0] = fused_data->linear_acceleration[0];
    features->accel_features[1] = fused_data->linear_acceleration[1];
    features->accel_features[2] = fused_data->linear_acceleration[2];
    
    features->gyro_features[0] = fused_data->angular_velocity[0];
    features->gyro_features[1] = fused_data->angular_velocity[1];
    features->gyro_features[2] = fused_data->angular_velocity[2];
    
    features->orientation_features[0] = fused_data->orientation[0];
    features->orientation_features[1] = fused_data->orientation[1];
    features->orientation_features[2] = fused_data->orientation[2];
    
    /* Calculate motion magnitude */
    features->motion_magnitude = sqrtf(
        fused_data->linear_acceleration[0] * fused_data->linear_acceleration[0] +
        fused_data->linear_acceleration[1] * fused_data->linear_acceleration[1] +
        fused_data->linear_acceleration[2] * fused_data->linear_acceleration[2]
    );
    
    features->step_frequency = fused_data->gait_analysis.step_frequency;
    features->gait_symmetry = fused_data->health_metrics.gait_symmetry;

    /* Environmental features (2 values) */
    features->temperature_norm = fused_data->temperature;
    features->humidity_norm = fused_data->humidity;

    /* Derived features (8 values) */
    features->center_of_pressure_x = fused_data->pressure_map.center_of_pressure_x;
    features->center_of_pressure_y = fused_data->pressure_map.center_of_pressure_y;
    features->total_force_norm = fused_data->pressure_map.total_force;
    features->contact_area_norm = fused_data->pressure_map.contact_area;
    features->arch_support_index = fused_data->health_metrics.arch_support_index;
    features->pressure_distribution = fused_data->health_metrics.pressure_distribution;
    features->balance_stability = fused_data->health_metrics.balance_stability;
    features->activity_intensity = fused_data->health_metrics.activity_intensity;

    features->is_valid = true;
    return FS_OK;
}

/**
  * @brief  Normalize feature vector
  * @param  features: Pointer to feature vector
  * @retval FS_Status_t
  */
FS_Status_t AIEngine_NormalizeFeatures(AIFeatureVector_t *features)
{
    if (features == NULL || !features->is_valid) {
        return FS_ERROR;
    }

    /* Normalize pressure features */
    for (int i = 0; i < 16; i++) {
        features->pressure_features[i] = CalculateFeatureZ_Score(features->pressure_features[i], i);
    }

    /* Normalize motion features */
    features->accel_features[0] = CalculateFeatureZ_Score(features->accel_features[0], 16);
    features->accel_features[1] = CalculateFeatureZ_Score(features->accel_features[1], 17);
    features->accel_features[2] = CalculateFeatureZ_Score(features->accel_features[2], 18);
    
    features->gyro_features[0] = CalculateFeatureZ_Score(features->gyro_features[0], 19);
    features->gyro_features[1] = CalculateFeatureZ_Score(features->gyro_features[1], 20);
    features->gyro_features[2] = CalculateFeatureZ_Score(features->gyro_features[2], 21);
    
    features->orientation_features[0] = CalculateFeatureZ_Score(features->orientation_features[0], 22);
    features->orientation_features[1] = CalculateFeatureZ_Score(features->orientation_features[1], 23);
    features->orientation_features[2] = CalculateFeatureZ_Score(features->orientation_features[2], 24);
    
    features->motion_magnitude = CalculateFeatureZ_Score(features->motion_magnitude, 25);
    features->step_frequency = CalculateFeatureZ_Score(features->step_frequency, 26);
    features->gait_symmetry = CalculateFeatureZ_Score(features->gait_symmetry, 27);

    /* Normalize environmental features */
    features->temperature_norm = CalculateFeatureZ_Score(features->temperature_norm, 28);
    features->humidity_norm = CalculateFeatureZ_Score(features->humidity_norm, 29);

    /* Normalize derived features */
    features->center_of_pressure_x = CalculateFeatureZ_Score(features->center_of_pressure_x, 30);
    features->center_of_pressure_y = CalculateFeatureZ_Score(features->center_of_pressure_y, 31);
    features->total_force_norm = CalculateFeatureZ_Score(features->total_force_norm, 32);
    features->contact_area_norm = CalculateFeatureZ_Score(features->contact_area_norm, 33);
    features->arch_support_index = CalculateFeatureZ_Score(features->arch_support_index, 34);
    features->pressure_distribution = CalculateFeatureZ_Score(features->pressure_distribution, 35);
    features->balance_stability = CalculateFeatureZ_Score(features->balance_stability, 36);
    features->activity_intensity = CalculateFeatureZ_Score(features->activity_intensity, 37);

    /* Update feature statistics for online adaptation */
    UpdateFeatureStatistics(features);

    return FS_OK;
}

/**
  * @brief  Validate feature vector
  * @param  features: Pointer to feature vector
  * @retval FS_Status_t
  */
FS_Status_t AIEngine_ValidateFeatures(AIFeatureVector_t *features)
{
    if (features == NULL) {
        return FS_ERROR;
    }

    /* Check for NaN or infinite values */
    float *feature_array = (float*)features;
    for (int i = 0; i < AI_FEATURE_VECTOR_SIZE; i++) {
        if (isnan(feature_array[i]) || isinf(feature_array[i])) {
            features->is_valid = false;
            return FS_ERROR;
        }
    }

    /* Check reasonable ranges for key features */
    if (features->motion_magnitude > 100.0f || features->motion_magnitude < 0.0f) {
        features->is_valid = false;
        return FS_ERROR;
    }

    if (features->step_frequency > 5.0f || features->step_frequency < 0.0f) {
        features->is_valid = false;
        return FS_ERROR;
    }

    features->is_valid = true;
    return FS_OK;
}

/**
  * @brief  Run AI inference
  * @param  features: Pointer to feature vector
  * @param  result: Pointer to AI result
  * @retval FS_Status_t
  */
FS_Status_t AIEngine_RunInference(AIFeatureVector_t *features, AIResult_t *result)
{
    if (!ai_engine_initialized || features == NULL || result == NULL || !features->is_valid) {
        return FS_ERROR;
    }

    /* Initialize result structure */
    memset(result, 0, sizeof(AIResult_t));
    result->timestamp = features->timestamp;
    
    uint32_t start_time = HAL_GetTick();

    /* Prepare input buffer for NanoEdge AI */
    float input_buffer[AI_FEATURE_VECTOR_SIZE];
    
    /* Copy features to input buffer */
    int idx = 0;
    for (int i = 0; i < 16; i++) input_buffer[idx++] = features->pressure_features[i];
    for (int i = 0; i < 3; i++) input_buffer[idx++] = features->accel_features[i];
    for (int i = 0; i < 3; i++) input_buffer[idx++] = features->gyro_features[i];
    for (int i = 0; i < 3; i++) input_buffer[idx++] = features->orientation_features[i];
    input_buffer[idx++] = features->motion_magnitude;
    input_buffer[idx++] = features->step_frequency;
    input_buffer[idx++] = features->gait_symmetry;
    input_buffer[idx++] = features->temperature_norm;
    input_buffer[idx++] = features->humidity_norm;
    input_buffer[idx++] = features->center_of_pressure_x;
    input_buffer[idx++] = features->center_of_pressure_y;
    input_buffer[idx++] = features->total_force_norm;
    input_buffer[idx++] = features->contact_area_norm;
    input_buffer[idx++] = features->arch_support_index;
    input_buffer[idx++] = features->pressure_distribution;
    input_buffer[idx++] = features->balance_stability;
    input_buffer[idx++] = features->activity_intensity;

    /* Run classification */
    int classification_result = NanoEdge_Classify(input_buffer, AI_FEATURE_VECTOR_SIZE);
    float confidence = NanoEdge_GetConfidence();
    
    /* Run anomaly detection */
    float anomaly_score = NanoEdge_DetectAnomalies(input_buffer, AI_FEATURE_VECTOR_SIZE);

    /* Calculate inference time */
    uint32_t end_time = HAL_GetTick();
    result->inference_time_us = (end_time - start_time) * 1000;  // Convert to microseconds

    /* Post-process results */
    if (PostProcess_Classification(classification_result, confidence, result) != FS_OK) {
        return FS_ERROR;
    }

    if (PostProcess_AnomalyDetection(anomaly_score, result) != FS_OK) {
        return FS_ERROR;
    }

    /* Validate results */
    if (result->confidence >= model_configs[AI_MODEL_CLASSIFICATION].inference_threshold) {
        result->is_valid = true;
    } else {
        result->is_valid = false;
    }

    ai_statistics.total_inferences++;
    if (result->is_valid) {
        ai_statistics.valid_inferences++;
        ai_statistics.classifications[result->classification]++;
    }

    return FS_OK;
}

/**
  * @brief  Initialize NanoEdge AI
  * @retval FS_Status_t
  */
FS_Status_t NanoEdge_Init(void)
{
    FS_DEBUG_PRINTF("Initializing NanoEdge AI...");
    
    /* Placeholder for NanoEdge AI initialization */
    /* In a real implementation, this would call the generated NanoEdge functions */
    
    neai_model_loaded = true;
    
    FS_DEBUG_PRINTF("NanoEdge AI initialized - Model ID: %s", NEAI_ID);
    return FS_OK;
}

/**
  * @brief  Run NanoEdge classification
  * @param  input_buffer: Input feature buffer
  * @param  input_size: Size of input buffer
  * @retval int: Classification result
  */
int NanoEdge_Classify(float *input_buffer, uint16_t input_size)
{
    if (!neai_model_loaded || input_buffer == NULL || input_size != NEAI_INPUT_SIZE) {
        return -1;
    }
    
    /* Placeholder for NanoEdge classification */
    /* In real implementation, this would call: neai_classification(input_buffer) */
    
    /* Simple heuristic classification for demonstration */
    float arch_support = input_buffer[34];  // arch_support_index
    float pressure_dist = input_buffer[35]; // pressure_distribution
    float gait_symmetry = input_buffer[27]; // gait_symmetry
    
    if (arch_support < -1.0f) {
        return AI_CLASS_FLAT_FOOT;
    } else if (arch_support > 1.0f) {
        return AI_CLASS_HIGH_ARCH;
    } else if (pressure_dist < -1.0f) {
        return AI_CLASS_PRESSURE_HOTSPOT;
    } else if (gait_symmetry < -1.0f) {
        return AI_CLASS_ABNORMAL_GAIT;
    } else {
        return AI_CLASS_NORMAL_GAIT;
    }
}

/**
  * @brief  Get NanoEdge classification confidence
  * @retval float: Confidence value (0.0-1.0)
  */
float NanoEdge_GetConfidence(void)
{
    if (!neai_model_loaded) {
        return 0.0f;
    }
    
    /* Placeholder for NanoEdge confidence */
    /* In real implementation, this would call: neai_get_confidence() */
    
    return 0.85f + (rand() % 150) / 1000.0f;  // Random confidence between 0.85-1.0
}

/**
  * @brief  Run NanoEdge anomaly detection
  * @param  input_buffer: Input feature buffer
  * @param  input_size: Size of input buffer
  * @retval float: Anomaly score (0.0-1.0)
  */
float NanoEdge_DetectAnomalies(float *input_buffer, uint16_t input_size)
{
    if (!neai_model_loaded || input_buffer == NULL || input_size != NEAI_INPUT_SIZE) {
        return 0.0f;
    }
    
    /* Placeholder for NanoEdge anomaly detection */
    /* In real implementation, this would call: neai_anomaly_detection(input_buffer) */
    
    /* Simple heuristic anomaly scoring */
    float anomaly_score = 0.0f;
    
    /* Check for extreme values */
    for (int i = 0; i < input_size; i++) {
        if (fabsf(input_buffer[i]) > 3.0f) {  // More than 3 standard deviations
            anomaly_score += 0.1f;
        }
    }
    
    return fminf(1.0f, anomaly_score);
}

/**
  * @brief  Post-process classification results
  * @param  raw_output: Raw classification output
  * @param  confidence: Classification confidence
  * @param  result: AI result structure
  * @retval FS_Status_t
  */
FS_Status_t PostProcess_Classification(int raw_output, float confidence, AIResult_t *result)
{
    if (result == NULL) {
        return FS_ERROR;
    }
    
    /* Validate classification output */
    if (raw_output < 0 || raw_output >= AI_CLASS_COUNT) {
        result->classification = AI_CLASS_NORMAL_GAIT;
        result->confidence = 0.0f;
    } else {
        result->classification = (AIClassification_t)raw_output;
        result->confidence = fmaxf(0.0f, fminf(1.0f, confidence));
    }
    
    /* Map confidence to level */
    result->confidence_level = MapConfidenceToLevel(result->confidence);
    
    return FS_OK;
}

/**
  * @brief  Post-process anomaly detection results
  * @param  anomaly_score: Raw anomaly score
  * @param  result: AI result structure
  * @retval FS_Status_t
  */
FS_Status_t PostProcess_AnomalyDetection(float anomaly_score, AIResult_t *result)
{
    if (result == NULL) {
        return FS_ERROR;
    }
    
    /* Clamp anomaly score to valid range */
    result->anomaly_score = fmaxf(0.0f, fminf(1.0f, anomaly_score));
    
    /* Update statistics if anomaly detected */
    if (result->anomaly_score > AI_ANOMALY_THRESHOLD) {
        ai_statistics.anomalies_detected++;
    }
    
    return FS_OK;
}

/**
  * @brief  Generate health assessment based on AI results
  * @param  ai_result: AI inference result
  * @param  fused_data: Fused sensor data
  * @param  assessment: Health assessment output
  * @retval FS_Status_t
  */
FS_Status_t GenerateHealthAssessment(AIResult_t *ai_result, FusedData_t *fused_data, HealthAssessment_t *assessment)
{
    if (ai_result == NULL || fused_data == NULL || assessment == NULL) {
        return FS_ERROR;
    }
    
    /* Initialize assessment */
    memset(assessment, 0, sizeof(HealthAssessment_t));
    assessment->primary_concern = ai_result->classification;
    
    /* Calculate severity score */
    assessment->severity_score = ai_result->anomaly_score * ai_result->confidence;
    
    /* Determine if immediate attention is required */
    assessment->immediate_attention_required = 
        (ai_result->confidence > 0.9f) && 
        (ai_result->classification == AI_CLASS_PRESSURE_HOTSPOT ||
         ai_result->classification == AI_CLASS_ABNORMAL_GAIT) &&
        (ai_result->anomaly_score > 0.8f);
    
    /* Determine if long-term monitoring is advised */
    assessment->long_term_monitoring_advised = 
        (ai_result->classification == AI_CLASS_FLAT_FOOT ||
         ai_result->classification == AI_CLASS_HIGH_ARCH ||
         ai_result->classification == AI_CLASS_PRONATION ||
         ai_result->classification == AI_CLASS_SUPINATION);
    
    /* Generate recommendations */
    switch (ai_result->classification) {
        case AI_CLASS_NORMAL_GAIT:
            strcpy(assessment->recommendation, "Normal gait pattern detected. Continue regular activity.");
            break;
            
        case AI_CLASS_FLAT_FOOT:
            strcpy(assessment->recommendation, "Flat foot pattern detected. Consider arch support insoles.");
            break;
            
        case AI_CLASS_HIGH_ARCH:
            strcpy(assessment->recommendation, "High arch detected. Monitor for pressure points.");
            break;
            
        case AI_CLASS_PRESSURE_HOTSPOT:
            strcpy(assessment->recommendation, "Pressure hotspot detected. Check footwear and consider redistribution.");
            break;
            
        case AI_CLASS_ABNORMAL_GAIT:
            strcpy(assessment->recommendation, "Abnormal gait pattern. Consider consulting healthcare provider.");
            break;
            
        default:
            strcpy(assessment->recommendation, "Continue monitoring foot health patterns.");
            break;
    }
    
    return FS_OK;
}

/**
  * @brief  Initialize feature normalization parameters
  * @retval None
  */
static void InitializeFeatureNormalization(void)
{
    /* Initialize with default values */
    for (int i = 0; i < AI_FEATURE_VECTOR_SIZE; i++) {
        feature_min[i] = -10.0f;
        feature_max[i] = 10.0f;
        feature_mean[i] = 0.0f;
        feature_std[i] = 1.0f;
    }
}

/**
  * @brief  Update feature statistics for online adaptation
  * @param  features: Feature vector
  * @retval None
  */
static void UpdateFeatureStatistics(AIFeatureVector_t *features)
{
    static uint32_t sample_count = 0;
    float alpha = 0.01f;  // Learning rate for online statistics
    
    float *feature_array = (float*)features;
    
    sample_count++;
    
    /* Update running mean and standard deviation */
    for (int i = 0; i < AI_FEATURE_VECTOR_SIZE; i++) {
        float value = feature_array[i];
        
        /* Update min/max */
        feature_min[i] = fminf(feature_min[i], value);
        feature_max[i] = fmaxf(feature_max[i], value);
        
        /* Update running mean */
        float delta = value - feature_mean[i];
        feature_mean[i] += alpha * delta;
        
        /* Update running variance (simplified) */
        float delta2 = value - feature_mean[i];
        feature_std[i] = sqrtf(alpha * delta * delta2 + (1.0f - alpha) * feature_std[i] * feature_std[i]);
        
        if (feature_std[i] < 0.001f) {
            feature_std[i] = 0.001f;  // Prevent division by zero
        }
    }
}

/**
  * @brief  Calculate z-score for feature normalization
  * @param  value: Feature value
  * @param  feature_index: Feature index
  * @retval float: Normalized value
  */
static float CalculateFeatureZ_Score(float value, uint16_t feature_index)
{
    if (feature_index >= AI_FEATURE_VECTOR_SIZE) {
        return 0.0f;
    }
    
    return (value - feature_mean[feature_index]) / feature_std[feature_index];
}

/**
  * @brief  Map confidence value to confidence level
  * @param  confidence: Confidence value (0.0-1.0)
  * @retval AIConfidenceLevel_t: Confidence level
  */
static AIConfidenceLevel_t MapConfidenceToLevel(float confidence)
{
    if (confidence >= 0.9f) {
        return AI_CONFIDENCE_VERY_HIGH;
    } else if (confidence >= 0.75f) {
        return AI_CONFIDENCE_HIGH;
    } else if (confidence >= 0.6f) {
        return AI_CONFIDENCE_MEDIUM;
    } else {
        return AI_CONFIDENCE_LOW;
    }
}

/**
  * @brief  Update AI statistics
  * @param  result: AI result
  * @retval None
  */
static void UpdateAIStatistics(AIResult_t *result)
{
    if (result == NULL) {
        return;
    }
    
    /* Update average confidence */
    float alpha = 0.1f;  // Smoothing factor
    ai_statistics.average_confidence = ai_statistics.average_confidence * (1.0f - alpha) + 
                                     result->confidence * alpha;
    
    /* Update average inference time */
    ai_statistics.average_inference_time_us = ai_statistics.average_inference_time_us * (1.0f - alpha) + 
                                             result->inference_time_us * alpha;
}

/**
  * @brief  Get AI statistics
  * @param  stats: Statistics structure
  * @retval FS_Status_t
  */
FS_Status_t AIEngine_GetStatistics(AIStatistics_t *stats)
{
    if (stats == NULL) {
        return FS_ERROR;
    }
    
    memcpy(stats, &ai_statistics, sizeof(AIStatistics_t));
    return FS_OK;
}

/**
  * @brief  Check if AI model is ready
  * @param  model_type: Model type
  * @retval bool: true if ready
  */
bool AIEngine_IsModelReady(AIModelType_t model_type)
{
    return ai_engine_initialized && neai_model_loaded && (model_type < AI_MODEL_COUNT);
}