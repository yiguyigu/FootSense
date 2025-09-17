/**
  ******************************************************************************
  * @file    communication.h
  * @brief   Header file for communication system
  ******************************************************************************
  * @attention
  *
  * FootSense - Communication interface for foot health monitoring
  * Copyright (c) 2025 Yigu
  *
  ******************************************************************************
  */

#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "data_fusion.h"
#include "ai_engine.h"
#include <stdint.h>
#include <stdbool.h>

/* Communication Protocol Types */
typedef enum {
    COMM_PROTOCOL_UART = 0,
    COMM_PROTOCOL_BLE,
    COMM_PROTOCOL_WIFI,
    COMM_PROTOCOL_USB,
    COMM_PROTOCOL_COUNT
} CommProtocol_t;

/* Message Types */
typedef enum {
    MSG_TYPE_SENSOR_DATA = 0x01,
    MSG_TYPE_FUSED_DATA = 0x02,
    MSG_TYPE_AI_RESULT = 0x03,
    MSG_TYPE_HEALTH_ASSESSMENT = 0x04,
    MSG_TYPE_SYSTEM_STATUS = 0x05,
    MSG_TYPE_CONFIG_REQUEST = 0x06,
    MSG_TYPE_CONFIG_RESPONSE = 0x07,
    MSG_TYPE_COMMAND = 0x08,
    MSG_TYPE_ACK = 0x09,
    MSG_TYPE_ERROR = 0x0A
} MessageType_t;

/* Communication Packet Structure */
typedef struct {
    uint8_t header[4];          // Packet header "FTSN"
    uint16_t length;            // Payload length
    MessageType_t type;         // Message type
    uint8_t sequence;           // Sequence number
    uint32_t timestamp;         // Timestamp
    uint8_t payload[256];       // Data payload
    uint16_t checksum;          // CRC16 checksum
} __attribute__((packed)) CommPacket_t;

/* System Status Structure */
typedef struct {
    uint8_t system_state;       // System operational state
    uint32_t uptime_seconds;    // System uptime
    uint16_t battery_level;     // Battery level (mV)
    float temperature;          // System temperature (°C)
    uint32_t free_heap;         // Free heap memory
    uint8_t sensor_status;      // Bitmask of sensor status
    uint32_t error_code;        // Last error code
    uint8_t firmware_version[3]; // Major.Minor.Patch
} SystemStatus_t;

/* Configuration Structure */
typedef struct {
    /* Sensor configuration */
    uint16_t sensor_sampling_rate;  // Hz
    uint8_t sensor_filters_enabled;
    uint8_t auto_calibration_enabled;
    
    /* AI configuration */
    float ai_confidence_threshold;
    uint8_t ai_learning_enabled;
    uint8_t ai_anomaly_detection_enabled;
    
    /* Communication configuration */
    uint32_t data_transmission_interval; // ms
    uint8_t compression_enabled;
    uint8_t encryption_enabled;
    
    /* Power management */
    uint8_t low_power_mode_enabled;
    uint16_t sleep_timeout_seconds;
    
    /* Health monitoring */
    uint8_t health_alerts_enabled;
    float alert_thresholds[8];
} DeviceConfig_t;

/* Command Types */
typedef enum {
    CMD_START_MONITORING = 0x01,
    CMD_STOP_MONITORING = 0x02,
    CMD_CALIBRATE_SENSORS = 0x03,
    CMD_RESET_SYSTEM = 0x04,
    CMD_ENTER_SLEEP_MODE = 0x05,
    CMD_WAKE_UP = 0x06,
    CMD_SET_CONFIG = 0x07,
    CMD_GET_CONFIG = 0x08,
    CMD_GET_STATUS = 0x09,
    CMD_CLEAR_DATA = 0x0A,
    CMD_FACTORY_RESET = 0x0B
} CommandType_t;

/* Communication Configuration */
typedef struct {
    CommProtocol_t protocol;
    uint32_t baud_rate;
    uint8_t data_bits;
    uint8_t stop_bits;
    uint8_t parity;
    bool flow_control;
    uint16_t max_packet_size;
    uint32_t timeout_ms;
} CommConfig_t;

/* Function Prototypes */
FS_Status_t Communication_Init(void);
FS_Status_t Communication_DeInit(void);
FS_Status_t Communication_Configure(CommConfig_t *config);

/* Data transmission functions */
FS_Status_t Communication_SendSensorData(SensorData_t *sensor_data);
FS_Status_t Communication_SendFusedData(FusedData_t *fused_data);
FS_Status_t Communication_SendAIResult(AIResult_t *ai_result);
FS_Status_t Communication_SendHealthAssessment(HealthAssessment_t *assessment);
FS_Status_t Communication_SendSystemStatus(SystemStatus_t *status);

/* Packet management */
FS_Status_t Communication_CreatePacket(MessageType_t type, uint8_t *payload, uint16_t payload_size, CommPacket_t *packet);
FS_Status_t Communication_SendPacket(CommPacket_t *packet, CommProtocol_t protocol);
FS_Status_t Communication_ReceivePacket(CommPacket_t *packet, CommProtocol_t protocol);
FS_Status_t Communication_ValidatePacket(CommPacket_t *packet);

/* Command processing */
FS_Status_t Communication_ProcessCommand(CommandType_t command, uint8_t *params, uint16_t param_size);
FS_Status_t Communication_SendResponse(CommandType_t command, FS_Status_t result, uint8_t *response_data, uint16_t data_size);

/* Protocol-specific functions */
FS_Status_t UART_Init(CommConfig_t *config);
FS_Status_t UART_SendData(uint8_t *data, uint16_t length);
FS_Status_t UART_ReceiveData(uint8_t *data, uint16_t length, uint32_t timeout);

FS_Status_t BLE_Init(CommConfig_t *config);
FS_Status_t BLE_SendData(uint8_t *data, uint16_t length);
FS_Status_t BLE_ReceiveData(uint8_t *data, uint16_t length, uint32_t timeout);
FS_Status_t BLE_IsConnected(void);

/* Data formatting and compression */
FS_Status_t DataFormat_CompressSensorData(SensorData_t *input, uint8_t *output, uint16_t *output_size);
FS_Status_t DataFormat_CompressFusedData(FusedData_t *input, uint8_t *output, uint16_t *output_size);
FS_Status_t DataFormat_SerializeAIResult(AIResult_t *input, uint8_t *output, uint16_t *output_size);

/* Security functions */
FS_Status_t Security_EncryptData(uint8_t *input, uint16_t input_size, uint8_t *output, uint16_t *output_size);
FS_Status_t Security_DecryptData(uint8_t *input, uint16_t input_size, uint8_t *output, uint16_t *output_size);
uint16_t Security_CalculateCRC16(uint8_t *data, uint16_t length);

/* Utility functions */
bool Communication_IsConnected(CommProtocol_t protocol);
uint32_t Communication_GetBytesTransmitted(CommProtocol_t protocol);
uint32_t Communication_GetBytesReceived(CommProtocol_t protocol);
float Communication_GetSignalStrength(CommProtocol_t protocol);

/* Task function */
void CommunicationTask(void *argument);

/* Mobile app interface */
typedef struct {
    char device_name[32];
    char device_id[16];
    uint8_t pairing_code[4];
    bool is_paired;
    uint32_t session_key[4];
} MobileAppInterface_t;

FS_Status_t MobileApp_Initialize(void);
FS_Status_t MobileApp_Pair(uint8_t *pairing_code);
FS_Status_t MobileApp_SendData(uint8_t *data, uint16_t length);
bool MobileApp_IsConnected(void);

/* Real-time streaming */
typedef struct {
    bool streaming_enabled;
    uint16_t stream_rate_hz;
    MessageType_t stream_types;
    uint32_t stream_start_time;
    uint32_t packets_streamed;
} StreamConfig_t;

FS_Status_t Streaming_Configure(StreamConfig_t *config);
FS_Status_t Streaming_Start(void);
FS_Status_t Streaming_Stop(void);
bool Streaming_IsActive(void);

#ifdef __cplusplus
}
#endif

#endif /* __COMMUNICATION_H */