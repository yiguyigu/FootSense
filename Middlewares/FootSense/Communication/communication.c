/**
  ******************************************************************************
  * @file    communication.c
  * @brief   Communication system implementation
  ******************************************************************************
  * @attention
  *
  * FootSense - Communication interface for foot health monitoring
  * Copyright (c) 2025 Yigu
  *
  ******************************************************************************
  */

#include "communication.h"
#include "main.h"
#include <string.h>

/* Private defines */
#define COMM_HEADER_SIZE          16
#define COMM_MAX_RETRY_COUNT      3
#define COMM_ACK_TIMEOUT_MS       1000
#define COMM_PACKET_QUEUE_SIZE    10

/* Private variables */
static CommConfig_t comm_config;
static SystemStatus_t system_status;
static DeviceConfig_t device_config;
static MobileAppInterface_t mobile_app;
static StreamConfig_t stream_config;
static bool communication_initialized = false;

/* Communication statistics */
static uint32_t bytes_transmitted[COMM_PROTOCOL_COUNT];
static uint32_t bytes_received[COMM_PROTOCOL_COUNT];
static uint32_t packet_count_tx = 0;
static uint8_t sequence_number = 0;

/* External variables */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern QueueHandle_t AIResultQueue;
extern QueueHandle_t CommDataQueue;

/* Private function prototypes */
static void UpdateSystemStatus(void);
static FS_Status_t ProcessIncomingData(void);

/**
  * @brief  Initialize communication system
  * @retval FS_Status_t
  */
FS_Status_t Communication_Init(void)
{
    if (communication_initialized) {
        return FS_OK;
    }

    FS_DEBUG_PRINTF("Initializing Communication System...");

    /* Initialize default configuration */
    comm_config.protocol = COMM_PROTOCOL_UART;
    comm_config.baud_rate = 115200;
    comm_config.data_bits = 8;
    comm_config.stop_bits = 1;
    comm_config.parity = 0;
    comm_config.flow_control = false;
    comm_config.max_packet_size = 256;
    comm_config.timeout_ms = 1000;

    /* Initialize device configuration */
    device_config.sensor_sampling_rate = 50;
    device_config.sensor_filters_enabled = 1;
    device_config.auto_calibration_enabled = 1;
    device_config.ai_confidence_threshold = 0.7f;
    device_config.ai_learning_enabled = 1;
    device_config.data_transmission_interval = 1000;

    /* Initialize mobile app interface */
    strcpy(mobile_app.device_name, "FootSense-001");
    strcpy(mobile_app.device_id, "FS001");
    mobile_app.is_paired = false;

    /* Initialize streaming */
    stream_config.streaming_enabled = false;
    stream_config.stream_rate_hz = 10;

    /* Clear statistics */
    memset(bytes_transmitted, 0, sizeof(bytes_transmitted));
    memset(bytes_received, 0, sizeof(bytes_received));

    communication_initialized = true;
    FS_DEBUG_PRINTF("Communication System initialized successfully");

    return FS_OK;
}

/**
  * @brief  Communication task implementation
  * @param  argument: Not used
  * @retval None
  */
void CommunicationTask(void *argument)
{
    AIResult_t ai_result;
    CommPacket_t packet;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(device_config.data_transmission_interval);

    FS_DEBUG_PRINTF("Communication Task started");

    /* Initialize the xLastWakeTime variable with the current time */
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        /* Process incoming data */
        ProcessIncomingData();

        /* Send AI results if available */
        if (xQueueReceive(AIResultQueue, &ai_result, 0) == pdTRUE) {
            Communication_SendAIResult(&ai_result);
        }

        /* Send periodic system status */
        UpdateSystemStatus();
        Communication_SendSystemStatus(&system_status);

        /* Handle streaming if enabled */
        if (stream_config.streaming_enabled) {
            /* Streaming logic would be implemented here */
        }

        /* Wait for the next cycle */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
  * @brief  Send AI result
  * @param  ai_result: Pointer to AI result
  * @retval FS_Status_t
  */
FS_Status_t Communication_SendAIResult(AIResult_t *ai_result)
{
    if (!communication_initialized || ai_result == NULL) {
        return FS_ERROR;
    }

    CommPacket_t packet;
    uint8_t payload[sizeof(AIResult_t)];
    uint16_t payload_size = sizeof(AIResult_t);

    /* Serialize AI result */
    memcpy(payload, ai_result, payload_size);

    /* Create packet */
    if (Communication_CreatePacket(MSG_TYPE_AI_RESULT, payload, payload_size, &packet) != FS_OK) {
        return FS_ERROR;
    }

    /* Send packet */
    return Communication_SendPacket(&packet, comm_config.protocol);
}

/**
  * @brief  Send system status
  * @param  status: Pointer to system status
  * @retval FS_Status_t
  */
FS_Status_t Communication_SendSystemStatus(SystemStatus_t *status)
{
    if (!communication_initialized || status == NULL) {
        return FS_ERROR;
    }

    CommPacket_t packet;
    uint8_t payload[sizeof(SystemStatus_t)];
    uint16_t payload_size = sizeof(SystemStatus_t);

    /* Serialize system status */
    memcpy(payload, status, payload_size);

    /* Create packet */
    if (Communication_CreatePacket(MSG_TYPE_SYSTEM_STATUS, payload, payload_size, &packet) != FS_OK) {
        return FS_ERROR;
    }

    /* Send packet */
    return Communication_SendPacket(&packet, comm_config.protocol);
}

/**
  * @brief  Create communication packet
  * @param  type: Message type
  * @param  payload: Payload data
  * @param  payload_size: Payload size
  * @param  packet: Output packet
  * @retval FS_Status_t
  */
FS_Status_t Communication_CreatePacket(MessageType_t type, uint8_t *payload, uint16_t payload_size, CommPacket_t *packet)
{
    if (packet == NULL || payload_size > 256) {
        return FS_ERROR;
    }

    /* Clear packet */
    memset(packet, 0, sizeof(CommPacket_t));

    /* Set header */
    packet->header[0] = 'F';
    packet->header[1] = 'T';
    packet->header[2] = 'S';
    packet->header[3] = 'N';

    /* Set packet fields */
    packet->length = payload_size;
    packet->type = type;
    packet->sequence = sequence_number++;
    packet->timestamp = HAL_GetTick();

    /* Copy payload */
    if (payload != NULL && payload_size > 0) {
        memcpy(packet->payload, payload, payload_size);
    }

    /* Calculate checksum */
    packet->checksum = Security_CalculateCRC16((uint8_t*)packet, sizeof(CommPacket_t) - 2);

    return FS_OK;
}

/**
  * @brief  Send packet via specified protocol
  * @param  packet: Packet to send
  * @param  protocol: Communication protocol
  * @retval FS_Status_t
  */
FS_Status_t Communication_SendPacket(CommPacket_t *packet, CommProtocol_t protocol)
{
    if (packet == NULL || protocol >= COMM_PROTOCOL_COUNT) {
        return FS_ERROR;
    }

    FS_Status_t result = FS_ERROR;
    uint16_t packet_size = sizeof(CommPacket_t);

    switch (protocol) {
        case COMM_PROTOCOL_UART:
            result = UART_SendData((uint8_t*)packet, packet_size);
            break;

        case COMM_PROTOCOL_BLE:
            result = BLE_SendData((uint8_t*)packet, packet_size);
            break;

        default:
            return FS_ERROR;
    }

    if (result == FS_OK) {
        bytes_transmitted[protocol] += packet_size;
        packet_count_tx++;
    }

    return result;
}

/**
  * @brief  Calculate CRC16 checksum
  * @param  data: Data buffer
  * @param  length: Data length
  * @retval uint16_t: CRC16 checksum
  */
uint16_t Security_CalculateCRC16(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return crc;
}

/**
  * @brief  Initialize UART communication
  * @param  config: Communication configuration
  * @retval FS_Status_t
  */
FS_Status_t UART_Init(CommConfig_t *config)
{
    /* UART initialization is handled in main.c */
    return FS_OK;
}

/**
  * @brief  Send data via UART
  * @param  data: Data buffer
  * @param  length: Data length
  * @retval FS_Status_t
  */
FS_Status_t UART_SendData(uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, data, length, 1000);
    return (status == HAL_OK) ? FS_OK : FS_ERROR;
}

/**
  * @brief  Initialize BLE communication
  * @param  config: Communication configuration
  * @retval FS_Status_t
  */
FS_Status_t BLE_Init(CommConfig_t *config)
{
    /* BLE initialization would be implemented here */
    return FS_OK;
}

/**
  * @brief  Send data via BLE
  * @param  data: Data buffer
  * @param  length: Data length
  * @retval FS_Status_t
  */
FS_Status_t BLE_SendData(uint8_t *data, uint16_t length)
{
    /* BLE data transmission would be implemented here */
    return FS_OK;
}

/**
  * @brief  Update system status
  * @retval None
  */
static void UpdateSystemStatus(void)
{
    system_status.uptime_seconds = HAL_GetTick() / 1000;
    system_status.free_heap = xPortGetFreeHeapSize();
    system_status.temperature = 25.0f;  // Placeholder
    system_status.battery_level = 3700;  // Placeholder (mV)
    system_status.system_state = 1;  // Operational
    
    /* Set firmware version */
    system_status.firmware_version[0] = FOOTSENSE_VERSION_MAJOR;
    system_status.firmware_version[1] = FOOTSENSE_VERSION_MINOR;
    system_status.firmware_version[2] = FOOTSENSE_VERSION_PATCH;
}

/**
  * @brief  Process incoming data
  * @retval FS_Status_t
  */
static FS_Status_t ProcessIncomingData(void)
{
    /* Placeholder for incoming data processing */
    return FS_OK;
}

/**
  * @brief  Check if protocol is connected
  * @param  protocol: Communication protocol
  * @retval bool: Connection status
  */
bool Communication_IsConnected(CommProtocol_t protocol)
{
    switch (protocol) {
        case COMM_PROTOCOL_UART:
            return true;  // UART is always "connected"
            
        case COMM_PROTOCOL_BLE:
            return BLE_IsConnected();
            
        default:
            return false;
    }
}

/**
  * @brief  Check BLE connection status
  * @retval bool: true if connected
  */
bool BLE_IsConnected(void)
{
    /* BLE connection check would be implemented here */
    return false;
}

/**
  * @brief  Get bytes transmitted count
  * @param  protocol: Communication protocol
  * @retval uint32_t: Bytes transmitted
  */
uint32_t Communication_GetBytesTransmitted(CommProtocol_t protocol)
{
    if (protocol < COMM_PROTOCOL_COUNT) {
        return bytes_transmitted[protocol];
    }
    return 0;
}