## 基于 STM32F407 的 AI 足部健康监测智能穿戴设备

### 1. 项目概述

- **项目名称**: 基于 STM32F407 的 AI 足部健康监测智能穿戴设备
- **目标**: 通过多传感器融合与边缘 AI，实时识别用户足部姿态、足弓状态及出汗情况，并通过 BLE 传输结果至手机端。

### 2. 系统架构

| **模块** | **硬件/软件** | **说明** |
| --- | --- | --- |
| 主控 MCU | STM32F407 + FreeRTOS | 运行多任务、采集传感器、AI 推理 |
| 传感器 | 8 路压力传感器、6 轴 IMU、温湿度传感器 | 足底压力分布、姿态角度、环境温湿度 |
| 无线通信 | BLE 模块（UART 接口） | 将预测结果发送到手机 |
| AI 引擎 | STM32Cube.AI + NanoEdge AI Studio | 一个 NanoEdge 分类模型、两个规则模型 |
| 移动端 | Android/iOS 应用 | 显示实时监测数据 |

### 3. 数据流与处理

- **同步采集**
  - 10 ms 定时器中断读取所有传感器值。
  - 每次采样获取 8 压力 + 6 IMU + 2 温湿度 = 16 维数据。
  - 使用 `HAL_GetTick()` 作为时间戳，保证一次采样的 16 维数据共享同一时间戳。

- **特征提取**
  - 采用 200 ms 滑动窗口（约 20 个采样点），每 100 ms 更新一次。
  - 提取 20 维精简特征：4路压力均值 + 6轴IMU均值 + 2路环境均值 + 压力分布方差 + 姿态变化幅度。
  - 特征向量示例：`[p1_mean, p2_mean, p3_mean, p4_mean, ax, ay, az, gx, gy, gz, temp, humid, pressure_var, motion_amp, ...]`。

- **模型推理**
  - 采用混合推理方案（1个AI模型 + 2个规则模型）：
    - 姿态模型：NanoEdge AI 分类（正常 / 内八 / 外八）
    - 足弓模型：基于压力分布方差的规则判断（正常 / 扁平）
    - 出汗模型：基于温湿度阈值的规则判断（正常 / 多汗）
  - 共享 20 维特征向量，分别输入不同模型进行推理。

- **UART 传输**
  - 将预测结果和时间戳封装成 JSON 字符串，通过 UART2（115200 波特率）发送至上位机。

示例 JSON：

```json
{"ts":123456,"stance":1,"arch":0,"sweat":1,"probs":[0.1,0.8,0.1]}
```

### 4. 模型训练与部署

- **数据采集**
  - 10 ms 采样间隔，每人每状态 30 分钟步行数据。
  - 每个窗口自动附带三个标签：[姿态, 足弓, 出汗]。

- **训练步骤**
  - 使用 Python 脚本切片成 200 ms 窗口，提取 20 维精简特征。
  - 在 NanoEdge 中训练 1 个姿态分类模型。
  - 足弓和出汗模型采用规则判断，基于压力分布和温湿度阈值。
  - 结果：姿态模型在验证集上准确率 > 90%，规则模型通过阈值调优。

- **部署要点**
  - 混合方案兼顾精度与效率：复杂姿态用 AI，简单判断用规则。
  - 20 维特征大幅减少计算量和存储需求。

### 5. 关键设计亮点

- 多任务实时架构：FreeRTOS 四任务并行，保证数据实时采集与处理。
- 统一时间戳采样：10 ms 定时器中断保证所有传感器同步。
- 混合推理方案：AI + 规则结合，平衡精度与效率。
- 环形缓冲区：保证窗口切片与特征提取无数据丢失。

---

## FreeRTOS 多任务伪代码

```c
/* ========== 全局定义 ========== */
#define SAMPLE_PERIOD_MS 10
#define WINDOW_SIZE 20   // 200 ms / 10 ms
#define FEATURE_DIM 20   // 20维精简特征
RingBuffer sensor_buf;   // 保存最近WINDOW_SIZE条采样
float feature_vector[FEATURE_DIM];

TaskHandle_t taskSampling, taskFeature, taskAI, taskBLE;

/* ========== 传感器采集任务 ========== */
void Task_Sampling(void *pv) {
    SensorData data;
    for(;;) {
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
        data.timestamp = HAL_GetTick();
        read_pressure(data.pressure);   // 8 values
        read_imu(data.imu);             // 6 values
        read_temp_humid(data.env);      // 2 values
        ringbuffer_push(&sensor_buf, data);
    }
}

/* ========== 特征提取任务 ========== */
void Task_Feature(void *pv) {
    for(;;) {
        if (ringbuffer_count(&sensor_buf) >= WINDOW_SIZE) {
            SensorData window[WINDOW_SIZE];
            ringbuffer_get_window(&sensor_buf, window);
            extract_features_window(window, feature_vector); // 20维精简特征
            xQueueSend(qFeature, feature_vector, 0);  // 发送给 AI 任务
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // 每100ms滑动一次
    }
}

/* ========== AI 推理任务 ========== */
void Task_AI(void *pv) {
    float feature[FEATURE_DIM];
    AI_Result result;
    for(;;) {
        if (xQueueReceive(qFeature, feature, portMAX_DELAY)) {
            // 姿态模型：NanoEdge AI
            ai_run_stance(feature, &result.stance);
            // 足弓模型：规则判断
            result.arch = (feature[13] > 0.5f) ? 1 : 0;
            // 出汗模型：规则判断  
            result.sweat = (feature[12] > 25.0f && feature[13] > 50.0f) ? 1 : 0;
            xQueueSend(qBLE, &result, 0);
        }
    }
}

/* ========== BLE 发送任务 ========== */
void Task_BLE(void *pv) {
    AI_Result res;
    char msg[128];
    for(;;) {
        if (xQueueReceive(qBLE, &res, portMAX_DELAY)) {
            snprintf(msg, sizeof(msg), 
                     "{\"ts\":%lu,\"stance\":%u,\"arch\":%u,\"sweat\":%u,\"probs\":[%.3f,%.3f,%.3f]}\r\n",
                     res.ts, res.stance, res.arch, res.sweat,
                     res.probs[0], res.probs[1], res.probs[2]);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
        }
    }
}

/* ========== 系统启动 ========== */
int main(void) {
    hardware_init();
    ringbuffer_init(&sensor_buf);

    xTaskCreate(Task_Sampling, "Sampling", 512, NULL, 3, &taskSampling);
    xTaskCreate(Task_Feature,  "Feature",  1024, NULL, 2, &taskFeature);
    xTaskCreate(Task_AI,       "AI",       1024, NULL, 2, &taskAI);
    xTaskCreate(Task_BLE,      "BLE",      512,  NULL, 1, &taskBLE);

    vTaskStartScheduler();
}
```

---

## 技术实现要点

### 核心算法

- **特征提取**：200ms 滑动窗口，20维精简特征向量
  - 4路压力传感器均值（足弓判断）
  - 6轴IMU均值（姿态判断）
  - 2路环境均值（出汗判断）
  - 压力分布方差 + 姿态变化幅度

- **混合推理**：1个AI模型 + 2个规则模型
  - 姿态：NanoEdge AI 分类（正常/内八/外八）
  - 足弓：压力分布方差阈值判断（正常/扁平）
  - 出汗：温湿度阈值判断（正常/多汗）

### 系统架构

- **实时性**：10ms 采样周期，100ms 特征更新
- **多任务**：FreeRTOS 四任务并行（采集/特征/AI/输出）
- **数据流**：环形缓冲 → 窗口统计 → 混合推理 → UART输出
- **通信**：UART2（115200波特率）JSON格式输出

### 亮点

> 独立完成多传感器同步采集、200 ms 窗口特征（20 维）、混合推理方案（1个AI + 2个规则）与 STM32 部署；基于 FreeRTOS 的四任务并行实时系统，将足部姿态/足弓/出汗预测以 UART 持续下发。


