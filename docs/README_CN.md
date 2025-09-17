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
| AI 引擎 | STM32Cube.AI + NanoEdge AI Studio | 三个 NanoEdge 分类模型 |
| 移动端 | Android/iOS 应用 | 显示实时监测数据 |

### 3. 数据流与处理

- **同步采集**
  - 10 ms 定时器中断读取所有传感器值。
  - 每次采样获取 8 压力 + 6 IMU + 2 温湿度 = 16 维数据。
  - 使用 `HAL_GetTick()` 作为时间戳，保证一次采样的 16 维数据共享同一时间戳。

- **特征提取**
  - 采用 200 ms 滑动窗口（约 20 个采样点），每 100 ms 更新一次。
  - 对每路数据计算均值、标准差、最大最小值、FFT 低频能量等 → 约 50 维特征向量。
  - 特征向量示例：`[mean_p1, std_p1, …, fft_acc_z_peak, …]`。

- **模型推理**
  - 使用 NanoEdge 训练并部署 3 个模型：
    - 姿态模型：正常 / 内八 / 外八
    - 足弓模型：正常 / 扁平
    - 出汗模型：正常 / 多汗
  - MCU 调用三次推理函数，将同一特征向量输入三个模型，得到三个预测值。

- **BLE 传输**
  - 将预测结果和时间戳封装成 JSON 字符串，通过 BLE UART 发送至手机端应用。

示例 JSON：

```json
{"ts":123456,"stance":1,"arch":0,"sweat":2}
```

### 4. 模型训练与部署

- **数据采集**
  - 10 ms 采样间隔，每人每状态 30 分钟步行数据。
  - 每个窗口自动附带三个标签：[姿态, 足弓, 出汗]。

- **训练步骤**
  - 使用 Python 脚本切片成 200 ms 窗口，提取与 MCU 一致的特征。
  - 在 NanoEdge 中分别训练 3 个分类模型。
  - 结果：各模型在验证集上的准确率均在 90% 以上。

- **部署要点**
  - MCU 侧共享一份特征向量，复用到三个模型，提升效率与节省资源。

### 5. 关键设计亮点

- 多任务实时架构：FreeRTOS 四任务并行，保证数据实时采集与处理。
- 统一时间戳采样：10 ms 定时器中断保证所有传感器同步。
- 模块化 AI 部署：同一特征向量共享三模型，提高运算与存储效率。
- 环形缓冲区：保证窗口切片与特征提取无数据丢失。

---

## FreeRTOS 多任务伪代码

```c
/* ========== 全局定义 ========== */
#define SAMPLE_PERIOD_MS 10
#define WINDOW_SIZE 20   // 200 ms / 10 ms
RingBuffer sensor_buf;   // 保存最近WINDOW_SIZE条采样
float feature_vector[50];

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
            extract_features(window, feature_vector); // 均值/方差/FFT
            xQueueSend(qFeature, feature_vector, 0);  // 发送给 AI 任务
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // 每100ms滑动一次
    }
}

/* ========== AI 推理任务 ========== */
void Task_AI(void *pv) {
    float feature[50];
    AI_Result result;
    for(;;) {
        if (xQueueReceive(qFeature, feature, portMAX_DELAY)) {
            ai_run(stance_model, feature, &result.stance);
            ai_run(arch_model,   feature, &result.arch);
            ai_run(sweat_model,  feature, &result.sweat);
            xQueueSend(qBLE, &result, 0);
        }
    }
}

/* ========== BLE 发送任务 ========== */
void Task_BLE(void *pv) {
    AI_Result res;
    char msg[64];
    for(;;) {
        if (xQueueReceive(qBLE, &res, portMAX_DELAY)) {
            sprintf(msg, "{\"stance\":%d,\"arch\":%d,\"sweat\":%d}",
                    res.stance, res.arch, res.sweat);
            ble_send(msg);
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

## 代码组织与可移植核心层

### 目录概览

- `firmware/Core`、`firmware/Drivers`：CubeMX 自动生成（外设初始化、HAL/CMSIS）。
- `firmware/MyCode`：IIC、MPU6050、Invensense DMP 等库与驱动。
- `firmware/App`：可移植核心层（本仓库新增）：
  - `app_types.h`：`SensorData`、`FeatureVector` 通用数据结构。
  - `ring_buffer.h`：单生产者/单消费者环形缓冲。
  - `feature_extract.h`：最小特征（均值/标准差，预留 FFT 能量）。
  - `ai_adapter.h`：`NanoEdgeAI.h` 轻量封装，统一推理入口。
  - `osal.h`：OS 抽象（FreeRTOS 或 HAL Delay）。
- `ai_models`：`NanoEdgeAI.h`、`knowledge.h`、`libneai.a` 等 AI 模型资源。
- `firmware/MDK-ARM`：Keil 工程与构建目录（工程文件保留，构建产物忽略）。

### 亮点

> 独立完成多传感器同步采集、200 ms 窗口特征（50 维）、NanoEdge 三模型训练与 STM32 部署；基于 FreeRTOS 的四任务并行实时系统，将足部姿态/足弓/出汗预测以 BLE 持续下发。


