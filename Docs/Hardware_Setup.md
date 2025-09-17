# Hardware Setup Guide

## Overview
FootSense is designed around the STM32F407VG microcontroller with multiple sensors for comprehensive foot health monitoring.

## Hardware Components

### Main Processing Unit
- **STM32F407VG**: ARM Cortex-M4F, 168MHz
- **Flash**: 1MB for firmware and AI models
- **RAM**: 192KB for real-time processing
- **FPU**: Hardware floating point for AI calculations

### Sensor Array
- **Pressure Sensors**: 16x FSR (Force Sensitive Resistor) sensors
- **IMU**: MPU-6050 or MPU-9250 (6/9-axis motion sensor)
- **Environmental**: BME280 (temperature, humidity, pressure)
- **Power**: Battery monitoring via ADC

### Communication Modules
- **BLE**: ESP32 or Nordic nRF52 module
- **Debug**: ST-Link V2 or compatible
- **USB**: Type-C connector for charging and data

### Power Management
- **Battery**: 3.7V Li-Po, 1000mAh capacity
- **Charging**: USB-C with protection circuit
- **Regulators**: 3.3V and 1.8V for different components

## Pin Configuration

### GPIO Assignments
```
PA0  - Sensor Power Control
PA5  - SPI1 SCK (IMU)
PA6  - SPI1 MISO (IMU)
PA7  - SPI1 MOSI (IMU)
PA9  - USART1 TX (BLE Module)
PA10 - USART1 RX (BLE Module)

PB6  - I2C1 SCL (Environmental Sensor)
PB7  - I2C1 SDA (Environmental Sensor)

PC13 - Status LED
```

### Analog Inputs (Pressure Sensors)
```
PA1-PA8   - Arch pressure sensors (8 sensors)
PB0-PB3   - Heel pressure sensors (4 sensors)
PC0-PC3   - Toe pressure sensors (4 sensors)
```

## Wiring Diagram

```
STM32F407VG
     │
     ├── I2C1 ────── BME280 (Environmental)
     ├── SPI1 ────── MPU-6050 (IMU)
     ├── UART1 ───── BLE Module
     ├── UART2 ───── Debug Console
     ├── ADC1 ────── Pressure Sensor Array
     └── GPIO ────── Power Control & LEDs
```

## Assembly Instructions

### 1. PCB Assembly
1. Mount STM32F407VG in LQFP100 package
2. Install crystal oscillator (25MHz)
3. Add decoupling capacitors and power regulators
4. Mount sensor connectors and communication modules

### 2. Sensor Placement
1. Arrange pressure sensors in foot-shaped pattern
2. Place IMU at heel for optimal motion detection
3. Position environmental sensor for accurate readings
4. Ensure proper isolation between sensors

### 3. Enclosure Design
1. Use flexible PCB for pressure sensors
2. Waterproof rating IPX7 for sweat resistance
3. Comfortable insole form factor
4. Easy battery access for charging

## Debugging Setup

### ST-Link Connection
```
ST-Link    STM32F407
VCC   →    3.3V
GND   →    GND
SWDIO →    PA13
SWCLK →    PA14
```

### Serial Debug (UART2)
```
USB-TTL    STM32F407
GND   →    GND
TX    →    PA3 (UART2_RX)
RX    →    PA2 (UART2_TX)
```

## Power Consumption Analysis

### Typical Operating Modes
- **Active Monitoring**: 45mA @ 3.3V
- **Idle State**: 15mA @ 3.3V
- **Sleep Mode**: 2mA @ 3.3V
- **Deep Sleep**: 0.5mA @ 3.3V

### Battery Life Estimation
- **Continuous Use**: ~20 hours
- **Normal Use**: ~3-5 days
- **Standby**: ~2 weeks

## Calibration Procedure

### Initial Setup
1. Power on device with no load
2. Perform zero-point calibration for pressure sensors
3. IMU bias calibration (flat surface, stationary)
4. Environmental sensor baseline establishment

### User Calibration
1. Step on device with known weight
2. Record pressure distribution
3. Perform gait calibration walk
4. Validate against reference measurements

## Troubleshooting

### Common Issues
- **No sensor readings**: Check power and I2C/SPI connections
- **Erratic IMU data**: Verify crystal oscillator and grounding
- **Communication failure**: Check UART baud rate and wiring
- **High power consumption**: Review sleep mode configuration

### Debug Commands
```bash
# Check sensor status
echo "sensor_status" > /dev/ttyUSB0

# Calibrate pressure sensors
echo "calibrate_pressure" > /dev/ttyUSB0

# Reset to factory defaults
echo "factory_reset" > /dev/ttyUSB0
```

## Safety Considerations

### Electrical Safety
- Ensure proper ESD protection during assembly
- Use appropriate voltage levels for each component
- Implement overcurrent protection for battery charging

### Medical Device Considerations
- Follow IEC 60601 guidelines for medical electronics
- Ensure biocompatibility of materials in contact with skin
- Implement proper data privacy and security measures