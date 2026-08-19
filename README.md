# Incliometer

基于 STM32F401RE（NUCLEO-F401RE）的水平仪数据采集与初始位置稳定检测项目。

通过 USART1（Modbus-RTU）读取倾角传感器的 Roll / Pitch / Yaw 角度，通过 USART2 将角度打印到 PC，并支持从串口终端发送命令控制「初始位置稳定检测」流程：回到初始位置 → 点亮 PA15 LED → 计时 5 秒 → 确认稳定。

## 硬件连接

| 功能 | 引脚 | 说明 |
|------|------|------|
| USART1 TX | PA9 | 连接倾角传感器 RX |
| USART1 RX | PA10 | 连接倾角传感器 TX |
| USART2 TX | PA2 | 连接 USB-TTL / ST-Link VCP 的 RX |
| USART2 RX | PA3 | 连接 USB-TTL / ST-Link VCP 的 TX |
| LED（稳定指示） | PA15 | 初始位置稳定期间点亮 |
| 板载 LED | PA5 (LD2) | 板载用户 LED |

- 主控：STM32F401RETx，系统时钟 84 MHz（HSI → PLL）
- 倾角传感器：Modbus-RTU，从站地址 `0x05`，功能码 `0x04`，读取 3 个寄存器（Roll / Pitch / Yaw），数值为实际角度 × 100 的有符号 16 位整数

## 串口配置

| 串口 | 波特率 | 用途 |
|------|--------|------|
| USART1 | 115200 | 与倾角传感器通信（Modbus-RTU） |
| USART2 | 115200 | 与 PC 终端通信（printf / 命令） |

## 工作原理

### 1. 角度读取

主循环每 100 ms 向传感器发送一次 Modbus 查询指令：

```
0x05 0x04 0x00 0x00 0x00 0x03 0xB1 0x8F
```

响应通过 USART1 的 **空闲中断（`HAL_UARTEx_ReceiveToIdle_IT`）** 不定长接收，校验 CRC16-Modbus 后更新全局角度变量：

```c
volatile float angle_roll;
volatile float angle_pitch;
volatile float angle_yaw;
```

### 2. 终端命令控制

通过 USART2 发送**单个字符**命令：

| 命令 | 作用 |
|------|------|
| `1` / `s` / `S` | 记录当前角度为基准位置，并设置 `start_read = 1`，开始检测 |
| `0` / `r` / `R` | 设置 `start_read = 0`，停止检测并熄灭 LED |

启动时会打印帮助信息：

```
=== Terminal commands ===
'1' or 's' : start_read = 1 (begin detection)
'0' or 'r' : start_read = 0 (stop detection)
```

### 3. 初始位置稳定检测

`start_read` 置 1 时：

1. 记录当前角度作为**基准位置**（baseline）
2. 持续检测当前角度是否偏离基准超过阈值（默认 `±5°`，见 `STABLE_THRESHOLD_DEG`）
3. 处于初始位置时：点亮 **PA15**，开始计时，并每 100 ms 打印 `hold: <elapsed> / 5000 ms`
4. 稳定保持 **5 秒**（`STABLE_HOLD_MS`）后：设置 `settled = 1`，复位 `start_read = 0`，熄灭 LED
5. 中途偏离基准：熄灭 LED 并重新计时

关键变量：

```c
volatile uint8_t start_read;  // 置 1 后开始检测
volatile uint8_t settled;     // 稳定 5 秒后置 1
```

> **注意**：初始位置是**相对基准位置**，而不是绝对 0°。因为 Yaw 可能处于任意角度（例如 -54°），若以绝对 0° 为基准会导致检测永远无法触发。

## 可调参数

在 `Src/main.c` 的 `USER CODE BEGIN PV` 区域：

| 宏 | 默认值 | 说明 |
|----|--------|------|
| `STABLE_THRESHOLD_DEG` | `5.0f` | 偏离基准位置的容差（度） |
| `STABLE_HOLD_MS` | `5000` | 需要稳定保持的时间（毫秒） |

## 构建

项目使用 STM32CubeMX 生成，支持 CMake + Ninja 构建（预设为 `Debug` / `Release`）：

```bash
cmake --preset Debug
cmake --build build/Debug
```

也可以在 STM32CubeIDE / CLion 中直接打开 `CMakeLists.txt` 或 `.ioc` 工程构建。

## 代码结构

```
Src/main.c              # 主程序：Modbus 查询、终端命令、稳定检测、UART 回调
Src/gpio.c              # GPIO 初始化（PA15、LD2、B1 按键）
Src/usart.c             # USART1 / USART2 初始化
Src/stm32f4xx_it.c      # 中断服务程序（USART1、USART2）
Inc/main.h              # 引脚定义与导出
```

## 终端输出示例

```
[CMD] start_read = 1 (baseline R=-0.01 P=0.02 Y=-54.02)
Roll: -0.01 | Pitch: 0.01 | Yaw: -54.02
hold: 100 / 5000 ms
hold: 200 / 5000 ms
...
hold: 4900 / 5000 ms
[OK] stable 5000 ms -> settled = 1
```
