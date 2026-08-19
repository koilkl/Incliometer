/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 供 STM32CubeMonitor 和串口打印读取的全局角度变量
volatile float angle_roll  = 0.0f;
volatile float angle_pitch = 0.0f;
volatile float angle_yaw   = 0.0f;

// 预设的 Modbus 查询指令帧 (读 0000H~0002H: Roll, Pitch, Yaw)
uint8_t modbus_query_cmd[8] = {0x05, 0x04, 0x00, 0x00, 0x00, 0x03, 0xB1, 0x8F};
uint8_t rx_buffer[32]; // 接收缓冲区稍微放大，防止越界

// ⭐ PA15 亮灯 + 初始位置稳定检测相关变量
volatile uint8_t start_read = 0;   // 第一个变量：置 true 后开始读取 IMU 并检测初始位置
volatile uint8_t settled    = 0;   // 第二个变量：保持初始位置满 5 秒后置 true

#define STABLE_THRESHOLD_DEG  5.0f  // 初始位置阈值(度)：相对基准位置的偏差小于它即视为"仍在初始位置"
#define STABLE_HOLD_MS        5000  // 需要保持在初始位置的时间(毫秒)

static uint8_t   in_position       = 0; // 当前是否正处于初始位置
static uint32_t  stable_start_tick = 0; // 进入初始位置的系统时刻(ms)

// ⭐ 初始位置基准：收到 start_read=1 时记录当前角度，之后检测是否偏离该基准
static float baseline_roll  = 0.0f;
static float baseline_pitch = 0.0f;
static float baseline_yaw   = 0.0f;

// ⭐ 终端(USART2)命令接收变量
volatile uint8_t terminal_rx_byte     = 0; // 中断接收到的字符
volatile uint8_t terminal_cmd_pending = 0; // 主循环待处理命令标志
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
#else
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
#endif

// Modbus CRC16 校验函数声明
uint16_t Modbus_CRC16(const uint8_t *buffer, uint16_t length);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Modbus CRC16 校验函数实现
uint16_t Modbus_CRC16(const uint8_t *buffer, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= buffer[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // ⭐ 核心修改 1：弃用定长接收，改用带有“空闲中断”的不定长接收
  // 只要总线空闲超过一个字节的时间，就会立即触发回调，彻底杜绝字节错位！
  HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_buffer, sizeof(rx_buffer));

  // ⭐ 终端(USART2)命令接收：启用中断并接收单字节命令
  HAL_NVIC_SetPriority(USART2_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  HAL_UART_Receive_IT(&huart2, (uint8_t *)&terminal_rx_byte, 1);

  printf("\r\n=== Terminal commands ===\r\n");
  printf("'1' or 's' : start_read = 1 (begin detection)\r\n");
  printf("'0' or 'r' : start_read = 0 (stop detection)\r\n\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 每隔 100ms 向水平仪发送一次查询指令 (给接收留出充足余量)
    HAL_UART_Transmit(&huart1, modbus_query_cmd, 8, 100); 
    
    // 打印三个角度到电脑
    printf("Roll: %.2f | Pitch: %.2f | Yaw: %.2f \r\n", angle_roll, angle_pitch, angle_yaw); 
    
    HAL_Delay(100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // ⭐ 终端命令处理：通过 PC 串口(UART2) 输入字符来修改 start_read
    if (terminal_cmd_pending)
    {
        terminal_cmd_pending = 0;

        switch (terminal_rx_byte)
        {
            case '1':
            case 's':
            case 'S':
                baseline_roll  = angle_roll;   // 记录当前角度作为初始位置基准
                baseline_pitch = angle_pitch;
                baseline_yaw   = angle_yaw;
                start_read  = 1;
                settled     = 0;        // 重新开始一轮检测
                in_position = 0;
                printf("[CMD] start_read = 1 (baseline R=%.2f P=%.2f Y=%.2f)\r\n",
                       baseline_roll, baseline_pitch, baseline_yaw);
                break;

            case '0':
            case 'r':
            case 'R':
                start_read  = 0;
                in_position = 0;
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // 停止时灭灯
                printf("[CMD] start_read = 0\r\n");
                break;

            default:
                printf("[CMD] unknown cmd: '%c'\r\n", terminal_rx_byte);
                break;
        }
    }

    // ⭐ PA15 亮灯 + 初始位置稳定检测：
    // start_read 置 true 后，检测水平仪是否保持在初始位置(相对基准偏差在阈值内)。
    // 处于初始位置时点亮 PA15 并计时，保持满 5 秒则置 settled=true 并复位 start_read。
    if (start_read)
    {
        uint8_t at_initial = (angle_roll  > baseline_roll  - STABLE_THRESHOLD_DEG && angle_roll  < baseline_roll  + STABLE_THRESHOLD_DEG) &&
                             (angle_pitch > baseline_pitch - STABLE_THRESHOLD_DEG && angle_pitch < baseline_pitch + STABLE_THRESHOLD_DEG) &&
                             (angle_yaw   > baseline_yaw   - STABLE_THRESHOLD_DEG && angle_yaw   < baseline_yaw   + STABLE_THRESHOLD_DEG);

        if (at_initial)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // 亮灯

            if (!in_position)                       // 刚进入初始位置，开始计时
            {
                in_position = 1;
                stable_start_tick = HAL_GetTick();
            }

            uint32_t elapsed_ms = HAL_GetTick() - stable_start_tick;

            if (elapsed_ms >= STABLE_HOLD_MS)
            {
                // 已稳定保持 5 秒：确认稳定，复位触发变量
                settled = 1;
                start_read = 0;
                in_position = 0;
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // 完成，灭灯
                printf("[OK] stable %d ms -> settled = 1\r\n", STABLE_HOLD_MS);
            }
            else
            {
                // 打印计时时间
                printf("hold: %lu / %d ms\r\n", (unsigned long)elapsed_ms, STABLE_HOLD_MS);
            }
        }
        else
        {
            // 偏离初始位置：灭灯并重新计时
            in_position = 0;
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
        }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// ⭐ 终端(USART2)接收回调：收到字符后置待处理标志，并重新开启接收
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        terminal_cmd_pending = 1;
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&terminal_rx_byte, 1);
    }
}

// ⭐ 核心修改 2：使用串口空闲中断回调（替代原本的 RxCpltCallback）
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        // 确保收到的字节数足够且帧头正确（站号 0x05, 功能码 0x04, 数据字节 0x06）
        if (Size >= 11 && rx_buffer[0] == 0x05 && rx_buffer[1] == 0x04 && rx_buffer[2] == 0x06) 
        {
            // 提取传感器传来的 CRC 值 (低字节在前，高字节在后)
            uint16_t received_crc = rx_buffer[9] | (rx_buffer[10] << 8);
            
            // 单片机自己计算前 9 个字节的 CRC 值
            uint16_t calculated_crc = Modbus_CRC16(rx_buffer, 9);

            // 只有当两者完全一致时，才确认数据有效并进行更新
            if (received_crc == calculated_crc)
            {
                int16_t raw_roll  = (int16_t)((rx_buffer[3] << 8) | rx_buffer[4]); 
                int16_t raw_pitch = (int16_t)((rx_buffer[5] << 8) | rx_buffer[6]); 
                int16_t raw_yaw   = (int16_t)((rx_buffer[7] << 8) | rx_buffer[8]); 

                angle_roll  = raw_roll / 100.0f; 
                angle_pitch = raw_pitch / 100.0f; 
                angle_yaw   = raw_yaw / 100.0f; 
            }
        }
        
        // 处理完毕后，重新开启不定长空闲中断接收
        HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_buffer, sizeof(rx_buffer));
    }
}

// ⭐ 核心修改 3：增加错误回调，防止因程序卡顿导致串口溢出错误 (ORE) 彻底死机
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // 强行清除所有错误标志位
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        
        // 重新开启接收，让串口“起死回生”
        HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_buffer, sizeof(rx_buffer));
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
