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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
  uint8_t buffer[1024];
  volatile uint16_t head;
  volatile uint16_t tail;
} byte_queue_t;

typedef enum
{
  RX_SYNC_1,
  RX_SYNC_2,
  RX_SEQUENCE,
  RX_DATA,
  RX_CRC_0,
  RX_CRC_1,
  RX_CRC_2,
  RX_CRC_3
} rx_frame_state_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define QUEUE_SIZE (sizeof(((byte_queue_t *)0)->buffer))

#define WIRE_REPETITIONS 301U /* majoraty vote */
#define WIRE_PAUSE_US    50U  
#define FRAME_COPIES     3U
#define FRAME_SYNC_1     0x55U
#define FRAME_SYNC_2     0xAAU
#define FRAME_RESET_DATA 0xC3U
#define FRAME_RESET_XOR  0xA5F03C69UL
#define WIRE_MASK_COUNT  8U

#if ((WIRE_REPETITIONS % 2U) == 0U)
#error "WIRE_REPETITIONS must be odd"
#endif

#define RX_STATUS_CORRECTED 0x01U
#define RX_STATUS_LOST      0x02U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

uint8_t rx3; /* 1 byte laptop (USART3) */
uint8_t rx2; /* 1 byte cable  (USART2) */

static byte_queue_t wire_tx_queue; /* laptop -> encoded cable */
static byte_queue_t pc_tx_queue;   /* decoded cable -> laptop */
static byte_queue_t pc_status_queue;

static const uint8_t wire_masks[WIRE_MASK_COUNT] = {
    0x00U, 0xA7U, 0x3CU, 0xD2U, 0x69U, 0xF0U, 0x5BU, 0x96U};
static uint16_t repetition_bit_counts[8];
static uint16_t repetition_count;
static bool wire_cycle_counter_ready;

static rx_frame_state_t rx_frame_state = RX_SYNC_1;
static uint8_t rx_frame_bytes[6];
static bool rx_copy_corrected;
static bool rx_pending_error;
static uint8_t tx_sequence;
static uint8_t rx_expected_sequence;
static bool tx_message_start = true;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void queue_push(byte_queue_t *queue, uint8_t data);
static bool queue_pop(byte_queue_t *queue, uint8_t *data);
static void link_send_data(uint8_t data);
static void link_receive_byte(uint8_t byte);
static void message_receive_byte(uint8_t data, uint8_t status);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void queue_push(byte_queue_t *queue, uint8_t data)
{
  uint16_t next = (uint16_t)((queue->head + 1U) % QUEUE_SIZE);

  if (next != queue->tail)
  {
    queue->buffer[queue->head] = data;
    queue->head = next;
  }
}

static bool queue_pop(byte_queue_t *queue, uint8_t *data)
{
  if (queue->tail == queue->head)
  {
    return false;
  }

  *data = queue->buffer[queue->tail];
  queue->tail = (uint16_t)((queue->tail + 1U) % QUEUE_SIZE);
  return true;
}

static uint32_t crc32(uint8_t sequence, uint8_t data)
{
  uint32_t crc = 0xFFFFFFFFUL;
  uint8_t values[2] = {sequence, data};

  for (uint8_t index = 0U; index < 2U; index++)
  {
    crc ^= (uint32_t)values[index];
    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
      crc = (crc & 1UL) != 0UL
                ? (crc >> 1U) ^ 0xEDB88320UL
                : crc >> 1U;
    }
  }

  return ~crc;
}

static uint8_t rotate_left8(uint8_t value, uint8_t shift)
{
  shift &= 7U;
  if (shift == 0U)
  {
    return value;
  }

  return (uint8_t)((value << shift) | (value >> (8U - shift)));
}

static uint8_t rotate_right8(uint8_t value, uint8_t shift)
{
  shift &= 7U;
  if (shift == 0U)
  {
    return value;
  }

  return (uint8_t)((value >> shift) | (value << (8U - shift)));
}

static uint8_t encode_wire_sample(uint8_t byte, uint16_t sample_index)
{
  uint8_t phase = (uint8_t)(sample_index % WIRE_MASK_COUNT);
  return rotate_left8(byte, phase) ^ wire_masks[phase];
}

static uint8_t decode_wire_sample(uint8_t byte, uint16_t sample_index)
{
  uint8_t phase = (uint8_t)(sample_index % WIRE_MASK_COUNT);
  return rotate_right8(byte ^ wire_masks[phase], phase);
}

static void wire_delay_us(uint32_t microseconds)
{
  if (!wire_cycle_counter_ready)
  {
    volatile uint32_t fallback_loops =
        (SystemCoreClock / 4000000U) * microseconds;
    while (fallback_loops-- > 0U)
    {
      __NOP();
    }
    return;
  }

  uint32_t start = DWT->CYCCNT;
  uint32_t cycles = (SystemCoreClock / 1000000U) * microseconds;

  while ((uint32_t)(DWT->CYCCNT - start) < cycles)
  {
  }
}

static void send_repeated_byte(uint8_t byte)
{
  for (uint16_t index = 0U; index < WIRE_REPETITIONS; index++)
  {
    uint8_t wire_byte = encode_wire_sample(byte, index);
    HAL_UART_Transmit(&huart2, &wire_byte, 1U, HAL_MAX_DELAY);
    wire_delay_us(WIRE_PAUSE_US);
  }
}

static void link_send_frame(uint8_t sequence, uint8_t data, uint32_t checksum)
{
  uint8_t frame[8] = {
      FRAME_SYNC_1,
      FRAME_SYNC_2,
      sequence,
      data,
      (uint8_t)(checksum >> 24U),
      (uint8_t)(checksum >> 16U),
      (uint8_t)(checksum >> 8U),
      (uint8_t)checksum,
  };

  for (uint8_t copy = 0U; copy < FRAME_COPIES; copy++)
  {
    for (uint8_t index = 0U; index < sizeof(frame); index++)
    {
      send_repeated_byte(frame[index]);
    }
  }
}

static void link_send_data(uint8_t data)
{
  /* sync reciever */
  if (tx_message_start)
  {
    uint32_t reset_checksum = crc32(tx_sequence, FRAME_RESET_DATA) ^
                              FRAME_RESET_XOR;
    link_send_frame(tx_sequence, FRAME_RESET_DATA, reset_checksum);
    tx_message_start = false;
  }

  uint8_t sequence = tx_sequence++;
  link_send_frame(sequence, data, crc32(sequence, data));

  if ((data == '\r') || (data == '\n'))
  {
    tx_message_start = true;
  }
}

static void receive_valid_frame(uint8_t sequence, uint8_t data,
                                bool corrected)
{
  uint8_t distance = (uint8_t)(sequence - rx_expected_sequence);

  /* A positive forward distance means one or more complete characters were lost */
  if ((distance > 0U) && (distance < 128U))
  {
    while (rx_expected_sequence != sequence)
    {
      queue_push(&pc_tx_queue, (uint8_t)'?');
      queue_push(&pc_status_queue, RX_STATUS_LOST);
      rx_expected_sequence++;
    }
  }
  else if (distance >= 128U)
  {
    if (sequence == (uint8_t)(rx_expected_sequence - 1U))
    {
      return; 
    }

    rx_expected_sequence = sequence;
    rx_pending_error = true;
  }

  queue_push(&pc_tx_queue, data);
  queue_push(&pc_status_queue,
             (corrected || rx_pending_error) ? RX_STATUS_CORRECTED : 0U);
  rx_expected_sequence++;
  rx_pending_error = false;

  if (corrected)
  {
    BSP_LED_Toggle(LED_YELLOW);
  }
}

static bool frame_checksum_matches(uint32_t checksum, bool *corrected)
{
  for (uint8_t index = 0U; index < 4U; index++)
  {
    uint8_t expected = (uint8_t)(checksum >> (24U - (8U * index)));
    uint8_t received = rx_frame_bytes[index + 2U];

    if (received == expected)
    {
      continue;
    }
    if (received == (uint8_t)~expected)
    {
      *corrected = true;
      continue;
    }

    return false;
  }

  return true;
}

static void process_complete_frame(void)
{
  /* try both polarities to fix inversion */
  for (uint8_t sequence_inverted = 0U; sequence_inverted <= 1U;
       sequence_inverted++)
  {
    for (uint8_t data_inverted = 0U; data_inverted <= 1U; data_inverted++)
    {
      uint8_t sequence = rx_frame_bytes[0] ^
                         (sequence_inverted != 0U ? 0xFFU : 0x00U);
      uint8_t data = rx_frame_bytes[1] ^
                     (data_inverted != 0U ? 0xFFU : 0x00U);
      uint32_t checksum = crc32(sequence, data);
      bool polarity_corrected = (sequence_inverted != 0U) ||
                                (data_inverted != 0U);

      if (data == FRAME_RESET_DATA)
      {
        bool reset_corrected = polarity_corrected;
        if (frame_checksum_matches(checksum ^ FRAME_RESET_XOR,
                                   &reset_corrected))
        {
          rx_expected_sequence = sequence;
          rx_pending_error = false;
          if (rx_copy_corrected || reset_corrected)
          {
            BSP_LED_Toggle(LED_YELLOW);
          }
          return;
        }
      }

      if (frame_checksum_matches(checksum, &polarity_corrected))
      {
        receive_valid_frame(sequence, data,
                            rx_copy_corrected || polarity_corrected);
        return;
      }
    }
  }

  rx_pending_error = true;
  BSP_LED_Toggle(LED_RED);
}

static void process_majority_byte(uint8_t byte, bool corrected)
{
  switch (rx_frame_state)
  {
  case RX_SYNC_1:
    if ((byte == FRAME_SYNC_1) || (byte == FRAME_SYNC_2))
    {
      rx_copy_corrected = corrected || (byte != FRAME_SYNC_1);
      rx_frame_state = RX_SYNC_2;
    }
    break;

  case RX_SYNC_2:
    if ((byte == FRAME_SYNC_2) || (byte == FRAME_SYNC_1))
    {
      rx_copy_corrected |= corrected || (byte != FRAME_SYNC_2);
      rx_frame_state = RX_SEQUENCE;
    }
    else
    {
      rx_frame_state = RX_SYNC_1;
    }
    break;

  case RX_SEQUENCE:
    rx_frame_bytes[0] = byte;
    rx_copy_corrected |= corrected;
    rx_frame_state = RX_DATA;
    break;

  case RX_DATA:
    rx_frame_bytes[1] = byte;
    rx_copy_corrected |= corrected;
    rx_frame_state = RX_CRC_0;
    break;

  case RX_CRC_0:
    rx_frame_bytes[2] = byte;
    rx_copy_corrected |= corrected;
    rx_frame_state = RX_CRC_1;
    break;

  case RX_CRC_1:
    rx_frame_bytes[3] = byte;
    rx_copy_corrected |= corrected;
    rx_frame_state = RX_CRC_2;
    break;

  case RX_CRC_2:
    rx_frame_bytes[4] = byte;
    rx_copy_corrected |= corrected;
    rx_frame_state = RX_CRC_3;
    break;

  case RX_CRC_3:
    rx_frame_bytes[5] = byte;
    rx_copy_corrected |= corrected;
    process_complete_frame();
    rx_frame_state = RX_SYNC_1;
    break;
  }
}

static void link_receive_byte(uint8_t byte)
{
  byte = decode_wire_sample(byte, repetition_count);

  for (uint8_t bit = 0U; bit < 8U; bit++)
  {
    repetition_bit_counts[bit] += (uint16_t)((byte >> bit) & 1U);
  }

  repetition_count++;
  if (repetition_count < WIRE_REPETITIONS)
  {
    return;
  }

  uint8_t majority_byte = 0U;
  bool corrected = false;
  for (uint8_t bit = 0U; bit < 8U; bit++)
  {
    if (repetition_bit_counts[bit] > (WIRE_REPETITIONS / 2U))
    {
      majority_byte |= (uint8_t)(1U << bit);
    }
    if ((repetition_bit_counts[bit] != 0U) &&
        (repetition_bit_counts[bit] != WIRE_REPETITIONS))
    {
      corrected = true;
    }
    repetition_bit_counts[bit] = 0U;
  }

  repetition_count = 0U;
  process_majority_byte(majority_byte, corrected);
}

static void message_receive_byte(uint8_t data, uint8_t status)
{
  (void)status;
  HAL_UART_Transmit(&huart3, &data, 1U, HAL_MAX_DELAY);
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
  MX_USART3_UART_Init();

  /* Cycle counter for the short pacing delay between cable bytes. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->LAR = 0xC5ACCE55UL;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  uint32_t cycle_counter_start = DWT->CYCCNT;
  for (volatile uint32_t index = 0U; index < 100U; index++)
  {
    __NOP();
  }
  wire_cycle_counter_ready = DWT->CYCCNT != cycle_counter_start;

  /* Green: ready. Yellow: majority vote used. Red: CRC rejected a copy. */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);
  BSP_LED_On(LED_GREEN);

  uint8_t hello[] = "Robust link ready. Characters are printed immediately.\r\n";
  HAL_UART_Transmit(&huart3, hello, sizeof(hello) - 1U, HAL_MAX_DELAY);

  HAL_UART_Receive_IT(&huart3, &rx3, 1U);
  HAL_UART_Receive_IT(&huart2, &rx2, 1U);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint8_t data;

    /* Laptop -> cable: strongly encode one queued character. */
    if (queue_pop(&wire_tx_queue, &data))
    {
      link_send_data(data);
    }

    /* Cable -> laptop: collect characters and print the complete message. */
    if (queue_pop(&pc_tx_queue, &data))
    {
      uint8_t status = 0U;
      (void)queue_pop(&pc_status_queue, &status);
      message_receive_byte(data, status);
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
   */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
  {
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    uint8_t byte = rx3;

    /* Re-arm */
    HAL_UART_Receive_IT(&huart3, &rx3, 1U);

    /* Laptop -> cable */
    queue_push(&wire_tx_queue, byte);
  }
  else if (huart->Instance == USART2)
  {
    uint8_t byte = rx2;

    HAL_UART_Receive_IT(&huart2, &rx2, 1U);
    link_receive_byte(byte);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
 
  if (huart->Instance == USART2)
  {
    repetition_count = 0U;
    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
      repetition_bit_counts[bit] = 0U;
    }
    rx_frame_state = RX_SYNC_1;
    BSP_LED_Toggle(LED_RED);
    HAL_UART_Receive_IT(&huart2, &rx2, 1U);
  }
  else if (huart->Instance == USART3)
  {
    HAL_UART_Receive_IT(&huart3, &rx3, 1U);
  }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug User can add his own implementation to report the HAL error return state */
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
