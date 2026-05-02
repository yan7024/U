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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "rtklib_app.h"
#include "rtklib_port.h"
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
static uint32_t s_main_hb_ms = 0u;
static volatile uint32_t s_uart1_hb_pulse_count;
static uint32_t s_uart1_hb_pulse_seen;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_User_LED_Init();
  MX_DMA_Init();
  /* USART2 before USART1: ST-Link COM works even if USART1/DMA init fails */
  MX_USART2_UART_Init();
  {
    static const char k_early[] = "\r\n$BOOT,EARLY_USART2\r\n";
    rtklib_port_uart2_send((const uint8_t *)k_early, (uint16_t)(sizeof(k_early) - 1u));
  }
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  /* $BOOT via debug_send → USART3 + USART2 mirror (ST-Link COM at 115200 for SSCOM). */
  {
    static const char k_boot[] = "\r\n$BOOT,MCU_OK\r\n";
    const uint16_t blen = (uint16_t)(sizeof(k_boot) - 1u);
    rtklib_port_debug_send((const uint8_t *)k_boot, blen);
  }
  /* Init RTK before RX IRQ: init_raw uses ~700 B stack; nested USART ISR + flood risks overflow */
  rtklib_init();
  rtklib_uart_start_rx_it();
  /* F9P: assume 115200 link first; rtklib_process retries 38400 if rx1 stays 0 */
  rtklib_rover_ubx_link_train(115200u);
  /* Independent of main-loop RTK work: periodic ISR proves MCU alive + UART TX path */
  (void)HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    rtklib_process();
    /*
     * $HB (USART1 or USART3 per rtklib_app.h / rtklib_port_debug_send) — TIM2 ISR counts pulses only.
     * (HAL_UART_Transmit from ISR + $PRTKMON interleaved bytes on the wire.)
     */
    while (s_uart1_hb_pulse_seen != s_uart1_hb_pulse_count) {
      char buf[112];
      int n;

      s_uart1_hb_pulse_seen++;
      n = snprintf(buf, sizeof(buf),
                   "$HB,t=%lu,r1=%lu,r2=%lu,u=%lu,p2=%lu,p1=%lu\r\n",
                   (unsigned long)HAL_GetTick(),
                   (unsigned long)rtklib_serial_get_rx1_total(),
                   (unsigned long)rtklib_serial_get_rx2_total(),
                   (unsigned long)rtklib_serial_get_ubx_sync_total(),
                   (unsigned long)rtklib_serial_get_rtcm_preamble_total(),
                   (unsigned long)rtklib_serial_get_rtcm_preamble_uart1_total());
      if (n > 0 && n < (int)sizeof(buf)) {
        rtklib_port_debug_send((const uint8_t *)buf, (uint16_t)n);
      }
    }
    /*
     * LD2: always 1 Hz toggle = main loop alive.
     * (Old logic kept LD2 solid ON when basN>0 — looks like "fault" and hides hangs.)
     * Base RTCM status: use $PRTKMON basN / rtklib_get_base_obs_ready().
     */
    if ((HAL_GetTick() - s_main_hb_ms) >= 1000u) {
      s_main_hb_ms = HAL_GetTick();
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
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
  uint32_t pll_ok_hse = 1u;

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /*
   * Nucleo-L476RG: 8 MHz HSE + PLL -> 80 MHz.
   * Custom boards without crystal: HAL_RCC_OscConfig(HSE) fails — fall back to HSI16 + PLL (still 80 MHz SYSCLK).
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    pll_ok_hse = 0u;
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 10;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();
    }
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  if (pll_ok_hse != 0u)
  {
    HAL_RCC_EnableCSS();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM2) {
    return;
  }
  /* UART TX happens in main loop — ISR must stay minimal */
  s_uart1_hb_pulse_count++;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  volatile uint32_t i;
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    for (i = 0; i < 300000u; i++) {
      __NOP();
    }
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
