/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief Stack Overflow Detection Demonstration in a FreeRTOS-based System
 *
 * This demonstration showcases the importance of stack management in embedded systems
 * running FreeRTOS and illustrates the functionality of FreeRTOS's stack overflow
 * detection mechanism.
 *
 * ## Overview
 *
 * The demo consists of two primary tasks:
 *
 * 1. `UART_task`: This task is responsible for continuously sending a predefined
 *    message ("Hello, UART!\r\n") over UART at one-second intervals. The consistent
 *    operation of this task serves as an indicator of the system's stability.
 *
 * 2. `stackoverflow_t`: A task intentionally designed to consume more stack space
 *    than allocated, leading to a stack overflow condition. This task demonstrates how
 *    an overflow can compromise system stability and functionality.
 *
 * ## Objective
 *
 * The primary objective is to demonstrate the difference in system behavior with and
 * without FreeRTOS's stack overflow detection enabled:
 *
 * - With stack overflow detection disabled, the system may exhibit erratic behavior or
 *   crash after sending a few UART messages, indicating that the stack overflow has
 *   corrupted the system state.
 *
 * - With stack overflow detection enabled (via the `configCHECK_FOR_STACK_OVERFLOW`
 *   setting in `FreeRTOSConfig.h`), the system is configured to respond to the overflow
 *   condition by executing the `vApplicationStackOverflowHook` function. In this demo,
 *   the hook function toggles an LED to visually indicate that a stack overflow has
 *   been detected, providing a clear and immediate indication of the issue.
 *
 * ## Setup Instructions
 *
 * 1. Ensure that the UART peripheral is correctly initialized and configured for
 *    communication at the desired baud rate. (should already be done within this example)
 *
 * 2. Verify that the LED (or other indicator used by `vApplicationStackOverflowHook`)
 *    is properly initialized. (should already be done within this example)
 *
 * 3. Configure `configCHECK_FOR_STACK_OVERFLOW` in `FreeRTOSConfig.h` to `0` to observe
 *    the system behavior without overflow detection, and then to `1` or `2` to enable
 *    detection and observe the alternate behavior.
 *    --> This is done within the IOC under the FreeRTOS configurations.
 *    --> Search for the "Config Parameters" and there look for "CHECK_FOR_STACK_OVERFLOW".
 *    --> Here you can disable it or set it to option1 or option2.
 *    IMPORTANT: This demo will only show a difference if option2 is selected!!!!!
 *
 *    You know need to write the content of the `vApplicationStackOverflowHook` function.
 *    Copy-paste the following code into the corresponding location within the "freertos.c" file.
 *
 *    * void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
 *   // Prevent unused parameter compiler warnings.
 *   (void) xTask;
 *   (void) pcTaskName;
 *
 *   // Enter an infinite loop, indicating error condition.
 *   while(1) {
 *       // Optionally toggle an LED or similar to indicate error visually
 *       HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
 *       for (volatile int i = 0; i < 1000000; i++) {} // Basic delay
 *   }
 *
 *
 * 4. Build and flash the firmware to the target device, monitoring the UART output and
 *    LED status. Use the system in debug mode!
 *
 * ## Expected Outcomes
 *
 * - **Without Stack Overflow Detection**: The system should begin sending UART messages
 *   but may soon behave unpredictably or crash due to the stack overflow induced by
 *   `stackoverflow_t`.
 *
 * - **With Stack Overflow Detection**: The system will trigger `vApplicationStackOverflowHook`
 *   upon detecting a stack overflow, resulting in the LED toggling to signal the error
 *   condition, thus preventing undefined system behavior or a crash.
 *
 * This demonstration underscores the critical role of stack size management and overflow
 * detection in ensuring the reliability and robustness of embedded systems running FreeRTOS.
 *
 *
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"

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
UART_HandleTypeDef huart2;

/* Definitions for stackoverflow_t */
osThreadId_t stackoverflow_tHandle;
const osThreadAttr_t stackoverflow_t_attributes = {
  .name = "stackoverflow_t",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UART_task */
osThreadId_t UART_taskHandle;
const osThreadAttr_t UART_task_attributes = {
  .name = "UART_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void stackoverflow_f(void *argument);
void UART_task_func(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void RecursiveFunction(int depth) {
	volatile uint32_t stackArray[1]; // Consumes stack space with each call
	if (depth > 0) {
		RecursiveFunction(depth - 1);
	} else {
		// Base case: do nothing, just return
	}
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of stackoverflow_t */
  stackoverflow_tHandle = osThreadNew(stackoverflow_f, NULL, &stackoverflow_t_attributes);

  /* creation of UART_task */
  UART_taskHandle = osThreadNew(UART_task_func, NULL, &UART_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_stackoverflow_f */
/**
 * @brief  Function implementing the stackoverflow_t thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_stackoverflow_f */
void stackoverflow_f(void *argument)
{
  /* USER CODE BEGIN 5 */
	uint32_t recursions = 0;

	while (1) {
		// Start recursive calls with a depth that's likely to overflow the stack
		RecursiveFunction(recursions); // Adjust this depth based on your system's limits
		recursions++;

		osDelay(100); // Loop indefinitely if we somehow avoid overflow
	}
	/*
	 * If we run this application with this task in a standard configuration where the task stack size is 128 words long - your uC will crash!
	 * You will end in the "HardFault_Handler" observe this via the debugger.
	 *
	 * How can we prevent this from happening?
	 * 1. During programming keep an eye on variable sizes and recursive function calls which will require memory.
	 * 2. Activate Stack Overflow Detection in the FreeRTOS configuration --> inside the IOC go to "Config Parameters" and here choose "Option1" for "CHECK_FOR_STACK_OVERFLOW"
	 * 3. Consider using static stack allocation as this will allow you to precisely control the stack size of each task and monitor its usage more easily.
	 * Static allocation also eliminates the risk of heap fragmentation that could affect dynamic allocation.
	 * 4. Monitor stack usage using FreeRTOS functions
	 * 5. Code analysis tools, code reviews, testing and verification
	 */


  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_UART_task_func */
/**
 * @brief Function implementing the UART_task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_UART_task_func */
void UART_task_func(void *argument)
{
  /* USER CODE BEGIN UART_task_func */
	/* Infinite loop */
	for(;;)
	{
		// Ensure UART is properly initialized before this task runs
		const char msg[] = "Hello, UART!\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg) - 1, HAL_MAX_DELAY);
		osDelay(1000); // Delay for 1 second
	}
  /* USER CODE END UART_task_func */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

#ifdef  USE_FULL_ASSERT
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
