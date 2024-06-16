/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Demonstration of printf retargeting to UART in a FreeRTOS environment
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software component is licensed under terms that can be found in the
 * LICENSE file in the root directory of this software component. If no LICENSE
 * file comes with this software component, it is provided AS-IS.
 *
 ******************************************************************************
 *
 * ## Demonstration Overview
 *
 * This example demonstrates how to retarget the `printf` function to send output
 * over UART instead of the standard output (STDOUT), which is not natively
 * available on bare-metal microcontroller systems. By retargeting `printf`, we can
 * use it for debugging or sending messages over UART, enhancing the debugging
 * capabilities during development.
 *
 * ## Key Components
 *
 * - `MX_USART2_UART_Init()`: Initializes the USART2 peripheral for UART communication.
 * - `__io_putchar()`: Overrides the default putchar function to transmit characters over UART.
 * - `_write()`: System call override used by newlib (or similar C libraries) to implement
 *    low-level write functionality. This function calls `__io_putchar()` to transmit data.
 *
 * ## Tasks
 *
 * - `StartDefaultTask`: A placeholder default task, demonstrating the FreeRTOS task setup.
 * - `demo_task_GPIO_func`: Toggles an LED at a fixed interval, serving as a simple visual indicator.
 * - `demo_task_UART_func`: Periodically sends a message over UART using the retargeted `printf`.
 *
 * ## How It Works
 *
 * The `printf` function is commonly used for sending formatted output to STDOUT. In embedded
 * systems, STDOUT is not available, requiring us to retarget `printf` to a different output
 * mechanism, such as UART. This is achieved by overriding the `_write` system call and
 * `__io_putchar()` function, directing `printf` output to UART.
 *
 * This demonstration configures a UART peripheral for communication and implements the necessary
 * functions for `printf` retargeting. It includes tasks for visual (LED) and UART output to
 * illustrate real-time system behavior and debugging techniques in a FreeRTOS environment.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {                                //message object data type
    uint32_t counter;
    uint8_t taskID;
  } MSGQUEUE_OBJ_t;

  typedef struct {                                // command object data type
      uint32_t ivTask1;
      uint32_t ivTask2;
      uint32_t ivTask3;

    } CMDQUEUE_OBJ_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MSGQUEUE_OBJECTS 16
#define BUFFER_SIZE  3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* Definitions for Monitoring_Task */
osThreadId_t Monitoring_TaskHandle;
const osThreadAttr_t Monitoring_Task_attributes = {
  .name = "Monitoring_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ProducerTask01 */
osThreadId_t ProducerTask01Handle;
const osThreadAttr_t ProducerTask01_attributes = {
  .name = "ProducerTask01",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ProducerTask02 */
osThreadId_t ProducerTask02Handle;
const osThreadAttr_t ProducerTask02_attributes = {
  .name = "ProducerTask02",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ProducerTask03 */
osThreadId_t ProducerTask03Handle;
const osThreadAttr_t ProducerTask03_attributes = {
  .name = "ProducerTask03",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

osMessageQueueId_t ProducerMessageQueue;

osMessageQueueId_t CommandMessageQueue;


uint8_t uart_rx_buffer[BUFFER_SIZE];

CMDQUEUE_OBJ_t commando;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void Monitoring_Task_Entry(void *argument);
void ProducerTask01_Entry(void *argument);
void ProducerTask02_Entry(void *argument);
void ProducerTask03_Entry(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief   Print given character on UART 2. Translate '\n' to "\r\n" on the fly.
 *          Retargets printf function from stdout to UART interface by overwriting putchar function.
 *          This function is a custom implementation of the low-level IO function used by printf.
 *          By retargeting stdout to UART, we enable printf to send formatted strings over UART.
 * @note    This approach is particularly useful in embedded systems where direct console
 *          access is not available, and UART is commonly used for logs and debugging output.
 * @param   int ch ... Character to be transmitted.
 * @retval  (int) ... Return character.
 */
int __io_putchar(int ch)
{
	int ret;
	while ((ret=HAL_UART_GetState(&huart2)) == HAL_UART_STATE_BUSY_TX)
		;

	if (ch == '\n')
	{
		static uint8_t buf[2] = { '\r', '\n' };
		HAL_UART_Transmit(&huart2, buf, sizeof(buf), 1000);
	}
	else
	{
		static char buf;
		buf = ch;
		HAL_UART_Transmit(&huart2, (uint8_t *)&buf, 1, 1000);
	}
	return ch;
}

/**
 * @brief   Called by "printf" to output formatted string by calling __io_putchar() function.
 *          Used to retarget printf function from stdout to UART interface.
 *          This is part of the low-level system IO overrides that direct printf output
 *          through our custom UART transmission logic.
 * @note    _write is a system call that newlib's printf relies on to output characters.
 *          By overriding this function, all standard IO operations can be redirected to
 *          use UART, making printf an effective tool for embedded debugging.
 * @param   int file ... parameter not used by this retarget implementation (normally used as a file pointer)
 * @param   char *ptr ... ptr to data array
 * @param   int len ... length of data to be transmitted (in byte)
 * @retval  (int) ... Return length in byte.
 */
int _write(int file, char *ptr, int len)
{
	for(int DataIdx = 0; DataIdx < len; DataIdx++) {
		__io_putchar(*ptr++);
	}
	return len;
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

  HAL_UART_Receive_IT(&huart2, uart_rx_buffer, BUFFER_SIZE);

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

//Message queue from Producers to Monitoring Task
ProducerMessageQueue = osMessageQueueNew(MSGQUEUE_OBJECTS, sizeof(MSGQUEUE_OBJ_t), NULL);

//Message queue from Uart to Monitoring Task tbd
CommandMessageQueue = osMessageQueueNew(MSGQUEUE_OBJECTS, sizeof(CMDQUEUE_OBJ_t), NULL);



	if(ProducerMessageQueue== NULL)
		printf("creation of message queue failed!\n");
	else
		printf("message queue created successfully! ID: %d\n", (int)ProducerMessageQueue);

	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Monitoring_Task */
  Monitoring_TaskHandle = osThreadNew(Monitoring_Task_Entry, NULL, &Monitoring_Task_attributes);

  /* creation of ProducerTask01 */
  ProducerTask01Handle = osThreadNew(ProducerTask01_Entry, NULL, &ProducerTask01_attributes);

  /* creation of ProducerTask02 */
  ProducerTask02Handle = osThreadNew(ProducerTask02_Entry, NULL, &ProducerTask02_attributes);

  /* creation of ProducerTask03 */
  ProducerTask03Handle = osThreadNew(ProducerTask03_Entry, NULL, &ProducerTask03_attributes);

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


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	/*checking command scheme
	 *
	 * 0...task1 speed
	 * 1...task2 speed
	 * 2...task3 speed
	 *
	 *
	 *
	 *
	 *
	 **/


	for(int i=0; i<3;i++)
	{
		if(uart_rx_buffer[i] > 48 && uart_rx_buffer[i] < 58)
			printf("Task %d Interval = %c00ms\n",i, uart_rx_buffer[i]);
		else
			printf("command invalid\n");

	}

commando.ivTask1=(uart_rx_buffer[0]-48)*100;
commando.ivTask2=(uart_rx_buffer[1]-48)*100;
commando.ivTask3=(uart_rx_buffer[2]-48)*100;

	//HAL_UART_Transmit(&huart2, uart_rx_buffer, BUFFER_SIZE,300);
	HAL_UART_Receive_IT(&huart2, uart_rx_buffer, BUFFER_SIZE);

}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_Monitoring_Task_Entry */
/**
  * @brief  Function implementing the Monitoring_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Monitoring_Task_Entry */
void Monitoring_Task_Entry(void *argument)
{
  /* USER CODE BEGIN 5 */

	//init message queues
	MSGQUEUE_OBJ_t recMsg;


	//init uart2




	commando.ivTask1= 1000;
	commando.ivTask2= 1000;
	commando.ivTask3= 1000;

	/* Infinite loop */
	for(;;)
	{

		//reading UART and sending commandos to tasks





		if(osMessageQueuePut(CommandMessageQueue, &commando, NULL, 50)!= osOK)
			printf("failed!\n");



		//reading and updating the task counters
		osMessageQueueGet(ProducerMessageQueue, &recMsg, NULL, osWaitForever);

		printf("Task %d counter %d\n",recMsg.taskID,recMsg.counter);


		//printf("message count Command queue: %d of max %d\n", osMessageQueueGetCount(CommandMessageQueue),(int)MSGQUEUE_OBJECTS);
		//printf("message count Message queue: %d of max %d\n", osMessageQueueGetCount(ProducerMessageQueue),(int)MSGQUEUE_OBJECTS);


		osDelay(200);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_ProducerTask01_Entry */
/**
* @brief Function implementing the ProducerTask01 thread.
* this task puts an object of struct MSGQUEUE_OBJ_t and puts it into the message queue.
* it also puts the task id in it for information.
*
* The counter- interval can be changed by the monitor task via a message command.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ProducerTask01_Entry */
void ProducerTask01_Entry(void *argument)
{
  /* USER CODE BEGIN ProducerTask01_Entry */

	//initializing the message task object
	MSGQUEUE_OBJ_t msgtask1;
	msgtask1.counter=0;
	msgtask1.taskID=1;

	CMDQUEUE_OBJ_t commando;

	uint32_t taskdelay=500;

  /* Infinite loop */
  for(;;)
  {
	osMessageQueuePut(ProducerMessageQueue, &msgtask1,NULL, osWaitForever);
	//printf("Task01 interval %d \n",taskdelay);

	//Updating counter
	msgtask1.counter++;

	//receiving Command messages and eventually updating the task interval
	if(osMessageQueueGet(CommandMessageQueue, &commando, NULL, osWaitForever)== osOK)
		{
			//set in case of interval change
			if(taskdelay!= commando.ivTask1)
				taskdelay=commando.ivTask1;
		}

    osDelay(taskdelay);

  }
  /* USER CODE END ProducerTask01_Entry */
}

/* USER CODE BEGIN Header_ProducerTask02_Entry */
/**
* @brief Function implementing the ProducerTask02 thread. see task1
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ProducerTask02_Entry */
void ProducerTask02_Entry(void *argument)
{
  /* USER CODE BEGIN ProducerTask02_Entry */

	//initializing the message task object
	MSGQUEUE_OBJ_t msgtask2;
	msgtask2.counter=0;
	msgtask2.taskID=2;

	CMDQUEUE_OBJ_t commando;

	uint32_t taskdelay=500;

  /* Infinite loop */
  for(;;)
  {
	osMessageQueuePut(ProducerMessageQueue, &msgtask2,NULL, osWaitForever);
	//printf("Task02 interval %d \n",taskdelay);

	//Updating counter
	msgtask2.counter++;

	//receiving Command messages and eventually updating the task interval
	if(osMessageQueueGet(CommandMessageQueue, &commando, NULL, osWaitForever)== osOK)
		{
			//set in case of interval change
			if(taskdelay!= commando.ivTask2)
				taskdelay=commando.ivTask2;
		}

    osDelay(taskdelay);

  }
  /* USER CODE END ProducerTask02_Entry */
}

/* USER CODE BEGIN Header_ProducerTask03_Entry */
/**
* @brief Function implementing the ProducerTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ProducerTask03_Entry */
void ProducerTask03_Entry(void *argument)
{
  /* USER CODE BEGIN ProducerTask03_Entry */

	//initializing the message task object
	MSGQUEUE_OBJ_t msgtask3;
	msgtask3.counter=0;
	msgtask3.taskID=3;

	CMDQUEUE_OBJ_t commando;

	uint32_t taskdelay=500;

  /* Infinite loop */
  for(;;)
  {
	osMessageQueuePut(ProducerMessageQueue, &msgtask3,NULL, osWaitForever);
	//printf("Task03 interval %d \n",taskdelay);

	//Updating counter
	msgtask3.counter++;

	//receiving Command messages and eventually updating the task interval
	if(osMessageQueueGet(CommandMessageQueue, &commando, NULL, osWaitForever)== osOK)
		{
			//set in case of interval change
			if(taskdelay!= commando.ivTask3)
				taskdelay=commando.ivTask3;
		}

    osDelay(taskdelay);

  }
  /* USER CODE END ProducerTask03_Entry */
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
