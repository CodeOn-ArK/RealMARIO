/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdarg.h>
#include "ANSI_Escape_Sequences.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TRUE 1
#define FALSE 0

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
uint8_t receive_buf;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
void SystemClock_Config(void);

void RTOS_delay(uint32_t delay_count);
HAL_StatusTypeDef printmsg(const char *format,...);
uint8_t get_cmnd_code(uint8_t val);
void get_args(uint8_t *buffer);
void led_status(uint8_t var);
void led_toggle(uint8_t var, uint32_t duration);
void get_led_status(char *);
void get_time(void);
void exit_app(void);
void led_toggle_start(TimerHandle_t xTimer );

HAL_StatusTypeDef printmsg(const char *format,...)
{
       char str[1024];

        /*extract the arg list using va apis */
       va_list args;
       va_start(args, format);
       vsprintf(str, format, args);

      // strcat(str,"\n\r");
       va_end(args);
      return HAL_UART_Transmit_DMA(&huart2, (uint8_t *)str, strlen(str));
      // HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str),HAL_MAX_DELAY);
}
/* USER CODE BEGIN PFP */
//Task Prototypes
static void Tmenu_print(void *parameter);
static void Tcmnd_handling(void *parameter);
static void Tcmnd_processing(void *parameter);
static void Tuart_write(void *parameter);
static void lol_task(void *arg);

//Software timer handle
xTimerHandle LED_Timer = NULL;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 *///0x200132ac   0x1fff

/**
  * @brief  The application entry point.
  * @retval int
  */
//Tasks handles
TaskHandle_t Hmenu_print = NULL,
		 	 Hcmnd_processing = NULL,
			 Hcmnd_handling = NULL,
			 Huart_write = NULL;

//Queue handle
QueueHandle_t cmnd_queue = NULL,
			  uart_write_queue = NULL;

typedef struct APP_CMND{
	uint8_t cmnd_num;
	uint8_t cmnd_args[10];
}app_cmnd_t;

uint8_t cmnd_buffer[20];
uint8_t cmnd_len = 0;
extern object_t main_char_body;

#define LED_ON				0
#define LED_OFF				1
#define LED_TOGGLE_ON		2
#define LED_TOGGLE_OFF		3
#define LED_READ_STATUS		4
#define RTC_PRINT_TIME		5
#define EXIT_APP			6

#define DELAY_TICKS pdMS_TO_TICKS(500)


uint8_t menu[] =
{
	"\033[?25l"
		/*
		ANSI_MOVE_CURSOR_TO_PO"Hello\n"
		"Hello - 2 \n"
		ANSI_MOVE_CURSOR_TO_POS(10,10)"Hello\n"
		"Hello -4"
*/
  		ANSI_CLR_SCR
		 ANSI_MOVE_CURSOR_TO_POS_LINE(20)ANSI_ESC"1m"ANSI_ESC "97m    ▄▄▄"ANSI_ESC"0m"
		 ANSI_MOVE_CURSOR_TO_POS_LINE(21)ANSI_ESC"11m"ANSI_ESC"97m ▄█████▄▄ "ANSI_ESC"0m"
		 ANSI_MOVE_CURSOR_TO_POS_LINE(22)ANSI_ESC"11m"ANSI_ESC"97m███"ANSI_ESC"46m▀▀▀▀"ANSI_ESC"40m▀"ANSI_ESC"46m▀"ANSI_ESC"40m▀"ANSI_ESC"46m▀"ANSI_ESC"0m"
		 ANSI_MOVE_CURSOR_TO_POS_LINE(23)ANSI_ESC"11m"ANSI_ESC"97m███"ANSI_ESC"46m▄   "ANSI_ESC"22m"ANSI_ESC"30m▀ ▀"ANSI_ESC"0m"ANSI_ESC"36m▀"ANSI_ESC"0m"
		 ANSI_MOVE_CURSOR_TO_POS_LINE(24)ANSI_ESC"11m"ANSI_ESC"97m ▄"ANSI_ESC"46m  "ANSI_ESC"0m"ANSI_ESC"1m"ANSI_ESC"97m█████▄ "ANSI_ESC"22m"ANSI_ESC"31m█▄"ANSI_ESC"0m"
		 ANSI_MOVE_CURSOR_TO_POS_LINE(25)ANSI_ESC"122m"ANSI_ESC"31m▀▀"ANSI_ESC"0m"ANSI_ESC"1m"ANSI_ESC"41m"ANSI_ESC"97m▄"ANSI_ESC"46m▄   "ANSI_ESC"41m▄▄▄"ANSI_ESC"0m"ANSI_ESC"22m"ANSI_ESC"31m▀██▀"ANSI_ESC"0m"
};

uint8_t leg_1[] = {
		ANSI_MOVE_CURSOR_TO_POS_LINE(26) ANSI_ESC"11m"ANSI_ESC"97m ██▀▀▀██▀  "ANSI_ESC"22m"ANSI_ESC"31m▀"ANSI_ESC"0m"
		ANSI_MOVE_CURSOR_TO_POS_LINE(27) ANSI_ESC"11m"ANSI_ESC"97m ▀▀▀▀ ▀▀▀▀"ANSI_ESC"0m"
};

uint8_t leg_2[] = {
		 ANSI_MOVE_CURSOR_TO_POS_LINE(26)ANSI_ESC"11m"ANSI_ESC"97m ██▀▀▀██▀  "ANSI_ESC"22m"ANSI_ESC"31m▀"ANSI_ESC"0m"
		 ANSI_MOVE_CURSOR_TO_POS_LINE(27)ANSI_ESC"11m"ANSI_ESC"97m  ▀▀▀▀ ▀▀▀▀"ANSI_ESC"0m"

};
uint8_t var[] = "%s";
#define DATA	ANSI_CLR_SCR ANSI_MOVE_CURSOR_TO_PO ANSI_256_COLOR_F " _______________________________________ "ANSI_TERMINATE

int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */


  /* MCU Configuration--------------------------------------------------------*/

  DWT->CTRL |= (0x1 << 0);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  __unused BaseType_t status;

	//printf("The application has started\n\r");

  //Start Recording
  SEGGER_SYSVIEW_Conf();
  SEGGER_SYSVIEW_Start();

  //lets create command queue
  cmnd_queue = xQueueCreate(10, sizeof(int *));
  uart_write_queue = xQueueCreate(10, sizeof(char *));

  //object_init(NULL);
 // HAL_UART_Receive_IT(&huart2, cmnd_buffer, 2);
  HAL_UART_Receive_IT(&huart2, &receive_buf, 1);
 HAL_UART_Transmit_DMA(&huart2, (uint8_t *)"hello", 5);
  if((cmnd_queue != NULL) && (uart_write_queue != NULL)){
//	  status = xTaskCreate(lol_task, "lol", 1024, NULL, 1, NULL);
/*
	status = xTaskCreate(Tmenu_print, "Menu Print", 1000, NULL, 1, &Hmenu_print);
	configASSERT(status == pdPASS);
	status = xTaskCreate(Tcmnd_handling, "Cmnd Handler", 1000, NULL, 2, &Hcmnd_handling);
	configASSERT(status == pdPASS);
	status = xTaskCreate(Tcmnd_processing, "Cmnd Processing", 1000, NULL, 2, &Hcmnd_processing);
	configASSERT(status == pdPASS);
	status = xTaskCreate(Tuart_write,"UART Write" ,	1000, NULL, 2, &Huart_write);
	configASSERT(status == pdPASS);
*/

	//start FreeRTOS scheduler
	vTaskStartScheduler();

  }else{
	 printmsg("ERR : Queue creation unsuccessful!");

  }

	//if the control comes here, the cpu comes here due to unsufficient task in the memory

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
/* USER CODE BEGIN 4 */
static void lol_task(void *arg){
	static uint16_t i=0;
	while(1){
		if(i == 256) i=0;
		//printmsg(menu,i);// i, i, i, i, i, i, i , i);
		printmsg((const char *)menu, i,i,i,i,i,i);
		vTaskDelay(pdMS_TO_TICKS(10));
		printmsg((const char *)leg_1,i,i);
		//(i%2 == 0)? printmsg((const char *)leg_1,i,i):printmsg((const char *)leg_2,i,i);
		i++;
		vTaskDelay(pdMS_TO_TICKS(100));
		//printmsg(menu,i,i);
		//vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
static void Tmenu_print(void *parameter){
	/*
	 * This task is responsible for displaying menu
	 * It sends the address of data pointer to menu to uart_write_queue
	 * After writing data into the queue it waits for a notification from other task, until which it blocks indefinitely
	 */

	uint8_t *pData = menu;										//Create a pointer to menu array
	while(1){

		xQueueSend(uart_write_queue, &pData, portMAX_DELAY);//Send the pointer to a QUEUE uart_write_Q and wait indefinitely

		xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);			//Block the task indefinitely until Notification comes
	}
}
static void Tcmnd_handling(void *parameter){

	app_cmnd_t *new_cmnd;

	while(1){
		//Block the task upon entrance
		xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);

		//send command to queue
		new_cmnd = (app_cmnd_t *)pvPortMalloc(sizeof(app_cmnd_t));

		taskENTER_CRITICAL();
		new_cmnd->cmnd_num = get_cmnd_code(cmnd_buffer[0]);
		//get_args(new_cmnd->cmnd_args);
		taskEXIT_CRITICAL();

		//send the command to command queue
		xQueueSend(cmnd_queue, &new_cmnd, portMAX_DELAY);
	}
}
static void Tcmnd_processing(void *parameter){

	char pdata[50];
	app_cmnd_t *app_cmnd;

	while(1){
		xQueueReceive(cmnd_queue, (void *)&app_cmnd, portMAX_DELAY);

		if(app_cmnd->cmnd_num == LED_ON){
			led_status(GPIO_PIN_SET);

		}else if(app_cmnd->cmnd_num == LED_OFF){
			led_status(GPIO_PIN_RESET);

		}else if(app_cmnd->cmnd_num == LED_TOGGLE_ON){
			led_toggle(GPIO_PIN_SET, DELAY_TICKS);

		}else if(app_cmnd->cmnd_num == LED_TOGGLE_OFF){
			led_toggle(GPIO_PIN_RESET, DELAY_TICKS);

		}else if(app_cmnd->cmnd_num == LED_READ_STATUS){
			get_led_status(pdata);

		}else if(app_cmnd->cmnd_num == RTC_PRINT_TIME){
			get_time();

		}else if(app_cmnd->cmnd_num == EXIT_APP){
			exit_app();

		}else{
			printmsg("Err : Command Not Found");
		}

		//free the allocated memory
		vPortFree(app_cmnd);
	}
}

static void Tuart_write(void *parameter){

	/*
	 * This task reads the uart_write_queue and prints the data via uart
	 */

	char *pData = NULL;
	while(1){
		xQueueReceive(uart_write_queue, &pData, portMAX_DELAY);
		printmsg(pData);
/*		printmsg(var, "ello");*/
	}
}

/* USER CODE END 4 */

void RTOS_delay(uint32_t delay_count)
{
	uint32_t data = 0;
	data = 1000/configTICK_RATE_HZ;
	delay_count /= data;

	volatile uint32_t present_tick = xTaskGetTickCount();
	while((xTaskGetTickCount() - present_tick) < delay_count){
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *Huart){
/*
	BaseType_t pxHigherPriorityTaskWoken;

	HAL_UART_Receive_IT(&huart2, cmnd_buffer, 2);
	//Cmnd received over UART, notify the cmnd handler
	xTaskNotifyFromISR(Hcmnd_handling, 0, eNoAction, &pxHigherPriorityTaskWoken);
	//Notify the Menu handler to print the menu once again
	xTaskNotifyFromISR(Hmenu_print, 0, eNoAction, &pxHigherPriorityTaskWoken);

	//if any higher priority task is woken up, yield the CPU manually
	if(pxHigherPriorityTaskWoken){
		taskYIELD();				//Give control to higher priority TASK
	}
*/
	static int i=0;
	  HAL_UART_Receive_IT(&huart2, &receive_buf, 1);
	if(receive_buf == 'l'){
		if(i == 256) i=0;
		i++;
		printmsg((const char *)menu, i,i,i,i,i,i,i,i);
	}else if(receive_buf == 'h'){
		if(i == 0) i=1;
		i--;
		printmsg((const char *)menu, i,i,i,i,i,i,i,i);
	}
	else if(receive_buf == 'c')
		//huartHAL_UART_Transmit(Huart, (uint8_t *)"\033[2J", 4, HAL_MAX_DELAY);
		printmsg(ANSI_CLR_SCR);
	memset(&receive_buf, 0 , sizeof(receive_buf));
}

uint8_t get_cmnd_code(uint8_t val){

	return (uint8_t)(val - 48);
}

void get_args(uint8_t *buffer){

}

void led_status(uint8_t var){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, var);
}

void led_toggle_start(TimerHandle_t xTimer ){
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

	static uint16_t i=0;
	if(i == 256) i=0;
	printmsg(menu,i, i+1, i+2, i+3, i+4, i+5, i+6, i +7 );
	i += 7;
}

void led_toggle(uint8_t var, uint32_t duration){
	if(var == GPIO_PIN_SET){
		if(LED_Timer == NULL){
			//create a software timer
			LED_Timer = xTimerCreate("Toggle-Timer", duration, pdTRUE, NULL, led_toggle_start);

			//start the timer
			xTimerStart(LED_Timer, portMAX_DELAY);

		}else{
			//start the timer
			xTimerStart(LED_Timer, portMAX_DELAY);

		}

	}else{
		//stop the timer
		xTimerStop(LED_Timer, portMAX_DELAY);

	}
}

void get_led_status(char *data){

	sprintf(data,"The LED status is : %s",HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)? "ON" : "OFF" );
	xQueueSend(uart_write_queue, &data, portMAX_DELAY);
}

void get_time(void){

}

void exit_app(void){
	vTaskDelete(Huart_write);
	vTaskDelete(Hmenu_print);
	vTaskDelete(Hcmnd_handling);
	vTaskDelete(Hcmnd_processing);

	HAL_NVIC_DisableIRQ(USART2_IRQn);
}

void vApplicationIdleHook( void ){
	__WFI();
}
 /******************************* SYS SETUP ************************************/
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  HAL_NVIC_SetPriority(USART2_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE END USART2_Init 2 */
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*
void  HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	  HAL_UART_Receive_IT(&huart2, &receive_buf, 1);
	if(receive_buf == 'h')
		HAL_UART_Transmit(huart, (uint8_t *)"hello\n", 6, HAL_MAX_DELAY);
	else if(receive_buf == 'c')
		HAL_UART_Transmit(huart, (uint8_t *)"\033[2J", 4, HAL_MAX_DELAY);
	memset(&receive_buf, 0 , sizeof(receive_buf));
	//if(receive_buf == '\r')
		//receive_buf = 'h';

}
*/

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	  HAL_UART_Receive_IT(&huart2, &receive_buf, 1);
	HAL_UART_Transmit(huart, (uint8_t *)"ERR\n", 4, HAL_MAX_DELAY);
	memset(&receive_buf, 0 , sizeof(receive_buf));
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
