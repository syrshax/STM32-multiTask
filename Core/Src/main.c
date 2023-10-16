/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "ssd1306.h"

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
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for getDatos */
osThreadId_t getDatosHandle;
const osThreadAttr_t getDatos_attributes = {
  .name = "getDatos",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for sendDatos */
osThreadId_t sendDatosHandle;
const osThreadAttr_t sendDatos_attributes = {
  .name = "sendDatos",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for motor */
osThreadId_t motorHandle;
const osThreadAttr_t motor_attributes = {
  .name = "motor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Temperatura */
osThreadId_t TemperaturaHandle;
const osThreadAttr_t Temperatura_attributes = {
  .name = "Temperatura",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Potenciometro */
osThreadId_t PotenciometroHandle;
const osThreadAttr_t Potenciometro_attributes = {
  .name = "Potenciometro",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Servomotor */
osThreadId_t ServomotorHandle;
const osThreadAttr_t Servomotor_attributes = {
  .name = "Servomotor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Pantalla */
osThreadId_t PantallaHandle;
const osThreadAttr_t Pantalla_attributes = {
  .name = "Pantalla",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for colaMenu */
osMessageQueueId_t colaMenuHandle;
const osMessageQueueAttr_t colaMenu_attributes = {
  .name = "colaMenu"
};
/* Definitions for semaforo_uart */
osSemaphoreId_t semaforo_uartHandle;
const osSemaphoreAttr_t semaforo_uart_attributes = {
  .name = "semaforo_uart"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);
void startgetDatos(void *argument);
void startsendDatos(void *argument);
void startmotor(void *argument);
void StartTemperatura(void *argument);
void StartPotenciometro(void *argument);
void StartServo(void *argument);
void StartPantalla(void *argument);

/* USER CODE BEGIN PFP */
//char numfase = 1;
int periodo  = 0;
int temp_entera,temp_decimal,mostrar;
short int bit_stop;
char direccion;
char datosuart[4];
char messg[22];
int TiempoGrados,GradosFinal;
int ControlServo=2;
int MuestreoTemp = 100;
int VelocidadServo,TiempoServo,ServoStop;
char GradosPotenciometro;

float temperature;
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
  MX_USART2_UART_Init();
  MX_ADC_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of semaforo_uart */
  semaforo_uartHandle = osSemaphoreNew(1, 1, &semaforo_uart_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of colaMenu */
  colaMenuHandle = osMessageQueueNew (4, sizeof(uint8_t), &colaMenu_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of getDatos */
  getDatosHandle = osThreadNew(startgetDatos, NULL, &getDatos_attributes);

  /* creation of sendDatos */
  sendDatosHandle = osThreadNew(startsendDatos, NULL, &sendDatos_attributes);

  /* creation of motor */
  motorHandle = osThreadNew(startmotor, NULL, &motor_attributes);

  /* creation of Temperatura */
  TemperaturaHandle = osThreadNew(StartTemperatura, NULL, &Temperatura_attributes);

  /* creation of Potenciometro */
  PotenciometroHandle = osThreadNew(StartPotenciometro, NULL, &Potenciometro_attributes);

  /* creation of Servomotor */
  ServomotorHandle = osThreadNew(StartServo, NULL, &Servomotor_attributes);

  /* creation of Pantalla */
  PantallaHandle = osThreadNew(StartPantalla, NULL, &Pantalla_attributes);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart2.Init.BaudRate = 38400;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, in4_Pin|in1_Pin|in3_Pin|in2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : in4_Pin in1_Pin in3_Pin in2_Pin */
  GPIO_InitStruct.Pin = in4_Pin|in1_Pin|in3_Pin|in2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart){
	for (int i = 0; i<4; i++){
	osMessageQueuePut(colaMenuHandle, &datosuart[i], 1, 0);
	}
	HAL_UART_Receive_IT(&huart2, &datosuart, 4);

}

void HAL_UART_TxCpltCallback (UART_HandleTypeDef *huart){
	osSemaphoreRelease(semaforo_uartHandle);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	char status[4];
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_WritePin(GPIOA, LED2_Pin, 1);
	status[0] = 0x69;
	status[1] = 0xFF;
	status[2] = 0xFF;
	status[3] = 0xFF;
	osSemaphoreAcquire(semaforo_uartHandle, osWaitForever);
	HAL_UART_Transmit_IT(&huart2,&status,4);
	osDelay(700);

	HAL_GPIO_WritePin(GPIOA, LED2_Pin, 0);
	status[0] = 0x70;
	status[1] = 0xFF;
	status[2] = 0xFF;
	status[3] = 0xFF;
	osDelay(700);
	osSemaphoreAcquire(semaforo_uartHandle, osWaitForever);
	HAL_UART_Transmit_IT(&huart2,&status,4);


  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startgetDatos */
/**
* @brief Function implementing the getDatos thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startgetDatos */
void startgetDatos(void *argument)
{
  /* USER CODE BEGIN startgetDatos */
	char menu[4];
	char messg[10];
	HAL_UART_Receive_IT(&huart2, &datosuart, 4);
  /* Infinite loop */
  for(;;)
  {
	  for (int i = 0; i<4; i++){
	  osMessageQueueGet(colaMenuHandle, &menu[i], 1, osWaitForever);
	  }
	 /* sprintf(messg,"%d",osMessageQueueGetCount(colaMenuHandle));
	  osSemaphoreAcquire(semaforo_uartHandle, osWaitForever);
	  HAL_UART_Transmit_IT(&huart2, messg, sizeof(messg));
	  */



	  switch (menu[0]){

	  	  case 0x4D: // MOTOR EN VELOCIDAD INDICADA
	  		osThreadResume(motorHandle);
	  		if (menu[1] == 0)
	  		{
	  		 direccion = 'F';
	  		 periodo = 10;

	  		}
	  		periodo = menu[1];


	  		  	 break;
	  	  case 0x44:
	  		osThreadResume(motorHandle);
	  		   direccion ='D';
	  		   break;
	  	  case 0x49:
	  		osThreadResume(motorHandle);
	  			  direccion = 'I';
	  			  break;


	  	  case 0x54: //lectura de la temperatura
	  		  osThreadResume(TemperaturaHandle);
	  		  osThreadResume(PotenciometroHandle);
	  		  MuestreoTemp = menu[1]*10;
	  		  mostrar=1;
	  		  break;

	  	  case 0x31: //parar lectura de temp;
	  		  osThreadSuspend(TemperaturaHandle);
	  		  mostrar=0;
	  		  break;

	  	  case 0x32: //lectura de grados;

	  		  mostrar = 2;
	  		  break;


	  	  case 0x40: //servo con potenciometro

	  		  		  ControlServo = 1;
	  		  		  osThreadResume(TemperaturaHandle);
	  		  		  osThreadResume(ServomotorHandle);

	  		  	  break;

	  	  case 0x53:	// servo indicando grados

	  		  		  		  		  ControlServo = 2;
	  		  		  		  		  GradosFinal = menu[1];
	  		  		  		  		  osThreadResume(TemperaturaHandle);
	  		  		  		  		  osThreadResume(ServomotorHandle);


	  		  	  break;

	  	  case 0x41: //servo controlado por tiempo y velocidad cierre
	  		  	  	  ControlServo = 3;
	  		  	  	  TiempoServo = menu[1]*1000;
	  		  	  	  VelocidadServo = menu[2];
	  		  	  	  osThreadResume(ServomotorHandle);
	  		  	  	  osThreadResume(TemperaturaHandle);
	  		  	  	  break;


	  	  case 0x58:
	  		  osThreadResume(PantallaHandle);
	  		  break;
	  	  case 0x59:
	  		ssd1306_Fill(Black);
	  		ssd1306_UpdateScreen();
	  		osThreadSuspend(PantallaHandle);
	  		  break;


	  	  case 0x43: //CANCELAR TODAS LAS OPERACIONES
	  		osThreadSuspend(ServomotorHandle);
	  		osThreadSuspend(TemperaturaHandle);
	  		osThreadSuspend(PotenciometroHandle);
	  		osThreadSuspend(PantallaHandle);
	  		osThreadSuspend(motorHandle);
	  		ssd1306_Fill(Black);
	  		ssd1306_UpdateScreen();
	  		temp_entera=0;
	  		temp_decimal=0;
	  		periodo=0;
	  		GradosPotenciometro=0;

	  		break;


	  }
    osDelay(10);
  }
  /* USER CODE END startgetDatos */
}

/* USER CODE BEGIN Header_startsendDatos */
/**
* @brief Function implementing the sendDatos thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startsendDatos */
void startsendDatos(void *argument)
{
  /* USER CODE BEGIN startsendDatos */
	char bufferdata[4];
  /* Infinite loop */
  for(;;)
  {


//	  	 bufferdata[0]=0x30;
//	  	 bufferdata[1]=temp_entera;     			// REALIZACION DEL ENVIO DE LOS DATOS GRADOS A LA APP DE VB.
//	  	 bufferdata[2]=temp_decimal;
//	  	 bufferdata[3]=0xE0;
//
//	 	 osSemaphoreAcquire(semaforo_uartHandle, osWaitForever);
//	 	 HAL_UART_Transmit_IT(&huart2,&bufferdata,4);
//	 	 osDelay(20);


	 	 //bufferdata[0]=0x20;

	  	 bufferdata[0]=0x40;
	  	 bufferdata[1]=(11-periodo);     			// REALIZACION DEL ENVIO DE LOS DATOS de VELOCIDAD MOTOR;
	  	 bufferdata[2]=0xFF;
	  	 bufferdata[3]=0xE0;
	  	 osSemaphoreAcquire(semaforo_uartHandle, osWaitForever);
	  	 HAL_UART_Transmit_IT(&huart2,&bufferdata,4);

	  	osDelay(10);


	  	 bufferdata[0]=0x20;
	  	 bufferdata[1]=GradosPotenciometro;     			// REALIZACION DEL ENVIO DE LOS DATOS de VELOCIDAD MOTOR;
	  	 bufferdata[2]=0xFF;
	  	 bufferdata[3]=0xE0;
	  	 osSemaphoreAcquire(semaforo_uartHandle, osWaitForever);
	  	 HAL_UART_Transmit_IT(&huart2,&bufferdata,4);

	  	 osDelay(10);




  }
  /* USER CODE END startsendDatos */
}

/* USER CODE BEGIN Header_startmotor */
/**
* @brief Function implementing the motor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startmotor */
void startmotor(void *argument)
{
  /* USER CODE BEGIN startmotor */
	char numfase = 1;
	char messg[30];
  /* Infinite loop */
	for(;;)
		 {
		switch(numfase) {
		 case 1:
		 HAL_GPIO_WritePin(GPIOB,in1_Pin,1); //1er medio paso
		 HAL_GPIO_WritePin(GPIOB,in2_Pin,0);
		 HAL_GPIO_WritePin(GPIOB,in3_Pin,0);
		 HAL_GPIO_WritePin(GPIOB,in4_Pin,0);
		 break;
		 case 2:
		 HAL_GPIO_WritePin(GPIOB,in1_Pin,1); //1 completo
		 HAL_GPIO_WritePin(GPIOB,in2_Pin,1);
		 HAL_GPIO_WritePin(GPIOB,in3_Pin,0);
		 HAL_GPIO_WritePin(GPIOB,in4_Pin,0);
		 break;
		 case 3:
		 HAL_GPIO_WritePin(GPIOB,in1_Pin,0); //2 medio
		 HAL_GPIO_WritePin(GPIOB,in2_Pin,1);
		 HAL_GPIO_WritePin(GPIOB,in3_Pin,0);
		 HAL_GPIO_WritePin(GPIOB,in4_Pin,0);
		 break;
		 case 4:
		 HAL_GPIO_WritePin(GPIOB,in1_Pin,0); //2 completo
		 HAL_GPIO_WritePin(GPIOB,in2_Pin,1);
		 HAL_GPIO_WritePin(GPIOB,in3_Pin,1);
		 HAL_GPIO_WritePin(GPIOB,in4_Pin,0);
		 break;
		 case 5:
		 HAL_GPIO_WritePin(GPIOB,in1_Pin,0);// 3 medio
		 HAL_GPIO_WritePin(GPIOB,in2_Pin,0);
		 HAL_GPIO_WritePin(GPIOB,in3_Pin,1);
		 HAL_GPIO_WritePin(GPIOB,in4_Pin,0);
		 break;
		 case 6:
		 HAL_GPIO_WritePin(GPIOB,in1_Pin,0);// 3completo
		 HAL_GPIO_WritePin(GPIOB,in2_Pin,0);
		 HAL_GPIO_WritePin(GPIOB,in3_Pin,1);
		 HAL_GPIO_WritePin(GPIOB,in4_Pin,1);
		 break;
		 case 7:
		 HAL_GPIO_WritePin(GPIOB,in1_Pin,0); //4medio
		 HAL_GPIO_WritePin(GPIOB,in2_Pin,0);
		 HAL_GPIO_WritePin(GPIOB,in3_Pin,0);
		 HAL_GPIO_WritePin(GPIOB,in4_Pin,1);
		 break;
		 case 8:
		 HAL_GPIO_WritePin(GPIOB,in1_Pin,1);//4completo
		 HAL_GPIO_WritePin(GPIOB,in2_Pin,0);
		 HAL_GPIO_WritePin(GPIOB,in3_Pin,0);
		 HAL_GPIO_WritePin(GPIOB,in4_Pin,1);
		 break;

		 }
		 if(direccion=='D')
		 {
			 bit_stop=1;
		numfase++;
		 if(numfase>9)
		 numfase=1;
		 }
		 if(direccion=='I')
		 {
			 bit_stop=1;
		 numfase--;
		 if(numfase<=0)
		 numfase=8;
		 }

		 if(direccion=='F'){ //Pulsamos F y paramos... nos quedamos en la fase previa
		  // por lo que asi conseguimos una parada efectiva, y cuando reanudemos, reanudamos por
		  //la fase en la que estamos.
			 numfase=1;
		 }

		/*if(direccion=='F'){

			 bit_stop=0;

		 }*/

		 osDelay(11-periodo);

		 }
  /* USER CODE END startmotor */
}

/* USER CODE BEGIN Header_StartTemperatura */
/**
* @brief Function implementing the Temperatura thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTemperatura */
void StartTemperatura(void *argument)
{
  /* USER CODE BEGIN StartTemperatura */
	uint16_t valortmp;
	float adc_valor_temp;
	int entera, decimal;
	HAL_ADCEx_Calibration_Start(&hadc);
	HAL_StatusTypeDef status;
	char messg[22];
	uint16_t adc_value = 0;
	//osThreadSuspend(TemperaturaHandle);
  /* Infinite loop */
  for(;;)
  {
	  HAL_ADC_Start(&hadc);
	  status = HAL_ADC_PollForConversion(&hadc, 30);

	  if (status == HAL_OK){
		  valortmp = HAL_ADC_GetValue(&hadc);
	  }

	  adc_valor_temp = (valortmp/4096.0)*3300;
	  temperature = adc_valor_temp/100;

	  entera = temperature;
	  decimal = (temperature - entera) *100;

	  temp_entera = entera;
	  temp_decimal = decimal;


	  status = HAL_ADC_PollForConversion(&hadc,30);

	  if (status==HAL_OK){

	  	  		  adc_value = HAL_ADC_GetValue(&hadc);
	  	  }



	  	  TiempoGrados = ((adc_value/4096.0)*5)*200+1000;

	  	  HAL_ADC_Stop(&hadc);


	  //sprintf(messg,"La temp es:  %d . %d\n\r",entera, decimal);

	 // osSemaphoreAcquire(semaforo_uartHandle, osWaitForever);
	 // HAL_UART_Transmit_IT(&huart2, messg, sizeof(messg));

	  osDelay(10);
  }
  /* USER CODE END StartTemperatura */
}

/* USER CODE BEGIN Header_StartPotenciometro */
/**
* @brief Function implementing the Potenciometro thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPotenciometro */
void StartPotenciometro(void *argument)
{
  /* USER CODE BEGIN StartPotenciometro */
//		uint16_t adc_value = 0;
//		HAL_ADCEx_Calibration_Start(&hadc);
//		HAL_StatusTypeDef status;
		//osThreadSuspend(PotenciometroHandle);

	char bufferdata[4];

  /* Infinite loop */
  for(;;)
  {
/*
	  if (mostrar==1 || mostrar == 3){
	  sprintf(messg,"La temp es:  %d . %d\n\r",temp_entera, temp_decimal);
	  osSemaphoreAcquire(semaforo_uartHandle, osWaitForever);
	  HAL_UART_Transmit_IT(&huart2, messg, sizeof(messg));
	  osDelay(MuestreoTemp);
	  }

	  if (mostrar==1 || mostrar == 3){
		  sprintf(messg,"Los grados son %d",GradosPotenciometro);
		  osSemaphoreAcquire(semaforo_uartHandle, osWaitForever);
		  HAL_UART_Transmit_IT(&huart2, messg, sizeof(messg));
		  osDelay(200);
	  }

*/

	  	 bufferdata[0]=0x30;
	  	 bufferdata[1]=temp_entera;     			// REALIZACION DEL ENVIO DE LOS DATOS GRADOS A LA APP DE VB.
	  	 bufferdata[2]=temp_decimal;
	  	 bufferdata[3]=0xE0;

	 	 osSemaphoreAcquire(semaforo_uartHandle, osWaitForever);
	 	 HAL_UART_Transmit_IT(&huart2,&bufferdata,4);
	 	 osDelay(MuestreoTemp);

  }
  /* USER CODE END StartPotenciometro */
}

/* USER CODE BEGIN Header_StartServo */
/**
* @brief Function implementing the Servomotor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartServo */
void StartServo(void *argument)
{
  /* USER CODE BEGIN StartServo */
	int conversion1,conversion2,gradoslocal;
	float middlenumber;
	char messag[17];
	//osThreadSuspend(ServomotorHandle);
  /* Infinite loop */
  for(;;)
  {
	  if (ControlServo==1){
	  	  htim3.Instance -> CCR2 = TiempoGrados;
	  	  GradosPotenciometro = ((90*TiempoGrados-90000)/1000);
	  	  osDelay(10);
	  	  }
	  if (ControlServo==2){

		  gradoslocal = (1000+11*GradosFinal);
		  GradosPotenciometro = GradosFinal;
		  htim3.Instance -> CCR2 = gradoslocal;
	  	  osDelay(10);
	  	  	  }

	  if (ControlServo ==3){
		  gradoslocal = 1000; // 0 grados

		  while (gradoslocal <=2000){
			  GradosPotenciometro = ((90*gradoslocal-90000)/1000);
			  gradoslocal = gradoslocal+11;
			  htim3.Instance -> CCR2 = gradoslocal;
			  osDelay(1000/VelocidadServo);
		  }

		  osDelay(TiempoServo);
		 while (gradoslocal >= 1000){
			 	 	  GradosPotenciometro = ((90*gradoslocal-90000)/1000);
		 			  gradoslocal = gradoslocal-11;
		 			  htim3.Instance -> CCR2 = gradoslocal;
		 			  osDelay(1000/VelocidadServo);

		  }

		  ControlServo = 0;
	  }


  }
  /* USER CODE END StartServo */
}

/* USER CODE BEGIN Header_StartPantalla */
/**
* @brief Function implementing the Pantalla thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPantalla */
void StartPantalla(void *argument)
{
  /* USER CODE BEGIN StartPantalla */
	 ssd1306_Init();
	osThreadSuspend(PantallaHandle);
	int temp1,temp2;
	char messgTemp[20];
	char messgVel[20];
	char messgMotor[20];
	uint8_t cursor;
  /* Infinite loop */
  for(;;)
  {
	cursor = 0;
	ssd1306_SetCursor(0,cursor);
	temp1 = temp_entera;
	temp2 = temp_decimal;
	sprintf(messgTemp,"TEMPERATURA %d . %d ",temp1,temp2);
	ssd1306_WriteString (messgTemp, Font_6x8,White);
	cursor =+8;
	ssd1306_SetCursor(0,cursor);
	sprintf(messgVel,"Grados Servo %d ", GradosPotenciometro);
	ssd1306_WriteString (messgVel, Font_6x8,White);
	cursor =+16;
	ssd1306_SetCursor(0,cursor);
	sprintf(messgMotor,"VELOCIDAD MOTROR %d",periodo);
	ssd1306_WriteString (messgMotor, Font_6x8,White);
	ssd1306_UpdateScreen();


    osDelay(700);
  }
  /* USER CODE END StartPantalla */
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
