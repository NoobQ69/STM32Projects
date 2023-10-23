/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hx711.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HX711_CLK_GPIO_Port 	GPIOB
#define HX711_CLK_Pin					GPIO_PIN_6
#define HX711_DATA_GPIO_Port 	GPIOB
#define HX711_DATA_Pin				GPIO_PIN_7

#define ENCODER_PULSE 200

#define MODE_BUTTON 		GPIO_PIN_0
#define SET_BUTTON 			GPIO_PIN_1
#define CHANGE_BUTTON 	GPIO_PIN_2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
hx711_t loadcell;
uint8_t weight;

volatile uint16_t EnableEOS = 0, setEnable = 0, UpToSecond = 0, CountingPulse = 0, PulseWidth = 10000, PulseLever = 5, RPM = 0;
uint8_t buttonBuffer[4] = {0, 0, 0, 0};
//------------------------------FOR UART'S VARIABLES-----------------------------
volatile uint8_t txBuffer[5] = {0,0,0,0,0};
volatile uint8_t rxBuffer[5] = {0,0,0,0,0};

volatile uint8_t Minutes = 0, Seconds = 0, SetMinutes = 2, SetSeconds = 5, ModeAck = 0;
volatile uint8_t Set = 1, CheckLoadCellTime = 0, button;
// ------------------------------------------------------------------------------

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ******************************** FOR EOS FUNCTIONS **************************
void EOS_Enable(uint32_t x) //  Bat EOS
{
	EnableEOS = 1;
	
	SysTick->LOAD = SystemCoreClock / 1000*x;
	SysTick->VAL = 0;
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

void EOS_Disable()					// Tat EOS va cau hinh lai SYSTICK TIMER
{
	EnableEOS = 0;
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
	HAL_InitTick(uwTickPrio);
}

// ******************************** FOR UART FUNCTIONS ********************************
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_DMA(&huart1, rxBuffer, 5);
}

// ******************************** FOR TIME FUNCTIONS ********************************
void runTime()
{
	Seconds++;
	CheckLoadCellTime++;
	if (Seconds > 59)
	{
		Seconds = 0;
		Minutes++;
		if (Minutes > 59)
		{
			Minutes = 0;
		}
	}
}

// ******************************** FOR ENCODER GPIO FUNCTIONS **************************

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) // Ham dem xung khi co ngat
{
	CountingPulse++;
}
// ************************** FOR BUTTON FUNCTIONS ******************************

uint8_t readModeButton()							// doc nut nhan mode
{
	return (GPIOA->IDR & GPIO_PIN_0) == 0;
}

uint8_t readSetButton()							// doc nut nhan set
{
	return (GPIOA->IDR & GPIO_PIN_1) == 0;
}

uint8_t readUpButton()						// doc nut nhan up, nut nay thay tang gia tri thoi gian phut, giay
{
	return (GPIOA->IDR & GPIO_PIN_2) == 0;
}

uint8_t readDownButton()						// doc nut nhan down, nut nay giam gia tri thoi gian phut, giay
{
	return (GPIOA->IDR & GPIO_PIN_3) == 0;
}

uint8_t readButtons()							// doc nut nhan va tra ve gia tri nut nhan
{
	uint8_t temp = 0, result = 0;
	if (readModeButton())
	{
		temp = 1;
	}
	else if (readSetButton())
	{
		temp = 2;
	}
	else if (readUpButton())
	{
		temp = 3;
	}
	else if (readDownButton())
	{
		temp = 4;
	}
	
	buttonBuffer[0] = buttonBuffer[1]; //Store data

	buttonBuffer[1] = buttonBuffer[2];

	buttonBuffer[2] = temp;
	
	if (buttonBuffer[0] != 0 && (buttonBuffer[0] == buttonBuffer[1])&&(buttonBuffer[2] == 0))
	{
		result = buttonBuffer[0];
	}

	return result;
}

void putValueToTXBuffer(unsigned char command, uint16_t value)	// truyen gia tri vao TX buffer
{
	txBuffer[0] = command;
	txBuffer[1] = (value % 10000 / 1000) + 0x30;
	txBuffer[2] = (value % 1000 / 100) + 0x30;
	txBuffer[3] = (value % 100 / 10) + 0x30;
	txBuffer[4] = (value % 10) + 0x30;
}

void setUpTime()						// Ham nay doc nut nhan va gui gia tri vua doc len esp32
{
	button = 0;
	button = readButtons();
	if (button == 1)
	{
		setEnable = 1;
		ModeAck = 0;
		putValueToTXBuffer('e', EnableEOS);
		while (HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen((uint8_t*)txBuffer), HAL_MAX_DELAY) != HAL_OK);
	}
	else if (button == 2)
	{
		Set = (Set+1) % 3;
		putValueToTXBuffer('s', Set);
		while (HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen((uint8_t*)txBuffer), HAL_MAX_DELAY) != HAL_OK);
	}
	else if (button == 3)
	{
		if (Set == 2)
		{
			if (PulseWidth < 20000)
			{
				PulseWidth += 2000;
				PulseLever++;
			}
			// Send to esp32 through UART communication
			putValueToTXBuffer('p', PulseLever);											// 230 = 2*10^3 *(-1)*0 = 2000
		}
		else if (Set == 1)
		{
			SetMinutes++;
			// Send to esp32 through UART communication
			putValueToTXBuffer('m', SetMinutes);
		}
		else if (Set == 0)
		{
			SetSeconds++;
			// Send to esp32 through UART communication
			putValueToTXBuffer('s', SetSeconds);
		}
		while (HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen((uint8_t*)txBuffer), HAL_MAX_DELAY) != HAL_OK);
	}
	else if (button == 4)
	{
		if (Set == 2)
		{
			if (PulseWidth != 0)
			{
				PulseWidth -= 2000;
				PulseLever--;
			}
			// Send to esp32 through UART communication
			putValueToTXBuffer('p', PulseLever);									// 231 = 2*10^3 *(-1)*1 = -2000
		}
		else if (Set == 1)
		{
			SetMinutes--;
			// Send to esp32 through UART communication
			putValueToTXBuffer('m', SetMinutes);
		}
		else if (Set == 0)
		{
			SetSeconds--;
			// Send to esp32 through UART communication
			putValueToTXBuffer('s', SetSeconds);
		}
		while (HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen((uint8_t*)txBuffer), HAL_MAX_DELAY) != HAL_OK);
	}
	HAL_Delay(10);
}

void checkFood()
{
	weight = hx711_weight(&loadcell, 10) - 821;
	if (weight < 20)
	{
		putValueToTXBuffer('o', 0);
		while (HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen((uint8_t*)txBuffer), HAL_MAX_DELAY) != HAL_OK);
	}
	else
	{
		putValueToTXBuffer('o', 1);
		while (HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen((uint8_t*)txBuffer), HAL_MAX_DELAY) != HAL_OK);
	}
}

void handleTime(volatile uint8_t *time)
{
	(*time) = (rxBuffer[1]-0x30)*1000 + (rxBuffer[2]-0x30)*100 + (rxBuffer[3]-0x30)*10 + (rxBuffer[4]-0x30);
}

void handlePWM(volatile uint16_t *pulseWidth)
{
	uint16_t temp = (rxBuffer[1]-0x30)*1000 + (rxBuffer[2]-0x30)*100 + (rxBuffer[3]-0x30)*10 + (rxBuffer[4]-0x30);
	if (temp > 10) temp = 10;
	(*pulseWidth) = temp*2000;
}

// ------------------------------------ MAIN MONITOR ------------------------------------------
void mainMenu()																								// Chuong trinh dieu khien he thong
{
	if (setEnable == 1)
	{
		EnableEOS = 1;
		Seconds = 0;
		Minutes = 0;
		
		setEnable = 0;
	}
	if (EnableEOS == 1)
	{
		EOS_Enable(50);																						// Khoi chay EOS 50ms bat len 1 lan
		while (EnableEOS == 1)																		// Cho` den khi enableEOS == 0
		{
			HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI); // bat che ngu cho vi dieu khien
		}
		EOS_Disable();																						// Tat EOS
	}
// ---------------
	if (Minutes == SetMinutes)																	// Neu phut tang len bang phut da thiet lap
	{
		Minutes = 0;					// set thoi gian cho an va thoi gian nghi ve 0
		Seconds = SetSeconds;
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);									// cho phep timer tao xung PWM
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PulseWidth);
		
		for (uint8_t i = 0; i < SetSeconds; i++)
		{
			HAL_Delay(1000);
			RPM = (CountingPulse/(float)ENCODER_PULSE)*60;
			CountingPulse = 0;		// reset bien dem xung
			
			putValueToTXBuffer('W', RPM);
			while (HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen((uint8_t*)txBuffer), HAL_MAX_DELAY) != HAL_OK); // truyen gia tri toc do dong co len esp32
		}
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);								// tat timer tao xung PWM
		
		CountingPulse = 0;		// reset bien dem xung
		RPM = 0;
		putValueToTXBuffer('W', 0);
		while (HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen((uint8_t*)txBuffer), HAL_MAX_DELAY) != HAL_OK);
		EnableEOS = 1;				// Bat lai EOS
	}
	if (CheckLoadCellTime == 30)
	{
		checkFood();
		CheckLoadCellTime = 0;
		EnableEOS = 1;
	}
// ---------------
	if (rxBuffer[0] != 0) 																		// Cho nay xu li du lieu esp gui ve
	{
		HAL_Delay(10);
		if (rxBuffer[0] == 'e')																	// Bat tat de set up thoi gian (e: enable, e0: chay EOS, e1: tat EOS va vao menu)
		{
			if (rxBuffer[4] == '1') 
			{
				EnableEOS = 1;
				Seconds = 0;
				Minutes = 0;
			}
		}
		else if (rxBuffer[0] == 's')																	// Xu li gia tri thoi gian : sxx myy (s: giay, xx so giay, m: phut, yy: so phut)
		{
			// handle Seconds
			handleTime(&SetSeconds);															// Xu li giay
		}
		else if (rxBuffer[0] == 'm')
		{
			// handle Minutes
			handleTime(&SetMinutes);														  // Xu li phut
			//
		}
		else if (rxBuffer[0] == 'p')
		{
			// handle Pulse
			handlePWM(&PulseWidth);														  // Xu li xung
			// 
		}
		//memset(rxBuffer, 0, 4);        // xoa mang RX buffer cua UART
		rxBuffer[0] = 0; rxBuffer[1] = 0; rxBuffer[2] = 0; rxBuffer[3] = 0; rxBuffer[4] = 0;
	}
// ---------------
	if (EnableEOS == 0)
	{
		setUpTime();
	}
}

void setUpSystemInit()
{
	putValueToTXBuffer('e', EnableEOS);
	while (HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen((uint8_t*)txBuffer), HAL_MAX_DELAY) != HAL_OK);
	putValueToTXBuffer('s', SetSeconds);
	while (HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen((uint8_t*)txBuffer), HAL_MAX_DELAY) != HAL_OK);
	putValueToTXBuffer('m', SetMinutes);
	while (HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen((uint8_t*)txBuffer), HAL_MAX_DELAY) != HAL_OK);
	putValueToTXBuffer('p', PulseWidth);
	while (HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen((uint8_t*)txBuffer), HAL_MAX_DELAY) != HAL_OK);
	putValueToTXBuffer('W', 0);
	while (HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen((uint8_t*)txBuffer), HAL_MAX_DELAY) != HAL_OK);
	checkFood();
}

// ----------------------------------- SYSTICK HANDLER ----------------------------------------
void SysTick_Handler(void)
{
	if (EnableEOS == 1)
	{
		UpToSecond++;
		if (UpToSecond > 19)  // EOS 50ms bat len, upToSecond = 20 => 20*50ms = 1s -> tang 1 giay
		{
			UpToSecond = 0;
			runTime();					// tang 1 giay len
			
			if (Minutes == SetMinutes) EnableEOS = 0;
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);				// Kiem tra xem chuong trinh co bi treo hay khong bang cach cho LED chop tat
		}
		if (Minutes == SetMinutes)
		{
			 EnableEOS = 0;
		}
		if (CheckLoadCellTime == 30)
		{
			EnableEOS = 0;
		}
		if (readButtons() == 1)				// neu nhan nut mode thi thoat ra khoi EOS
		{
			ModeAck++;
			if (ModeAck >= 3) EnableEOS = (EnableEOS + 1) % 2;
		}
		if (rxBuffer[0] == 'e')							// neu esp32 gui lenh ngung EOS thi thoat ra khoi EOS
		{
			if (rxBuffer[4] == '0') EnableEOS = 0;
		}
	}
	else
	{
		HAL_IncTick();
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  hx711_init(&loadcell, HX711_CLK_GPIO_Port, HX711_CLK_Pin, HX711_DATA_GPIO_Port, HX711_DATA_Pin);
  hx711_coef_set(&loadcell, 1001.81); 																		  // read afer calibration
  hx711_tare(&loadcell, 10);
	
  HAL_UART_Receive_DMA(&huart1, rxBuffer, 5); 												 // Receive data from data buffer interrupt mode
	setUpSystemInit();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		mainMenu();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

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
