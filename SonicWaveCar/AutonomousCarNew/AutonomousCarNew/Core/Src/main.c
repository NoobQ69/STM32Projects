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
#include "NRF24L01.h"
#include "HC_SR04.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_LENGTH_DATA 32
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
extern uint8_t Distance;

uint8_t LimitDistance = 20, ReadingSensorAvailable = 1; // 5cm

uint32_t Count1 = 0, Count2 = 0;

uint16_t value;

uint8_t RX_Address[] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};

uint8_t RX_Data[MAX_LENGTH_DATA];

typedef struct ReceivedValue
{
	char command1;
	char command2;
	uint16_t data;
} RECEIVED_VALUE;

typedef struct Direction
{
	uint8_t left;
	uint8_t right;
	uint8_t up;
	uint8_t back;
} DIRECTION;
	
RECEIVED_VALUE DataPackage;

DIRECTION DirectionValue;

void (*ModeFunction) ();

// =============================== FOR PID VARIABLE =================================
//double T, Pulse, Speed, ConstitutionSpeed, E, E1, E2, Alpha, Beta, Gamma, Kp, Kd, Ki, Output, LastOutPut;
double T;

typedef struct PIDType
{
	double Pulse, Speed, ConstitutionSpeed, E, E1, E2, Alpha, Beta, Gamma, Kp, Kd, Ki, Output, LastOutPut;
} PID_TYPE;

PID_TYPE Motor1;
PID_TYPE Motor2;
	

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// =========================== FOR PID ALGORITHM =================================

void SetUpPID(PID_TYPE *motor, double cSpeed) 
{
	motor->ConstitutionSpeed = cSpeed; // constitution speed in RPM
	//Speed = 0;
	motor->E = motor->E1 = motor->E2 = 0;
	//motor->Output  0;
	motor->LastOutPut = 0;
	T = 0.01;
	motor->Kp = 5, motor->Kd = 0.01, motor->Ki = 0.03;
}

double PID(PID_TYPE *motor, uint32_t *pulse, double T)
{
	motor->Speed = ((double)(*pulse)/20)*(1/T)*60;
	*pulse = 0;
	motor->E = motor->ConstitutionSpeed - motor->Speed;
	motor->Alpha = 2*T*motor->Kp + motor->Ki*T*T + 2*motor->Kd;
	motor->Beta = T*T*motor->Ki - 4*motor->Kd - 2*T*motor->Kp;
	motor->Gamma = 2*motor->Kd;
	motor->Output = (motor->Alpha*motor->E + motor->Beta*motor->E1 + motor->Gamma*motor->E2 + 2*T*motor->LastOutPut)/(2*T);
	motor->LastOutPut = motor->Output;
	motor->E2 = motor->E1;
	motor->E1 = motor->E;
	if (motor->Output > 2000) motor->Output = 2000;
	if (motor->Output < 0) motor->Output = 0;
	
	return motor->Output;
}

// =========================== FOR ENCODER =================================
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == ENCODER1_Pin)
	{
		Count1++;
	}
	else if (GPIO_Pin == ENCODER2_Pin)
	{
		Count2++;
	}
}
// =========================== FOR SERVO MOTOR

void RotateAngle(uint16_t angleValue)
{
	uint16_t pwmValue = angleValue * ((2500 - 500)/ 180 - 0) + 500;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwmValue);
}

// =========================== FOR MOTOR =================================
void MoveUp(double speed, uint8_t isPID)
{
	HAL_GPIO_WritePin(GPIOB, IN1_A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, IN1_B_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, IN2_A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, IN2_B_Pin, GPIO_PIN_RESET);
	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	
	if (isPID)
	{
		SetUpPID(&Motor1,600);
		SetUpPID(&Motor2,600);
		for (int i = 0; i < 10; i++)
		{
			speed = PID(&Motor1,&Count1, 0.01);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed); 
			speed = PID(&Motor2,&Count2, 0.01);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
			HAL_Delay(10);
		}
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
		HAL_Delay(80);
	}
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
}

void MoveBack(uint16_t speed)
{
	HAL_GPIO_WritePin(GPIOB, IN1_A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, IN1_B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, IN2_A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, IN2_B_Pin, GPIO_PIN_SET);
	
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed); 
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_Delay(80);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
}

void TurnLeft(uint16_t speed)
{
	HAL_GPIO_WritePin(GPIOB, IN1_A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, IN1_B_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, IN2_A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, IN2_B_Pin, GPIO_PIN_SET);
	
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_Delay(80);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
}

void TurnRight(uint16_t speed)
{
	HAL_GPIO_WritePin(GPIOB, IN1_A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, IN1_B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, IN2_A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, IN2_B_Pin, GPIO_PIN_RESET);
	
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_Delay(80);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
}

void StopMove()
{
	HAL_GPIO_WritePin(GPIOB, IN1_A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, IN1_B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, IN2_A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, IN2_B_Pin, GPIO_PIN_RESET);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_Delay(80);
}

// =========================== FOR HANDLE RECEIVED VALUES =================================
void ClearDataPackage()
{
	DataPackage.command1 = 0;
	DataPackage.command2 = 0;
	DataPackage.data = 0;
}

void ResetDirection()
{
	DirectionValue.back = 0;
	DirectionValue.left = 0;
	DirectionValue.right = 0;
	DirectionValue.up = 0;
}

uint16_t MappingValue(uint16_t value)
{
	return value * ((float)(2000 - 500) / (float)(2048)) + 500;
}

uint16_t ConvertStringToIntData()
{
	value = 0;
	for (uint8_t i = 2; i < 6; i++)
	{
		value = value*10 + (RX_Data[i] - 0x30);
	}
	return value;
}

void Mode0()
{
	switch (DataPackage.command1)
	{
		case 'L':
		{
			TurnLeft(DataPackage.data);
			ClearDataPackage();
			break;
		}
		case 'R':
		{
			TurnRight(DataPackage.data);
			ClearDataPackage();
			break;
		}
		case 'U':
		{
			MoveUp(DataPackage.data, 0);
			ClearDataPackage();
			break;
		}
		case 'B':
		{
			MoveBack(DataPackage.data);
			ClearDataPackage();
			break;
		}
		default:
		{
			break;
		}
	}
}

void Mode1()
{
	if (ReadingSensorAvailable)
	{
		HCSR04_Read();
		if (Distance < LimitDistance)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			ReadingSensorAvailable = 0;
			StopMove();
			RotateAngle(15);
			HCSR04_Read();
			HAL_Delay(200);
			uint8_t valueLeft = Distance;
			RotateAngle(170);
			HCSR04_Read();
			HAL_Delay(200);
			uint8_t valueRight = Distance;
			RotateAngle(80);
			
			if (valueLeft < 20 && valueRight < 20)
			{
				DirectionValue.back = 5;
			}
			if (valueLeft > valueRight)
			{
				DirectionValue.left = 12;
			}
			else
			{
				DirectionValue.right = 12;
			}
			DirectionValue.back += 5;
		}
		else
		{
			ResetDirection();
		}
	}
	if (DirectionValue.back > 0)
	{
		MoveBack(1000);
		DirectionValue.back--;
		return;
	}
	else if (DirectionValue.left > 0)
	{
		TurnLeft(1500);
		DirectionValue.left--;
		return;
	}
	else if (DirectionValue.right > 0)
	{
		TurnRight(1500);
		DirectionValue.right--;
		return;
	}
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	ReadingSensorAvailable = 1;
	MoveUp(500, 1); // 500 rpm
}

void ProcessReceivedValue()
{
	if (RX_Data[0] == 'M')
	{
		if (RX_Data[1] == '0')
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			ModeFunction = Mode0;
		}
		else if (RX_Data[1] == '1')
		{
			ModeFunction = Mode1;
		}
	}
	else
	{
		if (RX_Data[0] == 'L' || RX_Data[0] == 'B')
		{
			DataPackage.command1 = RX_Data[0];
			DataPackage.data = MappingValue(2048 - ConvertStringToIntData());
		}
		else if (RX_Data[0] == 'R' || RX_Data[0] == 'U')
		{
			DataPackage.command1 = RX_Data[0];
			DataPackage.data = MappingValue(ConvertStringToIntData() - 2048);
		}
	}
}

// =========================== FOR MAIN PROCESS FUNCTION =================================

void MainLoop()
{
	if (isDataAvailable(1) == 1)
	{
		NRF24_Receive(RX_Data);
		ProcessReceivedValue();
	}
	ModeFunction();
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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	NRF24_Init();
	NRF24_RxMode(RX_Address, 10);
	
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	ModeFunction = Mode0;
	RotateAngle(80);
	
	/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		MainLoop();
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim1.Init.Prescaler = 16-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2000;
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
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffffffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 16-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000;
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
  sConfigOC.Pulse = 2000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|CE_Pin|CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IN1_A_Pin|IN1_B_Pin|IN2_A_Pin|IN2_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 CE_Pin CSN_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|CE_Pin|CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_A_Pin IN1_B_Pin IN2_A_Pin IN2_B_Pin */
  GPIO_InitStruct.Pin = IN1_A_Pin|IN1_B_Pin|IN2_A_Pin|IN2_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ENCODER1_Pin ENCODER2_Pin */
  GPIO_InitStruct.Pin = ENCODER1_Pin|ENCODER2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	 /*Configure GPIO pins : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
