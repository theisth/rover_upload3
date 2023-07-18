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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DC_MOTORS.h"
#include "stdlib.h"
#include <stdbool.h>
#include <math.h>
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint32_t LeftFront_EncoderCounter = 0;
volatile uint32_t RightFront_EncoderCounter = 0;

volatile uint32_t LeftRear_EncoderCounter = 0;
volatile uint32_t RightRear_EncoderCounter = 0;

volatile bool velocityCalculationFlag = true;

//--------------------- PID --------------------------------//


volatile bool pidFlag = false;

//float setVelocity = 3.2;

//--------------------- RX Data ----------------------------//
uint8_t rxbuff[8];
uint8_t txbuff[16];
uint8_t txempty;
uint8_t rxfull;

int32_t linear_number;
int32_t angular_number;

float linearVel_MeterDivSec = 0.0;
float angularVel_RadDivSec = 0.0;

int16_t speedLeftWHL = 0;
int16_t speedRightWHL = 0;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if (huart == &huart2)
  {
    rxfull = 1;
  }
}

//        //	HAL_TIM_ACTIVE_CHANNEL_1 ||------|| HAL_TIM_ACTIVE_CHANNEL_2
//        //	   	         				|  |
//        //		         				|  |
//        //	HAL_TIM_ACTIVE_CHANNEL_3 ||------|| HAL_TIM_ACTIVE_CHANNEL_4


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){

	 if((htim->Instance == TIM2) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)){

		 LeftFront_EncoderCounter++;
//		if(velocityCalculationFlag_WHL_1){
//			//HAL_TIM_Base_Start_IT(&htim4);
//			velocityCalculationFlag_WHL_1 = false;
//		}
	 }
	 if((htim->Instance == TIM3) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)){

		 RightFront_EncoderCounter++;
//		if(velocityCalculationFlag_WHL_2){
//			//HAL_TIM_Base_Start_IT(&htim4);
//			velocityCalculationFlag_WHL_2 = false;
//		}
	 }
	 if((htim->Instance == TIM4) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)){

		 LeftRear_EncoderCounter++;
//		if(velocityCalculationFlag_WHL_3){
//			//HAL_TIM_Base_Start_IT(&htim4);
//			velocityCalculationFlag_WHL_3 = false;
//		}
	 }
	 if((htim->Instance == TIM5) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)){

		 RightRear_EncoderCounter++;
//		if(velocityCalculationFlag_WHL_4){
//			//HAL_TIM_Base_Start_IT(&htim4);
//			velocityCalculationFlag_WHL_4 = false;
//		}
	 }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	velocityCalculationFlag = true;

//	HAL_TIM_Base_Stop_IT(&htim4);
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

//        	0,2,4,6 IN1 for forward
//        	1,3,5,7 IN2 for backward
//
//          Left (+) a. vel.		Right (-) a. vel.
//
//        	Pin_B 0,1 ||------|| Pin_B 4,5
//        	   	         |  |
//        		         |  |
//        	Pin_B 2,3 ||------|| Pin_B 6,7

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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  /*------------------------ Encoders -------------------------*/
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim9);

  struct WHEEL L_Front_WHL = {0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0};
  struct WHEEL R_Front_WHL = {0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0};
  struct WHEEL L_Rear_WHL = {0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0};
  struct WHEEL R_Rear_WHL = {0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0};

  HAL_UART_Receive_DMA(&huart2, rxbuff, 8);
  rxfull = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      if (rxfull == 1){
        rxfull = 0;
        linear_number = (int32_t)(rxbuff[0]<<24) + (int32_t)(rxbuff[1]<<16) + (int32_t)(rxbuff[2]<<8) + rxbuff[3];
        angular_number = (int32_t)(rxbuff[4]<<24) + (int32_t)(rxbuff[5]<<16) + (int32_t)(rxbuff[6]<<8) + rxbuff[7];

        linearVel_MeterDivSec = (float)(linear_number/10000000);
        angularVel_RadDivSec = (float)(angular_number/10000000);


		speedLeftWHL = (linearVel_MeterDivSec - angularVel_RadDivSec);
		speedRightWHL = (linearVel_MeterDivSec + angularVel_RadDivSec);

     }

     if(velocityCalculationFlag){

    	 //------------------------------------------- 1 ---------------------------------------//

		 L_Front_WHL.encoderCount = LeftFront_EncoderCounter;
		 LeftFront_EncoderCounter = 0; // clear the counter

		 L_Front_WHL.rotationWheel = ((float)L_Front_WHL.encoderCount*100.0)/(float)(1024*100);
		 L_Front_WHL.rotationEncoder = ((float)L_Front_WHL.encoderCount*100.0)/(float)(1024); // not necessary, just for control

		 // check the math
		 L_Front_WHL.wheelRPM = (L_Front_WHL.rotationWheel)*60;
		 L_Front_WHL.wheelRadDivSec = (L_Front_WHL.wheelRPM)*RPMtoRadDivSec;

		 L_Front_WHL.errorPI = (fabs(speedLeftWHL)) - L_Front_WHL.wheelRadDivSec;
		 L_Front_WHL.integralPI += L_Front_WHL.errorPI;

		 //pwmSignal = Kp*error + Ki*integral;
		 if(speedLeftWHL > 0)
			 L_Front_WHL.signalPWM = 1525 + (KP*L_Front_WHL.errorPI + KI*L_Front_WHL.integralPI);
		 else if(speedLeftWHL == 0)		// ??? check this condition
			 L_Front_WHL.signalPWM = 0;
		 else
			 L_Front_WHL.signalPWM = 1525 - (KP*L_Front_WHL.errorPI + KI*L_Front_WHL.integralPI);

		 //------------------------------------------- 2 ---------------------------------------//
		 R_Front_WHL.encoderCount = RightFront_EncoderCounter;
		 RightFront_EncoderCounter = 0; // clear the counter

		 R_Front_WHL.rotationWheel = ((float)R_Front_WHL.encoderCount*100.0)/(float)(1024*100);
		 R_Front_WHL.rotationEncoder = ((float)R_Front_WHL.encoderCount*100.0)/(float)(1024); // not necessary, just for control

		 // check the math
		 R_Front_WHL.wheelRPM = (R_Front_WHL.rotationWheel)*60;
		 R_Front_WHL.wheelRadDivSec = (R_Front_WHL.wheelRPM)*RPMtoRadDivSec;

		 R_Front_WHL.errorPI = (fabs(speedLeftWHL)) - R_Front_WHL.wheelRadDivSec;
		 R_Front_WHL.integralPI += R_Front_WHL.errorPI;

		 //pwmSignal = Kp*error + Ki*integral;
		 if(speedLeftWHL > 0)
			 R_Front_WHL.signalPWM = 1525 + (KP*R_Front_WHL.errorPI + KI*R_Front_WHL.integralPI);
		 else if(speedLeftWHL == 0)		// ??? check this condition
			 R_Front_WHL.signalPWM = 0;
		 else
			 R_Front_WHL.signalPWM = 1525 - (KP*R_Front_WHL.errorPI + KI*R_Front_WHL.integralPI);
		 //------------------------------------------- 3 ---------------------------------------//
		 L_Rear_WHL.encoderCount = LeftRear_EncoderCounter;
		 LeftRear_EncoderCounter = 0; // clear the counter

		 L_Rear_WHL.rotationWheel = ((float)L_Rear_WHL.encoderCount*100.0)/(float)(1024*100);
		 L_Rear_WHL.rotationEncoder = ((float)L_Rear_WHL.encoderCount*100.0)/(float)(1024); // not necessary, just for control

		 // check the math
		 L_Rear_WHL.wheelRPM = (L_Rear_WHL.rotationWheel)*60;
		 L_Rear_WHL.wheelRadDivSec = (L_Rear_WHL.wheelRPM)*RPMtoRadDivSec;

		 L_Rear_WHL.errorPI = (fabs(speedLeftWHL)) - L_Rear_WHL.wheelRadDivSec;
		 L_Rear_WHL.integralPI += L_Rear_WHL.errorPI;

		 //pwmSignal = Kp*error + Ki*integral;
		 if(speedLeftWHL > 0)
			 L_Rear_WHL.signalPWM = 1525 + (KP*L_Rear_WHL.errorPI + KI*L_Rear_WHL.integralPI);
		 else if(speedLeftWHL == 0)		// ??? check this condition
			 L_Rear_WHL.signalPWM = 0;
		 else
			 L_Rear_WHL.signalPWM = 1525 - (KP*L_Rear_WHL.errorPI + KI*L_Rear_WHL.integralPI);
		 //------------------------------------------- 4 ---------------------------------------//
		 R_Rear_WHL.encoderCount = RightRear_EncoderCounter;
		 RightRear_EncoderCounter = 0; // clear the counter

		 R_Rear_WHL.rotationWheel = ((float)R_Rear_WHL.encoderCount*100.0)/(float)(1024*100);
		 R_Rear_WHL.rotationEncoder = ((float)R_Rear_WHL.encoderCount*100.0)/(float)(1024); // not necessary, just for control

		 // check the math
		 R_Rear_WHL.wheelRPM = (R_Rear_WHL.rotationWheel)*60;
		 R_Rear_WHL.wheelRadDivSec = (R_Rear_WHL.wheelRPM)*RPMtoRadDivSec;

		 R_Rear_WHL.errorPI = (fabs(speedLeftWHL)) - R_Rear_WHL.wheelRadDivSec;
		 R_Rear_WHL.integralPI += R_Rear_WHL.errorPI;

		 //pwmSignal = Kp*error + Ki*integral;
		 if(speedLeftWHL > 0)
			 R_Rear_WHL.signalPWM = 1525 + (KP*R_Rear_WHL.errorPI + KI*R_Rear_WHL.integralPI);
		 else if(speedLeftWHL == 0)		// ??? check this condition
			 R_Rear_WHL.signalPWM = 0;
		 else
			 R_Rear_WHL.signalPWM = 1525 - (KP*R_Rear_WHL.errorPI + KI*R_Rear_WHL.integralPI);


		 velocityCalculationFlag = false;
     }


//---------------------------------- 12.04.23 -----------------------------------------------------------//
/*

		if(pwmSignal == 0)
			pwmSignal2 = 1525;
		else if(pwmSignal > 1750)
			pwmSignal2 = 1750;
		else if(pwmSignal < 1250)
			pwmSignal2 = 1250;
		else
			pwmSignal2 = pwmSignal;

		htim1.Instance->CCR1 = pwmSignal2;
		htim1.Instance->CCR2 = pwmSignal2;
		htim1.Instance->CCR3 = pwmSignal2;
		htim1.Instance->CCR4 = pwmSignal2;
*/
//---------------------------------- 12.04.23 -----------------------------------------------------------//

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 62500-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 2000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 42000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 2000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 42000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 2000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 42000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 2000-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 42000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 20-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 42000-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

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
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

