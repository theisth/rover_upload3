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
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
static void MX_USART6_UART_Init(void);
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

float speedLeftWHL_1_ZEROcond = 0;
float speedLeftWHL_2_ZEROcond = 0;

float speedRightWHL_1_ZEROcond = 0;
float speedRightWHL_2_ZEROcond = 0;

float speedLeftWHL = 0;
float speedRightWHL = 0;

//float KI = 3.03;
//float KP = 21.2;

float KI = 0.019;
float KP = 0.76;

float variableFRONT = 0;
float variableREAR = 0;

//volatile uint16_t speedArrayLF[500] = {0};
//volatile uint16_t speedArrayLR[500] = {0};
//volatile uint16_t speedArrayRF[500] = {0};
//volatile uint16_t speedArrayRR[500] = {0};

//------------- TEMP ---------------//
int16_t linear_pwm_value=0;
int16_t angular_pwm_value=0;

float errorPWM_WHL1 = 0;
float errorPWM_WHL2 = 0;
float errorPWM_WHL3 = 0;
float errorPWM_WHL4 = 0;

int16_t speed_left = 0;
int16_t speed_right = 0;
//---------------------------------//

struct WHEEL L_Front_WHL = {0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1525};
struct WHEEL R_Front_WHL = {0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1525};
struct WHEEL L_Rear_WHL = {0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1525};
struct WHEEL R_Rear_WHL = {0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1525};

//---------------- TEMP -----------------------------//
uint16_t pwmValue = 1525;
uint16_t pwmValue2 = 1525;

uint16_t fuckingWait = 0;
uint8_t oneSecCounter = 0;
uint16_t arrayCounter = 0;
int address = 0x08008000;
//---------------------------------------------------//

uint32_t data1,data2;
uint32_t tim3_counter=0;
uint8_t transmitbuffer[44]={0};
uint8_t datax=0;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if (huart == &huart2){
    rxfull = 1;
  }
}

//        //	HAL_TIM_ACTIVE_CHANNEL_1 ||------|| HAL_TIM_ACTIVE_CHANNEL_2
//        //	   	         				|  |
//        //		         				|  |
//        //	HAL_TIM_ACTIVE_CHANNEL_3 ||------|| HAL_TIM_ACTIVE_CHANNEL_4


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	velocityCalculationFlag = true;
}

void my_transmit(void);

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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  /*------------------------ Encoders -------------------------*/
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);

  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);

  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);

  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_2);


  HAL_TIM_Base_Start_IT(&htim9);

  HAL_UART_Receive_DMA(&huart2, rxbuff, 8);
  rxfull = 0;

  //HAL_Delay(10000);

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

        linearVel_MeterDivSec = ((float)linear_number/1000000.0); // x10
        angularVel_RadDivSec = ((float)angular_number/1000000.0); // x10

     }

	TIM1->CCR1 = R_Rear_WHL.signalPWM;
	TIM1->CCR2 = R_Front_WHL.signalPWM;
//	TIM1->CCR3 = L_Rear_WHL.signalPWM;
//	TIM1->CCR4 = L_Front_WHL.signalPWM;


//	TIM1->CCR1 = pwmValue;
//	TIM1->CCR2 = pwmValue;
//	TIM1->CCR3 = pwmValue2;
//	TIM1->CCR4 = pwmValue2;

     if(velocityCalculationFlag){

    	 /*fuckingWait += 1;
    	 oneSecCounter +=1;

    	 if(fuckingWait >= 100){
    		 fuckingWait = 0;
    		 if(pwmValue < 1900){
    			 pwmValue += 10;
    			 pwmValue2 -= 10;
    		 }
    		 else{
    			 pwmValue = 1525;
    			 pwmValue2 = 1525;
    			 HAL_Delay(100000);
    		 }


    	 }
    	 if(oneSecCounter >= 10){
    		 oneSecCounter = 0;

    		 if(arrayCounter < 500){
        		 speedArrayLF[arrayCounter] = (uint16_t)(L_Front_WHL.wheelRadDivSec 	* 100);
        		 speedArrayLR[arrayCounter] = (uint16_t)(L_Rear_WHL.wheelRadDivSec 		* 100);
        		 speedArrayRF[arrayCounter] = (uint16_t)(R_Front_WHL.wheelRadDivSec 	* 100);
        		 speedArrayRR[arrayCounter] = (uint16_t)(R_Rear_WHL.wheelRadDivSec 		* 100);
        			HAL_FLASH_Unlock();
        			HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, speedArrayLF[arrayCounter]);
        			address += 2;
        			HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, speedArrayLR[arrayCounter]);
        			address += 2;
        			HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, speedArrayRF[arrayCounter]);
        			address += 2;
        			HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, speedArrayRR[arrayCounter]);
        			address += 2;
        			HAL_FLASH_Lock();
        		 arrayCounter += 1;
    		 }

    	 }*/

    	 speedLeftWHL = (linearVel_MeterDivSec + angularVel_RadDivSec);
    	 speedRightWHL = (linearVel_MeterDivSec - angularVel_RadDivSec);

		 LeftFront_EncoderCounter = __HAL_TIM_GET_COUNTER(&htim2);
		 if(LeftFront_EncoderCounter>30000)
		 {
			 LeftFront_EncoderCounter = 65000 - LeftFront_EncoderCounter;
		 }
		 LeftRear_EncoderCounter = __HAL_TIM_GET_COUNTER(&htim4);
		 if(LeftRear_EncoderCounter>30000)
		 {
			 LeftRear_EncoderCounter = 65000 - LeftRear_EncoderCounter;
		 }
		 RightFront_EncoderCounter = __HAL_TIM_GET_COUNTER(&htim3);
		 if(RightFront_EncoderCounter>30000)
		 {
			 RightFront_EncoderCounter = 65000 - RightFront_EncoderCounter;
		 }
		 RightRear_EncoderCounter = __HAL_TIM_GET_COUNTER(&htim5);
		 if(RightRear_EncoderCounter>30000)
		 {
			 RightRear_EncoderCounter = 65000 - RightRear_EncoderCounter;
		 }

		TIM2->CNT = 0;
		TIM3->CNT = 0;
		TIM4->CNT = 0;
		TIM5->CNT = 0;

    	 //------------------------------------------- 1 ---------------------------------------//

//		 L_Front_WHL.encoderCount = LeftFront_EncoderCounter;
//		 //LeftFront_EncoderCounter = 0; // clear the counter
//
//		 //L_Front_WHL.rotationEncoder = ((float)L_Front_WHL.encoderCount*100.0)/(4096.0); // not necessary, just for control
//		 L_Front_WHL.rotationWheel = ((float)L_Front_WHL.encoderCount)/(4096.0);
//
//
//		 // check the math
//		 L_Front_WHL.wheelRPM = (L_Front_WHL.rotationWheel)*600.0;
//		 L_Front_WHL.wheelRadDivSec = (L_Front_WHL.wheelRPM)*RPMtoRadDivSec;
//
//		 errorPWM_WHL1 = (fabs(speedLeftWHL)) - L_Front_WHL.wheelRadDivSec;
//
//		 L_Front_WHL.errorPI = round(240/8.94*errorPWM_WHL1 + 23.6241);
//		 L_Front_WHL.integralPI += L_Front_WHL.errorPI;
//
//		 //pwmSignal = Kp*error + Ki*integral;
//		 if(speedLeftWHL > 0)
//			 L_Front_WHL.signalPWM = 1525 + fabs(KP*L_Front_WHL.errorPI + KI*L_Front_WHL.integralPI);
//		 else if(speedLeftWHL == 0){		// ??? check this condition
//			 L_Front_WHL.signalPWM = 1525;
//		 	 L_Front_WHL.integralPI = 0;
//		 }
//		 else
//			 L_Front_WHL.signalPWM = 1525 - fabs(KP*L_Front_WHL.errorPI + KI*L_Front_WHL.integralPI);
//
//		 if(L_Front_WHL.signalPWM > 2000)
//			 L_Front_WHL.signalPWM = 1900;
//		 if(L_Front_WHL.signalPWM < 1000)
//			 L_Front_WHL.signalPWM = 1100;

		 //------------------------------------------- 2 ---------------------------------------//
		 R_Front_WHL.encoderCount = RightFront_EncoderCounter;
		 //RightFront_EncoderCounter = 0; // clear the counter

		 //R_Front_WHL.rotationEncoder = ((float)R_Front_WHL.encoderCount*100.0)/(4096.0);
		 R_Front_WHL.rotationWheel = ((float)R_Front_WHL.encoderCount)/(4096.0); // not necessary, just for control

		 // check the math
		 R_Front_WHL.wheelRPM = (R_Front_WHL.rotationWheel)*600.0;
		 R_Front_WHL.wheelRadDivSec = (R_Front_WHL.wheelRPM)*RPMtoRadDivSec;

		 R_Front_WHL.errorPI = (fabs(speedRightWHL)) - R_Front_WHL.wheelRadDivSec;
		 R_Front_WHL.integralPI += R_Front_WHL.errorPI;

//		 errorPWM_WHL2 = ((fabs(speedRightWHL)) - R_Front_WHL.wheelRadDivSec);
//
//		 R_Front_WHL.errorPI = round((240.0/8.94)*fabs(errorPWM_WHL2) + 23.6241);
//
//		 R_Front_WHL.integralPI += R_Front_WHL.errorPI;


		 //pwmSignal = Kp*error + Ki*integral;
		 if(speedRightWHL > 0)
			 R_Front_WHL.signalPWM = 1525 + round((240.0/8.94)*fabs(KPzzzzzzujhb                                                bbbbbbbbbbbbbbbbbbbbbbbbbb                                                        ::5*R_Front_WHL.errorPI + KI*R_Front_WHL.integralPI) + 23.6241);
		 else if(speedRightWHL == 0){		// ??? check this condition
			 R_Front_WHL.signalPWM = 1525;
		 	 R_Front_WHL.integralPI = 0;
		 }
		 else
			 R_Front_WHL.signalPWM = 1525 - round((240.0/8.94)*fabs(KP*R_Front_WHL.errorPI + KI*R_Front_WHL.integralPI) + 23.6241);

		 if(R_Front_WHL.signalPWM > 2000)
			 R_Front_WHL.signalPWM = 1900;
		 if(R_Front_WHL.signalPWM < 1000)
			 R_Front_WHL.signalPWM = 1100;
		 //------------------------------------------- 3 ---------------------------------------//
//		 L_Rear_WHL.encoderCount = LeftRear_EncoderCounter;
//		 //LeftRear_EncoderCounter = 0; // clear the counter
//
//		 //L_Rear_WHL.rotationEncoder = ((float)L_Rear_WHL.encoderCount*100.0)/(4096.0); // not necessary, just for control
//		 L_Rear_WHL.rotationWheel = ((float)L_Rear_WHL.encoderCount)/(4096.0); // 1024*4
//
//		 // check the math
//		 L_Rear_WHL.wheelRPM = (L_Rear_WHL.rotationWheel)*600.0; // 60 sec * 10 msec
//		 L_Rear_WHL.wheelRadDivSec = (L_Rear_WHL.wheelRPM)*RPMtoRadDivSec;
//
//		 errorPWM_WHL3 = (fabs(speedLeftWHL)) - L_Rear_WHL.wheelRadDivSec;
//
//		 //23.6241
//		 L_Rear_WHL.errorPI = round(240/8.94*errorPWM_WHL3);
//		 L_Rear_WHL.integralPI += L_Rear_WHL.errorPI;
//
//		 //pwmSignal = Kp*error + Ki*integral;
//		 if(speedLeftWHL > 0)
//			 L_Rear_WHL.signalPWM = 1525 + fabs(KP*L_Rear_WHL.errorPI + KI*L_Rear_WHL.integralPI);
//		 else if(speedLeftWHL == 0){		// ??? check this condition
//			 L_Rear_WHL.signalPWM = 1525;
//		 	 L_Rear_WHL.integralPI = 0;
//		 }
//		 else
//			 L_Rear_WHL.signalPWM = 1525 - fabs(KP*L_Rear_WHL.errorPI + KI*L_Rear_WHL.integralPI);
//
//		 if(L_Rear_WHL.signalPWM > 2000)
//			 L_Rear_WHL.signalPWM = 1900;
//		 if(L_Rear_WHL.signalPWM < 1000)
//			 L_Rear_WHL.signalPWM = 1100;
		 //------------------------------------------- 4 ---------------------------------------//
		 R_Rear_WHL.encoderCount = RightRear_EncoderCounter;
		 //RightRear_EncoderCounter = 0; // clear the counter

		 //R_Rear_WHL.rotationEncoder = ((float)R_Rear_WHL.encoderCount*100.0)/(4096.0); // not necessary, just for control
		 R_Rear_WHL.rotationWheel = ((float)R_Rear_WHL.encoderCount)/(4096.0);

		 // check the math
		 R_Rear_WHL.wheelRPM = (R_Rear_WHL.rotationWheel)*600.0;
		 R_Rear_WHL.wheelRadDivSec = (R_Rear_WHL.wheelRPM)*RPMtoRadDivSec;

 		 R_Rear_WHL.errorPI = (fabs(speedRightWHL)) - R_Rear_WHL.wheelRadDivSec;
 		 R_Rear_WHL.integralPI += R_Rear_WHL.errorPI;
//		 errorPWM_WHL4 = (fabs(speedRightWHL)) - R_Rear_WHL.wheelRadDivSec;
//
//		 R_Rear_WHL.errorPI = round((240.0/8.94)*fabs(errorPWM_WHL4) + 23.6241);
//		 R_Rear_WHL.integralPI += R_Rear_WHL.errorPI;

		 //variableREAR = (KP*R_Rear_WHL.errorPI + KI*R_Rear_WHL.integralPI);

		 //pwmSignal = Kp*error + Ki*integral;
		 if(speedRightWHL > 0)
			 R_Rear_WHL.signalPWM = 1525 + round((240.0/8.94)*fabs(KP*R_Rear_WHL.errorPI + KI*R_Rear_WHL.integralPI) + 23.6241);
			 //R_Rear_WHL.signalPWM = 1525 + abs(KP*R_Rear_WHL.errorPI + KI*R_Rear_WHL.integralPI);
		 else if(speedRightWHL == 0){		// ??? check this condition
			 R_Rear_WHL.signalPWM = 1525;
			 R_Rear_WHL.integralPI = 0;
		 }
		 else
			 R_Rear_WHL.signalPWM = 1525 - round((240.0/8.94)*fabs(KP*R_Rear_WHL.errorPI + KI*R_Rear_WHL.integralPI) + 23.6241);;

		 if(R_Rear_WHL.signalPWM > 2000)
			 R_Rear_WHL.signalPWM = 1900;
		 if(R_Rear_WHL.signalPWM < 1000)
			 R_Rear_WHL.signalPWM = 1100;

		 velocityCalculationFlag = false;

		 variableFRONT = speedRightWHL - R_Front_WHL.wheelRadDivSec;
		 variableREAR = speedRightWHL - R_Rear_WHL.wheelRadDivSec;

		 //UART gönderilen datalar
		 /*
		  * linear_number (int32_t)
		  * angular_number (int32_t)
		  * encodercount (uint32_t)
		  * wheelRadDivSec (float,x100,uint16_t)
		  * toplam = 34 byte
		  */
		 //my_transmit();


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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  htim1.Init.Prescaler = 84-1;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 101-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 101-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 101-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 101-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
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
  htim9.Init.Prescaler = 2000-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 4200-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void my_transmit(void)
{
			 transmitbuffer[0]='G';
			 transmitbuffer[1]='G';
			 transmitbuffer[2]='T';
		 	 transmitbuffer[3]='U';

			 transmitbuffer[4]=(uint8_t)((linear_number>>24) & 0x000000FF);
			 transmitbuffer[5]=(uint8_t)((linear_number>>16) & 0x000000FF);
			 transmitbuffer[6]=(uint8_t)((linear_number>>8) & 0x000000FF);
			 transmitbuffer[7]=(uint8_t)(linear_number & 0x000000FF);

			 transmitbuffer[8]=(uint8_t)((angular_number>>24) & 0x000000FF);
			 transmitbuffer[9]=(uint8_t)((angular_number>>16) & 0x000000FF);
			 transmitbuffer[10]=(uint8_t)((angular_number>>8) & 0x000000FF);
			 transmitbuffer[11]=(uint8_t)(angular_number & 0x000000FF);

			 transmitbuffer[12]=(uint8_t)((L_Front_WHL.encoderCount>>24) & 0x000000FF);
			 transmitbuffer[13]=(uint8_t)((L_Front_WHL.encoderCount>>16) & 0x000000FF);
			 transmitbuffer[14]=(uint8_t)((L_Front_WHL.encoderCount>>8) & 0x000000FF);
			 transmitbuffer[15]=(uint8_t)(L_Front_WHL.encoderCount & 0x000000FF);

			 transmitbuffer[16]=(uint8_t)((L_Rear_WHL.encoderCount>>24) & 0x000000FF);
			 transmitbuffer[17]=(uint8_t)((L_Rear_WHL.encoderCount>>16) & 0x000000FF);
			 transmitbuffer[18]=(uint8_t)((L_Rear_WHL.encoderCount>>8) & 0x000000FF);
			 transmitbuffer[19]=(uint8_t)(L_Rear_WHL.encoderCount & 0x000000FF);

			 transmitbuffer[20]=(uint8_t)((R_Front_WHL.encoderCount>>24) & 0x000000FF);
			 transmitbuffer[21]=(uint8_t)((R_Front_WHL.encoderCount>>16) & 0x000000FF);
			 transmitbuffer[22]=(uint8_t)((R_Front_WHL.encoderCount>>8) & 0x000000FF);
			 transmitbuffer[23]=(uint8_t)(R_Front_WHL.encoderCount & 0x000000FF);

			 transmitbuffer[24]=(uint8_t)((R_Rear_WHL.encoderCount>>24) & 0x000000FF);
			 transmitbuffer[25]=(uint8_t)((R_Rear_WHL.encoderCount>>16) & 0x000000FF);
			 transmitbuffer[26]=(uint8_t)((R_Rear_WHL.encoderCount>>8) & 0x000000FF);
			 transmitbuffer[27]=(uint8_t)(R_Rear_WHL.encoderCount & 0x000000FF);

			 int16_t temp = 0;

			 temp = (int16_t)(L_Front_WHL.wheelRadDivSec * 100.0);
			 transmitbuffer[28]=(uint8_t)((temp>>8) & 0x00FF);
			 transmitbuffer[29]=(uint8_t)(temp & 0x00FF);

			 temp = (int16_t)(L_Rear_WHL.wheelRadDivSec * 100.0);
			 transmitbuffer[30]=(uint8_t)((temp>>8) & 0x00FF);
			 transmitbuffer[31]=(uint8_t)(temp & 0x00FF);

			 temp = (int16_t)(R_Front_WHL.wheelRadDivSec * 100.0);
			 transmitbuffer[32]=(uint8_t)((temp>>8) & 0x00FF);
			 transmitbuffer[33]=(uint8_t)(temp & 0x00FF);

			 temp = (int16_t)(R_Rear_WHL.wheelRadDivSec * 100.0);
			 transmitbuffer[34]=(uint8_t)((temp>>8) & 0x00FF);
			 transmitbuffer[35]=(uint8_t)(temp & 0x00FF);

			 transmitbuffer[36]=(uint8_t)((L_Front_WHL.signalPWM>>8) & 0x000000FF);
			 transmitbuffer[37]=(uint8_t)(L_Front_WHL.signalPWM & 0x000000FF);

			 transmitbuffer[38]=(uint8_t)((L_Rear_WHL.signalPWM>>8) & 0x000000FF);
			 transmitbuffer[39]=(uint8_t)(L_Rear_WHL.signalPWM & 0x000000FF);

			 transmitbuffer[40]=(uint8_t)((R_Front_WHL.signalPWM>>8) & 0x000000FF);
			 transmitbuffer[41]=(uint8_t)(R_Front_WHL.signalPWM & 0x000000FF);

			 transmitbuffer[42]=(uint8_t)((R_Rear_WHL.signalPWM>>8) & 0x000000FF);
			 transmitbuffer[43]=(uint8_t)(R_Rear_WHL.signalPWM & 0x000000FF);
//			 HAL_UART_Transmit(&huart6, 0x00, 1, 100);
//			 datax=2;
//			 HAL_UART_Transmit(&huart6, &datax, 1, 100);
//			 datax=15;
//			 HAL_UART_Transmit(&huart6, &datax, 1, 100);
//			 HAL_UART_Transmit(&huart2, &datax, 1, 100);
			 HAL_UART_Transmit(&huart6, transmitbuffer, 44, 100);
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
