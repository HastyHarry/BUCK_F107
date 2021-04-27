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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BUCK_Application_Conf.h"
#include "DPC_Timeout.h"
#include "Buck_Control.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LoWord(param) ((unsigned *)&param)[0]
#define HiWord(param) ((unsigned *)&param)[1]

#define Lowest(param) ((uint8_t *)&param)[0]
#define Lo(param) 	  ((uint8_t *)&param)[1]
#define Hi(param) 	  ((uint8_t *)&param)[2]
#define Highest(param)((uint8_t *)&param)[3]

#define LED_H2 GPIO_PIN_10
#define LED_H2_PORT GPIOE

#define LED_VD3 GPIO_PIN_2
#define LED_VD3_PORT GPIOE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
BUCK_PWM_Source_Struct BUCK_PWM_SRC;
BUCK_OC_Source_Struct BUCK_OC_SRC;

PI_STRUCT_t pPI_VDC_CTRL;
PI_STRUCT_t pPI_VDC_CTRL_BURST;

CAN_Messages_Struct CAN_Messages;
uint8_t               TxData[8];
uint8_t               RxData[8];
uint32_t              TxMailbox;

TO_RET_STATE TO_State;
uint16_t StartUp;
volatile FlagStatus Calc_Start;

volatile float Service_Data[10][500];
uint32_t Service_step;
uint32_t Service_step2;
uint16_t j;
FlagStatus DMA_FLAG;
uint32_t ADC_Ch_Selected;


float PID_Result;
float PID_Result_V;
float PID_Result_I;
float PWM;
uint16_t Duty_To_Send;
uint16_t Tim_Counter;

uint32_t p_ADC1_Data[ADC1_CHs];                                 /*!< */

ADC_Conf_TypeDef ADC_Conf;
Cooked_ADC_Struct VDC_ADC_IN_PHY;



PID_Control_Struct PID_CONF_Burst;
PID_Control_Struct PID_CONF_StartUp;

//Option 1
PID_Control_Struct Current_PID_CONF;
PID_Control_Struct Voltage_PID_CONF;

//Option 2
PID_Control_Struct UDC_LIMIT_PID;
PID_Control_Struct IDC_LIMIT_PID;

extern RAW_ADC_Struct Raw_ADC;
extern RAW_ADC_Struct Raw_DMA;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ADC_Config_Ch( uint32_t Channel);
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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_Delay(500);
//  HAL_TIM_MspPostInit(&htim1);
//  HAL_TIM_MspPostInit(&htim2);
//  HAL_TIM_MspPostInit(&htim4);

  DPC_TO_Init();

  //Buck_PID_Init(&PID_CONF_StartUp, BUCK_PID_K_P,BUCK_PID_K_I,BUCK_PID_K_D, BUCK_Math_Frequency, BUCK_PID_W_F, BUCK_PID_SAT_UP_BURST, BUCK_PID_SAT_DOWN_BURST, BUCK_PID_HIST, BUCK_PID_BASE_VAL, BUCK_INT_SAT_UP, BUCK_INT_SAT_DOWN, BUCK_PID_ANTIWINDUP);

  Buck_PID_Init(&Voltage_PID_CONF, V_PID_K_P,V_PID_K_I,V_PID_K_D, BUCK_Math_Frequency, BUCK_PID_W_F, V_PID_SAT_UP, V_PID_SAT_DOWN, V_PID_HIST, V_PID_RESOLUTION,V_PID_INT_SAT_UP,V_PID_INT_SAT_DOWN, V_PID_ANTIWINDUP);
  Buck_PID_Init(&Current_PID_CONF, I_PID_K_P,I_PID_K_I,I_PID_K_D, BUCK_Math_Frequency, BUCK_PID_W_F, I_PID_SAT_UP, I_PID_SAT_DOWN, I_PID_HIST, I_PID_RESOLUTION, I_PID_INT_SAT_UP, I_PID_INT_SAT_DOWN, I_PID_ANTIWINDUP);

  Buck_PID_Init(&UDC_LIMIT_PID, V_LIM_PID_K_P,V_LIM_PID_K_I,V_LIM_PID_K_D, BUCK_Math_Frequency, V_LIM_PID_W_F, V_LIM_PID_SAT_UP, V_LIM_PID_SAT_DOWN, V_LIM_PID_HIST, V_LIM_PID_RESOLUTION, V_LIM_INT_SAT_UP, V_LIM_INT_SAT_DOWN, V_LIM_PID_ANTIWINDUP);
  Buck_PID_Init(&IDC_LIMIT_PID, I_LIM_PID_K_P,I_LIM_PID_K_I,I_LIM_PID_K_D, BUCK_Math_Frequency, I_LIM_PID_W_F, I_LIM_PID_SAT_UP, I_LIM_PID_SAT_DOWN, I_LIM_PID_HIST, I_LIM_PID_RESOLUTION, I_LIM_INT_SAT_UP, I_LIM_INT_SAT_DOWN, I_LIM_PID_ANTIWINDUP);

  Buck_PID_Init(&PID_CONF_StartUp, STUP_PID_K_P,STUP_PID_K_I,STUP_PID_K_D, BUCK_Math_Frequency, STUP_PID_W_F, STUP_PID_SAT_UP, STUP_PID_SAT_DOWN, STUP_PID_HIST, STUP_PID_RESOLUTION, STUP_INT_SAT_UP, STUP_INT_SAT_DOWN, STUP_PID_ANTIWINDUP);

  DPC_PID_Init(&pPI_VDC_CTRL,DPC_VCTRL_KP,DPC_VCTRL_KI,DPC_PI_VDC_TS,DPC_VCTRL_PI_sat_up,DPC_VCTRL_PI_sat_down,DPC_VCTRL_PI_SAT_EN,DPC_VCTRL_PI_AW_EN,DPC_VCTRL_PI_AWTG);
  DPC_PID_Init(&pPI_VDC_CTRL_BURST,DPC_VCTRL_KP,DPC_VCTRL_KI,DPC_PI_VDC_TS,DPC_VCTRL_BURST_PI_sat_up,DPC_VCTRL_PI_sat_down,DPC_VCTRL_PI_SAT_EN,DPC_VCTRL_PI_AW_EN,DPC_VCTRL_PI_AWTG);

  Buck_Tim_PWM_Init(&BUCK_Tim1, BUCK_SW_Frequency);
  Buck_Tim_PWM_Init(&BUCK_Tim4, BUCK_SW_Frequency);
  Buck_Tim_Init(&BUCK_Tim2, BUCK_Math_Frequency);
  Buck_Tim_Init(&BUCK_Tim5, BUCK_TO_Timer_Frequency);

  BUCK_ADC_Init(&ADC_Conf,G_VAC,B_VAC,G_IAC,B_IAC,G_VDC,B_VDC,G_IDC,B_IDC);
  //CAN_Start_Setup();
  HAL_TIMEx_PWMN_Start_DMA(&BUCK_Tim1, BUCK_Tim1_PWM_CH, &BUCK_PWM_SRC.PWM_A, 1);

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
  ADC_Ch_Selected=1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(Calc_Start==SET){
		  DATA_Processing();
		  ADC_MA_VAL_Collection();
		  ADC2Phy_VDC_ProcessData(&ADC_Conf,(RAW_ADC_Struct*)Read_Volt_DC(), &VDC_ADC_IN_PHY);
		  ADC2Phy_IDC_ProcessData(&ADC_Conf,(RAW_ADC_Struct*)Read_Volt_DC(), &VDC_ADC_IN_PHY);

		  	if (((float)VDC_ADC_IN_PHY.Vdc) > BUCK_VDC_REF_LOW_REF){
		  		StartUp=2;
		  	}
		  	else if (((float)VDC_ADC_IN_PHY.Vdc) < 20){
		  		StartUp=0;
		  		PID_CONF_StartUp.resetPI=SET;
		  	}

			if (StartUp==0){
				Voltage_PID_CONF.resetPI = SET;
				Current_PID_CONF.resetPI = SET;
				UDC_LIMIT_PID.resetPI = SET;
				IDC_LIMIT_PID.resetPI = SET;
				PID_Result_V = PID_Control(BUCK_VDC_REF, VDC_ADC_IN_PHY.Vdc, &PID_CONF_StartUp);
				PWM = PID_Result_V/BUCK_VAC_REF;
				IDC_LIMIT_PID.resetPI = SET;
				UDC_LIMIT_PID.resetPI = SET;
			}
			else if (StartUp==2){
				Voltage_PID_CONF.Antiwindup_Switch = SET;
				Current_PID_CONF.Antiwindup_Switch = RESET;
				//PID_Result = Buck_Control(&Voltage_PID_CONF,&Current_PID_CONF,BUCK_VDC_REF, (float)(VDC_ADC_IN_PHY.Vdc/* - PID_Result*/), VDC_ADC_IN_PHY.Idc);
				PID_Result_V = PID_Control(BUCK_VDC_REF, VDC_ADC_IN_PHY.Vdc, &UDC_LIMIT_PID);
				PID_Result_I = Current_Control(BUCK_IDC_LIM, VDC_ADC_IN_PHY.Idc, &IDC_LIMIT_PID);
				//PID_Result_I = BUCK_VAC_REF;

				if (PID_Result_V<=PID_Result_I){
					PWM = PID_Result_V/BUCK_VAC_REF;

				}
				else if (PID_Result_V>PID_Result_I){
					PWM = PID_Result_I/BUCK_VAC_REF;
					//UDC_LIMIT_PID.resetPI = SET;
				}

				//PWM = PID_Result_I/BUCK_VAC_REF;
				PID_CONF_StartUp.resetPI = SET;

				//PWM=0.7;
			}

			BUCK_PWM_Processing(PWM, &BUCK_Tim1, &BUCK_PWM_SRC);

			if (VDC_ADC_IN_PHY.Vdc>=BUCK_VDC_OV){
				HAL_TIMEx_PWMN_Stop_DMA(&BUCK_Tim1, BUCK_Tim1_PWM_CH);
			}
			else if (VDC_ADC_IN_PHY.Vdc <= BUCK_VDC_REF_LOW_REF){
				HAL_TIMEx_PWMN_Start_DMA(&BUCK_Tim1, BUCK_Tim1_PWM_CH, &BUCK_PWM_SRC.PWM_A, 1);
			}


			Service_Data[0][Service_step] = (float)VDC_ADC_IN_PHY.Vdc;
			Service_Data[1][Service_step] = (float)VDC_ADC_IN_PHY.Idc;
			Service_Data[2][Service_step] = (float)(PID_Result_V);
			Service_Data[3][Service_step] = (float)(PID_Result_I);
	  		Service_Data[4][Service_step] = (float)(PID_Result_V);

//	  		Service_Data[4][Service_step] = (float)(PID_CONF.Ud_previous*100);
			//Service_Data[1][Service_step] = (float)VDC_ADC_IN_PHY.Idc;

			if (Service_step==499){
	  //			HAL_TIM_PWM_Stop_DMA(&BUCK_Tim4, BUCK_Tim4_PWM_CH);
				Service_step=0;
				Service_step2--;

			}
			else {
					Service_step++;
		//			HAL_TIMEx_PWMN_Start_DMA(&BUCK_Tim1, BUCK_Tim1_PWM_CH, &BUCK_PWM_SRC.PWM_A, 1);
					}
			Calc_Start = RESET;
			//HAL_ADC_Start_DMA(&BUCK_ADC1, p_ADC1_Data, ADC1_MA_PERIOD_RAW*ADC1_CHs);
	  }
	  //HAL_ADC_Start_DMA(&BUCK_ADC1, p_ADC1_Data, ADC1_CHs);




//	TO_State=DPC_TO_Check(RELAY_TO_CH);
//	if (TO_State==TO_OUT_TOOK){
//	  HAL_GPIO_TogglePin(LED_H2_PORT, LED_H2);
//	}
//	else if (TO_State==TO_OUT_OK){
//
//	}
//	else{
//	  DPC_TO_Set(RELAY_TO_CH, RELAY_TIMEOUT);
//	}
//
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL2_ON;
  RCC_OscInitStruct.PLL2.PLL2MUL = RCC_PLL2_MUL8;
  RCC_OscInitStruct.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV5;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/* USER CODE BEGIN 4 */



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim ->Instance == TIM2){

		Calc_Start = SET;
		DMA_FLAG = SET;
//		HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
		//HAL_ADC_Start_IT(&BUCK_ADC1);
	}
	else if (htim ->Instance == TIM5){
		TimeoutMng();

//		TO_State=DPC_TO_Check(1);
//		if (TO_State==TO_OUT_TOOK){
//			TxHeader.StdId = 0x321;
//			TxHeader.ExtId = 0x01;
//			TxHeader.RTR = CAN_RTR_DATA;
//			TxHeader.IDE = CAN_ID_STD;
//			TxHeader.DLC = 8;
//			TxHeader.TransmitGlobalTime = DISABLE;
//			Duty_To_Send = (uint16_t)(PID_Result*100);
//
//			TxData[0] = Lo(Duty_To_Send);
//			TxData[1] = Lowest(Duty_To_Send);
//
//			TxData[2] = Lo(VDC_ADC_IN_PHY.Vdc);
//			TxData[3] = Lowest(VDC_ADC_IN_PHY.Vdc);
//
//			TxMailbox = 1;
//			if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
//			{
//				Error_Handler();
//			}
//			DPC_TO_Set(1, 100);
//
//		}
//		else if (TO_State==TO_OUT_OK){
//
//		}
//		else{
//			DPC_TO_Set(1, 100);
//		}
	}
	else if (htim ->Instance == TIM1){
		HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
	}

}

//void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* CanHandle){
//
//	if(CanHandle->Instance == CAN1){
//
//	}
//}

//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
//{
//
//  if ((RxHeader.StdId == 0x321) && (RxHeader.IDE == CAN_ID_STD) && (RxHeader.DLC == 8))
//  {
//
//  }
//}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	HAL_ADC_Stop_DMA(&BUCK_ADC1);
	//HAL_ADC_Stop_IT(&BUCK_ADC1);
	//HAL_GPIO_TogglePin(LED_VD3_PORT, LED_VD3);
	HAL_GPIO_WritePin(LED_VD3_PORT, LED_VD3, 0);
	DATA_Acquisition_from_DMA(p_ADC1_Data);
//	if (ADC_Ch_Selected==1){
//		//HAL_GPIO_WritePin(LED_VD3_PORT, LED_VD3, 0);
//		p_ADC1_Data[ADC_Ch_Selected] = HAL_ADC_GetValue(&BUCK_ADC1);
//		ADC_Config_Ch(ADC_CHANNEL_2);
//		ADC_Ch_Selected = 2;
//		HAL_ADC_Start_IT(&BUCK_ADC1);
//	}
//	else if (ADC_Ch_Selected==2) {
//		//HAL_GPIO_WritePin(LED_VD3_PORT, LED_VD3, 1);
//		p_ADC1_Data[ADC_Ch_Selected] = HAL_ADC_GetValue(&BUCK_ADC1);
//		ADC_Config_Ch(ADC_CHANNEL_3);
//		ADC_Ch_Selected = 3;
//		HAL_ADC_Start_IT(&BUCK_ADC1);
//	}
//	else if (ADC_Ch_Selected==3) {
//		ADC_Ch_Selected = 1;
//		HAL_GPIO_WritePin(LED_VD3_PORT, LED_VD3, 0);
//		p_ADC1_Data[ADC_Ch_Selected] = HAL_ADC_GetValue(&BUCK_ADC1);
//	}


	//HAL_ADC_Start_DMA(&BUCK_ADC1, p_ADC1_Data, ADC1_MA_PERIOD_RAW*ADC1_CHs);
	//HAL_ADC_Stop_DMA(&BUCK_ADC1);
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
		HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
		//HAL_GPIO_TogglePin(LED_VD3_PORT, LED_VD3);
		HAL_GPIO_WritePin(LED_VD3_PORT, LED_VD3, 1);
		HAL_ADC_Start_DMA(&BUCK_ADC1, p_ADC1_Data, ADC1_MA_PERIOD_RAW*ADC1_CHs);
	}
}

void ADC_Config_Ch( uint32_t Channel){

	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = Channel;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}
}

//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
//	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
//		HAL_GPIO_TogglePin(LED_VD3_PORT, LED_VD3);
//	}
//}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  //__disable_irq();
  HAL_GPIO_WritePin(LED_H2_PORT, LED_H2,1);
  HAL_TIMEx_PWMN_Stop_DMA(&BUCK_Tim1, BUCK_Tim1_PWM_CH);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
