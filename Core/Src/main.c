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

#define LED_H2 GPIO_PIN_10
#define LED_H2_PORT GPIOE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
BUCK_PWM_Source_Struct BUCK_PWM_SRC;

float PID_Result;

uint32_t p_ADC1_Data[ADC1_CHs];                                 /*!< */

float Service_data[100];
uint32_t service_step;

ADC_Conf_TypeDef ADC_Conf;
Cooked_ADC_Struct VDC_ADC_IN_PHY;
PID_Control_Struct PID_CONF;
PID_Control_Struct PID_CONF_Burst;

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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_MspPostInit(&htim1);
  HAL_TIM_MspPostInit(&htim2);
  HAL_TIM_MspPostInit(&htim3);
  HAL_TIM_MspPostInit(&htim4);

  DPC_TO_Init();
  Buck_PID_Init(&PID_CONF, BUCK_PID_K_P,BUCK_PID_K_I,BUCK_PID_K_D, BUCK_SW_Frequency, BUCK_PID_W_F, BUCK_PID_SAT_UP, BUCK_PID_SAT_DOWN);

  Buck_PID_Init(&PID_CONF_Burst, BUCK_PID_K_P,BUCK_PID_K_I,BUCK_PID_K_D, BUCK_SW_Frequency, BUCK_PID_W_F, BUCK_PID_SAT_UP_BURST, BUCK_PID_SAT_DOWN_BURST);
//  Buck_Tim_PWM_Init(&BUCK_Tim1, BUCK_SW_Frequency);
//  Buck_Tim_PWM_Init(&BUCK_Tim4, BUCK_SW_Frequency);
//  Buck_Tim_Init(&BUCK_Tim2, BUCK_Math_Frequency);
//  Buck_Tim_Init(&BUCK_Tim3, BUCK_TO_Timer_Frequency);


  BUCK_ADC_Init(&ADC_Conf,G_VAC,B_VAC,G_IAC,B_IAC,G_VDC,B_VDC,G_IDC,B_IDC);

  HAL_TIM_PWM_Start_DMA(&BUCK_Tim1, BUCK_Tim1_PWM_CH, &BUCK_PWM_SRC.PWM_A, 1);
  HAL_TIM_PWM_Start_DMA(&BUCK_Tim4, BUCK_Tim4_PWM_CH, &BUCK_PWM_SRC.PWM_B, 1);
  HAL_ADC_Start_DMA(&BUCK_ADC1, p_ADC1_Data, ADC1_CHs);

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
		//HAL_GPIO_WritePin(LED_H2_PORT, LED_H2, GPIO_PIN_SET);
//		HAL_GPIO_TogglePin(LED_H2_PORT, LED_H2);
		DATA_Acquisition_from_DMA(p_ADC1_Data);

		ADC2Phy_VDC_ProcessData(&ADC_Conf,(uint32_t*)Read_Volt_DC(), &VDC_ADC_IN_PHY);
		VDC_ADC_IN_PHY.Vdc=10;

		if ((float)(BUCK_VDC_REF - VDC_ADC_IN_PHY.Vdc) > 20){
			PID_Result = Buck_Control(&PID_CONF_Burst,BUCK_VDC_REF, VDC_ADC_IN_PHY.Vdc);
		}
		else {
			PID_Result = Buck_Control(&PID_CONF,BUCK_VDC_REF, VDC_ADC_IN_PHY.Vdc);
		}

		if (VDC_ADC_IN_PHY.Vdc>=BUCK_VDC_OV){
			  HAL_TIM_PWM_Stop_DMA(&BUCK_Tim1, BUCK_Tim1_PWM_CH);
			  HAL_TIM_PWM_Stop_DMA(&BUCK_Tim4, BUCK_Tim4_PWM_CH);
		}
		else if (VDC_ADC_IN_PHY.Vdc <= BUCK_VDC_REF_HIGH_REF){
			  HAL_TIM_PWM_Start_DMA(&BUCK_Tim1, BUCK_Tim1_PWM_CH, &BUCK_PWM_SRC.PWM_A, 1);
			  HAL_TIM_PWM_Start_DMA(&BUCK_Tim4, BUCK_Tim4_PWM_CH, &BUCK_PWM_SRC.PWM_B, 1);
		}



		BUCK_PWM_Processing(PID_Result, &BUCK_Tim1, &BUCK_PWM_SRC);

	}
	else if (htim ->Instance == TIM3){
		TimeoutMng();
	}
}

//void ADC_DMAConvCplt(DMA_HandleTypeDef *hdma){
//
//
//
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
