/**
******************************************************************************
* @file           : Buck_Control.c
* @brief          : Control and init functions for Buck Converter
******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "Buck_Control.h"
#include "tim.h"

/* Private variables ---------------------------------------------------------*/

RAW_ADC_Struct Raw_ADC;
RAW_ADC_Struct Cooked_ADC;

/* Private typedef -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  Buck_Tim_Init
  * @param  BuckTIM
  * @param  Freq_Desidered
  *
  * @retval None
  *
  * @note Function valid for STM32F1
  */
void Buck_Tim_Init(TIM_HandleTypeDef* BuckTIM, uint32_t  Freq_Desidered){
	uint32_t Timers_Clock;                                                                ///
	uint32_t Timers_PSC;                                                                  ///
	uint32_t Timers_ClockPSCed;                                                           ///

	Timers_PSC=(uint32_t)(READ_REG(BuckTIM->Instance->PSC));                                ///
	Timers_Clock=HAL_RCC_GetPCLK2Freq();                                                  ///

	Timers_ClockPSCed=(Timers_Clock/(Timers_PSC+1));                                      ///

	BuckTIM->Instance = TIM1;
	BuckTIM->Init.Prescaler = 0;
	BuckTIM->Init.CounterMode = TIM_COUNTERMODE_UP;
	BuckTIM->Init.Period = ((Timers_ClockPSCed/Freq_Desidered) - 1);
	BuckTIM->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	BuckTIM->Init.RepetitionCounter = 0;
	BuckTIM->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

//	HAL_TIM_Base_Init(&BuckTIM);
	//HAL_TIM_Base_Start_IT(&BuckTIM);

}

/**
  * @brief  Buck_Tim_PWM_Init
  * @param  BuckTIM
  * @param  Freq_Desidered
  *
  * @retval None
  *
  * @note Function valid for STM32F1
  */
void Buck_Tim_PWM_Init(TIM_HandleTypeDef* BuckTIM, uint32_t  Freq_Desidered){
	uint32_t Timers_Clock;                                                                ///
	uint32_t Timers_PSC;                                                                  ///
	uint32_t Timers_ClockPSCed;                                                           ///

	Timers_PSC=(uint32_t)(READ_REG(BuckTIM->Instance->PSC));                                ///
	Timers_Clock=HAL_RCC_GetPCLK2Freq();                                                  ///

	Timers_ClockPSCed=(Timers_Clock/(Timers_PSC+1));                                      ///

	BuckTIM->Instance = TIM1;
	BuckTIM->Init.Prescaler = 0;
	BuckTIM->Init.CounterMode = TIM_COUNTERMODE_UP;
	BuckTIM->Init.Period = ((Timers_ClockPSCed/Freq_Desidered) - 1);
	BuckTIM->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	BuckTIM->Init.RepetitionCounter = 0;
	BuckTIM->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

//	HAL_TIM_PWM_Init(&BuckTIM);
}
/**
  * @brief  Buck_PWM_Init
  * @param  BuckTIM
  * @param  PWM_СH
  * @param  PWM_DMA_Pntr Pointer to Variable with actual sat values
  * @param  Length
  *
  * @retval None
  *
  * @note Function valid for STM32F1
  */
//void Buck_PWM_Init(TIM_HandleTypeDef *BuckTIM, uint32_t PWM_СH, uint32_t *PWM_DMA_Pntr, uint16_t Length){
//void Buck_PWM_Init(TIM_HandleTypeDef *BuckTIM, uint32_t BUCK_PWM_СH/*, uint32_t *PWM_DMA_Pntr, uint16_t Length*/){
//
//	//HAL_TIM_PWM_Start_DMA(BuckTIM, PWM_СH, PWM_DMA_Pntr, Length);
//}

/**
  * @brief  Buck_PID_Init
  * @param  Kp
  * @param  Ki
  * @param  Kd
  * @param  Freq
  * @param  Omega
  * @param  Sat

  * @retval Res PID Output
  */
void Buck_PID_Init(PID_Control_Struct* PID_CONFIG, float Kp, float Ki, float Kd, float Freq, float Omega, float Sat_Up, float Sat_Down ){


	PID_CONFIG->SW_Freq = Freq;
	PID_CONFIG->Omega = Omega;
	PID_CONFIG->Kp = Kp;
	PID_CONFIG->Ki = Ki;
	PID_CONFIG->Kd = Kd;
	PID_CONFIG->Sat_Up = Sat_Up;
	PID_CONFIG->Sat_Down = Sat_Down;
	PID_CONFIG->Init_Complete = SET;

}


/**
  * @brief  Buck_Control
  * @param  Ref
  * @param  Feed
  * @retval Res PID Output
  */
float Buck_Control(PID_Control_Struct* PID_CONFIG, float Ref, float Feed){
	float Res;
	if (PID_CONFIG->Init_Complete!=SET){
		Res = 0;
	}
	else {
		Res = PID_Control(Ref, Feed, PID_CONFIG);
	}
	return Res;
}

/**
  * @brief  PID_Control
  * @param  Ref
  * @param  Feed
  * @param  Conf_struct
  * @retval Result
  */
float PID_Control(float Ref, float Feed, PID_Control_Struct* Conf_struct){

	float Err;
//	float SW_Freq;
	float Result;

	float Up;
	float Ui;
	float Ud;

//	Err_Prev = Conf_struct->Err_pr;
//	Ui_prev = Conf_struct->Ui_previous;
//	Ud_prev = Conf_struct->Ud_previous;

	Err = Ref - Feed;

	Up = Conf_struct->Kp * Err;
	Ui = (Conf_struct->Ui_previous * 2 * Conf_struct->SW_Freq + (Err + Conf_struct->Err_pr)*Conf_struct->Ki) /(2 * Conf_struct->SW_Freq);
	Ud = ((Err - Conf_struct->Err_pr)*Conf_struct->Kd * 2 * Conf_struct->SW_Freq * Conf_struct->Omega - Conf_struct->Ud_previous *(Conf_struct->Omega-2*Conf_struct->SW_Freq )) / (Conf_struct->Omega+2*Conf_struct->SW_Freq);

	Result = Up+Ui+Ud;

	if (Result>=Conf_struct->Sat_Up){
		Result = Conf_struct->Sat_Up;
	}
	else if (Result<=Conf_struct->Sat_Down){
		Result = Conf_struct->Sat_Down;
	}

	Conf_struct->Err_pr = Err;
	Conf_struct->Ui_previous = Ui;
	Conf_struct->Ud_previous = Ud;

	return Result;

}

/**
  * @brief  DATA_Acquisition_from_DMA
  * @param  p_ADC1_Data

  * @retval None
  */

void DATA_Acquisition_from_DMA(uint32_t* p_ADC1_Data) {
	Raw_ADC.Vdc = p_ADC1_Data[0];
	Raw_ADC.Idc = p_ADC1_Data[1];
}

/**
  * @brief  ADC2Phy_VDC_ProcessData
  * @param  ADC_Conf
  * @param  p_Data_Sub
  * @param  Cooked_Values
  * @retval Cooked_Values
  */
void ADC2Phy_VDC_ProcessData(ADC_Conf_TypeDef *ADC_Conf,uint32_t* p_Data_Sub, Cooked_ADC_Struct* Cooked_Values){

	float B_Vdc=ADC_Conf->B_Vdc;
	float G_Vdc=ADC_Conf->G_Vdc;
	float invG_Vdc=ADC_Conf->invG_Vdc;

	Cooked_Values->Vdc = ((float)((int16_t)p_Data_Sub[0]-B_Vdc)*(float)(invG_Vdc));

}

/**
  * @brief  Read_Volt_DC
  * @param  None
  * @retval Cooked_ADC_Struct
  */

Cooked_ADC_Struct* Read_Volt_DC(void){
  return &Raw_ADC;
}

/**
  * @brief  BUCK_ADC_Init
  * @param  ADC_Conf_TypeDef
  * @retval None
  */

void BUCK_ADC_Init(ADC_Conf_TypeDef *BUCK_ADC_Conf,float G_Vac,float B_Vac,float G_Iac,float B_Iac,float G_Vdc,float B_Vdc,float G_Idc,float B_Idc){

	BUCK_ADC_Conf->B_Vac=B_Vac;
	BUCK_ADC_Conf->G_Vac=G_Vac;
	BUCK_ADC_Conf->invG_Vac=(float)(1.0/G_Vac);

	BUCK_ADC_Conf->B_Vdc=B_Vdc;
	BUCK_ADC_Conf->G_Vdc=G_Vdc;
	BUCK_ADC_Conf->invG_Vdc=(float)(1.0/G_Vdc);

	BUCK_ADC_Conf->B_Iac=B_Iac;
	BUCK_ADC_Conf->G_Iac=G_Iac;
	BUCK_ADC_Conf->invG_Iac=(float)(1.0/G_Iac);

	BUCK_ADC_Conf->B_Idc=B_Idc;
	BUCK_ADC_Conf->G_Idc=G_Idc;
	BUCK_ADC_Conf->invG_Idc=(float)(1.0/G_Idc);


	BUCK_ADC_Conf->ADC_Conf_Complete=SET;

}

/**
  * @brief  ADC2Phy_VDC_ProcessData
  * @param  ADC_Conf
  * @param  p_Data_Sub
  * @param  Cooked_Values
  * @retval Cooked_Values
  */
void BUCK_PWM_Processing(float PWM_Value, TIM_HandleTypeDef *PWM_Tim, BUCK_PWM_Source_Struct* DMA_PWM_SOURCE){
	uint16_t PWM_Period;
	uint32_t Duty;
	PWM_Period = PWM_Tim->Init.Period;

	if (PWM_Value>1) PWM_Value=1;
	else if (PWM_Value<0) PWM_Value=0;

	Duty=(uint32_t)(PWM_Period * PWM_Value);
	DMA_PWM_SOURCE->PWM_A = Duty;
	DMA_PWM_SOURCE->PWM_B = Duty;
}
