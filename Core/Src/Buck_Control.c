/**
******************************************************************************
* @file           : Buck_Control.c
* @brief          : Control and init functions for Buck Converter
******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "Buck_Control.h"
#include "tim.h"
#include "BUCK_Application_Conf.h"

/* Private variables ---------------------------------------------------------*/

RAW_ADC_Struct Raw_ADC;



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


	BuckTIM->Init.Prescaler = Timers_PSC;
	BuckTIM->Init.CounterMode = TIM_COUNTERMODE_UP;
	BuckTIM->Init.Period = ((Timers_ClockPSCed/Freq_Desidered) - 1);
	BuckTIM->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	BuckTIM->Init.RepetitionCounter = 0;
	BuckTIM->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	HAL_TIM_Base_Init(BuckTIM);
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

	BuckTIM->Init.Prescaler = 0;
	BuckTIM->Init.CounterMode = TIM_COUNTERMODE_UP;
	BuckTIM->Init.Period = ((Timers_ClockPSCed/Freq_Desidered) - 1);
	BuckTIM->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	BuckTIM->Init.RepetitionCounter = 0;
	BuckTIM->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	HAL_TIM_Base_Init(BuckTIM);

//	HAL_TIM_PWM_Init(&BuckTIM);
}
/**
  * @brief  Buck_PWM_Init
  * @param  BuckTIM
  * @param  PWM_??H
  * @param  PWM_DMA_Pntr Pointer to Variable with actual sat values
  * @param  Length
  *
  * @retval None
  *
  * @note Function valid for STM32F1
  */
//void Buck_PWM_Init(TIM_HandleTypeDef *BuckTIM, uint32_t PWM_??H, uint32_t *PWM_DMA_Pntr, uint16_t Length){
//void Buck_PWM_Init(TIM_HandleTypeDef *BuckTIM, uint32_t BUCK_PWM_??H/*, uint32_t *PWM_DMA_Pntr, uint16_t Length*/){
//
//	//HAL_TIM_PWM_Start_DMA(BuckTIM, PWM_??H, PWM_DMA_Pntr, Length);
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

	if (Conf_struct->resetPI == SET){
		Conf_struct->Ui_previous = 0;
		Conf_struct->Ud_previous = 0;
		Conf_struct->Err_pr = 0;
		Conf_struct->resetPI = RESET;
	}

	Err = Ref - Feed;

	Up = Conf_struct->Kp * Err;
	Ui = (Conf_struct->Ui_previous * 2 * Conf_struct->SW_Freq + (Err + Conf_struct->Err_pr)*Conf_struct->Ki) /(2 * Conf_struct->SW_Freq);
	Ud = (Err - Conf_struct->Err_pr*Conf_struct->Kd * 2 * Conf_struct->SW_Freq * Conf_struct->Omega - Conf_struct->Ud_previous *(Conf_struct->Omega-2*Conf_struct->SW_Freq )) / (Conf_struct->Omega+2*Conf_struct->SW_Freq);

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

//	uint16_t MA_Period;


//	MA_Period=10;
//
	Raw_ADC.Vdc[Raw_ADC.MA_Counter] = p_ADC1_Data[1];
	Raw_ADC.Idc[Raw_ADC.MA_Counter] = p_ADC1_Data[2];
	Raw_ADC.Vac[Raw_ADC.MA_Counter] = p_ADC1_Data[0];
	Raw_ADC.MA_Counter++;
	if (Raw_ADC.MA_Counter>=ADC1_MA_PERIOD){
		Raw_ADC.MA_Counter=0;
	}
//	Value2 = 0;
//	for (i=0;i<ADC1_MA_PERIOD;i++){
//		//Value1 = Value1 + p_ADC1_Data[i*ADC1_CHs];
//		Value2 = Value2 + p_ADC1_Data[i*ADC1_CHs+1];
//		//Value3 = Value3 + p_ADC1_Data[i*ADC1_CHs+2];
//	}

	//Raw_ADC.Vac_MA = (float)(Value1/(float)(ADC1_MA_PERIOD));
	//Raw_ADC.Vdc_MA = (float)(Value2/(float)(ADC1_MA_PERIOD));
//	if (Raw_ADC.Vdc_MA - Raw_ADC.Vdc_MA_prev > 100 ){
//		Raw_ADC.Vdc_MA = Raw_ADC.Vdc_MA_prev + ((Raw_ADC.Vdc_MA - Raw_ADC.Vdc_MA_prev)*ADC_VAL_CHANGE_SPD_K);
//	}
//	else if ( Raw_ADC.Vdc_MA_prev - Raw_ADC.Vdc_MA > 100){
//		Raw_ADC.Vdc_MA = Raw_ADC.Vdc_MA + ((Raw_ADC.Vdc_MA_prev - Raw_ADC.Vdc_MA)*ADC_VAL_CHANGE_SPD_K);
//	}
	//Raw_ADC.Vdc_MA_prev = Raw_ADC.Vdc_MA;
	//Raw_ADC.Idc_MA = (float)(Value3/(float)(ADC1_MA_PERIOD));
}

void ADC_MA_VAL_Collection(){
	uint16_t i;
	float Value1 =0;
	float Value2 =0;
	float Value3 =0;

	for (i=0;i<ADC1_MA_PERIOD;i++){
		//Value1 = Value1 + Raw_ADC.Vac[i*ADC1_CHs];
		Value2 = Value2 + Raw_ADC.Vdc[i];
		//Value3 = Value3 + Raw_ADC.Idc[i*ADC1_CHs+2];
	}
	Raw_ADC.Vac_MA = (float)(Value1/(float)(ADC1_MA_PERIOD));
	Raw_ADC.Vdc_MA = (float)(Value2/(float)(ADC1_MA_PERIOD));
	Raw_ADC.Idc_MA = (float)(Value3/(float)(ADC1_MA_PERIOD));
}



/**
  * @brief  ADC2Phy_VDC_ProcessData
  * @param  ADC_Conf
  * @param  p_Data_Sub
  * @param  Cooked_Values
  * @retval Cooked_Values
  */
void ADC2Phy_VDC_ProcessData(ADC_Conf_TypeDef *ADC_Conf, RAW_ADC_Struct* p_Data_Sub, Cooked_ADC_Struct* Cooked_Values){

	float B_Vdc=ADC_Conf->B_Vdc;
	float G_Vdc=ADC_Conf->G_Vdc;
	float invG_Vdc=ADC_Conf->invG_Vdc;

	Cooked_Values->Vdc = ((float)((int16_t)p_Data_Sub->Vdc_MA-B_Vdc)*(float)(G_Vdc));

}

/**
  * @brief  Read_Volt_DC
  * @param  None
  * @retval Cooked_ADC_Struct
  */

RAW_ADC_Struct* Read_Volt_DC(void){
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

	if (PWM_Value<0.02) PWM_Value=0;

	Duty=(uint32_t)((float)PWM_Period * PWM_Value);
	DMA_PWM_SOURCE->PWM_A = Duty;
	DMA_PWM_SOURCE->PWM_B = Duty;
}

void ADC_Trigger_Init(uint32_t Pulse_Val){

	  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	  TIM_MasterConfigTypeDef sMasterConfig = {0};
	  TIM_OC_InitTypeDef sConfigOC = {0};
	  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};


	  sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
	  sConfigOC.Pulse = Pulse_Val;
	  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	  HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

}

float PID(float Ref, float Feed , PI_STRUCT_t *pPI)
{
pPI->Ref=Ref;
pPI->Feed=Feed;

  if(pPI->resetPI==SET)
  {
    pPI->Integral=0;
    pPI->resetPI = RESET;
  }
  else{
    pPI->error=(float)Ref-(float)Feed;
    pPI->Integral=pPI->Integral+(pPI->k1*pPI->error)+pPI->Antiwindup_Term;
    pPI->Integralout=pPI->Integral;
    pPI->PIout=(pPI->k0*pPI->error)+pPI->Integralout;
  }

  //Start Check Saturation
  if (pPI->satPI_toggle==SET){
    //Saturation
    if(    pPI->PIout>pPI->PIsat_up)
    {
      pPI->PIout_sat=pPI->PIsat_up;
    }
    else if(    pPI->PIout<pPI->PIsat_down)
    {
      pPI->PIout_sat=pPI->PIsat_down;
    }
    else {
      pPI->PIout_sat=pPI->PIout;
    }

    //Start Check Antiwindup
    if (pPI->antiwindPI_toggle==SET){
      //Saturation
      pPI->Antiwindup_Term=(pPI->PIout_sat-pPI->PIout)*pPI->Antiwindup_Gain;
    }
    else {
      pPI->Antiwindup_Term=0;
    }
    //End Check Antiwindup
  }
  else {
    pPI->PIout_sat=pPI->PIout;
    pPI->Antiwindup_Term=0;
  }
  //End Check Saturation

  return pPI->PIout_sat;
}

void DPC_PID_Init(PI_STRUCT_t *pPI,float Init_Val_Kp,float Init_Val_Ki,float Init_Val_Ts,float Init_PIsat_up, float Init_PIsat_down,FlagStatus satPI_toggle_local,FlagStatus antiwindPI_toggle_local,float Antiwindup_Gain_local)
{
  pPI->Kp=Init_Val_Kp;
  pPI->Ki=Init_Val_Ki;
  pPI->Ts=Init_Val_Ts;
  pPI->Integral=0;
  pPI->PIout=0;
  pPI->PIsat_up=Init_PIsat_up;
  pPI->PIsat_down=Init_PIsat_down;
  pPI->error=0;
  pPI->Integralout=0;
  pPI->resetPI=RESET;
  pPI->k0=Init_Val_Kp; //K0=Kp
  pPI->k1=Init_Val_Ki*Init_Val_Ts; //K1=Ki*Ts
  pPI->satPI_toggle=satPI_toggle_local;
  pPI->antiwindPI_toggle=antiwindPI_toggle_local;
  pPI->Antiwindup_Gain=Antiwindup_Gain_local;
}

void PID_RESET(PI_STRUCT_t *pPI)
{
  pPI->Integral=0;
}

