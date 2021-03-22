/*
******************************************************************************
* @file           : Buck_Control.h
* @brief          : Control and init functions for Buck Converter
******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BUCK_CTRL_H
#define __BUCK_CTRL_H

//#include "stdint.h"
//#include "stdio.h"
#include "stm32f1xx_hal.h"

/* Exported types ------------------------------------------------------------*/
typedef struct {
float Kp;
float Ki;
float Kd;
float SW_Freq;
float Omega;
float Err_pr;
float Ui_pr;
float Ud_pr;
float Ui_previous;
float Ud_previous;
float Sat_Up;
float Sat_Down;
FlagStatus Init_Complete;
}PID_Control_Struct;

typedef struct{
  uint32_t Vdc;
  uint32_t Vac;
  uint32_t Idc;
}RAW_ADC_Struct;

typedef struct{
  uint32_t Vdc;
  uint32_t Vac;
  uint32_t Idc;
}Cooked_ADC_Struct;

typedef struct {
float G_Vac;
float invG_Vac;
float B_Vac;
float G_Iac;
float invG_Iac;
float B_Iac;
float G_Vdc;
float invG_Vdc;
float B_Vdc;
float G_Idc;
float invG_Idc;
float B_Idc;
FlagStatus ADC_Conf_Complete;
}ADC_Conf_TypeDef;

typedef struct{
	uint32_t PWM_A;
	uint32_t PWM_B;
}BUCK_PWM_Source_Struct;



void Buck_PID_Init(PID_Control_Struct* PID_CONFIG, float Kp, float Ki, float Kd, float Freq, float Omega, float Sat_Up, float Sat_Down );
float Buck_Control(PID_Control_Struct* PID_CONFIG, float Ref, float Feed);
float PID_Control(float Ref, float Feed, PID_Control_Struct* Conf_struct);
void Buck_Tim_Init(TIM_HandleTypeDef* BuckTIM, uint32_t  Freq_Desidered);
void Buck_Tim_PWM_Init(TIM_HandleTypeDef* BuckTIM, uint32_t  Freq_Desidered);
void DATA_Acquisition_from_DMA(uint32_t* p_ADC1_Data);
void ADC2Phy_VDC_ProcessData(ADC_Conf_TypeDef *ADC_Conf,uint32_t* p_Data_Sub, Cooked_ADC_Struct* Cooked_Values);
Cooked_ADC_Struct* Read_Volt_DC(void);
void BUCK_ADC_Init(ADC_Conf_TypeDef *BUCK_ADC_Conf,float G_Vac,float B_Vac,float G_Iac,float B_Iac,float G_Vdc,float B_Vdc,float G_Idc,float B_Idc);
void BUCK_PWM_Processing(float PWM_Value, TIM_HandleTypeDef *PWM_Tim, BUCK_PWM_Source_Struct* DMA_PWM_SOURCE);
//void Buck_PWM_Init(TIM_HandleTypeDef *BuckTIM, uint32_t BUCK_PWM_СH/*, uint32_t *PWM_DMA_Pntr, uint16_t Length*/);

#endif //__BUCK_CTRL_H
