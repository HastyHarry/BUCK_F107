
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BUCK_APPLICATION_CONF_H
#define __BUCK_APPLICATION_CONF_H

#define PI 				 3.14
#define SQRT_2           1.41421                                                        /*!< Square Root of number 2 */ 
#define SQRT_3           1.73205                                                        /*!< Square Root of number 3 */ 

#define BUCK_Tim1                       htim1
#define BUCK_Tim2                       htim2
#define BUCK_Tim3                       htim3
#define BUCK_Tim4                       htim4

#define BUCK_SW_Frequency				10000											/*!<Switching Freq*/
#define BUCK_Math_Frequency				10000											/*!<Calculations Freq*/
#define BUCK_TO_Timer_Frequency			1000											/*!<TO Freq*/

#define BUCK_Tim1_PWM_CH 				TIM_CHANNEL_3
#define BUCK_Tim4_PWM_CH 				TIM_CHANNEL_2

#define ADC1_CHs						2
#define BUCK_ADC1 						hadc1

#define BUCK_VDC_REF					400

#define BUCK_PID_K_P					1
#define BUCK_PID_K_I					0.01
#define BUCK_PID_K_D					0.001
#define BUCK_PID_W_F					2*PI*BUCK_SW_Frequency

#endif //__BUCK_APPLICATION_CONF_H
