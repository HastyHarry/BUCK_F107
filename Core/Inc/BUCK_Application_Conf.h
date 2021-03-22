
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

#define BUCK_PID_K_P					0.01
#define BUCK_PID_K_I					1
#define BUCK_PID_K_D					0.0001
#define BUCK_PID_W_F					2*PI*BUCK_SW_Frequency/10


#define G_VAC                           4.25//4.708                                     /*!< Gain terms of the AC voltage sensing */
#define B_VAC                           1975                                            /*!< Bias terms of the AC voltage sensing */
#define G_IAC                           32.5//42.67                                     /*!< Gain terms of the AC current sensing */
#define B_IAC                           1958                                            /*!< Bias terms of the AC current sensing */
#define G_VDC                           7.87//7.726                                     /*!< Gain terms of the DC voltage sensing */
#define B_VDC                           0                                               /*!< Bias terms of the DC voltage sensing */
#define G_IDC                           102.4                                           /*!< Gain terms of the DC current sensing */
#define B_IDC                           2048                                            /*!< Bias terms of the DC current sensing */

#endif //__BUCK_APPLICATION_CONF_H
