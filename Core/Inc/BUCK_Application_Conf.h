
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BUCK_APPLICATION_CONF_H
#define __BUCK_APPLICATION_CONF_H

#define PI		 		 3.14
#define SQRT_2           1.41421                                                        /*!< Square Root of number 2 */ 
#define SQRT_3           1.73205                                                        /*!< Square Root of number 3 */ 

#define BUCK_Tim1                       htim1
#define BUCK_Tim2                       htim2
#define BUCK_Tim3                       htim3
#define BUCK_Tim4                       htim4
#define BUCK_Tim5                       htim5
#define BUCK_Tim1_OC_CH					TIM_CHANNEL_1
#define BUCK_Tim4_OC_CH					TIM_CHANNEL_4
#define BUCK_Tim5_OC_CH					TIM_CHANNEL_1									/*!<Not in USE>*/

#define BUCK_SW_Frequency				10000											/*!<Switching Freq*/
#define BUCK_Math_Frequency				10000											/*!<Calculations Freq*/
#define BUCK_TO_Timer_Frequency			1000											/*!<TO Freq*/

#define BUCK_Tim1_PWM_CH 				TIM_CHANNEL_3
#define BUCK_Tim4_PWM_CH 				TIM_CHANNEL_2

#define ADC1_CHs						3
#define ADC1_MA_PERIOD					100
#define BUCK_ADC1 						hadc1
#define ADC_VAL_CHANGE_SPD_K			0.5


#define BUCK_VDC_REF					100
#define BUCK_VDC_OV						150
#define BUCK_VDC_HIST					10												/*!< value in %*/
#define BUCK_VDC_REF_LOW_REF			BUCK_VDC_REF-(BUCK_VDC_REF*BUCK_VDC_HIST/100)
#define BUCK_VDC_REF_HIGH_REF			BUCK_VDC_REF+(BUCK_VDC_REF*BUCK_VDC_HIST/100)

#define BUCK_PID_K_P					0.6
#define BUCK_PID_K_I					0.5
#define BUCK_PID_K_D					0
#define BUCK_PID_W_F					2*PI*BUCK_SW_Frequency/10
#define BUCK_PID_SAT_UP					0.9
#define BUCK_PID_SAT_DOWN				0.0

#define BUCK_PID_SAT_UP_BURST			0.1
#define BUCK_PID_SAT_DOWN_BURST			0.0

#define G_VAC                           4.25//4.708                                     /*!< Gain terms of the AC voltage sensing */
#define B_VAC                           1975                                            /*!< Bias terms of the AC voltage sensing */
#define G_IAC                           32.5//42.67                                     /*!< Gain terms of the AC current sensing */
#define B_IAC                           1958                                            /*!< Bias terms of the AC current sensing */
#define G_VDC                           0.185//8.4//7.726                                     /*!< Gain terms of the DC voltage sensing */
#define B_VDC                           0                                               /*!< Bias terms of the DC voltage sensing */
#define G_IDC                           102.4                                           /*!< Gain terms of the DC current sensing */
#define B_IDC                           2048                                            /*!< Bias terms of the DC current sensing */


///DPC PID
#define DPC_VCTRL_KP                    4E-4                                            /*!< VCTRL - Proportional gain of the PI regulator related to DC voltage control*/
#define DPC_VCTRL_KI                    0.3                                             /*!< VCTRL - Integral gain of the PI regulator related to DC voltage control*/
#define DPC_PFC_VDC                     100                                				/*!< VCTRL - DC Voltage referance value of the PFC [Expresed in Volt]*/
#define DPC_PFC_Iref_sat                10//22                                          /*!< VCTRL - d-q axis AC Current referance limit of the PFC [Expresed in AMPs]*/
#define DPC_VCTRL_PI_AWTG               0.02                                            /*!< VCTRL - Anti Wind-up GAIN*/
#define DPC_VCTRL_PI_sat_up             0.9          									/*!< VCTRL - Higher Current Referance Saturation LIMIT*/
#define DPC_VCTRL_PI_sat_down           0                                               /*!< VCTRL - Lower Current Referance Saturation LIMIT*/
#define DPC_VCTRL_PI_SAT_EN             SET                                             /*!< VCTRL - Current Referance Saturation Enable*/
#define DPC_VCTRL_PI_AW_EN              SET                                             /*!< VCTRL - Anti Wind-up Enable*/
#define DPC_PI_VDC_TS                   ((float)1/BUCK_SW_Frequency)

#define DPC_VCTRL_BURST_PI_sat_up       0.3

#endif //__BUCK_APPLICATION_CONF_H
