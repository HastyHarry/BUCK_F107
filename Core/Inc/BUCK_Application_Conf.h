
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BUCK_APPLICATION_CONF_H
#define __BUCK_APPLICATION_CONF_H

#define PI		 		 				3.14
#define SQRT_2           				1.41421                                                        /*!< Square Root of number 2 */
#define SQRT_3           				1.73205                                                        /*!< Square Root of number 3 */

#define BUCK_Tim1                       htim1
#define BUCK_Tim2                       htim2
#define BUCK_Tim3                       htim3
#define BUCK_Tim4                       htim4
#define BUCK_Tim5                       htim5
#define BUCK_Tim1_OC_CH					TIM_CHANNEL_1
#define BUCK_Tim4_OC_CH					TIM_CHANNEL_4
#define BUCK_Tim5_OC_CH					TIM_CHANNEL_1									/*!<Not in USE>*/

#define BUCK_SW_Frequency				12000											/*!<Switching Freq*/
#define BUCK_Math_Frequency				12000											/*!<Calculations Freq*/
#define BUCK_TO_Timer_Frequency			1000											/*!<TO Freq*/

#define BUCK_Tim1_PWM_CH 				TIM_CHANNEL_3
#define BUCK_Tim4_PWM_CH 				TIM_CHANNEL_2

#define ADC1_CHs						3
#define ADC1_MA_PERIOD_VDC				5
#define ADC1_MA_PERIOD_IDC				50
#define ADC1_MA_PERIOD_RAW				5//10
#define BUCK_ADC1 						hadc1
#define ADC_VAL_CHANGE_SPD_K			0.5

#define BUCK_IDC_LIM					25
#define BUCK_VAC_REF					400
#define BUCK_VDC_REF					275
#define BUCK_VDC_OV						430
#define BUCK_VDC_HIST					10												/*!< value in %*/
#define BUCK_VDC_REF_LOW_REF			BUCK_VDC_REF-(BUCK_VDC_REF*BUCK_VDC_HIST/100)
#define BUCK_VDC_REF_HIGH_REF			BUCK_VDC_REF+(BUCK_VDC_REF*BUCK_VDC_HIST/100)


//Option 1

#define BUCK_PID_W_F					2*PI*BUCK_Math_Frequency/10

#define V_PID_K_P						0.05//0.2 2304
#define V_PID_K_I						0.005//0.002//0.005 2304
#define V_PID_K_D						0.0//0.01
#define V_PID_W_F						2*PI*BUCK_Math_Frequency/10
#define V_PID_SAT_UP					1000
#define V_PID_SAT_DOWN					-1000.0
#define V_PID_HIST						1.0												/*!<Value in %>*/
#define V_PID_BASE_VAL					0.0*BUCK_VAC_REF
#define V_PID_SAT_UP_BURST				0.2
#define V_PID_SAT_DOWN_BURST			0.0
#define V_PID_MA						1
#define V_PID_RESOLUTION				1
#define V_PID_INT_SAT_UP				50
#define V_PID_INT_SAT_DOWN				-50
#define V_PID_ANTIWINDUP				0.01

#define I_PID_K_P						10//20//2 2304
#define I_PID_K_I						0
#define I_PID_K_D						0.0
#define I_PID_W_F						2*PI*BUCK_Math_Frequency/10
#define I_PID_SAT_UP					1000.0
#define I_PID_SAT_DOWN					0.0
#define I_PID_HIST						0.0												/*!<Value in %>*/
#define I_PID_BASE_VAL					0.5*BUCK_VAC_REF
#define I_PID_SAT_UP_BURST				0.2
#define I_PID_SAT_DOWN_BURST			0.0
#define I_PID_MA						1
#define I_PID_RESOLUTION				1
#define I_PID_INT_SAT_UP				10
#define I_PID_INT_SAT_DOWN				-400
#define I_PID_ANTIWINDUP				0.0


//Option 2
#define V_LIM_PID_K_P					0.5//1
#define V_LIM_PID_K_I					0.02//0.05
#define V_LIM_PID_K_D					0.5//0.5
#define V_LIM_PID_W_F					2*PI*BUCK_Math_Frequency/10
#define V_LIM_PID_SAT_UP				BUCK_VAC_REF
#define V_LIM_PID_SAT_DOWN				0.0
#define V_LIM_PID_HIST					0.0												/*!<Value in %>*/
#define V_LIM_PID_BASE_VAL				0.0*BUCK_VAC_REF
#define V_LIM_PID_SAT_UP_BURST			1000
#define V_LIM_PID_SAT_DOWN_BURST		0.0
#define V_LIM_PID_MA					1
#define V_LIM_PID_RESOLUTION			1
#define V_LIM_INT_SAT_UP				400
#define V_LIM_INT_SAT_DOWN				-400
#define V_LIM_PID_ANTIWINDUP			0.0

#define I_LIM_PID_K_P					10.0
#define I_LIM_PID_K_I					0.5
#define I_LIM_PID_K_D					0.0
#define I_LIM_PID_W_F					2*PI*BUCK_Math_Frequency/10
#define I_LIM_PID_SAT_UP				BUCK_VAC_REF
#define I_LIM_PID_SAT_DOWN				0.0
#define I_LIM_PID_HIST					0.0												/*!<Value in %>*/
#define I_LIM_PID_BASE_VAL				1.0*BUCK_VAC_REF
#define I_LIM_PID_SAT_UP_BURST			1000
#define I_LIM_PID_SAT_DOWN_BURST		0.0
#define I_LIM_PID_MA					1
#define I_LIM_PID_RESOLUTION			1
#define I_LIM_INT_SAT_UP				10
#define I_LIM_INT_SAT_DOWN				-400
#define I_LIM_PID_ANTIWINDUP			0.0

#define STUP_PID_K_P					0.0
#define STUP_PID_K_I					0.001
#define STUP_PID_K_D					0.0
#define STUP_PID_W_F					2*PI*BUCK_Math_Frequency/10
#define STUP_PID_SAT_UP					BUCK_VAC_REF
#define STUP_PID_SAT_DOWN				0.1*BUCK_VAC_REF
#define STUP_PID_HIST					0.0												/*!<Value in %>*/
#define STUP_PID_BASE_VAL				0.5
#define STUP_PID_SAT_UP_BURST			1000
#define STUP_PID_SAT_DOWN_BURST			0.0
#define STUP_PID_MA						1
#define STUP_PID_RESOLUTION				1
#define STUP_INT_SAT_UP					400
#define STUP_INT_SAT_DOWN				-400
#define STUP_PID_ANTIWINDUP				0.0


//ADC Settings
#define G_VAC                           4.25//4.708                                     /*!< Gain terms of the AC voltage sensing */
#define B_VAC                           1975                                            /*!< Bias terms of the AC voltage sensing */
#define G_IAC                           10//42.67                                     /*!< Gain terms of the AC current sensing */
#define B_IAC                           1290                                            /*!< Bias terms of the AC current sensing */
#define G_VDC                           0.16//200v devider - .058                      /*!< Gain terms of the DC voltage sensing */
#define B_VDC                           0                                               /*!< Bias terms of the DC voltage sensing */
#define G_IDC                           0.028                                          /*!< Gain terms of the DC current sensing */
#define B_IDC                           0                                           /*!< Bias terms of the DC current sensing */


///DPC PID
#define DPC_VCTRL_KP                    4E-4                                            /*!< VCTRL - Proportional gain of the PI regulator related to DC voltage control*/
#define DPC_VCTRL_KI                    0.3                                             /*!< VCTRL - Integral gain of the PI regulator related to DC voltage control*/
#define DPC_PFC_VDC                     50                                				/*!< VCTRL - DC Voltage referance value of the PFC [Expresed in Volt]*/
#define DPC_PFC_Iref_sat                10//22                                          /*!< VCTRL - d-q axis AC Current referance limit of the PFC [Expresed in AMPs]*/
#define DPC_VCTRL_PI_AWTG               0.02                                            /*!< VCTRL - Anti Wind-up GAIN*/
#define DPC_VCTRL_PI_sat_up             0.9          									/*!< VCTRL - Higher Current Referance Saturation LIMIT*/
#define DPC_VCTRL_PI_sat_down           0                                               /*!< VCTRL - Lower Current Referance Saturation LIMIT*/
#define DPC_VCTRL_PI_SAT_EN             SET                                             /*!< VCTRL - Current Referance Saturation Enable*/
#define DPC_VCTRL_PI_AW_EN              SET                                             /*!< VCTRL - Anti Wind-up Enable*/
#define DPC_PI_VDC_TS                   ((float)1/BUCK_SW_Frequency)

#define DPC_VCTRL_BURST_PI_sat_up       0.3

#endif //__BUCK_APPLICATION_CONF_H
