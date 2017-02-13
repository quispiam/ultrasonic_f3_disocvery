/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOF
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define LD4_Pin GPIO_PIN_8
#define LD4_GPIO_Port GPIOE
#define LD3_Pin GPIO_PIN_9
#define LD3_GPIO_Port GPIOE
#define LD5_Pin GPIO_PIN_10
#define LD5_GPIO_Port GPIOE
#define LD7_Pin GPIO_PIN_11
#define LD7_GPIO_Port GPIOE
#define LD9_Pin GPIO_PIN_12
#define LD9_GPIO_Port GPIOE
#define LD10_Pin GPIO_PIN_13
#define LD10_GPIO_Port GPIOE
#define LD8_Pin GPIO_PIN_14
#define LD8_GPIO_Port GPIOE
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOE
#define DM_Pin GPIO_PIN_11
#define DM_GPIO_Port GPIOA
#define DP_Pin GPIO_PIN_12
#define DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* Sonar echo pulse pins */
#define sonar_0_echo_pin GPIO_PIN_10
#define sonar_0_echo_port GPIOF
#define sonar_1_echo_pin GPIO_PIN_15
#define sonar_1_echo_port GPIOC
#define sonar_2_echo_pin GPIO_PIN_13
#define sonar_2_echo_port GPIOC
#define sonar_3_echo_pin GPIO_PIN_5
#define sonar_3_echo_port GPIOE
#define sonar_4_echo_pin GPIO_PIN_3
#define sonar_4_echo_port GPIOE
#define sonar_5_echo_pin GPIO_PIN_1
#define sonar_5_echo_port GPIOE
#define sonar_6_echo_pin GPIO_PIN_9
#define sonar_6_echo_port GPIOB
#define sonar_7_echo_pin GPIO_PIN_7
#define sonar_7_echo_port GPIOB

/* Sonar trigger pins */
#define sonar_0_trigger_pin GPIO_PIN_9
#define sonar_0_trigger_port GPIOF
#define sonar_1_trigger_pin GPIO_PIN_14
#define sonar_1_trigger_port GPIOC
#define sonar_2_trigger_pin GPIO_PIN_6
#define sonar_2_trigger_port GPIOE
#define sonar_3_trigger_pin GPIO_PIN_4
#define sonar_3_trigger_port GPIOE
#define sonar_4_trigger_pin GPIO_PIN_2
#define sonar_4_trigger_port GPIOE
#define sonar_5_trigger_pin GPIO_PIN_0
#define sonar_5_trigger_port GPIOE
#define sonar_6_trigger_pin GPIO_PIN_8
#define sonar_6_trigger_port GPIOB
#define sonar_7_trigger_pin GPIO_PIN_6
#define sonar_7_trigger_port GPIOB

/* Speed of sound in 15C cave in um/us. based on 340 m/s. Divide the result by 1000 to get it in mm*/
#define speed_of_sound_at_15c_um_us 340

/* Exponential Moving Average decay coefficient used on the sonar.
 * We are using the EMA that has the coefficient in front of the t-1 value.
 * This coefficient results in a fairly slow update and good rejection of
 * noise, correspondingly it won't respond to quick changes it readings.
 * That it, don't fly flat out at the wall and expect it to stop. An interesting
 * effect of the sonar is that as the distance decreases the reflection time
 * decreases and thus the update rate can increase. This means that more samples
 * come in and the EMA will respond faster. */
#define ema_coefficient 0.9f


/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
