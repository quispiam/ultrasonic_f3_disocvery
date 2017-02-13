/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "crc.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint32_t sonar_0_range_um;
uint32_t sonar_1_range_um;
uint32_t sonar_2_range_um;
uint32_t sonar_3_range_um;
uint32_t sonar_4_range_um;
uint32_t sonar_5_range_um;
uint32_t sonar_6_range_um;
uint32_t sonar_7_range_um;

uint16_t sonar_0_range_ema_mm = 1337;
uint16_t sonar_1_range_ema_mm = 1337;
uint16_t sonar_2_range_ema_mm = 1337;
uint16_t sonar_3_range_ema_mm = 1337;
uint16_t sonar_4_range_ema_mm = 1337;
uint16_t sonar_5_range_ema_mm = 1337;
uint16_t sonar_6_range_ema_mm = 1337;
uint16_t sonar_7_range_ema_mm = 1337;

// bit 0:7 represent sonar 0:7
uint8_t sonar_running = 0;

uint8_t results_string[19];

// variables to store when each sensor was triggered to allow for timeout retriggering
uint32_t usonic_0_trigger_time = 0;
uint32_t usonic_1_trigger_time = 0;
uint32_t usonic_2_trigger_time = 0;
uint32_t usonic_3_trigger_time = 0;
uint32_t usonic_4_trigger_time = 0;
uint32_t usonic_5_trigger_time = 0;
uint32_t usonic_6_trigger_time = 0;
uint32_t usonic_7_trigger_time = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void delay_us(uint32_t us);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void sonar_trigger(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */

  uint16_t count = 0;
  uint8_t teststring[] = "hello\n";

  HAL_UART_Transmit_IT(&huart1, teststring, sizeof(teststring));

  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);

  HAL_TIM_Base_Start_IT(&htim2);



  HAL_GPIO_WritePin(sonar_0_trigger_port, sonar_0_trigger_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(sonar_1_trigger_port, sonar_1_trigger_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(sonar_2_trigger_port, sonar_2_trigger_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(sonar_3_trigger_port, sonar_3_trigger_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(sonar_4_trigger_port, sonar_4_trigger_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(sonar_5_trigger_port, sonar_5_trigger_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(sonar_6_trigger_port, sonar_6_trigger_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(sonar_7_trigger_port, sonar_7_trigger_pin, GPIO_PIN_RESET);

  results_string[0] = 'T';
  results_string[1] = 'H';
  results_string[2] = (uint8_t)(sonar_0_range_ema_mm >> 8);
  results_string[3] = (uint8_t)(sonar_0_range_ema_mm);
  results_string[4] = (uint8_t)(sonar_1_range_ema_mm >> 8);;
  results_string[5] = (uint8_t)(sonar_1_range_ema_mm);
  results_string[6] = (uint8_t)(sonar_2_range_ema_mm >> 8);
  results_string[7] = (uint8_t)(sonar_2_range_ema_mm);
  results_string[8] = (uint8_t)(sonar_3_range_ema_mm >> 8);
  results_string[9] = (uint8_t)(sonar_3_range_ema_mm);
  results_string[10] = (uint8_t)(sonar_4_range_ema_mm >> 8);
  results_string[11] = (uint8_t)(sonar_4_range_ema_mm);
  results_string[12] = (uint8_t)(sonar_5_range_ema_mm >> 8);
  results_string[13] = (uint8_t)(sonar_5_range_ema_mm);
  results_string[14] = (uint8_t)(sonar_6_range_ema_mm >> 8);
  results_string[15] = (uint8_t)(sonar_6_range_ema_mm);
  results_string[16] = (uint8_t)(sonar_7_range_ema_mm >> 8);
  results_string[17] = (uint8_t)(sonar_7_range_ema_mm);
  results_string[18] = crc_crc8(results_string, 18);

  HAL_UART_Transmit_IT(&huart1, results_string, sizeof(results_string));


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
  /* USER CODE END WHILE */


  /* USER CODE BEGIN 3 */
      sonar_trigger();

      results_string[2] = (uint8_t)(sonar_0_range_ema_mm >> 8);
      results_string[3] = (uint8_t)(sonar_0_range_ema_mm);
      results_string[4] = (uint8_t)(sonar_1_range_ema_mm >> 8);;
      results_string[5] = (uint8_t)(sonar_1_range_ema_mm);
      results_string[6] = (uint8_t)(sonar_2_range_ema_mm >> 8);
      results_string[7] = (uint8_t)(sonar_2_range_ema_mm);
      results_string[8] = (uint8_t)(sonar_3_range_ema_mm >> 8);
      results_string[9] = (uint8_t)(sonar_3_range_ema_mm);
      results_string[10] = (uint8_t)(sonar_4_range_ema_mm >> 8);
      results_string[11] = (uint8_t)(sonar_4_range_ema_mm);
      results_string[12] = (uint8_t)(sonar_5_range_ema_mm >> 8);
      results_string[13] = (uint8_t)(sonar_5_range_ema_mm);
      results_string[14] = (uint8_t)(sonar_6_range_ema_mm >> 8);
      results_string[15] = (uint8_t)(sonar_6_range_ema_mm);
      results_string[16] = (uint8_t)(sonar_7_range_ema_mm >> 8);
      results_string[17] = (uint8_t)(sonar_7_range_ema_mm);
      results_string[18] = crc_crc8(results_string, 18);

      HAL_UART_Transmit_IT(&huart1, results_string, sizeof(results_string));

      delay_us(10000);
      //HAL_UART_Transmit_IT(&huart1, teststring, sizeof(teststring));


    }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint32_t usonic_0_start_time = 0;
  static uint32_t usonic_1_start_time = 0;
  static uint32_t usonic_2_start_time = 0;
  static uint32_t usonic_3_start_time = 0;
  static uint32_t usonic_4_start_time = 0;
  static uint32_t usonic_5_start_time = 0;
  static uint32_t usonic_6_start_time = 0;
  static uint32_t usonic_7_start_time = 0;

  /* find which which pin triggered the interrupt. If the sonar echo has just gone high,
   *  then we need to start counting. If it just went low, then we need to stop counting
   *  and calculate the distance   */
  switch(GPIO_Pin)
  {
    case sonar_0_echo_pin :
      if (HAL_GPIO_ReadPin(sonar_0_echo_port, sonar_0_echo_pin))
	{
	  usonic_0_start_time = __HAL_TIM_GET_COUNTER(&htim2);
	}
      else
	{
	  //get the timer value and calc the distance in um
	  sonar_0_range_um = ((__HAL_TIM_GET_COUNTER(&htim2) - usonic_0_start_time) * speed_of_sound_at_15c_um_us);
	    //ensure that when the timer wraps we default to the last measurement
	    if (sonar_0_range_um <= 0)
	    {
		sonar_0_range_um = sonar_0_range_ema_mm;
	    }
	  //use an exponential moving average to remove spurious readings. Also convert to mm
	  sonar_0_range_ema_mm = (uint16_t)((sonar_0_range_ema_mm * ema_coefficient) + ((1 - ema_coefficient) * (sonar_0_range_um / 1000)));

	  //clear the sonar running bit so that it can be triggered again
	  sonar_running &= ~(1 << 0);
	}
      break;

    case sonar_1_echo_pin :
      if (HAL_GPIO_ReadPin(sonar_1_echo_port, sonar_1_echo_pin))
	{
	  usonic_1_start_time = __HAL_TIM_GET_COUNTER(&htim2);
	}
      else
	{
	  //get the timer value and calc the distance in um
	  sonar_1_range_um = ((__HAL_TIM_GET_COUNTER(&htim2) - usonic_1_start_time) * speed_of_sound_at_15c_um_us);
	  //ensure that when the timer wraps we default to the last measurement
	  if (sonar_0_range_um <= 0)
	  {
	      sonar_0_range_um = sonar_0_range_ema_mm;
	  }
	  //use an exponential moving average to remove spurious readings. Also convert to mm
	  sonar_1_range_ema_mm = (uint16_t)((sonar_1_range_ema_mm * ema_coefficient) + ((1 - ema_coefficient) * (sonar_1_range_um / 1000)));

	  //clear the sonar running bit so that it can be triggered again
	  sonar_running &= ~(1 << 1);
	}
      break;


    case sonar_2_echo_pin :
      if (HAL_GPIO_ReadPin(sonar_2_echo_port, sonar_2_echo_pin))
	{
	  usonic_2_start_time = __HAL_TIM_GET_COUNTER(&htim2);
	}
      else
	{
	  //get the timer value and calc the distance in um
	  sonar_2_range_um = ((__HAL_TIM_GET_COUNTER(&htim2) - usonic_2_start_time) * speed_of_sound_at_15c_um_us);
	  //ensure that when the timer wraps we default to the last measurement
	  if (sonar_0_range_um <= 0)
	  {
	      sonar_0_range_um = sonar_0_range_ema_mm;
	  }
	  //use an exponential moving average to remove spurious readings. Also convert to mm
	  sonar_2_range_ema_mm = (uint16_t)((sonar_2_range_ema_mm * ema_coefficient) + ((1 - ema_coefficient) * (sonar_2_range_um / 1000)));

	  //clear the sonar running bit so that it can be triggered again
	  sonar_running &= ~(1 << 2);
	}
      break;

  }


}

void sonar_trigger(void)
{
  //check if sonar x is running, if not then trigger it
  if ( !(sonar_running & (1 << 0)))
    {
      HAL_GPIO_WritePin(sonar_0_trigger_port, sonar_0_trigger_pin, GPIO_PIN_SET);
      delay_us(50);
      HAL_GPIO_WritePin(sonar_0_trigger_port, sonar_0_trigger_pin, GPIO_PIN_RESET);
      sonar_running |= (1 << 0);
      usonic_0_trigger_time = __HAL_TIM_GET_COUNTER(&htim2);
    }
  else if ( ( __HAL_TIM_GET_COUNTER(&htim2) - usonic_0_trigger_time) > sonar_timeout_retrigger_us)
    {
      HAL_GPIO_WritePin(sonar_0_trigger_port, sonar_0_trigger_pin, GPIO_PIN_SET);
      delay_us(50);
      HAL_GPIO_WritePin(sonar_0_trigger_port, sonar_0_trigger_pin, GPIO_PIN_RESET);
      sonar_running |= (1 << 0);
      usonic_0_trigger_time = __HAL_TIM_GET_COUNTER(&htim2);
    }

  //check if sonar x is running, if not then trigger it
  if ( !(sonar_running & (1 << 1)))
  	{

	  HAL_GPIO_WritePin(sonar_1_trigger_port, sonar_1_trigger_pin, GPIO_PIN_SET);
	  delay_us(50);
	  HAL_GPIO_WritePin(sonar_1_trigger_port, sonar_1_trigger_pin, GPIO_PIN_RESET);
	  sonar_running |= (1 << 1);
  	}

  //check if sonar x is running, if not then trigger it
  if ( !(sonar_running & (1 << 2)))
  	{

	  HAL_GPIO_WritePin(sonar_2_trigger_port, sonar_2_trigger_pin, GPIO_PIN_SET);
	  delay_us(50);
	  HAL_GPIO_WritePin(sonar_2_trigger_port, sonar_2_trigger_pin, GPIO_PIN_RESET);
	  sonar_running |= (1 << 2);
  	}
}

void delay_us(uint32_t us)
{
  uint32_t time_start = __HAL_TIM_GET_COUNTER(&htim2);
  uint32_t max = 0xffffffff;

  //handle counter overflow
  if (time_start > max - us)
    {
      uint32_t overflow = (time_start - (max - us));
      while (__HAL_TIM_GET_COUNTER(&htim2) > time_start)
	{
	}
      while (__HAL_TIM_GET_COUNTER(&htim2) < overflow)
	{
	}
    }
  //normal upcounting
  else
    {
      while (__HAL_TIM_GET_COUNTER(&htim2) < (time_start + us))
        {
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
