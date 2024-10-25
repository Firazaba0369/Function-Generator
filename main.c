/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "DAC.h"
#include "timer.h"
#include "keypad.h"
#include "LUTs.h"

// Global variable to store keypress value
volatile uint8_t keypress_value = 9; //initialized to 9 for square wave
//Global variable to store frequency
volatile uint16_t freq = 1; //initialized to 1 for 100Hz
//prototype for interrupt function
void TIM2_IRQHandler(void);
//prototype for output waveform function
void output_waveform(uint8_t keypress);

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */

//  //---------------Configure GPIOA port for ISR time measuring--------------------//
  //configure GPIOA clock
  RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOAEN);
  //setup MODER for row output
  GPIOA->MODER &= ~(GPIO_MODER_MODE1);
  GPIOA->MODER |= (GPIO_MODER_MODE1_0);
  //set push-pull output type
  GPIOA->OTYPER &= ~(GPIO_OTYPER_OT1);
  //no PUPD
  GPIOA->PUPDR |= (GPIO_PUPDR_PUPD1_1);
  //set to high speed
  GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED1_Msk);
  GPIOA->ODR &= ~GPIO_ODR_OD1; //set bit low

  //initialize DAC, TIM2, and keypad
  DAC_init();
  TIM2_init((uint16_t)685);
  keypad_init();
  /* Infinite loop */
  while (1)
  {

  }
}

volatile uint16_t lut_index = 0; //index for
void TIM2_IRQHandler(void){
	//check for update event flag
	if (TIM2->SR & TIM_SR_UIF){
		GPIOA->ODR |= (GPIO_ODR_OD1); //set bit high
		output_waveform(keypress_value);
		//DAC_write(DAC_volt_conv(sine[lut_index])); //write the voltage point to the DAC
		lut_index+=freq; //index by frequency
		if (lut_index >= 588) {
			lut_index = 0; // Loop back to the start
		}
		TIM2->SR &= ~(TIM_SR_UIF);	// clear update event interrupt flag
		GPIOA->ODR &= ~GPIO_ODR_OD1; //set bit low
	}
}

/**
  * @brief Output Waveform
  * @retval None
  */
void output_waveform(uint8_t keypress){
	if(keypress == 6){
		DAC_write(DAC_volt_conv(sine[lut_index])); //output sine wave
	}
	else if(keypress == 7){
			DAC_write(DAC_volt_conv(triangle[lut_index])); //output triangle wave
	}
	else if(keypress == 8){
			DAC_write(DAC_volt_conv(ramp[lut_index])); //output ramp wave
	}
	else if(keypress == 9){
		DAC_write(DAC_volt_conv(square[lut_index])); //output square wave
	}
	else{
		DAC_write(0);//error
	}
}



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
