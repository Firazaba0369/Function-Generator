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
#define LUT_SIZE 588 //size of look up arrays
#define VOLT_HIGH (uint16_t)3000 //high voltage for duty cycle
#define VOLT_LOW (uint16_t)0 //low voltage for duty cycle
#define MAX_DUTY_CYCLE 9 //max duty cycle
#define MIN_DUTY_CYCLE 1 //min duty cycle

//prototype for interrupt function
void TIM2_IRQHandler(void);
//prototype for select waveform function
void select_waveform(int8_t keypress);
//prototype for output waveform function
void output_waveform(int8_t keypress);
//prototype for update frequency function
void update_freq(int8_t keypress);
//prototype for update duty cycle function
void update_duty_cycle(int8_t keypress);

// Global variable to store keypress value
volatile int8_t keypress_val = 9; //initialized to 9 for square wave
//Global variable to store frequency
volatile uint16_t freq = 1; //initialized to 1 for 100Hz
//Global variable to store duty cycle
volatile uint8_t duty_cycle = 5; //initialize to 50%
//Global variable for adding duty cycle to square wave
volatile uint16_t volt_val = 0; //initialize to low
//Global variable to detect if sine wave
volatile uint8_t is_sine = 0; //initialize to no square
//Global variable to detect if triangle wave
volatile uint8_t is_triangle = 0; //initialize to no triangle
//Global variable to detect if ramp wave
volatile uint8_t is_ramp = 0; //initialize to no ramp
//Global variable to detect if square wave
volatile uint8_t is_square = 1; //initialize to square
//Global variable for LUT index
volatile uint16_t lut_index = 0;
//Square wave array
volatile uint16_t square[588];
gen_square_wave(square, duty_cycle);//fill in square array

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
  //---------------Configure GPIOA port for ISR time measuring--------------------//
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
  TIM2_init();
  keypad_init();

  /* Infinite loop */
  while (1)
  {
	  int8_t keypad = NO_PRESS; //variable to read key-press
	  while(keypad == NO_PRESS){ //wait for key-press
		  keypad = keypad_func(); //read key-press
	  }
	  while(keypad_func() != NO_PRESS); //wait for key release
	  keypress_val = keypad; //update global key-press to keypad value
	  select_waveform(keypress_val); //select waveform to output
	  update_freq(keypress_val); //update frequency if needed
	  //check if waveform is square wave
	  if (is_square == 1){
		  update_duty_cycle(keypress_val); //update duty cycle if needed
	  }
  }
}


void TIM2_IRQHandler(void){
	//check for ARR flag
	if (TIM2->SR & TIM_SR_UIF){
		//GPIOA->ODR |= (GPIO_ODR_OD1); //set bit high
		if (is_square == 1){
			DAC_write(DAC_volt_conv(VOLT_HIGH));
		}
		else{
			output_waveform(keypress_val); //output according waveform
		}
		TIM2->SR &= ~(TIM_SR_UIF);	// clear update event interrupt flag
		//GPIOA->ODR &= ~GPIO_ODR_OD1; //set bit low
	}
    // Check for CCR1 flag
	else if (TIM2->SR & TIM_SR_CC1IF) {
    	GPIOA->ODR |= (GPIO_ODR_OD1); //set bit high
    	DAC_write(DAC_volt_conv(VOLT_LOW));
    	//output_waveform(keypress_val); //add a high or low volt value to square wave volt_val
        TIM2->SR &= ~TIM_SR_CC1IF;  // Clear CCR1 flag
        GPIOA->ODR &= ~GPIO_ODR_OD1; //set bit low
    }
}

/**
  * @brief Output Waveform
  * @retval None
  */
void output_waveform(int8_t keypress){
	if(is_sine == 1){
		DAC_write(DAC_volt_conv(sine[lut_index])); //output sine wave
		lut_index+=freq; //index by frequency
		if (lut_index >= LUT_SIZE) {
			lut_index = 0; // Loop back to the start
		}
	}
	else if(is_triangle == 1){
		DAC_write(DAC_volt_conv(triangle[lut_index])); //output triangle wave
		lut_index+=freq; //index by frequency
		if (lut_index >= LUT_SIZE) {
			lut_index = 0; // Loop back to the start
		}
	}
	else if(is_ramp == 1){
		DAC_write(DAC_volt_conv(ramp[lut_index])); //output ramp wave
		lut_index+=freq; //index by frequency
		if (lut_index >= LUT_SIZE) {
			lut_index = 0; // Loop back to the start
		}
	}
	else if(is_square == 1){
		lut_index ^= 1;
		DAC_write(DAC_volt_conv(square[lut_index])); //output square wave
	}
	return;
}


/**
  * @brief Select Waveform
  * @retval None
  */
void select_waveform(int8_t keypress){
	if(keypress == 6){
		is_sine = 1; //set sine wave
		is_triangle = 0; //no triangle wave
		is_ramp = 0; //no ramp wave
		is_square = 0; //no square wave
		disable_CCR(); //turn off CCR interrupt
	}
	else if(keypress == 7){
		is_sine = 0; //no sine wave
		is_triangle = 1; //no triangle wave
		is_ramp = 0; //no ramp wave
		is_square = 0;	//no square wave
		disable_CCR(); //turn off CCR interrupt
	}
	else if(keypress == 8){
		is_sine = 0; //no sine wave
		is_triangle = 0; //no triangle wave
		is_ramp = 1; //set ramp wave
		is_square = 0; //no square wave
		disable_CCR(); //turn off CCR interrupt
	}
	else if(keypress == 9){
		is_sine = 0; //no sine wave
		is_triangle = 0; //no triangle wave
		is_ramp = 0; //no ramp wave
		is_square = 1; //set square wave
		//set freq for square wave
		enable_CCR();
		TIM2->ARR = ARR_VAL/freq;
		TIM2->CCR1 = CCR_VAL/freq;
	}
	return;
}

/**
  * @brief Update Frequency
  * @retval None
  */
void update_freq(int8_t keypress){
	if(keypress == 1){
		freq = 1; //update frequency to 100Hz
	}
	else if(keypress == 2){
		freq = 2; //update frequency to 200Hz
	}
	else if(keypress == 3){
		freq = 3; //update frequency to 300Hz
		}
	else if(keypress == 4){
		freq = 4; //update frequency to 400Hz
	}
	else if(keypress == 5){
		freq = 5; //update frequency to 500Hz
	}
	else{
		return; //frequency stays the same
	}
}

/**
  * @brief Update Duty Cycle
  * @retval None
  */
void update_duty_cycle(int8_t keypress){
	//check if duty cycle needs to be increased by 10%
	if(keypress == ASTERISK && duty_cycle != MAX_DUTY_CYCLE){
		enable_CCR(); //turn on CCR timer
		TIM2->CCR1 += ARR_VAL / 10 ; //increase CCR time by 10% of ARR
		duty_cycle += 1; //increase duty cycle by 10%

//	    //check if duty cycle is above or below 50%
//		if(duty_cycle > 5){
//			volt_val = VOLT_HIGH; //set voltage high
//		}
//		else{
//			volt_val = VOLT_LOW; //set voltage low
//		}
	}
	//check if duty cycle needs to be decreased by 10%
	else if(keypress == POUND && duty_cycle != MIN_DUTY_CYCLE){
		enable_CCR(); //turn on CCR timer
		TIM2->CCR1 += ARR_VAL / 10 ;//decrease CCR time by 10% of ARR
		duty_cycle -= 1; //decrease duty cycle by 10%

		//check if duty cycle is above or below 50%
//		if(duty_cycle > 5){
//			volt_val = VOLT_HIGH; //set voltage high
//		}
//		else{
//			volt_val = VOLT_LOW; //set voltage low
//		}
	}
	//check if duty cycle needs to be reset to 50%
	else if(keypress == 0){
		//disable CCR for duty cycle that is not 50% already
		if (duty_cycle != 5){
			disable_CCR();
		}
		//reset to 50% duty cycle
		duty_cycle = 5;
	}
	//otherwise do nothing
	else{
		return;
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
