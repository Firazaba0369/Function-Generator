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
#define SINE 0
#define TRIANGLE 1
#define RAMP 2
#define SQUARE 3
#define ONE_HUNDRED 1
#define TWO_HUNDRED 2
#define THREE_HUNDRED 3
#define FOUR_HUNDRED 4
#define FIVE_HUNDRED 5
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
void output_waveform();
//prototype for update frequency function
void update_freq(int8_t keypress);
//prototype for update duty cycle function
void update_duty_cycle(int8_t keypress);
// Global variable to store keypress value
volatile int8_t keypress_val = 9; //initialized to 9 for square wave
//Global variable to store frequency
volatile uint16_t freq = ONE_HUNDRED; //initialized to 1 for 100Hz
//Global variable to store duty cycle
volatile uint8_t duty_cycle = 5; //initialize to 50%
//Global variable for wave sel SINE = 0, TRIANGLE = 1, RAMP = 2, SQUARE = 3
volatile uint8_t wave_sel = SQUARE; //initialize to square
//Global variable for LUT index
volatile uint16_t lut_index = 0;
//Square wave array
uint16_t square[588];


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
  //initialize DAC, TIM2, and keypad
  DAC_init();
  TIM2_init();
  keypad_init();

  //configure square wave
  gen_square_wave(square, duty_cycle);//fill in square array

  /* Infinite loop */
  while (1)
  {
	  int8_t keypad = NO_PRESS;
	  while(keypad == NO_PRESS){ //wait for key-press
		  keypad = keypad_func(); //read key-press
	  }
	  while(keypad_func() != NO_PRESS); //wait for key release
	  keypress_val = keypad; //update global key-press to keypad value
	  select_waveform(keypress_val); //select waveform to output
	  update_freq(keypress_val); //update frequency if needed

	  //check if waveform is square wave
	  if (wave_sel == SQUARE){
		  update_duty_cycle(keypress_val); //update duty cycle if needed
	  }
  }
}


void TIM2_IRQHandler(void){
	//check for ARR flag
	if (TIM2->SR & TIM_SR_UIF){
		output_waveform(keypress_val); //output according waveform
		lut_index+=freq; //index by frequency
		if (lut_index >= LUT_SIZE) {
			lut_index = 0; // Loop back to the start
		}
		TIM2->SR &= ~(TIM_SR_UIF);	// clear update event interrupt flag
	}
}

/**
  * @brief Output Waveform
  * @retval None
  */
void output_waveform(){
	if(wave_sel == SINE){
		DAC_write(DAC_volt_conv(sine[lut_index])); //output sine wave
	}
	else if(wave_sel == TRIANGLE){
		DAC_write(DAC_volt_conv(triangle[lut_index])); //output triangle wave
	}
	else if(wave_sel == RAMP){
		DAC_write(DAC_volt_conv(ramp[lut_index])); //output ramp wave
	}
	else if(wave_sel == SQUARE){
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
		wave_sel = SINE; //sel sine wave
	}
	else if(keypress == 7){
		wave_sel = TRIANGLE; //sel triangle wave
	}
	else if(keypress == 8){
		wave_sel = RAMP; //sel ramp wave
	}
	else if(keypress == 9){
		wave_sel = SQUARE; //sel square wave
	}
	return;
}

/**
  * @brief Update Frequency
  * @retval None
  */
void update_freq(int8_t keypress){
	if(keypress == 1){
		freq = ONE_HUNDRED; //update frequency to 100Hz
	}
	else if(keypress == 2){
		freq = TWO_HUNDRED; //update frequency to 200Hz
	}
	else if(keypress == 3){
		freq = THREE_HUNDRED; //update frequency to 300Hz
		}
	else if(keypress == 4){
		freq = FOUR_HUNDRED; //update frequency to 400Hz
	}
	else if(keypress == 5){
		freq = FIVE_HUNDRED; //update frequency to 500Hz
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
		duty_cycle += 1; //increase duty cycle by 10%
		gen_square_wave(square, duty_cycle); //change square wave array
	}
	//check if duty cycle needs to be decreased by 10%
	else if(keypress == POUND && duty_cycle != MIN_DUTY_CYCLE){
		duty_cycle -= 1; //decrease duty cycle by 10%
		gen_square_wave(square, duty_cycle); // change square wave array
	}
	//check if duty cycle needs to be reset to 50%
	else if(keypress == 0){
		duty_cycle = 5; //set duty cycle to 50%
		gen_square_wave(square, duty_cycle); //change square wave array
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
