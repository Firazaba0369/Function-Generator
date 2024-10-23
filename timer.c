/*
 * timer.c
 *
 *  Created on: Oct 23, 2024
 *      Author: firaz
 */
#include "main.h"

void TIM2_init(void){
	/*------------------- Configure PC0 for GPIOC output -------------------*/
	//configure GPIOC clock
	RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOCEN);
	//setup MODER for row output
	GPIOC->MODER &= ~(GPIO_MODER_MODE0);
	GPIOC->MODER |= (GPIO_MODER_MODE0_0);
	//set push-pull output type
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT0);
	//no PUPD
	GPIOC->PUPDR |= (GPIO_PUPDR_PUPD0_1);
	//set to high speed
	GPIOC->OSPEEDR |= (GPIO_OSPEEDR_OSPEED0_Msk);
	/*------------------- Configure TIM2 and interrupt -------------------*/
	//configure TIM2 clock
	RCC->APB1ENR1 |= (RCC_APB1ENR1_TIM2EN);
	//set TIM2 to count up
	TIM2->CR1 &= ~(TIM_CR1_DIR);
	//set ARR clock to 799 seconds for 5kHz square wave
	TIM2->ARR = ARR_VAL - 1;
	//set CCR1 to interrupt @ a 25% duty cycle
	TIM2->CCR1 = CCR_VAL - 1;
	//enable update event interrupt in TIM2
	TIM2->DIER |= (TIM_DIER_UIE | TIM_DIER_CC1IE);
	//clear the flag before starting
	TIM2->SR &= ~(TIM_SR_UIF | TIM_SR_CC1IF);
	//start timer
	TIM2->CR1 |= TIM_CR1_CEN;
	//enable interrupts globally
	__enable_irq();
	//enable TIM2 in NVIC
	NVIC->ISER[0] = (1 << TIM2_IRQn);
}
