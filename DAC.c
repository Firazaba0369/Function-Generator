/*
 * DAC.c
 *
 *  Created on: Oct 23, 2024
 *      Author: firaz
 */
/**
  * @brief initialize the SPI peripheral to communicate with the DAC
  * @retval None
  */
#include "main.h"
#define AF5 5 //Alternate Function Register
#define MAX_12BIT_VAL 0xFFF //max 12-bit value
#define MAX_VOLT 3300 //max mV value
#define DEFAULT_DAC 0x3000 //default settings for first four bits of DAC
#define MASK 0xF000//mask for DAC initialization

void SystemClock_Config(void);

void DAC_init(void){
	//set clock for GPIOA and SPI
	RCC->AHB2ENR  |= (RCC_AHB2ENR_GPIOAEN);
	RCC->APB2ENR |= (RCC_APB2ENR_SPI1EN);
	/*------------------- Configure GPIOA PA4, PA5, PA7 for NSS, MOSI, and SCK from SPI -------------------*/
	//setup MODER in Alternate Function Mode
	GPIOA->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE7);
	GPIOA->MODER |= (GPIO_MODER_MODE4_1 | GPIO_MODER_MODE5_1 | GPIO_MODER_MODE7_1);
	//set alternate function I/O to map to AF5
	GPIOA-> AFR[0] &= ~(GPIO_AFRL_AFSEL4 | GPIO_AFRL_AFSEL5 | GPIO_AFRL_AFSEL7);
	GPIOA-> AFR[0] |= (AF5 << GPIO_AFRL_AFSEL4_Pos | AF5 << GPIO_AFRL_AFSEL5_Pos | AF5 << GPIO_AFRL_AFSEL7_Pos);
	//set push pull output type
	GPIOA->OTYPER  &=  ~(GPIO_OTYPER_OT4 | GPIO_OTYPER_OT5 | GPIO_OTYPER_OT7);
	//no PUPD
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5 |GPIO_PUPDR_PUPD7);
	//set to high speed
	GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED4 | GPIO_OSPEEDR_OSPEED5 | GPIO_OSPEEDR_OSPEED7);

	/*------------------- Configure SPI to communicate with DAC ------------------------------*/
	//set baud rate to 2MHz (4Mhz clock/2)
	SPI1->CR1 &= ~(SPI_CR1_BR);
	//set clock polarity to 0 when idle
	SPI1->CR1 &= ~(SPI_CR1_CPOL);
	//set clock phase to 0 to capture data on first transition
	SPI1->CR1 &= ~(SPI_CR1_CPHA);
	//set to FULLPLEX
	SPI1->CR1 &= ~(SPI_CR1_RXONLY);
	//set frame format to MSB first
	SPI1->CR1 &= ~(SPI_CR1_LSBFIRST);
	//disable CRC calculation
	SPI1->CR1 &= ~(SPI_CR1_CRCEN);
	//Disable SSM
	SPI1->CR1 &= ~(SPI_CR1_SSM);
	//set MCU as master
	SPI1->CR1 |= (SPI_CR1_MSTR);
	//set data size to 16-bit
	SPI1->CR2 &= ~(SPI_CR2_DS);
	SPI1->CR2 |= (SPI_CR2_DS);
	//enable SS output
	SPI1->CR2 |= (SPI_CR2_SSOE);
	//set frame format to Motorola mode
	SPI1->CR2 &= ~(SPI_CR2_FRF);
	//enable NSS pulse management
	SPI1->CR2 |= (SPI_CR2_NSSP);
	//set FIFO reception threshold to 16-bit
	SPI1->CR2 &= ~(SPI_CR2_FRXTH);
	//enable SPI
	SPI1->CR1 |= SPI_CR1_SPE;
}

/**
  * @brief write a 12-bit value to the DAC
  * @retval None
  */
void DAC_write(uint16_t transmission_data){
	uint16_t data_frame = transmission_data & ~(MASK); //Mask first 4 bits
	data_frame |= (DEFAULT_DAC);
	while(!(SPI1->SR & SPI_SR_TXE)) {}; // Wait while transmission buffer is full, check other SR flags(?)
	SPI1->DR = data_frame;// load data into data register
}

/**
  * @brief convert a voltage value into a 12-bit value to control the DAC
  * @retval uint16_t
  */
uint16_t DAC_volt_conv (uint16_t voltage_value){
	if (voltage_value > MAX_VOLT){
		return MAX_12BIT_VAL; //return max 12-bit value
	}
	else{
		return 	(voltage_value*MAX_12BIT_VAL)/MAX_VOLT; //formula for voltage conversion
	}
}



