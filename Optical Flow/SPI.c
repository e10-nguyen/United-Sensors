#include "SPI.h"
#include "SysTimer.h"
#include "stm32l476xx.h"


extern uint8_t Rx1_Counter;
extern uint8_t Rx2_Counter;


void SPI_GPIO_Init(void) {// need check
	// initialize SPI1 GPIO pins
	//RCC->AHB2ENR|= RCC_AHB2ENR_GPIOBEN;  //enable the GPIO B
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; // enable GPIOA
	
	GPIOB->MODER &= ~(GPIO_MODER_MODE3 | GPIO_MODER_MODE3 |
										GPIO_MODER_MODE4 | GPIO_MODER_MODE5); //clear the bit in moder PB3,4,5
	
	
	GPIOB->MODER |= GPIO_MODER_MODE3_1 | GPIO_MODER_MODE4_1 |
									GPIO_MODER_MODE5_1; //set PB3,4,5 into Alternative function mode
	
	
	//set PB3,4,5 to AF5(0101)
	GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL3 | GPIO_AFRL_AFSEL4 | GPIO_AFRL_AFSEL5);//clear
	
	
	GPIOB->AFR[0] |= GPIO_AFRL_AFSEL3_0 | GPIO_AFRL_AFSEL3_2;//for PB3

	GPIOB->AFR[0] |= GPIO_AFRL_AFSEL4_0 | GPIO_AFRL_AFSEL4_2;//for PB4
	
	GPIOB->AFR[0] |= GPIO_AFRL_AFSEL5_0 | GPIO_AFRL_AFSEL5_2;//for PB5
	
	
	
	//set PB3,4,5 & PA15 to push-pull output type, which is "0" in this case
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT3 | GPIO_OTYPER_OT4 | GPIO_OTYPER_OT5);
	
	//set PB3,4,5 & PA15 to very high speed, which is "11" in this case
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED3 | GPIO_OSPEEDR_OSPEED4 | GPIO_OSPEEDR_OSPEED5;
	
	
	//set PB3,4,5 & PA15 to no pull, which is "00"
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD3 | GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5);
	
}

//SPI Init for ILI9341 on Nucleo board
void SPI_Init(void){
	// Enable SPI clock and Reset SPI
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	
	RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST; // reset
	RCC->APB2RSTR &= ~(RCC_APB2RSTR_SPI1RST); // clear again,(not stay in reset forever)
	// Disable SPI
	SPI1->CR1 &= ~(SPI_CR1_SPE); // disable the SPI1 enable-bit
	// Configure for Full Duplex Communication
	SPI1->CR1 &= ~(SPI_CR1_RXONLY);
	// Configure for 2-line Unidirectional Data Mode
	SPI1->CR1 &= ~(SPI_CR1_BIDIMODE); 
	// Set Frame Format
	SPI1->CR2 |= (SPI_CR2_DS); // 8 bit mode 0000
	SPI1->CR2 &= ~SPI_CR2_DS_3; // 0111
	// Configure Clock
	
	// Set Baud Rate Prescaler.
	//TODO: set to the highest one : 40M
	SPI1->CR1 &= ~(SPI_CR1_BR); // Baudrate/2
	SPI1->CR1 |= SPI_CR1_BR_0;
	
	// Disable Hardware CRC Calculation
	SPI1->CR1 &= ~(SPI_CR1_CRCEN);
	
	// Set as Master and Enable Software Slave Management and NSS Pulse Management
	SPI1->CR1 |= SPI_CR1_MSTR; // Set as master
		
	// Manage NSS using Software
	SPI1->CR1 |= SPI_CR1_SSM; // enable SSM mode
	SPI1->CR2 |= SPI_CR2_NSSP; // NSS pulse generated
	
	// Set FIFO Reception Threshold
	SPI1->CR2 |= SPI_CR2_FRXTH; 	// 8 bit mode

	// Enable SPI
	SPI1->CR1 |= SPI_CR1_SPE;
}


void SPI_Write(SPI_TypeDef * SPIx, uint8_t *txBuffer, int size) {
	//Set up SPI TX
	for (int i = 0; i < size; i++){
		while(!(SPIx->SR & SPI_SR_TXE)); // while this flag is not on, go next line
		*((volatile uint8_t*) &SPIx->DR) = *txBuffer; // cast data format
		while(SPIx->SR & SPI_SR_BSY); // while busy is 1->0, go next line
		txBuffer++;
	}
}
