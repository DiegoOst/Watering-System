/* We need to configure the push button as a input, the blue push button.
 * So let's find it in the guide first.
 *
 * From the stm32 manual:
 * B1 USER: the user button is connected to the I/O PC13 (pin 2) of the STM32 microcontroller.
 *
 *
 * 1. Before configuring a port or a peripheral WE NEED TO ENABLE CLOCK ACCESS TO IT
 * --- we need to know which bus is connected to it.
 * --- from datasheet -> BLOCK DIAGRAM and we look for GPIO PORT C and AHB1 IS CONNECTED
 * --- so we look for RCC AHB1 and we find that BIT 2 enables GPIOCEN FOR AHB1
 */
#include "stm32f4xx.h"
#include <stdint.h> //because we are using uint32_t


#define GPIOAEN				(1U<<0)
#define UART2EN				(1U<<17)

#define CR1_TE				(1U<<3)
#define CR1_UE				(1U<<13)

#define SR_TXE				(1U<<7)

#define SYS_FREQ			16000000 //16 milion cycle for sec
#define APB1_CLK			SYS_FREQ

#define UART_BAUDRATE		115200

/* USART: in this case usart 2 it's connected to USB, if it wasn't we would need a converter
 * or wires to connect to a device. It is connected to APB1.
 * --- looking for APB1ENR on REFERENCE MANUAL
 * --- finding out that 17th bit is the one needed to enable CLOCK for USART2
 * --- tx,rx are GPIO pins so we need to see which PINs UART2 uses.
 * --- search for alternate func in STM manual to find which pins usart2 uses
 * --- PA2 USART2_TX, PA3 USART_RX IN AF07 Columns
 * --- so we need to configure PA2 and PA3 in alternate function AF07
 *
 */

static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate);
static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate);
void uart2_write(int ch);



int main(void)
{
	uar2_tx_init();


	while(1)
	{
		uart2_write('s');
	}

}

void uar2_tx_init(void)
{
	/******** Configure uart gpio pin ********/

	/* Enable clock access to gpioa => since we are using PA2 AND PA3 */
	RCC->AHB1ENR |= GPIOAEN; //GPIOA is connected to AHB1 so we use the AHB1ENR to enable it

	/* Set PA2 mode to alternate function mode */
	// --- We search moder in the reference manual
	// --- Look for MODER2 since we are referring to pin 2, set it to 10 (so pin 5 to 1, pin 4 to 0)
	GPIOA->MODER &=~ (1U<<4);
	GPIOA->MODER |= (1U<<5);

	/* Set PA2 alternate function type to UART_TX (AF07) */
	// We do it in the alternate function register
	// Since we are interested in PIN2 we look for AFRL2 that goes from pin 8 to 11
	// If we want AF07 we need to set those bits to 0111 (pin 8 to 11)
	// Index 0 = AFRL, Index 1 = AFRH.
	GPIOA->AFR[0] |= (1U<<8);
	GPIOA->AFR[0] |= (1U<<9);
	GPIOA->AFR[0] |= (1U<<10);
	GPIOA->AFR[0] &=~ (1U<<11);

	/******** Configure uart module ********/
	/* Enable clock access to uart2 */
	RCC->APB1ENR |= UART2EN;

	/* Configure baudrate */
	// We use a formula->function
	uart_set_baudrate(USART2, APB1_CLK, UART_BAUDRATE);

	/* Configure the tranfer direction */
	// Tx, so we go control register 1 and we see what it enables the tx
	// bit number 3 => TE: transmitter enable
	USART2->CR1 = CR1_TE; // Not | operator because i want to clean everything else (THE REST IS 0)

	/* Enable uart module */
	USART2->CR1 |= CR1_UE;
}

void uart2_write(int ch)
{
	/* Make sure the transmit data register is empty */
	while(!(USART2->SR & SR_TXE)) {}

	/* Write to transmit data register */

	USART2->DR = (ch & 0xFF); // it's because we want to trasmit 8 bits so we mask it
}


static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate)
{
	USARTx->BRR = compute_uart_bd(PeriphClk, BaudRate); //BRR is baudrate register
}

static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate)
{
	return ((PeriphClk + (BaudRate/2U))/BaudRate);
}











