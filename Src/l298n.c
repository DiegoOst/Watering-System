/*
 * L298N.c
 *
 *  Created on: 30 set 2021
 *      Author: diego
 */


/* ---- There would be two version of this for educational purposes, bare metal and HAL ---- */


#include "l298n.h"
#include "stm32f4xx.h"

#define GPIOAEN			(1U<<0)
#define PIN6_ON			(1U<<6)
#define PIN7_ON			(1U<<7)
#define PIN6_OFF		(1U<<22)
#define PIN7_OFF		(1U<<23)

#define IN1_ON			PIN6_ON
#define IN2_ON			PIN7_ON
#define IN1_OFF			PIN6_OFF
#define IN2_OFF			PIN7_OFF

/* For this project we will use just one direction for the motor. So we use the IN1 and IN2
 * to control the direction of the motor. For this i will just need DIGITAL OUTPUTS
 * ENA will be used to control the speed with PWM
 *
 * IN1, IN2 -> GPIO -> PA, how do i chose pins for gpioa -> PA6, PA7
 * ENA -> PWM ->
 *
 *
 *
 */


void gpioa_init(void)
{
	/* *** Enabling PA6, PA7 as OUTPUT PINs *** */
	/* Enabling clock access AHB1EN for GPIOA */
	RCC->AHB1ENR |= GPIOAEN;

	/* Set PA7,PA6 as OUTPUT MODE: set 01 the corresponding MODER registers. Position 15,14: PA7, Position 13,12: PA6 */
	GPIOA->MODER &=~ (1U<<15);
	GPIOA->MODER |=  (1U<<14);

	GPIOA->MODER &=~ (1U<<13);
	GPIOA->MODER |=	 (1U<<12);

	/* *** Enabling PWM on *** */
}


/* The motor turn one direction with
 * IN1: HIGH and
 * IN2: LOW
 * This function may take an argument in the future to decide whether to turn left or right*/

void set_direction(void)
{
	/* Use BSRR for setting and resetting registers
	 * Let's set   in1: pa6
	 * Let's reset in2: pa7*/

	GPIOA->BSRR =  IN1_ON | IN2_OFF;

}



/* To set PWM we need to set first TIMERs, since PWM is generated by timer
 * we need to copy the TIMERs in compare value mode such that compare is a sort
 * of triggering event */

/* From BLOCK DIAG in stm32f446xe manual -> TIMER1/PWM uses APB2 as BUS */


#define TIM1EN				(1U<<0)


/* Prescaler probably determine how fast can a motor go. So if you have a very low freq and a DC of 10% it may be that
 * it stays on 1 min at 100% speed and off 9 min at 100% speed
 * frequency here may describe how fast this happens and that of course depends on the motor
 * but HOW?
 */

/* I have to set:
 * Prescaler
 * ARR
 * Enable timer
 *
 * frequency determined by TIMx_ARR
 * DC determined by TIMx_CCRx
 *
 *
 * Pulse Width Modulation mode allows to generate a signal with a frequency determined by
the value of the TIMx_ARR register and a duty cycle determined by the value of the
TIMx_CCRx register.
The PWM mode can be selected independently on each channel (one PWM per OCx
output) by writing ‘110’ (PWM mode 1) or ‘111’ (PWM mode 2) in the OCxM bits in the
TIMx_CCMRx register. The corresponding preload register must be enabled by setting the
OCxPE bit in the TIMx_CCMRx register, and eventually the auto-reload preload register (in
upcounting or center-aligned modes) by setting the ARPE bit in the TIMx_CR1 register.
As the preload registers are transferred to the shadow registers only when an update event
occurs, before starting the counter, all registers must be initialized by setting the UG bit in
the TIMx_EGR register.
OCx polarity is software programmable using the CCxP bit in the TIMx_CCER register. It
can be programmed as active high or active low. OCx output is enabled by a combination of
the CCxE, CCxNE, MOE, OSSI and OSSR bits (TIMx_CCER and TIMx_BDTR registers).
Refer to the TIMx_CCER register description for more details.
In PWM mode (1 or 2), TIMx_CNT and TIMx_CCRx are always compared to determine
whether TIMx_CCRx ≤ TIMx_CNT or TIMx_CNT ≤ TIMx_CCRx (depending on the direction
of the counter).
The timer is able to generate PWM in edge-aligned mode or center-aligned mode
depending on the CMS bits in the TIMx_CR1 register.
 */



void tim1_init()
{
	/* Enable clock access to tim2 */
	RCC->APB2ENR |= TIM1EN;

	/* Set prescaler value */
	TIM1->PSC = 1600 - 1; //	16 000 000 / 1 600 = 10 000

	/* Set auto-reload value */
	TIM1->ARR = 10000 - 1;	// 10 000 / 10 000 = 1 hz

	/* Clear counter */
	TIM1->CNT = 0;

	/* Enable timer */
	TIM1->CR1 |= CR1_CEN;
}


