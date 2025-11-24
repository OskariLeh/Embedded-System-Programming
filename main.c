/*
	!!We are using project template from moodle hardware / software tab!!
*/
#include <stdio.h>
#include <bitwise.h>   // bitwise operation macros
#include <stm32f4xx.h> // defines hardware registers



volatile uint8_t flag_MODEL = 0;
volatile uint8_t flag_CONTROLLER = 0;



void System_init()
{

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;// enable timer 2 clock

	// GPIOB and and GPIOC clocks (leds and buttons)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

}


//Buttons and LED init
void Gpio_init()
{
	//buttons 4 PC9, PC8, PC6, PC5

	//Input mode for buttons
	GPIOC->MODER &= ~(3 <<(9*2)); // btn PC9
	GPIOC->MODER &= ~(3 <<(8*2)); // btn PC8
	GPIOC->MODER &= ~(3 <<(6*2)); // btn PC6
	GPIOC->MODER &= ~(3 <<(5*2)); // btn PC5
	// clear pull-up/pull-down
	GPIOC->PUPDR &= ~(3 <<(9*2)); // btn PC9
	GPIOC->PUPDR &= ~(3 <<(8*2)); // btn PC8
	GPIOC->PUPDR &= ~(3 <<(6*2)); // btn PC6
	GPIOC->PUPDR &= ~(3 <<(5*2)); // btn PC5
	// pull-up
	GPIOC->PUPDR |= (1 <<(9*2)); // btn PC9
	GPIOC->PUPDR |= (1 <<(8*2)); // btn PC8
	GPIOC->PUPDR |= (1 <<(6*2)); // btn PC6
	GPIOC->PUPDR |= (1 <<(5*2)); // btn PC5

	//Leds 3 PB10, PB4, PB5
	GPIOB->MODER &= ~(3 << (10*2)); // led PB10
	GPIOB->MODER &= ~(3 << (5*2)); // led PB5
	GPIOB->MODER &= ~(3 << (4*2)); // led PB4
	// output
	GPIOB->MODER |= (1 << (10*2));
	GPIOB->MODER |= (1 << (5*2));
	GPIOB->MODER |= (1 << (4*2));


}

void Timer2_config()
{

	TIM2->PSC = 1 << 8; // prescaler
	TIM2->ARR = 1 << 12; // auto-reload
	TIM2->DIER |= TIM_DIER_UIE; // enable interrupt
	TIM2->CR1 |= TIM_CR1_URS; // counter overflow as only event source

	TIM2->CR2 |= TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM2_IRQn); // enable interrupt request
}

void Interrupt_handler()
{

}

int main()
{

	// System init
	System_init();
	// Buttons and leds
	Gpio_init();
	// Timer configuration
	Timer2_config();

	while (1)
	{
		if(flag_MODEL)
		{

		}
		if(flag_CONTROLLER)
		{

		}
	}
}
