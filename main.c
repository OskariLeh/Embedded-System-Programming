/*
	!!We are using project template from moodle hardware / software tab!!
*/
#include <stdio.h>
#include <bitwise.h>   // bitwise operation macros
#include <stm32f4xx.h> // defines hardware registers



volatile uint8_t flag_MODEL = 0;
volatile uint8_t flag_CONTROLLER = 0;

// converter model state and matrices
// state vector x = [i1, u1, i2, u2, i3, u3]^T
static float x_state[6] = {0};

// A matrix and B vector 

static const float A[6][6] = {
{0.9652, -0.0172,  0.0057, -0.0058,  0.0052, -0.0251},
{0.7732,  0.1252,  0.2315, -0.0700,  0.1282,  0.7754},
{0.8278, -0.7522, -0.0956,  0.3299, -0.4855,  0.3915},
{0.9948,  0.2655, -0.3848,  0.4212,  0.3927,  0.2899},
{0.7648, -0.4165, -0.4855, -0.3366, -0.0986,  0.7281},
{1.1056,  0.7587,  0.1179,  0.0748, -0.2192,  0.1491}
};

static const float B[6] = {0.0471, 0.0377, 0.0404, 0.0485, 0.0373, 0.0539};

// Compute x_(k+1) = A*x_k + B*U_in

void model_update(float u_in)
{
	float x_next[6];
	for (int i = 0; i < 6; ++i)
	{
		float s = 0.0;
		for (int j = 0; j < 6; ++j)
		{
			s += A[i][j] * x_state[j];	
		}
		s += B[i] * u_in;
		x_next[i] = s;
	}
	for (int i = 0; i < 6; ++i)
		{
			x_state[i] = x_next[i];
		}
}








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

// IRQHandler
void Timer2_IRQHandler()
{
	if(TIM2->SR & TIM_SR_UIF)
	{
		TIM2->SR &= ~TIM_SR_UIF;  // clear update interrupt flag

		static unit32_t tick = 0; // short scheduler action only
		tick++;

		// set flags for main loop to run model
		flag_MODEL = 1;
		if((tick % 10) == 0) flag_CONTROLLER = 1;


	}
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
			flag_MODEL = 0;
			float u_in = (mode == 2) ? controller_out : 0.0;
			model_update(u_in);   // run model
		}
		
		if(flag_CONTROLLER)
		{
			flag_CONTROLLER = 0;
			float meas = model_output_u3();  // model output U3

		}
	}
}
