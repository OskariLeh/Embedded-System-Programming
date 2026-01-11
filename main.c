/*
	!!We are using project template from moodle hardware / software tab!!
*/
#include <stdio.h>
#include <bitwise.h>   // bitwise operation macros
#include <stm32f4xx.h> // defines hardware registers

#ifndef MODE_MODULATE
#define MODE_MODULATE 2U
#endif

extern uint32_t SystemCoreClock; // system clock frequency 
// convert baud into BRR value
#define baud(bps) \
	(((SystemCoreClock/((bps)*16)) << 4) | ((SystemCoreClock/(bps)) % 16))

// Mode flag supplied by UI logic; determines whether control is active.
volatile uint8_t mode;

// Simple PI controller container used by scheduler.
typedef struct
{
	float kp;
	float ki;
	float Ts;
	float integrator;
	float prev_error;
	float u_min;
	float u_max;
} PIController;

// Controller and PWM helper prototypes.
static float clampf(float value, float min_value, float max_value);
static void pi_init(PIController *pi, float kp, float ki, float Ts, float u_min, float u_max);
static float pi_step(PIController *pi, float reference, float measurement);
void pi_set_kp(float kp);
void pi_set_ki(float ki);
static void pwm_init(void);
static void pwm_set_duty(float duty);
static void controller_subsystem_init(void);

// Global controller state available to other modules (e.g., UI adjustments).
static PIController g_pi;
volatile float controller_out = 0.0f;
volatile float reference_uout = 5.0f;
static uint32_t pwm_arr = 0;
static const float TS_CONTROL = 0.010f;


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


// keeps controller outputs within allowed range
static float clampf(float value, float min_value, float max_value)
{
	if (value < min_value) return min_value;
	if (value > max_value) return max_value;
	return value;
}

// Initialize PI controller
static void pi_init(PIController *pi, float kp, float ki, float Ts, float u_min, float u_max)
{
	if (!pi) return;
	pi->kp = kp;
	pi->ki = ki;
	pi->Ts = Ts;
	pi->integrator = 0.0f;
	pi->prev_error = 0.0f;
	pi->u_min = u_min;
	pi->u_max = u_max;
}

// PI iteration using trapezoidal integration with anti-windup
static float pi_step(PIController *pi, float reference, float measurement)
{
	if (!pi) return 0.0f;
	float error = reference - measurement;
	pi->integrator += 0.5f * pi->ki * pi->Ts * (error + pi->prev_error);
	float u = (pi->kp * error) + pi->integrator;
	float u_sat = clampf(u, pi->u_min, pi->u_max);
	if (u_sat != u)
	{
		pi->integrator = u_sat - (pi->kp * error);
	}
	pi->prev_error = error;
	return u_sat;
}

// adjust proportional gain
void pi_set_kp(float kp)
{
	g_pi.kp = kp;
}

// adjust integral gain
void pi_set_ki(float ki)
{
	g_pi.ki = ki;
}

// PA5/TIM2 CH1 to output a PWM duty proportional to controller_out
static void pwm_init(void)
{
	// configure PA5 as TIM2_CH1 (green LED)
	GPIOA->MODER &= ~(3U << (5U * 2U));
	GPIOA->MODER |= (2U << (5U * 2U));
	GPIOA->AFR[0] &= ~(0xFU << (5U * 4U));
	GPIOA->AFR[0] |= (1U << (5U * 4U));
	GPIOA->OTYPER &= ~(1U << 5);
	GPIOA->OSPEEDR |= (3U << (5U * 2U));
	GPIOA->PUPDR &= ~(3U << (5U * 2U));

	TIM2->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_OC1M);
	TIM2->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);
	TIM2->CCMR1 |= TIM_CCMR1_OC1PE;
	TIM2->CCER &= ~TIM_CCER_CC1P;
	TIM2->CCER |= TIM_CCER_CC1E;
	TIM2->CR1 |= TIM_CR1_ARPE;
	TIM2->EGR |= TIM_EGR_UG;

	pwm_arr = TIM2->ARR;
	TIM2->CCR1 = 0;
}

// translate duty ratio (0..1) to CCR1 counts
static void pwm_set_duty(float duty)
{
	float clamped = clampf(duty, 0.0f, 1.0f);
	uint32_t arr = TIM2->ARR;
	if (arr != pwm_arr)
	{
		pwm_arr = arr;
	}
	if (pwm_arr == 0)
	{
		TIM2->CCR1 = 0;
		return;
	}
	uint32_t ccr = (uint32_t)((clamped * (float)pwm_arr) + 0.5f);
	if (ccr > pwm_arr)
	{
		ccr = pwm_arr;
	}
	TIM2->CCR1 = ccr;
}

// Bring up PWM hardware and seed PI controller before scheduler starts
static void controller_subsystem_init(void)
{
	pwm_init();
	pi_init(&g_pi, 0.1f, 50.0f, TS_CONTROL, 0.0f, 1.0f);
	controller_out = 0.0f;
	pwm_set_duty(0.0f);
}

void uart_init(){
	// enable USART2 clock
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	// set PA2 & PA3 alternate functions to AF7 (USART2 RX/TX)
	bits_val(GPIOA->AFR[0], 4, 2, 7); // PA2 -> USART2_TX
	bits_val(GPIOA->AFR[0], 4, 3, 7); // PA3 -> USART2_RX
	bits_val(GPIOA->MODER , 2, 2, 2); // PA2 -> alternate function mode
	bits_val(GPIOA->MODER , 2, 3, 2); // PA3 -> alternate function mode
	// configure UART as 8N1 at 115200bps
	USART2->BRR = baud(115200);
	USART2->CR1 |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;
}

// send single character to UART
void uart_send(char c) {
	// wait for the transmit buffer to become empty
	while (!(USART2->SR & USART_SR_TXE));
	// begin transmission of character
	USART2->DR = c;
}

// send string to UART
void uart_send_str(char *s) {
	char c;
	// transmit whole string
	while ((c = *s++) != '\0')
	uart_send(c);
}
// receive single character from UART (blocking)
char uart_recv() {
	// wait for a single byte to be received
	printf("select mode (1, 2, 3): \n");
	while (!(USART2->SR & USART_SR_RXNE));
	// return received byte
	return USART2->DR;
}


void System_init()
{

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;// enable timer 2 clock

	// GPIOB and and GPIOC clocks (leds and buttons)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

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
	TIM2->CR1 |= TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM2_IRQn); // enable interrupt request
}

// IRQHandler
void TIM2_IRQHandler(void)
{
	if(TIM2->SR & TIM_SR_UIF)
	{
		TIM2->SR &= ~TIM_SR_UIF;  // clear update interrupt flag

		static uint32_t tick = 0; // short scheduler action only
		tick++;

		// set flags for main loop to run model
		flag_MODEL = 1;
		if((tick % 10) == 0) flag_CONTROLLER = 1;


	}
}


void Interrupt_handler()
{


}
static float model_output_u3(void)
{
	return x_state[5];
}

int main()
{

	// System init
	System_init();
	// Buttons and leds
	Gpio_init();
	// Timer configuration drives both scheduler ticks and shared PWM base
	Timer2_config();
	controller_subsystem_init();

	while (1)
	{
		if(flag_MODEL)
		{
			flag_MODEL = 0;
			float u_in = (mode == MODE_MODULATE) ? controller_out : 0.0f;
			model_update(u_in);   // run model with controller output when enabled
		}
		
		if(flag_CONTROLLER)
		{
			flag_CONTROLLER = 0;
			float meas = model_output_u3();  // model output U3
			if(mode == MODE_MODULATE)
			{
				// Closed-loop PI: drive measured voltage toward reference and update PWM
				controller_out = pi_step(&g_pi, reference_uout, meas);
				pwm_set_duty(controller_out);
			}
			else
			{
				// Modulation disabled: force zero duty and controller output
				controller_out = 0.0f;
				pwm_set_duty(0.0f);
			}

		}
	}
}
