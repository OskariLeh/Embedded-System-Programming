// sleep.c - implementation of __sleep()

#include <stdint.h>
#include <stm32f4xx.h>

extern uint32_t SystemCoreClock; // system clock frequency

void __sleep(unsigned int t, unsigned int s)
{
	unsigned short psc = 0;
	double tmp, arr;

	// calculate timer parameters (prefer small prescaler)
	// arr = t*SystemCoreClock/(s*(psc + 1)) - 1
	tmp = (double)t*(double)SystemCoreClock/(double)s;
retry:
	// calculate auto-reload value
	arr = tmp/(psc + 1.0) - 1.0;

	// arr can't be represented by a short
	if (arr > (unsigned short)~0) {
		// increment prescaler by powers of 2
		psc = ((psc + 1) << 1) - 1;
		goto retry;
	}

	// configure auto-reload and prescaler
	TIM10->PSC = psc;
	TIM10->ARR = arr;

	// enable timer (one shot)
	TIM10->CNT  = 0;
	TIM10->CR1 |= TIM_CR1_OPM | TIM_CR1_CEN;

	// wait for timer
	while (!(TIM10->SR & TIM_SR_UIF))
		__WFE(); // suspend until event

	// clear timer
	TIM10->SR &= ~TIM_SR_UIF;
	NVIC_ClearPendingIRQ(TIM1_UP_TIM10_IRQn);
}

// sleep timer initilization, gets run before main()
static __attribute__((constructor)) void init()
{
	// enable timer 10 clock
	RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;

	// configure timer 10 (upcount mode)
	TIM10->DIER |= TIM_DIER_UIE; // enable interrupt
	TIM10->CR1  |= TIM_CR1_URS;  // set counter overflow as only event source

	// allow events to wake the MCU
	SCB->SCR |= SCB_SCR_SEVONPEND_Msk;
}
