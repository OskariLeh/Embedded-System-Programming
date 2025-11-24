// stdio_uart.c - redirect standard streams to UART

#include <stdint.h>
#include <bitwise.h>
#include <stm32f4xx.h>

// building with FreeRTOS
#ifdef FREERTOS
#include <FreeRTOS.h>
#include <task.h>
#endif

extern uint32_t SystemCoreClock; // system clock frequency

// convert baud into BRR value
#define baud(bps) \
	(((SystemCoreClock/((bps)*16)) << 4) | ((SystemCoreClock/(bps)) % 16))

#if ENABLE_SERIAL == 1

// _write() syscall redirect to UART2
int _write(int fd, char *data, int size)
{
	// only write to stdout/stderr (no distinction)
	if ((fd != 1) && (fd != 2))
		return -1;

#ifdef FREERTOS
	vTaskSuspendAll(); // FreeRTOS: no context switch
#endif

	// enable TXE event
	USART2->CR1 |= USART_CR1_TXEIE;

	// write requested amount
	for (int i = 0; i < size; i++)
	{
		// wait for the transmit buffer to become empty
		while (!(USART2->SR & USART_SR_TXE))
			__WFE(); // suspend until event

		// begin transmission of current character
		USART2->DR = data[i];

		// interrupt needs to be cleared manually
		NVIC_ClearPendingIRQ(USART2_IRQn);
	}

	// clear eveything
	USART2->CR1 &= ~USART_CR1_TXEIE;
	USART2->SR  &= ~USART_SR_TXE;
	NVIC_ClearPendingIRQ(USART2_IRQn);

#ifdef FREERTOS
	xTaskResumeAll(); // FreeRTOS: allow context switch
#endif

	return size;
}

// _read() syscall redirect to UART2
int _read(int fd, char *data, int size)
{
	int i = 0;

	// only read from stdin
	if (fd != 0)
		return -1;

#ifdef FREERTOS
	vTaskSuspendAll(); // FreeRTOS: no context switch
#endif

	// enable RXNE event
	USART2->CR1 |= USART_CR1_RXNEIE;

	// read as many characters as possible but
	// only wait ~1ms when nothing is coming in
	while (i < size)
	{
		// wait for a single byte to be received
		while (!(USART2->SR & USART_SR_RXNE) && !(TIM11->SR & TIM_SR_UIF))
			__WFE(); // suspend until event

		// timeout
		if (TIM11->SR & TIM_SR_UIF)
			break;

		// read buffered byte
		data[i++] = USART2->DR;

		// done
		if (i >= size)
			break;

		// clear interrupt
		NVIC_ClearPendingIRQ(USART2_IRQn);

		// start timeout
		TIM11->CNT  = 0;
		TIM11->CR1 |= TIM_CR1_CEN;
	}

	// clear USART
	USART2->CR1 &= ~USART_CR1_RXNEIE;
	USART2->SR  &= ~USART_SR_RXNE;
	NVIC_ClearPendingIRQ(USART2_IRQn);

	// clear timer 11
	TIM11->CR1  &= ~TIM_CR1_CEN;
	TIM11->SR   &= ~TIM_SR_UIF;
	NVIC_ClearPendingIRQ(TIM1_TRG_COM_TIM11_IRQn);

#ifdef FREERTOS
	xTaskResumeAll(); // FreeRTOS: allow context switch
#endif

	return i;
}

// UART initialization, run before main()
static __attribute__((constructor)) void init()
{
	// enable GPIOA clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	// enable USART2 clock
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

	// enable timer 11 clock
	RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;

	// set PA2 & PA3 alternate functions to AF7
	bits_val(GPIOA->AFR[0], 4, 2, 7); // PA2 -> USART2_TX
	bits_val(GPIOA->AFR[0], 4, 3, 7); // PA3 -> USART2_RX
	bits_val(GPIOA->MODER , 2, 2, 2); // PA2 -> AF mode
	bits_val(GPIOA->MODER , 2, 3, 2); // PA3 -> AF mode

	// configure UART as 8N1
	USART2->BRR  = baud(115200);
	USART2->CR1 |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;

	// configure timer 11 (upcount mode)
	TIM11->PSC   = 1 << 8;
	TIM11->ARR   = 1 << 12;
	TIM11->DIER |= TIM_DIER_UIE; // enable interrupt
	TIM11->CR1  |= TIM_CR1_URS;  // set counter overflow as only event source

	// allow events to wake the MCU
	SCB->SCR |= SCB_SCR_SEVONPEND_Msk;
}

#else

// _write() syscall stub that can be overridden
int __attribute__((weak)) _write(int fd, char *data, int size)
{
	return -1;
}

// _read() syscall stub that can be overridden
int __attribute__((weak)) _read(int fd, char *data, int size)
{
	return -1;
}

#endif // ENABLE_SERIAL == 1
