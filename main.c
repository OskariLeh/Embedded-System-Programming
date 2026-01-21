#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <bitwise.h>
#include <stm32f4xx.h>

#define MODE_CONFIG    0U
#define MODE_IDLE      1U
#define MODE_MODULATE  2U

#define baud(bps) \
	(((SystemCoreClock/((bps)*16)) << 4) | ((SystemCoreClock/(bps)) % 16))


// Weak default; if another module defines mode, it overrides this.
__attribute__((weak)) volatile uint8_t mode = MODE_MODULATE;

// ---------------------- Timing ----------------------
#define TICK_HZ             1000U           // 1 ms system tick
#define CONTROLLER_DIV      10U             // controller every 10 ms
#define CONTROL_HZ          (TICK_HZ / CONTROLLER_DIV)
static const float TS_CONTROL = (1.0f / (float)CONTROL_HZ);

#define MODEL_HZ            50000U          // converter model sampling rate per spec
#if (MODEL_HZ % TICK_HZ) != 0
#error "MODEL_HZ must be an integer multiple of TICK_HZ"
#endif
#define MODEL_STEPS_PER_TICK (MODEL_HZ / TICK_HZ)

#define PWM_HZ              100000U           // LED brightness carrier frequency

#define CONTROLLER_U_MIN     0.0f
#define CONTROLLER_U_MAX     3.0f
#define DEFAULT_REF_UOUT     1.5f

static volatile uint32_t g_ms = 0;          // millisecond timebase (from TIM2 IRQ)

// ---------------------- Scheduler flags ----------------------
volatile uint8_t flag_MODEL = 0;
volatile uint8_t flag_CONTROLLER = 0;

// ---------------------- UI semaphore / lockouts ----------------------
// UART takes semaphore while in CONFIG mode requested via console.
// Buttons do nothing while UART semaphore reserved.
static volatile uint8_t ui_sem_uart = 0;

// Optional timed lockout (recommended in pdf as a possible enhancement):
// after button action, prevent UART mode/value changes for 5 seconds.
static volatile uint32_t ui_uart_lock_until_ms = 0;

// ---------------------- PI Controller ----------------------
// Discrete PI controller with trapezoidal integration and anti-windup
typedef struct
{
    float kp;           // Proportional gain
    float ki;           // Integral gain
    float Ts;           // Sampling period [s]
    float integrator;   // Integrator state
    float prev_error;   // Previous error (for trapezoidal integration)
    float u_min;        // Output lower saturation limit
    float u_max;        // Output upper saturation limit
} PIController;

static float clampf(float value, float min_value, float max_value);
static void pi_init(PIController *pi, float kp, float ki, float Ts, float u_min, float u_max);
static float pi_step(PIController *pi, float reference, float measurement);
void pi_set_kp(float kp);
void pi_set_ki(float ki);

static void pwm_init(void);
static void pwm_set_duty(PIController *pi, float duty);
static void controller_subsystem_init(void);

// Global controller state
static PIController g_pi;
volatile float controller_out = 0.0f;
volatile float reference_uout = DEFAULT_REF_UOUT;
static uint32_t pwm_period = 0; // ARR+1 snapshot
volatile float g_uout = 0.0f;
volatile float g_pwm_duty = 0.0f;

// ---------------------- Converter model ----------------------
// Discrete state-space model: x[k+1] = A*x[k] + B*u[k]
// State vector x = [i1, u1, i2, u2, i3, u3]^T
// Output of interest: u3 = x[5] (voltage at third stage)
static float x_state[6] = {0};

static const float A[6][6] = {
    {0.9652, -0.0172,  0.0057, -0.0058,  0.0052, -0.0251},
    {0.7732,  0.1252,  0.2315, 0.0700,  0.1282,  0.7754},
    {0.8278, -0.7522, -0.0956,  0.3299, -0.4855,  0.3915},
    {0.9948,  0.2655, -0.3848,  0.4212,  0.3927,  0.2899},
    {0.7648, -0.4165, -0.4855, -0.3366, -0.0986,  0.7281},
    {1.1056,  0.7587,  0.1179,  0.0748, -0.2192,  0.1491}
};

static const float B[6] = {0.0471, 0.0377, 0.0404, 0.0485, 0.0373, 0.0539};

// Advance the converter model by one time step
// u_in: control input (clamped to valid range before calling)
void model_update(float u_in)
{
    float x_next[6];
    for (int i = 0; i < 6; ++i)
    {
        float s = 0.0f;
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

// Returns the model output voltage u3 (used as feedback for PI controller)
// Weak attribute allows override for unit testing
__attribute__((weak)) float model_output_u3(void)
{
    return x_state[5];
}

// ---------------------- UART (USART2) non-blocking: IRQ + ring buffers ----------------------
// Serial console interface at 115200 baud. Commands (newline-terminated):
//   help                              - Show available commands
//   status                            - Print current system state
//   mode <config|idle|mod|0|1|2>      - Change operating mode
//   kp <float>                        - Set Kp (config mode only)
//   ki <float>                        - Set Ki (config mode only)
//   ref <float>                       - Set output reference (mod mode only)

#define UART_RX_BUF_SZ 128U
#define UART_TX_BUF_SZ 256U

static volatile uint8_t  uart_rx_buf[UART_RX_BUF_SZ];
static volatile uint16_t uart_rx_head = 0, uart_rx_tail = 0;

static volatile uint8_t  uart_tx_buf[UART_TX_BUF_SZ];
static volatile uint16_t uart_tx_head = 0, uart_tx_tail = 0;

static inline uint16_t rb_next(uint16_t v, uint16_t sz) { return (uint16_t)((v + 1U) % sz); }

static void uart_init(void)
{
    // Enable USART2 clock
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // PA2, PA3 -> AF7 USART2
    bits_val(GPIOA->AFR[0], 4, 2, 7);
    bits_val(GPIOA->AFR[0], 4, 3, 7);
    bits_val(GPIOA->MODER , 2, 2, 2);
    bits_val(GPIOA->MODER , 2, 3, 2);

    USART2->BRR = baud(115200);
    USART2->CR1 = 0;
    USART2->CR1 |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;
    USART2->CR1 |= USART_CR1_RXNEIE; // RX interrupt

    NVIC_EnableIRQ(USART2_IRQn);
}

static void uart_tx_kick(void)
{
    USART2->CR1 |= USART_CR1_TXEIE; // enable TXE interrupt
}

static void uart_write_char(char c)
{
    uint16_t next = rb_next(uart_tx_head, UART_TX_BUF_SZ);
    if (next == uart_tx_tail) {
        // TX buffer full -> drop (rate limiting + small messages should avoid this)
        return;
    }
    uart_tx_buf[uart_tx_head] = (uint8_t)c;
    uart_tx_head = next;
    uart_tx_kick();
}

static void uart_write_str(const char *s)
{
    while (*s) uart_write_char(*s++);
}

static int uart_read_char_nonblocking(char *out)
{
    if (uart_rx_tail == uart_rx_head) return 0;
    *out = (char)uart_rx_buf[uart_rx_tail];
    uart_rx_tail = rb_next(uart_rx_tail, UART_RX_BUF_SZ);
    return 1;
}

void USART2_IRQHandler(void)
{
    // RX
    if (USART2->SR & USART_SR_RXNE)
    {
        uint8_t c = (uint8_t)USART2->DR;
        uint16_t next = rb_next(uart_rx_head, UART_RX_BUF_SZ);
        if (next != uart_rx_tail) {
            uart_rx_buf[uart_rx_head] = c;
            uart_rx_head = next;
        }
    }

    // TX
    if ((USART2->SR & USART_SR_TXE) && (USART2->CR1 & USART_CR1_TXEIE))
    {
        if (uart_tx_tail == uart_tx_head) {
            USART2->CR1 &= ~USART_CR1_TXEIE; // nothing to send
        } else {
            USART2->DR = uart_tx_buf[uart_tx_tail];
            uart_tx_tail = rb_next(uart_tx_tail, UART_TX_BUF_SZ);
        }
    }
}

// ---------------------- Buttons (EXTI) ----------------------
// Button flags set by EXTI ISR, cleared by main loop after processing
#define BTN1_PC5 (1U << 0)  // Cycle through modes: CONFIG -> IDLE -> MOD
#define BTN2_PC6 (1U << 1)  // Toggle selected parameter (Kp/Ki) in CONFIG mode
#define BTN3_PC8 (1U << 2)  // Decrease selected parameter or reference
#define BTN4_PC9 (1U << 3)  // Increase selected parameter or reference

static volatile uint8_t g_btn_flags = 0;

static void buttons_exti_init(void)
{
    // Need SYSCFG for EXTI mapping
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Map EXTI lines to Port C
    // EXTI5,6 in EXTICR[1], EXTI8,9 in EXTICR[2]
    SYSCFG->EXTICR[1] &= ~((0xFU << 4) | (0xFU << 8));  // lines 5,6
    SYSCFG->EXTICR[1] |=  ((0x2U << 4) | (0x2U << 8));  // port C = 0b0010
    SYSCFG->EXTICR[2] &= ~((0xFU << 0) | (0xFU << 4));  // lines 8,9
    SYSCFG->EXTICR[2] |=  ((0x2U << 0) | (0x2U << 4));

    // Unmask EXTI lines
    EXTI->IMR  |= (1U << 5) | (1U << 6) | (1U << 8) | (1U << 9);

    // Falling edge trigger (buttons pull pin low when pressed; you configured pull-ups)
    EXTI->RTSR &= ~((1U << 5) | (1U << 6) | (1U << 8) | (1U << 9));
    EXTI->FTSR |=  (1U << 5) | (1U << 6) | (1U << 8) | (1U << 9);

    // Clear pending
    EXTI->PR = (1U << 5) | (1U << 6) | (1U << 8) | (1U << 9);

    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void EXTI9_5_IRQHandler(void)
{
    uint32_t pr = EXTI->PR;

    if (pr & (1U << 5)) { EXTI->PR = (1U << 5); g_btn_flags |= BTN1_PC5; }
    if (pr & (1U << 6)) { EXTI->PR = (1U << 6); g_btn_flags |= BTN2_PC6; }
    if (pr & (1U << 8)) { EXTI->PR = (1U << 8); g_btn_flags |= BTN3_PC8; }
    if (pr & (1U << 9)) { EXTI->PR = (1U << 9); g_btn_flags |= BTN4_PC9; }
}

// ---------------------- Helpers ----------------------
static float clampf(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

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

static float pi_step(PIController *pi, float reference, float measurement)
{
    if (!pi) return 0.0f;
    float error = reference - measurement;

    // trapezoidal integration
    pi->integrator += 0.5f * pi->ki * pi->Ts * (error + pi->prev_error);

    float u = (pi->kp * error) + pi->integrator;
    float u_sat = clampf(u, pi->u_min, pi->u_max);

    // anti-windup back-calc
    if (u_sat != u)
    {
        pi->integrator = u_sat - (pi->kp * error);
    }

    pi->prev_error = error;
    return u_sat;
}

void pi_set_kp(float kp) { g_pi.kp = kp; }
void pi_set_ki(float ki) { g_pi.ki = ki; }

// ---------------------- PWM on PB5 / TIM3_CH2 ----------------------
static void pwm_init(void)
{
    // configure PB5 as TIM3_CH2 (RGB LED brightness channel)
    GPIOB->MODER &= ~(3U << (5U * 2U));
    GPIOB->MODER |=  (2U << (5U * 2U));
    GPIOB->AFR[0] &= ~(0xFU << (5U * 4U));
    GPIOB->AFR[0] |=  (2U << (5U * 4U));   // AF2 TIM3
    GPIOB->OTYPER &= ~(1U << 5);
    GPIOB->OSPEEDR |= (3U << (5U * 2U));
    GPIOB->PUPDR &= ~(3U << (5U * 2U));

    uint32_t tim_clk = SystemCoreClock;
    uint32_t ppre1 = (RCC->CFGR >> 10) & 0x7U;
    if (ppre1 >= 4U) tim_clk *= 2U; // TIMx clock doubles when APB prescaler != 1

    const uint32_t target_cnt = 1000000U; // 1 MHz carrier clock
    uint32_t psc = tim_clk / target_cnt;
    if (psc == 0U) psc = 1U;
    psc -= 1U;

    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM3->PSC = psc;
    TIM3->ARR = (target_cnt / PWM_HZ) - 1U;
    TIM3->CNT = 0U;

    // PWM mode 1 on CH2
    TIM3->CCMR1 &= ~(TIM_CCMR1_CC2S | TIM_CCMR1_OC2M);
    TIM3->CCMR1 |=  (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);
    TIM3->CCMR1 |=  TIM_CCMR1_OC2PE;

    TIM3->CCER &= ~TIM_CCER_CC2P;
    TIM3->CCER |=  TIM_CCER_CC2E;

    TIM3->CR1 |= TIM_CR1_ARPE;
    TIM3->EGR |= TIM_EGR_UG;

    pwm_period = (TIM3->ARR + 1U);
    TIM3->CCR2 = 0;
    TIM3->CR1 |= TIM_CR1_CEN;
}

static void pwm_set_duty(PIController *pi, float control_signal)
{
    if (!pi) return;

    float limited = clampf(control_signal, pi->u_min, pi->u_max);
    float span = pi->u_max - pi->u_min;
    float duty = (span > 0.0f) ? ((limited - pi->u_min) / span) : 0.0f;
    duty = clampf(duty, 0.0f, 1.0f);

    uint32_t period = (TIM3->ARR + 1U);
    if (period != pwm_period) pwm_period = period;

    if (pwm_period == 0U)
    {
        TIM3->CCR2 = 0;
        g_pwm_duty = duty;
        return;
    }

    if (duty <= 0.0f)
    {
        TIM3->CCR2 = 0;
        g_pwm_duty = 0.0f;
        return;
    }
    if (duty >= 1.0f)
    {
        TIM3->CCR2 = pwm_period;
        g_pwm_duty = 1.0f;
        return; // CCR>ARR => always high
    }

    uint32_t ccr = (uint32_t)(duty * (float)pwm_period + 0.5f);
    if (ccr > pwm_period) ccr = pwm_period;
    TIM3->CCR2 = ccr;
    g_pwm_duty = duty;
}

static void controller_subsystem_init(void)
{
    pwm_init();
    pi_init(&g_pi, 0.1f, 0.1f, TS_CONTROL, CONTROLLER_U_MIN, CONTROLLER_U_MAX);
    controller_out = 0.0f;
    reference_uout = clampf(reference_uout, g_pi.u_min, g_pi.u_max);
    pwm_set_duty(&g_pi, 0.0f);
}

// ---------------------- GPIO / LEDs ----------------------
void System_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
}

void Gpio_init(void)
{
    // buttons PC9, PC8, PC6, PC5 input + pull-up
    GPIOC->MODER &= ~(3U << (9U * 2U));
    GPIOC->MODER &= ~(3U << (8U * 2U));
    GPIOC->MODER &= ~(3U << (6U * 2U));
    GPIOC->MODER &= ~(3U << (5U * 2U));

    GPIOC->PUPDR &= ~(3U << (9U * 2U));
    GPIOC->PUPDR &= ~(3U << (8U * 2U));
    GPIOC->PUPDR &= ~(3U << (6U * 2U));
    GPIOC->PUPDR &= ~(3U << (5U * 2U));

    GPIOC->PUPDR |= (1U << (9U * 2U));
    GPIOC->PUPDR |= (1U << (8U * 2U));
    GPIOC->PUPDR |= (1U << (6U * 2U));
    GPIOC->PUPDR |= (1U << (5U * 2U));

    // leds PB10, PB4 output (PB5 configured for PWM in pwm_init)
    GPIOB->MODER &= ~(3U << (10U * 2U));
    GPIOB->MODER &= ~(3U << (4U * 2U));

    GPIOB->MODER |= (1U << (10U * 2U));
    GPIOB->MODER |= (1U << (4U * 2U));
}

static inline void led_pb10_set(int on) { if (on) GPIOB->ODR |= (1U<<10); else GPIOB->ODR &= ~(1U<<10); }
static inline void led_pb4_set (int on) { if (on) GPIOB->ODR |= (1U<<4 ); else GPIOB->ODR &= ~(1U<<4 ); }

// Update LED indicators based on current operating mode:
// - CONFIG:   PB10 blinks at 2 Hz (250 ms on/off)
// - IDLE:     PB4 blinks at 1 Hz (500 ms on/off)
// - MODULATE: PB10/PB4 off, PB5 (PWM) brightness reflects controller output
static void ui_update_leds(void)
{
    if (mode == MODE_CONFIG) {
        led_pb4_set(0);
        // 2 Hz blink -> 250 ms on / 250 ms off
        led_pb10_set(((g_ms / 250U) % 2U) ? 1 : 0);
    } else if (mode == MODE_IDLE) {
        led_pb10_set(0);
        // 1 Hz blink -> 500 ms on / 500 ms off
        led_pb4_set(((g_ms / 500U) % 2U) ? 1 : 0);
    } else { // MODE_MODULATE
        led_pb10_set(0);
        led_pb4_set(0);
        // PWM hardware on PB5 (TIM3_CH2) already reflects controller_out
    }
}

// ---------------------- TIM2: System tick (1 kHz) ----------------------
// Generates 1 ms interrupts for timing, model updates, and controller scheduling
void Timer2_config(void)
{
    // Configure TIM2 update to 1 kHz (1ms). Timer clock depends on APB1 prescaler.
    uint32_t tim_clk = SystemCoreClock;
    uint32_t ppre1 = (RCC->CFGR >> 10) & 0x7U;
    if (ppre1 >= 4U) tim_clk *= 2U; // TIMx clock doubles when APB prescaler != 1

    const uint32_t target_cnt = 1000000U; // 1 MHz counter clock
    uint32_t psc = (tim_clk / target_cnt);
    if (psc == 0U) psc = 1U;
    psc -= 1U;

    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM2->PSC = psc;
    TIM2->ARR = (target_cnt / TICK_HZ) - 1U; // 999
    TIM2->CNT = 0;

    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->CR1 |= TIM_CR1_URS;
    TIM2->EGR |= TIM_EGR_UG;
    TIM2->SR = 0;

    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 |= TIM_CR1_CEN;
}

void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF)
    {
        TIM2->SR &= ~TIM_SR_UIF;

        g_ms++;

        static uint32_t tick = 0;
        tick++;

        flag_MODEL = 1;
        if ((tick % CONTROLLER_DIV) == 0U) flag_CONTROLLER = 1;
    }
}

// ---------------------- UART command handling ----------------------
static void print_status(void)
{
    char buf[160];
    float uout = model_output_u3();
    const char *m = (mode==MODE_CONFIG) ? "config" : (mode==MODE_IDLE) ? "idle" : "mod";
    int n = snprintf(buf, sizeof(buf),
        "mode=%s sem_uart=%u ref=%.3f kp=%.3f ki=%.3f uout=%.3f duty=%.3f\r\n",
        m, (unsigned)ui_sem_uart, reference_uout, g_pi.kp, g_pi.ki, uout, g_pwm_duty);
    if (n > 0) uart_write_str(buf);
}

static void print_help(void)
{
    uart_write_str(
        "Commands:\r\n"
        "  help\r\n"
        "  status\r\n"
        "  mode <config|idle|mod|0|1|2>\r\n"
        "  kp <float>      (config mode)\r\n"
        "  ki <float>      (config mode)\r\n"
        "  ref <float>     (mod mode)\r\n");

    char buf[64];
    int n = snprintf(buf, sizeof(buf), "    ref range: %.1f..%.1f\r\n", g_pi.u_min, g_pi.u_max);
    if (n > 0) uart_write_str(buf);
}

static uint8_t parse_mode_token(const char *t, uint8_t *out_mode)
{
    if (!t || !out_mode) return 0;
    if (!strcmp(t, "0") || !strcmp(t, "config")) { *out_mode = MODE_CONFIG; return 1; }
    if (!strcmp(t, "1") || !strcmp(t, "idle"))   { *out_mode = MODE_IDLE; return 1; }
    if (!strcmp(t, "2") || !strcmp(t, "mod") || !strcmp(t, "modulate")) { *out_mode = MODE_MODULATE; return 1; }
    return 0;
}

static void handle_command(char *line)
{
    // optional timed lockout: if button recently acted, ignore UART changes
    if (g_ms < ui_uart_lock_until_ms) {
        uart_write_str("UART locked by button (wait)\r\n");
        return;
    }

    // trim leading spaces
    while (*line == ' ' || *line == '\t') line++;

    if (*line == 0) return;

    if (!strcmp(line, "help")) { print_help(); return; }
    if (!strcmp(line, "status")) { print_status(); return; }

    // tokenization (in-place)
    char *cmd = strtok(line, " \t");
    char *arg = strtok(NULL, " \t");

    if (!cmd) return;

    if (!strcmp(cmd, "mode"))
    {
        uint8_t new_mode;
        if (!arg || !parse_mode_token(arg, &new_mode)) {
            uart_write_str("ERR: mode <config|idle|mod|0|1|2>\r\n");
            return;
        }

        mode = new_mode;

        // Take/release semaphore based on new mode
        if (mode == MODE_CONFIG) ui_sem_uart = 1;
        else ui_sem_uart = 0;

        uart_write_str("OK\r\n");
        print_status();
        return;
    }

    if (!strcmp(cmd, "kp"))
    {
        if (mode != MODE_CONFIG) { uart_write_str("ERR: kp only in config\r\n"); return; }
        if (!arg) { uart_write_str("ERR: kp <float>\r\n"); return; }
        float v = strtof(arg, NULL);
        pi_set_kp(v);
        uart_write_str("OK\r\n");
        print_status();
        return;
    }

    if (!strcmp(cmd, "ki"))
    {
        if (mode != MODE_CONFIG) { uart_write_str("ERR: ki only in config\r\n"); return; }
        if (!arg) { uart_write_str("ERR: ki <float>\r\n"); return; }
        float v = strtof(arg, NULL);
        pi_set_ki(v);
        uart_write_str("OK\r\n");
        print_status();
        return;
    }

    if (!strcmp(cmd, "ref"))
    {
        if (mode != MODE_MODULATE) { uart_write_str("ERR: ref only in modulating\r\n"); return; }
        if (!arg) { uart_write_str("ERR: ref <float>\r\n"); return; }
        float v = strtof(arg, NULL);
        float clamped = clampf(v, g_pi.u_min, g_pi.u_max);
        reference_uout = clamped;
        uart_write_str("OK\r\n");
        if (clamped != v) uart_write_str("WARN: ref limited to controller range\r\n");
        print_status();
        return;
    }

    uart_write_str("ERR: unknown command (type 'help')\r\n");
}

// ---------------------- Button handling ----------------------
static uint8_t selected_param = 0; // 0=Kp, 1=Ki

static void handle_buttons(void)
{
    uint8_t flags = g_btn_flags;
    if (!flags) return;
    g_btn_flags = 0;

    // Ignore buttons while UART holds the semaphore (is in CONFIG mode)
    if (ui_sem_uart) return;

    // BTN1: cycle mode
    if (flags & BTN1_PC5)
    {
        if (mode == MODE_CONFIG) mode = MODE_IDLE;
        else if (mode == MODE_IDLE) mode = MODE_MODULATE;
        else mode = MODE_CONFIG;

        // If button changes mode, lock UART mode/value changes for 5 seconds (optional enhancement)
        ui_uart_lock_until_ms = g_ms + 5000U;

        // If entering CONFIG via button, UART semaphore is not taken (only console-taking required).
        // If leaving CONFIG, ensure semaphore is released.
        if (mode != MODE_CONFIG) ui_sem_uart = 0;

        uart_write_str("BTN mode change\r\n");
        print_status();
    }

    // BTN2: in config, select parameter Kp/Ki
    if ((flags & BTN2_PC6) && (mode == MODE_CONFIG))
    {
        selected_param ^= 1U;
        uart_write_str(selected_param ? "Selected: Ki\r\n" : "Selected: Kp\r\n");
    }

    // BTN3/BTN4: decrease/increase
    if (flags & (BTN3_PC8 | BTN4_PC9))
    {
        int inc = (flags & BTN4_PC9) ? 1 : (flags & BTN3_PC8) ? -1 : 0;

        if (mode == MODE_CONFIG)
        {
            if (selected_param == 0) {
                g_pi.kp += inc * 0.01f;
                if (g_pi.kp < 0.0f) g_pi.kp = 0.0f;
            } else {
                g_pi.ki += inc * 1.0f;
                if (g_pi.ki < 0.0f) g_pi.ki = 0.0f;
            }
            uart_write_str("BTN param change\r\n");
            print_status();
        }
        else if (mode == MODE_MODULATE)
        {
            reference_uout += inc * 0.5f;
            reference_uout = clampf(reference_uout, g_pi.u_min, g_pi.u_max);
            uart_write_str("BTN ref change\r\n");
            print_status();
        }
    }
}

// ---------------------- main ----------------------
int main(void)
{
    System_init();
    Gpio_init();

    uart_init();
    buttons_exti_init();

    Timer2_config();
    controller_subsystem_init();

    uart_write_str("READY\r\n");
    print_help();
    print_status();

    // UART line accumulator
    static char cmd_buf[96];
    static uint32_t cmd_len = 0;

    uint32_t last_status_ms = 0;

    while (1)
    {
        // periodic LED updates (no delays)
        ui_update_leds();

        // Handle button events (from EXTI IRQ)
        handle_buttons();

        // UART: consume RX and form lines
        char c;
        while (uart_read_char_nonblocking(&c))
        {
            if (c == '\r' || c == '\n')
            {
                if (cmd_len > 0)
                {
                    cmd_buf[cmd_len] = 0;
                    handle_command(cmd_buf);
                    cmd_len = 0;
                }
            }
            else
            {
                if (cmd_len < (sizeof(cmd_buf) - 1U))
                {
                    cmd_buf[cmd_len++] = c;
                }
                else
                {
                    // overflow: reset line
                    cmd_len = 0;
                    uart_write_str("ERR: line too long\r\n");
                }
            }
        }

        // Model update (every 1 ms)
        if (flag_MODEL)
        {
            flag_MODEL = 0;
            float u_in = (mode == MODE_MODULATE) ? controller_out : 0.0f;
            u_in = clampf(u_in, g_pi.u_min, g_pi.u_max);
            for (uint32_t i = 0; i < MODEL_STEPS_PER_TICK; ++i)
            {
                model_update(u_in);
            }
        }

        // Controller update (every 10 ms)
        if (flag_CONTROLLER)
        {
            flag_CONTROLLER = 0;

            float meas = model_output_u3();
            g_uout = meas;
            if (mode == MODE_MODULATE)
            {
                controller_out = pi_step(&g_pi, reference_uout, meas);
                pwm_set_duty(&g_pi, controller_out);
            }
            else
            {
                controller_out = 0.0f;
                pwm_set_duty(&g_pi, 0.0f);
            }
        }

        // Periodic status output every 500 ms
        if ((g_ms - last_status_ms) >= 500U)
        {
            last_status_ms = g_ms;
            print_status();
        }
    }
}
