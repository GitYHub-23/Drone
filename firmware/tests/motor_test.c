#include <stdint.h>

/* STM32F0 register addresses */
#define RCC_AHBENR     (*(volatile uint32_t *)0x40021014U)

#define GPIOA_MODER    (*(volatile uint32_t *)0x48000000U)
#define GPIOA_BSRR     (*(volatile uint32_t *)0x48000018U)

#define SYST_CSR       (*(volatile uint32_t *)0xE000E010U)
#define SYST_RVR       (*(volatile uint32_t *)0xE000E014U)
#define SYST_CVR       (*(volatile uint32_t *)0xE000E018U)

/* GPIOA clock enable */
#define GPIOA_EN       (1U << 17)

static void gpio_output(uint32_t pin)
{
    GPIOA_MODER &= ~(3U << (pin * 2U));
    GPIOA_MODER |=  (1U << (pin * 2U));
}

static void pin_high(uint32_t pin)
{
    GPIOA_BSRR = (1U << pin);
}

static void pin_low(uint32_t pin)
{
    GPIOA_BSRR = (1U << (pin + 16U));
}

static void systick_init(void)
{
    /*
     * STM32 starts from the internal ~8 MHz HSI clock.
     * 8000 ticks ≈ 1 ms.
     */
    SYST_CSR = 0;
    SYST_RVR = 7999U;
    SYST_CVR = 0;
    SYST_CSR = (1U << 2) | (1U << 0);
}

static void delay_ms(uint32_t ms)
{
    while (ms--)
    {
        while ((SYST_CSR & (1U << 16)) == 0)
        {
        }
    }
}

static void motor_test(uint32_t pin)
{
    /*
     * About 20% software PWM:
     * 2 ms ON, 8 ms OFF.
     * 50 cycles ≈ 0.5 second.
     */
    for (uint32_t i = 0; i < 50; i++)
    {
        pin_high(pin);
        delay_ms(2);

        pin_low(pin);
        delay_ms(8);
    }

    pin_low(pin);
}

int main(void)
{
    /* Enable GPIOA */
    RCC_AHBENR |= GPIOA_EN;

    /*
     * Possible regulator-enable pin on this board family.
     * Prepare PA1 HIGH.
     */
    pin_high(1);

    /*
     * Keep all possible motor outputs LOW
     * before changing them to outputs.
     */
    pin_low(8);
    pin_low(9);
    pin_low(10);
    pin_low(11);

    gpio_output(1);
    gpio_output(8);
    gpio_output(9);
    gpio_output(10);
    gpio_output(11);

    systick_init();

    /* Give one second after startup */
    delay_ms(1000);

    /* FIRST TEST: PA8 only */
    motor_test(8);

    /* Everything remains OFF */
    while (1)
    {
        pin_low(8);
        pin_low(9);
        pin_low(10);
        pin_low(11);
    }
}
