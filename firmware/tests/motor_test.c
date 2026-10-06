#include <stdint.h>

/* ============================================================
   STM32F031K4 - MOTOR DIRECTION TEST
   PROPELLERS MUST BE REMOVED

   Confirmed mapping:
     M1 = PA11 = TIM1_CH4 = FRONT-RIGHT
     M2 = PA9  = TIM1_CH2 = FRONT-LEFT
     M3 = PA8  = TIM1_CH1 = REAR-LEFT
     M4 = PA10 = TIM1_CH3 = REAR-RIGHT

   CPU clock: 8 MHz HSI
   PWM: 1 kHz

   Sequence:
     wait 5 s
     M1
     pause
     M2
     pause
     M3
     pause
     M4
     pause
     repeat
   ============================================================ */

#define REG32(addr) (*(volatile uint32_t *)(addr))

/* ================= RCC ================= */

#define RCC_AHBENR      REG32(0x40021014U)
#define RCC_APB2RSTR    REG32(0x4002100CU)
#define RCC_APB2ENR     REG32(0x40021018U)

#define GPIOA_EN        (1U << 17)
#define TIM1_EN         (1U << 11)

/* ================= GPIOA ================= */

#define GPIOA_BASE      0x48000000U

#define GPIOA_MODER     REG32(GPIOA_BASE + 0x00U)
#define GPIOA_OTYPER    REG32(GPIOA_BASE + 0x04U)
#define GPIOA_OSPEEDR   REG32(GPIOA_BASE + 0x08U)
#define GPIOA_PUPDR     REG32(GPIOA_BASE + 0x0CU)
#define GPIOA_BSRR      REG32(GPIOA_BASE + 0x18U)
#define GPIOA_AFRH      REG32(GPIOA_BASE + 0x24U)

/* ================= TIM1 ================= */

#define TIM1_BASE       0x40012C00U

#define TIM1_CR1        REG32(TIM1_BASE + 0x00U)
#define TIM1_EGR        REG32(TIM1_BASE + 0x14U)
#define TIM1_CCMR1      REG32(TIM1_BASE + 0x18U)
#define TIM1_CCMR2      REG32(TIM1_BASE + 0x1CU)
#define TIM1_CCER       REG32(TIM1_BASE + 0x20U)
#define TIM1_PSC        REG32(TIM1_BASE + 0x28U)
#define TIM1_ARR        REG32(TIM1_BASE + 0x2CU)

#define TIM1_CCR1       REG32(TIM1_BASE + 0x34U)
#define TIM1_CCR2       REG32(TIM1_BASE + 0x38U)
#define TIM1_CCR3       REG32(TIM1_BASE + 0x3CU)
#define TIM1_CCR4       REG32(TIM1_BASE + 0x40U)

#define TIM1_BDTR       REG32(TIM1_BASE + 0x44U)

/* ================= SysTick ================= */

#define SYST_CSR        REG32(0xE000E010U)
#define SYST_RVR        REG32(0xE000E014U)
#define SYST_CVR        REG32(0xE000E018U)

/* ================= PWM ================= */

/*
   8 MHz / 8000 = 1000 Hz
*/
#define PWM_PERIOD_TICKS    8000U

#define START_PERCENT       45U
#define RUN_PERCENT         25U

#define START_TIME_MS       100U
#define RUN_TIME_MS         2000U
#define PAUSE_TIME_MS       2000U
#define INITIAL_WAIT_MS     5000U

/* ============================================================
   DEBUG VARIABLES
   Add these to Live Expressions
   ============================================================ */

volatile uint32_t system_ms = 0;

/*
   motor_test_stage:

   0 = initial wait
   1 = M1
   2 = M2
   3 = M3
   4 = M4
   5 = pause
*/

volatile uint32_t motor_test_stage = 0;
volatile uint32_t motor_test_cycle = 0;

volatile uint32_t motor1_percent = 0;
volatile uint32_t motor2_percent = 0;
volatile uint32_t motor3_percent = 0;
volatile uint32_t motor4_percent = 0;


/* ============================================================
   SysTick
   ============================================================ */

void SysTick_Handler(void)
{
    system_ms++;
}


static void systick_init(void)
{
    /*
       8 MHz / 8000 = 1000 Hz
       = 1 ms interrupt
    */

    SYST_RVR = 7999U;
    SYST_CVR = 0U;

    /*
       bit 0 ENABLE
       bit 1 TICKINT
       bit 2 CLKSOURCE = CPU
    */

    SYST_CSR = 7U;
}


static void delay_ms(uint32_t ms)
{
    uint32_t start = system_ms;

    while ((uint32_t)(system_ms - start) < ms)
    {
        /* wait */
    }
}


/* ============================================================
   PA1 BOARD ENABLE
   ============================================================ */

static void board_enable_init(void)
{
    /*
       Set output value HIGH first
    */

    GPIOA_BSRR = (1U << 1);

    /*
       PA1 MODER = 01 = output
    */

    GPIOA_MODER &= ~(3U << (1U * 2U));
    GPIOA_MODER |=  (1U << (1U * 2U));

    /*
       Push-pull
    */

    GPIOA_OTYPER &= ~(1U << 1);
}


/* ============================================================
   MOTOR GPIO
   PA8  = AF2 = TIM1_CH1 = M3
   PA9  = AF2 = TIM1_CH2 = M2
   PA10 = AF2 = TIM1_CH3 = M4
   PA11 = AF2 = TIM1_CH4 = M1
   ============================================================ */

static void motor_gpio_init(void)
{
    uint32_t pin;

    for (pin = 8U; pin <= 11U; pin++)
    {
        /*
           Alternate function mode = 10
        */

        GPIOA_MODER &= ~(3U << (pin * 2U));
        GPIOA_MODER |=  (2U << (pin * 2U));

        /*
           Push-pull
        */

        GPIOA_OTYPER &= ~(1U << pin);

        /*
           High speed = 11
        */

        GPIOA_OSPEEDR &= ~(3U << (pin * 2U));
        GPIOA_OSPEEDR |=  (3U << (pin * 2U));

        /*
           No pull-up / pull-down
        */

        GPIOA_PUPDR &= ~(3U << (pin * 2U));
    }

    /*
       AFRH:
       PA8  AF2
       PA9  AF2
       PA10 AF2
       PA11 AF2
    */

    GPIOA_AFRH &= ~(
          (0xFU << 0U)
        | (0xFU << 4U)
        | (0xFU << 8U)
        | (0xFU << 12U)
    );

    GPIOA_AFRH |= (
          (2U << 0U)
        | (2U << 4U)
        | (2U << 8U)
        | (2U << 12U)
    );
}


/* ============================================================
   TIM1
   ============================================================ */

static void tim1_init(void)
{
    /*
       Enable TIM1 clock
    */

    RCC_APB2ENR |= TIM1_EN;

    /*
       Reset TIM1
    */

    RCC_APB2RSTR |= TIM1_EN;
    RCC_APB2RSTR &= ~TIM1_EN;

    /*
       Timer clock = 8 MHz
       PSC = 0

       ARR = 7999

       PWM frequency:
       8,000,000 / 8000 = 1000 Hz
    */

    TIM1_PSC = 0U;
    TIM1_ARR = PWM_PERIOD_TICKS - 1U;

    /*
       Start with ALL motors OFF
    */

    TIM1_CCR1 = 0U;
    TIM1_CCR2 = 0U;
    TIM1_CCR3 = 0U;
    TIM1_CCR4 = 0U;

    /*
       PWM Mode 1

       CH1:
       OC1M = 110
       OC1PE = 1

       CH2:
       OC2M = 110
       OC2PE = 1
    */

    TIM1_CCMR1 =
          (6U << 4U)
        | (1U << 3U)
        | (6U << 12U)
        | (1U << 11U);

    /*
       CH3 + CH4
    */

    TIM1_CCMR2 =
          (6U << 4U)
        | (1U << 3U)
        | (6U << 12U)
        | (1U << 11U);

    /*
       Enable CH1-CH4 outputs
    */

    TIM1_CCER =
          (1U << 0U)
        | (1U << 4U)
        | (1U << 8U)
        | (1U << 12U);

    /*
       Main Output Enable
    */

    TIM1_BDTR = (1U << 15U);

    /*
       Update registers
    */

    TIM1_EGR = 1U;

    /*
       ARPE + Counter Enable
    */

    TIM1_CR1 =
          (1U << 7U)
        | (1U << 0U);
}


/* ============================================================
   MOTOR CONTROL
   ============================================================ */

static uint32_t percent_to_ccr(uint32_t percent)
{
    if (percent > 100U)
    {
        percent = 100U;
    }

    return (PWM_PERIOD_TICKS * percent) / 100U;
}


static void motors_off(void)
{
    TIM1_CCR1 = 0U;
    TIM1_CCR2 = 0U;
    TIM1_CCR3 = 0U;
    TIM1_CCR4 = 0U;

    motor1_percent = 0U;
    motor2_percent = 0U;
    motor3_percent = 0U;
    motor4_percent = 0U;
}


static void motor_set(uint32_t motor, uint32_t percent)
{
    uint32_t ccr;

    ccr = percent_to_ccr(percent);

    /*
       Always ensure every OTHER motor is OFF.
    */

    motors_off();

    switch (motor)
    {
        /*
           M1 = PA11 = TIM1_CH4
        */

        case 1U:
            TIM1_CCR4 = ccr;
            motor1_percent = percent;
            break;

        /*
           M2 = PA9 = TIM1_CH2
        */

        case 2U:
            TIM1_CCR2 = ccr;
            motor2_percent = percent;
            break;

        /*
           M3 = PA8 = TIM1_CH1
        */

        case 3U:
            TIM1_CCR1 = ccr;
            motor3_percent = percent;
            break;

        /*
           M4 = PA10 = TIM1_CH3
        */

        case 4U:
            TIM1_CCR3 = ccr;
            motor4_percent = percent;
            break;

        default:
            motors_off();
            break;
    }
}


/* ============================================================
   TEST ONE MOTOR
   ============================================================ */

static void test_motor(uint32_t motor)
{
    motor_test_stage = motor;

    /*
       Short startup boost
    */

    motor_set(motor, START_PERCENT);
    delay_ms(START_TIME_MS);

    /*
       Lower power for observing direction
    */

    motor_set(motor, RUN_PERCENT);
    delay_ms(RUN_TIME_MS);

    /*
       OFF
    */

    motors_off();

    motor_test_stage = 5U;

    delay_ms(PAUSE_TIME_MS);
}


/* ============================================================
   MAIN
   ============================================================ */

int main(void)
{
    /*
       Enable GPIOA clock
    */

    RCC_AHBENR |= GPIOA_EN;

    (void)RCC_AHBENR;

    /*
       PA1 board enable
    */

    board_enable_init();

    /*
       Prepare motor pins
    */

    motor_gpio_init();

    /*
       Start with motors OFF
    */

    motor1_percent = 0U;
    motor2_percent = 0U;
    motor3_percent = 0U;
    motor4_percent = 0U;

    /*
       1 ms system timer
    */

    systick_init();

    /*
       Hardware PWM
    */

    tim1_init();

    motors_off();

    /*
       Give plenty of time after startup.
    */

    motor_test_stage = 0U;

    delay_ms(INITIAL_WAIT_MS);

    /*
       Repeat forever:
       M1 -> M2 -> M3 -> M4
    */

    while (1)
    {
        motor_test_cycle++;

        test_motor(1U);
        test_motor(2U);
        test_motor(3U);
        test_motor(4U);

        motors_off();

        motor_test_stage = 0U;

        /*
           Long pause before next complete cycle
        */

        delay_ms(3000U);
    }
}
