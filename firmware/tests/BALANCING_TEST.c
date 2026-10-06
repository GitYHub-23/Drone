/*
 * ============================================================
 * DRONE_ - STEP 5
 * FIRST REAL CLOSED-LOOP MOTOR TEST
 *
 * STM32F031K4 + M540
 *
 * !!! REMOVE ALL 4 PROPELLERS !!!
 *
 * Sequence:
 *
 *   boot
 *    |
 *   gyro calibration
 *    |
 *   3 seconds motors OFF
 *    |
 *   120 ms startup kick @ 50%
 *    |
 *   5 seconds REAL RATE-CONTROL test @ ~35%
 *    |
 *   motors OFF permanently until RESET
 *
 * Any IMU error during test -> ALL MOTORS OFF.
 *
 *
 * Geometry:
 *
 *                  FRONT
 *                    ^
 *
 *             M2 CCW     M1 CW
 *             LEFT       RIGHT
 *
 *             M3 CW      M4 CCW
 *             LEFT       RIGHT
 *
 *
 * Electrical mapping:
 *
 *   PA8  / TIM1_CH1 -> M3
 *   PA9  / TIM1_CH2 -> M2
 *   PA10 / TIM1_CH3 -> M4
 *   PA11 / TIM1_CH4 -> M1
 *
 *
 * VERIFIED MIXER:
 *
 *   M1 = T + R + P + Y
 *   M2 = T - R + P - Y
 *   M3 = T - R - P + Y
 *   M4 = T + R - P - Y
 *
 * ============================================================
 */

#include <stdint.h>


/* ============================================================
 * REGISTER ACCESS
 * ============================================================ */

#define REG32(addr) (*(volatile uint32_t *)(uintptr_t)(addr))


/* ============================================================
 * RCC
 * ============================================================ */

#define RCC_AHBENR      REG32(0x40021014U)
#define RCC_APB2RSTR    REG32(0x4002100CU)
#define RCC_APB1RSTR    REG32(0x40021010U)
#define RCC_APB2ENR     REG32(0x40021018U)
#define RCC_APB1ENR     REG32(0x4002101CU)
#define RCC_CFGR3       REG32(0x40021030U)

#define GPIOA_EN        (1U << 17)
#define GPIOB_EN        (1U << 18)

#define TIM1_EN         (1U << 11)
#define I2C1_EN         (1U << 21)


/* ============================================================
 * GPIO
 * ============================================================ */

#define GPIOA_BASE      0x48000000U
#define GPIOB_BASE      0x48000400U

#define GPIO_MODER(base)    REG32((base) + 0x00U)
#define GPIO_OTYPER(base)   REG32((base) + 0x04U)
#define GPIO_OSPEEDR(base)  REG32((base) + 0x08U)
#define GPIO_PUPDR(base)    REG32((base) + 0x0CU)
#define GPIO_BSRR(base)     REG32((base) + 0x18U)
#define GPIO_AFRL(base)     REG32((base) + 0x20U)
#define GPIO_AFRH(base)     REG32((base) + 0x24U)


/* ============================================================
 * SYSTICK
 * ============================================================ */

#define SYST_CSR        REG32(0xE000E010U)
#define SYST_RVR        REG32(0xE000E014U)
#define SYST_CVR        REG32(0xE000E018U)


/* ============================================================
 * I2C1
 * ============================================================ */

#define I2C1_BASE       0x40005400U

#define I2C1_CR1        REG32(I2C1_BASE + 0x00U)
#define I2C1_CR2        REG32(I2C1_BASE + 0x04U)
#define I2C1_TIMINGR    REG32(I2C1_BASE + 0x10U)
#define I2C1_ISR        REG32(I2C1_BASE + 0x18U)
#define I2C1_ICR        REG32(I2C1_BASE + 0x1CU)
#define I2C1_RXDR       REG32(I2C1_BASE + 0x24U)
#define I2C1_TXDR       REG32(I2C1_BASE + 0x28U)

#define I2C_CR1_PE      (1U << 0)

#define I2C_CR2_RD_WRN  (1U << 10)
#define I2C_CR2_START   (1U << 13)
#define I2C_CR2_AUTOEND (1U << 25)

#define I2C_ISR_TXIS    (1U << 1)
#define I2C_ISR_RXNE    (1U << 2)
#define I2C_ISR_NACKF   (1U << 4)
#define I2C_ISR_STOPF   (1U << 5)
#define I2C_ISR_TC      (1U << 6)
#define I2C_ISR_BERR    (1U << 8)
#define I2C_ISR_ARLO    (1U << 9)
#define I2C_ISR_OVR     (1U << 10)
#define I2C_ISR_BUSY    (1U << 15)

#define I2C_ICR_NACKCF  (1U << 4)
#define I2C_ICR_STOPCF  (1U << 5)
#define I2C_ICR_BERRCF  (1U << 8)
#define I2C_ICR_ARLOCF  (1U << 9)
#define I2C_ICR_OVRCF   (1U << 10)


/* ============================================================
 * TIM1
 * ============================================================ */

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


/* ============================================================
 * M540
 * ============================================================ */

#define M540_ADDR           0x69U

#define M540_REG_ACCEL      0x3BU
#define M540_REG_CONFIG     0x1AU
#define M540_REG_GYRO_CFG   0x1BU
#define M540_REG_ACCEL_CFG  0x1CU
#define M540_REG_ACCEL_CFG2 0x1DU
#define M540_REG_PWR_MGMT   0x6BU
#define M540_REG_WHO_AM_I   0x75U

#define M540_WHO_VALUE      0x7DU


/* ============================================================
 * CONTROL PARAMETERS
 * ============================================================ */

#define LOOP_PERIOD_MS          5U

#define GYRO_LSB_PER_DPS        16.4f

/*
 * Deliberately weak gains.
 * This is NOT flight PID tuning.
 */

#define ROLL_KP                 0.040f
#define PITCH_KP                0.040f
#define YAW_KP                  0.030f

#define ROLL_PID_LIMIT          8.0f
#define PITCH_PID_LIMIT         8.0f
#define YAW_PID_LIMIT           6.0f


/* ============================================================
 * PHYSICAL TEST LIMITS
 * ============================================================ */

#define TEST_WAIT_MS            3000U

#define MOTOR_START_KICK_MS     120U
#define START_KICK_PERCENT      50.0f

#define ACTIVE_TEST_MS          5000U

#define TEST_BASE_PERCENT       35.0f

#define TEST_MIN_PERCENT        20.0f
#define TEST_MAX_PERCENT        50.0f


/* ============================================================
 * GLOBAL DEBUG VARIABLES
 * ============================================================ */

volatile uint32_t system_ms = 0U;


/* IMU */

volatile uint32_t imu_ok = 0U;

volatile uint8_t imu_whoami = 0U;

volatile uint32_t imu_frame_counter = 0U;
volatile uint32_t imu_error_counter = 0U;

volatile int16_t accel_x = 0;
volatile int16_t accel_y = 0;
volatile int16_t accel_z = 0;

volatile int16_t gyro_x = 0;
volatile int16_t gyro_y = 0;
volatile int16_t gyro_z = 0;


/* I2C */

volatile uint32_t i2c_hw_enabled = 0U;
volatile uint32_t i2c_last_error = 0U;
volatile uint32_t i2c_berr_counter = 0U;
volatile uint32_t i2c_transfer_counter = 0U;

volatile uint32_t i2c_isr_debug = 0U;
volatile uint32_t i2c_timingr_debug = 0U;


/* Calibration */

volatile uint32_t calibration_status = 0U;
volatile uint32_t calibration_samples = 0U;

volatile float gyro_x_offset = 0.0f;
volatile float gyro_y_offset = 0.0f;
volatile float gyro_z_offset = 0.0f;


/* Gyro deg/s */

volatile float gyro_roll_dps = 0.0f;
volatile float gyro_pitch_dps = 0.0f;
volatile float gyro_yaw_dps = 0.0f;


/* Controller */

volatile float pid_roll_output = 0.0f;
volatile float pid_pitch_output = 0.0f;
volatile float pid_yaw_output = 0.0f;


/* Mixer */

volatile float mix_m1_percent = 0.0f;
volatile float mix_m2_percent = 0.0f;
volatile float mix_m3_percent = 0.0f;
volatile float mix_m4_percent = 0.0f;


/* ACTUAL motor command */

volatile uint32_t motor1_percent = 0U;
volatile uint32_t motor2_percent = 0U;
volatile uint32_t motor3_percent = 0U;
volatile uint32_t motor4_percent = 0U;


/*
 * 1 = physical motors OFF
 * 0 = test currently allows PWM
 */

volatile uint32_t physical_motor_lock = 1U;


/*
 * motor_test_state:
 *
 * 0 = waiting 3 seconds
 * 1 = 120 ms startup kick
 * 2 = active 5-second PID/mixer test
 * 3 = test finished, motors OFF forever
 * 4 = FAULT, motors OFF forever
 */

volatile uint32_t motor_test_state = 0U;

volatile uint32_t motor_test_remaining_ms = 0U;


/*
 * fault:
 *
 * 0  = none
 * 1  = M540 init failure
 * 2  = gyro calibration failure
 * 3  = IMU read failure during test
 * 99 = internal state error
 */

volatile uint32_t motor_test_fault_code = 0U;


/* Loop */

volatile uint32_t loop_dt_ms = 0U;
volatile float loop_hz = 200.0f;


/* ============================================================
 * SMALL HELPERS
 * ============================================================ */

static float clampf_local(
    float x,
    float min_value,
    float max_value
)
{
    if (x < min_value)
    {
        return min_value;
    }

    if (x > max_value)
    {
        return max_value;
    }

    return x;
}


static uint32_t round_percent(float x)
{
    if (x <= 0.0f)
    {
        return 0U;
    }

    if (x >= 100.0f)
    {
        return 100U;
    }

    return (uint32_t)(x + 0.5f);
}


/* ============================================================
 * GPIO HELPERS
 * ============================================================ */

static void gpio_high(
    uint32_t base,
    uint32_t pin
)
{
    GPIO_BSRR(base) =
        (1U << pin);
}


static void gpio_low(
    uint32_t base,
    uint32_t pin
)
{
    GPIO_BSRR(base) =
        (1U << (pin + 16U));
}


static void gpio_output(
    uint32_t base,
    uint32_t pin
)
{
    uint32_t shift;

    shift = pin * 2U;

    GPIO_MODER(base) =
        (
            GPIO_MODER(base)
            &
            ~(3U << shift)
        )
        |
        (1U << shift);
}


/* ============================================================
 * SYSTICK
 * ============================================================ */

void SysTick_Handler(void)
{
    system_ms++;
}


static void systick_init(void)
{
    /*
     * CPU = 8 MHz
     *
     * 8000000 / 1000 = 8000
     */

    SYST_RVR = 7999U;

    SYST_CVR = 0U;

    /*
     * ENABLE
     * TICKINT
     * CLKSOURCE CPU
     */

    SYST_CSR = 0x07U;
}


static void delay_ms(uint32_t ms)
{
    uint32_t start;

    start = system_ms;

    while (
        (uint32_t)(
            system_ms -
            start
        )
        <
        ms
    )
    {
        /* wait */
    }
}


/* ============================================================
 * MOTOR GPIO SAFE STATE
 * ============================================================ */

static void motor_gpio_prepare_low(void)
{
    uint32_t pin;

    for (
        pin = 8U;
        pin <= 11U;
        pin++
    )
    {
        gpio_low(
            GPIOA_BASE,
            pin
        );

        gpio_output(
            GPIOA_BASE,
            pin
        );
    }
}


/* ============================================================
 * TIM1 INITIALIZATION
 * ============================================================ */

static void tim1_init(void)
{
    uint32_t afr;

    RCC_APB2ENR |= TIM1_EN;

    RCC_APB2RSTR |= TIM1_EN;
    RCC_APB2RSTR &= ~TIM1_EN;


    /*
     * PA8..PA11 -> Alternate Function
     */

    GPIO_MODER(GPIOA_BASE) &= ~(
        (3U << 16U) |
        (3U << 18U) |
        (3U << 20U) |
        (3U << 22U)
    );

    GPIO_MODER(GPIOA_BASE) |= (
        (2U << 16U) |
        (2U << 18U) |
        (2U << 20U) |
        (2U << 22U)
    );


    /*
     * Push-pull
     */

    GPIO_OTYPER(GPIOA_BASE) &= ~(
        (1U << 8U) |
        (1U << 9U) |
        (1U << 10U) |
        (1U << 11U)
    );


    /*
     * High speed
     */

    GPIO_OSPEEDR(GPIOA_BASE) |= (
        (3U << 16U) |
        (3U << 18U) |
        (3U << 20U) |
        (3U << 22U)
    );


    /*
     * AF2:
     *
     * PA8  TIM1_CH1
     * PA9  TIM1_CH2
     * PA10 TIM1_CH3
     * PA11 TIM1_CH4
     */

    afr =
        GPIO_AFRH(GPIOA_BASE);

    afr &= ~(
        (0xFU << 0U) |
        (0xFU << 4U) |
        (0xFU << 8U) |
        (0xFU << 12U)
    );

    afr |= (
        (2U << 0U) |
        (2U << 4U) |
        (2U << 8U) |
        (2U << 12U)
    );

    GPIO_AFRH(GPIOA_BASE) =
        afr;


    /*
     * 8 MHz / 8000 = 1 kHz PWM
     *
     * We already proved this frequency works
     * with the physical motor stage.
     */

    TIM1_PSC = 0U;
    TIM1_ARR = 7999U;


    /*
     * PWM Mode 1
     * Preload enabled
     */

    TIM1_CCMR1 = 0x6868U;
    TIM1_CCMR2 = 0x6868U;


    /*
     * CH1..CH4 enabled
     */

    TIM1_CCER = 0x1111U;


    /*
     * START SAFE
     */

    TIM1_CCR1 = 0U;
    TIM1_CCR2 = 0U;
    TIM1_CCR3 = 0U;
    TIM1_CCR4 = 0U;


    /*
     * Main Output Enable
     */

    TIM1_BDTR =
        (1U << 15U);


    /*
     * Update
     */

    TIM1_EGR = 1U;


    /*
     * ARPE + CEN
     */

    TIM1_CR1 =
        (1U << 7U)
        |
        1U;
}


/* ============================================================
 * PWM CONVERSION
 * ============================================================ */

static uint32_t duty_to_ccr(float percent)
{
    percent =
        clampf_local(
            percent,
            0.0f,
            100.0f
        );

    return
        (uint32_t)(
            (
                percent *
                7999.0f
            )
            /
            100.0f
        );
}


/* ============================================================
 * HARD MOTOR OFF
 * ============================================================ */

static void motor_all_off(void)
{
    TIM1_CCR1 = 0U;
    TIM1_CCR2 = 0U;
    TIM1_CCR3 = 0U;
    TIM1_CCR4 = 0U;

    motor1_percent = 0U;
    motor2_percent = 0U;
    motor3_percent = 0U;
    motor4_percent = 0U;

    physical_motor_lock = 1U;
}


/* ============================================================
 * REAL MOTOR OUTPUT
 * ============================================================ */

static void motor_write_percent(
    float m1,
    float m2,
    float m3,
    float m4
)
{
    /*
     * Additional absolute 50% ceiling.
     */

    m1 =
        clampf_local(
            m1,
            0.0f,
            TEST_MAX_PERCENT
        );

    m2 =
        clampf_local(
            m2,
            0.0f,
            TEST_MAX_PERCENT
        );

    m3 =
        clampf_local(
            m3,
            0.0f,
            TEST_MAX_PERCENT
        );

    m4 =
        clampf_local(
            m4,
            0.0f,
            TEST_MAX_PERCENT
        );


    /*
     * IMPORTANT PHYSICAL MAPPING:
     *
     * CCR1 -> M3
     * CCR2 -> M2
     * CCR3 -> M4
     * CCR4 -> M1
     */

    TIM1_CCR1 =
        duty_to_ccr(m3);

    TIM1_CCR2 =
        duty_to_ccr(m2);

    TIM1_CCR3 =
        duty_to_ccr(m4);

    TIM1_CCR4 =
        duty_to_ccr(m1);


    motor1_percent =
        round_percent(m1);

    motor2_percent =
        round_percent(m2);

    motor3_percent =
        round_percent(m3);

    motor4_percent =
        round_percent(m4);


    physical_motor_lock = 0U;
}


/* ============================================================
 * I2C FLAGS
 * ============================================================ */

static void i2c_clear_flags(void)
{
    I2C1_ICR =
        I2C_ICR_NACKCF |
        I2C_ICR_STOPCF |
        I2C_ICR_BERRCF |
        I2C_ICR_ARLOCF |
        I2C_ICR_OVRCF;
}


static uint32_t i2c_check_errors(void)
{
    uint32_t isr;

    isr =
        I2C1_ISR;

    i2c_isr_debug =
        isr;


    /*
     * STM32F0 spurious BERR:
     * clear/count, do not abort solely for BERR.
     */

    if (
        (isr & I2C_ISR_BERR)
        !=
        0U
    )
    {
        I2C1_ICR =
            I2C_ICR_BERRCF;

        i2c_berr_counter++;
    }


    if (
        (isr & I2C_ISR_NACKF)
        !=
        0U
    )
    {
        I2C1_ICR =
            I2C_ICR_NACKCF;

        i2c_last_error = 1U;

        return 0U;
    }


    if (
        (isr & I2C_ISR_ARLO)
        !=
        0U
    )
    {
        I2C1_ICR =
            I2C_ICR_ARLOCF;

        i2c_last_error = 2U;

        return 0U;
    }


    if (
        (isr & I2C_ISR_OVR)
        !=
        0U
    )
    {
        I2C1_ICR =
            I2C_ICR_OVRCF;

        i2c_last_error = 3U;

        return 0U;
    }


    return 1U;
}


static uint32_t i2c_wait_flag(uint32_t flag)
{
    uint32_t start;

    start =
        system_ms;

    while (
        (I2C1_ISR & flag)
        ==
        0U
    )
    {
        if (
            i2c_check_errors()
            ==
            0U
        )
        {
            return 0U;
        }


        if (
            (uint32_t)(
                system_ms -
                start
            )
            >=
            10U
        )
        {
            i2c_last_error = 4U;

            return 0U;
        }
    }


    return 1U;
}


static uint32_t i2c_wait_not_busy(void)
{
    uint32_t start;

    start =
        system_ms;

    while (
        (I2C1_ISR & I2C_ISR_BUSY)
        !=
        0U
    )
    {
        if (
            (uint32_t)(
                system_ms -
                start
            )
            >=
            10U
        )
        {
            i2c_last_error = 5U;

            return 0U;
        }
    }


    return 1U;
}


/* ============================================================
 * I2C GPIO
 * ============================================================ */

static void i2c_gpio_init(void)
{
    uint32_t value;


    /*
     * PB7 = SDA
     * PB8 = SCL
     */

    GPIO_MODER(GPIOB_BASE) &= ~(
        (3U << 14U) |
        (3U << 16U)
    );

    GPIO_MODER(GPIOB_BASE) |= (
        (2U << 14U) |
        (2U << 16U)
    );


    /*
     * Open drain
     */

    GPIO_OTYPER(GPIOB_BASE) |=
        (1U << 7U) |
        (1U << 8U);


    /*
     * High speed
     */

    GPIO_OSPEEDR(GPIOB_BASE) |=
        (3U << 14U) |
        (3U << 16U);


    /*
     * Pull-ups
     */

    GPIO_PUPDR(GPIOB_BASE) &= ~(
        (3U << 14U) |
        (3U << 16U)
    );

    GPIO_PUPDR(GPIOB_BASE) |= (
        (1U << 14U) |
        (1U << 16U)
    );


    /*
     * PB7 AF1
     */

    value =
        GPIO_AFRL(GPIOB_BASE);

    value &=
        ~(0xFU << 28U);

    value |=
        (1U << 28U);

    GPIO_AFRL(GPIOB_BASE) =
        value;


    /*
     * PB8 AF1
     */

    value =
        GPIO_AFRH(GPIOB_BASE);

    value &=
        ~(0xFU << 0U);

    value |=
        (1U << 0U);

    GPIO_AFRH(GPIOB_BASE) =
        value;
}


/* ============================================================
 * I2C INIT
 * ============================================================ */

static void i2c_init(void)
{
    i2c_gpio_init();


    /*
     * I2C1 clock = HSI
     */

    RCC_CFGR3 &=
        ~(1U << 4U);


    RCC_APB1ENR |=
        I2C1_EN;


    RCC_APB1RSTR |=
        I2C1_EN;

    RCC_APB1RSTR &=
        ~I2C1_EN;


    I2C1_CR1 &=
        ~I2C_CR1_PE;


    /*
     * 8 MHz kernel
     * ~100 kHz Standard Mode
     */

    I2C1_TIMINGR =
        0x10420F13U;


    i2c_timingr_debug =
        I2C1_TIMINGR;


    i2c_clear_flags();


    I2C1_CR1 |=
        I2C_CR1_PE;


    i2c_hw_enabled = 1U;
}


/* ============================================================
 * M540 WRITE
 * ============================================================ */

static uint32_t m540_write_reg(
    uint8_t reg,
    uint8_t value
)
{
    uint32_t cr2;


    if (
        i2c_wait_not_busy()
        ==
        0U
    )
    {
        return 0U;
    }


    i2c_last_error = 0U;

    i2c_clear_flags();


    cr2 =
        ((uint32_t)M540_ADDR << 1U)
        |
        (2U << 16U)
        |
        I2C_CR2_AUTOEND
        |
        I2C_CR2_START;


    I2C1_CR2 =
        cr2;


    if (
        i2c_wait_flag(I2C_ISR_TXIS)
        ==
        0U
    )
    {
        return 0U;
    }


    I2C1_TXDR =
        reg;


    if (
        i2c_wait_flag(I2C_ISR_TXIS)
        ==
        0U
    )
    {
        return 0U;
    }


    I2C1_TXDR =
        value;


    if (
        i2c_wait_flag(I2C_ISR_STOPF)
        ==
        0U
    )
    {
        return 0U;
    }


    I2C1_ICR =
        I2C_ICR_STOPCF;


    i2c_transfer_counter++;


    return 1U;
}


/* ============================================================
 * M540 READ
 * ============================================================ */

static uint32_t m540_read_regs(
    uint8_t reg,
    uint8_t *buffer,
    uint32_t length
)
{
    uint32_t cr2;
    uint32_t i;


    if (
        (length == 0U)
        ||
        (length > 255U)
    )
    {
        return 0U;
    }


    if (
        i2c_wait_not_busy()
        ==
        0U
    )
    {
        return 0U;
    }


    i2c_last_error = 0U;

    i2c_clear_flags();


    /*
     * Write register address
     */

    cr2 =
        ((uint32_t)M540_ADDR << 1U)
        |
        (1U << 16U)
        |
        I2C_CR2_START;


    I2C1_CR2 =
        cr2;


    if (
        i2c_wait_flag(I2C_ISR_TXIS)
        ==
        0U
    )
    {
        return 0U;
    }


    I2C1_TXDR =
        reg;


    if (
        i2c_wait_flag(I2C_ISR_TC)
        ==
        0U
    )
    {
        return 0U;
    }


    /*
     * Repeated START read
     */

    cr2 =
        ((uint32_t)M540_ADDR << 1U)
        |
        (length << 16U)
        |
        I2C_CR2_RD_WRN
        |
        I2C_CR2_AUTOEND
        |
        I2C_CR2_START;


    I2C1_CR2 =
        cr2;


    for (
        i = 0U;
        i < length;
        i++
    )
    {
        if (
            i2c_wait_flag(I2C_ISR_RXNE)
            ==
            0U
        )
        {
            return 0U;
        }


        buffer[i] =
            (uint8_t)I2C1_RXDR;
    }


    if (
        i2c_wait_flag(I2C_ISR_STOPF)
        ==
        0U
    )
    {
        return 0U;
    }


    I2C1_ICR =
        I2C_ICR_STOPCF;


    i2c_transfer_counter++;


    return 1U;
}


/* ============================================================
 * M540 INIT
 * ============================================================ */

static uint32_t m540_init(void)
{
    uint8_t who;


    /*
     * Reset
     */

    if (
        m540_write_reg(
            M540_REG_PWR_MGMT,
            0x80U
        )
        ==
        0U
    )
    {
        return 0U;
    }


    delay_ms(100U);


    /*
     * Wake
     */

    if (
        m540_write_reg(
            M540_REG_PWR_MGMT,
            0x01U
        )
        ==
        0U
    )
    {
        return 0U;
    }


    delay_ms(20U);


    /*
     * Accel +-16g
     */

    if (
        m540_write_reg(
            M540_REG_ACCEL_CFG,
            0x18U
        )
        ==
        0U
    )
    {
        return 0U;
    }


    /*
     * Accel filter
     */

    if (
        m540_write_reg(
            M540_REG_ACCEL_CFG2,
            0x05U
        )
        ==
        0U
    )
    {
        return 0U;
    }


    /*
     * Gyro +-2000 dps
     */

    if (
        m540_write_reg(
            M540_REG_GYRO_CFG,
            0x18U
        )
        ==
        0U
    )
    {
        return 0U;
    }


    if (
        m540_write_reg(
            M540_REG_CONFIG,
            0x00U
        )
        ==
        0U
    )
    {
        return 0U;
    }


    delay_ms(20U);


    if (
        m540_read_regs(
            M540_REG_WHO_AM_I,
            &who,
            1U
        )
        ==
        0U
    )
    {
        return 0U;
    }


    imu_whoami =
        who;


    if (
        who
        !=
        M540_WHO_VALUE
    )
    {
        return 0U;
    }


    return 1U;
}


/* ============================================================
 * M540 RAW FRAME
 * ============================================================ */

static uint32_t m540_read_raw(void)
{
    uint8_t data[14];


    if (
        m540_read_regs(
            M540_REG_ACCEL,
            data,
            14U
        )
        ==
        0U
    )
    {
        return 0U;
    }


    accel_x =
        (int16_t)(
            ((uint16_t)data[0] << 8U)
            |
            data[1]
        );


    accel_y =
        (int16_t)(
            ((uint16_t)data[2] << 8U)
            |
            data[3]
        );


    accel_z =
        (int16_t)(
            ((uint16_t)data[4] << 8U)
            |
            data[5]
        );


    /*
     * data[6..7] = temperature
     */


    gyro_x =
        (int16_t)(
            ((uint16_t)data[8] << 8U)
            |
            data[9]
        );


    gyro_y =
        (int16_t)(
            ((uint16_t)data[10] << 8U)
            |
            data[11]
        );


    gyro_z =
        (int16_t)(
            ((uint16_t)data[12] << 8U)
            |
            data[13]
        );


    imu_frame_counter++;


    return 1U;
}


/* ============================================================
 * GYRO CALIBRATION
 * ============================================================ */

static uint32_t gyro_calibrate(void)
{
    uint32_t i;
    uint32_t valid;

    int32_t sum_x;
    int32_t sum_y;
    int32_t sum_z;


    valid = 0U;

    sum_x = 0;
    sum_y = 0;
    sum_z = 0;


    calibration_status = 1U;
    calibration_samples = 0U;


    /*
     * KEEP THE DRONE COMPLETELY STILL HERE.
     */

    for (
        i = 0U;
        i < 500U;
        i++
    )
    {
        if (
            m540_read_raw()
            !=
            0U
        )
        {
            sum_x +=
                gyro_x;

            sum_y +=
                gyro_y;

            sum_z +=
                gyro_z;


            valid++;

            calibration_samples =
                valid;
        }


        delay_ms(3U);
    }


    if (
        valid
        <
        450U
    )
    {
        calibration_status = 3U;

        return 0U;
    }


    gyro_x_offset =
        (float)sum_x /
        (float)valid;


    gyro_y_offset =
        (float)sum_y /
        (float)valid;


    gyro_z_offset =
        (float)sum_z /
        (float)valid;


    calibration_status = 2U;


    return 1U;
}


/* ============================================================
 * GYRO UPDATE
 * ============================================================ */

static void gyro_update(void)
{
    /*
     * Confirmed:
     *
     * X = roll
     * Y = pitch
     * Z = yaw
     */

    gyro_roll_dps =
        (
            (float)gyro_x
            -
            gyro_x_offset
        )
        /
        GYRO_LSB_PER_DPS;


    gyro_pitch_dps =
        (
            (float)gyro_y
            -
            gyro_y_offset
        )
        /
        GYRO_LSB_PER_DPS;


    gyro_yaw_dps =
        (
            (float)gyro_z
            -
            gyro_z_offset
        )
        /
        GYRO_LSB_PER_DPS;
}


/* ============================================================
 * RATE CONTROLLER
 * ============================================================ */

static void rate_controller(void)
{
    /*
     * Rate setpoints are all ZERO.
     *
     * error = target - measured
     */


    pid_roll_output =
        ROLL_KP *
        (-gyro_roll_dps);


    pid_pitch_output =
        PITCH_KP *
        (-gyro_pitch_dps);


    pid_yaw_output =
        YAW_KP *
        (-gyro_yaw_dps);


    pid_roll_output =
        clampf_local(
            pid_roll_output,
            -ROLL_PID_LIMIT,
            ROLL_PID_LIMIT
        );


    pid_pitch_output =
        clampf_local(
            pid_pitch_output,
            -PITCH_PID_LIMIT,
            PITCH_PID_LIMIT
        );


    pid_yaw_output =
        clampf_local(
            pid_yaw_output,
            -YAW_PID_LIMIT,
            YAW_PID_LIMIT
        );
}


/* ============================================================
 * VERIFIED MIXER
 * ============================================================ */

static void mixer_update(void)
{
    /*
     *                  FRONT
     *
     *             M2          M1
     *             CCW         CW
     *
     *
     *             M3          M4
     *             CW          CCW
     *
     *
     * Final experimentally checked mixer:
     *
     * M1 = T + R + P + Y
     * M2 = T - R + P - Y
     * M3 = T - R - P + Y
     * M4 = T + R - P - Y
     */


    mix_m1_percent =
        clampf_local(
            TEST_BASE_PERCENT
            +
            pid_roll_output
            +
            pid_pitch_output
            +
            pid_yaw_output,

            TEST_MIN_PERCENT,
            TEST_MAX_PERCENT
        );


    mix_m2_percent =
        clampf_local(
            TEST_BASE_PERCENT
            -
            pid_roll_output
            +
            pid_pitch_output
            -
            pid_yaw_output,

            TEST_MIN_PERCENT,
            TEST_MAX_PERCENT
        );


    mix_m3_percent =
        clampf_local(
            TEST_BASE_PERCENT
            -
            pid_roll_output
            -
            pid_pitch_output
            +
            pid_yaw_output,

            TEST_MIN_PERCENT,
            TEST_MAX_PERCENT
        );


    mix_m4_percent =
        clampf_local(
            TEST_BASE_PERCENT
            +
            pid_roll_output
            -
            pid_pitch_output
            -
            pid_yaw_output,

            TEST_MIN_PERCENT,
            TEST_MAX_PERCENT
        );
}


/* ============================================================
 * MAIN
 * ============================================================ */

int main(void)
{
    uint32_t last_loop_ms;

    uint32_t now_ms;
    uint32_t elapsed;

    uint32_t state_start_ms;


    /* ========================================================
     * GPIO CLOCKS
     * ======================================================== */

    RCC_AHBENR |=
        GPIOA_EN |
        GPIOB_EN;


    (void)RCC_AHBENR;


    /* ========================================================
     * BOARD ENABLE PA1
     * ======================================================== */

    gpio_high(
        GPIOA_BASE,
        1U
    );


    gpio_output(
        GPIOA_BASE,
        1U
    );


    /* ========================================================
     * MOTOR PINS SAFE
     * ======================================================== */

    motor_gpio_prepare_low();


    /* ========================================================
     * TIME
     * ======================================================== */

    systick_init();


    delay_ms(20U);


    /* ========================================================
     * TIM1
     *
     * Initially 0%.
     * ======================================================== */

    tim1_init();


    motor_all_off();


    /* ========================================================
     * HARDWARE I2C
     * ======================================================== */

    i2c_init();


    delay_ms(20U);


    /* ========================================================
     * IMU INIT
     * ======================================================== */

    if (
        m540_init()
        ==
        0U
    )
    {
        imu_ok = 0U;

        motor_test_state = 4U;

        motor_test_fault_code = 1U;
    }
    else
    {
        imu_ok = 1U;


        /* ====================================================
         * GYRO CALIBRATION
         *
         * DO NOT MOVE THE DRONE.
         * ==================================================== */

        if (
            gyro_calibrate()
            ==
            0U
        )
        {
            imu_ok = 0U;

            motor_test_state = 4U;

            motor_test_fault_code = 2U;
        }
    }


    imu_error_counter = 0U;


    state_start_ms =
        system_ms;


    last_loop_ms =
        system_ms;


    /* ========================================================
     * MAIN LOOP
     * ======================================================== */

    while (1)
    {
        now_ms =
            system_ms;


        elapsed =
            (uint32_t)(
                now_ms -
                last_loop_ms
            );


        if (
            elapsed
            <
            LOOP_PERIOD_MS
        )
        {
            continue;
        }


        last_loop_ms =
            now_ms;


        loop_dt_ms =
            elapsed;


        if (
            elapsed
            >
            0U
        )
        {
            loop_hz =
                1000.0f /
                (float)elapsed;
        }
        else
        {
            loop_hz =
                0.0f;
        }


        /* ====================================================
         * FINISHED OR FAULT
         *
         * MOTOR OUTPUT CAN NEVER RESTART.
         * ==================================================== */

        if (
            (motor_test_state == 3U)
            ||
            (motor_test_state == 4U)
        )
        {
            motor_all_off();

            continue;
        }


        /* ====================================================
         * IMU READ
         * ==================================================== */

        if (
            m540_read_raw()
            ==
            0U
        )
        {
            /*
             * Immediate hard shutdown.
             */

            imu_ok = 0U;

            imu_error_counter++;


            motor_test_fault_code = 3U;

            motor_test_state = 4U;


            motor_all_off();


            continue;
        }


        imu_ok = 1U;


        /* ====================================================
         * IMU -> RATE PID -> MIXER
         * ==================================================== */

        gyro_update();

        rate_controller();

        mixer_update();


        /* ====================================================
         * STATE 0
         *
         * 3 seconds waiting.
         * Motors physically OFF.
         * ==================================================== */

        if (
            motor_test_state
            ==
            0U
        )
        {
            motor_all_off();


            if (
                (uint32_t)(
                    now_ms -
                    state_start_ms
                )
                >=
                TEST_WAIT_MS
            )
            {
                motor_test_state = 1U;

                state_start_ms =
                    now_ms;


                motor_test_remaining_ms =
                    MOTOR_START_KICK_MS;
            }
            else
            {
                motor_test_remaining_ms =
                    TEST_WAIT_MS
                    -
                    (uint32_t)(
                        now_ms -
                        state_start_ms
                    );
            }
        }


        /* ====================================================
         * STATE 1
         *
         * 120 ms startup kick.
         *
         * All four motors = 50%.
         * ==================================================== */

        else if (
            motor_test_state
            ==
            1U
        )
        {
            motor_write_percent(
                START_KICK_PERCENT,
                START_KICK_PERCENT,
                START_KICK_PERCENT,
                START_KICK_PERCENT
            );


            if (
                (uint32_t)(
                    now_ms -
                    state_start_ms
                )
                >=
                MOTOR_START_KICK_MS
            )
            {
                motor_test_state = 2U;

                state_start_ms =
                    now_ms;


                motor_test_remaining_ms =
                    ACTIVE_TEST_MS;
            }
            else
            {
                motor_test_remaining_ms =
                    MOTOR_START_KICK_MS
                    -
                    (uint32_t)(
                        now_ms -
                        state_start_ms
                    );
            }
        }


        /* ====================================================
         * STATE 2
         *
         * REAL PID -> REAL MOTORS
         *
         * Only 5 seconds.
         * ==================================================== */

        else if (
            motor_test_state
            ==
            2U
        )
        {
            motor_write_percent(
                mix_m1_percent,
                mix_m2_percent,
                mix_m3_percent,
                mix_m4_percent
            );


            if (
                (uint32_t)(
                    now_ms -
                    state_start_ms
                )
                >=
                ACTIVE_TEST_MS
            )
            {
                /*
                 * TEST COMPLETE.
                 *
                 * Cannot restart without MCU reset.
                 */

                motor_test_state = 3U;

                motor_test_remaining_ms = 0U;


                motor_all_off();
            }
            else
            {
                motor_test_remaining_ms =
                    ACTIVE_TEST_MS
                    -
                    (uint32_t)(
                        now_ms -
                        state_start_ms
                    );
            }
        }


        /* ====================================================
         * IMPOSSIBLE STATE -> FAULT
         * ==================================================== */

        else
        {
            motor_test_state = 4U;

            motor_test_fault_code = 99U;


            motor_all_off();
        }


        i2c_isr_debug =
            I2C1_ISR;


        i2c_timingr_debug =
            I2C1_TIMINGR;
    }
}
