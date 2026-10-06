/*
 * ============================================================
 * DRONE_ - STM32F031K4
 * STEP 4.2
 *
 * Hardware I2C + M540
 * 200 Hz control loop
 * Virtual RATE controller
 * Corrected ROLL mixer
 * Automatic PITCH direction diagnostic
 *
 * IMPORTANT:
 *   PHYSICAL MOTORS ARE HARD-LOCKED OFF.
 *   mix_mX_percent = virtual motor command only.
 *
 * Drone geometry:
 *
 *                  FRONT
 *                    ^
 *
 *             M2 CCW     M1 CW
 *             LEFT       RIGHT
 *
 *
 *             M3 CW      M4 CCW
 *             LEFT       RIGHT
 *
 * Motor electrical mapping:
 *   PA8  TIM1_CH1 -> M3
 *   PA9  TIM1_CH2 -> M2
 *   PA10 TIM1_CH3 -> M4
 *   PA11 TIM1_CH4 -> M1
 *
 * IMU:
 *   M540
 *   PB8 = I2C1_SCL
 *   PB7 = I2C1_SDA
 *   address = 0x69
 *   WHO_AM_I = 0x7D
 *
 * ============================================================
 */

#include <stdint.h>


/* ============================================================
 * BASIC REGISTER ACCESS
 * ============================================================ */

#define REG32(addr) (*(volatile uint32_t *)(addr))


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
#define GPIO_ODR(base)      REG32((base) + 0x14U)
#define GPIO_BSRR(base)     REG32((base) + 0x18U)
#define GPIO_AFRL(base)     REG32((base) + 0x20U)
#define GPIO_AFRH(base)     REG32((base) + 0x24U)


/* ============================================================
 * SYSTICK
 * ============================================================ */

#define SYST_CSR        REG32(0xE000E010U)
#define SYST_RVR        REG32(0xE000E014U)
#define SYST_CVR        REG32(0xE000E018U)

volatile uint32_t system_ms = 0;


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
#define I2C_CR2_STOP    (1U << 14)
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
#define M540_REG_GYRO       0x43U

#define M540_REG_GYRO_CFG   0x1BU
#define M540_REG_ACCEL_CFG  0x1CU
#define M540_REG_ACCEL_CFG2 0x1DU

#define M540_REG_CONFIG     0x1AU
#define M540_REG_PWR_MGMT   0x6BU

#define M540_REG_WHO_AM_I   0x75U

#define M540_WHO_VALUE      0x7DU


/* ============================================================
 * FLIGHT CONSTANTS
 * ============================================================ */

#define LOOP_PERIOD_MS          5U

#define ACCEL_LSB_PER_G         2048.0f
#define GYRO_LSB_PER_DPS        16.4f

#define COMPLEMENTARY_GYRO      0.99f
#define COMPLEMENTARY_ACCEL     0.01f

/*
 * This is NOT final PID tuning.
 *
 * It is deliberately a simple virtual RATE P controller
 * for checking control direction.
 */
#define TEST_RATE_KP            0.10f

#define TEST_PID_LIMIT          35.0f
#define TEST_BASE_THROTTLE      50.0f

#define PITCH_TRIGGER_DPS       20.0f
#define PITCH_CAPTURE_MS        700U
#define PITCH_ARM_DELAY_MS      1500U


/* ============================================================
 * IMU DEBUG VARIABLES
 * ============================================================ */

volatile uint32_t imu_ok = 0;
volatile uint8_t imu_whoami = 0;

volatile uint32_t imu_frame_counter = 0;
volatile uint32_t imu_error_counter = 0;
volatile uint32_t imu_consecutive_errors = 0;

volatile int16_t accel_x = 0;
volatile int16_t accel_y = 0;
volatile int16_t accel_z = 0;

volatile int16_t gyro_x = 0;
volatile int16_t gyro_y = 0;
volatile int16_t gyro_z = 0;


/* ============================================================
 * I2C DEBUG
 * ============================================================ */

volatile uint32_t i2c_hw_enabled = 0;

volatile uint32_t i2c_last_error = 0;
volatile uint32_t i2c_berr_counter = 0;
volatile uint32_t i2c_transfer_counter = 0;

volatile uint32_t i2c_isr_debug = 0;
volatile uint32_t i2c_timingr_debug = 0;


/* ============================================================
 * CALIBRATION
 * ============================================================ */

volatile uint32_t calibration_status = 0;
volatile uint32_t calibration_samples = 0;

volatile float gyro_x_offset = 0.0f;
volatile float gyro_y_offset = 0.0f;
volatile float gyro_z_offset = 0.0f;


/* ============================================================
 * ATTITUDE
 * ============================================================ */

volatile float accel_roll_deg = 0.0f;
volatile float accel_pitch_deg = 0.0f;

volatile float gyro_roll_dps = 0.0f;
volatile float gyro_pitch_dps = 0.0f;
volatile float gyro_yaw_dps = 0.0f;

volatile float roll_angle = 0.0f;
volatile float pitch_angle = 0.0f;
volatile float yaw_angle = 0.0f;


/* ============================================================
 * LOOP DEBUG
 * ============================================================ */

volatile uint32_t loop_dt_ms = 0;
volatile uint32_t loop_dt_max_ms = 0;

volatile float loop_dt = 0.005f;
volatile float loop_hz = 200.0f;

volatile uint32_t imu_read_time_ms = 0;
volatile uint32_t imu_read_time_max_ms = 0;


/* ============================================================
 * PHYSICAL MOTOR OUTPUTS
 *
 * MUST STAY ZERO DURING THIS TEST
 * ============================================================ */

volatile uint32_t motor1_percent = 0;
volatile uint32_t motor2_percent = 0;
volatile uint32_t motor3_percent = 0;
volatile uint32_t motor4_percent = 0;

/*
 * Informational variable only.
 *
 * Even if this is changed in Live Expressions,
 * motor_all_off() is still called unconditionally.
 */
volatile uint32_t physical_motor_lock = 1;


/* ============================================================
 * VIRTUAL RATE CONTROLLER
 * ============================================================ */

volatile float pid_roll_output = 0.0f;
volatile float pid_pitch_output = 0.0f;
volatile float pid_yaw_output = 0.0f;


/* ============================================================
 * VIRTUAL MOTOR MIXER
 * ============================================================ */

volatile float mix_m1_percent = 50.0f;
volatile float mix_m2_percent = 50.0f;
volatile float mix_m3_percent = 50.0f;
volatile float mix_m4_percent = 50.0f;


/* ============================================================
 * PITCH AUTOMATIC DIAGNOSTIC
 *
 * state:
 *   0 = initial delay
 *   1 = armed / recording
 *   2 = finished, values frozen
 * ============================================================ */

volatile uint32_t pitch_diag_state = 0;

volatile int32_t pitch_diag_first_sign = 0;

volatile float pitch_diag_first_gyro_dps = 0.0f;
volatile float pitch_diag_peak_gyro_dps = 0.0f;

volatile float pitch_diag_pid_at_peak = 0.0f;

volatile float pitch_diag_m1_at_peak = 0.0f;
volatile float pitch_diag_m2_at_peak = 0.0f;
volatile float pitch_diag_m3_at_peak = 0.0f;
volatile float pitch_diag_m4_at_peak = 0.0f;

volatile uint32_t pitch_diag_capture_remaining_ms = 0;


/* ============================================================
 * OLD ROLL RESULT VARIABLES
 *
 * Kept so your existing Live Expressions do not disappear.
 * They are not used for the new test.
 * ============================================================ */

volatile uint32_t roll_diag_state = 2;

volatile int32_t roll_diag_first_sign = 0;

volatile float roll_diag_first_gyro_dps = 0.0f;
volatile float roll_diag_peak_gyro_dps = 0.0f;

volatile float roll_diag_pid_at_peak = 0.0f;

volatile float roll_diag_m1_at_peak = 0.0f;
volatile float roll_diag_m2_at_peak = 0.0f;
volatile float roll_diag_m3_at_peak = 0.0f;
volatile float roll_diag_m4_at_peak = 0.0f;


/* ============================================================
 * SMALL HELPERS
 * ============================================================ */

static float absf_local(float x)
{
    if (x < 0.0f)
    {
        return -x;
    }

    return x;
}


static float clampf_local(float x, float min_value, float max_value)
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


static float fast_sqrtf_local(float x)
{
    float guess;
    uint32_t i;

    if (x <= 0.0f)
    {
        return 0.0f;
    }

    guess = x;

    if (guess < 1.0f)
    {
        guess = 1.0f;
    }

    for (i = 0; i < 6U; i++)
    {
        guess = 0.5f * (guess + (x / guess));
    }

    return guess;
}


/*
 * Fast atan2 approximation.
 * Good enough for attitude diagnostics.
 */
static float fast_atan2_deg(float y, float x)
{
    float abs_y;
    float r;
    float angle;

    const float RAD_TO_DEG = 57.2957795f;

    abs_y = absf_local(y) + 0.000001f;

    if (x >= 0.0f)
    {
        r = (x - abs_y) / (x + abs_y);

        angle =
            0.785398163f +
            (0.1963f * r * r * r) -
            (0.9817f * r);
    }
    else
    {
        r = (x + abs_y) / (abs_y - x);

        angle =
            2.35619449f +
            (0.1963f * r * r * r) -
            (0.9817f * r);
    }

    if (y < 0.0f)
    {
        angle = -angle;
    }

    return angle * RAD_TO_DEG;
}


/* ============================================================
 * GPIO HELPERS
 * ============================================================ */

static void gpio_high(uint32_t base, uint32_t pin)
{
    GPIO_BSRR(base) = (1U << pin);
}


static void gpio_low(uint32_t base, uint32_t pin)
{
    GPIO_BSRR(base) = (1U << (pin + 16U));
}


static void gpio_output(uint32_t base, uint32_t pin)
{
    uint32_t shift;

    shift = pin * 2U;

    GPIO_MODER(base) &= ~(3U << shift);
    GPIO_MODER(base) |=  (1U << shift);
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
     * 8,000,000 / 1000 = 8000
     */

    SYST_RVR = 7999U;
    SYST_CVR = 0U;

    /*
     * ENABLE
     * TICKINT
     * CLKSOURCE = CPU
     */

    SYST_CSR = 0x07U;
}


static void delay_ms(uint32_t delay)
{
    uint32_t start;

    start = system_ms;

    while ((uint32_t)(system_ms - start) < delay)
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

    /*
     * PA8..PA11 = GPIO output LOW before TIM1 takes control.
     */

    for (pin = 8U; pin <= 11U; pin++)
    {
        gpio_low(GPIOA_BASE, pin);
        gpio_output(GPIOA_BASE, pin);
    }
}


/* ============================================================
 * TIM1 MOTOR PWM
 * ============================================================ */

static void tim1_init(void)
{
    uint32_t afr;

    RCC_APB2ENR |= TIM1_EN;

    RCC_APB2RSTR |= TIM1_EN;
    RCC_APB2RSTR &= ~TIM1_EN;

    /*
     * PA8..PA11 alternate function mode
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
     * PA8  = TIM1_CH1
     * PA9  = TIM1_CH2
     * PA10 = TIM1_CH3
     * PA11 = TIM1_CH4
     */

    afr = GPIO_AFRH(GPIOA_BASE);

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

    GPIO_AFRH(GPIOA_BASE) = afr;

    /*
     * Known-safe diagnostic PWM:
     *
     * 8 MHz / 8000 = 1 kHz
     */

    TIM1_PSC = 0U;
    TIM1_ARR = 7999U;

    /*
     * PWM Mode 1
     * preload enabled
     */

    TIM1_CCMR1 = 0x6868U;
    TIM1_CCMR2 = 0x6868U;

    /*
     * Enable CH1..CH4
     */

    TIM1_CCER = 0x1111U;

    /*
     * ABSOLUTELY ZERO MOTOR COMMAND
     */

    TIM1_CCR1 = 0U;
    TIM1_CCR2 = 0U;
    TIM1_CCR3 = 0U;
    TIM1_CCR4 = 0U;

    /*
     * Main Output Enable
     */

    TIM1_BDTR = (1U << 15U);

    /*
     * Update registers
     */

    TIM1_EGR = 1U;

    /*
     * ARPE + CEN
     */

    TIM1_CR1 = (1U << 7U) | 1U;
}


static void motor_all_off(void)
{
    /*
     * Electrical mapping:
     *
     * CCR1 -> M3
     * CCR2 -> M2
     * CCR3 -> M4
     * CCR4 -> M1
     */

    TIM1_CCR1 = 0U;
    TIM1_CCR2 = 0U;
    TIM1_CCR3 = 0U;
    TIM1_CCR4 = 0U;

    motor1_percent = 0U;
    motor2_percent = 0U;
    motor3_percent = 0U;
    motor4_percent = 0U;
}


/* ============================================================
 * I2C ERROR HANDLING
 * ============================================================ */

static void i2c_clear_errors(void)
{
    I2C1_ICR =
        I2C_ICR_NACKCF |
        I2C_ICR_STOPCF |
        I2C_ICR_BERRCF |
        I2C_ICR_ARLOCF |
        I2C_ICR_OVRCF;
}


static uint32_t i2c_poll_errors(void)
{
    uint32_t isr;

    isr = I2C1_ISR;
    i2c_isr_debug = isr;

    /*
     * STM32F0 may report a spurious BERR in master mode.
     * Count and clear it, but do not automatically abort.
     */

    if ((isr & I2C_ISR_BERR) != 0U)
    {
        i2c_berr_counter++;

        I2C1_ICR = I2C_ICR_BERRCF;
    }

    if ((isr & I2C_ISR_NACKF) != 0U)
    {
        I2C1_ICR = I2C_ICR_NACKCF;

        i2c_last_error = 1U;

        return 0U;
    }

    if ((isr & I2C_ISR_ARLO) != 0U)
    {
        I2C1_ICR = I2C_ICR_ARLOCF;

        i2c_last_error = 2U;

        return 0U;
    }

    if ((isr & I2C_ISR_OVR) != 0U)
    {
        I2C1_ICR = I2C_ICR_OVRCF;

        i2c_last_error = 3U;

        return 0U;
    }

    return 1U;
}


static uint32_t i2c_wait_flag(uint32_t flag)
{
    uint32_t start;

    start = system_ms;

    while ((I2C1_ISR & flag) == 0U)
    {
        if (i2c_poll_errors() == 0U)
        {
            return 0U;
        }

        if ((uint32_t)(system_ms - start) >= 10U)
        {
            i2c_last_error = 4U;

            return 0U;
        }
    }

    return 1U;
}


/* ============================================================
 * I2C INITIALIZATION
 * ============================================================ */

static void i2c_gpio_init(void)
{
    uint32_t value;

    /*
     * PB7 = SDA
     * PB8 = SCL
     *
     * Alternate Function mode
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

    GPIO_OTYPER(GPIOB_BASE) |= (
        (1U << 7U) |
        (1U << 8U)
    );

    /*
     * High speed
     */

    GPIO_OSPEEDR(GPIOB_BASE) |= (
        (3U << 14U) |
        (3U << 16U)
    );

    /*
     * Pull-up
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

    value = GPIO_AFRL(GPIOB_BASE);

    value &= ~(0xFU << 28U);
    value |=  (1U << 28U);

    GPIO_AFRL(GPIOB_BASE) = value;

    /*
     * PB8 AF1
     */

    value = GPIO_AFRH(GPIOB_BASE);

    value &= ~(0xFU << 0U);
    value |=  (1U << 0U);

    GPIO_AFRH(GPIOB_BASE) = value;
}


static void i2c_init(void)
{
    i2c_gpio_init();

    /*
     * I2C1 clock source = HSI
     */

    RCC_CFGR3 &= ~(1U << 4U);

    RCC_APB1ENR |= I2C1_EN;

    RCC_APB1RSTR |= I2C1_EN;
    RCC_APB1RSTR &= ~I2C1_EN;

    /*
     * Peripheral disabled while configuring.
     */

    I2C1_CR1 &= ~I2C_CR1_PE;

    /*
     * 8 MHz I2C kernel clock
     * Standard Mode ~100 kHz
     *
     * PRESC  = 1
     * SCLDEL = 4
     * SDADEL = 2
     * SCLH   = 0x0F
     * SCLL   = 0x13
     */

    I2C1_TIMINGR = 0x10420F13U;

    i2c_timingr_debug = I2C1_TIMINGR;

    i2c_clear_errors();

    I2C1_CR1 |= I2C_CR1_PE;

    i2c_hw_enabled = 1U;
}


/* ============================================================
 * I2C WRITE
 * ============================================================ */

static uint32_t m540_write_reg(uint8_t reg, uint8_t value)
{
    uint32_t cr2;

    i2c_last_error = 0U;

    i2c_clear_errors();

    /*
     * address + NBYTES=2 + AUTOEND
     */

    cr2 =
        ((uint32_t)M540_ADDR << 1U) |
        (2U << 16U) |
        I2C_CR2_AUTOEND |
        I2C_CR2_START;

    I2C1_CR2 = cr2;

    if (i2c_wait_flag(I2C_ISR_TXIS) == 0U)
    {
        return 0U;
    }

    I2C1_TXDR = reg;

    if (i2c_wait_flag(I2C_ISR_TXIS) == 0U)
    {
        return 0U;
    }

    I2C1_TXDR = value;

    if (i2c_wait_flag(I2C_ISR_STOPF) == 0U)
    {
        return 0U;
    }

    I2C1_ICR = I2C_ICR_STOPCF;

    i2c_transfer_counter++;

    return 1U;
}


/* ============================================================
 * I2C READ
 * ============================================================ */

static uint32_t m540_read_regs(
    uint8_t reg,
    uint8_t *buffer,
    uint32_t length
)
{
    uint32_t cr2;
    uint32_t i;

    if ((length == 0U) || (length > 255U))
    {
        return 0U;
    }

    i2c_last_error = 0U;

    i2c_clear_errors();

    /*
     * First transaction:
     * send register address
     *
     * No AUTOEND.
     */

    cr2 =
        ((uint32_t)M540_ADDR << 1U) |
        (1U << 16U) |
        I2C_CR2_START;

    I2C1_CR2 = cr2;

    if (i2c_wait_flag(I2C_ISR_TXIS) == 0U)
    {
        return 0U;
    }

    I2C1_TXDR = reg;

    /*
     * Wait Transfer Complete before repeated START.
     */

    if (i2c_wait_flag(I2C_ISR_TC) == 0U)
    {
        return 0U;
    }

    /*
     * Repeated START:
     * READ
     */

    cr2 =
        ((uint32_t)M540_ADDR << 1U) |
        (length << 16U) |
        I2C_CR2_RD_WRN |
        I2C_CR2_AUTOEND |
        I2C_CR2_START;

    I2C1_CR2 = cr2;

    for (i = 0U; i < length; i++)
    {
        if (i2c_wait_flag(I2C_ISR_RXNE) == 0U)
        {
            return 0U;
        }

        buffer[i] = (uint8_t)I2C1_RXDR;
    }

    if (i2c_wait_flag(I2C_ISR_STOPF) == 0U)
    {
        return 0U;
    }

    I2C1_ICR = I2C_ICR_STOPCF;

    i2c_transfer_counter++;

    return 1U;
}


/* ============================================================
 * M540 INITIALIZATION
 * ============================================================ */

static uint32_t m540_init(void)
{
    uint8_t who;

    /*
     * Reset
     */

    if (m540_write_reg(M540_REG_PWR_MGMT, 0x80U) == 0U)
    {
        return 0U;
    }

    delay_ms(100U);

    /*
     * Wake + clock
     */

    if (m540_write_reg(M540_REG_PWR_MGMT, 0x01U) == 0U)
    {
        return 0U;
    }

    delay_ms(20U);

    /*
     * Accelerometer +-16 g
     */

    if (m540_write_reg(M540_REG_ACCEL_CFG, 0x18U) == 0U)
    {
        return 0U;
    }

    /*
     * Accelerometer filter
     */

    if (m540_write_reg(M540_REG_ACCEL_CFG2, 0x05U) == 0U)
    {
        return 0U;
    }

    /*
     * Gyro +-2000 dps
     */

    if (m540_write_reg(M540_REG_GYRO_CFG, 0x18U) == 0U)
    {
        return 0U;
    }

    /*
     * Gyro config
     */

    if (m540_write_reg(M540_REG_CONFIG, 0x00U) == 0U)
    {
        return 0U;
    }

    delay_ms(20U);

    if (m540_read_regs(M540_REG_WHO_AM_I, &who, 1U) == 0U)
    {
        return 0U;
    }

    imu_whoami = who;

    if (who != M540_WHO_VALUE)
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

    if (m540_read_regs(M540_REG_ACCEL, data, 14U) == 0U)
    {
        return 0U;
    }

    accel_x =
        (int16_t)(
            ((uint16_t)data[0] << 8U) |
            (uint16_t)data[1]
        );

    accel_y =
        (int16_t)(
            ((uint16_t)data[2] << 8U) |
            (uint16_t)data[3]
        );

    accel_z =
        (int16_t)(
            ((uint16_t)data[4] << 8U) |
            (uint16_t)data[5]
        );

    /*
     * data[6], data[7] = temperature
     */

    gyro_x =
        (int16_t)(
            ((uint16_t)data[8] << 8U) |
            (uint16_t)data[9]
        );

    gyro_y =
        (int16_t)(
            ((uint16_t)data[10] << 8U) |
            (uint16_t)data[11]
        );

    gyro_z =
        (int16_t)(
            ((uint16_t)data[12] << 8U) |
            (uint16_t)data[13]
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

    sum_x = 0;
    sum_y = 0;
    sum_z = 0;

    valid = 0U;

    calibration_status = 1U;
    calibration_samples = 0U;

    for (i = 0U; i < 500U; i++)
    {
        if (m540_read_raw() != 0U)
        {
            sum_x += gyro_x;
            sum_y += gyro_y;
            sum_z += gyro_z;

            valid++;

            calibration_samples = valid;
        }

        delay_ms(3U);
    }

    if (valid < 450U)
    {
        calibration_status = 3U;

        return 0U;
    }

    gyro_x_offset = (float)sum_x / (float)valid;
    gyro_y_offset = (float)sum_y / (float)valid;
    gyro_z_offset = (float)sum_z / (float)valid;

    calibration_status = 2U;

    return 1U;
}


/* ============================================================
 * ATTITUDE UPDATE
 * ============================================================ */

static void attitude_update(float dt)
{
    float ay;
    float az;

    float yz_length;

    /*
     * Raw gyro axis mapping already established:
     *
     * X = roll
     * Y = pitch
     * Z = yaw
     */

    gyro_roll_dps =
        ((float)gyro_x - gyro_x_offset) /
        GYRO_LSB_PER_DPS;

    gyro_pitch_dps =
        ((float)gyro_y - gyro_y_offset) /
        GYRO_LSB_PER_DPS;

    gyro_yaw_dps =
        ((float)gyro_z - gyro_z_offset) /
        GYRO_LSB_PER_DPS;

    /*
     * Accelerometer angles.
     */

    accel_roll_deg =
        fast_atan2_deg(
            (float)accel_y,
            (float)accel_z
        );

    ay = (float)accel_y;
    az = (float)accel_z;

    yz_length =
        fast_sqrtf_local(
            (ay * ay) +
            (az * az)
        );

    accel_pitch_deg =
        fast_atan2_deg(
            -(float)accel_x,
            yz_length
        );

    /*
     * Complementary estimator.
     */

    roll_angle =
        COMPLEMENTARY_GYRO *
        (
            roll_angle +
            gyro_roll_dps * dt
        )
        +
        COMPLEMENTARY_ACCEL *
        accel_roll_deg;

    pitch_angle =
        COMPLEMENTARY_GYRO *
        (
            pitch_angle +
            gyro_pitch_dps * dt
        )
        +
        COMPLEMENTARY_ACCEL *
        accel_pitch_deg;

    yaw_angle += gyro_yaw_dps * dt;

    if (yaw_angle > 180.0f)
    {
        yaw_angle -= 360.0f;
    }

    if (yaw_angle < -180.0f)
    {
        yaw_angle += 360.0f;
    }
}


/* ============================================================
 * VIRTUAL RATE CONTROLLER
 * ============================================================ */

static void virtual_rate_controller(void)
{
    float roll_error;
    float pitch_error;

    /*
     * Desired rate = 0 dps.
     *
     * error = target - measured
     */

    roll_error = -gyro_roll_dps;
    pitch_error = -gyro_pitch_dps;

    pid_roll_output =
        TEST_RATE_KP *
        roll_error;

    pid_pitch_output =
        TEST_RATE_KP *
        pitch_error;

    pid_roll_output =
        clampf_local(
            pid_roll_output,
            -TEST_PID_LIMIT,
            TEST_PID_LIMIT
        );

    pid_pitch_output =
        clampf_local(
            pid_pitch_output,
            -TEST_PID_LIMIT,
            TEST_PID_LIMIT
        );

    /*
     * YAW disabled during this test.
     */

    pid_yaw_output = 0.0f;
}


/* ============================================================
 * VIRTUAL MOTOR MIXER
 * ============================================================ */

static void virtual_motor_mixer(void)
{
    float m1;
    float m2;
    float m3;
    float m4;

    /*
     * Geometry:
     *
     *             FRONT
     *
     *       M2             M1
     *       LEFT           RIGHT
     *
     *
     *       M3             M4
     *       LEFT           RIGHT
     *
     *
     * ========================================================
     * ROLL CORRECTION
     * ========================================================
     *
     * IMPORTANT:
     *
     * This is the FIX from the previous test.
     *
     * Positive roll correction:
     *
     * RIGHT motors:
     *   M1 ↑
     *   M4 ↑
     *
     * LEFT motors:
     *   M2 ↓
     *   M3 ↓
     *
     *
     * ========================================================
     * PITCH
     * ========================================================
     *
     * Current hypothesis:
     *
     * positive pitch correction:
     *
     * front M1/M2 decrease
     * rear  M3/M4 increase
     *
     * We are testing this now.
     */

    m1 =
        TEST_BASE_THROTTLE
        + pid_roll_output
        - pid_pitch_output;

    m2 =
        TEST_BASE_THROTTLE
        - pid_roll_output
        - pid_pitch_output;

    m3 =
        TEST_BASE_THROTTLE
        - pid_roll_output
        + pid_pitch_output;

    m4 =
        TEST_BASE_THROTTLE
        + pid_roll_output
        + pid_pitch_output;

    mix_m1_percent =
        clampf_local(m1, 0.0f, 100.0f);

    mix_m2_percent =
        clampf_local(m2, 0.0f, 100.0f);

    mix_m3_percent =
        clampf_local(m3, 0.0f, 100.0f);

    mix_m4_percent =
        clampf_local(m4, 0.0f, 100.0f);
}


/* ============================================================
 * PITCH DIRECTION DIAGNOSTIC
 * ============================================================ */

static void pitch_diagnostic_update(void)
{
    static uint32_t diag_start_ms = 0U;
    static uint32_t capture_start_ms = 0U;

    float current_abs;
    float peak_abs;

    /*
     * State 0:
     * wait after startup/calibration.
     */

    if (pitch_diag_state == 0U)
    {
        if (diag_start_ms == 0U)
        {
            diag_start_ms = system_ms;
        }

        if (
            (uint32_t)(system_ms - diag_start_ms)
            >= PITCH_ARM_DELAY_MS
        )
        {
            pitch_diag_state = 1U;
        }

        return;
    }

    /*
     * State 2:
     * result frozen.
     */

    if (pitch_diag_state == 2U)
    {
        return;
    }

    /*
     * State 1 + first_sign == 0:
     *
     * Armed and waiting for deliberate pitch movement.
     */

    if (pitch_diag_first_sign == 0)
    {
        if (
            absf_local(gyro_pitch_dps)
            >= PITCH_TRIGGER_DPS
        )
        {
            if (gyro_pitch_dps > 0.0f)
            {
                pitch_diag_first_sign = 1;
            }
            else
            {
                pitch_diag_first_sign = -1;
            }

            pitch_diag_first_gyro_dps =
                gyro_pitch_dps;

            pitch_diag_peak_gyro_dps =
                gyro_pitch_dps;

            pitch_diag_pid_at_peak =
                pid_pitch_output;

            pitch_diag_m1_at_peak =
                mix_m1_percent;

            pitch_diag_m2_at_peak =
                mix_m2_percent;

            pitch_diag_m3_at_peak =
                mix_m3_percent;

            pitch_diag_m4_at_peak =
                mix_m4_percent;

            capture_start_ms = system_ms;

            pitch_diag_capture_remaining_ms =
                PITCH_CAPTURE_MS;
        }

        return;
    }

    /*
     * Capture the largest absolute pitch rate.
     */

    current_abs =
        absf_local(gyro_pitch_dps);

    peak_abs =
        absf_local(pitch_diag_peak_gyro_dps);

    if (current_abs > peak_abs)
    {
        pitch_diag_peak_gyro_dps =
            gyro_pitch_dps;

        pitch_diag_pid_at_peak =
            pid_pitch_output;

        pitch_diag_m1_at_peak =
            mix_m1_percent;

        pitch_diag_m2_at_peak =
            mix_m2_percent;

        pitch_diag_m3_at_peak =
            mix_m3_percent;

        pitch_diag_m4_at_peak =
            mix_m4_percent;
    }

    /*
     * Remaining capture time for Live Expressions.
     */

    if (
        (uint32_t)(system_ms - capture_start_ms)
        < PITCH_CAPTURE_MS
    )
    {
        pitch_diag_capture_remaining_ms =
            PITCH_CAPTURE_MS -
            (uint32_t)(
                system_ms -
                capture_start_ms
            );
    }
    else
    {
        pitch_diag_capture_remaining_ms = 0U;

        /*
         * Freeze result.
         */

        pitch_diag_state = 2U;
    }
}


/* ============================================================
 * MAIN
 * ============================================================ */

int main(void)
{
    uint32_t last_loop_ms;
    uint32_t now_ms;
    uint32_t elapsed;

    uint32_t read_start;
    uint32_t read_end;

    /*
     * ========================================================
     * GPIO CLOCKS
     * ========================================================
     */

    RCC_AHBENR |=
        GPIOA_EN |
        GPIOB_EN;

    (void)RCC_AHBENR;


    /*
     * ========================================================
     * BOARD ENABLE PA1
     * ========================================================
     */

    gpio_high(
        GPIOA_BASE,
        1U
    );

    gpio_output(
        GPIOA_BASE,
        1U
    );


    /*
     * ========================================================
     * MOTOR PINS SAFE BEFORE TIM1
     * ========================================================
     */

    motor_gpio_prepare_low();


    /*
     * ========================================================
     * SYSTEM TIME
     * ========================================================
     */

    systick_init();

    delay_ms(20U);


    /*
     * ========================================================
     * HARDWARE I2C
     * ========================================================
     */

    i2c_init();

    delay_ms(20U);


    /*
     * ========================================================
     * M540
     * ========================================================
     */

    if (m540_init() != 0U)
    {
        imu_ok = 1U;
    }
    else
    {
        imu_ok = 0U;
    }


    /*
     * ========================================================
     * TIM1
     *
     * Initialized but all CCR registers remain ZERO.
     * ========================================================
     */

    tim1_init();

    motor_all_off();


    /*
     * ========================================================
     * GYRO CALIBRATION
     * ========================================================
     */

    if (imu_ok != 0U)
    {
        if (gyro_calibrate() == 0U)
        {
            imu_ok = 0U;
        }
    }


    /*
     * ========================================================
     * INITIAL ATTITUDE
     * ========================================================
     */

    if (imu_ok != 0U)
    {
        if (m540_read_raw() != 0U)
        {
            float ay;
            float az;
            float yz;

            accel_roll_deg =
                fast_atan2_deg(
                    (float)accel_y,
                    (float)accel_z
                );

            ay = (float)accel_y;
            az = (float)accel_z;

            yz =
                fast_sqrtf_local(
                    ay * ay +
                    az * az
                );

            accel_pitch_deg =
                fast_atan2_deg(
                    -(float)accel_x,
                    yz
                );

            roll_angle =
                accel_roll_deg;

            pitch_angle =
                accel_pitch_deg;

            yaw_angle = 0.0f;
        }
    }


    /*
     * ========================================================
     * RESET DIAGNOSTIC
     * ========================================================
     */

    pitch_diag_state = 0U;

    pitch_diag_first_sign = 0;

    pitch_diag_first_gyro_dps = 0.0f;
    pitch_diag_peak_gyro_dps = 0.0f;

    pitch_diag_pid_at_peak = 0.0f;

    pitch_diag_m1_at_peak = 0.0f;
    pitch_diag_m2_at_peak = 0.0f;
    pitch_diag_m3_at_peak = 0.0f;
    pitch_diag_m4_at_peak = 0.0f;

    pitch_diag_capture_remaining_ms = 0U;


    /*
     * ========================================================
     * MAIN 200 Hz LOOP
     * ========================================================
     */

    last_loop_ms = system_ms;

    while (1)
    {
        now_ms = system_ms;

        elapsed =
            (uint32_t)(
                now_ms -
                last_loop_ms
            );

        if (elapsed >= LOOP_PERIOD_MS)
        {
            last_loop_ms = now_ms;

            loop_dt_ms = elapsed;

            if (elapsed > loop_dt_max_ms)
            {
                loop_dt_max_ms = elapsed;
            }

            loop_dt =
                (float)elapsed /
                1000.0f;

            if (loop_dt > 0.0f)
            {
                loop_hz =
                    1.0f /
                    loop_dt;
            }


            /*
             * ================================================
             * READ IMU
             * ================================================
             */

            read_start = system_ms;

            if (m540_read_raw() != 0U)
            {
                imu_ok = 1U;

                imu_consecutive_errors = 0U;

                /*
                 * Attitude
                 */

                attitude_update(loop_dt);


                /*
                 * Virtual RATE controller
                 */

                virtual_rate_controller();


                /*
                 * Virtual motor mixer
                 */

                virtual_motor_mixer();


                /*
                 * Automatic pitch direction recorder
                 */

                pitch_diagnostic_update();
            }
            else
            {
                imu_ok = 0U;

                imu_error_counter++;
                imu_consecutive_errors++;
            }

            read_end = system_ms;

            imu_read_time_ms =
                (uint32_t)(
                    read_end -
                    read_start
                );

            if (
                imu_read_time_ms >
                imu_read_time_max_ms
            )
            {
                imu_read_time_max_ms =
                    imu_read_time_ms;
            }


            /*
             * ================================================
             * ABSOLUTE PHYSICAL MOTOR SAFETY
             * ================================================
             *
             * NO CONDITION.
             * NO ARM VARIABLE.
             * NO PID OUTPUT.
             *
             * Hardware motor PWM is always ZERO.
             */

            physical_motor_lock = 1U;

            motor_all_off();


            /*
             * Keep useful I2C register visible.
             */

            i2c_isr_debug = I2C1_ISR;
            i2c_timingr_debug = I2C1_TIMINGR;
        }
    }
} * REAL MOTORS ARE HARD LOCKED OFF.
 *
 * mix_m1_percent ... mix_m4_percent
 * are VIRTUAL motor commands only.
 *
 * ============================================================ */


#define REG32(addr) (*(volatile uint32_t *)(addr))


/* ============================================================
 * RCC
 * ============================================================ */

#define RCC_AHBENR      REG32(0x40021014U)
#define RCC_APB1RSTR    REG32(0x40021010U)
#define RCC_APB1ENR     REG32(0x4002101CU)
#define RCC_APB2RSTR    REG32(0x4002100CU)
#define RCC_APB2ENR     REG32(0x40021018U)
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

#define GPIO_MODER(b)   REG32((b) + 0x00U)
#define GPIO_OTYPER(b)  REG32((b) + 0x04U)
#define GPIO_OSPEEDR(b) REG32((b) + 0x08U)
#define GPIO_PUPDR(b)   REG32((b) + 0x0CU)

#define GPIO_ODR(b)     REG32((b) + 0x14U)
#define GPIO_BSRR(b)    REG32((b) + 0x18U)

#define GPIO_AFRL(b)    REG32((b) + 0x20U)
#define GPIO_AFRH(b)    REG32((b) + 0x24U)


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


/*
 * 8 MHz / 8000 = 1 kHz
 */
#define TIM1_PWM_ARR    7999U


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
#define I2C_CR2_STOP    (1U << 14)

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


#define I2C_TIMEOUT_MS  10U


/* ============================================================
 * SysTick
 * ============================================================ */

#define SYST_CSR        REG32(0xE000E010U)
#define SYST_RVR        REG32(0xE000E014U)
#define SYST_CVR        REG32(0xE000E018U)


/* ============================================================
 * M540
 * ============================================================ */

#define M540_ADDR               0x69U

#define M540_REG_ACCEL_XOUT_H   0x3BU

#define M540_REG_CONFIG         0x1AU
#define M540_REG_GYRO_CONFIG    0x1BU
#define M540_REG_ACCEL_CONFIG   0x1CU
#define M540_REG_ACCEL_CONFIG2  0x1DU

#define M540_REG_PWR_MGMT_1     0x6BU

#define M540_REG_WHO_AM_I       0x75U
#define M540_WHO_AM_I_VALUE     0x7DU


#define ACCEL_LSB_PER_G         2048.0f

#define GYRO_LSB_PER_DPS        16.4f


/* ============================================================
 * CONTROL
 * ============================================================ */

#define LOOP_PERIOD_MS          5U


/*
 * Virtual throttle only.
 *
 * REAL motors remain 0%.
 */
#define VIRTUAL_THROTTLE        50.0f


/*
 * Initial RATE PID.
 *
 * At this stage I and D are intentionally zero.
 *
 * First we verify:
 *
 * gyro sign
 * ->
 * PID sign
 * ->
 * mixer sign
 *
 * before allowing any physical motor response.
 */

#define RATE_ROLL_KP            0.10f
#define RATE_ROLL_KI            0.00f
#define RATE_ROLL_KD            0.00f

#define RATE_PITCH_KP           0.10f
#define RATE_PITCH_KI           0.00f
#define RATE_PITCH_KD           0.00f

#define RATE_YAW_KP             0.08f
#define RATE_YAW_KI             0.00f
#define RATE_YAW_KD             0.00f


#define PID_OUTPUT_LIMIT        35.0f


/* ============================================================
 * GLOBAL DEBUG VARIABLES
 * ============================================================ */


/* ===================== SYSTEM =============================== */

volatile uint32_t system_ms = 0U;


/* ===================== IMU ================================== */

volatile uint32_t imu_ok = 0U;

volatile uint8_t imu_whoami = 0U;


volatile uint32_t calibration_status = 0U;

volatile uint32_t calibration_samples = 0U;


volatile uint32_t imu_frame_counter = 0U;

volatile uint32_t imu_error_counter = 0U;

volatile uint32_t imu_consecutive_errors = 0U;


/* ===================== RAW IMU ============================== */

volatile int16_t accel_x = 0;

volatile int16_t accel_y = 0;

volatile int16_t accel_z = 0;


volatile int16_t gyro_x = 0;

volatile int16_t gyro_y = 0;

volatile int16_t gyro_z = 0;


/* ===================== GYRO OFFSETS ========================= */

volatile float gyro_x_offset = 0.0f;

volatile float gyro_y_offset = 0.0f;

volatile float gyro_z_offset = 0.0f;


/* ===================== GYRO DPS ============================= */

volatile float gyro_roll_dps = 0.0f;

volatile float gyro_pitch_dps = 0.0f;

volatile float gyro_yaw_dps = 0.0f;


/* ===================== SETPOINTS ============================ */

volatile float rate_roll_setpoint_dps = 0.0f;

volatile float rate_pitch_setpoint_dps = 0.0f;

volatile float rate_yaw_setpoint_dps = 0.0f;


/* ===================== PID OUTPUT =========================== */

volatile float pid_roll_output = 0.0f;

volatile float pid_pitch_output = 0.0f;

volatile float pid_yaw_output = 0.0f;


/* ===================== VIRTUAL MIXER ======================== */

volatile float mix_m1_percent = 0.0f;

volatile float mix_m2_percent = 0.0f;

volatile float mix_m3_percent = 0.0f;

volatile float mix_m4_percent = 0.0f;


/* ===================== REAL MOTOR COMMAND ================== */

volatile uint32_t motor1_percent = 0U;

volatile uint32_t motor2_percent = 0U;

volatile uint32_t motor3_percent = 0U;

volatile uint32_t motor4_percent = 0U;


/*
 * ABSOLUTE SAFETY LOCK
 *
 * DO NOT CHANGE THIS TO 0.
 */
volatile uint32_t physical_motor_lock = 1U;


/* ===================== LOOP DEBUG =========================== */

volatile uint32_t loop_dt_ms = 0U;

volatile float loop_dt = 0.005f;

volatile float loop_hz = 0.0f;

volatile uint32_t loop_dt_max_ms = 0U;


volatile uint32_t imu_read_time_ms = 0U;

volatile uint32_t imu_read_time_max_ms = 0U;


/* ===================== I2C DEBUG ============================ */

volatile uint32_t i2c_hw_enabled = 0U;

volatile uint32_t i2c_last_error = 0U;

volatile uint32_t i2c_berr_counter = 0U;

volatile uint32_t i2c_transfer_counter = 0U;

volatile uint32_t i2c_isr_debug = 0U;

volatile uint32_t i2c_timingr_debug = 0U;


/* ============================================================
 * AUTOMATIC ROLL DIAGNOSTIC
 * ============================================================
 *
 * roll_diag_state:
 *
 * 0 = waiting
 * 1 = movement detected / recording
 * 2 = finished / frozen
 *
 * To repeat test:
 *
 * set
 *
 * roll_diag_reset = 1
 *
 * in Live Expressions.
 *
 * ============================================================ */

volatile uint32_t roll_diag_reset = 0U;


volatile uint32_t roll_diag_state = 0U;


volatile int32_t roll_diag_first_sign = 0;


volatile float roll_diag_first_gyro_dps = 0.0f;


volatile float roll_diag_peak_gyro_dps = 0.0f;


volatile float roll_diag_peak_pid_abs = 0.0f;


volatile float roll_diag_pid_at_peak = 0.0f;


volatile float roll_diag_m1_at_peak = 0.0f;

volatile float roll_diag_m2_at_peak = 0.0f;

volatile float roll_diag_m3_at_peak = 0.0f;

volatile float roll_diag_m4_at_peak = 0.0f;


volatile uint32_t roll_diag_quiet_ms = 0U;


/* ============================================================
 * HELPERS
 * ============================================================ */

static float abs_float(float x)
{
    if (x < 0.0f)
    {
        return -x;
    }

    return x;
}


static float clamp_float(
    float x,
    float lo,
    float hi
)
{
    if (x < lo)
    {
        return lo;
    }

    if (x > hi)
    {
        return hi;
    }

    return x;
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

    GPIO_MODER(base) &=
        ~(3U << shift);

    GPIO_MODER(base) |=
        (1U << shift);
}


/* ============================================================
 * SYSTEM TIME
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
     * 8,000,000 / 8000 = 1000 Hz
     *
     * therefore:
     *
     * SysTick = 1 ms
     */

    SYST_RVR = 7999U;

    SYST_CVR = 0U;


    SYST_CSR =
        (1U << 2) |
        (1U << 1) |
        (1U << 0);
}


static void delay_ms(
    uint32_t ms
)
{
    uint32_t start;

    start = system_ms;


    while (
        (uint32_t)(system_ms - start)
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
    gpio_low(
        GPIOA_BASE,
        8U
    );

    gpio_low(
        GPIOA_BASE,
        9U
    );

    gpio_low(
        GPIOA_BASE,
        10U
    );

    gpio_low(
        GPIOA_BASE,
        11U
    );


    gpio_output(
        GPIOA_BASE,
        8U
    );

    gpio_output(
        GPIOA_BASE,
        9U
    );

    gpio_output(
        GPIOA_BASE,
        10U
    );

    gpio_output(
        GPIOA_BASE,
        11U
    );
}


/* ============================================================
 * TIM1 MOTOR PWM INITIALIZATION
 * ============================================================ */

static void tim1_motor_init(void)
{
    uint32_t v;
    uint32_t pin;
    uint32_t shift;


    RCC_APB2ENR |=
        TIM1_EN;


    /*
     * Reset TIM1
     */

    RCC_APB2RSTR |=
        TIM1_EN;

    RCC_APB2RSTR &=
        ~TIM1_EN;


    /*
     * PA8
     * PA9
     * PA10
     * PA11
     *
     * Alternate Function mode
     */

    for (
        pin = 8U;
        pin <= 11U;
        pin++
    )
    {
        shift =
            pin * 2U;


        GPIO_MODER(GPIOA_BASE) &=
            ~(3U << shift);


        GPIO_MODER(GPIOA_BASE) |=
            (2U << shift);
    }


    /*
     * AF2:
     *
     * PA8  = TIM1_CH1
     * PA9  = TIM1_CH2
     * PA10 = TIM1_CH3
     * PA11 = TIM1_CH4
     */

    v =
        GPIO_AFRH(GPIOA_BASE);


    v &=
        ~(
            (0xFU << 0)  |
            (0xFU << 4)  |
            (0xFU << 8)  |
            (0xFU << 12)
        );


    v |=
        (
            (2U << 0)  |
            (2U << 4)  |
            (2U << 8)  |
            (2U << 12)
        );


    GPIO_AFRH(GPIOA_BASE) =
        v;


    /*
     * Stop timer while configuring
     */

    TIM1_CR1 =
        0U;


    TIM1_PSC =
        0U;


    TIM1_ARR =
        TIM1_PWM_ARR;


    /*
     * REAL MOTOR DUTY = ZERO
     */

    TIM1_CCR1 =
        0U;

    TIM1_CCR2 =
        0U;

    TIM1_CCR3 =
        0U;

    TIM1_CCR4 =
        0U;


    /*
     * PWM MODE 1
     *
     * CH1 + CH2
     */

    TIM1_CCMR1 =
        (6U << 4) |
        (1U << 3) |
        (6U << 12) |
        (1U << 11);


    /*
     * PWM MODE 1
     *
     * CH3 + CH4
     */

    TIM1_CCMR2 =
        (6U << 4) |
        (1U << 3) |
        (6U << 12) |
        (1U << 11);


    /*
     * Enable CH1..CH4
     */

    TIM1_CCER =
        (1U << 0) |
        (1U << 4) |
        (1U << 8) |
        (1U << 12);


    /*
     * Main Output Enable
     */

    TIM1_BDTR =
        (1U << 15);


    /*
     * Generate update event
     */

    TIM1_EGR =
        1U;


    /*
     * ARPE
     * CEN
     */

    TIM1_CR1 =
        (1U << 7) |
        (1U << 0);
}


/* ============================================================
 * FORCE ALL PHYSICAL MOTORS OFF
 * ============================================================ */

static void motor_all_off(void)
{
    /*
     * Electrical mapping:
     *
     * TIM1_CH4 -> M1
     * TIM1_CH2 -> M2
     * TIM1_CH1 -> M3
     * TIM1_CH3 -> M4
     */


    TIM1_CCR4 =
        0U;


    TIM1_CCR2 =
        0U;


    TIM1_CCR1 =
        0U;


    TIM1_CCR3 =
        0U;


    motor1_percent =
        0U;


    motor2_percent =
        0U;


    motor3_percent =
        0U;


    motor4_percent =
        0U;
}


/* ============================================================
 * I2C GPIO INITIALIZATION
 * ============================================================ */

static void i2c_gpio_init(void)
{
    uint32_t v;


    /*
     * PB7 = SDA
     * PB8 = SCL
     *
     * Alternate Function mode
     */

    GPIO_MODER(GPIOB_BASE) &=
        ~(
            (3U << (7U * 2U)) |
            (3U << (8U * 2U))
        );


    GPIO_MODER(GPIOB_BASE) |=
        (
            (2U << (7U * 2U)) |
            (2U << (8U * 2U))
        );


    /*
     * Open Drain
     */

    GPIO_OTYPER(GPIOB_BASE) |=
        (1U << 7) |
        (1U << 8);


    /*
     * High speed
     */

    GPIO_OSPEEDR(GPIOB_BASE) |=
        (
            (3U << (7U * 2U)) |
            (3U << (8U * 2U))
        );


    /*
     * Internal pull-ups
     */

    GPIO_PUPDR(GPIOB_BASE) &=
        ~(
            (3U << (7U * 2U)) |
            (3U << (8U * 2U))
        );


    GPIO_PUPDR(GPIOB_BASE) |=
        (
            (1U << (7U * 2U)) |
            (1U << (8U * 2U))
        );


    /*
     * PB7 = AF1
     */

    v =
        GPIO_AFRL(GPIOB_BASE);


    v &=
        ~(0xFU << 28);


    v |=
        (1U << 28);


    GPIO_AFRL(GPIOB_BASE) =
        v;


    /*
     * PB8 = AF1
     */

    v =
        GPIO_AFRH(GPIOB_BASE);


    v &=
        ~(0xFU << 0);


    v |=
        (1U << 0);


    GPIO_AFRH(GPIOB_BASE) =
        v;
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


/* ============================================================
 * I2C ERROR CHECK
 * ============================================================ */

static int i2c_check_errors(void)
{
    uint32_t isr;


    isr =
        I2C1_ISR;


    i2c_isr_debug =
        isr;


    /*
     * STM32F031 erratum:
     *
     * spurious BERR can occur in master mode.
     *
     * Therefore:
     *
     * count it,
     * clear it,
     * continue.
     */

    if (
        isr &
        I2C_ISR_BERR
    )
    {
        I2C1_ICR =
            I2C_ICR_BERRCF;


        i2c_berr_counter++;
    }


    if (
        isr &
        I2C_ISR_NACKF
    )
    {
        I2C1_ICR =
            I2C_ICR_NACKCF;


        i2c_last_error =
            1U;


        return 0;
    }


    if (
        isr &
        I2C_ISR_ARLO
    )
    {
        I2C1_ICR =
            I2C_ICR_ARLOCF;


        i2c_last_error =
            2U;


        return 0;
    }


    if (
        isr &
        I2C_ISR_OVR
    )
    {
        I2C1_ICR =
            I2C_ICR_OVRCF;


        i2c_last_error =
            3U;


        return 0;
    }


    return 1;
}


/* ============================================================
 * I2C WAIT FOR FLAG
 * ============================================================ */

static int i2c_wait_for(
    uint32_t flag
)
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
            !i2c_check_errors()
        )
        {
            return 0;
        }


        if (
            (uint32_t)(
                system_ms -
                start
            )
            >=
            I2C_TIMEOUT_MS
        )
        {
            i2c_last_error =
                4U;


            return 0;
        }
    }


    i2c_isr_debug =
        I2C1_ISR;


    return 1;
}


/* ============================================================
 * I2C WAIT BUS FREE
 * ============================================================ */

static int i2c_wait_not_busy(void)
{
    uint32_t start;


    start =
        system_ms;


    while (
        I2C1_ISR &
        I2C_ISR_BUSY
    )
    {
        if (
            (uint32_t)(
                system_ms -
                start
            )
            >=
            I2C_TIMEOUT_MS
        )
        {
            i2c_last_error =
                5U;


            return 0;
        }
    }


    return 1;
}


/* ============================================================
 * I2C INITIALIZATION
 * ============================================================ */

static void i2c_init(void)
{
    /*
     * I2C1 clock source = HSI
     */

    RCC_CFGR3 &=
        ~(1U << 4);


    /*
     * Enable I2C1 clock
     */

    RCC_APB1ENR |=
        I2C1_EN;


    /*
     * Reset I2C1
     */

    RCC_APB1RSTR |=
        I2C1_EN;


    RCC_APB1RSTR &=
        ~I2C1_EN;


    /*
     * GPIO
     */

    i2c_gpio_init();


    /*
     * Disable I2C before TIMINGR configuration
     */

    I2C1_CR1 &=
        ~I2C_CR1_PE;


    /*
     * 8 MHz I2C kernel clock
     *
     * Standard Mode = 100 kHz
     *
     * PRESC  = 1
     * SCLDEL = 4
     * SDADEL = 2
     * SCLH   = 0x0F
     * SCLL   = 0x13
     */

    I2C1_TIMINGR =
        0x10420F13U;


    i2c_timingr_debug =
        I2C1_TIMINGR;


    /*
     * Clear errors
     */

    i2c_clear_flags();


    /*
     * Enable peripheral
     */

    I2C1_CR1 |=
        I2C_CR1_PE;


    i2c_hw_enabled =
        1U;


    i2c_last_error =
        0U;
}


/* ============================================================
 * M540 WRITE REGISTER
 * ============================================================ */

static int m540_write_reg(
    uint8_t reg,
    uint8_t value
)
{
    uint32_t cr2;


    if (
        !i2c_wait_not_busy()
    )
    {
        return 0;
    }


    i2c_last_error =
        0U;


    i2c_clear_flags();


    /*
     * 7-bit address shifted left
     *
     * NBYTES = 2
     *
     * byte 1 = register
     * byte 2 = value
     */

    cr2 =
        ((uint32_t)M540_ADDR << 1) |
        (2U << 16) |
        I2C_CR2_AUTOEND |
        I2C_CR2_START;


    I2C1_CR2 =
        cr2;


    /*
     * Send register
     */

    if (
        !i2c_wait_for(
            I2C_ISR_TXIS
        )
    )
    {
        return 0;
    }


    I2C1_TXDR =
        reg;


    /*
     * Send value
     */

    if (
        !i2c_wait_for(
            I2C_ISR_TXIS
        )
    )
    {
        return 0;
    }


    I2C1_TXDR =
        value;


    /*
     * Wait STOP
     */

    if (
        !i2c_wait_for(
            I2C_ISR_STOPF
        )
    )
    {
        return 0;
    }


    I2C1_ICR =
        I2C_ICR_STOPCF;


    i2c_transfer_counter++;


    return 1;
}


/* ============================================================
 * M540 READ REGISTERS
 * ============================================================ */

static int m540_read_regs(
    uint8_t reg,
    uint8_t *data,
    uint32_t len
)
{
    uint32_t cr2;

    uint32_t i;


    if (
        len == 0U
    )
    {
        return 0;
    }


    if (
        len > 255U
    )
    {
        return 0;
    }


    if (
        !i2c_wait_not_busy()
    )
    {
        return 0;
    }


    i2c_last_error =
        0U;


    i2c_clear_flags();


    /*
     * PHASE 1
     *
     * Send register address.
     *
     * NO AUTOEND.
     */

    cr2 =
        ((uint32_t)M540_ADDR << 1) |
        (1U << 16) |
        I2C_CR2_START;


    I2C1_CR2 =
        cr2;


    if (
        !i2c_wait_for(
            I2C_ISR_TXIS
        )
    )
    {
        return 0;
    }


    I2C1_TXDR =
        reg;


    /*
     * Wait Transfer Complete
     */

    if (
        !i2c_wait_for(
            I2C_ISR_TC
        )
    )
    {
        return 0;
    }


    /*
     * PHASE 2
     *
     * Repeated START
     *
     * Read len bytes
     */

    cr2 =
        ((uint32_t)M540_ADDR << 1) |
        I2C_CR2_RD_WRN |
        (len << 16) |
        I2C_CR2_AUTOEND |
        I2C_CR2_START;


    I2C1_CR2 =
        cr2;


    for (
        i = 0U;
        i < len;
        i++
    )
    {
        if (
            !i2c_wait_for(
                I2C_ISR_RXNE
            )
        )
        {
            return 0;
        }


        data[i] =
            (uint8_t)I2C1_RXDR;
    }


    /*
     * Wait STOP
     */

    if (
        !i2c_wait_for(
            I2C_ISR_STOPF
        )
    )
    {
        return 0;
    }


    I2C1_ICR =
        I2C_ICR_STOPCF;


    i2c_transfer_counter++;


    return 1;
}


/* ============================================================
 * M540 READ SINGLE REGISTER
 * ============================================================ */

static int m540_read_reg(
    uint8_t reg,
    uint8_t *value
)
{
    return m540_read_regs(
        reg,
        value,
        1U
    );
}


/* ============================================================
 * M540 INITIALIZATION
 * ============================================================ */

static int m540_init(void)
{
    uint8_t who;


    who =
        0U;


    imu_ok =
        0U;


    /*
     * Initial WHO_AM_I
     */

    if (
        !m540_read_reg(
            M540_REG_WHO_AM_I,
            &who
        )
    )
    {
        return 0;
    }


    imu_whoami =
        who;


    if (
        who !=
        M540_WHO_AM_I_VALUE
    )
    {
        return 0;
    }


    /*
     * Reset
     */

    if (
        !m540_write_reg(
            M540_REG_PWR_MGMT_1,
            0x80U
        )
    )
    {
        return 0;
    }


    delay_ms(
        100U
    );


    /*
     * Wake
     *
     * clock = 1
     */

    if (
        !m540_write_reg(
            M540_REG_PWR_MGMT_1,
            0x01U
        )
    )
    {
        return 0;
    }


    delay_ms(
        10U
    );


    /*
     * Accelerometer
     *
     * +-16 g
     */

    if (
        !m540_write_reg(
            M540_REG_ACCEL_CONFIG,
            0x18U
        )
    )
    {
        return 0;
    }


    /*
     * Accel filter
     */

    if (
        !m540_write_reg(
            M540_REG_ACCEL_CONFIG2,
            0x05U
        )
    )
    {
        return 0;
    }


    /*
     * Gyroscope
     *
     * +-2000 dps
     */

    if (
        !m540_write_reg(
            M540_REG_GYRO_CONFIG,
            0x18U
        )
    )
    {
        return 0;
    }


    /*
     * Gyro filter / raw mode
     */

    if (
        !m540_write_reg(
            M540_REG_CONFIG,
            0x00U
        )
    )
    {
        return 0;
    }


    delay_ms(
        10U
    );


    /*
     * Verify WHO_AM_I again
     */

    if (
        !m540_read_reg(
            M540_REG_WHO_AM_I,
            &who
        )
    )
    {
        return 0;
    }


    imu_whoami =
        who;


    if (
        who !=
        M540_WHO_AM_I_VALUE
    )
    {
        return 0;
    }


    imu_ok =
        1U;


    return 1;
}


/* ============================================================
 * INT16 CONVERSION
 * ============================================================ */

static int16_t make_i16(
    uint8_t hi,
    uint8_t lo
)
{
    uint16_t v;


    v =
        ((uint16_t)hi << 8) |
        (uint16_t)lo;


    return (int16_t)v;
}


/* ============================================================
 * READ M540 RAW FRAME
 * ============================================================ */

static int m540_read_raw(void)
{
    uint8_t b[14];


    if (
        !m540_read_regs(
            M540_REG_ACCEL_XOUT_H,
            b,
            14U
        )
    )
    {
        imu_error_counter++;

        imu_consecutive_errors++;


        imu_ok =
            0U;


        return 0;
    }


    /*
     * ACCEL
     */

    accel_x =
        make_i16(
            b[0],
            b[1]
        );


    accel_y =
        make_i16(
            b[2],
            b[3]
        );


    accel_z =
        make_i16(
            b[4],
            b[5]
        );


    /*
     * b[6], b[7]
     *
     * temperature
     */


    /*
     * GYRO
     */

    gyro_x =
        make_i16(
            b[8],
            b[9]
        );


    gyro_y =
        make_i16(
            b[10],
            b[11]
        );


    gyro_z =
        make_i16(
            b[12],
            b[13]
        );


    imu_frame_counter++;


    imu_consecutive_errors =
        0U;


    imu_ok =
        1U;


    return 1;
}


/* ============================================================
 * GYRO CALIBRATION
 * ============================================================ */

static void gyro_calibrate(void)
{
    int32_t sum_x;

    int32_t sum_y;

    int32_t sum_z;


    uint32_t good;


    sum_x =
        0;


    sum_y =
        0;


    sum_z =
        0;


    good =
        0U;


    calibration_status =
        1U;


    calibration_samples =
        0U;


    /*
     * Drone MUST remain still here.
     */

    while (
        good < 500U
    )
    {
        if (
            m540_read_raw()
        )
        {
            sum_x +=
                gyro_x;


            sum_y +=
                gyro_y;


            sum_z +=
                gyro_z;


            good++;


            calibration_samples =
                good;
        }


        delay_ms(
            3U
        );
    }


    gyro_x_offset =
        (float)sum_x /
        500.0f;


    gyro_y_offset =
        (float)sum_y /
        500.0f;


    gyro_z_offset =
        (float)sum_z /
        500.0f;


    calibration_status =
        2U;
}


/* ============================================================
 * RATE PID
 * ============================================================ */

typedef struct
{
    float kp;

    float ki;

    float kd;


    float integral;

    float previous_error;

} RatePID;


/* ============================================================
 * PID INSTANCES
 * ============================================================ */

static RatePID roll_pid =
{
    RATE_ROLL_KP,
    RATE_ROLL_KI,
    RATE_ROLL_KD,

    0.0f,
    0.0f
};


static RatePID pitch_pid =
{
    RATE_PITCH_KP,
    RATE_PITCH_KI,
    RATE_PITCH_KD,

    0.0f,
    0.0f
};


static RatePID yaw_pid =
{
    RATE_YAW_KP,
    RATE_YAW_KI,
    RATE_YAW_KD,

    0.0f,
    0.0f
};


/* ============================================================
 * PID UPDATE
 * ============================================================ */

static float rate_pid_update(
    RatePID *pid,
    float setpoint,
    float measured,
    float dt
)
{
    float error;

    float derivative;

    float output;


    if (
        dt <= 0.0f
    )
    {
        dt =
            0.005f;
    }


    /*
     * PID error
     *
     * Desired angular velocity
     * minus
     * measured angular velocity
     */

    error =
        setpoint -
        measured;


    /*
     * Integral
     */

    pid->integral +=
        error *
        dt;


    pid->integral =
        clamp_float(
            pid->integral,
            -100.0f,
            100.0f
        );


    /*
     * Derivative
     */

    derivative =
        (
            error -
            pid->previous_error
        )
        /
        dt;


    pid->previous_error =
        error;


    /*
     * PID output
     */

    output =
        (
            pid->kp *
            error
        )
        +
        (
            pid->ki *
            pid->integral
        )
        +
        (
            pid->kd *
            derivative
        );


    /*
     * Limit
     */

    output =
        clamp_float(
            output,
            -PID_OUTPUT_LIMIT,
            PID_OUTPUT_LIMIT
        );


    return output;
}


/* ============================================================
 * RATE CONTROLLER
 * ============================================================ */

static void rate_controller_update(
    float dt
)
{
    pid_roll_output =
        rate_pid_update(
            &roll_pid,

            rate_roll_setpoint_dps,

            gyro_roll_dps,

            dt
        );


    pid_pitch_output =
        rate_pid_update(
            &pitch_pid,

            rate_pitch_setpoint_dps,

            gyro_pitch_dps,

            dt
        );


    pid_yaw_output =
        rate_pid_update(
            &yaw_pid,

            rate_yaw_setpoint_dps,

            gyro_yaw_dps,

            dt
        );
}


/* ============================================================
 * VIRTUAL MOTOR MIXER
 * ============================================================ */

static void motor_mixer_update(void)
{
    float base;


    base =
        VIRTUAL_THROTTLE;


    /*
     * TOP VIEW
     *
     *          FRONT
     *
     *      M1         M2
     *
     *      CW        CCW
     *
     *
     *      M4         M3
     *
     *     CCW         CW
     *
     *
     * This is still only a VIRTUAL mixer.
     *
     * The diagnostic test is specifically intended
     * to verify the correction signs before these
     * values can ever reach the real motors.
     */


    mix_m1_percent =
        base
        -
        pid_roll_output
        -
        pid_pitch_output
        -
        pid_yaw_output;


    mix_m2_percent =
        base
        +
        pid_roll_output
        -
        pid_pitch_output
        +
        pid_yaw_output;


    mix_m3_percent =
        base
        +
        pid_roll_output
        +
        pid_pitch_output
        -
        pid_yaw_output;


    mix_m4_percent =
        base
        -
        pid_roll_output
        +
        pid_pitch_output
        +
        pid_yaw_output;


    /*
     * Limit virtual commands
     */

    mix_m1_percent =
        clamp_float(
            mix_m1_percent,
            0.0f,
            100.0f
        );


    mix_m2_percent =
        clamp_float(
            mix_m2_percent,
            0.0f,
            100.0f
        );


    mix_m3_percent =
        clamp_float(
            mix_m3_percent,
            0.0f,
            100.0f
        );


    mix_m4_percent =
        clamp_float(
            mix_m4_percent,
            0.0f,
            100.0f
        );
}


/* ============================================================
 * RESET ROLL DIAGNOSTIC
 * ============================================================ */

static void roll_pid_diagnostic_reset_now(void)
{
    roll_diag_state =
        0U;


    roll_diag_first_sign =
        0;


    roll_diag_first_gyro_dps =
        0.0f;


    roll_diag_peak_gyro_dps =
        0.0f;


    roll_diag_peak_pid_abs =
        0.0f;


    roll_diag_pid_at_peak =
        0.0f;


    roll_diag_m1_at_peak =
        0.0f;


    roll_diag_m2_at_peak =
        0.0f;


    roll_diag_m3_at_peak =
        0.0f;


    roll_diag_m4_at_peak =
        0.0f;


    roll_diag_quiet_ms =
        0U;
}


/* ============================================================
 * AUTOMATIC ROLL PID DIRECTION RECORDER
 * ============================================================ */

static void roll_pid_diagnostic_update(void)
{
    float roll_abs;

    float pitch_abs;

    float yaw_abs;

    float pid_abs;


    /*
     * Manual reset from Live Expressions
     */

    if (
        roll_diag_reset != 0U
    )
    {
        roll_diag_reset =
            0U;


        roll_pid_diagnostic_reset_now();


        return;
    }


    /*
     * State 2 = finished.
     *
     * Freeze everything.
     */

    if (
        roll_diag_state == 2U
    )
    {
        return;
    }


    roll_abs =
        abs_float(
            gyro_roll_dps
        );


    pitch_abs =
        abs_float(
            gyro_pitch_dps
        );


    yaw_abs =
        abs_float(
            gyro_yaw_dps
        );


    /* ========================================================
     * STATE 0
     *
     * WAITING FOR ROLL MOVEMENT
     * ======================================================== */

    if (
        roll_diag_state == 0U
    )
    {
        /*
         * Movement must be:
         *
         * > 20 degrees/sec
         *
         * and ROLL must clearly dominate
         * pitch and yaw.
         */

        if (
            (roll_abs > 20.0f)
            &&
            (
                roll_abs >
                (pitch_abs * 1.5f)
            )
            &&
            (
                roll_abs >
                (yaw_abs * 1.5f)
            )
        )
        {
            roll_diag_state =
                1U;


            roll_diag_first_gyro_dps =
                gyro_roll_dps;


            if (
                gyro_roll_dps >= 0.0f
            )
            {
                roll_diag_first_sign =
                    1;
            }
            else
            {
                roll_diag_first_sign =
                    -1;
            }


            /*
             * Initial peak
             */

            roll_diag_peak_gyro_dps =
                gyro_roll_dps;


            roll_diag_peak_pid_abs =
                abs_float(
                    pid_roll_output
                );


            roll_diag_pid_at_peak =
                pid_roll_output;


            roll_diag_m1_at_peak =
                mix_m1_percent;


            roll_diag_m2_at_peak =
                mix_m2_percent;


            roll_diag_m3_at_peak =
                mix_m3_percent;


            roll_diag_m4_at_peak =
                mix_m4_percent;


            roll_diag_quiet_ms =
                0U;
        }


        return;
    }


    /* ========================================================
     * STATE 1
     *
     * RECORDING
     * ======================================================== */

    if (
        roll_diag_state == 1U
    )
    {
        pid_abs =
            abs_float(
                pid_roll_output
            );


        /*
         * Save the instant when the PID
         * correction is strongest.
         */

        if (
            pid_abs >
            roll_diag_peak_pid_abs
        )
        {
            roll_diag_peak_pid_abs =
                pid_abs;


            roll_diag_peak_gyro_dps =
                gyro_roll_dps;


            roll_diag_pid_at_peak =
                pid_roll_output;


            roll_diag_m1_at_peak =
                mix_m1_percent;


            roll_diag_m2_at_peak =
                mix_m2_percent;


            roll_diag_m3_at_peak =
                mix_m3_percent;


            roll_diag_m4_at_peak =
                mix_m4_percent;
        }


        /*
         * Movement is considered finished when
         * roll rate remains under 4 deg/s
         * for approximately 300 ms.
         *
         * Loop = 5 ms
         */

        if (
            roll_abs <
            4.0f
        )
        {
            roll_diag_quiet_ms +=
                LOOP_PERIOD_MS;


            if (
                roll_diag_quiet_ms >=
                300U
            )
            {
                /*
                 * FREEZE RESULT
                 */

                roll_diag_state =
                    2U;
            }
        }
        else
        {
            roll_diag_quiet_ms =
                0U;
        }
    }
}


/* ============================================================
 * MAIN
 * ============================================================ */

int main(void)
{
    uint32_t last_loop_ms;

    uint32_t now_ms;

    uint32_t elapsed;


    uint32_t read_start;

    uint32_t read_end;


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
     * SYSTEM TIME
     * ======================================================== */

    systick_init();


    delay_ms(
        10U
    );


    /* ========================================================
     * TIM1
     *
     * initialized,
     * but ALL CCR registers remain ZERO.
     * ======================================================== */

    tim1_motor_init();


    motor_all_off();


    /* ========================================================
     * HARDWARE I2C
     * ======================================================== */

    i2c_init();


    delay_ms(
        10U
    );


    /* ========================================================
     * M540
     * ======================================================== */

    if (
        !m540_init()
    )
    {
        imu_ok =
            0U;


        /*
         * IMU FAIL
         *
         * permanent motor OFF
         */

        while (1)
        {
            motor_all_off();
        }
    }


    /* ========================================================
     * GYRO CALIBRATION
     * ========================================================
     *
     * DO NOT MOVE DRONE HERE.
     *
     * ~1.5 seconds.
     * ======================================================== */

    gyro_calibrate();


    /* ========================================================
     * RESET RUNTIME COUNTERS
     * ======================================================== */

    imu_frame_counter =
        0U;


    imu_error_counter =
        0U;


    imu_consecutive_errors =
        0U;


    i2c_berr_counter =
        0U;


    i2c_transfer_counter =
        0U;


    /* ========================================================
     * RESET ROLL TEST
     * ======================================================== */

    roll_pid_diagnostic_reset_now();


    /* ========================================================
     * RATE SETPOINTS
     * ========================================================
     *
     * Zero means:
     *
     * "I want zero angular velocity."
     *
     * Therefore when we rotate the drone by hand,
     * PID should command the opposite torque.
     * ======================================================== */

    rate_roll_setpoint_dps =
        0.0f;


    rate_pitch_setpoint_dps =
        0.0f;


    rate_yaw_setpoint_dps =
        0.0f;


    /* ========================================================
     * ABSOLUTE MOTOR LOCK
     * ======================================================== */

    physical_motor_lock =
        1U;


    motor_all_off();


    /* ========================================================
     * START LOOP
     * ======================================================== */

    last_loop_ms =
        system_ms;


    while (1)
    {
        now_ms =
            system_ms;


        elapsed =
            (uint32_t)(
                now_ms -
                last_loop_ms
            );


        /*
         * 200 Hz
         *
         * 5 ms
         */

        if (
            elapsed >=
            LOOP_PERIOD_MS
        )
        {
            last_loop_ms =
                now_ms;


            /* ================================================
             * LOOP TIMING
             * ================================================ */

            loop_dt_ms =
                elapsed;


            loop_dt =
                (float)elapsed *
                0.001f;


            if (
                loop_dt >
                0.0f
            )
            {
                loop_hz =
                    1.0f /
                    loop_dt;
            }


            if (
                elapsed >
                loop_dt_max_ms
            )
            {
                loop_dt_max_ms =
                    elapsed;
            }


            /* ================================================
             * READ IMU
             * ================================================ */

            read_start =
                system_ms;


            if (
                m540_read_raw()
            )
            {
                read_end =
                    system_ms;


                imu_read_time_ms =
                    (uint32_t)(
                        read_end -
                        read_start
                    );


                if (
                    imu_read_time_ms >
                    imu_read_time_max_ms
                )
                {
                    imu_read_time_max_ms =
                        imu_read_time_ms;
                }


                /* ============================================
                 * RAW GYRO -> deg/s
                 * ============================================
                 *
                 * Confirmed mapping:
                 *
                 * raw X = ROLL
                 * raw Y = PITCH
                 * raw Z = YAW
                 * ============================================ */

                gyro_roll_dps =
                    (
                        (float)gyro_x -
                        gyro_x_offset
                    )
                    /
                    GYRO_LSB_PER_DPS;


                gyro_pitch_dps =
                    (
                        (float)gyro_y -
                        gyro_y_offset
                    )
                    /
                    GYRO_LSB_PER_DPS;


                gyro_yaw_dps =
                    (
                        (float)gyro_z -
                        gyro_z_offset
                    )
                    /
                    GYRO_LSB_PER_DPS;


                /* ============================================
                 * RATE PID
                 * ============================================ */

                rate_controller_update(
                    loop_dt
                );


                /* ============================================
                 * VIRTUAL MOTOR MIXER
                 * ============================================ */

                motor_mixer_update();


                /* ============================================
                 * AUTOMATIC ROLL TEST
                 * ============================================ */

                roll_pid_diagnostic_update();
            }
            else
            {
                /*
                 * IMU failure:
                 *
                 * PID = zero
                 * virtual motors = zero
                 */

                pid_roll_output =
                    0.0f;


                pid_pitch_output =
                    0.0f;


                pid_yaw_output =
                    0.0f;


                mix_m1_percent =
                    0.0f;


                mix_m2_percent =
                    0.0f;


                mix_m3_percent =
                    0.0f;


                mix_m4_percent =
                    0.0f;
            }


            /* ================================================
             * ABSOLUTE PHYSICAL MOTOR SAFETY
             * ================================================
             *
             * IMPORTANT:
             *
             * mix_m1_percent...
             * mix_m4_percent...
             *
             * ARE NOT SENT TO TIM1.
             *
             * CCR1..CCR4 are forced ZERO every loop.
             *
             * Even if physical_motor_lock is accidentally
             * changed in Live Expressions, this firmware
             * still keeps the motors OFF.
             * ================================================ */

            physical_motor_lock =
                1U;


            motor_all_off();
        }
    }
}
