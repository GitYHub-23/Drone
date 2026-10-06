#include <stdint.h>

/* ============================================================
   DRONE_ - FLIGHT CONTROLLER STEP 3.2

   STM32F031K4 + M540 + TIM1

   IMU:
       PB8 = I2C1_SCL AF1
       PB7 = I2C1_SDA AF1
       address = 0x69
       WHO_AM_I = 0x7D

   MOTORS:
       PA8  TIM1_CH1 -> M3
       PA9  TIM1_CH2 -> M2
       PA10 TIM1_CH3 -> M4
       PA11 TIM1_CH4 -> M1

   STEP 3.2:
       - Hardware I2C1 @ 100 kHz
       - 200 Hz flight loop
       - M540 gyro calibration
       - Roll / Pitch complementary filter
       - Gyro direction recorder
       - Motors HARD LOCKED to 0%

   IMPORTANT:
       PROPELLERS MUST BE REMOVED.
   ============================================================ */


/* ============================================================
   Register helper
   ============================================================ */

#define REG32(addr) (*(volatile uint32_t *)(addr))


/* ============================================================
   RCC
   ============================================================ */

#define RCC_AHBENR      REG32(0x40021014U)
#define RCC_APB1RSTR    REG32(0x40021010U)
#define RCC_APB1ENR     REG32(0x4002101CU)
#define RCC_APB2RSTR    REG32(0x4002100CU)
#define RCC_APB2ENR     REG32(0x40021018U)
#define RCC_CFGR3       REG32(0x40021030U)

#define GPIOA_EN        (1U << 17)
#define GPIOB_EN        (1U << 18)

#define I2C1_EN         (1U << 21)
#define TIM1_EN         (1U << 11)


/* ============================================================
   GPIO
   ============================================================ */

#define GPIOA_BASE      0x48000000U
#define GPIOB_BASE      0x48000400U

#define GPIO_MODER(p)   REG32((p) + 0x00U)
#define GPIO_OTYPER(p)  REG32((p) + 0x04U)
#define GPIO_OSPEEDR(p) REG32((p) + 0x08U)
#define GPIO_PUPDR(p)   REG32((p) + 0x0CU)
#define GPIO_IDR(p)     REG32((p) + 0x10U)
#define GPIO_BSRR(p)    REG32((p) + 0x18U)
#define GPIO_AFRL(p)    REG32((p) + 0x20U)
#define GPIO_AFRH(p)    REG32((p) + 0x24U)


/* ============================================================
   SysTick
   ============================================================ */

#define SYST_CSR        REG32(0xE000E010U)
#define SYST_RVR        REG32(0xE000E014U)
#define SYST_CVR        REG32(0xE000E018U)


/* ============================================================
   I2C1
   ============================================================ */

#define I2C1_BASE       0x40005400U

#define I2C1_CR1        REG32(I2C1_BASE + 0x00U)
#define I2C1_CR2        REG32(I2C1_BASE + 0x04U)
#define I2C1_TIMINGR    REG32(I2C1_BASE + 0x10U)
#define I2C1_ISR        REG32(I2C1_BASE + 0x18U)
#define I2C1_ICR        REG32(I2C1_BASE + 0x1CU)
#define I2C1_RXDR       REG32(I2C1_BASE + 0x24U)
#define I2C1_TXDR       REG32(I2C1_BASE + 0x28U)

/* CR1 */

#define I2C_CR1_PE      (1U << 0)

/* CR2 */

#define I2C_CR2_RD_WRN  (1U << 10)
#define I2C_CR2_START   (1U << 13)
#define I2C_CR2_STOP    (1U << 14)
#define I2C_CR2_AUTOEND (1U << 25)

/* ISR */

#define I2C_ISR_TXIS    (1U << 1)
#define I2C_ISR_RXNE    (1U << 2)
#define I2C_ISR_NACKF   (1U << 4)
#define I2C_ISR_STOPF   (1U << 5)
#define I2C_ISR_TC      (1U << 6)

#define I2C_ISR_BERR    (1U << 8)
#define I2C_ISR_ARLO    (1U << 9)
#define I2C_ISR_OVR     (1U << 10)

#define I2C_ISR_BUSY    (1U << 15)

/* ICR */

#define I2C_ICR_NACKCF  (1U << 4)
#define I2C_ICR_STOPCF  (1U << 5)
#define I2C_ICR_BERRCF  (1U << 8)
#define I2C_ICR_ARLOCF  (1U << 9)
#define I2C_ICR_OVRCF   (1U << 10)


/* ============================================================
   TIM1
   ============================================================ */

#define TIM1_BASE       0x40012C00U

#define TIM1_CR1        REG32(TIM1_BASE + 0x00U)
#define TIM1_CR2        REG32(TIM1_BASE + 0x04U)
#define TIM1_SMCR       REG32(TIM1_BASE + 0x08U)
#define TIM1_DIER       REG32(TIM1_BASE + 0x0CU)
#define TIM1_SR         REG32(TIM1_BASE + 0x10U)
#define TIM1_EGR        REG32(TIM1_BASE + 0x14U)

#define TIM1_CCMR1      REG32(TIM1_BASE + 0x18U)
#define TIM1_CCMR2      REG32(TIM1_BASE + 0x1CU)
#define TIM1_CCER       REG32(TIM1_BASE + 0x20U)

#define TIM1_CNT        REG32(TIM1_BASE + 0x24U)
#define TIM1_PSC        REG32(TIM1_BASE + 0x28U)
#define TIM1_ARR        REG32(TIM1_BASE + 0x2CU)
#define TIM1_RCR        REG32(TIM1_BASE + 0x30U)

#define TIM1_CCR1       REG32(TIM1_BASE + 0x34U)
#define TIM1_CCR2       REG32(TIM1_BASE + 0x38U)
#define TIM1_CCR3       REG32(TIM1_BASE + 0x3CU)
#define TIM1_CCR4       REG32(TIM1_BASE + 0x40U)

#define TIM1_BDTR       REG32(TIM1_BASE + 0x44U)


/* ============================================================
   M540
   ============================================================ */

#define M540_ADDR       0x69U

#define M540_WHO_AM_I   0x75U
#define M540_DATA_START 0x3BU


/* ============================================================
   Timing
   ============================================================ */

#define LOOP_PERIOD_MS  5U

/* 8 MHz / 8000 = 1 ms SysTick */

#define I2C_TIMEOUT_MS  10U


/* ============================================================
   PWM
   ============================================================ */

#define PWM_ARR_VALUE   7999U


/* ============================================================
   Sensor scale
   ============================================================ */

#define ACCEL_LSB_PER_G   2048.0f
#define GYRO_LSB_PER_DPS  16.4f


/* ============================================================
   Complementary filter
   ============================================================ */

#define FILTER_ALPHA        0.99f
#define FILTER_ACCEL_ALPHA  0.01f


/* ============================================================
   GYRO TEST

   After calibration:
       3 seconds preparation time
       then recorder becomes ARMED.

   Motion threshold:
       20 deg/s

   Recorder:
       detects dominant axis
       records FIRST direction
       then records same-direction peak for 600 ms

   gyro_test_state:
       0 = preparation
       1 = ARMED
       2 = RECORDING
       3 = DONE

   gyro_test_axis:
       0 = none
       1 = ROLL
       2 = PITCH
       3 = YAW

   gyro_test_first_sign:
       +1 = positive
       -1 = negative
   ============================================================ */

#define GYRO_TEST_ARM_DELAY_MS  3000U
#define GYRO_TEST_RECORD_MS      600U
#define GYRO_TEST_THRESHOLD_DPS  20.0f


/* ============================================================
   Global time
   ============================================================ */

volatile uint32_t system_ms = 0;


/* ============================================================
   IMU status
   ============================================================ */

volatile uint32_t imu_ok = 0;
volatile uint8_t imu_whoami = 0;

volatile uint32_t imu_read_ok = 0;

volatile uint32_t imu_frame_counter = 0;
volatile uint32_t imu_error_counter = 0;
volatile uint32_t imu_consecutive_errors = 0;


/* ============================================================
   Hardware I2C diagnostics
   ============================================================ */

volatile uint32_t i2c_hw_enabled = 0;

volatile uint32_t i2c_last_error = 0;

/*
    0 = no error
    1 = NACK
    2 = arbitration lost
    3 = overrun
    4 = timeout
    5 = BUSY timeout
*/

volatile uint32_t i2c_berr_counter = 0;
volatile uint32_t i2c_transfer_counter = 0;

volatile uint32_t i2c_isr_debug = 0;
volatile uint32_t i2c_timingr_debug = 0;


/* ============================================================
   RAW accelerometer
   ============================================================ */

volatile int16_t accel_x = 0;
volatile int16_t accel_y = 0;
volatile int16_t accel_z = 0;


/* ============================================================
   RAW gyro
   ============================================================ */

volatile int16_t gyro_x = 0;
volatile int16_t gyro_y = 0;
volatile int16_t gyro_z = 0;


/* ============================================================
   Temperature
   ============================================================ */

volatile int16_t imu_temperature_raw = 0;


/* ============================================================
   Calibration
   ============================================================ */

volatile uint32_t calibration_status = 0;

/*
    0 = not started
    1 = calibrating
    2 = complete
    3 = failed
*/

volatile uint32_t calibration_samples = 0;

volatile float gyro_x_offset = 0.0f;
volatile float gyro_y_offset = 0.0f;
volatile float gyro_z_offset = 0.0f;


/* ============================================================
   Corrected gyro
   ============================================================ */

volatile float gyro_roll_raw = 0.0f;
volatile float gyro_pitch_raw = 0.0f;
volatile float gyro_yaw_raw = 0.0f;

volatile float gyro_roll_dps = 0.0f;
volatile float gyro_pitch_dps = 0.0f;
volatile float gyro_yaw_dps = 0.0f;


/* ============================================================
   Accelerometer
   ============================================================ */

volatile float accel_x_g = 0.0f;
volatile float accel_y_g = 0.0f;
volatile float accel_z_g = 0.0f;

volatile float accel_roll_deg = 0.0f;
volatile float accel_pitch_deg = 0.0f;


/* ============================================================
   Attitude
   ============================================================ */

volatile float roll_angle = 0.0f;
volatile float pitch_angle = 0.0f;
volatile float yaw_angle = 0.0f;

volatile uint32_t attitude_initialized = 0;


/* ============================================================
   Loop diagnostics
   ============================================================ */

volatile uint32_t flight_loop_counter = 0;

volatile uint32_t loop_dt_ms = 0;
volatile float loop_dt = 0.0f;
volatile float loop_hz = 0.0f;

volatile uint32_t loop_dt_max_ms = 0;

volatile uint32_t imu_read_time_ms = 0;
volatile uint32_t imu_read_time_max_ms = 0;


/* ============================================================
   GYRO DIRECTION RECORDER
   ============================================================ */

volatile uint32_t gyro_test_state = 0;

volatile uint32_t gyro_test_axis = 0;

volatile int32_t gyro_test_first_sign = 0;

volatile float gyro_test_first_dps = 0.0f;
volatile float gyro_test_peak_dps = 0.0f;

volatile uint32_t gyro_test_start_ms = 0;
volatile uint32_t gyro_test_elapsed_ms = 0;

volatile uint32_t gyro_test_arm_delay_remaining_ms = 0;


/* ============================================================
   Motors

   ABSOLUTE SAFETY LOCK:
   MUST REMAIN ZERO.
   ============================================================ */

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
    SYST_CSR = 0U;

    SYST_RVR = 7999U;

    SYST_CVR = 0U;

    SYST_CSR =
        (1U << 2) |     /* CPU clock */
        (1U << 1) |     /* interrupt */
        (1U << 0);      /* enable */
}


/* ============================================================
   Delay
   ============================================================ */

static void delay_ms(uint32_t ms)
{
    uint32_t start = system_ms;

    while ((uint32_t)(system_ms - start) < ms)
    {
    }
}


/* ============================================================
   GPIO helpers
   ============================================================ */

static void gpio_output(uint32_t port, uint32_t pin)
{
    GPIO_MODER(port) &=
        ~(3U << (pin * 2U));

    GPIO_MODER(port) |=
        (1U << (pin * 2U));
}


static void gpio_high(uint32_t port, uint32_t pin)
{
    GPIO_BSRR(port) =
        (1U << pin);
}


static void gpio_low(uint32_t port, uint32_t pin)
{
    GPIO_BSRR(port) =
        (1U << (pin + 16U));
}


/* ============================================================
   Small math helpers
   ============================================================ */

static float absf_local(float x)
{
    if (x < 0.0f)
    {
        return -x;
    }

    return x;
}


/*
   Fast atan2 approximation.

   Good enough for the complementary attitude estimator
   without pulling in the full math library.
*/

static float fast_atan2_deg(float y, float x)
{
    const float PI_4 =
        0.78539816339f;

    const float THREE_PI_4 =
        2.35619449019f;

    const float RAD_TO_DEG =
        57.2957795131f;

    float abs_y;
    float r;
    float angle;

    abs_y = absf_local(y) + 0.0000001f;

    if (x < 0.0f)
    {
        r =
            (x + abs_y) /
            (abs_y - x);

        angle = THREE_PI_4;
    }
    else
    {
        r =
            (x - abs_y) /
            (x + abs_y);

        angle = PI_4;
    }

    angle +=
        (0.1963f * r * r - 0.9817f) * r;

    if (y < 0.0f)
    {
        angle = -angle;
    }

    return angle * RAD_TO_DEG;
}


/* Integer square root */

static uint32_t integer_sqrt_u32(uint32_t value)
{
    uint32_t result = 0U;

    uint32_t bit =
        1UL << 30;

    while (bit > value)
    {
        bit >>= 2;
    }

    while (bit != 0U)
    {
        if (value >= result + bit)
        {
            value -= result + bit;

            result =
                (result >> 1) + bit;
        }
        else
        {
            result >>= 1;
        }

        bit >>= 2;
    }

    return result;
}


/* ============================================================
   Hardware I2C GPIO
   ============================================================ */

static void i2c_gpio_init(void)
{
    /*
       PB7 = SDA
       PB8 = SCL

       Alternate function mode
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


    /* Open drain */

    GPIO_OTYPER(GPIOB_BASE) |=
        (1U << 7U) |
        (1U << 8U);


    /* High speed */

    GPIO_OSPEEDR(GPIOB_BASE) &=
        ~(
            (3U << (7U * 2U)) |
            (3U << (8U * 2U))
         );

    GPIO_OSPEEDR(GPIOB_BASE) |=
        (
            (3U << (7U * 2U)) |
            (3U << (8U * 2U))
        );


    /* Internal pull-ups */

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
       PB7 AF1
    */

    GPIO_AFRL(GPIOB_BASE) &=
        ~(0xFU << 28U);

    GPIO_AFRL(GPIOB_BASE) |=
        (1U << 28U);


    /*
       PB8 AF1
    */

    GPIO_AFRH(GPIOB_BASE) &=
        ~(0xFU << 0U);

    GPIO_AFRH(GPIOB_BASE) |=
        (1U << 0U);
}


/* ============================================================
   I2C error helpers
   ============================================================ */

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
    uint32_t isr =
        I2C1_ISR;

    i2c_isr_debug =
        isr;


    /*
       STM32F031 erratum:
       BERR may be detected spuriously in master mode.

       Count it, clear it, but do not abort transfer.
    */

    if ((isr & I2C_ISR_BERR) != 0U)
    {
        I2C1_ICR =
            I2C_ICR_BERRCF;

        i2c_berr_counter++;
    }


    if ((isr & I2C_ISR_NACKF) != 0U)
    {
        I2C1_ICR =
            I2C_ICR_NACKCF;

        i2c_last_error = 1U;

        return 0U;
    }


    if ((isr & I2C_ISR_ARLO) != 0U)
    {
        I2C1_ICR =
            I2C_ICR_ARLOCF;

        i2c_last_error = 2U;

        return 0U;
    }


    if ((isr & I2C_ISR_OVR) != 0U)
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
    uint32_t start =
        system_ms;

    while ((I2C1_ISR & flag) == 0U)
    {
        if (i2c_check_errors() == 0U)
        {
            return 0U;
        }

        if ((uint32_t)(system_ms - start) >=
            I2C_TIMEOUT_MS)
        {
            i2c_last_error = 4U;

            return 0U;
        }
    }

    i2c_isr_debug =
        I2C1_ISR;

    return 1U;
}


static uint32_t i2c_wait_not_busy(void)
{
    uint32_t start =
        system_ms;

    while ((I2C1_ISR & I2C_ISR_BUSY) != 0U)
    {
        if ((uint32_t)(system_ms - start) >=
            I2C_TIMEOUT_MS)
        {
            i2c_last_error = 5U;

            return 0U;
        }
    }

    return 1U;
}


/* ============================================================
   I2C init
   ============================================================ */

static void i2c_init(void)
{
    i2c_hw_enabled = 0U;

    i2c_last_error = 0U;

    i2c_berr_counter = 0U;

    i2c_transfer_counter = 0U;


    /*
       Select HSI as I2C1 clock.
    */

    RCC_CFGR3 &=
        ~(1U << 4);


    /*
       Enable I2C1 clock.
    */

    RCC_APB1ENR |=
        I2C1_EN;

    (void)RCC_APB1ENR;


    /*
       Reset I2C1.
    */

    RCC_APB1RSTR |=
        I2C1_EN;

    RCC_APB1RSTR &=
        ~I2C1_EN;


    /*
       Peripheral disabled while configuring.
    */

    I2C1_CR1 = 0U;


    /*
       8 MHz I2C kernel clock
       Standard Mode ~100 kHz

       PRESC  = 1
       SCLDEL = 4
       SDADEL = 2
       SCLH   = 0x0F
       SCLL   = 0x13
    */

    I2C1_TIMINGR =
        0x10420F13U;


    i2c_timingr_debug =
        I2C1_TIMINGR;


    i2c_clear_flags();


    /*
       Enable I2C1.
    */

    I2C1_CR1 =
        I2C_CR1_PE;


    i2c_hw_enabled = 1U;
}


/* ============================================================
   I2C register write
   ============================================================ */

static uint32_t m540_write_reg(
    uint8_t reg,
    uint8_t value)
{
    if (i2c_wait_not_busy() == 0U)
    {
        return 0U;
    }


    i2c_last_error = 0U;

    i2c_clear_flags();


    /*
       2 bytes:
       register + value

       AUTOEND generates STOP automatically.
    */

    I2C1_CR2 =
        ((uint32_t)M540_ADDR << 1U) |
        (2U << 16U) |
        I2C_CR2_AUTOEND;


    I2C1_CR2 |=
        I2C_CR2_START;


    if (i2c_wait_flag(I2C_ISR_TXIS) == 0U)
    {
        return 0U;
    }

    I2C1_TXDR =
        reg;


    if (i2c_wait_flag(I2C_ISR_TXIS) == 0U)
    {
        return 0U;
    }

    I2C1_TXDR =
        value;


    if (i2c_wait_flag(I2C_ISR_STOPF) == 0U)
    {
        return 0U;
    }


    I2C1_ICR =
        I2C_ICR_STOPCF;


    i2c_transfer_counter++;

    i2c_isr_debug =
        I2C1_ISR;


    return 1U;
}


/* ============================================================
   I2C register read
   ============================================================ */

static uint32_t m540_read_regs(
    uint8_t reg,
    uint8_t *buffer,
    uint32_t length)
{
    uint32_t i;


    if ((length == 0U) ||
        (length > 255U))
    {
        return 0U;
    }


    if (i2c_wait_not_busy() == 0U)
    {
        return 0U;
    }


    i2c_last_error = 0U;

    i2c_clear_flags();


    /*
       PHASE 1:
       send register address.

       NBYTES = 1
       AUTOEND = 0

       Therefore transfer stops at TC,
       without STOP condition.
    */

    I2C1_CR2 =
        ((uint32_t)M540_ADDR << 1U) |
        (1U << 16U);


    I2C1_CR2 |=
        I2C_CR2_START;


    if (i2c_wait_flag(I2C_ISR_TXIS) == 0U)
    {
        return 0U;
    }


    I2C1_TXDR =
        reg;


    if (i2c_wait_flag(I2C_ISR_TC) == 0U)
    {
        return 0U;
    }


    /*
       PHASE 2:
       repeated START
       read N bytes
       AUTOEND generates STOP.
    */

    I2C1_CR2 =
        ((uint32_t)M540_ADDR << 1U) |
        I2C_CR2_RD_WRN |
        (length << 16U) |
        I2C_CR2_AUTOEND;


    I2C1_CR2 |=
        I2C_CR2_START;


    for (i = 0U;
         i < length;
         i++)
    {
        if (i2c_wait_flag(I2C_ISR_RXNE) == 0U)
        {
            return 0U;
        }

        buffer[i] =
            (uint8_t)I2C1_RXDR;
    }


    if (i2c_wait_flag(I2C_ISR_STOPF) == 0U)
    {
        return 0U;
    }


    I2C1_ICR =
        I2C_ICR_STOPCF;


    i2c_transfer_counter++;

    i2c_isr_debug =
        I2C1_ISR;


    return 1U;
}


/* ============================================================
   M540 init
   ============================================================ */

static uint32_t m540_init(void)
{
    uint8_t value = 0U;


    /*
       WHO_AM_I
    */

    if (m540_read_regs(
            M540_WHO_AM_I,
            &value,
            1U) == 0U)
    {
        return 0U;
    }


    imu_whoami =
        value;


    if (imu_whoami != 0x7DU)
    {
        return 0U;
    }


    /*
       Device reset.
    */

    if (m540_write_reg(
            0x6BU,
            0x80U) == 0U)
    {
        return 0U;
    }


    delay_ms(100U);


    /*
       Wake + clock source.
    */

    if (m540_write_reg(
            0x6BU,
            0x01U) == 0U)
    {
        return 0U;
    }


    delay_ms(20U);


    /*
       Accelerometer ±16 g.
    */

    if (m540_write_reg(
            0x1CU,
            0x18U) == 0U)
    {
        return 0U;
    }


    /*
       Accelerometer filter.
    */

    if (m540_write_reg(
            0x1DU,
            0x05U) == 0U)
    {
        return 0U;
    }


    /*
       Gyroscope ±2000 dps.
    */

    if (m540_write_reg(
            0x1BU,
            0x18U) == 0U)
    {
        return 0U;
    }


    /*
       Gyro filter / raw bring-up setting.
    */

    if (m540_write_reg(
            0x1AU,
            0x00U) == 0U)
    {
        return 0U;
    }


    delay_ms(50U);


    /*
       Confirm WHO_AM_I again.
    */

    value = 0U;


    if (m540_read_regs(
            M540_WHO_AM_I,
            &value,
            1U) == 0U)
    {
        return 0U;
    }


    imu_whoami =
        value;


    if (imu_whoami != 0x7DU)
    {
        return 0U;
    }


    return 1U;
}


/* ============================================================
   M540 raw frame
   ============================================================ */

static uint32_t m540_read_raw(void)
{
    uint8_t data[14];


    if (m540_read_regs(
            M540_DATA_START,
            data,
            14U) == 0U)
    {
        imu_read_ok = 0U;

        imu_error_counter++;

        imu_consecutive_errors++;

        return 0U;
    }


    accel_x =
        (int16_t)(
            ((uint16_t)data[0] << 8U) |
            data[1]
        );


    accel_y =
        (int16_t)(
            ((uint16_t)data[2] << 8U) |
            data[3]
        );


    accel_z =
        (int16_t)(
            ((uint16_t)data[4] << 8U) |
            data[5]
        );


    imu_temperature_raw =
        (int16_t)(
            ((uint16_t)data[6] << 8U) |
            data[7]
        );


    gyro_x =
        (int16_t)(
            ((uint16_t)data[8] << 8U) |
            data[9]
        );


    gyro_y =
        (int16_t)(
            ((uint16_t)data[10] << 8U) |
            data[11]
        );


    gyro_z =
        (int16_t)(
            ((uint16_t)data[12] << 8U) |
            data[13]
        );


    imu_read_ok = 1U;

    imu_frame_counter++;

    imu_consecutive_errors = 0U;


    return 1U;
}


/* ============================================================
   Gyro calibration
   ============================================================ */

static uint32_t gyro_calibrate(void)
{
    int32_t sum_x = 0;
    int32_t sum_y = 0;
    int32_t sum_z = 0;

    uint32_t attempts = 0U;


    calibration_status = 1U;

    calibration_samples = 0U;


    while (calibration_samples < 500U)
    {
        attempts++;


        if (m540_read_raw() != 0U)
        {
            sum_x +=
                (int32_t)gyro_x;

            sum_y +=
                (int32_t)gyro_y;

            sum_z +=
                (int32_t)gyro_z;


            calibration_samples++;
        }


        /*
           Do not allow endless calibration
           if communication suddenly fails.
        */

        if (attempts > 1000U)
        {
            calibration_status = 3U;

            return 0U;
        }


        delay_ms(3U);
    }


    gyro_x_offset =
        (float)sum_x /
        (float)calibration_samples;


    gyro_y_offset =
        (float)sum_y /
        (float)calibration_samples;


    gyro_z_offset =
        (float)sum_z /
        (float)calibration_samples;


    calibration_status = 2U;


    return 1U;
}


/* ============================================================
   TIM1 GPIO
   ============================================================ */

static void motor_gpio_prepare_low(void)
{
    /*
       Force motor control pins LOW
       before switching them to TIM1.
    */

    gpio_low(GPIOA_BASE, 8U);
    gpio_low(GPIOA_BASE, 9U);
    gpio_low(GPIOA_BASE, 10U);
    gpio_low(GPIOA_BASE, 11U);


    gpio_output(GPIOA_BASE, 8U);
    gpio_output(GPIOA_BASE, 9U);
    gpio_output(GPIOA_BASE, 10U);
    gpio_output(GPIOA_BASE, 11U);
}


static void motor_gpio_tim1_af(void)
{
    /*
       PA8..PA11 = alternate function.
    */

    GPIO_MODER(GPIOA_BASE) &=
        ~(
            (3U << (8U * 2U)) |
            (3U << (9U * 2U)) |
            (3U << (10U * 2U)) |
            (3U << (11U * 2U))
         );


    GPIO_MODER(GPIOA_BASE) |=
        (
            (2U << (8U * 2U)) |
            (2U << (9U * 2U)) |
            (2U << (10U * 2U)) |
            (2U << (11U * 2U))
        );


    /*
       PA8..PA11 AF2 = TIM1.
    */

    GPIO_AFRH(GPIOA_BASE) &=
        ~(
            (0xFU << 0U) |
            (0xFU << 4U) |
            (0xFU << 8U) |
            (0xFU << 12U)
         );


    GPIO_AFRH(GPIOA_BASE) |=
        (
            (2U << 0U) |
            (2U << 4U) |
            (2U << 8U) |
            (2U << 12U)
        );
}


/* ============================================================
   TIM1 init

   PWM remains initialized because this is the future motor
   output peripheral, BUT CCR1..CCR4 are forced to ZERO.
   ============================================================ */

static void tim1_motor_init(void)
{
    RCC_APB2ENR |=
        TIM1_EN;

    (void)RCC_APB2ENR;


    RCC_APB2RSTR |=
        TIM1_EN;

    RCC_APB2RSTR &=
        ~TIM1_EN;


    motor_gpio_tim1_af();


    TIM1_CR1 = 0U;
    TIM1_CR2 = 0U;
    TIM1_SMCR = 0U;
    TIM1_DIER = 0U;

    TIM1_CCER = 0U;


    /*
       8 MHz timer clock.

       PSC = 0
       ARR = 7999

       PWM frequency = 1 kHz.
    */

    TIM1_PSC =
        0U;

    TIM1_ARR =
        PWM_ARR_VALUE;


    /*
       ABSOLUTE OFF before outputs enabled.
    */

    TIM1_CCR1 = 0U;
    TIM1_CCR2 = 0U;
    TIM1_CCR3 = 0U;
    TIM1_CCR4 = 0U;


    /*
       PWM mode 1 + preload.

       CH1 + CH2
    */

    TIM1_CCMR1 =
        (6U << 4U) |
        (1U << 3U) |
        (6U << 12U) |
        (1U << 11U);


    /*
       CH3 + CH4
    */

    TIM1_CCMR2 =
        (6U << 4U) |
        (1U << 3U) |
        (6U << 12U) |
        (1U << 11U);


    /*
       Enable CH1..CH4 outputs.
    */

    TIM1_CCER =
        (1U << 0U) |
        (1U << 4U) |
        (1U << 8U) |
        (1U << 12U);


    /*
       Main Output Enable.
    */

    TIM1_BDTR =
        (1U << 15U);


    /*
       Generate update event.
    */

    TIM1_EGR =
        1U;


    /*
       ARPE + counter enable.
    */

    TIM1_CR1 =
        (1U << 7U) |
        (1U << 0U);


    /*
       Double safety.
    */

    TIM1_CCR1 = 0U;
    TIM1_CCR2 = 0U;
    TIM1_CCR3 = 0U;
    TIM1_CCR4 = 0U;
}


/* ============================================================
   MOTOR SAFETY LOCK
   ============================================================ */

static void motors_force_off(void)
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


/* ============================================================
   Attitude update
   ============================================================ */

static void attitude_update(float dt)
{
    int32_t ay;
    int32_t az;

    uint32_t yz_squared;
    uint32_t yz_length;


    /*
       Gyro mapping already established:

       raw X = roll
       raw Y = pitch
       raw Z = yaw
    */

    gyro_roll_raw =
        (float)gyro_x -
        gyro_x_offset;


    gyro_pitch_raw =
        (float)gyro_y -
        gyro_y_offset;


    gyro_yaw_raw =
        (float)gyro_z -
        gyro_z_offset;


    gyro_roll_dps =
        gyro_roll_raw /
        GYRO_LSB_PER_DPS;


    gyro_pitch_dps =
        gyro_pitch_raw /
        GYRO_LSB_PER_DPS;


    gyro_yaw_dps =
        gyro_yaw_raw /
        GYRO_LSB_PER_DPS;


    /*
       Acceleration in g.
    */

    accel_x_g =
        (float)accel_x /
        ACCEL_LSB_PER_G;


    accel_y_g =
        (float)accel_y /
        ACCEL_LSB_PER_G;


    accel_z_g =
        (float)accel_z /
        ACCEL_LSB_PER_G;


    /*
       Roll from accelerometer.

       Confirmed experimentally:

       LEFT DOWN  -> positive roll
       RIGHT DOWN -> negative roll
    */

    accel_roll_deg =
        fast_atan2_deg(
            (float)accel_y,
            (float)accel_z
        );


    /*
       Pitch denominator:
       sqrt(Y^2 + Z^2)
    */

    ay = (int32_t)accel_y;
    az = (int32_t)accel_z;


    yz_squared =
        (uint32_t)(ay * ay) +
        (uint32_t)(az * az);


    yz_length =
        integer_sqrt_u32(
            yz_squared
        );


    /*
       Confirmed experimentally:

       NOSE UP   -> positive pitch
       NOSE DOWN -> negative pitch
    */

    accel_pitch_deg =
        fast_atan2_deg(
            -(float)accel_x,
            (float)yz_length
        );


    /*
       Initial attitude.
    */

    if (attitude_initialized == 0U)
    {
        roll_angle =
            accel_roll_deg;

        pitch_angle =
            accel_pitch_deg;

        yaw_angle =
            0.0f;


        attitude_initialized = 1U;

        return;
    }


    /*
       Complementary filter.
    */

    roll_angle =
        FILTER_ALPHA *
        (
            roll_angle +
            gyro_roll_dps * dt
        )
        +
        FILTER_ACCEL_ALPHA *
        accel_roll_deg;


    pitch_angle =
        FILTER_ALPHA *
        (
            pitch_angle +
            gyro_pitch_dps * dt
        )
        +
        FILTER_ACCEL_ALPHA *
        accel_pitch_deg;


    /*
       No magnetometer.

       Yaw is gyro integration only.
    */

    yaw_angle +=
        gyro_yaw_dps * dt;


    /*
       Keep yaw readable.
    */

    if (yaw_angle > 180.0f)
    {
        yaw_angle -=
            360.0f;
    }


    if (yaw_angle < -180.0f)
    {
        yaw_angle +=
            360.0f;
    }
}


/* ============================================================
   Gyro direction recorder helpers
   ============================================================ */

static float gyro_test_axis_value(uint32_t axis)
{
    if (axis == 1U)
    {
        return gyro_roll_dps;
    }


    if (axis == 2U)
    {
        return gyro_pitch_dps;
    }


    if (axis == 3U)
    {
        return gyro_yaw_dps;
    }


    return 0.0f;
}


/* ============================================================
   Gyro direction recorder
   ============================================================ */

static void gyro_test_update(void)
{
    float roll_abs;
    float pitch_abs;
    float yaw_abs;

    float selected_value;
    float current_value;

    uint32_t elapsed;


    /*
       STATE 0:
       preparation delay after calibration.

       This gives you time to pick the drone up
       and hold it horizontally.
    */

    if (gyro_test_state == 0U)
    {
        elapsed =
            (uint32_t)(
                system_ms -
                gyro_test_start_ms
            );


        if (elapsed >=
            GYRO_TEST_ARM_DELAY_MS)
        {
            gyro_test_arm_delay_remaining_ms =
                0U;

            gyro_test_state =
                1U;
        }
        else
        {
            gyro_test_arm_delay_remaining_ms =
                GYRO_TEST_ARM_DELAY_MS -
                elapsed;
        }


        return;
    }


    /*
       STATE 1:
       armed.

       Wait for one clear movement.
    */

    if (gyro_test_state == 1U)
    {
        roll_abs =
            absf_local(
                gyro_roll_dps
            );


        pitch_abs =
            absf_local(
                gyro_pitch_dps
            );


        yaw_abs =
            absf_local(
                gyro_yaw_dps
            );


        /*
           Find dominant axis.
        */

        gyro_test_axis =
            1U;

        selected_value =
            gyro_roll_dps;


        if (pitch_abs > roll_abs)
        {
            gyro_test_axis =
                2U;

            selected_value =
                gyro_pitch_dps;

            roll_abs =
                pitch_abs;
        }


        if (yaw_abs > roll_abs)
        {
            gyro_test_axis =
                3U;

            selected_value =
                gyro_yaw_dps;

            roll_abs =
                yaw_abs;
        }


        /*
           Not moving fast enough yet.
        */

        if (roll_abs <
            GYRO_TEST_THRESHOLD_DPS)
        {
            gyro_test_axis =
                0U;

            return;
        }


        /*
           FIRST direction captured here.

           This sign will NEVER change,
           even if the drone is moved back.
        */

        if (selected_value >= 0.0f)
        {
            gyro_test_first_sign =
                1;
        }
        else
        {
            gyro_test_first_sign =
                -1;
        }


        gyro_test_first_dps =
            selected_value;


        gyro_test_peak_dps =
            selected_value;


        gyro_test_start_ms =
            system_ms;


        gyro_test_elapsed_ms =
            0U;


        gyro_test_state =
            2U;


        return;
    }


    /*
       STATE 2:
       record peak for 600 ms.

       Only update the peak in the SAME
       direction as the first detected motion.
    */

    if (gyro_test_state == 2U)
    {
        current_value =
            gyro_test_axis_value(
                gyro_test_axis
            );


        if (gyro_test_first_sign > 0)
        {
            if (current_value >
                gyro_test_peak_dps)
            {
                gyro_test_peak_dps =
                    current_value;
            }
        }
        else
        {
            if (current_value <
                gyro_test_peak_dps)
            {
                gyro_test_peak_dps =
                    current_value;
            }
        }


        gyro_test_elapsed_ms =
            (uint32_t)(
                system_ms -
                gyro_test_start_ms
            );


        if (gyro_test_elapsed_ms >=
            GYRO_TEST_RECORD_MS)
        {
            gyro_test_state =
                3U;
        }


        return;
    }


    /*
       STATE 3:
       DONE.

       Values remain frozen until RESET.
    */
}


/* ============================================================
   MAIN
   ============================================================ */

int main(void)
{
    uint32_t last_loop_ms;
    uint32_t now_ms;

    uint32_t read_start_ms;
    uint32_t read_end_ms;


    /* ========================================================
       GPIO clocks
       ======================================================== */

    RCC_AHBENR |=
        GPIOA_EN |
        GPIOB_EN;

    (void)RCC_AHBENR;


    /* ========================================================
       Board enable PA1
       ======================================================== */

    gpio_high(
        GPIOA_BASE,
        1U
    );

    gpio_output(
        GPIOA_BASE,
        1U
    );


    /* ========================================================
       Force motor pins LOW BEFORE TIM1
       ======================================================== */

    motor_gpio_prepare_low();


    /* ========================================================
       SysTick
       ======================================================== */

    systick_init();


    delay_ms(10U);


    /* ========================================================
       Hardware I2C GPIO
       ======================================================== */

    i2c_gpio_init();


    /* ========================================================
       Hardware I2C1
       ======================================================== */

    i2c_init();


    delay_ms(10U);


    /* ========================================================
       TIM1

       Motors remain ZERO.
       ======================================================== */

    tim1_motor_init();

    motors_force_off();


    /* ========================================================
       M540
       ======================================================== */

    if (m540_init() == 0U)
    {
        imu_ok = 0U;


        while (1)
        {
            motors_force_off();

            i2c_isr_debug =
                I2C1_ISR;
        }
    }


    imu_ok = 1U;


    /* ========================================================
       Gyro calibration

       KEEP DRONE COMPLETELY STILL.
       ======================================================== */

    if (gyro_calibrate() == 0U)
    {
        imu_ok = 0U;

        calibration_status =
            3U;


        while (1)
        {
            motors_force_off();
        }
    }


    /* ========================================================
       First attitude frame
       ======================================================== */

    if (m540_read_raw() != 0U)
    {
        attitude_update(
            0.005f
        );
    }


    /* ========================================================
       Start gyro test preparation countdown.

       3 seconds to pick up drone and hold it flat.
       ======================================================== */

    gyro_test_state =
        0U;

    gyro_test_axis =
        0U;

    gyro_test_first_sign =
        0;

    gyro_test_first_dps =
        0.0f;

    gyro_test_peak_dps =
        0.0f;

    gyro_test_start_ms =
        system_ms;

    gyro_test_arm_delay_remaining_ms =
        GYRO_TEST_ARM_DELAY_MS;


    /* ========================================================
       Flight loop scheduler
       ======================================================== */

    last_loop_ms =
        system_ms;


    while (1)
    {
        /*
           Absolute motor safety.
        */

        motors_force_off();


        now_ms =
            system_ms;


        if ((uint32_t)(
                now_ms -
                last_loop_ms
            ) < LOOP_PERIOD_MS)
        {
            continue;
        }


        /* ====================================================
           Actual dt
           ==================================================== */

        loop_dt_ms =
            (uint32_t)(
                now_ms -
                last_loop_ms
            );


        last_loop_ms =
            now_ms;


        loop_dt =
            (float)loop_dt_ms /
            1000.0f;


        if (loop_dt > 0.0f)
        {
            loop_hz =
                1.0f /
                loop_dt;
        }


        if (loop_dt_ms >
            loop_dt_max_ms)
        {
            loop_dt_max_ms =
                loop_dt_ms;
        }


        flight_loop_counter++;


        /* ====================================================
           Read IMU
           ==================================================== */

        read_start_ms =
            system_ms;


        if (m540_read_raw() != 0U)
        {
            read_end_ms =
                system_ms;


            imu_read_time_ms =
                (uint32_t)(
                    read_end_ms -
                    read_start_ms
                );


            if (imu_read_time_ms >
                imu_read_time_max_ms)
            {
                imu_read_time_max_ms =
                    imu_read_time_ms;
            }


            /* ================================================
               Attitude
               ================================================ */

            attitude_update(
                loop_dt
            );


            /* ================================================
               Gyro direction recorder
               ================================================ */

            gyro_test_update();
        }
        else
        {
            read_end_ms =
                system_ms;


            imu_read_time_ms =
                (uint32_t)(
                    read_end_ms -
                    read_start_ms
                );
        }


        /* ====================================================
           I2C diagnostics
           ==================================================== */

        i2c_isr_debug =
            I2C1_ISR;


        /* ====================================================
           ABSOLUTE MOTOR SAFETY

           DO NOT REMOVE YET.
           ==================================================== */

        motors_force_off();
    }
}
