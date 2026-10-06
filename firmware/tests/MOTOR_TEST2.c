#include <stdint.h>

#define REG32(addr) (*(volatile uint32_t *)(uintptr_t)(addr))

/* ============================================================
   DRONE_ - FLIGHT CONTROLLER
   STEP 4: RATE PID + X MIXER - DRY RUN

   STM32F031K4
   M540 IMU
   Hardware I2C1 @ 100 kHz
   Control loop = 200 Hz

   MOTOR GEOMETRY:

                    FRONT
                      ^

              M2 CCW       M1 CW
             front-left   front-right

              M3 CW        M4 CCW
              rear-left    rear-right


   ELECTRICAL MAPPING:

       M1 = PA11 = TIM1_CH4
       M2 = PA9  = TIM1_CH2
       M3 = PA8  = TIM1_CH1
       M4 = PA10 = TIM1_CH3


   CONFIRMED FC SIGNS:

       Roll+  = LEFT side DOWN
       Roll-  = RIGHT side DOWN

       Pitch+ = NOSE UP
       Pitch- = NOSE DOWN

       Yaw+   = RIGHT
       Yaw-   = LEFT


   IMPORTANT:

       THIS IS DRY-RUN FIRMWARE.

       PID and mixer are calculated,
       BUT PHYSICAL MOTOR PWM IS FORCED TO ZERO.

       TIM1_CCR1 = 0
       TIM1_CCR2 = 0
       TIM1_CCR3 = 0
       TIM1_CCR4 = 0

   ============================================================ */


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

#define TIM1_EN         (1U << 11)
#define I2C1_EN         (1U << 21)


/* ============================================================
   GPIO
   ============================================================ */

#define GPIOA_BASE      0x48000000U
#define GPIOB_BASE      0x48000400U

#define GPIO_MODER(p)   REG32((p) + 0x00U)
#define GPIO_OTYPER(p)  REG32((p) + 0x04U)
#define GPIO_OSPEEDR(p) REG32((p) + 0x08U)
#define GPIO_PUPDR(p)   REG32((p) + 0x0CU)
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
   TIM1
   ============================================================ */

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
   CONSTANTS
   ============================================================ */

#define M540_ADDR               0x69U

#define LOOP_PERIOD_MS          5U
#define I2C_TIMEOUT_MS          10U

#define ACCEL_LSB_PER_G         2048.0f
#define GYRO_LSB_PER_DPS        16.4f

#define FILTER_ALPHA            0.99f
#define FILTER_ACCEL_ALPHA      0.01f

#define PWM_PERIOD_TICKS        8000U


/* ============================================================
   DRY-RUN PID SETTINGS

   Virtual throttle is only a mathematical center point.
   It DOES NOT reach the motors.
   ============================================================ */

#define VIRTUAL_THROTTLE        50.0f

#define PID_OUTPUT_LIMIT        25.0f
#define INTEGRAL_LIMIT          100.0f


/*
   First test intentionally uses P only.

   I = 0
   D = 0

   First we verify SIGNS.

   After mixer directions are confirmed,
   we will enable/tune I and D.
*/

#define ROLL_KP                 0.10f
#define ROLL_KI                 0.00f
#define ROLL_KD                 0.00f

#define PITCH_KP                0.10f
#define PITCH_KI                0.00f
#define PITCH_KD                0.00f

#define YAW_KP                  0.08f
#define YAW_KI                  0.00f
#define YAW_KD                  0.00f


/* ============================================================
   SYSTEM DIAGNOSTICS
   ============================================================ */

volatile uint32_t system_ms = 0;

volatile uint32_t imu_ok = 0;
volatile uint8_t imu_whoami = 0;

volatile uint32_t calibration_status = 0;
volatile uint32_t calibration_samples = 0;

volatile uint32_t imu_frame_counter = 0;
volatile uint32_t imu_error_counter = 0;
volatile uint32_t imu_consecutive_errors = 0;


/* ============================================================
   I2C DIAGNOSTICS
   ============================================================ */

volatile uint32_t i2c_hw_enabled = 0;

volatile uint32_t i2c_last_error = 0;
volatile uint32_t i2c_berr_counter = 0;
volatile uint32_t i2c_transfer_counter = 0;

volatile uint32_t i2c_isr_debug = 0;
volatile uint32_t i2c_timingr_debug = 0;


/* ============================================================
   LOOP DIAGNOSTICS
   ============================================================ */

volatile uint32_t loop_dt_ms = 0;
volatile uint32_t loop_dt_max_ms = 0;

volatile float loop_dt = 0.0f;
volatile float loop_hz = 0.0f;

volatile uint32_t imu_read_time_ms = 0;
volatile uint32_t imu_read_time_max_ms = 0;


/* ============================================================
   RAW IMU
   ============================================================ */

volatile int16_t accel_x = 0;
volatile int16_t accel_y = 0;
volatile int16_t accel_z = 0;

volatile int16_t gyro_x = 0;
volatile int16_t gyro_y = 0;
volatile int16_t gyro_z = 0;


/* ============================================================
   GYRO CALIBRATION
   ============================================================ */

volatile float gyro_x_offset = 0.0f;
volatile float gyro_y_offset = 0.0f;
volatile float gyro_z_offset = 0.0f;


/* ============================================================
   IMU PHYSICAL VALUES
   ============================================================ */

volatile float gyro_roll_dps = 0.0f;
volatile float gyro_pitch_dps = 0.0f;
volatile float gyro_yaw_dps = 0.0f;

volatile float accel_roll_deg = 0.0f;
volatile float accel_pitch_deg = 0.0f;


/* ============================================================
   ATTITUDE
   ============================================================ */

volatile float roll_angle = 0.0f;
volatile float pitch_angle = 0.0f;
volatile float yaw_angle = 0.0f;

volatile uint32_t attitude_initialized = 0;


/* ============================================================
   RATE SETPOINTS

   No receiver yet.

   Therefore desired angular velocity = 0 deg/s.
   ============================================================ */

volatile float rate_roll_setpoint = 0.0f;
volatile float rate_pitch_setpoint = 0.0f;
volatile float rate_yaw_setpoint = 0.0f;


/* ============================================================
   RATE ERRORS
   ============================================================ */

volatile float rate_roll_error = 0.0f;
volatile float rate_pitch_error = 0.0f;
volatile float rate_yaw_error = 0.0f;


/* ============================================================
   PID DEBUG
   ============================================================ */

volatile float pid_roll_p = 0.0f;
volatile float pid_roll_i = 0.0f;
volatile float pid_roll_d = 0.0f;
volatile float pid_roll_output = 0.0f;

volatile float pid_pitch_p = 0.0f;
volatile float pid_pitch_i = 0.0f;
volatile float pid_pitch_d = 0.0f;
volatile float pid_pitch_output = 0.0f;

volatile float pid_yaw_p = 0.0f;
volatile float pid_yaw_i = 0.0f;
volatile float pid_yaw_d = 0.0f;
volatile float pid_yaw_output = 0.0f;


/* ============================================================
   VIRTUAL MIXER

   These are calculated motor commands.

   THEY DO NOT GO TO TIM1.
   ============================================================ */

volatile float mix_m1_percent = 0.0f;
volatile float mix_m2_percent = 0.0f;
volatile float mix_m3_percent = 0.0f;
volatile float mix_m4_percent = 0.0f;


/* ============================================================
   PHYSICAL MOTOR OUTPUT

   MUST ALWAYS REMAIN ZERO.
   ============================================================ */

volatile uint32_t motor1_percent = 0;
volatile uint32_t motor2_percent = 0;
volatile uint32_t motor3_percent = 0;
volatile uint32_t motor4_percent = 0;

volatile uint32_t dry_run_enabled = 1U;
volatile uint32_t physical_motor_lock = 1U;


/* ============================================================
   INTERNAL PID STATE
   ============================================================ */

static float roll_integral = 0.0f;
static float pitch_integral = 0.0f;
static float yaw_integral = 0.0f;

static float roll_prev_error = 0.0f;
static float pitch_prev_error = 0.0f;
static float yaw_prev_error = 0.0f;


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
       = 1 ms
    */

    SYST_RVR = 7999U;
    SYST_CVR = 0U;

    SYST_CSR = 7U;
}


static void delay_ms(uint32_t ms)
{
    uint32_t start = system_ms;

    while ((uint32_t)(system_ms - start) < ms)
    {
    }
}


/* ============================================================
   BASIC GPIO
   ============================================================ */

static void gpio_output(
    uint32_t port,
    uint32_t pin)
{
    GPIO_MODER(port) &=
        ~(3U << (pin * 2U));

    GPIO_MODER(port) |=
        (1U << (pin * 2U));

    GPIO_OTYPER(port) &=
        ~(1U << pin);
}


static void gpio_high(
    uint32_t port,
    uint32_t pin)
{
    GPIO_BSRR(port) =
        (1U << pin);
}


static void gpio_low(
    uint32_t port,
    uint32_t pin)
{
    GPIO_BSRR(port) =
        (1U << (pin + 16U));
}


/* ============================================================
   MATH
   ============================================================ */

static float clampf(
    float x,
    float lo,
    float hi)
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


static uint32_t integer_sqrt(
    uint32_t n)
{
    uint32_t x;
    uint32_t y;

    if (n == 0U)
    {
        return 0U;
    }

    x = n;

    y =
        (x + 1U) >>
        1U;

    while (y < x)
    {
        x = y;

        y =
            (x + n / x) >>
            1U;
    }

    return x;
}


static float fast_atan2_deg(
    float y,
    float x)
{
    const float c1 = 45.0f;
    const float c2 = 135.0f;

    float abs_y;
    float angle;
    float r;

    abs_y =
        ((y < 0.0f) ? -y : y) +
        0.000001f;

    if (x >= 0.0f)
    {
        r =
            (x - abs_y) /
            (x + abs_y);

        angle =
            c1 -
            c1 * r;
    }
    else
    {
        r =
            (x + abs_y) /
            (abs_y - x);

        angle =
            c2 -
            c1 * r;
    }

    if (y < 0.0f)
    {
        return -angle;
    }

    return angle;
}


/* ============================================================
   MOTOR GPIO PREPARE
   ============================================================ */

static void motor_gpio_prepare_low(void)
{
    uint32_t pin;

    for (pin = 8U;
         pin <= 11U;
         pin++)
    {
        gpio_low(
            GPIOA_BASE,
            pin);

        gpio_output(
            GPIOA_BASE,
            pin);
    }
}


/* ============================================================
   TIM1 PWM

   Hardware initialized normally,
   but CCR values will remain ZERO.
   ============================================================ */

static void tim1_pwm_init(void)
{
    uint32_t pin;

    RCC_APB2ENR |=
        TIM1_EN;

    RCC_APB2RSTR |=
        TIM1_EN;

    RCC_APB2RSTR &=
        ~TIM1_EN;


    /*
       PA8..PA11 = AF2
    */

    for (pin = 8U;
         pin <= 11U;
         pin++)
    {
        GPIO_MODER(GPIOA_BASE) &=
            ~(3U << (pin * 2U));

        GPIO_MODER(GPIOA_BASE) |=
            (2U << (pin * 2U));

        GPIO_OTYPER(GPIOA_BASE) &=
            ~(1U << pin);

        GPIO_OSPEEDR(GPIOA_BASE) |=
            (3U << (pin * 2U));

        GPIO_PUPDR(GPIOA_BASE) &=
            ~(3U << (pin * 2U));
    }


    /*
       PA8  AF2
       PA9  AF2
       PA10 AF2
       PA11 AF2
    */

    GPIO_AFRH(GPIOA_BASE) &=
        ~0x0000FFFFU;

    GPIO_AFRH(GPIOA_BASE) |=
        0x00002222U;


    /*
       8 MHz / 8000 = 1 kHz PWM
    */

    TIM1_PSC = 0U;

    TIM1_ARR =
        PWM_PERIOD_TICKS -
        1U;


    /*
       ABSOLUTE OFF
    */

    TIM1_CCR1 = 0U;
    TIM1_CCR2 = 0U;
    TIM1_CCR3 = 0U;
    TIM1_CCR4 = 0U;


    /*
       PWM Mode 1
    */

    TIM1_CCMR1 =
          (6U << 4U)
        | (1U << 3U)
        | (6U << 12U)
        | (1U << 11U);

    TIM1_CCMR2 =
          (6U << 4U)
        | (1U << 3U)
        | (6U << 12U)
        | (1U << 11U);


    /*
       Enable channels
    */

    TIM1_CCER =
          (1U << 0U)
        | (1U << 4U)
        | (1U << 8U)
        | (1U << 12U);


    /*
       Main Output Enable
    */

    TIM1_BDTR =
        (1U << 15U);


    TIM1_EGR = 1U;


    /*
       ARPE + CEN
    */

    TIM1_CR1 =
          (1U << 7U)
        | (1U << 0U);
}


/* ============================================================
   ABSOLUTE MOTOR OFF
   ============================================================ */

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
}


/* ============================================================
   I2C GPIO

   PB7 = SDA
   PB8 = SCL
   AF1
   ============================================================ */

static void i2c_gpio_init(void)
{
    /*
       Alternate Function mode
    */

    GPIO_MODER(GPIOB_BASE) &=
        ~(
            (3U << 14U) |
            (3U << 16U)
        );

    GPIO_MODER(GPIOB_BASE) |=
        (
            (2U << 14U) |
            (2U << 16U)
        );


    /*
       Open drain
    */

    GPIO_OTYPER(GPIOB_BASE) |=
        (1U << 7U) |
        (1U << 8U);


    /*
       High speed
    */

    GPIO_OSPEEDR(GPIOB_BASE) |=
        (3U << 14U) |
        (3U << 16U);


    /*
       Pull-up
    */

    GPIO_PUPDR(GPIOB_BASE) &=
        ~(
            (3U << 14U) |
            (3U << 16U)
        );

    GPIO_PUPDR(GPIOB_BASE) |=
        (
            (1U << 14U) |
            (1U << 16U)
        );


    /*
       PB7 = AF1
    */

    GPIO_AFRL(GPIOB_BASE) &=
        ~(0xFU << 28U);

    GPIO_AFRL(GPIOB_BASE) |=
        (1U << 28U);


    /*
       PB8 = AF1
    */

    GPIO_AFRH(GPIOB_BASE) &=
        ~(0xFU << 0U);

    GPIO_AFRH(GPIOB_BASE) |=
        (1U << 0U);
}


/* ============================================================
   I2C
   ============================================================ */

static void i2c_clear_flags(void)
{
    I2C1_ICR =
          I2C_ICR_NACKCF
        | I2C_ICR_STOPCF
        | I2C_ICR_BERRCF
        | I2C_ICR_ARLOCF
        | I2C_ICR_OVRCF;
}


static void i2c_init(void)
{
    i2c_gpio_init();


    /*
       I2C1 kernel clock = HSI
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
       8 MHz I2C kernel
       100 kHz Standard Mode
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


static uint32_t i2c_wait_flag(
    uint32_t flag)
{
    uint32_t start;

    start = system_ms;


    while ((I2C1_ISR & flag) == 0U)
    {
        uint32_t isr;

        isr = I2C1_ISR;

        i2c_isr_debug = isr;


        /*
           STM32F031 BERR erratum workaround:
           clear BERR and continue.
        */

        if (isr & I2C_ISR_BERR)
        {
            I2C1_ICR =
                I2C_ICR_BERRCF;

            i2c_berr_counter++;
        }


        if (isr & I2C_ISR_NACKF)
        {
            i2c_last_error = 1U;

            I2C1_ICR =
                I2C_ICR_NACKCF;

            return 0U;
        }


        if (isr & I2C_ISR_ARLO)
        {
            i2c_last_error = 2U;

            I2C1_ICR =
                I2C_ICR_ARLOCF;

            return 0U;
        }


        if (isr & I2C_ISR_OVR)
        {
            i2c_last_error = 3U;

            I2C1_ICR =
                I2C_ICR_OVRCF;

            return 0U;
        }


        if ((uint32_t)(
                system_ms -
                start) >=
            I2C_TIMEOUT_MS)
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

    start = system_ms;


    while (I2C1_ISR &
           I2C_ISR_BUSY)
    {
        if ((uint32_t)(
                system_ms -
                start) >=
            I2C_TIMEOUT_MS)
        {
            i2c_last_error = 5U;

            return 0U;
        }
    }


    return 1U;
}


/* ============================================================
   M540 WRITE
   ============================================================ */

static uint32_t m540_write_reg(
    uint8_t reg,
    uint8_t value)
{
    if (!i2c_wait_not_busy())
    {
        return 0U;
    }


    i2c_last_error = 0U;

    i2c_clear_flags();


    /*
       2-byte transaction:

       register
       value
    */

    I2C1_CR2 =
          ((uint32_t)(
              M540_ADDR << 1U))
        | (2U << 16U)
        | I2C_CR2_AUTOEND
        | I2C_CR2_START;


    if (!i2c_wait_flag(
            I2C_ISR_TXIS))
    {
        goto fail;
    }


    I2C1_TXDR = reg;


    if (!i2c_wait_flag(
            I2C_ISR_TXIS))
    {
        goto fail;
    }


    I2C1_TXDR = value;


    if (!i2c_wait_flag(
            I2C_ISR_STOPF))
    {
        goto fail;
    }


    I2C1_ICR =
        I2C_ICR_STOPCF;


    i2c_transfer_counter++;


    return 1U;


fail:

    I2C1_CR2 |=
        I2C_CR2_STOP;

    i2c_clear_flags();

    return 0U;
}


/* ============================================================
   M540 READ
   ============================================================ */

static uint32_t m540_read_regs(
    uint8_t reg,
    uint8_t *data,
    uint32_t count)
{
    uint32_t i;


    if ((count == 0U) ||
        (count > 255U))
    {
        return 0U;
    }


    if (!i2c_wait_not_busy())
    {
        return 0U;
    }


    i2c_last_error = 0U;

    i2c_clear_flags();


    /*
       Send register address.
       No AUTOEND.
    */

    I2C1_CR2 =
          ((uint32_t)(
              M540_ADDR << 1U))
        | (1U << 16U)
        | I2C_CR2_START;


    if (!i2c_wait_flag(
            I2C_ISR_TXIS))
    {
        goto fail;
    }


    I2C1_TXDR = reg;


    if (!i2c_wait_flag(
            I2C_ISR_TC))
    {
        goto fail;
    }


    /*
       Repeated START
       Read N bytes
    */

    I2C1_CR2 =
          ((uint32_t)(
              M540_ADDR << 1U))
        | (count << 16U)
        | I2C_CR2_RD_WRN
        | I2C_CR2_AUTOEND
        | I2C_CR2_START;


    for (i = 0U;
         i < count;
         i++)
    {
        if (!i2c_wait_flag(
                I2C_ISR_RXNE))
        {
            goto fail;
        }


        data[i] =
            (uint8_t)I2C1_RXDR;
    }


    if (!i2c_wait_flag(
            I2C_ISR_STOPF))
    {
        goto fail;
    }


    I2C1_ICR =
        I2C_ICR_STOPCF;


    i2c_transfer_counter++;


    return 1U;


fail:

    I2C1_CR2 |=
        I2C_CR2_STOP;

    i2c_clear_flags();

    return 0U;
}


static uint32_t m540_read_reg(
    uint8_t reg,
    uint8_t *value)
{
    return
        m540_read_regs(
            reg,
            value,
            1U);
}


/* ============================================================
   M540 INITIALIZATION
   ============================================================ */

static uint32_t m540_init(void)
{
    uint8_t id = 0U;


    /*
       Reset
    */

    if (!m540_write_reg(
            0x6BU,
            0x80U))
    {
        return 0U;
    }


    delay_ms(50U);


    /*
       Wake
    */

    if (!m540_write_reg(
            0x6BU,
            0x01U))
    {
        return 0U;
    }


    delay_ms(10U);


    /*
       WHO_AM_I
    */

    if (!m540_read_reg(
            0x75U,
            &id))
    {
        return 0U;
    }


    imu_whoami = id;


    if (id != 0x7DU)
    {
        return 0U;
    }


    /*
       Accel +/-16g
    */

    if (!m540_write_reg(
            0x1CU,
            0x18U))
    {
        return 0U;
    }


    /*
       Accel filter
    */

    if (!m540_write_reg(
            0x1DU,
            0x05U))
    {
        return 0U;
    }


    /*
       Gyro +/-2000 dps
    */

    if (!m540_write_reg(
            0x1BU,
            0x18U))
    {
        return 0U;
    }


    /*
       Gyro filter
    */

    if (!m540_write_reg(
            0x1AU,
            0x00U))
    {
        return 0U;
    }


    delay_ms(10U);


    return 1U;
}


/* ============================================================
   M540 RAW FRAME
   ============================================================ */

static uint32_t m540_read_raw(void)
{
    uint8_t data[14];


    if (!m540_read_regs(
            0x3BU,
            data,
            14U))
    {
        return 0U;
    }


    accel_x =
        (int16_t)(
            ((uint16_t)data[0] << 8U) |
            data[1]);


    accel_y =
        (int16_t)(
            ((uint16_t)data[2] << 8U) |
            data[3]);


    accel_z =
        (int16_t)(
            ((uint16_t)data[4] << 8U) |
            data[5]);


    gyro_x =
        (int16_t)(
            ((uint16_t)data[8] << 8U) |
            data[9]);


    gyro_y =
        (int16_t)(
            ((uint16_t)data[10] << 8U) |
            data[11]);


    gyro_z =
        (int16_t)(
            ((uint16_t)data[12] << 8U) |
            data[13]);


    return 1U;
}


/* ============================================================
   GYRO CALIBRATION
   ============================================================ */

static uint32_t gyro_calibrate(void)
{
    int32_t sum_x = 0;
    int32_t sum_y = 0;
    int32_t sum_z = 0;

    uint32_t valid = 0U;
    uint32_t i;


    calibration_status = 1U;

    calibration_samples = 0U;


    /*
       IMPORTANT:
       Keep drone completely still.
    */

    for (i = 0U;
         i < 500U;
         i++)
    {
        if (m540_read_raw())
        {
            sum_x +=
                (int32_t)gyro_x;

            sum_y +=
                (int32_t)gyro_y;

            sum_z +=
                (int32_t)gyro_z;


            valid++;


            calibration_samples =
                valid;
        }


        delay_ms(3U);
    }


    if (valid < 450U)
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
   ATTITUDE
   ============================================================ */

static void attitude_update(
    float dt)
{
    float gyro_roll;
    float gyro_pitch;
    float gyro_yaw;

    int32_t ay;
    int32_t az;

    uint32_t yz_squared;
    uint32_t yz_length_half;

    float yz_length;


    /*
       Correct gyro offset
       and convert to deg/s.
    */

    gyro_roll =
        ((float)gyro_x -
         gyro_x_offset) /
        GYRO_LSB_PER_DPS;


    gyro_pitch =
        ((float)gyro_y -
         gyro_y_offset) /
        GYRO_LSB_PER_DPS;


    gyro_yaw =
        ((float)gyro_z -
         gyro_z_offset) /
        GYRO_LSB_PER_DPS;


    gyro_roll_dps =
        gyro_roll;

    gyro_pitch_dps =
        gyro_pitch;

    gyro_yaw_dps =
        gyro_yaw;


    /*
       Accelerometer Roll
    */

    accel_roll_deg =
        fast_atan2_deg(
            (float)accel_y,
            (float)accel_z);


    /*
       Accelerometer Pitch
    */

    ay =
        ((int32_t)accel_y) /
        2;

    az =
        ((int32_t)accel_z) /
        2;


    yz_squared =
        (uint32_t)(
            (ay * ay) +
            (az * az));


    yz_length_half =
        integer_sqrt(
            yz_squared);


    yz_length =
        (float)yz_length_half *
        2.0f;


    accel_pitch_deg =
        fast_atan2_deg(
            -(float)accel_x,
            yz_length);


    /*
       First frame
    */

    if (!attitude_initialized)
    {
        roll_angle =
            accel_roll_deg;

        pitch_angle =
            accel_pitch_deg;

        yaw_angle =
            0.0f;


        attitude_initialized =
            1U;


        return;
    }


    /*
       Complementary filter
    */

    roll_angle =
          FILTER_ALPHA *
          (
              roll_angle +
              gyro_roll * dt
          )
        +
          FILTER_ACCEL_ALPHA *
          accel_roll_deg;


    pitch_angle =
          FILTER_ALPHA *
          (
              pitch_angle +
              gyro_pitch * dt
          )
        +
          FILTER_ACCEL_ALPHA *
          accel_pitch_deg;


    /*
       Yaw integration
    */

    yaw_angle +=
        gyro_yaw *
        dt;


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
   ONE PID AXIS
   ============================================================ */

static float pid_axis(
    float setpoint,
    float measured,

    float kp,
    float ki,
    float kd,

    float dt,

    float *integral,
    float *previous_error,

    volatile float *p_debug,
    volatile float *i_debug,
    volatile float *d_debug,

    volatile float *error_debug)
{
    float error;
    float derivative;

    float p;
    float i;
    float d;

    float output;


    /*
       Rate PID:

       error =
           desired angular velocity
           -
           measured angular velocity
    */

    error =
        setpoint -
        measured;


    derivative = 0.0f;


    if (dt > 0.0001f)
    {
        derivative =
            (error -
             *previous_error) /
            dt;
    }


    /*
       Integral
    */

    *integral +=
        error *
        dt;


    *integral =
        clampf(
            *integral,
            -INTEGRAL_LIMIT,
            INTEGRAL_LIMIT);


    /*
       PID terms
    */

    p =
        kp *
        error;


    i =
        ki *
        (*integral);


    d =
        kd *
        derivative;


    /*
       Output = correction in
       virtual motor percentage points.
    */

    output =
        p +
        i +
        d;


    output =
        clampf(
            output,
            -PID_OUTPUT_LIMIT,
            PID_OUTPUT_LIMIT);


    *previous_error =
        error;


    *error_debug =
        error;


    *p_debug = p;
    *i_debug = i;
    *d_debug = d;


    return output;
}


/* ============================================================
   RATE PID
   ============================================================ */

static void rate_pid_update(
    float dt)
{
    pid_roll_output =
        pid_axis(
            rate_roll_setpoint,
            gyro_roll_dps,

            ROLL_KP,
            ROLL_KI,
            ROLL_KD,

            dt,

            &roll_integral,
            &roll_prev_error,

            &pid_roll_p,
            &pid_roll_i,
            &pid_roll_d,

            &rate_roll_error);


    pid_pitch_output =
        pid_axis(
            rate_pitch_setpoint,
            gyro_pitch_dps,

            PITCH_KP,
            PITCH_KI,
            PITCH_KD,

            dt,

            &pitch_integral,
            &pitch_prev_error,

            &pid_pitch_p,
            &pid_pitch_i,
            &pid_pitch_d,

            &rate_pitch_error);


    pid_yaw_output =
        pid_axis(
            rate_yaw_setpoint,
            gyro_yaw_dps,

            YAW_KP,
            YAW_KI,
            YAW_KD,

            dt,

            &yaw_integral,
            &yaw_prev_error,

            &pid_yaw_p,
            &pid_yaw_i,
            &pid_yaw_d,

            &rate_yaw_error);
}


/* ============================================================
   X MIXER - DRY RUN

   M1 = front-right = CW
   M2 = front-left  = CCW
   M3 = rear-left   = CW
   M4 = rear-right  = CCW


   Mixer:

   M1 = T + R + P - Y
   M2 = T - R + P + Y
   M3 = T - R - P - Y
   M4 = T + R - P + Y

   ============================================================ */

static void mixer_dry_run(void)
{
    float throttle;

    throttle =
        VIRTUAL_THROTTLE;


    /*
       FRONT-RIGHT
       M1 CW
    */

    mix_m1_percent =
        throttle
        + pid_roll_output
        + pid_pitch_output
        - pid_yaw_output;


    /*
       FRONT-LEFT
       M2 CCW
    */

    mix_m2_percent =
        throttle
        - pid_roll_output
        + pid_pitch_output
        + pid_yaw_output;


    /*
       REAR-LEFT
       M3 CW
    */

    mix_m3_percent =
        throttle
        - pid_roll_output
        - pid_pitch_output
        - pid_yaw_output;


    /*
       REAR-RIGHT
       M4 CCW
    */

    mix_m4_percent =
        throttle
        + pid_roll_output
        - pid_pitch_output
        + pid_yaw_output;


    /*
       Virtual clamp
    */

    mix_m1_percent =
        clampf(
            mix_m1_percent,
            0.0f,
            100.0f);


    mix_m2_percent =
        clampf(
            mix_m2_percent,
            0.0f,
            100.0f);


    mix_m3_percent =
        clampf(
            mix_m3_percent,
            0.0f,
            100.0f);


    mix_m4_percent =
        clampf(
            mix_m4_percent,
            0.0f,
            100.0f);


    /*
       ==============================================
       ABSOLUTE SAFETY

       MIXER IS NOT CONNECTED TO MOTORS.
       ==============================================
    */

    motor_all_off();
}


/* ============================================================
   MAIN
   ============================================================ */

int main(void)
{
    uint32_t last_loop_ms;

    uint32_t now;
    uint32_t elapsed;

    uint32_t read_start;
    uint32_t read_end;


    /* ========================================================
       GPIO CLOCKS
       ======================================================== */

    RCC_AHBENR |=
        GPIOA_EN |
        GPIOB_EN;


    (void)RCC_AHBENR;


    /* ========================================================
       BOARD ENABLE PA1
       ======================================================== */

    gpio_high(
        GPIOA_BASE,
        1U);


    gpio_output(
        GPIOA_BASE,
        1U);


    /* ========================================================
       MOTOR PINS SAFE
       ======================================================== */

    motor_gpio_prepare_low();


    /* ========================================================
       SYSTEM TIME
       ======================================================== */

    systick_init();


    delay_ms(100U);


    /* ========================================================
       TIM1
       ======================================================== */

    tim1_pwm_init();


    motor_all_off();


    /* ========================================================
       HARDWARE I2C
       ======================================================== */

    i2c_init();


    /* ========================================================
       M540
       ======================================================== */

    imu_ok =
        m540_init();


    if (!imu_ok)
    {
        calibration_status =
            3U;


        while (1)
        {
            motor_all_off();
        }
    }


    /* ========================================================
       GYRO CALIBRATION

       DO NOT MOVE DRONE.
       ======================================================== */

    if (!gyro_calibrate())
    {
        while (1)
        {
            motor_all_off();
        }
    }


    /* ========================================================
       RESET RUNTIME STATE
       ======================================================== */

    imu_frame_counter = 0U;

    imu_error_counter = 0U;

    imu_consecutive_errors = 0U;


    loop_dt_max_ms = 0U;

    imu_read_time_max_ms = 0U;


    attitude_initialized = 0U;


    roll_integral = 0.0f;
    pitch_integral = 0.0f;
    yaw_integral = 0.0f;


    roll_prev_error = 0.0f;
    pitch_prev_error = 0.0f;
    yaw_prev_error = 0.0f;


    /*
       No transmitter yet.

       Desired angular velocity = zero.
    */

    rate_roll_setpoint = 0.0f;

    rate_pitch_setpoint = 0.0f;

    rate_yaw_setpoint = 0.0f;


    /*
       Safety status
    */

    physical_motor_lock = 1U;

    dry_run_enabled = 1U;


    last_loop_ms =
        system_ms;


    /* ========================================================
       FLIGHT LOOP
       200 Hz
       ======================================================== */

    while (1)
    {
        now =
            system_ms;


        elapsed =
            (uint32_t)(
                now -
                last_loop_ms);


        if (elapsed >=
            LOOP_PERIOD_MS)
        {
            last_loop_ms =
                now;


            /* ================================================
               LOOP TIMING
               ================================================ */

            loop_dt_ms =
                elapsed;


            loop_dt =
                (float)elapsed *
                0.001f;


            if (elapsed > 0U)
            {
                loop_hz =
                    1000.0f /
                    (float)elapsed;
            }


            if (elapsed >
                loop_dt_max_ms)
            {
                loop_dt_max_ms =
                    elapsed;
            }


            /* ================================================
               IMU READ
               ================================================ */

            read_start =
                system_ms;


            if (m540_read_raw())
            {
                read_end =
                    system_ms;


                imu_read_time_ms =
                    (uint32_t)(
                        read_end -
                        read_start);


                if (imu_read_time_ms >
                    imu_read_time_max_ms)
                {
                    imu_read_time_max_ms =
                        imu_read_time_ms;
                }


                imu_frame_counter++;


                imu_consecutive_errors =
                    0U;


                /* ============================================
                   ATTITUDE
                   ============================================ */

                attitude_update(
                    loop_dt);


                /* ============================================
                   RATE PID
                   ============================================ */

                rate_pid_update(
                    loop_dt);


                /* ============================================
                   X MIXER
                   ============================================ */

                mixer_dry_run();
            }


            /* ================================================
               IMU ERROR
               ================================================ */

            else
            {
                read_end =
                    system_ms;


                imu_read_time_ms =
                    (uint32_t)(
                        read_end -
                        read_start);


                imu_error_counter++;


                imu_consecutive_errors++;


                /*
                   Kill all calculated control output.
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


                motor_all_off();
            }


            /* ================================================
               SECOND INDEPENDENT PHYSICAL MOTOR LOCK

               Even if mixer code changes accidentally,
               CCR registers are overwritten with ZERO.
               ================================================ */

            TIM1_CCR1 = 0U;
            TIM1_CCR2 = 0U;
            TIM1_CCR3 = 0U;
            TIM1_CCR4 = 0U;


            motor1_percent = 0U;
            motor2_percent = 0U;
            motor3_percent = 0U;
            motor4_percent = 0U;
        }
    }
}
