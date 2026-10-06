#include <stdint.h>

/* =========================================================
   DRONE_ - FLIGHT CONTROLLER STEP 2
   STM32F031 + M540 + TIM1

   IMU:
       PB8 = SCL
       PB7 = SDA
       address = 0x69
       WHO_AM_I = 0x7D

   MOTORS:
       PA8  TIM1_CH1 -> M3
       PA9  TIM1_CH2 -> M2
       PA10 TIM1_CH3 -> M4
       PA11 TIM1_CH4 -> M1

   STEP 2:
       - faster software I2C
       - target attitude loop = 200 Hz
       - gyro calibration
       - Roll / Pitch complementary filter
       - loop timing diagnostics
       - I2C error diagnostics

   MOTORS ARE FORCED TO 0%
   ========================================================= */


/* =========================================================
   Register helper
   ========================================================= */

#define REG32(addr) (*(volatile uint32_t *)(addr))


/* =========================================================
   RCC
   ========================================================= */

#define RCC_AHBENR      REG32(0x40021014U)
#define RCC_APB2RSTR    REG32(0x4002100CU)
#define RCC_APB2ENR     REG32(0x40021018U)

#define GPIOA_EN        (1U << 17)
#define GPIOB_EN        (1U << 18)
#define TIM1_EN         (1U << 11)


/* =========================================================
   GPIO
   ========================================================= */

#define GPIOA_BASE      0x48000000U
#define GPIOB_BASE      0x48000400U

#define GPIO_MODER(p)   REG32((p) + 0x00U)
#define GPIO_OTYPER(p)  REG32((p) + 0x04U)
#define GPIO_OSPEEDR(p) REG32((p) + 0x08U)
#define GPIO_PUPDR(p)   REG32((p) + 0x0CU)
#define GPIO_IDR(p)     REG32((p) + 0x10U)
#define GPIO_BSRR(p)    REG32((p) + 0x18U)
#define GPIO_AFRH(p)    REG32((p) + 0x24U)


/* =========================================================
   SysTick
   ========================================================= */

#define SYST_CSR        REG32(0xE000E010U)
#define SYST_RVR        REG32(0xE000E014U)
#define SYST_CVR        REG32(0xE000E018U)


/* =========================================================
   TIM1
   ========================================================= */

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


/* =========================================================
   M540
   ========================================================= */

#define IMU_PORT        GPIOB_BASE

#define IMU_SDA_PIN     7U
#define IMU_SCL_PIN     8U

#define M540_ADDR       0x69U


/* =========================================================
   Timing
   ========================================================= */

/*
   CPU currently remains at the already-working 8 MHz.

   SysTick:
       8 MHz / 8000 = 1 kHz
       = 1 ms
*/

#define LOOP_PERIOD_MS  5U

/*
   5 ms = 200 Hz target flight loop
*/


/* =========================================================
   Software I2C speed

   Previous stable version:
       40 iterations

   Step 2:
       8 iterations

   We do NOT jump directly to extremely fast timing.
   ========================================================= */

#define SOFT_I2C_DELAY_COUNT 8U


/* =========================================================
   PWM
   ========================================================= */

#define PWM_PERIOD      8000U
#define PWM_ARR_VALUE   7999U


/* =========================================================
   Sensor scale
   ========================================================= */

#define ACCEL_LSB_PER_G  2048.0f

/*
   Temporary working gyro scale.

   We will calibrate this later experimentally.
*/

#define GYRO_LSB_PER_DPS 16.4f


/* =========================================================
   Complementary filter

   At ~200 Hz use 0.99 instead of old 0.98.
   ========================================================= */

#define FILTER_ALPHA       0.99f
#define FILTER_ACCEL_ALPHA 0.01f


/* =========================================================
   Global system time
   ========================================================= */

volatile uint32_t system_ms = 0;


/* =========================================================
   IMU status
   ========================================================= */

volatile uint32_t imu_ok = 0;
volatile uint8_t imu_whoami = 0;

volatile uint32_t imu_read_ok = 0;

volatile uint32_t imu_frame_counter = 0;

volatile uint32_t imu_error_counter = 0;

volatile uint32_t imu_consecutive_errors = 0;


/* =========================================================
   RAW accelerometer
   ========================================================= */

volatile int16_t accel_x = 0;
volatile int16_t accel_y = 0;
volatile int16_t accel_z = 0;


/* =========================================================
   RAW gyro
   ========================================================= */

volatile int16_t gyro_x = 0;
volatile int16_t gyro_y = 0;
volatile int16_t gyro_z = 0;


/* =========================================================
   Temperature
   ========================================================= */

volatile int16_t imu_temperature_raw = 0;


/* =========================================================
   Calibration
   ========================================================= */

/*
   0 = not started
   1 = calibrating
   2 = complete
   3 = failed
*/

volatile uint32_t calibration_status = 0;

volatile uint32_t calibration_samples = 0;

volatile float gyro_x_offset = 0.0f;
volatile float gyro_y_offset = 0.0f;
volatile float gyro_z_offset = 0.0f;


/* =========================================================
   Corrected gyro
   ========================================================= */

volatile float gyro_roll_raw = 0.0f;
volatile float gyro_pitch_raw = 0.0f;
volatile float gyro_yaw_raw = 0.0f;

volatile float gyro_roll_dps = 0.0f;
volatile float gyro_pitch_dps = 0.0f;
volatile float gyro_yaw_dps = 0.0f;


/* =========================================================
   Accelerometer
   ========================================================= */

volatile float accel_x_g = 0.0f;
volatile float accel_y_g = 0.0f;
volatile float accel_z_g = 0.0f;

volatile float accel_roll_deg = 0.0f;
volatile float accel_pitch_deg = 0.0f;


/* =========================================================
   Attitude
   ========================================================= */

volatile float roll_angle = 0.0f;
volatile float pitch_angle = 0.0f;
volatile float yaw_angle = 0.0f;

volatile uint32_t attitude_initialized = 0;


/* =========================================================
   LOOP DIAGNOSTICS
   ========================================================= */

volatile uint32_t flight_loop_counter = 0;

/*
   Actual time between attitude updates.
*/

volatile uint32_t loop_dt_ms = 0;

volatile float loop_dt = 0.0f;

volatile float loop_hz = 0.0f;


/*
   Maximum observed dt after startup.
*/

volatile uint32_t loop_dt_max_ms = 0;


/*
   Approximate time required for one IMU read.
*/

volatile uint32_t imu_read_time_ms = 0;

volatile uint32_t imu_read_time_max_ms = 0;


/* =========================================================
   Motors

   MUST REMAIN ZERO
   ========================================================= */

volatile uint32_t motor1_percent = 0;
volatile uint32_t motor2_percent = 0;
volatile uint32_t motor3_percent = 0;
volatile uint32_t motor4_percent = 0;


/* =========================================================
   SysTick ISR
   ========================================================= */

void SysTick_Handler(void)
{
    system_ms++;
}


/* =========================================================
   SysTick init
   ========================================================= */

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


/* =========================================================
   Delay
   ========================================================= */

static void delay_ms(uint32_t ms)
{
    uint32_t start = system_ms;

    while ((uint32_t)(system_ms - start) < ms)
    {
    }
}


/* =========================================================
   GPIO helpers
   ========================================================= */

static void gpio_output(uint32_t port, uint32_t pin)
{
    GPIO_MODER(port) &=
        ~(3U << (pin * 2U));

    GPIO_MODER(port) |=
        (1U << (pin * 2U));
}


static void gpio_input_pullup(uint32_t port, uint32_t pin)
{
    GPIO_MODER(port) &=
        ~(3U << (pin * 2U));

    GPIO_PUPDR(port) &=
        ~(3U << (pin * 2U));

    GPIO_PUPDR(port) |=
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


static uint32_t gpio_read(uint32_t port, uint32_t pin)
{
    return
        (GPIO_IDR(port) >> pin) & 1U;
}


/* =========================================================
   SOFTWARE I2C
   ========================================================= */

static void i2c_delay(void)
{
    for (volatile uint32_t i = 0;
         i < SOFT_I2C_DELAY_COUNT;
         i++)
    {
    }
}


/* SDA LOW */

static void sda_low(void)
{
    gpio_low(
        IMU_PORT,
        IMU_SDA_PIN);

    gpio_output(
        IMU_PORT,
        IMU_SDA_PIN);
}


/* SDA released HIGH */

static void sda_release(void)
{
    gpio_input_pullup(
        IMU_PORT,
        IMU_SDA_PIN);
}


/* SCL LOW */

static void scl_low(void)
{
    gpio_low(
        IMU_PORT,
        IMU_SCL_PIN);

    gpio_output(
        IMU_PORT,
        IMU_SCL_PIN);
}


/* SCL released HIGH */

static void scl_release(void)
{
    gpio_input_pullup(
        IMU_PORT,
        IMU_SCL_PIN);
}


static void i2c_init(void)
{
    sda_release();
    scl_release();

    i2c_delay();
}


static void i2c_start(void)
{
    sda_release();
    scl_release();

    i2c_delay();

    sda_low();

    i2c_delay();

    scl_low();

    i2c_delay();
}


static void i2c_stop(void)
{
    sda_low();

    i2c_delay();

    scl_release();

    i2c_delay();

    sda_release();

    i2c_delay();
}


/* =========================================================
   Write one byte
   ========================================================= */

static uint32_t i2c_write_byte(uint8_t value)
{
    for (uint32_t i = 0;
         i < 8U;
         i++)
    {
        if (value & 0x80U)
        {
            sda_release();
        }
        else
        {
            sda_low();
        }


        i2c_delay();

        scl_release();

        i2c_delay();

        scl_low();

        i2c_delay();


        value <<= 1;
    }


    /* ACK */

    sda_release();

    i2c_delay();

    scl_release();

    i2c_delay();


    uint32_t ack =
        (gpio_read(
            IMU_PORT,
            IMU_SDA_PIN) == 0U);


    scl_low();

    i2c_delay();


    return ack;
}


/* =========================================================
   Read one byte
   ========================================================= */

static uint8_t i2c_read_byte(uint32_t send_ack)
{
    uint8_t value = 0U;


    sda_release();


    for (uint32_t i = 0;
         i < 8U;
         i++)
    {
        value <<= 1;


        scl_release();

        i2c_delay();


        if (gpio_read(
                IMU_PORT,
                IMU_SDA_PIN))
        {
            value |= 1U;
        }


        scl_low();

        i2c_delay();
    }


    if (send_ack)
    {
        sda_low();
    }
    else
    {
        sda_release();
    }


    i2c_delay();

    scl_release();

    i2c_delay();

    scl_low();

    i2c_delay();

    sda_release();


    return value;
}


/* =========================================================
   M540 write
   ========================================================= */

static uint32_t m540_write_reg(
    uint8_t reg,
    uint8_t value)
{
    i2c_start();


    if (!i2c_write_byte(
            (uint8_t)(M540_ADDR << 1)))
    {
        i2c_stop();
        return 0U;
    }


    if (!i2c_write_byte(reg))
    {
        i2c_stop();
        return 0U;
    }


    if (!i2c_write_byte(value))
    {
        i2c_stop();
        return 0U;
    }


    i2c_stop();

    return 1U;
}


/* =========================================================
   M540 read multiple
   ========================================================= */

static uint32_t m540_read_regs(
    uint8_t reg,
    uint8_t *data,
    uint32_t count)
{
    if (count == 0U)
    {
        return 0U;
    }


    i2c_start();


    if (!i2c_write_byte(
            (uint8_t)(M540_ADDR << 1)))
    {
        i2c_stop();
        return 0U;
    }


    if (!i2c_write_byte(reg))
    {
        i2c_stop();
        return 0U;
    }


    /* repeated START */

    i2c_start();


    if (!i2c_write_byte(
            (uint8_t)((M540_ADDR << 1) | 1U)))
    {
        i2c_stop();
        return 0U;
    }


    for (uint32_t i = 0;
         i < count;
         i++)
    {
        data[i] =
            i2c_read_byte(
                (i + 1U) < count);
    }


    i2c_stop();

    return 1U;
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


/* =========================================================
   M540 initialization
   ========================================================= */

static uint32_t m540_init(void)
{
    uint8_t id = 0U;


    /* reset */

    if (!m540_write_reg(
            0x6BU,
            0x80U))
    {
        return 0U;
    }


    delay_ms(50);


    /* wake */

    if (!m540_write_reg(
            0x6BU,
            0x01U))
    {
        return 0U;
    }


    delay_ms(10);


    /* WHO_AM_I */

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


    /* accel +/-16g */

    if (!m540_write_reg(
            0x1CU,
            0x18U))
    {
        return 0U;
    }


    /* accel filter */

    if (!m540_write_reg(
            0x1DU,
            0x05U))
    {
        return 0U;
    }


    /* gyro +/-2000 */

    if (!m540_write_reg(
            0x1BU,
            0x18U))
    {
        return 0U;
    }


    /* gyro filter */

    if (!m540_write_reg(
            0x1AU,
            0x00U))
    {
        return 0U;
    }


    delay_ms(10);


    return 1U;
}


/* =========================================================
   Read M540 raw frame
   ========================================================= */

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
            ((uint16_t)data[0] << 8) |
             data[1]);


    accel_y =
        (int16_t)(
            ((uint16_t)data[2] << 8) |
             data[3]);


    accel_z =
        (int16_t)(
            ((uint16_t)data[4] << 8) |
             data[5]);


    imu_temperature_raw =
        (int16_t)(
            ((uint16_t)data[6] << 8) |
             data[7]);


    gyro_x =
        (int16_t)(
            ((uint16_t)data[8] << 8) |
             data[9]);


    gyro_y =
        (int16_t)(
            ((uint16_t)data[10] << 8) |
             data[11]);


    gyro_z =
        (int16_t)(
            ((uint16_t)data[12] << 8) |
             data[13]);


    return 1U;
}


/* =========================================================
   MOTOR GPIO
   ========================================================= */

static void motor_gpio_init(void)
{
    /* push-pull */

    GPIO_OTYPER(GPIOA_BASE) &=
        ~(
            (1U << 8) |
            (1U << 9) |
            (1U << 10) |
            (1U << 11)
        );


    /* high speed */

    for (uint32_t pin = 8U;
         pin <= 11U;
         pin++)
    {
        GPIO_OSPEEDR(GPIOA_BASE) &=
            ~(3U << (pin * 2U));

        GPIO_OSPEEDR(GPIOA_BASE) |=
            (3U << (pin * 2U));


        GPIO_PUPDR(GPIOA_BASE) &=
            ~(3U << (pin * 2U));


        GPIO_MODER(GPIOA_BASE) &=
            ~(3U << (pin * 2U));

        GPIO_MODER(GPIOA_BASE) |=
            (2U << (pin * 2U));
    }


    /* PA8-PA11 = AF2 */

    GPIO_AFRH(GPIOA_BASE) &=
        ~0x0000FFFFU;


    GPIO_AFRH(GPIOA_BASE) |=
        (2U << 0)  |
        (2U << 4)  |
        (2U << 8)  |
        (2U << 12);
}


/* =========================================================
   TIM1 PWM initialization
   ========================================================= */

static void tim1_pwm_init(void)
{
    RCC_APB2ENR |= TIM1_EN;

    (void)RCC_APB2ENR;


    RCC_APB2RSTR |= TIM1_EN;
    RCC_APB2RSTR &= ~TIM1_EN;


    TIM1_CR1  = 0U;
    TIM1_CR2  = 0U;
    TIM1_SMCR = 0U;
    TIM1_DIER = 0U;

    TIM1_CCER = 0U;
    TIM1_BDTR = 0U;


    TIM1_PSC = 0U;
    TIM1_ARR = PWM_ARR_VALUE;
    TIM1_RCR = 0U;
    TIM1_CNT = 0U;


    /* motors OFF */

    TIM1_CCR1 = 0U;
    TIM1_CCR2 = 0U;
    TIM1_CCR3 = 0U;
    TIM1_CCR4 = 0U;


    /* CH1 + CH2 PWM mode 1 */

    TIM1_CCMR1 =
        (6U << 4) |
        (1U << 3) |
        (6U << 12) |
        (1U << 11);


    /* CH3 + CH4 PWM mode 1 */

    TIM1_CCMR2 =
        (6U << 4) |
        (1U << 3) |
        (6U << 12) |
        (1U << 11);


    /* enable CH1-CH4 */

    TIM1_CCER =
        (1U << 0) |
        (1U << 4) |
        (1U << 8) |
        (1U << 12);


    /* MOE */

    TIM1_BDTR =
        (1U << 15);


    /* ARR preload */

    TIM1_CR1 |=
        (1U << 7);


    TIM1_EGR =
        (1U << 0);


    TIM1_SR = 0U;


    motor_gpio_init();


    TIM1_CR1 |=
        (1U << 0);
}


/* =========================================================
   Motors OFF

   We write CCR registers directly because STEP 2
   does NOT allow motor operation.
   ========================================================= */

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


/* =========================================================
   Integer square root
   ========================================================= */

static uint32_t integer_sqrt(uint32_t value)
{
    uint32_t result = 0U;

    uint32_t bit =
        (1UL << 30);


    while (bit > value)
    {
        bit >>= 2;
    }


    while (bit != 0U)
    {
        if (value >= result + bit)
        {
            value -=
                result + bit;

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


/* =========================================================
   Fast atan2 approximation
   ========================================================= */

static float fast_atan2_deg(
    float y,
    float x)
{
    const float PI_4 =
        0.78539816339f;

    const float PI_3_4 =
        2.35619449019f;

    const float RAD_TO_DEG =
        57.295779513f;


    float abs_y =
        (y < 0.0f)
        ? -y
        : y;


    abs_y +=
        0.000001f;


    float r;
    float angle;


    if (x < 0.0f)
    {
        r =
            (x + abs_y) /
            (abs_y - x);

        angle =
            PI_3_4;
    }
    else
    {
        r =
            (x - abs_y) /
            (x + abs_y);

        angle =
            PI_4;
    }


    angle +=
        (0.1963f * r * r - 0.9817f) * r;


    if (y < 0.0f)
    {
        angle =
            -angle;
    }


    return
        angle * RAD_TO_DEG;
}


/* =========================================================
   Gyro calibration
   ========================================================= */

static uint32_t gyro_calibrate(void)
{
    int32_t sum_x = 0;
    int32_t sum_y = 0;
    int32_t sum_z = 0;

    uint32_t valid = 0U;


    calibration_status = 1U;

    calibration_samples = 0U;


    /*
       500 samples.

       At faster I2C this now takes much less time.
       3 ms gap keeps calibration calm and predictable.
    */

    for (uint32_t i = 0U;
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


/* =========================================================
   ATTITUDE UPDATE
   ========================================================= */

static void attitude_update(float dt)
{
    /* =====================================================
       Accelerometer -> g
       ===================================================== */

    accel_x_g =
        (float)accel_x /
        ACCEL_LSB_PER_G;


    accel_y_g =
        (float)accel_y /
        ACCEL_LSB_PER_G;


    accel_z_g =
        (float)accel_z /
        ACCEL_LSB_PER_G;


    /* =====================================================
       Correct gyro offsets
       ===================================================== */

    gyro_roll_raw =
        (float)gyro_x -
        gyro_x_offset;


    gyro_pitch_raw =
        (float)gyro_y -
        gyro_y_offset;


    gyro_yaw_raw =
        (float)gyro_z -
        gyro_z_offset;


    /* =====================================================
       raw -> deg/s
       ===================================================== */

    gyro_roll_dps =
        gyro_roll_raw /
        GYRO_LSB_PER_DPS;


    gyro_pitch_dps =
        gyro_pitch_raw /
        GYRO_LSB_PER_DPS;


    gyro_yaw_dps =
        gyro_yaw_raw /
        GYRO_LSB_PER_DPS;


    /* =====================================================
       Accelerometer ROLL

       IMPORTANT:
       We are deliberately NOT changing the sign yet.

       Exact physical orientation test will determine
       final FC convention.
       ===================================================== */

    accel_roll_deg =
        fast_atan2_deg(
            (float)accel_y,
            (float)accel_z);


    /* =====================================================
       Accelerometer PITCH
       ===================================================== */

    int32_t ay =
        ((int32_t)accel_y) / 2;

    int32_t az =
        ((int32_t)accel_z) / 2;


    uint32_t yz_squared =
        (uint32_t)(
            (ay * ay) +
            (az * az));


    uint32_t yz_length_half =
        integer_sqrt(
            yz_squared);


    float yz_length =
        (float)yz_length_half *
        2.0f;


    accel_pitch_deg =
        fast_atan2_deg(
            -(float)accel_x,
            yz_length);


    /* =====================================================
       First attitude frame
       ===================================================== */

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


    /* =====================================================
       Gyro prediction
       ===================================================== */

    float gyro_roll_angle =
        roll_angle +
        gyro_roll_dps * dt;


    float gyro_pitch_angle =
        pitch_angle +
        gyro_pitch_dps * dt;


    /* =====================================================
       Complementary filter
       ===================================================== */

    roll_angle =
        FILTER_ALPHA *
        gyro_roll_angle +

        FILTER_ACCEL_ALPHA *
        accel_roll_deg;


    pitch_angle =
        FILTER_ALPHA *
        gyro_pitch_angle +

        FILTER_ACCEL_ALPHA *
        accel_pitch_deg;


    /* =====================================================
       Yaw integration
       ===================================================== */

    yaw_angle +=
        gyro_yaw_dps *
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


/* =========================================================
   MAIN
   ========================================================= */

int main(void)
{
    /* =====================================================
       GPIO clocks
       ===================================================== */

    RCC_AHBENR |=
        GPIOA_EN |
        GPIOB_EN;


    (void)RCC_AHBENR;


    /* =====================================================
       Board enable PA1
       ===================================================== */

    gpio_high(
        GPIOA_BASE,
        1U);


    gpio_output(
        GPIOA_BASE,
        1U);


    /* =====================================================
       Force motor pins LOW before TIM1
       ===================================================== */

    gpio_low(
        GPIOA_BASE,
        8U);

    gpio_low(
        GPIOA_BASE,
        9U);

    gpio_low(
        GPIOA_BASE,
        10U);

    gpio_low(
        GPIOA_BASE,
        11U);


    gpio_output(
        GPIOA_BASE,
        8U);

    gpio_output(
        GPIOA_BASE,
        9U);

    gpio_output(
        GPIOA_BASE,
        10U);

    gpio_output(
        GPIOA_BASE,
        11U);


    /* =====================================================
       System time
       ===================================================== */

    systick_init();


    delay_ms(100U);


    /* =====================================================
       Motor hardware PWM
       ===================================================== */

    tim1_pwm_init();

    motor_all_off();


    /* =====================================================
       I2C
       ===================================================== */

    i2c_init();


    /* =====================================================
       M540
       ===================================================== */

    imu_ok =
        m540_init();


    if (!imu_ok)
    {
        calibration_status = 3U;


        while (1)
        {
            motor_all_off();
        }
    }


    /* =====================================================
       Gyro calibration

       DO NOT MOVE THE BOARD
       ===================================================== */

    if (!gyro_calibrate())
    {
        while (1)
        {
            motor_all_off();
        }
    }


    /* =====================================================
       Reset runtime diagnostics
       ===================================================== */

    imu_frame_counter = 0U;

    imu_error_counter = 0U;

    imu_consecutive_errors = 0U;

    flight_loop_counter = 0U;

    loop_dt_max_ms = 0U;

    imu_read_time_max_ms = 0U;

    attitude_initialized = 0U;


    uint32_t last_loop_ms =
        system_ms;


    /* =====================================================
       FLIGHT CONTROLLER LOOP
       ===================================================== */

    while (1)
    {
        uint32_t now =
            system_ms;


        uint32_t elapsed =
            (uint32_t)(
                now -
                last_loop_ms);


        /*
           Target:
               every 5 ms
               = 200 Hz
        */

        if (elapsed >= LOOP_PERIOD_MS)
        {
            last_loop_ms =
                now;


            /* =============================================
               Actual dt
               ============================================= */

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


            /* =============================================
               IMU read timing
               ============================================= */

            uint32_t read_start =
                system_ms;


            imu_read_ok =
                m540_read_raw();


            uint32_t read_end =
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


            /* =============================================
               Successful frame
               ============================================= */

            if (imu_read_ok)
            {
                imu_frame_counter++;

                imu_consecutive_errors =
                    0U;


                attitude_update(
                    loop_dt);


                flight_loop_counter++;
            }


            /* =============================================
               I2C error
               ============================================= */

            else
            {
                imu_error_counter++;

                imu_consecutive_errors++;


                /*
                   If communication ever becomes unstable,
                   motors remain OFF.
                */
            }


            /* =============================================
               ABSOLUTE MOTOR SAFETY
               ============================================= */

            motor_all_off();
        }
    }
}
