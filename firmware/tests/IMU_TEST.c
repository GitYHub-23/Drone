#include <stdint.h>

/* =========================================================
   DRONE FLIGHT CONTROLLER - STEP 1

   MCU:
       STM32F031K4

   IMU:
       M540
       PB8 = SCL
       PB7 = SDA
       I2C address = 0x69
       WHO_AM_I = 0x7D

   MOTOR PWM:
       PA8  -> TIM1_CH1 -> M3
       PA9  -> TIM1_CH2 -> M2
       PA10 -> TIM1_CH3 -> M4
       PA11 -> TIM1_CH4 -> M1

   CURRENT FEATURES:
       - M540 initialization
       - accelerometer reading
       - gyroscope reading
       - automatic gyro calibration
       - gyro bias correction
       - Roll angle
       - Pitch angle
       - Yaw rate / integrated yaw
       - TIM1 initialized
       - ALL MOTORS FORCED OFF

   IMPORTANT:
       REMOVE PROPELLERS
       KEEP DRONE STILL DURING CALIBRATION
   ========================================================= */


/* =========================================================
   Register access
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
#define DBGMCU_EN       (1U << 22)


/* =========================================================
   GPIO
   ========================================================= */

#define GPIOA_BASE      0x48000000U
#define GPIOB_BASE      0x48000400U

#define MODER(base)     REG32((base) + 0x00U)
#define OTYPER(base)    REG32((base) + 0x04U)
#define OSPEEDR(base)   REG32((base) + 0x08U)
#define PUPDR(base)     REG32((base) + 0x0CU)
#define IDR(base)       REG32((base) + 0x10U)
#define BSRR(base)      REG32((base) + 0x18U)
#define AFRH(base)      REG32((base) + 0x24U)


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
   Debug
   ========================================================= */

#define DBGMCU_APB2_FZ  REG32(0x4001580CU)

#define DBG_TIM1_STOP   (1U << 11)


/* =========================================================
   M540
   ========================================================= */

#define IMU_PORT        GPIOB_BASE

#define IMU_SDA_PIN     7U
#define IMU_SCL_PIN     8U

#define M540_ADDR       0x69U


/* =========================================================
   Motors
   ========================================================= */

#define MOTOR_M1        1U
#define MOTOR_M2        2U
#define MOTOR_M3        3U
#define MOTOR_M4        4U


/* =========================================================
   PWM

   We keep the already physically tested 1 kHz PWM.

   8 MHz / 8000 = 1000 Hz
   ========================================================= */

#define PWM_PERIOD      8000U
#define PWM_ARR_VALUE   7999U


/* =========================================================
   Sensor constants
   ========================================================= */

/*
   Accelerometer is configured for +/-16g.

   Our physical tests already showed:
       ~2048 raw = 1g
*/

#define ACCEL_LSB_PER_G 2048.0f


/*
   Gyro register is configured as +/-2000 deg/s.

   16.4 LSB/(deg/s) is currently used as the MPU-compatible
   scale.

   We will verify the exact M540 scale experimentally later.
*/

#define GYRO_LSB_PER_DPS 16.4f


/* =========================================================
   Complementary filter
   ========================================================= */

#define FILTER_GYRO_WEIGHT  0.98f
#define FILTER_ACCEL_WEIGHT 0.02f


/* =========================================================
   Time
   ========================================================= */

volatile uint32_t system_ms = 0;


/* =========================================================
   IMU status
   ========================================================= */

volatile uint8_t imu_whoami = 0;

volatile uint32_t imu_ok = 0;
volatile uint32_t imu_read_ok = 0;
volatile uint32_t imu_frame_counter = 0;


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
   Gyro calibration

   status:
       0 = not started
       1 = calibrating
       2 = calibration complete
       3 = calibration failed
   ========================================================= */

volatile uint32_t calibration_status = 0;

volatile uint32_t calibration_samples = 0;

volatile float gyro_x_offset = 0.0f;
volatile float gyro_y_offset = 0.0f;
volatile float gyro_z_offset = 0.0f;


/* =========================================================
   Corrected gyro RAW values
   ========================================================= */

volatile float gyro_roll_raw = 0.0f;
volatile float gyro_pitch_raw = 0.0f;
volatile float gyro_yaw_raw = 0.0f;


/* =========================================================
   Gyroscope deg/s
   ========================================================= */

volatile float gyro_roll_dps = 0.0f;
volatile float gyro_pitch_dps = 0.0f;
volatile float gyro_yaw_dps = 0.0f;


/* =========================================================
   Accelerometer in g
   ========================================================= */

volatile float accel_x_g = 0.0f;
volatile float accel_y_g = 0.0f;
volatile float accel_z_g = 0.0f;


/* =========================================================
   Accelerometer-only angles
   ========================================================= */

volatile float accel_roll_deg = 0.0f;
volatile float accel_pitch_deg = 0.0f;


/* =========================================================
   Fused attitude

   THESE ARE THE INTERESTING VARIABLES :)
   ========================================================= */

volatile float roll_angle = 0.0f;
volatile float pitch_angle = 0.0f;

volatile float yaw_angle = 0.0f;


/* =========================================================
   Attitude state
   ========================================================= */

volatile uint32_t attitude_initialized = 0;

volatile float loop_dt = 0.0f;

volatile uint32_t flight_loop_counter = 0;


/* =========================================================
   Motors

   They MUST remain zero in this firmware.
   ========================================================= */

volatile uint32_t motor1_percent = 0;
volatile uint32_t motor2_percent = 0;
volatile uint32_t motor3_percent = 0;
volatile uint32_t motor4_percent = 0;


/* =========================================================
   SysTick interrupt

   Called every 1 ms.
   ========================================================= */

void SysTick_Handler(void)
{
    system_ms++;
}


/* =========================================================
   SysTick initialization
   ========================================================= */

static void systick_init(void)
{
    /*
       CPU = 8 MHz

       8,000,000 / 8000 = 1000 Hz

       interrupt every 1 ms
    */

    SYST_CSR = 0U;

    SYST_RVR = 7999U;

    SYST_CVR = 0U;

    SYST_CSR =
        (1U << 2) |     /* CPU clock */
        (1U << 1) |     /* interrupt enable */
        (1U << 0);      /* SysTick enable */
}


/* =========================================================
   Millisecond delay
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

static void gpio_output(
    uint32_t port,
    uint32_t pin)
{
    MODER(port) &= ~(3U << (pin * 2U));

    MODER(port) |=
        (1U << (pin * 2U));
}


static void gpio_input_pullup(
    uint32_t port,
    uint32_t pin)
{
    MODER(port) &=
        ~(3U << (pin * 2U));

    PUPDR(port) &=
        ~(3U << (pin * 2U));

    PUPDR(port) |=
        (1U << (pin * 2U));
}


static void gpio_high(
    uint32_t port,
    uint32_t pin)
{
    BSRR(port) =
        (1U << pin);
}


static void gpio_low(
    uint32_t port,
    uint32_t pin)
{
    BSRR(port) =
        (1U << (pin + 16U));
}


static uint32_t gpio_read(
    uint32_t port,
    uint32_t pin)
{
    return
        (IDR(port) >> pin) & 1U;
}


/* =========================================================
   SOFTWARE I2C
   ========================================================= */

static void i2c_delay(void)
{
    for (volatile uint32_t i = 0; i < 40U; i++)
    {
    }
}


static void sda_low(void)
{
    gpio_low(
        IMU_PORT,
        IMU_SDA_PIN);

    gpio_output(
        IMU_PORT,
        IMU_SDA_PIN);
}


static void sda_release(void)
{
    gpio_input_pullup(
        IMU_PORT,
        IMU_SDA_PIN);
}


static void scl_low(void)
{
    gpio_low(
        IMU_PORT,
        IMU_SCL_PIN);

    gpio_output(
        IMU_PORT,
        IMU_SCL_PIN);
}


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
   I2C write
   ========================================================= */

static uint32_t i2c_write_byte(uint8_t value)
{
    for (uint32_t i = 0; i < 8U; i++)
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
   I2C read
   ========================================================= */

static uint8_t i2c_read_byte(uint32_t send_ack)
{
    uint8_t value = 0;


    sda_release();


    for (uint32_t i = 0; i < 8U; i++)
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
   M540 write register
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
   M540 read registers
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


    /* Starting register */

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


    /* Repeated START */

    i2c_start();


    if (!i2c_write_byte(
            (uint8_t)((M540_ADDR << 1) | 1U)))
    {
        i2c_stop();

        return 0U;
    }


    for (uint32_t i = 0; i < count; i++)
    {
        data[i] =
            i2c_read_byte(
                (i + 1U) < count);
    }


    i2c_stop();


    return 1U;
}


/* =========================================================
   M540 read one register
   ========================================================= */

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
    uint8_t id = 0;


    /* Reset */

    if (!m540_write_reg(
            0x6BU,
            0x80U))
    {
        return 0U;
    }


    delay_ms(50);


    /* Wake */

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


    /* Accelerometer +/-16g */

    if (!m540_write_reg(
            0x1CU,
            0x18U))
    {
        return 0U;
    }


    /* Accelerometer filter */

    if (!m540_write_reg(
            0x1DU,
            0x05U))
    {
        return 0U;
    }


    /* Gyroscope +/-2000 deg/s */

    if (!m540_write_reg(
            0x1BU,
            0x18U))
    {
        return 0U;
    }


    /* Gyroscope filter */

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
   M540 raw measurement
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


    /* Accelerometer */

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


    /* Temperature */

    imu_temperature_raw =
        (int16_t)(
            ((uint16_t)data[6] << 8) |
             data[7]);


    /* Gyroscope */

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
   TIM1 motor GPIO
   ========================================================= */

static void motor_gpio_init(void)
{
    /* Push-pull */

    OTYPER(GPIOA_BASE) &=
        ~(
            (1U << 8)  |
            (1U << 9)  |
            (1U << 10) |
            (1U << 11)
        );


    /* High speed */

    for (uint32_t pin = 8U;
         pin <= 11U;
         pin++)
    {
        OSPEEDR(GPIOA_BASE) &=
            ~(3U << (pin * 2U));

        OSPEEDR(GPIOA_BASE) |=
            (3U << (pin * 2U));
    }


    /* No pull */

    for (uint32_t pin = 8U;
         pin <= 11U;
         pin++)
    {
        PUPDR(GPIOA_BASE) &=
            ~(3U << (pin * 2U));
    }


    /* Alternate function */

    for (uint32_t pin = 8U;
         pin <= 11U;
         pin++)
    {
        MODER(GPIOA_BASE) &=
            ~(3U << (pin * 2U));

        MODER(GPIOA_BASE) |=
            (2U << (pin * 2U));
    }


    /* AF2 */

    AFRH(GPIOA_BASE) &=
        ~0x0000FFFFU;


    AFRH(GPIOA_BASE) |=
        (2U << 0)  |
        (2U << 4)  |
        (2U << 8)  |
        (2U << 12);
}


/* =========================================================
   TIM1 PWM
   ========================================================= */

static void tim1_pwm_init(void)
{
    RCC_APB2ENR |= TIM1_EN;

    (void)RCC_APB2ENR;


    /* Reset TIM1 */

    RCC_APB2RSTR |= TIM1_EN;

    RCC_APB2RSTR &= ~TIM1_EN;


    TIM1_CR1 = 0U;
    TIM1_CR2 = 0U;
    TIM1_SMCR = 0U;
    TIM1_DIER = 0U;

    TIM1_CCER = 0U;
    TIM1_BDTR = 0U;


    /* 1 kHz */

    TIM1_PSC = 0U;

    TIM1_ARR =
        PWM_ARR_VALUE;

    TIM1_RCR = 0U;

    TIM1_CNT = 0U;


    /* ALL MOTORS OFF */

    TIM1_CCR1 = 0U;
    TIM1_CCR2 = 0U;
    TIM1_CCR3 = 0U;
    TIM1_CCR4 = 0U;


    /* CH1 + CH2 */

    TIM1_CCMR1 =
        (6U << 4)  |
        (1U << 3)  |
        (6U << 12) |
        (1U << 11);


    /* CH3 + CH4 */

    TIM1_CCMR2 =
        (6U << 4)  |
        (1U << 3)  |
        (6U << 12) |
        (1U << 11);


    /* Enable all four outputs */

    TIM1_CCER =
        (1U << 0)  |
        (1U << 4)  |
        (1U << 8)  |
        (1U << 12);


    /* Main Output Enable */

    TIM1_BDTR =
        (1U << 15);


    /* ARR preload */

    TIM1_CR1 |=
        (1U << 7);


    /* Update */

    TIM1_EGR =
        (1U << 0);

    TIM1_SR = 0U;


    motor_gpio_init();


    /* Start TIM1 */

    TIM1_CR1 |=
        (1U << 0);
}


/* =========================================================
   Motor set
   ========================================================= */

static void motor_set(
    uint32_t motor,
    uint32_t percent)
{
    if (percent > 100U)
    {
        percent = 100U;
    }


    uint32_t pwm =
        (PWM_PERIOD * percent) / 100U;


    switch (motor)
    {
        /* M1 = PA11 = CH4 */

        case MOTOR_M1:

            TIM1_CCR4 = pwm;
            motor1_percent = percent;

            break;


        /* M2 = PA9 = CH2 */

        case MOTOR_M2:

            TIM1_CCR2 = pwm;
            motor2_percent = percent;

            break;


        /* M3 = PA8 = CH1 */

        case MOTOR_M3:

            TIM1_CCR1 = pwm;
            motor3_percent = percent;

            break;


        /* M4 = PA10 = CH3 */

        case MOTOR_M4:

            TIM1_CCR3 = pwm;
            motor4_percent = percent;

            break;


        default:

            break;
    }
}


/* =========================================================
   Motors OFF
   ========================================================= */

static void motor_all_off(void)
{
    motor_set(
        MOTOR_M1,
        0U);

    motor_set(
        MOTOR_M2,
        0U);

    motor_set(
        MOTOR_M3,
        0U);

    motor_set(
        MOTOR_M4,
        0U);
}


/* =========================================================
   Integer square root

   Avoids large math library on our small STM32F031K4.
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

   Returns angle in degrees.

   This avoids linking the full math library.
   Accuracy is sufficient for our current bring-up.
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


    /*
       Prevent 0 / 0.
    */

    abs_y += 0.000001f;


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
   Gyroscope calibration

   Drone MUST remain completely still.

   500 samples x 5 ms
   ~2.5 seconds
   ========================================================= */

static uint32_t gyro_calibrate(void)
{
    int32_t sum_x = 0;
    int32_t sum_y = 0;
    int32_t sum_z = 0;


    uint32_t valid = 0;


    calibration_status = 1U;

    calibration_samples = 0U;


    for (uint32_t i = 0;
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


        delay_ms(5);
    }


    /*
       Require at least 450 valid samples.
    */

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
   Calculate attitude
   ========================================================= */

static void attitude_update(float dt)
{
    /* =====================================================
       Convert accelerometer to g
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
       Correct gyro bias

       Confirmed physical mapping:

       raw X = ROLL
       raw Y = PITCH
       raw Z = YAW
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
       Convert gyro to degrees per second
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
       Accelerometer Roll

       Level:
           Y ~= 0
           Z ~= +2048
           roll ~= 0

       Right side down:
           Y positive
           roll positive
       ===================================================== */

    accel_roll_deg =
        fast_atan2_deg(
            (float)accel_y,
            (float)accel_z);


    /* =====================================================
       Accelerometer Pitch

       Need sqrt(Y^2 + Z^2).

       Scale values by 2 first to prevent overflow.
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


    /*
       Nose down produced raw X negative in our test.

       Therefore -X gives positive pitch for nose-down.
    */

    accel_pitch_deg =
        fast_atan2_deg(
            -(float)accel_x,
            yz_length);


    /* =====================================================
       First frame

       Initialize filter directly from accelerometer.
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
       Integrate gyro
       ===================================================== */

    float gyro_roll_angle =
        roll_angle +
        gyro_roll_dps * dt;


    float gyro_pitch_angle =
        pitch_angle +
        gyro_pitch_dps * dt;


    /* =====================================================
       Complementary filter

       Gyro:
           fast / smooth motion

       Accelerometer:
           long-term reference to gravity
       ===================================================== */

    roll_angle =
        FILTER_GYRO_WEIGHT *
        gyro_roll_angle +

        FILTER_ACCEL_WEIGHT *
        accel_roll_deg;


    pitch_angle =
        FILTER_GYRO_WEIGHT *
        gyro_pitch_angle +

        FILTER_ACCEL_WEIGHT *
        accel_pitch_deg;


    /* =====================================================
       Yaw

       No magnetometer exists in our current system,
       therefore yaw can only be integrated from gyro.

       It WILL slowly drift. That is normal.
       ===================================================== */

    yaw_angle +=
        gyro_yaw_dps * dt;


    if (yaw_angle > 180.0f)
    {
        yaw_angle -= 360.0f;
    }


    if (yaw_angle < -180.0f)
    {
        yaw_angle += 360.0f;
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
       Board power / enable

       PA1
       ===================================================== */

    gpio_high(
        GPIOA_BASE,
        1U);


    gpio_output(
        GPIOA_BASE,
        1U);


    /* =====================================================
       Before PWM takes control:
       motor pins LOW
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
       1 ms system timer
       ===================================================== */

    systick_init();


    delay_ms(100);


    /* =====================================================
       Debug safety

       Freeze TIM1 when debugger halts CPU.
       ===================================================== */

    RCC_APB2ENR |=
        DBGMCU_EN;


    DBGMCU_APB2_FZ |=
        DBG_TIM1_STOP;


    /* =====================================================
       Motor PWM
       ===================================================== */

    tim1_pwm_init();


    /*
       IMPORTANT:
       motors remain OFF
    */

    motor_all_off();


    /* =====================================================
       Software I2C
       ===================================================== */

    i2c_init();


    /* =====================================================
       Initialize M540
       ===================================================== */

    imu_ok =
        m540_init();


    if (!imu_ok)
    {
        /*
           IMU failed.

           Motors remain permanently OFF.
        */

        calibration_status = 3U;


        while (1)
        {
            motor_all_off();
        }
    }


    /* =====================================================
       GYRO CALIBRATION

       DO NOT MOVE THE DRONE HERE.
       ===================================================== */

    if (!gyro_calibrate())
    {
        while (1)
        {
            motor_all_off();
        }
    }


    /* =====================================================
       Flight loop timing
       ===================================================== */

    uint32_t last_loop_ms =
        system_ms;


    /* =====================================================
       MAIN FLIGHT CONTROLLER LOOP
       ===================================================== */

    while (1)
    {
        uint32_t now =
            system_ms;


        uint32_t elapsed =
            (uint32_t)(
                now - last_loop_ms);


        /*
           Run attitude loop approximately 100 Hz.
        */

        if (elapsed >= 10U)
        {
            last_loop_ms =
                now;


            loop_dt =
                (float)elapsed *
                0.001f;


            /* =============================================
               Read IMU
               ============================================= */

            imu_read_ok =
                m540_read_raw();


            if (imu_read_ok)
            {
                imu_frame_counter++;


                /* =========================================
                   Calculate attitude
                   ========================================= */

                attitude_update(
                    loop_dt);


                flight_loop_counter++;
            }


            /* =============================================
               SAFETY

               NO MOTOR MOVEMENT IN THIS FIRMWARE.
               ============================================= */

            motor_all_off();
        }
    }
}
