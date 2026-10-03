#include <stdint.h>

/* =========================================================
   RCC / GPIO
   ========================================================= */

#define RCC_AHBENR (*(volatile uint32_t *)0x40021014U)

#define GPIOA_EN (1U << 17)
#define GPIOB_EN (1U << 18)

#define GPIOA_BASE 0x48000000U
#define GPIOB_BASE 0x48000400U

#define REG32(addr) (*(volatile uint32_t *)(addr))

#define MODER(base) REG32((base) + 0x00U)
#define PUPDR(base) REG32((base) + 0x0CU)
#define IDR(base)   REG32((base) + 0x10U)
#define BSRR(base)  REG32((base) + 0x18U)

/* =========================================================
   SysTick
   ========================================================= */

#define SYST_CSR (*(volatile uint32_t *)0xE000E010U)
#define SYST_RVR (*(volatile uint32_t *)0xE000E014U)
#define SYST_CVR (*(volatile uint32_t *)0xE000E018U)

/* =========================================================
   Motors - CONFIRMED
   ========================================================= */

#define M1_PIN 11U
#define M2_PIN 9U
#define M3_PIN 8U
#define M4_PIN 10U

/* =========================================================
   M540 IMU - CONFIRMED
   ========================================================= */

#define IMU_PORT    GPIOB_BASE
#define IMU_SDA_PIN 7U
#define IMU_SCL_PIN 8U

#define M540_ADDR 0x69U

/* =========================================================
   Variables for STM32CubeIDE Live Expressions
   ========================================================= */

volatile uint8_t imu_whoami = 0;

volatile uint32_t imu_ok = 0;
volatile uint32_t imu_read_ok = 0;
volatile uint32_t imu_frame_counter = 0;

/* Accelerometer */

volatile int16_t accel_x = 0;
volatile int16_t accel_y = 0;
volatile int16_t accel_z = 0;

/* Gyroscope */

volatile int16_t gyro_x = 0;
volatile int16_t gyro_y = 0;
volatile int16_t gyro_z = 0;

/* Temperature */

volatile int16_t imu_temperature_raw = 0;

/* =========================================================
   Gyroscope peak recorder

   Used to determine which physical movement corresponds
   to raw gyro X / Y / Z.
   ========================================================= */

volatile int16_t gyro_x_min = 32767;
volatile int16_t gyro_x_max = -32768;

volatile int16_t gyro_y_min = 32767;
volatile int16_t gyro_y_max = -32768;

volatile int16_t gyro_z_min = 32767;
volatile int16_t gyro_z_max = -32768;

/* =========================================================
   SysTick
   ========================================================= */

static void systick_init(void)
{
    SYST_CSR = 0;
    SYST_RVR = 7999U;
    SYST_CVR = 0;

    SYST_CSR =
        (1U << 2) |
        (1U << 0);
}

static void delay_ms(uint32_t ms)
{
    while (ms--)
    {
        while ((SYST_CSR & (1U << 16)) == 0U)
        {
        }
    }
}

/* =========================================================
   GPIO
   ========================================================= */

static void gpio_output(uint32_t port, uint32_t pin)
{
    MODER(port) &= ~(3U << (pin * 2U));
    MODER(port) |=  (1U << (pin * 2U));
}

static void gpio_input_pullup(uint32_t port, uint32_t pin)
{
    MODER(port) &= ~(3U << (pin * 2U));

    PUPDR(port) &= ~(3U << (pin * 2U));
    PUPDR(port) |=  (1U << (pin * 2U));
}

static void gpio_high(uint32_t port, uint32_t pin)
{
    BSRR(port) = 1U << pin;
}

static void gpio_low(uint32_t port, uint32_t pin)
{
    BSRR(port) = 1U << (pin + 16U);
}

static uint32_t gpio_read(uint32_t port, uint32_t pin)
{
    return (IDR(port) >> pin) & 1U;
}

/* =========================================================
   Software I2C

   PB8 = SCL
   PB7 = SDA

   Open-drain emulation:

   LOW  = GPIO output driving LOW
   HIGH = GPIO input / released
   ========================================================= */

static void i2c_delay(void)
{
    for (volatile uint32_t i = 0; i < 40U; i++)
    {
    }
}

static void sda_low(void)
{
    gpio_low(IMU_PORT, IMU_SDA_PIN);
    gpio_output(IMU_PORT, IMU_SDA_PIN);
}

static void sda_release(void)
{
    gpio_input_pullup(IMU_PORT, IMU_SDA_PIN);
}

static void scl_low(void)
{
    gpio_low(IMU_PORT, IMU_SCL_PIN);
    gpio_output(IMU_PORT, IMU_SCL_PIN);
}

static void scl_release(void)
{
    gpio_input_pullup(IMU_PORT, IMU_SCL_PIN);
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
   Send one I2C byte

   return:
   1 = ACK
   0 = NO ACK
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

    /* ACK bit */

    sda_release();

    i2c_delay();

    scl_release();

    i2c_delay();

    uint32_t ack =
        (gpio_read(IMU_PORT, IMU_SDA_PIN) == 0U);

    scl_low();

    i2c_delay();

    return ack;
}

/* =========================================================
   Read one I2C byte

   send_ack = 1 -> ACK
   send_ack = 0 -> NACK
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

        if (gpio_read(IMU_PORT, IMU_SDA_PIN))
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
   M540 - write one register
   ========================================================= */

static uint32_t m540_write_reg(
    uint8_t reg,
    uint8_t value)
{
    i2c_start();

    /* M540 address + WRITE */

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
   M540 - read multiple registers
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

    /* Select starting register */

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

    /* M540 address + READ */

    if (!i2c_write_byte(
            (uint8_t)((M540_ADDR << 1) | 1U)))
    {
        i2c_stop();
        return 0U;
    }

    for (uint32_t i = 0; i < count; i++)
    {
        /*
           ACK every byte except the final byte.
        */

        data[i] =
            i2c_read_byte(
                i + 1U < count);
    }

    i2c_stop();

    return 1U;
}

/* =========================================================
   M540 - read one register
   ========================================================= */

static uint32_t m540_read_reg(
    uint8_t reg,
    uint8_t *value)
{
    return m540_read_regs(
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

    /* -----------------------------------------------------
       Reset
       ----------------------------------------------------- */

    if (!m540_write_reg(
            0x6BU,
            0x80U))
    {
        return 0U;
    }

    delay_ms(50);

    /* -----------------------------------------------------
       Wake sensor / clock
       ----------------------------------------------------- */

    if (!m540_write_reg(
            0x6BU,
            0x01U))
    {
        return 0U;
    }

    delay_ms(10);

    /* -----------------------------------------------------
       WHO_AM_I

       Confirmed on our board:
       register = 0x75
       value    = 0x7D
       ----------------------------------------------------- */

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

    /* -----------------------------------------------------
       Accelerometer +/-16g
       ----------------------------------------------------- */

    if (!m540_write_reg(
            0x1CU,
            0x18U))
    {
        return 0U;
    }

    /* -----------------------------------------------------
       Accelerometer filter
       ----------------------------------------------------- */

    if (!m540_write_reg(
            0x1DU,
            0x05U))
    {
        return 0U;
    }

    /* -----------------------------------------------------
       Gyroscope +/-2000 deg/s
       ----------------------------------------------------- */

    if (!m540_write_reg(
            0x1BU,
            0x18U))
    {
        return 0U;
    }

    /* -----------------------------------------------------
       Gyroscope filter
       ----------------------------------------------------- */

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
   Read complete M540 measurement

   0x3B-0x40 = accelerometer
   0x41-0x42 = temperature
   0x43-0x48 = gyroscope
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

    /* =====================================================
       Accelerometer
       ===================================================== */

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

    /* =====================================================
       Temperature
       ===================================================== */

    imu_temperature_raw =
        (int16_t)(
            ((uint16_t)data[6] << 8) |
             data[7]);

    /* =====================================================
       Gyroscope
       ===================================================== */

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

    /* =====================================================
       Record gyro MIN / MAX
       ===================================================== */

    if (gyro_x < gyro_x_min)
    {
        gyro_x_min = gyro_x;
    }

    if (gyro_x > gyro_x_max)
    {
        gyro_x_max = gyro_x;
    }

    if (gyro_y < gyro_y_min)
    {
        gyro_y_min = gyro_y;
    }

    if (gyro_y > gyro_y_max)
    {
        gyro_y_max = gyro_y;
    }

    if (gyro_z < gyro_z_min)
    {
        gyro_z_min = gyro_z;
    }

    if (gyro_z > gyro_z_max)
    {
        gyro_z_max = gyro_z;
    }

    return 1U;
}

/* =========================================================
   MAIN
   ========================================================= */

int main(void)
{
    /* =====================================================
       Enable GPIO clocks
       ===================================================== */

    RCC_AHBENR |=
        GPIOA_EN |
        GPIOB_EN;

    /* =====================================================
       Board power
       ===================================================== */

    gpio_high(
        GPIOA_BASE,
        1U);

    gpio_output(
        GPIOA_BASE,
        1U);

    /* =====================================================
       Motors OFF

       Motors must NOT run during this experiment.
       ===================================================== */

    gpio_low(GPIOA_BASE, M1_PIN);
    gpio_low(GPIOA_BASE, M2_PIN);
    gpio_low(GPIOA_BASE, M3_PIN);
    gpio_low(GPIOA_BASE, M4_PIN);

    gpio_output(GPIOA_BASE, M1_PIN);
    gpio_output(GPIOA_BASE, M2_PIN);
    gpio_output(GPIOA_BASE, M3_PIN);
    gpio_output(GPIOA_BASE, M4_PIN);

    /* =====================================================
       Time
       ===================================================== */

    systick_init();

    delay_ms(100);

    /* =====================================================
       I2C
       ===================================================== */

    i2c_init();

    /* =====================================================
       M540
       ===================================================== */

    imu_ok = m540_init();

    /* =====================================================
       Main loop
       ===================================================== */

    while (1)
    {
        if (imu_ok)
        {
            imu_read_ok =
                m540_read_raw();

            if (imu_read_ok)
            {
                imu_frame_counter++;
            }
        }
        else
        {
            /*
               Initialization failed.
               Retry once per second.
            */

            delay_ms(1000);

            imu_ok =
                m540_init();
        }

        /*
           Approximately 50 Hz for this test.
        */

        delay_ms(20);
    }
}
