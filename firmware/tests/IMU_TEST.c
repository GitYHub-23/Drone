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

#define MODER(base)  REG32((base) + 0x00U)
#define OTYPER(base) REG32((base) + 0x04U)
#define PUPDR(base)  REG32((base) + 0x0CU)
#define IDR(base)    REG32((base) + 0x10U)
#define BSRR(base)   REG32((base) + 0x18U)

/* =========================================================
   SysTick
   ========================================================= */

#define SYST_CSR (*(volatile uint32_t *)0xE000E010U)
#define SYST_RVR (*(volatile uint32_t *)0xE000E014U)
#define SYST_CVR (*(volatile uint32_t *)0xE000E018U)

/* =========================================================
   Motors
   ========================================================= */

#define M1_PIN 11U
#define M2_PIN 9U
#define M3_PIN 8U
#define M4_PIN 10U

/* =========================================================
   M540 - confirmed
   ========================================================= */

#define M540_ADDRESS 0x69U

#define M540_SCL 8U      /* PB8 */
#define M540_SDA 7U      /* PB7 */

#define WHO_AM_I 0x75U

/* MPU-like data registers */
#define ACCEL_X_H 0x3BU
#define ACCEL_Y_H 0x3DU
#define ACCEL_Z_H 0x3FU

#define GYRO_X_H  0x43U
#define GYRO_Y_H  0x45U
#define GYRO_Z_H  0x47U

/* =========================================================
   Delay
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
        while ((SYST_CSR & (1U << 16)) == 0)
        {
        }
    }
}

static void i2c_delay(void)
{
    for (volatile uint32_t i = 0; i < 150U; i++)
    {
    }
}

/* =========================================================
   GPIO A
   ========================================================= */

static void gpioa_output(uint32_t pin)
{
    MODER(GPIOA_BASE) &= ~(3U << (pin * 2U));
    MODER(GPIOA_BASE) |=  (1U << (pin * 2U));
}

static void gpioa_high(uint32_t pin)
{
    BSRR(GPIOA_BASE) = 1U << pin;
}

static void gpioa_low(uint32_t pin)
{
    BSRR(GPIOA_BASE) = 1U << (pin + 16U);
}

/* =========================================================
   GPIO B / I2C
   ========================================================= */

static void gpiob_open_drain(uint32_t pin)
{
    BSRR(GPIOB_BASE) = 1U << pin;

    OTYPER(GPIOB_BASE) |= 1U << pin;

    PUPDR(GPIOB_BASE) &= ~(3U << (pin * 2U));
    PUPDR(GPIOB_BASE) |=  (1U << (pin * 2U));

    MODER(GPIOB_BASE) &= ~(3U << (pin * 2U));
    MODER(GPIOB_BASE) |=  (1U << (pin * 2U));
}

static void gpiob_high(uint32_t pin)
{
    BSRR(GPIOB_BASE) = 1U << pin;
}

static void gpiob_low(uint32_t pin)
{
    BSRR(GPIOB_BASE) = 1U << (pin + 16U);
}

static uint32_t gpiob_read(uint32_t pin)
{
    return (IDR(GPIOB_BASE) >> pin) & 1U;
}

/* =========================================================
   I2C
   ========================================================= */

static void scl_high(void)
{
    gpiob_high(M540_SCL);
}

static void scl_low(void)
{
    gpiob_low(M540_SCL);
}

static void sda_high(void)
{
    gpiob_high(M540_SDA);
}

static void sda_low(void)
{
    gpiob_low(M540_SDA);
}

static uint32_t sda_read(void)
{
    return gpiob_read(M540_SDA);
}

static void i2c_start(void)
{
    sda_high();
    scl_high();
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

    scl_high();
    i2c_delay();

    sda_high();
    i2c_delay();
}

static uint32_t i2c_write_byte(uint8_t value)
{
    for (uint32_t i = 0; i < 8; i++)
    {
        if (value & 0x80U)
            sda_high();
        else
            sda_low();

        i2c_delay();

        scl_high();
        i2c_delay();

        scl_low();
        i2c_delay();

        value <<= 1;
    }

    /* ACK */
    sda_high();
    i2c_delay();

    scl_high();
    i2c_delay();

    uint32_t ack =
        (sda_read() == 0U);

    scl_low();
    i2c_delay();

    return ack;
}

static uint8_t i2c_read_byte(uint32_t ack)
{
    uint8_t value = 0;

    sda_high();

    for (uint32_t i = 0; i < 8; i++)
    {
        value <<= 1;

        scl_high();
        i2c_delay();

        if (sda_read())
            value |= 1U;

        scl_low();
        i2c_delay();
    }

    if (ack)
        sda_low();
    else
        sda_high();

    i2c_delay();

    scl_high();
    i2c_delay();

    scl_low();
    i2c_delay();

    sda_high();

    return value;
}

/* =========================================================
   Register read
   ========================================================= */

static uint32_t m540_read_register(
    uint8_t reg,
    uint8_t *value)
{
    i2c_start();

    /* WRITE address */
    if (!i2c_write_byte(
        (uint8_t)(M540_ADDRESS << 1)))
    {
        i2c_stop();
        return 0;
    }

    /* Register */
    if (!i2c_write_byte(reg))
    {
        i2c_stop();
        return 0;
    }

    /* Repeated START */
    i2c_start();

    /* READ address */
    if (!i2c_write_byte(
        (uint8_t)((M540_ADDRESS << 1) | 1U)))
    {
        i2c_stop();
        return 0;
    }

    *value = i2c_read_byte(0);

    i2c_stop();

    return 1;
}

/* =========================================================
   Read signed 16-bit value
   ========================================================= */

static uint32_t m540_read_i16(
    uint8_t high_register,
    int32_t *result)
{
    uint8_t high;
    uint8_t low;

    if (!m540_read_register(
        high_register, &high))
        return 0;

    if (!m540_read_register(
        high_register + 1U, &low))
        return 0;

    int16_t value =
        (int16_t)(
            ((uint16_t)high << 8) |
             (uint16_t)low);

    *result = (int32_t)value;

    return 1;
}

/* =========================================================
   Motor signals
   ========================================================= */

static void motor_pulse(uint32_t pin)
{
    for (uint32_t i = 0; i < 20; i++)
    {
        gpioa_high(pin);
        delay_ms(2);

        gpioa_low(pin);
        delay_ms(8);
    }

    gpioa_low(pin);
}

static void motor_signal(
    uint32_t pin,
    uint32_t count)
{
    for (uint32_t i = 0; i < count; i++)
    {
        motor_pulse(pin);
        delay_ms(300);
    }
}

/* =========================================================
   MAIN
   ========================================================= */

int main(void)
{
    uint8_t who = 0;

    RCC_AHBENR |= GPIOA_EN | GPIOB_EN;

    /* Board power */
    gpioa_high(1U);
    gpioa_output(1U);

    /* Motors OFF */
    gpioa_low(M1_PIN);
    gpioa_low(M2_PIN);
    gpioa_low(M3_PIN);
    gpioa_low(M4_PIN);

    gpioa_output(M1_PIN);
    gpioa_output(M2_PIN);
    gpioa_output(M3_PIN);
    gpioa_output(M4_PIN);

    /* M540 bus */
    gpiob_open_drain(M540_SCL);
    gpiob_open_drain(M540_SDA);

    scl_high();
    sda_high();

    systick_init();

    /*
       Put board on table and leave it still.
    */
    delay_ms(2000);

    /* Make sure sensor still responds */
    if (!m540_read_register(WHO_AM_I, &who))
    {
        motor_signal(M4_PIN, 3);

        while (1) {}
    }

    /*
       Previous test returned 0x7D.
       If it suddenly changed, signal error.
    */
    if (who != 0x7DU)
    {
        motor_signal(M4_PIN, 3);

        while (1) {}
    }

    /*
       Initial values.
    */
    int32_t ax, ay, az;
    int32_t gx, gy, gz;

    if (!m540_read_i16(ACCEL_X_H, &ax) ||
        !m540_read_i16(ACCEL_Y_H, &ay) ||
        !m540_read_i16(ACCEL_Z_H, &az) ||
        !m540_read_i16(GYRO_X_H,  &gx) ||
        !m540_read_i16(GYRO_Y_H,  &gy) ||
        !m540_read_i16(GYRO_Z_H,  &gz))
    {
        motor_signal(M4_PIN, 3);

        while (1) {}
    }

    /*
       Start with current values as min/max.
    */
    int32_t min_ax = ax, max_ax = ax;
    int32_t min_ay = ay, max_ay = ay;
    int32_t min_az = az, max_az = az;

    int32_t min_gx = gx, max_gx = gx;
    int32_t min_gy = gy, max_gy = gy;
    int32_t min_gz = gz, max_gz = gz;

    /*
       M4 x1 =
       START MOVING THE BOARD NOW.
    */
    motor_signal(M4_PIN, 1);

    /*
       About 4 seconds.

       During this time:
       tilt it forward/back,
       left/right,
       and rotate it.
    */
    for (uint32_t sample = 0;
         sample < 400U;
         sample++)
    {
        if (!m540_read_i16(ACCEL_X_H, &ax) ||
            !m540_read_i16(ACCEL_Y_H, &ay) ||
            !m540_read_i16(ACCEL_Z_H, &az) ||
            !m540_read_i16(GYRO_X_H,  &gx) ||
            !m540_read_i16(GYRO_Y_H,  &gy) ||
            !m540_read_i16(GYRO_Z_H,  &gz))
        {
            motor_signal(M4_PIN, 3);

            while (1) {}
        }

        if (ax < min_ax) min_ax = ax;
        if (ax > max_ax) max_ax = ax;

        if (ay < min_ay) min_ay = ay;
        if (ay > max_ay) max_ay = ay;

        if (az < min_az) min_az = az;
        if (az > max_az) max_az = az;

        if (gx < min_gx) min_gx = gx;
        if (gx > max_gx) max_gx = gx;

        if (gy < min_gy) min_gy = gy;
        if (gy > max_gy) max_gy = gy;

        if (gz < min_gz) min_gz = gz;
        if (gz > max_gz) max_gz = gz;

        delay_ms(10);
    }

    /*
       Calculate how much every channel moved.
    */
    int32_t accel_range_x = max_ax - min_ax;
    int32_t accel_range_y = max_ay - min_ay;
    int32_t accel_range_z = max_az - min_az;

    int32_t gyro_range_x = max_gx - min_gx;
    int32_t gyro_range_y = max_gy - min_gy;
    int32_t gyro_range_z = max_gz - min_gz;

    /*
       Fairly conservative thresholds.
       Real movement should exceed these
       by a lot if these are the correct registers.
    */
    uint32_t accel_found =
        (accel_range_x > 500) ||
        (accel_range_y > 500) ||
        (accel_range_z > 500);

    uint32_t gyro_found =
        (gyro_range_x > 200) ||
        (gyro_range_y > 200) ||
        (gyro_range_z > 200);

    delay_ms(1000);

    /*
       RESULTS
    */

    if (accel_found && gyro_found)
    {
        /*
           ACCEL works:
           M3 x2

           then GYRO works:
           M2 x2
        */
        motor_signal(M3_PIN, 2);

        delay_ms(1000);

        motor_signal(M2_PIN, 2);
    }
    else if (accel_found)
    {
        /*
           Only accel block changes.
        */
        motor_signal(M3_PIN, 2);
    }
    else if (gyro_found)
    {
        /*
           Only gyro block changes.
        */
        motor_signal(M2_PIN, 2);
    }
    else
    {
        /*
           Registers 0x3B-0x48 do not
           behave like motion data.
        */
        motor_signal(M1_PIN, 3);
    }

    while (1)
    {
        gpioa_low(M1_PIN);
        gpioa_low(M2_PIN);
        gpioa_low(M3_PIN);
        gpioa_low(M4_PIN);
    }
}
