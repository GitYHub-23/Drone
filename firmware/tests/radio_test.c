
/*
 * Drone_ Step 10
 * STM32F050K4 - Radio Link Diagnostics
 *
 * RADIO HW-725:
 * CSN  PA15
 * SCK  PB3
 * MISO PB4
 * MOSI PB5
 * CE   PB6
 * IRQ  not used
 *
 * IMU M540:
 * SDA PB7
 * SCL PB8
 * Address 0x69
 *
 * SAFETY:
 * PA8..PA11 permanently GPIO LOW.
 * TIM1 outputs disabled.
 * No ARM transition.
 * No motor control from radio.
 *
 * Experimental bench firmware.
 * Remove propellers and disconnect motors.
 */

#include <stdint.h>

#define R(a) (*(volatile uint32_t *)(uintptr_t)(a))

/* RCC */

#define RCC_CR       R(0x40021000U)
#define RCC_CFGR     R(0x40021004U)
#define RCC_APB2RSTR R(0x4002100CU)
#define RCC_APB1RSTR R(0x40021010U)
#define RCC_AHBENR   R(0x40021014U)
#define RCC_APB2ENR  R(0x40021018U)
#define RCC_APB1ENR  R(0x4002101CU)
#define RCC_CSR      R(0x40021024U)
#define RCC_CFGR3    R(0x40021030U)

/* GPIO */

#define GA 0x48000000U
#define GB 0x48000400U

#define MODE(b)  R((b)+0x00U)
#define TYPE(b)  R((b)+0x04U)
#define SPEED(b) R((b)+0x08U)
#define PUPD(b)  R((b)+0x0CU)
#define ODR(b)   R((b)+0x14U)
#define BSRR(b)  R((b)+0x18U)
#define AFRL(b)  R((b)+0x20U)
#define AFRH(b)  R((b)+0x24U)

/* SysTick */

#define SYST_CSR R(0xE000E010U)
#define SYST_RVR R(0xE000E014U)
#define SYST_CVR R(0xE000E018U)

/* Watchdog */

#define IWDG_KR   R(0x40003000U)
#define IWDG_PR   R(0x40003004U)
#define IWDG_RLR  R(0x40003008U)
#define IWDG_SR   R(0x4000300CU)
#define IWDG_WINR R(0x40003010U)

#define DBGMCU_APB1_FZ R(0x40015808U)

/* TIM1 */

#define TCR1  R(0x40012C00U)
#define TEGR  R(0x40012C14U)
#define TCCER R(0x40012C20U)
#define TCCR1 R(0x40012C34U)
#define TCCR2 R(0x40012C38U)
#define TCCR3 R(0x40012C3CU)
#define TCCR4 R(0x40012C40U)
#define TBDTR R(0x40012C44U)

/* I2C1 */

#define I2C_CR1    R(0x40005400U)
#define I2C_CR2    R(0x40005404U)
#define I2C_TIMING R(0x40005410U)
#define I2C_ISR    R(0x40005418U)
#define I2C_ICR    R(0x4000541CU)
#define I2C_RXDR   R(0x40005424U)
#define I2C_TXDR   R(0x40005428U)

/* SPI1 */

#define SP_CR1 R(0x40013000U)
#define SP_CR2 R(0x40013004U)
#define SP_SR  R(0x40013008U)

#define SP_DR8 \
 (*(volatile uint8_t *)(uintptr_t)0x4001300CU)

/* Configuration */

#define MOTOR_MASK (0xFU << 8U)
#define MOTOR_MODE_MASK (0xFFU << 16U)
#define MOTOR_OUTPUT_MODES (0x55U << 16U)

#define LOOP_MS 5U
#define RADIO_TIMEOUT_MS 250U
#define CALIBRATION_SAMPLES 500U

/* ====================================================
 * LIVE EXPRESSIONS
 * ==================================================== */

volatile uint32_t system_ms = 0;

volatile uint32_t fc_state = 0;
volatile uint32_t fault_code = 0;

volatile uint32_t physical_motor_lock = 1;
volatile uint32_t outputs_verified_off = 0;

volatile uint32_t motor1_percent = 0;
volatile uint32_t motor2_percent = 0;
volatile uint32_t motor3_percent = 0;
volatile uint32_t motor4_percent = 0;

/* IMU */

volatile uint32_t imu_ok = 0;
volatile uint32_t imu_whoami = 0;

volatile uint32_t imu_frame_counter = 0;
volatile uint32_t imu_error_counter = 0;

volatile int16_t accel_x = 0;
volatile int16_t accel_y = 0;
volatile int16_t accel_z = 0;

volatile int16_t gyro_x = 0;
volatile int16_t gyro_y = 0;
volatile int16_t gyro_z = 0;

volatile uint32_t calibration_status = 0;
volatile uint32_t calibration_samples = 0;

volatile float gyro_x_offset = 0;
volatile float gyro_y_offset = 0;
volatile float gyro_z_offset = 0;

volatile float gyro_roll_dps = 0;
volatile float gyro_pitch_dps = 0;
volatile float gyro_yaw_dps = 0;

/* Computation only */

volatile float pid_roll_output = 0;
volatile float pid_pitch_output = 0;
volatile float pid_yaw_output = 0;

volatile float mix_m1_percent = 0;
volatile float mix_m2_percent = 0;
volatile float mix_m3_percent = 0;
volatile float mix_m4_percent = 0;

/* Loop */

volatile uint32_t loop_counter = 0;
volatile uint32_t loop_dt_ms = 0;
volatile uint32_t loop_overrun_counter = 0;
volatile uint32_t loop_exec_ms = 0;
volatile uint32_t loop_max_exec_ms = 0;

/* Watchdog */

volatile uint32_t reset_cause = 0;
volatile uint32_t watchdog_reset_detected = 0;
volatile uint32_t iwdg_refresh_counter = 0;

/* ARM always blocked */

volatile uint32_t arm_request = 0;
volatile uint32_t arm_reject_counter = 0;
volatile uint32_t arm_block_flags = 1;

/* Radio SPI */

volatile uint32_t radio_spi_initialized = 0;
volatile uint32_t radio_spi_ok = 0;

volatile uint32_t radio_spi_error_counter = 0;
volatile uint32_t radio_spi_last_error = 0;

/* Radio RX */

volatile uint32_t radio_rx_configured = 0;
volatile uint32_t radio_rx_config_error = 0;

volatile uint32_t radio_rx_poll_counter = 0;
volatile uint32_t radio_rx_raw_packets = 0;
volatile uint32_t radio_rx_invalid_packets = 0;
volatile uint32_t radio_rx_read_errors = 0;

volatile uint32_t radio_packets_received = 0;

volatile uint32_t radio_rx_status = 0xFF;
volatile uint32_t radio_rx_fifo_status = 0xFF;

volatile uint32_t radio_rx_throttle_raw = 0xE1;
volatile uint32_t radio_rx_yaw_raw = 0x8D;
volatile uint32_t radio_rx_roll_raw = 0x70;
volatile uint32_t radio_rx_pitch_raw = 0x70;

volatile uint32_t radio_rx_last_packet_ms = 0;

volatile uint8_t radio_last_packet[9] = {0};

/* ====================================================
 * STEP 10 - RADIO LINK DIAGNOSTICS
 * ==================================================== */

volatile uint32_t radio_diag_alive = 0;
volatile uint32_t radio_diag_age_ms = UINT32_MAX;

volatile uint32_t radio_diag_lost_count = 0;
volatile uint32_t radio_diag_reconnect_count = 0;

volatile uint32_t radio_diag_seq_gap_count = 0;
volatile uint32_t radio_diag_period_ms = 0;

volatile uint32_t radio_diag_duplicate_count = 0;
volatile uint32_t radio_diag_last_seq = 0;

volatile uint32_t radio_diag_throttle = 0xE1;
volatile uint32_t radio_diag_yaw = 0x8D;
volatile uint32_t radio_diag_roll = 0x70;
volatile uint32_t radio_diag_pitch = 0x70;

/* Internal variables */

static uint32_t spi_last_ms = 0;
static uint32_t link_last_ms = 0;
static uint32_t link_ever_seen = 0;

static uint32_t seq_have = 0;
static uint32_t seq_previous = 0;
static uint32_t link_last_period_ms = 0;

static uint32_t i2c_begin_ms = 0;
static uint32_t calib_begin_ms = 0;

static int32_t gyro_sum[3] = {0, 0, 0};
static int16_t gyro_low[3];
static int16_t gyro_high[3];

/*
 * Internal state:
 * 0 BOOT
 * 1 CALIBRATING
 * 2 DISARMED
 * 4 FAILSAFE
 */

static uint32_t internal_state = 0;

/* ====================================================
 * MOTOR HARDWARE LOCK
 * ==================================================== */

static void motor_hardware_off(void)
{
    RCC_AHBENR |= (1U << 17);
    RCC_APB2ENR |= (1U << 11);

    (void)RCC_AHBENR;
    (void)RCC_APB2ENR;

    TBDTR = 0;
    TCCER = 0;
    TCR1 = 0;

    BSRR(GA) = (MOTOR_MASK << 16);

    TYPE(GA) &= ~MOTOR_MASK;

    PUPD(GA) =
        (PUPD(GA) & ~MOTOR_MODE_MASK) |
        (0xAAU << 16);

    MODE(GA) =
        (MODE(GA) & ~MOTOR_MODE_MASK) |
        MOTOR_OUTPUT_MODES;

    TCCR1 = 0;
    TCCR2 = 0;
    TCCR3 = 0;
    TCCR4 = 0;

    TEGR = 1;
}

void SystemInit(void)
{
    motor_hardware_off();
}

static uint32_t motor_off_verified(void)
{
    return (
        (MODE(GA) & MOTOR_MODE_MASK)
            == MOTOR_OUTPUT_MODES &&

        (ODR(GA) & MOTOR_MASK) == 0 &&

        TBDTR == 0 &&
        TCCER == 0 &&
        TCR1 == 0 &&

        TCCR1 == 0 &&
        TCCR2 == 0 &&
        TCCR3 == 0 &&
        TCCR4 == 0
    );
}

static void motor_all_off(void)
{
    motor_hardware_off();

    physical_motor_lock = 1;

    motor1_percent = 0;
    motor2_percent = 0;
    motor3_percent = 0;
    motor4_percent = 0;

    outputs_verified_off = motor_off_verified();
}

static void fail(uint32_t code)
{
    motor_all_off();

    if (fault_code == 0)
        fault_code = code;

    internal_state = 4;
    fc_state = 4;

    pid_roll_output = 0;
    pid_pitch_output = 0;
    pid_yaw_output = 0;

    mix_m1_percent = 0;
    mix_m2_percent = 0;
    mix_m3_percent = 0;
    mix_m4_percent = 0;
}

void HardFault_Handler(void)
{
    motor_hardware_off();

    for (;;)
    {
    }
}

void NMI_Handler(void)
{
    motor_hardware_off();

    for (;;)
    {
    }
}

/* ====================================================
 * CLOCK AND WATCHDOG
 * ==================================================== */

void SysTick_Handler(void)
{
    system_ms++;
}

static void delay_ms(uint32_t n)
{
    uint32_t t = system_ms;

    while ((uint32_t)(system_ms - t) < n)
    {
    }
}

static uint32_t clock_init(void)
{
    uint32_t budget = 200000;

    RCC_CR |= 1;

    while (!(RCC_CR & 2))
    {
        if (--budget == 0)
            return 0;
    }

    RCC_CFGR &= ~3U;

    budget = 200000;

    while (RCC_CFGR & (3U << 2))
    {
        if (--budget == 0)
            return 0;
    }

    RCC_CFGR &= ~(
        (0xFU << 4) |
        (7U << 8)
    );

    SYST_RVR = 7999;
    SYST_CVR = 0;
    SYST_CSR = 7;

    return 1;
}

static uint32_t watchdog_init(void)
{
    uint32_t t;

    RCC_APB2ENR |= (1U << 22);

    DBGMCU_APB1_FZ |= (1U << 12);

    IWDG_KR = 0xCCCC;
    IWDG_KR = 0x5555;

    IWDG_PR = 4;
    IWDG_RLR = 624;

    t = system_ms;

    while (IWDG_SR & 7U)
    {
        if ((uint32_t)(system_ms - t) > 100)
            return 0;
    }

    IWDG_WINR = 0xFFF;

    t = system_ms;

    while (IWDG_SR & 7U)
    {
        if ((uint32_t)(system_ms - t) > 100)
            return 0;
    }

    IWDG_KR = 0xAAAA;

    return 1;
}

static void watchdog_feed(void)
{
    if (motor_off_verified())
    {
        IWDG_KR = 0xAAAA;
        iwdg_refresh_counter++;
    }
}

/* ====================================================
 * M540 I2C
 * ==================================================== */

static void i2c_init(void)
{
    RCC_AHBENR |= (1U << 18);

    MODE(GB) =
        (MODE(GB) &
        ~((3U << 14) | (3U << 16))) |
        ((2U << 14) | (2U << 16));

    TYPE(GB) |=
        (1U << 7) |
        (1U << 8);

    SPEED(GB) |=
        (3U << 14) |
        (3U << 16);

    PUPD(GB) =
        (PUPD(GB) &
        ~((3U << 14) | (3U << 16))) |
        ((1U << 14) | (1U << 16));

    AFRL(GB) =
        (AFRL(GB) & ~(0xFU << 28)) |
        (1U << 28);

    AFRH(GB) =
        (AFRH(GB) & ~0xFU) |
        1U;

    RCC_CFGR3 &= ~(1U << 4);

    RCC_APB1ENR |= (1U << 21);

    RCC_APB1RSTR |= (1U << 21);
    RCC_APB1RSTR &= ~(1U << 21);

    I2C_CR1 = 0;
    I2C_TIMING = 0x10420F13;

    I2C_ICR =
        (1U << 4) |
        (1U << 5) |
        (1U << 8) |
        (1U << 9) |
        (1U << 10);

    I2C_CR1 = 1;
}

static uint32_t i2c_wait(uint32_t flag)
{
    for (;;)
    {
        uint32_t sr = I2C_ISR;

        if (sr & (1U << 8))
            I2C_ICR = (1U << 8);

        if (sr & (
            (1U << 4) |
            (1U << 9) |
            (1U << 10)))
        {
            return 0;
        }

        if (sr & flag)
            return 1;

        if ((uint32_t)(
            system_ms - i2c_begin_ms
        ) >= 4U)
        {
            return 0;
        }
    }
}

static uint32_t i2c_ready(void)
{
    while (I2C_ISR & (1U << 15))
    {
        if ((uint32_t)(
            system_ms - i2c_begin_ms
        ) >= 4U)
        {
            return 0;
        }
    }

    return 1;
}

static uint32_t imu_write(
    uint8_t reg,
    uint8_t val
)
{
    i2c_begin_ms = system_ms;

    if (!i2c_ready())
        return 0;

    I2C_ICR =
        (1U << 4) |
        (1U << 5) |
        (1U << 8) |
        (1U << 9) |
        (1U << 10);

    I2C_CR2 =
        (0x69U << 1) |
        (2U << 16) |
        (1U << 25) |
        (1U << 13);

    if (!i2c_wait(1U << 1))
        return 0;

    I2C_TXDR = reg;

    if (!i2c_wait(1U << 1))
        return 0;

    I2C_TXDR = val;

    if (!i2c_wait(1U << 5))
        return 0;

    I2C_ICR = (1U << 5);

    return 1;
}

static uint32_t imu_read(
    uint8_t reg,
    uint8_t *d,
    uint32_t n
)
{
    uint32_t i;

    if (n == 0 || n > 255)
        return 0;

    i2c_begin_ms = system_ms;

    if (!i2c_ready())
        return 0;

    I2C_ICR =
        (1U << 4) |
        (1U << 5) |
        (1U << 8) |
        (1U << 9) |
        (1U << 10);

    I2C_CR2 =
        (0x69U << 1) |
        (1U << 16) |
        (1U << 13);

    if (!i2c_wait(1U << 1))
        return 0;

    I2C_TXDR = reg;

    if (!i2c_wait(1U << 6))
        return 0;

    I2C_CR2 =
        (0x69U << 1) |
        (n << 16) |
        (1U << 10) |
        (1U << 25) |
        (1U << 13);

    for (i = 0; i < n; i++)
    {
        if (!i2c_wait(1U << 2))
            return 0;

        d[i] = (uint8_t)I2C_RXDR;
    }

    if (!i2c_wait(1U << 5))
        return 0;

    I2C_ICR = (1U << 5);

    return 1;
}

/* ====================================================
 * M540 INITIALIZATION
 * ==================================================== */

static uint32_t imu_init(void)
{
    uint8_t w = 0;

    if (!imu_write(0x6B, 0x80))
        return 0;

    delay_ms(100);

    if (!imu_write(0x6B, 0x01))
        return 0;

    delay_ms(20);

    if (!imu_write(0x1C, 0x18))
        return 0;

    if (!imu_write(0x1D, 0x05))
        return 0;

    if (!imu_write(0x1B, 0x18))
        return 0;

    if (!imu_write(0x1A, 0x00))
        return 0;

    delay_ms(20);

    if (!imu_read(0x75, &w, 1))
        return 0;

    imu_whoami = w;

    return w == 0x7D;
}

static uint32_t imu_sample(void)
{
    uint8_t d[14];

    if (!imu_read(0x3B, d, 14))
        return 0;

    accel_x = (int16_t)(
        ((uint16_t)d[0] << 8) | d[1]
    );

    accel_y = (int16_t)(
        ((uint16_t)d[2] << 8) | d[3]
    );

    accel_z = (int16_t)(
        ((uint16_t)d[4] << 8) | d[5]
    );

    gyro_x = (int16_t)(
        ((uint16_t)d[8] << 8) | d[9]
    );

    gyro_y = (int16_t)(
        ((uint16_t)d[10] << 8) | d[11]
    );

    gyro_z = (int16_t)(
        ((uint16_t)d[12] << 8) | d[13]
    );

    imu_frame_counter++;

    if ((
        accel_x |
        accel_y |
        accel_z |
        gyro_x |
        gyro_y |
        gyro_z
    ) == 0)
    {
        return 0;
    }

    if (
        gyro_x >= 32760 || gyro_x <= -32760 ||
        gyro_y >= 32760 || gyro_y <= -32760 ||
        gyro_z >= 32760 || gyro_z <= -32760
    )
    {
        return 0;
    }

    return 1;
}

/* ====================================================
 * GYRO CALIBRATION
 * ==================================================== */

static void calibrate_start(void)
{
    uint32_t i;

    gyro_sum[0] = 0;
    gyro_sum[1] = 0;
    gyro_sum[2] = 0;

    for (i = 0; i < 3; i++)
    {
        gyro_low[i] = INT16_MAX;
        gyro_high[i] = INT16_MIN;
    }

    calibration_samples = 0;
    calibration_status = 1;

    calib_begin_ms = system_ms;

    internal_state = 1;
    fc_state = 1;
}

static void calibrate_sample(void)
{
    int16_t v[3] = {
        gyro_x,
        gyro_y,
        gyro_z
    };

    uint32_t i;

    if ((uint32_t)(
        system_ms - calib_begin_ms
    ) > 4000U)
    {
        calibration_status = 3;
        fail(2);
        return;
    }

    for (i = 0; i < 3; i++)
    {
        if (v[i] < gyro_low[i])
            gyro_low[i] = v[i];

        if (v[i] > gyro_high[i])
            gyro_high[i] = v[i];

        if (
            (int32_t)gyro_high[i] -
            (int32_t)gyro_low[i] > 164 ||

            v[i] > 1312 ||
            v[i] < -1312
        )
        {
            calibration_status = 3;
            fail(2);
            return;
        }

        gyro_sum[i] += v[i];
    }

    calibration_samples++;

    if (calibration_samples ==
        CALIBRATION_SAMPLES)
    {
        gyro_x_offset =
            (float)gyro_sum[0] /
            CALIBRATION_SAMPLES;

        gyro_y_offset =
            (float)gyro_sum[1] /
            CALIBRATION_SAMPLES;

        gyro_z_offset =
            (float)gyro_sum[2] /
            CALIBRATION_SAMPLES;

        calibration_status = 2;

        internal_state = 2;
        fc_state = 2;
    }
}

/* ====================================================
 * COMPUTATION-ONLY PID AND MIXER
 * ==================================================== */

static float limit(
    float v,
    float lo,
    float hi
)
{
    return v < lo ?
        lo : (v > hi ? hi : v);
}

static void compute_only(void)
{
    float r, p, y;

    gyro_roll_dps =
        ((float)gyro_x - gyro_x_offset) /
        16.4f;

    gyro_pitch_dps =
        ((float)gyro_y - gyro_y_offset) /
        16.4f;

    gyro_yaw_dps =
        ((float)gyro_z - gyro_z_offset) /
        16.4f;

    pid_roll_output = limit(
        -0.04f * gyro_roll_dps, -8, 8
    );

    pid_pitch_output = limit(
        -0.04f * gyro_pitch_dps, -8, 8
    );

    pid_yaw_output = limit(
        -0.03f * gyro_yaw_dps, -6, 6
    );

    r = pid_roll_output;
    p = pid_pitch_output;
    y = pid_yaw_output;

    mix_m1_percent = limit(
        35 + r + p + y, 20, 50
    );

    mix_m2_percent = limit(
        35 - r + p - y, 20, 50
    );

    mix_m3_percent = limit(
        35 - r - p + y, 20, 50
    );

    mix_m4_percent = limit(
        35 + r - p - y, 20, 50
    );

    /* No motor PWM writes. */
}

/* ====================================================
 * SPI INITIALIZATION
 * ==================================================== */

static void spi_init(void)
{
    RCC_AHBENR |=
        (1U << 17) |
        (1U << 18);

    RCC_APB2ENR |= (1U << 12);

    /* CSN high */

    BSRR(GA) = (1U << 15);

    /* CE low */

    BSRR(GB) = (1U << (6 + 16));

    TYPE(GA) &= ~(1U << 15);

    TYPE(GB) &= ~(
        (1U << 3) |
        (1U << 5) |
        (1U << 6)
    );

    MODE(GA) =
        (MODE(GA) & ~(3U << 30)) |
        (1U << 30);

    PUPD(GA) &= ~(3U << 30);

    MODE(GB) =
        (MODE(GB) & ~(
            (3U << 6) |
            (3U << 8) |
            (3U << 10) |
            (3U << 12)
        )) |
        (
            (2U << 6) |
            (2U << 8) |
            (2U << 10) |
            (1U << 12)
        );

    /* AF0 = SPI1 */

    AFRL(GB) &= ~(
        (0xFU << 12) |
        (0xFU << 16) |
        (0xFU << 20)
    );

    PUPD(GB) &= ~(
        (3U << 6) |
        (3U << 8) |
        (3U << 10) |
        (3U << 12)
    );

    SPEED(GB) |=
        (3U << 6) |
        (3U << 10);

    SP_CR1 = 0;

    SP_CR2 =
        (7U << 8) |
        (1U << 12);

    SP_CR1 =
        (1U << 2) |
        (4U << 3) |
        (1U << 8) |
        (1U << 9) |
        (1U << 6);

    radio_spi_initialized = 1;
}

/* ====================================================
 * SPI TRANSFER FUNCTIONS
 * ==================================================== */

static uint32_t spi_wait(
    uint32_t mask,
    uint32_t set,
    uint32_t code
)
{
    uint32_t t = system_ms;

    for (;;)
    {
        if (
            ((SP_SR & mask) != 0U) ==
            (set != 0U)
        )
        {
            return 1;
        }

        if ((uint32_t)(
            system_ms - t
        ) >= 2U)
        {
            radio_spi_last_error = code;
            return 0;
        }
    }
}

static uint32_t spi_byte(
    uint8_t tx,
    uint8_t *rx
)
{
    if (!spi_wait(2U, 1U, 1U))
        return 0;

    SP_DR8 = tx;

    if (!spi_wait(1U, 1U, 2U))
        return 0;

    *rx = SP_DR8;

    return 1;
}

static uint32_t spi_xfer(
    const uint8_t *tx,
    uint8_t *rx,
    uint32_t n
)
{
    uint32_t i;
    uint32_t ok = 1;

    if (n == 0 || n > 11)
        return 0;

    /* CSN low */

    BSRR(GA) = (1U << 31);

    for (i = 0; i < n; i++)
    {
        uint8_t b = 0;

        if (!spi_byte(tx[i], &b))
        {
            ok = 0;
            break;
        }

        if (rx)
            rx[i] = b;
    }

    if (ok)
        ok = spi_wait(1U << 7, 0U, 3U);

    /* CSN high even after error */

    BSRR(GA) = (1U << 15);

    if (SP_SR & (1U << 6))
    {
        volatile uint8_t d = SP_DR8;
        volatile uint32_t s = SP_SR;

        (void)d;
        (void)s;

        radio_spi_last_error = 4;
        ok = 0;
    }

    if (!ok)
        radio_spi_error_counter++;

    return ok;
}

static uint32_t write_reg(
    uint8_t reg,
    const uint8_t *data,
    uint32_t n
)
{
    uint8_t tx[11];
    uint32_t i;

    if (n == 0 || n > 10)
        return 0;

    tx[0] = 0x20U | (reg & 0x1FU);

    for (i = 0; i < n; i++)
        tx[1 + i] = data[i];

    return spi_xfer(tx, 0, n + 1);
}

static uint32_t read_reg(
    uint8_t reg,
    uint8_t *val
)
{
    uint8_t tx[2] = {
        (uint8_t)(reg & 0x1FU),
        0xFFU
    };

    uint8_t rx[2];

    if (!spi_xfer(tx, rx, 2))
        return 0;

    *val = rx[1];

    return 1;
}

static uint32_t cmd(uint8_t op)
{
    return spi_xfer(&op, 0, 1);
}

static uint32_t wr1(
    uint8_t reg,
    uint8_t val
)
{
    return write_reg(reg, &val, 1);
}

/* ====================================================
 * NRF24L01+ RECEIVER CONFIGURATION
 * ==================================================== */

static void rx_init(void)
{
    static const uint8_t reg[] = {
        0x00,
        0x01,
        0x02,
        0x03,
        0x04,
        0x05,
        0x06,
        0x11,
        0x1C,
        0x1D
    };

    static const uint8_t val[] = {
        0x0C,
        0x00,
        0x01,
        0x03,
        0x00,
        0x2D,
        0x07,
        0x09,
        0x00,
        0x00
    };

    const uint8_t addr[5] = {
        0xCC,
        0xCC,
        0xCC,
        0xCC,
        0xCC
    };

    uint32_t i;
    uint8_t got;

    radio_rx_configured = 0;
    radio_rx_config_error = 0;

    /* CE low */

    BSRR(GB) = (1U << (6 + 16));

    for (i = 0; i < sizeof(reg); i++)
    {
        if (
            !wr1(reg[i], val[i]) ||
            !read_reg(reg[i], &got) ||
            got != val[i]
        )
        {
            radio_rx_config_error =
                0x100U | reg[i];

            return;
        }
    }

    /* RX_ADDR_P0 */

    if (!write_reg(0x0A, addr, 5))
    {
        radio_rx_config_error = 0x10A;
        return;
    }

    /* TX_ADDR */

    if (!write_reg(0x10, addr, 5))
    {
        radio_rx_config_error = 0x110;
        return;
    }

    /* FLUSH_RX */

    if (!cmd(0xE2))
    {
        radio_rx_config_error = 0x1E2;
        return;
    }

    /* Clear interrupt flags */

    if (!wr1(7, 0x70))
    {
        radio_rx_config_error = 0x107;
        return;
    }

    /* CONFIG: PWR_UP + PRIM_RX + CRC */

    if (
        !wr1(0, 0x0F) ||
        !read_reg(0, &got) ||
        got != 0x0F
    )
    {
        radio_rx_config_error = 0x200;
        return;
    }

    delay_ms(2);

    /* CE high: start listening */

    BSRR(GB) = (1U << 6);

    radio_rx_configured = 1;
    radio_spi_ok = 1;
}

/* ====================================================
 * STEP 10: VALIDATED PACKET PROCESSING
 * ==================================================== */

static void diag_received(
    const uint8_t *b,
    uint32_t now
)
{
    uint32_t i;
    uint32_t sum = 0x76U;

    uint32_t seq = b[7];
    uint32_t delta;

    /* Verify application checksum */

    for (i = 0; i < 8; i++)
        sum += b[i];

    if (((uint8_t)sum) != b[8])
    {
        radio_rx_invalid_packets++;
        return;
    }

    radio_packets_received++;

    radio_rx_throttle_raw = b[0];
    radio_rx_yaw_raw = b[1];
    radio_rx_roll_raw = b[2];
    radio_rx_pitch_raw = b[3];

    /* Update link diagnostics */

    if (radio_diag_alive)
    {
        radio_diag_period_ms =
            (uint32_t)(
                now - link_last_period_ms
            );

        if (seq_have)
        {
            delta = (uint8_t)(
                seq - (uint8_t)seq_previous
            );

            if (delta == 0)
            {
                radio_diag_duplicate_count++;
            }
            else if (delta > 1)
            {
                radio_diag_seq_gap_count +=
                    (delta - 1U);
            }
        }
    }
    else
    {
        if (link_ever_seen)
            radio_diag_reconnect_count++;

        seq_have = 0;
        radio_diag_period_ms = 0;
    }

    seq_previous = seq;
    seq_have = 1;

    radio_diag_last_seq = seq;

    link_last_period_ms = now;
    link_last_ms = now;

    link_ever_seen = 1;

    radio_diag_alive = 1;
    radio_diag_age_ms = 0;

    /* Display values only */

    radio_diag_throttle = b[0];
    radio_diag_yaw = b[1];
    radio_diag_roll = b[2];
    radio_diag_pitch = b[3];
}

/* ====================================================
 * RADIO POLLING
 * ==================================================== */

static void radio_poll(uint32_t now)
{
    uint8_t st = 0;
    uint8_t fifo = 0;

    uint8_t b[9];
    uint8_t tx[10];
    uint8_t rx[10];

    uint32_t i;

    if (!radio_rx_configured)
        return;

    if ((uint32_t)(
        now - spi_last_ms
    ) < 10U)
    {
        return;
    }

    spi_last_ms = now;

    radio_rx_poll_counter++;

    if (
        !read_reg(7, &st) ||
        !read_reg(0x17, &fifo)
    )
    {
        radio_rx_read_errors++;
        return;
    }

    radio_rx_status = st;
    radio_rx_fifo_status = fifo;

    /* RX_EMPTY */

    if (fifo & 1U)
        return;

    /* R_RX_PAYLOAD */

    tx[0] = 0x61;

    for (i = 1; i < 10; i++)
        tx[i] = 0xFF;

    if (!spi_xfer(tx, rx, 10))
    {
        radio_rx_read_errors++;
        return;
    }

    for (i = 0; i < 9; i++)
    {
        b[i] = rx[i + 1];
        radio_last_packet[i] = b[i];
    }

    radio_rx_raw_packets++;
    radio_rx_last_packet_ms = now;

    diag_received(b, now);

    /* Clear RX_DR */

    if (!wr1(7, 0x40))
        radio_rx_read_errors++;
}

/* ====================================================
 * LINK LOSS AND RECONNECT
 * ==================================================== */

static void radio_link_update(uint32_t now)
{
    if (!link_ever_seen)
    {
        radio_diag_age_ms = UINT32_MAX;
        return;
    }

    radio_diag_age_ms =
        (uint32_t)(now - link_last_ms);

    if (
        radio_diag_alive &&
        radio_diag_age_ms >= RADIO_TIMEOUT_MS
    )
    {
        radio_diag_alive = 0;
        radio_diag_lost_count++;

        seq_have = 0;

        /* Safe diagnostic defaults */

        radio_diag_throttle = 0xE1;
        radio_diag_yaw = 0x8D;
        radio_diag_roll = 0x70;
        radio_diag_pitch = 0x70;

        /*
         * These variables NEVER control motors.
         */
    }
}

/* ====================================================
 * MAIN CONTROL CYCLE
 * ==================================================== */

static void control_cycle(uint32_t elapsed)
{
    uint32_t state_mirror = fc_state;

    if (
        !motor_off_verified() ||
        !outputs_verified_off
    )
    {
        fail(5);
    }

    motor_all_off();

    loop_counter++;
    loop_dt_ms = elapsed;

    if (elapsed > 6U)
    {
        loop_overrun_counter++;
        fail(4);
    }

    if (state_mirror != internal_state)
        fail(99);

    /* ARM is always rejected */

    if (arm_request)
    {
        arm_request = 0;
        arm_reject_counter++;
        arm_block_flags = 1;
    }

    if (internal_state == 4)
        return;

    if (
        internal_state != 1 &&
        internal_state != 2
    )
    {
        fail(99);
        return;
    }

    if (!imu_sample())
    {
        imu_ok = 0;
        imu_error_counter++;

        fail(3);
        return;
    }

    imu_ok = 1;

    if (internal_state == 1)
        calibrate_sample();
    else
        compute_only();
}

/* ====================================================
 * MAIN
 * ==================================================== */

int main(void)
{
    uint32_t last_loop;

    motor_all_off();

    reset_cause = RCC_CSR;

    watchdog_reset_detected =
        (reset_cause & (1U << 29)) ?
        1U : 0U;

    RCC_CSR |= (1U << 24);

    if (!clock_init())
    {
        fail(10);

        for (;;)
            motor_hardware_off();
    }

    if (!watchdog_init())
    {
        fail(11);

        for (;;)
            motor_hardware_off();
    }

    RCC_AHBENR |= (1U << 18);

    /* Board enable PA1 */

    BSRR(GA) = (1U << 1);

    TYPE(GA) &= ~(1U << 1);

    MODE(GA) =
        (MODE(GA) & ~(3U << 2)) |
        (1U << 2);

    /* Initialize radio */

    spi_init();
    rx_init();

    /* Initialize IMU */

    i2c_init();
    delay_ms(20);

    if (watchdog_reset_detected)
    {
        fail(8);
    }
    else if (!imu_init())
    {
        imu_error_counter++;
        fail(1);
    }
    else
    {
        imu_ok = 1;
        calibrate_start();
    }

    motor_all_off();
    watchdog_feed();

    last_loop = system_ms;

    for (;;)
    {
        uint32_t now = system_ms;

        uint32_t elapsed =
            (uint32_t)(now - last_loop);

        if (elapsed < LOOP_MS)
            continue;

        last_loop = now;

        /* IMU and computations */

        control_cycle(elapsed);

        /* Receive radio packets */

        radio_poll(system_ms);

        /* Check link freshness */

        radio_link_update(system_ms);

        loop_exec_ms =
            (uint32_t)(system_ms - now);

        if (loop_exec_ms > loop_max_exec_ms)
            loop_max_exec_ms = loop_exec_ms;

        if (loop_exec_ms >= LOOP_MS)
        {
            loop_overrun_counter++;
            fail(4);
        }

        motor_all_off();

        if (!outputs_verified_off)
            fail(5);

        watchdog_feed();
    }
}
