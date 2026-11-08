
/*
 * DRONE_ STEP 15
 * STM32F050K4
 *
 * NRF24L01+ HW-725:
 * PA15 - CSN
 * PB3  - SCK
 * PB4  - MISO
 * PB5  - MOSI
 * PB6  - CE
 *
 * M540 IMU:
 * PB7 - SDA
 * PB8 - SCL
 * I2C address 0x69
 * WHO_AM_I = 0x7D
 *
 * Motors:
 * M1 PA11
 * M2 PA9
 * M3 PA8
 * M4 PA10
 *
 * ALL MOTOR OUTPUTS PERMANENTLY OFF.
 * NO ARM.
 * NO MOTOR PWM.
 *
 * Step 15:
 * Complementary attitude filter
 * Roll/Pitch angle diagnostics
 * Radio and rate-controller diagnostics
 *
 * BENCH TEST ONLY.
 */

#include <stdint.h>

/* ========================================
 * HARDWARE REGISTERS
 * ======================================== */

#define REG(a) \
    (*(volatile uint32_t *)(uintptr_t)(a))

#define RCC_CR       REG(0x40021000U)
#define RCC_CFGR     REG(0x40021004U)
#define RCC_APB1RSTR REG(0x40021010U)
#define RCC_AHBENR   REG(0x40021014U)
#define RCC_APB2ENR  REG(0x40021018U)
#define RCC_APB1ENR  REG(0x4002101CU)
#define RCC_CSR      REG(0x40021024U)
#define RCC_CFGR3    REG(0x40021030U)

#define GPIOA_BASE 0x48000000U
#define GPIOB_BASE 0x48000400U

#define MODER(b)   REG((b)+0x00U)
#define OTYPER(b)  REG((b)+0x04U)
#define OSPEEDR(b) REG((b)+0x08U)
#define PUPDR(b)   REG((b)+0x0CU)
#define ODR(b)     REG((b)+0x14U)
#define BSRR(b)    REG((b)+0x18U)
#define AFRL(b)    REG((b)+0x20U)
#define AFRH(b)    REG((b)+0x24U)

#define SYST_CSR REG(0xE000E010U)
#define SYST_RVR REG(0xE000E014U)
#define SYST_CVR REG(0xE000E018U)

#define TIM1_CR1  REG(0x40012C00U)
#define TIM1_EGR  REG(0x40012C14U)
#define TIM1_CCER REG(0x40012C20U)
#define TIM1_CCR1 REG(0x40012C34U)
#define TIM1_CCR2 REG(0x40012C38U)
#define TIM1_CCR3 REG(0x40012C3CU)
#define TIM1_CCR4 REG(0x40012C40U)
#define TIM1_BDTR REG(0x40012C44U)

#define IWDG_KR   REG(0x40003000U)
#define IWDG_PR   REG(0x40003004U)
#define IWDG_RLR  REG(0x40003008U)
#define IWDG_SR   REG(0x4000300CU)
#define IWDG_WINR REG(0x40003010U)

#define DBGMCU_APB1_FZ REG(0x40015808U)

#define I2C_CR1    REG(0x40005400U)
#define I2C_CR2    REG(0x40005404U)
#define I2C_TIMING REG(0x40005410U)
#define I2C_ISR    REG(0x40005418U)
#define I2C_ICR    REG(0x4000541CU)
#define I2C_RXDR   REG(0x40005424U)
#define I2C_TXDR   REG(0x40005428U)

#define SPI_CR1 REG(0x40013000U)
#define SPI_CR2 REG(0x40013004U)
#define SPI_SR  REG(0x40013008U)

#define SPI_DR8 \
    (*(volatile uint8_t *)(uintptr_t)0x4001300CU)

/* ========================================
 * CONFIGURATION
 * ======================================== */

#define MOTOR_PINS (0xFU << 8U)
#define MOTOR_MODE_MASK (0xFFU << 16U)
#define MOTOR_OUTPUT_MODES (0x55U << 16U)

#define LOOP_PERIOD_MS 5U
#define RADIO_POLL_MS 10U
#define RADIO_TIMEOUT_MS 250U
#define CAL_COUNT 500U

#define MAX_ROLL_DPS 90.0f
#define MAX_PITCH_DPS 90.0f
#define MAX_YAW_DPS 60.0f

#define SIM_BASE_PERCENT 35.0f

/* Step 15 filter */

#define ATTITUDE_ALPHA 0.98f
#define RAD_TO_DEG 57.2957795f

/*
 * Preliminary axis mapping.
 * To be checked with physical tilt tests.
 */

#define ACC_ROLL_SIGN  1.0f
#define ACC_PITCH_SIGN -1.0f

/* ========================================
 * LIVE EXPRESSIONS
 * ======================================== */

/* System */

volatile uint32_t system_ms = 0;

volatile uint32_t fc_state = 0;
volatile uint32_t fault_code = 0;

volatile uint32_t physical_motor_lock = 1;
volatile uint32_t outputs_verified_off = 0;

volatile uint32_t motor1_percent = 0;
volatile uint32_t motor2_percent = 0;
volatile uint32_t motor3_percent = 0;
volatile uint32_t motor4_percent = 0;

volatile uint32_t arm_request = 0;
volatile uint32_t arm_reject_counter = 0;
volatile uint32_t arm_block_flags = 1;

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

/* Controller */

volatile float pid_roll_output = 0;
volatile float pid_pitch_output = 0;
volatile float pid_yaw_output = 0;

/* Mixer */

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

/* SPI */

volatile uint32_t radio_spi_initialized = 0;
volatile uint32_t radio_spi_ok = 0;
volatile uint32_t radio_spi_error_counter = 0;
volatile uint32_t radio_spi_last_error = 0;

/* Radio receiver */

volatile uint32_t radio_rx_configured = 0;
volatile uint32_t radio_rx_config_error = 0;

volatile uint32_t radio_rx_poll_counter = 0;
volatile uint32_t radio_rx_raw_packets = 0;

volatile uint32_t radio_rx_invalid_packets = 0;
volatile uint32_t radio_rx_read_errors = 0;

volatile uint32_t radio_packets_received = 0;

volatile uint32_t radio_rx_status = 0xFF;
volatile uint32_t radio_rx_fifo_status = 0xFF;

volatile uint32_t radio_rx_throttle_raw = 225;
volatile uint32_t radio_rx_yaw_raw = 141;
volatile uint32_t radio_rx_roll_raw = 112;
volatile uint32_t radio_rx_pitch_raw = 112;

volatile uint32_t radio_rx_last_packet_ms = 0;

volatile uint8_t radio_last_packet[9] = {0};

/* Link diagnostics */

volatile uint32_t radio_diag_alive = 0;
volatile uint32_t radio_diag_age_ms = UINT32_MAX;

volatile uint32_t radio_diag_lost_count = 0;
volatile uint32_t radio_diag_reconnect_count = 0;

volatile uint32_t radio_diag_seq_gap_count = 0;
volatile uint32_t radio_diag_period_ms = 0;

volatile uint32_t radio_diag_duplicate_count = 0;
volatile uint32_t radio_diag_last_seq = 0;

volatile uint32_t radio_diag_throttle = 225;
volatile uint32_t radio_diag_yaw = 141;
volatile uint32_t radio_diag_roll = 112;
volatile uint32_t radio_diag_pitch = 112;

/* Stick simulation */

volatile uint32_t sim_ready = 0;
volatile uint32_t sim_link_ok = 0;
volatile uint32_t sim_inputs_ok = 0;

volatile uint32_t sim_throttle_locked = 1;
volatile uint32_t sim_update_counter = 0;

volatile float sim_roll_stick = 0;
volatile float sim_pitch_stick = 0;
volatile float sim_yaw_stick = 0;

volatile float sim_roll_target_dps = 0;
volatile float sim_pitch_target_dps = 0;
volatile float sim_yaw_target_dps = 0;

volatile float sim_roll_error_dps = 0;
volatile float sim_pitch_error_dps = 0;
volatile float sim_yaw_error_dps = 0;

/* ========================================
 * STEP 15 - ATTITUDE ESTIMATION
 * ======================================== */

volatile uint32_t attitude_valid = 0;

volatile uint32_t attitude_accel_used = 0;
volatile uint32_t attitude_accel_rejected = 0;

volatile float attitude_roll_deg = 0;
volatile float attitude_pitch_deg = 0;

volatile float attitude_accel_roll_deg = 0;
volatile float attitude_accel_pitch_deg = 0;

volatile float attitude_gyro_roll_deg = 0;
volatile float attitude_gyro_pitch_deg = 0;

volatile float attitude_accel_norm2 = 0;

static uint32_t attitude_initialized = 0;
static uint32_t attitude_last_accel_ms = 0;

/* ========================================
 * INTERNAL VARIABLES
 * ======================================== */

/*
 * 0 = BOOT
 * 1 = CALIBRATING
 * 2 = DISARMED
 * 4 = FAILSAFE
 */

static uint32_t core_state = 0;

static uint32_t i2c_start_ms = 0;
static uint32_t calib_start_ms = 0;

static uint32_t last_radio_poll_ms = 0;
static uint32_t last_valid_ms = 0;
static uint32_t last_period_ms = 0;

static uint32_t ever_seen = 0;
static uint32_t have_seq = 0;
static uint32_t previous_seq = 0;

static int32_t gyro_sum[3] = {0, 0, 0};
static int16_t gyro_min[3];
static int16_t gyro_max[3];

/* ========================================
 * HELPERS
 * ======================================== */

static float clampf(
    float value,
    float minimum,
    float maximum
)
{
    if (value < minimum)
        return minimum;

    if (value > maximum)
        return maximum;

    return value;
}

static float absf(float value)
{
    return value < 0.0f ? -value : value;
}

/* ========================================
 * MOTOR HARDWARE LOCK
 * ======================================== */

static void motor_hardware_off(void)
{
    RCC_AHBENR |= (1U << 17);
    RCC_APB2ENR |= (1U << 11);

    (void)RCC_AHBENR;
    (void)RCC_APB2ENR;

    TIM1_BDTR = 0;
    TIM1_CCER = 0;
    TIM1_CR1 = 0;

    /* All motor pins LOW */

    BSRR(GPIOA_BASE) = MOTOR_PINS << 16U;

    OTYPER(GPIOA_BASE) &= ~MOTOR_PINS;

    PUPDR(GPIOA_BASE) =
        (PUPDR(GPIOA_BASE) & ~MOTOR_MODE_MASK) |
        (0xAAU << 16U);

    MODER(GPIOA_BASE) =
        (MODER(GPIOA_BASE) & ~MOTOR_MODE_MASK) |
        MOTOR_OUTPUT_MODES;

    TIM1_CCR1 = 0;
    TIM1_CCR2 = 0;
    TIM1_CCR3 = 0;
    TIM1_CCR4 = 0;

    TIM1_EGR = 1;
}

/*
 * Retained for compatibility with the
 * previous bare-metal project.
 *
 * If the CubeIDE project already provides
 * a strong SystemInit implementation,
 * only one definition must remain.
 */

void SystemInit(void)
{
    motor_hardware_off();
}

static uint32_t motor_off_verified(void)
{
    return (
        ((MODER(GPIOA_BASE) & MOTOR_MODE_MASK)
            == MOTOR_OUTPUT_MODES) &&

        ((OTYPER(GPIOA_BASE) & MOTOR_PINS) == 0) &&
        ((ODR(GPIOA_BASE) & MOTOR_PINS) == 0) &&

        TIM1_BDTR == 0 &&
        TIM1_CCER == 0 &&
        (TIM1_CR1 & 1U) == 0 &&

        TIM1_CCR1 == 0 &&
        TIM1_CCR2 == 0 &&
        TIM1_CCR3 == 0 &&
        TIM1_CCR4 == 0
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

/* ========================================
 * ZERO COMPUTED COMMANDS
 * ======================================== */

static void sim_zero(void)
{
    sim_ready = 0;
    sim_link_ok = 0;
    sim_inputs_ok = 0;

    sim_roll_stick = 0;
    sim_pitch_stick = 0;
    sim_yaw_stick = 0;

    sim_roll_target_dps = 0;
    sim_pitch_target_dps = 0;
    sim_yaw_target_dps = 0;

    sim_roll_error_dps = 0;
    sim_pitch_error_dps = 0;
    sim_yaw_error_dps = 0;

    pid_roll_output = 0;
    pid_pitch_output = 0;
    pid_yaw_output = 0;

    mix_m1_percent = 0;
    mix_m2_percent = 0;
    mix_m3_percent = 0;
    mix_m4_percent = 0;
}

/* ========================================
 * FAILSAFE
 * ======================================== */

static void fail(uint32_t code)
{
    motor_all_off();

    if (fault_code == 0)
        fault_code = code;

    core_state = 4;
    fc_state = 4;

    sim_zero();
    attitude_valid = 0;
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

/* ========================================
 * CLOCK
 * ======================================== */

void SysTick_Handler(void)
{
    system_ms++;
}

static void delay_ms(uint32_t ms)
{
    uint32_t start = system_ms;

    while ((uint32_t)(system_ms - start) < ms)
    {
    }
}

static uint32_t clock_init(void)
{
    uint32_t budget = 200000;

    RCC_CR |= 1U;

    while (!(RCC_CR & 2U))
    {
        if (--budget == 0)
            return 0;
    }

    /* HSI 8 MHz */

    RCC_CFGR &= ~3U;

    budget = 200000;

    while (RCC_CFGR & (3U << 2U))
    {
        if (--budget == 0)
            return 0;
    }

    RCC_CFGR &= ~(
        (0xFU << 4U) |
        (7U << 8U)
    );

    /* SysTick 1 ms */

    SYST_RVR = 7999;
    SYST_CVR = 0;
    SYST_CSR = 7;

    return 1;
}

/* ========================================
 * WATCHDOG
 * ======================================== */

static uint32_t watchdog_init(void)
{
    uint32_t start;

    RCC_APB2ENR |= (1U << 22);

    DBGMCU_APB1_FZ |= (1U << 12);

    IWDG_KR = 0xCCCC;
    IWDG_KR = 0x5555;

    IWDG_PR = 4;
    IWDG_RLR = 624;

    start = system_ms;

    while (IWDG_SR & 7U)
    {
        if ((uint32_t)(system_ms - start) >= 100U)
            return 0;
    }

    IWDG_WINR = 0xFFF;

    start = system_ms;

    while (IWDG_SR & 7U)
    {
        if ((uint32_t)(system_ms - start) >= 100U)
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

/* ========================================
 * I2C INITIALIZATION
 * ======================================== */

static void i2c_init(void)
{
    RCC_AHBENR |= (1U << 18);

    /* PB7 SDA, PB8 SCL */

    MODER(GPIOB_BASE) =
        (MODER(GPIOB_BASE) &
         ~((3U << 14) | (3U << 16))) |
        ((2U << 14) | (2U << 16));

    OTYPER(GPIOB_BASE) |=
        (1U << 7) | (1U << 8);

    OSPEEDR(GPIOB_BASE) |=
        (3U << 14) | (3U << 16);

    PUPDR(GPIOB_BASE) =
        (PUPDR(GPIOB_BASE) &
         ~((3U << 14) | (3U << 16))) |
        ((1U << 14) | (1U << 16));

    AFRL(GPIOB_BASE) =
        (AFRL(GPIOB_BASE) & ~(0xFU << 28)) |
        (1U << 28);

    AFRH(GPIOB_BASE) =
        (AFRH(GPIOB_BASE) & ~0xFU) |
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
        uint32_t status = I2C_ISR;

        if (status & (1U << 8))
            I2C_ICR = (1U << 8);

        if (status & (
            (1U << 4) |
            (1U << 9) |
            (1U << 10)))
        {
            return 0;
        }

        if (status & flag)
            return 1;

        if ((uint32_t)(
            system_ms - i2c_start_ms
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
            system_ms - i2c_start_ms
        ) >= 4U)
        {
            return 0;
        }
    }

    return 1;
}

static uint32_t imu_write(
    uint8_t reg,
    uint8_t value
)
{
    i2c_start_ms = system_ms;

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

    I2C_TXDR = value;

    if (!i2c_wait(1U << 5))
        return 0;

    I2C_ICR = (1U << 5);

    return 1;
}

static uint32_t imu_read(
    uint8_t reg,
    uint8_t *data,
    uint32_t count
)
{
    uint32_t i;

    if (count == 0 || count > 255)
        return 0;

    i2c_start_ms = system_ms;

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
        (count << 16) |
        (1U << 10) |
        (1U << 25) |
        (1U << 13);

    for (i = 0; i < count; i++)
    {
        if (!i2c_wait(1U << 2))
            return 0;

        data[i] = (uint8_t)I2C_RXDR;
    }

    if (!i2c_wait(1U << 5))
        return 0;

    I2C_ICR = (1U << 5);

    return 1;
}

/* ========================================
 * M540 INITIALIZATION
 * ======================================== */

static uint32_t imu_init(void)
{
    uint8_t who = 0;

    if (!imu_write(0x6B, 0x80))
        return 0;

    delay_ms(100);

    if (!imu_write(0x6B, 0x01))
        return 0;

    delay_ms(20);

    /* Accelerometer +/-16g */

    if (!imu_write(0x1C, 0x18))
        return 0;

    if (!imu_write(0x1D, 0x05))
        return 0;

    /* Gyroscope +/-2000dps */

    if (!imu_write(0x1B, 0x18))
        return 0;

    if (!imu_write(0x1A, 0x00))
        return 0;

    delay_ms(20);

    if (!imu_read(0x75, &who, 1))
        return 0;

    imu_whoami = who;

    return who == 0x7D;
}

/* ========================================
 * IMU SAMPLE
 * ======================================== */

static uint32_t imu_sample(void)
{
    uint8_t d[14];

    if (!imu_read(0x3B, d, 14))
        return 0;

    accel_x = (int16_t)(
        ((uint16_t)d[0] << 8) | d[1]);

    accel_y = (int16_t)(
        ((uint16_t)d[2] << 8) | d[3]);

    accel_z = (int16_t)(
        ((uint16_t)d[4] << 8) | d[5]);

    gyro_x = (int16_t)(
        ((uint16_t)d[8] << 8) | d[9]);

    gyro_y = (int16_t)(
        ((uint16_t)d[10] << 8) | d[11]);

    gyro_z = (int16_t)(
        ((uint16_t)d[12] << 8) | d[13]);

    imu_frame_counter++;

    if (
        accel_x == 0 &&
        accel_y == 0 &&
        accel_z == 0 &&
        gyro_x == 0 &&
        gyro_y == 0 &&
        gyro_z == 0
    )
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

/* ========================================
 * GYROSCOPE CALIBRATION
 * ======================================== */

static void calibration_start(void)
{
    uint32_t i;

    for (i = 0; i < 3; i++)
    {
        gyro_sum[i] = 0;
        gyro_min[i] = INT16_MAX;
        gyro_max[i] = INT16_MIN;
    }

    calibration_status = 1;
    calibration_samples = 0;

    calib_start_ms = system_ms;

    core_state = 1;
    fc_state = 1;
}

static void calibration_update(void)
{
    int16_t values[3] = {
        gyro_x,
        gyro_y,
        gyro_z
    };

    uint32_t i;

    if ((uint32_t)(
        system_ms - calib_start_ms
    ) > 4000U)
    {
        calibration_status = 3;
        fail(2);
        return;
    }

    for (i = 0; i < 3; i++)
    {
        if (values[i] < gyro_min[i])
            gyro_min[i] = values[i];

        if (values[i] > gyro_max[i])
            gyro_max[i] = values[i];

        if (
            (int32_t)gyro_max[i] -
            (int32_t)gyro_min[i] > 164 ||

            values[i] > 1312 ||
            values[i] < -1312
        )
        {
            calibration_status = 3;
            fail(2);
            return;
        }

        gyro_sum[i] += values[i];
    }

    calibration_samples++;

    if (calibration_samples == CAL_COUNT)
    {
        gyro_x_offset =
            (float)gyro_sum[0] / CAL_COUNT;

        gyro_y_offset =
            (float)gyro_sum[1] / CAL_COUNT;

        gyro_z_offset =
            (float)gyro_sum[2] / CAL_COUNT;

        calibration_status = 2;

        core_state = 2;
        fc_state = 2;
    }
}

/* ========================================
 * STEP 15 - COMPLEMENTARY FILTER
 * ======================================== */

/*
 * For diagnostic tilts up to ~15 degrees.
 *
 * Gyro integration:
 *    angle += gyro_rate * dt
 *
 * Accelerometer correction:
 *    complementary filtering
 *
 * Axis signs have NOT yet been verified.
 * Filter outputs do not control motors.
 */

static void attitude_update(uint32_t dt_ms)
{
    float dt;
    float ax;
    float ay;
    float az;

    float norm2;

    float gyro_roll;
    float gyro_pitch;

    float predicted_roll;
    float predicted_pitch;

    uint32_t accel_ok;

    if (
        core_state != 2 ||
        calibration_status != 2 ||
        imu_ok != 1
    )
    {
        attitude_valid = 0;
        attitude_initialized = 0;
        return;
    }

    dt = (float)dt_ms * 0.001f;

    if (dt <= 0.0f || dt > 0.020f)
    {
        attitude_valid = 0;
        return;
    }

    /* Gyroscope */

    gyro_roll =
        ((float)gyro_x - gyro_x_offset) /
        16.4f;

    gyro_pitch =
        ((float)gyro_y - gyro_y_offset) /
        16.4f;

    /* Raw accelerometer */

    ax = (float)accel_x;
    ay = (float)accel_y;
    az = (float)accel_z;

    norm2 = ax * ax + ay * ay + az * az;

    attitude_accel_norm2 = norm2;

    /*
     * Only accept acceleration near 1g
     * and relatively small tilts.
     *
     * 2048 counts/g at +/-16g.
     */

    accel_ok =
        az > 1750.0f &&
        az < 2400.0f &&

        absf(ax) < 700.0f &&
        absf(ay) < 700.0f &&

        norm2 > 2890000.0f &&
        norm2 < 5760000.0f;

    /*
     * Integrate gyroscope angular rates.
     * Diagnostic values may drift.
     */

    if (attitude_initialized)
    {
        attitude_gyro_roll_deg +=
            gyro_roll * dt;

        attitude_gyro_pitch_deg +=
            gyro_pitch * dt;
    }

    predicted_roll =
        attitude_roll_deg + gyro_roll * dt;

    predicted_pitch =
        attitude_pitch_deg + gyro_pitch * dt;

    if (accel_ok)
    {
        /*
         * Small-angle approximation.
         * Approximately valid near level.
         *
         * These signs are provisional.
         */

        attitude_accel_roll_deg =
            ACC_ROLL_SIGN *
            (ay / az) * RAD_TO_DEG;

        attitude_accel_pitch_deg =
            ACC_PITCH_SIGN *
            (ax / az) * RAD_TO_DEG;

        if (!attitude_initialized)
        {
            attitude_roll_deg =
                attitude_accel_roll_deg;

            attitude_pitch_deg =
                attitude_accel_pitch_deg;

            attitude_gyro_roll_deg = 0.0f;
            attitude_gyro_pitch_deg = 0.0f;

            attitude_initialized = 1;
        }
        else
        {
            attitude_roll_deg =
                ATTITUDE_ALPHA *
                predicted_roll +

                (1.0f - ATTITUDE_ALPHA) *
                attitude_accel_roll_deg;

            attitude_pitch_deg =
                ATTITUDE_ALPHA *
                predicted_pitch +

                (1.0f - ATTITUDE_ALPHA) *
                attitude_accel_pitch_deg;
        }

        attitude_last_accel_ms = system_ms;

        attitude_accel_used++;
    }
    else
    {
        /*
         * Accelerometer may be unreliable
         * during rapid motion.
         * Temporarily use gyroscope only.
         */

        attitude_roll_deg = predicted_roll;
        attitude_pitch_deg = predicted_pitch;

        attitude_accel_rejected++;
    }

    /* Limit diagnostic estimate */

    attitude_roll_deg = clampf(
        attitude_roll_deg,
        -180.0f,
        180.0f
    );

    attitude_pitch_deg = clampf(
        attitude_pitch_deg,
        -180.0f,
        180.0f
    );

    attitude_valid =
        attitude_initialized &&
        ((uint32_t)(
            system_ms - attitude_last_accel_ms
        ) <= 500U);
}

/* ========================================
 * RADIO SPI INITIALIZATION
 * ======================================== */

static void spi_init(void)
{
    RCC_AHBENR |=
        (1U << 17) |
        (1U << 18);

    RCC_APB2ENR |= (1U << 12);

    (void)RCC_AHBENR;
    (void)RCC_APB2ENR;

    /* CSN HIGH */

    BSRR(GPIOA_BASE) = (1U << 15);

    /* CE LOW */

    BSRR(GPIOB_BASE) = (1U << 22);

    OTYPER(GPIOA_BASE) &= ~(1U << 15);

    OTYPER(GPIOB_BASE) &= ~(
        (1U << 3) |
        (1U << 5) |
        (1U << 6)
    );

    MODER(GPIOA_BASE) =
        (MODER(GPIOA_BASE) &
         ~(3U << 30)) |
        (1U << 30);

    PUPDR(GPIOA_BASE) &= ~(3U << 30);

    /* PB3 PB4 PB5 AF0, PB6 GPIO */

    MODER(GPIOB_BASE) =
        (MODER(GPIOB_BASE) & ~(
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

    PUPDR(GPIOB_BASE) &= ~(
        (3U << 6) |
        (3U << 8) |
        (3U << 10) |
        (3U << 12)
    );

    OSPEEDR(GPIOB_BASE) |=
        (3U << 6) |
        (3U << 10);

    AFRL(GPIOB_BASE) &= ~(
        (0xFU << 12) |
        (0xFU << 16) |
        (0xFU << 20)
    );

    /* SPI mode 0, 8-bit */

    SPI_CR1 = 0;

    SPI_CR2 =
        (7U << 8) |
        (1U << 12);

    SPI_CR1 =
        (1U << 2) |
        (4U << 3) |
        (1U << 8) |
        (1U << 9) |
        (1U << 6);

    radio_spi_initialized = 1;
}

/* ========================================
 * RADIO SPI TRANSFER
 * ======================================== */

static uint32_t spi_wait(
    uint32_t flag,
    uint32_t required,
    uint32_t error
)
{
    uint32_t start = system_ms;

    for (;;)
    {
        if (
            ((SPI_SR & flag) != 0U) ==
            (required != 0U)
        )
            return 1;

        if ((uint32_t)(
            system_ms - start
        ) >= 2U)
        {
            radio_spi_last_error = error;
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

    SPI_DR8 = tx;

    if (!spi_wait(1U, 1U, 2U))
        return 0;

    *rx = SPI_DR8;

    return 1;
}

static uint32_t spi_xfer(
    const uint8_t *tx,
    uint8_t *rx,
    uint32_t count
)
{
    uint32_t i;
    uint32_t ok = 1;

    if (count == 0 || count > 11)
        return 0;

    /* CSN LOW */

    BSRR(GPIOA_BASE) = (1U << 31);

    for (i = 0; i < count; i++)
    {
        uint8_t value = 0;

        if (!spi_byte(tx[i], &value))
        {
            ok = 0;
            break;
        }

        if (rx)
            rx[i] = value;
    }

    if (ok)
        ok = spi_wait(1U << 7, 0U, 3U);

    /* CSN HIGH */

    BSRR(GPIOA_BASE) = (1U << 15);

    if (SPI_SR & (1U << 6))
    {
        volatile uint8_t d = SPI_DR8;
        volatile uint32_t s = SPI_SR;

        (void)d;
        (void)s;

        radio_spi_last_error = 4;
        ok = 0;
    }

    if (!ok)
        radio_spi_error_counter++;

    return ok;
}

static uint32_t reg_write(
    uint8_t address,
    const uint8_t *data,
    uint32_t count
)
{
    uint8_t tx[11];
    uint32_t i;

    if (count == 0 || count > 10)
        return 0;

    tx[0] = (uint8_t)(
        0x20U | (address & 0x1FU)
    );

    for (i = 0; i < count; i++)
        tx[i + 1] = data[i];

    return spi_xfer(tx, 0, count + 1);
}

static uint32_t reg_wr1(
    uint8_t address,
    uint8_t value
)
{
    return reg_write(address, &value, 1);
}

static uint32_t reg_read(
    uint8_t address,
    uint8_t *value
)
{
    uint8_t tx[2] = {
        (uint8_t)(address & 0x1FU),
        0xFF
    };

    uint8_t rx[2];

    if (!spi_xfer(tx, rx, 2))
        return 0;

    *value = rx[1];

    return 1;
}

static uint32_t radio_cmd(uint8_t command)
{
    return spi_xfer(&command, 0, 1);
}

/* ========================================
 * NRF24L01 RECEIVER
 * ======================================== */

static void rx_init(void)
{
    static const uint8_t regs[10] = {
        0x00, 0x01, 0x02, 0x03, 0x04,
        0x05, 0x06, 0x11, 0x1C, 0x1D
    };

    static const uint8_t values[10] = {
        0x0C, 0x00, 0x01, 0x03, 0x00,
        0x2D, 0x07, 0x09, 0x00, 0x00
    };

    static const uint8_t address[5] = {
        0xCC, 0xCC, 0xCC, 0xCC, 0xCC
    };

    uint32_t i;
    uint8_t got = 0;

    radio_rx_configured = 0;
    radio_rx_config_error = 0;

    /* CE LOW */

    BSRR(GPIOB_BASE) = (1U << 22);

    for (i = 0; i < 10; i++)
    {
        if (
            !reg_wr1(regs[i], values[i]) ||
            !reg_read(regs[i], &got) ||
            got != values[i]
        )
        {
            radio_rx_config_error =
                0x100U | regs[i];

            return;
        }
    }

    if (!reg_write(0x0A, address, 5))
    {
        radio_rx_config_error = 0x10A;
        return;
    }

    if (!reg_write(0x10, address, 5))
    {
        radio_rx_config_error = 0x110;
        return;
    }

    if (
        !radio_cmd(0xE2) ||
        !reg_wr1(0x07, 0x70)
    )
    {
        radio_rx_config_error = 0x1E2;
        return;
    }

    /* Enable receiver */

    if (
        !reg_wr1(0x00, 0x0F) ||
        !reg_read(0x00, &got) ||
        got != 0x0F
    )
    {
        radio_rx_config_error = 0x200;
        return;
    }

    delay_ms(2);

    /* CE HIGH */

    BSRR(GPIOB_BASE) = (1U << 6);

    radio_rx_configured = 1;
    radio_spi_ok = 1;
}

/* ========================================
 * RADIO PACKET VALIDATION
 * ======================================== */

static void radio_accept(
    const uint8_t *data,
    uint32_t now
)
{
    uint32_t i;
    uint32_t sum = 0x76U;
    uint32_t seq = data[7];

    for (i = 0; i < 8; i++)
        sum += data[i];

    if (
        (uint8_t)sum != data[8] ||

        data[0] > 225U ||
        data[1] > 225U ||
        data[2] > 225U ||
        data[3] > 225U ||

        data[4] != 0x05 ||
        data[5] != 0x38 ||
        data[6] != 0x00
    )
    {
        radio_rx_invalid_packets++;
        return;
    }

    if (radio_diag_alive && have_seq)
    {
        uint32_t delta = (uint8_t)(
            seq - (uint8_t)previous_seq
        );

        if (delta == 0)
        {
            radio_diag_duplicate_count++;
            return;
        }

        if (delta >= 128U)
        {
            radio_rx_invalid_packets++;
            return;
        }

        if (delta > 1U)
            radio_diag_seq_gap_count +=
                delta - 1U;

        radio_diag_period_ms =
            (uint32_t)(now - last_period_ms);
    }
    else
    {
        if (ever_seen)
            radio_diag_reconnect_count++;

        radio_diag_period_ms = 0;
    }

    previous_seq = seq;
    have_seq = 1;

    radio_diag_last_seq = seq;

    last_period_ms = now;
    last_valid_ms = now;

    ever_seen = 1;

    radio_diag_alive = 1;
    radio_diag_age_ms = 0;

    radio_diag_throttle = data[0];
    radio_diag_yaw = data[1];
    radio_diag_roll = data[2];
    radio_diag_pitch = data[3];

    radio_rx_throttle_raw = data[0];
    radio_rx_yaw_raw = data[1];
    radio_rx_roll_raw = data[2];
    radio_rx_pitch_raw = data[3];

    radio_rx_last_packet_ms = now;

    radio_packets_received++;

    for (i = 0; i < 9; i++)
        radio_last_packet[i] = data[i];
}

/* ========================================
 * RADIO POLLING
 * ======================================== */

static void radio_poll(uint32_t now)
{
    uint8_t status = 0;
    uint8_t fifo = 0;

    uint8_t tx[10];
    uint8_t rx[10];
    uint8_t packet[9];

    uint32_t i;

    if (!radio_rx_configured)
        return;

    if ((uint32_t)(
        now - last_radio_poll_ms
    ) < RADIO_POLL_MS)
        return;

    last_radio_poll_ms = now;

    radio_rx_poll_counter++;

    if (
        !reg_read(0x07, &status) ||
        !reg_read(0x17, &fifo)
    )
    {
        radio_rx_read_errors++;
        return;
    }

    radio_rx_status = status;
    radio_rx_fifo_status = fifo;

    /* RX empty */

    if (fifo & 1U)
        return;

    /* Read 9-byte payload */

    tx[0] = 0x61;

    for (i = 1; i < 10; i++)
        tx[i] = 0xFF;

    if (!spi_xfer(tx, rx, 10))
    {
        radio_rx_read_errors++;
        return;
    }

    for (i = 0; i < 9; i++)
        packet[i] = rx[i + 1];

    radio_rx_raw_packets++;

    radio_accept(packet, now);

    /* Clear RX_DR */

    if (!reg_wr1(0x07, 0x40))
        radio_rx_read_errors++;
}

/* ========================================
 * LINK LOSS DETECTION
 * ======================================== */

static void link_update(uint32_t now)
{
    if (!ever_seen)
    {
        radio_diag_age_ms = UINT32_MAX;
        return;
    }

    radio_diag_age_ms =
        (uint32_t)(now - last_valid_ms);

    if (
        radio_diag_alive &&
        radio_diag_age_ms >= RADIO_TIMEOUT_MS
    )
    {
        radio_diag_alive = 0;
        radio_diag_lost_count++;

        have_seq = 0;

        radio_diag_throttle = 225;
        radio_diag_yaw = 141;
        radio_diag_roll = 112;
        radio_diag_pitch = 112;

        sim_zero();
    }
}

/* ========================================
 * JOYSTICK NORMALIZATION
 * ======================================== */

static float axis_normalize(
    uint32_t value,
    uint32_t middle
)
{
    float normalized;

    if (value > 225U)
        return 0.0f;

    if (
        value >= middle - 3U &&
        value <= middle + 3U
    )
        return 0.0f;

    if (value >= middle)
    {
        normalized =
            (float)(value - middle) /
            (float)(225U - middle);
    }
    else
    {
        normalized =
            -((float)(middle - value) /
              (float)middle);
    }

    return clampf(
        normalized,
        -1.0f,
        1.0f
    );
}

/* ========================================
 * RATE CONTROLLER SIMULATION
 * ======================================== */

static void simulation_update(void)
{
    float r;
    float p;
    float y;

    sim_update_counter++;

    sim_throttle_locked =
        (radio_diag_throttle == 225U) ?
        1U : 0U;

    if (
        core_state != 2 ||
        imu_ok != 1 ||
        calibration_status != 2 ||

        !radio_diag_alive ||
        radio_diag_age_ms >= RADIO_TIMEOUT_MS
    )
    {
        sim_zero();
        return;
    }

    sim_link_ok = 1;

    /*
     * Keep mathematical controller
     * blocked above minimum throttle.
     */

    if (
        radio_diag_throttle != 225U ||

        radio_diag_yaw > 225U ||
        radio_diag_roll > 225U ||
        radio_diag_pitch > 225U
    )
    {
        sim_zero();
        return;
    }

    sim_inputs_ok = 1;

    /* Gyroscope rates */

    gyro_roll_dps =
        ((float)gyro_x - gyro_x_offset) /
        16.4f;

    gyro_pitch_dps =
        ((float)gyro_y - gyro_y_offset) /
        16.4f;

    gyro_yaw_dps =
        ((float)gyro_z - gyro_z_offset) /
        16.4f;

    /* Joystick commands */

    sim_roll_stick =
        axis_normalize(
            radio_diag_roll, 112
        );

    sim_pitch_stick =
        axis_normalize(
            radio_diag_pitch, 112
        );

    sim_yaw_stick =
        axis_normalize(
            radio_diag_yaw, 141
        );

    /* Angular velocity targets */

    sim_roll_target_dps =
        MAX_ROLL_DPS * sim_roll_stick;

    sim_pitch_target_dps =
        MAX_PITCH_DPS * sim_pitch_stick;

    sim_yaw_target_dps =
        MAX_YAW_DPS * sim_yaw_stick;

    /* Angular velocity errors */

    sim_roll_error_dps =
        sim_roll_target_dps -
        gyro_roll_dps;

    sim_pitch_error_dps =
        sim_pitch_target_dps -
        gyro_pitch_dps;

    sim_yaw_error_dps =
        sim_yaw_target_dps -
        gyro_yaw_dps;

    /* Proportional controller */

    pid_roll_output = clampf(
        0.060f * sim_roll_error_dps,
        -8.0f,
        8.0f
    );

    pid_pitch_output = clampf(
        0.060f * sim_pitch_error_dps,
        -8.0f,
        8.0f
    );

    pid_yaw_output = clampf(
        0.050f * sim_yaw_error_dps,
        -6.0f,
        6.0f
    );

    r = pid_roll_output;
    p = pid_pitch_output;
    y = pid_yaw_output;

    /*
     * Diagnostic mixer only.
     *
     * These expressions do NOT
     * control the actual motors.
     */

    mix_m1_percent = clampf(
        SIM_BASE_PERCENT + r + p + y,
        0.0f,
        100.0f
    );

    mix_m2_percent = clampf(
        SIM_BASE_PERCENT - r + p - y,
        0.0f,
        100.0f
    );

    mix_m3_percent = clampf(
        SIM_BASE_PERCENT - r - p + y,
        0.0f,
        100.0f
    );

    mix_m4_percent = clampf(
        SIM_BASE_PERCENT + r - p - y,
        0.0f,
        100.0f
    );

    sim_ready = 1;
}

/* ========================================
 * CONTROL CYCLE
 * ======================================== */

static void control_cycle(uint32_t elapsed)
{
    uint32_t previous_state = fc_state;

    uint32_t off_before =
        motor_off_verified();

    motor_all_off();

    loop_counter++;
    loop_dt_ms = elapsed;

    if (!off_before || !outputs_verified_off)
        fail(5);

    if (elapsed > 6U)
    {
        loop_overrun_counter++;
        fail(4);
    }

    if (previous_state != core_state)
        fail(99);

    /* ARM is always rejected */

    if (arm_request != 0U)
    {
        arm_request = 0;
        arm_reject_counter++;
        arm_block_flags = 1;
    }

    if (core_state == 4)
    {
        sim_zero();
        return;
    }

    if (
        core_state != 1 &&
        core_state != 2
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

    if (core_state == 1)
        calibration_update();

    /* Update gyro diagnostics independently
       of the throttle lock. */

    if (calibration_status == 2)
    {
        gyro_roll_dps =
            ((float)gyro_x - gyro_x_offset) /
            16.4f;

        gyro_pitch_dps =
            ((float)gyro_y - gyro_y_offset) /
            16.4f;

        gyro_yaw_dps =
            ((float)gyro_z - gyro_z_offset) /
            16.4f;
    }
}

/* ========================================
 * MAIN
 * ======================================== */

int main(void)
{
    uint32_t last_loop;

    /* Disable everything immediately */

    motor_all_off();

    reset_cause = RCC_CSR;

    watchdog_reset_detected =
        (reset_cause & (1U << 29)) ?
        1U : 0U;

    RCC_CSR |= (1U << 24);

    /* Clock */

    if (!clock_init())
    {
        fail(10);

        for (;;)
            motor_hardware_off();
    }

    /* Watchdog */

    if (!watchdog_init())
    {
        fail(11);

        for (;;)
            motor_hardware_off();
    }

    RCC_AHBENR |= (1U << 18);

    /* Original board enable PA1 */

    BSRR(GPIOA_BASE) = (1U << 1);

    OTYPER(GPIOA_BASE) &= ~(1U << 1);

    MODER(GPIOA_BASE) =
        (MODER(GPIOA_BASE) & ~(3U << 2)) |
        (1U << 2);

    /* Radio */

    spi_init();
    rx_init();

    /* IMU */

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
        calibration_start();
    }

    motor_all_off();
    watchdog_feed();

    last_loop = system_ms;

    /* ====================================
     * MAIN LOOP
     * ==================================== */

    for (;;)
    {
        uint32_t now = system_ms;

        uint32_t elapsed =
            (uint32_t)(now - last_loop);

        if (elapsed < LOOP_PERIOD_MS)
            continue;

        last_loop = now;

        /* IMU and gyro calibration */

        control_cycle(elapsed);

        /* Step 15 attitude estimation */

        attitude_update(elapsed);

        /* Radio */

        radio_poll(system_ms);
        link_update(system_ms);

        /* Calculation-only controller */

        simulation_update();

        /* Timing */

        loop_exec_ms =
            (uint32_t)(system_ms - now);

        if (loop_exec_ms > loop_max_exec_ms)
            loop_max_exec_ms = loop_exec_ms;

        if (loop_exec_ms >= LOOP_PERIOD_MS)
        {
            loop_overrun_counter++;
            fail(4);
        }

        /* Keep all real motor outputs OFF */

        motor_all_off();

        if (!outputs_verified_off)
            fail(5);

        watchdog_feed();
    }
}

