
/*
 * Drone_ STEP 11
 * STM32F050K4
 * Radio + IMU + computation-only rate controller
 *
 * HW-725 radio:
 * PA15 CSN
 * PB3  SCK
 * PB4  MISO
 * PB5  MOSI
 * PB6  CE
 *
 * M540 IMU:
 * PB7 SDA
 * PB8 SCL
 * I2C address 0x69
 *
 * IMPORTANT:
 * Motors PA8..PA11 are ALWAYS GPIO LOW.
 * TIM1 motor outputs disabled.
 * NO ARM transition.
 * NO motor PWM.
 *
 * BENCH TEST ONLY.
 * Remove propellers and disconnect motors.
 */

#include <stdint.h>

/* ====================================================
 * REGISTERS
 * ==================================================== */

#define REG(a) (*(volatile uint32_t *)(uintptr_t)(a))

#define RCC_CR       REG(0x40021000U)
#define RCC_CFGR     REG(0x40021004U)
#define RCC_APB2RSTR REG(0x4002100CU)
#define RCC_APB1RSTR REG(0x40021010U)
#define RCC_AHBENR   REG(0x40021014U)
#define RCC_APB2ENR  REG(0x40021018U)
#define RCC_APB1ENR  REG(0x4002101CU)
#define RCC_CSR      REG(0x40021024U)
#define RCC_CFGR3    REG(0x40021030U)

#define GA 0x48000000U
#define GB 0x48000400U

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

/* ====================================================
 * CONFIGURATION
 * ==================================================== */

#define MOTOR_PINS (0xFU << 8U)
#define MOTOR_MODE_MASK (0xFFU << 16U)
#define MOTOR_OUTPUT_MODES (0x55U << 16U)

#define LOOP_PERIOD_MS 5U
#define RADIO_TIMEOUT_MS 250U
#define RADIO_POLL_MS 10U
#define CAL_COUNT 500U

/* Diagnostic target angular rates */

#define SIM_MAX_ROLL_DPS 90.0f
#define SIM_MAX_PITCH_DPS 90.0f
#define SIM_MAX_YAW_DPS 60.0f

/* Mathematical base only, NOT motor throttle */

#define SIM_BASE_PERCENT 35.0f

/* ====================================================
 * LIVE EXPRESSIONS
 * ==================================================== */

volatile uint32_t system_ms=0U;

volatile uint32_t fc_state=0U;
volatile uint32_t fault_code=0U;

volatile uint32_t physical_motor_lock=1U;
volatile uint32_t outputs_verified_off=0U;

volatile uint32_t motor1_percent=0U;
volatile uint32_t motor2_percent=0U;
volatile uint32_t motor3_percent=0U;
volatile uint32_t motor4_percent=0U;

volatile uint32_t arm_request=0U;
volatile uint32_t arm_reject_counter=0U;
volatile uint32_t arm_block_flags=1U;

/* IMU */

volatile uint32_t imu_ok=0U;
volatile uint32_t imu_whoami=0U;

volatile uint32_t imu_frame_counter=0U;
volatile uint32_t imu_error_counter=0U;

volatile int16_t accel_x=0;
volatile int16_t accel_y=0;
volatile int16_t accel_z=0;

volatile int16_t gyro_x=0;
volatile int16_t gyro_y=0;
volatile int16_t gyro_z=0;

volatile uint32_t calibration_status=0U;
volatile uint32_t calibration_samples=0U;

volatile float gyro_x_offset=0.0f;
volatile float gyro_y_offset=0.0f;
volatile float gyro_z_offset=0.0f;

volatile float gyro_roll_dps=0.0f;
volatile float gyro_pitch_dps=0.0f;
volatile float gyro_yaw_dps=0.0f;

/* Controller */

volatile float pid_roll_output=0.0f;
volatile float pid_pitch_output=0.0f;
volatile float pid_yaw_output=0.0f;

/* Mixer */

volatile float mix_m1_percent=0.0f;
volatile float mix_m2_percent=0.0f;
volatile float mix_m3_percent=0.0f;
volatile float mix_m4_percent=0.0f;

/* Loop */

volatile uint32_t loop_counter=0U;
volatile uint32_t loop_dt_ms=0U;

volatile uint32_t loop_overrun_counter=0U;
volatile uint32_t loop_exec_ms=0U;
volatile uint32_t loop_max_exec_ms=0U;

/* Watchdog */

volatile uint32_t reset_cause=0U;
volatile uint32_t watchdog_reset_detected=0U;
volatile uint32_t iwdg_refresh_counter=0U;

/* SPI */

volatile uint32_t radio_spi_initialized=0U;
volatile uint32_t radio_spi_ok=0U;

volatile uint32_t radio_spi_error_counter=0U;
volatile uint32_t radio_spi_last_error=0U;

/* Radio receiver */

volatile uint32_t radio_rx_configured=0U;
volatile uint32_t radio_rx_config_error=0U;

volatile uint32_t radio_rx_poll_counter=0U;
volatile uint32_t radio_rx_raw_packets=0U;

volatile uint32_t radio_rx_invalid_packets=0U;
volatile uint32_t radio_rx_read_errors=0U;

volatile uint32_t radio_packets_received=0U;

volatile uint32_t radio_rx_status=0xFFU;
volatile uint32_t radio_rx_fifo_status=0xFFU;

volatile uint32_t radio_rx_throttle_raw=225U;
volatile uint32_t radio_rx_yaw_raw=141U;
volatile uint32_t radio_rx_roll_raw=112U;
volatile uint32_t radio_rx_pitch_raw=112U;

volatile uint32_t radio_rx_last_packet_ms=0U;

volatile uint8_t radio_last_packet[9]={0};

/* Step 10 link diagnostics */

volatile uint32_t radio_diag_alive=0U;
volatile uint32_t radio_diag_age_ms=UINT32_MAX;

volatile uint32_t radio_diag_lost_count=0U;
volatile uint32_t radio_diag_reconnect_count=0U;

volatile uint32_t radio_diag_seq_gap_count=0U;
volatile uint32_t radio_diag_period_ms=0U;

volatile uint32_t radio_diag_duplicate_count=0U;
volatile uint32_t radio_diag_last_seq=0U;

volatile uint32_t radio_diag_throttle=225U;
volatile uint32_t radio_diag_yaw=141U;
volatile uint32_t radio_diag_roll=112U;
volatile uint32_t radio_diag_pitch=112U;

/* ====================================================
 * STEP 11 NEW DIAGNOSTICS
 * ==================================================== */

volatile uint32_t sim_ready=0U;
volatile uint32_t sim_link_ok=0U;
volatile uint32_t sim_inputs_ok=0U;

volatile uint32_t sim_throttle_locked=1U;
volatile uint32_t sim_update_counter=0U;

volatile float sim_roll_stick=0.0f;
volatile float sim_pitch_stick=0.0f;
volatile float sim_yaw_stick=0.0f;

volatile float sim_roll_target_dps=0.0f;
volatile float sim_pitch_target_dps=0.0f;
volatile float sim_yaw_target_dps=0.0f;

volatile float sim_roll_error_dps=0.0f;
volatile float sim_pitch_error_dps=0.0f;
volatile float sim_yaw_error_dps=0.0f;

/* ====================================================
 * INTERNAL STATE
 * ==================================================== */

/*
 * 0 BOOT
 * 1 CALIBRATING
 * 2 DISARMED
 * 4 FAILSAFE
 */

static uint32_t core_state=0U;

static uint32_t i2c_start_ms=0U;
static uint32_t calib_start_ms=0U;

static uint32_t last_radio_poll_ms=0U;
static uint32_t last_valid_ms=0U;
static uint32_t last_period_ms=0U;

static uint32_t ever_seen=0U;
static uint32_t have_seq=0U;
static uint32_t previous_seq=0U;

static int32_t gyro_sum[3]={0,0,0};

static int16_t gyro_min[3];
static int16_t gyro_max[3];

/* ====================================================
 * HELPERS
 * ==================================================== */

static float clampf(
    float x,
    float lo,
    float hi
)
{
    if(x<lo) return lo;
    if(x>hi) return hi;
    return x;
}

static float absf(float x)
{
    return x<0.0f?-x:x;
}

/* ====================================================
 * MOTOR HARDWARE LOCK
 * ==================================================== */

static void motor_hardware_off(void)
{
    RCC_AHBENR|=(1U<<17);
    RCC_APB2ENR|=(1U<<11);

    (void)RCC_AHBENR;
    (void)RCC_APB2ENR;

    TIM1_BDTR=0U;
    TIM1_CCER=0U;
    TIM1_CR1=0U;

    BSRR(GA)=MOTOR_PINS<<16U;

    OTYPER(GA)&=~MOTOR_PINS;

    PUPDR(GA)=
        (PUPDR(GA)&~MOTOR_MODE_MASK)|
        (0xAAU<<16U);

    MODER(GA)=
        (MODER(GA)&~MOTOR_MODE_MASK)|
        MOTOR_OUTPUT_MODES;

    TIM1_CCR1=0U;
    TIM1_CCR2=0U;
    TIM1_CCR3=0U;
    TIM1_CCR4=0U;

    TIM1_EGR=1U;
}

void SystemInit(void)
{
    motor_hardware_off();
}

static uint32_t motor_off_verified(void)
{
    return (
        ((MODER(GA)&MOTOR_MODE_MASK)
            ==MOTOR_OUTPUT_MODES) &&

        ((OTYPER(GA)&MOTOR_PINS)==0U) &&

        ((ODR(GA)&MOTOR_PINS)==0U) &&

        (TIM1_BDTR==0U) &&
        (TIM1_CCER==0U) &&
        ((TIM1_CR1&1U)==0U) &&

        (TIM1_CCR1==0U) &&
        (TIM1_CCR2==0U) &&
        (TIM1_CCR3==0U) &&
        (TIM1_CCR4==0U)
    )?1U:0U;
}

static void motor_all_off(void)
{
    motor_hardware_off();

    motor1_percent=0U;
    motor2_percent=0U;
    motor3_percent=0U;
    motor4_percent=0U;

    physical_motor_lock=1U;
    outputs_verified_off=motor_off_verified();
}

/* ====================================================
 * ZERO ALL COMPUTED COMMANDS
 * ==================================================== */

static void sim_zero(void)
{
    sim_ready=0U;
    sim_link_ok=0U;
    sim_inputs_ok=0U;

    sim_roll_stick=0.0f;
    sim_pitch_stick=0.0f;
    sim_yaw_stick=0.0f;

    sim_roll_target_dps=0.0f;
    sim_pitch_target_dps=0.0f;
    sim_yaw_target_dps=0.0f;

    sim_roll_error_dps=0.0f;
    sim_pitch_error_dps=0.0f;
    sim_yaw_error_dps=0.0f;

    pid_roll_output=0.0f;
    pid_pitch_output=0.0f;
    pid_yaw_output=0.0f;

    mix_m1_percent=0.0f;
    mix_m2_percent=0.0f;
    mix_m3_percent=0.0f;
    mix_m4_percent=0.0f;
}

/* ====================================================
 * FAILSAFE
 * ==================================================== */

static void fail(uint32_t code)
{
    motor_all_off();

    if(fault_code==0U)
        fault_code=code;

    core_state=4U;
    fc_state=4U;

    sim_zero();
}

void HardFault_Handler(void)
{
    motor_hardware_off();

    fault_code=12U;
    fc_state=4U;

    for(;;){}
}

void NMI_Handler(void)
{
    motor_hardware_off();

    fault_code=13U;
    fc_state=4U;

    for(;;){}
}

/* ====================================================
 * CLOCK AND WATCHDOG
 * ==================================================== */

void SysTick_Handler(void)
{
    system_ms++;
}

static void delay_ms(uint32_t ms)
{
    uint32_t t=system_ms;

    while((uint32_t)(system_ms-t)<ms){}
}

static uint32_t clock_init(void)
{
    uint32_t budget=200000U;

    RCC_CR|=1U;

    while(!(RCC_CR&2U))
    {
        if(--budget==0U)
            return 0U;
    }

    RCC_CFGR&=~3U;
    budget=200000U;

    while(RCC_CFGR&(3U<<2U))
    {
        if(--budget==0U)
            return 0U;
    }

    RCC_CFGR&=~(
        (0xFU<<4U)|
        (7U<<8U)
    );

    SYST_RVR=7999U;
    SYST_CVR=0U;
    SYST_CSR=7U;

    return 1U;
}

static uint32_t watchdog_init(void)
{
    uint32_t t;

    RCC_APB2ENR|=(1U<<22U);
    (void)RCC_APB2ENR;

    DBGMCU_APB1_FZ|=(1U<<12U);

    IWDG_KR=0xCCCCU;
    IWDG_KR=0x5555U;

    IWDG_PR=4U;
    IWDG_RLR=624U;

    t=system_ms;

    while(IWDG_SR&7U)
    {
        if((uint32_t)(system_ms-t)>=100U)
            return 0U;
    }

    IWDG_WINR=0xFFFU;

    t=system_ms;

    while(IWDG_SR&7U)
    {
        if((uint32_t)(system_ms-t)>=100U)
            return 0U;
    }

    IWDG_KR=0xAAAAU;

    return 1U;
}

static void watchdog_feed(void)
{
    if(motor_off_verified())
    {
        IWDG_KR=0xAAAAU;
        iwdg_refresh_counter++;
    }
}

/* ====================================================
 * M540 I2C
 * ==================================================== */

static void i2c_init(void)
{
    RCC_AHBENR|=(1U<<18U);

    MODER(GB)=
        (MODER(GB)&~(
            (3U<<14U)|
            (3U<<16U)
        ))|
        ((2U<<14U)|(2U<<16U));

    OTYPER(GB)|=(1U<<7U)|(1U<<8U);

    OSPEEDR(GB)|=(3U<<14U)|(3U<<16U);

    PUPDR(GB)=
        (PUPDR(GB)&~(
            (3U<<14U)|
            (3U<<16U)
        ))|
        ((1U<<14U)|(1U<<16U));

    AFRL(GB)=
        (AFRL(GB)&~(0xFU<<28U))|
        (1U<<28U);

    AFRH(GB)=
        (AFRH(GB)&~0xFU)|1U;

    RCC_CFGR3&=~(1U<<4U);

    RCC_APB1ENR|=(1U<<21U);

    RCC_APB1RSTR|=(1U<<21U);
    RCC_APB1RSTR&=~(1U<<21U);

    I2C_CR1=0U;

    I2C_TIMING=0x10420F13U;

    I2C_ICR=
        (1U<<4U)|
        (1U<<5U)|
        (1U<<8U)|
        (1U<<9U)|
        (1U<<10U);

    I2C_CR1=1U;
}

static uint32_t i2c_wait(uint32_t flag)
{
    for(;;)
    {
        uint32_t sr=I2C_ISR;

        if(sr&(1U<<8U))
            I2C_ICR=(1U<<8U);

        if(sr&(
            (1U<<4U)|
            (1U<<9U)|
            (1U<<10U)
        ))
            return 0U;

        if(sr&flag)
            return 1U;

        if((uint32_t)(
            system_ms-i2c_start_ms
        )>=4U)
            return 0U;
    }
}

static uint32_t i2c_not_busy(void)
{
    while(I2C_ISR&(1U<<15U))
    {
        if((uint32_t)(
            system_ms-i2c_start_ms
        )>=4U)
            return 0U;
    }

    return 1U;
}

static uint32_t imu_write(
    uint8_t reg,
    uint8_t value
)
{
    i2c_start_ms=system_ms;

    if(!i2c_not_busy())
        return 0U;

    I2C_ICR=
        (1U<<4U)|
        (1U<<5U)|
        (1U<<8U)|
        (1U<<9U)|
        (1U<<10U);

    I2C_CR2=
        (0x69U<<1U)|
        (2U<<16U)|
        (1U<<25U)|
        (1U<<13U);

    if(!i2c_wait(1U<<1U))
        return 0U;

    I2C_TXDR=reg;

    if(!i2c_wait(1U<<1U))
        return 0U;

    I2C_TXDR=value;

    if(!i2c_wait(1U<<5U))
        return 0U;

    I2C_ICR=(1U<<5U);

    return 1U;
}

static uint32_t imu_read(
    uint8_t reg,
    uint8_t *buf,
    uint32_t count
)
{
    uint32_t i;

    if(count==0U || count>255U)
        return 0U;

    i2c_start_ms=system_ms;

    if(!i2c_not_busy())
        return 0U;

    I2C_ICR=
        (1U<<4U)|
        (1U<<5U)|
        (1U<<8U)|
        (1U<<9U)|
        (1U<<10U);

    I2C_CR2=
        (0x69U<<1U)|
        (1U<<16U)|
        (1U<<13U);

    if(!i2c_wait(1U<<1U))
        return 0U;

    I2C_TXDR=reg;

    if(!i2c_wait(1U<<6U))
        return 0U;

    I2C_CR2=
        (0x69U<<1U)|
        (count<<16U)|
        (1U<<10U)|
        (1U<<25U)|
        (1U<<13U);

    for(i=0U;i<count;i++)
    {
        if(!i2c_wait(1U<<2U))
            return 0U;

        buf[i]=(uint8_t)I2C_RXDR;
    }

    if(!i2c_wait(1U<<5U))
        return 0U;

    I2C_ICR=(1U<<5U);

    return 1U;
}

/* ====================================================
 * M540 INITIALIZATION
 * ==================================================== */

static uint32_t imu_init(void)
{
    uint8_t who=0U;

    if(!imu_write(0x6BU,0x80U))
        return 0U;

    delay_ms(100U);

    if(!imu_write(0x6BU,0x01U))
        return 0U;

    delay_ms(20U);

    if(!imu_write(0x1CU,0x18U))
        return 0U;

    if(!imu_write(0x1DU,0x05U))
        return 0U;

    if(!imu_write(0x1BU,0x18U))
        return 0U;

    if(!imu_write(0x1AU,0x00U))
        return 0U;

    delay_ms(20U);

    if(!imu_read(0x75U,&who,1U))
        return 0U;

    imu_whoami=who;

    return who==0x7DU?1U:0U;
}

/* ====================================================
 * IMU SAMPLE
 * ==================================================== */

static uint32_t imu_sample(void)
{
    uint8_t d[14];

    if(!imu_read(0x3BU,d,14U))
        return 0U;

    accel_x=(int16_t)(
        ((uint16_t)d[0]<<8U)|d[1]);

    accel_y=(int16_t)(
        ((uint16_t)d[2]<<8U)|d[3]);

    accel_z=(int16_t)(
        ((uint16_t)d[4]<<8U)|d[5]);

    gyro_x=(int16_t)(
        ((uint16_t)d[8]<<8U)|d[9]);

    gyro_y=(int16_t)(
        ((uint16_t)d[10]<<8U)|d[11]);

    gyro_z=(int16_t)(
        ((uint16_t)d[12]<<8U)|d[13]);

    imu_frame_counter++;

    if(
        accel_x==0 &&
        accel_y==0 &&
        accel_z==0 &&
        gyro_x==0 &&
        gyro_y==0 &&
        gyro_z==0
    )
        return 0U;

    if(
        gyro_x>=32760 || gyro_x<=-32760 ||
        gyro_y>=32760 || gyro_y<=-32760 ||
        gyro_z>=32760 || gyro_z<=-32760
    )
        return 0U;

    return 1U;
}

/* ====================================================
 * GYRO CALIBRATION
 * ==================================================== */

static void calibration_start(void)
{
    uint32_t i;

    for(i=0U;i<3U;i++)
    {
        gyro_sum[i]=0;
        gyro_min[i]=INT16_MAX;
        gyro_max[i]=INT16_MIN;
    }

    calibration_status=1U;
    calibration_samples=0U;

    calib_start_ms=system_ms;

    core_state=1U;
    fc_state=1U;
}

static void calibration_update(void)
{
    int16_t v[3]={
        gyro_x,
        gyro_y,
        gyro_z
    };

    uint32_t i;

    if((uint32_t)(
        system_ms-calib_start_ms
    )>4000U)
    {
        calibration_status=3U;
        fail(2U);
        return;
    }

    for(i=0U;i<3U;i++)
    {
        if(v[i]<gyro_min[i])
            gyro_min[i]=v[i];

        if(v[i]>gyro_max[i])
            gyro_max[i]=v[i];

        if(
            (int32_t)gyro_max[i]-
            (int32_t)gyro_min[i]>164 ||

            v[i]>1312 ||
            v[i]<-1312
        )
        {
            calibration_status=3U;
            fail(2U);
            return;
        }

        gyro_sum[i]+=v[i];
    }

    calibration_samples++;

    if(calibration_samples==CAL_COUNT)
    {
        gyro_x_offset=
            (float)gyro_sum[0]/CAL_COUNT;

        gyro_y_offset=
            (float)gyro_sum[1]/CAL_COUNT;

        gyro_z_offset=
            (float)gyro_sum[2]/CAL_COUNT;

        calibration_status=2U;

        core_state=2U;
        fc_state=2U;
    }
}

/* ====================================================
 * RADIO SPI INITIALIZATION
 * ==================================================== */

static void spi_init(void)
{
    RCC_AHBENR|=
        (1U<<17U)|
        (1U<<18U);

    RCC_APB2ENR|=(1U<<12U);

    (void)RCC_AHBENR;
    (void)RCC_APB2ENR;

    /* CSN HIGH, CE LOW */

    BSRR(GA)=(1U<<15U);
    BSRR(GB)=(1U<<22U);

    OTYPER(GA)&=~(1U<<15U);

    OTYPER(GB)&=~(
        (1U<<3U)|
        (1U<<5U)|
        (1U<<6U)
    );

    MODER(GA)=
        (MODER(GA)&~(3U<<30U))|
        (1U<<30U);

    PUPDR(GA)&=~(3U<<30U);

    MODER(GB)=
        (MODER(GB)&~(
            (3U<<6U)|
            (3U<<8U)|
            (3U<<10U)|
            (3U<<12U)
        ))|
        (
            (2U<<6U)|
            (2U<<8U)|
            (2U<<10U)|
            (1U<<12U)
        );

    PUPDR(GB)&=~(
        (3U<<6U)|
        (3U<<8U)|
        (3U<<10U)|
        (3U<<12U)
    );

    OSPEEDR(GB)|=
        (3U<<6U)|
        (3U<<10U);

    /* AF0 = SPI1 */

    AFRL(GB)&=~(
        (0xFU<<12U)|
        (0xFU<<16U)|
        (0xFU<<20U)
    );

    SPI_CR1=0U;

    SPI_CR2=
        (7U<<8U)|
        (1U<<12U);

    SPI_CR1=
        (1U<<2U)|
        (4U<<3U)|
        (1U<<8U)|
        (1U<<9U)|
        (1U<<6U);

    radio_spi_initialized=1U;
}

/* ====================================================
 * RADIO SPI TRANSFERS
 * ==================================================== */

static uint32_t spi_wait(
    uint32_t flag,
    uint32_t value,
    uint32_t error
)
{
    uint32_t start=system_ms;

    for(;;)
    {
        if(
            ((SPI_SR&flag)!=0U)==
            (value!=0U)
        )
            return 1U;

        if((uint32_t)(
            system_ms-start
        )>=2U)
        {
            radio_spi_last_error=error;
            return 0U;
        }
    }
}

static uint32_t spi_byte(
    uint8_t tx,
    uint8_t *rx
)
{
    if(!spi_wait(2U,1U,1U))
        return 0U;

    SPI_DR8=tx;

    if(!spi_wait(1U,1U,2U))
        return 0U;

    *rx=SPI_DR8;

    return 1U;
}

static uint32_t spi_xfer(
    const uint8_t *tx,
    uint8_t *rx,
    uint32_t n
)
{
    uint32_t i;
    uint32_t ok=1U;

    if(n==0U || n>11U)
        return 0U;

    /* CSN LOW */

    BSRR(GA)=(1U<<31U);

    for(i=0U;i<n;i++)
    {
        uint8_t value=0U;

        if(!spi_byte(tx[i],&value))
        {
            ok=0U;
            break;
        }

        if(rx)
            rx[i]=value;
    }

    if(ok)
        ok=spi_wait(1U<<7U,0U,3U);

    /* Always release CSN */

    BSRR(GA)=(1U<<15U);

    if(SPI_SR&(1U<<6U))
    {
        volatile uint8_t d=SPI_DR8;
        volatile uint32_t s=SPI_SR;

        (void)d;
        (void)s;

        radio_spi_last_error=4U;
        ok=0U;
    }

    if(!ok)
        radio_spi_error_counter++;

    return ok;
}

static uint32_t reg_write(
    uint8_t addr,
    const uint8_t *bytes,
    uint32_t count
)
{
    uint8_t tx[11];
    uint32_t i;

    if(count==0U || count>10U)
        return 0U;

    tx[0]=(uint8_t)(
        0x20U|(addr&0x1FU));

    for(i=0U;i<count;i++)
        tx[i+1U]=bytes[i];

    return spi_xfer(tx,0,count+1U);
}

static uint32_t reg_wr1(
    uint8_t addr,
    uint8_t val
)
{
    return reg_write(addr,&val,1U);
}

static uint32_t reg_read(
    uint8_t addr,
    uint8_t *val
)
{
    uint8_t tx[2]={
        (uint8_t)(addr&0x1FU),
        0xFFU
    };

    uint8_t rx[2];

    if(!spi_xfer(tx,rx,2U))
        return 0U;

    *val=rx[1];

    return 1U;
}

static uint32_t radio_cmd(uint8_t val)
{
    return spi_xfer(&val,0,1U);
}

/* ====================================================
 * NRF24 RECEIVER INITIALIZATION
 * ==================================================== */

static void rx_init(void)
{
    static const uint8_t registers[10]={
        0x00U,0x01U,0x02U,0x03U,0x04U,
        0x05U,0x06U,0x11U,0x1CU,0x1DU
    };

    static const uint8_t values[10]={
        0x0CU,0x00U,0x01U,0x03U,0x00U,
        0x2DU,0x07U,0x09U,0x00U,0x00U
    };

    static const uint8_t address[5]={
        0xCC,0xCC,0xCC,0xCC,0xCC
    };

    uint32_t i;
    uint8_t got=0U;

    radio_rx_configured=0U;
    radio_rx_config_error=0U;

    /* CE LOW */

    BSRR(GB)=(1U<<22U);

    for(i=0U;i<10U;i++)
    {
        if(
            !reg_wr1(
                registers[i],
                values[i]
            ) ||

            !reg_read(
                registers[i],
                &got
            ) ||

            got!=values[i]
        )
        {
            radio_rx_config_error=
                0x100U|registers[i];

            return;
        }
    }

    if(!reg_write(0x0AU,address,5U))
    {
        radio_rx_config_error=0x10AU;
        return;
    }

    if(!reg_write(0x10U,address,5U))
    {
        radio_rx_config_error=0x110U;
        return;
    }

    if(
        !radio_cmd(0xE2U) ||
        !reg_wr1(0x07U,0x70U)
    )
    {
        radio_rx_config_error=0x1E2U;
        return;
    }

    if(
        !reg_wr1(0x00U,0x0FU) ||

        !reg_read(0x00U,&got) ||

        got!=0x0FU
    )
    {
        radio_rx_config_error=0x200U;
        return;
    }

    delay_ms(2U);

    /* CE HIGH: RX MODE */

    BSRR(GB)=(1U<<6U);

    radio_rx_configured=1U;
    radio_spi_ok=1U;
}

/* ====================================================
 * VALIDATE RECEIVED PACKET
 * ==================================================== */

static void radio_accept(
    const uint8_t *b,
    uint32_t now
)
{
    uint32_t i;
    uint32_t sum=0x76U;
    uint32_t seq=b[7];

    for(i=0U;i<8U;i++)
        sum+=b[i];

    if(
        (uint8_t)sum!=b[8] ||

        b[4]!=0x05U ||
        b[5]!=0x38U ||
        b[6]!=0x00U ||

        b[1]>225U ||
        b[2]>225U ||
        b[3]>225U
    )
    {
        radio_rx_invalid_packets++;
        return;
    }

    if(radio_diag_alive && have_seq)
    {
        uint32_t delta=(uint8_t)(
            seq-(uint8_t)previous_seq
        );

        if(delta==0U)
        {
            radio_diag_duplicate_count++;
            return;
        }

        if(delta>=128U)
        {
            radio_rx_invalid_packets++;
            return;
        }

        if(delta>1U)
        {
            radio_diag_seq_gap_count+=
                delta-1U;
        }

        radio_diag_period_ms=
            (uint32_t)(
                now-last_period_ms
            );
    }
    else
    {
        if(ever_seen)
            radio_diag_reconnect_count++;

        radio_diag_period_ms=0U;
    }

    previous_seq=seq;
    have_seq=1U;

    radio_diag_last_seq=seq;

    last_period_ms=now;
    last_valid_ms=now;

    ever_seen=1U;

    radio_diag_alive=1U;
    radio_diag_age_ms=0U;

    radio_diag_throttle=b[0];
    radio_diag_yaw=b[1];
    radio_diag_roll=b[2];
    radio_diag_pitch=b[3];

    radio_rx_throttle_raw=b[0];
    radio_rx_yaw_raw=b[1];
    radio_rx_roll_raw=b[2];
    radio_rx_pitch_raw=b[3];

    radio_rx_last_packet_ms=now;

    radio_packets_received++;

    for(i=0U;i<9U;i++)
        radio_last_packet[i]=b[i];
}

/* ====================================================
 * RADIO POLLING
 * ==================================================== */

static void radio_poll(uint32_t now)
{
    uint8_t status=0U;
    uint8_t fifo=0U;

    uint8_t tx[10];
    uint8_t rx[10];
    uint8_t b[9];

    uint32_t i;

    if(!radio_rx_configured)
        return;

    if((uint32_t)(
        now-last_radio_poll_ms
    )<RADIO_POLL_MS)
        return;

    last_radio_poll_ms=now;

    radio_rx_poll_counter++;

    if(
        !reg_read(0x07U,&status) ||
        !reg_read(0x17U,&fifo)
    )
    {
        radio_rx_read_errors++;
        return;
    }

    radio_rx_status=status;
    radio_rx_fifo_status=fifo;

    /* RX FIFO EMPTY */

    if(fifo&1U)
        return;

    /* R_RX_PAYLOAD */

    tx[0]=0x61U;

    for(i=1U;i<10U;i++)
        tx[i]=0xFFU;

    if(!spi_xfer(tx,rx,10U))
    {
        radio_rx_read_errors++;
        return;
    }

    for(i=0U;i<9U;i++)
        b[i]=rx[i+1U];

    radio_rx_raw_packets++;

    radio_accept(b,now);

    /* Clear RX_DR */

    if(!reg_wr1(0x07U,0x40U))
        radio_rx_read_errors++;
}

/* ====================================================
 * RADIO LINK TIMEOUT
 * ==================================================== */

static void link_update(uint32_t now)
{
    if(!ever_seen)
    {
        radio_diag_age_ms=UINT32_MAX;
        return;
    }

    radio_diag_age_ms=
        (uint32_t)(now-last_valid_ms);

    if(
        radio_diag_alive &&
        radio_diag_age_ms>=RADIO_TIMEOUT_MS
    )
    {
        radio_diag_alive=0U;
        radio_diag_lost_count++;

        have_seq=0U;

        radio_diag_throttle=225U;
        radio_diag_yaw=141U;
        radio_diag_roll=112U;
        radio_diag_pitch=112U;

        sim_zero();
    }
}

/* ====================================================
 * NORMALIZE JOYSTICK AXES
 * ==================================================== */

/*
 * Convert 0..225 to -1.0 .. +1.0.
 *
 * Yaw center:   141
 * Roll center:  112
 * Pitch center: 112
 *
 * Deadband: +-3 raw units.
 */

static float axis_normalize(
    uint32_t value,
    uint32_t middle
)
{
    float ratio;

    if(value>225U)
        return 0.0f;

    if(
        value>=middle-3U &&
        value<=middle+3U
    )
        return 0.0f;

    if(value>=middle)
    {
        ratio=
            (float)(value-middle)/
            (float)(225U-middle);
    }
    else
    {
        ratio=
            -((float)(middle-value)/
              (float)middle);
    }

    return clampf(ratio,-1.0f,1.0f);
}

/* ====================================================
 * STEP 11: RATE CONTROLLER AND MIXER
 * COMPUTATION ONLY
 * ==================================================== */

static void simulation_update(void)
{
    float r,p,y;

    sim_update_counter++;

    if(
        core_state!=2U ||
        imu_ok!=1U ||
        calibration_status!=2U ||

        !radio_diag_alive ||

        radio_diag_age_ms>=RADIO_TIMEOUT_MS
    )
    {
        sim_zero();
        return;
    }

    sim_link_ok=1U;

    /*
     * Throttle MUST remain locked at 0xE1.
     * If not, reject simulation commands.
     */

    if(
        radio_diag_throttle!=225U ||

        radio_diag_yaw>225U ||
        radio_diag_roll>225U ||
        radio_diag_pitch>225U
    )
    {
        sim_zero();
        return;
    }

    sim_inputs_ok=1U;

    /* M540 measured angular rates */

    gyro_roll_dps=
        ((float)gyro_x-gyro_x_offset)/
        16.4f;

    gyro_pitch_dps=
        ((float)gyro_y-gyro_y_offset)/
        16.4f;

    gyro_yaw_dps=
        ((float)gyro_z-gyro_z_offset)/
        16.4f;

    if(
        absf(gyro_roll_dps)>4000.0f ||

        absf(gyro_pitch_dps)>4000.0f ||

        absf(gyro_yaw_dps)>4000.0f
    )
    {
        sim_zero();
        fail(14U);
        return;
    }

    /* Normalize received sticks */

    sim_roll_stick=axis_normalize(
        radio_diag_roll,112U);

    sim_pitch_stick=axis_normalize(
        radio_diag_pitch,112U);

    sim_yaw_stick=axis_normalize(
        radio_diag_yaw,141U);

    /* Desired angular rates */

    sim_roll_target_dps=
        SIM_MAX_ROLL_DPS*sim_roll_stick;

    sim_pitch_target_dps=
        SIM_MAX_PITCH_DPS*sim_pitch_stick;

    sim_yaw_target_dps=
        SIM_MAX_YAW_DPS*sim_yaw_stick;

    /* Rate errors */

    sim_roll_error_dps=
        sim_roll_target_dps-
        gyro_roll_dps;

    sim_pitch_error_dps=
        sim_pitch_target_dps-
        gyro_pitch_dps;

    sim_yaw_error_dps=
        sim_yaw_target_dps-
        gyro_yaw_dps;

    /*
     * Proportional rate controller.
     * This is not a complete flight PID.
     */

    pid_roll_output=clampf(
        0.060f*sim_roll_error_dps,
        -8.0f,8.0f);

    pid_pitch_output=clampf(
        0.060f*sim_pitch_error_dps,
        -8.0f,8.0f);

    pid_yaw_output=clampf(
        0.050f*sim_yaw_error_dps,
        -6.0f,6.0f);

    r=pid_roll_output;
    p=pid_pitch_output;
    y=pid_yaw_output;

    /*
     * Provisional mixer signs.
     * Physical directions are not verified.
     * NO PWM IS WRITTEN.
     */

    mix_m1_percent=clampf(
        SIM_BASE_PERCENT+r+p+y,
        0.0f,100.0f);

    mix_m2_percent=clampf(
        SIM_BASE_PERCENT-r+p-y,
        0.0f,100.0f);

    mix_m3_percent=clampf(
        SIM_BASE_PERCENT-r-p+y,
        0.0f,100.0f);

    mix_m4_percent=clampf(
        SIM_BASE_PERCENT+r-p-y,
        0.0f,100.0f);

    sim_ready=1U;
}

/* ====================================================
 * MAIN CONTROL CYCLE
 * ==================================================== */

static void control_cycle(uint32_t elapsed)
{
    uint32_t observed=fc_state;

    uint32_t off_before=
        motor_off_verified();

    motor_all_off();

    loop_counter++;
    loop_dt_ms=elapsed;

    if(!off_before || !outputs_verified_off)
        fail(5U);

    if(elapsed>6U)
    {
        loop_overrun_counter++;
        fail(4U);
    }

    if(observed!=core_state)
        fail(99U);

    /* Always reject ARM */

    if(arm_request!=0U)
    {
        arm_request=0U;
        arm_reject_counter++;
        arm_block_flags=1U;
    }

    if(core_state==4U)
    {
        sim_zero();
        return;
    }

    if(
        core_state!=1U &&
        core_state!=2U
    )
    {
        fail(99U);
        return;
    }

    if(!imu_sample())
    {
        imu_ok=0U;
        imu_error_counter++;

        fail(3U);
        return;
    }

    imu_ok=1U;

    if(core_state==1U)
        calibration_update();
}

/* ====================================================
 * MAIN
 * ==================================================== */

int main(void)
{
    uint32_t last_loop;

    motor_all_off();

    reset_cause=RCC_CSR;

    watchdog_reset_detected=
        (reset_cause&(1U<<29U))?
        1U:0U;

    RCC_CSR|=(1U<<24U);

    if(!clock_init())
    {
        fail(10U);

        for(;;)
            motor_hardware_off();
    }

    if(!watchdog_init())
    {
        fail(11U);

        for(;;)
            motor_hardware_off();
    }

    RCC_AHBENR|=(1U<<18U);

    /* Original board enable PA1 */

    BSRR(GA)=(1U<<1U);

    OTYPER(GA)&=~(1U<<1U);

    MODER(GA)=
        (MODER(GA)&~(3U<<2U))|
        (1U<<2U);

    /* Radio initialization */

    spi_init();
    rx_init();

    /* IMU initialization */

    i2c_init();
    delay_ms(20U);

    if(watchdog_reset_detected)
    {
        fail(8U);
    }
    else if(!imu_init())
    {
        imu_error_counter++;
        fail(1U);
    }
    else
    {
        imu_ok=1U;
        calibration_start();
    }

    motor_all_off();
    watchdog_feed();

    last_loop=system_ms;

    for(;;)
    {
        uint32_t now=system_ms;

        uint32_t elapsed=
            (uint32_t)(now-last_loop);

        if(elapsed<LOOP_PERIOD_MS)
            continue;

        last_loop=now;

        control_cycle(elapsed);

        radio_poll(system_ms);

        link_update(system_ms);

        simulation_update();

        loop_exec_ms=
            (uint32_t)(system_ms-now);

        if(loop_exec_ms>loop_max_exec_ms)
            loop_max_exec_ms=loop_exec_ms;

        if(loop_exec_ms>=LOOP_PERIOD_MS)
        {
            loop_overrun_counter++;
            fail(4U);
        }

        motor_all_off();

        if(!outputs_verified_off)
            fail(5U);

        watchdog_feed();
    }
}
