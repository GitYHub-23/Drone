
/*
 * DRONE_ STEP 31.2 -- STM32F031K4Ux, 8 MHz HSI
 * COMPLETE main.c (replaces Step 26).
 *
 * Four brushed motors:
 * M1 PA11 / TIM1_CH4  front right
 * M2 PA9  / TIM1_CH2  front left
 * M3 PA8  / TIM1_CH1  rear left
 * M4 PA10 / TIM1_CH3  rear right
 *
 * NRF24: PA15 CSN, PB3 SCK, PB4 MISO, PB5 MOSI, PB6 CE.
 * IMU: PB7 SDA, PB8 SCL, I2C 0x69, WHO_AM_I 0x7D.
 *
 * Keep the 1 KB calibration flash page reserved at 0x08003C00.
 * Keep the Step 26 radio packet format and motor mixer directions.
 * No battery-voltage sensing (no divider connected).
 *
 * BENCH/TEST FIRMWARE. NOT QUALIFIED FOR FREE FLIGHT.
 * First motor tests without propellers, on fixed frame, with physical
 * motor-battery disconnect. Never pause debugger with motor power on.
 */

#include <stdint.h>

#define REG(a) (*(volatile uint32_t *)(uintptr_t)(a))

#define RCC_CR       REG(0x40021000U)
#define RCC_CFGR     REG(0x40021004U)
#define RCC_AHBENR   REG(0x40021014U)
#define RCC_APB2ENR  REG(0x40021018U)
#define RCC_APB1ENR  REG(0x4002101CU)
#define RCC_APB1RSTR REG(0x40021010U)
#define RCC_CFGR3    REG(0x40021030U)
#define RCC_CSR      REG(0x40021024U)

#define GA 0x48000000U
#define GB 0x48000400U

#define MODER(b)   REG((b)+0x00U)
#define OTYPER(b)  REG((b)+0x04U)
#define OSPEEDR(b) REG((b)+0x08U)
#define PUPDR(b)   REG((b)+0x0CU)
#define BSRR(b)    REG((b)+0x18U)
#define AFRL(b)    REG((b)+0x20U)
#define AFRH(b)    REG((b)+0x24U)

#define SYST_CSR REG(0xE000E010U)
#define SYST_RVR REG(0xE000E014U)
#define SYST_CVR REG(0xE000E018U)

#define TIM1_CR1   REG(0x40012C00U)
#define TIM1_EGR   REG(0x40012C14U)
#define TIM1_CCMR1 REG(0x40012C18U)
#define TIM1_CCMR2 REG(0x40012C1CU)
#define TIM1_CCER  REG(0x40012C20U)
#define TIM1_PSC   REG(0x40012C28U)
#define TIM1_ARR   REG(0x40012C2CU)
#define TIM1_CCR1  REG(0x40012C34U)
#define TIM1_CCR2  REG(0x40012C38U)
#define TIM1_CCR3  REG(0x40012C3CU)
#define TIM1_CCR4  REG(0x40012C40U)
#define TIM1_BDTR  REG(0x40012C44U)

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

#define IWDG_KR REG(0x40003000U)
#define IWDG_PR REG(0x40003004U)
#define IWDG_RLR REG(0x40003008U)
#define IWDG_SR REG(0x4000300CU)
#define IWDG_WINR REG(0x40003010U)

#define DBGMCU_APB1_FZ REG(0x40015808U)

#define MOTOR_MASK       (0xFU<<8U)
#define MOTOR_MODE_MASK  (0xFFU<<16U)
#define MOTOR_GPIO_MODE  (0x55U<<16U)
#define MOTOR_AF_MODE    (0xAAU<<16U)

#define LOOP_PERIOD_MS 10U
#define LOOP_WARN_MS 12U
#define LOOP_LIMIT_MS 25U

#define RADIO_POLL_MS 10U
#define RADIO_TIMEOUT_MS 150U
#define RADIO_ARM_MAX_AGE_MS 80U

#define GYRO_CAL_SAMPLES 250U

#define MAX_ANGLE_DEG 20.0f
#define MAX_RATE_DPS 90.0f
#define MAX_YAW_DPS 60.0f

#define ANGLE_KP 3.0f
#define ROLL_KP 0.060f
#define PITCH_KP 0.060f
#define YAW_KP 0.050f

#define MAX_CORRECTION_PERCENT 6.0f
#define MAX_YAW_CORRECTION_PERCENT 5.0f

#define MOTOR_MAX_PERCENT 55.0f
#define MOTOR_SLEW_PM 12U
#define MOTOR_START_THR_PERCENT 4.0f

#define ARM_HOLD_MS 1200U
#define DISARM_HOLD_MS 400U
#define ARM_YAW_RAW 210U
#define DISARM_YAW_RAW 30U

#define ARM_MAX_TILT_DEG 10.0f
#define MOTOR_MAX_TILT_DEG 70.0f

#define FILTER_ALPHA 0.98f
#define GYRO_MONITOR_ALPHA 0.70f
#define RAD_TO_DEG 57.2957795f

#define CAL_FLASH_ADDR 0x08003C00U
#define CAL_MAGIC 0x4D54U
#define CAL_VERSION 1U
#define CAL_COMMIT 0xA55AU

#define FC_BOOT 0U
#define FC_CALIBRATING 1U
#define FC_DISARMED 2U
#define FC_ARMED 3U
#define FC_FAULT 4U

#define DISARM_NONE 0U
#define DISARM_STICK 1U
#define DISARM_RADIO 2U
#define DISARM_IMU 3U
#define DISARM_TILT 4U
#define DISARM_DEBUG 5U
#define DISARM_TIMING 6U

extern const uint8_t __drone_flash_cal_boundary__;

/* LIVE EXPRESSIONS */

volatile uint32_t system_ms=0U;

volatile uint32_t fc_state=FC_BOOT;
volatile uint32_t fault_code=0U;

volatile uint32_t arm_state=0U;
volatile uint32_t sim_arm_state=0U;

volatile uint32_t disarm_request=0U;
volatile uint32_t arm_counter=0U;
volatile uint32_t disarm_counter=0U;
volatile uint32_t last_disarm_reason=DISARM_NONE;

volatile uint32_t arm_gesture_progress_ms=0U;
volatile uint32_t disarm_gesture_progress_ms=0U;
volatile uint32_t arm_rearm_latch=1U;

volatile uint32_t physical_motor_lock=1U;
volatile uint32_t outputs_verified_off=1U;
volatile uint32_t motor_pwm_active=0U;

volatile uint32_t motor1_pwm_permille=0U;
volatile uint32_t motor2_pwm_permille=0U;
volatile uint32_t motor3_pwm_permille=0U;
volatile uint32_t motor4_pwm_permille=0U;

volatile float mix_m1_percent=0.0f;
volatile float mix_m2_percent=0.0f;
volatile float mix_m3_percent=0.0f;
volatile float mix_m4_percent=0.0f;

volatile float pid_roll_output=0.0f;
volatile float pid_pitch_output=0.0f;
volatile float pid_yaw_output=0.0f;

volatile uint32_t imu_ok=0U;
volatile uint32_t imu_whoami=0U;
volatile uint32_t imu_frame_counter=0U;
volatile uint32_t imu_error_counter=0U;
volatile uint32_t imu_consecutive_errors=0U;

volatile uint32_t imu_last_good_age_ms=UINT32_MAX;
volatile uint32_t imu_read_duration_ms=0U;
volatile uint32_t imu_read_max_ms=0U;

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

volatile float gyro_filtered_roll_dps=0.0f;
volatile float gyro_filtered_pitch_dps=0.0f;
volatile float gyro_filtered_yaw_dps=0.0f;

volatile float attitude_roll_deg=0.0f;
volatile float attitude_pitch_deg=0.0f;

volatile uint32_t attitude_valid=0U;
volatile uint32_t attitude_accel_norm_ok=0U;
volatile uint32_t attitude_tilt_cutoff=0U;

volatile float imu_level_roll_zero_deg=0.0f;
volatile float imu_level_pitch_zero_deg=0.0f;
volatile uint32_t imu_flash_load_status=0U;

volatile uint32_t radio_spi_initialized=0U;
volatile uint32_t radio_spi_error_counter=0U;
volatile uint32_t radio_rx_configured=0U;
volatile uint32_t radio_rx_config_error=0U;

volatile uint32_t radio_rx_invalid_packets=0U;
volatile uint32_t radio_packets_received=0U;

volatile uint32_t radio_diag_alive=0U;
volatile uint32_t radio_diag_age_ms=UINT32_MAX;
volatile uint32_t radio_diag_lost_count=0U;
volatile uint32_t radio_diag_last_seq=0U;

volatile uint32_t radio_diag_throttle=225U;
volatile uint32_t radio_diag_yaw=141U;
volatile uint32_t radio_diag_roll=112U;
volatile uint32_t radio_diag_pitch=112U;

volatile float sim_throttle_percent=0.0f;
volatile float sim_roll_stick=0.0f;
volatile float sim_pitch_stick=0.0f;
volatile float sim_yaw_stick=0.0f;

volatile uint32_t loop_counter=0U;
volatile uint32_t loop_dt_ms=0U;
volatile uint32_t loop_overrun_counter=0U;
volatile uint32_t loop_work_ms=0U;
volatile uint32_t loop_max_work_ms=0U;

volatile uint32_t reset_cause=0U;
volatile uint32_t watchdog_reset_detected=0U;

/* INTERNAL STATE */

static uint32_t i2c_started_ms=0U;
static uint32_t last_radio_poll_ms=0U;
static uint32_t last_valid_radio_ms=0U;
static uint32_t radio_seen=0U;
static uint32_t radio_have_seq=0U;
static uint8_t previous_seq=0U;

static uint32_t gyro_cal_started_ms=0U;
static int32_t gyro_sums[3];
static int16_t gyro_min[3];
static int16_t gyro_max[3];

static uint32_t attitude_initialized=0U;
static uint32_t gyro_monitor_initialized=0U;
static uint32_t last_accel_valid_ms=0U;
static uint32_t last_imu_good_ms=0U;

static uint32_t arm_hold_start_ms=0U;
static uint32_t disarm_hold_start_ms=0U;
static uint32_t arm_hold_active=0U;
static uint32_t disarm_hold_active=0U;

static uint16_t applied_pwm[4]={0U,0U,0U,0U};

/* HELPERS */

static float clampf(float v,float lo,float hi)
{
    if(v<lo) return lo;
    if(v>hi) return hi;
    return v;
}

static float absf(float v)
{
    return v<0.0f?-v:v;
}

static float atan_ratio(float z)
{
    return z*(0.7853981634f+0.273f*(1.0f-z));
}

static float atan2_fast(float y,float x)
{
    float ax=absf(x);
    float ay=absf(y);
    float a;

    if(ax<0.000001f && ay<0.000001f)
        return 0.0f;

    if(ax>=ay)
        a=atan_ratio(ay/ax);
    else
        a=1.5707963268f-atan_ratio(ax/ay);

    if(x<0.0f)
        a=3.1415926536f-a;

    if(y<0.0f)
        a=-a;

    return a*RAD_TO_DEG;
}

static float hypot_fast(float x,float y)
{
    float a=absf(x);
    float b=absf(y);
    float hi=a>b?a:b;
    float lo=a>b?b:a;
    float ratio,n,root;

    if(hi<0.000001f)
        return 0.0f;

    ratio=lo/hi;
    n=1.0f+ratio*ratio;

    root=1.0f+0.4142135624f*ratio;
    root=0.5f*(root+n/root);
    root=0.5f*(root+n/root);

    return hi*root;
}

/* MOTOR HARDWARE */

static void motor_hardware_off(void)
{
    RCC_AHBENR|=(1U<<17);
    RCC_APB2ENR|=(1U<<11);

    (void)RCC_AHBENR;
    (void)RCC_APB2ENR;

    TIM1_BDTR=0U;
    TIM1_CCER=0U;
    TIM1_CR1=0U;

    TIM1_CCR1=0U;
    TIM1_CCR2=0U;
    TIM1_CCR3=0U;
    TIM1_CCR4=0U;

    BSRR(GA)=MOTOR_MASK<<16U;

    OTYPER(GA)&=~MOTOR_MASK;

    PUPDR(GA)=
        (PUPDR(GA)&~MOTOR_MODE_MASK)|
        (0xAAU<<16U);

    MODER(GA)=
        (MODER(GA)&~MOTOR_MODE_MASK)|
        MOTOR_GPIO_MODE;
}

void SystemInit(void)
{
    motor_hardware_off();
}

static void motor_stop(void)
{
    uint32_t i;

    motor_hardware_off();

    for(i=0U;i<4U;i++)
        applied_pwm[i]=0U;

    motor1_pwm_permille=0U;
    motor2_pwm_permille=0U;
    motor3_pwm_permille=0U;
    motor4_pwm_permille=0U;

    motor_pwm_active=0U;
    physical_motor_lock=1U;
    outputs_verified_off=1U;
}

static void motor_pwm_start(void)
{
    motor_hardware_off();

    /* 1 kHz from 8 MHz */
    TIM1_PSC=0U;
    TIM1_ARR=7999U;

    TIM1_CCMR1=0x6868U;
    TIM1_CCMR2=0x6868U;

    TIM1_CCR1=0U;
    TIM1_CCR2=0U;
    TIM1_CCR3=0U;
    TIM1_CCR4=0U;

    TIM1_EGR=1U;

    /* PA8..PA11: AF2 */
    AFRH(GA)=
        (AFRH(GA)&~0xFFFFU)|
        0x2222U;

    MODER(GA)=
        (MODER(GA)&~MOTOR_MODE_MASK)|
        MOTOR_AF_MODE;

    TIM1_CCER=0x1111U;
    TIM1_BDTR=(1U<<15);
    TIM1_CR1=(1U<<7)|1U;

    motor_pwm_active=1U;
    physical_motor_lock=0U;
    outputs_verified_off=0U;
}

static uint16_t slew_pwm(uint16_t old,uint16_t target)
{
    if(target>old &&
       target-old>MOTOR_SLEW_PM)
        return (uint16_t)(old+MOTOR_SLEW_PM);

    if(old>target &&
       old-target>MOTOR_SLEW_PM)
        return (uint16_t)(old-MOTOR_SLEW_PM);

    return target;
}

static uint16_t percent_to_pm(float percent)
{
    return (uint16_t)(
        clampf(percent,0.0f,MOTOR_MAX_PERCENT)*10.0f
    );
}

static void motor_write(float m1,float m2,float m3,float m4)
{
    uint16_t t[4];
    uint32_t i;

    t[0]=percent_to_pm(m1);
    t[1]=percent_to_pm(m2);
    t[2]=percent_to_pm(m3);
    t[3]=percent_to_pm(m4);

    for(i=0U;i<4U;i++)
        applied_pwm[i]=slew_pwm(applied_pwm[i],t[i]);

    if(!motor_pwm_active)
        motor_pwm_start();

    TIM1_CCR1=(uint32_t)applied_pwm[2]*8U; /* M3 */
    TIM1_CCR2=(uint32_t)applied_pwm[1]*8U; /* M2 */
    TIM1_CCR3=(uint32_t)applied_pwm[3]*8U; /* M4 */
    TIM1_CCR4=(uint32_t)applied_pwm[0]*8U; /* M1 */

    motor1_pwm_permille=applied_pwm[0];
    motor2_pwm_permille=applied_pwm[1];
    motor3_pwm_permille=applied_pwm[2];
    motor4_pwm_permille=applied_pwm[3];
}

/* SYSTEM CLOCK AND WATCHDOG */

void SysTick_Handler(void)
{
    system_ms++;
}

static void delay_ms(uint32_t ms)
{
    uint32_t t=system_ms;

    while((uint32_t)(system_ms-t)<ms)
    {
    }
}

static uint32_t clock_init(void)
{
    uint32_t budget=200000U;

    RCC_CR|=1U;

    while(!(RCC_CR&2U))
        if(--budget==0U)
            return 0U;

    RCC_CFGR&=~3U;

    budget=200000U;

    while(RCC_CFGR&(3U<<2))
        if(--budget==0U)
            return 0U;

    RCC_CFGR&=~((0xFU<<4)|(7U<<8));

    SYST_RVR=7999U;
    SYST_CVR=0U;
    SYST_CSR=7U;

    return 1U;
}

static uint32_t watchdog_init(void)
{
    uint32_t t;

    RCC_APB2ENR|=(1U<<22);
    (void)RCC_APB2ENR;

    /*
     * Do not intentionally freeze IWDG in Debug.
     * This does not guarantee safety if a debugger
     * modifies peripheral state while CPU is halted.
     */
    DBGMCU_APB1_FZ&=~(1U<<12);

    IWDG_KR=0xCCCCU;
    IWDG_KR=0x5555U;

    /* ~1 second with nominal 40 kHz LSI */
    IWDG_PR=4U;
    IWDG_RLR=624U;

    t=system_ms;

    while(IWDG_SR&7U)
        if((uint32_t)(system_ms-t)>=100U)
            return 0U;

    IWDG_WINR=0xFFFU;

    t=system_ms;

    while(IWDG_SR&7U)
        if((uint32_t)(system_ms-t)>=100U)
            return 0U;

    IWDG_KR=0xAAAAU;

    return 1U;
}

static void watchdog_feed(void)
{
    DBGMCU_APB1_FZ&=~(1U<<12);
    IWDG_KR=0xAAAAU;
}

static void fail(uint32_t code)
{
    motor_stop();

    if(fault_code==0U)
        fault_code=code;

    arm_state=0U;
    sim_arm_state=0U;
    arm_rearm_latch=1U;

    fc_state=FC_FAULT;
}

void HardFault_Handler(void)
{
    motor_hardware_off();

    for(;;)
    {
    }
}

void NMI_Handler(void)
{
    motor_hardware_off();

    for(;;)
    {
    }
}

/* IMU / I2C */

static void i2c_init(void)
{
    RCC_AHBENR|=(1U<<18);

    MODER(GB)=
        (MODER(GB)&~((3U<<14)|(3U<<16)))|
        ((2U<<14)|(2U<<16));

    OTYPER(GB)|=(1U<<7)|(1U<<8);
    OSPEEDR(GB)|=(3U<<14)|(3U<<16);

    PUPDR(GB)=
        (PUPDR(GB)&~((3U<<14)|(3U<<16)))|
        ((1U<<14)|(1U<<16));

    AFRL(GB)=
        (AFRL(GB)&~(0xFU<<28))|
        (1U<<28);

    AFRH(GB)=
        (AFRH(GB)&~0xFU)|1U;

    RCC_CFGR3&=~(1U<<4);

    RCC_APB1ENR|=(1U<<21);
    RCC_APB1RSTR|=(1U<<21);
    RCC_APB1RSTR&=~(1U<<21);

    I2C_CR1=0U;
    I2C_TIMING=0x10420F13U;

    I2C_ICR=
        (1U<<4)|(1U<<5)|
        (1U<<8)|(1U<<9)|(1U<<10);

    I2C_CR1=1U;
}

static uint32_t i2c_wait(uint32_t flag)
{
    for(;;)
    {
        uint32_t s=I2C_ISR;

        if(s&(1U<<8))
            I2C_ICR=(1U<<8);

        if(s&((1U<<4)|(1U<<9)|(1U<<10)))
            return 0U;

        if(s&flag)
            return 1U;

        if((uint32_t)(
            system_ms-i2c_started_ms
        )>=4U)
            return 0U;
    }
}

static uint32_t i2c_ready(void)
{
    while(I2C_ISR&(1U<<15))
    {
        if((uint32_t)(
            system_ms-i2c_started_ms
        )>=4U)
            return 0U;
    }

    return 1U;
}

static uint32_t imu_write(uint8_t reg,uint8_t value)
{
    i2c_started_ms=system_ms;

    if(!i2c_ready())
        return 0U;

    I2C_ICR=
        (1U<<4)|(1U<<5)|
        (1U<<8)|(1U<<9)|(1U<<10);

    I2C_CR2=
        (0x69U<<1)|
        (2U<<16)|
        (1U<<25)|
        (1U<<13);

    if(!i2c_wait(1U<<1))
        return 0U;

    I2C_TXDR=reg;

    if(!i2c_wait(1U<<1))
        return 0U;

    I2C_TXDR=value;

    if(!i2c_wait(1U<<5))
        return 0U;

    I2C_ICR=(1U<<5);

    return 1U;
}

static uint32_t imu_read(
    uint8_t reg,
    uint8_t *data,
    uint32_t count)
{
    uint32_t i;

    if(count==0U || count>255U)
        return 0U;

    i2c_started_ms=system_ms;

    if(!i2c_ready())
        return 0U;

    I2C_ICR=
        (1U<<4)|(1U<<5)|
        (1U<<8)|(1U<<9)|(1U<<10);

    I2C_CR2=
        (0x69U<<1)|
        (1U<<16)|
        (1U<<13);

    if(!i2c_wait(1U<<1))
        return 0U;

    I2C_TXDR=reg;

    if(!i2c_wait(1U<<6))
        return 0U;

    I2C_CR2=
        (0x69U<<1)|
        (count<<16)|
        (1U<<10)|
        (1U<<25)|
        (1U<<13);

    for(i=0U;i<count;i++)
    {
        if(!i2c_wait(1U<<2))
            return 0U;

        data[i]=(uint8_t)I2C_RXDR;
    }

    if(!i2c_wait(1U<<5))
        return 0U;

    I2C_ICR=(1U<<5);

    return 1U;
}

static uint32_t imu_init(void)
{
    uint8_t who=0U;

    if(!imu_write(0x6B,0x80))
        return 0U;

    delay_ms(100U);

    if(!imu_write(0x6B,0x01))
        return 0U;

    delay_ms(20U);

    if(!imu_write(0x1C,0x18))
        return 0U;

    if(!imu_write(0x1D,0x05))
        return 0U;

    if(!imu_write(0x1B,0x18))
        return 0U;

    if(!imu_write(0x1A,0x00))
        return 0U;

    delay_ms(20U);

    if(!imu_read(0x75,&who,1U))
        return 0U;

    imu_whoami=who;

    return who==0x7DU;
}

static uint32_t imu_sample(void)
{
    uint8_t d[14];

    if(!imu_read(0x3B,d,14U))
        return 0U;

    accel_x=(int16_t)(((uint16_t)d[0]<<8)|d[1]);
    accel_y=(int16_t)(((uint16_t)d[2]<<8)|d[3]);
    accel_z=(int16_t)(((uint16_t)d[4]<<8)|d[5]);

    gyro_x=(int16_t)(((uint16_t)d[8]<<8)|d[9]);
    gyro_y=(int16_t)(((uint16_t)d[10]<<8)|d[11]);
    gyro_z=(int16_t)(((uint16_t)d[12]<<8)|d[13]);

    imu_frame_counter++;

    if(accel_x==0 && accel_y==0 && accel_z==0 &&
       gyro_x==0 && gyro_y==0 && gyro_z==0)
        return 0U;

    if(gyro_x>=32760 || gyro_x<=-32760 ||
       gyro_y>=32760 || gyro_y<=-32760 ||
       gyro_z>=32760 || gyro_z<=-32760)
        return 0U;

    return 1U;
}

/* GYRO CALIBRATION */

static void gyro_cal_start(void)
{
    uint32_t i;

    for(i=0U;i<3U;i++)
    {
        gyro_sums[i]=0;
        gyro_min[i]=INT16_MAX;
        gyro_max[i]=INT16_MIN;
    }

    calibration_status=1U;
    calibration_samples=0U;

    gyro_cal_started_ms=system_ms;
    fc_state=FC_CALIBRATING;
}

static void gyro_cal_update(void)
{
    int16_t v[3]={gyro_x,gyro_y,gyro_z};
    uint32_t i;

    if((uint32_t)(
        system_ms-gyro_cal_started_ms
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

        if((int32_t)gyro_max[i]-
           (int32_t)gyro_min[i]>164 ||
           v[i]>1312 || v[i]<-1312)
        {
            calibration_status=3U;
            fail(2U);
            return;
        }

        gyro_sums[i]+=v[i];
    }

    calibration_samples++;

    if(calibration_samples>=GYRO_CAL_SAMPLES)
    {
        gyro_x_offset=
            (float)gyro_sums[0]/GYRO_CAL_SAMPLES;

        gyro_y_offset=
            (float)gyro_sums[1]/GYRO_CAL_SAMPLES;

        gyro_z_offset=
            (float)gyro_sums[2]/GYRO_CAL_SAMPLES;

        calibration_status=2U;
        fc_state=FC_DISARMED;
    }
}

/* FLASH CALIBRATION */

typedef union {
    float f;
    uint32_t bits;
} float_bits_t;

static uint16_t crc16_words(const uint16_t words[6])
{
    uint16_t crc=0xFFFFU;
    uint32_t i,j,b;

    for(i=0U;i<6U;i++)
    {
        uint16_t w=words[i];

        for(j=0U;j<2U;j++)
        {
            crc^=(uint16_t)(
                ((j==0U)?(w&255U):(w>>8U))<<8U
            );

            for(b=0U;b<8U;b++)
            {
                crc=(crc&0x8000U)
                    ?(uint16_t)((crc<<1U)^0x1021U)
                    :(uint16_t)(crc<<1U);
            }
        }
    }

    return crc;
}

static void load_level_calibration(void)
{
    const volatile uint16_t *slot=
        (const volatile uint16_t *)
        (uintptr_t)CAL_FLASH_ADDR;

    uint16_t words[6];
    uint32_t i;
    float_bits_t r,p;

    imu_flash_load_status=3U;

    if((uintptr_t)&__drone_flash_cal_boundary__ !=
       (uintptr_t)CAL_FLASH_ADDR)
        return;

    if(slot[0]==0xFFFFU)
    {
        imu_flash_load_status=2U;
        return;
    }

    if(slot[0]!=CAL_MAGIC ||
       slot[1]!=CAL_VERSION ||
       slot[7]!=CAL_COMMIT)
        return;

    for(i=0U;i<6U;i++)
        words[i]=slot[i];

    if(slot[6]!=crc16_words(words))
        return;

    r.bits=((uint32_t)slot[3]<<16U)|slot[2];
    p.bits=((uint32_t)slot[5]<<16U)|slot[4];

    if(!(r.f>=-8.0f && r.f<=8.0f &&
         p.f>=-8.0f && p.f<=8.0f))
        return;

    imu_level_roll_zero_deg=r.f;
    imu_level_pitch_zero_deg=p.f;

    imu_flash_load_status=1U;
}

/* ATTITUDE AND GYRO MONITORING */

static void attitude_update(uint32_t dt_ms)
{
    float dt;
    float ax,ay,az,norm2;
    float gr,gp,gy;
    float accel_roll,accel_pitch;
    uint32_t norm_ok;

    attitude_valid=0U;

    if(calibration_status!=2U || imu_ok!=1U)
        return;

    dt=(float)dt_ms*0.001f;

    if(dt<=0.0f || dt>0.020f)
        return;

    gr=((float)gyro_x-gyro_x_offset)/16.4f;
    gp=((float)gyro_y-gyro_y_offset)/16.4f;
    gy=((float)gyro_z-gyro_z_offset)/16.4f;

    gyro_roll_dps=gr;
    gyro_pitch_dps=gp;
    gyro_yaw_dps=gy;

    /*
     * Filter is used only for diagnostics.
     * The controller still uses raw gyro
     * rates, as in Step 26.
     */

    if(!gyro_monitor_initialized)
    {
        gyro_filtered_roll_dps=gr;
        gyro_filtered_pitch_dps=gp;
        gyro_filtered_yaw_dps=gy;
        gyro_monitor_initialized=1U;
    }
    else
    {
        gyro_filtered_roll_dps +=
            GYRO_MONITOR_ALPHA*
            (gr-gyro_filtered_roll_dps);

        gyro_filtered_pitch_dps +=
            GYRO_MONITOR_ALPHA*
            (gp-gyro_filtered_pitch_dps);

        gyro_filtered_yaw_dps +=
            GYRO_MONITOR_ALPHA*
            (gy-gyro_filtered_yaw_dps);
    }

    ax=(float)accel_x;
    ay=(float)accel_y;
    az=(float)accel_z;

    norm2=ax*ax+ay*ay+az*az;

    norm_ok=
        norm2>2890000.0f &&
        norm2<5760000.0f;

    attitude_accel_norm_ok=norm_ok;

    if(norm_ok)
    {
        accel_roll=
            atan2_fast(ay,az)-
            imu_level_roll_zero_deg;

        accel_pitch=
            atan2_fast(-ax,hypot_fast(ay,az))-
            imu_level_pitch_zero_deg;

        if(!attitude_initialized)
        {
            attitude_roll_deg=accel_roll;
            attitude_pitch_deg=accel_pitch;
            attitude_initialized=1U;
        }
        else
        {
            attitude_roll_deg=
                FILTER_ALPHA*
                (attitude_roll_deg+gr*dt)+
                (1.0f-FILTER_ALPHA)*accel_roll;

            attitude_pitch_deg=
                FILTER_ALPHA*
                (attitude_pitch_deg+gp*dt)+
                (1.0f-FILTER_ALPHA)*accel_pitch;
        }

        last_accel_valid_ms=system_ms;
    }
    else if(attitude_initialized)
    {
        attitude_roll_deg+=gr*dt;
        attitude_pitch_deg+=gp*dt;
    }

    attitude_roll_deg=
        clampf(attitude_roll_deg,-180.0f,180.0f);

    attitude_pitch_deg=
        clampf(attitude_pitch_deg,-180.0f,180.0f);

    attitude_tilt_cutoff=
        (absf(attitude_roll_deg)>MOTOR_MAX_TILT_DEG ||
         absf(attitude_pitch_deg)>MOTOR_MAX_TILT_DEG)
        ?1U:0U;

    attitude_valid=
        attitude_initialized &&
        ((uint32_t)(
            system_ms-last_accel_valid_ms
        )<=500U) &&
        !attitude_tilt_cutoff;
}

/* NRF24L01 -- STEP 26 RADIO PROTOCOL */

static void spi_init(void)
{
    RCC_AHBENR|=(1U<<17)|(1U<<18);
    RCC_APB2ENR|=(1U<<12);

    (void)RCC_AHBENR;
    (void)RCC_APB2ENR;

    BSRR(GA)=(1U<<15);
    BSRR(GB)=(1U<<22);

    OTYPER(GA)&=~(1U<<15);

    OTYPER(GB)&=~(
        (1U<<3)|(1U<<5)|(1U<<6)
    );

    MODER(GA)=
        (MODER(GA)&~(3U<<30))|
        (1U<<30);

    MODER(GB)=
        (MODER(GB)&~(
            (3U<<6)|(3U<<8)|
            (3U<<10)|(3U<<12)
        ))|
        ((2U<<6)|(2U<<8)|
         (2U<<10)|(1U<<12));

    PUPDR(GB)&=~(
        (3U<<6)|(3U<<8)|
        (3U<<10)|(3U<<12)
    );

    OSPEEDR(GB)|=(3U<<6)|(3U<<10);

    AFRL(GB)&=~(
        (0xFU<<12)|
        (0xFU<<16)|
        (0xFU<<20)
    );

    SPI_CR1=0U;
    SPI_CR2=(7U<<8)|(1U<<12);

    SPI_CR1=
        (1U<<2)|
        (4U<<3)|
        (1U<<8)|
        (1U<<9)|
        (1U<<6);

    radio_spi_initialized=1U;
}

static uint32_t spi_wait(
    uint32_t flag,
    uint32_t required)
{
    uint32_t t=system_ms;

    for(;;)
    {
        if(((SPI_SR&flag)!=0U)==(required!=0U))
            return 1U;

        if((uint32_t)(system_ms-t)>=2U)
            return 0U;
    }
}

static uint32_t spi_byte(uint8_t tx,uint8_t *rx)
{
    if(!spi_wait(2U,1U))
        return 0U;

    SPI_DR8=tx;

    if(!spi_wait(1U,1U))
        return 0U;

    *rx=SPI_DR8;

    return 1U;
}

static uint32_t spi_xfer(
    const uint8_t *tx,
    uint8_t *rx,
    uint32_t count)
{
    uint32_t i;
    uint32_t ok=1U;

    if(count==0U || count>11U)
        return 0U;

    BSRR(GA)=(1U<<31);

    for(i=0U;i<count;i++)
    {
        uint8_t v=0U;

        if(!spi_byte(tx[i],&v))
        {
            ok=0U;
            break;
        }

        if(rx)
            rx[i]=v;
    }

    if(ok)
        ok=spi_wait(1U<<7,0U);

    BSRR(GA)=(1U<<15);

    if(SPI_SR&(1U<<6))
    {
        volatile uint8_t d=SPI_DR8;
        volatile uint32_t s=SPI_SR;

        (void)d;
        (void)s;

        ok=0U;
    }

    if(!ok)
        radio_spi_error_counter++;

    return ok;
}

static uint32_t reg_write(
    uint8_t address,
    const uint8_t *data,
    uint32_t count)
{
    uint8_t tx[11];
    uint32_t i;

    if(count==0U || count>10U)
        return 0U;

    tx[0]=(uint8_t)(0x20U|(address&0x1FU));

    for(i=0U;i<count;i++)
        tx[i+1U]=data[i];

    return spi_xfer(tx,0,count+1U);
}

static uint32_t reg_wr1(
    uint8_t address,
    uint8_t value)
{
    return reg_write(address,&value,1U);
}

static uint32_t reg_read(
    uint8_t address,
    uint8_t *value)
{
    uint8_t tx[2]={
        (uint8_t)(address&0x1FU),
        0xFFU
    };

    uint8_t rx[2];

    if(!spi_xfer(tx,rx,2U))
        return 0U;

    *value=rx[1];

    return 1U;
}

static uint32_t radio_cmd(uint8_t cmd)
{
    return spi_xfer(&cmd,0,1U);
}

static void radio_init(void)
{
    static const uint8_t regs[10]={
        0x00,0x01,0x02,0x03,0x04,
        0x05,0x06,0x11,0x1C,0x1D
    };

    static const uint8_t values[10]={
        0x0C,0x00,0x01,0x03,0x00,
        0x2D,0x07,0x09,0x00,0x00
    };

    static const uint8_t address[5]={
        0xCC,0xCC,0xCC,0xCC,0xCC
    };

    uint32_t i;
    uint8_t got=0U;

    radio_rx_configured=0U;
    radio_rx_config_error=0U;

    BSRR(GB)=(1U<<22);

    for(i=0U;i<10U;i++)
    {
        if(!reg_wr1(regs[i],values[i]) ||
           !reg_read(regs[i],&got) ||
           got!=values[i])
        {
            radio_rx_config_error=
                0x100U|regs[i];

            return;
        }
    }

    if(!reg_write(0x0A,address,5U))
    {
        radio_rx_config_error=0x10AU;
        return;
    }

    if(!reg_write(0x10,address,5U))
    {
        radio_rx_config_error=0x110U;
        return;
    }

    if(!radio_cmd(0xE2U) ||
       !reg_wr1(0x07U,0x70U))
    {
        radio_rx_config_error=0x1E2U;
        return;
    }

    if(!reg_wr1(0x00U,0x0FU) ||
       !reg_read(0x00U,&got) ||
       got!=0x0FU)
    {
        radio_rx_config_error=0x200U;
        return;
    }

    delay_ms(2U);

    BSRR(GB)=(1U<<6);

    radio_rx_configured=1U;
}

static void radio_accept(
    const uint8_t *data,
    uint32_t now)
{
    uint32_t i;
    uint32_t sum=0x76U;
    uint32_t seq=data[7];

    for(i=0U;i<8U;i++)
        sum+=data[i];

    if((uint8_t)sum!=data[8] ||
       data[0]>225U ||
       data[1]>225U ||
       data[2]>225U ||
       data[3]>225U ||
       data[4]!=0x05U ||
       data[5]!=0x38U ||
       data[6]!=0x00U)
    {
        radio_rx_invalid_packets++;
        return;
    }

    if(radio_diag_alive && radio_have_seq)
    {
        uint32_t delta=
            (uint8_t)(seq-previous_seq);

        if(delta==0U)
            return;

        if(delta>=128U)
        {
            radio_rx_invalid_packets++;
            return;
        }
    }

    previous_seq=(uint8_t)seq;
    radio_have_seq=1U;
    radio_seen=1U;

    radio_diag_last_seq=seq;

    last_valid_radio_ms=now;
    radio_diag_age_ms=0U;
    radio_diag_alive=1U;

    radio_diag_throttle=data[0];
    radio_diag_yaw=data[1];
    radio_diag_roll=data[2];
    radio_diag_pitch=data[3];

    radio_packets_received++;
}

static void radio_poll(uint32_t now)
{
    uint8_t status=0U;
    uint8_t fifo=0U;
    uint8_t tx[10];
    uint8_t rx[10];
    uint8_t packet[9];
    uint32_t i;

    if(!radio_rx_configured)
        return;

    if((uint32_t)(
        now-last_radio_poll_ms
    )<RADIO_POLL_MS)
        return;

    last_radio_poll_ms=now;

    if(!reg_read(0x07U,&status) ||
       !reg_read(0x17U,&fifo))
        return;

    (void)status;

    if(fifo&1U)
        return;

    tx[0]=0x61U;

    for(i=1U;i<10U;i++)
        tx[i]=0xFFU;

    if(!spi_xfer(tx,rx,10U))
        return;

    for(i=0U;i<9U;i++)
        packet[i]=rx[i+1U];

    radio_accept(packet,now);

    reg_wr1(0x07U,0x40U);
}

static void radio_link_update(uint32_t now)
{
    if(!radio_seen)
    {
        radio_diag_alive=0U;
        radio_diag_age_ms=UINT32_MAX;
        return;
    }

    radio_diag_age_ms=
        (uint32_t)(now-last_valid_radio_ms);

    if(radio_diag_alive &&
       radio_diag_age_ms>=RADIO_TIMEOUT_MS)
    {
        radio_diag_alive=0U;
        radio_diag_lost_count++;
        radio_have_seq=0U;

        radio_diag_throttle=225U;
        radio_diag_yaw=141U;
        radio_diag_roll=112U;
        radio_diag_pitch=112U;
    }
}

/* STICK CONVERSION */

static float stick_axis(
    uint32_t raw,
    uint32_t middle)
{
    float value;

    if(raw>225U)
        return 0.0f;

    if(raw>=middle-3U &&
       raw<=middle+3U)
        return 0.0f;

    if(raw>=middle)
        value=
            (float)(raw-middle)/
            (float)(225U-middle);
    else
        value=
            -((float)(middle-raw)/
              (float)middle);

    return clampf(value,-1.0f,1.0f);
}

static void update_sticks(void)
{
    sim_throttle_percent=
        (225.0f-(float)radio_diag_throttle)*
        (100.0f/225.0f);

    sim_roll_stick=
        stick_axis(radio_diag_roll,112U);

    sim_pitch_stick=
        stick_axis(radio_diag_pitch,112U);

    sim_yaw_stick=
        stick_axis(radio_diag_yaw,141U);
}

/* ARM / DISARM */

static void disarm(uint32_t reason)
{
    motor_stop();

    if(arm_state)
        disarm_counter++;

    arm_state=0U;
    sim_arm_state=0U;

    last_disarm_reason=reason;

    arm_rearm_latch=1U;

    arm_hold_active=0U;
    disarm_hold_active=0U;

    arm_gesture_progress_ms=0U;
    disarm_gesture_progress_ms=0U;

    if(fc_state!=FC_FAULT)
        fc_state=FC_DISARMED;
}

static uint32_t near_u32(
    uint32_t value,
    uint32_t center,
    uint32_t tolerance)
{
    return
        value>=center-tolerance &&
        value<=center+tolerance;
}

static uint32_t safe_rearm_center(void)
{
    return
        radio_diag_alive==1U &&
        radio_diag_age_ms<RADIO_ARM_MAX_AGE_MS &&
        radio_diag_throttle>=223U &&
        near_u32(radio_diag_yaw,141U,15U) &&
        near_u32(radio_diag_roll,112U,7U) &&
        near_u32(radio_diag_pitch,112U,7U);
}

static uint32_t arm_conditions_ok(void)
{
    if(fc_state!=FC_DISARMED ||
       fault_code!=0U ||
       imu_ok!=1U ||
       calibration_status!=2U ||
       attitude_valid!=1U)
        return 0U;

    if(radio_diag_alive!=1U ||
       radio_diag_age_ms>=RADIO_ARM_MAX_AGE_MS)
        return 0U;

    if(radio_diag_throttle<223U)
        return 0U;

    if(!near_u32(radio_diag_roll,112U,7U) ||
       !near_u32(radio_diag_pitch,112U,7U))
        return 0U;

    if(radio_diag_yaw<ARM_YAW_RAW)
        return 0U;

    if(absf(attitude_roll_deg)>ARM_MAX_TILT_DEG ||
       absf(attitude_pitch_deg)>ARM_MAX_TILT_DEG)
        return 0U;

    return 1U;
}

static void arm_update(uint32_t now)
{
    uint32_t request=disarm_request;
    disarm_request=0U;

    if(request)
    {
        disarm(DISARM_DEBUG);
        return;
    }

    if(fc_state==FC_FAULT)
    {
        motor_stop();
        return;
    }

    if(arm_state)
    {
        if(radio_diag_alive!=1U ||
           radio_diag_age_ms>=RADIO_TIMEOUT_MS)
        {
            disarm(DISARM_RADIO);
            return;
        }

        if(imu_ok!=1U ||
           calibration_status!=2U ||
           attitude_valid!=1U)
        {
            disarm(DISARM_IMU);
            return;
        }

        if(radio_diag_throttle>=223U &&
           radio_diag_yaw<=DISARM_YAW_RAW)
        {
            if(!disarm_hold_active)
            {
                disarm_hold_active=1U;
                disarm_hold_start_ms=now;
            }

            disarm_gesture_progress_ms=
                (uint32_t)(
                    now-disarm_hold_start_ms
                );

            if(disarm_gesture_progress_ms>=
               DISARM_HOLD_MS)
            {
                disarm(DISARM_STICK);
                return;
            }
        }
        else
        {
            disarm_hold_active=0U;
            disarm_gesture_progress_ms=0U;
        }

        return;
    }

    /*
     * New safety latch:
     * Yaw must return to center at zero
     * throttle after boot or any disarm.
     */

    if(arm_rearm_latch)
    {
        if(safe_rearm_center())
            arm_rearm_latch=0U;

        arm_hold_active=0U;
        arm_gesture_progress_ms=0U;

        return;
    }

    if(!arm_conditions_ok())
    {
        arm_hold_active=0U;
        arm_gesture_progress_ms=0U;
        return;
    }

    if(!arm_hold_active)
    {
        arm_hold_active=1U;
        arm_hold_start_ms=now;
    }

    arm_gesture_progress_ms=
        (uint32_t)(now-arm_hold_start_ms);

    if(arm_gesture_progress_ms>=ARM_HOLD_MS)
    {
        arm_state=1U;
        sim_arm_state=1U;
        fc_state=FC_ARMED;

        arm_counter++;

        arm_hold_active=0U;
        arm_gesture_progress_ms=0U;

        motor_stop();
    }
}

/* FOUR-MOTOR MIXER */

static void motor_control_update(void)
{
    float roll_target,pitch_target;
    float roll_rate_target,pitch_rate_target;
    float yaw_rate_target;

    float r,p,y;
    float base;
    float m1,m2,m3,m4;

    if(!arm_state || fc_state!=FC_ARMED)
    {
        motor_stop();
        return;
    }

    if(radio_diag_alive!=1U ||
       radio_diag_age_ms>=RADIO_TIMEOUT_MS)
    {
        disarm(DISARM_RADIO);
        return;
    }

    if(imu_ok!=1U ||
       attitude_valid!=1U)
    {
        disarm(DISARM_IMU);
        return;
    }

    if(sim_throttle_percent<
       MOTOR_START_THR_PERCENT)
    {
        motor_stop();

        mix_m1_percent=0.0f;
        mix_m2_percent=0.0f;
        mix_m3_percent=0.0f;
        mix_m4_percent=0.0f;

        pid_roll_output=0.0f;
        pid_pitch_output=0.0f;
        pid_yaw_output=0.0f;

        return;
    }

    roll_target=
        MAX_ANGLE_DEG*sim_roll_stick;

    pitch_target=
        MAX_ANGLE_DEG*sim_pitch_stick;

    roll_rate_target=clampf(
        ANGLE_KP*
        (roll_target-attitude_roll_deg),
        -MAX_RATE_DPS,
        MAX_RATE_DPS
    );

    pitch_rate_target=clampf(
        ANGLE_KP*
        (pitch_target-attitude_pitch_deg),
        -MAX_RATE_DPS,
        MAX_RATE_DPS
    );

    yaw_rate_target=
        MAX_YAW_DPS*sim_yaw_stick;

    /*
     * Preserve Step 26 raw-gyro feedback.
     */

    r=clampf(
        ROLL_KP*
        (roll_rate_target-gyro_roll_dps),
        -MAX_CORRECTION_PERCENT,
        MAX_CORRECTION_PERCENT
    );

    p=clampf(
        PITCH_KP*
        (pitch_rate_target-gyro_pitch_dps),
        -MAX_CORRECTION_PERCENT,
        MAX_CORRECTION_PERCENT
    );

    y=clampf(
        YAW_KP*
        (yaw_rate_target-gyro_yaw_dps),
        -MAX_YAW_CORRECTION_PERCENT,
        MAX_YAW_CORRECTION_PERCENT
    );

    pid_roll_output=r;
    pid_pitch_output=p;
    pid_yaw_output=y;

    base=
        15.0f+
        sim_throttle_percent*0.40f;

    m1=clampf(
        base+r+p+y,0.0f,MOTOR_MAX_PERCENT
    );

    m2=clampf(
        base-r+p-y,0.0f,MOTOR_MAX_PERCENT
    );

    m3=clampf(
        base-r-p+y,0.0f,MOTOR_MAX_PERCENT
    );

    m4=clampf(
        base+r-p-y,0.0f,MOTOR_MAX_PERCENT
    );

    mix_m1_percent=m1;
    mix_m2_percent=m2;
    mix_m3_percent=m3;
    mix_m4_percent=m4;

    motor_write(m1,m2,m3,m4);
}

/* MAIN */

int main(void)
{
    uint32_t last_loop;

    motor_stop();

    reset_cause=RCC_CSR;

    watchdog_reset_detected=
        (reset_cause&(1U<<29))?1U:0U;

    RCC_CSR|=(1U<<24);

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

    RCC_AHBENR|=(1U<<18);

    BSRR(GA)=(1U<<1);
    OTYPER(GA)&=~(1U<<1);

    MODER(GA)=
        (MODER(GA)&~(3U<<2))|
        (1U<<2);

    spi_init();
    radio_init();

    i2c_init();
    delay_ms(20U);

    load_level_calibration();

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
        gyro_cal_start();
    }

    motor_stop();

    last_loop=system_ms;

    for(;;)
    {
        uint32_t now=system_ms;

        uint32_t elapsed=
            (uint32_t)(now-last_loop);

        uint32_t work_started_ms;

        if(elapsed<LOOP_PERIOD_MS)
            continue;

        last_loop=now;
        work_started_ms=now;

        loop_counter++;
        loop_dt_ms=elapsed;

        if(elapsed>LOOP_WARN_MS)
            loop_overrun_counter++;

        if(fc_state==FC_FAULT)
        {
            motor_stop();
        }
        else
        {
            if(elapsed>LOOP_LIMIT_MS)
                disarm(DISARM_TIMING);

            now=system_ms;

            if(!imu_sample())
            {
                imu_ok=0U;
                imu_error_counter++;
                imu_consecutive_errors++;

                imu_last_good_age_ms=
                    (uint32_t)(
                        now-last_imu_good_ms
                    );

                fail(3U);
            }
            else
            {
                uint32_t end=system_ms;

                imu_ok=1U;
                imu_consecutive_errors=0U;

                last_imu_good_ms=end;
                imu_last_good_age_ms=0U;

                imu_read_duration_ms=
                    (uint32_t)(end-now);

                if(imu_read_duration_ms>
                   imu_read_max_ms)
                {
                    imu_read_max_ms=
                        imu_read_duration_ms;
                }

                if(fc_state==FC_CALIBRATING)
                    gyro_cal_update();

                if(calibration_status==2U)
                    attitude_update(elapsed);

                if(fc_state==FC_ARMED &&
                   attitude_tilt_cutoff)
                {
                    disarm(DISARM_TILT);
                }
            }
        }

        radio_poll(system_ms);
        radio_link_update(system_ms);
        update_sticks();

        arm_update(system_ms);

        if(fc_state==FC_ARMED &&
           radio_diag_age_ms<RADIO_TIMEOUT_MS &&
           imu_ok==1U &&
           attitude_valid==1U)
        {
            motor_control_update();
        }
        else
        {
            motor_stop();
        }

        loop_work_ms=
            (uint32_t)(
                system_ms-work_started_ms
            );

        if(loop_work_ms>loop_max_work_ms)
            loop_max_work_ms=loop_work_ms;

        watchdog_feed();
    }
}
