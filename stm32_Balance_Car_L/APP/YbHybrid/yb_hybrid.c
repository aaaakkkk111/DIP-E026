/* PC supervision is asynchronous and offboard. NEVER parse JSON, train, or
 * call an LLM from this ISR. No serial packet can arm motors or command PWM. */
#include "AllHeader.h"
#include "yb_hybrid.h"
#include "yb_protocol.h"
#include "policy.h"
#include <string.h>
#include <math.h>

#define DWT_CTRL_REG (*(volatile uint32_t *)0xE0001000UL)
#define DWT_CYCLE_REG (*(volatile uint32_t *)0xE0001004UL)
#define DEMCR_REG (*(volatile uint32_t *)0xE000EDFCUL)
#define RX_SIZE 256U
#define COUNTS_PER_WHEEL_REV (4.0f*30.0f*11.0f)
#define DEG_TO_RAD 0.017453292519943295f
extern float Balance_Kp, Balance_Kd, Velocity_Kp, Velocity_Ki, Turn_Kp, Turn_Kd;
extern float Roll;
float Yb_Roll_Rate_Raw=0;
static volatile uint8_t controller=YB_STOCK_PID;
static volatile uint16_t faults=0;
static float boot_gains[6], active_gains[6];
static uint8_t rx[RX_SIZE];
static volatile uint16_t rx_write=0, rx_read=0;
static uint8_t packet[34];
static unsigned used=0;
static uint16_t last_seq=0;
static int have_seq=0, was_stopped=1;
static float wheel_l=0, wheel_r=0, forward_x=0;
static uint32_t tick_start=0, inference_cycles=0, max_tick_cycles=0;
static uint32_t sensor_cycles=0,max_sensor_cycles=0,max_inference_cycles=0;
static uint32_t previous_tick_start=0,period_cycles=0,min_period_cycles=0xffffffffU,max_period_cycles=0;
static uint32_t qualified_ticks=0;
static int did_inference=0;
static uint32_t first_fault_tick=0,first_loop_cycles=0,first_sensor_cycles=0;
static uint32_t first_inference_cycles=0,first_period_cycles=0;
static volatile uint32_t tick=0;
static uint32_t last_sent=0;
/* Exactly 76 bytes on the wire, encoded explicitly, not C struct padding. */
static uint8_t telemetry[76];
static volatile uint32_t snapshot_version=0;
/* Last 256 armed control ticks, encoded little-endian (48 bytes each).
 * Adds exactly 12 KiB trace RAM plus a 48-byte staging row. Inspect actual
 * target map/stack after rebuild: total firmware RAM remains unverified.
 * Frozen at first stopped/fault tick; never erased by stopped preflight. */
static uint8_t trace_rows[YB_TRACE_ROWS][YB_TRACE_ROW_BYTES];
static uint8_t trace_stage[YB_TRACE_ROW_BYTES];
static unsigned trace_next=0,trace_count=0;
static int trace_live=0;
#ifndef POLICY_CHECKPOINT_SHA256
#define POLICY_CHECKPOINT_SHA256 "UNVERIFIED"
#endif
static const char checkpoint_sha256[]=POLICY_CHECKPOINT_SHA256;
static int raw_data_ready_verified=0;
static int encoder_hist_l[YB_ENCODER_WINDOW],encoder_hist_r[YB_ENCODER_WINDOW];
static int encoder_sum_l=0,encoder_sum_r=0;
static unsigned encoder_head=0,encoder_used=0;
static uint32_t gyro_cal_count=0,gyro_cal_resets=0;
static int gyro_cal_ready=0;
static float gyro_bias[3],gyro_cal_sum[3],gyro_cal_sum2[3],gyro_cal_std[3];
static float cal_pitch_min,cal_pitch_max,cal_roll_min,cal_roll_max,cal_z_min,cal_z_max;

static void gyro_cal_reset(void)
{
    gyro_cal_count=0;gyro_cal_ready=0;
    memset(gyro_bias,0,sizeof gyro_bias);
    memset(gyro_cal_sum,0,sizeof gyro_cal_sum);
    memset(gyro_cal_sum2,0,sizeof gyro_cal_sum2);
    memset(gyro_cal_std,0,sizeof gyro_cal_std);
    cal_pitch_min=cal_pitch_max=cal_roll_min=cal_roll_max=cal_z_min=cal_z_max=0;
}
static void gyro_cal_update(const float raw[3],float pitch,float roll,int enc_l,int enc_r)
{
    unsigned i;
    float mean,variance;
    int valid=1;
    /* Raw/DMP calibration are different paths. Require a stationary supported
     * chassis, never infer pitch/roll mounting offsets from an arbitrary pose.
     * Constant external yaw motion is unobservable to a 6-axis IMU: the human
     * must actually keep the support still, not merely hold a constant tilt. */
    if(!Stop_Flag || gyro_cal_ready || faults) return;
    if(enc_l || enc_r || fabsf(pitch)>0.175f || fabsf(roll)>0.175f ||
       !yb_finite(Acceleration_Z) || Acceleration_Z<14745.0f || Acceleration_Z>18022.0f)
        valid=0;
    for(i=0;i<3;i++) if(!yb_finite(raw[i]) || fabsf(raw[i])>0.15f) valid=0;
    if(!valid) { gyro_cal_reset();++gyro_cal_resets;return; }
    if(!gyro_cal_count) {
        cal_pitch_min=cal_pitch_max=pitch;cal_roll_min=cal_roll_max=roll;
        cal_z_min=cal_z_max=Acceleration_Z;
    }
    if(pitch<cal_pitch_min) cal_pitch_min=pitch;
    if(pitch>cal_pitch_max) cal_pitch_max=pitch;
    if(roll<cal_roll_min) cal_roll_min=roll;
    if(roll>cal_roll_max) cal_roll_max=roll;
    if(Acceleration_Z<cal_z_min) cal_z_min=Acceleration_Z;
    if(Acceleration_Z>cal_z_max) cal_z_max=Acceleration_Z;
    if(cal_pitch_max-cal_pitch_min>0.004363323f ||
       cal_roll_max-cal_roll_min>0.004363323f || cal_z_max-cal_z_min>327.68f) {
        gyro_cal_reset();++gyro_cal_resets;return;
    }
    for(i=0;i<3;i++) { gyro_cal_sum[i]+=raw[i];gyro_cal_sum2[i]+=raw[i]*raw[i]; }
    if(++gyro_cal_count<YB_GYRO_CAL_TICKS) return;
    for(i=0;i<3;i++) {
        mean=gyro_cal_sum[i]*(1.0f/YB_GYRO_CAL_TICKS);
        variance=gyro_cal_sum2[i]*(1.0f/YB_GYRO_CAL_TICKS)-mean*mean;
        gyro_cal_std[i]=sqrtf(variance>0?variance:0);
        if(fabsf(mean)>YB_GYRO_CAL_MAX_BIAS || gyro_cal_std[i]>YB_GYRO_CAL_MAX_STD) valid=0;
        gyro_bias[i]=mean;
    }
    if(valid) gyro_cal_ready=1;
    else { gyro_cal_reset();++gyro_cal_resets; }
}

static void encoder_filter_reset(void)
{
    memset(encoder_hist_l,0,sizeof encoder_hist_l);
    memset(encoder_hist_r,0,sizeof encoder_hist_r);
    encoder_sum_l=encoder_sum_r=0;encoder_head=encoder_used=0;
}
static void encoder_velocity(int enc_l,int enc_r,float *wl,float *wr)
{
    static const float inverse_samples[4]={1.0f,0.5f,0.333333333333f,0.25f};
    float scale;
    /* A 5ms single count is 0.952 rad/s (~0.032m/s). Current window=1:
     * RAW counts, zero averaging delay. Do not silently add delayed smoothing;
     * earlier 4-sample ablation worsened yaw/stationkeeping. Raw counts still
     * integrate wheel position exactly; checkpoint weights are unchanged. */
    encoder_sum_l+=enc_l-encoder_hist_l[encoder_head];
    encoder_sum_r+=enc_r-encoder_hist_r[encoder_head];
    encoder_hist_l[encoder_head]=enc_l;encoder_hist_r[encoder_head]=enc_r;
    encoder_head=(encoder_head+1U)%YB_ENCODER_WINDOW;
    if(encoder_used<YB_ENCODER_WINDOW) ++encoder_used;
    /* No runtime software-float divide in the nearly full 5ms budget. */
    scale=(6.28318530718f*POLICY_CONTROL_HZ/COUNTS_PER_WHEEL_REV)*inverse_samples[encoder_used-1U];
    *wl=(float)encoder_sum_l*scale;*wr=(float)encoder_sum_r*scale;
}

static uint32_t cycles_us(uint32_t cycles) { return cycles/(SystemCoreClock/1000000U); }

static void read_gains(float g[6])
{
    g[0]=Balance_Kp; g[1]=Balance_Kd; g[2]=Velocity_Kp;
    g[3]=Velocity_Ki; g[4]=Turn_Kp; g[5]=Turn_Kd;
}
static void write_gains(const float g[6])
{
    Balance_Kp=g[0]; Balance_Kd=g[1]; Velocity_Kp=g[2];
    Velocity_Ki=g[3]; Turn_Kp=g[4]; Turn_Kd=g[5];
}
static uint32_t enter_critical(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq(); return mask;
}
static void leave_critical(uint32_t mask) { __set_PRIMASK(mask); }
static void put16(uint8_t *p, uint16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0]|((uint16_t)p[1]<<8)); }
static void put32(uint8_t *p, uint32_t v) { memcpy(p,&v,4); }
static void putfloat(uint8_t *p, float v) { memcpy(p,&v,4); }

static void send_frame(uint8_t type, uint16_t seq, const uint8_t *body, uint16_t len)
{
    uint8_t frame[86];
    frame[0]=0xaa; frame[1]=0x55; frame[2]=1; frame[3]=type;
    put16(frame+4,seq); put16(frame+6,len);
    if(len) memcpy(frame+8,body,len);
    put16(frame+8+len,yb_crc16(frame+2,6+len));
    USART1_Send_ArrayU8(frame,(uint16_t)(10+len)); /* foreground only */
}
static void ack(uint8_t request, uint16_t seq, uint8_t status)
{
    uint8_t body[28]; body[0]=request; body[1]=status;
    put16(body+2,last_seq); /* lets a reconnected PC choose the next sequence */
    if(request==0x12) memcpy(body+4,boot_gains,24);
    send_frame(0x80,seq,body,(uint16_t)(request==0x12 ? 28 : 4));
}
/* Boot-only, checked register access. The vendor DMP startup is retained for
 * its calibration, but Kalman/raw acquisition MUST NOT run from DMP events
 * (tap/orientation/FIFO events are not a guaranteed raw 200 Hz clock).
 * IIC_Start/Wait_Ack in this BSP return 1 on success, not the usual 0. */
static int imu_write_reg(uint8_t reg,uint8_t value)
{
    if(!IIC_Start()) { IIC_Stop();return 0; }
    IIC_Send_Byte(devAddr);
    if(!IIC_Wait_Ack()) { IIC_Stop();return 0; }
    IIC_Send_Byte(reg);
    if(!IIC_Wait_Ack()) { IIC_Stop();return 0; }
    IIC_Send_Byte(value);
    if(!IIC_Wait_Ack()) { IIC_Stop();return 0; }
    IIC_Stop();return 1;
}
static int imu_read_reg(uint8_t reg,uint8_t *value)
{
    if(!IIC_Start()) { IIC_Stop();return 0; }
    IIC_Send_Byte(devAddr);
    if(!IIC_Wait_Ack()) { IIC_Stop();return 0; }
    IIC_Send_Byte(reg);
    if(!IIC_Wait_Ack()) { IIC_Stop();return 0; }
    if(!IIC_Start()) { IIC_Stop();return 0; }
    IIC_Send_Byte(devAddr+1);
    if(!IIC_Wait_Ack()) { IIC_Stop();return 0; }
    *value=IIC_Read_Byte(0);IIC_Stop();return 1;
}
static int imu_expect_reg(uint8_t reg,uint8_t mask,uint8_t expected)
{
    uint8_t value=0;
    return imu_read_reg(reg,&value) && (value & mask)==expected;
}
static int configure_raw_data_ready(void)
{
    /* Only the selected Kalman path uses this. Do not alter DMP mode 1.
     * 1 kHz/(1+4)=200 Hz with DLPF_CFG=2 (98 Hz gyro bandwidth).
     * Keep the existing +/-2000 dps and +/-2 g sensor scaling/calibration.
     * Disable DMP/FIFO, then latch active-low DATA_RDY until a register read.
     * ANY_RD_CLEAR lets the coherent sensor burst release INT early in ISR. */
    if(mpu_set_dmp_state(0) || mpu_configure_fifo(0) ||
       mpu_set_sample_rate(200) || mpu_set_int_level(1) ||
       mpu_set_int_latched(1)) return 0;
    if(!imu_write_reg(MPU6050_RA_INT_ENABLE,0) ||
       !imu_write_reg(MPU6050_RA_USER_CTRL,0) ||
       !imu_write_reg(MPU6050_RA_FIFO_EN,0) ||
       !imu_write_reg(MPU6050_RA_INT_PIN_CFG,0xb0) ||
       !imu_write_reg(MPU6050_RA_INT_ENABLE,1)) return 0;
    return imu_expect_reg(MPU6050_RA_SMPLRT_DIV,0xff,4) &&
           imu_expect_reg(MPU6050_RA_CONFIG,7,2) &&
           imu_expect_reg(MPU6050_RA_GYRO_CONFIG,0x18,0x18) &&
           imu_expect_reg(MPU6050_RA_ACCEL_CONFIG,0x18,0) &&
           imu_expect_reg(MPU6050_RA_USER_CTRL,0xe0,0) &&
           imu_expect_reg(MPU6050_RA_FIFO_EN,0xff,0) &&
           imu_expect_reg(MPU6050_RA_INT_PIN_CFG,0xb0,0xb0) &&
           imu_expect_reg(MPU6050_RA_INT_ENABLE,0xff,1);
}
void Hybrid_ConfigureInterrupts(void)
{
    uint32_t mask=enter_critical();
    uint8_t pending=0;
    /* Existing group 2 = two preemption + two subpriority bits. The vendor
     * TIM6 preemption value 4 is out of range and wraps to highest priority!
     * These CMSIS values are the four-bit logical priorities, NOT shifted.
     * Keep short USART1 RX above IMU; defer timer/ultrasonic/Bluetooth work. */
    if(((SCB->AIRCR>>8)&7U)!=5U) {
        faults|=YB_FAULT_CONFIG;Stop_Flag=1;Set_Pwm(0,0);
    } else {
        NVIC_SetPriority(USART1_IRQn,0U);      /* preemption 0, subpriority 0 */
        NVIC_SetPriority(EXTI15_10_IRQn,4U);   /* preemption 1, subpriority 0 */
        NVIC_SetPriority(UART5_IRQn,8U);      /* preemption 2, subpriority 0 */
        NVIC_SetPriority(TIM2_IRQn,8U);       /* preemption 2, subpriority 0 */
        NVIC_SetPriority(TIM6_IRQn,12U);      /* preemption 3, subpriority 0 */
    }
    /* Raw IRQ may already be latched low before EXTI is enabled. Clear the
     * MCU pending bit BEFORE the IMU read so the next falling edge is kept. */
    if(raw_data_ready_verified) {
        EXTI->PR=EXTI_Line12;
        if(!imu_read_reg(MPU6050_RA_INT_STATUS,&pending)) {
            faults|=YB_FAULT_SENSOR;Stop_Flag=1;Set_Pwm(0,0);
        }
    }
    leave_critical(mask);
}
void Hybrid_Init(void)
{
    uint32_t wait_start,wait_spins;
    read_gains(boot_gains); memcpy(active_gains,boot_gains,sizeof boot_gains);
    DEMCR_REG |= 1UL<<24; DWT_CYCLE_REG=0; DWT_CTRL_REG |= 1U;
    /* Quantize before enabling IWDG; weights never change in the ISR. */
    if(!policy_prepare() || !policy_self_test()) {
        faults|=YB_FAULT_CONFIG;Stop_Flag=1;Set_Pwm(0,0);
        OLED_Draw_Line("FAULT NN selftest",2,false,true);return;
    }
    if(GET_Angle_Way==2) {
        raw_data_ready_verified=configure_raw_data_ready();
        if(!raw_data_ready_verified) {
            faults|=YB_FAULT_CONFIG;Stop_Flag=1;Set_Pwm(0,0);
            OLED_Draw_Line("FAULT raw IMU200Hz",2,false,true);return;
        }
    }
    /* Independent watchdog: missing/stalled IMU ISR resets to stopped boot.
     * Nominal 200 ms at 40 kHz LSI; actual LSI frequency varies by device.
     * Refreshed in EndTick, NEVER by the PC or foreground main loop. */
    RCC->CSR |= RCC_CSR_LSION;
    wait_start=DWT_CYCLE_REG;wait_spins=0;
    while((RCC->CSR & RCC_CSR_LSIRDY)==0) {
        if((uint32_t)(DWT_CYCLE_REG-wait_start)>SystemCoreClock/20U || ++wait_spins>1000000U) {
            faults|=YB_FAULT_CONFIG;Stop_Flag=1;Set_Pwm(0,0);
            OLED_Draw_Line("FAULT LSI startup",2,false,true);return;
        }
    }
    /* Enable before waiting for PR/RLR synchronization (confirmed PPO
     * startup blocker on this board). Keep watchdog and motor guards. */
    IWDG->KR=0xcccc;
    IWDG->KR=0x5555; IWDG->PR=4; IWDG->RLR=124;
    wait_start=DWT_CYCLE_REG;wait_spins=0;
    while(IWDG->SR!=0) {
        if((uint32_t)(DWT_CYCLE_REG-wait_start)>SystemCoreClock/20U || ++wait_spins>1000000U) {
            faults|=YB_FAULT_CONFIG;Stop_Flag=1;Set_Pwm(0,0);
            OLED_Draw_Line("FAULT IWDG startup",2,false,true);return;
        }
    }
    IWDG->KR=0xaaaa;
    policy_reset_state(0,0,0);
    USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
}
int Hybrid_IsNN(void) { return controller==YB_TZEJUN_NN; }
int Hybrid_HasFault(void) { return faults!=0; }
int Hybrid_CanArm(void)
{
    return faults==0 && Stop_Flag==1 &&
           (!Hybrid_IsNN() || (raw_data_ready_verified && gyro_cal_ready && qualified_ticks>=YB_PREFLIGHT_TICKS));
}
void Hybrid_SensorFault(void) { faults|=YB_FAULT_SENSOR;Stop_Flag=1;Set_Pwm(0,0); }
void Hybrid_SensorsReady(void)
{
    sensor_cycles=DWT_CYCLE_REG-tick_start;
    if(sensor_cycles>max_sensor_cycles) max_sensor_cycles=sensor_cycles;
}
void Hybrid_PhysicalStop(void)
{
    uint32_t mask=enter_critical();
    faults|=YB_FAULT_PHYSICAL_STOP; /* prevents factory put-down re-arm */
    Stop_Flag=1;Set_Pwm(0,0);g_newcarstate=enSTOP;
    leave_critical(mask);
}
void Hybrid_BeginTick(void)
{
    tick_start=DWT_CYCLE_REG; inference_cycles=0; sensor_cycles=0;did_inference=0;
    if(previous_tick_start) {
        period_cycles=tick_start-previous_tick_start;
        if(period_cycles<min_period_cycles) min_period_cycles=period_cycles;
        if(period_cycles>max_period_cycles) max_period_cycles=period_cycles;
        if(Hybrid_IsNN() && (period_cycles<SystemCoreClock/1000U*3U ||
                            period_cycles>SystemCoreClock/1000U*7U)) {
            faults|=YB_FAULT_CADENCE;Stop_Flag=1;Set_Pwm(0,0);
        }
    }
    previous_tick_start=tick_start;
    /* The old Bluetooth gain editor must not overwrite supervised gains.
     * Bluetooth movement commands remain intact; NN does not use PID gains. */
    if(controller!=YB_STOCK_PID) write_gains(active_gains);
    if(faults) Stop_Flag=1; /* latched; power/reset board to clear */
}
static void nn_commands(float *speed, float *yaw)
{
    *speed=0; *yaw=0;
    switch(g_newcarstate) {
    case enRUN: *speed=YB_NN_SPEED_M_S; break;
    case enBACK: *speed=-YB_NN_SPEED_M_S; break;
    case enLEFT: case enTLEFT: *yaw=YB_NN_YAW_RAD_S; break;
    case enRIGHT: case enTRIGHT: *yaw=-YB_NN_YAW_RAD_S; break;
    default: break; /* unsupported autonomous commands -> stand */
    }
}
void Hybrid_Control(int enc_l, int enc_r, int *pwm_l, int *pwm_r)
{
    float obs[POLICY_N_OBS], action[2]={NAN,NAN}, gyro_body[3];
    float wl,wr,wl_raw,wr_raw,v,pitch,roll,cmd_v,cmd_yaw;
    uint32_t start;
    int calibrated_before=gyro_cal_ready;
    wl_raw=(float)enc_l*(6.28318530718f*POLICY_CONTROL_HZ/COUNTS_PER_WHEEL_REV);
    wr_raw=(float)enc_r*(6.28318530718f*POLICY_CONTROL_HZ/COUNTS_PER_WHEEL_REV);
    wl=wl_raw;wr=wr_raw;
    if(Hybrid_IsNN()) {
        if(Stop_Flag || was_stopped) encoder_filter_reset();
        encoder_velocity(enc_l,enc_r,&wl,&wr);
    }
    /* Kalman branch KF_X uses -gyro_x; KF_Y uses +gyro_y. Transform the
     * sensor's (X,Y,Z) into robot (forward,axle,up) = (Y,-X,Z), SI units. */
    pitch=YB_PITCH_SIGN*(Angle_Balance-(float)Mid_Angle)*DEG_TO_RAD;
    roll=YB_ROLL_SIGN*Roll*DEG_TO_RAD;
    gyro_body[0]=YB_ROLL_SIGN*Yb_Roll_Rate_Raw/YB_GYRO_LSB_PER_RAD_S;
    gyro_body[1]=YB_PITCH_SIGN*Gyro_Balance/YB_GYRO_LSB_PER_RAD_S;
    gyro_body[2]=YB_YAW_SIGN*Gyro_Turn/YB_GYRO_LSB_PER_RAD_S;
    if(Hybrid_IsNN()) {
        gyro_cal_update(gyro_body,pitch,roll,enc_l,enc_r);
        if(gyro_cal_ready) {
            gyro_body[0]-=gyro_bias[0];gyro_body[1]-=gyro_bias[1];gyro_body[2]-=gyro_bias[2];
        }
    }
    /* Encoders measure wheel rotation RELATIVE to chassis. No-slip axle
     * translation also includes chassis pitch rotation; project into forward
     * body axis as in MuJoCo. Assumes no slip; measured calibration pending. */
    v=(policy_odometry(wl,wr)+gyro_body[1]*POLICY_WHEEL_RADIUS)*cosf(pitch);
    if(!yb_finite(pitch)||!yb_finite(roll)||!yb_finite(v)||
       !yb_finite(gyro_body[0])||!yb_finite(gyro_body[1])||!yb_finite(gyro_body[2]))
        faults |= YB_FAULT_SENSOR;
    if(Hybrid_IsNN()) {
        if(mode!=Normal || GET_Angle_Way!=2) faults |= YB_FAULT_CONFIG;
        if(sizeof(checkpoint_sha256)!=65U) faults |= YB_FAULT_CONFIG;
        if(fabsf(pitch)>POLICY_FALL_ANGLE_LIMIT || fabsf(roll)>POLICY_FALL_ANGLE_LIMIT)
            faults |= YB_FAULT_TILT;
    }
    if(faults) Stop_Flag=1;
    if(Stop_Flag) {
        /* Reset every stopped tick. A physical re-arm starts with actual IMU
         * history, zero position, zero integrals and no stale command. */
        wheel_l=wheel_r=forward_x=0;
        if(yb_finite(pitch)&&yb_finite(gyro_body[1])&&yb_finite(v))
            policy_reset_state(pitch,gyro_body[1],v);
        was_stopped=1;
        if(Hybrid_IsNN()) { *pwm_l=*pwm_r=0; g_newcarstate=enSTOP; }
        /* Real on-target inference while UNARMED. Outputs are discarded.
         * Includes reset/build/infer work so timing cannot be certified by a
         * zero-inference stopped sample as in the previous firmware. */
        if(Hybrid_IsNN() && !faults && gyro_cal_ready && calibrated_before) {
            start=DWT_CYCLE_REG;
            policy_build_obs_rp(obs,roll,pitch,0,0,v,gyro_body,wl,wr,0,0);
            policy_infer(obs,action);
            inference_cycles=DWT_CYCLE_REG-start; did_inference=1;
            if(!yb_finite(action[0])||!yb_finite(action[1])) faults|=YB_FAULT_SENSOR;
        }
    } else {
        if(was_stopped) {
            policy_reset_state(pitch,gyro_body[1],v); was_stopped=0;
            if(Hybrid_IsNN()) {
                trace_next=trace_count=0;trace_live=1;
            }
        }
        wheel_l+=wl_raw*POLICY_DT; wheel_r+=wr_raw*POLICY_DT; forward_x+=v*POLICY_DT;
        if(Hybrid_IsNN()) {
            nn_commands(&cmd_v,&cmd_yaw);
            start=DWT_CYCLE_REG;
            policy_build_obs_rp(obs,roll,pitch,wheel_l,wheel_r,v,gyro_body,wl,wr,cmd_v,cmd_yaw);
            policy_infer(obs,action);
            inference_cycles=DWT_CYCLE_REG-start;
            did_inference=1;
            if(!yb_finite(action[0])||!yb_finite(action[1])) { faults|=YB_FAULT_SENSOR; Stop_Flag=1; }
            if(Stop_Flag) *pwm_l=*pwm_r=0;
            else { *pwm_l=policy_action_to_pwm(action[0]); *pwm_r=policy_action_to_pwm(action[1]); }
            /* No PID residual added: this checkpoint is a STANDALONE policy.
             * Native PWM_Ignore() and 2600 ceiling are applied AFTER this. */
        }
    }
    if(inference_cycles>max_inference_cycles) max_inference_cycles=inference_cycles;
    snapshot_version++;
    telemetry[0]=controller; telemetry[1]=Stop_Flag; put16(telemetry+2,faults);
    put32(telemetry+4,++tick);
    /* Preserve actions BEFORE obs[] is reused for the six legacy gains.
     * A stopped forecast records the actual discarded NN output; NaN means
     * no inference ran (fault/config/another controller), not zero effort. */
    put32(trace_stage,tick);
    putfloat(trace_stage+4,pitch/DEG_TO_RAD);putfloat(trace_stage+8,roll/DEG_TO_RAD);
    putfloat(trace_stage+12,gyro_body[1]);putfloat(trace_stage+16,wl);putfloat(trace_stage+20,wr);
    putfloat(trace_stage+24,action[0]);putfloat(trace_stage+28,action[1]);
    putfloat(trace_stage+40,v);
    putfloat(telemetry+8,pitch/DEG_TO_RAD); putfloat(telemetry+12,roll/DEG_TO_RAD);
    putfloat(telemetry+16,gyro_body[1]); putfloat(telemetry+20,gyro_body[2]);
    putfloat(telemetry+24,v); putfloat(telemetry+28,forward_x);
    putfloat(telemetry+40,battery);
    read_gains(obs); memcpy(telemetry+44,obs,24);
    put32(telemetry+68,inference_cycles);
    snapshot_version++;
}
void Hybrid_EndTick(int pwm_l,int pwm_r)
{
    uint32_t elapsed=DWT_CYCLE_REG-tick_start;
    unsigned trace_written=YB_TRACE_ROWS; /* sentinel: no current armed row */
    if(elapsed>max_tick_cycles) max_tick_cycles=elapsed;
    if(Hybrid_IsNN() && elapsed>SystemCoreClock/1000000U*YB_CONTROL_BUDGET_US) {
        faults|=YB_FAULT_DEADLINE; Stop_Flag=1;
        Set_Pwm(0,0); pwm_l=pwm_r=0;
    }
    if(faults && !first_fault_tick) {
        first_fault_tick=tick;
        first_loop_cycles=elapsed;first_sensor_cycles=sensor_cycles;
        first_inference_cycles=inference_cycles;first_period_cycles=period_cycles;
    }
    if(Hybrid_IsNN() && Stop_Flag==1 && !faults && did_inference && period_cycles &&
       elapsed<SystemCoreClock/1000000U*YB_CONTROL_BUDGET_US) {
        if(qualified_ticks<YB_PREFLIGHT_TICKS) ++qualified_ticks;
    } else if(faults) qualified_ticks=0;
    /* Save what actually reached the bridge, including the guard's zero
     * output on the terminating tick. Ring order is logical oldest-first. */
    putfloat(trace_stage+32,(float)pwm_l);putfloat(trace_stage+36,(float)pwm_r);
    put16(trace_stage+44,Stop_Flag);put16(trace_stage+46,faults);
    if(trace_live) {
        trace_written=trace_next;
        memcpy(trace_rows[trace_next],trace_stage,YB_TRACE_ROW_BYTES);
        trace_next=(trace_next+1U)%YB_TRACE_ROWS;
        if(trace_count<YB_TRACE_ROWS) ++trace_count;
        if(Stop_Flag || faults) trace_live=0;
    }
    /* The diagnostic capture is part of the SAME 5 ms deadline, not free
     * work outside the budget. If capture itself crosses the deadline,
     * correct this tick's saved actual outputs and freeze the trace. */
    elapsed=DWT_CYCLE_REG-tick_start;
    if(elapsed>max_tick_cycles) max_tick_cycles=elapsed;
    if(Hybrid_IsNN() && elapsed>SystemCoreClock/1000000U*YB_CONTROL_BUDGET_US) {
        faults|=YB_FAULT_DEADLINE;Stop_Flag=1;qualified_ticks=0;
        Set_Pwm(0,0);pwm_l=pwm_r=0;trace_live=0;
        putfloat(trace_stage+32,0.0f);putfloat(trace_stage+36,0.0f);
        put16(trace_stage+44,Stop_Flag);put16(trace_stage+46,faults);
        if(trace_written<YB_TRACE_ROWS) {
            putfloat(trace_rows[trace_written]+32,0.0f);putfloat(trace_rows[trace_written]+36,0.0f);
            put16(trace_rows[trace_written]+44,Stop_Flag);put16(trace_rows[trace_written]+46,faults);
        }
        if(!first_fault_tick) {
            first_fault_tick=tick;first_loop_cycles=elapsed;first_sensor_cycles=sensor_cycles;
            first_inference_cycles=inference_cycles;first_period_cycles=period_cycles;
        }
    }
    snapshot_version++;
    telemetry[1]=Stop_Flag; put16(telemetry+2,faults);
    putfloat(telemetry+32,(float)pwm_l); putfloat(telemetry+36,(float)pwm_r);
    put32(telemetry+72,max_tick_cycles);
    snapshot_version++;
    IWDG->KR=0xaaaa;
}
void Hybrid_RxByte(uint8_t byte)
{
    uint16_t next=(uint16_t)((rx_write+1U)&(RX_SIZE-1U));
    if(next!=rx_read) { rx[rx_write]=byte; rx_write=next; } /* full: discard */
}
static void process_frame(void)
{
    uint8_t type=packet[3], status=0;
    uint16_t seq=get16(packet+4),len=get16(packet+6);
    float proposed[6];
    uint32_t mask;
    if(type==0x12 && len==0) { ack(type,seq,0); last_sent=0; return; }
    if(type==0x13 && len==0) {
        uint8_t body[68];
        memset(body,0,sizeof body);body[0]=type;put16(body+2,last_seq);
        if(sizeof(checkpoint_sha256)==65U) memcpy(body+4,checkpoint_sha256,64);
        else body[1]=3;
        send_frame(0x80,seq,body,sizeof body);return;
    }
    if(type==0x14 && len==0) {
        uint8_t body[64];
        memset(body,0,sizeof body);body[0]=type;
        mask=enter_critical();
        /* Schema 3 additionally requires stationary raw-gyro calibration.
         * Layout is unchanged so old diagnostics remain readable on PC. */
        put16(body+2,last_seq);put16(body+4,3);put16(body+6,Hybrid_CanArm()?1:0);
        put32(body+8,qualified_ticks);put32(body+12,cycles_us(max_tick_cycles));
        put32(body+16,cycles_us(max_sensor_cycles));put32(body+20,cycles_us(max_inference_cycles));
        put32(body+24,min_period_cycles==0xffffffffU?0:cycles_us(min_period_cycles));
        put32(body+28,cycles_us(max_period_cycles));put16(body+32,faults);
        body[34]=Stop_Flag;body[35]=controller;put32(body+36,first_fault_tick);
        put32(body+40,cycles_us(first_loop_cycles));put32(body+44,cycles_us(first_sensor_cycles));
        put32(body+48,cycles_us(first_inference_cycles));put32(body+52,cycles_us(first_period_cycles));
        put32(body+56,SystemCoreClock);put32(body+60,YB_PREFLIGHT_TICKS);
        leave_critical(mask);send_frame(0x80,seq,body,sizeof body);return;
    }
    if(type==0x15 && len==2) {
        uint8_t body[10+YB_TRACE_ROW_BYTES];
        unsigned index=get16(packet+8),oldest,physical;
        uint16_t body_len=10;
        memset(body,0,sizeof body);body[0]=type;
        mask=enter_critical();
        put16(body+2,last_seq);put16(body+4,1);put16(body+6,(uint16_t)trace_count);
        put16(body+8,(uint16_t)index);
        if(Stop_Flag!=1 || trace_live) body[1]=1;
        else if(index>=trace_count) body[1]=3;
        else {
            oldest=trace_count==YB_TRACE_ROWS?trace_next:0;
            physical=(oldest+index)%YB_TRACE_ROWS;
            memcpy(body+10,trace_rows[physical],YB_TRACE_ROW_BYTES);
            body_len=sizeof body;
        }
        leave_critical(mask);send_frame(0x80,seq,body,body_len);return;
    }
    if(type==0x16 && len==0) {
        uint8_t body[4+YB_TRACE_ROW_BYTES];
        memset(body,0,sizeof body);body[0]=type;
        mask=enter_critical();
        put16(body+2,last_seq);
        if(Stop_Flag!=1) body[1]=1;
        else if(!Hybrid_IsNN() || !did_inference || faults) body[1]=3;
        memcpy(body+4,trace_stage,YB_TRACE_ROW_BYTES);
        leave_critical(mask);send_frame(0x80,seq,body,sizeof body);return;
    }
    if(type==0x17 && len==0) {
        uint8_t body[64];unsigned i;
        memset(body,0,sizeof body);body[0]=type;
        mask=enter_critical();
        put16(body+2,last_seq);put16(body+4,1);put16(body+6,(uint16_t)gyro_cal_ready);
        put32(body+8,gyro_cal_count);put32(body+12,YB_GYRO_CAL_TICKS);put32(body+16,gyro_cal_resets);
        for(i=0;i<3;i++) { putfloat(body+20+4*i,gyro_bias[i]);putfloat(body+32+4*i,gyro_cal_std[i]); }
        putfloat(body+44,(cal_pitch_max-cal_pitch_min)/DEG_TO_RAD);
        putfloat(body+48,(cal_roll_max-cal_roll_min)/DEG_TO_RAD);
        putfloat(body+52,(cal_z_max-cal_z_min)/16384.0f);
        put32(body+56,(uint32_t)raw_data_ready_verified);put32(body+60,YB_ENCODER_WINDOW);
        leave_critical(mask);send_frame(0x80,seq,body,sizeof body);return;
    }
    if(type!=0x10 && type!=0x11) { ack(type,seq,5); return; }
    if((type==0x10 && len!=1)||(type==0x11 && len!=24)) { ack(type,seq,2); return; }
    if(type==0x11) memcpy(proposed,packet+8,24);
    mask=enter_critical();
    /* Re-check while interrupts disabled, so arming cannot race a gain write. */
    if(Stop_Flag!=1 || mode!=Normal || faults) status=1;
    else if(have_seq && (uint16_t)(seq-last_seq)>=0x8000U) status=4;
    else if(have_seq && seq==last_seq) status=4;
    else if(type==0x10 && packet[8]>YB_TZEJUN_NN) status=5;
    else if(type==0x11 && (controller!=YB_OLLAMA_PID ||
             !yb_gains_valid(proposed,active_gains,boot_gains))) status=3;
    else {
        if(type==0x10) {
            controller=packet[8];
            gyro_cal_reset();gyro_cal_resets=0;
            qualified_ticks=0;max_tick_cycles=0;max_sensor_cycles=0;max_inference_cycles=0;
            min_period_cycles=0xffffffffU;max_period_cycles=0;previous_tick_start=0;period_cycles=0;
            if(controller==YB_STOCK_PID) memcpy(active_gains,boot_gains,24);
        } else memcpy(active_gains,proposed,24);
            write_gains(active_gains); was_stopped=1;
        last_seq=seq; have_seq=1;
    }
    leave_critical(mask);
    ack(type,seq,status);
}
void Hybrid_Poll(void)
{
    uint16_t len;
    uint8_t ch, snapshot[76];
    uint32_t mask;
    while(rx_read!=rx_write) {
        ch=rx[rx_read]; rx_read=(uint16_t)((rx_read+1U)&(RX_SIZE-1U));
        if(used==0) { if(ch==0xaa) packet[used++]=ch; continue; }
        if(used==1 && ch!=0x55) { used=(ch==0xaa)?1:0; continue; }
        packet[used++]=ch;
        if(used<8) continue;
        len=get16(packet+6);
        if(packet[2]!=1 || len>24) { used=0; continue; }
        if(used==10U+len) {
            if(yb_crc16(packet+2,6U+len)==get16(packet+8+len)) process_frame();
            used=0;
        }
    }
    if(tick-last_sent<10U) return; /* 20 Hz, never stream inside the ISR */
    mask=enter_critical();
    memcpy(snapshot,telemetry,76);
    leave_critical(mask);
    last_sent=tick;
    send_frame(0x81,(uint16_t)tick,snapshot,76);
}
