/*
 *
 * IF :
 *   - extern uint16_t display_channels[8];   // 1000..2000us RC pulse values
 *   - mpu_gyro_read() fills Gx,Gy,Gz in deg/s
 *   - extern TIM_HandleTypeDef htim2, htim3;
 *
 * Behaviour: rate controller (roll/pitch) -> elevon mix -> TIM compare outputs.
 */

#include "acro.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdint.h>

/* --- external resources from your main/MPU --- */
extern uint16_t display_channels[8]; //ppm values ip
extern float Gx, Gy, Gz;             /* gyro in deg/s */
void mpu_gyro_read(void);            /* you must implement this to update Gx,Gy,Gz */

//timers in main
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

//pid struc
typedef struct {
    float kp, ki, kd;
    float integrator;
    float last_err;
    float i_limit;
    float out_limit;
} pid_t;

//PID
static void pid_init(pid_t* p, float kp, float ki, float kd, float i_limit, float out_limit) {
    if (!p) return;
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->integrator = 0.0f; p->last_err = 0.0f;
    p->i_limit = i_limit; p->out_limit = out_limit;
}
static float pid_update(pid_t* p, float err, float dt) {
    if (!p || dt <= 0.0f) return 0.0f;
    float P = p->kp * err;
    p->integrator += err * dt;
    if (p->integrator > p->i_limit) p->integrator = p->i_limit;
    if (p->integrator < -p->i_limit) p->integrator = -p->i_limit;
    float I = p->ki * p->integrator;
    float D = p->kd * ((err - p->last_err) / dt);
    p->last_err = err;
    float out = P + I + D;
    if (out > p->out_limit) out = p->out_limit;
    if (out < -p->out_limit) out = -p->out_limit;
    return out;
}
static void pid_reset(pid_t* p) { if (p) { p->integrator = 0.0f; p->last_err = 0.0f; } }

//config
#define PID_OUT_LIMIT 400.0f
#define MAX_ROLL_RATE_DPS   360.0f
#define MAX_PITCH_RATE_DPS  360.0f

/* Servo timing */
static const uint32_t SERVO_MIN_US = 500;
static const uint32_t SERVO_MAX_US = 2500;
static const uint32_t SERVO_MID_US = 1500;
static const float SERVO_TRAVEL_US = 400.0f;           /* +/- travel from mid (tweak) */

/* servo directions */
// flip if reverse
static const int LEFT_ELEVON_DIR  = +1;
static const int RIGHT_ELEVON_DIR = -1;

/* static state */
static pid_t pid_roll, pid_pitch;
static volatile uint8_t acro_enabled = 0;

/* helpers */
static inline float clampf_local(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* Convert rc pulse (1000..2000) to -1..+1 stick (center 1500) */
static float stick_norm_from_us(uint16_t us) {
    /* protect */
    if (us < 800) us = 800;
    if (us > 2200) us = 2200;
    float v = ((float)us - 1500.0f) / 500.0f; /* -1..+1 */
    return clampf_local(v, -1.0f, 1.0f);
}

/* Convert rc pulse (1000..2000) to throttle 0..1 */
static float throttle_norm_from_us(uint16_t us) {
    if (us < 900) us = 900;
    if (us > 2100) us = 2100;
    float v = ((float)us - 1000.0f) / 1000.0f;
    return clampf_local(v, 0.0f, 1.0f);
}


void acro_enable(void)  { acro_enabled = 1; pid_reset(&pid_roll); pid_reset(&pid_pitch); }
void acro_disable(void) { acro_enabled = 0; pid_reset(&pid_roll); pid_reset(&pid_pitch); }
uint8_t acro_is_enabled(void) { return acro_enabled; }

void acro_set_gains(float kp_r, float ki_r, float kd_r,
                    float kp_p, float ki_p, float kd_p)
{
    pid_init(&pid_roll,  kp_r, ki_r, kd_r, 200.0f, PID_OUT_LIMIT);
    pid_init(&pid_pitch, kp_p, ki_p, kd_p, 200.0f, PID_OUT_LIMIT);
}

void acro_init(void)
{
    /* conservative defaults */
    pid_init(&pid_roll,  0.012f, 0.002f, 0.0005f, 200.0f, PID_OUT_LIMIT);
    pid_init(&pid_pitch, 0.012f, 0.002f, 0.0005f, 200.0f, PID_OUT_LIMIT);

    acro_enabled = 0;

    /* set neutral outputs (1500us) and throttle low */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SERVO_MID_US);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, SERVO_MID_US);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO_MIN_US);
}

/* dt in seconds */
void acro_run(float dt)
{
    if (dt <= 0.0f) return;

    /* Channel mapping
       index 0 -> roll stick
       index 1 -> pitch stick
       index 2 -> throttle
       index 3 -> Yaw
    */

    uint16_t ch_roll  = display_channels[0];
    uint16_t ch_pitch = display_channels[1];
    uint16_t ch_thr   = display_channels[2];
    /* yaw available as display_channels[3] if needed */

    if (!acro_enabled) {
        /* passthrough: throttle -> thr channel; keep servos centered */
        uint32_t thr_pulse_us = (uint32_t)clampf_local((float)ch_thr, (float)SERVO_MIN_US, (float)SERVO_MAX_US);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, thr_pulse_us);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SERVO_MID_US);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, SERVO_MID_US);
        return;
    }

    /* 1) read sticks normalized */
    float thr = throttle_norm_from_us(ch_thr);    /* 0..1 */
    float stick_roll  = stick_norm_from_us(ch_roll);  /* -1..1 */
    float stick_pitch = stick_norm_from_us(ch_pitch); /* -1..1 */

    /* 2) read gyro (single read) */
    mpu_gyro_read(); /* must fill Gx,Gy,Gz in deg/s */
    float gx = Gx;
    float gy = Gy;

    /* 3) desired rates */
    float desired_roll_rate  = stick_roll  * MAX_ROLL_RATE_DPS;
    float desired_pitch_rate = stick_pitch * MAX_PITCH_RATE_DPS;

    /* 4) rate error */
    float err_roll  = desired_roll_rate  - gx;
    float err_pitch = desired_pitch_rate - gy;

    /* 5) pid outputs */
    float out_roll  = pid_update(&pid_roll,  err_roll,  dt);
    float out_pitch = pid_update(&pid_pitch, err_pitch, dt);

    /* 6) normalize to -1..1 */
    float norm_roll  = clampf_local(out_roll  / PID_OUT_LIMIT, -1.0f, 1.0f);
    float norm_pitch = clampf_local(out_pitch / PID_OUT_LIMIT, -1.0f, 1.0f);

    /* 7) elevon mix (left/right) */
    float left = norm_pitch + norm_roll;
    float right = norm_pitch - norm_roll;
    left = clampf_local(left, -1.0f, 1.0f) * LEFT_ELEVON_DIR;
    right = clampf_local(right, -1.0f, 1.0f) * RIGHT_ELEVON_DIR;

    /* 8) convert to pulse (1500 +/- travel). */
    uint32_t left_pulse_us  = (uint32_t)(SERVO_MID_US + left  * SERVO_TRAVEL_US);
    uint32_t right_pulse_us = (uint32_t)(SERVO_MID_US + right * SERVO_TRAVEL_US);

    /* throttle passthrough: map thr 0..1 -> 1000..2000us */
    uint32_t thr_pulse_us = (uint32_t)(1000.0f + thr * 1000.0f);

    /* clamp pulses */
    if (left_pulse_us  < SERVO_MIN_US) left_pulse_us = SERVO_MIN_US;
    if (left_pulse_us  > SERVO_MAX_US) left_pulse_us = SERVO_MAX_US;
    if (right_pulse_us < SERVO_MIN_US) right_pulse_us = SERVO_MIN_US;
    if (right_pulse_us > SERVO_MAX_US) right_pulse_us = SERVO_MAX_US;
    if (thr_pulse_us < SERVO_MIN_US) thr_pulse_us = SERVO_MIN_US;
    if (thr_pulse_us > SERVO_MAX_US) thr_pulse_us = SERVO_MAX_US;

    /* apply outputs using same timers your main expects */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, left_pulse_us);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, right_pulse_us);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, thr_pulse_us);
}

void acro_reset_integrators(void)
{
    pid_reset(&pid_roll);
    pid_reset(&pid_pitch);
}

/* optional RTOS task wrapper - run at ~200Hz */
// void acro_task(void *argument)
// {
//     uint32_t last = HAL_GetTick();
 //   for (;;) {
  //      uint32_t now = HAL_GetTick();
 //       float dt = (now - last) * 1e-3f;
  //      if (dt <= 0.0f) dt = 0.005f;
 //       last = now;

 //       acro_run(dt);

 //       osDelay(5); /* ~200Hz */
 //   }
//}
