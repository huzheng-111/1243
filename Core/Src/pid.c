#include "pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd, 
              float out_max, float out_min)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->output_max = out_max;
    pid->output_min = out_min;
    pid->target = 0;
    pid->error = 0;
    pid->prev_error = 0;
    pid->integral = 0;
    pid->output = 0;
    pid->integral_max = out_max;
    pid->enable = true;
}

void PID_SetTarget(PID_t *pid, float target)
{
    pid->target = target;
}

float PID_Calculate(PID_t *pid, float feedback, float dt)
{
    if (!pid->enable || dt <= 0) {
        return 0;
    }

    pid->prev_error = pid->error;
    pid->error = pid->target - feedback;

    /* Proportional term */
    float p_term = pid->Kp * pid->error;

    /* Integral term with anti-windup */
    pid->integral += pid->error * dt;
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    }
    if (pid->integral < -pid->integral_max) {
        pid->integral = -pid->integral_max;
    }
    float i_term = pid->Ki * pid->integral;

    /* Derivative term */
    float d_term = pid->Kd * (pid->error - pid->prev_error) / dt;

    /* Calculate output */
    pid->output = p_term + i_term + d_term;

    /* Limit output */
    if (pid->output > pid->output_max) {
        pid->output = pid->output_max;
    }
    if (pid->output < pid->output_min) {
        pid->output = pid->output_min;
    }

    return pid->output;
}

void PID_Reset(PID_t *pid)
{
    pid->error = 0;
    pid->prev_error = 0;
    pid->integral = 0;
    pid->output = 0;
}

void PID_Enable(PID_t *pid, bool enable)
{
    pid->enable = enable;
    if (!enable) {
        PID_Reset(pid);
    }
}
