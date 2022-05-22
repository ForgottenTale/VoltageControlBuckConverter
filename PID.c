#include "PID.h"

void PIDController_Init(PIDController *pid)
{
    pid->integrator = 0.0f;
    pid->prevError = 0.0f;
    pid->differentiator = 0.0f;
    pid->prevMeasurement = 0.0f;
    pid->out = 0.0f;
}

float PIDController_Update(PIDController *pid, float setpoint, float measurement)
{

    /*Error signal*/
    float error = setpoint - measurement;

    /* Proportional*/
    float proportional = pid->Kp * error;

    /* Integral*/
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

    // /* Anti-wind-up via integrator clamping */

    if (pid->out != pid->limMax && error * pid->out >= 0.0f)
    {
        pid->integrator = 0.0f;
    }
    /*Compute output and apply limits*/

    pid->out = proportional + pid->integrator;

    if (pid->out > pid->limMax)
    {

        pid->out = pid->limMax;
    }
    else if (pid->out < pid->limMin)
    {

        pid->out = pid->limMin;
    }

    /* Store error and measurement for later use */
    pid->prevError = error;
    pid->prevMeasurement = measurement;

    /* Return controller output */
    return pid->out;
}
