/* 

*/

#include "log.h"
#include "param.h"
#include "math3d.h"

#include "stabilizer_types.h"

#define INTEGRAL_SATURATION 5000;
#define DT ;

typedef struct { 
    float acc_err;
    float prev_err;
    float kp;
    float ki;
    float kd;
} pid_gains_t; 

float compute_pid(pid_gains_t *gains, float state, float setpoint) {

    float error = state - setpoint; 

    float proportional = gains->kp*error; 

    if(gains->acc_err < INTEGRAL_SATURATION){
        gains->acc_err += error;
    }
    float integral = (gains->ki)*(gains->acc_err);

    // TODO check for NAN derivative
    float derivative = (gains->kd)*((error-gains->prev_err)/DT);

    gains->prev_err = error; 

    float control = proportional + integral+ derivative; 
    return control; 
}

