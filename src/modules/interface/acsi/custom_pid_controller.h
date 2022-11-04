/* 

*/

#ifndef __CUSTOM_PID_CONTROLLER_H__
#define __CUSTOM_PID_CONTROLLER_H__

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

float compute_pid(pid_gains_t *gains, float state, float setpoint);

// void controllerPidInit(void);
// bool controllerPidTest(void);
// void controllerPid(control_t *control, setpoint_t *setpoint,
//                                          const sensorData_t *sensors,
//                                          const state_t *state,
//                                          const uint32_t tick);

#endif //__CUSTOM_PID_CONTROLLER_H__