/* 

*/
#include "stabilizer_types.h"
#include "stabilizer.h"

#define INTEGRAL_SATURATION 5000
#define DT 1/CONTROLLER_RATE

// thrust, pitch/theta, roll/phi, yaw/psi


typedef struct { 
    float acc_err;
    float prev_err;
    float kp;
    float ki;
    float kd;
} pid_gains_t; 

typedef struct {
    float thrust;
    float pitch;
    float roll;
    float yaw;
} control_output_t;

void gainsInit(pid_gains_t *gains, float kp, float ki, float kd);

float compute_pid(pid_gains_t *gains, float state, float setpoint, float *control);

void copterGainsInit(pid_gains_t **gains_arr, float *KP, float *KI, float *KD);

void copterPIDWrapper(pid_gains_t **gains_arr, state_t *state, setpoint_t *setpoint, control_output_t *control);

// void controllerPidInit(void);
// bool controllerPidTest(void);
// void controllerPid(control_t *control, setpoint_t *setpoint,
//                                          const sensorData_t *sensors,
//                                          const state_t *state,
//                                          const uint32_t tick);
