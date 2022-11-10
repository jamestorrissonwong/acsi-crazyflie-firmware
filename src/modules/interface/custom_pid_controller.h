/* 

*/

#include "param.h"
#include "math3d.h"


#include "stabilizer_types.h"

#define INTEGRAL_SATURATION 5000
#define DT 1/CONTROLLER_RATE

// thrust, pitch/theta, roll/phi, yaw/psi
#define KP {100, 100, 100, 100}
#define KI {10, 10, 10, 10}
#define KD {1, 1, 1, 1}

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

void copterGainsInit(pid_gains_t **gains_arr);

void copterPIDWrapper(pid_gains_t **gains_arr, state_t *state, setpoint_t *setpoint, control_t *control);

// void controllerPidInit(void);
// bool controllerPidTest(void);
// void controllerPid(control_t *control, setpoint_t *setpoint,
//                                          const sensorData_t *sensors,
//                                          const state_t *state,
//                                          const uint32_t tick);
