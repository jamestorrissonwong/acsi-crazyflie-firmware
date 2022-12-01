/* 

*/
#include "stabilizer_types.h"
#include "stabilizer.h"
#include <stdbool.h>

#define INTEGRAL_SATURATION 5000
#define DT 1/CONTROLLER_RATE

typedef enum{
    PID_THRUST, 
    PID_ROLL, 
    PID_PITCH, 
    PID_YAW,
    NUM_PID,
} PIDIndex;

// thrust, pitch/theta, roll/phi, yaw/psi


typedef struct { 
    float acc_err;
    float prev_err;
    float kp;
    float ki;
    float kd;
} pid_gains_t; 

// typedef struct {
//     float thrust;
//     float pitch;
//     float roll;
//     float yaw;
// } control_output_t;


// void gainsInit(pid_gains_t *gains, float kp, float ki, float kd);

// // Should extract 9 errors
// void computePID(pid_gains_t *gains, float state, float setpoint, float *control);

// void copterGainsInit(pid_gains_t **gains_arr, float *KP, float *KI, float *KD);

// void copterPIDWrapper(pid_gains_t **gains_arr, state_t *all_state, setpoint_t *all_setpoint, control_t *control);

void gainsInit(pid_gains_t *gains, float kp, float ki, float kd);

// Should extract 9 errors
void computePID(pid_gains_t *gains, float state, float setpoint, float *control);

void copterGainsInit(float *KP, float *KI, float *KD);

void customDummyInit(void);

bool customControllerTest(void);


void copterPIDWrapper(control_t *control, setpoint_t *all_setpoint, const sensorData_t *sensors, const state_t *all_state, const uint32_t tick);