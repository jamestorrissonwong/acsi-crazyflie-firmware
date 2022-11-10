/* 

*/
#include "stabilizer_types.h"
#include "custom_pid_controller.h"
#include <math3d.h>

// PID should have 3 gains for each control output
// typedef struct { 
//     float acc_err;
//     float prev_err;
//     float kp;
//     float ki;
//     float kd;
// } pid_gains_t; 

// typedef struct {
//     float thrust;
//     float pitch;
//     float roll;
//     float yaw;
// } control_output_t

void gainsInit(pid_gains_t *gains, float kp, float ki, float kd) {
    gains->acc_err = 0;
    gains->prev_err = 0;
    gains->kp = kp;
    gains->ki = ki;
    gains->kd = kd;
}

// Should extract 9 errors
void computePID(pid_gains_t *gains, float state, float setpoint, float *control) {

    float error = state - setpoint; 

    float proportional = gains->kp*error; 

    if(gains->acc_err < INTEGRAL_SATURATION){
        gains->acc_err += error;
    }
    float integral = (gains->ki)*(gains->acc_err);

    // TODO check for NAN derivative
    float derivative = (gains->kd)*((error-gains->prev_err)/DT);

    gains->prev_err = error; 

    *control = proportional + integral+ derivative; 
}

void copterGainsInit(pid_gains_t **gains_arr, float *KP, float *KI, float *KD){
    // pid_gains_t *thr_gains;
    // pid_gains_t *the_gains;
    // pid_gains_t *phi_gains;
    // pid_gains_t *psi_gains;

    // gains_arr = {thr_gains, the_gains, phi_gains, psi_gains};

    for (int i = 0; i < 4; i++){
        pid_gains_t *gains_curr = gains_arr[i]; 
        gainsInit(gains_curr, KP[i], KI[i], KD[i]);
    }
}

void copterPIDWrapper(pid_gains_t **gains_arr, state_t *all_state, setpoint_t *all_setpoint, control_t *control) {
    float g = 9.81;
    float zero = 0.0;
    float additive_arr[4] = {g, zero, zero, zero};
    float Ixx = .000023951;
    float Iyy = Ixx;
    float Izz = 0.00000362347;
    float m = 0.027

    float phi = all_state->attitude.roll;
    float theta = all_state->attitude.pitch;

    float multiplicative_arr[4] = {m/(math.cos(phi)*math.cos(theta)), Ixx, Iyy, Izz};//{m/cos(phi)/cos(theta), Ixx, Iyy, Izz};
    float temp_control[4];
    float state[4] = {all_state->position.z, all_state->attitude.pitch, all_state->attitude.roll, all_state->attitude.yaw};

    float pos_setpoint[4] = {all_setpoint->position.z, all_setpoint->attitude.pitch, all_setpoint->attitude.roll, all_setpoint->attitude.yaw};
    // float vel_setpoint[4] = {all_setpoint->velocity.z, all_setpoint->attitudeRate.pitch, all_setpoint->attitudeRate.roll, all_setpoint->attitudeRate.yaw};

    for (int i = 0; i < 4; i++){
        pid_gains_t *gains_curr = gains_arr[i];
        computePID(gains_curr, state[i], pos_setpoint[i], &temp_control[i]);
        temp_control[i] = (temp_control[i]+additive_arr[i])*multiplicative_arr[i];
    }

    control->thrust = temp_control[0];
    control->pitch = temp_control[1];
    control->roll = temp_control[2];
    control->yaw = temp_control[3];
}