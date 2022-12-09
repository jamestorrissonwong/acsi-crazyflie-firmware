/* 

*/
#include "stabilizer_types.h"
#include "custom_pid_controller.h"
#include "num.h"
#include <math.h>
#include "log.h"
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
#define MIN_THRUST 20000.0f

static bool isInit;
static pid_gains_t *gains_arr[NUM_PID];

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;


static inline int16_t saturateSignedInt16(float in)
{
  // don't use INT16_MIN, because later we may negate it, which won't work for that value.
  if (in > INT16_MAX)
    return INT16_MAX;
  else if (in < -INT16_MAX)
    return -INT16_MAX;
  else
    return (int16_t)in;
}

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

    *control = proportional + integral + derivative; 
}

void copterGainsInit(float *KP, float *KI, float *KD){
    // pid_gains_t *thr_gains;
    // pid_gains_t *the_gains;
    // pid_gains_t *phi_gains;
    // pid_gains_t *psi_gains;

    // gains_arr = {thr_gains, the_gains, phi_gains, psi_gains};

    for (int i = 0; i < NUM_PID; i++){
        pid_gains_t *gains_curr = gains_arr[i]; 
        gainsInit(gains_curr, KP[i], KI[i], KD[i]);
    }

    isInit = true; 
}

void customDummyInit(void){
    return;
}

bool customControllerTest(void){
    return isInit;
}


void copterPIDWrapper(control_t *control, setpoint_t *all_setpoint, const sensorData_t *sensors, const state_t *all_state, const uint32_t tick) {
    // MOVE THESE CONSTANTS
    float g = 9.81;
    float additive_arr[NUM_PID] = {g, 0.0, 0.0, 0.0};
    float Ixx = 0.000023951*1000.0f;
    float Iyy = Ixx;
    float Izz = 0.00000362347*1000.0f;
    float m = 0.027;

    // control->thrust = -1;

    if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
        float phi = all_state->attitude.roll;
        float theta = all_state->attitude.pitch;

        float multiplicative_arr[NUM_PID] = {m/(cosf(phi)*cosf(theta)), Ixx, Iyy, Izz};//{m/cos(phi)/cos(theta), Ixx, Iyy, Izz};
        float temp_control[NUM_PID];
        float state[NUM_PID] = {all_state->position.z, all_state->attitude.pitch, all_state->attitude.roll, all_state->attitude.yaw};

        float pos_setpoint[NUM_PID] = {all_setpoint->position.z, all_setpoint->attitude.pitch, all_setpoint->attitude.roll, all_setpoint->attitude.yaw};
        // float vel_setpoint[4] = {all_setpoint->velocity.z, all_setpoint->attitudeRate.pitch, all_setpoint->attitudeRate.roll, all_setpoint->attitudeRate.yaw};

        for (int i = 0; i < NUM_PID; i++){
            pid_gains_t *gains_curr = gains_arr[i];
            computePID(gains_curr, state[i], pos_setpoint[i], &temp_control[i]);
            temp_control[i] = (temp_control[i]+additive_arr[i])*multiplicative_arr[i];
        }

        // if (all_setpoint->thrust != 0){
        // if (all_setpoint->velocity.z == 0) {
        if (all_setpoint->mode.z == modeDisable) {
            control->thrust = 0;
            control->pitch = 0;
            control->roll = 0;
            control->yaw = 0;

            cmd_thrust = control->thrust;
            cmd_pitch = control->pitch;
            cmd_roll = control->roll;
            cmd_yaw = control->yaw;
        }
        else {
            control->thrust = constrain(temp_control[0]*10000.0f, 0, UINT16_MAX);
            // if (control->thrust < MIN_THRUST){
            //     control->thrust = MIN_THRUST;
            // }
            control->pitch = saturateSignedInt16(temp_control[1]*10000.0f);
            control->roll = saturateSignedInt16(temp_control[2]*10000.0f);
            control->yaw = saturateSignedInt16(temp_control[3]*10000.0f);

            cmd_thrust = control->thrust;
            cmd_pitch = control->pitch;
            cmd_roll = control->roll;
            cmd_yaw = control->yaw;
        }
        
    }
}

/**
 * Logging variables for the command and reference signals for the
 *  PID controller
 */
LOG_GROUP_START(controller)
/**
 * @brief Thrust command
 */
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
/**
 * @brief Roll command
 */
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
/**
 * @brief Pitch command
 */
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
/**
 * @brief yaw command
 */
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_GROUP_STOP(controller)