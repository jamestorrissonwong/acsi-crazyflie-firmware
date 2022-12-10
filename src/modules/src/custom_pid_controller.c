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

static float T_KP;
static float T_KI;
static float T_KD;
static float T_acc;
static float T_prev;

static float R_KP;
static float R_KI;
static float R_KD;
static float R_acc;
static float R_prev;

static float P_KP;
static float P_KI;
static float P_KD;
static float P_acc;
static float P_prev;

static float Y_KP;
static float Y_KI;
static float Y_KD;
static float Y_acc;
static float Y_prev;

static bool isInit;
// static pid_gains_t *gains_arr[NUM_PID];

static float c_thrust;
static float c_roll;
static float c_pitch;
static float c_yaw;


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

void gainsInit(float tp, float ti, float td, 
               float rp, float ri, float rd, 
               float pp, float pi, float pd, 
               float yp, float yi, float yd) {
    T_KP = tp;
    T_KI = ti;
    T_KD = td;
    T_acc = 0;
    T_prev = 0;

    R_KP = rp;
    R_KI = ri;
    R_KD = rd;
    R_acc = 0;
    R_prev = 0;

    P_KP = pp;
    P_KI = pi;
    P_KD = pd;
    P_acc = 0;
    P_prev = 0;

    Y_KP = yp;
    Y_KI = yi;
    Y_KD = yd;
    Y_acc = 0;
    Y_prev = 0;

    isInit = true;
}

// void computePID(pid_gains_t *gains, float state, float setpoint, float *control) {

//     float error = state - setpoint; 

//     float proportional = gains->kp*error; 

//     if(gains->acc_err < INTEGRAL_SATURATION){
//         gains->acc_err += error;
//     }
//     float integral = (gains->ki)*(gains->acc_err);

//     // TODO check for NAN derivative
//     float derivative = (gains->kd)*((error-gains->prev_err)/DT);

//     gains->prev_err = error; 

//     *control = proportional + integral + derivative; 
// }

// void copterGainsInit(float *KP, float *KI, float *KD){
//     // pid_gains_t *thr_gains;
//     // pid_gains_t *the_gains;
//     // pid_gains_t *phi_gains;
//     // pid_gains_t *psi_gains;

//     // gains_arr = {thr_gains, the_gains, phi_gains, psi_gains};

//     // for (int i = 0; i < NUM_PID; i++){
//     //     pid_gains_t *gains_curr = gains_arr[i]; 
//     //     gainsInit(gains_curr, KP[i], KI[i], KD[i]);
//     // }

//     isInit = true; 
// }

void customDummyInit(void){
    return;
}

bool customControllerTest(void){
    return isInit;
}


void customPID(control_t *control, setpoint_t *all_setpoint, const sensorData_t *sensors, const state_t *all_state, const uint32_t tick) {
    // MOVE THESE CONSTANTS
    float g = 9.81f;
    // float additive_arr[NUM_PID] = {g, 0.0, 0.0, 0.0};
    float Ixx = 0.000023951f;
    float Iyy = Ixx;
    float Izz = 0.00000362347f;
    float m = 0.0322f;

    // control->thrust = -1;

    if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
        
        if (all_setpoint->mode.z == modeDisable) {
            control->thrust = 0;
            control->pitch = 0;
            control->roll = 0;
            control->yaw = 0;

            c_thrust = control->thrust;
            c_pitch = control->pitch;
            c_roll = control->roll;
            c_yaw = control->yaw;
        }
        else {
            float phi = all_state->attitude.roll;
            float theta = all_state->attitude.pitch;

            float t_err = (all_setpoint->position.z - all_state->position.z);
            float p_err = (all_setpoint->attitude.pitch - all_state->attitude.pitch);
            float r_err = (all_setpoint->attitude.roll - all_state->attitude.roll);
            float y_err = (all_setpoint->attitude.yaw - all_state->attitude.yaw);

            float thrust = (T_KP*t_err) + (T_KI*T_acc) + (T_KD*(t_err-T_prev)/DT);
            float pitch = (P_KP*p_err) + (P_KI*P_acc) + (P_KD*(p_err-P_prev)/DT);
            float roll = (R_KP*r_err) + (R_KI*R_acc) + (R_KD*(r_err-R_prev)/DT);
            float yaw = (Y_KP*y_err) + (Y_KI*Y_acc) + (Y_KD*(y_err-Y_prev)/DT);

            thrust = (thrust + g)*(m/(cosf(phi)*cosf(theta)));
            pitch = (pitch)*(Iyy);
            roll = (roll)*(Ixx);
            yaw = (yaw)*(Izz);

            thrust *= 10000.0f;
            pitch *= 1000.0f;
            roll *= 1000.0f;
            yaw *= 1000.0f;

            control->thrust = constrain(thrust, 0, UINT16_MAX);
            control->pitch = saturateSignedInt16(-pitch);
            control->roll = saturateSignedInt16(roll);
            control->yaw = saturateSignedInt16(-yaw);

            c_thrust = control->thrust;
            c_pitch = control->pitch;
            c_roll = control->roll;
            c_yaw = control->yaw;
        }
        
    }
}

/**
 * Logging variables for the command and reference signals for the
 *  PID controller
 */
// LOG_GROUP_START(controller)
// /**
//  * @brief Thrust command
//  */
// LOG_ADD(LOG_FLOAT, cmd_thrust, &c_thrust)
// /**
//  * @brief Roll command
//  */
// LOG_ADD(LOG_FLOAT, cmd_roll, &c_roll)
// /**
//  * @brief Pitch command
//  */
// LOG_ADD(LOG_FLOAT, cmd_pitch, &c_pitch)
// /**
//  * @brief yaw command
//  */
// LOG_ADD(LOG_FLOAT, cmd_yaw, &c_yaw)
// LOG_GROUP_STOP(controller)