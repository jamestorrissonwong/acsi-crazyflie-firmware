#include "cf_math.h"
#include "stabilizer_types.h"
#include "rls_mass_estimator.h"
#include "custom_pid_controller.h"

typedef struct{
    float lambda;
    __attribute__((aligned(4))) float y;
    arm_matrix_instance_f32 ym;

    __attribute__((aligned(4))) float L[2][1]; // Is this correct?
    arm_matrix_instance_f32 Lm;

    __attribute__((aligned(4))) float phi[2][1]; // Is this correct?
    arm_matrix_instance_f32 phim;

    __attribute__((aligned(4))) float theta[2][1]; // Is this correct?
    arm_matrix_instance_f32 thetam;

    __attribute__((aligned(4))) float P[1][1]; // Is this correct?
    arm_matrix_instance_f32 Pm;

} massEst_t;

void rls_init(massEst_t *me){
    me->lambda = 0.5; 
    me->y = 27;
    me->ym.numCols = 1;
    me->ym.numRows = 1;
    me->ym.pData = (float*)me->y;

    me->Lm.numCols = 2;
    me->Lm.numRows = 1;
    me->Lm.pData = (float*)me->L;

    me->phim.numCols = 2;
    me->phim.numRows = 1;
    me->phim.pData = (float*)me->phi;

    me->thetam.numCols = 2;
    me->thetam.numRows = 1;
    me->thetam.pData = (float*)me->theta;

    me->Pm.numCols = 1;
    me->Pm.numRows = 1;
    me->Pm.pData = (float*)me->P;
    return;
}

void update_phi(control_output_t *control, state_t *state, massEst_t *me){
    float T = control->thrust;
    float x = state->position.x;
    float z = state->position.z;

    float xacc = state->acc.x;
    float zacc = state->acc.z;

    float theta = state->attitude.pitch; // is this in rad? 

    float phi1 = -T*arm_sin_f32(theta) - (9.18e-7*xacc);
    float phi2 = T*arm_cos_f32(theta) - (1.03e-6*zacc);

    me->phi[0] = phi1;
    me->phi[1] = phi2;
}

float rls_estimate(control_output_t *control, state_t *state, massEst_t *me){
    // regressor update?
    update_phi(control, state, me);
    
    // Temporary matrices
    NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmp1d[2];
    static arm_matrix_instance_f32 tmp1m = {2, 1, tmp1d}; // P*phi
    NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmp2d[2];
    static arm_matrix_instance_f32 tmp2m = {1, 2, tmp2d}; // phi'
    NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmp3d[1];
    static arm_matrix_instance_f32 tmp3m = {1, 1, tmp3d}; // phi' * P * phi

    NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmpId[4];
    static arm_matrix_instance_f32 tmpIm = {2, 2, tmpId}; // 
    NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmp4d[4];
    static arm_matrix_instance_f32 tmp4m = {2, 2, tmp4d}; // 
    NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmp5d[4];
    static arm_matrix_instance_f32 tmp5m = {2, 2, tmp5d}; // 

    NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmp6d[4];
    static arm_matrix_instance_f32 tmp6m = {2, 2, tmp6d}; // 
    NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmp7d[4];
    static arm_matrix_instance_f32 tmp7m = {2, 2, tmp7d}; // 
    NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmp8d[4];
    static arm_matrix_instance_f32 tmp8m = {2, 2, tmp8d}; // 

    // L = (P*phi)*(lambda + (phi'*P*phi))^-1;
    mat_mult(me->Pm, me->phim, tmp1m); // P*phi
    mat_trans(me->phim, tmp2m); // phi'
    mat_mult(tmp2m, tmp1m, tmp3m); // phi' * P * phi
    float den = 1.0/(me->lambda + tmp3m->pData); // (phi' * P * phi)^-1
    mat_scale(tmp1m,den,me->Lm);


    // P = (I - L*phi')P*1/lambda 
    tmpId[0] = 1;
    tmpId[3] = 1;

    mat_mult(me->Lm, tmp2m, tmp4m); //L*phi'
    arm_sub_f32(tmpIm, tmp4m, tmp5m); //(I - L*phi')
    mat_mult(tmp5m, me->Pm, me->Pm); //(I - L*phi')P
    mat_scale(me->Pm, 1.0/(me->lambda), me->Pm); 

    // theta = theta + L(y-phi'*theta)
    mat_mult(tmp2m, me->theta, tmp6m); //phi'*theta
    arm_sub_f32(me->ym, tmp6m, tmp7m); // y-phi'*theta
    mat_mult(me->Lm, tmp7m, tmp8m); // L(y-phi'*theta)
    arm_mat_add_f32(me->theta, tmp8m, me->theta);
    
    mat_mult(tmp2m, me->theta, me->ym);

    return 1.0/(me->y);
}