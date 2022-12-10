#include "rls_mass_estimator.h"



void rls_init(massEst_t *me){
    me->lambda = 0.8; 
    
    // me->ym.numRows = 3;
    // me->ym.numCols = 1;
    // me->ym.pData = (float*)me->y;

    // me->Lm.numRows = 3;
    // me->Lm.numCols = 1;
    // me->Lm.pData = (float*)me->L;

    // me->phim.numRows = 3;
    // me->phim.numCols = 3;
    // me->phim.pData = (float*)me->phi;

    // me->thetam.numRows = 3;
    // me->thetam.numCols = 1;
    // me->thetam.pData = (float*)me->theta;

    // me->theta[0][0] = (1.0f)/(0.0322f);
    // me->theta[1][0] = (1.0f)/(0.0322f);
    // me->theta[2][0] = (1.0f)/(0.0322f);

    // me->Pm.numCols = 3;
    // me->Pm.numRows = 3;
    // me->Pm.pData = (float*)me->P;

    // me->P[0][0] = 0.015f;
    // me->P[1][1] = 0.015f;
    // me->P[2][2] = 0.015f;

    me->theta = (1.0f)/(0.0322f);
    me->P = 0.015f;
    me->L = 1; 


    return;
}

void update_phi(control_t *control, state_t *state, massEst_t *me){
    float T = control->thrust*1000000.0f;
    // float xvel = state->velocity.x;
    // float yvel = state->velocity.y;
    float zvel = state->velocity.z;

    // float xacc = 9.81f*(state->acc.x);     // need to convert to m/s2
    // float yacc = 9.81f*(state->acc.y);
    float zacc = 9.81f*(state->acc.z) + 9.81f;

    float theta = PI*(state->attitude.pitch)/180; // is this in rad? 
    // float phi = PI*(state->attitude.roll)/180; 

    // float phi1 = -T*arm_sin_f32(theta) - (((float)(9.18e-7))*xvel);
    // float phi2 = T*arm_cos_f32(theta)*arm_sin_f32(phi) - (((float)(9.18e-7))*yvel);
    float phi3 = T*arm_cos_f32(theta) - (((float)(1.03e-6))*zvel);

    // me->phi[0][0] = phi1;
    // me->phi[1][1] = phi2;
    // me->phi[2][2] = phi3;

    // me->y[0][0] = xacc;
    // me->y[1][0] = yacc;
    // me->y[2][0] = zacc;
    me->phi = phi3; 
    me-> y = zacc;
}

float rls_estimate(control_t *control, state_t *state, massEst_t *me){
    // regressor update?
    update_phi(control, state, me);

    // P 3x3
    // L 3x3 
    // phi 3x3
    // theta 3x1
    // y 3x1
    
    // Temporary matrices
    // NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmp1d[9];
    // static arm_matrix_instance_f32 tmp1m = {3, 3, tmp1d}; // P*phi
    // NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmp2d[9];
    // static arm_matrix_instance_f32 tmp2m = {3, 3, tmp2d}; // phi'
    // NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmp3d[9];
    // static arm_matrix_instance_f32 tmp3m = {3, 3, tmp3d}; // phi' * P * phi

    // NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmpId[9];
    // static arm_matrix_instance_f32 tmpIm = {3, 3, tmpId}; // I
    // NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmp4d[9];
    // static arm_matrix_instance_f32 tmp4m = {3, 3, tmp4d}; // L*phi'
    // NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmp5d[9];
    // static arm_matrix_instance_f32 tmp5m = {3, 3, tmp5d}; // (I - L*phi')

    // NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmp6d[3];
    // static arm_matrix_instance_f32 tmp6m = {3, 1, tmp6d}; // phi'*theta
    // NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmp7d[3];
    // static arm_matrix_instance_f32 tmp7m = {3, 1, tmp7d}; // y-phi'*theta
    // NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmp8d[3];
    // static arm_matrix_instance_f32 tmp8m = {3, 1, tmp8d}; // L(y-phi'*theta)

    // // L = (P*phi)*(lambda + (phi'*P*phi))^-1;
    // mat_mult(&me->Pm, &me->phim, &tmp1m); // P*phi
    // mat_trans(&me->phim, &tmp2m); // phi'
    // mat_mult(&tmp2m, &tmp1m, &tmp3m); // phi' * P * phi
    
    // //float den = 1.0f/(me->lambda + *tmp3m.pData); // (phi' * P * phi)^-1
    // mat_scale(&tmp1m, den, &me->Lm);


    // // P = (I - L*phi')P*1/lambda 
    // tmpId[0] = 1;
    // tmpId[4] = 1;
    // tmpId[8] = 1;

    // mat_mult(&me->Lm, &tmp2m, &tmp4m); // L*phi'
    // arm_mat_sub_f32(&tmpIm, &tmp4m, &tmp5m); // (I - L*phi')
    // mat_mult(&tmp5m, &me->Pm, &me->Pm); // (I - L*phi')P
    // mat_scale(&me->Pm, (float)1.0/(me->lambda), &me->Pm); 

    // // theta = theta + L(y-phi'*theta)
    // mat_mult(&tmp2m, &me->thetam, &tmp6m); // phi'*theta
    // arm_mat_sub_f32(&me->ym, &tmp6m, &tmp7m); // y-phi'*theta
    // mat_mult(&me->Lm, &tmp7m, &tmp8m); // L(y-phi'*theta)
    // arm_mat_add_f32(&me->thetam, &tmp8m, &me->thetam);
    
    // mat_mult(&tmp2m, &me->thetam, &me->ym);


    // // float xest = (float)1.0/(me->y[0][0]);
    // // float yest = (float)1.0/(me->y[1][0]);
    // float zest = (float)1.0/(me->theta[2][0]);

    me->L = 1.0f/(me->lambda + (me->phi)*(me->P)*(me->phi));
    me->L = me->L*(me->P*me->phi);

    me->P = (1-(me->L*me->phi))*me->P*me->lambda;

    me->theta = me->theta + (me->L)*(me->y - ((me->phi)*(me->theta)));

    float zest = 1.0f/(me->theta);

    /* NaN is the only floating point value that does NOT equal itself.
    * Therefore if n != n, then it is NaN. */
    if (zest!=zest) {
        zest = -1.0;
    }

    return zest;
}