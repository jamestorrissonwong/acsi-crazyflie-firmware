#include "rls_mass_estimator.h"

void rls_init(massEst_t *me){
    me->lambda = 0.8;            // Forgetting Factor
    me->theta = (1.0f)/(0.027f); // Initial mass guess
    me->P = 10.0f;               // Covariance 
    me->L = 1;                   // Initial gain

    me->clamp = 0;

    return;
}

void update_phi(control_t *control, state_t *state, massEst_t *me){
    float T = control->thrust/100000.0f; // unit conversion
    float zvel = state->velocity.z;


    float zacc = 9.81f*(state->acc.z) + 9.81f;

    float theta = PI*(state->attitude.pitch)/180.0f; 

    float phi3 = T*arm_cos_f32(theta) - (((float)(1.03e-6))*zvel);

    me->phi = phi3; 
    me->y = zacc;
}

float rls_estimate(control_t *control, state_t *state, massEst_t *me){
    update_phi(control, state, me);

    // Calculate gain
    me->L = 1.0f/(me->lambda + (me->phi)*(me->P)*(me->phi));
    me->L = me->L*(me->P*me->phi);

    // If gain goes outside bounds, do not use the next estimate for mass
    if (me->L < 0 || me->L > 1){
        me->clamp = 1;
    }
    else {
        me->clamp = 0;
    }

    // Covariance update
    me->P = ((1-(me->L*me->phi))*me->P)/(me->lambda);

    // Calculate mass (we saw a constant error experimentally)
    me->theta = me->theta + (me->L)*(me->y - ((me->phi)*(me->theta)));
    float zest = (1.0f/(me->theta))-0.0092f;

    /* NaN is the only floating point value that does NOT equal itself.
    * Therefore if n != n, then it is NaN. */
    if (zest!=zest) {
        zest = -1.0;
    }

    return zest;
}