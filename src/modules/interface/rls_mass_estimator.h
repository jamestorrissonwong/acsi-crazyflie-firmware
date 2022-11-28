#include "cf_math.h"
#include "stabilizer_types.h"
#include "custom_pid_controller.h"
#include "static_mem.h"

typedef struct{
    float lambda;
    __attribute__((aligned(4))) float *y;
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

void rls_init(massEst_t *me);

void update_phi(control_output_t *control, state_t *state, massEst_t *me);

float rls_estimate(control_output_t *control, state_t *state, massEst_t *me);