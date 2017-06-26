#include "attitude_loop.h"
#include "math.h"

#define cpx -0.1
#define cpy -0.1
#define cpz -0.1
#define cdx -0.001
#define cdy -0.001
#define cdz -0.001
#define rad2pi 0.0175
#define x_arm 1.0f /0.105
#define y_arm 1.0f /0.105
//#define z_arm 0.25f/0.105
#define z_arm 0.0f/0.105
//#define drag  23.6f
#define drag  0.25f

rc_vector_t attitude_demand;
rc_vector_t attitude_estimation;
rc_vector_t attitude_error;
rc_matrix_t A;
rc_matrix_t moments;

// 
int attitude_loop_initialize (attitude_loop_cmd_t *q_dmd,attitude_loop_data_t* cmd_data){
	rc_send_esc_pulse_normalized_all(0);
	
    q_dmd->q_dmd[0] = 1; q_dmd->q_dmd[1] = 0; q_dmd->q_dmd[2] = 0; q_dmd->q_dmd[3] = 0;
    q_dmd->rotor_speed[0] = 0; q_dmd->rotor_speed[1] = 0; q_dmd->rotor_speed[2] = 0; q_dmd->rotor_speed[3] = 0;
 
    attitude_demand     = rc_empty_vector();
	attitude_estimation = rc_empty_vector();
	rc_vector_zeros(&attitude_demand    , 4);
	rc_vector_zeros(&attitude_estimation, 4);
	rc_vector_zeros(&attitude_error     , 4);
    A = rc_empty_matrix(); 	rc_matrix_zeros(&A,4,4);
    moments = rc_empty_matrix(); rc_matrix_zeros(&moments,4,1);
    rc_set_matrix_entry(&A,0,0, y_arm);rc_set_matrix_entry(&A,0,1,-x_arm);rc_set_matrix_entry(&A,0,2, z_arm);rc_set_matrix_entry(&A,0,3,0.25f/drag);
    rc_set_matrix_entry(&A,1,0, y_arm);rc_set_matrix_entry(&A,1,1, x_arm);rc_set_matrix_entry(&A,1,2,-z_arm);rc_set_matrix_entry(&A,1,3,0.25f/drag);
    rc_set_matrix_entry(&A,2,0,-y_arm);rc_set_matrix_entry(&A,2,1,-x_arm);rc_set_matrix_entry(&A,2,2,-z_arm);rc_set_matrix_entry(&A,2,3,0.25f/drag);
    rc_set_matrix_entry(&A,3,0,-y_arm);rc_set_matrix_entry(&A,3,1, x_arm);rc_set_matrix_entry(&A,3,2, z_arm);rc_set_matrix_entry(&A,3,3,0.25f/drag);

    
}

const float thr_max = 0.95;
const float thr_min =   0;
//
int attitude_loop_step       (attitude_loop_cmd_t *q_dmd,attitude_loop_data_t* cmd_data, acq_data_t* acq_data){
    rc_vector_t q_est_c = rc_empty_vector();
    if (q_dmd->control_ovr){
        cmd_data->rotor_speed[0] = fmaxf(fminf(q_dmd->rotor_speed[0],thr_max),thr_min);
        cmd_data->rotor_speed[1] = fmaxf(fminf(q_dmd->rotor_speed[1],thr_max),thr_min);
        cmd_data->rotor_speed[2] = fmaxf(fminf(q_dmd->rotor_speed[2],thr_max),thr_min);
        cmd_data->rotor_speed[3] = fmaxf(fminf(q_dmd->rotor_speed[3],thr_max),thr_min);
    }
    else{
        rc_vector_from_array(&attitude_demand,q_dmd->q_dmd,4);
        rc_vector_from_array(&attitude_estimation,acq_data->q,4);
        rc_quaternion_conjugate(attitude_estimation, &q_est_c);
        rc_quaternion_multiply(attitude_demand,q_est_c,&attitude_error); 
        if (attitude_error.d[0] < 0){
            rc_quaternion_conjugate_inplace(&attitude_error);
        }
        cmd_data->moments[0] = cpx*attitude_error.d[1] + cdx*acq_data->imu.gyro[0]*rad2pi; 
        cmd_data->moments[1] = cpx*attitude_error.d[2] + cdx*acq_data->imu.gyro[1]*rad2pi; 
        cmd_data->moments[2] = cpx*attitude_error.d[3] + cdx*acq_data->imu.gyro[2]*rad2pi; 
        cmd_data->moments[0] *= q_dmd->clg;
        cmd_data->moments[1] *= q_dmd->clg;
        cmd_data->moments[2] *= q_dmd->clg;
        
        rc_set_matrix_entry(&moments,0,0, cmd_data->moments[0]);
        rc_set_matrix_entry(&moments,1,0, cmd_data->moments[1]);
        rc_set_matrix_entry(&moments,2,0, cmd_data->moments[2]);
        rc_set_matrix_entry(&moments,3,0, q_dmd->thr);

        rc_multiply_matrices(A,moments,&moments);
        cmd_data->rotor_speed[0] = fmaxf(fminf(rc_get_matrix_entry(moments,0,0),thr_max),thr_min);
        cmd_data->rotor_speed[1] = fmaxf(fminf(rc_get_matrix_entry(moments,1,0),thr_max),thr_min);
        cmd_data->rotor_speed[2] = fmaxf(fminf(rc_get_matrix_entry(moments,2,0),thr_max),thr_min);
        cmd_data->rotor_speed[3] = fmaxf(fminf(rc_get_matrix_entry(moments,3,0),thr_max),thr_min);
            
    }
    rc_send_esc_pulse_normalized(2, cmd_data->rotor_speed[0]);
    rc_send_esc_pulse_normalized(4, cmd_data->rotor_speed[1]);
    rc_send_esc_pulse_normalized(3, cmd_data->rotor_speed[2]);
    rc_send_esc_pulse_normalized(1, cmd_data->rotor_speed[3]);
    return 0;
}

//
int attitude_loop_end        (void){
    rc_send_esc_pulse_normalized_all(-0.1);
    return 0;
}