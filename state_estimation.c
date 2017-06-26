#include "acquisition.h"

#define dt  0.005

const float s_init[9]   = {0,0,0,0,0,0,0,0,0};
const float Q_init[9]   = {1,0,0,1,0,0,1,0,0};
const float R_init[9]   = {1,0,0,1,0,0,1,0,0};

rc_imu_data_t rc_imu_data; 
acq_data_t    acq_data;
s_est_data_t  s_est_data;

int s_est_initialize (void){
    
    float F[9] = {1,   dt,   dt*dt/2,
                  0,    1,        dt,
                  0,    0,         1};
                  
    memcpy (s_est_data.s,s_init,sizeof(float)*9);
    memcpy (s_est_data.Q,Q_init,sizeof(float)*9);
    memcpy (s_est_data.R,R_init,sizeof(float)*9);
    memset (s_est_data.P,0,sizeof(float)*9);
    memcpy (s_est_data.F,F,sizeof(float)*9);
    memset (s_est_data.H,0,sizeof(float)*9);
    memset (s_est_data.S,0,sizeof(float)*9);
    
    
	return 0;
}

int s_est_step        (s_est_data_t* s_est_data, acq_data_t* acq_data){

}

int s_est_end         (void){

}
