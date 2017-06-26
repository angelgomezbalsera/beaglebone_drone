#ifndef __MAIN_TYPES_H__
#define __MAIN_TYPES_H__

#define x   0
#define dx  1
#define ddx 2
#define y   3
#define dy  4
#define ddy 5
#define z   6
#define dz  7
#define ddz 8

typedef struct imu_data{
    float accel[3];
    float gyro[3];
    float mag[3];
} imu_data_t;

typedef struct baro_data{
    float height;
} baro_data_t;

typedef struct acq_data{
    imu_data_t imu;
    baro_data_t baro;
    float q[4];
    float w[3];
} acq_data_t;

typedef struct attitude_loop_data{
    float moments[3];
    float rotor_speed[4];
} attitude_loop_data_t;

typedef struct attitude_loop_cmd{
    float q_dmd[4];
    float rotor_speed[4];
    char  control_ovr;
    float clg;
    float thr;
} attitude_loop_cmd_t;

typedef struct comms_data_tx{
    float q[4];
    float accel[3];
    float gyro[3];
    float mag[3];
    float moments[3];
    float rotor_speed[4];
    float tms[2];
    float flow_camera[5]; //dt,dx,dy,z,quality
    float baro;
    float q_dmd[4];
    float thrust;
} comms_data_tx_t;

typedef struct comms_data_rx{
    float control_gain;
    float ovr;
    float rotor_speed_ovr[4];
} comms_data_rx_t;

typedef struct s_est_data{
    float s[9];
    float Q[9];
    float R[9];
    float P[9];
    float F[9];
    float H[9];
    float S[9];
} s_est_data_t;

typedef struct dsm_data{
    char  dsm_enabled;
    float thrust;
    float attitude_demand[4];
} dsm_data_t;
#endif