/*******************************************************************************
* rc_project_template.c
*
* This is meant to be a skeleton program for robotics cape projects. 
* Change this description and file name 
*******************************************************************************/

// usefulincludes is a collection of common system includes for the lazy
// This is not necessary for roboticscape projects but here for convenience
#include <rc_usefulincludes.h> 
// main roboticscape API header
#include <roboticscape.h>

#include "main_types.h"
// HW acquisition library
#include "acquisition.h"
// Communications library
#include "comms_link.h"
// Attitude control loop library
#include "attitude_loop.h"
// State estimation library
#include "state_estimation.h"

void dsm_initialize(dsm_data_t* dsm_data);
void dsm_step(dsm_data_t* dsm_data);

/*******************************************************************************
* int main() 
*
* This template main function contains these critical components
* - call to rc_initialize() at the beginning
* - main while loop that checks for EXITING condition
* - rc_cleanup() at the end
*******************************************************************************/
int main(){
	uint32_t time_counter = 0;
	uint64_t timestamp = 0, time_reference = 0;
	acq_data_t           acq_data;
	comms_data_tx_t      comms_data_tx;
	comms_data_rx_t      comms_data_rx;
	attitude_loop_data_t attitude_loop_data;
	attitude_loop_cmd_t  attitude_loop_cmd;
	dsm_data_t			 dsm_data;
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to initialize rc_initialize(), are you root?\n");
		return 0;
	}
	
	dsm_initialize(&dsm_data);	
	acq_initialize();
	comms_initialize();
	attitude_loop_initialize(&attitude_loop_cmd,&attitude_loop_data);
	s_est_initialize();
	// done initializing so set state to RUNNING
	rc_set_state(RUNNING); 
	// Keep looping until state changes to EXITING
	timestamp = rc_nanos_since_boot();
	
	while(rc_get_state()!=EXITING){
	
		comms_step(&comms_data_tx,&comms_data_rx,1);//(time_counter%5)==0);
		dsm_step(&dsm_data);

		attitude_loop_cmd.control_ovr = comms_data_rx.ovr>0.5f;
		attitude_loop_cmd.thr         = dsm_data.thrust;
		attitude_loop_cmd.clg         = comms_data_rx.control_gain;

		memcpy(attitude_loop_cmd.rotor_speed,comms_data_rx.rotor_speed_ovr,sizeof(float)*4);
		memcpy(attitude_loop_cmd.q_dmd,dsm_data.attitude_demand,sizeof(float)*4);
		acq_step(&acq_data);
		attitude_loop_step(&attitude_loop_cmd,&attitude_loop_data,&acq_data);
		
		memcpy(comms_data_tx.q,acq_data.q,sizeof(float)*4);
		memcpy(comms_data_tx.accel,acq_data.imu.accel,sizeof(float)*9);
		memcpy(comms_data_tx.moments,attitude_loop_data.moments,sizeof(float)*3);
		memcpy(comms_data_tx.rotor_speed,attitude_loop_data.rotor_speed,sizeof(float)*4);
		comms_data_tx.baro   = acq_data.baro.height;
		comms_data_tx.thrust = dsm_data.thrust;
		memcpy(comms_data_tx.q_dmd,dsm_data.attitude_demand,sizeof(float)*4);
		
		// Time functions
		time_counter++;
		timestamp += 5000000;
		comms_data_tx.tms[0] = rc_nanos_since_boot();

		rc_nanosleep(timestamp-comms_data_tx.tms[0]);
	}
	
	acq_end();
	comms_end();
	// exit cleanly
	rc_cleanup(); 
	return 0;
}

void dsm_initialize(dsm_data_t* dsm_data){
	int i = 10;
    dsm_data->thrust = 0;
    dsm_data->attitude_demand[0] = 1;
    dsm_data->attitude_demand[1] = 0;
    dsm_data->attitude_demand[2] = 0;
    dsm_data->attitude_demand[3] = 0;
	if(rc_initialize_dsm()){
		printf("ERROR: failed to initialize DSM link\n");
		dsm_data->dsm_enabled = 0;
		return;
	}
	printf("Waiting for DSM link\n");
	while(rc_is_new_dsm_data()==0){
		
		if (--i == 0){
                	printf("DSM link discarded\n");
               	    dsm_data->dsm_enabled = 0;
               	    return;
		}
		rc_usleep(50000);
	}
    printf("DSM link detected\n");
    dsm_data->dsm_enabled = 1;
}

void dsm_step(dsm_data_t* dsm_data){
	const float thrust_gain   = 0.5;   // 100% max thrust
	const float yaw_rate_gain = 0.0005;// +-0.1rad/s*dt max
	const float pitch_gain    = -1;     // +-1 rad max
	const float roll_gain     = 1;
	static float euler[3] = {0,0,0};
	
	if (dsm_data->dsm_enabled == 1){
		if (rc_is_new_dsm_data()){
			dsm_data->thrust = (rc_get_dsm_ch_normalized(1)+1)*thrust_gain;
			euler[2] += rc_get_dsm_ch_normalized(4)*yaw_rate_gain;
			euler[1]  = rc_get_dsm_ch_normalized(3)*roll_gain;
			euler[0]  = rc_get_dsm_ch_normalized(2)*pitch_gain;
			rc_tb_to_quaternion_array(euler,dsm_data->attitude_demand);
			
		}		
	}

}
