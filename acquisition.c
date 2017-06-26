#include "acquisition.h"

rc_imu_data_t rc_imu_data; 
acq_data_t    acq_data;

void imu_interrupt(){
	memcpy(&(acq_data.imu),&rc_imu_data,sizeof(float)*9);
	memcpy(acq_data.q,rc_imu_data.dmp_quat,sizeof(float)*4);
	rc_read_barometer();
}
int acq_initialize (void){
	
	float ground_pressure = 0.0f;
	rc_imu_orientation_t orientation = ORIENTATION_X_FORWARD;
	rc_imu_config_t conf = rc_default_imu_config();
	conf.enable_magnetometer = 0;
	conf.dmp_sample_rate	 = 200;
	conf.orientation	     = orientation;
	
	// 183 hz sampling, HW filter off
	if(rc_initialize_barometer(BMP_OVERSAMPLE_1, BMP_FILTER_OFF)<0){
		fprintf(stderr,"ERROR: rc_initialize_barometer failed\n");
		return -1;
	}
	
	while(rc_read_barometer());

	ground_pressure = rc_bmp_get_pressure_pa();
	rc_set_sea_level_pressure_pa(ground_pressure);

	
	if(rc_initialize_imu_dmp(&rc_imu_data, conf)){
		fprintf(stderr,"rc_initialize_imu_failed\n");
		return -1;
	}
	
	rc_set_imu_interrupt_func(&imu_interrupt);
	
	return 0;
}

int acq_step       (acq_data_t* data){
    memcpy (data,&acq_data,sizeof(acq_data_t));
    acq_data.baro.height = rc_bmp_get_altitude_m();
}

int acq_end         (void){
    rc_power_off_imu();
}
