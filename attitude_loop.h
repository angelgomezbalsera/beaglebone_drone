#include <rc_usefulincludes.h> 
// main roboticscape API header
#include <roboticscape.h>

#include "main_types.h"
// 
int attitude_loop_initialize (attitude_loop_cmd_t *q_dmd,attitude_loop_data_t* attitude_loop_data);

//
int attitude_loop_step       (attitude_loop_cmd_t *q_dmd,attitude_loop_data_t* attitude_loop_data, acq_data_t *acq_data);

//
int attitude_loop_end        (void);