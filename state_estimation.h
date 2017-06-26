// usefulincludes is a collection of common system includes for the lazy
// This is not necessary for roboticscape projects but here for convenience
#include <rc_usefulincludes.h> 
// main roboticscape API header
#include <roboticscape.h>

#include <string.h>

#include "main_types.h"

// 
int s_est_initialize (void);

//
int s_est_step       (s_est_data_t* s_est_data, acq_data_t* acq_data);

//
int s_est_end        (void);