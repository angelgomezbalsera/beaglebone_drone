// usefulincludes is a collection of common system includes for the lazy
// This is not necessary for roboticscape projects but here for convenience
#include <rc_usefulincludes.h> 
// main roboticscape API header
#include <roboticscape.h>

#include <string.h>

#include "main_types.h"

// 
int acq_initialize (void);

// 
int acq_calibrate  (void);

//
int acq_step       (acq_data_t* data);

//
int acq_end        (void);