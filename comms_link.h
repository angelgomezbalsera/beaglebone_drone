#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <errno.h>

#include "main_types.h"
// 
int comms_initialize (void);

//
int comms_step       (comms_data_tx_t* data_tx,comms_data_rx_t* data_rx,char tx_trigger);

//
int comms_end        (void);
