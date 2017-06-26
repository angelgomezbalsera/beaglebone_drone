#include "comms_link.h" 

#define client_port 3000
#define server_port 3001
#define buffer_size 21

struct sockaddr_in server_address;
struct sockaddr_in local_address;
int server_handle, local_handle;

char buffer[buffer_size];

int comms_initialize (void){

        server_handle = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        local_handle = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if ((server_handle <= 0)||(local_handle <= 0))
        {
                printf("failed to create socket\n");
                return 1;
        }
        printf("sockets successfully initialized\n");
        memset((char *) &server_address, 0, sizeof(server_address));
        memset((char *) &local_address, 0, sizeof(local_address));
        
        server_address.sin_family = AF_INET;
        server_address.sin_addr.s_addr = inet_addr("192.168.7.1");
        server_address.sin_port = htons(client_port);

        local_address.sin_family = AF_INET;
        local_address.sin_port = htons(server_port);
        memset(&local_address.sin_addr, INADDR_ANY, sizeof(struct in_addr));
        
        if (bind(local_handle, (const struct sockaddr*) &local_address, sizeof(struct sockaddr_in)) < 0){
                printf("failed to bind socket (%s)\n", strerror(errno)); // Cannot assign requested address
                return 1;
        }
        return 0;
}


//
int comms_step (comms_data_tx_t* data_tx,comms_data_rx_t* data_rx, char tx_trigger){
        int read_bytes;
        if (tx_trigger){
                int sent_bytes = sendto(server_handle, data_tx, sizeof(comms_data_tx_t), 0, (const struct sockaddr*) &server_address, sizeof(struct sockaddr_in));

                if (sent_bytes != sizeof(comms_data_tx_t))
                {
                        return 3;
                }
        }
        read_bytes = recv(local_handle, buffer, buffer_size, MSG_PEEK | MSG_DONTWAIT);
        if (read_bytes >= buffer_size){
                read_bytes = recv(local_handle, buffer, buffer_size, 0);
                memcpy(data_rx,buffer,buffer_size);
        }
}
//
int comms_end        (void){
        return 0;
}

