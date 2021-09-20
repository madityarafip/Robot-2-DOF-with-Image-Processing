//Program Final Proyek Akhir Robotika Cerdas\\
//Robot Lengan dengan Image Processing\\
//Slave Robot (Roboard)\\

#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <stdlib.h>
#include <unistd.h>
// #include <roboard.h>
#include <iostream>


#define ROBOT_PORT  45000       // Roboard Port Number

unsigned short PORT_NO;
int  sock,addrLen;
struct sockaddr_in addr;
char  s1[1024];
int command;

// Structure
typedef struct  {
    float titikX_merah;
    float titikY_merah;
    float titikX_biru;
    float titikY_biru;
    float titikX_hijau;
    float titikY_hijau;
    int cmd;
    float sudut1;
    float sudut2;
    float deltaq;
} roboard_t;

roboard_t	data_recv;

int CreateTCPServerSocket(unsigned short port)
{
    int sock;                        /* socket to create */
    struct sockaddr_in echoServAddr; /* Local address */

    /* Create socket for incoming connections */
    if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
        return -1;
      
    /* Construct local address structure */
    memset(&echoServAddr, 0, sizeof(echoServAddr));   /* Zero out structure */
    echoServAddr.sin_family = AF_INET;                /* Internet address family */
    echoServAddr.sin_addr.s_addr = htonl(INADDR_ANY); /* Any incoming interface */
    echoServAddr.sin_port = htons(port);              /* Local port */

    /* Bind to the local address */
    if (bind(sock, (struct sockaddr *) &echoServAddr, sizeof(echoServAddr)) < 0) {
        perror("bind() failed");
        return -1;
    }

    return sock;
}

char *EncodeIPAddress(char *s, struct sockaddr_in *sin)
{
    sprintf(s, "port :%d", 
        htons(sin->sin_port)
    );
    return s;
}

int koneksi_init(void)
{
 //struct sockaddr_in addr;

    PORT_NO=ROBOT_PORT;
    
    if((sock = CreateTCPServerSocket(PORT_NO)) < 0) {
        exit(-1);
    }

    addrLen = sizeof(addr);
    printf("rx bound to address %s\n", EncodeIPAddress(s1, &addr));

}

int main(void){

	koneksi_init();
for(;;){
	if (recv(sock, &data_recv, sizeof(data_recv), 0)!=sizeof(data_recv)) {
        printf("Receive data not equal\n");		
    } 
    else 
    {
    command = data_recv.cmd;
    printf("cmd : %d\n",command);	
    if (command == 2){
    	printf("Robot Stopping...\n");
    	}
    else{
    	printf("Receive data :\n");
    	printf("X Merah : %.2f | Y Merah : %.2f || X Biru : %.2f | Y Biru : %.2f || X Hijau : %.2f | Y Hijau : %.2f\n", 
            data_recv.titikX_merah, data_recv.titikY_merah, 
            data_recv.titikX_biru, data_recv.titikY_biru, 
            data_recv.titikX_hijau, data_recv.titikY_hijau);
        printf("q1 : %.2f | q2 : %.2f | delta_q : %.2f\n", 
            data_recv.sudut1,
            data_recv.sudut2,
            data_recv.deltaq);
		}
	}
}
return 0;
}