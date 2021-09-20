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
#include <roboard.h>
#include <iostream>

#define PI 3.14159265
#define ROBOT_PORT  45000       // Roboard Port Number
#define RTD     180.0/PI                   // Konversi radian ke sudut

unsigned short PORT_NO;
int  sock,addrLen;
struct sockaddr_in addr;
char  s1[1024];
int command;
float q1 = 0;
float q2 = 0;
float delta_q = 0;
float error=0;
float error_old=0;
float sum_error=0;
float Kp=0.01;
float Ki=0.0001;
float Kd=0.005;
float MV=0;
int posisi_target;
unsigned long home[32]  = {0L};
unsigned long frame[32] = {0L};

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
    float posobjek;
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
void control_robot(void){
    q1 = data_recv.sudut1;
    q2 = data_recv.sudut2;
    delta_q = q2-q1;
    error = delta_q - error_old;
    sum_error = delta_q + sum_error;
    MV = Kp*delta_q + Ki*sum_error + Kd*error;
    if(abs(sum_error)>10000) {sum_error=0;} //integral anti wind up
    if (MV>180) MV=180;
    if (MV<-180) MV=-180;
}
void mukul_objek(void){
    frame[3] = 900;
    rcservo_MoveOne(RCSERVO_PINS4, frame[3], 20L);
    frame[3] = 1500;
    rcservo_MoveOne(RCSERVO_PINS4, frame[3], 20L);
    frame[3] = 900;
    rcservo_MoveOne(RCSERVO_PINS4, frame[3], 20L);
    frame[3] = 1500;
    rcservo_MoveOne(RCSERVO_PINS4, frame[3], 20L);
    frame[3] = 900;
    rcservo_MoveOne(RCSERVO_PINS4, frame[3], 20L);              
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
    
    // first set the correct RoBoard version
    roboio_SetRBVer(RB_100);

	koneksi_init();

    rcservo_SetServo(RCSERVO_PINS3, RCSERVO_SERVO_DEFAULT_NOFB);     // select the servo model on pin S1 as non-feedback servo
    rcservo_SetServo(RCSERVO_PINS4, RCSERVO_SERVO_DEFAULT_NOFB);     // select the servo model on pin S2 as non-feedback servo
    rcservo_SetServo(RCSERVO_PINS7, RCSERVO_SERVO_DEFAULT_NOFB);     // select the servo model on pin S2 as non-feedback servo
    if (rcservo_Init(RCSERVO_USEPINS7 + RCSERVO_USEPINS4 + RCSERVO_USEPINS3) == false)  // set PWM/GPIO pins S1 & S2 as Servo mode
    {
        printf("ERROR: fail to init RC Servo lib (%s)!\n", roboio_GetErrMsg());
        return -1;
    }

    home[6] = home[2] = home[3] = 1500L;  // set the initial home position of all servos as 1500us
    rcservo_EnterPlayMode_HOME(home);  // enter Action Playing Mode for moving servos

for(;;){
	if (recv(sock, &data_recv, sizeof(data_recv), 0)!=sizeof(data_recv)) {
        printf("Receive data not equal\n");		
    } 
    else 
    {
    printf("Receive data :\n");
    command = data_recv.cmd;
    printf("cmd : %d\n",command);        
    printf("X Merah : %.2f | Y Merah : %.2f || X Biru : %.2f | Y Biru : %.2f || X Hijau : %.2f | Y Hijau : %.2f\n", 
            data_recv.titikX_merah, data_recv.titikY_merah, 
            data_recv.titikX_biru, data_recv.titikY_biru, 
            data_recv.titikX_hijau, data_recv.titikY_hijau);
    printf("q1 : %.2f | q2 : %.2f | delta_q : %.2f\n", 
            data_recv.sudut1,
            data_recv.sudut2,
            data_recv.deltaq);
    posisi_target = data_recv.posobjek;

    if (command == 2){
        frame[2] = 1900L;    // move the servo on pin S1 to position 1900us
        frame[3] = 900L;    // move the servo on pin S2 to position 900us
        frame[6] = 900L;   // move the servo on pin S2 to position 900us
        rcservo_MoveTo(frame, 20L);  // move servos to the target positions in 30ms
    	printf("Robot Stopping...\n");
        rcservo_Close();
    	}
    else if (command == 3){
        frame[2] = 1700L;   // move the servo on pin S1 to position 900us
        frame[3] = 1500L;   // move the servo on pin S2 to position 900us
        frame[6] = 1500L;   // move the servo on pin S2 to position 900us
        rcservo_MoveTo(frame, 20L);  // move servos to the target positions in 30ms
        printf("Servo Aktif!\n");
    }  
    else if (command == 4){
        frame[6] = (unsigned long) (posisi_target*(2000-1000)/180)+1500;
        if (frame[6]>1800L) frame[6]=1800L;
        rcservo_MoveOne(RCSERVO_PINS7, frame[6], 20L);
        q2 = data_recv.sudut2;        
        control_robot();     
        frame[2]=(unsigned long) (MV*(2000-1000)/180)+frame[2];
        if (frame[2]>1900L) frame[2]=1900L;
        if (frame[2]<900L)frame[2]=900L;
        rcservo_MoveOne(RCSERVO_PINS3, frame[2], 20L); 
        if(data_recv.deltaq < 5 && data_recv.deltaq > -5){
        mukul_objek();    
        }
        if(data_recv.deltaq >= 5 || data_recv.deltaq <= -5){
            frame[3] = 1500;
            rcservo_MoveOne(RCSERVO_PINS4, frame[3], 100L);            
        }
    }  
    else if (command == 5){
        frame[6] = (unsigned long) (posisi_target*(2000-1000)/180)+1500;
        if (frame[6]<1100L) frame[6]=1100L;
        rcservo_MoveOne(RCSERVO_PINS7, frame[6], 20L);
        q2 = data_recv.sudut2;    
        control_robot();
        frame[2]=(unsigned long) (MV*(2000-1000)/180)+frame[2];
        if (frame[2]>1900L) frame[2]=1900L;
        if (frame[2]<900L)frame[2]=900L;
        rcservo_MoveOne(RCSERVO_PINS3, frame[2], 20L);         
        if(data_recv.deltaq < 5 && data_recv.deltaq > -5){
        mukul_objek();    
        }
        if(data_recv.deltaq >= 5 || data_recv.deltaq <= -5){
            frame[3] = 1500;
            rcservo_MoveOne(RCSERVO_PINS4, frame[3], 100L);            
        }         
    }
    else{
        printf("Servo Diaktifkan...\n");
        rcservo_EnterPlayMode_HOME(home);  // enter Action Playing Mode for moving servos
		}
	}
}
return 0;
}