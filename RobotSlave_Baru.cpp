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
#define RTD     180.0/PI        // Konversi radian ke sudut

unsigned short PORT_NO;
int  sock,addrLen;
struct sockaddr_in addr;
char  s1[1024];
int command;
float q1 = 0;
float q2 = 0;
float delta_q = 0;
float error_q = 10;
float d_error=0;
float error_lama=0;
float sum_error=0;
float Kp=0.02;
float Ki=0.0015;//0.001
float Kd=0.0015;//0.005
float u=0;
int posisi_target = 0;
int perubahan_sudut = 0;
int count_pukul = 0;
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
    int posobjek;
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
    error_lama = error_q;
    error_q    = q2 - q1;
    d_error    = error_q - error_lama;
    sum_error  = error_q + sum_error;
    u = Kp*error_q + Ki*sum_error + Kd*d_error;
    if(abs(sum_error)>10000) {sum_error=0;} //integral anti wind up
    if (u>180) u=180;
    if (u<-180) u=-180;
}
void gerak_objek (void){
    posisi_target = data_recv.posobjek;
    frame[6] = (unsigned long) (posisi_target*(2000-1000)/180)+1500;
    if (frame[6]>1800L) frame[6]=1800L;
    if (frame[6]<1100L) frame[6]=1100L;
    rcservo_MoveOne(RCSERVO_PINS7, frame[6], 20L);
}
void mukul_objek(void){
    frame[3] = 900L;
    rcservo_MoveOne(RCSERVO_PINS4, frame[3], 100L);
    frame[3] = 1500L;
    rcservo_MoveOne(RCSERVO_PINS4, frame[3], 100L);            
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

// set the initial home position of all servos
    printf("Servo Diaktifkan...\n");
    usleep(1000);
    home[2] = home[3] = home[6] = 1500L;
    rcservo_EnterPlayMode_HOME(home);  // enter Action Playing Mode for moving servos
    printf("Servo Aktif!\n");     

    printf("Tekan ENTER untuk menggerakkan robot dan objek ke kanan!\n"); getchar();
    frame[2] = 900L;   // move the servo on pin S1 to position 900us
    frame[3] = 1800L;   // move the servo on pin S2 to position 1800us
    frame[6] = 1800L;   // move the servo on pin S2 to position 1800us
    rcservo_MoveTo(frame, 5000L);  // move servos to the target positions in 2s

    printf("Tekan ENTER untuk menggerakkan robot dan objek ke tengah!\n"); getchar();
    frame[2] = 1500L;   // move the servo on pin S1 to position 1500us
    frame[3] = 1500L;   // move the servo on pin S2 to position 1500us
    frame[6] = 1500L;   // move the servo on pin S2 to position 1500us
    rcservo_MoveTo(frame, 5000L);  // move servos to the target positions in 1s

    printf("Tekan ENTER untuk menggerakkan robot dan objek ke kiri!\n"); getchar();
    frame[2] = 1900L;    // move the servo on pin S1 to position 1900us
    frame[3] = 1500L;    // move the servo on pin S2 to position 900us
    frame[6] = 900L;   // move the servo on pin S7 to position 900us
    rcservo_MoveTo(frame, 5000L);  // move servos to the target positions in 2s

    printf("Tekan ENTER untuk menstabilkan objek!\n"); getchar();
    frame[6] = 1500L;   // move the servo on pin S7 to position 900us
    rcservo_MoveOne(RCSERVO_PINS7, frame[6], 7500L);    
    printf("Servo Aktif dan Siap digunakan!\n"); 
    u = 0;

    for(;;){
        //Receive Data
        if (recv(sock, &data_recv, sizeof(data_recv), 0)!=sizeof(data_recv)) {
        printf("Receive data not equal\n");     
        } 
        else {
        printf("Receive data :\n");
        posisi_target = data_recv.posobjek;
        perubahan_sudut = -posisi_target;     
        command = data_recv.cmd;
        q1 = data_recv.sudut1;
        q2 = data_recv.sudut2;
        delta_q = q2 - q1;
        printf("cmd : %d\n",command);        
        printf("Mx : %.2f | My : %.2f || Bx : %.2f | By : %.2f || Hx : %.2f | Hy : %.2f\n", 
            data_recv.titikX_merah, data_recv.titikY_merah, 
            data_recv.titikX_biru, data_recv.titikY_biru, 
            data_recv.titikX_hijau, data_recv.titikY_hijau);
        printf("q1 : %.2f | q2 : %.2f | delta_q : %.2f || Objek : %lu || Perubahan Posisi Objek : %d\n",
            q1,
            q2,
            delta_q,
            frame[6],
            perubahan_sudut);
        }

        gerak_objek();      
        if (abs(delta_q)>5){
            control_robot();
            //Gerak
            frame[2]=(unsigned long) (u*(2000-1000)/180)+frame[2];
            if (frame[2]>1900L) frame[2]=1900L;
            if (frame[2]<1000L) frame[2]=1000L;
            rcservo_MoveOne(RCSERVO_PINS3, frame[2], 20L); 
            count_pukul = 0;                         
        } else {
            count_pukul++;
            if (count_pukul%30==0){               
                printf("Memukul Objek.......\n");
                mukul_objek();
                printf("Objek Terpukul!\n");  
            }                             
        }

        if (command==2){
            frame[2] = 1900L;    // move the servo on pin S1 to position 1900us
            frame[3] = 900L;    // move the servo on pin S2 to position 900us
            frame[6] = 900L;   // move the servo on pin S7 to position 900us
            rcservo_MoveTo(frame, 1000L);  // move servos to the target positions in 30ms
            printf("Robot Stopping...\n");
            count_pukul = 0.0;
        } 


    }
rcservo_Close();    
return 0;
}   