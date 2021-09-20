//Program Final Proyek Akhir Robotika Cerdas\\
//Robot Lengan dengan Image Processing\\
//Master Robot (PC)\\


#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/fast_math.hpp"
#include "highgui.h"
#include <math.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <unistd.h>

using namespace cv;
using namespace std;

#define PI 3.14159265
#define DTR     PI/180.0                   // Konversi sudut ke radian
#define RTD     180.0/PI                   // Konversi radian ke sudut

#define ROBOT_PORT  45000   // Port Roboard
int OpenThr=0;
int robo_ready = 0;
int objek = 0;
int merah_x, merah_y, hijau_x, hijau_y, biru_x, biru_y, koor_merah, koor_hijau, koor_biru, posisirobot, posisistarget;
float q1, q2, rr, rt, xr, xt, delta_theta;

int  sock;
unsigned short PORT_NO;
char iprobo[INET_ADDRSTRLEN]="127.0.0.1";
//char iprobo[INET_ADDRSTRLEN]="192.168.0.20";

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
} pc_t;

pc_t data_send;

typedef struct {
    int iLowH;
    int iHighH;
    int iLowS;
    int iHighS;
    int iLowV;
    int iHighV;
} batasTh;

std::vector<cv::Point> CariRectangle(Mat imgThresholded) {
std::vector< std::vector<cv::Point> > contours;
std::vector<cv::Point> points;
cv::findContours(imgThresholded, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
	for (size_t i=0; i<contours.size(); i++) {
		for (size_t j = 0; j < contours[i].size(); j++) {
        	cv::Point p = contours[i][j];
        	points.push_back(p);
        }
    }
    return points;
}

int CreateTCPClientSocket(unsigned short port, const char *servIP)
{
   int sock;                        // Socket
   struct sockaddr_in echoServAddr; /* Local address */

    /* Create socket for incoming connections */
    if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0)return -1;
      
    /* Construct the server address structure */
    memset(&echoServAddr, 0, sizeof(echoServAddr));     /* Zero out structure */
    echoServAddr.sin_family      = AF_INET;             /* Internet address family */
    echoServAddr.sin_addr.s_addr = inet_addr(servIP);   /* Server IP address */
    echoServAddr.sin_port        = htons(port); /* Server port */

    // To Enable Broadcast message 255.255.255.255 (some network adapter cann't receive unless broadcast)
    int on = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (char *)&on, sizeof(on));

    /* Establish the connection to the echo server */
    if (connect(sock, (struct sockaddr *) &echoServAddr, sizeof(echoServAddr)) < 0) {
        perror("connect() failed");
        printf("You are currently not connected, please check your connection first! \n");
        return -1;
    }

    return sock;
}

int koneksi_robot(char *ip_rbt)
{
    PORT_NO=ROBOT_PORT; 
    if((sock = CreateTCPClientSocket(PORT_NO, ip_rbt)) < 0) {
            exit(-1);
    }
    printf("tx bound to address %d\n", PORT_NO);
    robo_ready =1;
}

int send_robot(void)
{

    if(send(sock, &data_send, sizeof(data_send), 0) < 0) {
        close(sock);
        printf("Data Sending Failed, Please Check the Connection!\n");
        robo_ready =0;

    }
    else
    {
        printf("Sending Data to Roboard\r");
    }
}

int main( int argc, char** argv ){
    VideoCapture cap(0); //capture the video from web cam//1webcam
	
    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the web cam" << endl;
         return -1;
    }
	Mat image_frame, image_HSV, image_ColorM, image_ColorH, image_ColorB, image_fusion;
	Point center_1, center_2, center_3;

//                         ( B,   G,   R )
    Scalar color_1 = Scalar( 0, 255, 255 ); 
    Scalar color_2 = Scalar( 255, 255, 0 ); 
    Scalar color_3 = Scalar( 255, 0, 255 ); 

    //Lampu Nyala
    //batasTh merah   = {  0,  67, 168, 255, 192, 255}; //merah
    batasTh merah   = {147, 195, 160, 255, 170, 255}; //pink
    batasTh hijau   = { 48,  93, 136, 255,  85, 255};
    batasTh biru    = { 87, 122, 155, 255, 132, 255};

/*    //Lampu Mati
    //batasTh merah   = {  0,  67, 168, 255, 192, 255}; //merah
    batasTh merah   = {  0,  43,  63, 255, 108, 255}; //pink
    batasTh hijau   = { 23,  76,  27, 255, 111, 255};
    batasTh biru    = { 87, 111, 124, 184,  64, 125};*/

    printf("!!!-----------------------Main Menu-------------------------------!!!\n");
    printf("Tekan ESC untuk menutup semua window")
    printf("Tekan 'm' -> Adjust treshold warna merah\n");
    printf("Tekan 'h' -> Adjust treshold warna hijau\n");
    printf("Tekan 'j' -> Adjust treshold warna Biru\n");
    printf("Tekan 'f' -> Munculkan Threshold Gabungan\n");
    printf("=====================================================================\n");
    printf("Tekan 'k' -> Terhubung dengan Roboard dan Robot\n");
    printf("Tekan '6' -> Menggerakkan Objek ke Kanan\n");
    printf("Tekan '4' -> Menggerakkan Objek ke Kiri\n");
    printf("Tekan 's' -> Berhenti Mengirim Data\n");

    batasTh *aktifh = &hijau;
    batasTh *aktifm = &merah;
    batasTh *aktifb = &biru;

    while(true){
    	bool bSuccess = cap.read(image_frame); // read a new frame from video
		if (!bSuccess) {
        	cout << "Cannot read a frame from video stream" << endl;
        	break;
        }

        // Generates HSV Matrix
        cv::cvtColor(image_frame,   // Input image
                     image_HSV, // output image in HSV
                     CV_BGR2HSV); // constant refering to color space transformation
        
        // Filtering image for colors with HSV Threshold
        aktifm = &merah;
        cv::inRange(image_HSV, 
                    cv::Scalar(aktifm->iLowH, aktifm->iLowS, aktifm->iLowV), 
                    cv::Scalar(aktifm->iHighH, aktifm->iHighS, aktifm->iHighV), 
                    image_ColorM); //Threshold the image
        aktifh = &hijau;
        cv::inRange(image_HSV, 
                    cv::Scalar(aktifh->iLowH, aktifh->iLowS, aktifh->iLowV), 
                    cv::Scalar(aktifh->iHighH, aktifh->iHighS, aktifh->iHighV), 
                    image_ColorH); //Threshold the image
        aktifb = &biru;
        cv::inRange(image_HSV, 
                    cv::Scalar(aktifb->iLowH, aktifb->iLowS, aktifb->iLowV), 
                    cv::Scalar(aktifb->iHighH, aktifb->iHighS, aktifb->iHighV), 
                    image_ColorB); //Threshold the image

        if(OpenThr==1){
            cv::imshow("Thresholded imgThrHijau", image_ColorH); //show the thresholded image  
        }
        if(OpenThr==2){
            cv::imshow("Thresholded imgThrMerah", image_ColorM); //show the thresholded image    
        }
        if(OpenThr==3){
            cv::imshow("Thresholded imgThrBiru", image_ColorB); //show the thresholded image 
        }

        cv::erode(image_ColorM,  image_ColorM, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
        cv::dilate(image_ColorM, image_ColorM, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
        cv::dilate(image_ColorM, image_ColorM, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
        cv::erode(image_ColorM,  image_ColorM, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
        
        cv::erode(image_ColorH,  image_ColorH, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
        cv::dilate(image_ColorH, image_ColorH, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
        cv::dilate(image_ColorH, image_ColorH, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
        cv::erode(image_ColorH,  image_ColorH, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
        
        cv::erode(image_ColorB,  image_ColorB, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
        cv::dilate(image_ColorB, image_ColorB, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
        cv::dilate(image_ColorB, image_ColorB, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
        cv::erode(image_ColorB,  image_ColorB, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

        std::vector<cv::Point> pointMerah, pointHijau, pointBiru;
        pointMerah = CariRectangle(image_ColorM);
        pointHijau = CariRectangle(image_ColorH);
        pointBiru  = CariRectangle(image_ColorB);

        if (pointMerah.size() > 0) {
            cv::Rect brectM = cv::boundingRect(cv::Mat(pointMerah).reshape(2));
            merah_x = brectM.x + (brectM.width/2);
            merah_y = brectM.y + (brectM.height/2);
            cv::rectangle(image_frame, brectM.tl(), brectM.br(), cv::Scalar(100, 100, 200), 2, CV_AA);
            center_1 = Point(merah_x,merah_y);
            data_send.titikX_merah = merah_x;
            data_send.titikY_merah = merah_y;
            koor_merah = 1;
        } else {
        	koor_merah = 0;
        }

        if (pointHijau.size() > 0) {
            cv::Rect brectH = cv::boundingRect(cv::Mat(pointHijau).reshape(2));
            hijau_x = brectH.x + (brectH.width/2);
            hijau_y = brectH.y + (brectH.height/2);
            cv::rectangle(image_frame, brectH.tl(), brectH.br(), cv::Scalar(100, 100, 200), 2, CV_AA);
            center_2 = Point(hijau_x,hijau_y);
            data_send.titikX_hijau = hijau_x;
            data_send.titikY_hijau = hijau_y;
            koor_hijau = 1;
        } else {
        	koor_hijau = 0;
        }

        if (pointBiru.size() > 0) {
            cv::Rect brectB = cv::boundingRect(cv::Mat(pointBiru).reshape(2));
			biru_x = brectB.x + (brectB.width/2);
            biru_y = brectB.y + (brectB.height/2);
            cv::rectangle(image_frame, brectB.tl(), brectB.br(), cv::Scalar(100, 100, 200), 2, CV_AA);
            center_3 = Point(biru_x,biru_y);
            data_send.titikX_biru = biru_x;
            data_send.titikY_biru = biru_y;
            koor_biru = 1;
        } else {
        	koor_biru = 0;
        }
      	
        if ((koor_merah && koor_biru) == 1){ //Robot (Base dan End Effector)
            line(image_frame, center_3, center_1, color_1,  3, 8);
            line(image_frame,center_1,Point(image_frame.cols,center_1.y),color_3,3,8);
            rr = sqrt(((biru_y - merah_y)*(biru_y - merah_y)) + ((biru_x - merah_x)*(biru_x - merah_x)));
            xr = biru_x - merah_x;
            q1 = acos(xr/rr)*RTD;
            putText(image_frame, "q1 = " + to_string(q1) + " Deg", (center_1 + Point(-100,-50)),CV_FONT_HERSHEY_DUPLEX,1,color_1,1,8);
            data_send.sudut1 = q1;
            posisirobot = 1;
        } else {
            posisirobot = 0;
        }

        if ((koor_merah && koor_hijau) == 1){ //Robot(Base) dan Target
            line(image_frame, center_1, center_2, color_2,  3, 8);
            line(image_frame,center_1,Point(image_frame.cols,center_1.y),color_3,3,8);
            rt = sqrt(((hijau_y - merah_y)*(hijau_y - merah_y)) + ((hijau_x - merah_x)*(hijau_x - merah_x)));
            xt = hijau_x - merah_x;
            q2 = acos(xt/rt)*RTD;
            putText(image_frame, "q2 = " + to_string(q2) + " Deg", (center_1 + Point(-200,-100)),CV_FONT_HERSHEY_DUPLEX,1,color_2,1,8);
            data_send.sudut2 = q2;
            posisistarget = 1;
        } else {
            posisistarget = 0;
        }
        
        if((posisirobot && posisistarget) == 1){
        	delta_theta = q2 - q1;
        	data_send.deltaq = delta_theta;
            data_send.posobjek = objek;
        }

        if(robo_ready != 0){
            send_robot();
        }

        cv::imshow("Gambar Original", image_frame);
        image_fusion = image_ColorM + image_ColorB + image_ColorH;

char c = cvWaitKey(10);
        
        switch (c) {
            case 'h': {
                cv::namedWindow("Control Hijau", CV_WINDOW_AUTOSIZE); //create a window called "Control"
                cvCreateTrackbar("LowH", "Control Hijau", &aktifh->iLowH, 255); //Hue (0 - 255)
                cvCreateTrackbar("HighH", "Control Hijau", &aktifh->iHighH, 255);

                cvCreateTrackbar("LowS", "Control Hijau", &aktifh->iLowS, 255); //Saturation (0 - 255)
                cvCreateTrackbar("HighS", "Control Hijau", &aktifh->iHighS, 255);

                cvCreateTrackbar("LowV", "Control Hijau", &aktifh->iLowV, 255); //Value (0 - 255)
                cvCreateTrackbar("HighV", "Control Hijau", &aktifh->iHighV, 255);
                OpenThr=1;
                
                break;
            }
            case 'm': {
                cv::namedWindow("Control Merah", CV_WINDOW_AUTOSIZE); //create a window called "Control"
                //Create trackbars in "Control Merah" window
                cvCreateTrackbar("LowH", "Control Merah", &aktifm->iLowH, 255); //Hue (0 - 255)
                cvCreateTrackbar("HighH", "Control Merah", &aktifm->iHighH, 255);
                
                cvCreateTrackbar("LowS", "Control Merah", &aktifm->iLowS, 255); //Saturation (0 - 255)
                cvCreateTrackbar("HighS", "Control Merah", &aktifm->iHighS, 255);
                
                cvCreateTrackbar("LowV", "Control Merah", &aktifm->iLowV, 255); //Value (0 - 255)
                cvCreateTrackbar("HighV", "Control Merah", &aktifm->iHighV, 255);
                OpenThr=2;
                
                break;
            }
            case 'b': {
                cv::namedWindow("Control Biru", CV_WINDOW_AUTOSIZE); //create a window called "Control"
                //Create trackbars in "Control Biru" window
                cvCreateTrackbar("LowH", "Control Biru", &aktifb->iLowH, 255); //Hue (0 - 255)
                cvCreateTrackbar("HighH", "Control Biru", &aktifb->iHighH, 255);
                
                cvCreateTrackbar("LowS", "Control Biru", &aktifb->iLowS, 255); //Saturation (0 - 255)
                cvCreateTrackbar("HighS", "Control Biru", &aktifb->iHighS, 255);
                
                cvCreateTrackbar("LowV", "Control Biru", &aktifb->iLowV, 255); //Value (0 - 255)
                cvCreateTrackbar("HighV", "Control Biru", &aktifb->iHighV, 255);
                OpenThr=3;
                
                break;
            }
            case 'f':{
                cv::imshow("Gambar Thresholded", image_fusion);
                break;
            }
            case 'k': {
                data_send.cmd=1;
                printf("Mengaktifkan Servo\n");
                koneksi_robot(iprobo);
                break;
            }
            case 's': {
                data_send.cmd=2;
                printf("Hentikan Robot\n");
                break;
            }
            case '6': {
                printf("Gerakkan Objek ke Kanan\n");
                objek=45;
                if (objek<-90)objek=-90;                             
                break;
            }
            case '4': {
                printf("Gerakkan Objek ke Kiri\n");
                objek=-45;
                if (objek<-90)objek=-90;                  
                break;
            }            
        }

        if (data_send.cmd==1){ 
            printf("Berhasil Terhubung.....\n");     
            cout <<"Point 1" <<center_1 <<" | Point 2"<< center_2 <<" | Point 3"<< center_3
            << " || angle 1 :" << q1 <<" | angle 2 :"<< q2 << " || Delta :" << delta_theta 
            <<'\n';
        }

        if(cv::waitKey(30) >= 27) { 
            data_send.cmd=2;
            robo_ready=0;
            send_robot();
            std::cout << "esc key is pressed by user" << std::endl;
            break;
        }
    }
    // The camera will be deinitialized automatically in VideoCapture destructoratan
    return 0;
}