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

namespace cv
{
    using std::vector;
}

#define PI 3.14159265
#define DTR     PI/180.0                   // Konversi sudut ke radian
#define RTD     180.0/PI                   // Konversi radian ke sudut

#define ROBOT_PORT  45000   // Port Roboard
int OpenThr=0;
int robo_ready = 0;
float q1,q2,r1,r2,x1,x2,theta;

int  sock;
unsigned short PORT_NO;
char iprobo[INET_ADDRSTRLEN]="127.0.0.1";

// Structure
typedef struct  {
    float titikX_merah;
    float titikY_merah;
    float titikX_Biru;
    float titikY_Biru;
    float titikX_hijau;
    float titikY_hijau;
    int cmd;
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

int main(int, char**)
{
    cv::VideoCapture cap(0); // open the default camera
    if(!cap.isOpened()) { // check if we succeeded
        std::cout << "Cannot open the web cam" << std::endl;
        return -1;
    }

    cv::Mat image_HSV;
    cv::Mat image_ColorM; //Matriks Warna Merah
    cv::Mat image_ColorH; //Matriks Warna Hijau
    cv::Mat image_ColorB; //Matriks Warna Biru

    cv::vector<cv::vector<cv::Point> > contours_1;
    cv::vector<cv::vector<cv::Point> > contours_2;
    cv::vector<cv::vector<cv::Point> > contours_3;
    cv::vector<cv::Vec4i> hierarchy_1;
    cv::vector<cv::Vec4i> hierarchy_2;
    cv::vector<cv::Vec4i> hierarchy_3;
    
//                                 ( B,   G,   R )
    cv::Scalar color_1 = cv::Scalar( 0, 255, 255 ); 
    cv::Scalar color_2 = cv::Scalar( 255, 255, 0 ); 
    cv::Scalar color_3 = cv::Scalar( 255, 0, 255 ); 

    cv::Mat image_frame;

    std::string point1;
    std::string point2;
    std::string point3;

    cv::Point center_1;
    cv::Point center_2;
    cv::Point center_3;

    batasTh merah   = { 0,  66,  68, 255, 236, 255};
    batasTh hijau   = {44,  87,  48, 255, 153, 255};
    batasTh Biru    = { 0, 255, 209, 255, 174, 255};
    
    printf("!!!Petunjuk Penggunaan Program Lengan robot berbasis OpenCV dan Roboard!!!\n");
    printf("Press 'm' -> adjust treshold warna merah\n");
    printf("Press 'h' -> adjust treshold warna hijau\n");
    printf("Press 'j' -> adjust treshold warna Biru\n");
    printf("Press 'c' -> menutup adjust treshold warna\n");
    printf("Perhatian!!! jika deteksi warna sudah stabil\n");
    printf("Press 's' button to start the roboard connection\n");
    printf("Press 'S' button to stop the roboard\n");

    batasTh *aktifh = &hijau;
    batasTh *aktifm = &merah;
    batasTh *aktifb = &biru;

    while (true) 
    {
        cap >> image_frame; // get a new frame from camera
        if (image_frame.empty()) {
            std::cout << "Cannot read a frame from video stream" << std::endl;
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
                    image_ColorJ); //Threshold the image

        if(OpenThr==1){
            cv::imshow("Thresholded imgThrHijau", image_ColorH); //show the thresholded image  
        }
        if(OpenThr==2){
            cv::imshow("Thresholded imgThrMerah", image_ColorM); //show the thresholded image    
        }
        if(OpenThr==3){
            cv::imshow("Thresholded imgThrJinga", image_ColorJ); //show the thresholded image 
        }

        cv::erode(image_ColorM,  image_ColorM, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
        cv::dilate(image_ColorM, image_ColorM, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
        cv::dilate(image_ColorM, image_ColorM, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
        cv::erode(image_ColorM,  image_ColorM, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
        
        cv::erode(image_ColorH,  image_ColorH, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
        cv::dilate(image_ColorH, image_ColorH, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
        cv::dilate(image_ColorH, image_ColorH, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
        cv::erode(image_ColorH,  image_ColorH, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
        
        cv::erode(image_ColorJ,  image_ColorJ, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
        cv::dilate(image_ColorJ, image_ColorJ, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
        cv::dilate(image_ColorJ, image_ColorJ, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
        cv::erode(image_ColorJ,  image_ColorJ, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

        // Find contours
        findContours( image_ColorM, // input image
                      contours_1, // vector to save contours
                      hierarchy_1,
                      CV_RETR_LIST,
                      CV_CHAIN_APPROX_NONE,
                      cv::Point(0, 0) );


        findContours( image_ColorH, // input image
                      contours_2, // vector to save contours
                      hierarchy_2,
                      CV_RETR_LIST,
                      CV_CHAIN_APPROX_NONE,
                      cv::Point(0, 0) );

        findContours( image_ColorJ, // input image
                      contours_3, // vector to save contours
                      hierarchy_3,
                      CV_RETR_LIST,
                      CV_CHAIN_APPROX_NONE,
                      cv::Point(0, 0) );

        // Get the moments
        cv::vector<cv::Moments> mu_1(contours_1.size() ); // initialize a vector of moments called mu, vector size the number of contours
        for( int i = 0; i < contours_1.size(); i++ )
        { mu_1[i] = moments( contours_1[i], false ); }

        cv::vector<cv::Moments> mu_2(contours_2.size() ); // initialize a vector of moments called mu, vector size the number of contours
        for( int i = 0; i < contours_2.size(); i++ )
        { mu_2[i] = moments( contours_2[i], false ); }

        cv::vector<cv::Moments> mu_3(contours_3.size() ); // initialize a vector of moments called mu, vector size the number of contours
        for( int i = 0; i < contours_3.size(); i++ )
        { mu_3[i] = moments( contours_3[i], false ); }

        // Get the mass centers
        cv::vector<cv::Point2f> mc_1(contours_1.size()); //vector to store all the center points of the contours.
        for( int i = 0; i < contours_1.size(); i++ )
        { mc_1[i] = cv::Point2f( mu_1[i].m10/mu_1[i].m00 , mu_1[i].m01/mu_1[i].m00 );}

        cv::vector<cv::Point2f> mc_2(contours_2.size()); //vector to store all the center points of the contours.
        for( int i = 0; i < contours_2.size(); i++ )
        { mc_2[i] = cv::Point2f( mu_2[i].m10/mu_2[i].m00 , mu_2[i].m01/mu_2[i].m00 );}

        cv::vector<cv::Point2f> mc_3(contours_3.size()); //vector to store all the center points of the contours.
        for( int i = 0; i < contours_3.size(); i++ )
        { mc_3[i] = cv::Point2f( mu_3[i].m10/mu_3[i].m00 , mu_3[i].m01/mu_3[i].m00 );}

        // Draw contours
        for( int i = 0; i< contours_1.size(); i++ )
        {
            if  (mu_1[i].m00>1000){
                center_1=mc_1[i];

                drawContours( image_frame, contours_1, i, color_1, 2, 8, hierarchy_1, 0, cv::Point() );
                circle( image_frame, mc_1[i], 4, color_1, -1, 8, 0 );

            }
            data_send.titikX_merah = center_1.x;
            data_send.titikY_merah = center_1.y;
        }

        for( int i = 0; i< contours_2.size(); i++ )
        {
            if  (mu_2[i].m00>1000){
                center_2=mc_2[i];
                drawContours( image_frame, contours_2, i, color_2, 2, 8, hierarchy_2, 0, cv::Point() );
                circle( image_frame, center_2, 4, color_2, -1, 8, 0 );

                line(image_frame,center_2,center_1,color_2,4,8,0);
                cv::putText(image_frame, (std::to_string(int(atan ( (abs(center_1.y-center_2.y)*1.0 /(center_1.x-center_2.x)*1.0))*(180.0/PI))) + "Deg"), (center_2 + cv::Point(-100,0)),CV_FONT_HERSHEY_DUPLEX,1,cv::Scalar(255,255,0),1,8);
                line(image_frame,center_2,cv::Point(image_frame.cols,center_2.y),color_2,4,8,0);
            }
            data_send.titikX_hijau = center_2.x;
            data_send.titikY_hijau = center_2.y;
        }

        for( int i = 0; i< contours_3.size(); i++ )
        {
            if  (mu_3[i].m00>1000){
                center_3=mc_3[i];
                drawContours( image_frame, contours_3, i, color_3, 2, 8, hierarchy_3, 0, cv::Point() );
                circle( image_frame, center_3, 4, color_3, -1, 8, 0 );

                line(image_frame,center_3,center_2,color_3,4,8,0);
                cv::putText(image_frame, (std::to_string(int(atan ( (abs(center_2.y-center_3.y)*1.0 /(center_2.x-center_3.x)*1.0))*(180.0/PI))) + "Deg"), (center_3 + cv::Point(-100,0)),CV_FONT_HERSHEY_DUPLEX,1,cv::Scalar(0,255,255),1,8);
                line(image_frame,center_3,cv::Point(image_frame.cols,center_3.y),color_3,4,8,0);
            }
            data_send.titikX_Biru = center_3.x;
            data_send.titikY_Biru = center_3.y;
        }

        if(robo_ready != 0){
            send_robot();
        }

        cv::imshow("Gambar Original", image_frame);

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
            case 'c': {
                cvDestroyWindow("Control Hijau");
                cvDestroyWindow("Control Merah");
                cvDestroyWindow("Control Biru");
                cvDestroyWindow("Thresholded imgThrHijau");
                cvDestroyWindow("Thresholded imgThrMerah");
                cvDestroyWindow("Thresholded imgThrJinga");
                OpenThr=0;
                break;
            }
            case 's': {
                data_send.cmd=1;
                koneksi_robot(iprobo);
                break;
            }
            case 'S': {
                data_send.cmd=2;
                printf("Command : 2 = stop roboard \n");
                break;
            }
        }

        if (data_send.cmd==1){        
            std::cout <<"Point 1" <<center_1 <<" | Point 2"<< center_2 <<" | Point 3"<< center_3
            << " || angle 1 :" <<atan ( (abs(center_1.y-center_2.y)*1.0 /(center_1.x-center_2.x)*1.0))*(180.0/PI)<<" | angle 2 :"<<atan ( (abs(center_2.y-center_3.y)*1.0 /(center_2.x-center_3.x)*1.0))*(180.0/PI)
            <<'\n';
        }

        if(cv::waitKey(30) >= 0) { 
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