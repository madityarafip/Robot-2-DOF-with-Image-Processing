//Program Final Proyek Akhir Robotika Cerdas\\
//Robot Lengan dengan Image Processing\\
//Master Robot (PC)\\


#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/fast_math.hpp"
#include <opencv2/core/core.hpp>
#include "highgui.h"
#include <math.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <unistd.h>

#include <GL/glut.h> // Header File For The GLUT Library
#include <GL/gl.h> // Header File For The OpenGL32 Library
#include <GL/glu.h> // Header File For The GLu32 Library

#include "gambargl.c"

unsigned char gambarGray[img_height+2][img_width+2]; // pixel untuk menampung gambar grayscale
unsigned char gambarR[img_height+2][img_width+2]; // pixel untuk menampung gambar merah
unsigned char gambarG[img_height+2][img_width+2]; // pixel untuk menampung gambar hijau
unsigned char gambarB[img_height+2][img_width+2]; // pixel untuk menampung gambar biru

void Sim_main(void); // Deklarasi lebih awal agar bisa diakses oleh fungsi sebelumnya
void display(void); // fungsi untuk menampilkan gambar robot / tampilan camera awal
void camera_result(void); // fungsi untuk menampilkan hasil olah camera 

using namespace cv;
using namespace std;

#define PI 3.14159265
#define DTR     PI/180.0                   // Konversi sudut ke radian
#define RTD     180.0/PI                   // Konversi radian ke sudut

#define ROBOT_PORT  45000   // Port Roboard
#define ESCkey  27

int OpenThr=0;
int robo_ready = 0;
int objek = 0;
int perubahan_sudut = 0;
int merah_x, merah_y, hijau_x, hijau_y, biru_x, biru_y, koor_merah, koor_hijau, koor_biru, posisirobot, posisistarget;
float sudut1, sudut2, rr, rt, xr, xt, delta_theta;

int  sock;
unsigned short PORT_NO;
char iprobo[INET_ADDRSTRLEN]="192.168.0.20";

// Structure Data yang ingin dikirim
typedef struct  {
    float titikX_merah;
    float titikY_merah;
    float titikX_biru;
    float titikY_biru;
    float titikX_hijau;
    float titikY_hijau;
    int cmd;
    float sudut1; //Sudut Robot
    float sudut2; //Sudut Objek
    float deltaq;
    int posobjek; //Perubahan posisi objek
} pc_t; //Data yang dikirim

pc_t data_send;

// Structure Data Threshold
typedef struct {
    int iLowH;
    int iHighH;
    int iLowS;
    int iHighS;
    int iLowV;
    int iHighV;
} batasTh;

VideoCapture cap(0); //capture the video from web cam//1webcam
	
std::vector<cv::Point> CariRectangle(Mat imgThresholded) {
std::vector< std::vector<cv::Point> > contours;
std::vector<cv::Point> points;
cv::findContours(imgThresholded, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
	for (size_t i=0; i<contours.size(); i++) {
		for (size_t j = 0; j < contours[i].size(); j++) {
        	cv::Point p = contours[i][j];
        	points.push_back(p); // -> pushback nambah point contours di vektor paling akhir
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
    robo_ready = 1;
}

int send_robot(void)
{

    if(send(sock, &data_send, sizeof(data_send), 0) < 0) {
        close(sock); //Closes the file currently associated with the object, disassociating it from the stream.
        printf("Data Sending Failed, Please Check the Connection!\n");
        robo_ready =0;

    }
    else
    {
        printf("Sending Data to Roboard\n");
    }
}

void Sim_main(void)
{
	unsigned long Xr=0,Yr=0, Xg=0,Yg=0, Xb=0,Yb=0; // titik untuk menghitung sum
	int Nr=0, Ng=0, Nb=0;
	static unsigned int Rx,Ry, Gx,Gy, Bx,By; // untuk menyimpan hasil titik berat
	unsigned int i,j,k;
  glutSetWindow(window);
  display();

  // Cari titik berat Titik titik feature
  for (i=0;i<img_width;i++)
   for (j=0;j<img_height;j++) {
      // Tes Treshold
      if (gambarR[j][i]>threshold) {Xr+=i;Yr+=j;Nr++;
          if (debug) printf("%d,%d ",i,j);}
      if (gambarG[j][i]>threshold) {Xg+=i;Yg+=j;Ng++;}
      if (gambarB[j][i]>threshold) {Xb+=i;Yb+=j;Nb++;}
   }
  if (debug) {
    printf("\n");
    for (k=0;k<img_width*img_height;k++)
      if (*(&gambarR[0][0]+k)>threshold) printf("%d ",k);
    printf("\n");
    
  }
  // Hitung titik berat 
  if (Nr) {Rx=Xr/Nr;  Ry=Yr/Nr;}
  if (Ng) {Gx=Xg/Ng;  Gy=Yg/Ng;}
  if (Nb) {Bx=Xb/Nb;  By=Yb/Nb;}
  //Display hasil extract ke gambarGray untuk ditampilkan
  gambarGray[Ry][Rx]=255;
  gambarGray[Gy][Gx]=255;
  gambarGray[By][Bx]=255;


    Mat image_frame, image_HSV, image_ColorM, image_ColorH, image_ColorB, image_fusion; //Buat Matriks 
    Point center_1, center_2, center_3; //Buat titik

//                         ( B,   G,   R   )
    Scalar color_1 = Scalar(   0, 255, 255 ); //Yellow
    Scalar color_2 = Scalar( 128, 128,   0 ); //Teal
    Scalar color_3 = Scalar( 204,   0, 102 ); //Violet
    Scalar color_4 = Scalar(   0,   0, 128 ); //Maroon
    Scalar color_5 = Scalar(   0, 100,   0 ); //Darkgreen
    Scalar color_6 = Scalar( 128,   0,   0 ); //Navy
    Scalar color_7 = Scalar(   0,   0,   0 ); //Black

    //Lampu Nyala
    //batasTh merah   = {  0,  67, 168, 255, 192, 255}; //merah
    batasTh merah   = {147, 195, 160, 255, 170, 255}; //pink
    batasTh hijau   = { 48,  93, 136, 255,  85, 255};
    batasTh biru    = { 87, 122, 155, 255, 132, 255};
    //batasTh biru    = { 89, 152, 106, 255, 152, 255}; // ada gelas    

    batasTh *aktifh = &hijau;
    batasTh *aktifm = &merah;
    batasTh *aktifb = &biru;

    bool bSuccess = cap.read(image_frame); // read a new frame from video
	if (!bSuccess) {
        cout << "Cannot read a frame from video stream" << endl;
        //break;
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
        //morphological opening (remove small objects from the foreground)
        cv::erode(image_ColorM,  image_ColorM, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
        cv::dilate(image_ColorM, image_ColorM, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
        //morphological closing (fill small holes in the foreground)
        cv::dilate(image_ColorM, image_ColorM, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
        cv::erode(image_ColorM,  image_ColorM, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
        
        //morphological opening (remove small objects from the foreground)
        cv::erode(image_ColorH,  image_ColorH, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
        cv::dilate(image_ColorH, image_ColorH, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
        //morphological closing (fill small holes in the foreground)
        cv::dilate(image_ColorH, image_ColorH, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
        cv::erode(image_ColorH,  image_ColorH, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
        
        //morphological opening (remove small objects from the foreground)
        cv::erode(image_ColorB,  image_ColorB, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
        cv::dilate(image_ColorB, image_ColorB, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
        //morphological closing (fill small holes in the foreground)
        cv::dilate(image_ColorB, image_ColorB, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
        cv::erode(image_ColorB,  image_ColorB, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

//Cari Kontur dari warna yang terbaca
        std::vector<cv::Point> pointMerah, pointHijau, pointBiru;
        pointMerah = CariRectangle(image_ColorM);
        pointHijau = CariRectangle(image_ColorH);
        pointBiru  = CariRectangle(image_ColorB);

//Membaca Warna dan mencari titik berat
        if (pointMerah.size() > 0) {
            cv::Rect brectM = cv::boundingRect(cv::Mat(pointMerah).reshape(2));
            merah_x = brectM.x + (brectM.width/2);
            merah_y = brectM.y + (brectM.height/2);
            cv::rectangle(image_frame, brectM.tl(), brectM.br(), color_4, 2, CV_AA);
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
            cv::rectangle(image_frame, brectH.tl(), brectH.br(), color_5, 2, CV_AA);
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
            cv::rectangle(image_frame, brectB.tl(), brectB.br(), color_6, 2, CV_AA);
            center_3 = Point(biru_x,biru_y);
            data_send.titikX_biru = biru_x;
            data_send.titikY_biru = biru_y;
            koor_biru = 1;
        } else {
        	koor_biru = 0;
        }

//-----------center_1 = merah; center_2 = hijau; center_3 = biru---------------\\ 
        //Hitung sudut dan buat garis antara center   
        if ((koor_merah && koor_biru) == 1){ //Robot (Base dan End Effector)
            line(image_frame, center_3, center_1, color_1,  3, 8);
            line(image_frame,center_1,Point(image_frame.cols,center_1.y),color_3,3,8);
            circle(image_frame,center_1,5,color_4,-1,8,0);
            circle(image_frame,center_2,5,color_5,-1,8,0);
         
            if (biru_x > merah_x){
                sudut1 = atan(((biru_y - merah_y)*1.0/(merah_x - biru_x)*1.0))*RTD;
            } else{
                sudut1 = 180-atan(((biru_y - merah_y)*1.0/(biru_x - merah_x)*1.0))*RTD;
            }
/*          rr = sqrt(((biru_y - merah_y)*(biru_y - merah_y)) + ((biru_x - merah_x)*(biru_x - merah_x)));
            xr = biru_x - merah_x;            
            sudut1 = acos(xr/rr)*RTD;*/
            putText(image_frame, "sudut1 = " + to_string(sudut1) + " Deg", (center_1 + Point(-100,-50)),CV_FONT_HERSHEY_DUPLEX,1,color_1,1,8);
            data_send.sudut1 = sudut1;          
            posisirobot = 1;
        } else {
            posisirobot = 0;
        }

        if ((koor_merah && koor_hijau) == 1){ //Robot(Base) dan Target
            line(image_frame, center_1, center_2, color_2,  3, 8);
            line(image_frame,center_1,Point(image_frame.cols,center_1.y),color_3,3,8);
            circle(image_frame,center_3,5,color_6,-1,8,0);

            if (hijau_x > merah_x){
                sudut2 = atan(((hijau_y - merah_y)*1.0/(merah_x - hijau_x)*1.0))*RTD;
            } else {
                sudut2 = 180-atan(((hijau_y - merah_y)*1.0/(hijau_x - merah_x)*1.0))*RTD;
            }
/*          rt = sqrt(((hijau_y - merah_y)*(hijau_y - merah_y)) + ((hijau_x - merah_x)*(hijau_x - merah_x)));
            xt = hijau_x - merah_x;            
            sudut2 = acos(xt/rt)*RTD;*/
            putText(image_frame, "sudut2 = " + to_string(sudut2) + " Deg", (center_1 + Point(-200,-100)),CV_FONT_HERSHEY_DUPLEX,1,color_2,1,8);
            data_send.sudut2 = sudut2;
            posisistarget = 1;
        } else {
            posisistarget = 0;
        }

        //Hitung Perbedaaan sudut
        if((posisirobot && posisistarget) == 1){
        	delta_theta = sudut2 - sudut1;
        	data_send.deltaq = delta_theta;
            putText(image_frame,"delta_q = " + to_string(delta_theta) + " Deg", (Point(500,50)),CV_FONT_HERSHEY_DUPLEX,1,color_7,1,8);
        }

        if(robo_ready != 0){
            send_robot();
        }

        //update Visualisasi OpenGL
        *tetha1=sudut1*DTR;
        if (abs(delta_theta)>5){ //Animasi pukul
            *tetha2=0*DTR;
        } else {
            //Visualisasi pukul objek
			*tetha2=-90*DTR;                         
        }
        *x=(hijau_x-merah_x)/1100.0; // 1100 -> pixel camera/pixel open gl
        *y=(-hijau_y+merah_y)/1100.0;

        //Munculkan gambar hasil pembacaan kamera
        cv::imshow("Gambar Original", image_frame);
        image_fusion = image_ColorM + image_ColorB + image_ColorH;

//Keyboard
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
                break;
            }
            case 's': {
                data_send.cmd=2;
                printf("Hentikan Robot\n");
                usleep(5000000);                
                exit(1);
                break;
            }
            case '6': {
                printf("Gerakkan Objek ke Kanan\n");
                objek=objek+10;
                if (objek>70)objek=70;   
                data_send.posobjek = objek;                                          
                break;
            }
            case '4': {              
                printf("Gerakkan Objek ke Kiri\n");
                objek=objek-15;
                if (objek<-70)objek=-70;      
                data_send.posobjek = objek;                            
                break;
            }            
        }
        perubahan_sudut = -objek;
        if (data_send.cmd==1){ 
        	koneksi_robot(iprobo);
            printf("Berhasil Terhubung.....\n");     
            cout <<"M" <<center_1 <<" | H"<< center_2 <<" | B"<< center_3
            << " || q1 :" << sudut1 <<" | q2 :"<< sudut2 << " || delta_q :" << delta_theta 
            <<" || Perubahan Posisi Objek: "<<perubahan_sudut<<'\n';       
        }

        if(cv::waitKey(30) == ESCkey) { 
            data_send.cmd=2;
            robo_ready=0;
            send_robot();
            std::cout << "esc key is pressed by user" << std::endl;
            exit(1);
            //break;
        }
    
}

void keyboard(unsigned char key, int i, int j)
{
	 switch(key){
      case ESCkey: exit(1); break;
      case '1': *x=*x+0.01; break;
      case '2': *x=*x-0.01; break;
      case '5': *y=*y+0.01; break;
      case '6': *y=*y-0.01; break;
      case 'd': debug=(~debug) & 0x1; break;
      case 's': glutIdleFunc(NULL); break;
      case 'r': glutIdleFunc(&Sim_main); break;
      case 'a': *tetha1 += 10*DTR;break;
      case 'A': *tetha1 -= 10*DTR;break;
      case 'z': *tetha2 += 10*DTR;break;
      case 'Z': *tetha2 -= 10*DTR;break;
   }
}

void camera_result(void)
{
   glClear(GL_COLOR_BUFFER_BIT);
   if (debug) 
   	 glDrawPixels(img_width, img_height, GL_LUMINANCE, GL_UNSIGNED_BYTE, gambarR);
   else
   	 glDrawPixels(img_width, img_height, GL_LUMINANCE, GL_UNSIGNED_BYTE, gambarGray);

   glutSwapBuffers();  
}

void camera_window(void)
{
	/*----------Camera Window----------*/
    //glutInitDisplayMode(GLUT_DOUBLE |  GLUT_DEPTH);

	 glutInitWindowSize(img_width,img_height);		
   glutInitWindowPosition (500, 100);
	 wcam=glutCreateWindow("Camera Process");
   glClearColor(0.0f, 0.0f, 1.0f, 1.0f); 
   glutDisplayFunc (&camera_result) ;
   glutKeyboardFunc(&keyboard);
}


void init(void) 
{ 
   obj = gluNewQuadric(); 
   /* Clear background to (Red, Green, Blue, Alpha) */
   glClearColor(0.0f, 0.0f, 0.0f, 0.0f) ;
   glEnable(GL_DEPTH_TEST); // Enables Depth Testing
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   gluPerspective(20.0, 1, 0.5, 10);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
   gluLookAt(0, 0.3, 3.5,  0.0, 0.4, 0.0,  0.0, 1.0, 0); 
	 lighting();
	 
   /* When the shading model is GL_FLAT only one colour per polygon is used, 
      whereas when the shading model is set to GL_SMOOTH the colour of 
      a polygon is interpolated among the colours of its vertices.  */
   glShadeModel(GL_SMOOTH) ; 
   glutDisplayFunc (&display) ;
   glutKeyboardFunc(&keyboard);

}

// Draw Object
void display(void)
{
//   glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT) ; // Clear The Screen and The Depth Buffer 
   glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT) ; // Clear The Screen and The Depth Buffer 
   disp_floor();
   disp_robot();
   /* since window is double buffered, 
      Swap the front and back buffers (used in double buffering). */
   glutSwapBuffers() ; 
   
   // Transfer warna jadi grayscale, dan di buat masing2 0.15 scalanya (seharusnya 0.3333333)
   glPixelTransferf(GL_RED_SCALE,0.15);
	 glPixelTransferf(GL_GREEN_SCALE,0.15);
	 glPixelTransferf(GL_BLUE_SCALE,0.15); 
	 glReadPixels(0,0, img_width,img_height, GL_LUMINANCE,GL_UNSIGNED_BYTE, gambarGray);
	 
	 // ambil warna merah saja
	 glPixelTransferf(GL_RED_SCALE,1);
	 glPixelTransferf(GL_GREEN_SCALE,0);
	 glPixelTransferf(GL_BLUE_SCALE,0); 
	 glReadPixels(1,1,img_width,img_height, GL_LUMINANCE,GL_UNSIGNED_BYTE, gambarR);
	 // ambil warna hijau saja
	 glPixelTransferf(GL_RED_SCALE,0);
	 glPixelTransferf(GL_GREEN_SCALE,1);
	 glPixelTransferf(GL_BLUE_SCALE,0); 
	 glReadPixels(0,0, img_width,img_height, GL_LUMINANCE,GL_UNSIGNED_BYTE, gambarG);
	 // ambil warna biru saja
	 glPixelTransferf(GL_RED_SCALE,0);
	 glPixelTransferf(GL_GREEN_SCALE,0);
	 glPixelTransferf(GL_BLUE_SCALE,1); 
	 glReadPixels(0,0, img_width,img_height, GL_LUMINANCE,GL_UNSIGNED_BYTE, gambarB);

}
	
int main( int argc, char** argv )
{
   glutInit(&argc, argv);                 // Initialize GLUT
      glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB );
      glutInitWindowSize(img_width,img_height);	
   glutInitWindowPosition (500, 400);
    window = glutCreateWindow ("Robot Visualization");
       init() ;	

    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the web cam" << endl;
         return -1;
    }

    printf("!!!-----------------------Main Menu-------------------------------!!!\n");
    printf("Tekan ESC untuk menutup semua window\n");
    printf("Tekan 'm' -> Adjust treshold warna merah\n");
    printf("Tekan 'h' -> Adjust treshold warna hijau\n");
    printf("Tekan 'j' -> Adjust treshold warna Biru\n");
    printf("Tekan 'f' -> Munculkan Threshold Gabungan\n");
    printf("=====================================================================\n");
    printf("Tekan 'k' -> Terhubung dengan Roboard dan Robot\n");
    printf("Tekan '6' -> Menggerakkan Objek ke Kanan\n");
    printf("Tekan '4' -> Menggerakkan Objek ke Kiri\n");
    printf("Tekan 's' -> Berhenti Mengirim Data\n");

    Sim_main();
   		
   		glutIdleFunc(&Sim_main); // Register display callback handler for window re-paint 
   		glutMainLoop(); 
   

    // The camera will be deinitialized automatically in VideoCapture destructoratan
    return 0;
}
