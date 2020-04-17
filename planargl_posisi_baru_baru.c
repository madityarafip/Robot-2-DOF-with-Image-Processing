/////////////////////////////////////////////////////////////
/* Template OpengGL sengaja dibuat untuk kuliah robotik 
*  di Departemen Teknik Elektro
*  Bagi yang ingin memodifikasi untuk keperluan yang lain,
*  dipersilahkan dengan menuliskan acknowledgement pada
*    Dr. Abdul Muis, MEng.
*    Autonomous Control Electronics (ACONICS) Research Group
*    http://www.ee.ui.ac.id/aconics
*////////////////////////////////////////////////////////////

#include <stdio.h> 
#include <stdlib.h> 
#include <GL/glut.h> // Header File For The GLUT Library
#include <GL/gl.h> // Header File For The OpenGL32 Library
#include <GL/glu.h> // Header File For The GLu32 Library
#include <unistd.h> // Header file for sleeping.
#include <math.h> 
#include <fcntl.h>			/* File control definitions */
#include <errno.h>			/* Error number definitions */
#include <termios.h>		/* POSIX terminal control definitions */
#include <sys/time.h>
#include "planar.c"

FILE *filesim;

/* ascii code for the escape key */
#define ESCkey	27

/* The number/handle of our GLUT window */
int window, wcam;  
 


/* To draw a quadric model */
GLUquadricObj *obj;

// ROBOT MODEL PARAMATER
#define Xoffset	0.0	
#define Yoffset	0.0
#define Zoffset	0.2
#define img_height 400
#define img_width 600
#define threshold 230

#define Link1 L1
#define Link2 L2


//deklarasi variabel
//step 1
float *tetha1=&q1;
float *tetha2=&q2;

float *xbase=&objx;
float *ybase=&objy;

float x_awal = 0, x_final = 0, y_awal = 0, y_final = 0;
float x = 0, dx = 0, ddx = 0, y = 0, dy = 0, ddy = 0;
float x_error = 0, x_error_old = 0, dx_error = 0, y_error = 0, y_error_old = 0, dy_error = 0;
float dq1 = 0, ddq1 = 0, dq2 = 0,ddq2 = 0;
float xd = 0, yd = 0;

float Kp = 2, Kd = 3;

float dt = 0.02;
float t = 0;

char debug=0;

int gerak=0;
int counter=0;

//deklarasi variabel Jinv
float J11,J12,J21,J22,I_J11,I_J12,I_J21,I_J22;

static unsigned int h = 0;

void Sim_main(void); // Deklarasi lebih awal agar bisa diakses oleh fungsi sebelumnya
void display(void); // fungsi untuk menampilkan gambar robot / tampilan camera awal
void camera_result(void); // fungsi untuk menampilkan hasil olah camera

// Hasil gambar titik awal di pojok kiri bawah
/*  | gambar[img_height][0]
*   |
*   |
*   |
*   |
*   |
*   |
*   |
*   |
*   x________________________ gambar[0][img_width]
*
*///////////////////////////////////////////////////

unsigned char gambarGray[img_height+2][img_width+2]; // pixel untuk menampung gambar grayscale
unsigned char gambarR[img_height+2][img_width+2]; // pixel untuk menampung gambar merah
unsigned char gambarG[img_height+2][img_width+2]; // pixel untuk menampung gambar hijau
unsigned char gambarB[img_height+2][img_width+2]; // pixel untuk menampung gambar biru


  
/* define color */  
GLfloat green1[4]  ={0.8, 1.0, 0.8, 1.0};
GLfloat blue1[4]  ={0.1, 0.1, 1.0, 1.0};
GLfloat blue2[4]  ={0.2, 0.2, 1.0, 1.0};
GLfloat blue3[4]  ={0.3, 0.3, 1.0, 1.0};
GLfloat yellow1[4]={0.1, 0.1, 0.0, 1.0};
GLfloat yellow2[4]={0.2, 0.2, 0.0, 1.0};
GLfloat pink6[4] ={0.8, 0.55, 0.6, 1.0};
GLfloat yellow5[4]={0.8, 0.8, 0.0, 1.0};
GLfloat abu2[4]={0.5,0.5,0.5,1.0};


void  drawOneLine(double x1, double y1, double x2, double y2) 
   {glBegin(GL_LINES); glVertex3f((x1),(y1),0.0); glVertex3f((x2),(y2),0.0); glEnd();}
   
void  model_cylinder(GLUquadricObj * object, GLdouble lowerRadius,
  GLdouble upperRadius, GLdouble length, GLint res, GLfloat *color1, GLfloat *color2)
{
  glPushMatrix();
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color1);
    glTranslatef(0,0,-length/2);
	  gluCylinder(object, lowerRadius, upperRadius, length, 20, res);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color2);
    gluDisk(object, 0.01, lowerRadius, 20, res); 
    glTranslatef(0, 0, length);
    gluDisk(object, 0.01, upperRadius, 20, res); 
  glPopMatrix();
}

void disp_floor(void)
{
  int i,j,flagc=1;

  glPushMatrix();
  
  GLfloat dxf=4.5,dyf=4.5;
  GLint amount=15;
  GLfloat x_min=-dxf/2.0, x_max=dxf/2.0, x_sp=(GLfloat) dxf/amount, y_min=-dyf/2.0, y_max=dyf/2.0, y_sp=(GLfloat) dyf/amount;

  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, green1);
  for(i = 0; i<=48; i++){
     drawOneLine(-2.4+0.1*i, -2.4,       -2.4+0.1*i,  2.4);
     drawOneLine(-2.4,       -2.4+0.1*i,  2.4,       -2.4+0.1*i);
  }

  glPopMatrix();
}

void  lighting(void)
{

	GLfloat light_ambient[] =  {0.2, 0.2, 0.2, 1.0};
	GLfloat light_diffuse[] =  {0.4, 0.4, 0.4, 1.0};
	GLfloat light_specular[] = {0.3, 0.3, 0.3, 1.0};
	GLfloat light_position[] = {2, 0.1, 7,1.0};
	GLfloat spot_direction[] = {0.0, -0.1, -1.0, 1.0};

	glClearColor(0.0, 0.0, 0.0, 0.0);     
  
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glLightf(GL_LIGHT0, GL_SPOT_CUTOFF, 40.0);
	glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, spot_direction);
	glLightf(GL_LIGHT0, GL_SPOT_EXPONENT, 4);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST);
}

void disp_robot(void)
{
  glPushMatrix();
    glTranslatef(Xoffset, Yoffset, Zoffset/2);
    // Draw base
    model_cylinder(obj, 0.1, 0.1, Zoffset, 2, abu2, abu2);
    // Menuju joint-1
    glTranslatef(0, 0, Zoffset/2);
    glRotatef(*tetha1*RTD,0,0,1);
    glPushMatrix();
      // Gambar link1
      glRotatef(-90,1,0,0);
      glTranslatef(0,0,Link1/2);
      model_cylinder(obj, 0.03, 0.03, Link1, 2, pink6, yellow2);
    glPopMatrix();
    // Menuju joint-2
    glTranslatef(0,Link1, 0);
    glRotatef(*tetha2*RTD,0,0,1);
    glPushMatrix();
      glDisable(GL_LIGHTING);
      glColor3f(0.0,1.0,0.0);  // buat tanda hijau (R G B) = 0 1 0             
      glTranslatef(0,0,0.04); // pindahkan offset 1cm diatas sendi 2
      gluDisk(obj, 0.001, 0.01, 20, 2); // bikin tanda lingkaran merah diatas sendi 2 
      glEnable(GL_LIGHTING);
    glPopMatrix();
    glPushMatrix();
      // Gambar link2
      glRotatef(-90,1,0,0);
      glTranslatef(0,0,Link2/2);
      model_cylinder(obj, 0.03, 0.03, Link2, 2, yellow5, yellow2);
    glPopMatrix();
    glPushMatrix();
      glDisable(GL_LIGHTING);
      glColor3f(0.0,0.0,1.0);  // buat tanda biru (R G B) = 0 0 1             
      glTranslatef(0,Link2,0.04); // pindahkan offset 1cm diatas sendi 2
      gluDisk(obj, 0.001, 0.01, 20, 2); // bikin tanda lingkaran merah diatas sendi 2 
      glEnable(GL_LIGHTING);
    glPopMatrix();
  glPopMatrix();
  glPushMatrix();
    glTranslatef(*xbase, *ybase, Zoffset/2);
    // Draw obj
    model_cylinder(obj, 0.03, 0.03, Zoffset, 2, yellow5, abu2);
    
    glDisable(GL_LIGHTING);
    glColor3f(1.0,0.0,0.0); // buat tanda merah (R G B) = 1 0 0 
    glTranslatef(0,0,Zoffset/2+0.01); // pindahkan offset 1cm diatas object
    gluDisk(obj, 0.001, 0.01, 20, 2); // bikin tanda lingkaran merah diatas object 
    glEnable(GL_LIGHTING);

  glPopMatrix();

}

// Draw Object
void display(void)
{
//   glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT) ; // Clear The Screen and The Depth Buffer 
   glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT) ; // Clear The Screen and The Depth Buffer 
   //glLoadIdentity();  // Reset View
   disp_floor();
   hitung_robot();
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

//step 2
void forward_kinematics() {
	x = L1*cos(q1) + L2*cos(q1+q2);
	y = L1*sin(q1) + L2*sin(q1+q2);
}

void inverse_jacobian() {

  float detJ = J11*J22 - J12*J21;
  if (abs(detJ) < 0.001){

}
else {
//Matriks JACOBIAN J = [J11 J12;J21 J22]
  J11 = -(L1*sin(q1) + L2*sin(q1+q2));
  J12 = -L2*sin(q1+q2);
  //J21 = L1*sin(q1) + L2*sin(q1+q2);
  J21 = L1*cos(q1) + L2*cos(q1+q2);
  J22 = L2*cos(q1+q2);

//Step 6
  ddq1=(1/detJ)*((J22*ddx)-(J12*ddy));
  ddq2=(1/detJ)*((-J21*ddx)+(J11*ddy));
  }
}

void controller_robot () {
//Step 4
  x_error= xd - x;
  y_error= yd - y;  

  dx_error = (x_error-x_error_old)/dt;
  dy_error = (y_error-y_error_old)/dt;
    
  x_error_old = x_error;
  y_error_old = y_error;

//Step 5 
  ddx = (Kp*x_error + Kd*dx_error);
  ddy = (Kp*y_error + Kd*dy_error);
}

void integrator_sudut () {
//Step 7
  dq1 = dq1 + ddq1*dt;
  dq2 = dq2 + ddq2*dt;

//Step 8
  q1 = q1 + dq1*dt;
  q2 = q2 + dq2*dt;
}

void kontrol_posisi_robot () {
//Step 3
  if ((x != x_final) || (y != y_final)){
    
    xd = x_awal + (x_final - x_awal)*h/(50*3);
    yd = y_awal + (y_final- y_awal)*h/(50*3);
    h++;
    
    if (h > 50*3){
      x_awal = x_final;
      y_awal = y_final;
      h = 0;
    }
  controller_robot();

  inverse_jacobian();

  integrator_sudut();
  }
}

void kom_ser() {
  //untuk kirim ke serial
  unsigned char sudut1, sudut2, header;
  header = 0xF5;
  //scalling sudut 0-180 derajat
  sudut1=*tetha1*RTD+90;
  sudut2=*tetha2*RTD+90;
  
  //saturasi sudut1
  if (sudut1 < 0)
  {
  sudut1=0; 
  }
  else if (sudut1 > 180)
  {
    sudut1=180;
  }

  //saturasi sudut2
  if (sudut2 < 0)
  {
  sudut2=0; 
  }
  else if (sudut2 > 180)
  {
    sudut2=180;
  }

  //kirim ke arduino
  write(fd,&header, sizeof(header));
  write(fd,&sudut1, sizeof(sudut1));
  write(fd,&sudut2, sizeof(sudut2));
}

void gerak_otomatis() {
  if(gerak==1) //bagian gerak otomatis
  {
      counter++;
    fncgerakotom(counter);
  }
  
}

void Sim_main(void)
{
	unsigned long Xr=0,Yr=0, Xg=0,Yg=0, Xb=0,Yb=0; // titik untuk menghitung sum
	int Nr=0, Ng=0, Nb=0;
	static unsigned int Rx,Ry, Gx,Gy, Bx,By; // untuk menyimpan hasil titik berat
	unsigned int i,j,k;
    glutSetWindow(window);
	
	forward_kinematics();
	
  kontrol_posisi_robot();
  
  fprintf(filesim,"%.4f, %d, %.4f, %.4f, %.4f, %.4f, \n",t,h,xd,x,yd,y);
  printf("== %u == \n",(unsigned)h);
  t+=dt;

  gerak_otomatis();

	kom_ser();
	
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
  
  printf("=%d %d %d %d %d %d\n",Rx, Ry, Gx, Gy, Bx, By);
  
  glutSetWindow(wcam);
  camera_result(); 	
	
}


void keyboard(unsigned char key, int i, int j)
{
	 switch(key){
	  case ESCkey: fclose(filesim); exit(1); break;
    //Posisi objek
	  case '6': *xbase=*xbase+0.01; break;
    case '4': *xbase=*xbase-0.01; break;
    case '8': *ybase=*ybase+0.01; break;
    case '2': *ybase=*ybase-0.01; break;
	  //Kontrol berdasarkan sudut
	  case 'a': *tetha1+=10*DTR; break;
	  case 's': *tetha1-=10*DTR; break;
	  case 'x': *tetha2+=10*DTR; break;
	  case 'z': *tetha2-=10*DTR; break;
	  case 'l': gerak = 1; break;
	  case 'L': gerak = 0; break;
	  //Kontrol berdasarkan posisi
  	case 'g': x_awal = x; y_awal = y; x_final = 1; y_final = 2; break;
    case 'h': x_awal = x; y_awal = y; x_final = x + 0.1; y_final = y + 0.1; break; 
    case 'j': x_awal = x; y_awal = y; x_final = x - 0.1; y_final = y - 0.1; break;
  //  case 'c': x_final = *xbase; y_final = *ybase; break;       
	 //Reset robot
	  case 'm': *tetha1 = 0*DTR; *tetha2 = 15*DTR; xd = 0; yd = 0; x = 0; y = 0; break;

	  case 'd': debug=(~debug) & 0x1; break;
    case 'q': glutIdleFunc(NULL); break;
    case 'r': glutIdleFunc(&Sim_main); break;
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

// Main Program
int main(int argc, char** argv)
{
 // Initialize GLUT
   /* Initialize GLUT state - glut will take any command line arguments 
      see summary on OpenGL Summary */  
   glutInit (&argc, argv);
   fd = open_port();
   init_port(fd);
   filesim = fopen("DataSim.csv","w");
   filesim = fopen("DataSim.dat","w");
   
   /* Select type of Display mode:   
      Double buffer 
      RGBA color
      Alpha components supported 
      Depth buffer */  
   //glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
   glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB );
   /* set a 400 (width) x 400 (height) window and its position */
   glutInitWindowSize(img_width,img_height);	
   glutInitWindowPosition (40, 100);

   /* Open a window */  
   window = glutCreateWindow ("Simple Window");

   /* Initialize our window. */
   init() ;
   camera_window(); 
   init_robot();

   /* Register the function to do all our OpenGL drawing. */
   glutIdleFunc(&Sim_main); // fungsi untuk simulasi utama

   /* Start Event Processing Engine */ 
   glutMainLoop () ;
   return 0 ;
}           
