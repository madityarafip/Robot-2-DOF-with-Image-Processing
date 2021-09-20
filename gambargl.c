// ROBOT MODEL PARAMATER
#define Xoffset	0.0	
#define Yoffset	0.0
#define Zoffset	0.2
#define img_height 480
#define img_width 640
#define threshold 230
#define PI 3.14159265
#define RTD 180.0/PI

#define L1	0.3   // link1
#define L2	0.2   // link2
#define Link1 L1
#define Link2 L2

#define ESCkey	27
float q1;
float q2;
float objx=0.2;
float objy=0.6;
/* To draw a quadric model */
GLUquadricObj *obj;
float *tetha1=&q1;
float *tetha2=&q2;
float *x=&objx;
float *y=&objy;
char debug=0;
int window, wcam; 

  
/* define color */  
GLfloat green1[4]  = {0.8, 1.0 , 0.8, 1.0};
GLfloat green2[4]  = {0.5, 1.0 , 0.5, 1.0};
GLfloat blue1[4]   = {0.1, 0.1 , 1.0, 1.0};
GLfloat blue2[4]   = {0.2, 0.2 , 1.0, 1.0};
GLfloat blue3[4]   = {0.3, 0.3 , 1.0, 1.0};
GLfloat yellow1[4] = {0.1, 0.1 , 0.0, 1.0};
GLfloat yellow2[4] = {0.2, 0.2 , 0.0, 1.0};
GLfloat pink6[4]   = {1.0, 0.5,  0.5, 1.0};
GLfloat yellow5[4] = {0.8, 0.8 , 0.0, 1.0};
GLfloat abu2[4]    = {0.5, 0.5 , 0.5, 1.0};
GLfloat putih1[4]  = {1.0, 1.0,  1.0, 1.0};



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
  GLfloat dx=4.5,dy=4.5;
  GLint amount=15;
  GLfloat x_min=-dx/2.0, x_max=dx/2.0, x_sp=(GLfloat) dx/amount, y_min=-dy/2.0, y_max=dy/2.0, y_sp=(GLfloat) dy/amount;

  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, green1);
  for(i = 0; i<=48; i++){
     drawOneLine(-2.4+0.1*i, -2.4,       -2.4+0.1*i,  2.4);
     drawOneLine(-2.4,       -2.4+0.1*i,  2.4,       -2.4+0.1*i);
  }

  glPopMatrix();
}


void disp_robot(void)
{
  glPushMatrix();
    glTranslatef(Xoffset, Yoffset, Zoffset/2);
    // Draw base
    model_cylinder(obj, 0.1, 0.1, Zoffset, 2, putih1, putih1);
    // Menuju joint-1 -> bawah
    glTranslatef(0, 0, Zoffset/2);
    glRotatef(*tetha1*RTD-90,0,0,1);
    glPushMatrix();
      glDisable(GL_LIGHTING);    
      glColor3f(1.0, 0.5,  0.5);  // buat tanda biru (R G B) = 0 0 1      
      glTranslatef(0,0,0.04); // pindahkan offset 1cm diatas sendi 1
      gluDisk(obj, 0.001, 0.03, 20, 2); // bikin tanda lingkaran merah diatas sendi 1  
      glEnable(GL_LIGHTING);     
      // Gambar link1
      glRotatef(-90,1,0,0);
      glTranslatef(0,0,Link1/2);
      model_cylinder(obj, 0.03, 0.03, Link1, 2, abu2, abu2);
    glPopMatrix();
    // Menuju joint-2 -> pukul
    glTranslatef(0,Link1, 0);
    glRotatef(*tetha2*RTD,0,0,1);
    glPushMatrix();
      glDisable(GL_LIGHTING);
      glColor3f(0.0,0.0,1.0);  // buat tanda biru (R G B) = 0 0 1             
      glTranslatef(0,0,0.04); // pindahkan offset 1cm diatas sendi 1
      gluDisk(obj, 0.001, 0.03, 20, 2); // bikin tanda lingkaran merah diatas sendi 1
      glEnable(GL_LIGHTING);
    glPopMatrix();
    glPushMatrix();
      // Gambar link2
      glRotatef(-90,1,0,0);
      glTranslatef(0,0,Link2/2);
      model_cylinder(obj, 0.03, 0.03, Link2, 2, abu2, abu2);
    glPopMatrix();
    glPushMatrix();
      glDisable(GL_LIGHTING);
      glColor3f(0.5,0.5,0.5);  // buat tanda biru (R G B) = 0 0 1             
      glTranslatef(0,Link2,0.04); // pindahkan offset 1cm diatas sendi 2
      gluDisk(obj, 0.001, 0.01, 20, 2); // bikin tanda lingkaran merah diatas sendi 2 
      glEnable(GL_LIGHTING);
    glPopMatrix();
  glPopMatrix();
  glPushMatrix();
    glTranslatef(*x, *y, Zoffset/2);
    // Draw obj
    glRotatef(-90,1,0,0);    //rotasi objek
    model_cylinder(obj, 0.03, 0.03, Zoffset, 2, green2, green2);
    
    glDisable(GL_LIGHTING);
    glColor3f(1.0,0.0,0.0); // buat tanda merah (R G B) = 1 0 0 
    glTranslatef(0,0,Zoffset/2+0.01); // pindahkan offset 1cm diatas object
    //gluDisk(obj, 0.001, 0.01, 20, 2); // bikin tanda lingkaran merah diatas object 
    glEnable(GL_LIGHTING);

  glPopMatrix();

}





