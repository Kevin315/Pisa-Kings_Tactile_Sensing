#include <GL/glut.h>
#include <math.h>
#include <cstdio>

#include <sr_grasp_msgs/KCL_ContactStateStamped.h>

#include <ros/ros.h>

double A;//=0.9; //0.86
double B;//=0.9; //1.6
double C;//=0.9; //0.95
double Z0;//=1.1;

int changingcolor=0;
int ch=0;
bool fullscreen = false;
bool mouseDown = false;

float xrot = 0.0f;
float yrot = 0.0f;

float xdiff = 0.0f;
float ydiff = 0.0f;


float fx=0;
float fy=0;
float fz=-3;
float Ftot=0;

float x;
float y;
float z;

float fn[3];
float ft[3];

float lt;

void drawTorque(float lt){

	float fRadius = 0.2f;
	//float fRadius = 0.4f;
	float fPrecision = 0.05f;

	float fCenterX = 0.0f;
	float fCenterY = 0.0f;

	float fAngle;
	float fX=0.0f;
	float fY=0.0f;

	glBegin(GL_LINE_STRIP);
	for(fAngle = 0.0f; fAngle <= fabs(0.05*lt * 3.14159); fAngle += fPrecision)
	{
		fX = fCenterX + (fRadius* static_cast<float>(sin(fAngle)));
		fY = fCenterY + (fRadius* static_cast<float>(cos(fAngle)));

		if (lt>0)  fX=-fX;
		glVertex3f(fX, fY, 0);
	}
	glEnd();

	glBegin(GL_TRIANGLES);						// Drawing Using Triangles
	glVertex3f( fX,fY+0.06f,0);				// Top //Default all 0.02
	glVertex3f( fX,fY-0.06f,0);				// Bottom Left
	glVertex3f( fX+0.06f,fY,0);				// Bottom Right
	glEnd();							// Finished Drawing The Triangle


}

void drawBox()
{
	glBegin(GL_QUADS);

	glColor3f(1.0f, 0.0f, 0.0f);

	// BOTTOM
	glVertex3f(-0.5f, -1.5f, 0.5f);
	glVertex3f(-0.5f, -1.5f, -0.5f);
	glVertex3f( 0.5f, -1.5f, -0.5f);
	glVertex3f( 0.5f, -1.5f, 0.5f);
	glEnd();



}

void light(){
	glEnable(GL_LIGHTING);
	GLfloat specular[] = {1.0, 1.0, 1.0, 1.0};
	GLfloat ambientLight[] = { 0.2f, 0.2f, 0.2f, 1.0f };

	glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
	glShadeModel(GL_SMOOTH);
	glEnable(GL_LIGHT0);

	glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);

	//specular = {1.0, 1.0, 1.0, 1.0};
	GLfloat position[] = { 0, 0, 5.0f, 1.0f };
	glLightfv(GL_LIGHT0, GL_POSITION, position);

	position[2] =5.0;
	glLightfv(GL_LIGHT1, GL_POSITION, position);




	glLightfv(GL_LIGHT1, GL_SPECULAR, specular);
	glEnable(GL_LIGHT1);

}


void drawEllipsoid(float a, float b, float c, int lats, int longs)
{

	glColor3f(0.1f, 0.2f, 0.0f);
//	glColor3f(0.0f, 1.0f, 1.0f);
	int i, j;
	for(i = lats/2; i <= lats; i++)
	{
		float lat0 =  M_PI * (-0.5f + (float) (i - 1) / lats);
		float z0 = sin(lat0);
		float zr0 = cos(lat0);

		float lat1 = M_PI * (-0.5f + (float) i / lats);
		float z1 = sin(lat1);
		float zr1 = cos(lat1);

		glBegin(GL_QUAD_STRIP);
		for(j = 0; j <= longs; j++)
		{
			float lng = 2* M_PI * (float) (j - 1) / longs;
			float x = cos(lng);
			float y = sin(lng);

			glNormal3f(x * zr0, y * zr0, z0);
			glVertex3f(x * zr0 * a, y * zr0 * b, z0 * c);
			glNormal3f(x * zr1, y * zr1, z1);
			glVertex3f(x * zr1 * a, y * zr1 * b, z1 * c);
		}
		glEnd();
	}

}


bool init()
{
	glClearColor(0.93f, 0.93f, 0.93f, 0.0f);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glClearDepth(1.0f);

	return true;
}

void display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	gluLookAt(
			0.0f, 0.0f, 6.0f,
			0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f);

	glRotatef(xrot, 1.0f, 0.0f, 0.0f);
	glRotatef(yrot, 0.0f, 1.0f, 0.0f);


	drawBox();
//	glRotatef(120,0,0,1);
		glTranslatef(0,0,Z0);
	drawEllipsoid(A,B,C, 15,30);
	glTranslatef(0,0,-Z0);
	light();
	static GLUquadricObj *q;
	q = gluNewQuadric();
	gluQuadricNormals (q,GLU_TRUE);

	if(changingcolor==100) changingcolor=0;
	changingcolor++;

	float ecolor[] = { 0.1*(changingcolor/10), 0.0f,1-0.1*changingcolor/10, 0.1f };
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, ecolor);


	glColor3f(0.2f, 0.1f, 0.6f);

	float theta1;
	float theta2;

	glTranslated( x, y, z);

	glBegin(GL_LINES);
	glVertex3f(0,0,0);
	//glVertex3f(x, y, z); // origin of the line
	glVertex3f(fn[0]/10,fn[1]/10, fn[2]/10); // ending point of the line
	glEnd( );

	glBegin(GL_LINES);
	glVertex3f(0,0,0);
	//glVertex3f(x, y, z); // origin of the line
	glVertex3f(ft[0]/10,ft[1]/10, ft[2]/10); // ending point of the line
	glEnd( );

	//if(fx>=0)*/ theta1=-(180/M_PI)*atan2(fz,fx);
	/*else */theta1=180-(180/M_PI)*atan2(fz,fx);
	theta2=(180/M_PI)*atan2(fy, sqrt(fx*fx+fz*fz));

	glRotated( 90, 0, 1,0);
	glRotated(theta1, 0,1 , 0);
	glRotated(theta2 ,1, 0, 0);
	//printf("theta1=%f\ntheta2=%f",(180/M_PI)*atan2(fz,fx),(180/M_PI)*atan2( fy,sqrt(fx*fx+fz*fz)));
	if(Ftot>0.2) {
		//gluCylinder(q,0.01,0.01*Ftot,Ftot/10,30,20);  //default
		gluCylinder(q,0.01,0.05*Ftot,Ftot/2,30,20);
		drawTorque(lt);
	}


	float mcolor[] = { 0.8f, 0.8f, 0.8f, 0.1f };
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, mcolor);


	glFlush();
	glutSwapBuffers();
	//glutPostRedisplay();
	//ros::spinOnce();
}

void resize(int w, int h)
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	glViewport(0, 0, w, h);

	gluPerspective(45.0f, 1.0f * w / h, 1.0f, 100.0f);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void idle()
{
/*	if (!mouseDown)
	{
		xrot += 0.3f;
		yrot += 0.4f;
	}*/
  	ros::spinOnce();
	usleep(10000);
	//glutPostRedisplay();
}

void keyboard(unsigned char key, int x, int y) {
	switch(key)
	{
	case 27 :
		exit(1); break;
	}
}

void specialKeyboard(int key, int x, int y)
{
	if (key == GLUT_KEY_F1)
	{
		fullscreen = !fullscreen;

		if (fullscreen)
			glutFullScreen();
		else
		{
			glutReshapeWindow(500, 500);
			glutPositionWindow(50, 50);
		}
	}
}

void mouse(int button, int state, int x, int y)
{
	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
	{
		mouseDown = true;

		xdiff = x - yrot;
		ydiff = -y + xrot;
	}
	else
		mouseDown = false;
}

void mouseMotion(int x, int y)
{
	if (mouseDown)
	{
		yrot = x - xdiff;
		xrot = y + ydiff;

		glutPostRedisplay();
	}
}

void Callback_contactstate(const sr_grasp_msgs::KCL_ContactStateStamped::ConstPtr& msg)
{
	/*
	// contloc
	x=(msg->contact_position.x*100.0);
	z=-(msg->contact_position.y*100.0);
	y=(msg->contact_position.z*100.0)-B;
	
	// Fnormal
  double Fnormal=msg->Fnormal;
	fn[0]=msg->contact_normal.x*Fnormal;
  fn[2]=-msg->contact_normal.y*Fnormal;
  fn[1]=msg->contact_normal.z*Fnormal;

	ft[0]=msg->tangential_force.x;
	ft[2]=-msg->tangential_force.y;
	ft[1]=msg->tangential_force.z;
	*/
		// contloc
	x=(msg->contact_position.x/10);
	z=(msg->contact_position.z/10);
	y=(msg->contact_position.y/10);
	
	// Fnormal
  double Fnormal=msg->Fnormal;
	fn[0]=msg->contact_normal.x*Fnormal;
  fn[1]=msg->contact_normal.y*Fnormal;
  fn[2]=msg->contact_normal.z*Fnormal;

	ft[0]=msg->tangential_force.x;
	ft[1]=msg->tangential_force.y;
	ft[2]=msg->tangential_force.z;
	
  Ftot=sqrt((fn[0]+ft[0])*(fn[0]+ft[0])+(fn[1]+ft[1])*(fn[1]+ft[1])+(fn[2]+ft[2])*(fn[2]+ft[2]));

	fx=fn[0]+ft[0];
  fy=fn[1]+ft[1];
  fz=fn[2]+ft[2];

  lt=(float) msg->Ltorque;

	glutPostRedisplay();
}

int main(int argc, char *argv[])
{



    ros::init(argc, argv, "visualiser");
	ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<sr_grasp_msgs::KCL_ContactStateStamped>("ContactState", 1000, Callback_contactstate);

    n.getParam("contact/a", A);
    n.getParam("contact/b", B);
    n.getParam("contact/c", C);
    n.getParam("contact/z0", Z0);
    /*A = 8.5;
    B = 16;
    C = 9.5;
    Z0 = 0;*/
    
    /*A = 16.7;
    B = 16.7;
    C = 11.5;
    Z0 = 0;*/
        
    A = 17.88;
    B = 17.88;
    C = 15.5;
    Z0 = 0;

A/=10;
B/=10;
C/=10;
Z0/=10;
	glutInit(&argc, argv);

	glutInitWindowPosition(50, 50);
	glutInitWindowSize(500, 500);

	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);

	std::string wintitle="Contact Point Visualisator "+n.getNamespace();
	glutCreateWindow(wintitle.c_str());

	glutDisplayFunc(display);
	glutKeyboardFunc(keyboard);
	glutSpecialFunc(specialKeyboard);
	glutMouseFunc(mouse);
	glutMotionFunc(mouseMotion);
	glutReshapeFunc(resize);
	glutIdleFunc(idle);

	if (!init())
		return 1;

	glutMainLoop();
	return 0;
}
