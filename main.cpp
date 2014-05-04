#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include "opencv2/nonfree/features2d.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdlib.h>
#include <GL/glew.h>
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <math.h>

#include "XForm.h"
#include "GLCamera.h"
#include "ReadInDataSet.h"
#include "dfsFolder.h"
#include "Lumigraph.h"


using namespace std;
using namespace cv;


//Globals
GLCamera camera;
xform xf;
vector<Lumigraph> lg;
static unsigned buttonstate = 0;
Vec3f center(0,0,0);
double size = 5.0;

#define WIDTH 640
#define HEIGHT 480
#define DEBUG 0


//Display Function
void redraw()
{
	camera.setupGL(xf * center, size);
	glClearColor(1, 1, 1, 1);
	glClearDepth(1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDisable(GL_CULL_FACE);
	float* pixels = (float*) malloc(sizeof(float) * lg[0].t_width * lg[0].s_height * 3);
	int index = 0;

	glMatrixMode(GL_PROJECTION);
	glMultMatrixd(xf);
	Mat Img_ori = lg[0].DrawImage(xf);
	Mat Img;
	Size size(WIDTH,HEIGHT);
	resize(Img_ori,Img,size);

	for(int s = 0;s<HEIGHT;s++)
	{
		for(int t=0;t<WIDTH;t++)
		{
			pixels[index] = *(Img.data + Img.step[0]*s + Img.step[1]*t);
			pixels[index+1] = *(Img.data + Img.step[0]*s + Img.step[1]*t + 1);
			pixels[index+2] = *(Img.data + Img.step[0]*s + Img.step[1]*t + 2);
			index += 3;
		}
	}
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDrawPixels(WIDTH,HEIGHT,GL_RGB,GL_UNSIGNED_BYTE,pixels);
	glutSwapBuffers();
	glPopMatrix();
}

//Mouse Function
void mousemotionfunc(int x, int y)
{
	static const Mouse::button physical_to_logical_map[] = {
		Mouse::NONE, Mouse::ROTATE, Mouse::MOVEXY, Mouse::MOVEZ,
		Mouse::MOVEZ, Mouse::MOVEXY, Mouse::MOVEXY, Mouse::MOVEXY,
	};
	
	Mouse::button b = Mouse::NONE;
	
	if (buttonstate & (1 << 4))
		b = Mouse::WHEELUP;
	else 
	{
		if (buttonstate & (1 << 3))
			b = Mouse::WHEELDOWN;
		else
			b = physical_to_logical_map[buttonstate & 7];
	}
	//b = Mouse::MOVEXY;
	camera.mouse(x, y, b, xf * center, size, xf);
	if (b != Mouse::NONE)
		glutPostRedisplay();
}

void mousebuttonfunc(int button, int state, int x, int y)
{
	if (state == GLUT_DOWN)
		buttonstate |= (1 << button);
	else
		buttonstate = 0;//buttonstate &= ~(1 << button);
	
	mousemotionfunc(x, y);
}

void init(string in)
{
	lg.push_back(Lumigraph(in));

}

void resetview()
{
	xf = xform::trans(0, 0, -5.0f * size) * xform::trans(-center);
	//camera.stopspin();
	buttonstate = 0;
}

int main( int argc, char** argv )
{

	//string filepath = "C:\\HomeWork&Project\\CS684\\DataBase\\preview\\";
	try
	{
	string filepath = "C:\\OpenCV_Project\\SFM_Exp\\Test\\";
	string data = "Building";
	int WriteTXT = 0;
	if(WriteTXT == 1)
	{
		string dataTXT = filepath + data + ".txt";
		ofstream fout( dataTXT, ios::app);
		dfsFolder(filepath, fout);
	}
	ReadInImages(filepath,data);
	
	glutInitWindowPosition(100, 0);
	glutInitWindowSize(WIDTH,HEIGHT);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInit(&argc, argv);
	glutCreateWindow("Unstructured Lumigraph Viewer");
	glEnable(GL_TEXTURE_2D);

	glutDisplayFunc(redraw);
	glutMouseFunc(mousebuttonfunc);
	glutMotionFunc(mousemotionfunc);
	resetview();

	string filename = filepath + data;
	init(filename.c_str());
	glutMainLoop();
	}
	catch(cv::Exception & e)
	{
		cout<<e.msg<<endl;
	}

}