#ifndef GLCAMERA_H
#define GLCAMERA_H
/*
Szymon Rusinkiewicz
Princeton University

GLCamera.h
Manages OpenGL camera and trackball/arcball interaction
*/

#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include "opencv2/nonfree/features2d.hpp"
#include <opencv2/core/core.hpp>




#include <stdlib.h>
//#include <windows.h>
#include <GL/glut.h>
#include <GL/gl.h>
#include <math.h>

//#include "Vec.h"
#include "XForm.h"
//#include "timestamp.h"
using namespace std;
using namespace cv;

namespace Mouse {
	enum button { NONE, ROTATE, MOVEXY, MOVEZ, WHEELUP, WHEELDOWN, LIGHT };
};

class GLCamera {
private:
	int lastmousex, lastmousey;
	Mouse::button lastb;
	//timestamp last_time;

	Vec3f lightdir;

	bool dospin;
	Vec3f spincenter;
	Vec3f spinaxis;
	float spinspeed;

	float fov, pixscale;
	mutable float surface_depth;
	float click_depth;
	float tb_screen_x, tb_screen_y, tb_screen_size;
	bool read_depth(int x, int y, Vec3f &p) const;

	void startspin();
	Vec3f mouse2tb(float x, float y);
	void rotate(int mousex, int mousey, xform &xf);
	void movexy(int mousex, int mousey, xform &xf);
	void movez(int mousex, int mousey, xform &xf);
	void wheel(Mouse::button updown, xform &xf);
	void relight(int mousex, int mousey);
	void mouse_click(int mousex, int mousey,
			 const Vec3f &scene_center, float scene_size);

public:
	void set_fov(float _fov) { fov = _fov; }
	void mouse(int mousex, int mousey, Mouse::button b,
		   const Vec3f &scene_center, float scene_size,
		   xform &xf);
	void stopspin() { dospin = false; }
	bool autospin(xform &xf);
	void setupGL(const Vec3f &scene_center, float scene_size) const;
	GLCamera(float _fov = 0.7f) : lastb(Mouse::NONE), dospin(false),
				      spinspeed(0), fov(_fov),
				      surface_depth(0), click_depth(0)
	{
		lightdir[0] = lightdir[1] = 0; lightdir[2] = 1;
		//last_time = now();
	}
};


#endif
