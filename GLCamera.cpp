/*
Szymon Rusinkiewicz
Princeton University

GLCamera.cc
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
#include <GL/glew.h>
#include <GL/glut.h>
#include <GL/gl.h>

#include <math.h>
#include "GLCamera.h"

#ifndef M_PI
# define M_PI 3.14159265358979323846
#endif

#define DOF 10.0f
#define MAXDOF 10000.0f
#define SPIN_TIME 0.1f
#define SPIN_SPEED 0.05f
#define TRACKBALL_R 0.8f
#define DEPTH_FUDGE (1.0f + 0.2f * fov)
#define MOVEZ_MULT 5.0f
#define WHEEL_MOVE 0.2f
#define MAX_LIGHT M_PI


// Read back the framebuffer at the given pixel, and determine
// the 3D point there.  If there's nothing there, reads back a
// number of pixels farther and farther away.

float len(Vec3f Vectors)
{
	float a = Vectors[0]*Vectors[0] + Vectors[1]*Vectors[1] + Vectors[2]*Vectors[2];
	float b = sqrt(a);
	return b;
}

float sqr(Vec3f Vectors)
{
	float a = Vectors[0]*Vectors[0] + Vectors[1]*Vectors[1] + Vectors[2]*Vectors[2];
	return a;
}


bool GLCamera::read_depth(int x, int y, Vec3f &p) const
{
	double M[16], P[16]; int V[4];
	glGetDoublev(GL_MODELVIEW_MATRIX, M);
	glGetDoublev(GL_PROJECTION_MATRIX, P);
	glGetIntegerv(GL_VIEWPORT, V);

	static const float dx[] =
		{ 0, 1,-1,-1, 1, 3,-3, 0, 0, 6,-6,-6, 6, 25,-25,  0,  0 };
	static const float dy[] =
		{ 0, 1, 1,-1,-1, 0, 0, 3,-3, 6, 6,-6,-6,  0,  0, 25,-25 };
	const float scale = 0.01f;
	const int displacements = sizeof(dx) / sizeof(float);

	int xmin = V[0], xmax = V[0]+V[2]-1, ymin = V[1], ymax = V[1]+V[3]-1;

	for (int i = 0; i < displacements; i++) {
		int xx = min(max(x + int(dx[i]*scale*V[2]), xmin), xmax);
		int yy = min(max(y + int(dy[i]*scale*V[3]), ymin), ymax);
		float d;
		glReadPixels(xx, yy, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &d);
		if (d > 0.0001f && d < 0.9999f) {
			double X, Y, Z;
			gluUnProject(xx, yy, d, M, P, V, &X, &Y, &Z);
			p = Vec3f(X, Y, Z);
			return true;
		}
	}
	return false;
}


// Mouse helper - decide whether to start auto-spin
void GLCamera::startspin()
{
	//float dt = now() - last_time;
	float dt = 0.1;
	if (dt < SPIN_TIME && fabs(spinspeed) > SPIN_SPEED)
		dospin = true;
}


// Mouse rotation helper - compute trackball position from mouse pos
Vec3f GLCamera::mouse2tb(float x, float y)
{
	float r2 = x*x + y*y;
	float t = 0.5 * TRACKBALL_R*TRACKBALL_R;

	Vec3f pos(x, y, 0);
	if (r2 < t)
		pos[2] = sqrt(2*t - r2);
	else
		pos[2] = t / sqrt(r2);
	return pos;
}

// Mouse helper - rotate
void GLCamera::rotate(int mousex, int mousey, xform &xf)
{
	float ox = (lastmousex - tb_screen_x) / tb_screen_size;
	float oy = (lastmousey - tb_screen_y) / tb_screen_size;
	float nx = (    mousex - tb_screen_x) / tb_screen_size;
	float ny = (    mousey - tb_screen_y) / tb_screen_size;

	Vec3f pos1 = mouse2tb(ox, oy);
	Vec3f pos2 = mouse2tb(nx, ny);
	//spinaxis = pos1 CROSS pos2;
	spinaxis[0] = pos1[1]*pos2[2]-pos1[2]*pos2[1];
	spinaxis[1] = pos2[0]*pos1[2]-pos1[0]*pos2[2];
	spinaxis[2] = pos1[0]*pos2[1]-pos2[0]*pos1[1];
	float spinamount = sqrt((nx-ox)*(nx-ox)+(ny-oy)*(ny-oy));

	xf = xform::trans(spincenter) * xform::rot(spinamount, spinaxis) *
	     xform::trans(-spincenter) * xf;

	float dt = 0.1;//now() - last_time;
	if (dt > SPIN_TIME)
		spinspeed = spinamount / SPIN_TIME;
	else
		spinspeed = (spinamount / SPIN_TIME) +
			    (1.0f-dt/SPIN_TIME)*spinspeed;
}


// Mouse helper - translate
void GLCamera::movexy(int mousex, int mousey, xform &xf)
{
	float dx = pixscale * click_depth * (mousex - lastmousex);
	float dy = pixscale * click_depth * (mousey - lastmousey);
	xf = xform::trans(dx, dy, 0) * xf;
}


// Mouse helper - translate in Z
// In order to be extra-clever, though, this actually translates along the
// direction of the center of the trackball
void GLCamera::movez(int mousex, int mousey, xform &xf)
{
	float delta = MOVEZ_MULT / fov * pixscale *
		      ((mousex-lastmousex) - (mousey-lastmousey));
	float dz = click_depth * (exp(-delta) - 1.0f);
	//xf = xform::trans(0, 0, -dz) * xf;
	//xf = xform::trans(dz * spincenter / len(spincenter)) * xf;
	float lenspin = spincenter[0]*spincenter[0] + spincenter[1]*spincenter[1] + spincenter[2]*spincenter[2];
	lenspin = sqrt(lenspin);
	xf = xform::trans(dz * spincenter / lenspin) * xf;

	surface_depth += dz;
	if (surface_depth < 0)
		surface_depth = 0;
	click_depth += dz;
	if (click_depth < 0)
		click_depth = 0;
}


// Mouse helper - wheel motion
void GLCamera::wheel(Mouse::button updown, xform &xf)
{
	float dz = click_depth * WHEEL_MOVE;
	if (updown == Mouse::WHEELUP)
		dz = -dz;
	xf = xform::trans(0, 0, dz) * xf;

	surface_depth -= dz;
	if (surface_depth < 0)
		surface_depth = 0;
	click_depth -= dz;
	if (click_depth < 0)
		click_depth = 0;
}


// Mouse helper - change lighting direction
void GLCamera::relight(int mousex, int mousey)
{
	int V[4];
	glGetIntegerv(GL_VIEWPORT, V);

	float x = 2.0f * float(mousex - V[0]) / float(V[2]) - 1.0f;
	float y = 2.0f * float(mousey - V[1]) / float(V[3]) - 1.0f;

	float theta = MAX_LIGHT * min(sqrt(x*x+y*y), 1.0f);
	float phi = atan2(y, x);

	lightdir[0] = sin(theta)*cos(phi);
	lightdir[1] = sin(theta)*sin(phi);
	lightdir[2] = cos(theta);
}


// Handle a mouse click - sets up rotation center, pixscale, and click_depth
void GLCamera::mouse_click(int mousex, int mousey,
			   const Vec3f &scene_center, float scene_size)
{
	double M[16], P[16]; int V[4];
#if 0
	glGetDoublev(GL_MODELVIEW_MATRIX, M);
#else
	M[0] = M[5] = M[10] = M[15] = 1.0;
	M[1] = M[2] = M[3] = M[4] =
	M[6] = M[7] = M[8] = M[9] =
	M[11] = M[12] = M[13] = M[14] = 0;
#endif
	glGetDoublev(GL_PROJECTION_MATRIX, P);
	glGetIntegerv(GL_VIEWPORT, V);

	// Assume glFrustum only...
	pixscale = 2.0f / max(P[0]*V[2], P[5]*V[3]);


	Vec3f surface_point;
	if (read_depth(mousex, mousey, surface_point))
		click_depth = -surface_point[2];
	else
		click_depth = surface_depth;


	double cx, cy, cz;
	gluProject(scene_center[0], scene_center[1], scene_center[2],
		   M, P, V,
		   &cx, &cy, &cz);

	double csize = max(V[2], V[3]);
	int xmin = V[0], xmax = V[0]+V[2], ymin = V[1], ymax = V[1]+V[3];
	if (scene_center[2] < 0 && len(scene_center) > scene_size) {
		csize = -scene_size / scene_center[2] / pixscale;
		xmin = min(max(int(cx - csize), V[0]), V[0]+V[2]);
		xmax = min(max(int(cx + csize), V[0]), V[0]+V[2]);
		ymin = min(max(int(cy - csize), V[1]), V[1]+V[3]);
		ymax = min(max(int(cy + csize), V[1]), V[1]+V[3]);
	}

	double s[3];
	gluUnProject((xmin+xmax)/2, (ymin+ymax)/2, 1,
		     M, P, V,
		     &s[0], &s[1], &s[2]);
	spincenter = Vec3f(s[0],s[1],s[2]);
	normalize(spincenter);
	if (read_depth((xmin+xmax)/2, (ymin+ymax)/2, surface_point))
		spincenter *=  DEPTH_FUDGE * surface_point[2] / spincenter[2];
	else
		spincenter *= -DEPTH_FUDGE * click_depth / spincenter[2];

	float f = csize / max(V[2], V[3]);
	f = min(max(2.0f * f - 1.0f, 0.0f), 1.0f);
	spincenter = f * spincenter + (1.0f - f) * scene_center;

	gluProject(spincenter[0], spincenter[1], spincenter[2],
		   M, P, V,
		   &cx, &cy, &cz);
	tb_screen_x = cx;
	tb_screen_y = cy;
	tb_screen_size = csize;
	tb_screen_size = min(tb_screen_size, 0.7f * min(V[2], V[3]));
	tb_screen_size = max(tb_screen_size, 0.3f * min(V[2], V[3]));
}


// Handle a mouse event 
void GLCamera::mouse(int mousex, int mousey, Mouse::button b,
		     const Vec3f &scene_center, float scene_size,
		     xform &xf)
{
	if (b == Mouse::NONE && lastb == Mouse::NONE)
		return;

	int V[4];
	glGetIntegerv(GL_VIEWPORT, V);
	mousey = V[1] + V[3] - 1 - mousey;

	dospin = false;
	if ((b != lastb) && (b != Mouse::NONE))
		mouse_click(mousex, mousey, scene_center, scene_size);

	// Handle rotation
	if ((b == Mouse::ROTATE) && (lastb == Mouse::NONE))
		spinspeed = 0;
	if ((b == Mouse::ROTATE) && (lastb == Mouse::ROTATE))
		rotate(mousex, mousey, xf);
	if ((b == Mouse::NONE) && (lastb == Mouse::ROTATE))
		startspin();

	// Handle translation
	if ((b == Mouse::MOVEXY) && (lastb == Mouse::MOVEXY))
		movexy(mousex, mousey, xf);
	if ((b == Mouse::MOVEZ) && (lastb == Mouse::MOVEZ))
		movez(mousex, mousey, xf);
	if (b == Mouse::WHEELUP || b == Mouse::WHEELDOWN)
		wheel(b, xf);

	// Handle lighting
	if (b == Mouse::LIGHT)
		relight(mousex, mousey);


	lastmousex = mousex;  lastmousey = mousey;  lastb = b;
	//last_time = now();
}


// Idle loop - handles auto-rotate.  Returns true iff auto-rotating
bool GLCamera::autospin(xform &xf)
{
	if (!dospin)
		return false;

	//float dt = now() - last_time;
	float dt = 0.1;
	float spinamount = spinspeed * dt;

	xf = xform::trans(spincenter) * xform::rot(spinamount, spinaxis) *
	     xform::trans(-spincenter) * xf;

	//last_time = now();
	return true;
}


// Set up the OpenGL camera for rendering
void GLCamera::setupGL(const Vec3f &scene_center, float scene_size) const
{
	int V[4];
	glGetIntegerv(GL_VIEWPORT, V);
	int width = V[2], height = V[3];

	Vec3f surface_point;
	if (read_depth(width/2, height/2, surface_point))
		surface_depth = -surface_point[2];

	float fardist  = max(-(scene_center[2] - scene_size),
			     scene_size / DOF);
	float neardist = max(-(scene_center[2] + scene_size),
			     scene_size / MAXDOF);
	surface_depth = min(surface_depth, fardist);
	surface_depth = max(surface_depth, neardist);
	surface_depth = max(surface_depth, fardist / MAXDOF);
	neardist = max(neardist, surface_depth / DOF);
	//neardist -= 20;

	float diag = sqrt(float(width*width + height*height));
	float top = (float) height/diag * 0.5f*fov * neardist;
	float bottom = -top;
	float right = (float) width/diag * 0.5f*fov * neardist;
	float left = -right;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(left, right, bottom, top, neardist, fardist);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	GLfloat light0_position[] = { lightdir[0], lightdir[1], lightdir[2], 0 };
	GLfloat light1_position[] = { -lightdir[0], -lightdir[1], -lightdir[2], 0 };
	GLfloat light2_position[] = { lightdir[2], 0, -lightdir[0], 0 };
	GLfloat light3_position[] = { -lightdir[2], 0, lightdir[0], 0 };
	GLfloat light4_position[] = { 0, lightdir[2], -lightdir[1], 0 };
	GLfloat light5_position[] = { 0, -lightdir[2], lightdir[1], 0 };
	glLightfv(GL_LIGHT0, GL_POSITION, light0_position);
	glLightfv(GL_LIGHT1, GL_POSITION, light1_position);
	glLightfv(GL_LIGHT2, GL_POSITION, light2_position);
	glLightfv(GL_LIGHT3, GL_POSITION, light3_position);
	glLightfv(GL_LIGHT4, GL_POSITION, light4_position);
	glLightfv(GL_LIGHT5, GL_POSITION, light5_position);
}

