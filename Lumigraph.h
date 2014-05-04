#pragma once

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
//#include <windows.h>
#include <GL/glut.h>
#include <GL/gl.h>
#include <math.h>
#include <algorithm>
#include <utility> 
#include<iostream>
#include "XForm.h"
#include "ReadInDataSet.h"


using namespace std;
using namespace cv;


class Lumigraph
{
public:
	Lumigraph(void);
	Lumigraph(String);

	~Lumigraph(void);
	int uv_count;
	int t_width;
	int s_height;
	unsigned char*ImgDataSeq;
	vector<pair<int,double>> CurrFrameWeights;
	vector<Point3d> proxyData;
	
	vector <Matx34d> AllCameraMat; 
	Mat Camera_K; // Camera Intrinsic Matrxi
	Mat discoeff;
	vector<pair<int,double>> GetWeights(Point3d proxyPoint, Point3d VirtualCameraLoc, vector<Point3d> AllCameraLocsP);
	vector<Matx33d> CalculateFundamentalMat(vector<Matx34d> allcameraMat, Matx34d curP);
	int kth;
	Mat RenderImage(vector<Point3d> proxyPoint, Point3d VirtualCameraLoc, vector<Point3d> AllCameraLocs,Matx33d VirtualRot,Vec3d VirtualTrans);
	Mat DrawImage(xform xf);
	Mat InterpolateRenderImage(Mat Img, vector<Vec2d> proxy2DPoint);
	int proxyWidth;
	int proxyHeight;
};

