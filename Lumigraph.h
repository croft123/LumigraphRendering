#pragma once

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
#include <algorithm>
#include <utility> 
#include<iostream>

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
	
	vector <Matx34d> AllCameraMat;
	vector<pair<int,double>> GetWeights(Point3d proxyPoint, Point3d VirtualCameraLoc, vector<Point3d> AllCameraLocs);
};

