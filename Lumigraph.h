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

using namespace std;
using namespace cv;


class Lumigraph
{
public:
	Lumigraph(void);
	Lumigraph(String);

	~Lumigraph(void);
	int u_count;
	int v_count;
	int t_width;
	int s_height;
	unsigned char*ImgDataSeq;
};

