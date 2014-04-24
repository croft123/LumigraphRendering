#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include "opencv2/nonfree/features2d.hpp"
#include <opencv2/core/core.hpp>
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


using namespace std;
using namespace cv;


int main( int argc, char** argv )
{
	string filepath = "C:\\HomeWork&Project\\CS684\\DataBase\\preview\\";
	string data = "jelly_beans";
	int WriteTXT = 0;
	if(WriteTXT == 1)
	{
		string dataTXT = "C:\\" + data + ".txt";
		ofstream fout( dataTXT, ios::app);
		dfsFolder(filepath, fout);
	}
	ReadInImages(filepath,data);


}