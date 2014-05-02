
#include "ReadInDataSet.h"


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


 bool readStringList(const string& filename, vector<string>& l)
{
  l.clear();
  FileStorage fs(filename, FileStorage::READ);
  if( !fs.isOpened() )
    return false;
  FileNode n = fs.getFirstTopLevelNode();
  if( n.type() != FileNode::SEQ )
    return false;
  FileNodeIterator it = n.begin(), it_end = n.end();
  for( ; it != it_end; ++it )
    l.push_back((string)*it);
  return true;
}

bool ReadInImages(string Path,string dataname)
{
	string FullPathXML = Path + dataname + ".xml";
	//FileStorage fs(FullPathXML,FileStorage::READ);
	vector<string> imagelists;
	if(!readStringList(FullPathXML, imagelists))
	{
		cout << "Can't locate the file!" << endl;
    return 1;
	}
	string FullPathData = Path + dataname + "_data";

	FILE *lfdatafile1 = fopen(FullPathData.c_str(),"w"); //delete existing file
	fclose(lfdatafile1);	
	FILE *lfdatafile = fopen(FullPathData.c_str(),"ab");
	//fseek ( lfdatafile , 100 , SEEK_SET );
	for(int i = 0; i < imagelists.size(); i++)
	{
		
		string imgPath = Path + imagelists[i];
		Mat img = imread(imgPath);
		//unsigned char *input = (unsigned char*) malloc(sizeof(unsigned char) * img.cols * img.rows *3);
		unsigned char *input = (unsigned char *)(img.data);
		int de = img.channels();
		int t = fwrite(input,sizeof(unsigned char),3*img.cols*img.rows,lfdatafile);
		/*for(int s=0;s<img.rows;s++)
		{
			for(int t=0;t<img.cols;t++)
			{
				unsigned char bgrcolor[3];
				bgrcolor[0] = input[(img.cols*3)*s+3*t];
				bgrcolor[1] = input[(img.cols*3)*s+3*t+1];
				bgrcolor[2] = input[(img.cols*3)*s+3*t+2];				
				
				fwrite(bgrcolor,sizeof(unsigned char),sizeof(bgrcolor),lfdatafile);
				//fwrite(&bgrcolor[0],sizeof(unsigned char),1,lfdatafile);
				//fwrite(&bgrcolor[1],sizeof(unsigned char),1,lfdatafile);
				//fwrite(&bgrcolor[2],sizeof(unsigned char),1,lfdatafile);
			}
		}*/
		
			
	}
	
	fclose(lfdatafile);	
	int debug = 0;
	if(debug == 1)
	{
		string imgPath = Path + imagelists[0];
		Mat img = imread(imgPath);
		CvSize mSize;
		mSize.height = img.rows;
		mSize.width = img.cols;
		/*string imgPath = Path + imagelists[24];
		Mat img = imread(imgPath);
		namedWindow("Display window2", WINDOW_AUTOSIZE);
		imshow("Display window2",img);
		waitKey(0);*/
		FILE *lfdatafile = fopen(FullPathData.c_str(),"rb");
		//unsigned char *input = (unsigned char *)(img.data);
		char *input = (char*) malloc(sizeof(char) * mSize.height * mSize.width *3);
		fread(input,sizeof(char),3*mSize.width*mSize.height,lfdatafile);
		fclose(lfdatafile);
		
		/*//img.data = input;
		namedWindow("Display window", WINDOW_AUTOSIZE);
		imshow("Display window",img);
		waitKey(0);*/


		CvSize mSize1;
		mSize1.height = img.rows;
		mSize1.width = img.cols;
		IplImage* image1 = cvCreateImage(mSize1, 8, 3);
		//memcpy( image1->imageData, input, 3*mSize.height*mSize.width);
		for(int s=0;s<img.rows;s++)
		{
			for(int t=0;t<img.cols;t++)
			{
				*(image1->imageData + s*image1->widthStep+3*t) = input[img.cols*3*s+3*t];
				*(image1->imageData + image1->widthStep*s+3*t+1) = input[img.cols*3*s+3*t+1];
				*(image1->imageData + image1->widthStep*s+3*t+2) = input[img.cols*3*s+3*t+2];
			
			}
		}
		cvNamedWindow( "Display Window3", WINDOW_AUTOSIZE);
		cvShowImage( "Display Window3", image1 );
		waitKey(0);
		cvDestroyWindow("Display Window3");
		free(input);
	}

}

