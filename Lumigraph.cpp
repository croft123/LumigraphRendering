#include "Lumigraph.h"

bool paircomp (pair<int,double> p, pair<int,double> q) { return (p.second < q.second); }

Lumigraph::Lumigraph(void)
	: uv_count(0)
	, t_width(0)
	, s_height(0)
	, ImgDataSeq(NULL)
	, kth(0)
	, proxyWidth(0)
{
}


Lumigraph::~Lumigraph(void)
{
}

Lumigraph::Lumigraph(string filename)
{
	string param = filename + "_parameters.txt";
	string data = filename + "_data";

	// First read in parameters

	FILE* lfparamfile = fopen(param.c_str(), "r");

	if (lfparamfile == NULL) {
		printf("Could not open file %s\n", param.c_str()); 
		return;
	}

	// Read in parameters
	int t_width, s_height, uv_count;

	//Read in Image data
	this->ImgDataSeq = (unsigned char*) malloc(sizeof(unsigned char) * t_width * s_height * uv_count *3);
}





vector<pair<int,double>> Lumigraph::GetWeights(Point3d proxyPoint, Point3d VirtualCameraLoc, vector<Point3d> AllCameraLocs)
{
	vector<pair<int,double>> weights;
	vector<pair<int,double>> penalities;
	
	Point3d VirCamVec  = VirtualCameraLoc - proxyPoint;
	double dist_VirCam = norm(VirCamVec);
	for(int i = 0; i < AllCameraLocs.size(); i++){
		Vec3d curpenalities;

		//Angle Penalty: Compute Angles
		Point3d CurCamVec = AllCameraLocs[i] - proxyPoint;
		double dist_CurCam = norm(CurCamVec);
		double curangle = CurCamVec.dot(VirCamVec);
		curangle = curangle / (dist_VirCam * dist_CurCam);
		curangle = acos(curangle);
		curpenalities[0] = (curangle);

		//Resolution Penalty
		curpenalities[1] = (dist_CurCam - dist_CurCam > 0)?(dist_CurCam - dist_CurCam):0;

		//Field of View Penalty
		// project to current camera plane
		//decomposition
		Matx33d K, R;
		Vec4d T;
		Vec2d curimgPoint;
		decomposeProjectionMatrix(AllCameraMat[i], K, R, T);
		projectPoints((Vec3d)proxyPoint, K, R, T, NULL, curimgPoint);
		if(curimgPoint[0] > t_width || curimgPoint[1] > s_height)curpenalities[2] = DBL_MAX;
		else curpenalities[2] = 0;

		//total penalty
		double totalcurpenalty;
		double alpha = 0, beta = 0, gamma = 0;
		totalcurpenalty = alpha * curpenalities[0] + beta * curpenalities[1] + gamma * curpenalities[2];
		
		penalities.push_back(make_pair(i,totalcurpenalty));
	}
	
	// find the first kth smallest element in penalities
	nth_element(penalities.begin(), penalities.begin() + 5, penalities.end(), paircomp);
	double totalweights = 0;
	for(int i = 0; i < this->kth; i++){
		weights.push_back(penalities[i]);
		weights[i].second = 1 - weights[i].second / (penalities.begin() + this->kth)->second;
		totalweights += weights[i].second;
	}
	//normalize
	for(int i = 0; i < this->kth; i++){
		weights[i].second /= totalweights;
	}		
	return weights;
}


vector<Matx33d> Lumigraph::CalculateFundamentalMat(vector<Matx34d> allcameraMat, Matx34d curP)
{
	vector<Matx33d> F;
	Matx41d C(0,0,0,1);
	for(int i=0;i<allcameraMat.size();i++)
	{	
		Matx33d temp_F;
		Matx31d P2C = allcameraMat[i] * C;
		Matx33d P2Cx(0, -P2C(2,0), P2C(1,0),
						P2C(2,0), 0, -P2C(0,0),
						-P2C(1,0), P2C(0,0), 0);
		SVD svd(allcameraMat[i]);
		Mat_<double> pinvP(4,3);
		pinvP = svd.vt.t() * Mat::diag(1./svd.w)*svd.u.t();
		Matx43d temp_P(pinvP(0,0),pinvP(0,1),pinvP(0,2),
						pinvP(1,0),pinvP(1,1),pinvP(1,2),
						pinvP(2,0),pinvP(2,1),pinvP(2,2),
						pinvP(3,0),pinvP(3,1),pinvP(3,2));

		temp_F = P2Cx * curP * temp_P;
		F.push_back(temp_F);
	}
	return F;
}


Mat Lumigraph::RenderImage(vector<Point3d> proxyPoint, Point3d VirtualCameraLoc, vector<Point3d> AllCameraLocs,Matx34d VirtualP)
{
	Mat Img;
	vector<Vec2d> ProxyProjectPoint;
	
	for(int numP = 0;numP<proxyPoint.size();numP++)
	{
		CurrFrameWeights = this->GetWeights(proxyPoint[numP],VirtualCameraLoc,AllCameraLocs);
		double addWeightInten_R = 0;
		double addWeightInten_G = 0;
		double addWeightInten_B = 0;
		for(int i=0;i<this->kth;i++)
		{
			Matx33d K, R;
			Vec4d T;
			Vec2d curimgPoint;
			int Num = CurrFrameWeights[i].first;
			decomposeProjectionMatrix(this->AllCameraMat[Num], K, R, T);
			projectPoints((Vec3d)proxyPoint[numP], K, R, T, NULL, curimgPoint);

			unsigned char curimgIntensity_R = this->ImgDataSeq[t_width*s_height*3*Num + (int)curimgPoint[0]*t_width*3 + (int)curimgPoint[1]];
			unsigned char curimgIntensity_G = this->ImgDataSeq[t_width*s_height*3*Num + (int)curimgPoint[0]*t_width*3 + (int)curimgPoint[1] + 1];
			unsigned char curimgIntensity_B = this->ImgDataSeq[t_width*s_height*3*Num + (int)curimgPoint[0]*t_width*3 + (int)curimgPoint[1] + 2];
			addWeightInten_R += curimgIntensity_R * CurrFrameWeights[i].second;
			addWeightInten_G += curimgIntensity_G * CurrFrameWeights[i].second;
			addWeightInten_B += curimgIntensity_B * CurrFrameWeights[i].second;
		}
		Matx33d K1, R1;
		Vec4d T1;
		Vec2d virtualPoint;
		//int Num = CurrFrameWeights[i].first;
		decomposeProjectionMatrix(VirtualP, K1, R1, T1);
		projectPoints((Vec3d)proxyPoint[numP], K1, R1, T1, NULL, virtualPoint);
		//assign intesity to Img
		*(Img.data + Img.step[0]*(int)virtualPoint[0] + Img.step[1]*(int)virtualPoint[1]) = addWeightInten_R;
		*(Img.data + Img.step[0]*(int)virtualPoint[0] + Img.step[1]*(int)virtualPoint[1] + 1) = addWeightInten_G;
		*(Img.data + Img.step[0]*(int)virtualPoint[0] + Img.step[1]*(int)virtualPoint[1] + 2) = addWeightInten_B;
		ProxyProjectPoint.push_back(virtualPoint);
	}
	Mat RenderedImg = this->InterpolateRenderImage(Img,ProxyProjectPoint);

	return RenderedImg;
}



Mat Lumigraph::InterpolateRenderImage(Mat Img, vector<Vec2d> proxy2DPoint)
{

	for(int numP = 0;numP<proxy2DPoint.size();numP++)
	{
		for(int y = proxy2DPoint[numP][0];y<proxy2DPoint[numP+1][0];y++)
		{
			for(int x = proxy2DPoint[numP][1];x<proxy2DPoint[numP+proxyWidth][1];x++)
			{
				double x1 = proxy2DPoint[numP][1];
				double y1 = proxy2DPoint[numP][0];
				double x2 = proxy2DPoint[numP+1][1];
				double y2 = proxy2DPoint[numP+1][0];
				double x3 = proxy2DPoint[numP+proxyWidth][1];
				double y3 = proxy2DPoint[numP+proxyWidth][0];
				double x4 = proxy2DPoint[numP+proxyWidth+1][1];
				double y4 = proxy2DPoint[numP+proxyWidth+1][0];
				double Q11 = *(Img.data + Img.step[0]*(int)y1 + Img.step[1]*(int)x1);
				double Q12 = *(Img.data + Img.step[0]*(int)y2 + Img.step[1]*(int)x2);
				double Q21 = *(Img.data + Img.step[0]*(int)y3 + Img.step[1]*(int)x3);
				double Q22 = *(Img.data + Img.step[0]*(int)y4 + Img.step[1]*(int)x4);
				double R1 = (double)(x2-x)/(x2-x1)*Q11 + (double)(x-x1)/(x2-x1)*Q12;
				double R2 = (double)(x4-x)/(x4-x3)*Q21 + (double)(x-x3)/(x4-x3)*Q22;
				int P = (double)(y2-y)/(y2-y1)*R1 + (double)(y-y3)/(y4-y3)*R2;
				*(Img.data + Img.step[0]*(int)y + Img.step[1]*(int)x) = (unsigned char) P;
			}
		}
	}

	return Img;
}


Mat Lumigraph::DrawImage(xform xf)
{
	Point3d vCameraLoc;
	vCameraLoc.x = xf[12];
	vCameraLoc.y = xf[13];
	vCameraLoc.z = xf[14];
	Matx34d vP(xf[0],xf[4],xf[8],
				xf[1],xf[5],xf[9],
				xf[2],xf[6],xf[10]);
	vector<Point3d> AllCameraLocs;
	for(int i=0;i<this->AllCameraMat.size();i++)
	{
		Matx33d K, R;
		Vec4d T;
		Point3d vLoc;
		decomposeProjectionMatrix(this->AllCameraMat[i], K, R, T);
		vLoc.x = T[0];
		vLoc.y = T[1];
		vLoc.z = T[2];
		AllCameraLocs.push_back(vLoc);

	}
	Mat myImg = this->RenderImage(this->proxyData,vCameraLoc,AllCameraLocs,vP);
	return myImg;

}