#include "Lumigraph.h"

bool paircomp (pair<int,double> p, pair<int,double> q) { return (p.second < q.second); }

Lumigraph::Lumigraph(void)
	: uv_count(0)
	, t_width(0)
	, s_height(0)
	, ImgDataSeq(NULL)
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
	for(int i = 0; i < 5; i++){
		weights.push_back(penalities[i]);
		weights[i].second = 1 - weights[i].second / (penalities.begin() + 5)->second;
		totalweights += weights[i].second;
	}
	//normalize
	for(int i = 0; i < 5; i++){
		weights[i].second /= totalweights;
	}		
	return weights;
}
