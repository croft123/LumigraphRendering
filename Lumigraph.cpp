#include "Lumigraph.h"


Lumigraph::Lumigraph(void)
	: u_count(0)
	, v_count(0)
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
		printf("Could not open file %s", param.c_str()); 
		return;
	}

	// Read in parameters
	int t_width, s_height, u_count, v_count;

	//Read in Image data
	this->ImgDataSeq = (unsigned char*) malloc(sizeof(unsigned char) * t_width * s_height * u_count * v_count *3);

	
}