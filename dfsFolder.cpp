

#include <io.h>
#include <string>
#include <fstream>
#include <iostream>
using namespace std;
void dfsFolder(string folderPath, ofstream &fout)
{
	_finddata_t FileInfo;
	string strfind = folderPath + "\\*";
	long Handle = _findfirst(strfind.c_str(), &FileInfo);

	if (Handle == -1L)
	{
		cerr << "can not match the folder path" << endl;
		exit(-1);
	}
	do
	{
		//
		if (FileInfo.attrib & _A_SUBDIR) 
		{

			if( (strcmp(FileInfo.name,".") != 0 ) &&(strcmp(FileInfo.name,"..") != 0)) 
			{
				string newPath = folderPath + "\\" + FileInfo.name;
				dfsFolder(newPath, fout);
			}
		}
		else 
		{
			//fout << folderPath << "\\" << FileInfo.name << " "; 
			fout << FileInfo.name << "\r\n";
		}
	}while (_findnext(Handle, &FileInfo) == 0); _findclose(Handle);
	fout.close();
}
