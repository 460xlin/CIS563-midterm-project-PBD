#include <iostream>
#include "particles.h"
#include <vector>
#include <fstream>
#include <sstream>
const unsigned int rows = 4;
const unsigned int coloums = 10;
const float step = 1.f;

using namespace std;


// #define _WRITEOBJ_
#ifdef _WRITEOBJ_
void writeObj( std::string & MeshFilePath)
{
    ofstream outfile(MeshFilePath);
    if(!outfile)
    {
        cout << "open error" << endl;
        exit(1);
    }

    outfile << "mtllib mesh.mtl" << "\n";
    outfile << "g default" << "\n";

    for(unsigned int i = 0; i < rows; ++i)
    {
    	for(unsigned int j = 0; j < coloums; ++j)
    	{
    		outfile << "v " << i*step << " "
                        	<< j*step << " " 
                       		<< 0 << "\n";  
    	}
   	
    }

    outfile << "s off" << endl;
    outfile << "gMesh1" << endl;
    outfile << "usemtl initialShadingGroup" << endl; 

    for(unsigned int i = 0; i < rows - 1; ++i)
    {
    	for(unsigned int j = 0; j < coloums - 1; ++j)
    	{
    		outfile << "f " << (i*coloums+j)+1 << " " << i*coloums+j+2 << " " << (i+1)*coloums+j+1 << " " << (i*coloums+j)+1 << endl;
            outfile << "f " << (i+1)*coloums+j+2 << " " << (i+1)*coloums+j+1 << " " << i*coloums+j+2 << " " << (i+1)*coloums+j+2 << endl;
    	}
    }

}
#endif




int main(int argc, char* argv[])
{
	Particles<float> * temp = new Particles<float>();
	temp->createFromObj("myMesh.obj");
	std::cout << "!!!" << std::endl;
	return 0;
}

