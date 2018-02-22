#include <iostream>
#include "particles.h"

int main(int argc, char* argv[])
{

	Particles<float> * temp = new Particles<float>();
	temp->createFromObj("Fidget_Spinner.obj");
	std::cout << "!!!" << std::endl;
	return 0;
}

