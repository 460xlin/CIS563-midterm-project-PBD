#include "PBD.h"
#include <iostream>

#ifndef STEP
#define STEP 0.025
#endif

#ifndef FRAME
#define FRAME 480
#endif

int main(int argc, char* argv[])
{
    PBD pbd;
    pbd.initialize(10,10,0.1,5,glm::vec3(-10,10,-10),glm::vec3(10,10.1,10));

    Scene scene;
    scene.insert_primitive(new Sphere(glm::vec3(0,0,0),5.f));

    for(int i = 1;i<=FRAME;i++)
    {
        pbd.update(&scene, STEP);
        //std::cout << "frame:" << i << std::endl;
        //pbd.printAll();
        pbd.exportFile("cloth", i);
    }

    printf("finished");
    return 0;
}
