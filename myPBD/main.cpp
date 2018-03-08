#include "PBD.h"
#include <iostream>

#ifndef STEP
#define STEP 0.02
#endif

#ifndef FRAME
#define FRAME 120
#endif

int main(int argc, char* argv[])
{
    std::string name = "dragon";//"cloth" or "dragon"
    //Adjust stretch stiff and bend stiff when necessary
    PBD pbd(5,0.1f,0.5f,0.1f);
    if(pbd.initializeFromObj(name + ".obj", glm::vec3(0,5,0), glm::vec3(45,0,0), glm::vec3(1.f,1.f,1.f)))
        return 1;


    Scene scene;
    scene.insert_primitive(new Plane(glm::vec3(0,1,0), 0));
    //scene.insert_primitive(new Sphere(glm::vec3(0,0,0),5.f));

    for(int i = 1;i<=FRAME;i++)
    {
        //std::cout << "frame:" << i << "::::::::::::" << std::endl;
        //##########################################################
        //##########################################################
        //Move the primitives in the scene before update pbd
        //Create functions for primitive and its sub-class when necessary
        //TO DO:

        //##########################################################
        //##########################################################
        pbd.update(&scene, STEP, i);//i for debug
        pbd.exportFile(name, i);
    }

    printf("finished");
    return 0;
}
