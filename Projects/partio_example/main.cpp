#include <Partio.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>

using T = double;
constexpr int dim = 3;
const int index = 21;
const double deltaT = 0.03; 
using namespace std;
template <class T, int dim>
void writePartio(const std::string& particleFile)
{
    Partio::ParticlesDataMutable* parts = Partio::create();
    Partio::ParticleAttribute posH, vH, mH;
    mH = parts->addAttribute("m", Partio::VECTOR, 1);
    posH = parts->addAttribute("position", Partio::VECTOR, 3);
    vH = parts->addAttribute("v", Partio::VECTOR, 3);
    for (int i=0; i<3; i++){
        int idx = parts->addParticle();
        float* m = parts->dataWrite<float>(mH, idx);
        float* p = parts->dataWrite<float>(posH, idx);
        float* v = parts->dataWrite<float>(vH, idx);
        m[0] = (T)i;
        for (int k = 0; k < 3; k++)
            p[k] = (T)i;
        for (int k = 0; k < 3; k++)
            v[k] = (T)i;
    }

    Partio::write(particleFile.c_str(), *parts);
    parts->release();
}


// ================================== Particle ===============================//

template <class T, int dim>
class Particles
{
public:
    void writeParticles(const std::string & particleFile);
    Particles();
    T getPositions(int);
    T getM(int);
    T getVelocity(int);
    void updatePosition();

private:
    T positions[index * 3];
    T m[index];
    T velocity[index * 3];
};


template <class T, int dim>
void Particles<T, dim>::updatePosition()
{
    for(int i = 0; i < index; i++)
    {
        for (int k = 0; k < 3; k++)
        {
            positions[i*3 + k]  = positions[i*3 + k] + velocity[i*3 + k] * deltaT;
            velocity[i*3 + k] = abs(2*sin((T)(rand()%31/10.)));
        }
        
    }
}

template <class T, int dim>
T Particles<T, dim>::getPositions(int temp)
{

    return positions[temp];

}

template <class T, int dim>
T Particles<T, dim>::getM(int temp)
{
    return m[temp];
}

template <class T, int dim>
T Particles<T, dim>::getVelocity(int temp)
{
    return velocity[temp];
}




template <class T, int dim>
Particles<T, dim>::Particles()
{
    for(int i = 0; i < index; i++)
    {
        m[i] = 1.;

        for(int k = 0; k < 3; k++)
        {
            positions[i*3 + k] = (T)(rand()%10);
        }
        for(int k = 0; k < 3; k++)
        {
            velocity[i*3 + k] = (T)(rand()%31/10.);
        }
    }
}

template <class T, int dim>
void Particles<T, dim>::writeParticles(const std::string & particleFile)
{

    Partio::ParticlesDataMutable* parts = Partio::create();
    Partio::ParticleAttribute posH, vH, mH;
    mH = parts->addAttribute("m", Partio::VECTOR, 1);
    posH = parts->addAttribute("position", Partio::VECTOR, 3);
    vH = parts->addAttribute("v", Partio::VECTOR, 3);
    for (int i = 0; i < index; i++)
    {
        int idx = parts->addParticle();
        float* m = parts->dataWrite<float>(mH, idx);
        float* p = parts->dataWrite<float>(posH, idx);
        float* v = parts->dataWrite<float>(vH, idx);

        m[0] = getM(i);

        for (int k = 0; k < 3; k++)
        {
            p[k] = getPositions(i*3 + k);
        }
        for (int k = 0; k < 3; k++)
        {
            v[k] = getVelocity(i*3 + k);
        }

    }
    Partio::write(particleFile.c_str(), *parts);
    parts->release();
    updatePosition();
}


// ==========================SegmentMesh ===================================
template <class T, int dim>
class SegmentMesh
{
public:
    SegmentMesh();
    void writePoly(const std::string & segmentFilePath);
    T getParticleM(int);
    T getParticleP(int);
    T getParticleV(int);
private:

    Particles<T, dim> myParticles;
    std::vector<int> myParticleIndices; 

};

template <class T, int dim>
SegmentMesh<T, dim>::SegmentMesh()
{

}

template <class T, int dim>
T SegmentMesh<T, dim>::getParticleM(int temp)
{
    return myParticles.getM(temp);

}

template <class T, int dim>
T SegmentMesh<T, dim>::getParticleP(int temp)
{
    return myParticles.getPositions(temp);
}

template <class T, int dim>
T SegmentMesh<T, dim>::getParticleV(int temp)
{
    return myParticles.getVelocity(temp);
}

template <class T, int dim>
void SegmentMesh<T, dim>::writePoly(const std::string & segmentFilePath)
{
    std::ofstream outfile(segmentFilePath);

    if(!outfile)
    {
        cout << "open error" << endl;
        exit(1);
    }

    outfile << "POINTS" << "\n";
    for(int i = 0; i < index; i++)
    {
        outfile << (i+1) << ": " << getParticleP(i*3+0) << " " 
                             << getParticleP(i*3+1) << " " 
                             << getParticleP(i*3+2) << "\n";
    }
    outfile << "POLYS" << "\n";
    outfile << 1 << ":";
    for(int i = 0; i < index; i++)
    {
        outfile << " "  << (i+1);
    }
    outfile << "\n";
    outfile << "END";
    outfile.close();

}


// ============================ TriangleMesh ===============================

template <class T, int dim>
class TriangleMesh
{
public:
    TriangleMesh();
    void writeObj(const std::string & MeshFilePath);
    T getParticleP(int);
    T getParticleV(int);
    T getParticleM(int);

private:
    Particles<T, dim> myParticles;

};
template <class T, int dim>
TriangleMesh<T, dim>::TriangleMesh()
{

}

template <class T, int dim>
T TriangleMesh<T, dim>::getParticleP(int temp)
{
    return myParticles.getPositions(temp);
}


template <class T, int dim>
T TriangleMesh<T, dim>::getParticleV(int temp)
{
    return myParticles.getVelocity(temp);
}


template <class T, int dim>
T TriangleMesh<T, dim>::getParticleM(int temp)
{
    return myParticles.getM(temp);
}




template <class T, int dim>
void TriangleMesh<T, dim>::writeObj(const std::string & MeshFilePath)
{
    ofstream outfile(MeshFilePath);
    if(!outfile)
    {
        cout << "open error" << endl;
        exit(1);
    }

    outfile << "mtllib mesh.mtl" << "\n";
    outfile << "g default" << "\n";
    for(int i = 0; i < index; i++)
    {
        outfile << "v " << getParticleP(i*3+0) << " "
                        << getParticleP(i*3+1) << " " 
                        << getParticleP(i*3+2) << "\n";                      
    }

    outfile << "s off" << endl;
    outfile << "gMesh1" << endl;
    outfile << "usemtl initialShadingGroup" << endl; 
    for(int i = 0; i < index; i=i+3)
    {
        outfile << "f " << i+1 << " " << i+2 << " " << i+3 << " " << i+1 << endl;
    }
}

// ============================= main ============================= //
int main(int argc, char* argv[])
{

    double mTime = 5.;
    std::vector<string> myFile;
    for(int i = 1; i <= (int)(mTime/deltaT); i++)
    {
        string FileName = "frame"+ std::to_string(i) + ".bgeo";       
        myFile.push_back(FileName);
    }
    //std::string file2 = "test2.bgeo";
    std::string file3 = "myPoly.poly";
    std::string file4 = "myMesh.obj";

    Particles<T, dim> *a = new Particles<T, dim>();
    for(int i = 1; i <= (int)(mTime/deltaT); i++)
    {
        a->writeParticles(myFile[i-1]);
    }



    SegmentMesh<T, dim> *mySegment = new SegmentMesh<T, dim>();
    TriangleMesh<T, dim> *myMesh = new TriangleMesh<T, dim>();


    mySegment->writePoly(file3);
    myMesh->writeObj(file4);

    delete a;
    return 0;
}
