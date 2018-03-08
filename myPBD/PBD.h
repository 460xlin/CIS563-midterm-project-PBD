//
// Created by sean on 3/2/18.
//

#ifndef MYPBD_PBD_H
#define MYPBD_PBD_H

#include "PointList.h"
#include "Constraint.h"
#include "Scene.h"
#include <sstream>

class PBD {
public:
    struct Edge{
        int v1, v2;
        int tri1, tri2;
    };

    int iteration;
    float thickness;

    float stretch_stiff;
    float bend_stiff;

    PointList pointList;
    std::vector<Constraint*> constraint;
    std::vector<CollisionConstraint> constraintCollision;
    std::vector<SelfCollisionConstraint> constraintSelfCollision;
    std::vector<int> triangleList;
    std::vector<Edge> edgeList;

    PBD(int iteration, float thickness, float stretch_stiff = 0.5f, float bend_stiff = 0.1f);
    ~PBD();
    void initialize(int _dimX, int _dimZ, glm::vec3 clothMin, glm::vec3 clothMax);
    int initializeFromObj(std::string name, glm::vec3 T, glm::vec3 R, glm::vec3 S);
    void update(Scene* s, float dt, int frame);//frame is for debug
    void exportFile(std::string name, int frame);
    void printAll();
    void printAllPredict();
    void printDebug();
private:
    void generateEdgeList();
    void generateInternalConstraints();
    void applyExternalForce(glm::vec3 force, float dt);
    void dampVelocity(float kDamp);
    void computePredictedPostion(float dt);
    void collisionDetection(Scene* s);
    void selfCollisionDetection();
    void resolveConstraints(int frame);//frame for debug
    void integration(float dt);
    void updateVelocity(float friction, float restitution);
    void cleanCollisionConstraints();
    int loadObj(std::string name, glm::vec3 T, glm::vec3 R, glm::vec3 S);
    void parseObjFace(std::stringstream& ss, std::vector<int>& index);
};


#endif //MYPBD_PBD_H
