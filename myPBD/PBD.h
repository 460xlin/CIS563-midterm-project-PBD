//
// Created by sean on 3/2/18.
//

#ifndef MYPBD_PBD_H
#define MYPBD_PBD_H

#include "PointList.h"
#include "Constraint.h"
#include "Scene.h"
#include <string>

class PBD {
public:
    struct Edge{
        int v1, v2;
        int tri1, tri2;
    };

    PointList pointList;
    int dimX;
    int dimZ;
    float thickness;
    int iteration;
    std::vector<Constraint*> constraint;
    std::vector<CollisionConstraint> constraintCollision;
    std::vector<SelfCollisionConstraint> constraintSelfCollision;
    std::vector<int> triangleList;
    std::vector<Edge> edgeList;

    PBD();
    ~PBD();
    void initialize(int _dimX, int _dimZ, float _thickness, int _iteration, glm::vec3 clothMin, glm::vec3 clothMax);
    void update(Scene* s, float dt);
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
    void resolveConstraints();
    void integration(float dt);
    void updateVelocity(float friction, float restitution);
    void cleanCollisionConstraints();
};


#endif //MYPBD_PBD_H
