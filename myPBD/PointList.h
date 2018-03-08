//
// Created by sean on 3/2/18.
//

#ifndef MYPBD_POINTLIST_H
#define MYPBD_POINTLIST_H

#include <vector>
#include "glm/glm.hpp"

class PointList {
public:
    std::vector<glm::vec3> pos;
    std::vector<glm::vec3> posPredict;
    std::vector<glm::vec3> velocity;
    std::vector<bool> posLock;
    std::vector<float> invMass;
    int size;

    PointList();
    ~PointList();
    void resize(int _size);
    void clear();
    void unlockPosAll();
};


#endif //MYPBD_POINTLIST_H
