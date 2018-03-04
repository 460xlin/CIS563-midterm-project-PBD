//
// Created by sean on 3/2/18.
//

#include "PointList.h"

PointList::PointList() {

}

PointList::~PointList() {

}

void PointList::resize(int _size) {
    pos.clear();
    posPredict.clear();
    velocity.clear();
    posLock.clear();
    invMass.clear();

    size = _size;
    pos.resize(size);
    posPredict.resize(size);
    velocity.resize(size);
    posLock.resize(size);
    invMass.resize(size);
}

void PointList::unlockPosAll() {
    for(std::vector<bool>::iterator i = posLock.begin(); i != posLock.end(); ++i)
    {
        *i = false;
    }
}