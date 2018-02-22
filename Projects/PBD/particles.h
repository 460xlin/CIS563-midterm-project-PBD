#ifndef _PARTICLES_H
#define _PARTICLES_H
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include "distance_constraint.h"

using namespace Eigen;



template <class T>
class Particles
{
public:
	void createFromObj(const char*);
	Particles();

	void initialState();  // 1-3
	void updatePosition(); // 7
	void updateVelocity();
	void applyExforceToVelocity(); // 5
	void dampVelocity();  // 6
	void collisionConstraint(); // 8
	Matrix<T, dim, 1> getPosition(unsigned int);
	Matrix<T, dim, 1> getVelocity(unsigned int);

	void assignDistanceConstraints();
	std::vector<DistanceConstraint*> myDisCst;


private:
	std::vector< Matrix<T, dim, 1> > positions;	
	std::vector< Matrix<T, dim, 1> > velocities;
	std::vector< Matrix<T, dim, 1> > tempPositions;
	std::vector<T> inverseMass;
	std::vector<unsigned int> myIndices;

	void assignMass();
	void assignInitialVelocity();
};


#endif // _PARTICLES_H