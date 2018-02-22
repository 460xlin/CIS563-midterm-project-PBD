#ifndef _DISTANCE_CONSTRAINT_H
#define _DISTANCE_CONSTRAINT_H
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
constexpr int dim = 3;
using T = float;
using  namespace Eigen;


class DistanceConstraint
{
public:
	DistanceConstraint(Matrix<T, dim, 1>, Matrix<T, dim, 1>, T);
	void projectConstraint();
	static T distance(Matrix<T, dim, 1>, Matrix<T, dim, 1>);

private:

	T stiff; 
	T restLength;
	Matrix<T, dim, 1> p1;
	Matrix<T, dim, 1> p2;


};



#endif // _DISTANCE_CONSTRAINT_H