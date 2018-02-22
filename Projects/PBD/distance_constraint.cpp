#include "distance_constraint.h"
#include <cmath>
const float eplison = 0.02f; 

// constexpr T distance(Matrix<T, dim, 1> a, Matrix<T, dim, 1> b)
// {

// }

DistanceConstraint::DistanceConstraint(Matrix<T, dim, 1> a, Matrix<T, dim, 1> b, T k)
{
	stiff = k;
	p1 = a;
	p2 = b;
	restLength = DistanceConstraint::distance(a, b);
}



T DistanceConstraint::distance(Matrix<T, dim, 1> a, Matrix<T, dim, 1> b)
{
	Matrix<T, dim, 1> temp = a - b;
	return sqrt(temp[0]*temp[0] + temp[1]*temp[1] + temp[2]*temp[2]);
}

void DistanceConstraint::projectConstraint()
{


}