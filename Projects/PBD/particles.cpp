#include "particles.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <Partio.h>
#include "tiny_obj_loader.h"

using namespace Eigen;
const float mass = 1;
const float timeStep = 0.01f;
const float gravityAcc = 9.8;
const float dampCoeff = 0.99f;



template<class T>
Particles<T>::Particles()
{
 

}



template<class T>
void Particles <T>::assignMass()
{
	inverseMass.clear();
	for(unsigned int i = 0; i < positions.size(); ++i)
	{
		inverseMass.push_back(1/mass);
	}


}

template<class T>
void Particles <T>::assignInitialVelocity()
{
	velocities.clear();
	for(unsigned int i = 0; i < positions.size(); ++i)
	{
		velocities.push_back(Matrix<T, dim, 1>(1,1,1));
	}
}

template<class T>
void Particles <T>::assignDistanceConstraints()
{
	// for(unsigned int i = 0; i < myIndices.size(); ++i)
	// {
		
	// }
}



template<class T>
void Particles <T>::collisionConstraint()
{

}

template<class T>
void Particles <T>::initialState()
{
	assignMass();
	assignInitialVelocity();
}



// This is a position before we account the internal forces
template<class T>
void Particles <T>::updatePosition()
{
	for(unsigned int i = 0; i < positions.size(); ++i)
	{
		tempPositions.at(i) = positions.at(i) + timeStep * velocities.at(i);
	}

}

template<class T>
void Particles <T>::updateVelocity()
{
	
}






template<class T>
Matrix<T, dim, 1> Particles <T>::getPosition(unsigned int index)
{
	if(index >= positions.size())
	{
		std::cout << "Index is out of positions' boundary!" << std::endl;
		return Matrix<T, dim, 1>(0, 0, 0);
	}
	return positions.at(index);
}


template<class T>
Matrix<T, dim, 1> Particles <T>::getVelocity(unsigned int index)
{
	if(index >= velocities.size())
	{
		std::cout << "Index is out of velocities' boundary!" << std::endl;
		return Matrix<T, dim, 1>(0, 0, 0);
	}
	return velocities.at(index);
}

template<class T>
void Particles <T>::applyExforceToVelocity()
{
	Matrix<T, dim, 1> dir = Matrix<T, dim, 1>(0, -1, 0);

	for(unsigned int i = 0; i < velocities.size(); i++)
	{
		velocities.at(i) += dir * timeStep * gravityAcc;
	}
}


template<class T>
void Particles <T>::dampVelocity()
{
	for(unsigned int i = 0; i < velocities.size(); ++i)
	{
		velocities.at(i) *= dampCoeff;
	}
} 

template <class T>	
void Particles <T>::createFromObj(const char* filename)
{

	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;
	const char* mtl_basspath = nullptr;
	std::string error = tinyobj::LoadObj(shapes, materials, filename, mtl_basspath);
	std::cout << error << std::endl;
	if(error.size() == 0){
		//count = 0;
		for(unsigned int i = 0; i < shapes.size(); ++i)
		{
			std::vector<T> &allPositions = shapes[i].mesh.positions;
			// std::vector<T> &normals = shapes[i].mesh.normals;
			// std::vector<T> &uvs = shapes[i].mesh.texcoords;
			std::vector<unsigned int> &indices = shapes[i].mesh.indices;

			// bool normalsExist = normals.size() > 0;
			// uvExist = uvs.size() > 0;

			for(unsigned int ui : indices)
			{
				myIndices.push_back(ui);
			}


			for(unsigned int j = 0; j < allPositions.size(); j+=3)
			{
				positions.push_back( Matrix<T, dim, 1>(allPositions.at(j), 
													   allPositions.at(j+1), 
													   allPositions.at(j+2)) );

				tempPositions.push_back( Matrix<T, dim, 1>(allPositions.at(j), 
													   allPositions.at(j+1), 
													   allPositions.at(j+2)) );
			}
			
				unsigned int tempIndex = positions.size()-1;
				std::cout <<"The last vertex position: " << positions.at(tempIndex)[0] << " " << positions.at(tempIndex)[1] << " " << positions.at(tempIndex)[2] << std::endl;
			
		}


	}
	else
	{
		std::cout << error << std::endl;
	}
	
}


template class Particles<float>;