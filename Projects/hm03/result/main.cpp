#include <iostream>
#include <fstream>
#include <sstream>
#include <Eigen/Core>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;
using T = double;
const int dim = 3;

Matrix<T, dim ,1> computeSpringForce(T k, T l0, Matrix<T, dim ,1> x0, Matrix<T, dim ,1> x1)
{
	Matrix<T, dim ,1> dir = x0 - x1;
	T L = sqrt(dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2]);
	Matrix<T, dim ,1> springForce = -1 * k / L * (L/l0 - 1) * dir;


    return springForce;

}

Matrix<T, dim ,1> computeDampingForce(T gamma, Matrix<T, dim ,1> v0, Matrix<T, dim ,1> v1,
											   Matrix<T, dim ,1> x0, Matrix<T, dim ,1> x1)
{
	Matrix<T, dim ,1> dir = x0 - x1;
	T L = sqrt(dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2]);
	dir = dir/L;

	Matrix<T, dim ,1> dampingForce = -1 * gamma * dir * dir.transpose() * (v0 - v1);

	return dampingForce;	


}											   


bool myReadFile(string fileName, 
	  		    std::vector< T > & myYM,
  			    std::vector< T > & gamma,
  		   	    std::vector< T > & l0,
  	            std::vector< Eigen::Matrix<T, dim ,1> > & myX0,
  	            std::vector< Eigen::Matrix<T, dim ,1> > & myX1,
  	            std::vector< Eigen::Matrix<T, dim ,1> > & myX2,
  	            std::vector< Eigen::Matrix<T, dim ,1> > & myX3,
  	            std::vector< Eigen::Matrix<T, dim ,1> > & myV0,
  	            std::vector< Eigen::Matrix<T, dim ,1> > & myV1,
  	            std::vector< Eigen::Matrix<T, dim ,1> > & myV2,
  	            std::vector< Eigen::Matrix<T, dim ,1> > & myV3)
{
  ifstream inFile(fileName);
  if(inFile.is_open())
  {

  	T temp;
  	while(inFile >> temp)
  	{
  		myYM.push_back(temp);
  		T g, l, x, y, z;
  		inFile >> g >> l;
  		gamma.push_back(g);
  		l0.push_back(l);
  		inFile >> x >> y >> z;
  		myX0.push_back(Matrix<T, dim ,1>(x,y,z));

  		inFile >> x >> y >> z;
  		myX1.push_back(Matrix<T, dim ,1>(x,y,z));

  		inFile >> x >> y >> z;
  		myX2.push_back(Matrix<T, dim ,1>(x,y,z));

  		inFile >> x >> y >> z;
  		myX3.push_back(Matrix<T, dim ,1>(x,y,z));

  		inFile >> x >> y >> z;
  		myV0.push_back(Matrix<T, dim ,1>(x,y,z));

  		inFile >> x >> y >> z;
  		myV1.push_back(Matrix<T, dim ,1>(x,y,z));

  		inFile >> x >> y >> z;
  		myV2.push_back(Matrix<T, dim ,1>(x,y,z));

  		inFile >> x >> y >> z;
  		myV3.push_back(Matrix<T, dim ,1>(x,y,z));
  	}
	cout << "open successfully" << endl;
  	inFile.close();
  	return true;
  }
  else
  {
  	cout << "unable to open file." << endl;
  	return false;
  }

}


void myWriteFile(std::string fileName, std::vector <Matrix<T, dim ,1>> & mySpringForce, std::vector <Matrix<T, dim ,1>> & myDampingForce)
{
	ofstream outFile(fileName);
	if(!outFile)
	{
		cout << "open error" << endl;
		exit(1);
	}

	for(int i = 0; i < (int) mySpringForce.size(); i=i+4)
	{
		outFile << mySpringForce.at(i)[0] << " " << mySpringForce.at(i)[1] << " " << mySpringForce.at(i)[2] << " ";
		outFile << mySpringForce.at(i+1)[0] << " " << mySpringForce.at(i+1)[1] << " " << mySpringForce.at(i+1)[2] << " ";
		outFile << mySpringForce.at(i+2)[0] << " " << mySpringForce.at(i+2)[1] << " " << mySpringForce.at(i+2)[2] << " ";
		outFile << mySpringForce.at(i+3)[0] << " " << mySpringForce.at(i+3)[1] << " " << mySpringForce.at(i+3)[2] << " ";

		outFile << myDampingForce.at(i)[0] << " " << myDampingForce.at(i)[1] << " " << myDampingForce.at(i)[2] << " ";
		outFile << myDampingForce.at(i+1)[0] << " " << myDampingForce.at(i+1)[1] << " " << myDampingForce.at(i+1)[2] << " ";
		outFile << myDampingForce.at(i+2)[0] << " " << myDampingForce.at(i+2)[1] << " " << myDampingForce.at(i+2)[2] << " ";
		outFile << myDampingForce.at(i+3)[0] << " " << myDampingForce.at(i+3)[1] << " " << myDampingForce.at(i+3)[2] << " ";
		outFile << "\n";
	}

	outFile.close();
}


int main(int argc, char* argv[])
{
  
  string fileName ="actual_input.txt";
  std::vector< T > myYM;
  std::vector< T > gamma;
  std::vector< T > l0;
  std::vector< Eigen::Matrix<T, dim ,1> > myX0;
  std::vector< Eigen::Matrix<T, dim ,1> > myX1;
  std::vector< Eigen::Matrix<T, dim ,1> > myX2;
  std::vector< Eigen::Matrix<T, dim ,1> > myX3;
  std::vector< Eigen::Matrix<T, dim ,1> > myV0;
  std::vector< Eigen::Matrix<T, dim ,1> > myV1;
  std::vector< Eigen::Matrix<T, dim ,1> > myV2;
  std::vector< Eigen::Matrix<T, dim ,1> > myV3;

  bool myResult = myReadFile(fileName, myYM, gamma, l0,
  							    myX0, myX1, myX2, myX3,
  							    myV0, myV1, myV2, myV3);
  if(myResult)
  {
  	std::vector <Matrix<T, dim ,1>> mySpringForce;
  	std::vector <Matrix<T, dim ,1>> myDampingForce;
  	for(int i = 0; i < (int)myYM.size(); ++i)
  	{
  		
  		Matrix<T, dim ,1> f0 = computeSpringForce(myYM.at(i), l0.at(i), myX0.at(i), myX3.at(i)) + 
  							   computeSpringForce(myYM.at(i), l0.at(i), myX0.at(i), myX1.at(i));
  							   

  		Matrix<T, dim ,1> f1 = computeSpringForce(myYM.at(i), l0.at(i), myX1.at(i), myX0.at(i)) +
  							   computeSpringForce(myYM.at(i), l0.at(i), myX1.at(i), myX3.at(i)) +
  							   computeSpringForce(myYM.at(i), l0.at(i), myX1.at(i), myX2.at(i));

  		Matrix<T, dim ,1> f2 = computeSpringForce(myYM.at(i), l0.at(i), myX2.at(i), myX1.at(i)) +
  							   computeSpringForce(myYM.at(i), l0.at(i), myX2.at(i), myX3.at(i));

        Matrix<T, dim ,1> f3 = computeSpringForce(myYM.at(i), l0.at(i), myX3.at(i), myX0.at(i)) +
  							   computeSpringForce(myYM.at(i), l0.at(i), myX3.at(i), myX1.at(i)) +
  							   computeSpringForce(myYM.at(i), l0.at(i), myX3.at(i), myX2.at(i));

  		mySpringForce.push_back(f0);
  		mySpringForce.push_back(f1);
  		mySpringForce.push_back(f2);
  		mySpringForce.push_back(f3);

  		Matrix<T, dim ,1> fd0 = computeDampingForce(gamma.at(i), myV0.at(i), myV3.at(i), myX0.at(i), myX3.at(i)) +
  								computeDampingForce(gamma.at(i), myV0.at(i), myV1.at(i), myX0.at(i), myX1.at(i));

        Matrix<T, dim ,1> fd1 = computeDampingForce(gamma.at(i), myV1.at(i), myV0.at(i), myX1.at(i), myX0.at(i)) +
  								computeDampingForce(gamma.at(i), myV1.at(i), myV3.at(i), myX1.at(i), myX3.at(i)) +
  								computeDampingForce(gamma.at(i), myV1.at(i), myV2.at(i), myX1.at(i), myX2.at(i));
        
        Matrix<T, dim ,1> fd2 = computeDampingForce(gamma.at(i), myV2.at(i), myV1.at(i), myX2.at(i), myX1.at(i)) +
  								computeDampingForce(gamma.at(i), myV2.at(i), myV3.at(i), myX2.at(i), myX3.at(i));

  		Matrix<T, dim ,1> fd3 = computeDampingForce(gamma.at(i), myV3.at(i), myV0.at(i), myX3.at(i), myX0.at(i)) +
  								computeDampingForce(gamma.at(i), myV3.at(i), myV1.at(i), myX3.at(i), myX1.at(i)) +
  								computeDampingForce(gamma.at(i), myV3.at(i), myV2.at(i), myX3.at(i), myX2.at(i));
  		
  		myDampingForce.push_back(fd0);
  		myDampingForce.push_back(fd1);
  		myDampingForce.push_back(fd2);
  		myDampingForce.push_back(fd3);
  	}

  	
  	string outputFileName = "result.txt";
  	myWriteFile(outputFileName, mySpringForce, myDampingForce);



  }
  
  return 0;
}
