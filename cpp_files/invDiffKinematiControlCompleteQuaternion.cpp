#include <iostream>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <string.h>
#include <vector>
#include <cmath>

//#include "ros/ros.h"
//#include <sensor_msgs/JointState.h>
//sensor_msgs::JointState msg;


int print_eigen(std::string str, Eigen::MatrixXd m)
{
	// Eigen Matrices do have rule to print them with std::cout
	std::cout << str<< std::endl << m << std::endl<< std::endl;
	return 0;
}


int main()
{

//statically allocated custom size matrix
Eigen::Matrix<double,1,6> A;  // 1 row and 6 columms
A << 0, -0.425, -0.3922, 0, 0, 0;
print_eigen("A", A);


Eigen::Matrix<double,1,6> D;  // 
D << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996;
print_eigen("D", D);


Eigen::Matrix<double,6,1> J1;  // 
J1 << 0.1625, 
	 0, 
	 1, 
	 0, 
	 0, 
	 0;
print_eigen("J1", J1);


//Access to the element of the matrix
//Remember the first element is 0
//std::cout << "The element at 1ft row and 1ft column is " << A(0, 0) << std::endl; 

// coseno : double cos(double x);
// seno : double sin(double x);

//How to do a function of a value in the matrix
//double cos1 = cos( A(0, 0));

//Could be printed with the variable or directrly with the function
//std :: cout << cos( A(0, 0)) << std::endl;;

//Multiply the cosine
//std :: cout << cos( A(0, 0))*cos( A(0, 1)) << std::endl;;

return 0;
}