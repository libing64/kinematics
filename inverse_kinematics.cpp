#include "inverse_kinematics.h"
#include <iostream>
#include <Eigen/Eigen>
#include <cstdlib>
#include <cstdio>

using namespace std;
using namespace Eigen;

#define UR10_PARAMS
#ifdef UR10_PARAMS
const double d[6]     =  {0.1273, 0, 0, 0.163941, 0.1157, 0.0922};
const double a[6]     =  {0, -0.612, -0.5723,  0,  0,  0};
const double alpha[6] =  {M_PI / 2, 0, 0, M_PI / 2, -M_PI / 2, 0};
const double offset[6] = {0, -M_PI / 2, 0, -M_PI / 2, 0, 0};
#endif

Matrix4d forward_kinematics(Vector6d theta)
{
	Matrix4d T = Matrix4d::Identity();
	for(int i = 0; i < 6; i++)
	{
		Matrix4d Ti;
		Ti << cos(theta(i)),    -sin(theta(i)) * cos(alpha[i]), sin(theta(i)) * sin(alpha[i]), a[i] * cos(theta(i)),
			  sin(theta(i)),    cos(theta(i))  * cos(alpha[i]), -cos(theta(i))*sin(alpha[i]),  a[i] * sin(theta(i)),
			  0,                sin(alpha[i]),                  cos(alpha[i]),                 d[i],
			  0,                0,                              0,                             1;
		T *= Ti;	  
	}
	return T;
}

Vector6d inverse_kinematics(Matrix4d T)
{
	Vector6d theta;
	return theta;
}

int main()
{
  	printf("a: %lf,%lf,%lf,%lf,%lf,%lf,\n", 
    	a[0], 
    	a[1], 
    	a[2], 
    	a[3],
    	a[4],
    	a[5]);

  	printf("d: %lf,%lf,%lf,%lf,%lf,%lf,\n", 
    	d[0], 
    	d[1], 
    	d[2], 
    	d[3],
    	d[4],
    	d[5]);


	srand(0);
  	Vector6d theta;
  	for (int i = 0; i < 6; i++)
  	{
    	theta(i) = 0;//i * M_PI / 2;
    	//theta(i) = rand() % 100 / 100.0 * M_PI;
  	}

  	cout << "theta: " << theta.transpose() << endl;

  	Matrix4d T = forward_kinematics(theta);
  	cout << "T:\n" << T << endl; 
  	cout << "T_inv:\n" << T.inverse() << endl; 
	return 0;
}