#include "inverse_kinematics.h"
#include <iostream>
#include <vector>
#include <Eigen/Eigen>
#include <cstdlib>
#include <cstdio>

using namespace std;
using namespace Eigen;

// #define UR10_PARAMS
#ifdef UR10_PARAMS
const double d[6]     =  {0.1273, 0, 0, 0.163941, 0.1157, 0.0922};
const double a[6]     =  {0, -0.612, -0.5723,  0,  0,  0};
const double alpha[6] =  {M_PI / 2, 0, 0, M_PI / 2, -M_PI / 2, 0};
const double offset[6] = {0, -M_PI / 2, 0, -M_PI / 2, 0, 0};
#endif

#define PUAM_PARAMS
#ifdef PUAM_PARAMS
const double d[3]     =  {0.1273, 0, 0};
const double a[3]     =  {0, -0.612, -0.5723};
const double alpha[3] =  {M_PI / 2, 0, 0};
#endif

Matrix4d transform(int i, double theta)
{
	Matrix4d Ti;
	Ti << cos(theta),    -sin(theta) * cos(alpha[i]), sin(theta) * sin(alpha[i]), a[i] * cos(theta),
	      sin(theta),    cos(theta)  * cos(alpha[i]), -cos(theta)*sin(alpha[i]),  a[i] * sin(theta),
	      0,             sin(alpha[i]),               cos(alpha[i]),              d[i],
	      0,             0,                           0,                          1;
	return Ti;
}

Matrix4d forward_kinematics(Vector3d theta)
{
	Matrix4d T = Matrix4d::Identity();
	for (int i = 0; i < 3; i++)
	{
		Matrix4d Ti;
		Ti << cos(theta(i)),    -sin(theta(i)) * cos(alpha[i]), sin(theta(i)) * sin(alpha[i]), a[i] * cos(theta(i)),
		      sin(theta(i)),    cos(theta(i))  * cos(alpha[i]), -cos(theta(i))*sin(alpha[i]),  a[i] * sin(theta(i)),
		      0,                sin(alpha[i]),                  cos(alpha[i]),                 d[i],
		      0,                0,                              0,                             1;
		T = T * Ti;
		// cout << "Ti:\n" << Ti << endl;
	}

	return T;
}

vector<Vector3d> inverse_kinematics(Matrix4d T)
{
	Vector3d theta;
	Vector3d Oc;
	Oc(0) = T(0, 3);
	Oc(1) = T(1, 3);
	Oc(2) = T(2, 3);
	//1. solve Oc and determine theta(0 : 2)
	// cout << "Oc: " << Oc.transpose() << endl;
	//two solutions
	theta(0) = atan2(Oc(1), Oc(0));
	double r = Vector2d(Oc(0), Oc(1)).norm();
	double s = Oc(2) - d[0];
	double cos_theta2 = -(a[1] * a[1] + a[2] * a[2] - r * r - s * s ) / (2 * a[1] * a[2]);
	// printf("cos_theta2: %lf\n", cos_theta2);
	double cos_theta_tmp = (r * r + s * s + a[1] * a[1] - a[2] * a[2]) / ( 2 * Vector2d(r, s).norm() * a[1]);
	// printf("cos_theta_tmp: %lf\n", cos_theta_tmp);
	theta(2) = acos(cos_theta2);
	theta(1) = atan2(s, r) - atan2(a[2] * sin(theta(2)), a[1] + a[2] * cos(theta(2)));
	//theta(0) = atan2(Oc(1), Oc(0)) + M_PI;
	//2. determine R_03 from theta(0 : 2)
	//3. determine theta(3 : 5)
	vector<Vector3d> theta_q(8);
	theta_q[0](0) = theta(0);        theta_q[0](1) = theta(1);        theta_q[0](2) = theta(2); 
	theta_q[1](0) = theta(0);        theta_q[1](1) = theta(1);        theta_q[1](2) = theta(2) + M_PI; 
	theta_q[2](0) = theta(0);        theta_q[2](1) = theta(1) + M_PI; theta_q[2](2) = theta(2); 
	theta_q[3](0) = theta(0);        theta_q[3](1) = theta(1) + M_PI; theta_q[3](2) = theta(2) + M_PI; 
	theta_q[4](0) = theta(0) + M_PI; theta_q[4](1) = theta(1);        theta_q[4](2) = theta(2); 
	theta_q[5](0) = theta(0) + M_PI; theta_q[5](1) = theta(1);        theta_q[5](2) = theta(2) + M_PI; 
	theta_q[6](0) = theta(0) + M_PI; theta_q[6](1) = theta(1)+ M_PI;  theta_q[6](2) = theta(2); 
	theta_q[7](0) = theta(0) + M_PI; theta_q[7](1) = theta(1)+ M_PI;  theta_q[7](2) = theta(2) + M_PI; 

	for(int i = 0; i < theta_q.size(); i++)
	{
		for(int j = 0; j < theta_q[i].rows(); j++)
		{
			if(theta_q[i](j) >= 2 * M_PI)
			{
				theta_q[i](j) -= 2*M_PI;
			}
		}
	}

	return theta_q;
}

int main()
{

	srand(time(0));
	Vector3d theta;
	for (int i = 0; i < 3; i++)
	{
		// theta(i) = 0;//i * M_PI / 2;
		theta(i) = rand() % 100 / 100.0 * M_PI;
	}

	cout << "theta: " << theta.transpose() << endl;

	Matrix4d T = forward_kinematics(theta);
	cout << "T:\n" << T << endl;
	vector<Vector3d> theta_q = inverse_kinematics(T);
	for(int i = 0; i < theta_q.size(); i++)
	{
		cout << "theta_e: " << theta_q[i].transpose() << endl;
	}
	return 0;
}