#include "inverse_kinematics.h"
#include <iostream>
#include <vector>
#include <Eigen/Eigen>
#include <cstdlib>
#include <cstdio>

using namespace std;
using namespace Eigen;

#define UR10_PARAMS
#ifdef UR10_PARAMS
const double d1 =  0.1273;
const double a2 = -0.612;
const double a3 = -0.5723;
const double d4 =  0.163941;
const double d5 =  0.1157;
const double d6 =  0.0922;
#endif

// #define PUAM_PARAMS
#ifdef PUAM_PARAMS
const int JOINTS_NUM = 3;
const double d[3]     =  {0.1273, 0, 0};
const double a[3]     =  {0, 0.612, 0.5723};
const double alpha[3] =  {M_PI / 2, 0, 0};
#endif

// #define SCARA_PARAMS
#ifdef SCARA_PARAMS
const int JOINTS_NUM = 4;
const double d4 = 0.1234;
const double a[4]     =  {-0.612, -0.5723, 0, 0};
const double alpha[4] =  {0, M_PI, 0, 0};
#endif

#ifdef STANFORD_PARAMS
const int JOINTS_NUM = 6;
const double d[6]     =  {0, 0.1273, x, 0, 0, 0.163941};
const double a[6]     =  {0, 0, 0,  0,  0,  0};
const double alpha[6] =  {-M_PI / 2, M_PI/2, 0, -M_PI / 2, M_PI / 2, 0};
#endif	

Matrix4d transform(int i, double _theta)
{
	double d[6] = {d1, 0, 0, d4, d5, d6};
	double a[6] = {0, a2, a3, 0, 0, 0};
	double alpha[6] =  {M_PI / 2, 0, 0, M_PI / 2, -M_PI / 2, 0};
	Vector6d theta = Vector6d::Zero();
	theta(i)  = _theta;
	Matrix4d Ti;
	Ti << cos(theta(i)),    -sin(theta(i)) * cos(alpha[i]), sin(theta(i)) * sin(alpha[i]), a[i] * cos(theta(i)),
	      sin(theta(i)),    cos(theta(i))  * cos(alpha[i]), -cos(theta(i))*sin(alpha[i]),  a[i] * sin(theta(i)),
	      0,                sin(alpha[i]),                  cos(alpha[i]),                 d[i],
	      0,                0,                              0,                             1;

	return Ti;
}

Matrix4d forward_kinematics(Vector6d theta)
{
	double d[6] = {d1, 0, 0, d4, d5, d6};
	double a[6] = {0, a2, a3, 0, 0, 0};
	double alpha[6] =  {M_PI / 2, 0, 0, M_PI / 2, -M_PI / 2, 0};
	double offset[6] = {0, -M_PI / 2, 0, -M_PI / 2, 0, 0};
	Matrix4d T = Matrix4d::Identity();
	for (int i = 0; i < 4; i++)
	{
		Matrix4d Ti;
		Ti << cos(theta(i)),    -sin(theta(i)) * cos(alpha[i]), sin(theta(i)) * sin(alpha[i]), a[i] * cos(theta(i)),
		      sin(theta(i)),    cos(theta(i))  * cos(alpha[i]), -cos(theta(i))*sin(alpha[i]),  a[i] * sin(theta(i)),
		      0,                sin(alpha[i]),                  cos(alpha[i]),                 d[i],
		      0,                0,                              0,                             1;
		T = T * Ti;
	}
	return T;
}

vector<Vector6d> inverse_kinematics(const Matrix4d T)
{
	Vector6d theta;
	Vector3d t;
	t(0) = T(0, 3);
	t(1) = T(1, 3);
	t(2) = T(2, 3);
	Matrix3d R = T.block<3, 3>(0, 0);
	Vector3d p_05 = R * Vector3d(0, 0, -d6) + t;

	//two solutions
	double alpha1 = atan2( p_05(1), p_05(0) );
	double r = p_05.segment<2>(0).norm();
	double alpha2 = asin(d4 / r);
	theta(0) = alpha1 + alpha2;
	
	Matrix4d T_01 = transform(0, theta(0));
	cout << "T_01:" << T_01 << endl;
	Matrix4d T_16 = T_01.inverse() * T;
	cout << "T_16: " << T_16 << endl;
	cout << "(T_16(1, 3) - d4) / d6: " << (T_16(1, 3) - d4) / d6 << endl;
	//d4 + cos(theta5) * d6 = p_16_z
	theta(4) = acos( (T_16(2, 3) - d4) / d6 );
	theta(5) = atan2( T_16(1, 3), T_16(0, 3) );

	Matrix4d T_56 = transform(5, theta(5));
	Matrix4d T_45 = transform(4, theta(4));

	Matrix4d T_14 = T_01.inverse() * T * (T_45 * T_56).inverse();

	//solve theta(1), theta(2), theta(3) from T_14

	vector<Vector6d> theta_q(1);
	theta_q[0] = theta;
	return theta_q;
}

int main()
{

	srand(time(0));
	Vector6d theta;
	theta <<  2.04204,  1.19381,  1.31947,  1.25664, 0.251327, 0.345575;
	// for (int i = 0; i < 6; i++)
	// {
	// 	// theta(i) = 0;//i * M_PI / 2;
	// 	theta(i) = rand() % 100 / 100.0 * M_PI;
	// }

	cout << "theta: " << theta.transpose() << endl;

	Matrix4d T = forward_kinematics(theta);
	cout << "T:\n" << T << endl;
	vector<Vector6d> theta_q = inverse_kinematics(T);
	for(int i = 0; i < theta_q.size(); i++)
	{
		cout << "theta_e: " << theta_q[i].transpose() << endl;
	}
	return 0;
}