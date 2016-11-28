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
const int JOINTS_NUM = 6;
const double d[6]     =  {0.1273, 0, 0, 0.163941, 0.1157, 0.0922};
const double a[6]     =  {0, -0.612, -0.5723,  0,  0,  0};
const double alpha[6] =  {M_PI / 2, 0, 0, M_PI / 2, -M_PI / 2, 0};
const double offset[6] = {0, -M_PI / 2, 0, -M_PI / 2, 0, 0};
#endif

// #define PUAM_PARAMS
#ifdef PUAM_PARAMS
const int JOINTS_NUM = 3;
const double d[3]     =  {0.1273, 0, 0};
const double a[3]     =  {0, 0.612, 0.5723};
const double alpha[3] =  {M_PI / 2, 0, 0};
#endif

#define SCARA_PARAMS
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


Matrix4d forward_kinematics(Vector4d theta)
{
	double d[4] = {0, 0, theta(2), d4};
	theta(2) = 0;
	Matrix4d T = Matrix4d::Identity();
	for (int i = 0; i < 4; i++)
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

vector<Vector4d> inverse_kinematics(Matrix4d T)
{
	Vector4d theta;
	Vector3d Oc;
	Oc(0) = T(0, 3);
	Oc(1) = T(1, 3);
	Oc(2) = T(2, 3);

	//two solutions
	double alpha = atan2(T(0, 1), T(0, 0) );
	theta(1) = acos( (Oc(0) * Oc(0) + Oc(1) * Oc(1) - a[0] * a[0] - a[1] * a[1]) / (2 * a[0] * a[1]));
	theta(0) = atan2(Oc(1), Oc(0)) - atan2(a[1] * sin(theta(1)), a[0] + a[1] * cos(theta(1)));
	theta(3) = theta(0) + theta(1) - alpha;
	if(theta(3) > 2*M_PI) 
		theta(3) -= 2 * M_PI;
	theta(2) = -(Oc(2) + d4);

	vector<Vector4d> theta_q(2);
	theta_q[0](0) = theta(0);        theta_q[0](1) = theta(1);        theta_q[0](2) = theta(2); theta_q[0](3) = theta(3);
	theta_q[1](0) = theta(0) + M_PI; theta_q[1](1) = theta(1);        theta_q[1](2) = theta(2); theta_q[1](3) = theta(3);
	
	return theta_q;
}

int main()
{

	srand(time(0));
	Vector4d theta;
	for (int i = 0; i < 4; i++)
	{
		// theta(i) = 0;//i * M_PI / 2;
		theta(i) = rand() % 100 / 100.0 * M_PI;
	}

	cout << "theta: " << theta.transpose() << endl;

	Matrix4d T = forward_kinematics(theta);
	cout << "T:\n" << T << endl;
	vector<Vector4d> theta_q = inverse_kinematics(T);
	for(int i = 0; i < theta_q.size(); i++)
	{
		cout << "theta_e: " << theta_q[i].transpose() << endl;
	}
	return 0;
}