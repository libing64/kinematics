#include <iostream>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

double alpha[3] = {M_PI/2, M_PI/2, M_PI/2};

Matrix3d forward_kinematics(Vector3d theta);
Vector3d inverse_kinematics(Matrix3d R);

Matrix3d skew_symmetric(const Vector3d& w)
{
	Matrix3d R;
	R << 0,     -w(2), w(1),
	     w(2),   0,    -w(0),
	     -w(1),  w(0),  0;
    return R;
}

Matrix3d exponential_map(const Vector3d& w)
{
	double theta = w.norm();
	Vector3d u = w / theta;
	Matrix3d R = Matrix3d::Identity() 
				 + skew_symmetric(u) * sin(theta) 
				 + skew_symmetric(u) * skew_symmetric(u) * (1 - cos(theta));
	return R;
}

Matrix3d forward_kinematics(Vector3d theta)
{
	Matrix3d R = Matrix3d::Identity();
	for(int i = 0; i < 3; i++)
	{
		Matrix3d Mi;
		Mi <<   1,        0,               0,
	            0,        cos(alpha[i]),   -sin(alpha[i]),
	            0,        sin(alpha[i]),   cos(alpha[i]);

		Vector3d p;
		p << 0, 0, 1;


		Matrix3d Ri = Mi *  exponential_map(p * theta(i));

		R *= Ri;
	}
	return R;
}

int main()
{
	Vector3d theta;
	for(int i = 0; i < 3; i++)
	{
		theta(i) = rand() % 100 / 100.0;
	}
	Matrix3d R = forward_kinematics(theta);
	cout << "R: " << R << endl;
}