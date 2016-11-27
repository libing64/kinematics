#ifndef __INVERSE_KINEMATICS_H
#define __INVERSE_KINEMATICS_H

#include <iostream>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;

Matrix4d forward_kinematics(Vector6d);
Vector6d inverse_kinematics(Matrix4d);

#endif