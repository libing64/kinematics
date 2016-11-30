#ifndef __INVERSE_KINEMATICS_H
#define __INVERSE_KINEMATICS_H

#include <iostream>
#include <vector>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;

Matrix4d forward_kinematics(Vector4d);
vector<Vector6d>  inverse_kinematics(Matrix4d);

#endif