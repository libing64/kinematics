cmake_minimum_required(VERSION 2.8.3)
project(kinematics)

find_package(Eigen3 REQUIRED)
include_directories(
${EIGEN3_INCLUDE_DIR}
)
add_executable(ur_kin ur_kin.cpp)
add_executable(inverse_kinematics inverse_kinematics.cpp)

add_executable(poe_demo poe_demo.cpp)

