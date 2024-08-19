cmake_minimum_required(VERSION 3.2)
project(orb3_c)

find_package(Eigen3 3.4.0 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
        # /home/zhaoshen/Mycode/utils/parse_dump/)

include_directories(../include/matrix_operation)
add_library(matrix_operation SHARED
        matrix_operation/geometry.cpp
        matrix_operation/solver.c)

add_library(matrix SHARED
        matrix_operation/Matrix.cc)

add_executable(SE3_test test/SE3_test.c)
target_link_libraries(SE3_test matrix_operation m)

add_executable(SVD_test test/SVD_test.cpp)
target_link_libraries(SVD_test matrix_operation m)

add_executable(SVD_QR_test test/SVD_QR_test.cpp)
target_link_libraries(SVD_QR_test matrix_operation m)




