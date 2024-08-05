cmake_minimum_required(VERSION 3.2)
project(orb3_c)

include_directories(../include/matrix_operation)
add_library(matrix_operation SHARED
        matrix_operation/geometry.c
        matrix_operation/solver.c)

add_library(matrix SHARED
        matrix_operation/Matrix.cc)
add_executable(SE3_test test/SE3_test.c)
target_link_libraries(SE3_test matrix_operation m)

# add_executable(matrix_test test/matrix_test.c)
# target_link_libraries(matrix_test matrix_operation m)




