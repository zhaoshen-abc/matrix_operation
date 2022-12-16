cmake_minimum_required(VERSION 3.2)
project(orb3_c)

include_directories(../include/matrix_operation)
add_library(matrix_operation SHARED
        matrix_operation/ac_auxiliary.c)

add_executable(matrix_test test/matrix_test.c)
target_link_libraries(matrix_test matrix_operation m)
