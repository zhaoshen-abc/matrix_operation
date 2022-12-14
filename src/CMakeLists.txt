cmake_minimum_required(VERSION 3.24)
project(orb3_c)

include_directories(../include/matrix_operation)
add_library(matrix_operation SHARED
        matrix_operation/ac_auxiliary.c)
