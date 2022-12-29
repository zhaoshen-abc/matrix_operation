cmake_minimum_required(VERSION 3.2)
project(orb3_c)

add_definitions(-DAC_DEBUG)
add_definitions(-DLOG_AC)
include_directories(../include/matrix_operation)
add_library(matrix_operation SHARED
        matrix_operation/ac_auxiliary.c
        matrix_operation/ia_log.c
        matrix_operation/geometry.c
        matrix_operation/ac_mem_allocate.c
        matrix_operation/solver.c)


add_executable(SE3_test test/SE3_test.c)
target_link_libraries(SE3_test matrix_operation m)

add_executable(matrix_test test/matrix_test.c)
target_link_libraries(matrix_test matrix_operation m)




