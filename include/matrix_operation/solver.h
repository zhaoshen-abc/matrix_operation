#ifndef _SOLVER_H
#define _SOLVER_H

#include "ia_abstraction.h"
#include "ac_auxiliary.h"
#include "macro.h"

#ifdef __cplusplus
extern "C" {
#endif

// copy from https://blog.csdn.net/YU_Xianguo/article/details/84665547
int RtDRdecomp(float *A, int N);

int RtDRdecomp_D(double *A, int N);

// ex:
// H = {a1, a2, a3,
//      a2, a4, a5,
//      a3, a5, a6}
// _A = {a1, a2, a3, a4, a5, a6}: input and output
// _x = b = {b1, b2, b3}: input and output
// _bufferNxN = {empty}3x3
int ldlt_solve(float *_A, float *_x, int _N);

int ldlt_solve_D(double *_A, double *_x, int _N);

#ifdef __cplusplus
}
#endif

#endif //_SOLVER_H
