#ifndef _SOLVER_H
#define _SOLVER_H

#include "ia_abstraction.h"
#include "ac_auxiliary.h"
#include "macro.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 *  \brief n x n dialog matrix.
 */
typedef struct
{
    double *data;       /*!< data of matrix*/
    uint32_t n;         /*!< number of col and row*/
    // uint32_t s;         /*!< number of stride*/
} symmetric_double_t;

/*!
 *  \brief n x n dialog matrix.
 */
typedef struct
{
    double *data;       /*!< data of matrix*/
    uint32_t n;         /*!< number of col and row*/
    // uint32_t s;         /*!< number of stride*/
} upper_triangular_double_t;

/*!
 *  \brief n x n dialog matrix.
 */
typedef struct
{
    double *data;       /*!< data of matrix*/
    uint32_t n;         /*!< number of col and row*/
    // uint32_t s;         /*!< number of stride*/
} zero_diag_triangular_double_t;

/*!
 *  \brief n x n dialog matrix.
 */
typedef struct
{
    double *data;       /*!< data of matrix*/
    uint32_t n;         /*!< number of col and row*/
    // uint32_t s;         /*!< number of stride*/
} unit_diag_triangular_double_t;

/*!
 *  \brief n x n dialog matrix.
 */
typedef struct
{
    double *data;       /*!< data of matrix*/
    uint32_t n;         /*!< number of col and row*/
    // uint32_t s;         /*!< number of stride*/
} diagonal_double_t;

double* symmetric_index(symmetric_double_t* A, int x, int y);

void symmetric_mat_print_d(symmetric_double_t* mat);

void upper_triangular_print_d(upper_triangular_double_t* mat);

void diagonal_mat_print_d(diagonal_double_t* mat);

// float* symmat2vector_N(const ac_matrix_f32_t* matrix);

double* symmat2vector_N_D(const ac_matrix_double_t* matrix, int N);

// copy from https://blog.csdn.net/YU_Xianguo/article/details/84665547
// int RtDRdecomp(float *A, int N);

int RtDRdecomp_D(double *A, int N);
// ia_err RtDRdecomp_D(upper_triangular_double_t* A);

ia_err Symmetric_RtDRdecomp(const symmetric_double_t* A, upper_triangular_double_t* R, diagonal_double_t* D);

// ex:
// H = {a1, a2, a3,
//      a2, a4, a5,
//      a3, a5, a6}
// _A = {a1, a2, a3, a4, a5, a6}: input and output
// _x = b = {b1, b2, b3}: input and output
// _bufferNxN = {empty}3x3
// int ldlt_solve(float *_A, float *_x, int _N);

int ldlt_solve_D(double *_A, double *_x, int _N);

#ifdef __cplusplus
}
#endif

#endif //_SOLVER_H
