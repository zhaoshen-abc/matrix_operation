/*
 * INTEL CONFIDENTIAL
 *
 * Copyright (C) 2021-2022 Intel Corporation
 *
 * This software and the related documents are Intel copyrighted materials,
 * and your use of them is governed by the express license under which they
 * were provided to you ("License"). Unless the License provides otherwise,
 * you may not use, modify, copy, publish, distribute, disclose or transmit
 * this software or the related documents without Intel's prior written permission.
 * This software and the related documents are provided as is, with no express
 * or implied warranties, other than those that are expressly
 * stated in the License.
 */

#ifndef AC_AUXILIARY_H_
#define AC_AUXILIARY_H_

#include "ia_types.h"
#include "ia_abstraction.h"
#include "auto_calib_internal.h"
#if BITMATCH_PIPE
#include <stdio.h>
#include <string.h>
#endif
 /* Import several math consts.*/
#define _USE_MATH_DEFINES
#include "math.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_AUGMENTED_MATRIX (10U*4U)

/*!
 * \brief create vector.
 *
 * \param[in] len                   Mandatory.\n
 *                                  length of vector.
 * \param[out] vector               Mandatory.\n
 *                                  new vector.
 * \return                          Error code.
 */
ia_err create_vector(uint32_t len, ac_vector_t *vector);

/*!
 * \brief release 1D vector.
 *
 * \param[out] vector               Mandatory.\n
 *                                  released vector.
 * \return                          Error code.
 */
ia_err release_vector(ac_vector_t *vector);

/*!
 * \brief create 2D matrix.
 *
 * \param[in] m                     Mandatory.\n
 *                                  row num of matrix.
 * \param[in] n                     Mandatory.\n
 *                                  column num of matrix.
 * \param[out] mat                  Mandatory.\n
 *                                  new matrix.
 * \return                          Error code.
 */
ia_err create_matrix(uint32_t m, uint32_t n, ac_matrix_f32_t *mat);
ia_err create_matrix_u16(uint32_t m, uint32_t n, ac_matrix_u16_t* mat);
ia_err create_matrix_s16(uint32_t m, uint32_t n, ac_matrix_s16_t* mat);

/*!
 * \brief create large size 1D vector.
 *
 * \param[in] ac_mem_alloc_ptr      Mandatory.\n
 *                                  memory allocator for large size variables
 * \param[in] offset                Mandatory.\n
 *                                  predfined offset from the memory allocation tool
 * \param[in] m                     Mandatory.\n
 *                                  length of vector.
 * \param[out] vector               Mandatory.\n
 *                                  new vector.
 * \return                          Error code.
 */
ia_err create_large_vector(ac_mem_t *ac_mem_alloc_ptr, size_t offset, uint32_t m, ac_vector_t *vector);

/*!
 * \brief create large size 2D matrix.
 *
 * \param[in] ac_mem_alloc_ptr      Mandatory.\n
 *                                  memory allocator for large size variables
 * \param[in] offset                Mandatory.\n
 *                                  predfined offset from the memory allocation tool
 * \param[in] m                     Mandatory.\n
 *                                  row num of matrix.
 * \param[in] n                     Mandatory.\n
 *                                  column num of matrix.
 * \param[out] mat                  Mandatory.\n
 *                                  new matrix.
 * \return                          Error code.
 */
ia_err create_large_matrix(ac_mem_t *ac_mem_alloc_ptr, size_t offset, uint32_t m, uint32_t n, ac_matrix_f32_t *mat);
ia_err create_large_matrix_u8(ac_mem_t *ac_mem_alloc_ptr, size_t offset, uint32_t m, uint32_t n, ac_matrix_u8_t *mat);

ia_err create_large_matrix_u16(ac_mem_t *ac_mem_alloc_ptr, size_t offset, uint32_t m, uint32_t n, ac_matrix_u16_t *mat);
ia_err create_large_matrix_s16(ac_mem_t *ac_mem_alloc_ptr, size_t offset, uint32_t m, uint32_t n, ac_matrix_s16_t *mat);

/*!
 * \brief create large size 3D tensor.
 *
 * \param[in] ac_mem_alloc_ptr      Mandatory.\n
 *                                  memory allocator for large size variables
 * \param[in] offset                Mandatory.\n
 *                                  predfined offset from the memory allocation tool
 * \param[in] m                     Mandatory.\n
 *                                  1st demension of matrix.
 * \param[in] n                     Mandatory.\n
 *                                  2nd demension of matrix.
 * \param[in] k                     Mandatory.\n
 *                                  3rd demension of matrix.
 * \param[out] tensor               Mandatory.\n
 *                                  new tensor.
 * \return                          Error code.
 */
ia_err create_large_tensor(ac_mem_t *ac_mem_alloc_ptr, size_t offset, uint32_t m, uint32_t n, uint32_t k, ac_3_tensor_t *tensor);

/*!
 * \brief release 2D matrix.
 *
 * \param[in] mat                   Mandatory.\n
 *                                  released matrix.
 * \return                          Error code.
 */
ia_err release_matrix(ac_matrix_f32_t *mat);
ia_err release_matrix_u16(ac_matrix_u16_t* mat);
ia_err release_matrix_s16(ac_matrix_s16_t* mat);

/*!
 * \brief release large size 1D vector.
 *
 * \param[in] ac_mem_alloc_ptr      Mandatory.\n
 *                                  memory allocator for large size variables
 * \param[in] vector                Mandatory.\n
 *                                  released vector.
 * \return                          Error code.
 */
ia_err release_large_vector(ac_mem_t *ac_mem_alloc_ptr, ac_vector_t *vector);

/*!
 * \brief release large size 2D matrix.
 *
 * \param[in] ac_mem_alloc_ptr      Mandatory.\n
 *                                  memory allocator for large size variables
 * \param[in] mat                   Mandatory.\n
 *                                  released matrix.
 * \return                          Error code.
 */
ia_err release_large_matrix(ac_mem_t *ac_mem_alloc_ptr, ac_matrix_f32_t *mat);
ia_err release_large_matrix_u8(ac_mem_t *ac_mem_alloc_ptr, ac_matrix_u8_t *mat);

ia_err release_large_matrix_u16(ac_mem_t *ac_mem_alloc_ptr, ac_matrix_u16_t *mat);
ia_err release_large_matrix_s16(ac_mem_t *ac_mem_alloc_ptr, ac_matrix_s16_t *mat);

/*!
 * \brief release large size 3D tensor.
 *
 * \param[in] ac_mem_alloc_ptr      Mandatory.\n
 *                                  memory allocator for large size variables
 * \param[in] tensor                Mandatory.\n
 *                                  released tensor.
 * \return                          Error code.
 */
ia_err release_large_tensor(ac_mem_t *ac_mem_alloc_ptr, ac_3_tensor_t *tensor);

/*!
 * \brief create 3D tensor.
 *
 * \param[in] m                     Mandatory.\n
 *                                  1st-d num of tensor.
 * \param[in] n                     Mandatory.\n
 *                                  2nd-d num of tensor.
 * \param[in] k                     Mandatory.\n
 *                                  3rd-d num of tensor.
 * \param[out] tensor               Mandatory.\n
 *                                  new tensor.
 * \return                          Error code.
 */
ia_err create_tensor_3(uint32_t m, uint32_t n, uint32_t k, ac_3_tensor_t *tensor);

/*!
 * \brief release 3D tensor.
 *
 * \param[in] tensor                Mandatory.\n
 *                                  released tensor.
 * \return                          Error code.
 */
ia_err release_tensor_3(ac_3_tensor_t *tensor);

/*!
 * \brief print vector_3 for debug.
 *
 * \param[in] vec                   Mandatory.\n
 *                                  VEC_3.
 */
void ac_vector3_print(VEC_3 vec);
void ac_vector3_print_d(VEC_D_3 vec);

/*!
 * \brief print matrix for debug.
 *
 * \param[in] mat                   Mandatory.\n
 *                                  mat.
 * \param[in] m                     Mandatory.\n
 *                                  row number.
 * \param[in] n                     Mandatory.\n
 *                                  col number.
 */
void ac_mat_print(ac_matrix_f32_t* mat, uint32_t m, uint32_t n);
void ac_mat_print_d(ac_matrix_double_t* mat, uint32_t m, uint32_t n);

/*!
 * \brief print MAT_3_3 for debug.
 *
 * \param[in] mat                   Mandatory.\n
 *                                  MAT_3_3.
 */
void ac_MAT_3_3_print(MAT_3_3 mat);

/*!
 * \brief Rectangular grid in 1-D space.
 *
 * \param[in] x_start               Mandatory.\n
 *                                  vertical start point value.
 * \param[in] x_end                 Mandatory.\n
 *                                  vertical end point value.
 * \param[in] x_step                Mandatory.\n
 *                                  vertical step size.
 * \param[out] x_mat                Mandatory.\n
 *                                  matrix with vertial vector.
 * \param[in] y_start               Mandatory.\n
 *                                  horizontal start point value.
 * \param[in] y_end                 Mandatory.\n
 *                                  horizontal end point value.
 * \param[in] y_step                Mandatory.\n
 *                                  horizontal step size.
 * \param[out] y_mat               Mandatory.\n
 *                                  matrix with horizontal vector..
 * \return                          Error code.
 */
ia_err ndgrid_2D(uint32_t x_start, uint32_t x_end, uint32_t x_step, ac_matrix_f32_t *x_mat,
                 uint32_t y_start, uint32_t y_end, uint32_t y_step, ac_matrix_f32_t *y_mat);

/*!
 *  \brief padding type.
 */
typedef enum
{
    padding_pre = 0,
    padding_post,
    padding_both,
} ac_padding_type_t;

/*!
 * \brief matrix padding.
 *
 * \param[in] mat                   Mandatory.\n
 *                                  padding matrix.
 * \param[in] pad_height            Mandatory.\n
 *                                  pad_height.
 * \param[in] pad_width             Mandatory.\n
 *                                  pad_width.
 * \param[in] type                  Mandatory.\n
 *                                  padding type.
 * \return                          Error code.
 */
ia_err pad_array_2D(ac_matrix_f32_t *mat, uint32_t pad_height, uint32_t pad_width, ac_padding_type_t type);

/*!
 * \brief matrix repeat.
 *
 * \param[in] mat                   Mandatory.\n
 *                                  repeat matrix.
 * \param[in] rowfactor             Mandatory.\n
 *                                  duplicate factor of row.
 * \param[in] colfactor             Mandatory.\n
 *                                  duplicate factor of column.
 * \param[in] repmat                  Mandatory.\n
 *                                  product of vector.
 * \return                          Error code.
 */
ia_err mat_repmat(ac_matrix_f32_t *mat, uint32_t rowfactor, uint32_t colfactor,  ac_matrix_f32_t *repmat);

/*!
 * \brief Fit curve or surface to data by type Linear polynomial surface ('poly11').
 * Using following formula:
 * M = [sum(x.^2), sum(x.*y), sum(x);
 *     sum(x.*y), sum(y.^2), sum(y);
 *     sum(x), sum(y), length(x)];
 * B = [sum(x.*z); sum(y.*z); sum(z)];
 * p = M \ B; % solves M * p = B in a least squares sense
 * \param[in] x                     Mandatory.\n
 *                                  input matrix of x coordinates(N*1).
 * \param[in] y                     Mandatory.\n
 *                                  input matrix of y coordinates(N*1).
 * \param[in] z                     Mandatory.\n
 *                                  input matrix of z coordinates(N*1).
 * \param[out] rmse                 Mandatory.\n
 *                                  Root mean squared error (standard error).
 * \return                          Error code.
 */
ia_err fit_plane(ac_matrix_u16_t *x, ac_matrix_u16_t *y, ac_matrix_u16_t *z, double *rmse);

/*!
*   \brief sine with degree value
*/
double sind(double degree);

/*!
*   \brief cosine with degree value
*/
float32_t cosd(float32_t degree);

/*!
 * \brief matrix mean.
 *
 * \param[in] mat                   Mandatory.\n
 *                                  input matrix.
 * \param[out] mean                 Mandatory.\n
 *                                  mean value of matrix
 * \return                          Error code.
 */
ia_err mean_2D(ac_matrix_f32_t *mat, float32_t *mean);

/*!
 * \brief matrix mean.
 *
 * \param[in] mat                   Mandatory.\n
 *                                  input matrix.
 * \param[out] mean                 Mandatory.\n
 *                                  mean value of matrix
 * \return                          Error code.
 */
#define ac_mean(TYPE, data, len, mean)                         \
{                                                              \
    if (data == NULL || len == 0U)                             \
    {                                                          \
        return ia_err_argument;                                \
    }                                                          \
    uint32_t i = 0U;                                           \
    mean = 0.0F;                                               \
    TYPE *cur = (TYPE*)data;                                   \
    for (i = 0U; i < (uint32_t)len; i++) {                     \
            mean += (double)cur[i];                         \
    }                                                          \
    mean = mean / (double)len;                              \
}

/*!
 * \brief sum for data buffer
 *
 * \param[in] TYPE                  Mandatory.\n
 *                                  data type, e.g float32_t.
 * \param[in] data                  Mandatory.\n
 *                                  input data buffer.
 * \param[in] len                   Mandatory.\n
 *                                  input buffer length.
 * \param[out] sum                  Mandatory.\n
 *                                  sum value of data buffer
 */
#define ac_sum(TYPE, TYPE1, data, len, sum) {        \
    if (data == NULL) {                              \
        return ia_err_argument;                      \
    }                                                \
    TYPE *cur = (TYPE*)data;                         \
    uint32_t i = 0U;                                 \
    for (i = 0U; i < (uint32_t)len; i++)             \
    {                                                \
        sum += (TYPE1)cur[i];                        \
    }                                                \
}

ia_err max_u64(ac_matrix_s64_t* mat, int64_t* max);

/*!
 * \brief find max value in matrix.
 *
 * \param[in] mat                   Mandatory.\n
 *                                  input matrix.
 * \param[in] direction             Mandatory.\n
 *                                  max values vertically/horizontally/whole
 * \param[out] max                  Mandatory.\n
 *                                  max value of matrix
 * \param[out] min                  Mandatory.\n
 *                                  min value of matrix
 * \return                          Error code.
 */
ia_err max_min_2D(ac_matrix_f32_t *mat, ac_direction_t direction, ac_vector_t *max_value, ac_vector_t *min_value);

/*!
 * \brief matrix median.
 *
 * \param[in] TYPE                  Mandatory.\n
 *                                  data type, e.g float32_t.
 * \param[in] data                  Mandatory.\n
 *                                  input data buffer.
 * \param[in] len                   Mandatory.\n
 *                                  input buffer length.
 * \param[out] median               Mandatory.\n
 *                                  median value of matrix
 */
#define median_2D(TYPE, data, len, median, compare) {  \
    qsort((void*)data, len, sizeof(TYPE), compare);    \
    uint32_t idx = len / 2U;                           \
    TYPE *cur = (TYPE*)data;                           \
    if ((len % 2 == 0) && (idx > 0U)) {                \
        float32_t sum = (float32_t) cur[idx - 1] + (float32_t) cur[idx];            \
        median =(float32_t)sum / 2.0F;                 \
    }                                                  \
    else                                               \
    {                                                  \
        median = (float32_t)cur[idx];                  \
    }                                                  \
}

/*!
 * \brief vector Quaternion.
 *
 * \param[in] factor1               Mandatory.\n
 *                                  1st factor.
 * \param[in] factor2               Mandatory.\n
 *                                  2nd factor.
 * \param[out] product              Mandatory.\n
 *                                  product of matrix Quaternion
 * \return                          Error code.
 */
ia_err quaternion_product(const VEC_D_3 factor1, const VEC_D_3 factor2, VEC_D_3 product);

/*!
 * \brief vector sine (in degree).
 *
 * \param[in] src                   Mandatory.\n
 *                                  input vector.
 * \param[out] dst                  Mandatory.
 *                                  sine value of the input.
 * \param[in] len                   Mandatory.\n
 *                                  length of the input vector.
 */
ia_err vec_sind(VEC_D_3 src, VEC_D_3 dst, uint32_t len);

/*!
 * \brief vector and scalar operation (+,-,*,/).
 *
 * \param[in] factor1               Mandatory.\n
 *                                  vector operand.
 * \param[in] factor2               Mandatory.\n
 *                                  scalar operand
 * \param[in] operator             Mandatory.\n
 *                                  operater (+,-,*,/).
 * \param[out] product              Mandatory.\n
 *                                  result for output.
 */
ia_err vector_scaler_operation(const VEC_D_3 factor1, double factor2, char operator, VEC_D_3 product);

/*!
*   \brief tranpose matrix multiplication for ac_matrix (a' * b)
* \param[in] a                  Mandatory.\n
*                               left operand which will be transposed
                                during the multiplication.
* \param[in] b                  Mandatory.\n
*                               right operand of the multiplication.
* \param[in] trans_prod                Mandatory.\n
*                               the matrix product of matrix a^t and b.
*/
ia_err mat_transpose_times(ac_matrix_f32_t* a, ac_matrix_f32_t* b, ac_matrix_f32_t* trans_prod);
ia_err mat_transpose_times_d64(ac_matrix_double_t* a, ac_matrix_double_t* b, ac_matrix_double_t* trans_prod);

/*!
* \brief transpose for ac_matrix
* \param[in] a                  Mandatory.\n
*                               origin matrix.
* \param[in] a_t                Mandatory.\n
*                               transpose of the origin matrix.
*/
ia_err mat_transpose(const ac_matrix_f32_t *a, ac_matrix_f32_t *a_t);
ia_err mat_transpose_d64(const ac_matrix_double_t *a, ac_matrix_double_t* a_t);

/*!
* \brief inplace transpose for ac_matrix
* \param[in, out] a                  Mandatory.\n
*                               origin/output matrix.
* Transpose matrix with O(1) memory
* Find each swap cycles and swap clockwise */
ia_err mat_transpose_inplace(ac_matrix_f32_t* a);

/*!
* \brief inverse matrix with fixed size 3*3 matrix
* \param[in] mat                Mandatory.\n
*                               input matrix.
* \param[out] inv_mat           Mandatory.\n
*                               inverse matrix of the input matrix.
* \param[out] det               Optional.\n
*                               determinant of input matrix.
* \return                       Error code.
*/
ia_err inv_3_3(const ac_matrix_f32_t *mat, ac_matrix_f32_t *inv_mat, float32_t *det);
ia_err inv_3_3_d64(const ac_matrix_double_t* mat, ac_matrix_double_t* inv_mat, double *det);

/*!
 * \brief matrix std.
 * std = sqrt(sum((x-mean(x))^2)/(length(x)-1)) where x is the vectorized matrix.
 * \param[in] mat                   Mandatory.\n
 *                                  input matrix.
 * \param[out] std                  Mandatory.\n
 *                                  std value of matrix
 * \return                          Error code.
 */
ia_err std_2D(ac_matrix_f32_t *mat, float32_t *std);

/*!
 * \brief matrix to matrix operation including addition(+)/subtraction(-)/
 *     multiplication(.*)/matrix multiplication(*)/division(/).
 *
 * \param[in] factor1               Mandatory.\n
 *                                  1st factor.
 * \param[in] factor2               Mandatory.\n
 *                                  2nd factor.
 * \param[in] operator              Mandatory.\n
 *                                  operator.
 * \param[out] product              Mandatory.\n
 *                                  product of operation.
 * \return                          Error code.
 */
ia_err matrix_matrix_operation(ac_matrix_f32_t factor1, ac_matrix_f32_t factor2, char operator, ac_matrix_f32_t product);
ia_err matrix_matrix_operation_d(ac_matrix_double_t factor1, ac_matrix_double_t factor2, char operator, ac_matrix_double_t product);

/*!
 * \brief matrix to scaler operation including addition(+)/subtraction(-)/
 *     matrix multiplication(*)/division(/).
 *
 * \param[in] factor1               Mandatory.\n
 *                                  1st factor.
 * \param[in] factor2               Mandatory.\n
 *                                  2nd factor.
 * \param[in] operator              Mandatory.\n
 *                                  operator.
 * \param[out] product              Mandatory.\n
 *                                  result of operation.
 * \return                          Error code.
 */
ia_err matrix_scaler_operation(ac_matrix_f32_t factor1, float32_t factor2, char operator, ac_matrix_f32_t product);

/*!
 * \brief generate the matrix that diag with vector.
 *
 * \param[in] src               Mandatory.\n
 *                              vector as diag.
 * \param[out] dst              Mandatory.\n
 *                              matrix output.
 *                              result of operation.
 * \return                      Error code.
 */
ia_err diag_mat_3_3(const VEC_3 src, ac_matrix_f32_t dst);

/*!
 * \brief matrix to vector operation including addition(+)/subtraction(-)/
 *     matrix multiplication(*)/division(/).
 *
 * \param[in] factor1               Mandatory.\n
 *                                  pointer of factor1 (p*q matrix).
 * \param[in] factor2               Mandatory.\n
 *                                  pointer of factor2 (p*1 vector).
 * \param[in] operator              Mandatory.\n
 *                                  operator, addition(+)/subtraction(-)/
 *                                  matrix multiplication(*)/division(/).
 * \param[out] product              Mandatory.\n
 *                                  product of operation.
 * \return                          Error code.
 */
ia_err ac_bsxfun_operation(const ac_matrix_f32_t* factor1, const ac_matrix_f32_t* factor2, char operator, ac_matrix_f32_t* product);

/*!
 * \brief calculate the inverse matrix and det for multiple matrices
 *
 * \param[in] matrices              Mandatory.\n
 *                                  3*3*n tensor, containing n 3*3 matrices.
 * \param[out] inv_matrices         optional.\n
 *                                  3*3*n tensor, 3*3 inverse matrices for n input matrices.
 * \param[out] dets                 optional.\n
 *                                  n*1 vector, det for n matrices.
 * \return                          Error code.
 */
ia_err invert_det_matrices(const ac_3_tensor_t *matrices,ac_3_tensor_t *inv_matrices, ac_vector_t *dets);

/*!
 * \brief calculate the inverse matrix for n*n matrix
 *
 * \param[in] matrix                Mandatory.\n
 *                                  n*n matrix. (n <= 4)
 * \param[out] inv_matrices         optional.\n
 *                                  n*n matrix, n*n inverse matrices for the input matrices.
 * \param[out] det                  optional.\n
 *                                  float, det for n matrices.
 * \return                          Error code.
 */
ia_err invert_det_matrix(const ac_matrix_f32_t *matrix, ac_matrix_f32_t *inv_matrix, float32_t *det);

/*!
 * \brief calculate the inverse matrix for n*n matrix
 *
 * \param[in] matrix                Mandatory.\n
 *                                  n*n double matrix. (n <= 4)
 * \param[out] inv_matrix           optional.\n
 *                                  n*n double matrix, n*n inverse matrix for the input matrix.
 * \param[out] det                  optional.\n
 *                                  double, det for n matrix.
 * \return                          Error code.
 */
ia_err invert_det_matrix_d64(const ac_matrix_double_t *matrix, ac_matrix_double_t *inv_matrix, double *det);

/*!
 * \brief calculate the square sum of elements in matrix
 *
 * \param[in] mat                   Mandatory.\n
 *                                  m*n matrix.
 * \param[out] sq_sum               Mandatory.\n
 *                                  float, square sum of elements in matrix.
 * \return                          Error code.
 */
ia_err mat_square_sum(ac_matrix_f32_t* mat, float32_t* sum);

/*!
* \brief find the square sum for ac_matrix
* \param[in] TYPE               Mandatory.\n
*                               data type.
* \param[in] data               Mandatory.\n
*                               pointer of data buf.
* \param[in] len                Mandatory.\n
*                               length of data buf.
* \param[out] sum               Mandatory.\n
*                               square sum of the data buffer.
*/
#define ac_square_sum(TYPE, TYPE1, data, len, sum)     \
{                                                      \
    if (data == NULL)                                  \
    {                                                  \
        return ia_err_argument;                        \
    }                                                  \
    uint32_t i = 0U;                                   \
    TYPE *cur = (TYPE*)data;                           \
    for (i = 0U; i < (uint32_t)len; i++)               \
    {                                                  \
        sum += (TYPE1)(cur[i] * cur[i]);               \
    }                                                  \
}

/*!
* \brief cast matrix from double to float
* \param[in] src                Mandatory.\n
*                               double type matrix
* \param[out] dst                Mandatory.\n
*                               float type matrix.
*/
void ac_mat_3_3_cast(MAT_D_3_3 src, MAT_3_3 dst);

ia_err ac_mat_cast(ac_matrix_double_t src, ac_matrix_f32_t dst);

/*!
* \brief cast vector from double to float
* \param[in] src                Mandatory.\n
*                               double type vector
* \param[out] dst               Mandatory.\n
*                               float type vector.
*/
void ac_vec_3_cast(VEC_D_3 src, VEC_3 dst);

/*!
 * \brief construce the intrinsic_matrix based on the calib.
 *
 * \param[in] calib                 Mandatory.\n
 *                                  calib data.
 * \param[in] Kc                    Mandatory.\n
 *                                  intrinsic matrix.
 */
void construct_intrinsic_matrix(ac_calibration calib, MAT_D_3_3 kc);

/*!
 * \brief construce the inverse_intrinsic_matrix based on the calib.
 *
 * \param[in] calib                 Mandatory.\n
 *                                  calib data.
 * \param[in] Kc                    Mandatory.\n
 *                                  intrinsic matrix.
 */
void construct_inverse_intrinsic_matrix(ac_calibration calib, MAT_D_3_3 kc);

/*!
 * \brief convert Quaternion to Euler Angles (z-x'-y'' sequences, refer to 
 *        ia_imaging-ia_ipa/IPUAlgorithms/Pals/ia_pal/srcs/broxton_ia_pal/ia_pal_internal_gdc7_wfov_common.cpp:view_generate_rotation_matrix())
 * \param[in] q             Mandatory.\n
 *                          Quaternion.
 * \param[out] eulerangles  Mandatory.\n
 *                          Euler angles.
 */
ia_err quaternion2Eulerangles(VEC_D_3 q, VEC_D_3 eulerangles);

/*!
 * \brief convert vector to matrix.
 *
 * \param[in] vec                 Mandatory.\n
 *                                vector.
 * \param[in] mat                 Mandatory.\n
 *                                matrix.
 */
ia_err Rotation3D_Vector2Matrix(const VEC_D_3 vec, MAT_D_3_3 mat);
/*!
 * \brief convert matrix to inverse of vector.
 *
 * \param[in] mat                 Mandatory.\n
 *                                matrix.
 * \param[out] vec                 Mandatory.\n
 *                                vector.
 */
ia_err Rotation3D_Matrix2InvVector(const MAT_D_3_3 mat, VEC_D_3 vec);
/*!
 * \brief generate unique(without duplicate) random numbers in range [1, max], and store to list.
 *
 * \param[in] rng_seed_in         Mandatory.\n
 *                                input random seed.
 * \param[in] rand_max            Mandatory.\n
 *                                upper bound of random numbers.
 * \param[in] n_rands             Mandatory.\n
 *                                number of random numbers.
 * \param[out] rng_seed_out       Mandatory.\n
 *                                output random seed.
 * \param[out] list               Mandatory.\n
 *                                list contains n_rands random numbers.
 */
ia_err Random_seed(int32_t rng_seed_in, int32_t rand_max, int32_t n_rands, int32_t *rng_seed_out, uint32_t* list);

/*!
 * \brief interpolation method to x_target for the given y_target providing (x_L,y_L) and (x_R, y_R).
 *
 * \param[in] x_L                 Mandatory.\n
 *                                double, value of left x.
 * \param[in] x_R                 Mandatory.\n
 *                                double, value of right x.
 * \param[in] y_L                 Mandatory.\n
 *                                double, value of left y.
 * \param[in] y_R                 Mandatory. \n
 *                                double, value of right y.
 * \param[in] y_target            Mandatory. \n
 *                                double, value of target y.
 * \return                        double, value of x correspond to target y.
 */
double interpolatepoint_d(double x_L, double x_R, double y_L, double y_R, double y_target);

int32_t ac_compare_f32(const void* a, const void* b);
int32_t ac_compare_u16(const void* a, const void* b);


#ifdef FILE_INJECTION

typedef enum
{
    U8_TYPE = 0,    /*!< uint8_t*/
    S16_TYPE,       /*!< int16_t*/
    F32_TYPE,       /*!< float32_t*/
    D64_TYPE,       /*!< double64_t*/
    INVALID_TYPE,   /*!< invalid type*/
} ac_plain_type_t;

/*!
 *  \brief input data type.
 */
typedef enum
{
    calib_disparity_map = 0,          /*!< disparity map*/
    calib_disparity_confidence,       /*!< disparity confidence*/
    calib_me,                         /*!< motion estimation from BBME*/
    calib_mg_x,                       /*!< motion grid x from BBME*/
    calib_mg_y,                       /*!< motion grid y from BBME*/
    calib_indicator,                  /*!< indictor from MATLAB*/
    calib_init_calib_left,            /*!< init left calib data from MATLAB*/
    calib_init_calib_right,           /*!< init right calib data from MATLAB*/
    calib_prod_calib_left,            /*!< production left calib data from MATLAB*/
    calib_prod_calib_right,           /*!< production right calib data from MATLAB*/
} ac_file_type_t;

/*!
 * \brief read the static data files to fill the matrix as input.
 *
 * \param[in] mat                 Mandatory.\n
 *                                matrix to store data buffer.
 * \param[in] w                   Mandatory.\n
 *                                width of data
 * \param[in] h                   Mandatory.\n
 *                                height of data
 * \param[in] type                Mandatory. \n
 *                                data buffer type
 * \return                        Error code..
 */
ia_err fill_data_buffer_f32(ac_matrix_f32_t* mat, uint32_t n_mem, ac_file_type_t type);
ia_err fill_data_buffer_s16(ac_matrix_s16_t* mat, uint32_t n_mem, ac_file_type_t type);
ia_err fill_data_buffer_u8(ac_matrix_u8_t* mat, uint32_t n_mem, ac_file_type_t type);
ia_err fill_data_buffer_d64(ac_matrix_double_t* mat, uint32_t n_mem, ac_file_type_t type);
ia_err dump_data_buffer_f32(char* file_name, ac_matrix_f32_t* mat, uint32_t n_mem);

#endif // FILE_INJECTION

#if BITMATCH_PIPE
ia_err bitmatch_value_d(char *bin_file, double* test_value, char *message);
ia_err bitmatch_value(char *bin_file, float32_t* test_value, char *message);
ia_err bitmatch_matrix(char *bin_file, const ac_matrix_f32_t* test_mat, int principle_dim, char *message);
ia_err bitmatch_tensor_3(char *bin_file, const ac_3_tensor_t* test_mat, int principle_dim, char *message);
ia_err bitmatch_matrix_double(char *bin_file, const ac_matrix_double_t* test_mat, int principle_dim, char *message);
ia_err inject_binary_data(void *dst, size_t size, char *src_file);
ia_err export_matrix_data(char *bin_file, const ac_matrix_f32_t* mat);
#endif

#ifdef __cplusplus
}
#endif

#endif // AC_AUXILIARY_H_
