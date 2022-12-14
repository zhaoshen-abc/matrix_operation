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

#include "math.h"
#include "ia_abstraction.h"
#include "ac_auxiliary.h"

#ifdef ENABLE_NEON
#if defined __aarch64__
#include <arm_neon.h>
#endif
#endif

#define MAX_COL 20U

/* Declaration of static functions*/
/* Comment unused functions to avoid compile warning on Linux*/
#if (0)
static ia_err mat_linsolve(ac_matrix_f32_t* a, ac_matrix_f32_t* b, ac_matrix_f32_t* x);
static ia_err prod_vector(uint32_t len, ac_vector_t* vec, float32_t* prod);
#endif //0
static ia_err mat_mtimes(ac_matrix_f32_t* a, ac_matrix_f32_t* b, ac_matrix_f32_t* mprod);
static ia_err mat_mtimes_u64_f32(ac_matrix_double_t* a, ac_matrix_u64_t* b, ac_matrix_double_t* mprod);
static ia_err mat_add(ac_matrix_f32_t* a, ac_matrix_f32_t* b, ac_matrix_f32_t* sum);
static ia_err mat_subtraction(ac_matrix_f32_t* a, ac_matrix_f32_t* b, ac_matrix_f32_t* diff);
static ia_err mat_times(ac_matrix_f32_t* a, ac_matrix_f32_t* b, ac_matrix_f32_t* prod);
static ia_err mat_rdivide(ac_matrix_f32_t* a, ac_matrix_f32_t* b, ac_matrix_f32_t* quot);

static ia_err mat_mtimes_d(ac_matrix_double_t* a, ac_matrix_double_t* b, ac_matrix_double_t* mprod);
static ia_err mat_add_d(ac_matrix_double_t* a, ac_matrix_double_t* b, ac_matrix_double_t* sum);
static ia_err mat_subtraction_d(ac_matrix_double_t* a, ac_matrix_double_t* b, ac_matrix_double_t* diff);
static ia_err mat_times_d(ac_matrix_double_t* a, ac_matrix_double_t* b, ac_matrix_double_t* prod);
static ia_err mat_rdivide_d(ac_matrix_double_t* a, ac_matrix_double_t* b, ac_matrix_double_t* quot);

//Plug-in for debug, will remove on product version
#ifdef AC_DEBUG
void ac_vector3_print(VEC_3 vec)
{
    char buf[MAX_COL * 32U] = {0};
    char *ptr = buf;
    uint32_t i = 0U;
    for (i = 0U; i < 3U; i++) {
        float32_t data = vec[i];
        int size = sprintf(ptr, "%f, ", data);
        ptr += size;
    }
    AC_LOG("%s", buf);
}

void ac_vector3_print_d(VEC_D_3 vec)
{
    char buf[MAX_COL * 64U] = {0};
    char *ptr = buf;
    uint32_t i = 0U;
    for (i = 0U; i < 3U; i++) {
        double data = vec[i];
        int size = sprintf(ptr, "%f, ", data);
        ptr += size;
    }
    AC_LOG("%s", buf);
}

void ac_MAT_3_3_print(MAT_3_3 mat)
{
    uint32_t i = 0U;
    for (i = 0U; i < 3U; i++)
    {
        AC_LOG("%f, %f, %f", mat[i][0], mat[i][1], mat[i][2]);
    }
}

void ac_mat_print(ac_matrix_f32_t* mat, uint32_t m, uint32_t n)
{
    if (mat == NULL || n > MAX_COL) {
        return;
    }

    uint32_t row = IA_MIN(mat->m, m);
    uint32_t col = IA_MIN(mat->n, n);
    char buf[MAX_COL * 32U] = {0};
    char *ptr = buf;
    uint32_t i = 0U;
    uint32_t j = 0U;
    uint32_t idx = 0U;
    for (i = 0U; i < row; i++) {
        ptr = buf;
        for (j = 0U; j < col; j++) {
            idx = i * col + j;
            float32_t data = mat->data[idx];
            int size = sprintf(ptr, "%f, ", data);
            ptr += size;
        }
        AC_LOG("%s", buf);
    }
}

void ac_mat_print_d(ac_matrix_double_t* mat, uint32_t m, uint32_t n)
{
    if (mat == NULL || n > MAX_COL) {
        return;
    }

    uint32_t row = IA_MIN(mat->m, m);
    uint32_t col = IA_MIN(mat->n, n);
    char buf[MAX_COL * 64U] = {0};
    char *ptr = buf;
    uint32_t i = 0U;
    uint32_t j = 0U;
    uint32_t idx = 0U;
    for (i = 0U; i < row; i++) {
        ptr = buf;
        for (j = 0U; j < col; j++) {
            idx = i * col + j;
            double data = mat->data[idx];
            int size = sprintf(ptr, "%f, ", data);
            ptr += size;
        }
        AC_LOG("%s", buf);
    }
}

#else
void ac_vector3_print(VEC_3 vec)
{
    IA_UNUSED(vec);
}

void ac_vector3_print_d(VEC_D_3 vec)
{
    IA_UNUSED(vec);
}

void ac_MAT_3_3_print(MAT_3_3 mat)
{
    IA_UNUSED(mat);
}

void ac_mat_print(ac_matrix_f32_t* mat, uint32_t m, uint32_t n)
{
    IA_UNUSED(mat);
    IA_UNUSED(m);
    IA_UNUSED(n);
}

void ac_mat_print_d(ac_matrix_double_t* mat, uint32_t m, uint32_t n)
{
    IA_UNUSED(mat);
    IA_UNUSED(m);
    IA_UNUSED(n);
}

#endif

/* Public functions' definition*/

/*!
 * \brief create vector.
 *
 * \param[in] len                   Mandatory.\n
 *                                  length of vector.
 * \param[out] vector               Mandatory.\n
 *                                  new vector.
 * \return                          Error code.
 */
ia_err create_vector(uint32_t len, ac_vector_t* vector)
{
    if ((len == 0U) || (vector == NULL)) {
        return ia_err_argument;
    }
    vector->data = (float32_t*)IA_ALLOC(len * (uint32_t)sizeof(float32_t));
    vector->len = len;
    return ia_err_none;
}

/*!
 * \brief release 1D vector.
 *
 * \param[out] vector               Mandatory.\n
 *                                  released vector.
 * \return                          Error code.
 */
ia_err release_vector(ac_vector_t* vector)
{
    if ((vector == NULL) || (vector->data == NULL)) {
        return ia_err_argument;
    }
    IA_FREEZ(vector->data);
    return ia_err_none;
}

/*!
 * \brief create matrix with m row and n col.
 *
 */
ia_err create_matrix(uint32_t m, uint32_t n, ac_matrix_f32_t* mat)
{
    if ((mat == NULL) || (m == 0U) || (n == 0U)) {
        return ia_err_argument;
    }
    mat->data = (float32_t*)IA_ALLOC(m * n * (uint32_t)sizeof(float32_t));
    mat->m = m;
    mat->n = n;
    IA_MEMSET(mat->data, 0, m * n * (uint32_t)sizeof(float32_t));
    return ia_err_none;
}

ia_err create_matrix_u16(uint32_t m, uint32_t n, ac_matrix_u16_t* mat)
{
    if ((mat == NULL) || (m == 0U) || (n == 0U)) {
        return ia_err_argument;
    }
    mat->data = (uint16_t*)IA_ALLOC(m * n * (uint32_t)sizeof(uint16_t));
    mat->m = m;
    mat->n = n;
    IA_MEMSET(mat->data, 0, m * n * (uint32_t)sizeof(uint16_t));
    return ia_err_none;
}

ia_err create_matrix_s16(uint32_t m, uint32_t n, ac_matrix_s16_t* mat)
{
    if ((mat == NULL) || (m == 0U) || (n == 0U)) {
        return ia_err_argument;
    }
    mat->data = (int16_t*)IA_ALLOC(m * n * (uint32_t)sizeof(int16_t));
    mat->m = m;
    mat->n = n;
    IA_MEMSET(mat->data, 0, m * n * (uint32_t)sizeof(int16_t));
    return ia_err_none;
}

/*!
 * \brief create large vector with length m.
 *
 */
ia_err create_large_vector(ac_mem_t *ac_mem_alloc_ptr, size_t offset, uint32_t m, ac_vector_t *vector)
{
    if ((vector == NULL) || (m == 0U)) {
        return ia_err_argument;
    }
    ia_err ret = ac_mem_alloc_var(ac_mem_alloc_ptr, m * (uint32_t)sizeof(float32_t), (void **)&(vector->data), offset);
    if (ret != ia_err_none) {
        AC_LOG("Error happen for large size variable allocation.");
        return ret;
    }
    vector->len = m;
    IA_MEMSET(vector->data, 0, m * (uint32_t)sizeof(float32_t));
    return ia_err_none;
}

/*!
 * \brief create large matrix with m row and n col.
 *
 */
ia_err create_large_matrix(ac_mem_t *ac_mem_alloc_ptr, size_t offset, uint32_t m, uint32_t n, ac_matrix_f32_t *mat)
{
    if ((mat == NULL) || (m == 0U) || (n == 0U)) {
        return ia_err_argument;
    }
    ia_err ret = ac_mem_alloc_var(ac_mem_alloc_ptr, m * n * (uint32_t)sizeof(float32_t), (void **)&(mat->data), offset);
    if (ret != ia_err_none) {
        AC_LOG("Error happen for large size variable allocation.");
        return ret;
    }
    mat->m = m;
    mat->n = n;
    IA_MEMSET(mat->data, 0, m * n * (uint32_t)sizeof(float32_t));
    return ia_err_none;
}

ia_err create_large_matrix_u8(ac_mem_t *ac_mem_alloc_ptr, size_t offset, uint32_t m, uint32_t n, ac_matrix_u8_t *mat)
{
    if ((mat == NULL) || (m == 0U) || (n == 0U)) {
        return ia_err_argument;
    }
    ia_err ret = ac_mem_alloc_var(ac_mem_alloc_ptr, m * n * (uint32_t)sizeof(uint8_t), (void **)&(mat->data), offset);
    if (ret != ia_err_none) {
        AC_LOG("Error happen for large size variable allocation.");
        return ret;
    }
    mat->m = m;
    mat->n = n;
    IA_MEMSET(mat->data, 0, m * n * (uint32_t)sizeof(uint8_t));
    return ia_err_none;
}

ia_err create_large_matrix_u16(ac_mem_t *ac_mem_alloc_ptr, size_t offset, uint32_t m, uint32_t n, ac_matrix_u16_t *mat)
{
    if ((mat == NULL) || (m == 0U) || (n == 0U)) {
        return ia_err_argument;
    }
    ia_err ret = ac_mem_alloc_var(ac_mem_alloc_ptr, m * n * (uint32_t)sizeof(uint16_t), (void **)&(mat->data), offset);
    if (ret != ia_err_none) {
        AC_LOG("Error happen for large size variable allocation.");
        return ret;
    }
    mat->m = m;
    mat->n = n;
    IA_MEMSET(mat->data, 0, m * n * (uint32_t)sizeof(uint16_t));
    return ia_err_none;
}

ia_err create_large_matrix_s16(ac_mem_t *ac_mem_alloc_ptr, size_t offset, uint32_t m, uint32_t n, ac_matrix_s16_t *mat)
{
    if ((mat == NULL) || (m == 0U) || (n == 0U)) {
        return ia_err_argument;
    }
    ia_err ret = ac_mem_alloc_var(ac_mem_alloc_ptr, m * n * (uint32_t)sizeof(uint16_t), (void **)&(mat->data), offset);
    if (ret != ia_err_none) {
        AC_LOG("Error happen for large size variable allocation.");
        return ret;
    }
    mat->m = m;
    mat->n = n;
    IA_MEMSET(mat->data, 0, m * n * (uint32_t)sizeof(int16_t));
    return ia_err_none;
}

/*!
 * \brief create large 3D tensor with shape (m,n,k).
 *
 */
ia_err create_large_tensor(ac_mem_t *ac_mem_alloc_ptr, size_t offset, uint32_t m, uint32_t n, uint32_t k, ac_3_tensor_t *tensor)
{
    if ((tensor == NULL) || (m == 0U) || (n == 0U) || (k == 0U)) {
        return ia_err_argument;
    }
    ia_err ret = ac_mem_alloc_var(ac_mem_alloc_ptr, m * n * k * (uint32_t)sizeof(float32_t), (void **)&(tensor->data), offset);
    if (ret != ia_err_none) {
        AC_LOG("Error happen for large size variable allocation.");
        return ret;
    }
    tensor->m = m;
    tensor->n = n;
    tensor->k = k;
    IA_MEMSET(tensor->data, 0, m * n * k * (uint32_t)sizeof(float32_t));
    return ia_err_none;
}

/*!
 * \brief release 2D matrix.
 *
 */
ia_err release_matrix(ac_matrix_f32_t* mat)
{
    if ((mat == NULL) || (mat->data == NULL)) {
        return ia_err_argument;
    }
    IA_FREEZ(mat->data);
    return ia_err_none;
}

ia_err release_matrix_u16(ac_matrix_u16_t* mat)
{
    if ((mat == NULL) || (mat->data == NULL)) {
        return ia_err_argument;
    }
    IA_FREEZ(mat->data);
    return ia_err_none;
}

ia_err release_matrix_s16(ac_matrix_s16_t* mat)
{
    if ((mat == NULL) || (mat->data == NULL)) {
        return ia_err_argument;
    }
    IA_FREEZ(mat->data);
    return ia_err_none;
}

/*!
 * \brief release large 1D tensor.
 *
 */
ia_err release_large_vector(ac_mem_t *ac_mem_alloc_ptr, ac_vector_t *vector)
{
    if ((vector == NULL) || (vector->data == NULL)) {
        return ia_err_argument;
    }
    ia_err ret = ac_mem_free_var(ac_mem_alloc_ptr, (void **)&(vector->data));
    if (ret != ia_err_none) {
        return ret;
    }
    vector->len = 0U;
    return ia_err_none;
}

/*!
 * \brief release large 2D matrix.
 *
 */
ia_err release_large_matrix(ac_mem_t *ac_mem_alloc_ptr, ac_matrix_f32_t *mat)
{
    if ((mat == NULL) || (mat->data == NULL)) {
        return ia_err_argument;
    }
    ia_err ret = ac_mem_free_var(ac_mem_alloc_ptr, (void **)&(mat->data));
    if (ret != ia_err_none) {
        return ret;
    }
    mat->m = 0U;
    mat->n = 0U;
    return ia_err_none;
}

ia_err release_large_matrix_u8(ac_mem_t *ac_mem_alloc_ptr, ac_matrix_u8_t *mat)
{
    if ((mat == NULL) || (mat->data == NULL)) {
        return ia_err_argument;
    }
    ia_err ret = ac_mem_free_var(ac_mem_alloc_ptr, (void **)&(mat->data));
    if (ret != ia_err_none) {
        return ret;
    }
    mat->m = 0U;
    mat->n = 0U;
    return ia_err_none;
}

ia_err release_large_matrix_u16(ac_mem_t *ac_mem_alloc_ptr, ac_matrix_u16_t *mat)
{
    if ((mat == NULL) || (mat->data == NULL)) {
        return ia_err_argument;
    }
    ia_err ret = ac_mem_free_var(ac_mem_alloc_ptr, (void **)&(mat->data));
    if (ret != ia_err_none) {
        return ret;
    }
    mat->m = 0U;
    mat->n = 0U;
    return ia_err_none;
}

ia_err release_large_matrix_s16(ac_mem_t *ac_mem_alloc_ptr, ac_matrix_s16_t *mat)
{
    if ((mat == NULL) || (mat->data == NULL)) {
        return ia_err_argument;
    }
    ia_err ret = ac_mem_free_var(ac_mem_alloc_ptr, (void **)&(mat->data));
    if (ret != ia_err_none) {
        return ret;
    }
    mat->m = 0U;
    mat->n = 0U;
    return ia_err_none;
}


/*!
 * \brief release large 3D tensor.
 *
 */
ia_err release_large_tensor(ac_mem_t *ac_mem_alloc_ptr, ac_3_tensor_t *tensor)
{
    if ((tensor == NULL) || (tensor->data == NULL)) {
        return ia_err_argument;
    }
    ia_err ret = ac_mem_free_var(ac_mem_alloc_ptr, (void **)&(tensor->data));
    if (ret != ia_err_none) {
        return ret;
    }
    tensor->m = 0U;
    tensor->n = 0U;
    tensor->k = 0U;
    return ia_err_none;
}

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
ia_err create_tensor_3(uint32_t m, uint32_t n, uint32_t k, ac_3_tensor_t* tensor)
{
    if ((tensor == NULL) || (m == 0U) || (n == 0U) || (k == 0U)) {
        return ia_err_argument;
    }
    tensor->data = (float32_t*)IA_ALLOC(m * n * k * (uint32_t)sizeof(float32_t));
    tensor->m = m;
    tensor->n = n;
    tensor->k = k;
    IA_MEMSET(tensor->data, 0, m * n * k * (uint32_t)sizeof(float32_t));
    return ia_err_none;
}

/*!
 * \brief release 3D tensor.
 *
 * \param[in] tensor                Mandatory.\n
 *                                  released tensor.
 * \return                          Error code.
 */
ia_err release_tensor_3(ac_3_tensor_t* tensor)
{
    if ((tensor == NULL) || (tensor->data == NULL)) {
        return ia_err_argument;
    }
    IA_FREEZ(tensor->data);
    return ia_err_none;
}


/*!
 * \brief Rectangular grid in 2-D space.
 *
 */
ia_err ndgrid_2D(uint32_t x_start, uint32_t x_end, uint32_t x_step, ac_matrix_f32_t* x_mat,
    uint32_t y_start, uint32_t y_end, uint32_t y_step, ac_matrix_f32_t* y_mat)
{
    if ((x_mat == NULL) || (x_mat->data == NULL) ||
        (y_mat == NULL) || (y_mat->data == NULL) ||
        (x_start >= x_end) || (y_start >= y_end) ||
        (x_step == 0U) || (y_step == 0U)) {
        return ia_err_argument;
    }

    uint32_t m = ((x_end - x_start) / x_step) + 1U;
    uint32_t n = ((y_end - y_start) / y_step) + 1U;
    if ((m > (uint32_t)x_mat->m) || (m > (uint32_t)y_mat->m) ||
        (n > (uint32_t)x_mat->n) || (n > (uint32_t)y_mat->n)) {
        return ia_err_argument;
    }
    //update row/col
    x_mat->m = m;
    x_mat->n = n;
    y_mat->m = m;
    y_mat->n = n;

    uint32_t i = 0U;
    uint32_t j = 0U;
    for (i = 0U; i < m; i++) {
        float32_t value = (float32_t)x_start + ((float32_t)i * (float32_t)x_step);
        for (j = 0U; j < n; j++) {
            x_mat->data[(i * n) + j] = value;
        }
    }

    for (i = 0U; i < n; i++) {
        float32_t value = (float32_t)y_start + ((float32_t)i * (float32_t)y_step);
        for (j = 0U; j < m; j++) {
            y_mat->data[(i * m) + j] = value;
        }
    }

    return ia_err_none;
}

/*!
*   \brief sine with degree value
*/
double sind(double degree)
{
    return sin(degree / 180.0f * (double)M_PI);
}

/*!
*   \brief cosine with degree value
*/
float32_t cosd(float32_t degree)
{
    float32_t rad = (float32_t)degree / 180.0f * (float32_t)M_PI;
    return (float32_t)cos(rad);
}

/*!
 * \brief matrix mean.
 *
 * \param[in] mat                   Mandatory.\n
 *                                  input matrix.
 * \param[out] mean                 Mandatory.\n
 *                                  mean value of matrix
 * \return                          Error code.
 */
ia_err mean_2D(ac_matrix_f32_t* mat, float32_t* mean)
{
    if (mat == NULL || mean == NULL)
    {
        return ia_err_argument;
    }
    uint32_t i = 0U;
    (*mean) = 0.0F;
    for (i = 0U; i < (mat->m * mat->n); i++) {
            (*mean) += mat->data[i];
    }
    float32_t sum = (float32_t)mat->m * (float32_t)mat->n;
    (*mean) = (*mean) / sum;
    return ia_err_none;
}

/*!
 * \brief matrix to matrix operation including addition(+)/subtraction(-)/
 *     element multiplication(.)/matrix multiplication(*)/division(/).
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
ia_err matrix_matrix_operation(ac_matrix_f32_t factor1, ac_matrix_f32_t factor2, char operator, ac_matrix_f32_t product)
{
    ia_err ret;
    switch (operator)
    {
    case '+':
        ret = mat_add(&factor1, &factor2, &product);
        break;
    case '-':
        ret = mat_subtraction(&factor1, &factor2, &product);
        break;
    case '*':
        ret = mat_mtimes(&factor1, &factor2, &product);
        break;
    case '.':
        ret = mat_times(&factor1, &factor2, &product);
        break;
    case '/':
        ret = mat_rdivide(&factor1, &factor2, &product);
        break;
    default:
        ret = ia_err_argument;
        break;
    }
    return ret;
}

ia_err matrix_matrix_operation_d(ac_matrix_double_t factor1, ac_matrix_double_t factor2, char operator, ac_matrix_double_t product)
{
    ia_err ret;
    switch (operator)
    {
    case '+':
        ret = mat_add_d(&factor1, &factor2, &product);
        break;
    case '-':
        ret = mat_subtraction_d(&factor1, &factor2, &product);
        break;
    case '*':
        ret = mat_mtimes_d(&factor1, &factor2, &product);
        break;
    case '.':
        ret = mat_times_d(&factor1, &factor2, &product);
        break;
    case '/':
        ret = mat_rdivide_d(&factor1, &factor2, &product);
        break;
    default:
        ret = ia_err_argument;
        break;
    }
    return ret;
}

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
 *                                  product of operation.
 * \return                          Error code.
 */
ia_err matrix_scaler_operation(ac_matrix_f32_t factor1, float32_t factor2, char operator, ac_matrix_f32_t product)
{
    ia_err ret = ia_err_none;
    uint32_t i = 0U;
    if (factor1.data == NULL)
    {
        return ia_err_data;
    }
    uint32_t numel = factor1.m * factor1.n;
    switch (operator)
    {
    case '+':
        for (i = 0U; i < numel; i++)
        {
            product.data[i] = factor1.data[i] + factor2;
        }
        break;
    case '-':
        for (i = 0U; i < numel; i++)
        {
            product.data[i] = factor1.data[i] - factor2;
        }
        break;
    case '*':
        for (i = 0U; i < numel; i++)
        {
            product.data[i] = factor1.data[i] * factor2;
        }
        break;
    case '/':
        if (factor2 < IA_EPSILON)
        {
            ret = ia_err_data;
            break;
        }
        for (i = 0U; i < numel; i++)
        {
            product.data[i] = factor1.data[i] / factor2;
        }
        break;
    default:
        ret = ia_err_argument;
        break;
    }
    return ret;
}

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
ia_err diag_mat_3_3(const VEC_3 src, ac_matrix_f32_t dst)
{
    if (dst.m != 3U || dst.n != 3U) {
        return ia_err_argument;
    }
    int32_t i = 0;
    int32_t j = 0;
    int32_t idx = 0;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            idx = i * 3 + j;
            dst.data[idx] = 0.0f;
            if (i == j) {
                dst.data[idx] = src[i];
            }
        }
    }
    return ia_err_none;
}

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
ia_err fit_plane(ac_matrix_u16_t *x, ac_matrix_u16_t *y, ac_matrix_u16_t *z, double *rmse)
{
    if (x == NULL || y == NULL || z == NULL || rmse == NULL ||
        x->m != y->m || x->m != z->m || x->n != 1 || y->n != 1 ||
        z->n != 1)
    {
        return ia_err_argument;
    }

    ia_err ret = ia_err_none;
    uint64_t m_data[9] = { 0 };
    double m_data_d[9] = { 0.0 };
    double inv_m_data[9] = { 0.0f };
    uint64_t b_data[3] = { 0 };
    double p_data[3] = { 0.0f };
    ac_matrix_u64_t m = { (uint64_t*)NULL, 3U, 3U, 3U };
    ac_matrix_double_t m_d = { (double*)NULL, 3U, 3U, 3U };
    ac_matrix_double_t inv_m = { (double*)NULL, 3U, 3U, 3U };
    ac_matrix_u64_t b = { (uint64_t*)NULL, 3U, 1U, 1U };
    ac_matrix_double_t p = { (double*)NULL, 3U, 1U, 1U };
    m.data = m_data;
    m_d.data = m_data_d;
    inv_m.data = inv_m_data;
    b.data = b_data;
    p.data = p_data;
    uint32_t i = 0U;
    /* Calculate M(symmetric) element-wise*/
    /* M[0,0] */
    uint32_t len = x->m * x->n;
    ac_square_sum(uint16_t, uint64_t, x->data, len, m_data[0]);
    /* M[0,1] and M[1,0] */
    uint64_t sum = 0U;
    for (i = 0U; i < x->m; i++)
    {
        sum += (uint64_t)(x->data[i] * y->data[i]);
    }
    m_data[1] = sum;
    m_data[3] = sum;
    /* M[0,2] and M[2,0] */
    ac_sum(uint16_t, uint64_t, x->data, len, m_data[2]);
    m_data[6] = m_data[2];
    /* M[1,1]*/
    ac_square_sum(uint16_t, uint64_t, y->data, len, m_data[4]);
    /* M[1,2] and M[2,1] */
    ac_sum(uint16_t, uint64_t, y->data, len, m_data[5]);
    m_data[7] = m_data[5];
    /* M[2,2]*/
    m_data[8] = x->m;

    /* Calculate B element-wise*/
    /* B[0] */
    sum = 0;
    for (i = 0U; i < x->m; i++)
    {
        sum += (uint64_t)(x->data[i] * z->data[i]);
    }
    b_data[0] = sum;
    /* B[1] */
    sum = 0;
    for (i = 0U; i < x->m; i++)
    {
        sum += (uint64_t)(y->data[i] * z->data[i]);
    }
    b_data[1] = sum;
    /* B[2] */
    len = z->m * z->n;
    ac_sum(uint16_t, uint64_t, z->data, len, b_data[2]);
    /* Calculate inv_M */
    for (i = 0U; i < 9U; i++) {
        /* orginal matrix "m_d" must be converted to double.
         * other overflow happen in calculating the uint64 matrix inversion
         */
        m_data_d[i] = (double)m.data[i] / 1000.0F;
    }
    ret = inv_3_3_d64(&m_d, &inv_m, NULL);
    //ret = inv_3_3(&m, &inv_m, NULL);
    if (ret != ia_err_none)
    {
        return ret;
    }

    for (i = 0U; i < 9U; i++) {
        inv_m_data[i] = inv_m_data[i] / 1000.0F;
    }

    /* Calculate p = inv_M * B */
    ret = mat_mtimes_u64_f32(&inv_m, &b, &p);
    if (ret != ia_err_none)
    {
        return ret;
    }

    /* Calculate rmse*/
    (*rmse) = 0.0f;
    double z_est = 0.0f;
    for (i = 0U; i < x->m; i++) {
        z_est = (p.data[0] * x->data[i]) + (p.data[1] * y->data[i]) + p.data[2];
        (*rmse) += (z_est - z->data[i]) * (z_est - z->data[i]);
    }
    (*rmse) = IA_SQRTD((*rmse) / (double)(z->m));

    return ia_err_none;
}

/*!
 * \brief matrix std.
 * std = sqrt(sum((x-mean(x))^2)/(length(x)-1)) where x is the vectorized matrix.
 * \param[in] mat                   Mandatory.\n
 *                                  input matrix.
 * \param[out] std                  Mandatory.\n
 *                                  std value of matrix
 * \return                          Error code.
 */
ia_err std_2D(ac_matrix_f32_t *mat, float32_t *std)
{
    if (mat == NULL || std == NULL || mat->m == 0 || mat->n == 0)
    {
        return ia_err_argument;
    }
    ia_err ret;
    float32_t mean = 0.0f;
    ret = mean_2D(mat, &mean);
    if (ret != ia_err_none) {
        return ret;
    }
    uint32_t i = 0U;
    (*std) = 0;
    for (i = 0U; i < (mat->m * mat->n); i++) {
        (*std) += (mat->data[i] - mean) * (mat->data[i] - mean);
    }
    uint32_t size = mat->m * mat->n;

    if (size != 1U) {
        (*std) = IA_SQRT((*std) / ((float32_t)size - 1.0F));
    }
    return ia_err_none;
}


/* Part II Static Functions */
/* +, - , *, .*, ./, \, etc. */

/*!
* \brief det of matrix with fixed size 2*2
* det([a b; c d]) = a * d - b * c
*/
static inline float32_t det_2_2(float32_t a, float32_t b, float32_t c, float32_t d)
{
    return ((a * d) - (b * c));
}

static inline int64_t det_2_2_i64(int64_t a, int64_t b, int64_t c, int64_t d)
{
    return ((a * d) - (b * c));
}


static inline double det_2_2_d64(double a, double b, double c, double d)
{
    return ((a * d) - (b * c));
}

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
ia_err mat_transpose_times(ac_matrix_f32_t* a, ac_matrix_f32_t* b, ac_matrix_f32_t* trans_prod)
{
    if (a == NULL || b == NULL || trans_prod == NULL ||
        a->data == NULL || b->data == NULL || trans_prod->data == NULL ||
        a->m != b->m || a->n != trans_prod->m || b->n != trans_prod->n)
    {
        return ia_err_argument;
    }

#ifdef ENABLE_NEON
#if defined __aarch64__
    // optimize 3x2' * 3x1 matrix operation with NEON
    if ((a->n == 2) && (a->m == 3) && (b->n == 1))
    {
        //transpose
        float32_t a_tmp[8] = {a->data[0], a->data[2], a->data[4], 0.0F, a->data[1], a->data[3], a->data[5], 0.0F};
        float32x2x4_t vec1;
        vec1 = vld4_f32(a_tmp);
        float32_t b_tmp[8] = {b->data[0], b->data[1], b->data[2], 0.0F};
        float32x4_t vec2;
        vec2 = vld1q_f32(b_tmp);
        float32x2_t result = vdup_n_f32(0);;
        result = vmla_laneq_f32(result, vec1.val[0], vec2, 0);
        result = vmla_laneq_f32(result, vec1.val[1], vec2, 1);
        result = vmla_laneq_f32(result, vec1.val[2], vec2, 2);
        vst1_f32(trans_prod->data, result);
        return ia_err_none;
    }
    // optimize 3x2' * 3x2 matrix operation with NEON
    if ((a->n == 2) && (a->m == 3) && (b->n == 2))
    {
        //transpose
        float32_t a_tmp[8] = {a->data[0], a->data[2], a->data[4], 0.0F, a->data[1], a->data[3], a->data[5], 0.0F};
        float32x2x4_t vec1;
        vec1 = vld4_f32(a_tmp);
        float32_t b0_tmp[8] = {b->data[0], b->data[2], b->data[4], 0.0F};
        float32_t b1_tmp[8] = {b->data[1], b->data[3], b->data[5], 0.0F};
        float32x4_t vec2;
        vec2 = vld1q_f32(b0_tmp);
        float32x2_t result = vdup_n_f32(0);
        result = vmla_laneq_f32(result, vec1.val[0], vec2, 0);
        result = vmla_laneq_f32(result, vec1.val[1], vec2, 1);
        result = vmla_laneq_f32(result, vec1.val[2], vec2, 2);
        vst1_f32(trans_prod->data, result);

        vec2 = vld1q_f32(b1_tmp);
        result = vdup_n_f32(0);
        result = vmla_laneq_f32(result, vec1.val[0], vec2, 0);
        result = vmla_laneq_f32(result, vec1.val[1], vec2, 1);
        result = vmla_laneq_f32(result, vec1.val[2], vec2, 2);
        vst1_f32(trans_prod->data+2, result);
        return ia_err_none;
    }
#endif
#endif

    uint32_t i = 0U;
    uint32_t j = 0U;
    uint32_t k = 0U;
    for (i = 0U; i < a->n; i++) {
        for (j = 0U; j < b->n; j++) {
            trans_prod->data[(i * b->n) + j] = 0.0f;
            for (k = 0U; k < a->m; k++) {
                trans_prod->data[(i * b->n) + j] += a->data[(k * a->n) + i] * b->data[(k * b->n) + j];
            }
        }
    }
    return ia_err_none;
}

ia_err mat_transpose_times_d64(ac_matrix_double_t* a, ac_matrix_double_t* b, ac_matrix_double_t* trans_prod)
{
    if (a == NULL || b == NULL || trans_prod == NULL ||
        a->data == NULL || b->data == NULL || trans_prod->data == NULL ||
        a->m != b->m || a->n != trans_prod->m || b->n != trans_prod->n)
    {
        return ia_err_argument;
    }

    uint32_t i = 0U;
    uint32_t j = 0U;
    uint32_t k = 0U;
    for (i = 0U; i < a->n; i++) {
        for (j = 0U; j < b->n; j++) {
            trans_prod->data[(i * b->n) + j] = 0.0f;
            for (k = 0U; k < a->m; k++) {
                trans_prod->data[(i * b->n) + j] += a->data[(k * a->n) + i] * b->data[(k * b->n) + j];
            }
        }
    }
    return ia_err_none;
}

/*!
* \brief transpose for ac_matrix
* \param[in] a                  Mandatory.\n
*                               origin matrix.
* \param[in] a_t                Mandatory.\n
*                               transpose of the origin matrix.
*/
ia_err mat_transpose(const ac_matrix_f32_t *a, ac_matrix_f32_t* a_t)
{
    if (a_t == NULL || a->m != a_t->n || a->n != a_t->m)
    {
        return ia_err_argument;
    }
    uint32_t i = 0U;
    uint32_t j = 0U;
    for (i = 0U; i < a->m; i++) {
        for (j = 0U; j < a->n; j++) {
            a_t->data[(j * a->m) + i] = a->data[(i * a->n) + j];
        }
    }
    return ia_err_none;
}

ia_err mat_transpose_d64(const ac_matrix_double_t *a, ac_matrix_double_t* a_t)
{
    if (a_t == NULL || a->m != a_t->n || a->n != a_t->m)
    {
        return ia_err_argument;
    }
    uint32_t i = 0U;
    uint32_t j = 0U;
    for (i = 0U; i < a->m; i++) {
        for (j = 0U; j < a->n; j++) {
            a_t->data[(j * a->m) + i] = a->data[(i * a->n) + j];
        }
    }
    return ia_err_none;
}

/*!
* \brief inplace transpose for ac_matrix
* \param[in, out] a                  Mandatory.\n
*                               origin/output matrix.
* Transpose matrix with O(1) memory
* Find each swap cycles and swap clockwise */
ia_err mat_transpose_inplace(ac_matrix_f32_t* a)
{
    if (a == NULL || a->data == NULL || a->m == 0 || a->n == 0)
    {
        return ia_err_argument;
    }
    uint32_t i = 0U;
    /* New row number */
    uint32_t row = a->n;
    /* New column number */
    uint32_t col = a->m;
    /* Index of next element */
    uint32_t next_i = 0U;
    /* Index of previous element */
    uint32_t prev_i = 0U;
    /* Index of temporary element for swap */
    uint32_t tmp_i = 0U;
    /* Value of temporary element for swap */
    float32_t tmp_value = 0.0f;

    for (i = 1U; i < (row * col) - 1; i++) {
        next_i = ((i % row) * col) + (i / row);
        /*To ignore repetitive cycles, and leave the cycle with minimum head index*/
        while (next_i > i) {
            next_i = ((next_i % row) * col) + (next_i / row);
        }
        /*Check if current cycle is legal*/
        if (next_i == i)
        {
            /* Swap elements clockwise (replace current element with its predecessor
            until it back to the i-th element itself) */
            tmp_value = a->data[i];
            tmp_i = i;
            prev_i = ((i % col) * row) + (i / col);
            while (prev_i != i)
            {
                a->data[tmp_i] = a->data[prev_i];
                tmp_i = prev_i;
                prev_i = ((prev_i % col) * row) + (prev_i / col);
            }
            /* Fill the i-th element to its successor*/
            next_i = ((i % row) * col) + (i / row);
            a->data[next_i] = tmp_value;
        }
    }
    /* Update row and column number*/
    a->m = row;
    a->n = col;
    return ia_err_none;
}

/*!
 * \brief find max/min value in matrix.
 */
ia_err max_min_2D(ac_matrix_f32_t *mat, ac_direction_t direction, ac_vector_t *max_value, ac_vector_t *min_value)
{
    if (mat == NULL || mat->data == NULL)
    {
        return ia_err_argument;
    }
    uint32_t i = 0U;
    switch (direction)
    {
    case horizontal:
        if ((max_value != NULL && (mat->m != max_value->len || max_value->data == NULL)) ||
            (min_value != NULL && (mat->m != min_value->len || min_value->data == NULL)))
        {
            return ia_err_argument;
        }
        for (i = 0U; i < mat->m; i++)
        {
            uint32_t j = 0U;
            float32_t min_data = 0.0F;
            float32_t max_data = 0.0F;
            for (j = 0U; j < mat->n; j++)
            {
                if (min_data > mat->data[i * mat->n + j])
                {
                    min_data = mat->data[i * mat->n + j];
                    break;
                }
                if (max_data < mat->data[i * mat->n + j])
                {
                    max_data = mat->data[i * mat->n + j];
                }
            }
            if (min_value != NULL)
            {
                min_value->data[i] = min_data;
            }
            if (max_value != NULL)
            {
                max_value->data[i] = max_data;
            }
        }
        break;
    case vertical:
        if ((max_value != NULL && (mat->n != max_value->len || max_value->data == NULL)) ||
            (min_value != NULL && (mat->n != min_value->len || min_value->data == NULL)))
        {
            return ia_err_argument;
        }
        for (i = 0U; i < mat->n; i++)
        {
            uint32_t j = 0U;
            float32_t min_data = 0.0F;
            float32_t max_data = 0.0F;
            for (j = 0U; j < mat->m; j++)
            {
                if (min_data > mat->data[j * mat->n + i])
                {
                    min_data = mat->data[j * mat->n + i];
                }
                if (max_data < mat->data[j * mat->n + i])
                {
                    max_data = mat->data[j * mat->n + i];
                }
            }
            if (min_value != NULL)
            {
                min_value->data[i] = min_data;
            }
            if (max_value != NULL)
            {
                max_value->data[i] = max_data;
            }
        }
        break;
    case direction_max:
        if ((max_value != NULL && (1U != max_value->len || max_value->data == NULL)) ||
            (min_value != NULL && (1U != min_value->len || min_value->data == NULL)))
        {
            return ia_err_argument;
        }
        float32_t min_data = 0.0F;
        float32_t max_data = 0.0F;
        for (i = 0U; i < mat->m * mat->n; i++)
        {
            if (min_data > mat->data[i])
            {
                min_data = mat->data[i];
                break;
            }
            if (max_data < mat->data[i])
            {
                max_data = mat->data[i];
            }
        }
        if (min_value != NULL)
        {
            min_value->data[0] = min_data;
        }
        if (max_value != NULL)
        {
            max_value->data[0] = max_data;
        }
        break;
    default:
        break;
    }
    return ia_err_none;
}

ia_err max_u64(ac_matrix_s64_t* mat, int64_t* max)
{
    if (mat == NULL || mat->data == NULL)
    {
        return ia_err_argument;
    }
    *max = mat->data[0];
    uint32_t i = 0U;
    for (i = 1U; i < mat->m * mat->n; i++)
    {
        if (*max < mat->data[i])
        {
            *max = mat->data[i];
        }
    }
    return ia_err_none;
}

/*!
 * \brief matrix padding (inplace).
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
ia_err pad_array_2D(ac_matrix_f32_t* mat, uint32_t pad_height, uint32_t pad_width, ac_padding_type_t type) {
    if (mat->m == 0 || mat->n == 0 || mat->data == NULL)
    {
        return ia_err_argument;
    }
    int32_t pad_left = 0;
    int32_t pad_top = 0;
    int32_t pad_right = 0;
    int32_t pad_bottom = 0;
    int32_t col = 0;
    int32_t row = 0;
    switch (type)
    {
    case padding_pre:
        pad_left = (int32_t)pad_width;
        pad_top = (int32_t)pad_height;
        row = (int32_t)mat->m + pad_top;
        col = (int32_t)mat->n + pad_left;
        break;
    case padding_post:
        pad_right = (int32_t)pad_width;
        pad_bottom = (int32_t)pad_height;
        row = (int32_t)mat->m + pad_bottom;
        col = (int32_t)mat->n + pad_right;
        break;
    case padding_both:
        pad_left = (int32_t)pad_width;
        pad_top = (int32_t)pad_height;
        pad_right = (int32_t)pad_width;
        pad_bottom = (int32_t)pad_height;
        row = (int32_t)mat->m + pad_top + pad_bottom;
        col = (int32_t)mat->n + pad_left + pad_right;
        break;
    default:
        /* no action required. */
        break;
    }
    int32_t i = 0;
    int32_t j = 0;
    /* k,l is temporary row and column index for indicating original position*/
    int32_t k = 0;
    int32_t l = 0;
    for (i = row - 1; i >= 0; i--)
    {
        if (i >= row - pad_bottom)
        {
            k = (int32_t)mat->m - 1;
        }
        else if (i < pad_top)
        {
            k = 0;
        }
        else
        {
            k = i - pad_top;
        }
        for (j = col - 1; j >= 0; j--)
        {
            if (j >= col - pad_right)
            {
                l = (int32_t)mat->n - 1;
            }
            else if (j < pad_left)
            {
                l = 0;
            }
            else
            {
                l = j - pad_left;
            }
            /* Always move element from top-left to bottom-right to avoid overwriting. */
            mat->data[i * col + j] = mat->data[(k * (int32_t)mat->n) + l];
        }
    }
    mat->m = (uint32_t)row;
    mat->n = (uint32_t)col;
    return ia_err_none;
}

/*!
 * \brief matrix repeat.
 *
 * \param[in] mat                   Mandatory.\n
 *                                  repeat matrix.
 * \param[in] rowfactor             Mandatory.\n
 *                                  duplicate factor of row.
 * \param[in] colfactor             Mandatory.\n
 *                                  duplicate factor of column.
 *
 * \param[in] repmat                  Mandatory.\n
 *                                  product of vector.
 * \return                          Error code.
 */
ia_err mat_repmat(ac_matrix_f32_t* mat, uint32_t rowfactor, uint32_t colfactor, ac_matrix_f32_t* repmat) {
    if (mat->m * rowfactor != repmat->m || mat->n * colfactor != repmat->n || mat->m == 0 || mat->n == 0)
    {
        return ia_err_argument;
    }
    uint32_t col = 0U;
    uint32_t row = 0U;
    for (row = 0U; row < repmat->m; row++)
    {
        for (col = 0U; col < repmat->n; col++)
        {
            repmat->data[(row * repmat->n) + col] = mat->data[((row % mat->m) * mat->n) + (col % mat->n)];
        }
    }
    return ia_err_none;
}

/*!
 * \brief vector Quaternion.
 *
 * \param[in] factor1               Mandatory.\n
 *                                  1st factor (Qv1).
 * \param[in] factor2               Mandatory.\n
 *                                  2nd factor (Qv2).
 * \param[out] product              Mandatory.\n
 *                                  product of matrix Quaternion (Qv).
 * \return                          Error code.
 */
ia_err quaternion_product(const VEC_D_3 factor1, const VEC_D_3 factor2, VEC_D_3 product) {
    /* Qs1/Qs2 = sqrt(max(0, 1 - sum(Qv1/Qv2 . ^ 2))) */
    double qs1 = IA_SQRTD(IA_MAX(0.0, 1.0 - (factor1[0] * factor1[0]) - (factor1[1] * factor1[1]) - (factor1[2] * factor1[2])));
    double qs2 = IA_SQRTD(IA_MAX(0.0, 1.0 - (factor2[0] * factor2[0]) - (factor2[1] * factor2[1]) - (factor2[2] * factor2[2])));
    /* Qs = Qs1 * Qs2 - dot(Qv1, Qv2) */
    double qs = (qs1 * qs2) - (factor1[0] * factor2[0]) - (factor1[1] * factor2[1]) - (factor1[2] * factor2[2]);
    /* Qv = Qs1 * Qv2 + Qv1 * Qs2 + cross(Qv1, Qv2) */
    product[0] = (qs1 * factor2[0]) + (qs2 * factor1[0]) + (factor1[1] * factor2[2]) - (factor1[2] * factor2[1]);
    product[1] = (qs1 * factor2[1]) + (qs2 * factor1[1]) - (factor1[0] * factor2[2]) + (factor1[2] * factor2[0]);
    product[2] = (qs1 * factor2[2]) + (qs2 * factor1[2]) + (factor1[0] * factor2[1]) - (factor1[1] * factor2[0]);
    if (qs < 0.0)
    {
        product[0] = -product[0]; product[1] = -product[1]; product[2] = -product[2];
    }
    return ia_err_none;
}

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
ia_err vec_sind(VEC_D_3 src, VEC_D_3 dst, uint32_t len)
{
    if (len != 3U)
    {
        return ia_err_argument;
    }
    uint32_t i = 0U;
    for (i = 0U; i < len; i++)
    {
        dst[i] = sind(src[i]);
    }
    return ia_err_none;
}

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
ia_err vector_scaler_operation(const VEC_D_3 factor1, double factor2, char operator, VEC_D_3 product) {
    ia_err ret = ia_err_none;
    switch (operator)
    {
    case '+':
        product[0] = factor1[0] + factor2;
        product[1] = factor1[1] + factor2;
        product[2] = factor1[2] + factor2;
        break;
    case '-':
        product[0] = factor1[0] - factor2;
        product[1] = factor1[1] - factor2;
        product[2] = factor1[2] - factor2;
        break;
    case '*':
        product[0] = factor1[0] * factor2;
        product[1] = factor1[1] * factor2;
        product[2] = factor1[2] * factor2;
        break;
    case '/':
        if (IA_FABS(factor2) < IA_EPSILON)
        {
            ret = ia_err_argument;
            product[0] = 0.0f; product[1] = 0.0f; product[2] = 0.0f;
        }
        else
        {
            product[0] = factor1[0] / factor2;
            product[1] = factor1[1] / factor2;
            product[2] = factor1[2] / factor2;
        }
        break;
    default:
        ret = ia_err_argument;
        break;
    }
    return ret;
}

/*!
 * \brief matrix to vector operation including addition(+)/subtraction(-)/
 *     matrix multiplication(*)/division(/).
 *
 * \param[in] factor1           Mandatory.\n
 *                                  pointer of factor1 (p*q matrix).
 * \param[in] factor2           Mandatory.\n
 *                                  pointer of factor2 (p*1 vector).
 * \param[in] operator              Mandatory.\n
 *                                  operator (/).
 * \param[out] product          Mandatory.\n
 *                                  product of operation.
 * \return                          Error code.
 */
ia_err ac_bsxfun_operation(const ac_matrix_f32_t* factor1, const ac_matrix_f32_t* factor2, char operator, ac_matrix_f32_t* product)
{
    if (factor1 == NULL || factor1->data == NULL || factor2 == NULL ||
        factor2->data == NULL || factor1->m != factor2->m || factor2->n != 1 ||
        factor1->m != product->m || factor1->n != product->n)
    {
        return ia_err_argument;
    }
    ia_err ret = ia_err_none;
    uint32_t i = 0U;
    uint32_t j = 0U;
    switch (operator)
    {
    case '+':
        for (i = 0U; i < factor1->m; i++)
        {
            for (j = 0U; j < factor1->n; j++)
            {
                product->data[(i * factor1->n) + j] = factor1->data[(i * factor1->n) + j] + factor2->data[i];
            }
        }
        break;
    case '-':
        for (i = 0U; i < factor1->m; i++)
        {
            for (j = 0U; j < factor1->n; j++)
            {
                product->data[(i * factor1->n) + j] = factor1->data[(i * factor1->n) + j] - factor2->data[i];
            }
        }
        break;
    case '*':
        for (i = 0U; i < factor1->m; i++)
        {
            for (j = 0U; j < factor1->n; j++)
            {
                product->data[(i * factor1->n) + j] = factor1->data[(i * factor1->n) + j] * factor2->data[i];
            }
        }
        break;
    case '/':
        for (i = 0U; i < factor1->m; i++)
        {
            for (j = 0U; j < factor1->n; j++)
            {
                if (IA_FABS(factor2->data[i]) < IA_EPSILON)
                {
                    ret = ia_err_argument;
                    break;
                }
                else
                {
                    product->data[(i * factor1->n) + j] = factor1->data[(i * factor1->n) + j] / factor2->data[i];
                }
            }
            if (ret != ia_err_none)
            {
                break;
            }
        }
        break;
    default:
        ret = ia_err_argument;
        break;

    }
    return ret;
}

/*!
 * \brief calculate the inverse matrix for n*n matrix
 *
 * \param[in] matrix                Mandatory.\n
 *                                  n*n matrix. (n <= 3)
 * \param[out] inv_matrix           Mandatory.\n
 *                                  n*n matrix, inverse matrix for the input matrix.
 * \param[out] det                  optional.\n
 *                                  float, determinant for n matrices.
 * \return                          Error code.
 */
ia_err invert_det_matrix(const ac_matrix_f32_t* matrix, ac_matrix_f32_t* inv_matrix, float32_t* det)
{
    if (matrix == NULL || inv_matrix == NULL || matrix->m != matrix->n || inv_matrix->m != inv_matrix->n || matrix->m != inv_matrix->m)
    {
        return ia_err_argument;
    }
    ia_err ret = ia_err_none;
    float32_t det_mat = 0.0f;
    switch (matrix->m)
    {
    case 1:
        det_mat = matrix->data[0];
        if (IA_FABS(det_mat) < IA_EPSILON)
        {
            inv_matrix->data[0] = 0.0f;
            if (det != NULL)
            {
                *det = 0.0f;
            }
            ret = ia_err_argument;
        }
        else
        {
            inv_matrix->data[0] = 1.0f / det_mat;
            if (det != NULL)
            {
                *det = det_mat;
            }
        }
        break;
    case 2:
        det_mat = (matrix->data[0] * matrix->data[3]) - (matrix->data[1] * matrix->data[2]);
#ifdef ENABLE_NEON
#if defined __aarch64__
        if (IA_FABS(det_mat) < IA_EPSILON)
        {
            inv_matrix->data[0] = 0.0f;
            inv_matrix->data[1] = 0.0f;
            inv_matrix->data[2] = 0.0f;
            inv_matrix->data[3] = 0.0f;
            if (det != NULL)
            {
                *det = det_mat;
            }
            ret = ia_err_argument;
        }
        else
        {
            /* inv(A) = Adjoint(A) / det(A) */
            float32_t data[4] = {matrix->data[3], -matrix->data[1], -matrix->data[2], matrix->data[0]};
            float32x4_t A;
            A = vld1q_f32(data);
            float32x4_t inv = vmulq_n_f32(A, 1/det_mat);
            vst1q_f32(inv_matrix->data, inv);
            if (det != NULL)
            {
                *det = det_mat;
            }
        }
        break;
#endif
#endif
        if (IA_FABS(det_mat) < IA_EPSILON)
        {
            inv_matrix->data[0] = 0.0f;
            inv_matrix->data[1] = 0.0f;
            inv_matrix->data[2] = 0.0f;
            inv_matrix->data[3] = 0.0f;
            if (det != NULL)
            {
                *det = det_mat;
            }
            ret = ia_err_argument;
        }
        else
        {
            /* inv(A) = Adjoint(A) / det(A) */
            inv_matrix->data[0] = matrix->data[3] / det_mat;
            inv_matrix->data[1] = -matrix->data[1] / det_mat;
            inv_matrix->data[2] = -matrix->data[2] / det_mat;
            inv_matrix->data[3] = matrix->data[0] / det_mat;
            if (det != NULL)
            {
                *det = det_mat;
            }
        }
        break;
    case 3:
        ret = inv_3_3(matrix, inv_matrix, det);
        break;
    default:
        /* No support for matrix inverse if the size of matrix is larger than 3.*/
        ret = ia_err_argument;
        break;
    }
    return ret;
}

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
ia_err invert_det_matrix_d64(const ac_matrix_double_t *matrix, ac_matrix_double_t *inv_matrix, double *det)
{
    if (matrix == NULL || inv_matrix == NULL || matrix->m != matrix->n || inv_matrix->m != inv_matrix->n || matrix->m != inv_matrix->m)
    {
        return ia_err_argument;
    }
    ia_err ret = ia_err_none;
    double det_mat = 0.0;
    switch (matrix->m)
    {
    case 1:
        det_mat = matrix->data[0];
        if (IA_FABSD(det_mat) < (double)IA_EPSILON)
        {
            inv_matrix->data[0] = 0.0;
            if (det != NULL)
            {
                *det = 0.0;
            }
            ret = ia_err_argument;
        }
        else
        {
            inv_matrix->data[0] = 1.0 / det_mat;
            if (det != NULL)
            {
                *det = det_mat;
            }
        }
        break;
    case 2:
        det_mat = (matrix->data[0] * matrix->data[3]) - (matrix->data[1] * matrix->data[2]);
        if (IA_FABS(det_mat) < IA_EPSILON)
        {
            inv_matrix->data[0] = 0.0;
            inv_matrix->data[1] = 0.0;
            inv_matrix->data[2] = 0.0;
            inv_matrix->data[3] = 0.0;
            if (det != NULL)
            {
                *det = det_mat;
            }
            ret = ia_err_argument;
        }
        else
        {
            /* inv(A) = Adjoint(A) / det(A) */
            inv_matrix->data[0] = matrix->data[3] / det_mat;
            inv_matrix->data[1] = -matrix->data[1] / det_mat;
            inv_matrix->data[2] = -matrix->data[2] / det_mat;
            inv_matrix->data[3] = matrix->data[0] / det_mat;
            if (det != NULL)
            {
                *det = det_mat;
            }
        }
        break;
    case 3:
        ret = inv_3_3_d64(matrix, inv_matrix, det);
        break;
    default:
        /* No support for matrix inverse if the size of matrix is larger than 3.*/
        ret = ia_err_argument;
        break;
    }
    return ret;
}

/*!
 * \brief calculate the inverse matrix and det for multiple matrices
 *
 * \param[in] matrices              Mandatory.\n
 *                                  3*3*n tensor, containing n 3*3 matrices.
 * \param[out] inv_matrices         Mandatory.\n
 *                                  3*3*n tensor, 3*3 inverse matrices for n input matrices.
 * \param[out] dets                 Mandatory.\n
 *                                  n*1 vector, determinants for n matrices.
 * \return                          Error code.
 */
ia_err invert_det_matrices(const ac_3_tensor_t* matrices, ac_3_tensor_t* inv_matrices, ac_vector_t* dets)
{
    if (matrices == NULL || inv_matrices == NULL || matrices->data == NULL ||
        inv_matrices->data == NULL || dets->data == NULL || matrices->k != dets->len)
    {
        return ia_err_argument;
    }
    /* Temporary data for matrix inverse. */
    float32_t temp_mat_data[9] = { 0.0f };
    float32_t temp_inv_mat_data[9] = { 0.0f };
    /* Temporary ac_matrix for matrix inverse. */
    ac_matrix_f32_t temp_mat = { temp_mat_data,0U,0U,0U };
    ac_matrix_f32_t temp_inv_mat = { temp_inv_mat_data,0U,0U,0U };
    uint32_t i = 0U;
    uint32_t j = 0U;
    uint32_t p = 0U;
    ia_err ret = ia_err_none;
    for (i = 0U; i < matrices->k; i++)
    {
        if (matrices->m == matrices->n && matrices->m <= 3U)
        {
            temp_mat.m = matrices->m;
            temp_mat.n = matrices->n;
            temp_inv_mat.m = matrices->m;
            temp_inv_mat.n = matrices->n;
            /* Extract data from ac_tensor */
            for (j = 0U; j < matrices->m; j++)
            {
                for (p = 0U; p < matrices->n; p++)
                {
                    temp_mat_data[j * matrices->n + p] = matrices->data[(j * matrices->n * matrices->k) + (p * matrices->k) + i];
                }
            }
            /* Compute inverse matrix and determinant for current M*N(3*3) matrix */
            ret = invert_det_matrix(&temp_mat, &temp_inv_mat, dets->data + i);
            if (ret != ia_err_none)
            {
                return ret;
            }
            /* Save inverse matrix to inv_matrices from tempoparay data */
            for (j = 0U; j < matrices->m; j++)
            {
                for (p = 0U; p < matrices->n; p++)
                {
                    inv_matrices->data[(j * matrices->n * matrices->k) + (p * matrices->k) + i] = temp_inv_mat_data[j * matrices->n + p];
                }
            }
        }
    }
    return ret;
}

/*!
 * \brief draw_next_rand generate rand using Schrage's method.
 * IQ = floor(IM / IA), IR = mod(IM, IA)
 * IA * S_in = IA * mod(S_in, IQ) + (IM - IR) * floor(S_in / IQ)
 * IA * mod(S_in, IQ) < IM
 * If: IR <= IQ (guarantied when IA <= sqrt(IM)), then: IR * floor(S_in / IQ) < IM
 * Result: IA * mod(S_in, IQ) - IR) * floor(S_in / IQ) = S_out or (S_out - IM) and can be coputed using int32 operations
 * \param[in] seed_in             Mandatory.\n
 *                                int32 number for input seed.
 * \param[in] rand_max            Mandatory.\n
 *                                upper bound of random number.
 * \param[out] seed_out           Mandatory.\n
 *                                int32 for output seed.
 * \param[out] rand_out           Mandatory.\n
 *                                random number range from [1, rand_max].
 */
static void draw_next_rand(int32_t seed_in, int32_t rand_max, int32_t* seed_out, int32_t* rand_out)
{
    static const int32_t IA = 16807;
    static const int32_t IM = 2147483647;
    static const int32_t IQ = 127773;
    static const int32_t IR = 2836;

    int32_t IK = seed_in / IQ;
    *seed_out = IA * (seed_in - IK * IQ) - IR * IK;
    if (*seed_out < 0)
    {
        *seed_out = *seed_out + IM;
    }
    *rand_out = (int32_t)(1 + (((int64_t)rand_max * (int64_t)(*seed_out)) / IM));
}


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
 *                                list (uint32_t) contains n_rands random numbers.
 */
ia_err Random_seed(int32_t rng_seed_in, int32_t rand_max, int32_t n_rands, int32_t *rng_seed_out, uint32_t* list)
{
    if (n_rands > rand_max || rand_max <= 0)
    {
        return ia_err_argument;
    }

    ia_err ret = ia_err_none;
    int32_t rng_seed = rng_seed_in;
    int32_t n_inserted = 0;
    int32_t i_rand = 0;
    int32_t rand_next = 0;

    for (n_inserted = 0; n_inserted < n_rands; n_inserted++)
    {
        /* Draw random number within [1, rand_max-n_inserted] and update rng_seed*/
        draw_next_rand(rng_seed, rand_max - n_inserted, &rng_seed, &rand_next);
        /* Adjust random number's range to [n_inserted+1, rand_max] */
        rand_next = rand_next + n_inserted;
        /* Insert the random number to output list with ascending order*/
        i_rand = n_inserted;
        while ((i_rand > 0) && ((int32_t) * (list + i_rand - 1) >= rand_next))
        {
            /* Move random number larger than current forward. */
            *(list + i_rand) = *(list + i_rand - 1);
            /* Adjust current random number's range if position changed. */
            rand_next = rand_next - 1;
            i_rand = i_rand - 1;
        }
        /* Put current random number into list. */
        if (rand_next >= 0)
        {
            *(list + i_rand) = (uint32_t)rand_next;
        }
        else
        {
            AC_LOG("Error in Random seed. Seed in: %d, Random number:%d",
                rng_seed_in, rand_next);
            ret = ia_err_data;
        }
    }
    *rng_seed_out = rng_seed;
    AC_LOG("Seed in: %d, Rand number:%d,%d,%d, Seed out:%d",
        rng_seed_in, *(list), *(list + 1), *(list + 2), *rng_seed_out);
    return ret;
}


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
double interpolatepoint_d(double x_L, double x_R, double y_L, double y_R, double y_target)
{
    double x_target = 0.0;
    if (IA_FABS(y_L - y_R) < (double)IA_EPSILON)
    {
        x_target = (x_L + x_R) / 2.0;
    }
    else
    {
        /* qx = qxL + (qxR - qxL) * (ff_target - ffL) / (ffR - ffL) */
        x_target = x_L + (x_R - x_L) * (y_target - y_L) / (y_R - y_L);
    }
    return x_target;
}

/*!
* \brief find the square sum for ac_matrix
* \param[in] mat                  Mandatory.\n
*                               origin matrix.
* \param[in] sum                Mandatory.\n
*                               sum of the matrix.
*/
ia_err mat_square_sum(ac_matrix_f32_t* mat, float32_t* sum)
{
    if (mat == NULL)
    {
        return ia_err_argument;
    }
    uint32_t i = 0U;
    *sum = 0.0f;
    for (i = 0U; i < (mat->m * mat->n); i++)
    {
        (*sum) += (mat->data[i] * mat->data[i]);
    }
    return ia_err_none;
}

/*!
* \brief cast matrix from double to float
* \param[in] src                Mandatory.\n
*                               double type matrix
* \param[out] dst                Mandatory.\n
*                               float type matrix.
*/
void ac_mat_3_3_cast(MAT_D_3_3 src, MAT_3_3 dst)
{
    int32_t i = 0;
    int32_t j = 0;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            dst[i][j] = (float32_t)src[i][j];
        }
    }
}

ia_err ac_mat_cast(ac_matrix_double_t src, ac_matrix_f32_t dst)
{
    if ((src.m != dst.m) || (src.n != dst.n)) {
        return ia_err_argument;
    }

    uint32_t size = src.n * src.m;
    uint32_t idx = 0U;
    for (idx = 0U; idx < size; idx++) {
        dst.data[idx] = (float32_t)src.data[idx];
    }
    return ia_err_none;
}

/*!
* \brief cast vector from double to float
* \param[in] src                Mandatory.\n
*                               double type vector
* \param[out] dst               Mandatory.\n
*                               float type vector.
*/
void ac_vec_3_cast(VEC_D_3 src, VEC_3 dst)
{
    int32_t i = 0;
    for (i = 0; i < 3; i++) {
            dst[i]= (float32_t)src[i];
    }
}

/* Comment unused functions to avoid compile warning on Linux*/
#if (0)

/*!
*   \brief Solve linear equation Ax=b for x, for ac_matrix
*          x = A^-1 * b if A is nonsingular square matrix
*          x^ = pinv(A)*b = (A^t*A)^-1 * (A^t *b) if A is
*          full column rank.
* \param[in] a                  Mandatory.\n
*                               left operand of .
* \param[in] b                  Mandatory.\n
*                               right operand of division
                                should have only 1 column.
* \param[in] x                  Mandatory.\n
*                               the solution (Ax=b).
*/
static ia_err mat_linsolve(ac_matrix_f32_t* a, ac_matrix_f32_t* b, ac_matrix_f32_t* x)
{
    if (a == NULL || b == NULL || x == NULL ||
        a->data == NULL || b->data == NULL || x->data == NULL)
    {
        return ia_err_argument;
    }

    if (a->m == a->n && a->n == x->m && x->n == 1 && a->m == b->m && b->n == 1)
    {
        /* Solve x with Gauss Jordan Elimination of [A|b]*/
        uint32_t i = 0U;
        uint32_t j = 0U;
        uint32_t k = 0U;
        ac_matrix_f32_t augmented_matrix = { (float*)NULL, 0U, 0U };
        float augmented_matrix_data[MAX_AUGMENTED_MATRIX];

        if ((a->m * (a->n + b->n)) > MAX_AUGMENTED_MATRIX)
        {
            return ia_err_argument;
        }
        augmented_matrix.m = a->m;
        augmented_matrix.n = a->n + b->n;
        augmented_matrix.data = augmented_matrix_data;

        /* Construct Augmented Matrix with A and b*/
        for (i = 0U; i < a->m; i++)
        {
            for (j = 0U; j < a->n; j++)
            {
                augmented_matrix.data[(i * augmented_matrix.n) + j] = a->data[(i * a->n) + j];
            }
            augmented_matrix.data[(i * augmented_matrix.n) + a->n] = b->data[i];
        }

        /* Gauss Jordan Elimination on Augmented Matrix*/
        float ratio = 0.0f;
        for (i = 0U; i < a->m; i++)
        {
            if (a->data[(i * a->n) + i] == 0)
            {
                return ia_err_argument;
            }
            for (j = 0U; j < a->m; j++)
            {
                if (i != j)
                {
                    ratio = augmented_matrix.data[(j * augmented_matrix.n) + i] / augmented_matrix.data[(i * augmented_matrix.n) + i];
                    for (k = 0U; k < augmented_matrix.n; k++)
                    {
                        augmented_matrix.data[(j * augmented_matrix.n) + k] -= ratio * augmented_matrix.data[(i * augmented_matrix.n) + k];
                    }
                }
            }
        }
        for (i = 0U; i < a->n; i++)
        {
            x->data[i] = augmented_matrix.data[((i + 1) * augmented_matrix.n) - 1] / augmented_matrix.data[(i * augmented_matrix.n) + i];
        }
        return ia_err_none;
    }
    else if (a->n == 3 && a->n == x->m && x->n == 1 && a->m == b->m && b->n == 1)
    {
        /*!
        * Solve least-squares solution x^ with equation x^ = pinv(A) * b = (A ^ t * A) ^ -1 * (A ^ t * b)
        * A should be n*3 here.
        */
        ia_err ret;
        float ata_data[9] = { 0.0f };
        float inv_ata_data[9] = { 0.0f };
        float atb_data[3] = { 0.0f };
        ac_matrix_f32_t ata = { (float*)NULL, 3U, 3U, 3U };
        ata.data = ata_data;
        ac_matrix_f32_t inv_ata = { (float*)NULL, 3U, 3U, 3U };
        inv_ata.data = inv_ata_data;
        ac_matrix_f32_t atb = { (float*)NULL, 3U, 1U, 1U };
        atb.data = atb_data;
        if (ata_data == NULL || inv_ata_data == NULL)
        {
            return ia_err_nomemory;
        }

        /* Calculate AtA = A^t*A */
        ret = mat_transpose_times(a, a, &ata);
        if (ret != ia_err_none)
        {
            return ret;
        }

        /* Calculate inv_AtA = (AtA)^-1 */
        /* Pseudo inverse of matrix only support 3*3 in auto-calibration */
        if (ata.n == 3 && ata.m == 3)
        {
            ret = inv_3_3(&ata, &inv_ata, NULL);
            if (ret != ia_err_none)
            {
                return ret;
            }
        }
        else
        {
            return ia_err_data;
        }

        /* Calculate (A ^ t * b) */
        ret = mat_transpose_times(a, b, &atb);
        if (ret != ia_err_none)
        {
            return ret;
        }

        /* Calculate x = (A ^ t * A) ^ -1 * (A ^ t * b) = inv_AtA * Atb */
        ret = mat_mtimes(&inv_ata, &atb, x);
        if (ret != ia_err_none)
        {
            return ret;
        }
        return ia_err_none;
    }
    else
    {
        return ia_err_not_run;
    }
}

/*!
 * \brief product of vector.
 *
 */
static ia_err prod_vector(uint32_t len, ac_vector_t* vec, float32_t* prod)
{
    if ((vec == NULL) || (len == 0U)) {
        return ia_err_argument;
    }
    float ret = 1.0F;
    uint32_t i = 0U;
    for (i = 0U; i < len; i++) {
        ret *= vec->data[i];
    }
    *prod = ret;

    return ia_err_none;
}
#endif // 0

ia_err Rotation3D_Vector2Matrix(const VEC_D_3 vec, MAT_D_3_3 mat)
{
    MAT_D_3_3 eye = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    double q_tmp[9] = {0};
    ac_matrix_double_t Q_tmp_m = {q_tmp, 3U, 3U, 3U};
    double sum = (vec[0] * vec[0]) + (vec[1] * vec[1]) + (vec[2] * vec[2]);
    double min = IA_MIN(1.0, sum);
    if (min == 0.0F) {
        IA_MEMCOPY(mat, eye, 9U * sizeof(double));
        return ia_err_none;
    }
    double ch = sqrt(1.0F - min);
    double qq[9] = {0.0, -vec[2], vec[1], vec[2], 0.0, -vec[0], -vec[1], vec[0], 0.0};
    ac_matrix_double_t Q_m = {qq, 3U, 3U, 3U};
    matrix_matrix_operation_d(Q_m, Q_m, '*', Q_tmp_m);
    uint8_t i = 0U;
    uint8_t j = 0U;
    uint32_t idx = 0U;
    //result = eye(3) + (2 * ch) * QQ + 2 * QQ * QQ;
    for(i = 0U;i < 3U;i++) {
        for(j = 0U;j < 3U;j++) {
            idx = i * 3U + j;
            mat[i][j] = eye[i][j] + (2.0F * ch * Q_m.data[idx]) + (2.0F * Q_tmp_m.data[idx]);
        }
    }
    return ia_err_none;
}

ia_err Rotation3D_Matrix2InvVector(const MAT_D_3_3 mat, VEC_D_3 vec)
{
    VEC_D_3 As = {(mat[2][1]-mat[1][2])/2.0, (mat[0][2]-mat[2][0])/2.0, (mat[1][0]-mat[0][1])/2.0};
    float64_t ch2_sq = IA_MAX(0.0, mat[0][0]+mat[1][1]+mat[2][2]+1);
    if (IA_FABSD(ch2_sq) < IA_EPSILON)
    {
        VEC_D_3 Rm1_sq = {(mat[0][0] + 1.0) * (mat[0][0] + 1.0) + mat[0][1] * mat[0][1] + mat[0][2] * mat[0][2],
                          (mat[1][1] + 1.0) * (mat[1][1] + 1.0) + mat[1][0] * mat[1][0] + mat[1][2] * mat[1][2],
                          (mat[2][2] + 1.0) * (mat[2][2] + 1.0) + mat[2][0] * mat[2][0] + mat[2][1] * mat[2][1]};
        int32_t ind_max = 0;
        if (Rm1_sq[0] >= Rm1_sq[1] && Rm1_sq[0] >= Rm1_sq[2])
        {
            ind_max = 0;
        } else if (Rm1_sq[1] >= Rm1_sq[2])
        {
            ind_max = 1;
        } else
        {
            ind_max = 2;
        }
        double divide = IA_SQRTD(Rm1_sq[ind_max]);   /* always larger than zero */
        vec[0] = -(mat[0][0] + 1.0)/divide;
        vec[1] = -mat[0][1]/divide;
        vec[2] = -mat[0][2]/divide;
    } else
    {
        double divide = IA_SQRTD(ch2_sq);
        vec[0] = -As[0]/divide;
        vec[1] = -As[1]/divide;
        vec[2] = -As[2]/divide;
    }
    return ia_err_none;
}

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
ia_err inv_3_3(const ac_matrix_f32_t* mat, ac_matrix_f32_t* inv_mat, float32_t *det)
{
    if ((mat->m == 3U) && (mat->n == 3U))
    {
        /* Adjoint matrix of ac_matrix mat */
        float32_t adj_mat[9] = { 0.0f };

        /* Calculate adjoint matrix (3*3) matrix M*/

        adj_mat[0] = det_2_2(mat->data[4], mat->data[5], mat->data[7], mat->data[8]);
        adj_mat[1] = -1.0f * det_2_2(mat->data[1], mat->data[2], mat->data[7], mat->data[8]);
        adj_mat[2] = det_2_2(mat->data[1], mat->data[2], mat->data[4], mat->data[5]);
        adj_mat[3] = -1.0f * det_2_2(mat->data[3], mat->data[5], mat->data[6], mat->data[8]);
        adj_mat[4] = det_2_2(mat->data[0], mat->data[2], mat->data[6], mat->data[8]);
        adj_mat[5] = -1.0f * det_2_2(mat->data[0], mat->data[2], mat->data[3], mat->data[5]);
        adj_mat[6] = det_2_2(mat->data[3], mat->data[4], mat->data[6], mat->data[7]);
        adj_mat[7] = -1.0f * det_2_2(mat->data[0], mat->data[1], mat->data[6], mat->data[7]);
        adj_mat[8] = det_2_2(mat->data[0], mat->data[1], mat->data[3], mat->data[4]);

        /* Calculate inverse matrix using A^-1 = det(A)^-1 * adj(A)*/
        float32_t det_mat = (adj_mat[0] * mat->data[0]) + (adj_mat[1] * mat->data[3]) + (adj_mat[2] * mat->data[6]);
        if (IA_FABS(det_mat) > IA_EPSILON) {
            inv_mat->data[0] = adj_mat[0] / det_mat;
            inv_mat->data[1] = adj_mat[1] / det_mat;
            inv_mat->data[2] = adj_mat[2] / det_mat;
            inv_mat->data[3] = adj_mat[3] / det_mat;
            inv_mat->data[4] = adj_mat[4] / det_mat;
            inv_mat->data[5] = adj_mat[5] / det_mat;
            inv_mat->data[6] = adj_mat[6] / det_mat;
            inv_mat->data[7] = adj_mat[7] / det_mat;
            inv_mat->data[8] = adj_mat[8] / det_mat;
            if (det != NULL)
            {
                *det = det_mat;
            }
            return ia_err_none;
        }
        else
        {
            /* singular matrix*/
            if (det != NULL)
            {
                *det = 0.0f;
            }
            return ia_err_data;
        }
    }
    else
    {
        return ia_err_argument;
    }
}

ia_err inv_3_3_d64(const ac_matrix_double_t* mat, ac_matrix_double_t* inv_mat, double *det)
{
    if ((mat->m == 3U) && (mat->n == 3U))
    {
        /* Adjoint matrix of ac_matrix mat */
        double adj_mat[9] = { 0 };
        /* Calculate adjoint matrix (3*3) matrix M*/
        adj_mat[0] = det_2_2_d64(mat->data[4], mat->data[5], mat->data[7], mat->data[8]);
        adj_mat[1] = -1 * det_2_2_d64(mat->data[1], mat->data[2], mat->data[7], mat->data[8]);
        adj_mat[2] = det_2_2_d64(mat->data[1], mat->data[2], mat->data[4], mat->data[5]);
        adj_mat[3] = -1 * det_2_2_d64(mat->data[3], mat->data[5], mat->data[6], mat->data[8]);
        adj_mat[4] = det_2_2_d64(mat->data[0], mat->data[2], mat->data[6], mat->data[8]);
        adj_mat[5] = -1 * det_2_2_d64(mat->data[0], mat->data[2], mat->data[3], mat->data[5]);
        adj_mat[6] = det_2_2_d64(mat->data[3], mat->data[4], mat->data[6], mat->data[7]);
        adj_mat[7] = -1 * det_2_2_d64(mat->data[0], mat->data[1], mat->data[6], mat->data[7]);
        adj_mat[8] = det_2_2_d64(mat->data[0], mat->data[1], mat->data[3], mat->data[4]);

        /* Calculate inverse matrix using A^-1 = det(A)^-1 * adj(A)*/
        double det_mat = (adj_mat[0] * mat->data[0]) + (adj_mat[1] * mat->data[3]) + (adj_mat[2] * mat->data[6]);
        if (IA_FABS(det_mat) > DET_MIN) {
            inv_mat->data[0] = adj_mat[0] / det_mat;
            inv_mat->data[1] = adj_mat[1] / det_mat;
            inv_mat->data[2] = adj_mat[2] / det_mat;
            inv_mat->data[3] = adj_mat[3] / det_mat;
            inv_mat->data[4] = adj_mat[4] / det_mat;
            inv_mat->data[5] = adj_mat[5] / det_mat;
            inv_mat->data[6] = adj_mat[6] / det_mat;
            inv_mat->data[7] = adj_mat[7] / det_mat;
            inv_mat->data[8] = adj_mat[8] / det_mat;
            if (det != NULL)
            {
                *det = det_mat;
            }
            return ia_err_none;
        }
        else
        {
            /* singular matrix*/
            if (det != NULL)
            {
                *det = 0.0f;
            }
            return ia_err_data;
        }
    }
    else
    {
        return ia_err_argument;
    }
}

/*!
* \brief matrix multiplication for ac_matrix
* \param[in] a                  Mandatory.\n
*                               left operand of multiplication.
* \param[in] b                  Mandatory.\n
*                               right operand of matrix multiplication.
* \param[in] mprod              Mandatory.\n
*                               the matrix product of matrix a and b.
*/
static ia_err mat_mtimes(ac_matrix_f32_t* a, ac_matrix_f32_t* b, ac_matrix_f32_t* mprod)
{
    if (a == NULL || b == NULL || mprod == NULL ||
        a->data == NULL || b->data == NULL || mprod->data == NULL ||
        a->n != b->m || a->m != mprod->m || b->n != mprod->n)
    {
        return ia_err_argument;
    }
#ifdef ENABLE_NEON
#if defined __aarch64__
    // optimize 2x2 * 2x1 matrix operation with NEON
    if ((a->n == 2) && (a->m == 2) && (b->n == 1))
    {
        float32x2x2_t vec1;
        vec1 = vld2_f32(a->data);
        float32x2_t vec2;
        vec2 = vld1_f32(b->data);
        float32x2_t result = vdup_n_f32(0);;
        result = vmla_lane_f32(result, vec1.val[0], vec2, 0);
        result = vmla_lane_f32(result, vec1.val[1], vec2, 1);
        vst1_f32(mprod->data, result);
        return ia_err_none;
    }
#endif
#endif
    uint32_t i = 0U;
    uint32_t j = 0U;
    uint32_t k = 0U;
    for (i = 0U; i < a->m; i++) {
        for (j = 0U; j < b->n; j++) {
            mprod->data[(i * b->n) + j] = 0.0f;
            for (k = 0U; k < a->n; k++) {
                mprod->data[(i * b->n) + j] += a->data[(i * a->n) + k] * b->data[(k * b->n) + j];
            }
        }
    }
    return ia_err_none;
}

static ia_err mat_mtimes_d(ac_matrix_double_t* a, ac_matrix_double_t* b, ac_matrix_double_t* mprod)
{
    if (a == NULL || b == NULL || mprod == NULL ||
        a->data == NULL || b->data == NULL || mprod->data == NULL ||
        a->n != b->m || a->m != mprod->m || b->n != mprod->n)
    {
        return ia_err_argument;
    }
#ifdef ENABLE_NEON
#if defined __aarch64__
    // optimize 2x2 * 2x1 matrix operation with NEON
    if ((a->n == 2) && (a->m == 2) && (b->n == 1))
    {
        float32x2x2_t vec1;
        vec1 = vld2_f32(a->data);
        float32x2_t vec2;
        vec2 = vld1_f32(b->data);
        float32x2_t result = vdup_n_f32(0);;
        result = vmla_lane_f32(result, vec1.val[0], vec2, 0);
        result = vmla_lane_f32(result, vec1.val[1], vec2, 1);
        vst1_f32(mprod->data, result);
        return ia_err_none;
    }
#endif
#endif
    uint32_t i = 0U;
    uint32_t j = 0U;
    uint32_t k = 0U;
    for (i = 0U; i < a->m; i++) {
        for (j = 0U; j < b->n; j++) {
            mprod->data[(i * b->n) + j] = 0.0f;
            for (k = 0U; k < a->n; k++) {
                mprod->data[(i * b->n) + j] += a->data[(i * a->n) + k] * b->data[(k * b->n) + j];
            }
        }
    }
    return ia_err_none;
}

static ia_err mat_mtimes_u64_f32(ac_matrix_double_t* a, ac_matrix_u64_t* b, ac_matrix_double_t* mprod)
{
    if (a == NULL || b == NULL || mprod == NULL ||
        a->data == NULL || b->data == NULL || mprod->data == NULL ||
        a->n != b->m || a->m != mprod->m || b->n != mprod->n)
    {
        return ia_err_argument;
    }
    uint32_t i = 0U;
    uint32_t j = 0U;
    uint32_t k = 0U;
    for (i = 0U; i < a->m; i++) {
        for (j = 0U; j < b->n; j++) {
            mprod->data[(i * b->n) + j] = 0.0f;
            for (k = 0U; k < a->n; k++) {
                mprod->data[(i * b->n) + j] += a->data[(i * a->n) + k] * (double)b->data[(k * b->n) + j];
            }
        }
    }
    return ia_err_none;
}

/*!
* \brief compare function for qsort
* \param[in] a                  Mandatory.\n
*                               origin matrix.
* \param[in] b                Mandatory.\n
*                               sum of the matrix.
* return                      Result value for qsort.\n
*                             when a>b return 1
*                                  a=b return 0
*                                  a<b return -1
*/
int32_t ac_compare_f32(const void* a, const void* b)
{
    if ((*(float32_t*)a) > (*(float32_t*)b))
    {
        return 1;
    }
    else if ((*(float32_t*)a) < (*(float32_t*)b))
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

int32_t ac_compare_u16(const void* a, const void* b)
{
    if ((*(uint16_t*)a) > (*(uint16_t*)b))
    {
        return 1;
    }
    else if ((*(uint16_t*)a) < (*(uint16_t*)b))
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

/*!
* \brief addition for ac_matrix
* \param[in] a                  Mandatory.\n
*                               left operand of matrix addition.
* \param[in] b                  Mandatory.\n
*                               right operand of matrix addition.
* \param[in] sum                Mandatory.\n
*                               the sum of matrix addition.
*/
static ia_err mat_add(ac_matrix_f32_t* a, ac_matrix_f32_t* b, ac_matrix_f32_t* sum)
{
    if (a == NULL || b == NULL || sum == NULL ||
        a->data == NULL || b->data == NULL || sum->data == NULL ||
        a->m != b->m || a->m != sum->m ||
        a->n != b->n || a->n != sum->n)
    {
        return ia_err_argument;
    }
    uint32_t i = 0U;
    uint32_t j = 0U;
    for (i = 0U; i < a->m; i++) {
        for (j = 0U; j < a->n; j++) {
            sum->data[(i * a->n) + j] = a->data[(i * a->n) + j] + b->data[(i * a->n) + j];
        }
    }
    return ia_err_none;
}

static ia_err mat_add_d(ac_matrix_double_t* a, ac_matrix_double_t* b, ac_matrix_double_t* sum)
{
    if (a == NULL || b == NULL || sum == NULL ||
        a->data == NULL || b->data == NULL || sum->data == NULL ||
        a->m != b->m || a->m != sum->m ||
        a->n != b->n || a->n != sum->n)
    {
        return ia_err_argument;
    }
    uint32_t i = 0U;
    uint32_t j = 0U;
    for (i = 0U; i < a->m; i++) {
        for (j = 0U; j < a->n; j++) {
            sum->data[(i * a->n) + j] = a->data[(i * a->n) + j] + b->data[(i * a->n) + j];
        }
    }
    return ia_err_none;
}

/*!
* \brief subtraction for ac_matrix
* \param[in] a                  Mandatory.\n
*                               left operand of matrix subtraction.
* \param[in] b                  Mandatory.\n
*                               right operand of matrix subtraction.
* \param[in] diff                Mandatory.\n
*                               the difference of matrix subtraction.
*/
static ia_err mat_subtraction(ac_matrix_f32_t* a, ac_matrix_f32_t* b, ac_matrix_f32_t* diff)
{
    if (a == NULL || b == NULL || diff == NULL ||
        a->data == NULL || b->data == NULL || diff->data == NULL ||
        a->m != b->m || a->m != diff->m ||
        a->n != b->n || a->n != diff->n)
    {
        return ia_err_argument;
    }
    uint32_t i = 0U;
    uint32_t j = 0U;
    for (i = 0U; i < a->m; i++) {
        for (j = 0U; j < a->n; j++) {
            diff->data[(i * a->n) + j] = a->data[(i * a->n) + j] - b->data[(i * a->n) + j];
        }
    }
    return ia_err_none;
}

static ia_err mat_subtraction_d(ac_matrix_double_t* a, ac_matrix_double_t* b, ac_matrix_double_t* diff)
{
    if (a == NULL || b == NULL || diff == NULL ||
        a->data == NULL || b->data == NULL || diff->data == NULL ||
        a->m != b->m || a->m != diff->m ||
        a->n != b->n || a->n != diff->n)
    {
        return ia_err_argument;
    }
    uint32_t i = 0U;
    uint32_t j = 0U;
    for (i = 0U; i < a->m; i++) {
        for (j = 0U; j < a->n; j++) {
            diff->data[(i * a->n) + j] = a->data[(i * a->n) + j] - b->data[(i * a->n) + j];
        }
    }
    return ia_err_none;
}

/*!
*   \brief element-wise multiplication for ac_matrix
* \param[in] a                  Mandatory.\n
*                               left operand of element-wise multiplication.
* \param[in] b                  Mandatory.\n
*                               right operand of element-wise multiplication.
* \param[in] prod                Mandatory.\n
*                               the element-wise product of matrix a and b.
*/
static ia_err mat_times(ac_matrix_f32_t* a, ac_matrix_f32_t* b, ac_matrix_f32_t* prod)
{
    if (a == NULL || b == NULL || prod == NULL ||
        a->data == NULL || b->data == NULL || prod->data == NULL ||
        a->m != b->m || a->m != prod->m ||
        a->n != b->n || a->n != prod->n)
    {
        return ia_err_argument;
    }
    uint32_t i = 0U;
    for (i = 0U; i < (a->m * a->n); i++)
    {
        prod->data[i] = a->data[i] * b->data[i];
    }
    return ia_err_none;
}

static ia_err mat_times_d(ac_matrix_double_t* a, ac_matrix_double_t* b, ac_matrix_double_t* prod)
{
    if (a == NULL || b == NULL || prod == NULL ||
        a->data == NULL || b->data == NULL || prod->data == NULL ||
        a->m != b->m || a->m != prod->m ||
        a->n != b->n || a->n != prod->n)
    {
        return ia_err_argument;
    }
    uint32_t i = 0U;
    for (i = 0U; i < (a->m * a->n); i++)
    {
        prod->data[i] = a->data[i] * b->data[i];
    }
    return ia_err_none;
}

/*!
*   \brief rdivide (element-wise right divide) for ac_matrix
* \param[in] a                  Mandatory.\n
*                               left operand of division.
* \param[in] b                  Mandatory.\n
*                               right operand of division.
* \param[in] quot                Mandatory.\n
*                               the quotient of division (a./b).
*/
static ia_err mat_rdivide(ac_matrix_f32_t* a, ac_matrix_f32_t* b, ac_matrix_f32_t* quot)
{
    if (a == NULL || b == NULL || quot == NULL ||
        a->data == NULL || b->data == NULL || quot->data == NULL ||
        a->m != b->m || a->m != quot->m ||
        a->n != b->n || a->n != quot->n)
    {
        return ia_err_argument;
    }
    uint32_t i = 0U;
    for (i = 0U; i < (a->m * a->n); i++)
    {
        if (IA_FABS(b->data[i]) < IA_EPSILON)
        {
            return ia_err_data;
        }
        else
        {
            quot->data[i] = a->data[i] / b->data[i];
        }
    }
    return ia_err_none;
}

static ia_err mat_rdivide_d(ac_matrix_double_t* a, ac_matrix_double_t* b, ac_matrix_double_t* quot)
{
    if (a == NULL || b == NULL || quot == NULL ||
        a->data == NULL || b->data == NULL || quot->data == NULL ||
        a->m != b->m || a->m != quot->m ||
        a->n != b->n || a->n != quot->n)
    {
        return ia_err_argument;
    }
    uint32_t i = 0U;
    for (i = 0U; i < (a->m * a->n); i++)
    {
        if (IA_FABS(b->data[i]) < IA_EPSILON)
        {
            return ia_err_data;
        }
        else
        {
            quot->data[i] = a->data[i] / b->data[i];
        }
    }
    return ia_err_none;
}

/*!
 * \brief construce the intrinsic_matrix based on the calib.
 *
 * \param[in] calib                 Mandatory.\n
 *                                  calib data.
 * \param[in] Kc                    Mandatory.\n
 *                                  intrinsic matrix.
 */
void construct_intrinsic_matrix(ac_calibration calib, MAT_D_3_3 kc)
{
    kc[0][0] = calib.focal_length.x;
    kc[0][1] = 0.0;
    kc[0][2] = 0.0;
    kc[1][0] = 0.0;
    kc[1][1] = calib.focal_length.y;
    kc[1][2] = 0.0;
    kc[2][0] = calib.principal_point.x;
    kc[2][1] = calib.principal_point.y;
    kc[2][2] = 1.0;
}

/*!
 * \brief construce the inverse_intrinsic_matrix based on the calib.
 *
 * \param[in] calib                 Mandatory.\n
 *                                  calib data.
 * \param[in] Kc                    Mandatory.\n
 *                                  intrinsic matrix.
 */
void construct_inverse_intrinsic_matrix(ac_calibration calib, MAT_D_3_3 kc)
{
    if ((IA_FABS(calib.focal_length.x) > IA_EPSILON) ||
        (IA_FABS(calib.focal_length.y) > IA_EPSILON)) {
        kc[0][0] = 1.0 / calib.focal_length.x;
        kc[0][1] = 0.0;
        kc[0][2] = 0.0;
        kc[1][0] = 0.0;
        kc[1][1] = 1.0 / calib.focal_length.y;
        kc[1][2] = 0.0;
        kc[2][0] = -calib.principal_point.x / calib.focal_length.x;
        kc[2][1] = -calib.principal_point.y / calib.focal_length.y;
        kc[2][2] = 1.0;
    } else
    {
        kc[0][0] = 1.0;
        kc[0][1] = 0.0;
        kc[0][2] = 0.0;
        kc[1][0] = 0.0;
        kc[1][1] = 1.0;
        kc[1][2] = 0.0;
        kc[2][0] = 0.0;
        kc[2][1] = 0.0;
        kc[2][2] = 1.0;
    }
    
}

/*!
 * \brief convert Quaternion to Euler Angles (z-x'-y'' sequences, refer to 
 *        ia_imaging-ia_ipa/IPUAlgorithms/Pals/ia_pal/srcs/broxton_ia_pal/ia_pal_internal_gdc7_wfov_common.cpp:view_generate_rotation_matrix())
 * \param[in] q             Mandatory.\n
 *                          Quaternion.
 * \param[out] eulerangles  Mandatory.\n
 *                          Euler angles.
 */
ia_err quaternion2Eulerangles(VEC_D_3 q, VEC_D_3 eulerangles)
{
    double q_4 = sqrt(1.0 - q[0] * q[0] - q[1] * q[1] - q[2] * q[2]);

    /* pitch */
    double sinp = 2.0 * (q_4 * q[0] - q[1] * q[2]);
    if (sinp > 1.0)
    {
        /* if sinp is larger than 1, pi/2 is assigned */
        eulerangles[0] = 90.0;
    }
    else if (sinp < -1.0)
    {
        /* if sinp is smaller than 1, pi/2 is assigned */
        eulerangles[0] = 90.0;
    }
    else
    {
        eulerangles[0] = asin(sinp) / M_PI * 180.0;
    }

    /* yaw */
    double siny_sinp = 2.0 * (q_4 * q[1] + q[0] * q[2]);
    double cosy_cosp = 1.0 - 2.0 * (q[0] * q[0] + q[1] * q[1]);
    eulerangles[1] = atan2(siny_sinp, cosy_cosp) / M_PI * 180.0;

    /* roll */
    double sinr_cosp = 2.0 * (q_4 * q[2] + q[0] * q[1]);
    double cosr_cosp = 1.0 - 2.0 * (q[0] * q[0] + q[2] * q[2]);
    eulerangles[2] = atan2(sinr_cosp, cosr_cosp) / M_PI * 180.0;
    return ia_err_none;
}

#ifdef FILE_INJECTION
#include <stdio.h>

#define READ_BUF(src, size, fp, size_of, type, cast_type) do {        \
type tmp;                                                \
uint32_t i = 0U;                                           \
size_t n_mem = 1U;                                         \
for (i = 0U; i < size; i++) {                              \
    size_t ret_len = fread((void*)(&tmp), size_of, n_mem, fp); \
    if (ret_len != n_mem) {                                     \
        AC_LOG("@%s, fread returns size:%d isn't the same with the params", __func__, ret_len); \
    }                                                                                           \
    *(src + i) = (cast_type)tmp;                                                                     \
}                                                                                               \
} while(0);

static ia_err load_data(char* file_name, uint32_t *data_size, uint32_t *nBytes, uint32_t n_mem, ac_file_type_t type, ac_plain_type_t *data_type)
{
    // the file name indicate the data type
    char* dp = "./auto_calib/data/disparity_map_W0256_H0144_int16.dat";
    char* dp_confidence = "./auto_calib/data/disparity_confidence_W0256_H0144_double.dat";
    char* mg_x = "./auto_calib/data/motion_grid_x_W0153_H0091_int16.dat";
    char* mg_y = "./auto_calib/data/motion_grid_y_W0153_H0091_int16.dat";
    char* me = "./auto_calib/data/images_me_left_W0608_H0360_uint8.dat";
    char* indicator = "./auto_calib/data/indicators_L0001.dat";
    char* init_calib_left = "./auto_calib/data/initialize_state__calib_safe_left_L0003.dat";
    char* init_calib_right = "./auto_calib/data/initialize_state__calib_safe_right_L0003.dat";
    char* prod_calib_left = "./auto_calib/data/calib_prod_left_L0003.dat";
    char* prod_calib_right = "./auto_calib/data/calib_prod_right_L0003.dat";
    char* fileName = NULL;
    uint32_t size_of = 0U;
    switch (type) {
        case calib_disparity_map:
            fileName = dp;
            size_of = sizeof(int16_t);
            *data_type = S16_TYPE;
            break;
        case calib_disparity_confidence:
            fileName = dp_confidence;
            size_of = sizeof(double);
            *data_type = D64_TYPE;
            break;
        case calib_mg_x:
            fileName = mg_x;
            size_of = sizeof(int16_t);
            *data_type = S16_TYPE;
            break;
        case calib_mg_y:
            fileName = mg_y;
            size_of = sizeof(int16_t);
            *data_type = S16_TYPE;
            break;
        case calib_me:
            fileName = me;
            size_of = sizeof(uint8_t);
            *data_type = U8_TYPE;
            break;
        case calib_indicator:
            fileName = indicator;
            size_of = sizeof(double);
            *data_type = D64_TYPE;
            break;
        case calib_init_calib_left:
            fileName = init_calib_left;
            size_of = sizeof(double);
            *data_type = D64_TYPE;
            break;
        case calib_init_calib_right:
            fileName = init_calib_right;
            size_of = sizeof(double);
            *data_type = D64_TYPE;
            break;
        case calib_prod_calib_left:
            fileName = prod_calib_left;
            size_of = sizeof(double);
            *data_type = D64_TYPE;
            break;
        case calib_prod_calib_right:
            fileName = prod_calib_right;
            size_of = sizeof(double);
            *data_type = D64_TYPE;
            break;
        default:
            AC_LOG("@%s, not support type:%d", __func__, type);
        }
    if (fileName == NULL) {
        return ia_err_argument;
    }
    file_name = memcpy(file_name, fileName, strlen(fileName));
    *nBytes = size_of;
    FILE *fp = fopen(fileName,"rb");
    if(fp == NULL){
        AC_LOG("@%s, fopen failed", __func__);
        return ia_err_internal;
    }
    fseek(fp, 0, SEEK_END);
    size_t size;
    size = (size_t)ftell(fp);
    fclose(fp);
    uint32_t buf_size = (uint32_t)(n_mem * size_of);
    if (size != buf_size) {
        AC_LOG("@%s, size mismatch(real:%u, buffer_size:%u", __func__, size, buf_size);
    }

    size = IA_MIN(size, buf_size);
    *data_size = size / size_of;
    AC_LOG("@%s, read size:%u unit float", __func__, size);
    return ia_err_none;
}


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
ia_err fill_data_buffer_f32(ac_matrix_f32_t* mat, uint32_t n_mem, ac_file_type_t type)
{
    char file[128] = {0};
    uint32_t size = 0U;
    uint32_t size_of = 0U;
    ac_plain_type_t data_type = INVALID_TYPE;
    ia_err ret = load_data(file, &size, &size_of, n_mem, type, &data_type);
    if (ret != ia_err_none) {
        return ret;
    }

    FILE *fp = fopen(file,"rb");
    switch(data_type) {
        case U8_TYPE:
            READ_BUF(mat->data, size, fp, size_of, uint8_t, float32_t);
            break;
        case S16_TYPE:
            READ_BUF(mat->data, size, fp, size_of, int16_t, float32_t);
            break;
        case D64_TYPE:
            READ_BUF(mat->data, size, fp, size_of, double, float32_t);
            break;
        default:
            AC_LOG("@%s, not support type:%d", __func__, data_type);
    }
    if (fp != NULL) {
        fclose(fp);
    }
    return ia_err_none;
}

ia_err fill_data_buffer_s16(ac_matrix_s16_t* mat, uint32_t n_mem, ac_file_type_t type)
{
    char file[128] = {0};
    uint32_t size = 0U;
    uint32_t size_of = 0U;
    ac_plain_type_t data_type = INVALID_TYPE;
    ia_err ret = load_data(file, &size, &size_of, n_mem, type, &data_type);
    if (ret != ia_err_none) {
        return ret;
    }

    FILE *fp = fopen(file,"rb");
    switch(data_type) {
    case U8_TYPE:
        READ_BUF(mat->data, size, fp, size_of, uint8_t, int16_t);
        break;
    case S16_TYPE:
        READ_BUF(mat->data, size, fp, size_of, int16_t, int16_t);
        break;
    case D64_TYPE:
        READ_BUF(mat->data, size, fp, size_of, double, int16_t);
        break;
    default:
        AC_LOG("@%s, not support type:%d", __func__, data_type);
    }
    if (fp != NULL) {
        fclose(fp);
    }
    return ia_err_none;

    }

ia_err fill_data_buffer_u8(ac_matrix_u8_t* mat, uint32_t n_mem, ac_file_type_t type)
{
    char file[128] = {0};
    uint32_t size = 0U;
    uint32_t size_of = 0U;
    ac_plain_type_t data_type = INVALID_TYPE;
    ia_err ret = load_data(file, &size, &size_of, n_mem, type, &data_type);
    if (ret != ia_err_none) {
        return ret;
    }

    FILE *fp = fopen(file,"rb");
    if (fp == NULL) {
        return ia_err_data;
    }
    switch(data_type) {
    case U8_TYPE:
        READ_BUF(mat->data, size, fp, size_of, uint8_t, uint8_t);
        break;
    default:
        AC_LOG("@%s, not support type:%d", __func__, data_type);
    }
    if (fp != NULL) {
        fclose(fp);
    }
    return ia_err_none;
}

ia_err fill_data_buffer_d64(ac_matrix_double_t* mat, uint32_t n_mem, ac_file_type_t type)
{
    char file[128] = {0};
    uint32_t size = 0U;
    uint32_t size_of = 0U;
    ac_plain_type_t data_type = INVALID_TYPE;
    ia_err ret = load_data(file, &size, &size_of, n_mem, type, &data_type);
    if (ret != ia_err_none) {
        return ret;
    }

    FILE *fp = fopen(file,"rb");
    if (fp == NULL) {
        return ia_err_data;
    }
    switch(data_type) {
    case D64_TYPE:
        READ_BUF(mat->data, size, fp, size_of, double, double);
        break;
    default:
        AC_LOG("@%s, not support type:%d", __func__, data_type);
    }
    if (fp != NULL) {
        fclose(fp);
    }
    return ia_err_none;
}

ia_err dump_data_buffer_f32(char* file_name, ac_matrix_f32_t* mat, uint32_t n_mem)
{
    char file[128] = {0};
    uint32_t size = n_mem;
    uint32_t size_of = sizeof(float32_t);
    memcpy(&file, file_name, 128);
    FILE *fp = fopen(file,"w+");
    size_t ret_len = fwrite((void*)(mat->data), size_of, size, fp);
    if (ret_len != size) {
        AC_LOG("@%s, fwrite returns size:%d isn't the same with the params", __func__, ret_len);
    }

    if (fp != NULL) {
        fclose(fp);
    }
    return ia_err_none;
}

#endif // FILE_INJECTION

#if BITMATCH_PIPE
ia_err bitmatch_value_d(char *bin_file, double* test_value, char *message) {
    double matlab_value = 0.0;
    FILE *bin = fopen(bin_file,"r");
    fseek(bin, 0, SEEK_SET);
    fread(&matlab_value, sizeof(double), 1, bin);
    double distance = fabs(matlab_value - *test_value);
    double err_pro = distance / matlab_value;
    if (err_pro == 0.0) {
        AC_LOG("%s: Fully Bitmatched.", message);
    } else {
        AC_LOG("%s: relative error %lf", message, err_pro);
    }
    fclose(bin);
    return ia_err_none;
}

ia_err bitmatch_value(char *bin_file, float32_t* test_value, char *message) {
    float32_t matlab_value = 0.0;
    FILE *bin = fopen(bin_file,"r");
    fseek(bin, 0, SEEK_SET);
    fread(&matlab_value, sizeof(float32_t), 1, bin);
    float32_t distance = fabs(matlab_value - *test_value);
    float32_t err_pro = distance / matlab_value;
    if (err_pro == 0.0) {
        AC_LOG("%s: Fully Bitmatched.", message);
    } else {
        AC_LOG("%s: relative error %lf", message, err_pro);
    }
    fclose(bin);
    return ia_err_none;
}

ia_err bitmatch_matrix(char *bin_file, const ac_matrix_f32_t* test_mat, int principle_dim, char *message) {
    FILE *bin = fopen(bin_file,"r");
    ac_matrix_f32_t matlab_data = {NULL, test_mat->m, test_mat->n, test_mat->m};
    matlab_data.data = calloc(test_mat->m * test_mat->n, sizeof(float));
    // float max_err = 0.0F;

    if (matlab_data.data != NULL) {
        fseek(bin, 0, SEEK_SET);
        fread(matlab_data.data, matlab_data.n * matlab_data.m * sizeof(float), 1, bin);
        int index = 0;
        int max_index = 0;
        float max_err = 0.0F;
        float ave_err = 0.0F;
        if (principle_dim == 0) {
            for(index = 0; index < (int)matlab_data.m; index++) {
                int i = 0;
                float distance = 0.0F;
                for(i = 0; i < (int)matlab_data.n; i++) {
                    distance += pow((test_mat->data[index * (int32_t)matlab_data.n + i] - matlab_data.data[index * (int32_t)matlab_data.n + i]), 2);
                }
                distance = pow(distance, 0.5F);
                float length = 0.0F;
                for(i = 0; i < (int)matlab_data.n; i++) {
                    length += pow((matlab_data.data[index * (int32_t)matlab_data.n + i]), 2);
                }
                length = pow(length, 0.5F);
                float err_pro = 0.0F;
                if (length == 0.0F && distance == 0.0F)
                {
                    err_pro = 0.0F;
                } else if (length == 0.0F)
                {
                    err_pro = INFINITY;
                } else
                {
                    err_pro = distance / length;
                }
                if (max_err < err_pro) {
                    max_err = err_pro;
                    max_index = index;
                }
                ave_err += err_pro;
            }
            ave_err /= (float)(int)matlab_data.m;
        } else if (principle_dim == 1)  {
            for(index = 0; index < (int)matlab_data.n; index++) {
                int i = 0;
                float distance = 0.0F;
                for(i = 0; i < (int)matlab_data.m; i++) {
                    distance += pow((test_mat->data[index + i * (int32_t)matlab_data.n] - matlab_data.data[index + i * (int32_t)matlab_data.n]), 2);
                }
                distance = pow(distance, 0.5F);
                float length = 0.0F;
                for(i = 0; i < (int)matlab_data.m; i++) {
                    length += pow((matlab_data.data[index + i * (int32_t)matlab_data.n]), 2);
                }
                length = pow(length, 0.5F);
                float err_pro = distance / length;
                if (max_err < err_pro) {
                    max_err = err_pro;
                    max_index = index;
                }
                ave_err += err_pro;
            }
            ave_err /= (float)(int)matlab_data.n;
        } else if (principle_dim == -1)  {
            float distance = 0.0F;
            float length = 0.0F;
            for(index = 0; index < (int)matlab_data.m * (int)matlab_data.n; index++) {
                distance += pow((test_mat->data[index] - matlab_data.data[index]), 2);
                length += pow((matlab_data.data[index]), 2);
            }
            distance = pow(distance, 0.5F);
            length = pow(length, 0.5F);
            ave_err = distance / length;
            max_index = -1;
            max_err = ave_err;
        }
        if (ave_err == 0.0F) {
            AC_LOG("%s: Fully Bitmatched.", message);
        } else {
            AC_LOG("%s: average relative error %E, %d-th sample has max relative error %E", message, ave_err, max_index, max_err);
        }
    } else {
        AC_LOG("CANNOT compare!");
        return ia_err_argument;
    }
    fclose(bin);
    return ia_err_none;
}

ia_err bitmatch_tensor_3(char *bin_file, const ac_3_tensor_t* test_mat, int principle_dim, char *message) {
    FILE *bin = fopen(bin_file,"r");
    ac_3_tensor_t matlab_data = {NULL, test_mat->m, test_mat->n, test_mat->k};
    matlab_data.data = calloc(test_mat->m * test_mat->n * test_mat->k, sizeof(float));

    // float max_err = 0.0F;

    if (matlab_data.data != NULL) {
        fseek(bin, 0, SEEK_SET);
        fread(matlab_data.data, matlab_data.n * matlab_data.m * matlab_data.k * sizeof(float), 1, bin);
        int index = 0;
        int max_index = 0;
        float max_err = 0.0F;
        float ave_err = 0.0F;
        if (principle_dim == 0) {
            for(index = 0; index < (int)matlab_data.m; index++) {
                int i = 0;
                float distance = 0.0F;
                for(i = 0; i < (int)matlab_data.n; i++) {
                    int j = 0;
                    for(j = 0; j < (int)matlab_data.k; j++) {
                        distance += pow((test_mat->data[index * (int32_t)matlab_data.n * (int32_t)matlab_data.k + i * (int32_t)matlab_data.k + j] - matlab_data.data[index * (int32_t)matlab_data.n * (int32_t)matlab_data.k + i * (int32_t)matlab_data.k + j]), 2);
                    }
                }
                distance = pow(distance, 0.5F);
                float length = 0.0F;
                for(i = 0; i < (int)matlab_data.n; i++) {
                    int j = 0;
                    for(j = 0; j < (int)matlab_data.k; j++) {
                        length += pow((matlab_data.data[index * (int32_t)matlab_data.n * (int32_t)matlab_data.k + i * (int32_t)matlab_data.k + j]), 2);
                    }
                }
                length = pow(length, 0.5F);
                float err_pro = distance / length;
                if (max_err < err_pro) {
                    max_err = err_pro;
                    max_index = index;
                }
                ave_err += err_pro;
            }
            ave_err /= (float)(int)matlab_data.m;
        } else if (principle_dim == 1) {
            for(index = 0; index < (int)matlab_data.n; index++) {
                int i = 0;
                float distance = 0.0F;
                for(i = 0; i < (int)matlab_data.m; i++) {
                    int j = 0;
                    for(j = 0; j < (int)matlab_data.k; j++) {
                        distance += pow((test_mat->data[i * (int32_t)matlab_data.n * (int32_t)matlab_data.k + index * (int32_t)matlab_data.k + j] - matlab_data.data[i *(int32_t) matlab_data.n * (int32_t)matlab_data.k + index * (int32_t)matlab_data.k + j]), 2);
                    }
                }
                distance = pow(distance, 0.5F);
                float length = 0.0F;
                for(i = 0; i < (int)matlab_data.m; i++) {
                    int j = 0;
                    for(j = 0; j < (int)matlab_data.k; j++) {
                        length += pow((matlab_data.data[i * (int32_t)matlab_data.n *(int32_t)matlab_data.k + index * (int32_t)matlab_data.k + j]), 2);
                    }
                }
                length = pow(length, 0.5F);
                float err_pro = distance / length;
                if (max_err < err_pro) {
                    max_err = err_pro;
                    max_index = index;
                }
                ave_err += err_pro;
            }
            ave_err /= (float)(int)matlab_data.n;
        } else if (principle_dim == 2) {
            for(index = 0; index < (int)matlab_data.k; index++) {
                int i = 0;
                float distance = 0.0F;
                for(i = 0; i < (int)matlab_data.m; i++) {
                    int j = 0;
                    for(j = 0; j < (int)matlab_data.n; j++) {
                        distance += pow((test_mat->data[i * (int32_t)matlab_data.n * (int32_t)matlab_data.k + j * (int32_t)matlab_data.k + index] - matlab_data.data[i * (int32_t)matlab_data.n * (int32_t)matlab_data.k + j *(int32_t) matlab_data.k + index]), 2);
                    }
                }
                distance = pow(distance, 0.5F);
                float length = 0.0F;
                for(i = 0; i < (int)matlab_data.m; i++) {
                    int j = 0;
                    for(j = 0; j < (int)matlab_data.n; j++) {
                        length += pow((matlab_data.data[i * (int32_t)matlab_data.n * (int32_t)matlab_data.k + j *(int32_t) matlab_data.k + index]), 2);
                    }
                }
                length = pow(length, 0.5F);
                float err_pro = distance / length;
                if (max_err < err_pro) {
                    max_err = err_pro;
                    max_index = index;
                }
                ave_err += err_pro;
            }
            ave_err /= (float)(int)matlab_data.k;
        }
        if (ave_err == 0.0F) {
            AC_LOG("%s: Fully Bitmatched.", message);
        } else {
            AC_LOG("%s: average relative error %f, %d-th sample has max relative error %f", message, ave_err, max_index, max_err);
        }
    } else {
        AC_LOG("CANNOT compare!");
        return ia_err_argument;
    }
    fclose(bin);
    return ia_err_none;
}

ia_err bitmatch_matrix_double(char *bin_file, const ac_matrix_double_t* test_mat, int principle_dim, char *message) {
    FILE *bin = fopen(bin_file,"r");
    ac_matrix_double_t matlab_data = {NULL, test_mat->m, test_mat->n, test_mat->m};
    matlab_data.data = calloc(test_mat->m * test_mat->n, sizeof(double));
    // float max_err = 0.0F;

    if (matlab_data.data != NULL) {
        fseek(bin, 0, SEEK_SET);
        fread(matlab_data.data, matlab_data.n * matlab_data.m * sizeof(double), 1, bin);
        int index = 0;
        int max_index = 0;
        double max_err = 0.0;
        double ave_err = 0.0;
        if (principle_dim == 0) {
            for(index = 0; index < (int)matlab_data.m; index++) {
                int i = 0;
                double distance = 0.0;
                for(i = 0; i < (int)matlab_data.n; i++) {
                    distance += pow((test_mat->data[index * (int32_t)matlab_data.n + i] - matlab_data.data[index * (int32_t)matlab_data.n + i]), 2);
                }
                distance = pow(distance, 0.5);
                double length = 0.0;
                for(i = 0; i < (int)matlab_data.n; i++) {
                    length += pow((matlab_data.data[index *(int32_t) matlab_data.n + i]), 2);
                }
                length = pow(length, 0.5);
                float err_pro = distance / length;
                if (max_err < err_pro) {
                    max_err = err_pro;
                    max_index = index;
                }
                ave_err += err_pro;
            }
            ave_err /= (double)(int)matlab_data.m;
        } else if (principle_dim == 1)  {
            for(index = 0; index < (int)matlab_data.n; index++) {
                int i = 0;
                double distance = 0.0;
                for(i = 0; i < (int)matlab_data.m; i++) {
                    distance += pow((test_mat->data[index + i * (int32_t)matlab_data.n] - matlab_data.data[index + i * (int32_t)matlab_data.n]), 2);
                }
                distance = pow(distance, 0.5);
                double length = 0.0;
                for(i = 0; i < (int)matlab_data.m; i++) {
                    length += pow((matlab_data.data[index + i * (int32_t)matlab_data.n]), 2);
                }
                length = pow(length, 0.5);
                double err_pro = distance / length;
                if (max_err < err_pro) {
                    max_err = err_pro;
                    max_index = index;
                }
                ave_err += err_pro;
            }
            ave_err /= (double)(int)matlab_data.n;
        } else if (principle_dim == -1)  {
            double distance = 0.0;
            double length = 0.0;
            for(index = 0; index < (int)matlab_data.m * (int)matlab_data.n; index++) {
                distance += pow((test_mat->data[index] - matlab_data.data[index]), 2);
                length += pow((matlab_data.data[index]), 2);
            }
            distance = pow(distance, 0.5);
            length = pow(length, 0.5);
            ave_err = distance / length;
            max_index = -1;
            max_err = ave_err;
        }
        if (ave_err == 0.0F) {
            AC_LOG("%s: Fully Bitmatched.", message);
        } else {
            AC_LOG("%s: average relative error %lf, %d-th sample has max relative error %lf", message, ave_err, max_index, max_err);
        }
    } else {
        AC_LOG("CANNOT compare!");
        return ia_err_argument;
    }
    fclose(bin);
    return ia_err_none;
}

ia_err inject_binary_data(void *dst, size_t size, char *src_file)
{
    if (dst == NULL || src_file == NULL) {
        return ia_err_argument;
    }
    FILE *binary_file = fopen(src_file,"r");
    if (binary_file == NULL) {
        AC_LOG("Cannot open %s", src_file);
        return ia_err_data;
    }
    fseek(binary_file, 0, SEEK_END);
    size_t file_size= (size_t)ftell (binary_file);
    if(file_size < size) {
        AC_LOG("Size of %s is too small.", src_file);
        return ia_err_data;
    }
    AC_LOG("Inject data from %s", src_file);
    fseek(binary_file, 0, SEEK_SET);
    fread(dst, size, 1, binary_file);
    fclose(binary_file);
    return ia_err_none;
}

/* tool function for export float32_t matrix for bitmatch */
ia_err export_matrix_data(char *bin_file, const ac_matrix_f32_t *mat)
{
    FILE *bin = fopen(bin_file, "w");
    if (bin != NULL)
    {
        fseek(bin, 0, SEEK_SET);
        size_t data_size = mat->m * mat->n * sizeof(*mat->data);
        fwrite(mat->data, data_size, 1, bin);
        fclose(bin);
    }
    return ia_err_none;
}
#endif
