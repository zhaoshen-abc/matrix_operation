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

/* THIS FILE IS AUTOMATICALLT GENERATED. PLEASE NOT MODIFY THIS FILE MANUALLY!!!!!! */

#ifndef AC_MEM_ALLOCATE_H_
#define AC_MEM_ALLOCATE_H_

#include "ia_types.h"
#include "ia_abstraction.h"

/* memory offset for each large size variables*/
#define DISPARITY_CONFIDENCE_OFFSET 0U
#define MATCH_TMP_OFFSET 48768U
#define GRID_NORM_H_OFFSET 48768U
#define GRID_NORM_V_OFFSET 104460U
#define TMP_0_OFFSET 64000U
#define TMP_1_OFFSET 192000U
#define BASE_POINTS_OFFSET 16256U
#define DEST_POINTS_OFFSET 0U
#define SALIENCY_OFFSET 48768U
#define TMP_2D_0_OFFSET 73152U
#define TMP_2D_1_OFFSET 56896U
#define M_XY_GRAD_NORM_OFFSET 32512U
#define XYZT_TEMP_OFFSET 89408U
#define IS_INLINE_OFFSET 48768U
#define IS_INLINER_TMP_OFFSET 113792U
#define XS_INIT_OFFSET 48768U
#define F_MIN_OFFSET 72768U
#define JV_OFFSET 176768U
#define JS_OFFSET 248768U
#define F_SHIFT_V_OFFSET 240768U
#define F_SHIFT_S_OFFSET 104768U
#define XS_SHIFT_OFFSET 136768U
#define XS_SHIFT_VEC_OFFSET 344768U
#define GS_OFFSET 104768U
#define JJVS_OFFSET 128768U
#define JJSS_OFFSET 176768U
#define XS_TRIAL_OFFSET 248768U
#define F_TRIAL_OFFSET 272768U
#define WSS_INV_OFFSET 304768U
#define WSS_DET_OFFSET 376768U
#define TVS_OFFSET 400768U
#define BS_OFFSET 448768U
#define DXS_OFFSET 376768U
#define ROTATED_BASE_POINTS_OFFSET 121920U
#define ROTATED_DEST_POINTS_OFFSET 138176U
#define FIT_CALIB_LINEAR_TEMP_OFFSET 154432U

/* maximum number of large size variables in the same time */
#define MAX_COCURRENT_OFFSET_NUM 14U
/* total memory usage in byte */
#define LARGE_VAR_MEMORY_USAGE 472768U

/*!
 *  \brief large size variable information.
 */
typedef struct
{
    bool in_use;        /*!< indictor whether the information data block is used currently */
    size_t offset;      /*!< offset for the large size variable for start address of the space allocate for large size variables share */
    size_t size;        /*!< size of the large size variable */
} var_info_t;

/*!
 *  \brief memory manager for large size variable.
 */
typedef struct
{
    void *start_ptr;                                    /*!< start address of the space allocate for large size variables share */
    size_t total_size;                                  /*!< total memory usage in byte */
    var_info_t var_info[MAX_COCURRENT_OFFSET_NUM];      /*!< large size variable information for current use */
    uint32_t var_in_use_counter;                        /*!< number of large size variables in used */
} ac_mem_t;

/*!
 * \brief Initialize Memory Allocator for Large-size Vaiables in Auto Calib (AC)
 *
 * \param[in] ac_mem_alloc_ptr               Mandatory.\n
 *                                           memory allocator handler for large size variable in auto-calib.
 * \param[in] total_size                     Mandatory.\n
 *                                           total memory usage in byte.
 */
ia_err ac_mem_init(ac_mem_t *ac_mem_alloc_ptr, size_t total_size);

/*!
 * \brief Reset status of Memory Allocator for Large-size Vaiables in Auto Calib (AC)
 *
 * \param[in] ac_mem_alloc_ptr               Mandatory.\n
 *                                           memory allocator handler for large size variable in auto-calib.
 * \param[in] total_size                     Mandatory.\n
 *                                           total memory usage in byte.
 */
ia_err ac_mem_reset(ac_mem_t *ac_mem_alloc_ptr);

/*!
 * \brief  Allocate a large size variable in Auto Calib (AC)
 *
 * \param[in] ac_mem_alloc_ptr               Mandatory.\n
 *                                           memory allocator handler for large size variable in auto-calib.
 * \param[in] var_size                       Mandatory.\n
 *                                           size of the variable.
 * \param[in] var_ptr                        Mandatory.\n
 *                                           pointer of the variable.
 * \param[in] offset                         Mandatory.\n
 *                                           offset of the variable.
 */
ia_err ac_mem_alloc_var(ac_mem_t *ac_mem_alloc_ptr, size_t var_size, void **var_ptr, size_t offset);

/*!
 * \brief  Allocate a large size variable in Auto Calib (AC)
 *
 * \param[in] ac_mem_alloc_ptr               Mandatory.\n
 *                                           memory allocator handler for large size variable in auto-calib.
 * \param[in] free_var                       Mandatory.\n
 *                                           pointer of the variable to be freed.
 */
ia_err ac_mem_free_var(ac_mem_t *ac_mem_alloc_ptr, void **free_var);

/*!
 * \brief Deinitialize Memory Allocator for Large-size Vaiables in Auto Calib (AC)
 *
 * \param[in] ac_mem_alloc_ptr               Mandatory.\n
 *                                           memory allocator handler for large size variable in auto-calib.
 */
ia_err ac_mem_deinit(ac_mem_t *ac_mem_alloc_ptr);

#endif // AC_MEM_ALLOCATE_H_