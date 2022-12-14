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

#ifndef AUTO_CALIB_INTERNAL_H_
#define AUTO_CALIB_INTERNAL_H_

#include "ia_types.h"
// #include "kalmanfilter.h"
#include "auto_calib_types.h"
#include "ia_log.h"
#include "ac_mem_allocate.h"
#ifdef INPUTS_IN_FILE
#include "ac_filedebug.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if defined _DEBUG || defined LOG_AC
    #define AC_DEBUG
    #define AC_LOG(fmt, ...)  IA_LOG(ia_log_debug, "AC: " fmt, ## __VA_ARGS__)
#else
    #define AC_LOG       IA_NOLOG
#endif

/*!
 *  \brief m x n matrix.
 */
typedef struct
{
    float32_t *data;    /*!< data of matrix*/
    uint32_t m;         /*!< number of row*/
    uint32_t n;         /*!< number of col*/
    uint32_t s;         /*!< number of stride*/
} ac_matrix_f32_t;

/*!
 *  \brief m x n matrix.
 */
typedef struct
{
    uint8_t *data;      /*!< data of matrix*/
    uint32_t m;         /*!< number of row*/
    uint32_t n;         /*!< number of col*/
    uint32_t s;         /*!< number of stride*/
} ac_matrix_u8_t;

/*!
 *  \brief m x n matrix.
 */
typedef struct
{
    double *data;       /*!< data of matrix*/
    uint32_t m;         /*!< number of row*/
    uint32_t n;         /*!< number of col*/
    uint32_t s;         /*!< number of stride*/
} ac_matrix_double_t;

/*!
 *  \brief m x n matrix.
 */
typedef struct
{
    uint16_t *data;     /*!< data of matrix*/
    uint32_t m;         /*!< number of row*/
    uint32_t n;         /*!< number of col*/
    uint32_t s;         /*!< number of stride*/
} ac_matrix_u16_t;

/*!
 *  \brief m x n matrix.
 */
typedef struct
{
    int16_t *data;      /*!< data of matrix*/
    uint32_t m;         /*!< number of row*/
    uint32_t n;         /*!< number of col*/
    uint32_t s;         /*!< number of stride*/
} ac_matrix_s16_t;

/*!
 *  \brief m x n matrix.
 */
typedef struct
{
    uint32_t *data;     /*!< data of matrix*/
    uint32_t m;         /*!< number of row*/
    uint32_t n;         /*!< number of col*/
    uint32_t s;         /*!< number of stride*/
} ac_matrix_u32_t;

/*!
 *  \brief m x n matrix.
 */
typedef struct
{
    uint64_t *data;     /*!< data of matrix*/
    uint32_t m;         /*!< number of row*/
    uint32_t n;         /*!< number of col*/
    uint32_t s;         /*!< number of stride*/
} ac_matrix_u64_t;

/*!
 *  \brief m x n matrix.
 */
typedef struct
{
    int64_t *data;     /*!< data of matrix*/
    uint32_t m;         /*!< number of row*/
    uint32_t n;         /*!< number of col*/
    uint32_t s;         /*!< number of stride*/
} ac_matrix_s64_t;

/*!
 *  \brief m x n x k tensor.
 */
typedef struct
{
    float32_t *data;    /*!< data of matrix*/
    uint32_t m;         /*!< number of dimension 1*/
    uint32_t n;         /*!< number of dimension 2*/
    uint32_t k;         /*!< number of dimension 3*/
} ac_3_tensor_t;

/*!
 *  \brief vector.
 */
typedef struct
{
    float32_t *data;   /*!< data of vector*/
    uint32_t len;      /*!< length of vector*/
} ac_vector_t;

/*!
 *  \brief range.
 */
typedef struct
{
    uint32_t w;         /*!< width of grid/image*/
    uint32_t h;         /*!< height of grid/image*/
} ac_grid_size_t;

/*!
 *  \brief range in double.
 */
typedef struct
{
    double w;         /*!< crop in width/width of image*/
    double h;         /*!< crop in height/height of image*/
} ac_crop_d_t, ac_image_size_d_t;

/*!
 *  \brief calibration type.
 */
typedef enum
{
    calib_stage_tuning = 0,          /*!< tuned calib*/
    calib_stage_predict,             /*!< predicted calib*/
    calib_stage_analyze_scene,       /*!< analyze the left image*/
    calib_stage_refinement,          /*!< update calib by refinement*/
    calib_stage_match,               /*!< calib with match*/
    calib_stage_ff,                  /*!< calib with fill factor*/
    calib_stage_depth,               /*!< calib with depth*/
    calib_stage_max,
} ac_calib_stage_t;

/*!
 *  \brief calibration stage in calib by depth.
 */
typedef enum
{
    calib_depth_init = 0,         /*!< stage 1 where initilzed in calib by depth */
    calib_depth_ff_L,             /*!< stage 2 where set fill factor for max left degree */
    calib_depth_ff_R,             /*!< stage 3 whereset fill factor for max right degree */
    calib_depth_tune_L,           /*!< stage 4 where iterativly updating tuned degree for left range */
    calib_depth_tune_R,           /*!< stage 5 where iterativly updating tuned degree for right range */
    calib_depth_tune,             /*!< stage 6 where calculate tuned degree */
    calib_depth_completed,        /*!< stage 7 where calib by match ends */
} ac_calib_depth_stage_t;

/*!
 *  \brief matrix direction.
 */
typedef enum
{
    horizontal = 0,
    vertical,
    direction_max,
} ac_direction_t;

/*!
 *  \brief camera type.
 */
typedef enum
{
    left_cam = 0,
    right_cam,
    max_num_cam,
} ac_cam_type_t;

/*!
 * \brief AC indicator structure.
 */
typedef struct
{
    double global_contrast;                  /*!< global constrast of disparity*/
    double local_contrast;                   /*!< mean of subblock constrast of disparity*/
    double disparity_median;                 /*!< median of disparity*/
    double disparity_std;                    /*!< std in disparity*/
    double planarity;                        /*!< planarity in disparity*/
    double multi_planarity;                  /*!< planarity in different layer of disparity*/
    double multi_planarity_inlier;           /*!< inlier in different layer of disparity*/
    float fill_factor0;                     /*!< fill factor from SDP for init calib_ff */
    float fill_factor1;                     /*!< fill factor from SDP for tuned calib_ff */
    double inliers_ratio;                    /*!< sum of is_inlier divide length of is_inlier */
    float motion_grad_norm_max;             /*!< max of normalizd motion grad */
    float motion_grad_norm_mean;            /*!< mean of normalizd motion grad */
    float motion_grad_norm_90;              /*!< 90 perc of normalizd motion grad */
} ac_indicator;

/*!
 * \brief structure for project_points_local()
 */
typedef struct
{
    ac_matrix_f32_t XYZ_c_mat;
} proj_point_local_t;

/*!
 * \brief matches structure.
 */
typedef struct
{
    ac_matrix_f32_t saliency;                                /*!< saliency matrix N*1 */
    ac_matrix_f32_t motion_xy_grad_norm;                     /*!< xy direction flat normlized the motion, size n * 2 */
} ac_matches;

/*!
 * \brief Reocord necessary data structure for calib by depth
 */

typedef struct
{
    ac_calib_depth_stage_t calib_depth_stage;

    float ff_left;
    float ff_right;
    float ff_tuned;
    float ff_target;
    double qx_left;
    double qx_right;
    double qx_tuned;
    double qx_err_max;

    double qx_tuned_r;
    double qx_tuned_l;

    /* stored middle values of qx during binary search
    [0]: left data
    [1]: tuned data
    [2]: right data */
    double qx_binary_search[3];
    float32_t ff_binary_search[3];

    int counter_iter;
} ac_depth_t;

/*!
 * \brief model structure.
 */
typedef struct
{
    VEC_D_3 R21_vec;                           /*!< Rotation vector*/
    MAT_D_3_3 R21;                             /*!< Rotation matrix*/
    VEC_D_3 T21;                               /*!< Translation vector*/
    MAT_D_3_3 E21;                             /*!< Essential matrix determined by extrinsic params*/
    double FG1;
    double FG2;
    uint32_t valid_FG1;
    uint32_t valid_FG2;
} ac_model;

/*!
 * \brief AC internal structure.
 */
typedef struct _ac_t
{
    uint64_t frame_id;                            /*!< frame ID for current iteration*/
    ac_mem_t ac_mem_allocator;                    /*!< manage memory allocation for large size variable in auto calibration*/
    ac_matrix_u8_t disparity_confidence;          /*!< disparity map confidence (0..1) from SDP*/
    ac_image_size_t valid_disparity_size;         /*!< size of disparity with valid confidence*/
    ac_image_size_t image_inp_size;               /*!< [height, width] of image inputform input system. */
    ac_matrix_s16_t match_tmp;                    /*!< tmp buffer for match calculation*/
    ac_matrix_f32_t grad_norm[direction_max];         /*!< max gradent in xy direction for both left and right images*/
    ac_matrix_u16_t tmp_mat[2];                   /*!< temp mat for calulation, used for fit_plane(x/y/z)*/
    ac_grid_size_t grid_size;                     /*!< grid size*/
    uint32_t max_disparity_pixels;                /*!< max disparity pixel num */
    ac_matches matches;                           /*!< structs for matches*/
    ac_matrix_f32_t block_contrast;                   /*!< constrast for subblock of left image*/
    ac_calibration_state calib[calib_stage_max];  /*!< calibrations for different stage*/
    ac_calibration depth_out;                     /*!< tuned calibration for depth output*/
    ac_calibration depth_out_720;                 /*!< tuned calibration for depth output adapted to resolution 1280x720 */
    ac_indicator indicator;                       /*!< indicator for auto calib result*/
    ac_depth_t calib_depth;                       /*!< reocord necessary data for calib by depth */
    kalman_pars_t kalman_state;                   /*!< store kalman filter state */
#ifdef INPUTS_IN_FILE
    ac_debug_t m_debug;
#endif
} ac_t;

#ifdef __cplusplus
}
#endif

#endif // AUTO_CALIB_INTERNAL_H_
