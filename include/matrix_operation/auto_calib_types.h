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

#ifndef AUTO_CALIB_TYPES_H_
#define AUTO_CALIB_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

#define KALMAN_FILTER_SIZE 128

#define VEC_DIM 3
typedef float32_t MAT_3_3[3][3];
typedef float32_t VEC_3[3];
typedef double MAT_D_3_3[3][3];
typedef double VEC_D_3[3];
typedef double VEC_D_2[2];

typedef struct _ac_t ac_t;

/*!
 *  \brief buffer type.
 */
typedef enum
{
    buf_type_disparity = 0,         /*!< disparity map*/
    buf_type_depth,                 /*!< depth map*/
    buf_type_me,                    /*!< motion estimation*/
    buf_type_mg,                    /*!< motion grid*/
} ac_buf_type_t;

/*!
 *  \brief image side.
 */
typedef enum
{
    ac_image_left = 0,              /*!< indict image side is left */
    ac_image_right,                 /*!< indict image side is right */
} ac_image_side_t;

/*!
 *  \brief REC Type.
 */
typedef enum
{
    ac_rec_disp = 0,                /*!< indict rectification is for disparity */
    ac_rec_ff_depth_trial,          /*!< indict rectification is for fill factor for estimate by depth */
} ac_rec_type_t;

/*!
 *  \brief image buffer property structure.
 */
typedef struct
{
    uint32_t size;                  /*!< buffer size. */
    ac_buf_type_t type;             /*!< buffer type. */
    uint32_t width;                 /*!< image width. */
    uint32_t height;                /*!< image height. */
    uint32_t stride;                /*!< image stride. */
    uint32_t bpp;                   /*!< bit per pixel. */
} ac_buf_prop;

/*!
 *  \brief image buffer parameter structure.
 */
typedef struct
{
    uint64_t frame_id;              /*!< frame id. */
    ac_buf_prop prop;               /*!< buffer property. */
    void *data;                     /*!< buffer data. */
} ac_buf;

/*!
 *  \brief 2D point/scale ratio defintion.
 */
typedef struct
{
    double x;                       /*!< horizontal. */
    double y;                       /*!< vertical. */
} ac_point, ac_scale_ratio;

/*!
 *  \brief range.
 */
typedef struct
{
    uint32_t w;         /*!< width of image/crop in width*/
    uint32_t h;         /*!< height of image/crop in height*/
} ac_image_size_t, ac_crop_t;

/*!
 *  \brief calibration structure.
 */
typedef struct
{
    ac_point focal_length;              /*!< focal_length from tuning. */
    ac_point principal_point;           /*!< center of image. */
    VEC_D_3 translation_vec;            /*!< translation_vec between 2 sensors. */
    VEC_D_3 rotation_vec;               /*!< rotation_vec between 2 sensors. */
    double focal_change;                /*!< focal change value. */
    VEC_D_3 radial_distortion;          /*!< radial distortion. */
    VEC_D_2 tangential_distortion;      /*!< tangential distortion. */
} ac_calibration;

/*!
 *  \brief calibration state structure.
 */
typedef struct
{
    uint32_t baseline;                  /*!< distance between 2 sensors. */
    ac_calibration left;                /*!< left sensors. */
    ac_calibration right;               /*!< right sensors. */
} ac_calibration_state;

/*!
 *  \brief input parameter structure of auto calibration.
 */
typedef struct
{
    ac_calibration_state calib;            /*!< init right/left calbration. */
    ac_calibration depth_out;              /*!< SDP default calibration. */
    ac_buf_prop disparity_prop;            /*!< property of disparity. */
    ac_buf_prop me_prop;                   /*!< property of motion estimation. */
    ac_buf_prop mg_prop;                   /*!< property of motion grid. */
    ia_binary_data kalman_state;           /*!< kalman filter state. */
    ac_image_size_t image_input_size;      /*!< [height, width] of image inputform input system. */
    MAT_D_3_3 rotation_left_mat;           /*!< Compensation for left sensor's rotation. */
    MAT_D_3_3 rotation_right_mat;          /*!< Compensation for right sensor's rotation. */
} ac_init_params;

/*!
 *  \brief RealSense format intrinsic & extrinisic parameters for REC(Rectification).
 */
typedef struct
{
    MAT_D_3_3 rotation_matrix;  /* rotation matrix */
    float64_t k[5];             /* distortion parameters */
    float64_t fx;               /* focal length in x-axis */
    float64_t fy;               /* focal length in y-axis */
    float64_t px;               /* x coordination of principle point */
    float64_t py;               /* y coordination of principle point */
} rec_params;

/*!
 * \brief intrinsics of the camera including 9 parameters
 */
typedef struct
{
    float32_t focal[2]; /* Two focal parameters */
    float32_t center[2]; /* Two center parameters */
    float32_t distortion[5];  /* Five distortion parameters */
} intrinsics;

/**
 * \brief Rec mode coefficients including 4 parameters
 */
typedef struct {
    float32_t focal[2]; /* Two focal parameters */
    float32_t center[2]; /* Two center parameters */
} calibrationrectparams;

/**
 * \brief Coefficients Table data structure
 */
typedef struct {
    intrinsics Intrinsic_left; /* Left intrinsics */
    intrinsics Intrinsic_right; /* Right intrinsics */
    MAT_3_3 rotation_left; /* Rotation matrix left */
    MAT_3_3 rotation_right; /* Rotation matrix right */
    float32_t baseline; /* Baseline: mm, in negative value */
    uint32_t usebrownmodel; /* Use Brown model */
    uint8_t reserved1[88]; /* Reserved */
    calibrationrectparams rec_modes[13]; /* Rec modes for specific resolutions */
    uint16_t width; /* Calibration width */
    uint16_t height; /* Calibration height */
    uint8_t reserved2[44]; /* Reserved */
} coeffparameters;

#ifdef __cplusplus
}
#endif

#endif // AUTO_CALIB_TYPES_H_
