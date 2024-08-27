#ifndef _GEOMETRY_H
#define _GEOMETRY_H

#include <math.h>
#include "macro.h"
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif
  typedef double MAT_D_3_3[3][3];
  typedef double POSE_D[3][4];
  typedef double VEC_D_3[3];
  typedef double VEC_D_4[4];
  typedef double VEC_D_6[6]; // [x y z rx ry rz]
  typedef double Quaternion_D[4];

  typedef struct MAT_DYNAMIC_D {
    double** p;
    uint32_t rows;
    uint32_t cols;
  } MAT_DYNAMIC_D;

  typedef struct SVD_DYNAMIC_D {
    MAT_DYNAMIC_D U; 
    MAT_DYNAMIC_D D;
    MAT_DYNAMIC_D V; // should not be used
  } SVD_DYNAMIC_D;

  void NEW_MAT_DYNAMIC_D(MAT_DYNAMIC_D* mat, const uint32_t rows, const uint32_t cols);
  void FREE_MAT_DYNAMIC_D(MAT_DYNAMIC_D* mat);
  void NEW_SVD_DYNAMIC_D(SVD_DYNAMIC_D* svd, const uint32_t mat_rows, const uint32_t mat_cols);
  void FREE_SVD_DYNAMIC_D(SVD_DYNAMIC_D* svd);
  void SET_ZERO_MAT_DYNAMIC_D(MAT_DYNAMIC_D* mat);
  void COPY_MAT_DYNAMIC_D(MAT_DYNAMIC_D* src, MAT_DYNAMIC_D* dest);
  void ATA_MAT_DYNAMIC_D(MAT_DYNAMIC_D* A, MAT_DYNAMIC_D* ATA);
  void TRANSPOSE_MAT_DYNAMIC_D(MAT_DYNAMIC_D* A, MAT_DYNAMIC_D* AT);

  // compute SVD decomposition for mat, saved in svd
  // mat = U*D*V^T
  void SVD_MAT_DYNAMIC_D(MAT_DYNAMIC_D* mat, SVD_DYNAMIC_D* svd);

  // solve linear equation Ax=b
  void SOLVE_A_x_b_MAT_by_colPivHouseholderQr_MAT_DYNAMIC_D(MAT_DYNAMIC_D* A, MAT_DYNAMIC_D* b, MAT_DYNAMIC_D* x);
  // x = A.colPivHouseholderQr().solve(b)
  void SOLVE_A_x_b_MAT_by_SVD_MAT_DYNAMIC_D(MAT_DYNAMIC_D* A, MAT_DYNAMIC_D* b, MAT_DYNAMIC_D* x);
  // Solver
  // x = A.SVD().solve(b)


  /*---------------New Add function-------------------*/
  // v = [0, 0, 0]^T
  void SET_ZERO_VEC_D_3(VEC_D_3 v);

  // v = v * n
  void MULTIPLY_SCALE_VEC_D_3(VEC_D_3 v, double n);

  // m = v * v^T
  void AAT_VEC_D_3(VEC_D_3 v, MAT_D_3_3 m);

  // m = U*D*V^T
  void SVD_MAT_D_3_3(MAT_D_3_3 m, MAT_D_3_3 U, MAT_D_3_3 V, MAT_D_3_3 D);

  // m = m^T inplace
  void TRANSPOSE_MAT_D_3_3(MAT_D_3_3 m);

  // return rank of m
  int RANK_MAT_D_3_3(MAT_D_3_3 m);

  // inv_m = m^-1
  void INVERSE_MAT_D_3_3(MAT_D_3_3 m, MAT_D_3_3 inv_m);

  // return v1^T * v2
  double DOT_PRODUCT_VEC_D_3(VEC_D_3 v1, VEC_D_3 v2);
  
  // return || v1 - v2 ||^2
  double SQUARED_L2_DIST_VEC_D_3(VEC_D_3 v1, VEC_D_3 v2);

  // return || p ||^2
  double squaredNorm_VEC_D_3(VEC_D_3 p);

  // m = [0, 0, 0; 0, 0, 0; 0, 0, 0;];
  void SET_ZERO_MAT_D_3_3(MAT_D_3_3 m);

  // return |m|
  double determinant_MAT_D_3_3(MAT_D_3_3 m);

  int Minor(MAT_D_3_3 arr1,int i,int n);

  // m_out = m_in
  void EQU_MAT_D_3_3(MAT_D_3_3 m_in, MAT_D_3_3 m_out);

  /*-------------New Add function End-----------------*/
















  void ac_MAT_D_3_3_print(const MAT_D_3_3 mat);

  void ac_POSE_D_print(const POSE_D mat);

  uint32_t copy_VEC_D_3(const VEC_D_3 v_in, VEC_D_3 v_out);

  uint32_t copy_Mat33d(const MAT_D_3_3 mat_in, MAT_D_3_3 mat_out);

  uint32_t transpose_Mat33d(const MAT_D_3_3 mat_in, MAT_D_3_3 mat_out);

  uint32_t transpose_Mat33d_inplace(MAT_D_3_3 mat);

  uint32_t set_Identity_POSE_D(POSE_D pose);

  uint32_t set_Translate_POSE_D(POSE_D pose, const VEC_D_3 trans);

  uint32_t set_Rotation_POSE_D(POSE_D pose, const MAT_D_3_3 rotation);

  uint32_t Pose_Translate_part(const POSE_D pose, VEC_D_3 trans);

  uint32_t Pose_Rotation_part(const POSE_D pose, MAT_D_3_3 rotation);

  uint32_t set_Identity_Mat33d(MAT_D_3_3 mat);

  uint32_t Mat33D_Vec3D_multiply(const MAT_D_3_3 mat, const VEC_D_3 v, VEC_D_3 res);

  uint32_t Mat33D_Vec3D_multiply_inplace(const MAT_D_3_3 mat, VEC_D_3 v);

  uint32_t MAT33D_times(const MAT_D_3_3 mat, double num, MAT_D_3_3 res);

  uint32_t MAT33D_times_inplace(MAT_D_3_3 mat, double num);

  uint32_t MAT33D_matrix_operation(const MAT_D_3_3 factor1, const MAT_D_3_3 factor2, char oper, MAT_D_3_3 res);

  uint32_t MAT33D_matrix_operation_inplace(const MAT_D_3_3 factor1, MAT_D_3_3 factor2, char oper);

  uint32_t skewd(const VEC_D_3 v, MAT_D_3_3 mat);

  double norm_V3d(const VEC_D_3 v);

  uint32_t Log_SO3d(const MAT_D_3_3 R, VEC_D_3 v);

  uint32_t Exp3d(const VEC_D_3 _dx, MAT_D_3_3 R);

  uint32_t Exp6d(const VEC_D_6 _dx, POSE_D T);
  
  // wxyz
  uint32_t mat2qua(const MAT_D_3_3 m, Quaternion_D qua);
  
  // wxyz
  uint32_t qua2mat(const Quaternion_D q, MAT_D_3_3 M);

  uint32_t normalize_q(Quaternion_D q);

  uint32_t inversePose(const POSE_D pose_in, POSE_D pose_out);
  
  uint32_t inversePose_inplace(POSE_D pose);

  uint32_t multipyPose(const POSE_D lpose, const POSE_D rpose, POSE_D res);

  uint32_t multipyPose_inplace(const POSE_D lpose, POSE_D rpose);

  uint32_t MultipyMatrix(const MAT_DYNAMIC_D* lmat, MAT_DYNAMIC_D* rmat);

  // inline Vec2d world2cam(const VEC_D_3 &xyz_c, const Mat3d &K) {
  //   Vec2d p = xyz_c.head<2>() / xyz_c(2);
  //   p(0) = K(0, 0) * p(0) + K(0, 2);
  //   p(1) = K(1, 1) * p(1) + K(1, 2);
  //   return p;
  // }

  // inline Vec2d world2cam(const VEC_D_3 &&xyz_c, const Mat3d &K) {
  //   Vec2d p = xyz_c.head<2>() / xyz_c(2);
  //   p(0) = K(0, 0) * p(0) + K(0, 2);
  //   p(1) = K(1, 1) * p(1) + K(1, 2);
  //   return p;
  // }

  // inline VEC_D_3 cam2world(const Vec2d &px, Mat3d Kinv) {
  //   VEC_D_3 p(px(0), px(1), 1.0);
  //   p(0) = Kinv(0, 0) * p(0) + Kinv(0, 2);
  //   p(1) = Kinv(1, 1) * p(1) + Kinv(1, 2);
  //   return p.normalized();
  // }

  // inline VEC_D_3 cam2world(const Vec2d &&px, Mat3d Kinv) {
  //   VEC_D_3 p(px(0), px(1), 1.0);
  //   p(0) = Kinv(0, 0) * p(0) + Kinv(0, 2);
  //   p(1) = Kinv(1, 1) * p(1) + Kinv(1, 2);
  //   return p.normalized();
  // }

  // inline VEC_D_3 transfromPoint(const Mat34d &T_12, const VEC_D_3 &p2) {
  //   VEC_D_3 p1 = T_12.block<3, 3>(0, 0) * p2 + T_12.block<3, 1>(0, 3);
  //   return p1;
  // }

  // bool Triangulate_PBA_depth(const Mat34d &dT_12,
  //                            const VEC_D_3 &v1,
  //                            const VEC_D_3 &v2, double &depth);

  // bool Triangulate_PBA_idepth(const Mat34d &dT_12,
  //                            const VEC_D_3 &v1,
  //                            const VEC_D_3 &v2, double &idepth);

  // VEC_D_3 ComputeEpipolarRes(const Mat34d &T1, const Mat34d &T2,
  //                          const VEC_D_3 &v1, const VEC_D_3 &v2);

  // bool checkEpipolarConstraint(const Mat34d &dT_12,
  //                              const VEC_D_3 &v1, const VEC_D_3 &v2,
  //                              double norm_th = 0.005);

  // void
  // calculate_RT_error(const megCV::POSE_D &_pose_cur_ref, const megCV::POSE_D &_guess_pose_cur_ref, float &_R_error,
  //                    float &_T_error);

  // int optimize_se3(const std::vector<VEC_D_3> &_pts3d_ref, const std::vector<Vec2d> &_pts2d_cur,
  //                  const Mat3d &_K_cur, Mat34d &_Tcr, std::vector<uchar> &_inliers,
  //                  int _optimize_max_ites);

  // class CamModel {
  // public:
  //   CamModel() {}

  //   // RadiaTan: fx, fy, cx, cy, d0, d1, d2, d3, d4
  //   // Equi: fx, fy, cx, cy, d0, d1, d2, d3
  //   // DoubleShpere: fx, fy, cx, cy, xi, alpha
  //   explicit CamModel(std::string _model, const std::vector<float> &_params);

  //   explicit CamModel(std::string _model, const std::vector<double> &_params);

  //   void setCamModel(std::string _model, const std::vector<float> &_params);

  //   void setCamModel(std::string _model, const std::vector<double> &_params);

  //   // Point in cam coordinate is normalized
  //   VEC_D_3 cam2world(const cv::Point2f &px) const;

  //   cv::Point2f world2cam(const VEC_D_3 &pt, bool distort = true) const;

  //   void world2cam(const vVEC_D_3 &pts3D, std::vector<cv::Point2f> &pts2D, bool distort = true) const;

  //   // Every points in cam coordinate is normalized
  //   void cam2world(const std::vector<cv::Point2f> &pts2D, vVEC_D_3 &pts3D) const;

  //   std::string getModelName() const;

  //   // RadiaTan: fx, fy, cx, cy, d0, d1, d2, d3, d4
  //   // Equi: fx, fy, cx, cy, d0, d1, d2, d3
  //   // DoubleShpere: fx, fy, cx, cy, xi, alpha
  //   void getIntrinsic(std::vector<float> &_intrinsic) const;

  //   void getIntrinsic(std::vector<double> &_intrinsic) const;

  //   bool alreadySetModel() const { return alreadySet; }

  // private:
  //   std::string model;

  //   bool alreadySet = false;

  //   //RadialTanCamera
  //   mutable std::vector<cv::Point3f> pt3d;
  //   mutable std::vector<cv::Point2f> pt2d;
  //   cv::Mat cvK;
  //   cv::Mat cvKinv;
  //   cv::Mat distortParam;

  //   //EquiCamera
  //   mutable std::vector<cv::Point2f> und_pt2d;
  //   mutable std::vector<cv::Point2f> dis_pt2d;

  //   //DoubleShpereCamera
  //   float xi, alpha;
  // };

  // void getWarpMatrixAffine(const Mat3d &cam_ref,
  //                          const Mat3d &cam_cur,
  //                          const Vec2d &px_ref,
  //                          const VEC_D_3 &f_ref,
  //                          const double depth_ref,
  //                          const Mat3d &R_cur_ref,
  //                          const VEC_D_3 &T_cur_ref,
  //                          const int level_ref,
  //                          const int halfpatch_size,
  //                          Mat2d &A_cur_ref);

  // void getWarpMatrixAffine(const CamModel &cam_ref,
  //                          const CamModel &cam_cur,
  //                          const vPoint2f &pt2ds_ref,
  //                          const vVEC_D_3 &pt3ds_ref,
  //                          const POSE_D &T_cur_ref,
  //                          const int halfpatch_size,
  //                          vMat2d &A_cur_ref);

  // void triangulate(const POSE_D &T_WC_1, const POSE_D &T_WC_2, const VEC_D_3 &f1, const VEC_D_3 &f2, VEC_D_3 &pw);

  // void computeCs(const POSE_D &Rwc_m, const VEC_D_3 &v1, const POSE_D &Rwc_a, const VEC_D_3 &v2, Vec2d &cstheta);



#ifdef __cplusplus
}
#endif

#endif //_GEOMETRY_H
