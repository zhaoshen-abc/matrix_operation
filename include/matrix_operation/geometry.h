#ifndef _GEOMETRY_H
#define _GEOMETRY_H

#include <math.h>
#include "ia_abstraction.h"
#include "ac_auxiliary.h"
#include "macro.h"

#ifdef __cplusplus
extern "C" {
#endif
  typedef double MAT_D_3_3[3][3];
  typedef double POSE_D[3][4];
  typedef double VEC_D_3[3];
  typedef double VEC_D_6[6]; // [x y z rx ry rz]
  typedef double Quaternion_D[4];

  // using vMat2d = aligned_vector<Mat2d>;

  // using vPoint2f = std::vector<cv::Point2f>;

  // const POSE_D &Identity34d();

  void ac_MAT_D_3_3_print(const MAT_D_3_3 mat);

  void ac_POSE_D_print(const POSE_D mat);

  ia_err copy_VEC_D_3(const VEC_D_3 v_in, VEC_D_3 v_out);

  ia_err copy_Mat33d(const MAT_D_3_3 mat_in, MAT_D_3_3 mat_out);

  ia_err transpose_Mat33d(const MAT_D_3_3 mat_in, MAT_D_3_3 mat_out);

  ia_err transpose_Mat33d_inplace(MAT_D_3_3 mat);

  ia_err set_Identity_POSE_D(POSE_D pose);

  ia_err set_Translate_POSE_D(POSE_D pose, const VEC_D_3 trans);

  ia_err set_Rotation_POSE_D(POSE_D pose, const MAT_D_3_3 rotation);

  ia_err Pose_Translate_part(const POSE_D pose, VEC_D_3 trans);

  ia_err Pose_Rotation_part(const POSE_D pose, MAT_D_3_3 rotation);

  ia_err set_Identity_Mat33d(MAT_D_3_3 mat);

  ia_err Mat33D_Vec3D_multiply(const MAT_D_3_3 mat, const VEC_D_3 v, VEC_D_3 res);

  ia_err Mat33D_Vec3D_multiply_inplace(const MAT_D_3_3 mat, VEC_D_3 v);

  ia_err MAT33D_times(const MAT_D_3_3 mat, double num, MAT_D_3_3 res);

  ia_err MAT33D_times_inplace(MAT_D_3_3 mat, double num);

  ia_err MAT33D_matrix_operation(const MAT_D_3_3 factor1, const MAT_D_3_3 factor2, char oper, MAT_D_3_3 res);

  ia_err MAT33D_matrix_operation_inplace(const MAT_D_3_3 factor1, MAT_D_3_3 factor2, char oper);

  ia_err skewd(const VEC_D_3 v, MAT_D_3_3 mat);

  double norm_V3d(const VEC_D_3 v);

  ia_err Log_SO3d(const MAT_D_3_3 R, VEC_D_3 v);

  ia_err Exp3d(const VEC_D_3 _dx, MAT_D_3_3 R);

  ia_err Exp6d(const VEC_D_6 _dx, POSE_D T);
  
  ia_err mat2qua(const MAT_D_3_3 m, Quaternion_D qua);

  ia_err inversePose(const POSE_D pose_in, POSE_D pose_out);
  
  ia_err inversePose_inplace(POSE_D pose);

  ia_err multipyPose(const POSE_D lpose, const POSE_D rpose, POSE_D res);

  ia_err multipyPose_inplace(const POSE_D lpose, POSE_D rpose);

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
