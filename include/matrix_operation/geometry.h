#ifndef _GEOMETRY_H
#define _GEOMETRY_H

#include "ia_abstraction.h"
#include "ac_auxiliary.h"
#include "macro.h"

#ifdef __cplusplus
extern "C" {
#endif
  typedef double MAT_D_3_3[3][3];
  typedef double Posed[3][4];
  typedef double Vec3d[3];
  typedef double Vec6d[6]; // [x y z rx ry rz]
  typedef double Quaterniond[3];

  // using vMat2d = aligned_vector<Mat2d>;

  // using vPoint2f = std::vector<cv::Point2f>;

  // const Posed &Identity34d();

  ia_err copy_Vec3d(const Vec3d v_in, Vec3d v_out);

  ia_err copy_Mat33d(const MAT_D_3_3 mat_in, MAT_D_3_3 mat_out);

  ia_err transpose_Mat33d(const MAT_D_3_3 mat_in, MAT_D_3_3 mat_out);

  ia_err transpose_Mat33d_inplace(MAT_D_3_3 mat);

  ia_err set_Identity_Posed(Posed pose);

  ia_err set_Translate_Posed(Posed pose, const Vec3d trans);

  ia_err set_Rotation_Posed(Posed pose, const MAT_D_3_3 rotation);

  ia_err Pose_Translate_part(const Posed pose, Vec3d trans);

  ia_err Pose_Rotation_part(const Posed pose, MAT_D_3_3 rotation);

  ia_err set_Identity_Mat33d(MAT_D_3_3 mat);

  ia_err Mat33D_Vec3D_multiply(const MAT_D_3_3 mat, const Vec3d v, Vec3d res);

  ia_err MAT33D_times(const MAT_D_3_3 mat, double num, MAT_D_3_3 res);

  ia_err MAT33D_matrix_operation(const MAT_D_3_3 factor1, const MAT_D_3_3 factor2, char oper, MAT_D_3_3 res);

  ia_err MAT33D_matrix_operation_inplace(const MAT_D_3_3 factor1, MAT_D_3_3 factor2, char oper);

  ia_err skewd(const Vec3d v, MAT_D_3_3 mat);

  double norm_V3d(const Vec3d v);

  ia_err Log_SO3d(const MAT_D_3_3 R, Vec3d v);

  ia_err Exp6d(const Vec6d _dx, Posed pose);
  
  ia_err mat2qua(const MAT_D_3_3 m, Quaterniond qua);

  ia_err inversePose(const Posed pose_in, Posed pose_out);
  
  ia_err inversePose_inplace(Posed pose);

  ia_err multipyPose(const Posed lpose, const Posed rpose, Posed res);

  ia_err multipyPose_inplace(const Posed lpose, Posed rpose);

  // inline Vec2d world2cam(const Vec3d &xyz_c, const Mat3d &K) {
  //   Vec2d p = xyz_c.head<2>() / xyz_c(2);
  //   p(0) = K(0, 0) * p(0) + K(0, 2);
  //   p(1) = K(1, 1) * p(1) + K(1, 2);
  //   return p;
  // }

  // inline Vec2d world2cam(const Vec3d &&xyz_c, const Mat3d &K) {
  //   Vec2d p = xyz_c.head<2>() / xyz_c(2);
  //   p(0) = K(0, 0) * p(0) + K(0, 2);
  //   p(1) = K(1, 1) * p(1) + K(1, 2);
  //   return p;
  // }

  // inline Vec3d cam2world(const Vec2d &px, Mat3d Kinv) {
  //   Vec3d p(px(0), px(1), 1.0);
  //   p(0) = Kinv(0, 0) * p(0) + Kinv(0, 2);
  //   p(1) = Kinv(1, 1) * p(1) + Kinv(1, 2);
  //   return p.normalized();
  // }

  // inline Vec3d cam2world(const Vec2d &&px, Mat3d Kinv) {
  //   Vec3d p(px(0), px(1), 1.0);
  //   p(0) = Kinv(0, 0) * p(0) + Kinv(0, 2);
  //   p(1) = Kinv(1, 1) * p(1) + Kinv(1, 2);
  //   return p.normalized();
  // }

  // inline Vec3d transfromPoint(const Mat34d &T_12, const Vec3d &p2) {
  //   Vec3d p1 = T_12.block<3, 3>(0, 0) * p2 + T_12.block<3, 1>(0, 3);
  //   return p1;
  // }

  // bool Triangulate_PBA_depth(const Mat34d &dT_12,
  //                            const Vec3d &v1,
  //                            const Vec3d &v2, double &depth);

  // bool Triangulate_PBA_idepth(const Mat34d &dT_12,
  //                            const Vec3d &v1,
  //                            const Vec3d &v2, double &idepth);

  // Vec3d ComputeEpipolarRes(const Mat34d &T1, const Mat34d &T2,
  //                          const Vec3d &v1, const Vec3d &v2);

  // bool checkEpipolarConstraint(const Mat34d &dT_12,
  //                              const Vec3d &v1, const Vec3d &v2,
  //                              double norm_th = 0.005);

  // void
  // calculate_RT_error(const megCV::Posed &_pose_cur_ref, const megCV::Posed &_guess_pose_cur_ref, float &_R_error,
  //                    float &_T_error);

  // int optimize_se3(const std::vector<Vec3d> &_pts3d_ref, const std::vector<Vec2d> &_pts2d_cur,
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
  //   Vec3d cam2world(const cv::Point2f &px) const;

  //   cv::Point2f world2cam(const Vec3d &pt, bool distort = true) const;

  //   void world2cam(const vVec3d &pts3D, std::vector<cv::Point2f> &pts2D, bool distort = true) const;

  //   // Every points in cam coordinate is normalized
  //   void cam2world(const std::vector<cv::Point2f> &pts2D, vVec3d &pts3D) const;

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
  //                          const Vec3d &f_ref,
  //                          const double depth_ref,
  //                          const Mat3d &R_cur_ref,
  //                          const Vec3d &T_cur_ref,
  //                          const int level_ref,
  //                          const int halfpatch_size,
  //                          Mat2d &A_cur_ref);

  // void getWarpMatrixAffine(const CamModel &cam_ref,
  //                          const CamModel &cam_cur,
  //                          const vPoint2f &pt2ds_ref,
  //                          const vVec3d &pt3ds_ref,
  //                          const Posed &T_cur_ref,
  //                          const int halfpatch_size,
  //                          vMat2d &A_cur_ref);

  // void triangulate(const Posed &T_WC_1, const Posed &T_WC_2, const Vec3d &f1, const Vec3d &f2, Vec3d &pw);

  // void computeCs(const Posed &Rwc_m, const Vec3d &v1, const Posed &Rwc_a, const Vec3d &v2, Vec2d &cstheta);



#ifdef __cplusplus
}
#endif

#endif //_GEOMETRY_H
