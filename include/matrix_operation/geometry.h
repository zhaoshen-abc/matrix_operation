#ifndef ACV_GEOMETRY_H
#define ACV_GEOMETRY_H

#include "Eigen/Dense"

#include "../base.h"

namespace megCV {
  typedef Eigen::Matrix<float_, 2, 1> Vec2d;
  typedef Eigen::Matrix<float_, 3, 1> Vec3f;
  typedef Eigen::Matrix<float_, 3, 1> Vec3d;
  typedef Eigen::Matrix<float_, 2, 2> Mat2d;
  typedef Eigen::Matrix<float_, 3, 3> Mat3d;
  typedef Eigen::Matrix<float_, 3, 4> Mat34d;
  typedef Eigen::Matrix<float_, 4, 4> Mat44d;
  typedef Eigen::Matrix<float_, 6, 1> Vec6d;
  typedef Eigen::Matrix<float_, 6, 6> Mat6d;

  typedef std::vector<Vec2d, Eigen::aligned_allocator<Vec2d>> vVec2d;
  typedef std::vector<Vec3d, Eigen::aligned_allocator<Vec3d>> vVec3d;
  typedef std::vector<Vec2d, Eigen::aligned_allocator<Vec2d>> vVec2d;
  typedef std::vector<Mat3d, Eigen::aligned_allocator<Mat3d>> vMat3d;
  typedef std::vector<Mat34d, Eigen::aligned_allocator<Mat34d>> vMat34d;

  typedef Eigen::Matrix<float_, Eigen::Dynamic, Eigen::Dynamic> MatXf;

  typedef Mat3d Rotationd;
  typedef Mat34d Posed;

  template<typename T>
  using aligned_vector = std::vector<T, Eigen::aligned_allocator<T>>;

  using vMat2d = aligned_vector<Mat2d>;

  using vPoint2f = std::vector<cv::Point2f>;

  const Posed &Identity34d();

  Mat3d skewd(const Vec3d &v);

  Mat3d skewd(const Vec3d &&v);

  Vec3d Log_SO3d(const Mat3d &R);

  Vec3d Log_SO3d(const Mat3d &&R);

  Posed Exp6d(const Vec6d &_dx);

  inline Vec2d world2cam(const Vec3d &xyz_c, const Mat3d &K) {
    Vec2d p = xyz_c.head<2>() / xyz_c(2);
    p(0) = K(0, 0) * p(0) + K(0, 2);
    p(1) = K(1, 1) * p(1) + K(1, 2);
    return p;
  }

  inline Vec2d world2cam(const Vec3d &&xyz_c, const Mat3d &K) {
    Vec2d p = xyz_c.head<2>() / xyz_c(2);
    p(0) = K(0, 0) * p(0) + K(0, 2);
    p(1) = K(1, 1) * p(1) + K(1, 2);
    return p;
  }

  inline Vec3d cam2world(const Vec2d &px, Mat3d Kinv) {
    Vec3d p(px(0), px(1), 1.0);
    p(0) = Kinv(0, 0) * p(0) + Kinv(0, 2);
    p(1) = Kinv(1, 1) * p(1) + Kinv(1, 2);
    return p.normalized();
  }

  inline Vec3d cam2world(const Vec2d &&px, Mat3d Kinv) {
    Vec3d p(px(0), px(1), 1.0);
    p(0) = Kinv(0, 0) * p(0) + Kinv(0, 2);
    p(1) = Kinv(1, 1) * p(1) + Kinv(1, 2);
    return p.normalized();
  }

  inline Vec3d transfromPoint(const Mat34d &T_12, const Vec3d &p2) {
    Vec3d p1 = T_12.block<3, 3>(0, 0) * p2 + T_12.block<3, 1>(0, 3);
    return p1;
  }

  Mat34d inversePose(const Mat34d &pose);

  Mat34d inversePose(const Mat34d &&pose);

  Mat34d multipyPose(const Mat34d &lth, const Mat34d &rth);

  Mat34d multipyPose(const Mat34d &&lth, const Mat34d &&rth);

  Mat34d multipyPose(const Mat34d &pose1, const Mat34d &pose2, const Mat34d &pose3);

  Mat34d multipyPose(const Mat34d &&pose1, const Mat34d &&pose2, const Mat34d &&pose3);


  bool Triangulate_PBA_depth(const Mat34d &dT_12,
                             const Vec3d &v1,
                             const Vec3d &v2, double &depth);

  bool Triangulate_PBA_idepth(const Mat34d &dT_12,
                             const Vec3d &v1,
                             const Vec3d &v2, double &idepth);

  Vec3d ComputeEpipolarRes(const Mat34d &T1, const Mat34d &T2,
                           const Vec3d &v1, const Vec3d &v2);

  bool checkEpipolarConstraint(const Mat34d &dT_12,
                               const Vec3d &v1, const Vec3d &v2,
                               double norm_th = 0.005);

  void
  calculate_RT_error(const megCV::Posed &_pose_cur_ref, const megCV::Posed &_guess_pose_cur_ref, float &_R_error,
                     float &_T_error);

  int optimize_se3(const std::vector<Vec3d> &_pts3d_ref, const std::vector<Vec2d> &_pts2d_cur,
                   const Mat3d &_K_cur, Mat34d &_Tcr, std::vector<uchar> &_inliers,
                   int _optimize_max_ites);

  class CamModel {
  public:
    CamModel() {}

    // RadiaTan: fx, fy, cx, cy, d0, d1, d2, d3, d4
    // Equi: fx, fy, cx, cy, d0, d1, d2, d3
    // DoubleShpere: fx, fy, cx, cy, xi, alpha
    explicit CamModel(std::string _model, const std::vector<float> &_params);

    explicit CamModel(std::string _model, const std::vector<double> &_params);

    void setCamModel(std::string _model, const std::vector<float> &_params);

    void setCamModel(std::string _model, const std::vector<double> &_params);

    // Point in cam coordinate is normalized
    Vec3d cam2world(const cv::Point2f &px) const;

    cv::Point2f world2cam(const Vec3d &pt, bool distort = true) const;

    void world2cam(const vVec3d &pts3D, std::vector<cv::Point2f> &pts2D, bool distort = true) const;

    // Every points in cam coordinate is normalized
    void cam2world(const std::vector<cv::Point2f> &pts2D, vVec3d &pts3D) const;

    std::string getModelName() const;

    // RadiaTan: fx, fy, cx, cy, d0, d1, d2, d3, d4
    // Equi: fx, fy, cx, cy, d0, d1, d2, d3
    // DoubleShpere: fx, fy, cx, cy, xi, alpha
    void getIntrinsic(std::vector<float> &_intrinsic) const;

    void getIntrinsic(std::vector<double> &_intrinsic) const;

    bool alreadySetModel() const { return alreadySet; }

  private:
    std::string model;

    bool alreadySet = false;

    //RadialTanCamera
    mutable std::vector<cv::Point3f> pt3d;
    mutable std::vector<cv::Point2f> pt2d;
    cv::Mat cvK;
    cv::Mat cvKinv;
    cv::Mat distortParam;

    //EquiCamera
    mutable std::vector<cv::Point2f> und_pt2d;
    mutable std::vector<cv::Point2f> dis_pt2d;

    //DoubleShpereCamera
    float xi, alpha;
  };

  void getWarpMatrixAffine(const Mat3d &cam_ref,
                           const Mat3d &cam_cur,
                           const Vec2d &px_ref,
                           const Vec3d &f_ref,
                           const double depth_ref,
                           const Mat3d &R_cur_ref,
                           const Vec3d &T_cur_ref,
                           const int level_ref,
                           const int halfpatch_size,
                           Mat2d &A_cur_ref);

  void getWarpMatrixAffine(const CamModel &cam_ref,
                           const CamModel &cam_cur,
                           const vPoint2f &pt2ds_ref,
                           const vVec3d &pt3ds_ref,
                           const Posed &T_cur_ref,
                           const int halfpatch_size,
                           vMat2d &A_cur_ref);

  void triangulate(const Posed &T_WC_1, const Posed &T_WC_2, const Vec3d &f1, const Vec3d &f2, Vec3d &pw);

  void computeCs(const Posed &Rwc_m, const Vec3d &v1, const Posed &Rwc_a, const Vec3d &v2, Vec2d &cstheta);
}


#endif //ACV_GEOMETRY_H
