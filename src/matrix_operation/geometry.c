#include "geometry.h"
#include "ia_abstraction.h"


  void ac_MAT_D_3_3_print(const MAT_D_3_3 mat) {
    uint32_t i = 0U;
    for (i = 0U; i < 3U; i++)
    {
        AC_LOG("%f, %f, %f", mat[i][0], mat[i][1], mat[i][2]);
    }
  }

  void ac_POSE_D_print(const POSE_D mat) {
    for (uint32_t i = 0U; i < 3U; i++)
    {
      AC_LOG("%f, %f, %f, %f", mat[i][0], mat[i][1], mat[i][2], mat[i][3]);
    }
    AC_LOG("%f, %f, %f, %f", 0.0, 0.0, 0.0, 1.0);
  }

  // void ac_VEC_D_3_print(const VEC_D_3 vec) {
  //   AC_LOG("%f, %f, %f", vec[0], vec[1], vec[2]);
  // }


  ia_err copy_VEC_D_3(const VEC_D_3 v_in, VEC_D_3 v_out) {
    v_out[0] = v_in[0];
    v_out[1] = v_in[1];
    v_out[2] = v_in[2];
    return ia_err_none;
  }

  ia_err copy_Mat33d(const MAT_D_3_3 mat_in, MAT_D_3_3 mat_out) {
    for (uint32_t i = 0; i < 3; i ++ ) {
      for (uint32_t j = 0; j < 3; j ++ ) {
        mat_out[i][j] = mat_in[i][j];
      }
    }
    return ia_err_none;
  }

  ia_err copy_POSE_D(const POSE_D pose_in, POSE_D pose_out) {
    for (uint32_t i = 0; i < 3; i ++ ) {
      for (uint32_t j = 0; j < 4; j ++ ) {
        pose_out[i][j] = pose_in[i][j];
      }
    }
    return ia_err_none;
  }

  ia_err transpose_Mat33d(const MAT_D_3_3 mat_in, MAT_D_3_3 mat_out) {
    for (uint32_t i = 0; i < 3; i ++ ) {
      for (uint32_t j = 0; j < 3; j ++ ) {
        mat_out[i][j] = mat_in[j][i];
      }
    }
  }

  ia_err transpose_Mat33d_inplace(MAT_D_3_3 mat) {
    for (uint32_t i = 0; i < 3; i ++ ) {
      for (uint32_t j = i+1; j < 3; j ++ ) {
        double tmp = mat[i][j];
        mat[i][j] = mat[j][i];
        mat[j][i] = tmp;
      }
    }
  }

  ia_err set_Identity_POSE_D(POSE_D pose) {
    memset(pose, 0, 12*sizeof(double));
    pose[0][0] = 1.0;
    pose[1][1] = 1.0;
    pose[2][2] = 1.0;
    return ia_err_none;
  }

  ia_err set_Translate_POSE_D(POSE_D pose, const VEC_D_3 trans) {
    for (uint32_t i = 0; i < 3; i ++ ) {
        pose[i][3] = trans[i];
    }
    return ia_err_none;
  }

  ia_err set_Rotation_POSE_D(POSE_D pose, const MAT_D_3_3 rotation) {
    for (uint32_t i = 0; i < 3; i ++ ) {
      for (uint32_t j = 0; j < 3; j ++ ) {
        pose[i][j] = rotation[i][j];
      }
    }
    return ia_err_none;
  }

  ia_err Pose_Translate_part(const POSE_D pose, VEC_D_3 trans) {
    trans[0] = pose[0][3];
    trans[1] = pose[1][3];
    trans[2] = pose[2][3];
  }

  ia_err Pose_Rotation_part(const POSE_D pose, MAT_D_3_3 rotation) {
    for (uint32_t i = 0U; i < 3; i++) {
      for (uint32_t j = 0U; j < 3; j++) {
        rotation[i][j] = pose[i][j];
      }
    }
  }

  ia_err set_Identity_Mat33d(MAT_D_3_3 mat) {
    memset(mat, 0, 9*sizeof(double));
    mat[0][0] = 1;
    mat[1][1] = 1;
    mat[2][2] = 1;
  }

  ia_err Mat33D_Vec3D_multiply(const MAT_D_3_3 mat, const VEC_D_3 v, VEC_D_3 res) {
    memset(res, 0, sizeof(double) * 3);
    for (uint32_t i = 0U; i < 3; i++) {
      for (uint32_t j = 0U; j < 3; j++) {
        res[i] += mat[i][j] * v[j];
      }
    }
    return ia_err_none;
  }

  ia_err Mat33D_Vec3D_multiply_inplace(const MAT_D_3_3 mat, VEC_D_3 v) {
    VEC_D_3 tmp = {0.0, 0.0, 0.0};
    for (uint32_t i = 0U; i < 3; i++) {
      for (uint32_t j = 0U; j < 3; j++) {
        tmp[i] += mat[i][j] * v[j];
      }
    }
    copy_VEC_D_3(tmp, v);
    return ia_err_none;
  }

  ia_err MAT33D_times(const MAT_D_3_3 mat, double num, MAT_D_3_3 res) {
    uint32_t N = 3, M = 3;
    for (uint32_t i = 0U; i < N; i++) {
      for (uint32_t j = 0U; j < M; j++) {
          res[i][j] = mat[i][j] * num;
        }
      }
    return ia_err_none;
  }

  ia_err MAT33D_times_inplace(MAT_D_3_3 mat, double num) {
    uint32_t N = 3, M = 3;
    for (uint32_t i = 0U; i < N; i++) {
      for (uint32_t j = 0U; j < M; j++) {
          mat[i][j] = mat[i][j] * num;
        }
      }
    return ia_err_none;
  }

  ia_err MAT33D_matrix_operation(const MAT_D_3_3 factor1, const MAT_D_3_3 factor2, char oper, MAT_D_3_3 res) {
    uint32_t N = 3, M = 3;
    switch (oper)
    {
    case '+':
        for (uint32_t i = 0U; i < N; i++) {
            for (uint32_t j = 0U; j < M; j++) {
                res[i][j] = factor1[i][j] + factor2[i][j];
            }
        }
        return ia_err_none;
    case '-':
        for (uint32_t i = 0U; i < N; i++) {
            for (uint32_t j = 0U; j < M; j++) {
                res[i][j] = factor1[i][j] - factor2[i][j];
            }
        }
        return ia_err_none;
    case '*':
        for(uint32_t i =0;i<N;i++) {             
          double sum = 0;
          for(uint32_t j=0;j<M;j++) {	
            sum=0;
            for(uint32_t k=0;k<M;k++) 
            {   
              sum+=factor1[i][k] * factor2[k][j]; 
            }
            res[i][j]=sum; 
          }	
        }
        return ia_err_none;
    case '.':
        for (uint32_t i = 0U; i < N; i++) {
            for (uint32_t j = 0U; j < M; j++) {
                res[i][j] = factor1[i][j] * factor2[i][j];
            }
        }
        return ia_err_none;
    case '/':
        for (uint32_t i = 0U; i < N; i++) {
            for (uint32_t j = 0U; j < M; j++) {
                if (IA_FABS(factor2[i][j]) < IA_EPSILON)
                {
                    return ia_err_data;
                }
                else
                {
                    res[i][j] = factor1[i][j] / factor2[i][j];
                }
            }
        }
        return ia_err_none;
    default:
        return ia_err_argument;
    }
  }

  // operation in place, save result in factor2
  ia_err MAT33D_matrix_operation_inplace(const MAT_D_3_3 factor1, MAT_D_3_3 factor2, char oper) {
    uint32_t N = 3, M = 3;
    switch (oper)
    {
    case '+':
        for (uint32_t i = 0U; i < N; i++) {
            for (uint32_t j = 0U; j < M; j++) {
                factor2[i][j] = factor1[i][j] + factor2[i][j];
            }
        }
        return ia_err_none;
    case '-':
        for (uint32_t i = 0U; i < N; i++) {
            for (uint32_t j = 0U; j < M; j++) {
                factor2[i][j] = factor1[i][j] - factor2[i][j];
            }
        }
        return ia_err_none;
    case '*':
    {
        MAT_D_3_3 tmp;
        for(uint32_t i =0;i<N;i++) {             
          double sum;
          for(uint32_t j=0;j<M;j++) {	
            sum=0;
            for(uint32_t k=0;k<M;k++) 
            {   
              sum+=factor1[i][k] * factor2[k][j]; 
            }
            tmp[i][j]=sum; 
          }	
        }
        copy_Mat33d(tmp, factor2);
        return ia_err_none;
    }
    case '.':
        for (uint32_t i = 0U; i < N; i++) {
            for (uint32_t j = 0U; j < M; j++) {
                factor2[i][j] = factor1[i][j] * factor2[i][j];
            }
        }
        return ia_err_none;
    case '/':
        for (uint32_t i = 0U; i < N; i++) {
            for (uint32_t j = 0U; j < M; j++) {
                if (IA_FABS(factor2[i][j]) < IA_EPSILON)
                {
                    return ia_err_data;
                }
                else
                {
                    factor2[i][j] = factor1[i][j] / factor2[i][j];
                }
            }
        }
        return ia_err_none;
    default:
        return ia_err_argument;
    }
  }

  ia_err skewd(const VEC_D_3 v, MAT_D_3_3 mat) {
    memset(mat, 0, 9*sizeof(double));
    mat[0][1] = -v[2];
    mat[0][2] = v[1];
    mat[1][0] = v[2];
    mat[1][2] = -v[0];
    mat[2][0] = -v[1];
    mat[2][1] = v[0];
    return ia_err_none;
  }

  double norm_V3d(const VEC_D_3 v) {
    double res = v[0] *v[0] + v[1] * v[1] + v[2] * v[2];
    return sqrt(res);
  }

  ia_err Log_SO3d(const MAT_D_3_3 R, VEC_D_3 v) {
    Quaternion_D q;
    mat2qua(R, q);
    //normlize

    double n = norm_V3d(q);
    double w = q[3];
    double squared_w = w * w;

    double two_atan_nbyw_by_n;
    // Atan-based log thanks to
    //
    // C. Hertzberg et al.:
    // "Integrating Generic Sensor Fusion Algorithms with Sound State
    // Representation through Encapsulation of Manifolds"
    // Information Fusion, 2011

    if (n < 1e-9) {
      // If quaternion is normalized and n=1, then w should be 1;
      // w=0 should never happen here!
      assert(fabs(w) > 1e-9);

      two_atan_nbyw_by_n = 2. / w - 2. * (n * n) / (w * squared_w);
    } else {
      if (fabs(w) < 1e-9) {
        if (w > 0) {
          two_atan_nbyw_by_n = M_PI / n;
        } else {
          two_atan_nbyw_by_n = -M_PI / n;
        }
      }
      two_atan_nbyw_by_n = 2 * atan(n / w) / n;
    }

    v[0] = two_atan_nbyw_by_n * q[0];
    v[1] = two_atan_nbyw_by_n * q[1];
    v[2] = two_atan_nbyw_by_n * q[2];

    return ia_err_none;
  }

  ia_err Exp3d(const VEC_D_3 _dx, MAT_D_3_3 R) {

  }

  // ia_err Exp6d(const VEC_D_6 _dx, POSE_D T) {
  //   Vec3d p = _dx.block<3, 1>(0, 0);
  //   Vec3d r = _dx.block<3, 1>(3, 0);
  //   Mat3d R;
  //   {
  //     float theta = r.norm();
  //     if (theta < 1e-9) {
  //       R = Mat3d::Identity();
  //     } else {
  //       Vec3d r_norm = r / theta;
  //       Mat3d hatdx;
  //       hatdx(0, 0) = 0.0;
  //       hatdx(0, 1) = -r_norm(2);
  //       hatdx(0, 2) = r_norm(1);
  //       hatdx(1, 0) = r_norm(2);
  //       hatdx(1, 1) = 0.0;
  //       hatdx(1, 2) = -r_norm(0);
  //       hatdx(2, 0) = -r_norm(1);
  //       hatdx(2, 1) = r_norm(0);
  //       hatdx(2, 2) = 0.0;
  //       R = Mat3d::Identity() + std::sin(theta) * hatdx +
  //           (1 - std::cos(theta)) * hatdx * hatdx;
  //     }
  //   }

  //   Mat3d j;
  //   {
  //     float theta = r.norm();
  //     if (theta < 1e-9) {
  //       j = Mat3d::Identity();
  //     } else {
  //       Vec3d r_norm = r / theta;
  //       Mat3d K;
  //       K(0, 0) = 0.0;
  //       K(0, 1) = -r_norm(2);
  //       K(0, 2) = r_norm(1);
  //       K(1, 0) = r_norm(2);
  //       K(1, 1) = 0.0;
  //       K(1, 2) = -r_norm(0);
  //       K(2, 0) = -r_norm(1);
  //       K(2, 1) = r_norm(0);
  //       K(2, 2) = 0.0;
  //       j = Mat3d::Identity() - (1 - std::cos(theta)) / theta * K +
  //           (1 - std::sin(theta) / theta) * K * K;
  //     }
  //   }

  //   Posed T = megCV::Identity34d();
  //   T.block<3, 3>(0, 0) = R;
  //   T.block<3, 1>(0, 3) = j * p;
  //   return T;
  // }


  // copy from https://blog.csdn.net/w_weixiaotao/article/details/109496434
  ia_err mat2qua(const MAT_D_3_3 m, Quaternion_D qua)
  {
    double q1 = sqrt(m[0][0] + m[1][1] + m[2][2] + 1) / 2;
    double q2, q3, q4, tr, s;
    if (q1 != 0.0) {
      q2 = (m[2][1] - m[1][2]) / 4 / q1;
      q3 = (m[0][2] - m[2][0]) / 4 / q1;
      q4 = (m[1][0] - m[0][1]) / 4 / q1;
    }
    else {
      tr = m[0][0] + m[1][1] + m[2][2];
      if (tr > 0) {
        s = sqrt(tr + 1.0) * 2;
        q1 = 0.25 * s;
        q2 = (m[2][1] - m[1][2]) / s;
        q3 = (m[0][2] - m[2][0]) / s;
        q4 = (m[1][0] - m[0][1]) / s;
      }
      else if ((m[0][0] > m[1][1]) && (m[0][0] > m[2][2])) {
        s = sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2;
        q1 = (m[2][1] - m[1][2]) / s;
        q2 = 0.25 * s;
        q3 = (m[0][1] + m[1][0]) / s;
        q4 = (m[0][2] + m[2][0]) / s;
      }
      else if(m[1][1] > m[2][2])
      {
        s = sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2;
        q1 = (m[0][2] - m[2][0]) / s;
        q2 = (m[0][1] + m[1][0]) / s;
        q3 = 0.25 * s;
        q4 = (m[1][2] + m[2][1]) / s;
      }
      else {
        s = sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2;
        q1 = (m[1][0] - m[0][1]) / s;
        q2 = (m[0][2] + m[2][0]) / s;
        q3 = (m[1][2] + m[2][1]) / s;
        q4 = 0.25 * s;
      }
    }
    qua[0] = q1;
    qua[1] = q2;
    qua[2] = q3;
    qua[3] = q4;
    return ia_err_none;
  }

  ia_err Exp6d(const VEC_D_6 _dx, POSE_D pose) {
    VEC_D_3 p, r;
    copy_VEC_D_3(_dx, p);
    copy_VEC_D_3(_dx + 3, p);
    MAT_D_3_3 R;
    {
      double theta = norm_V3d(r);
      if (theta < 1e-9) {
        set_Identity_Mat33d(R);
      } else {
        VEC_D_3 r_norm = {r[0]/theta, r[1]/theta, r[2]/theta};
        MAT_D_3_3 hatdx;
        skewd(r_norm, hatdx);

        MAT_D_3_3 tmp1, tmp2;
        set_Identity_Mat33d(R);
        MAT33D_times(hatdx, sin(theta), tmp1);
        MAT33D_matrix_operation(hatdx, hatdx, '*', tmp2);
        MAT33D_times_inplace(tmp2, (1 - cos(theta)));
        MAT33D_matrix_operation_inplace(R, tmp1, '+');
        MAT33D_matrix_operation_inplace(R, tmp2, '+');
      }
    }

    MAT_D_3_3 j;
    {
      double theta = norm_V3d(r);
      if (theta < 1e-9) {
        set_Identity_Mat33d(j);
      } else {
        VEC_D_3 r_norm = {r[0]/theta, r[1]/theta, r[2]/theta};
        MAT_D_3_3 K;
        skewd(r_norm, K);
        MAT_D_3_3 tmp1, tmp2;
        set_Identity_Mat33d(j);
        MAT33D_times(K, (1 - cos(theta)), tmp1);
        MAT33D_matrix_operation(K, K, '*', tmp2);
        MAT33D_times_inplace(tmp2, (1 - sin(theta) / theta));
        MAT33D_matrix_operation_inplace(j, tmp1, '+');
        MAT33D_matrix_operation_inplace(j, tmp2, '+');
      }
    }

    Mat33D_Vec3D_multiply_inplace(j, p);
    set_Rotation_POSE_D(pose, R);
    set_Translate_POSE_D(pose, p);
    return ia_err_none;
  }

  ia_err inversePose(const POSE_D pose_in, POSE_D pose_out) { 
      MAT_D_3_3 rotation;
      VEC_D_3 trans;
      Pose_Rotation_part(pose_in, rotation);
      Pose_Translate_part(pose_in, trans);

      transpose_Mat33d_inplace(rotation);
      set_Rotation_POSE_D(pose_out, rotation);
      MAT33D_times_inplace(rotation, -1.0);
      Mat33D_Vec3D_multiply_inplace(rotation, trans);
      set_Translate_POSE_D(pose_out, trans);

      return ia_err_none; 
  }


  ia_err inversePose_inplace(POSE_D pose) { 
      MAT_D_3_3 rotation;
      VEC_D_3 trans;
      Pose_Rotation_part(pose, rotation);
      Pose_Translate_part(pose, trans);

      transpose_Mat33d_inplace(rotation);
      set_Rotation_POSE_D(pose, rotation);
      MAT33D_times_inplace(rotation, -1.0);
      Mat33D_Vec3D_multiply_inplace(rotation, trans);
      set_Translate_POSE_D(pose, trans);

      return ia_err_none; 
  }

  ia_err multipyPose(const POSE_D lpose, const POSE_D rpose, POSE_D res) { \
    MAT_D_3_3 lrot, rrot, rot;
    Pose_Rotation_part(lpose, lrot);
    Pose_Rotation_part(rpose, rrot);
    MAT33D_matrix_operation(lrot, rrot, '*', rot);
    set_Rotation_POSE_D(res, rot);

    VEC_D_3 ltrans, rtrans, trans;
    Pose_Translate_part(lpose, ltrans);
    Pose_Translate_part(rpose, rtrans);
    Mat33D_Vec3D_multiply_inplace(lrot, rtrans);

    for (uint32_t i = 0; i < 3; i ++ ) 
      trans[i] = rtrans[i] + ltrans[i];
    set_Translate_POSE_D(res, trans);
    return ia_err_none;
  }

  // operation in place, save result in rpose
  ia_err multipyPose_inplace(const POSE_D lpose, POSE_D rpose) { 
    POSE_D tmp;
    multipyPose(lpose, rpose, tmp);
    copy_POSE_D(tmp, rpose);
    return ia_err_none;
  }


//   bool Triangulate_PBA_depth(const Mat34d &dT_12,
//                              const VEC_D_3 &v1,
//                              const VEC_D_3 &v2, double &depth) {
//     const VEC_D_3 &pa = dT_12.block<3, 1>(0, 3);
//     const VEC_D_3 &p = dT_12.block<3, 1>(0, 3);
//     const VEC_D_3 a = -pa;
//     const VEC_D_3 &dirMainWorld = v1;
//     const VEC_D_3 b = -p;
//     VEC_D_3 acrossn = a.cross(dirMainWorld);
//     float acrossnNorm = acrossn.norm();
//     acrossn.normalize();
//     VEC_D_3 aProj2n = a.dot(dirMainWorld) * dirMainWorld;
//     VEC_D_3 A = acrossnNorm * dirMainWorld;
//     VEC_D_3 B = b - aProj2n;
//     VEC_D_3 h = A.cross(B);
//     h.normalize();
//     VEC_D_3 v = dT_12.block<3, 3>(0, 0) * v2;
//     VEC_D_3 vh = v.dot(h) * h;
//     VEC_D_3 vp = v - vh;
//     VEC_D_3 evp = VEC_D_3::Zero();
//     Vec2d cstheta(1, 0);
//     if (vp.dot(B) > 0) {
//       evp = vp.normalized() - vp;
//       A.normalize();
//       cstheta[0] = vp.dot(A);
//       if (cstheta[0] > 0.999999) {
//         cstheta[1] = 0;
//       } else {
//         cstheta[1] = sqrt(1 - cstheta[0] * cstheta[0]);

//         float costheta = v1.dot(dT_12.block<3, 3>(0, 0) * v2);
//         if (abs(costheta - cstheta[0]) > 0.01) {
//           return false;
//         }
//       }
//     } else {
//       return false;
//       A.normalize();
//       if ((vp - A).norm() > (vp + A).norm()) {
//         evp = vp + A;
//         cstheta[0] = -1;
//         cstheta[1] = 0;
//       } else {
//         evp = vp - A;
//         cstheta[0] = 1;
//         cstheta[1] = 0;
//       }
//     }
//     VEC_D_3 error = vh + evp;

//     float anorm = a.norm();
//     float cosalpha = a.dot(v1) / anorm;
//     float sinalpha = sin(acos(cosalpha));
//     depth = anorm * (sinalpha * cstheta[0] - cosalpha * cstheta[1]) / cstheta[1];

//     if (cosalpha < -0.999999999999999 || cosalpha > 0.999999999999999) {
//       sinalpha = 0;
//       depth = 1.0e15;
//     }

//     if (depth < 0) {
//       return false;
//     }

//     if (error.norm() > 0.01) {
//       return false;
//     }
//     return true;

//   }

//   bool Triangulate_PBA_idepth(const Mat34d &dT_12,
//                               const VEC_D_3 &v1,
//                               const VEC_D_3 &v2, double &idepth) {
//     const VEC_D_3 &pa = dT_12.block<3, 1>(0, 3);
//     const VEC_D_3 &p = dT_12.block<3, 1>(0, 3);
//     const VEC_D_3 a = -pa;
//     const VEC_D_3 &dirMainWorld = v1;
//     const VEC_D_3 b = -p;
//     VEC_D_3 acrossn = a.cross(dirMainWorld);
//     float acrossnNorm = acrossn.norm();
//     acrossn.normalize();
//     VEC_D_3 aProj2n = a.dot(dirMainWorld) * dirMainWorld;
//     VEC_D_3 A = acrossnNorm * dirMainWorld;
//     VEC_D_3 B = b - aProj2n;
//     VEC_D_3 h = A.cross(B);
//     h.normalize();
//     VEC_D_3 v = dT_12.block<3, 3>(0, 0) * v2;
//     VEC_D_3 vh = v.dot(h) * h;
//     VEC_D_3 vp = v - vh;
//     VEC_D_3 evp = VEC_D_3::Zero();
//     Vec2d cstheta(1, 0);
//     if (vp.dot(B) > 0) {
//       evp = vp.normalized() - vp;
//       A.normalize();
//       cstheta[0] = vp.dot(A);
//       if (cstheta[0] > 0.999999) {
//         cstheta[1] = 0;
//       } else {
//         cstheta[1] = sqrt(1 - cstheta[0] * cstheta[0]);

//         float costheta = v1.dot(dT_12.block<3, 3>(0, 0) * v2);
//         if (abs(costheta - cstheta[0]) > 0.01) {
//           return false;
//         }
//       }
//     } else {
//       return false;
//       A.normalize();
//       if ((vp - A).norm() > (vp + A).norm()) {
//         evp = vp + A;
//         cstheta[0] = -1;
//         cstheta[1] = 0;
//       } else {
//         evp = vp - A;
//         cstheta[0] = 1;
//         cstheta[1] = 0;
//       }
//     }
//     VEC_D_3 error = vh + evp;

//     float anorm = a.norm();
//     float cosalpha = a.dot(v1) / anorm;
//     float sinalpha = sin(acos(cosalpha));
//     idepth = anorm * (sinalpha * cstheta[0] - cosalpha * cstheta[1]);
//     idepth = cstheta[1] / idepth;

//     if (cosalpha < -0.999999999999999 || cosalpha > 0.999999999999999) {
//       sinalpha = 0;
//       idepth = 0.0;
//     }

//     if (idepth < 0) {
//       return false;
//     }

//     if (error.norm() > 0.01) {
//       return false;
//     }

//     return true;
//   }

//   VEC_D_3 ComputeEpipolarRes(const Mat34d &T1, const Mat34d &T2,
//                            const VEC_D_3 &v1, const VEC_D_3 &v2) {
//     const Mat3d &Rm = T1.block<3, 3>(0, 0);
//     const VEC_D_3 &pm = T1.block<3, 1>(0, 3);
//     const VEC_D_3 &pa = T2.block<3, 1>(0, 3);
//     const VEC_D_3 &p = T2.block<3, 1>(0, 3);
//     const VEC_D_3 a = pm - pa;
//     const VEC_D_3 dirMainWorld = Rm * v1;
//     if (a.norm() < 1e-5) {
//       VEC_D_3 error = T2.block<3, 3>(0, 0).transpose() * dirMainWorld - v2;
//       return error;
//     }

//     const VEC_D_3 b = pm - p;
//     VEC_D_3 acrossn = a.cross(dirMainWorld);
//     float acrossnNorm = acrossn.norm();
//     acrossn.normalize();
//     VEC_D_3 aProj2n = a.dot(dirMainWorld) * dirMainWorld;
//     VEC_D_3 A = acrossnNorm * dirMainWorld;
//     VEC_D_3 B = b - aProj2n;
//     VEC_D_3 h = A.cross(B);
//     h.normalize();
//     VEC_D_3 v = T2.block<3, 3>(0, 0) * v2;
//     VEC_D_3 vh = v.dot(h) * h;
//     VEC_D_3 vp = v - vh;
//     VEC_D_3 evp = VEC_D_3::Zero();
//     if (vp.dot(B) > 0) {
//       evp = vp.normalized() - vp;
//     } else {
//       A.normalize();
//       if ((vp - A).norm() > (vp + A).norm())
//         evp = vp + A;
//       else evp = vp - A;
//     }
//     VEC_D_3 error = vh + evp;
//     return error;
//   }

//   bool checkEpipolarConstraint(const Mat34d &dT_12,
//                                const VEC_D_3 &v1, const VEC_D_3 &v2,
//                                double norm_th) {
//     Mat34d T1 = Mat34d::Zero();
//     T1.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Identity();
//     VEC_D_3 Error = ComputeEpipolarRes(T1, dT_12, v1, v2);

//     if (Error.norm() < norm_th) {
//       return true;
//     } else {
//       return false;
//     }
//   }

//   void
//   calculate_RT_error(const POSE_D &_pose_cur_ref, const POSE_D &_guess_pose_cur_ref, float &_R_error,
//                      float &_T_error) {

//     _T_error = (_guess_pose_cur_ref.block<3, 1>(0, 3) - _pose_cur_ref.block<3, 1>(0, 3)).norm();

//     Mat3d dR = _pose_cur_ref.block<3, 3>(0, 0).transpose() * _guess_pose_cur_ref.block<3, 3>(0, 0);
//     VEC_D_3 dr = megCV::Log_SO3d(dR);
//     _R_error = dr.norm() * 180 / 3.1415926;

//     return;
//   }



//   int optimize_se3(const std::vector<VEC_D_3> &_pts3d_ref, const std::vector<Vec2d> &_pts2d_cur,
//                    const Mat3d &_K_cur, Mat34d &_Tcr, std::vector<uchar> &_inliers,
//                    int _optimize_max_ites) {
//     _inliers.assign(_pts3d_ref.size(), 0);

//     int iterations = _optimize_max_ites;
//     float cost = 0;

//     Mat34d esti_T = _Tcr; // estimated pose

//     float fx = _K_cur(0, 0);
//     float fy = _K_cur(1, 1);

//     for (int iter = 0; iter < iterations; iter++) {
//       Eigen::Matrix<float_, 6, 6> H_GN = Eigen::Matrix<float_, 6, 6>::Zero();
//       VEC_D_6 b_GN = VEC_D_6::Zero();

//       cost = 0;

//       for (size_t i = 0; i < _pts3d_ref.size(); i++) {
//         const VEC_D_3 &p3D = _pts3d_ref[i];
//         const Vec2d &p2D = _pts2d_cur[i];
//         VEC_D_3 pcd = megCV::transfromPoint(esti_T, p3D);
//         Vec2d p2c = megCV::world2cam(pcd, _K_cur);
//         //SEbox::Vec2d err = p2D - p2c;
//         Vec2d err(p2D(0) - p2c(0), p2D(1) - p2c(1));
//         cost += err.dot(err);

//         //SEbox::Mat26d J;
//         Eigen::Matrix<float_, 2, 6> J;

//         J << fx / pcd(2), 0, -fx * pcd(0) / (pcd(2) * pcd(2)), -fx * pcd(0) * pcd(1) / (pcd(2) * pcd(2)),
//             fx + fx * pcd(0) * pcd(0) / (pcd(2) * pcd(2)), -fx * pcd(1) / pcd(2), 0,
//             fy / pcd(2), -fy * pcd(1) / (pcd(2) * pcd(2)), -fy - fy * pcd(1) * pcd(1) / (pcd(2) * pcd(2)),
//             fy * pcd(0) * pcd(1) / (pcd(2) * pcd(2)), fy * pcd(0) / pcd(2);
//         J = -J;

//         H_GN += J.transpose() * J;
//         b_GN += -J.transpose() * err;
//       }

//       VEC_D_6 dx;
//       dx = H_GN.ldlt().solve(b_GN);

//       if (std::isnan(dx[0])) {
//         break;
//       }

//       if (cost < 1e-6)
//         break;

//       POSE_D delta_T = Exp6d(dx);
//       esti_T = megCV::multipyPose(delta_T, esti_T);
//     }

//     _Tcr = esti_T;

//     int inliers_num = 0;

//     for (size_t i = 0; i < _pts3d_ref.size(); i++) {
//       const VEC_D_3 &pt3d = _pts3d_ref[i];
//       Vec2d pt2d_projected = megCV::world2cam(megCV::transfromPoint(_Tcr, pt3d), _K_cur);

//       if (pt2d_projected != Vec2d::Zero()) {
//         const Vec2d &pt2d = _pts2d_cur[i];

//         Vec2d err(pt2d_projected(0) - pt2d(0), pt2d_projected(1) - pt2d(1));

//         float error = err(0) * err(0) + err(1) * err(1);

//         if (error < 4) {
//           _inliers[i] = 1;
//           inliers_num++;
//         } else {
//           _inliers[i] = 0;
//         }
//       }
//     }

//     return inliers_num;
//   }

//   void getWarpMatrixAffine(const Mat3d &cam_ref,
//                            const Mat3d &cam_cur,
//                            const Vec2d &px_ref,
//                            const VEC_D_3 &f_ref,
//                            const double depth_ref,
//                            const Mat3d &R_cur_ref,
//                            const VEC_D_3 &T_cur_ref,
//                            const int level_ref,
//                            const int halfpatch_size,
//                            Mat2d &A_cur_ref) {
//     const VEC_D_3 xyz_ref(f_ref * depth_ref);
//     VEC_D_3 xyz_du_ref = cam2world(px_ref + Vec2d(halfpatch_size, 0) * (1 << level_ref), cam_ref.inverse());
//     VEC_D_3 xyz_dv_ref = cam2world(px_ref + Vec2d(0, halfpatch_size) * (1 << level_ref), cam_ref.inverse());
//     xyz_du_ref *= xyz_ref[2] / xyz_du_ref[2];
//     xyz_dv_ref *= xyz_ref[2] / xyz_dv_ref[2];

//     const Vec2d px_cur = world2cam(R_cur_ref * (xyz_ref) + T_cur_ref, cam_cur);
//     const Vec2d px_du = world2cam(R_cur_ref * (xyz_du_ref) + T_cur_ref, cam_cur);
//     const Vec2d px_dv = world2cam(R_cur_ref * (xyz_dv_ref) + T_cur_ref, cam_cur);
//     A_cur_ref.col(0) = (px_du - px_cur) / halfpatch_size;
//     A_cur_ref.col(1) = (px_dv - px_cur) / halfpatch_size;
//   }

//   void getWarpMatrixAffine(const CamModel &cam_ref,
//                            const CamModel &cam_cur,
//                            const vPoint2f &pt2ds_ref,
//                            const vVEC_D_3 &pt3ds_ref,
//                            const POSE_D &T_cur_ref,
//                            const int halfpatch_size,
//                            vMat2d &A_cur_ref) {
//     vPoint2f pt_du_ref, pt_dv_ref, pt_du_cur, pt_dv_cur, pt_cur;
//     vVEC_D_3 xyz_du_ref, xyz_dv_ref, xyz_cur;
//     pt_du_ref.assign(pt2ds_ref.begin(), pt2ds_ref.end());
//     pt_dv_ref.assign(pt2ds_ref.begin(), pt2ds_ref.end());
//     xyz_cur.resize(pt3ds_ref.size());
//     A_cur_ref.resize(pt3ds_ref.size());
//     size_t i = 0, pts_num = pt2ds_ref.size();
//     for (i = 0; i < pts_num; ++i) {
//       pt_du_ref[i].x += halfpatch_size;
//       pt_dv_ref[i].y += halfpatch_size;
//     }
//     cam_ref.cam2world(pt_du_ref, xyz_du_ref);
//     cam_ref.cam2world(pt_dv_ref, xyz_dv_ref);
//     for (i = 0; i < pts_num; ++i) {
//       xyz_du_ref[i] *= pt3ds_ref[i][2] / xyz_du_ref[i][2];
//       xyz_dv_ref[i] *= pt3ds_ref[i][2] / xyz_dv_ref[i][2];

//       xyz_du_ref[i] = transfromPoint(T_cur_ref, xyz_du_ref[i]);
//       xyz_dv_ref[i] = transfromPoint(T_cur_ref, xyz_dv_ref[i]);
//       xyz_cur[i] = transfromPoint(T_cur_ref, pt3ds_ref[i]);
//     }
//     cam_cur.world2cam(xyz_du_ref, pt_du_cur, true);
//     cam_cur.world2cam(xyz_dv_ref, pt_dv_cur, true);
//     cam_cur.world2cam(xyz_cur, pt_cur, true);
//     for (i = 0; i < pts_num; ++i) {
//       A_cur_ref[i](0, 0) = (pt_du_cur[i].x - pt_cur[i].x) / halfpatch_size;
//       A_cur_ref[i](1, 0) = (pt_du_cur[i].y - pt_cur[i].y) / halfpatch_size;
//       A_cur_ref[i](0, 1) = (pt_dv_cur[i].x - pt_cur[i].x) / halfpatch_size;
//       A_cur_ref[i](1, 1) = (pt_dv_cur[i].y - pt_cur[i].y) / halfpatch_size;

//       A_cur_ref[i].col(0).normalize();
//       A_cur_ref[i].col(1).normalize();
//     }
//   }

//   void triangulate(const POSE_D &T_WC_1, const POSE_D &T_WC_2, const VEC_D_3 &f1, const VEC_D_3 &f2, VEC_D_3 &pw) {
//     POSE_D T_CW_1 = POSE_D::Zero();
//     POSE_D T_CW_2 = POSE_D::Zero();
//     T_CW_1 = inversePose(T_WC_1);
//     T_CW_2 = inversePose(T_WC_2);

//     Mat44d A(4, 4);
//     A.row(0) = f1[0] * T_CW_1.row(2) - f1[2] * T_CW_1.row(0);
//     A.row(1) = f1[1] * T_CW_1.row(2) - f1[2] * T_CW_1.row(1);
//     A.row(2) = f2[0] * T_CW_2.row(2) - f2[2] * T_CW_2.row(0);
//     A.row(3) = f2[1] * T_CW_2.row(2) - f2[2] * T_CW_2.row(1);

//     Eigen::JacobiSVD<Mat44d> mySVD(A, Eigen::ComputeFullV);
//     pw[0] = mySVD.matrixV()(0, 3);
//     pw[1] = mySVD.matrixV()(1, 3);
//     pw[2] = mySVD.matrixV()(2, 3);
//     pw = pw / mySVD.matrixV()(3, 3);
//   }

//   void computeCs(const POSE_D &Rwc_m, const VEC_D_3 &v1, const POSE_D &Rwc_a, const VEC_D_3 &v2, Vec2d &cstheta) {
//     cstheta = Vec2d::Zero();
//     cstheta[0] = (Rwc_m.block<3, 3>(0, 0) * v1).dot(Rwc_a.block<3, 3>(0, 0) * v2);

//     if (fabs(cstheta[0]) >= 0.99999999) {
//       if (cstheta[0] > 0)
//         cstheta[0] = 0.99999999;
//       else
//         cstheta[0] = -0.99999999;
//       cstheta[1] = 0.000141421;
//     } else {
//       cstheta[1] = std::sqrt(1.0 - cstheta[0] *
//                                    cstheta[0]);
//     }
//   }

//   CamModel::CamModel(std::string _model, const std::vector<float> &_params) {
//     setCamModel(_model, _params);
//   }

//   CamModel::CamModel(std::string _model, const std::vector<double> &_params) {
//     setCamModel(_model, _params);
//   }

//   void CamModel::setCamModel(std::string _model, const std::vector<float> &_params) {
//     model = _model;

//     if (_model == "RadiaTan") {
//       // fx, fy, cx, cy, d0, d1, d2, d3, d4
//       if (_params.size() < 4) {
//         printf("error! RadialTanCamera params size < 4");
//         return;
//       }
//       cvK = cv::Mat::eye(3, 3, CV_32FC1);
//       cvK.at<float>(0, 0) = (float) _params[0];
//       cvK.at<float>(1, 1) = (float) _params[1];
//       cvK.at<float>(0, 2) = (float) _params[2];
//       cvK.at<float>(1, 2) = (float) _params[3];
//       cvKinv = cvK.inv();
//       distortParam = cv::Mat::zeros(5, 1, CV_32FC1);
//       if (_params.size() >= 9) {
//         distortParam.at<float>(0) = (float) _params[4];
//         distortParam.at<float>(1) = (float) _params[5];
//         distortParam.at<float>(2) = (float) _params[6];
//         distortParam.at<float>(3) = (float) _params[7];
//         distortParam.at<float>(4) = (float) _params[8];
//       }
//       pt3d.reserve(200);
//       pt2d.reserve(200);
//       alreadySet = true;
//       return;
//     }

//     if (_model == "Equi") {
//       // fx, fy, cx, cy, d0, d1, d2, d3
//       if (_params.size() < 4) {
//         printf("error! EquiCamera params size < 4");
//         return;
//       }
//       cvK = cv::Mat::eye(3, 3, CV_32FC1);
//       cvK.at<float>(0, 0) = (float) _params[0];
//       cvK.at<float>(1, 1) = (float) _params[1];
//       cvK.at<float>(0, 2) = (float) _params[2];
//       cvK.at<float>(1, 2) = (float) _params[3];
//       cvKinv = cvK.inv();
//       distortParam = cv::Mat::zeros(4, 1, CV_32FC1);
//       if (_params.size() >= 8) {
//         distortParam.at<float>(0) = (float) _params[4];
//         distortParam.at<float>(1) = (float) _params[5];
//         distortParam.at<float>(2) = (float) _params[6];
//         distortParam.at<float>(3) = (float) _params[7];
//       }
//       und_pt2d.reserve(200);
//       dis_pt2d.reserve(200);
//       alreadySet = true;
//       return;
//     }

//     if (_model == "DoubleShpere") {
//       // fx, fy, cx, cy, xi, alpha
//       if (_params.size() != 6) {
//         printf("error! DoubleShpereCamera params size != 6");
//         return;
//       }
//       cvK = cv::Mat::eye(3, 3, CV_32FC1);
//       cvK.at<float>(0, 0) = (float) _params[0];
//       cvK.at<float>(1, 1) = (float) _params[1];
//       cvK.at<float>(0, 2) = (float) _params[2];
//       cvK.at<float>(1, 2) = (float) _params[3];
//       xi = (float) _params[4];
//       alpha = (float) _params[5];
//       alreadySet = true;
//       return;
//     }
//   }

//   void CamModel::setCamModel(std::string _model, const std::vector<double> &_params) {
//     model = _model;

//     if (_model == "RadiaTan") {
//       // fx, fy, cx, cy, d0, d1, d2, d3, d4
//       if (_params.size() < 4) {
//         printf("error! RadialTanCamera params size < 4");
//         return;
//       }
//       cvK = cv::Mat::eye(3, 3, CV_32FC1);
//       cvK.at<float>(0, 0) = (float) _params[0];
//       cvK.at<float>(1, 1) = (float) _params[1];
//       cvK.at<float>(0, 2) = (float) _params[2];
//       cvK.at<float>(1, 2) = (float) _params[3];
//       cvKinv = cvK.inv();
//       distortParam = cv::Mat::zeros(5, 1, CV_32FC1);
//       if (_params.size() >= 9) {
//         distortParam.at<float>(0) = (float) _params[4];
//         distortParam.at<float>(1) = (float) _params[5];
//         distortParam.at<float>(2) = (float) _params[6];
//         distortParam.at<float>(3) = (float) _params[7];
//         distortParam.at<float>(4) = (float) _params[8];
//       }
//       pt3d.reserve(200);
//       pt2d.reserve(200);
//       alreadySet = true;
//       return;
//     }

//     if (_model == "Equi") {
//       // fx, fy, cx, cy, d0, d1, d2, d3
//       if (_params.size() < 4) {
//         printf("error! EquiCamera params size < 4");
//         return;
//       }
//       cvK = cv::Mat::eye(3, 3, CV_32FC1);
//       cvK.at<float>(0, 0) = (float) _params[0];
//       cvK.at<float>(1, 1) = (float) _params[1];
//       cvK.at<float>(0, 2) = (float) _params[2];
//       cvK.at<float>(1, 2) = (float) _params[3];
//       cvKinv = cvK.inv();
//       distortParam = cv::Mat::zeros(4, 1, CV_32FC1);
//       if (_params.size() >= 8) {
//         distortParam.at<float>(0) = (float) _params[4];
//         distortParam.at<float>(1) = (float) _params[5];
//         distortParam.at<float>(2) = (float) _params[6];
//         distortParam.at<float>(3) = (float) _params[7];
//       }
//       und_pt2d.reserve(200);
//       dis_pt2d.reserve(200);
//       alreadySet = true;
//       return;
//     }

//     if (_model == "DoubleShpere") {
//       // fx, fy, cx, cy, xi, alpha
//       if (_params.size() != 6) {
//         printf("error! DoubleShpereCamera params size != 6");
//         return;
//       }
//       cvK = cv::Mat::eye(3, 3, CV_32FC1);
//       cvK.at<float>(0, 0) = (float) _params[0];
//       cvK.at<float>(1, 1) = (float) _params[1];
//       cvK.at<float>(0, 2) = (float) _params[2];
//       cvK.at<float>(1, 2) = (float) _params[3];
//       xi = (float) _params[4];
//       alpha = (float) _params[5];
//       alreadySet = true;
//       return;
//     }
//   }

//   VEC_D_3 CamModel::cam2world(const cv::Point2f &px) const {
//     if (model == "RadiaTan") {
//       VEC_D_3 pt = VEC_D_3::Ones();
//       cv::Mat pxMat = cv::Mat::zeros(1, 1, CV_32FC2);
//       cv::Mat pxUMat = cv::Mat::zeros(1, 1, CV_32FC2);
//       pxMat.at<float>(0, 0) = px.x;
//       pxMat.at<float>(0, 1) = px.y;
//       cv::undistortPoints(pxMat, pxUMat, cvK, distortParam);
//       pt[0] = (double) pxUMat.at<float>(0, 0);
//       pt[1] = (double) pxUMat.at<float>(0, 1);

//       return pt.normalized();
//     }

//     if (model == "Equi") {
//       VEC_D_3 pt = VEC_D_3::Ones();
//       cv::Mat dis_pt2d_ = cv::Mat::zeros(1, 1, CV_32FC2);
//       dis_pt2d_.at<float>(0, 0) = px.x;
//       dis_pt2d_.at<float>(0, 1) = px.y;
//       cv::Mat und_pt2d_ = cv::Mat::zeros(1, 1, CV_32FC2);
//       cv::fisheye::undistortPoints(dis_pt2d_, und_pt2d_, cvK, distortParam);
//       pt[0] = (double) und_pt2d_.at<float>(0, 0);
//       pt[1] = (double) und_pt2d_.at<float>(0, 1);

//       return pt.normalized();
//     }

//     if (model == "DoubleShpere") {
//       float_ mx = ((float_) px.x - cvK.at<float>(0, 2)) / cvK.at<float>(0, 0);
//       float_ my = ((float_) px.y - cvK.at<float>(1, 2)) / cvK.at<float>(1, 1);
//       float_ r2 = mx * mx + my * my;
//       float_ limit = 1.0 / (2.0 * alpha - 1.0);
//       if (alpha > 0.5) {
//         if (r2 > limit) {
//           return VEC_D_3::Zero();
//         }
//       }
//       float_ mz = (1.0 - alpha * alpha * r2) /
//                   (alpha * sqrt(1.0 - (alpha * 2.0 - 1.0) * r2) + 1.0 - alpha);

//       float_ scale = (mz * xi + sqrt(mz * mz + (1.0 - xi * xi) * r2)) / (mz * mz + r2);
//       VEC_D_3 xyz = scale * VEC_D_3(mx, my, mz) - VEC_D_3(0, 0, xi);
//       xyz.normalize();
//       return xyz;
//     }

//     return VEC_D_3::Zero();
//   }

//   cv::Point2f CamModel::world2cam(const VEC_D_3 &pt, bool distort) const {
//     if (distort) {
//       if (model == "RadiaTan") {
//         cv::Point2f px;
//         cv::Mat t = cv::Mat::zeros(3, 1, CV_32FC1);
//         cv::Mat pt3d_ = cv::Mat::zeros(1, 1, CV_32FC3);
//         pt3d_.at<float>(0, 0) = (float) pt[0];
//         pt3d_.at<float>(0, 1) = (float) pt[1];
//         pt3d_.at<float>(0, 2) = (float) pt[2];
//         cv::Mat pt2d_ = cv::Mat::zeros(1, 1, CV_32FC2);
//         cv::projectPoints(pt3d_, t, t, cvK, distortParam, pt2d_);
//         px.x = pt2d_.at<float>(0, 0);
//         px.y = pt2d_.at<float>(0, 1);
//         return px;
//       }

//       if (model == "Equi") {
//         cv::Point2f px;
//         cv::Mat t = cv::Mat::zeros(3, 1, CV_32FC1);
//         cv::Mat und_pt2d_ = cv::Mat::zeros(1, 1, CV_32FC2);
//         und_pt2d_.at<float>(0, 0) = (float) (pt[0] / pt[2]);
//         und_pt2d_.at<float>(0, 1) = (float) (pt[1] / pt[2]);
//         cv::Mat pt2d_ = cv::Mat::zeros(1, 1, CV_32FC2);
//         cv::fisheye::distortPoints(und_pt2d_, pt2d_, cvK, distortParam);
//         px.x = pt2d_.at<float>(0, 0);
//         px.y = pt2d_.at<float>(0, 1);
//         return px;
//       }

//       if (model == "DoubleShpere") {
//         float_ w1 = 0.0;
//         if (alpha > 0.5) {
//           w1 = (1.0 - alpha) / alpha;
//         } else {
//           w1 = alpha / (1.0 - alpha);
//         }
//         float_ w2 = (w1 + xi) / sqrt(2.0 * w1 * xi + xi * xi + 1.0);
//         float_ d1 = pt.norm();
//         float_ d2 = sqrt(pt[0] * pt[0] + pt[1] * pt[1] +
//                          (xi * d1 + pt[2]) * (xi * d1 + pt[2]));
//         Vec2d uv = Vec2d::Zero();
//         uv[0] = cvK.at<float>(0, 0) * (pt[0]) / (alpha * d2 + (1 - alpha) * (xi * d1 + pt[2])) +
//                 cvK.at<float>(0, 2);
//         uv[1] = cvK.at<float>(1, 1) * (pt[1]) / (alpha * d2 + (1 - alpha) * (xi * d1 + pt[2])) +
//                 cvK.at<float>(1, 2);
//         if (pt[2] <= -w2 * d1) {
//           uv[0] = -1;
//           uv[1] = -1;
// //            std::cout << "function :DS_world2cam : bad bearing:" << bearing.transpose() << " uv:" << uv.transpose() << std::endl;//if this ocurrs please contact chun
//         }
//         cv::Point2f pt2d(uv[0], uv[1]);
//         return pt2d;
//       }
//     } else {
//       VEC_D_3 pt2 = pt / pt(2);
//       cv::Point2f px;
//       px.x = cvK.at<float>(0, 0) * pt2(0) + cvK.at<float>(0, 2);
//       px.y = cvK.at<float>(1, 1) * pt2(1) + cvK.at<float>(1, 2);
//       return px;
//     }

//     return cv::Point2f(0, 0);
//   }

//   void CamModel::world2cam(const vVEC_D_3 &pts3D, std::vector<cv::Point2f> &pts2D, bool distort) const {
//     if (pts3D.empty())
//       return;

//     if (distort) {
//       if (model == "RadiaTan") {
//         pts2D.clear();
//         pt3d.clear();
//         for (size_t i = 0; i < pts3D.size(); ++i) {
//           pt3d.emplace_back(pts3D[i][0], pts3D[i][1], pts3D[i][2]);
//         }
//         cv::Mat t = cv::Mat::zeros(3, 1, CV_32FC1);
//         cv::projectPoints(pt3d, t, t, cvK, distortParam, pts2D);
//       }

//       if (model == "Equi") {
//         pts2D.clear();
//         und_pt2d.clear();
//         for (size_t i = 0; i < pts3D.size(); ++i) {
//           und_pt2d.emplace_back(pts3D[i][0] / pts3D[i][2], pts3D[i][1] / pts3D[i][2]);
//         }
//         cv::Mat t = cv::Mat::zeros(3, 1, CV_32FC1);
//         cv::fisheye::distortPoints(und_pt2d, pts2D, cvK, distortParam);
//       }

//       if (model == "DoubleShpere") {
//         float_ w1 = 0.0;
//         if (alpha > 0.5) {
//           w1 = (1.0 - alpha) / alpha;
//         } else {
//           w1 = alpha / (1.0 - alpha);
//         }
//         float_ w2 = (w1 + xi) / sqrt(2.0 * w1 * xi + xi * xi + 1.0);
//         pts2D.clear();
//         for (size_t i = 0; i < pts3D.size(); ++i) {
//           const VEC_D_3 &pt = pts3D[i];
//           float_ d1 = pt.norm();
//           float_ d2 = sqrt(pt[0] * pt[0] + pt[1] * pt[1] +
//                            (xi * d1 + pt[2]) * (xi * d1 + pt[2]));
//           Vec2d uv = Vec2d::Zero();
//           uv[0] = cvK.at<float>(0, 0) * (pt[0]) / (alpha * d2 + (1 - alpha) * (xi * d1 + pt[2])) +
//                   cvK.at<float>(0, 2);
//           uv[1] = cvK.at<float>(1, 1) * (pt[1]) / (alpha * d2 + (1 - alpha) * (xi * d1 + pt[2])) +
//                   cvK.at<float>(1, 2);
//           if (pt[2] <= -w2 * d1) {
//             uv[0] = -1;
//             uv[1] = -1;
//           }
//           pts2D.emplace_back(uv[0], uv[1]);
//         }
//       }
//     } else {
//       pts2D.resize(pts3D.size());
//       for (size_t i = 0; i < pts3D.size(); ++i) {
//         pts2D[i] = world2cam(pts3D[i], false);
//       }
//     }
//   }

//   void CamModel::cam2world(const std::vector<cv::Point2f> &pts2D, vVEC_D_3 &pts3D) const {
//     if (pts2D.empty())
//       return;

//     if (pts3D.size() != pts2D.size())
//       pts3D.resize(pts2D.size());

//     if (model == "RadiaTan") {
//       cv::Mat pxMat = cv::Mat::zeros(pts2D.size(), 1, CV_32FC2);
//       cv::Mat pxUMat;
// //            cv::Mat pxUMat = cv::Mat::zeros(pts2D.size(), 1, CV_32FC2);
//       for (size_t i = 0; i < pts2D.size(); ++i) {
//         pxMat.at<float>(i, 0) = pts2D[i].x;
//         pxMat.at<float>(i, 1) = pts2D[i].y;
//       }
//       cv::undistortPoints(pxMat, pxUMat, cvK, distortParam);
//       for (size_t i = 0; i < pts2D.size(); ++i) {
//         pts3D[i][0] = (double) pxUMat.at<float>(i, 0);
//         pts3D[i][1] = (double) pxUMat.at<float>(i, 1);
//         pts3D[i][2] = 1;
//         pts3D[i].normalize();
//       }
//     }

//     if (model == "Equi") {
//       cv::Mat pxMat = cv::Mat::zeros(pts2D.size(), 1, CV_32FC2);
//       cv::Mat pxUMat;
// //            cv::Mat pxUMat = cv::Mat::zeros(pts2D.size(), 1, CV_32FC2);
//       for (size_t i = 0; i < pts2D.size(); ++i) {
//         pxMat.at<float>(i, 0) = pts2D[i].x;
//         pxMat.at<float>(i, 1) = pts2D[i].y;
//       }
//       cv::fisheye::undistortPoints(pxMat, pxUMat, cvK, distortParam);
//       for (size_t i = 0; i < pts2D.size(); ++i) {
//         pts3D[i][0] = (double) pxUMat.at<float>(i, 0);
//         pts3D[i][1] = (double) pxUMat.at<float>(i, 1);
//         pts3D[i][2] = 1;
//         pts3D[i].normalize();
//       }
//     }

//     if (model == "DoubleShpere") {
//       for (size_t i = 0; i < pts2D.size(); ++i) {
//         float_ mx = ((float_) pts2D[i].x - cvK.at<float>(0, 2)) / cvK.at<float>(0, 0);
//         float_ my = ((float_) pts2D[i].y - cvK.at<float>(1, 2)) / cvK.at<float>(1, 1);
//         float_ r2 = mx * mx + my * my;
//         float_ limit = 1.0 / (2.0 * alpha - 1.0);
//         if (alpha > 0.5) {
//           if (r2 > limit) {
//             pts3D[i] = VEC_D_3::Zero();
//           }
//         }

//         float_ mz = (1.0 - alpha * alpha * r2) /
//                     (alpha * sqrt(1.0 - (alpha * 2.0 - 1.0) * r2) + 1.0 - alpha);

//         float_ scale = (mz * xi + sqrt(mz * mz + (1.0 - xi * xi) * r2)) / (mz * mz + r2);
//         pts3D[i] = scale * VEC_D_3(mx, my, mz) - VEC_D_3(0, 0, xi);
//         pts3D[i].normalize();
//       }
//     }
//   }

//   std::string CamModel::getModelName() const {
//     return model;
//   }

//   void CamModel::getIntrinsic(std::vector<float> &_intrinsic) const {
//     // fx, fy, cx, cy
//     _intrinsic.push_back((float) cvK.at<float>(0, 0));
//     _intrinsic.push_back((float) cvK.at<float>(1, 1));
//     _intrinsic.push_back((float) cvK.at<float>(0, 2));
//     _intrinsic.push_back((float) cvK.at<float>(1, 2));

//     // fx, fy, cx, cy, d0, d1, d2, d3, d4
//     if (model == "RadiaTan") {
//       _intrinsic.push_back((float) distortParam.at<float>(0));
//       _intrinsic.push_back((float) distortParam.at<float>(1));
//       _intrinsic.push_back((float) distortParam.at<float>(2));
//       _intrinsic.push_back((float) distortParam.at<float>(3));
//       _intrinsic.push_back((float) distortParam.at<float>(4));
//     }

//     // fx, fy, cx, cy, d0, d1, d2, d3
//     if (model == "Equi") {
//       _intrinsic.push_back((float) distortParam.at<float>(0));
//       _intrinsic.push_back((float) distortParam.at<float>(1));
//       _intrinsic.push_back((float) distortParam.at<float>(2));
//       _intrinsic.push_back((float) distortParam.at<float>(3));
//     }

//     // fx, fy, cx, cy, xi, alpha
//     if (model == "DoubleShpere") {
//       _intrinsic.push_back((float) xi);
//       _intrinsic.push_back((float) alpha);
//     }
//   }

//   void CamModel::getIntrinsic(std::vector<double> &_intrinsic) const {
//     // fx, fy, cx, cy
//     _intrinsic.push_back((double) cvK.at<float>(0, 0));
//     _intrinsic.push_back((double) cvK.at<float>(1, 1));
//     _intrinsic.push_back((double) cvK.at<float>(0, 2));
//     _intrinsic.push_back((double) cvK.at<float>(1, 2));

//     // fx, fy, cx, cy, d0, d1, d2, d3, d4
//     if (model == "RadiaTan") {
//       _intrinsic.push_back((double) distortParam.at<float>(0));
//       _intrinsic.push_back((double) distortParam.at<float>(1));
//       _intrinsic.push_back((double) distortParam.at<float>(2));
//       _intrinsic.push_back((double) distortParam.at<float>(3));
//       _intrinsic.push_back((double) distortParam.at<float>(4));
//     }

//     // fx, fy, cx, cy, d0, d1, d2, d3
//     if (model == "Equi") {
//       _intrinsic.push_back((double) distortParam.at<float>(0));
//       _intrinsic.push_back((double) distortParam.at<float>(1));
//       _intrinsic.push_back((double) distortParam.at<float>(2));
//       _intrinsic.push_back((double) distortParam.at<float>(3));
//     }

//     // fx, fy, cx, cy, xi, alpha
//     if (model == "DoubleShpere") {
//       _intrinsic.push_back((double) xi);
//       _intrinsic.push_back((double) alpha);
//     }
//   }
