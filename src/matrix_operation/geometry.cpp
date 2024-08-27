#include "geometry.h"
#include <assert.h>
// #include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include "stdio.h"
#include <cstring>
#include "svd.h"
#include "qrdcmp.h"

  void ac_MAT_D_3_3_print(const MAT_D_3_3 mat) {
    uint32_t i = 0U;
    for (i = 0U; i < 3U; i++)
    {
        printf("%f, %f, %f\n", mat[i][0], mat[i][1], mat[i][2]);
    }
  }

  void ac_POSE_D_print(const POSE_D mat) {
    for (uint32_t i = 0U; i < 3U; i++)
    {
      printf("%f, %f, %f, %f\n", mat[i][0], mat[i][1], mat[i][2], mat[i][3]);
    }
    printf("%f, %f, %f, %f\n", 0.0, 0.0, 0.0, 1.0);
  }

  // void ac_VEC_D_3_print(const VEC_D_3 vec) {
  //   printf("%f, %f, %f", vec[0], vec[1], vec[2]);
  // }


  uint32_t copy_VEC_D_3(const VEC_D_3 v_in, VEC_D_3 v_out) {
    v_out[0] = v_in[0];
    v_out[1] = v_in[1];
    v_out[2] = v_in[2];
    return 0;
  }

  uint32_t copy_Mat33d(const MAT_D_3_3 mat_in, MAT_D_3_3 mat_out) {
    for (uint32_t i = 0; i < 3; i ++ ) {
      for (uint32_t j = 0; j < 3; j ++ ) {
        mat_out[i][j] = mat_in[i][j];
      }
    }
    return 0;
  }

  uint32_t copy_POSE_D(const POSE_D pose_in, POSE_D pose_out) {
    for (uint32_t i = 0; i < 3; i ++ ) {
      for (uint32_t j = 0; j < 4; j ++ ) {
        pose_out[i][j] = pose_in[i][j];
      }
    }
    return 0;
  }

  uint32_t transpose_Mat33d(const MAT_D_3_3 mat_in, MAT_D_3_3 mat_out) {
    for (uint32_t i = 0; i < 3; i ++ ) {
      for (uint32_t j = 0; j < 3; j ++ ) {
        mat_out[i][j] = mat_in[j][i];
      }
    }
  }

  uint32_t transpose_Mat33d_inplace(MAT_D_3_3 mat) {
    for (uint32_t i = 0; i < 3; i ++ ) {
      for (uint32_t j = i+1; j < 3; j ++ ) {
        double tmp = mat[i][j];
        mat[i][j] = mat[j][i];
        mat[j][i] = tmp;
      }
    }
  }

  uint32_t set_Identity_POSE_D(POSE_D pose) {
    memset(pose, 0, 12*sizeof(double));
    pose[0][0] = 1.0;
    pose[1][1] = 1.0;
    pose[2][2] = 1.0;
    return 0;
  }

  uint32_t set_Translate_POSE_D(POSE_D pose, const VEC_D_3 trans) {
    for (uint32_t i = 0; i < 3; i ++ ) {
        pose[i][3] = trans[i];
    }
    return 0;
  }

  uint32_t set_Rotation_POSE_D(POSE_D pose, const MAT_D_3_3 rotation) {
    for (uint32_t i = 0; i < 3; i ++ ) {
      for (uint32_t j = 0; j < 3; j ++ ) {
        pose[i][j] = rotation[i][j];
      }
    }
    return 0;
  }

  uint32_t Pose_Translate_part(const POSE_D pose, VEC_D_3 trans) {
    trans[0] = pose[0][3];
    trans[1] = pose[1][3];
    trans[2] = pose[2][3];
  }

  uint32_t Pose_Rotation_part(const POSE_D pose, MAT_D_3_3 rotation) {
    for (uint32_t i = 0U; i < 3; i++) {
      for (uint32_t j = 0U; j < 3; j++) {
        rotation[i][j] = pose[i][j];
      }
    }
  }

  uint32_t set_Identity_Mat33d(MAT_D_3_3 mat) {
    memset(mat, 0, 9*sizeof(double));
    mat[0][0] = 1;
    mat[1][1] = 1;
    mat[2][2] = 1;
  }

  uint32_t Mat33D_Vec3D_multiply(const MAT_D_3_3 mat, const VEC_D_3 v, VEC_D_3 res) {
    memset(res, 0, sizeof(double) * 3);
    for (uint32_t i = 0U; i < 3; i++) {
      for (uint32_t j = 0U; j < 3; j++) {
        res[i] += mat[i][j] * v[j];
      }
    }
    return 0;
  }

  uint32_t Mat33D_Vec3D_multiply_inplace(const MAT_D_3_3 mat, VEC_D_3 v) {
    VEC_D_3 tmp = {0.0, 0.0, 0.0};
    for (uint32_t i = 0U; i < 3; i++) {
      for (uint32_t j = 0U; j < 3; j++) {
        tmp[i] += mat[i][j] * v[j];
      }
    }
    copy_VEC_D_3(tmp, v);
    return 0;
  }

  uint32_t MAT33D_times(const MAT_D_3_3 mat, double num, MAT_D_3_3 res) {
    uint32_t N = 3, M = 3;
    for (uint32_t i = 0U; i < N; i++) {
      for (uint32_t j = 0U; j < M; j++) {
          res[i][j] = mat[i][j] * num;
        }
      }
    return 0;
  }

  uint32_t MAT33D_times_inplace(MAT_D_3_3 mat, double num) {
    uint32_t N = 3, M = 3;
    for (uint32_t i = 0U; i < N; i++) {
      for (uint32_t j = 0U; j < M; j++) {
          mat[i][j] = mat[i][j] * num;
        }
      }
    return 0;
  }

  uint32_t MAT33D_matrix_operation(const MAT_D_3_3 factor1, const MAT_D_3_3 factor2, char oper, MAT_D_3_3 res) {
    uint32_t N = 3, M = 3;
    switch (oper)
    {
    case '+':
        for (uint32_t i = 0U; i < N; i++) {
            for (uint32_t j = 0U; j < M; j++) {
                res[i][j] = factor1[i][j] + factor2[i][j];
            }
        }
        return 0;
    case '-':
        for (uint32_t i = 0U; i < N; i++) {
            for (uint32_t j = 0U; j < M; j++) {
                res[i][j] = factor1[i][j] - factor2[i][j];
            }
        }
        return 0;
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
        return 0;
    case '.':
        for (uint32_t i = 0U; i < N; i++) {
            for (uint32_t j = 0U; j < M; j++) {
                res[i][j] = factor1[i][j] * factor2[i][j];
            }
        }
        return 0;
    case '/':
        for (uint32_t i = 0U; i < N; i++) {
            for (uint32_t j = 0U; j < M; j++) {
                if (fabs(factor2[i][j]) < EPSILON)
                {
                    return 1;
                }
                else
                {
                    res[i][j] = factor1[i][j] / factor2[i][j];
                }
            }
        }
        return 0;
    default:
        return 1;
    }
  }

  // operation in place, save result in factor2
  uint32_t MAT33D_matrix_operation_inplace(const MAT_D_3_3 factor1, MAT_D_3_3 factor2, char oper) {
    uint32_t N = 3, M = 3;
    switch (oper)
    {
    case '+':
        for (uint32_t i = 0U; i < N; i++) {
            for (uint32_t j = 0U; j < M; j++) {
                factor2[i][j] = factor1[i][j] + factor2[i][j];
            }
        }
        return 0;
    case '-':
        for (uint32_t i = 0U; i < N; i++) {
            for (uint32_t j = 0U; j < M; j++) {
                factor2[i][j] = factor1[i][j] - factor2[i][j];
            }
        }
        return 0;
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
        return 0;
    }
    case '.':
        for (uint32_t i = 0U; i < N; i++) {
            for (uint32_t j = 0U; j < M; j++) {
                factor2[i][j] = factor1[i][j] * factor2[i][j];
            }
        }
        return 0;
    case '/':
        for (uint32_t i = 0U; i < N; i++) {
            for (uint32_t j = 0U; j < M; j++) {
                if (fabs(factor2[i][j]) < EPSILON)
                {
                    return 0;
                }
                else
                {
                    factor2[i][j] = factor1[i][j] / factor2[i][j];
                }
            }
        }
        return 0;
    default:
        return 1;
    }
  }

  uint32_t skewd(const VEC_D_3 v, MAT_D_3_3 mat) {
    memset(mat, 0, 9*sizeof(double));
    mat[0][1] = -v[2];
    mat[0][2] = v[1];
    mat[1][0] = v[2];
    mat[1][2] = -v[0];
    mat[2][0] = -v[1];
    mat[2][1] = v[0];
    return 0;
  }

  double norm_V3d(const VEC_D_3 v) {
    double res = v[0] *v[0] + v[1] * v[1] + v[2] * v[2];
    return sqrt(res);
  }

  uint32_t Log_SO3d(const MAT_D_3_3 R, VEC_D_3 v) {
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

    return 0;
  }

  uint32_t Exp3d(const VEC_D_3 _dx, MAT_D_3_3 R) {

  }

  // uint32_t Exp6d(const VEC_D_6 _dx, POSE_D T) {
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
  uint32_t mat2qua(const MAT_D_3_3 m, Quaternion_D qua)
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
    return 0;
  }

  uint32_t Exp6d(const VEC_D_6 _dx, POSE_D pose) {
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
    return 0;
  }

  uint32_t inversePose(const POSE_D pose_in, POSE_D pose_out) { 
      MAT_D_3_3 rotation;
      VEC_D_3 trans;
      Pose_Rotation_part(pose_in, rotation);
      Pose_Translate_part(pose_in, trans);

      transpose_Mat33d_inplace(rotation);
      set_Rotation_POSE_D(pose_out, rotation);
      MAT33D_times_inplace(rotation, -1.0);
      Mat33D_Vec3D_multiply_inplace(rotation, trans);
      set_Translate_POSE_D(pose_out, trans);

      return 0; 
  }


  uint32_t inversePose_inplace(POSE_D pose) { 
      MAT_D_3_3 rotation;
      VEC_D_3 trans;
      Pose_Rotation_part(pose, rotation);
      Pose_Translate_part(pose, trans);

      transpose_Mat33d_inplace(rotation);
      set_Rotation_POSE_D(pose, rotation);
      MAT33D_times_inplace(rotation, -1.0);
      Mat33D_Vec3D_multiply_inplace(rotation, trans);
      set_Translate_POSE_D(pose, trans);

      return 0; 
  }

  uint32_t multipyPose(const POSE_D lpose, const POSE_D rpose, POSE_D res) { \
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
    return 0;
  }

  // operation in place, save result in rpose
  uint32_t multipyPose_inplace(const POSE_D lpose, POSE_D rpose) { 
    POSE_D tmp;
    multipyPose(lpose, rpose, tmp);
    copy_POSE_D(tmp, rpose);
    return 0;
  }

void NEW_MAT_DYNAMIC_D(MAT_DYNAMIC_D* mat, const uint32_t rows, const uint32_t cols) {
  mat->rows = rows;
  mat->cols = cols;
  mat->p = (double**)calloc(rows, sizeof(double*));
  for (uint32_t i = 0; i < rows; ++i) {
    mat->p[i] = (double*)calloc(cols, sizeof(double));
  }
}

void FREE_MAT_DYNAMIC_D(MAT_DYNAMIC_D* mat) {
  for (uint32_t i = 0; i < mat->rows; ++i) {
    free(mat->p[i]);
  }
  free(mat->p);
  mat->rows = 0;
  mat->cols = 0;
}

// svd = U * D * V^T
void NEW_SVD_DYNAMIC_D(SVD_DYNAMIC_D* svd, const uint32_t mat_rows, const uint32_t mat_cols) {
  NEW_MAT_DYNAMIC_D(&svd->U, mat_rows, mat_rows);
  NEW_MAT_DYNAMIC_D(&svd->D, mat_rows, mat_cols);
  NEW_MAT_DYNAMIC_D(&svd->V, mat_cols, mat_cols);

  return;
}

void FREE_SVD_DYNAMIC_D(SVD_DYNAMIC_D* svd) {
  FREE_MAT_DYNAMIC_D(&svd->U);
  FREE_MAT_DYNAMIC_D(&svd->D);
  FREE_MAT_DYNAMIC_D(&svd->V);

  return;
}

void SET_ZERO_MAT_DYNAMIC_D(MAT_DYNAMIC_D* mat) {
  for (uint32_t i = 0; i < mat->rows; ++i) {
    for (uint32_t j = 0; j < mat->cols; ++j) {
      mat->p[i][j] = 0.0;
    }
  }
}

void SVD_MAT_D_3_3(MAT_D_3_3 m, MAT_D_3_3 U, MAT_D_3_3 V, MAT_D_3_3 D)
{
    MatDoub A_T(3, 3);
    for (uint32_t i = 0; i < 3; i ++ )
    {
      for (uint32_t j = 0; j < 3; j ++ )
      {
        A_T[i][j] = m[j][i];
      }
    }
    SVD sv(A_T);

    for (uint32_t i = 0; i < 3; i ++ )
    {
      for (uint32_t j = 0; j < 3; j ++ )
      {
        U[i][j] = sv.v[i][j];
      }
    }
    for (uint32_t i = 0; i < 3; i ++ )
    {
      for (uint32_t j = 0; j < 3; j ++ )
      {
        D[i][j] = i == j ? sv.w[i] : 0.0;
      }
    }

}

// Naive SVD implementation (for educational purposes only)
void SVD_MAT_DYNAMIC_D(MAT_DYNAMIC_D* mat, SVD_DYNAMIC_D* svd) {
  assert(svd->U.rows == mat->rows && svd->U.cols == mat->cols);
  assert(svd->D.rows == mat->rows && svd->D.cols == mat->rows);

  MatDoub A_T(mat->cols, mat->rows);
  for (uint32_t i = 0; i < mat->cols; i ++ )
  {
    for (uint32_t j = 0; j < mat->rows; j ++ )
    {
      A_T[i][j] = mat->p[j][i];
    }
  }
  SVD sv(A_T);

  for (uint32_t i = 0; i < svd->U.rows; i ++ )
  {
    for (uint32_t j = 0; j < svd->U.cols; j ++ )
    {
      svd->U.p[i][j] = sv.v[i][j];
    }
  }
  for (uint32_t i = 0; i < svd->D.rows; i ++ )
  {
    for (uint32_t j = 0; j < svd->D.cols; j ++ )
    {
      svd->D.p[i][j] = i == j ? sv.w[i] : 0.0;
    }
  }
}

// Copy matrix data from src to dest
void COPY_MAT_DYNAMIC_D(MAT_DYNAMIC_D* src, MAT_DYNAMIC_D* dest) {
  for (uint32_t i = 0; i < src->rows; ++i) {
    for (uint32_t j = 0; j < src->cols; ++j) {
      dest->p[i][j] = src->p[i][j];
    }
  }
}


// ATA = A^T * A
void ATA_MAT_DYNAMIC_D(MAT_DYNAMIC_D* A, MAT_DYNAMIC_D* ATA) {
  assert(A->cols == ATA->rows);
  assert(ATA->cols == ATA->rows);

  int m = A->rows;
  int n = A->cols;
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      double val = 0.0;
      for (int k = 0; k < m; ++k) {
        val += A->p[k][i] * A->p[k][j];
      }
      ATA->p[i][j] = val;
    }
  }

  return;
}

 void TRANSPOSE_MAT_DYNAMIC_D(MAT_DYNAMIC_D* A, MAT_DYNAMIC_D* AT) {
  assert(A->rows == AT->cols);
  assert(A->cols == AT->rows);

  for (int i = 0; i < AT->rows; ++i) {
    for (int j = 0; j < AT->cols; ++j) {
      AT->p[i][j] = A->p[j][i];
    }
  }

  return;
 }

// Solve a system of linear equations Ax = b using Gaussian elimination with partial pivoting.
// A is an n-by-n matrix, b is an n-by-1 matrix (column vector), and x is the solution vector.
// This function modifies the input matrices A and b.
void SOLVE_A_x_b_GaussianElimination(MAT_DYNAMIC_D* A, MAT_DYNAMIC_D* b, MAT_DYNAMIC_D* x) {
  uint32_t n = A->rows;
  for (uint32_t i = 0; i < n; ++i) {
    // Partial pivoting
    double max = fabs(A->p[i][i]);
    uint32_t maxRow = i;
    for (uint32_t k = i + 1; k < n; ++k) {
      if (fabs(A->p[k][i]) > max) {
        max = fabs(A->p[k][i]);
        maxRow = k;
      }
    }
    // Swap rows
    for (uint32_t k = i; k < n; ++k) {
      double tmp = A->p[maxRow][k];
      A->p[maxRow][k] = A->p[i][k];
      A->p[i][k] = tmp;
    }
    double tmp = b->p[maxRow][0];
    b->p[maxRow][0] = b->p[i][0];
    b->p[i][0] = tmp;

    // Eliminate column below pivot
    for (uint32_t k = i + 1; k < n; ++k) {
      double c = -A->p[k][i] / A->p[i][i];
      for (uint32_t j = i; j < n; ++j) {
        if (i == j) {
          A->p[k][j] = 0;
        } else {
          A->p[k][j] += c * A->p[i][j];
        }
      }
      b->p[k][0] += c * b->p[i][0];
    }
  }

  // Solve for solution vector x
  NEW_MAT_DYNAMIC_D(x, n, 1);
  for (int32_t i = n - 1; i >= 0; --i) {
    x->p[i][0] = b->p[i][0] / A->p[i][i];
    for (int32_t k = i - 1; k >= 0; --k) {
      b->p[k][0] -= A->p[k][i] * x->p[i][0];
    }
  }
}

void SOLVE_A_x_b_MAT_by_SVD_MAT_DYNAMIC_D(MAT_DYNAMIC_D* A, MAT_DYNAMIC_D* b, MAT_DYNAMIC_D* x)
{
  assert(A->cols == x->rows && A->rows == b->rows);
  assert(b->cols == 1 && x->cols == 1);

  MatDoub A_(A->rows, A->cols);
  for (uint32_t i = 0; i < A->rows; i ++ )
  {
    for (uint32_t j = 0; j < A->cols; j ++ )
    {
      A_[i][j] = A->p[i][j];
    }
  }

  MatDoub b_(b->rows, b->cols);
  MatDoub x_(x->rows, x->cols);
  for (uint32_t i = 0; i < b->rows; i ++ )
  {
    b_[i][0] = b->p[i][0];
  }

  SVD sv(A_);
  sv.solve(b_, x_);

  for (uint32_t i = 0; i < x->rows; i ++ )
  {
    x->p[i][0] = x_[i][0];
  }
}

void SOLVE_A_x_b_MAT_by_colPivHouseholderQr_MAT_DYNAMIC_D(MAT_DYNAMIC_D* A, MAT_DYNAMIC_D* b, MAT_DYNAMIC_D* x)
{
  assert(A->cols == x->rows && A->rows == b->rows);
  assert(b->cols == 1 && x->cols == 1);
  assert(A->cols <= A->rows);

  QRdcmp sv(A);
  sv.solve(b, x);

  printf("QT: \n");
  for (int i = 0; i < sv.QT.rows; i ++ )
  {
    for (int j = 0; j < sv.QT.cols; j ++ )
    {
      printf("%.3f ", sv.QT.p[i][j]);
    }
    printf("\n");
  }

  printf("R: \n");
  for (int i = 0; i < sv.R.rows; i ++ )
  {
    for (int j = 0; j < sv.R.cols; j ++ )
    {
      printf("%.3f ", sv.R.p[i][j]);
    }
    printf("\n");
  }
}

void SET_ZERO_VEC_D_3(VEC_D_3 v)
{
  for (int i = 0; i < 3; i ++ ) v[i] = 0.0;
}

// v = v * n
void MULTIPLY_SCALE_VEC_D_3(VEC_D_3 v, double n)
{
  for (int i = 0; i < 3; i ++ ) v[i] *= n;
}

// m = v * v^T
void AAT_VEC_D_3(VEC_D_3 v, MAT_D_3_3 m)
{

  for (int i = 0; i < 3; i ++ ) 
  {
    for (int j = 0; j < 3; j ++ )
    {
      m[i][j] = v[i] * v[j];
    }
  }
}

// m = m^T inplace
void TRANSPOSE_MAT_D_3_3(MAT_D_3_3 m)
{
  transpose_Mat33d_inplace(m);
}

// return rank of m
int RANK_MAT_D_3_3(MAT_D_3_3 m)
{
  MatDoub A_(3, 3);
  for (uint32_t i = 0; i < 3; i ++ )
  {
    for (uint32_t j = 0; j < 3; j ++ )
    {
      A_[i][j] = m[i][j];
    }
  }

  SVD sv(A_);
  return sv.rank();
}

// inv_m = m^-1
void INVERSE_MAT_D_3_3(MAT_D_3_3 A, MAT_D_3_3 B)
{
	int i, j, k;
	float max, temp;
  int n = 3;

	float t[n][n];                //临时矩阵
								  //将A矩阵存放在临时矩阵t[n][n]中
	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			t[i][j] = A[i][j];
		}
	}
	//初始化B矩阵为单位阵
	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			B[i][j] = (i == j) ? (float)1 : 0;
		}
	}
	for (i = 0; i < n; i++)
	{
		//寻找主元
		max = t[i][i];
		k = i;
		for (j = i + 1; j < n; j++)
		{
			if (fabsf(t[j][i]) > fabsf(max))
			{
				max = t[j][i];
				k = j;
			}
		}
		//如果主元所在行不是第i行，进行行交换
		if (k != i)
		{
			for (j = 0; j < n; j++)
			{
				temp = t[i][j];
				t[i][j] = t[k][j];
				t[k][j] = temp;
				//B伴随交换
				temp = B[i][j];
				B[i][j] = B[k][j];
				B[k][j] = temp;
			}
		}
		//判断主元是否为0, 若是, 则矩阵A不是满秩矩阵,不存在逆矩阵
		if (t[i][i] == 0)
		{
			printf("There is no inverse matrix!");
			// system("pause");
			exit(0);
		}
		//消去A的第i列除去i行以外的各行元素
		temp = t[i][i];
		for (j = 0; j < n; j++)
		{
			t[i][j] = t[i][j] / temp;        //主对角线上的元素变为1
			B[i][j] = B[i][j] / temp;        //伴随计算
		}
		for (j = 0; j < n; j++)        //第0行->第n行
		{
			if (j != i)                //不是第i行
			{
				temp = t[j][i];
				for (k = 0; k < n; k++)        //第j行元素 - i行元素*j列i行元素
				{
					t[j][k] = t[j][k] - t[i][k] * temp;
					B[j][k] = B[j][k] - B[i][k] * temp;
				}
			}
		}
	}
}

// return v1^T * v2
double DOT_PRODUCT_VEC_D_3(VEC_D_3 v1, VEC_D_3 v2)
{
  double sum = 0.0;
  for (int i = 0; i < 3; i ++ ) sum += v1[i]*v2[i];
  return sum;
}

// return || v1 - v2 ||^2
double SQUARED_L2_DIST_VEC_D_3(VEC_D_3 v1, VEC_D_3 v2)
{
  double sum = 0.0;
  for (int i = 0; i < 3; ++i) sum += (v1[i] - v2[i])*(v1[i] - v2[i]);
  return sum;
}

// return || p ||^2
double squaredNorm_VEC_D_3(VEC_D_3 p)
{
  double sum = 0.0;
  for (int i = 0; i < 3; i ++ ) sum += p[i]*p[i];
  return sum;
}

// m = [0, 0, 0; 0, 0, 0; 0, 0, 0;];
void SET_ZERO_MAT_D_3_3(MAT_D_3_3 m)
{
  for (int i = 0; i < 3; i ++ ) 
  {
    for (int j = 0; j < 3; j ++ )
    {
      m[i][j] = 0.0;
    }
  }
}

double DET(MAT_D_3_3 m, int n)
{
	int i, M;//i是第一行的列指标，M是余子式的值，sum是行列式的计算值
  double sum = 0.0;
	if (n == 1)//一阶行列式直接得出结果
		return m[0][0];
	else if (n > 1)
	{
		for (i = 0; i < n; i++)//按第一行展开
		{
			M = Minor(m, i, n);
			sum += pow(-1, i + 2) * m[0][i] * M;
		}
	}
	return sum;
}

int Minor(MAT_D_3_3 arr1,int i,int n)
{
	int  j, k,result;
	MAT_D_3_3 arr2;
	
	//以下为构造余子式的过程。由于C语言的特性，这个过程会比较复杂，需要观察余子式与原行列式的关系。
		for (j = 0; j < n - 1; j++)
		{
			for (k = 0; k < n - 1; k++)
			{
				if (k < i)
					arr2[j][k] = arr1[j + 1][k];
				else if (k >= i)
					arr2[j][k] = arr1[j + 1][k + 1];
			}
		}
		
	return DET(arr2, n - 1);//构造完后，余子式是一个新的行列式，返回DET函数进行计算。
}

// return |m|
double determinant_MAT_D_3_3(MAT_D_3_3 m)
{
  return DET(m, 3);
}

// m_out = m_in
void EQU_MAT_D_3_3(MAT_D_3_3 m_in, MAT_D_3_3 m_out)
{
  for (int i = 0; i < 3; i ++ ) 
  {
    for (int j = 0; j < 3; j ++ )
    {
      m_out[i][j] = m_in[i][j];
    }
  }
}

// from https://github.com/datenwolf/linmath.h/blob/master/linmath.h
  uint32_t qua2mat(const Quaternion_D q, MAT_D_3_3 M)
  {
    float a = q[0];
    float b = q[1];
    float c = q[2];
    float d = q[3];
    float a2 = a*a;
    float b2 = b*b;
    float c2 = c*c;
    float d2 = d*d;
    
    M[0][0] = a2 + b2 - c2 - d2;
    M[0][1] = 2.f*(b*c + a*d);
    M[0][2] = 2.f*(b*d - a*c);

    M[1][0] = 2*(b*c - a*d);
    M[1][1] = a2 - b2 + c2 - d2;
    M[1][2] = 2.f*(c*d + a*b);

    M[2][0] = 2.f*(b*d + a*c);
    M[2][1] = 2.f*(c*d - a*b);
    M[2][2] = a2 - b2 - c2 + d2;

    return 0;
  }

  uint32_t normalize_q(Quaternion_D q)
  {
    double norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    norm = 1.0 / norm;

    q[0] *= norm;
    q[1] *= norm;
    q[2] *= norm;
    q[3] *= norm;

    return 0;
  }

  // result will store into rmat
  uint32_t MultipyMatrix(const MAT_DYNAMIC_D* lmat, MAT_DYNAMIC_D* rmat) 
  {
    int N = lmat->rows;
    int M = rmat->cols;
    int K = lmat->cols;

    assert(K == rmat->rows);

    double tmp[N][M];

    for(uint32_t i = 0; i < N; i++) {             
      for(uint32_t j = 0; j < M; j++) {	
        double sum = 0;
        for(uint32_t k = 0; k < K; k++) {   
          sum += lmat->p[i][k] * rmat->p[k][j];
        }
        tmp[i][j] = sum;
      }	
    }

    for(uint32_t i = 0; i < N; i++) {             
      for(uint32_t j = 0; j < M; j++) {	
        rmat->p[i][j] = tmp[i][j];
      }	
    }
  } 
