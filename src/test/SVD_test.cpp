#include "geometry.h"
#include "solver.h"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <fstream>
void print_mat(const MAT_DYNAMIC_D* mat)
{
    printf("\n");
    for (uint32_t i = 0; i < mat->rows; ++i) {
        for (uint32_t j = 0; j < mat->cols; ++j) {
            printf("%f ", mat->p[i][j]);
        }
        printf("\n");
    }
    printf("\n");
}

void print_mat(const MAT_D_3_3 mat)
{
    printf("\n");
    for (uint32_t i = 0; i < 3; ++i) {
        for (uint32_t j = 0; j < 3; ++j) {
            printf("%f ", mat[i][j]);
        }
        printf("\n");
    }
    printf("\n");
}

void SVD_MAT_D_3_3_ref(MAT_D_3_3 m, MAT_D_3_3 U, MAT_D_3_3 V, MAT_D_3_3 D)
{
    // Copy input data to Eigen matrix
    Eigen::Matrix3d eigen_m;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            eigen_m(i, j) = m[i][j];
        }
    }

    // Perform Singular Value Decomposition
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(eigen_m, Eigen::ComputeFullU | Eigen::ComputeFullV);

    // Copy the results back to the output arrays
    Eigen::Matrix3d eigen_U = svd.matrixU();
    Eigen::Matrix3d eigen_V = svd.matrixV();
    Eigen::Matrix3d eigen_S = svd.singularValues().asDiagonal();

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            U[i][j] = eigen_U(i, j);
            V[i][j] = eigen_V(i, j);
            D[i][j] = i == j ? eigen_S(i, j) : 0.0; // D is a diagonal matrix
        }
    }
}

void SVD_MAT_DYNAMIC_D_ref(MAT_DYNAMIC_D* mat, SVD_DYNAMIC_D* svd) {
  // Convert MAT_DYNAMIC_D to Eigen::MatrixXd
  Eigen::MatrixXd eigen_mat(mat->rows, mat->cols);
  for (uint32_t i = 0; i < mat->rows; ++i) {
    for (uint32_t j = 0; j < mat->cols; ++j) {
      eigen_mat(i, j) = mat->p[i][j];
    }
  }

  // Compute SVD
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_solver(eigen_mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::MatrixXd U = svd_solver.matrixU();
  Eigen::MatrixXd D = svd_solver.singularValues().asDiagonal();
  Eigen::MatrixXd V = svd_solver.matrixV();

  // Convert Eigen::MatrixXd to MAT_DYNAMIC_D
  for (uint32_t i = 0; i < U.rows(); ++i) {
    for (uint32_t j = 0; j < U.cols(); ++j) {
      svd->U.p[i][j] = U(i, j);
    }
  }

  for (uint32_t i = 0; i < V.rows(); ++i) {
    for (uint32_t j = 0; j < V.cols(); ++j) {
      svd->V.p[i][j] = V(i, j);
    }
  }

  for (uint32_t i = 0; i < D.rows(); ++i) {
    for (uint32_t j = 0; j < D.cols(); ++j) {
        svd->D.p[i][j] = i == j ? D(i, j) : 0.0; // D is a diagonal matrix
    }
  }
}

void SOLVE_A_x_b_MAT_by_colPivHouseholderQr_MAT_DYNAMIC_D_ref(MAT_DYNAMIC_D* A, MAT_DYNAMIC_D* b, MAT_DYNAMIC_D* x) {
  // Convert MAT_DYNAMIC_D to Eigen::MatrixXd
  Eigen::MatrixXd eigen_A(A->rows, A->cols);
  Eigen::VectorXd eigen_b(b->rows);

  for (uint32_t i = 0; i < A->rows; ++i) {
    for (uint32_t j = 0; j < A->cols; ++j) {
      eigen_A(i, j) = A->p[i][j];
    }
    eigen_b(i) = b->p[i][0]; // Assuming b is a column vector
  }

  // Solve using ColPivHouseholderQR
  printf("rank: %d\n", eigen_A.colPivHouseholderQr().rank());
  Eigen::VectorXd eigen_x = eigen_A.colPivHouseholderQr().solve(eigen_b);

  // Convert Eigen::VectorXd to MAT_DYNAMIC_D
  NEW_MAT_DYNAMIC_D(x, A->cols, 1); // Assuming x is a column vector
  for (uint32_t i = 0; i < A->cols; ++i) {
    x->p[i][0] = eigen_x(i);
  }
}

void SOLVE_A_x_b_MAT_by_SVD_MAT_DYNAMIC_D_ref(MAT_DYNAMIC_D* A, MAT_DYNAMIC_D* b, MAT_DYNAMIC_D* x) {
  // Convert MAT_DYNAMIC_D to Eigen::MatrixXd
  Eigen::MatrixXd eigen_A(A->rows, A->cols);
  Eigen::VectorXd eigen_b(b->rows);

  for (uint32_t i = 0; i < A->rows; ++i) {
    for (uint32_t j = 0; j < A->cols; ++j) {
      eigen_A(i, j) = A->p[i][j];
    }
    eigen_b(i) = b->p[i][0]; // Assuming b is a column vector
  }

  // Solve using SVD
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(eigen_A, Eigen::ComputeFullV | Eigen::ComputeFullU);
  Eigen::VectorXd eigen_x = svd.solve(eigen_b);
//   Eigen::VectorXd eigen_x = eigen_A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(eigen_b);

  // Convert Eigen::VectorXd to MAT_DYNAMIC_D
  NEW_MAT_DYNAMIC_D(x, A->cols, 1); // Assuming x is a column vector
  for (uint32_t i = 0; i < A->cols; ++i) {
    x->p[i][0] = eigen_x(i);
  }
}

int main()
{

/*
{
  double tmp_our[12][12] = {
  {16, 0, -14.4368, -4, 0, 4.52234, -4, 0, 3.44429, -4, 0, 2.91934 },
{0, 16, -12.2606, 0, -4, 4.65721, 0, -4, 3.54572, 0, -4, 1.08855 },
{-14.4368, -12.2606, 24.4271, 4.52234, 4.65721, -9.41467, 3.44429, 3.54572, -6.47296, 2.91934, 1.08855, -2.19556 },
{-4, 0, 4.52234, 4, 0, -2.70518, -1.04569e-07, 0, -0.999983, 8.32797e-08, 0, 0.176669 },
{0, -4, 4.65721, 0, 4, -2.44346, 0, -1.04569e-07, -1.16212, 0, 8.32797e-08, 0.18334 },
{4.52234, 4.65721, -9.41467, -2.70518, -2.44346, 4.55417, -0.999983, -1.16212, 2.58005, 0.176669, 0.18334, -0.441273 },
{-4, 0, 3.44429, -1.04569e-07, 0, -0.999983, 4, 0, -2.66209, 5.5876e-09, 0, -0.0585204 },
{0, -4, 3.54572, 0, -1.04569e-07, -1.16212, 0, 4, -1.84863, 0, 5.5876e-09, -0.28953 },
{3.44429, 3.54572, -6.47296, -0.999983, -1.16212, 2.58005, -2.66209, -1.84863, 3.26962, -0.0585204, -0.28953, 0.191336 },
{-4, 0, 2.91934, 8.32797e-08, 0, 0.176669, 5.5876e-09, 0, -0.0585204, 4, 0, -2.98987 },
{0, -4, 1.08855, 0, 8.32797e-08, 0.18334, 0, 5.5876e-09, -0.28953, 0, 4, -1.38648 },
{2.91934, 1.08855, -2.19556, 0.176669, 0.18334, -0.441273, -0.0585204, -0.28953, 0.191336, -2.98987, -1.38648, 2.79483  }};

  double tmp_ref[12][12] = {
{16, 0, -14.4368, -4, 0, 4.52233, -4, 0, 3.44429, -4, 0, 2.91934 },
{0, 16, -12.2606, 0, -4, 4.65721, 0, -4, 3.54572, 0, -4, 1.08855 },
{-14.4368, -12.2606, 24.4271, 4.52233, 4.65721, -9.41467, 3.44429, 3.54572, -6.47296, 2.91934, 1.08855, -2.19556 },
{-4, 0, 4.52233, 4, 0, -2.70518, 6.66134e-16, 0,-0.999983, -1.55431e-15,0, 0.176669 },
{0, -4, 4.65721, 0, 4, -2.44346, 0, 6.66134e-16, -1.16212, 0, -1.55431e-15, 0.18334 },
{4.52233, 4.65721, -9.41467, -2.70518, -2.44346, 4.55417, -0.999983, -1.16212, 2.58005, 0.176669, 0.18334, -0.441273 },
{-4, 0, 3.44429, 6.66134e-16, 0, -0.999983, 4, 0, -2.66209, 1.58207e-15, 0, -0.0585203 },
{0, -4, 3.54572 ,0, 6.66134e-16, -1.16212, 0, 4,-1.84863, 0 ,1.58207e-15 ,-0.28953 },
{3.44429, 3.54572, -6.47296, -0.999983 ,-1.16212, 2.58005, -2.66209, -1.84863, 3.26962, -0.0585203, -0.28953, 0.191336 },
{-4, 0, 2.91934, -1.55431e-15, 0, 0.176669, 1.58207e-15, 0, -0.0585203, 4, 0, -2.98987 },
{0, -4, 1.08855, 0, -1.55431e-15, 0.18334, 0, 1.58207e-15, -0.28953, 0, 4, -1.38648 },
{2.91934, 1.08855, -2.19556, 0.176669, 0.18334, -0.441273, -0.0585203, -0.28953, 0.191336, -2.98987, -1.38648, 2.79483}, };


Eigen::Matrix<double, 12, 12> mat_ref_eigen;
Eigen::Matrix<double, 12, 12> mat_our_eigen;

MAT_DYNAMIC_D mat_ref;
MAT_DYNAMIC_D mat_our;
NEW_MAT_DYNAMIC_D(&mat_ref, 12, 12);
NEW_MAT_DYNAMIC_D(&mat_our, 12, 12);

for (int i = 0; i < 12; ++i) {
  for (int j = 0; j < 12; ++j) {
    mat_ref_eigen(i,j) = tmp_ref[i][j];
    mat_ref.p[i][j] = tmp_ref[i][j];

    mat_our_eigen(i,j) = tmp_our[i][j];
    mat_our.p[i][j] = tmp_our[i][j];
  }
}

  Eigen::JacobiSVD<Eigen::MatrixXd> svd_solver_ref(mat_ref_eigen, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::MatrixXd U_ref_eigen = svd_solver_ref.matrixU();
  Eigen::MatrixXd D_ref_eigen = svd_solver_ref.singularValues().asDiagonal();
  Eigen::MatrixXd V_ref_eigen = svd_solver_ref.matrixV();

  Eigen::JacobiSVD<Eigen::MatrixXd> svd_solver_our(mat_our_eigen, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::MatrixXd U_our_eigen = svd_solver_our.matrixU();
  Eigen::MatrixXd D_our_eigen = svd_solver_our.singularValues().asDiagonal();
  Eigen::MatrixXd V_our_eigen = svd_solver_our.matrixV();


  SVD_DYNAMIC_D svd_ref, svd_our;
  NEW_SVD_DYNAMIC_D(&svd_ref, 12, 12);
  NEW_SVD_DYNAMIC_D(&svd_our, 12, 12);

  SVD_MAT_DYNAMIC_D(&mat_ref, &svd_ref);
  SVD_MAT_DYNAMIC_D(&mat_our, &svd_our);

  MAT_DYNAMIC_D Ut_ref, Ut_our;
  NEW_MAT_DYNAMIC_D(&Ut_ref, 12, 12);
  NEW_MAT_DYNAMIC_D(&Ut_our, 12, 12);

  TRANSPOSE_MAT_DYNAMIC_D(&svd_ref.U, &Ut_ref);
  TRANSPOSE_MAT_DYNAMIC_D(&svd_our.U, &Ut_our);

  std::cout << "V_ref_eigen" << std::endl;
  std::cout << V_ref_eigen.transpose() << std::endl;
  std::cout << "V_ref" << std::endl;
  for (int i = 0; i < 12; ++i) {
    for (int j = 0; j < 12; ++j) {
      std::cout << Ut_ref.p[i][j] << " ";
    }
    std::cout << std::endl;
  }

  std::cout << std::endl;

  std::cout << "V_our_eigen" << std::endl;
  std::cout << V_our_eigen.transpose() << std::endl;
  std::cout << "V_our" << std::endl;
  for (int i = 0; i < 12; ++i) {
    for (int j = 0; j < 12; ++j) {
      std::cout << Ut_our.p[i][j] << " ";
    }
    std::cout << std::endl;
  }

}

*/

{

  std::string path_ref = "/home/zhaoshen/Mycode/matrix_operation/src/bin_for_test/ref.bin";
  std::string path_our = "/home/zhaoshen/Mycode/matrix_operation/src/bin_for_test/our.bin";
  std::ifstream ifs_ref(path_ref, std::ios::binary | std::ios::in);
  std::ifstream ifs_our(path_our, std::ios::binary | std::ios::in);

  Eigen::Matrix<double, 12, 12> mat_ref_eigen;
  Eigen::Matrix<double, 12, 12> mat_our_eigen;

  MAT_DYNAMIC_D mat_ref;
  MAT_DYNAMIC_D mat_our;
  NEW_MAT_DYNAMIC_D(&mat_ref, 12, 12);
  NEW_MAT_DYNAMIC_D(&mat_our, 12, 12);

  for (int i = 0; i < 12; ++i) {
    ifs_ref.read((char*)mat_ref.p[i], sizeof(double) * 12);
    ifs_our.read((char*)mat_our.p[i], sizeof(double) * 12);
  }
  ifs_ref.close();
  ifs_our.close();

  for (int i = 0; i < 12; ++i) {
    for (int j = 0; j < 12; ++j) {
      mat_ref_eigen(i,j) = mat_ref.p[i][j];
      mat_our_eigen(i,j) = mat_our.p[i][j];
    }
  }

  std::cout << "ref mat" << std::endl;
  std::cout << mat_ref_eigen << std::endl;
  std::cout << "our mat" << std::endl;
  std::cout << mat_our_eigen << std::endl;
  // std::cout << "our mat" << std::endl;
  // for (int i = 0; i < 12; ++i) {
  //   for (int j = 0; j < 12; ++j) {
  //     std::cout << mat_our.p[i][j] << " ";
  //   }
  //   std::cout << std::endl;
  // }
  // std::cout << "ref mat" << std::endl;
  // for (int i = 0; i < 12; ++i) {
  //   for (int j = 0; j < 12; ++j) {
  //     std::cout << mat_ref.p[i][j] << " ";
  //   }
  //   std::cout << std::endl;
  // }




  Eigen::JacobiSVD<Eigen::MatrixXd> svd_solver_ref(mat_ref_eigen, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::MatrixXd U_ref_eigen = svd_solver_ref.matrixU();
  Eigen::MatrixXd D_ref_eigen = svd_solver_ref.singularValues().asDiagonal();
  Eigen::MatrixXd V_ref_eigen = svd_solver_ref.matrixV();

  Eigen::JacobiSVD<Eigen::MatrixXd> svd_solver_our(mat_our_eigen, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::MatrixXd U_our_eigen = svd_solver_our.matrixU();
  Eigen::MatrixXd D_our_eigen = svd_solver_our.singularValues().asDiagonal();
  Eigen::MatrixXd V_our_eigen = svd_solver_our.matrixV();


  SVD_DYNAMIC_D svd_ref, svd_our;
  NEW_SVD_DYNAMIC_D(&svd_ref, 12, 12);
  NEW_SVD_DYNAMIC_D(&svd_our, 12, 12);

  SVD_MAT_DYNAMIC_D(&mat_ref, &svd_ref);
  SVD_MAT_DYNAMIC_D(&mat_our, &svd_our);

  MAT_DYNAMIC_D Ut_ref, Ut_our;
  NEW_MAT_DYNAMIC_D(&Ut_ref, 12, 12);
  NEW_MAT_DYNAMIC_D(&Ut_our, 12, 12);

  TRANSPOSE_MAT_DYNAMIC_D(&svd_ref.U, &Ut_ref);
  TRANSPOSE_MAT_DYNAMIC_D(&svd_our.U, &Ut_our);

  std::cout << "U_ref_eigen" << std::endl;
  std::cout << U_ref_eigen.transpose() << std::endl;

  std::cout << "U_ref_singl : " << std::endl;
  std::cout << D_our_eigen.transpose() << std::endl;
  std::cout << "U_ref" << std::endl;
  for (int i = 0; i < 12; ++i) {
    for (int j = 0; j < 12; ++j) {
      std::cout << Ut_ref.p[i][j] << " ";
    }
    std::cout << std::endl;
  }

  std::cout << std::endl;

  std::cout << "U_our_eigen" << std::endl;
  std::cout << U_our_eigen.transpose() << std::endl;
    std::cout << "U_our_singl : " << std::endl;
  for (int i = 0; i < 12; i ++ )
  {
    std::cout << svd_our.D.p[i][i] << " ";
  }

  std::cout << "U_our" << std::endl;
  for (int i = 0; i < 12; ++i) {
    for (int j = 0; j < 12; ++j) {
      std::cout << Ut_our.p[i][j] << " ";
    }
    std::cout << std::endl;
  }

  
}

/*
    {
        printf("svd decomposition 33 ref: \n");
        MAT_D_3_3 mat, U, D, V;
        for (int i = 0; i < 3; i ++ ) for (int j = 0; j < 3; j ++ ) U[i][j] = 0.0;
        for (int i = 0; i < 3; i ++ ) for (int j = 0; j < 3; j ++ ) D[i][j] = 0.0;
        for (int i = 0; i < 3; i ++ ) for (int j = 0; j < 3; j ++ ) V[i][j] = 0.0;
        // NEW_MAT_DYNAMIC_D(&b, 3, 1);

        // Fill A and b with your data
        mat[0][0] = 1;
        mat[0][1] = 1;
        mat[0][2] = 0;
        mat[1][0] = 2;
        mat[1][1] = 1;
        mat[1][2] = 0;
        mat[2][0] = 3;
        mat[2][1] = 1;
        mat[2][2] = 1;

        SVD_MAT_D_3_3_ref(mat, U, V, D);
        printf("A : \n");
        print_mat(mat);
        printf("U : \n");
        print_mat(U);
        printf("D : \n");
        print_mat(D);
        printf("V : \n");
        print_mat(V);
    }

    {
        printf("svd decomposition 33: \n");
        MAT_D_3_3 mat, U, D, V;
        for (int i = 0; i < 3; i ++ ) for (int j = 0; j < 3; j ++ ) U[i][j] = 0.0;
        for (int i = 0; i < 3; i ++ ) for (int j = 0; j < 3; j ++ ) D[i][j] = 0.0;
        for (int i = 0; i < 3; i ++ ) for (int j = 0; j < 3; j ++ ) V[i][j] = 0.0;

        // NEW_MAT_DYNAMIC_D(&b, 3, 1);

        // Fill A and b with your data
        mat[0][0] = 1;
        mat[0][1] = 1;
        mat[0][2] = 0;
        mat[1][0] = 2;
        mat[1][1] = 1;
        mat[1][2] = 0;
        mat[2][0] = 3;
        mat[2][1] = 1;
        mat[2][2] = 1;

        SVD_MAT_D_3_3(mat, U, V, D);
        printf("A : \n");
        print_mat(mat);
        printf("U : \n");
        print_mat(U);
        printf("D : \n");
        print_mat(D);
        printf("V : \n");
        print_mat(V);
    }

    {
        printf("svd decomposition dynamic ref: \n");
        MAT_DYNAMIC_D mat;
        SVD_DYNAMIC_D svd;
        NEW_MAT_DYNAMIC_D(&mat, 3, 3);
        NEW_MAT_DYNAMIC_D(&svd.U, mat.rows, mat.rows);
        NEW_MAT_DYNAMIC_D(&svd.D, mat.rows, mat.cols);
        NEW_MAT_DYNAMIC_D(&svd.V, mat.cols, mat.cols);

        // Fill A and b with your data
        mat.p[0][0] = 1;
        mat.p[0][1] = 1;
        mat.p[0][2] = 0;
        mat.p[1][0] = 2;
        mat.p[1][1] = 1;
        mat.p[1][2] = 0;
        mat.p[2][0] = 3;
        mat.p[2][1] = 1;
        mat.p[2][2] = 1;

        SVD_MAT_DYNAMIC_D_ref(&mat, &svd);
        printf("A : \n");
        print_mat(&mat);
        printf("U : \n");
        print_mat(&svd.U);
        printf("D : \n");
        print_mat(&svd.D);
        printf("V : \n");
        print_mat(&svd.V);

        // Free memory
        FREE_MAT_DYNAMIC_D(&mat);
        FREE_MAT_DYNAMIC_D(&svd.U);
        FREE_MAT_DYNAMIC_D(&svd.D);
        FREE_MAT_DYNAMIC_D(&svd.V);
    }

    {
        printf("svd decomposition dynamic: \n");
        MAT_DYNAMIC_D mat;
        SVD_DYNAMIC_D svd;
        NEW_MAT_DYNAMIC_D(&mat, 3, 3);
        NEW_MAT_DYNAMIC_D(&svd.U, mat.rows, mat.cols);
        NEW_MAT_DYNAMIC_D(&svd.D, mat.cols, mat.cols);
        NEW_MAT_DYNAMIC_D(&svd.V, mat.cols, mat.cols);

        // Fill A and b with your data
        mat.p[0][0] = 1;
        mat.p[0][1] = 1;
        mat.p[0][2] = 0;
        mat.p[1][0] = 2;
        mat.p[1][1] = 1;
        mat.p[1][2] = 0;
        mat.p[2][0] = 3;
        mat.p[2][1] = 1;
        mat.p[2][2] = 1;

        SVD_MAT_DYNAMIC_D(&mat, &svd);
        printf("A : \n");
        print_mat(&mat);
        printf("U : \n");
        print_mat(&svd.U);
        printf("D : \n");
        print_mat(&svd.D);
        printf("V : \n");
        print_mat(&svd.V);

        // Free memory
        FREE_MAT_DYNAMIC_D(&mat);
        FREE_MAT_DYNAMIC_D(&svd.U);
        FREE_MAT_DYNAMIC_D(&svd.D);
        FREE_MAT_DYNAMIC_D(&svd.V);
    }

    {
        printf("HouseholderQr ref: \n");
        MAT_DYNAMIC_D A, b, x;
        NEW_MAT_DYNAMIC_D(&A, 3, 3);
        NEW_MAT_DYNAMIC_D(&b, 3, 1);
        NEW_MAT_DYNAMIC_D(&x, 3, 1);

        // Fill A and b with your data
        A.p[0][0] = 1;
        A.p[0][1] = 1;
        A.p[0][2] = 0;
        A.p[1][0] = 0;
        A.p[1][1] = 1;
        A.p[1][2] = 0;
        A.p[2][0] = 1;
        A.p[2][1] = 1;
        A.p[2][2] = 1;

        b.p[0][0] = 1;
        b.p[1][0] = 1;
        b.p[2][0] = 0;

        SOLVE_A_x_b_MAT_by_colPivHouseholderQr_MAT_DYNAMIC_D_ref(&A, &b, &x);

        printf("A : \n");
        print_mat(&A);
        printf("b : \n");
        print_mat(&b);
        printf("x : \n");
        print_mat(&x);


        // Free memory
        FREE_MAT_DYNAMIC_D(&A);
        FREE_MAT_DYNAMIC_D(&b);
        FREE_MAT_DYNAMIC_D(&x);
    }

    {
        printf("HouseholderQr: \n");
        MAT_DYNAMIC_D A, b, x;
        NEW_MAT_DYNAMIC_D(&A, 3, 3);
        NEW_MAT_DYNAMIC_D(&b, 3, 1);
        NEW_MAT_DYNAMIC_D(&x, 3, 1);

        // Fill A and b with your data
        A.p[0][0] = 1;
        A.p[0][1] = 1;
        A.p[0][2] = 0;
        A.p[1][0] = 0;
        A.p[1][1] = 1;
        A.p[1][2] = 0;
        A.p[2][0] = 1;
        A.p[2][1] = 1;
        A.p[2][2] = 1;

        b.p[0][0] = 1;
        b.p[1][0] = 1;
        b.p[2][0] = 0;

        SOLVE_A_x_b_MAT_by_colPivHouseholderQr_MAT_DYNAMIC_D(&A, &b, &x);

        printf("A : \n");
        print_mat(&A);
        printf("b : \n");
        print_mat(&b);
        printf("x : \n");
        print_mat(&x);


        // Free memory
        FREE_MAT_DYNAMIC_D(&A);
        FREE_MAT_DYNAMIC_D(&b);
        FREE_MAT_DYNAMIC_D(&x);
    }

    {
        printf("SVD solve ref: \n");
        MAT_DYNAMIC_D A, b, x;
        NEW_MAT_DYNAMIC_D(&A, 3, 3);
        NEW_MAT_DYNAMIC_D(&b, 3, 1);
        NEW_MAT_DYNAMIC_D(&x, 3, 1);

        // Fill A and b with your data
        A.p[0][0] = 1;
        A.p[0][1] = 1;
        A.p[0][2] = 0;
        A.p[1][0] = 0;
        A.p[1][1] = 1;
        A.p[1][2] = 0;
        A.p[2][0] = 1;
        A.p[2][1] = 1;
        A.p[2][2] = 1;

        b.p[0][0] = 1;
        b.p[1][0] = 1;
        b.p[2][0] = 0;

        SOLVE_A_x_b_MAT_by_SVD_MAT_DYNAMIC_D_ref(&A, &b, &x);

        printf("A : \n");
        print_mat(&A);
        printf("b : \n");
        print_mat(&b);
        printf("x : \n");
        print_mat(&x);

        // Free memory
        FREE_MAT_DYNAMIC_D(&A);
        FREE_MAT_DYNAMIC_D(&b);
        FREE_MAT_DYNAMIC_D(&x);
    }

    {
        printf("SVD solve: \n");
        MAT_DYNAMIC_D A, b, x;
        NEW_MAT_DYNAMIC_D(&A, 3, 3);
        NEW_MAT_DYNAMIC_D(&b, 3, 1);
        NEW_MAT_DYNAMIC_D(&x, 3, 1);

        // Fill A and b with your data
        A.p[0][0] = 1;
        A.p[0][1] = 1;
        A.p[0][2] = 0;
        A.p[1][0] = 0;
        A.p[1][1] = 1;
        A.p[1][2] = 0;
        A.p[2][0] = 1;
        A.p[2][1] = 1;
        A.p[2][2] = 1;

        b.p[0][0] = 1;
        b.p[1][0] = 1;
        b.p[2][0] = 0;

        SOLVE_A_x_b_MAT_by_SVD_MAT_DYNAMIC_D(&A, &b, &x);

        printf("A : \n");
        print_mat(&A);
        printf("b : \n");
        print_mat(&b);
        printf("x : \n");
        print_mat(&x);

        // Free memory
        FREE_MAT_DYNAMIC_D(&A);
        FREE_MAT_DYNAMIC_D(&b);
        FREE_MAT_DYNAMIC_D(&x);
    }
*/
    return 0;
}
