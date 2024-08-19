#include "geometry.h"
#include "solver.h"
#include <eigen3/Eigen/Eigen>
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
  // NEW_MAT_DYNAMIC_D(x, A->cols, 1); // Assuming x is a column vector
  for (uint32_t i = 0; i < A->cols; ++i) {
    x->p[i][0] = eigen_x(i);
  }
}

int main()
{
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
        NEW_MAT_DYNAMIC_D(&A, 5, 4);
        NEW_MAT_DYNAMIC_D(&b, 5, 1);
        NEW_MAT_DYNAMIC_D(&x, 4, 1);

        // Fill A and b with your data
        A.p[0][0] = 17;
        A.p[0][1] = 24;
        A.p[0][2] = 1;
        A.p[0][3] = 8;
        // A.p[0][4] = 15;

        A.p[1][0] = 23;
        A.p[1][1] = 5;
        A.p[1][2] = 7;
        A.p[1][3] = 14;
        // A.p[1][4] = 16;

        A.p[2][0] = 4;
        A.p[2][1] = 6;
        A.p[2][2] = 13;
        A.p[2][3] = 20;
        // A.p[2][4] = 22;

        A.p[3][0] = 10;
        A.p[3][1] = 12;
        A.p[3][2] = 19;
        A.p[3][3] = 21;
        // A.p[3][4] = 3;

        A.p[4][0] = 11;
        A.p[4][1] = 18;
        A.p[4][2] = 25;
        A.p[4][3] = 2;
        // A.p[4][4] = 9;

        b.p[0][0] = 1;
        b.p[1][0] = 1;
        b.p[2][0] = 0;
        b.p[3][0] = 1;
        b.p[4][0] = 1;

        printf("A : \n");
        print_mat(&A);
        printf("b : \n");
        print_mat(&b);

        SOLVE_A_x_b_MAT_by_colPivHouseholderQr_MAT_DYNAMIC_D_ref(&A, &b, &x);

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
        NEW_MAT_DYNAMIC_D(&A, 5, 4);
        NEW_MAT_DYNAMIC_D(&b, 5, 1);
        NEW_MAT_DYNAMIC_D(&x, 4, 1);

        // Fill A and b with your data
        A.p[0][0] = 17;
        A.p[0][1] = 24;
        A.p[0][2] = 1;
        A.p[0][3] = 8;
        // A.p[0][4] = 15;

        A.p[1][0] = 23;
        A.p[1][1] = 5;
        A.p[1][2] = 7;
        A.p[1][3] = 14;
        // A.p[1][4] = 16;

        A.p[2][0] = 4;
        A.p[2][1] = 6;
        A.p[2][2] = 13;
        A.p[2][3] = 20;
        // A.p[2][4] = 22;

        A.p[3][0] = 10;
        A.p[3][1] = 12;
        A.p[3][2] = 19;
        A.p[3][3] = 21;
        // A.p[3][4] = 3;

        A.p[4][0] = 11;
        A.p[4][1] = 18;
        A.p[4][2] = 25;
        A.p[4][3] = 2;
        // A.p[4][4] = 9;

        b.p[0][0] = 1;
        b.p[1][0] = 1;
        b.p[2][0] = 0;
        b.p[3][0] = 1;
        b.p[4][0] = 1;

        printf("A : \n");
        print_mat(&A);
        printf("b : \n");
        print_mat(&b);

        SOLVE_A_x_b_MAT_by_colPivHouseholderQr_MAT_DYNAMIC_D(&A, &b, &x);

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

    return 0;
}
