#include <math.h>
#include <stdlib.h>

#include "macro.h"
#include "ia_abstraction.h"
#include "ac_auxiliary.h"
#include "solver.h"
// #define LOG_AC
// #define _DEBUG

void print_mat_N_D(const double* mat, int N) {
    double *p = mat;
    for (int i = 0 ; i < N; i ++ ) {
        for (int j = 0; j < N; j ++ ) {
            printf("%f ", *p++);
        }
        puts("");
    }
    puts("");
}

void print_vec_N_D(const double* vec, int N) {
    double *p = vec;
    for (int i = 0 ; i < N; i ++ ) {
        printf("%f ", *p++);
    }
    puts("");
}

// double* symmat2vector_N_D(const double* mat, int N) {
//     double* vec = malloc(N*(N + 1) / 2 * sizeof(double));
//     double *p = vec;
//     for (int i = 0 ; i < N; i ++ ) {
//         for (int j = i; j < N; j ++ ) {
//             *p++ = mat[i * N + j];
//         }
//     }
//     return vec;
// }

double** vector2symmat_N_D(const double* vec, int N) {
    double (*mat)[N] = (double(*)[N])malloc(N * N * sizeof(double));
    // int (*a)[2]=(int(*)[2])malloc(sizeof(int)*3*2);
    double *p = mat;
    int i = 0, j = 0;
    for (int k = 0; k < N*(N + 1) / 2; k ++ ) {
        if(j >= N) {
            i ++ ;
            j = i;
        }
        mat[i][j] = vec[k];
        j ++ ;
    }
    return mat;
}

// N x N matrix multiply
double** matrix_multiply_D_N(const double** mat1, const double** mat2, int N) {
    double (*mat)[N] = (double(*)[N])malloc(N * N * sizeof(double));
    for(int i =0;i<N;i++) {                 //对矩阵c进行处理
        double sum = 0;
        for(int j=0;j<N;j++) {	
            sum=0;
			for(int k=0;k<N;k++)   //矩阵c中每一个元素的循环计算
			{   
			  sum+=*((double*)mat1 + i*N+k) * *((double*)mat2 + k*N+j); //对矩阵c中的某一元素进行计算
			}
			mat[i][j]=sum;           //给矩阵c中的某一元素进行赋值
		}	
	}
    return mat;
}

int main() {
	uint32_t N = 4;
    uint32_t len = N*(N + 1) / 2;

    double A_data[N * N];
    symmetric_double_t A = {
        data: A_data, 
        n: N
    };
    
    for (int i = 0 ; i < N; i ++ ) {
        for (int j = i; j < N; j ++ ) {
            *symmetric_index(&A, i, j) = rand() % 50;
        }
    }

    // *symmetric_index(&A, 0, 0) = 33;
    // *symmetric_index(&A, 0, 1) = 0;
    // *symmetric_index(&A, 0, 2) = 12.718404;
    // *symmetric_index(&A, 0, 3) = -76.998098;
    // *symmetric_index(&A, 1, 1) = 3.727273;
    // *symmetric_index(&A, 1, 2) = 5.299335;
    // *symmetric_index(&A, 1, 3) = 0.000000;
    // *symmetric_index(&A, 2, 2) = 11.658537;
    // *symmetric_index(&A, 2, 3) = -140.015512;
    // *symmetric_index(&A, 3, 3) = -94.108787;

    puts("origin matrix :");
    symmetric_print_d(&A);

    double R_data[len];
    double D_data[N];
    unit_diag_triangular_double_t R = { (double*)NULL, N };
    diagonal_double_t D = { (double*)NULL, N };
    R.data = R_data;
    D.data = D_data;
    Symmetric_RtDRdecomp(&A, &R, &D);

    double A_mat_data[N*N];
    double R_mat_data[N*N];
    double D_mat_data[N*N];
    ac_matrix_double_t A_mat = {A_mat_data, N, N};
    ac_matrix_double_t R_mat = {R_mat_data, N, N};
    ac_matrix_double_t D_mat = {D_mat_data, N, N};
    symmetric_2_matrix(&A, &A_mat);
    unit_diag_triangular_2_matrix(&R, &R_mat);
    diagonal_2_matrix(&D, &D_mat);

    // double res_mat_data[N*N];
    // ac_matrix_double_t res_mat = {res_mat_data, N, N};
    // matrix_matrix_operation_d(R_mat, D_mat, '*', res_mat);
    // puts("product : ");
    // ac_mat_print_d(&res_mat, N, N);


    // puts("matrix A^T*D*A :");
    // double** matrix1 = matrix_multiply_D_N(A, D, N);
    // puts("1 :");
    // double** matrix2 = matrix_multiply_D_N(matrix1, A, N);
    // print_mat_N_D(matrix2, N);

    // }
	return 0;
}
