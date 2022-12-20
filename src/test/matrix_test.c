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
	int N = 4;
    AC_LOG("%s", "hello world");

    double mat[N][N];
    ac_matrix_double_t matrix = {
        data: mat, 
        m: N, 
        n: N, 
        s: 1
    };
    for (int i = 0 ; i < N; i ++ ) {
        for (int j = 0; j < N; j ++ ) {
            mat[i][j] = rand() % 50;
        }
    }
    puts("origin matrix :");
    ac_mat_print_d(&matrix, N, N);


    ac_mat_print_d(&matrix, N, N);

	double* vec = symmat2vector_N_D(&matrix, N);
    puts("before decompose :");
    double* p = vec;
    for(int i = 0; i < N*(N + 1) / 2; i++)
        printf("%f ", *p++);
    puts("");

    if(!RtDRdecomp(vec, N)) {
		printf("failed!");
		return getchar();
	}
    puts("after decompose :");

    double A[N][N];
    double D[N][N];

    double** mat2 = vector2symmat_N_D(vec, N);
    print_mat_N_D(mat2, N);
    for (int i = 0; i < N; i ++ ) {
        for (int j = i; j < N; j ++ ) {
            if (i == j) {
                D[i][j] = mat[i][j];
                A[i][j] = 1;
            }
            else {
                A[i][j] = mat[i][j];
                A[j][i] = mat[i][j];
                D[i][j] = D[j][i] = 0;
            }
        }
    }
    puts("D :");
    print_mat_N_D(D, N);
    puts("A :");
    print_mat_N_D(A, N);

    // puts("matrix A^T*D*A :");
    // double** matrix1 = matrix_multiply_D_N(A, D, N);
    // puts("1 :");
    // double** matrix2 = matrix_multiply_D_N(matrix1, A, N);
    // print_mat_N_D(matrix2, N);

    // }
	return 0;
}
