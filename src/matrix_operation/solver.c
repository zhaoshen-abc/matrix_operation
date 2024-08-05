#include "solver.h"
#include <stdlib.h>
#define MAX_COL 20U

// uint32_t symm2mat(symmetric_double_t* symm, matrix_double_t* mat)
// {
//     if (A == NULL || R == NULL || D == NULL ||
//         A->data == NULL || R->data == NULL || D->data == NULL ||
//         A->n != R->n || R->n != D->n)
//     {
//         return uint32_t_argument;
//     }

//     uint32_t col = mat->n;
//     uint32_t row = mat->n;
//     char buf[MAX_COL * 64U] = {0};
//     char *ptr = buf;
//     uint32_t i = 0U;
//     uint32_t j = 0U;
//     uint32_t k = 0U;
//     uint32_t idx = 0U;
//     for (i = 0U; i < row; i++) {
//         ptr = buf;
//         for (j = 0U; j < k; j++) {
//             int size = sprintf(ptr, "%f, ", 0.0);
//             ptr += size;
//         }
//         for (j = 0U; j < col; j++) {
//             idx = i * col + j - k;
//             double data = mat->data[idx];
//             int size = sprintf(ptr, "%f, ", data);
//             ptr += size;
//         }
//         k ++ ;
//         printf("%s", buf);
//     }
// }

void symmetric_print_d(symmetric_double_t* A) {
    if (A == NULL || A->data == NULL)
    {
        return;
    }

    uint32_t N = A->n;
    char buf[MAX_COL * 64U] = {0};
    char *ptr = buf;
    uint32_t i = 0U;
    uint32_t j = 0U;
    for (i = 0U; i < N; i++) {
        ptr = buf;
        for (j = 0U; j < N; j++) {
            double data;
            if (i > j) {
                data = *upper_triangular_index(A, j, i);
            } else {
                data = *upper_triangular_index(A, i, j);
            }
            int size = sprintf(ptr, "%f, ", data);
            ptr += size;
        }
        printf("%s\n", buf);
    }
}

uint32_t symmetric_2_matrix(symmetric_double_t* A, matrix_double_t* mat) {
    if (A == NULL || A->data == NULL || mat == NULL || mat->data == NULL)
    {
        return;
    }

    mat->m = A->n;
    mat->n = A->n;
    double *ptr = mat->data;
    uint32_t i = 0U;
    uint32_t j = 0U;
    for (i = 0U; i < mat->m; i++) {
        for (j = 0U; j < mat->n; j++) {
            if (i > j) {
                *ptr = *symmetric_index(A, j, i);
            } else {
                *ptr = *symmetric_index(A, i, j);
            }
            ptr ++ ;
        }
    }
}

void upper_triangular_print_d(upper_triangular_double_t* A) {
    if (A == NULL || A->data == NULL)
    {
        return;
    }

    uint32_t N = A->n;
    char buf[MAX_COL * 64U] = {0};
    char *ptr = buf;
    uint32_t i = 0U;
    uint32_t j = 0U;
    for (i = 0U; i < N; i++) {
        ptr = buf;
        for (j = 0U; j < i; j++) {
            int size = sprintf(ptr, "%f, ", 0.0);
            ptr += size;
        }
        for (j = i; j < N; j++) {
            double data = *upper_triangular_index(A, i, j);
            int size = sprintf(ptr, "%f, ", data);
            ptr += size;
        }
        printf("%s\n", buf);
    }
}

uint32_t upper_triangular_2_matrix(upper_triangular_double_t* A, matrix_double_t* mat) {
    if (A == NULL || A->data == NULL || mat == NULL || mat->data == NULL)
    {
        return;
    }

    mat->m = A->n;
    mat->n = A->n;
    double *ptr = mat->data;
    uint32_t i = 0U;
    uint32_t j = 0U;
    for (i = 0U; i < mat->m; i++) {
        for (j = 0U; j < i; j++) {
            *ptr = *upper_triangular_index(A, j, i);
            ptr ++ ;
        }
        for (j = i; j < mat->n; j++) {
            double data = *upper_triangular_index(A, i, j);
            ptr ++ ;
        }
    }
}

void zero_diag_triangular_print_d(zero_diag_triangular_double_t* A) {
    if (A == NULL || A->data == NULL)
    {
        return;
    }

    uint32_t N = A->n;
    char buf[MAX_COL * 64U] = {0};
    char *ptr = buf;
    uint32_t i = 0U;
    uint32_t j = 0U;
    for (i = 0U; i < N; i++) {
        ptr = buf;
        for (j = 0U; j <= i; j++) {
            int size = sprintf(ptr, "%f, ", 0.0);
            ptr += size;
        }
        for (j = i+1; j < N; j++) {
            double data = *upper_triangular_index(A, i, j);
            int size = sprintf(ptr, "%f, ", data);
            ptr += size;
        }
        printf("%s\n", buf);
    }
}

void unit_diag_triangular_print_d(unit_diag_triangular_double_t* A) {
    if (A == NULL || A->data == NULL)
    {
        return;
    }

    uint32_t N = A->n;
    char buf[MAX_COL * 64U] = {0};
    char *ptr = buf;
    uint32_t i = 0U;
    uint32_t j = 0U;
    for (i = 0U; i < N; i++) {
        ptr = buf;
        for (j = 0U; j < i; j++) {
            int size = sprintf(ptr, "%f, ", 0.0);
            ptr += size;
        }
        {
            int size = sprintf(ptr, "%f, ", 1.0);
            ptr += size;
        }
        for (j = i+1; j < N; j++) {
            double data = *unit_diag_triangular_index(A, i, j);
            int size = sprintf(ptr, "%f, ", data);
            ptr += size;
        }
        printf("%s\n", buf);
    }
}

uint32_t unit_diag_triangular_2_matrix(unit_diag_triangular_double_t* A, matrix_double_t* mat) {
    if (A == NULL || A->data == NULL || mat == NULL || mat->data == NULL)
    {
        return;
    }

    mat->m = A->n;
    mat->n = A->n;
    double *ptr = mat->data;
    uint32_t i = 0U;
    uint32_t j = 0U;
    for (i = 0U; i < mat->m; i++) {
        for (j = 0U; j < i; j++) {
            *ptr = 0.0;
            ptr ++ ;
        }
        {
            *ptr = 1.0;
            ptr ++ ;
        }
        for (j = i+1; j < mat->n; j++) {
            *ptr = *unit_diag_triangular_index(A, i, j);
            ptr ++ ;
        }
    }
}

void diagonal_mat_print_d(diagonal_double_t* A) {
    if (A == NULL || A->data == NULL)
    {
        return;
    }

    uint32_t N = A->n;
    char buf[MAX_COL * 64U] = {0};
    char *ptr = buf;
    uint32_t i = 0U;
    uint32_t j = 0U;
    for (i = 0U; i < N; i++) {
        double data = A->data[i];
        int size = sprintf(ptr, "%f, ", data);
        ptr += size;
    }
    printf("%s\n", buf);
}

uint32_t diagonal_2_matrix(diagonal_double_t* A, matrix_double_t* mat) {
    if (A == NULL || A->data == NULL || mat == NULL || mat->data == NULL)
    {
        return;
    }

    mat->m = A->n;
    mat->n = A->n;
    memset(mat->data, 0, mat->m * mat->n);
    for (uint32_t i = 0U; i < mat->m; i++) {
        mat->data[i * mat->m + i] = A->data[i];
    }
}

// (2*N-x)*(x+1)/2 - (N-y)
double* symmetric_index(symmetric_double_t* A, int x, int y) {
    if (A == NULL || A->data == NULL ||
        A->n <= x || A->n <= y || x > y)
    {
        printf("argument fault\n");
        return NULL;
    }
    int offset = (2*A->n-x)*(x+1)/2 - (A->n-y);
    return A->data + offset;
}

double* upper_triangular_index(upper_triangular_double_t* A, int x, int y) {
    if (A == NULL || A->data == NULL ||
        A->n <= x || A->n <= y || x > y)
    {
        printf("argument fault\n");
        return NULL;
    }
    int offset = (2*A->n-x)*(x+1)/2 - (A->n-y);
    return A->data + offset;
}

double* zero_diag_triangular_index(zero_diag_triangular_double_t* A, int x, int y) {
    if (A == NULL || A->data == NULL ||
        A->n <= x || A->n <= y || x >= y)
    {
        printf("argument fault\n");
        return NULL;
    }
    int offset = (2*A->n-x)*(x+1)/2 - (A->n-y) - x - 1;
    return A->data + offset;
}

double* unit_diag_triangular_index(unit_diag_triangular_double_t* A, int x, int y) {
    if (A == NULL || A->data == NULL ||
        A->n <= x || A->n <= y || x >= y)
    {
        printf("argument fault\n");
        return NULL;
    }
    int offset = (2*A->n-x)*(x+1)/2 - (A->n-y) - x - 1;
    return A->data + offset;
}

int RtDRdecomp_D(double *A, int N) {
    int i, j, k, Ni = N;
    double a, b, *p = A, *q;
    for(i = 0; i < N; i++, Ni--) {
        //Ni = N - i; p points to A[i][i];
        //for(j=0;j<i;j++)//here we use i-j as the loop index
        for(j = i, q = A; j; j--) {
            a = *q; q += j; b = *q++;
            a *= -b; *p += a * b;
            for(k = 1; k < Ni; k++)
                p[k] += a * *q++;
        }
        a = *p++;
        if(!a)
            return 0; //fail
        a = 1.0 / a;
        for(k = 1; k < Ni; k++)
            *p++ *= a;
    }
    return 1;
}

uint32_t Symmetric_RtDRdecomp(const symmetric_double_t* A, unit_diag_triangular_double_t* R, diagonal_double_t* D) {
    if (A == NULL || R == NULL || D == NULL ||
        A->data == NULL || R->data == NULL || D->data == NULL ||
        A->n != R->n || R->n != D->n)
    {
        return 1;
    }

    uint32_t N = A->n;
    uint32_t len = N*(N + 1) / 2;
    double tmp_data[len];
    upper_triangular_double_t tmp = { (double*)NULL, N};
    tmp.data = tmp_data;

    double *p = tmp_data;
    for (uint32_t i = 0 ; i < N; i ++ ) {
        for (uint32_t j = i; j < N; j ++ ) {
            *p++ = *symmetric_index(A, i, j);
        }
    }

    if(!RtDRdecomp_D(tmp_data, N)) 
		return 1;
	
    for (uint32_t i = 0; i < N; i ++ ) {
        for (uint32_t j = i; j < N; j ++ ) {
            if (i == j) {
                double* ptr_ij = upper_triangular_index(&tmp, i, j);
                D->data[i] = *ptr_ij;
            }
            else {
                *upper_triangular_index(R, i, j) = *upper_triangular_index(&tmp, i, j);
            }
        }
    }
    printf("D :\n");
    diagonal_mat_print_d(D);
    printf("R :\n");
    unit_diag_triangular_print_d(A);
}


// int ldlt_solve(float *_A, float *_x, int _N) {
//     if (!RtDRdecomp(_A, _N)) {
//         return 0;
//     }

//     float *p = _A;
//     float R[_N][_N];
//     for (int i = 0; i < _N; i++) {
//         for (int j = i; j < _N; j++) {
//             R[i][j] = *p++;
//         }
//     }

//     for (int k = 0; k < _N; k++) {
//         for (int i = 0; i < k; i++) {
//             _x[k] -= _x[i] * R[i][k];
//         }
//     }

//     for (int k = _N - 1; k >= 0; k--) {
//         _x[k] /= R[k][k];
//         for (int i = k + 1; i < _N; i++)
//             _x[k] -= _x[i] * R[k][i];
//     }

//     return 1;
// }

int ldlt_solve_D(double *_A, double *_x, int _N) {
    if (!RtDRdecomp_D(_A, _N)) {
        return 0;
    }

    double *p = _A;
    double R[_N][_N];
    for (int i = 0; i < _N; i++) {
        for (int j = i; j < _N; j++) {
            R[i][j] = *p++;
        }
    }

    for (int k = 0; k < _N; k++) {
        for (int i = 0; i < k; i++) {
            _x[k] -= _x[i] * R[i][k];
        }
    }

    for (int k = _N - 1; k >= 0; k--) {
        _x[k] /= R[k][k];
        for (int i = k + 1; i < _N; i++)
            _x[k] -= _x[i] * R[k][i];
    }
    
    return 1;
}

