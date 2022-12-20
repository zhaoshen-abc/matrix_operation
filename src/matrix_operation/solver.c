#include "solver.h"

#define MAX_COL 20U

// void symmetric_mat_print_d(symmetric_double_t* mat)
// {
//     if (mat == NULL || mat->n > MAX_COL) {
//         return;
//     }

//     uint32_t row = IA_MIN(mat->m, m);
//     uint32_t col = IA_MIN(mat->n, n);
//     char buf[MAX_COL * 64U] = {0};
//     char *ptr = buf;
//     uint32_t i = 0U;
//     uint32_t j = 0U;
//     uint32_t k = 0U;
//     uint32_t idx = 0U;
//     for (i = 0U; i < row; i++) {
//         ptr = buf;
//         for (j = 0U; j < col; j++) {
//             idx = i * col + j;
//             double data = mat->data[idx];
//             int size = sprintf(ptr, "%f, ", data);
//             ptr += size;
//         }
//         AC_LOG("%s", buf);
//     }
// }

// void upper_triangular_print_d(upper_triangular_double_t* mat);

// void diagonal_mat_print_d(diagonal_double_t* mat);


float* symmat2vector_N(const ac_matrix_f32_t* matrix, int N) {
    float* vec = malloc(N*(N + 1) / 2 * sizeof(float));
    float *p = vec;
    for (int i = 0 ; i < N; i ++ ) {
        for (int j = i; j < N; j ++ ) {
            *p++ = matrix->data[i][j];
        }
    }
    return vec;
}

double* symmat2vector_N_D(const ac_matrix_double_t* matrix, int N) {
    double* vec = malloc(N*(N + 1) / 2 * sizeof(double));
    double *p = vec;
    for (int i = 0 ; i < N; i ++ ) {
        for (int j = i; j < N; j ++ ) {
            *p++ = matrix->data[i][j];
        }
    }
    return vec;
}

int RtDRdecomp(float *A, int N) {
    int i, j, k, Ni = N;
    double a, b, *p = A, *q;
    for(i = 0; i < N; i++, Ni--) {
        //Ni = N - i; p points to A[i][i];
//            for(j=0;j<i;j++)//here we use i-j as the loop index
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

int RtDRdecomp_D(double *A, int N) {
    int i, j, k, Ni = N;
    float a, b, *p = A, *q;
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

ia_err Symmetric_RtDRdecomp(const symmetric_double_t* A, upper_triangular_double_t* R, diagonal_double_t* D) {
    if (A == NULL || R == NULL || D == NULL ||
        A->data == NULL || R->data == NULL || D->data == NULL ||
        A->n != R->n || R->n != D->n)
    {
        return ia_err_argument;
    }

    int N = A->n;
    float* vec = malloc(N*(N + 1) / 2 * sizeof(float));
    float *p = vec;
    for (int i = 0 ; i < N; i ++ ) {
        for (int j = i; j < N; j ++ ) {
            *p++ = A->data[i * N + j];
        }
    }


    if(!RtDRdecomp(vec, N)) 
		return ia_err_data;
	
    double** mat2 = vector2symmat_N_D(vec, N);
    print_mat_N_D(mat2, N);
    for (int i = 0; i < N; i ++ ) {
        for (int j = i; j < N; j ++ ) {
            if (i == j) {
                D->data[i * N + j] = A->data[i * N + j];
                R->data[i * N + j] = 1;
            }
            else {
                R->data[i * N + j] = A->data[i * N + j];
                R->data[i * N + j] = A->data[i * N + j];
                D->data[i * N + j] = D->data[i * N + j] = 0;
            }
        }
    }
    puts("D :");
    print_mat_N_D(D, N);
    puts("A :");
    print_mat_N_D(A, N);

}


int ldlt_solve(float *_A, float *_x, int _N) {
    if (!RtDRdecomp(_A, _N)) {
        return 0;
    }

    float *p = _A;
    float R[_N][_N];
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

int ldlt_solve_D(double *_A, double *_x, int _N) {
    if (!RtDRdecomp(_A, _N)) {
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

