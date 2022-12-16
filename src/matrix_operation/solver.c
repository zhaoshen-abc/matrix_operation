#include "solver.h"

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

int ldlt_solve(float *_A, float *_x, int _N, float *_bufferNxN) {
    if (!RtDRdecomp(_A, _N)) {
        return 0;
    }

    float *p = _A;
    if(!_bufferNxN) {
//            float R[_N][_N];
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
    }
    else {
        for (int i = 0; i < _N; i++) {
            for (int j = i; j < _N; j++) {
                _bufferNxN[i * _N + j] = *p++;
            }
        }

        for (int k = 0; k < _N; k++) {
            for (int i = 0; i < k; i++) {
                _x[k] -= _x[i] * _bufferNxN[i * _N + k];
            }
        }

        for (int k = _N - 1; k >= 0; k--) {
            _x[k] /= _bufferNxN[k * _N + k];
            for (int i = k + 1; i < _N; i++)
                _x[k] -= _x[i] * _bufferNxN[k * _N + i];
        }
    }

    return 1;
}

int ldlt_solve(double *_A, double *_x, int _N, double *_bufferNxN) {
    if (!RtDRdecomp(_A, _N)) {
        return 0;
    }

    double *p = _A;
    if(!_bufferNxN) {
//            double R[_N][_N];
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
    }
    else {
        for (int i = 0; i < _N; i++) {
            for (int j = i; j < _N; j++) {
                _bufferNxN[i * _N + j] = *p++;
            }
        }

        for (int k = 0; k < _N; k++) {
            for (int i = 0; i < k; i++) {
                _x[k] -= _x[i] * _bufferNxN[i * _N + k];
            }
        }

        for (int k = _N - 1; k >= 0; k--) {
            _x[k] /= _bufferNxN[k * _N + k];
            for (int i = k + 1; i < _N; i++)
                _x[k] -= _x[i] * _bufferNxN[k * _N + i];
        }
    }

    return 1;
}

