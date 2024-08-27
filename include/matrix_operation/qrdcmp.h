#pragma once
#include <nr3.h>

#include <cmath>
#include <stdlib.h>
#include <stdio.h>
#include "linearalgebra.h"
#include "geometry.h"

struct QRdcmp {
    int n, m;
    MAT_DYNAMIC_D QT, R;

    QRdcmp(MAT_DYNAMIC_D* A);

    ~QRdcmp();

    void solve(const MAT_DYNAMIC_D* b, MAT_DYNAMIC_D* x);

    void qtmult(const MAT_DYNAMIC_D* b, MAT_DYNAMIC_D* x);

    void rsolve(const MAT_DYNAMIC_D* b, MAT_DYNAMIC_D* x);

    double norm(double v[], int length);
};

QRdcmp::QRdcmp(MAT_DYNAMIC_D* A) {
	m = A->rows;
	n = A->cols;

    NEW_MAT_DYNAMIC_D(&QT, m, m);
    NEW_MAT_DYNAMIC_D(&R, m, n);

    for (int i = 0; i < m; i ++ )
    {
        for (int j = 0; j < m; j ++ )
        {
            QT.p[i][j] = i == j ? 1.0 : 0.0;
        }
    }

    for (int i = 0; i < m; i ++ )
    {
        for (int j = 0; j < n; j ++ )
        {
            R.p[i][j] = A->p[i][j];
        }
    }

    MAT_DYNAMIC_D H_mat;
    MAT_DYNAMIC_D* H = &H_mat;
    NEW_MAT_DYNAMIC_D(H, m, m);

    for (int k = 0; k < n; ++k) {
        double x[m - k];
        for (int i = 0; i < m-k; i ++ )
        {
            x[i] = R.p[k+i][k];
        }

        x[0] -= norm(x, m-k);

        double x_norm = norm(x, m-k);
        for (int i = 0; i < m-k; i ++ )
        {
            x[i] = x[i]/x_norm;
        }

        for (int i = 0; i < m; i ++ )
        {
            for (int j = 0; j < m; j ++ )
            {
                H->p[i][j] = i == j ? 1.0 : 0.0;
            }
        }

        for (int i = 0; i < m-k; i ++ )
        {
            for (int j = 0; j < m-k; j ++ )
            {
                H->p[i+k][j+k] -= 2.0 * x[i] * x[j];
            }
        }

        MultipyMatrix(H, &R);
        MultipyMatrix(H, &QT);
	}

    FREE_MAT_DYNAMIC_D(H);
}

QRdcmp::~QRdcmp()
{
    FREE_MAT_DYNAMIC_D(&QT);
    FREE_MAT_DYNAMIC_D(&R);
}

void QRdcmp::solve(const MAT_DYNAMIC_D* b, MAT_DYNAMIC_D* x) {
    MAT_DYNAMIC_D QTb;
    NEW_MAT_DYNAMIC_D(&QTb, b->rows,1);

    qtmult(b, &QTb);
    rsolve(&QTb, x);

    FREE_MAT_DYNAMIC_D(&QTb);
}

void QRdcmp::qtmult(const MAT_DYNAMIC_D* b, MAT_DYNAMIC_D* x) {
    int i, j;
    double sum;
    for (i = 0; i < m; i++) {
        sum = 0.;
        for (j = 0; j < m; j++) sum += QT.p[i][j] * b->p[j][0];
        x->p[i][0] = sum;
    }
}

double QRdcmp::norm(double v[], int length)
{
    double res = 0.0;
    for (int i = 0; i < length; i ++ )
    {
        res += v[i] * v[i];
    }
    return sqrt(res);
}

void QRdcmp::rsolve(const MAT_DYNAMIC_D* b, MAT_DYNAMIC_D* x) {
    int i, j;
    double sum;
    for (i = n - 1; i >= 0; i--) {
        sum = b->p[i][0];
        for (j = i + 1; j < n; j++) sum -= R.p[i][j] * x->p[j][0];
        x->p[i][0] = sum / R.p[i][i];
    }
}