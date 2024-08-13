/**************************************************************************
**
**  svd3
**
** Quick singular value decomposition as described by: 
** A. McAdams, A. Selle, R. Tamstorf, J. Teran and E. Sifakis, 
** "Computing the Singular Value Decomposition of 3x3 matrices 
** with minimal branching and elementary floating point operations",
**  University of Wisconsin - Madison technical report TR1690, May 2011
**  
**	OPTIMIZED CPU VERSION
** 	Implementation by: Eric Jang
**	 
**  13 Apr 2014
**
**************************************************************************/


#ifndef SVD3_H
#define SVD3_H

#define _gamma 5.828427124 // FOUR_GAMMA_SQUARED = sqrt(8)+3;
#define _cstar 0.923879532 // cos(pi/8)
#define _sstar 0.3826834323 // sin(p/8)
#define EPSILON 1e-6

#include <math.h>
#include <stdbool.h>
/* This is a novel and fast routine for the reciprocal square root of an
IEEE float (single precision).

http://www.lomont.org/Math/Papers/2003/InvSqrt.pdf
http://playstation2-linux.com/download/p2lsd/fastrsqrt.pdf
http://www.beyond3d.com/content/articles/8/
*/
inline float rsqrt(float x);

/* This is rsqrt with an additional step of the Newton iteration, for
increased accuracy. The constant 0x5f37599e makes the relative error
range from 0 to -0.00000463.
   You can't balance the error by adjusting the constant. */
inline float rsqrt1(float x);

inline float accurateSqrt(float x);
inline void condSwap(bool c, float &X, float &Y);

inline void condNegSwap(bool c, float &X, float &Y);

// matrix multiplication M = A * B
inline void multAB(float a11, float a12, float a13,
          float a21, float a22, float a23,
          float a31, float a32, float a33,
          //
          float b11, float b12, float b13,
          float b21, float b22, float b23,
          float b31, float b32, float b33,
          //
          float &m11, float &m12, float &m13,
          float &m21, float &m22, float &m23,
          float &m31, float &m32, float &m33);

// matrix multiplication M = Transpose[A] * B
inline void multAtB(float a11, float a12, float a13,
          float a21, float a22, float a23,
          float a31, float a32, float a33,
          //
          float b11, float b12, float b13,
          float b21, float b22, float b23,
          float b31, float b32, float b33,
          //
          float &m11, float &m12, float &m13,
          float &m21, float &m22, float &m23,
          float &m31, float &m32, float &m33);

inline void quatToMat3(const float * qV,
float &m11, float &m12, float &m13,
float &m21, float &m22, float &m23,
float &m31, float &m32, float &m33
);

inline void approximateGivensQuaternion(float a11, float a12, float a22, float &ch, float &sh);

inline void jacobiConjugation( const int x, const int y, const int z,
                        float &s11,
                        float &s21, float &s22,
                        float &s31, float &s32, float &s33,
                        float * qV);

inline float dist2(float x, float y, float z);

// finds transformation that diagonalizes a symmetric matrix
inline void jacobiEigenanlysis( // symmetric matrix
								float &s11,
								float &s21, float &s22,
								float &s31, float &s32, float &s33,
								// quaternion representation of V
                                float * qV);

inline void sortSingularValues(// matrix that we want to decompose
							float &b11, float &b12, float &b13,
							float &b21, float &b22, float &b23,
							float &b31, float &b32, float &b33,
						  // sort V simultaneously
							float &v11, float &v12, float &v13,
							float &v21, float &v22, float &v23,
                            float &v31, float &v32, float &v33);

void QRGivensQuaternion(float a1, float a2, float &ch, float &sh);

inline void QRDecomposition(// matrix that we want to decompose
							float b11, float b12, float b13,
							float b21, float b22, float b23,
							float b31, float b32, float b33,
							// output Q
							float &q11, float &q12, float &q13,
							float &q21, float &q22, float &q23,
							float &q31, float &q32, float &q33,
							// output R
							float &r11, float &r12, float &r13,
							float &r21, float &r22, float &r23,
							float &r31, float &r32, float &r33);

void svd(// input A
		float a11, float a12, float a13,
		float a21, float a22, float a23,
		float a31, float a32, float a33,
		// output U
		float &u11, float &u12, float &u13,
		float &u21, float &u22, float &u23,
		float &u31, float &u32, float &u33,
		// output S
		float &s11, float &s12, float &s13,
		float &s21, float &s22, float &s23,
        float &s31, float &s32, float &s33,
		// output V
		float &v11, float &v12, float &v13,
		float &v21, float &v22, float &v23,
		float &v31, float &v32, float &v33);

/// polar decomposition can be reconstructed trivially from SVD result
// A = UP
void pd(float a11, float a12, float a13,
        float a21, float a22, float a23,
        float a31, float a32, float a33,
        // output U
        float &u11, float &u12, float &u13,
        float &u21, float &u22, float &u23,
        float &u31, float &u32, float &u33,
        // output P
        float &p11, float &p12, float &p13,
        float &p21, float &p22, float &p23,
        float &p31, float &p32, float &p33);
#endif
