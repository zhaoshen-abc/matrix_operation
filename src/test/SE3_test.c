#include <math.h>
#include <stdio.h>
#include "macro.h"
#include "geometry.h"

void vector3_print_d(const VEC_D_3 vec)
{
    printf("%f, %f, %f\n", vec[0], vec[1], vec[2]);
};

int main() {
    MAT_D_3_3 mat = {1, 2, 3, 4, 5, 6, 7, 8, 9};

    printf("origin Mat33D:\n");
    ac_MAT_D_3_3_print(mat);

    MAT_D_3_3 trans_mat;
    printf("transpose:\n");
    transpose_Mat33d(mat, trans_mat);
    ac_MAT_D_3_3_print(trans_mat);
    printf("transpose inplace:\n");
    transpose_Mat33d_inplace(mat);
    ac_MAT_D_3_3_print(mat);

    MAT_D_3_3 res1;
    printf("x 1.5:\n");
    MAT33D_times(mat, 1.5, res1);
    ac_MAT_D_3_3_print(res1);
    printf("Matrix times 1.5 inplace:\n");
    MAT33D_times_inplace(mat, 1.5);
    ac_MAT_D_3_3_print(mat);

    
    VEC_D_3 v1 = {1, 2, 3}, v2;
    printf("vec3d:\n");
    vector3_print_d(v1);
    printf("Matrix multiply vec3d:\n");
    Mat33D_Vec3D_multiply(mat, v1, v2);
    vector3_print_d(v2);
    printf("Matrix multiply vec3d inplace:\n");
    Mat33D_Vec3D_multiply_inplace(mat, v1);
    vector3_print_d(v1);

    printf("Matrix operation:\n");
    MAT_D_3_3 mat1 = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    MAT_D_3_3 mat2 = {4, 5, 6, 1, 2, 3, 4, 5, 6};
    printf("mat1:\n");
    ac_MAT_D_3_3_print(mat1);
    printf("mat2:\n");
    ac_MAT_D_3_3_print(mat2);

    MAT_D_3_3 res2;
    MAT_D_3_3 res2_inplace;

    printf("+ :\n");
    MAT33D_matrix_operation(mat1, mat2, '+', res2);
    ac_MAT_D_3_3_print(res2);
    printf("+ inplace:\n");
    copy_Mat33d(mat2, res2_inplace);
    MAT33D_matrix_operation_inplace(mat1, res2_inplace, '+');
    ac_MAT_D_3_3_print(res2_inplace);

    printf("- :\n");
    MAT33D_matrix_operation(mat1, mat2, '-', res2);
    ac_MAT_D_3_3_print(res2);
    printf("- inplace:\n");
    copy_Mat33d(mat2, res2_inplace);
    MAT33D_matrix_operation_inplace(mat1, res2_inplace, '-');
    ac_MAT_D_3_3_print(res2_inplace);

    printf("* :\n");
    MAT33D_matrix_operation(mat1, mat2, '*', res2);
    ac_MAT_D_3_3_print(res2);
    printf("* inplace:\n");
    copy_Mat33d(mat2, res2_inplace);
    MAT33D_matrix_operation_inplace(mat1, res2_inplace, '*');
    ac_MAT_D_3_3_print(res2_inplace);

    printf(". :\n");
    MAT33D_matrix_operation(mat1, mat2, '.', res2);
    ac_MAT_D_3_3_print(res2);
    printf(". inplace:\n");
    copy_Mat33d(mat2, res2_inplace);
    MAT33D_matrix_operation_inplace(mat1, res2_inplace, '.');
    ac_MAT_D_3_3_print(res2_inplace);

    printf("/ :\n");
    MAT33D_matrix_operation(mat1, mat2, '/', res2);
    ac_MAT_D_3_3_print(res2);
    printf("/ inplace:\n");
    copy_Mat33d(mat2, res2_inplace);
    MAT33D_matrix_operation_inplace(mat1, res2_inplace, '/');
    ac_MAT_D_3_3_print(res2_inplace);

    printf("vec:\n");
    vector3_print_d(v1);
    printf("vec skewd:\n");
    MAT_D_3_3 v1_skewed;
    skewd(v1, v1_skewed);
    ac_MAT_D_3_3_print(v1_skewed);
    printf("vec norm:\n");
    printf("%f \n", norm_V3d(v1));


    printf("Identity Pose:\n");
    POSE_D identity_mat;
    set_Identity_POSE_D(identity_mat);
    ac_POSE_D_print(identity_mat);

    printf("Rotation Matrix :\n");
    MAT_D_3_3 R = {1, 0, 0,
                   0, 1, 0, 
                   0, 0, 1};
    ac_MAT_D_3_3_print(R);
    printf("Rotation Vector :\n");
    VEC_D_3 v_so3;
    Log_SO3d(R, v_so3);
    vector3_print_d(v_so3);
    // MAT_D_3_3 R_so3;
    // Exp6d(v_so3, v_so3);
    // vector3_print_d(v_so3);


    // ia_err Exp6d(const VEC_D_6 _dx, POSE_D pose);
    
    // ia_err mat2qua(const MAT_D_3_3 m, Quaternion_D qua);

    // ia_err inversePose(const POSE_D pose_in, POSE_D pose_out);
    
    // ia_err inversePose_inplace(POSE_D pose);

    // ia_err multipyPose(const POSE_D lpose, const POSE_D rpose, POSE_D res);

    // ia_err multipyPose_inplace(const POSE_D lpose, POSE_D rpose);

	return 0;
}
