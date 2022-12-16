#include <math.h>
#include "ia_abstraction.h"
#include "ac_auxiliary.h"
#include "macro.h"

// #define LOG_AC
// #define _DEBUG
int main() {
    AC_LOG(ia_log_error, "hello world");

    float32_t mat[3][3]; // MAT_3_3
    for (int i = 0; i < 3; i ++ ) {
        for (int j = 0; j < 3; j ++ ) {
            mat[i][j] = 0.1f+ i;
        }
    }
    
    for (int i = 0; i < 3; i ++ ) {
        for (int j = 0; j < 3; j ++ ) {
            printf("%f ", mat[i][j]);
        }
    }
    return 0;
}