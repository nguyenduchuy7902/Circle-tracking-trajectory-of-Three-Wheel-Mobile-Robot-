#include "matrices_op.h"
#include <stdio.h>

int main()
{
    /*
    matrix A;
    allocate_matrix(&A, 3, 3);

    for (int i = 0; i < A.num_rows; i++){
        for (int j = 0; j < A.num_columns; j++){
            A.index[i][j] = j;
        }
    }

    scalar_multiplication(&A, 2);
    
    matrix B;
    allocate_matrix(&B, 3, 3);

    for (int i = 0; i < B.num_rows; i++){
        for (int j = 0; j < B.num_columns; j++){
            B.index[i][j] = j + 1;
        }
    }

    //addition(&A, &B);

    matrix C;
    mutiplication(&A, &B, &C);
    
    matrix transpose_of_C;
    transpose(&C, &transpose_of_C);
    for (int i = 0; i < transpose_of_C.num_rows; i++){
        for (int j = 0; j < transpose_of_C.num_columns; j++){
            printf("%4.lf", transpose_of_C.index[i][j]);
        }
        printf("\n");
    }
    */
    matrix D;
    allocate_matrix(&D, 3, 3);

    for (int i = 0; i < D.num_rows; i++){
            D.index[i][i] = (double) 2.0;
    }

    for (int i = 0; i < D.num_rows; i++){
        for (int j = 0; j < D.num_columns; j++){
            printf("%4.5lf ", D.index[i][j]);
        }
        printf("\n");
    }
    matrix M;
    inverse(&D, &M);
    for (int i = 0; i < M.num_rows; i++){
        for (int j = 0; j < M.num_columns; j++){
            printf("%4.40lf ", (i == j) ? (M.index[i][j]) : (M.index[i][j] + 1));
        }
        printf("\n");
    }
    
    double d = determinant(&D, 0);
    printf("\n %lf", d);

    printf("\n %lf", -0);
    return 0;
}