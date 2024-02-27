#include <stdlib.h>

//1 byte integer
typedef unsigned char unsigned_int8;

//Structure of matrix
struct MATRIX_struct
{
    double **index;
    unsigned_int8 num_columns;
    unsigned_int8 num_rows;
};

//Define matrix type
typedef struct MATRIX_struct matrix;

//Standard matrices operator
void allocate_matrix(matrix *A, unsigned_int8 num_rows, unsigned_int8 num_columns);
void deallocate_matrix(matrix *A);
void reallocate_matrix(matrix *A, unsigned_int8 num_rows, unsigned_int8 num_columns);
void addition(matrix *A, matrix *B);
void scalar_multiplication(matrix *A, double scalar);
void mutiplication(matrix *A, matrix *B, matrix *Ans);
void transpose(matrix *A, matrix *transpose_A);
void inverse(matrix *A, matrix *inverse_of_A);
void adjoint(matrix *A, matrix *Ans);
double determinant(matrix *A, unsigned_int8 expand_row);
void minor(matrix *major, matrix *minor, unsigned_int8 skip_row, unsigned_int8 skip_column);
void subtract(matrix *A, matrix *B);
//Main program function
// void error();
// void virtual_veloctiy();
// void control_signal();
// void torque();
// void voltage();
// void next_state();