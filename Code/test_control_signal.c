#include<stdio.h>
#include<math.h>
#include<stdlib.h>
#include "matrices_op.c"

double sampling_interval = pow(10,-4);

// START
void control_signal(matrix *v_c_new, matrix *v_c_old, matrix *v)
{
    double k_4 = 0.01;
    matrix K_4;
    allocate_matrix(&K_4,2,2);
    K_4.index[0][0] = k_4 * 1;
    K_4.index[1][1] = k_4 * 1;

    matrix u;
    allocate_matrix(&u,2,1);

    double x  = (v_c_new->index[0][0] - v_c_old->index[0][0]) / sampling_interval;
    double y  = (v_c_new->index[1][0] - v_c_old->index[1][0]) / sampling_interval;
    matrix C;
    allocate_matrix(&C,2,1);
    subtract(v_c_new,v);
    mutiplication(&K_4, v_c_new, &C);
    double a = u.index[0][0] = x + C.index[0][0];
    double b = u.index[1][0] = y + C.index[1][0];
    printf("%.12lf, %.12lf", a,b);

    deallocate_matrix(&C);
    deallocate_matrix(&u);
  }

int main()
{   
    matrix v_c_new;
    allocate_matrix(&v_c_new,2,1);
    for (int i  = 0; i < 2; i++){
        for (int j = 0; j < 1; j++){
            scanf("%lf", &v_c_new.index[i][j]);
        }
    }

    matrix v_c_old;
    allocate_matrix(&v_c_old,2,1);
    for (int i  = 0; i < 2; i++){
        for (int j = 0; j < 1; j++){
            scanf("%lf", &v_c_old.index[i][j]);
        }
    }

    matrix v;
    allocate_matrix(&v,2,1);
    for (int i  = 0; i < 2; i++){
        for (int j = 0; j < 1; j++){
            scanf("%lf", &v.index[i][j]);
        }
    }


    control_signal(&v_c_new, &v_c_old, &v);
}