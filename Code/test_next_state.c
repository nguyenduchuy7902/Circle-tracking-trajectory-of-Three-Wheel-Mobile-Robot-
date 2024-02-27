#include<stdio.h>
#include<math.h>
#include<stdlib.h>
#include "matrices_op.c"

double x, y, theta;
double sampling_interval = pow(10,-5);
// START

void next_state(matrix *v, double *x, double *y, double *theta, double *x_old, double *y_old, double *theta_old)
  {
    {
    double derivative_x = cos(*theta_old) * v->index[0][0];
    double derivative_y = sin(*theta_old) * v->index[0][0];
    double derivative_theta = v->index[1][0];

     *x = *x_old + sampling_interval * derivative_x;
     *y = *y_old + sampling_interval * derivative_y;
     *theta = *theta_old + sampling_interval * derivative_theta;

     *x_old = *x;
     *y_old = *y;
     *theta_old = *theta;
    }

    printf("%.20lf %.20lf %.20lf", *x, *y, *theta);
  }

int main()
{
    double x_old, y_old, theta_old;
    scanf("%lf %lf %lf", &x_old, &y_old, &theta_old);

    matrix v;
    allocate_matrix(&v,2,1);
    for (int i  = 0; i < 2; i++){
        for (int j = 0; j < 1; j++){
            scanf("%lf", &v.index[i][j]);
        }
    }
    next_state(&v, &x, &y, &theta, &x_old, &y_old, &theta_old);
}