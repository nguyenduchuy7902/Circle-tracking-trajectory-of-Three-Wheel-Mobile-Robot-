#include<stdio.h>
#include<math.h>
#include<stdlib.h>
#include "matrices_op.c"


 
void virtual_control(double e_x, double e_y, double e_theta, double v_r, double w_r)
  {
    matrix K;
    allocate_matrix(&K, 3, 1); 
    matrix v_c;
    allocate_matrix(&v_c,2,1);
    K.index[0][0] = K.index[1][0] = K.index[2][0] = 1000;
    double a = v_c.index[0][0] = v_r * cos(e_theta) + K.index[0][0] * e_x;
    double b = v_c.index[1][0] = w_r + K.index[1][0] * v_r * e_y + K.index[2][0] * v_r * sin(e_theta);
    deallocate_matrix(&K);
    printf("%.12lf %.12lf", a, b);
  }

int main()
{
    double e_x,e_y,e_theta, v_r, w_r;
    scanf("%lf %lf %lf %lf %lf", &e_x,&e_y,&e_theta,&v_r,&w_r);
    virtual_control(e_x,e_y,e_theta,v_r,w_r);
}
