#include<stdio.h>
#include<math.h>
#include "matrices_op.c"

void error(double x, double y, double x_r, double y_r, double theta, double theta_r, double *e_x, double *e_y, double *e_theta)
{
    *e_x = cos(theta) * (x_r - x) + sin(theta) * (y_r - y);
    *e_y = (-1) * sin(theta) * (x_r - x) + cos(theta) * (y_r - y);
    *e_theta = theta_r - theta;
    printf("%.12lf %.12lf %.12lf", *e_x, *e_y, *e_theta);
}

int main()
{
    double e_x, e_y, e_theta;
    double x, y, x_r, y_r, theta, theta_r;
    scanf("%lf %lf %lf %lf %lf %lf", &x, &y, &x_r, &y_r, &theta, &theta_r);
    error(x, y, x_r, y_r, theta, theta_r, &e_x, &e_y, &e_theta);
}