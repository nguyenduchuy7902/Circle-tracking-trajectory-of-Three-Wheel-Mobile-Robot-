#include<stdio.h>
#include<math.h>
#include "matrices_op.c"

double sampling_interval = pow(10,-5);
double m = 1.5, d = 1, I = 1, r = 1, R = 1;
double x, y, theta, x_r, y_r, theta_r;
double e_x, e_y, e_theta;
double w_r, v_r;
double x_old, y_old, theta_old;


void error(double x, double y, double theta, double x_r, double y_r, double theta_r, double *e_x, double *e_y, double *e_theta)
{
    *e_x = cos(theta) * (x_r - x) + sin(theta) * (y_r - y);
    *e_y = (-1) * sin(theta) * (x_r - x) + cos(theta) * (y_r - y);
    *e_theta = theta_r - theta;
}

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
  }

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
    deallocate_matrix(&C);
    deallocate_matrix(&u);
  }

void next_state(matrix *v, double *x, double *y, double *theta, double *x_old, double *y_old, double *theta_old)
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

void torque(double theta, matrix *v, matrix *u )
  {
    double derivative_theta = v->index[1][0];

    matrix M;
    allocate_matrix(&M,3,3);
   
    M.index[0][0] = M.index[1][1] = m;
    M.index[0][2] = M.index[2][0] = m * d * sin(theta);
    M.index[1][2] = M.index[2][1] = -m * d * cos(theta);
    M.index[2][2] = I;

    matrix S;
    allocate_matrix(&S,3,2);
    S.index[0][0] = cos(theta);
    S.index[0][1] = -d * sin(theta);
    S.index[1][0] = sin(theta);
    S.index[1][1] = d*cos(theta);
    S.index[2][1] = 1;

    matrix V;
    allocate_matrix(&V,3,3);
    V.index[0][1] = m * d * pow(derivative_theta,2) * cos(theta);
    V.index[1][1] = m * d * pow(derivative_theta,2) * sin(theta);

    matrix derivative_S;
    allocate_matrix(&derivative_S,3,2);
    derivative_S.index[0][0] = - sin(theta) * derivative_theta;
    derivative_S.index[0][1] = -d * cos(theta) * derivative_theta;
    derivative_S.index[1][0] = cos(theta) * derivative_theta;
    derivative_S.index[1][1] = -d * sin(theta) * derivative_theta;

    matrix B;
    allocate_matrix(&B,3,2);
    B.index[0][0] = B.index[0][1] = (double) 1/r * cos(theta);
    B.index[1][0] = B.index[1][1] =  (double) 1/r * sin(theta);
    B.index[2][0] = (double) 1/r * R;
    B.index[2][1] = (double) 1/r * (-R);

    matrix S_t;
    allocate_matrix(&S_t,2,3);
    transpose(&S,&S_t);

    matrix S_t_B;
    allocate_matrix(&S_t_B,2,2);
    mutiplication(&S_t,&B,&S_t_B);
    

// done matrix B
    deallocate_matrix(&B);

    matrix S_t_B_inv;
    allocate_matrix(&S_t_B_inv,2,2);
    inverse(&S_t_B,&S_t_B_inv);

// done matrix S_t_B
    deallocate_matrix(&S_t_B);

    matrix S_t_M;
    allocate_matrix(&S_t_M,2,3);
    mutiplication(&S_t,&M,&S_t_M);

    matrix S_t_M_S;
    allocate_matrix(&S_t_M_S,2,2);
    mutiplication(&S_t_M, &S, &S_t_M_S);

// done matrix S_t_M
    deallocate_matrix(&S_t_M);

    matrix tu_1;
    allocate_matrix(&tu_1,2,1);
    mutiplication(&S_t_M_S, u,&tu_1);

//done matrix S_t_M_S
    deallocate_matrix(&S_t_M_S);

    matrix torque_1;
    allocate_matrix(&torque_1,2,1);
    mutiplication(&S_t_B_inv, &tu_1, &torque_1);

// done matrix S_t_B_inv, tu_1;
    deallocate_matrix(&S_t_B_inv);
    deallocate_matrix(&tu_1);

    matrix M_deri_S;
    allocate_matrix(&M_deri_S,3,2);
    mutiplication(&M,&derivative_S,&M_deri_S);

// done matrix derivative_S
    deallocate_matrix(&derivative_S);

// done matrix M
    deallocate_matrix(&M);

    matrix V_S;
    allocate_matrix(&V_S,3,2);
    mutiplication(&V,&S,&V_S);

// done matrix V
    deallocate_matrix(&V);

    addition(&M_deri_S, &V_S);

// done matrix V_S
    deallocate_matrix(&V_S);

    matrix S_t_M_deri_S;
    allocate_matrix(&S_t_M_deri_S,2,2);
    mutiplication(&S_t, &M_deri_S, &S_t_M_deri_S);

//done matrix S_t, M_deri_S
    deallocate_matrix(&S_t);
    deallocate_matrix(&M_deri_S);

    matrix tu_2;
    allocate_matrix(&tu_2,2,1);
    mutiplication(&S_t_M_deri_S, v, &tu_2);

// done matrix S_t_M_deri_S
    deallocate_matrix(&S_t_M_deri_S);

    matrix torque_2;
    allocate_matrix(&torque_2,2,1);
    mutiplication(&S_t_B_inv, &tu_2, &torque_2);

// done matrix tu_2, S_t_B_inv
    deallocate_matrix(&tu_2);
    deallocate_matrix(&S_t_B_inv);

    addition(&torque_1, &torque_2);

    deallocate_matrix(&torque_2);
  }

  
