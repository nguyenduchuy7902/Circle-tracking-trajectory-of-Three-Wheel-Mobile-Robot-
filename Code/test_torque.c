#include<stdio.h>
#include<math.h>
#include<stdlib.h>
#include "matrices_op.c"

double m = 1.5, d = 1, I = 1, r = 1, R = 1;
// START
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
    
    for (int i  = 0; i < 2; i++){
        for (int j = 0; j < 1; j++){
            printf("%lf ", torque_1.index[i][j]);
        }
    }

// done matrix torque_1, torque_2
    deallocate_matrix(&torque_1);
    deallocate_matrix(&torque_2);

  }


int main()
{
    double theta;
    scanf("%lf", &theta);

    matrix v;
    allocate_matrix(&v,2,1);
    for (int i  = 0; i < 2; i++){
        for (int j = 0; j < 1; j++){
            scanf("%lf", &v.index[i][j]);
        }
    }

    matrix u;
    allocate_matrix(&u,2,1);
    for (int i  = 0; i < 2; i++){
        for (int j = 0; j < 1; j++){
            scanf("%lf", &u.index[i][j]);
        }
    }

    torque(theta,&v,&u);
}