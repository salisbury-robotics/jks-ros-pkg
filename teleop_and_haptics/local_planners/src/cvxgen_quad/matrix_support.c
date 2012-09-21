/* Produced by CVXGEN, 2012-09-18 18:21:14 -0700.  */
/* CVXGEN is Copyright (C) 2006-2011 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2011 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */

#include "solver.h"

void CVX_Quad::multbymA(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-params.J_v[0])-rhs[1]*(-params.J_v[3])-rhs[2]*(-params.J_v[6])-rhs[3]*(-params.J_v[9])-rhs[4]*(-params.J_v[12])-rhs[5]*(-params.J_v[15])-rhs[6]*(-params.J_v[18])-rhs[7]*(-1);
  lhs[1] = -rhs[0]*(-params.J_v[1])-rhs[1]*(-params.J_v[4])-rhs[2]*(-params.J_v[7])-rhs[3]*(-params.J_v[10])-rhs[4]*(-params.J_v[13])-rhs[5]*(-params.J_v[16])-rhs[6]*(-params.J_v[19])-rhs[8]*(-1);
  lhs[2] = -rhs[0]*(-params.J_v[2])-rhs[1]*(-params.J_v[5])-rhs[2]*(-params.J_v[8])-rhs[3]*(-params.J_v[11])-rhs[4]*(-params.J_v[14])-rhs[5]*(-params.J_v[17])-rhs[6]*(-params.J_v[20])-rhs[9]*(-1);
  lhs[3] = -rhs[0]*(-params.J_w[0])-rhs[1]*(-params.J_w[3])-rhs[2]*(-params.J_w[6])-rhs[3]*(-params.J_w[9])-rhs[4]*(-params.J_w[12])-rhs[5]*(-params.J_w[15])-rhs[6]*(-params.J_w[18])-rhs[10]*(-1);
  lhs[4] = -rhs[0]*(-params.J_w[1])-rhs[1]*(-params.J_w[4])-rhs[2]*(-params.J_w[7])-rhs[3]*(-params.J_w[10])-rhs[4]*(-params.J_w[13])-rhs[5]*(-params.J_w[16])-rhs[6]*(-params.J_w[19])-rhs[11]*(-1);
  lhs[5] = -rhs[0]*(-params.J_w[2])-rhs[1]*(-params.J_w[5])-rhs[2]*(-params.J_w[8])-rhs[3]*(-params.J_w[11])-rhs[4]*(-params.J_w[14])-rhs[5]*(-params.J_w[17])-rhs[6]*(-params.J_w[20])-rhs[12]*(-1);
}

void CVX_Quad::multbymAT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-params.J_v[0])-rhs[1]*(-params.J_v[1])-rhs[2]*(-params.J_v[2])-rhs[3]*(-params.J_w[0])-rhs[4]*(-params.J_w[1])-rhs[5]*(-params.J_w[2]);
  lhs[1] = -rhs[0]*(-params.J_v[3])-rhs[1]*(-params.J_v[4])-rhs[2]*(-params.J_v[5])-rhs[3]*(-params.J_w[3])-rhs[4]*(-params.J_w[4])-rhs[5]*(-params.J_w[5]);
  lhs[2] = -rhs[0]*(-params.J_v[6])-rhs[1]*(-params.J_v[7])-rhs[2]*(-params.J_v[8])-rhs[3]*(-params.J_w[6])-rhs[4]*(-params.J_w[7])-rhs[5]*(-params.J_w[8]);
  lhs[3] = -rhs[0]*(-params.J_v[9])-rhs[1]*(-params.J_v[10])-rhs[2]*(-params.J_v[11])-rhs[3]*(-params.J_w[9])-rhs[4]*(-params.J_w[10])-rhs[5]*(-params.J_w[11]);
  lhs[4] = -rhs[0]*(-params.J_v[12])-rhs[1]*(-params.J_v[13])-rhs[2]*(-params.J_v[14])-rhs[3]*(-params.J_w[12])-rhs[4]*(-params.J_w[13])-rhs[5]*(-params.J_w[14]);
  lhs[5] = -rhs[0]*(-params.J_v[15])-rhs[1]*(-params.J_v[16])-rhs[2]*(-params.J_v[17])-rhs[3]*(-params.J_w[15])-rhs[4]*(-params.J_w[16])-rhs[5]*(-params.J_w[17]);
  lhs[6] = -rhs[0]*(-params.J_v[18])-rhs[1]*(-params.J_v[19])-rhs[2]*(-params.J_v[20])-rhs[3]*(-params.J_w[18])-rhs[4]*(-params.J_w[19])-rhs[5]*(-params.J_w[20]);
  lhs[7] = -rhs[0]*(-1);
  lhs[8] = -rhs[1]*(-1);
  lhs[9] = -rhs[2]*(-1);
  lhs[10] = -rhs[3]*(-1);
  lhs[11] = -rhs[4]*(-1);
  lhs[12] = -rhs[5]*(-1);
}

void CVX_Quad::multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-1);
  lhs[1] = -rhs[1]*(-1);
  lhs[2] = -rhs[2]*(-1);
  lhs[3] = -rhs[3]*(-1);
  lhs[4] = -rhs[4]*(-1);
  lhs[5] = -rhs[5]*(-1);
  lhs[6] = -rhs[6]*(-1);
  lhs[7] = -rhs[0]*(1);
  lhs[8] = -rhs[1]*(1);
  lhs[9] = -rhs[2]*(1);
  lhs[10] = -rhs[3]*(1);
  lhs[11] = -rhs[4]*(1);
  lhs[12] = -rhs[5]*(1);
  lhs[13] = -rhs[6]*(1);
}

void CVX_Quad::multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-1)-rhs[7]*(1);
  lhs[1] = -rhs[1]*(-1)-rhs[8]*(1);
  lhs[2] = -rhs[2]*(-1)-rhs[9]*(1);
  lhs[3] = -rhs[3]*(-1)-rhs[10]*(1);
  lhs[4] = -rhs[4]*(-1)-rhs[11]*(1);
  lhs[5] = -rhs[5]*(-1)-rhs[12]*(1);
  lhs[6] = -rhs[6]*(-1)-rhs[13]*(1);
  lhs[7] = 0;
  lhs[8] = 0;
  lhs[9] = 0;
  lhs[10] = 0;
  lhs[11] = 0;
  lhs[12] = 0;
}

void CVX_Quad::multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(2*params.weight_q[0]);
  lhs[1] = rhs[1]*(2*params.weight_q[0]);
  lhs[2] = rhs[2]*(2*params.weight_q[0]);
  lhs[3] = rhs[3]*(2*params.weight_q[0]);
  lhs[4] = rhs[4]*(2*params.weight_q[0]);
  lhs[5] = rhs[5]*(2*params.weight_q[0]);
  lhs[6] = rhs[6]*(2*params.weight_q[0]);
  lhs[7] = rhs[7]*(2*params.weight_x[0]);
  lhs[8] = rhs[8]*(2*params.weight_x[0]);
  lhs[9] = rhs[9]*(2*params.weight_x[0]);
  lhs[10] = rhs[10]*(2*params.weight_w[0]);
  lhs[11] = rhs[11]*(2*params.weight_w[0]);
  lhs[12] = rhs[12]*(2*params.weight_w[0]);
}

void CVX_Quad::fillq(void) {
  work.q[0] = 0;
  work.q[1] = 0;
  work.q[2] = 0;
  work.q[3] = 0;
  work.q[4] = 0;
  work.q[5] = 0;
  work.q[6] = 0;
  work.q[7] = 0;
  work.q[8] = 0;
  work.q[9] = 0;
  work.q[10] = 0;
  work.q[11] = 0;
  work.q[12] = 0;
}

void CVX_Quad::fillh(void) {
  work.h[0] = -(params.q_min[0]-params.q[0]);
  work.h[1] = -(params.q_min[1]-params.q[1]);
  work.h[2] = -(params.q_min[2]-params.q[2]);
  work.h[3] = -(params.q_min[3]-params.q[3]);
  work.h[4] = -(params.q_min[4]-params.q[4]);
  work.h[5] = -(params.q_min[5]-params.q[5]);
  work.h[6] = -(params.q_min[6]-params.q[6]);
  work.h[7] = -(params.q[0]-params.q_max[0]);
  work.h[8] = -(params.q[1]-params.q_max[1]);
  work.h[9] = -(params.q[2]-params.q_max[2]);
  work.h[10] = -(params.q[3]-params.q_max[3]);
  work.h[11] = -(params.q[4]-params.q_max[4]);
  work.h[12] = -(params.q[5]-params.q_max[5]);
  work.h[13] = -(params.q[6]-params.q_max[6]);
}

void CVX_Quad::fillb(void) {
  work.b[0] = -params.x_d[0];
  work.b[1] = -params.x_d[1];
  work.b[2] = -params.x_d[2];
  work.b[3] = -params.w_d[0];
  work.b[4] = -params.w_d[1];
  work.b[5] = -params.w_d[2];
}

void CVX_Quad::pre_ops(void) {
}
