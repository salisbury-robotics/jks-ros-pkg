/* Produced by CVXGEN, 2012-04-02 14:50:07 -0700.  */
/* CVXGEN is Copyright (C) 2006-2011 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2011 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */

#include "solver.h"

void multbymA(double *lhs, double *rhs) {
}

void multbymAT(double *lhs, double *rhs) {
  lhs[0] = 0;
  lhs[1] = 0;
  lhs[2] = 0;
  lhs[3] = 0;
  lhs[4] = 0;
  lhs[5] = 0;
}

void multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-params.n_0[0])-rhs[1]*(-params.n_0[1])-rhs[2]*(-params.n_0[2])-rhs[3]*(-params.rxn_0[0])-rhs[4]*(-params.rxn_0[1])-rhs[5]*(-params.rxn_0[2]);
  lhs[1] = -rhs[0]*(-params.n_1[0])-rhs[1]*(-params.n_1[1])-rhs[2]*(-params.n_1[2])-rhs[3]*(-params.rxn_1[0])-rhs[4]*(-params.rxn_1[1])-rhs[5]*(-params.rxn_1[2]);
  lhs[2] = -rhs[0]*(-params.n_2[0])-rhs[1]*(-params.n_2[1])-rhs[2]*(-params.n_2[2])-rhs[3]*(-params.rxn_2[0])-rhs[4]*(-params.rxn_2[1])-rhs[5]*(-params.rxn_2[2]);
  lhs[3] = -rhs[0]*(-params.n_3[0])-rhs[1]*(-params.n_3[1])-rhs[2]*(-params.n_3[2])-rhs[3]*(-params.rxn_3[0])-rhs[4]*(-params.rxn_3[1])-rhs[5]*(-params.rxn_3[2]);
  lhs[4] = -rhs[0]*(-params.n_4[0])-rhs[1]*(-params.n_4[1])-rhs[2]*(-params.n_4[2])-rhs[3]*(-params.rxn_4[0])-rhs[4]*(-params.rxn_4[1])-rhs[5]*(-params.rxn_4[2]);
  lhs[5] = -rhs[0]*(-params.n_5[0])-rhs[1]*(-params.n_5[1])-rhs[2]*(-params.n_5[2])-rhs[3]*(-params.rxn_5[0])-rhs[4]*(-params.rxn_5[1])-rhs[5]*(-params.rxn_5[2]);
  lhs[6] = -rhs[0]*(-params.n_6[0])-rhs[1]*(-params.n_6[1])-rhs[2]*(-params.n_6[2])-rhs[3]*(-params.rxn_6[0])-rhs[4]*(-params.rxn_6[1])-rhs[5]*(-params.rxn_6[2]);
  lhs[7] = -rhs[0]*(-params.n_7[0])-rhs[1]*(-params.n_7[1])-rhs[2]*(-params.n_7[2])-rhs[3]*(-params.rxn_7[0])-rhs[4]*(-params.rxn_7[1])-rhs[5]*(-params.rxn_7[2]);
  lhs[8] = -rhs[0]*(-params.n_8[0])-rhs[1]*(-params.n_8[1])-rhs[2]*(-params.n_8[2])-rhs[3]*(-params.rxn_8[0])-rhs[4]*(-params.rxn_8[1])-rhs[5]*(-params.rxn_8[2]);
  lhs[9] = -rhs[0]*(-params.n_9[0])-rhs[1]*(-params.n_9[1])-rhs[2]*(-params.n_9[2])-rhs[3]*(-params.rxn_9[0])-rhs[4]*(-params.rxn_9[1])-rhs[5]*(-params.rxn_9[2]);
  lhs[10] = -rhs[0]*(-params.n_10[0])-rhs[1]*(-params.n_10[1])-rhs[2]*(-params.n_10[2])-rhs[3]*(-params.rxn_10[0])-rhs[4]*(-params.rxn_10[1])-rhs[5]*(-params.rxn_10[2]);
}

void multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-params.n_0[0])-rhs[1]*(-params.n_1[0])-rhs[2]*(-params.n_2[0])-rhs[3]*(-params.n_3[0])-rhs[4]*(-params.n_4[0])-rhs[5]*(-params.n_5[0])-rhs[6]*(-params.n_6[0])-rhs[7]*(-params.n_7[0])-rhs[8]*(-params.n_8[0])-rhs[9]*(-params.n_9[0])-rhs[10]*(-params.n_10[0]);
  lhs[1] = -rhs[0]*(-params.n_0[1])-rhs[1]*(-params.n_1[1])-rhs[2]*(-params.n_2[1])-rhs[3]*(-params.n_3[1])-rhs[4]*(-params.n_4[1])-rhs[5]*(-params.n_5[1])-rhs[6]*(-params.n_6[1])-rhs[7]*(-params.n_7[1])-rhs[8]*(-params.n_8[1])-rhs[9]*(-params.n_9[1])-rhs[10]*(-params.n_10[1]);
  lhs[2] = -rhs[0]*(-params.n_0[2])-rhs[1]*(-params.n_1[2])-rhs[2]*(-params.n_2[2])-rhs[3]*(-params.n_3[2])-rhs[4]*(-params.n_4[2])-rhs[5]*(-params.n_5[2])-rhs[6]*(-params.n_6[2])-rhs[7]*(-params.n_7[2])-rhs[8]*(-params.n_8[2])-rhs[9]*(-params.n_9[2])-rhs[10]*(-params.n_10[2]);
  lhs[3] = -rhs[0]*(-params.rxn_0[0])-rhs[1]*(-params.rxn_1[0])-rhs[2]*(-params.rxn_2[0])-rhs[3]*(-params.rxn_3[0])-rhs[4]*(-params.rxn_4[0])-rhs[5]*(-params.rxn_5[0])-rhs[6]*(-params.rxn_6[0])-rhs[7]*(-params.rxn_7[0])-rhs[8]*(-params.rxn_8[0])-rhs[9]*(-params.rxn_9[0])-rhs[10]*(-params.rxn_10[0]);
  lhs[4] = -rhs[0]*(-params.rxn_0[1])-rhs[1]*(-params.rxn_1[1])-rhs[2]*(-params.rxn_2[1])-rhs[3]*(-params.rxn_3[1])-rhs[4]*(-params.rxn_4[1])-rhs[5]*(-params.rxn_5[1])-rhs[6]*(-params.rxn_6[1])-rhs[7]*(-params.rxn_7[1])-rhs[8]*(-params.rxn_8[1])-rhs[9]*(-params.rxn_9[1])-rhs[10]*(-params.rxn_10[1]);
  lhs[5] = -rhs[0]*(-params.rxn_0[2])-rhs[1]*(-params.rxn_1[2])-rhs[2]*(-params.rxn_2[2])-rhs[3]*(-params.rxn_3[2])-rhs[4]*(-params.rxn_4[2])-rhs[5]*(-params.rxn_5[2])-rhs[6]*(-params.rxn_6[2])-rhs[7]*(-params.rxn_7[2])-rhs[8]*(-params.rxn_8[2])-rhs[9]*(-params.rxn_9[2])-rhs[10]*(-params.rxn_10[2]);
}

void multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(params.M[0]);
  lhs[1] = rhs[1]*(params.M[1]);
  lhs[2] = rhs[2]*(params.M[2]);
  lhs[3] = rhs[3]*(params.I[0])+rhs[4]*(params.I[3])+rhs[5]*(params.I[6]);
  lhs[4] = rhs[3]*(params.I[1])+rhs[4]*(params.I[4])+rhs[5]*(params.I[7]);
  lhs[5] = rhs[3]*(params.I[2])+rhs[4]*(params.I[5])+rhs[5]*(params.I[8]);
}

void fillq(void) {
  work.q[0] = -params.M[0]*params.a_u[0];
  work.q[1] = -params.M[1]*params.a_u[1];
  work.q[2] = -params.M[2]*params.a_u[2];
  work.q[3] = -(params.I[0]*params.alpha_u[0]+params.I[3]*params.alpha_u[1]+params.I[6]*params.alpha_u[2]);
  work.q[4] = -(params.I[1]*params.alpha_u[0]+params.I[4]*params.alpha_u[1]+params.I[7]*params.alpha_u[2]);
  work.q[5] = -(params.I[2]*params.alpha_u[0]+params.I[5]*params.alpha_u[1]+params.I[8]*params.alpha_u[2]);
}

void fillh(void) {
  work.h[0] = 0;
  work.h[1] = 0;
  work.h[2] = 0;
  work.h[3] = 0;
  work.h[4] = 0;
  work.h[5] = 0;
  work.h[6] = 0;
  work.h[7] = 0;
  work.h[8] = 0;
  work.h[9] = 0;
  work.h[10] = 0;
}

void fillb(void) {
}

void pre_ops(void) {
  work.quad_322087960576[0] = params.a_u[0]*params.M[0]*params.a_u[0]+params.a_u[1]*params.M[1]*params.a_u[1]+params.a_u[2]*params.M[2]*params.a_u[2];

  work.quad_669775745024[0] = (params.alpha_u[0]*(params.I[0]*params.alpha_u[0]+params.I[3]*params.alpha_u[1]+params.I[6]*params.alpha_u[2])+params.alpha_u[1]*(params.I[1]*params.alpha_u[0]+params.I[4]*params.alpha_u[1]+params.I[7]*params.alpha_u[2])+params.alpha_u[2]*(params.I[2]*params.alpha_u[0]+params.I[5]*params.alpha_u[1]+params.I[8]*params.alpha_u[2]));

}
