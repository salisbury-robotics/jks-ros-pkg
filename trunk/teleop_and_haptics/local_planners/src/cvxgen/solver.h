/* Produced by CVXGEN, 2012-08-24 19:52:42 -0700.  */
/* CVXGEN is Copyright (C) 2006-2011 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2011 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.h. */
/* Description: Header file with relevant definitions. */

#ifndef SOLVER_SOLVER1_H
#define SOLVER_SOLVER1_H

/* Uncomment the next line to remove all library dependencies. */
/*#define ZERO_LIBRARY_MODE */

#ifdef MATLAB_MEX_FILE
/* Matlab functions. MATLAB_MEX_FILE will be defined by the mex compiler. */
/* If you are not using the mex compiler, this functionality will not intrude, */
/* as it will be completely disabled at compile-time. */
#include "mex.h"
#else
#ifndef ZERO_LIBRARY_MODE
#include <stdio.h>
#endif
#endif

/* Space must be allocated somewhere (testsolver.c, csolve.c or your own */
/* program) for the global variables vars, params, work and settings. */
/* At the bottom of this file, they are externed. */

#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define pm(A, m, n) printmatrix(#A, A, m, n, 1)
#endif

class CVX_Solver1 {
public:
typedef struct Params_t {
  double xdd[6];
  double J_goal[42];
  double normal_0[3];
  double Jac_0[21];
  double normal_1[3];
  double Jac_1[21];
  double normal_2[3];
  double Jac_2[21];
  double normal_3[3];
  double Jac_3[21];
  double normal_4[3];
  double Jac_4[21];
  double normal_5[3];
  double Jac_5[21];
  double normal_6[3];
  double Jac_6[21];
  double normal_7[3];
  double Jac_7[21];
  double normal_8[3];
  double Jac_8[21];
  double normal_9[3];
  double Jac_9[21];
  double normal_10[3];
  double Jac_10[21];
  double normal_11[3];
  double Jac_11[21];
  double normal_12[3];
  double Jac_12[21];
  double normal_13[3];
  double Jac_13[21];
  double normal_14[3];
  double Jac_14[21];
  double normal_15[3];
  double Jac_15[21];
  double normal_16[3];
  double Jac_16[21];
  double normal_17[3];
  double Jac_17[21];
  double normal_18[3];
  double Jac_18[21];
  double normal_19[3];
  double Jac_19[21];
  double normal_20[3];
  double Jac_20[21];
  double normal_21[3];
  double Jac_21[21];
  double normal_22[3];
  double Jac_22[21];
  double normal_23[3];
  double Jac_23[21];
  double normal_24[3];
  double Jac_24[21];
  double normal_25[3];
  double Jac_25[21];
  double normal_26[3];
  double Jac_26[21];
  double normal_27[3];
  double Jac_27[21];
  double normal_28[3];
  double Jac_28[21];
  double normal_29[3];
  double Jac_29[21];
  double normal_30[3];
  double Jac_30[21];
  double normal_31[3];
  double Jac_31[21];
  double normal_32[3];
  double Jac_32[21];
  double normal_33[3];
  double Jac_33[21];
  double normal_34[3];
  double Jac_34[21];
  double normal_35[3];
  double Jac_35[21];
  double normal_36[3];
  double Jac_36[21];
  double normal_37[3];
  double Jac_37[21];
  double normal_38[3];
  double Jac_38[21];
  double normal_39[3];
  double Jac_39[21];
  double normal_40[3];
  double Jac_40[21];
  double normal_41[3];
  double Jac_41[21];
  double normal_42[3];
  double Jac_42[21];
  double normal_43[3];
  double Jac_43[21];
  double normal_44[3];
  double Jac_44[21];
  double normal_45[3];
  double Jac_45[21];
  double normal_46[3];
  double Jac_46[21];
  double normal_47[3];
  double Jac_47[21];
  double normal_48[3];
  double Jac_48[21];
  double normal_49[3];
  double Jac_49[21];
  double normal_50[3];
  double Jac_50[21];
  double q_min[7];
  double q[7];
  double q_max[7];

  double *normal[51];
  double *Jac[51];
} Params;

typedef struct Vars_t {
  double *t_01; /* 6 rows. */
  double *qdd_c; /* 7 rows. */

} Vars;

typedef struct Workspace_t {
  double h[65];
  double s_inv[65];
  double s_inv_z[65];
  double b[6];
  double q[13];
  double rhs[149];
  double x[149];
  double *s;
  double *z;
  double *y;
  double lhs_aff[149];
  double lhs_cc[149];
  double buffer[149];
  double buffer2[149];

  double KKT[620];
  double L[505];
  double d[149];
  double v[149];
  double d_inv[149];

  double gap;
  double optval;

  double ineq_resid_squared;
  double eq_resid_squared;

  double block_33[1];

  /* Pre-op symbols. */

  int converged;
} Workspace;

typedef struct Settings_t {
  double resid_tol;
  double eps;
  int max_iters;
  int refine_steps;

  int better_start;
  /* Better start obviates the need for s_init and z_init. */
  double s_init;
  double z_init;

  int verbose;
  /* Show extra details of the iterative refinement steps. */
  int verbose_refinement;
  int debug;

  /* For regularization. Minimum value of abs(D_ii) in the kkt D factor. */
  double kkt_reg;
} Settings;

Vars vars;
Params params;
Workspace work;
Settings settings;

/* Function definitions in /home/jem/olsr/releases/20110330074202/lib/olsr.extra/qp_solver/solver.c: */
double eval_gap(void);
void set_defaults(void);
void setup_pointers(void);
void setup_indexing(void);
void set_start(void);
void fillrhs_aff(void);
void fillrhs_cc(void);
void refine(double *target, double *var);
double calc_ineq_resid_squared(void);
double calc_eq_resid_squared(void);
void better_start(void);
void fillrhs_start(void);
long solve(void);

/* Function definitions in /home/jem/olsr/releases/20110330074202/lib/olsr.extra/qp_solver/matrix_support.c: */
void multbymA(double *lhs, double *rhs);
void multbymAT(double *lhs, double *rhs);
void multbymG(double *lhs, double *rhs);
void multbymGT(double *lhs, double *rhs);
void multbyP(double *lhs, double *rhs);
void fillq(void);
void fillh(void);
void fillb(void);
void pre_ops(void);

/* Function definitions in /home/jem/olsr/releases/20110330074202/lib/olsr.extra/qp_solver/ldl.c: */
void ldl_solve(double *target, double *var);
void ldl_factor(void);
double check_factorization(void);
void matrix_multiply(double *result, double *source);
double check_residual(double *target, double *multiplicand);
void fill_KKT(void);

double eval_objv(void);
void setup_indexed_params(void);
void setup_indexed_optvars(void);
};
#endif
