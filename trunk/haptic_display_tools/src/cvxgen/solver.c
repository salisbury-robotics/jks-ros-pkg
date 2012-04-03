/* Produced by CVXGEN, 2012-04-02 14:50:07 -0700.  */
/* CVXGEN is Copyright (C) 2006-2011 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2011 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.c. */
/* Description: Main solver file. */

#include "solver.h"

double eval_gap(void) {
  int i;
  double gap;

  gap = 0;
  for (i = 0; i < 11; i++)
    gap += work.z[i]*work.s[i];

  return gap;
}

void set_defaults(void) {
  settings.resid_tol = 1e-6;
  settings.eps = 1e-4;
  settings.max_iters = 25;
  settings.refine_steps = 1;

  settings.s_init = 1;
  settings.z_init = 1;
  settings.debug = 0;
  settings.verbose = 1;
  settings.verbose_refinement = 0;

  settings.better_start = 1;

  settings.kkt_reg = 1e-7;
}

void setup_pointers(void) {
  work.y = work.x + 6;
  work.s = work.x + 6;
  work.z = work.x + 17;

  vars.a = work.x + 0;
  vars.alpha = work.x + 3;
}

void setup_indexed_params(void) {
  /* In CVXGEN, you can say */
  /*   parameters */
  /*     A[i] (5,3), i=1..4 */
  /*   end */
  /* This function sets up A[2] to be a pointer to A_2, which is a length-15 */
  /* vector of doubles. */
  /* If you access parameters that you haven't defined in CVXGEN, the result */
  /* is undefined. */

  params.n[0] = params.n_0;
  params.rxn[0] = params.rxn_0;
  params.n[1] = params.n_1;
  params.rxn[1] = params.rxn_1;
  params.n[2] = params.n_2;
  params.rxn[2] = params.rxn_2;
  params.n[3] = params.n_3;
  params.rxn[3] = params.rxn_3;
  params.n[4] = params.n_4;
  params.rxn[4] = params.rxn_4;
  params.n[5] = params.n_5;
  params.rxn[5] = params.rxn_5;
  params.n[6] = params.n_6;
  params.rxn[6] = params.rxn_6;
  params.n[7] = params.n_7;
  params.rxn[7] = params.rxn_7;
  params.n[8] = params.n_8;
  params.rxn[8] = params.rxn_8;
  params.n[9] = params.n_9;
  params.rxn[9] = params.rxn_9;
  params.n[10] = params.n_10;
  params.rxn[10] = params.rxn_10;
}

void setup_indexing(void) {
  setup_pointers();
  setup_indexed_params();
}

void set_start(void) {
  int i;

  for (i = 0; i < 6; i++)
    work.x[i] = 0;

  for (i = 0; i < 0; i++)
    work.y[i] = 0;

  for (i = 0; i < 11; i++)
    work.s[i] = (work.h[i] > 0) ? work.h[i] : settings.s_init;

  for (i = 0; i < 11; i++)
    work.z[i] = settings.z_init;
}

double eval_objv(void) {
  int i;
  double objv;

  /* Borrow space in work.rhs. */
  multbyP(work.rhs, work.x);

  objv = 0;
  for (i = 0; i < 6; i++)
    objv += work.x[i]*work.rhs[i];
  objv *= 0.5;

  for (i = 0; i < 6; i++)
    objv += work.q[i]*work.x[i];

  objv += 0.5*work.quad_322087960576[0]+0.5*work.quad_669775745024[0];

  return objv;
}

void fillrhs_aff(void) {
  int i;
  double *r1, *r2, *r3, *r4;

  r1 = work.rhs;
  r2 = work.rhs + 6;
  r3 = work.rhs + 17;
  r4 = work.rhs + 28;

  /* r1 = -A^Ty - G^Tz - Px - q. */
  multbymAT(r1, work.y);
  multbymGT(work.buffer, work.z);
  for (i = 0; i < 6; i++)
    r1[i] += work.buffer[i];
  multbyP(work.buffer, work.x);
  for (i = 0; i < 6; i++)
    r1[i] -= work.buffer[i] + work.q[i];

  /* r2 = -z. */
  for (i = 0; i < 11; i++)
    r2[i] = -work.z[i];

  /* r3 = -Gx - s + h. */
  multbymG(r3, work.x);
  for (i = 0; i < 11; i++)
    r3[i] += -work.s[i] + work.h[i];

  /* r4 = -Ax + b. */
  multbymA(r4, work.x);
  for (i = 0; i < 0; i++)
    r4[i] += work.b[i];
}

void fillrhs_cc(void) {
  int i;

  double *r2;
  double *ds_aff, *dz_aff;

  double mu;
  double alpha;
  double sigma;
  double smu;

  double minval;

  r2 = work.rhs + 6;
  ds_aff = work.lhs_aff + 6;
  dz_aff = work.lhs_aff + 17;

  mu = 0;
  for (i = 0; i < 11; i++)
    mu += work.s[i]*work.z[i];
  /* Don't finish calculating mu quite yet. */

  /* Find min(min(ds./s), min(dz./z)). */
  minval = 0;
  for (i = 0; i < 11; i++)
    if (ds_aff[i] < minval*work.s[i])
      minval = ds_aff[i]/work.s[i];
  for (i = 0; i < 11; i++)
    if (dz_aff[i] < minval*work.z[i])
      minval = dz_aff[i]/work.z[i];

  /* Find alpha. */
  if (-1 < minval)
      alpha = 1;
  else
      alpha = -1/minval;

  sigma = 0;
  for (i = 0; i < 11; i++)
    sigma += (work.s[i] + alpha*ds_aff[i])*
      (work.z[i] + alpha*dz_aff[i]);
  sigma /= mu;
  sigma = sigma*sigma*sigma;

  /* Finish calculating mu now. */
  mu *= 0.0909090909090909;

  smu = sigma*mu;

  /* Fill-in the rhs. */
  for (i = 0; i < 6; i++)
    work.rhs[i] = 0;
  for (i = 17; i < 28; i++)
    work.rhs[i] = 0;

  for (i = 0; i < 11; i++)
    r2[i] = work.s_inv[i]*(smu - ds_aff[i]*dz_aff[i]);
}

void refine(double *target, double *var) {
  int i, j;

  double *residual = work.buffer;
  double norm2;
  double *new_var = work.buffer2;
  for (j = 0; j < settings.refine_steps; j++) {
    norm2 = 0;
    matrix_multiply(residual, var);
    for (i = 0; i < 28; i++) {
      residual[i] = residual[i] - target[i];
      norm2 += residual[i]*residual[i];
    }

#ifndef ZERO_LIBRARY_MODE
    if (settings.verbose_refinement) {
      if (j == 0)
        printf("Initial residual before refinement has norm squared %.6g.\n", norm2);
      else
        printf("After refinement we get squared norm %.6g.\n", norm2);
    }
#endif

    /* Solve to find new_var = KKT \ (target - A*var). */
    ldl_solve(residual, new_var);

    /* Update var += new_var, or var += KKT \ (target - A*var). */
    for (i = 0; i < 28; i++) {
      var[i] -= new_var[i];
    }
  }

#ifndef ZERO_LIBRARY_MODE
  if (settings.verbose_refinement) {
    /* Check the residual once more, but only if we're reporting it, since */
    /* it's expensive. */
    norm2 = 0;
    matrix_multiply(residual, var);
    for (i = 0; i < 28; i++) {
      residual[i] = residual[i] - target[i];
      norm2 += residual[i]*residual[i];
    }

    if (j == 0)
      printf("Initial residual before refinement has norm squared %.6g.\n", norm2);
    else
      printf("After refinement we get squared norm %.6g.\n", norm2);
  }
#endif
}

double calc_ineq_resid_squared(void) {
  /* Calculates the norm ||-Gx - s + h||. */
  double norm2_squared;
  int i;

  /* Find -Gx. */
  multbymG(work.buffer, work.x);

  /* Add -s + h. */
  for (i = 0; i < 11; i++)
    work.buffer[i] += -work.s[i] + work.h[i];

  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 11; i++)
    norm2_squared += work.buffer[i]*work.buffer[i];

  return norm2_squared;
}

double calc_eq_resid_squared(void) {
  /* Calculates the norm ||-Ax + b||. */
  double norm2_squared;
  int i;

  /* Find -Ax. */
  multbymA(work.buffer, work.x);

  /* Add +b. */
  for (i = 0; i < 0; i++)
    work.buffer[i] += work.b[i];

  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 0; i++)
    norm2_squared += work.buffer[i]*work.buffer[i];

  return norm2_squared;
}

void better_start(void) {
  /* Calculates a better starting point, using a similar approach to CVXOPT. */
  /* Not yet speed optimized. */
  int i;
  double *x, *s, *z, *y;
  double alpha;

  work.block_33[0] = -1;
  /* Make sure sinvz is 1 to make hijacked KKT system ok. */
  for (i = 0; i < 11; i++)
    work.s_inv_z[i] = 1;
  fill_KKT();
  ldl_factor();

  fillrhs_start();
  /* Borrow work.lhs_aff for the solution. */
  ldl_solve(work.rhs, work.lhs_aff);
  /* Don't do any refinement for now. Precision doesn't matter too much. */

  x = work.lhs_aff;
  s = work.lhs_aff + 6;
  z = work.lhs_aff + 17;
  y = work.lhs_aff + 28;

  /* Just set x and y as is. */
  for (i = 0; i < 6; i++)
    work.x[i] = x[i];

  for (i = 0; i < 0; i++)
    work.y[i] = y[i];

  /* Now complete the initialization. Start with s. */
  /* Must have alpha > max(z). */
  alpha = -1e99;
  for (i = 0; i < 11; i++)
    if (alpha < z[i])
      alpha = z[i];

  if (alpha < 0) {
    for (i = 0; i < 11; i++)
      work.s[i] = -z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 11; i++)
      work.s[i] = -z[i] + alpha;
  }

  /* Now initialize z. */
  /* Now must have alpha > max(-z). */
  alpha = -1e99;
  for (i = 0; i < 11; i++)
    if (alpha < -z[i])
      alpha = -z[i];

  if (alpha < 0) {
    for (i = 0; i < 11; i++)
      work.z[i] = z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 11; i++)
      work.z[i] = z[i] + alpha;
  }
}

void fillrhs_start(void) {
  /* Fill rhs with (-q, 0, h, b). */
  int i;
  double *r1, *r2, *r3, *r4;

  r1 = work.rhs;
  r2 = work.rhs + 6;
  r3 = work.rhs + 17;
  r4 = work.rhs + 28;

  for (i = 0; i < 6; i++)
    r1[i] = -work.q[i];

  for (i = 0; i < 11; i++)
    r2[i] = 0;

  for (i = 0; i < 11; i++)
    r3[i] = work.h[i];

  for (i = 0; i < 0; i++)
    r4[i] = work.b[i];
}

long solve(void) {
  int i;
  int iter;

  double *dx, *ds, *dy, *dz;
  double minval;
  double alpha;
  work.converged = 0;

  setup_pointers();
  pre_ops();

#ifndef ZERO_LIBRARY_MODE
  if (settings.verbose)
    printf("iter     objv        gap       |Ax-b|    |Gx+s-h|    step\n");
#endif

  fillq();
  fillh();
  fillb();

  if (settings.better_start)
    better_start();
  else
    set_start();

  for (iter = 0; iter < settings.max_iters; iter++) {
    for (i = 0; i < 11; i++) {
      work.s_inv[i] = 1.0 / work.s[i];
      work.s_inv_z[i] = work.s_inv[i]*work.z[i];
    }

    work.block_33[0] = 0;
    fill_KKT();
    ldl_factor();
    /* Affine scaling directions. */
    fillrhs_aff();
    ldl_solve(work.rhs, work.lhs_aff);
    refine(work.rhs, work.lhs_aff);
    /* Centering plus corrector directions. */
    fillrhs_cc();
    ldl_solve(work.rhs, work.lhs_cc);
    refine(work.rhs, work.lhs_cc);

    /* Add the two together and store in aff. */
    for (i = 0; i < 28; i++)
      work.lhs_aff[i] += work.lhs_cc[i];

    /* Rename aff to reflect its new meaning. */
    dx = work.lhs_aff;
    ds = work.lhs_aff + 6;
    dz = work.lhs_aff + 17;
    dy = work.lhs_aff + 28;

    /* Find min(min(ds./s), min(dz./z)). */
    minval = 0;
    for (i = 0; i < 11; i++)
      if (ds[i] < minval*work.s[i])
        minval = ds[i]/work.s[i];
    for (i = 0; i < 11; i++)
      if (dz[i] < minval*work.z[i])
        minval = dz[i]/work.z[i];

    /* Find alpha. */
    if (-0.99 < minval)
      alpha = 1;
    else
      alpha = -0.99/minval;

    /* Update the primal and dual variables. */
    for (i = 0; i < 6; i++)
      work.x[i] += alpha*dx[i];
    for (i = 0; i < 11; i++)
      work.s[i] += alpha*ds[i];
    for (i = 0; i < 11; i++)
      work.z[i] += alpha*dz[i];
    for (i = 0; i < 0; i++)
      work.y[i] += alpha*dy[i];

    work.gap = eval_gap();
    work.eq_resid_squared = calc_eq_resid_squared();
    work.ineq_resid_squared = calc_ineq_resid_squared();
#ifndef ZERO_LIBRARY_MODE
    if (settings.verbose) {
      work.optval = eval_objv();
      printf("%3d   %10.3e  %9.2e  %9.2e  %9.2e  % 6.4f\n",
          iter+1, work.optval, work.gap, sqrt(work.eq_resid_squared),
          sqrt(work.ineq_resid_squared), alpha);
    }
#endif

    /* Test termination conditions. Requires optimality, and satisfied */
    /* constraints. */
    if (   (work.gap < settings.eps)
        && (work.eq_resid_squared <= settings.resid_tol*settings.resid_tol)
        && (work.ineq_resid_squared <= settings.resid_tol*settings.resid_tol)
       ) {

      work.converged = 1;
      work.optval = eval_objv();
      return iter+1;
    }
  }

  return iter;
}
