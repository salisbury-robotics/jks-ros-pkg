/* Produced by CVXGEN, 2012-08-24 19:52:41 -0700.  */
/* CVXGEN is Copyright (C) 2006-2011 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2011 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.c. */
/* Description: Main solver file. */

#include "solver.h"

double CVX_Solver1::eval_gap(void) {
  int i;
  double gap;

  gap = 0;
  for (i = 0; i < 65; i++)
    gap += work.z[i]*work.s[i];

  return gap;
}

void CVX_Solver1::set_defaults(void) {
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

void CVX_Solver1::setup_pointers(void) {
  work.y = work.x + 13;
  work.s = work.x + 19;
  work.z = work.x + 84;

  vars.qdd_c = work.x + 0;
}

void CVX_Solver1::setup_indexed_params(void) {
  /* In CVXGEN, you can say */
  /*   parameters */
  /*     A[i] (5,3), i=1..4 */
  /*   end */
  /* This function sets up A[2] to be a pointer to A_2, which is a length-15 */
  /* vector of doubles. */
  /* If you access parameters that you haven't defined in CVXGEN, the result */
  /* is undefined. */

  params.normal[0] = params.normal_0;
  params.Jac[0] = params.Jac_0;
  params.normal[1] = params.normal_1;
  params.Jac[1] = params.Jac_1;
  params.normal[2] = params.normal_2;
  params.Jac[2] = params.Jac_2;
  params.normal[3] = params.normal_3;
  params.Jac[3] = params.Jac_3;
  params.normal[4] = params.normal_4;
  params.Jac[4] = params.Jac_4;
  params.normal[5] = params.normal_5;
  params.Jac[5] = params.Jac_5;
  params.normal[6] = params.normal_6;
  params.Jac[6] = params.Jac_6;
  params.normal[7] = params.normal_7;
  params.Jac[7] = params.Jac_7;
  params.normal[8] = params.normal_8;
  params.Jac[8] = params.Jac_8;
  params.normal[9] = params.normal_9;
  params.Jac[9] = params.Jac_9;
  params.normal[10] = params.normal_10;
  params.Jac[10] = params.Jac_10;
  params.normal[11] = params.normal_11;
  params.Jac[11] = params.Jac_11;
  params.normal[12] = params.normal_12;
  params.Jac[12] = params.Jac_12;
  params.normal[13] = params.normal_13;
  params.Jac[13] = params.Jac_13;
  params.normal[14] = params.normal_14;
  params.Jac[14] = params.Jac_14;
  params.normal[15] = params.normal_15;
  params.Jac[15] = params.Jac_15;
  params.normal[16] = params.normal_16;
  params.Jac[16] = params.Jac_16;
  params.normal[17] = params.normal_17;
  params.Jac[17] = params.Jac_17;
  params.normal[18] = params.normal_18;
  params.Jac[18] = params.Jac_18;
  params.normal[19] = params.normal_19;
  params.Jac[19] = params.Jac_19;
  params.normal[20] = params.normal_20;
  params.Jac[20] = params.Jac_20;
  params.normal[21] = params.normal_21;
  params.Jac[21] = params.Jac_21;
  params.normal[22] = params.normal_22;
  params.Jac[22] = params.Jac_22;
  params.normal[23] = params.normal_23;
  params.Jac[23] = params.Jac_23;
  params.normal[24] = params.normal_24;
  params.Jac[24] = params.Jac_24;
  params.normal[25] = params.normal_25;
  params.Jac[25] = params.Jac_25;
  params.normal[26] = params.normal_26;
  params.Jac[26] = params.Jac_26;
  params.normal[27] = params.normal_27;
  params.Jac[27] = params.Jac_27;
  params.normal[28] = params.normal_28;
  params.Jac[28] = params.Jac_28;
  params.normal[29] = params.normal_29;
  params.Jac[29] = params.Jac_29;
  params.normal[30] = params.normal_30;
  params.Jac[30] = params.Jac_30;
  params.normal[31] = params.normal_31;
  params.Jac[31] = params.Jac_31;
  params.normal[32] = params.normal_32;
  params.Jac[32] = params.Jac_32;
  params.normal[33] = params.normal_33;
  params.Jac[33] = params.Jac_33;
  params.normal[34] = params.normal_34;
  params.Jac[34] = params.Jac_34;
  params.normal[35] = params.normal_35;
  params.Jac[35] = params.Jac_35;
  params.normal[36] = params.normal_36;
  params.Jac[36] = params.Jac_36;
  params.normal[37] = params.normal_37;
  params.Jac[37] = params.Jac_37;
  params.normal[38] = params.normal_38;
  params.Jac[38] = params.Jac_38;
  params.normal[39] = params.normal_39;
  params.Jac[39] = params.Jac_39;
  params.normal[40] = params.normal_40;
  params.Jac[40] = params.Jac_40;
  params.normal[41] = params.normal_41;
  params.Jac[41] = params.Jac_41;
  params.normal[42] = params.normal_42;
  params.Jac[42] = params.Jac_42;
  params.normal[43] = params.normal_43;
  params.Jac[43] = params.Jac_43;
  params.normal[44] = params.normal_44;
  params.Jac[44] = params.Jac_44;
  params.normal[45] = params.normal_45;
  params.Jac[45] = params.Jac_45;
  params.normal[46] = params.normal_46;
  params.Jac[46] = params.Jac_46;
  params.normal[47] = params.normal_47;
  params.Jac[47] = params.Jac_47;
  params.normal[48] = params.normal_48;
  params.Jac[48] = params.Jac_48;
  params.normal[49] = params.normal_49;
  params.Jac[49] = params.Jac_49;
  params.normal[50] = params.normal_50;
  params.Jac[50] = params.Jac_50;
}

void CVX_Solver1::setup_indexing(void) {
  setup_pointers();
  setup_indexed_params();
}

void CVX_Solver1::set_start(void) {
  int i;

  for (i = 0; i < 13; i++)
    work.x[i] = 0;

  for (i = 0; i < 6; i++)
    work.y[i] = 0;

  for (i = 0; i < 65; i++)
    work.s[i] = (work.h[i] > 0) ? work.h[i] : settings.s_init;

  for (i = 0; i < 65; i++)
    work.z[i] = settings.z_init;
}

double CVX_Solver1::eval_objv(void) {
  int i;
  double objv;

  /* Borrow space in work.rhs. */
  multbyP(work.rhs, work.x);

  objv = 0;
  for (i = 0; i < 13; i++)
    objv += work.x[i]*work.rhs[i];
  objv *= 0.5;

  for (i = 0; i < 13; i++)
    objv += work.q[i]*work.x[i];

  objv += 0;

  return objv;
}

void CVX_Solver1::fillrhs_aff(void) {
  int i;
  double *r1, *r2, *r3, *r4;

  r1 = work.rhs;
  r2 = work.rhs + 13;
  r3 = work.rhs + 78;
  r4 = work.rhs + 143;

  /* r1 = -A^Ty - G^Tz - Px - q. */
  multbymAT(r1, work.y);
  multbymGT(work.buffer, work.z);
  for (i = 0; i < 13; i++)
    r1[i] += work.buffer[i];
  multbyP(work.buffer, work.x);
  for (i = 0; i < 13; i++)
    r1[i] -= work.buffer[i] + work.q[i];

  /* r2 = -z. */
  for (i = 0; i < 65; i++)
    r2[i] = -work.z[i];

  /* r3 = -Gx - s + h. */
  multbymG(r3, work.x);
  for (i = 0; i < 65; i++)
    r3[i] += -work.s[i] + work.h[i];

  /* r4 = -Ax + b. */
  multbymA(r4, work.x);
  for (i = 0; i < 6; i++)
    r4[i] += work.b[i];
}

void CVX_Solver1::fillrhs_cc(void) {
  int i;

  double *r2;
  double *ds_aff, *dz_aff;

  double mu;
  double alpha;
  double sigma;
  double smu;

  double minval;

  r2 = work.rhs + 13;
  ds_aff = work.lhs_aff + 13;
  dz_aff = work.lhs_aff + 78;

  mu = 0;
  for (i = 0; i < 65; i++)
    mu += work.s[i]*work.z[i];
  /* Don't finish calculating mu quite yet. */

  /* Find min(min(ds./s), min(dz./z)). */
  minval = 0;
  for (i = 0; i < 65; i++)
    if (ds_aff[i] < minval*work.s[i])
      minval = ds_aff[i]/work.s[i];
  for (i = 0; i < 65; i++)
    if (dz_aff[i] < minval*work.z[i])
      minval = dz_aff[i]/work.z[i];

  /* Find alpha. */
  if (-1 < minval)
      alpha = 1;
  else
      alpha = -1/minval;

  sigma = 0;
  for (i = 0; i < 65; i++)
    sigma += (work.s[i] + alpha*ds_aff[i])*
      (work.z[i] + alpha*dz_aff[i]);
  sigma /= mu;
  sigma = sigma*sigma*sigma;

  /* Finish calculating mu now. */
  mu *= 0.0153846153846154;

  smu = sigma*mu;

  /* Fill-in the rhs. */
  for (i = 0; i < 13; i++)
    work.rhs[i] = 0;
  for (i = 78; i < 149; i++)
    work.rhs[i] = 0;

  for (i = 0; i < 65; i++)
    r2[i] = work.s_inv[i]*(smu - ds_aff[i]*dz_aff[i]);
}

void CVX_Solver1::refine(double *target, double *var) {
  int i, j;

  double *residual = work.buffer;
  double norm2;
  double *new_var = work.buffer2;
  for (j = 0; j < settings.refine_steps; j++) {
    norm2 = 0;
    matrix_multiply(residual, var);
    for (i = 0; i < 149; i++) {
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
    for (i = 0; i < 149; i++) {
      var[i] -= new_var[i];
    }
  }

#ifndef ZERO_LIBRARY_MODE
  if (settings.verbose_refinement) {
    /* Check the residual once more, but only if we're reporting it, since */
    /* it's expensive. */
    norm2 = 0;
    matrix_multiply(residual, var);
    for (i = 0; i < 149; i++) {
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

double CVX_Solver1::calc_ineq_resid_squared(void) {
  /* Calculates the norm ||-Gx - s + h||. */
  double norm2_squared;
  int i;

  /* Find -Gx. */
  multbymG(work.buffer, work.x);

  /* Add -s + h. */
  for (i = 0; i < 65; i++)
    work.buffer[i] += -work.s[i] + work.h[i];

  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 65; i++)
    norm2_squared += work.buffer[i]*work.buffer[i];

  return norm2_squared;
}

double CVX_Solver1::calc_eq_resid_squared(void) {
  /* Calculates the norm ||-Ax + b||. */
  double norm2_squared;
  int i;

  /* Find -Ax. */
  multbymA(work.buffer, work.x);

  /* Add +b. */
  for (i = 0; i < 6; i++)
    work.buffer[i] += work.b[i];

  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 6; i++)
    norm2_squared += work.buffer[i]*work.buffer[i];

  return norm2_squared;
}

void CVX_Solver1::better_start(void) {
  /* Calculates a better starting point, using a similar approach to CVXOPT. */
  /* Not yet speed optimized. */
  int i;
  double *x, *s, *z, *y;
  double alpha;

  work.block_33[0] = -1;
  /* Make sure sinvz is 1 to make hijacked KKT system ok. */
  for (i = 0; i < 65; i++)
    work.s_inv_z[i] = 1;
  fill_KKT();
  ldl_factor();

  fillrhs_start();
  /* Borrow work.lhs_aff for the solution. */
  ldl_solve(work.rhs, work.lhs_aff);
  /* Don't do any refinement for now. Precision doesn't matter too much. */

  x = work.lhs_aff;
  s = work.lhs_aff + 13;
  z = work.lhs_aff + 78;
  y = work.lhs_aff + 143;

  /* Just set x and y as is. */
  for (i = 0; i < 13; i++)
    work.x[i] = x[i];

  for (i = 0; i < 6; i++)
    work.y[i] = y[i];

  /* Now complete the initialization. Start with s. */
  /* Must have alpha > max(z). */
  alpha = -1e99;
  for (i = 0; i < 65; i++)
    if (alpha < z[i])
      alpha = z[i];

  if (alpha < 0) {
    for (i = 0; i < 65; i++)
      work.s[i] = -z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 65; i++)
      work.s[i] = -z[i] + alpha;
  }

  /* Now initialize z. */
  /* Now must have alpha > max(-z). */
  alpha = -1e99;
  for (i = 0; i < 65; i++)
    if (alpha < -z[i])
      alpha = -z[i];

  if (alpha < 0) {
    for (i = 0; i < 65; i++)
      work.z[i] = z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 65; i++)
      work.z[i] = z[i] + alpha;
  }
}

void CVX_Solver1::fillrhs_start(void) {
  /* Fill rhs with (-q, 0, h, b). */
  int i;
  double *r1, *r2, *r3, *r4;

  r1 = work.rhs;
  r2 = work.rhs + 13;
  r3 = work.rhs + 78;
  r4 = work.rhs + 143;

  for (i = 0; i < 13; i++)
    r1[i] = -work.q[i];

  for (i = 0; i < 65; i++)
    r2[i] = 0;

  for (i = 0; i < 65; i++)
    r3[i] = work.h[i];

  for (i = 0; i < 6; i++)
    r4[i] = work.b[i];
}

long CVX_Solver1::solve(void) {
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
    for (i = 0; i < 65; i++) {
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
    for (i = 0; i < 149; i++)
      work.lhs_aff[i] += work.lhs_cc[i];

    /* Rename aff to reflect its new meaning. */
    dx = work.lhs_aff;
    ds = work.lhs_aff + 13;
    dz = work.lhs_aff + 78;
    dy = work.lhs_aff + 143;

    /* Find min(min(ds./s), min(dz./z)). */
    minval = 0;
    for (i = 0; i < 65; i++)
      if (ds[i] < minval*work.s[i])
        minval = ds[i]/work.s[i];
    for (i = 0; i < 65; i++)
      if (dz[i] < minval*work.z[i])
        minval = dz[i]/work.z[i];

    /* Find alpha. */
    if (-0.99 < minval)
      alpha = 1;
    else
      alpha = -0.99/minval;

    /* Update the primal and dual variables. */
    for (i = 0; i < 13; i++)
      work.x[i] += alpha*dx[i];
    for (i = 0; i < 65; i++)
      work.s[i] += alpha*ds[i];
    for (i = 0; i < 65; i++)
      work.z[i] += alpha*dz[i];
    for (i = 0; i < 6; i++)
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
