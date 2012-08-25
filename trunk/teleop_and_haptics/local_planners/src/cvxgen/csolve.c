/* Produced by CVXGEN, 2012-08-24 19:52:43 -0700.  */
/* CVXGEN is Copyright (C) 2006-2011 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2011 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: csolve.c. */
/* Description: mex-able file for running cvxgen solver. */

#include "mex.h"
#include "solver.h"

Vars vars;
Params params;
Workspace work;
Settings settings;
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  int i, j;
  mxArray *xm, *cell, *xm_cell;
  double *src;
  double *dest;
  double *dest_cell;
  int valid_vars;
  int steps;
  int this_var_errors;
  int warned_diags;
  int prepare_for_c = 0;
  int extra_solves;
  const char *status_names[] = {"optval", "gap", "steps", "converged"};
  mwSize dims1x1of1[1] = {1};
  mwSize dims[1];

  const char *var_names[] = {"qdd_c"};
  const int num_var_names = 1;

  /* Avoid compiler warnings of unused variables by using a dummy assignment. */
  warned_diags = j = 0;
  extra_solves = 0;

  set_defaults();

  /* Check we got the right number of arguments. */
  if (nrhs == 0)
    mexErrMsgTxt("Not enough arguments: You need to specify at least the parameters.\n");

  if (nrhs > 1) {
    /* Assume that the second argument is the settings. */
    if (mxGetField(prhs[1], 0, "eps") != NULL)
      settings.eps = *mxGetPr(mxGetField(prhs[1], 0, "eps"));

    if (mxGetField(prhs[1], 0, "max_iters") != NULL)
      settings.max_iters = *mxGetPr(mxGetField(prhs[1], 0, "max_iters"));

    if (mxGetField(prhs[1], 0, "refine_steps") != NULL)
      settings.refine_steps = *mxGetPr(mxGetField(prhs[1], 0, "refine_steps"));

    if (mxGetField(prhs[1], 0, "verbose") != NULL)
      settings.verbose = *mxGetPr(mxGetField(prhs[1], 0, "verbose"));

    if (mxGetField(prhs[1], 0, "better_start") != NULL)
      settings.better_start = *mxGetPr(mxGetField(prhs[1], 0, "better_start"));

    if (mxGetField(prhs[1], 0, "verbose_refinement") != NULL)
      settings.verbose_refinement = *mxGetPr(mxGetField(prhs[1], 0,
            "verbose_refinement"));

    if (mxGetField(prhs[1], 0, "debug") != NULL)
      settings.debug = *mxGetPr(mxGetField(prhs[1], 0, "debug"));

    if (mxGetField(prhs[1], 0, "kkt_reg") != NULL)
      settings.kkt_reg = *mxGetPr(mxGetField(prhs[1], 0, "kkt_reg"));

    if (mxGetField(prhs[1], 0, "s_init") != NULL)
      settings.s_init = *mxGetPr(mxGetField(prhs[1], 0, "s_init"));

    if (mxGetField(prhs[1], 0, "z_init") != NULL)
      settings.z_init = *mxGetPr(mxGetField(prhs[1], 0, "z_init"));

    if (mxGetField(prhs[1], 0, "resid_tol") != NULL)
      settings.resid_tol = *mxGetPr(mxGetField(prhs[1], 0, "resid_tol"));

    if (mxGetField(prhs[1], 0, "extra_solves") != NULL)
      extra_solves = *mxGetPr(mxGetField(prhs[1], 0, "extra_solves"));
    else
      extra_solves = 0;

    if (mxGetField(prhs[1], 0, "prepare_for_c") != NULL)
      prepare_for_c = *mxGetPr(mxGetField(prhs[1], 0, "prepare_for_c"));
  }

  valid_vars = 0;

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "J_goal");

  if (xm == NULL) {
    printf("could not find params.J_goal.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 7))) {
      printf("J_goal must be size (6,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter J_goal must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter J_goal must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter J_goal must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.J_goal;
      src = mxGetPr(xm);

      for (i = 0; i < 42; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_0");

  if (xm == NULL) {
    printf("could not find params.Jac_0.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_0 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_0 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_0 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_0 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_0;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_1");

  if (xm == NULL) {
    /* Attempt to pull Jac_1 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 0);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_1 or params.Jac{1}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_1 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_1 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_1 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_1;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_2");

  if (xm == NULL) {
    /* Attempt to pull Jac_2 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 1);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_2 or params.Jac{2}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_2 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_2 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_2 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_2;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_3");

  if (xm == NULL) {
    /* Attempt to pull Jac_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_3 or params.Jac{3}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_3 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_3 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_3 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_3;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_4");

  if (xm == NULL) {
    /* Attempt to pull Jac_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_4 or params.Jac{4}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_4 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_4 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_4 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_4;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_5");

  if (xm == NULL) {
    /* Attempt to pull Jac_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_5 or params.Jac{5}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_5 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_5 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_5 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_5;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_6");

  if (xm == NULL) {
    /* Attempt to pull Jac_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_6 or params.Jac{6}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_6 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_6 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_6 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_6;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_7");

  if (xm == NULL) {
    /* Attempt to pull Jac_7 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 6);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_7 or params.Jac{7}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_7 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_7 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_7 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_7;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_8");

  if (xm == NULL) {
    /* Attempt to pull Jac_8 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 7);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_8 or params.Jac{8}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_8 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_8 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_8 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_8 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_8;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_9");

  if (xm == NULL) {
    /* Attempt to pull Jac_9 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 8);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_9 or params.Jac{9}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_9 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_9 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_9 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_9 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_9;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_10");

  if (xm == NULL) {
    /* Attempt to pull Jac_10 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 9);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_10 or params.Jac{10}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_10 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_10 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_10 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_10 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_10;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_11");

  if (xm == NULL) {
    /* Attempt to pull Jac_11 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 10);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_11 or params.Jac{11}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_11 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_11 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_11 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_11 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_11;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_12");

  if (xm == NULL) {
    /* Attempt to pull Jac_12 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 11);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_12 or params.Jac{12}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_12 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_12 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_12 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_12 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_12;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_13");

  if (xm == NULL) {
    /* Attempt to pull Jac_13 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 12);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_13 or params.Jac{13}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_13 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_13 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_13 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_13 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_13;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_14");

  if (xm == NULL) {
    /* Attempt to pull Jac_14 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 13);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_14 or params.Jac{14}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_14 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_14 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_14 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_14 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_14;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_15");

  if (xm == NULL) {
    /* Attempt to pull Jac_15 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 14);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_15 or params.Jac{15}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_15 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_15 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_15 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_15 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_15;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_16");

  if (xm == NULL) {
    /* Attempt to pull Jac_16 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 15);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_16 or params.Jac{16}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_16 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_16 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_16 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_16 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_16;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_17");

  if (xm == NULL) {
    /* Attempt to pull Jac_17 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 16);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_17 or params.Jac{17}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_17 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_17 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_17 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_17 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_17;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_18");

  if (xm == NULL) {
    /* Attempt to pull Jac_18 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 17);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_18 or params.Jac{18}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_18 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_18 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_18 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_18 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_18;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_19");

  if (xm == NULL) {
    /* Attempt to pull Jac_19 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 18);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_19 or params.Jac{19}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_19 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_19 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_19 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_19 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_19;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_20");

  if (xm == NULL) {
    /* Attempt to pull Jac_20 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 19);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_20 or params.Jac{20}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_20 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_20 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_20 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_20 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_20;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_21");

  if (xm == NULL) {
    /* Attempt to pull Jac_21 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 20);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_21 or params.Jac{21}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_21 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_21 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_21 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_21 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_21;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_22");

  if (xm == NULL) {
    /* Attempt to pull Jac_22 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 21);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_22 or params.Jac{22}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_22 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_22 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_22 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_22 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_22;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_23");

  if (xm == NULL) {
    /* Attempt to pull Jac_23 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 22);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_23 or params.Jac{23}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_23 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_23 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_23 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_23 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_23;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_24");

  if (xm == NULL) {
    /* Attempt to pull Jac_24 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 23);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_24 or params.Jac{24}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_24 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_24 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_24 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_24 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_24;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_25");

  if (xm == NULL) {
    /* Attempt to pull Jac_25 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 24);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_25 or params.Jac{25}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_25 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_25 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_25 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_25 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_25;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_26");

  if (xm == NULL) {
    /* Attempt to pull Jac_26 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 25);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_26 or params.Jac{26}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_26 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_26 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_26 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_26 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_26;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_27");

  if (xm == NULL) {
    /* Attempt to pull Jac_27 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 26);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_27 or params.Jac{27}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_27 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_27 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_27 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_27 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_27;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_28");

  if (xm == NULL) {
    /* Attempt to pull Jac_28 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 27);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_28 or params.Jac{28}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_28 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_28 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_28 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_28 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_28;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_29");

  if (xm == NULL) {
    /* Attempt to pull Jac_29 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 28);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_29 or params.Jac{29}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_29 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_29 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_29 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_29 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_29;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_30");

  if (xm == NULL) {
    /* Attempt to pull Jac_30 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 29);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_30 or params.Jac{30}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_30 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_30 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_30 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_30 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_30;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_31");

  if (xm == NULL) {
    /* Attempt to pull Jac_31 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 30);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_31 or params.Jac{31}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_31 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_31 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_31 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_31 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_31;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_32");

  if (xm == NULL) {
    /* Attempt to pull Jac_32 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 31);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_32 or params.Jac{32}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_32 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_32 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_32 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_32 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_32;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_33");

  if (xm == NULL) {
    /* Attempt to pull Jac_33 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 32);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_33 or params.Jac{33}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_33 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_33 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_33 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_33 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_33;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_34");

  if (xm == NULL) {
    /* Attempt to pull Jac_34 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 33);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_34 or params.Jac{34}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_34 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_34 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_34 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_34 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_34;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_35");

  if (xm == NULL) {
    /* Attempt to pull Jac_35 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 34);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_35 or params.Jac{35}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_35 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_35 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_35 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_35 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_35;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_36");

  if (xm == NULL) {
    /* Attempt to pull Jac_36 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 35);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_36 or params.Jac{36}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_36 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_36 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_36 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_36 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_36;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_37");

  if (xm == NULL) {
    /* Attempt to pull Jac_37 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 36);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_37 or params.Jac{37}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_37 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_37 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_37 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_37 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_37;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_38");

  if (xm == NULL) {
    /* Attempt to pull Jac_38 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 37);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_38 or params.Jac{38}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_38 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_38 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_38 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_38 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_38;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_39");

  if (xm == NULL) {
    /* Attempt to pull Jac_39 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 38);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_39 or params.Jac{39}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_39 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_39 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_39 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_39 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_39;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_40");

  if (xm == NULL) {
    /* Attempt to pull Jac_40 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 39);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_40 or params.Jac{40}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_40 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_40 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_40 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_40 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_40;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_41");

  if (xm == NULL) {
    /* Attempt to pull Jac_41 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 40);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_41 or params.Jac{41}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_41 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_41 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_41 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_41 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_41;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_42");

  if (xm == NULL) {
    /* Attempt to pull Jac_42 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 41);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_42 or params.Jac{42}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_42 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_42 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_42 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_42 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_42;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_43");

  if (xm == NULL) {
    /* Attempt to pull Jac_43 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 42);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_43 or params.Jac{43}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_43 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_43 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_43 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_43 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_43;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_44");

  if (xm == NULL) {
    /* Attempt to pull Jac_44 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 43);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_44 or params.Jac{44}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_44 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_44 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_44 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_44 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_44;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_45");

  if (xm == NULL) {
    /* Attempt to pull Jac_45 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 44);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_45 or params.Jac{45}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_45 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_45 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_45 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_45 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_45;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_46");

  if (xm == NULL) {
    /* Attempt to pull Jac_46 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 45);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_46 or params.Jac{46}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_46 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_46 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_46 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_46 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_46;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_47");

  if (xm == NULL) {
    /* Attempt to pull Jac_47 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 46);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_47 or params.Jac{47}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_47 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_47 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_47 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_47 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_47;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_48");

  if (xm == NULL) {
    /* Attempt to pull Jac_48 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 47);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_48 or params.Jac{48}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_48 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_48 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_48 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_48 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_48;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_49");

  if (xm == NULL) {
    /* Attempt to pull Jac_49 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 48);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_49 or params.Jac{49}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_49 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_49 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_49 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_49 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_49;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "Jac_50");

  if (xm == NULL) {
    /* Attempt to pull Jac_50 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jac");
    if (cell != NULL)
      xm = mxGetCell(cell, 49);
  }

  if (xm == NULL) {
    printf("could not find params.Jac_50 or params.Jac{50}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 7))) {
      printf("Jac_50 must be size (3,7), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter Jac_50 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter Jac_50 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter Jac_50 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.Jac_50;
      src = mxGetPr(xm);

      for (i = 0; i < 21; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_0");

  if (xm == NULL) {
    printf("could not find params.normal_0.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_0 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_0 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_0 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_0 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_0;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_1");

  if (xm == NULL) {
    /* Attempt to pull normal_1 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 0);
  }

  if (xm == NULL) {
    printf("could not find params.normal_1 or params.normal{1}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_1 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_1 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_1 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_1;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_2");

  if (xm == NULL) {
    /* Attempt to pull normal_2 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 1);
  }

  if (xm == NULL) {
    printf("could not find params.normal_2 or params.normal{2}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_2 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_2 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_2 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_2;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_3");

  if (xm == NULL) {
    /* Attempt to pull normal_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }

  if (xm == NULL) {
    printf("could not find params.normal_3 or params.normal{3}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_3 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_3 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_3 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_3;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_4");

  if (xm == NULL) {
    /* Attempt to pull normal_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }

  if (xm == NULL) {
    printf("could not find params.normal_4 or params.normal{4}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_4 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_4 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_4 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_4;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_5");

  if (xm == NULL) {
    /* Attempt to pull normal_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }

  if (xm == NULL) {
    printf("could not find params.normal_5 or params.normal{5}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_5 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_5 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_5 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_5;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_6");

  if (xm == NULL) {
    /* Attempt to pull normal_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }

  if (xm == NULL) {
    printf("could not find params.normal_6 or params.normal{6}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_6 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_6 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_6 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_6;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_7");

  if (xm == NULL) {
    /* Attempt to pull normal_7 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 6);
  }

  if (xm == NULL) {
    printf("could not find params.normal_7 or params.normal{7}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_7 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_7 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_7 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_7;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_8");

  if (xm == NULL) {
    /* Attempt to pull normal_8 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 7);
  }

  if (xm == NULL) {
    printf("could not find params.normal_8 or params.normal{8}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_8 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_8 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_8 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_8 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_8;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_9");

  if (xm == NULL) {
    /* Attempt to pull normal_9 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 8);
  }

  if (xm == NULL) {
    printf("could not find params.normal_9 or params.normal{9}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_9 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_9 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_9 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_9 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_9;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_10");

  if (xm == NULL) {
    /* Attempt to pull normal_10 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 9);
  }

  if (xm == NULL) {
    printf("could not find params.normal_10 or params.normal{10}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_10 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_10 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_10 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_10 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_10;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_11");

  if (xm == NULL) {
    /* Attempt to pull normal_11 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 10);
  }

  if (xm == NULL) {
    printf("could not find params.normal_11 or params.normal{11}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_11 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_11 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_11 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_11 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_11;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_12");

  if (xm == NULL) {
    /* Attempt to pull normal_12 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 11);
  }

  if (xm == NULL) {
    printf("could not find params.normal_12 or params.normal{12}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_12 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_12 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_12 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_12 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_12;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_13");

  if (xm == NULL) {
    /* Attempt to pull normal_13 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 12);
  }

  if (xm == NULL) {
    printf("could not find params.normal_13 or params.normal{13}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_13 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_13 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_13 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_13 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_13;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_14");

  if (xm == NULL) {
    /* Attempt to pull normal_14 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 13);
  }

  if (xm == NULL) {
    printf("could not find params.normal_14 or params.normal{14}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_14 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_14 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_14 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_14 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_14;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_15");

  if (xm == NULL) {
    /* Attempt to pull normal_15 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 14);
  }

  if (xm == NULL) {
    printf("could not find params.normal_15 or params.normal{15}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_15 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_15 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_15 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_15 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_15;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_16");

  if (xm == NULL) {
    /* Attempt to pull normal_16 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 15);
  }

  if (xm == NULL) {
    printf("could not find params.normal_16 or params.normal{16}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_16 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_16 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_16 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_16 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_16;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_17");

  if (xm == NULL) {
    /* Attempt to pull normal_17 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 16);
  }

  if (xm == NULL) {
    printf("could not find params.normal_17 or params.normal{17}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_17 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_17 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_17 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_17 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_17;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_18");

  if (xm == NULL) {
    /* Attempt to pull normal_18 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 17);
  }

  if (xm == NULL) {
    printf("could not find params.normal_18 or params.normal{18}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_18 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_18 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_18 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_18 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_18;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_19");

  if (xm == NULL) {
    /* Attempt to pull normal_19 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 18);
  }

  if (xm == NULL) {
    printf("could not find params.normal_19 or params.normal{19}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_19 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_19 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_19 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_19 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_19;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_20");

  if (xm == NULL) {
    /* Attempt to pull normal_20 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 19);
  }

  if (xm == NULL) {
    printf("could not find params.normal_20 or params.normal{20}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_20 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_20 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_20 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_20 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_20;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_21");

  if (xm == NULL) {
    /* Attempt to pull normal_21 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 20);
  }

  if (xm == NULL) {
    printf("could not find params.normal_21 or params.normal{21}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_21 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_21 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_21 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_21 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_21;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_22");

  if (xm == NULL) {
    /* Attempt to pull normal_22 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 21);
  }

  if (xm == NULL) {
    printf("could not find params.normal_22 or params.normal{22}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_22 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_22 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_22 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_22 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_22;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_23");

  if (xm == NULL) {
    /* Attempt to pull normal_23 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 22);
  }

  if (xm == NULL) {
    printf("could not find params.normal_23 or params.normal{23}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_23 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_23 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_23 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_23 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_23;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_24");

  if (xm == NULL) {
    /* Attempt to pull normal_24 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 23);
  }

  if (xm == NULL) {
    printf("could not find params.normal_24 or params.normal{24}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_24 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_24 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_24 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_24 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_24;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_25");

  if (xm == NULL) {
    /* Attempt to pull normal_25 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 24);
  }

  if (xm == NULL) {
    printf("could not find params.normal_25 or params.normal{25}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_25 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_25 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_25 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_25 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_25;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_26");

  if (xm == NULL) {
    /* Attempt to pull normal_26 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 25);
  }

  if (xm == NULL) {
    printf("could not find params.normal_26 or params.normal{26}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_26 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_26 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_26 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_26 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_26;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_27");

  if (xm == NULL) {
    /* Attempt to pull normal_27 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 26);
  }

  if (xm == NULL) {
    printf("could not find params.normal_27 or params.normal{27}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_27 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_27 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_27 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_27 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_27;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_28");

  if (xm == NULL) {
    /* Attempt to pull normal_28 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 27);
  }

  if (xm == NULL) {
    printf("could not find params.normal_28 or params.normal{28}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_28 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_28 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_28 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_28 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_28;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_29");

  if (xm == NULL) {
    /* Attempt to pull normal_29 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 28);
  }

  if (xm == NULL) {
    printf("could not find params.normal_29 or params.normal{29}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_29 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_29 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_29 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_29 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_29;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_30");

  if (xm == NULL) {
    /* Attempt to pull normal_30 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 29);
  }

  if (xm == NULL) {
    printf("could not find params.normal_30 or params.normal{30}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_30 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_30 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_30 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_30 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_30;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_31");

  if (xm == NULL) {
    /* Attempt to pull normal_31 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 30);
  }

  if (xm == NULL) {
    printf("could not find params.normal_31 or params.normal{31}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_31 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_31 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_31 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_31 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_31;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_32");

  if (xm == NULL) {
    /* Attempt to pull normal_32 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 31);
  }

  if (xm == NULL) {
    printf("could not find params.normal_32 or params.normal{32}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_32 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_32 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_32 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_32 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_32;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_33");

  if (xm == NULL) {
    /* Attempt to pull normal_33 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 32);
  }

  if (xm == NULL) {
    printf("could not find params.normal_33 or params.normal{33}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_33 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_33 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_33 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_33 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_33;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_34");

  if (xm == NULL) {
    /* Attempt to pull normal_34 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 33);
  }

  if (xm == NULL) {
    printf("could not find params.normal_34 or params.normal{34}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_34 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_34 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_34 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_34 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_34;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_35");

  if (xm == NULL) {
    /* Attempt to pull normal_35 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 34);
  }

  if (xm == NULL) {
    printf("could not find params.normal_35 or params.normal{35}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_35 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_35 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_35 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_35 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_35;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_36");

  if (xm == NULL) {
    /* Attempt to pull normal_36 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 35);
  }

  if (xm == NULL) {
    printf("could not find params.normal_36 or params.normal{36}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_36 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_36 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_36 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_36 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_36;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_37");

  if (xm == NULL) {
    /* Attempt to pull normal_37 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 36);
  }

  if (xm == NULL) {
    printf("could not find params.normal_37 or params.normal{37}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_37 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_37 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_37 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_37 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_37;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_38");

  if (xm == NULL) {
    /* Attempt to pull normal_38 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 37);
  }

  if (xm == NULL) {
    printf("could not find params.normal_38 or params.normal{38}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_38 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_38 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_38 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_38 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_38;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_39");

  if (xm == NULL) {
    /* Attempt to pull normal_39 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 38);
  }

  if (xm == NULL) {
    printf("could not find params.normal_39 or params.normal{39}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_39 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_39 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_39 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_39 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_39;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_40");

  if (xm == NULL) {
    /* Attempt to pull normal_40 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 39);
  }

  if (xm == NULL) {
    printf("could not find params.normal_40 or params.normal{40}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_40 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_40 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_40 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_40 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_40;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_41");

  if (xm == NULL) {
    /* Attempt to pull normal_41 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 40);
  }

  if (xm == NULL) {
    printf("could not find params.normal_41 or params.normal{41}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_41 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_41 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_41 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_41 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_41;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_42");

  if (xm == NULL) {
    /* Attempt to pull normal_42 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 41);
  }

  if (xm == NULL) {
    printf("could not find params.normal_42 or params.normal{42}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_42 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_42 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_42 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_42 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_42;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_43");

  if (xm == NULL) {
    /* Attempt to pull normal_43 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 42);
  }

  if (xm == NULL) {
    printf("could not find params.normal_43 or params.normal{43}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_43 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_43 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_43 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_43 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_43;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_44");

  if (xm == NULL) {
    /* Attempt to pull normal_44 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 43);
  }

  if (xm == NULL) {
    printf("could not find params.normal_44 or params.normal{44}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_44 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_44 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_44 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_44 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_44;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_45");

  if (xm == NULL) {
    /* Attempt to pull normal_45 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 44);
  }

  if (xm == NULL) {
    printf("could not find params.normal_45 or params.normal{45}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_45 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_45 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_45 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_45 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_45;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_46");

  if (xm == NULL) {
    /* Attempt to pull normal_46 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 45);
  }

  if (xm == NULL) {
    printf("could not find params.normal_46 or params.normal{46}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_46 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_46 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_46 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_46 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_46;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_47");

  if (xm == NULL) {
    /* Attempt to pull normal_47 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 46);
  }

  if (xm == NULL) {
    printf("could not find params.normal_47 or params.normal{47}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_47 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_47 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_47 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_47 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_47;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_48");

  if (xm == NULL) {
    /* Attempt to pull normal_48 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 47);
  }

  if (xm == NULL) {
    printf("could not find params.normal_48 or params.normal{48}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_48 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_48 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_48 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_48 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_48;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_49");

  if (xm == NULL) {
    /* Attempt to pull normal_49 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 48);
  }

  if (xm == NULL) {
    printf("could not find params.normal_49 or params.normal{49}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_49 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_49 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_49 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_49 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_49;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "normal_50");

  if (xm == NULL) {
    /* Attempt to pull normal_50 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "normal");
    if (cell != NULL)
      xm = mxGetCell(cell, 49);
  }

  if (xm == NULL) {
    printf("could not find params.normal_50 or params.normal{50}.\n");
  } else {
    if (!((mxGetM(xm) == 3) && (mxGetN(xm) == 1))) {
      printf("normal_50 must be size (3,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter normal_50 must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter normal_50 must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter normal_50 must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.normal_50;
      src = mxGetPr(xm);

      for (i = 0; i < 3; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "q");

  if (xm == NULL) {
    printf("could not find params.q.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 1))) {
      printf("q must be size (7,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter q must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter q must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter q must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.q;
      src = mxGetPr(xm);

      for (i = 0; i < 7; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "q_max");

  if (xm == NULL) {
    printf("could not find params.q_max.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 1))) {
      printf("q_max must be size (7,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter q_max must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter q_max must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter q_max must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.q_max;
      src = mxGetPr(xm);

      for (i = 0; i < 7; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "q_min");

  if (xm == NULL) {
    printf("could not find params.q_min.\n");
  } else {
    if (!((mxGetM(xm) == 7) && (mxGetN(xm) == 1))) {
      printf("q_min must be size (7,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter q_min must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter q_min must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter q_min must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.q_min;
      src = mxGetPr(xm);

      for (i = 0; i < 7; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  this_var_errors = 0;

  xm = mxGetField(prhs[0], 0, "xdd");

  if (xm == NULL) {
    printf("could not find params.xdd.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("xdd must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }

    if (mxIsComplex(xm)) {
      printf("parameter xdd must be real.\n");
      this_var_errors++;
    }

    if (!mxIsClass(xm, "double")) {
      printf("parameter xdd must be a full matrix of doubles.\n");
      this_var_errors++;
    }

    if (mxIsSparse(xm)) {
      printf("parameter xdd must be a full matrix.\n");
      this_var_errors++;
    }

    if (this_var_errors == 0) {
      dest = params.xdd;
      src = mxGetPr(xm);

      for (i = 0; i < 6; i++)
        *dest++ = *src++;

      valid_vars++;
    }
  }

  if (valid_vars != 107) {
    printf("Error: %d parameters are invalid.\n", 107 - valid_vars);
    mexErrMsgTxt("invalid parameters found.");
  }

  if (prepare_for_c) {
    printf("settings.prepare_for_c == 1. thus, outputting for C.\n");
    for (i = 0; i < 6; i++)
      printf("  params.xdd[%d] = %.6g;\n", i, params.xdd[i]);
    for (i = 0; i < 42; i++)
      printf("  params.J_goal[%d] = %.6g;\n", i, params.J_goal[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_0[%d] = %.6g;\n", i, params.normal_0[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_0[%d] = %.6g;\n", i, params.Jac_0[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_1[%d] = %.6g;\n", i, params.normal_1[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_1[%d] = %.6g;\n", i, params.Jac_1[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_2[%d] = %.6g;\n", i, params.normal_2[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_2[%d] = %.6g;\n", i, params.Jac_2[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_3[%d] = %.6g;\n", i, params.normal_3[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_3[%d] = %.6g;\n", i, params.Jac_3[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_4[%d] = %.6g;\n", i, params.normal_4[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_4[%d] = %.6g;\n", i, params.Jac_4[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_5[%d] = %.6g;\n", i, params.normal_5[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_5[%d] = %.6g;\n", i, params.Jac_5[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_6[%d] = %.6g;\n", i, params.normal_6[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_6[%d] = %.6g;\n", i, params.Jac_6[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_7[%d] = %.6g;\n", i, params.normal_7[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_7[%d] = %.6g;\n", i, params.Jac_7[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_8[%d] = %.6g;\n", i, params.normal_8[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_8[%d] = %.6g;\n", i, params.Jac_8[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_9[%d] = %.6g;\n", i, params.normal_9[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_9[%d] = %.6g;\n", i, params.Jac_9[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_10[%d] = %.6g;\n", i, params.normal_10[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_10[%d] = %.6g;\n", i, params.Jac_10[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_11[%d] = %.6g;\n", i, params.normal_11[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_11[%d] = %.6g;\n", i, params.Jac_11[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_12[%d] = %.6g;\n", i, params.normal_12[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_12[%d] = %.6g;\n", i, params.Jac_12[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_13[%d] = %.6g;\n", i, params.normal_13[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_13[%d] = %.6g;\n", i, params.Jac_13[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_14[%d] = %.6g;\n", i, params.normal_14[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_14[%d] = %.6g;\n", i, params.Jac_14[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_15[%d] = %.6g;\n", i, params.normal_15[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_15[%d] = %.6g;\n", i, params.Jac_15[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_16[%d] = %.6g;\n", i, params.normal_16[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_16[%d] = %.6g;\n", i, params.Jac_16[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_17[%d] = %.6g;\n", i, params.normal_17[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_17[%d] = %.6g;\n", i, params.Jac_17[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_18[%d] = %.6g;\n", i, params.normal_18[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_18[%d] = %.6g;\n", i, params.Jac_18[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_19[%d] = %.6g;\n", i, params.normal_19[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_19[%d] = %.6g;\n", i, params.Jac_19[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_20[%d] = %.6g;\n", i, params.normal_20[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_20[%d] = %.6g;\n", i, params.Jac_20[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_21[%d] = %.6g;\n", i, params.normal_21[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_21[%d] = %.6g;\n", i, params.Jac_21[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_22[%d] = %.6g;\n", i, params.normal_22[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_22[%d] = %.6g;\n", i, params.Jac_22[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_23[%d] = %.6g;\n", i, params.normal_23[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_23[%d] = %.6g;\n", i, params.Jac_23[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_24[%d] = %.6g;\n", i, params.normal_24[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_24[%d] = %.6g;\n", i, params.Jac_24[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_25[%d] = %.6g;\n", i, params.normal_25[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_25[%d] = %.6g;\n", i, params.Jac_25[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_26[%d] = %.6g;\n", i, params.normal_26[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_26[%d] = %.6g;\n", i, params.Jac_26[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_27[%d] = %.6g;\n", i, params.normal_27[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_27[%d] = %.6g;\n", i, params.Jac_27[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_28[%d] = %.6g;\n", i, params.normal_28[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_28[%d] = %.6g;\n", i, params.Jac_28[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_29[%d] = %.6g;\n", i, params.normal_29[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_29[%d] = %.6g;\n", i, params.Jac_29[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_30[%d] = %.6g;\n", i, params.normal_30[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_30[%d] = %.6g;\n", i, params.Jac_30[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_31[%d] = %.6g;\n", i, params.normal_31[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_31[%d] = %.6g;\n", i, params.Jac_31[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_32[%d] = %.6g;\n", i, params.normal_32[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_32[%d] = %.6g;\n", i, params.Jac_32[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_33[%d] = %.6g;\n", i, params.normal_33[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_33[%d] = %.6g;\n", i, params.Jac_33[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_34[%d] = %.6g;\n", i, params.normal_34[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_34[%d] = %.6g;\n", i, params.Jac_34[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_35[%d] = %.6g;\n", i, params.normal_35[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_35[%d] = %.6g;\n", i, params.Jac_35[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_36[%d] = %.6g;\n", i, params.normal_36[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_36[%d] = %.6g;\n", i, params.Jac_36[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_37[%d] = %.6g;\n", i, params.normal_37[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_37[%d] = %.6g;\n", i, params.Jac_37[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_38[%d] = %.6g;\n", i, params.normal_38[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_38[%d] = %.6g;\n", i, params.Jac_38[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_39[%d] = %.6g;\n", i, params.normal_39[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_39[%d] = %.6g;\n", i, params.Jac_39[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_40[%d] = %.6g;\n", i, params.normal_40[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_40[%d] = %.6g;\n", i, params.Jac_40[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_41[%d] = %.6g;\n", i, params.normal_41[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_41[%d] = %.6g;\n", i, params.Jac_41[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_42[%d] = %.6g;\n", i, params.normal_42[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_42[%d] = %.6g;\n", i, params.Jac_42[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_43[%d] = %.6g;\n", i, params.normal_43[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_43[%d] = %.6g;\n", i, params.Jac_43[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_44[%d] = %.6g;\n", i, params.normal_44[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_44[%d] = %.6g;\n", i, params.Jac_44[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_45[%d] = %.6g;\n", i, params.normal_45[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_45[%d] = %.6g;\n", i, params.Jac_45[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_46[%d] = %.6g;\n", i, params.normal_46[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_46[%d] = %.6g;\n", i, params.Jac_46[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_47[%d] = %.6g;\n", i, params.normal_47[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_47[%d] = %.6g;\n", i, params.Jac_47[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_48[%d] = %.6g;\n", i, params.normal_48[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_48[%d] = %.6g;\n", i, params.Jac_48[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_49[%d] = %.6g;\n", i, params.normal_49[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_49[%d] = %.6g;\n", i, params.Jac_49[i]);
    for (i = 0; i < 3; i++)
      printf("  params.normal_50[%d] = %.6g;\n", i, params.normal_50[i]);
    for (i = 0; i < 21; i++)
      printf("  params.Jac_50[%d] = %.6g;\n", i, params.Jac_50[i]);
    for (i = 0; i < 7; i++)
      printf("  params.q_min[%d] = %.6g;\n", i, params.q_min[i]);
    for (i = 0; i < 7; i++)
      printf("  params.q[%d] = %.6g;\n", i, params.q[i]);
    for (i = 0; i < 7; i++)
      printf("  params.q_max[%d] = %.6g;\n", i, params.q_max[i]);
  }

  /* Perform the actual solve in here. */
  steps = solve();

  /* For profiling purposes, allow extra silent solves if desired. */
  settings.verbose = 0;
  for (i = 0; i < extra_solves; i++)
    solve();

  /* Update the status variables. */
  plhs[1] = mxCreateStructArray(1, dims1x1of1, 4, status_names);

  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "optval", xm);
  *mxGetPr(xm) = work.optval;

  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "gap", xm);
  *mxGetPr(xm) = work.gap;

  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "steps", xm);
  *mxGetPr(xm) = steps;

  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "converged", xm);
  *mxGetPr(xm) = work.converged;

  /* Extract variable values. */
  plhs[0] = mxCreateStructArray(1, dims1x1of1, num_var_names, var_names);

  xm = mxCreateDoubleMatrix(7, 1, mxREAL);
  mxSetField(plhs[0], 0, "qdd_c", xm);
  dest = mxGetPr(xm);
  src = vars.qdd_c;
  for (i = 0; i < 7; i++) {
    *dest++ = *src++;
  }
}
