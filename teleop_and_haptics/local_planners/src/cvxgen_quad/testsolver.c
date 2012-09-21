/* Produced by CVXGEN, 2012-09-18 18:21:14 -0700.  */
/* CVXGEN is Copyright (C) 2006-2011 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2011 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */

#include "solver.h"

Vars vars;
Params params;
Workspace work;
Settings settings;
#define NUMTESTS 0

int main(int argc, char **argv) {
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif

  set_defaults();
  setup_indexing();
  load_default_data();

  /* Solve problem instance for the record. */
  settings.verbose = 1;
  num_iters = solve();

#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  settings.verbose = 0;

  tic();
  for (i = 0; i < NUMTESTS; i++) {
    solve();
  }
  time = tocq();

  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
  time_per = time / NUMTESTS;

  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
#endif
#endif

  return 0;
}

void load_default_data(void) {
  params.weight_x[0] = 1.10159580514915;
  params.x_d[0] = 0.832591290472419;
  params.x_d[1] = -0.836381044348223;
  params.x_d[2] = 0.0433104207906521;
  params.J_v[0] = 1.57178781739062;
  params.J_v[1] = 1.58517235573375;
  params.J_v[2] = -1.49765875814465;
  params.J_v[3] = -1.17102848744725;
  params.J_v[4] = -1.79413118679668;
  params.J_v[5] = -0.236760625397454;
  params.J_v[6] = -1.88049515648573;
  params.J_v[7] = -0.172667102421156;
  params.J_v[8] = 0.596576190459043;
  params.J_v[9] = -0.886050869408099;
  params.J_v[10] = 0.705019607920525;
  params.J_v[11] = 0.363451269665403;
  params.J_v[12] = -1.90407247049134;
  params.J_v[13] = 0.235416351963528;
  params.J_v[14] = -0.962990212370138;
  params.J_v[15] = -0.339595211959721;
  params.J_v[16] = -0.865899672914725;
  params.J_v[17] = 0.772551673251985;
  params.J_v[18] = -0.238185129317042;
  params.J_v[19] = -1.37252904610015;
  params.J_v[20] = 0.178596072127379;
  params.weight_w[0] = 1.56062952902273;
  params.w_d[0] = -0.774545870495281;
  params.w_d[1] = -1.11216846427127;
  params.w_d[2] = -0.448114969777405;
  params.J_w[0] = 1.74553459944172;
  params.J_w[1] = 1.90398168989174;
  params.J_w[2] = 0.689534703651255;
  params.J_w[3] = 1.61133643415359;
  params.J_w[4] = 1.38300348517272;
  params.J_w[5] = -0.488023834684443;
  params.J_w[6] = -1.6311319645131;
  params.J_w[7] = 0.613643610094145;
  params.J_w[8] = 0.231363049553804;
  params.J_w[9] = -0.553740947749688;
  params.J_w[10] = -1.09978198064067;
  params.J_w[11] = -0.373920334495006;
  params.J_w[12] = -0.124239005203324;
  params.J_w[13] = -0.923057686995755;
  params.J_w[14] = -0.83282890309827;
  params.J_w[15] = -0.169254402708088;
  params.J_w[16] = 1.44213565178771;
  params.J_w[17] = 0.345011617871286;
  params.J_w[18] = -0.866048550271161;
  params.J_w[19] = -0.888089973505595;
  params.J_w[20] = -0.181511697912213;
  params.weight_q[0] = 0.410820689209975;
  params.q_min[0] = -1.19448515582771;
  params.q_min[1] = 0.0561402392697676;
  params.q_min[2] = -1.65108252487678;
  params.q_min[3] = -0.0656578705936539;
  params.q_min[4] = -0.551295150448667;
  params.q_min[5] = 0.830746487262684;
  params.q_min[6] = 0.986984892408018;
  params.q[0] = 0.764371687423057;
  params.q[1] = 0.756721655019656;
  params.q[2] = -0.505599503404287;
  params.q[3] = 0.67253921894107;
  params.q[4] = -0.640605344172728;
  params.q[5] = 0.2911754794755;
  params.q[6] = -0.696771367740502;
  params.q_max[0] = -0.219419802945872;
  params.q_max[1] = -1.75388427668024;
  params.q_max[2] = -1.02929831126265;
  params.q_max[3] = 1.88641042469427;
  params.q_max[4] = -1.0776631825797;
  params.q_max[5] = 0.765910043789321;
  params.q_max[6] = 0.601907432854958;
}
