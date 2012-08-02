/* Produced by CVXGEN, 2012-04-02 14:50:07 -0700.  */
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
#define NUMTESTS 1000000

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
  params.a_u[0] = 0.203191610298302;
  params.a_u[1] = 0.832591290472419;
  params.a_u[2] = -0.836381044348223;
  params.M[0] = 1.51082760519766;
  params.M[1] = 1.89294695434765;
  params.M[2] = 1.89629308893344;
  params.alpha_u[0] = -1.49765875814465;
  params.alpha_u[1] = -1.17102848744725;
  params.alpha_u[2] = -1.79413118679668;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.I[0] = 1.44080984365064;
  params.I[3] = 0;
  params.I[6] = 0;
  params.I[1] = 0;
  params.I[4] = 1.02987621087857;
  params.I[7] = 0;
  params.I[2] = 0;
  params.I[5] = 0;
  params.I[8] = 1.45683322439471;
  params.n_0[0] = 0.596576190459043;
  params.n_0[1] = -0.886050869408099;
  params.n_0[2] = 0.705019607920525;
  params.rxn_0[0] = 0.363451269665403;
  params.rxn_0[1] = -1.90407247049134;
  params.rxn_0[2] = 0.235416351963528;
  params.n_1[0] = -0.962990212370138;
  params.n_1[1] = -0.339595211959721;
  params.n_1[2] = -0.865899672914725;
  params.rxn_1[0] = 0.772551673251985;
  params.rxn_1[1] = -0.238185129317042;
  params.rxn_1[2] = -1.37252904610015;
  params.n_2[0] = 0.178596072127379;
  params.n_2[1] = 1.12125905804547;
  params.n_2[2] = -0.774545870495281;
  params.rxn_2[0] = -1.11216846427127;
  params.rxn_2[1] = -0.448114969777405;
  params.rxn_2[2] = 1.74553459944172;
  params.n_3[0] = 1.90398168989174;
  params.n_3[1] = 0.689534703651255;
  params.n_3[2] = 1.61133643415359;
  params.rxn_3[0] = 1.38300348517272;
  params.rxn_3[1] = -0.488023834684443;
  params.rxn_3[2] = -1.6311319645131;
  params.n_4[0] = 0.613643610094145;
  params.n_4[1] = 0.231363049553804;
  params.n_4[2] = -0.553740947749688;
  params.rxn_4[0] = -1.09978198064067;
  params.rxn_4[1] = -0.373920334495006;
  params.rxn_4[2] = -0.124239005203324;
  params.n_5[0] = -0.923057686995755;
  params.n_5[1] = -0.83282890309827;
  params.n_5[2] = -0.169254402708088;
  params.rxn_5[0] = 1.44213565178771;
  params.rxn_5[1] = 0.345011617871286;
  params.rxn_5[2] = -0.866048550271161;
  params.n_6[0] = -0.888089973505595;
  params.n_6[1] = -0.181511697912213;
  params.n_6[2] = -1.17835862158005;
  params.rxn_6[0] = -1.19448515582771;
  params.rxn_6[1] = 0.0561402392697676;
  params.rxn_6[2] = -1.65108252487678;
  params.n_7[0] = -0.0656578705936539;
  params.n_7[1] = -0.551295150448667;
  params.n_7[2] = 0.830746487262684;
  params.rxn_7[0] = 0.986984892408018;
  params.rxn_7[1] = 0.764371687423057;
  params.rxn_7[2] = 0.756721655019656;
  params.n_8[0] = -0.505599503404287;
  params.n_8[1] = 0.67253921894107;
  params.n_8[2] = -0.640605344172728;
  params.rxn_8[0] = 0.2911754794755;
  params.rxn_8[1] = -0.696771367740502;
  params.rxn_8[2] = -0.219419802945872;
  params.n_9[0] = -1.75388427668024;
  params.n_9[1] = -1.02929831126265;
  params.n_9[2] = 1.88641042469427;
  params.rxn_9[0] = -1.0776631825797;
  params.rxn_9[1] = 0.765910043789321;
  params.rxn_9[2] = 0.601907432854958;
  params.n_10[0] = 0.895756557749928;
  params.n_10[1] = -0.0996455574622748;
  params.n_10[2] = 0.386655098407451;
  params.rxn_10[0] = -1.73212230426869;
  params.rxn_10[1] = -1.70975144871107;
  params.rxn_10[2] = -1.20409589481169;
}
