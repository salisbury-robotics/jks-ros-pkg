#include <cstdio>
#include <signal.h>
#include "hydra/hydra.h"
using namespace hydra;

static bool g_done = false;
void sigint_handler(int signal)
{
    g_done = true;
}

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    printf("usage: print DEVICE\n");
    return 1;
  }
  signal(SIGINT, sigint_handler);
  Hydra h;
  if (!h.init(argv[1]))
    return 1;

  printf("Finished initializing, now printing data stream:\n");

  while (!g_done)
  {
    if (h.poll(10))
    {
      printf("%.3f %.3f %.3f      %.3f %.3f %.3f\n",
             h.pos[0].x(), h.pos[0].y(), h.pos[0].z(),
             h.pos[1].x(), h.pos[1].y(), h.pos[1].z());
    }
  }
  return 0;
}

