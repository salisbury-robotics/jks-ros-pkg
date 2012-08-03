#include <cstdio>
#include <signal.h>
#include "hydra.h"
using namespace hydra;

static bool g_done = false;
void sigint_handler(int signal)
{
    g_done = true;
}

int main(int argc, char **argv)
{
  if (argc != 3)
  {
    printf("usage: print DEVICE FILENAME\n");
    return 1;
  }
  signal(SIGINT, sigint_handler);
  Hydra h;
  if (!h.init(argv[1]))
    return 1;
  FILE *f = fopen(argv[2], "w");
  static int print_count = 0;
  while (!g_done)
  {
    if (h.poll(10))
    {
      char buf[1000];
      snprintf(buf, sizeof(buf),
               "%.3f %.3f %.3f %.3f %.3f %.3f %.3f",
               h.pos[1].x(), h.pos[1].y(), h.pos[1].z(),
               h.quat[1].x(), h.quat[1].y(),
               h.quat[1].z(), h.quat[1].w());
      fprintf(f, "%s\n", buf);
      if (print_count++ % 100 == 0)
        printf("%s\n", buf);
    }
  }
  fclose(f);
  return 0;
}

