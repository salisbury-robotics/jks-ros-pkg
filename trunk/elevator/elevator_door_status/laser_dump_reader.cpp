#include "elevator_door_status.h"
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <string>
#include <vector>
using std::string;

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    printf("usage: laser_dump_reader DUMPFILE\n");
    return 1;
  }
  FILE *dumpfile = fopen(argv[1], "r");
  if (!dumpfile)
  {
    printf("couldn't open %s\n", argv[1]);
    return 1;
  }
  char linebuf[10000];
  std::vector<float> scan;
  ElevatorDoorStatus e;
  for (int scan_idx = 0; !feof(dumpfile); scan_idx++)
  {
    fgets(linebuf, sizeof(linebuf), dumpfile);
    size_t tok_idx = 0;
    for (char *tok = strtok(linebuf, " \n"); tok; tok = strtok(NULL, " \n"))
    {
      const float range = atof(tok) * 0.01;
      if (tok_idx >= scan.size())
        scan.push_back(range);
      else
        scan[tok_idx] = range;
      tok_idx++;
    }
    if (scan_idx == 0)
      e.set_reference(scan);
    else if (scan_idx % 100 == 0)
    {
      ElevatorDoorStatus::DoorState ds = e.detect(scan);
      if (ds == ElevatorDoorStatus::LEFT_OPEN)
        printf("%d: left\n", scan_idx);
      else if (ds == ElevatorDoorStatus::RIGHT_OPEN)
        printf("%d: right\n", scan_idx);
      else
        printf("%d: closed\n", scan_idx);
    }
  }
  fclose(dumpfile);
  return 0;
}

