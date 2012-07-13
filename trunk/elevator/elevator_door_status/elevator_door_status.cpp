#include <cmath>
#include <cstdio>
#include "elevator_door_status.h"
using std::vector;

ElevatorDoorStatus::ElevatorDoorStatus()
{
}

ElevatorDoorStatus::~ElevatorDoorStatus()
{
}

void ElevatorDoorStatus::set_reference(vector<float> scan)
{
  ref = scan;
}

ElevatorDoorStatus::DoorState ElevatorDoorStatus::detect(vector<float> scan)
{
  static int scan_count = 0;
  scan_count++;
  if (!ref.size() || ref.size() != scan.size())
    return INVALID_DATA;
  vector<float> diff = ref;
  // find sum of differences on the right and left sides of the scan
  float left_sum = 0, left_idx_sum = 0;
  float right_sum = 0, right_idx_sum = 0;
  for (size_t i = 0; i < scan.size(); i++)
  {
    diff[i] -= scan[i];
    if (diff[i] > 0)
      continue; // we're only looking for holes, not filled space
    if (i < scan.size() / 2)
    {
      left_sum += diff[i];
      left_idx_sum++;
    }
    else
    {
      right_sum += diff[i];
      right_idx_sum++;
    }
  }
  //if (left_sum < -3 || right_sum < -3) // depends on how far away you are
  //  printf("%.05f  %.05f\n", left_sum, right_sum);
  if (left_sum < -3)
    return LEFT_OPEN;
  else if (right_sum < -3)
    return RIGHT_OPEN;
  else
    return ALL_CLOSED;
}

