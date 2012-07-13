#ifndef ELEVATOR_DOOR_DETECTOR_H
#define ELEVATOR_DOOR_DETECTOR_H

#include <stdint.h>
#include <vector>

class ElevatorDoorStatus
{
public:
  enum DoorState
  {
    INVALID_DATA,
    ALL_CLOSED,
    LEFT_OPEN,
    RIGHT_OPEN
  };

  ElevatorDoorStatus();
  ~ElevatorDoorStatus();
  void set_reference(std::vector<float> scan);
  DoorState detect(std::vector<float> scan);

private:
  std::vector<float> ref; // the reference scan
};

#endif

