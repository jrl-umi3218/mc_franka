#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/vacuum_gripper.h>

#include <iostream>

void usage(const char * prog)
{
  std::cerr << "[usage] " << prog << " <fci-ip>\n";
}

int main(int argc, char * argv[])
{
  if(argc < 2)
  {
    usage(argv[0]);
    return 1;
  }
  try
  {
    franka::Robot robot(argv[1]);
    franka::VacuumGripper sucker(argv[1]);
    sucker.stop();
    robot.stop();
  }
  catch(const franka::Exception & e)
  {
    std::cerr << "franka::Exception: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
