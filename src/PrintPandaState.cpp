#include <franka/exception.h>
#include <franka/robot.h>

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
    // Initialize the robot
    franka::Robot robot(argv[1]);

    // Get the initial state of the robot
    const auto state = robot.readOnce();

    // Print the joint positions
    for(size_t i = 0; i < state.q.size(); ++i) {
      std::cout << i+1 << ". Joint position = " << state.q[i] << std::endl;
    }

    // Stop the robot
    robot.stop();
  }
  catch(const franka::Exception & e)
  {
    std::cerr << "franka::Exception " << e.what() << "\n";
    return 1;
  }
  return 0;
}
