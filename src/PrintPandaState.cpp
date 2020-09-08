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
    franka::Robot robot(argv[1]);
    const auto state = robot.readOnce();
    for(size_t i = 0; i < state.q.size(); ++i)
    {
      std::cout << "Joint " << i << " : " << state.q[i] << "\n";
    }
    robot.stop();
  }
  catch(const franka::Exception & e)
  {
    std::cerr << "franka::Exception: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
