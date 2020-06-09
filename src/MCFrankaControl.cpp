#include <mc_control/mc_global_controller.h>

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
    auto state = robot.readOnce();

    // Initialize mc_rtc
    mc_control::MCGlobalController controller;
    if(controller.controller().timeStep != 0.001)
    {
      LOG_ERROR_AND_THROW(std::runtime_error, "mc_rtc must be configured to run at 1kHz");
    }
    std::vector<double> initq;
    for(size_t i = 0; i < state.q.size(); ++i)
    {
      initq.push_back(state.q[i]);
    }
    // FIXME Temporary work-around until we handle the gripper
    while(controller.robot().refJointOrder().size() > initq.size())
    {
      initq.push_back(0);
    }
    controller.init(initq);
    controller.running = true;

    controller.controller().gui()->addElement({"Franka"},
                                              mc_rtc::gui::Button("Stop controller", [&controller]() { controller.running = false; }));

    // Start the control loop
    franka::JointPositions output(state.q);
    robot.control([&controller,&initq,&output](const franka::RobotState & state, franka::Duration) -> franka::JointPositions
                  {
                    for(size_t i = 0; i < state.q.size(); ++i)
                    {
                      initq[i] = state.q[i];
                    }
                    controller.setEncoderValues(initq);
                    if(controller.running && controller.run())
                    {
                      const auto & rjo = controller.robot().refJointOrder();
                      for(size_t i = 0; i < output.q.size(); ++i)
                      {
                        const auto & j = rjo[i];
                        output.q[i] = controller.robot().mbc().q[controller.robot().jointIndexByName(j)][0];
                      }
                      return output;
                    }
                    output.q = state.q;
                    return franka::MotionFinished(output);
                  });
  }
  catch(const franka::Exception & e)
  {
    std::cerr << "franka::Exception " << e.what() << "\n";
    return 1;
  }
  return 0;
}
