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
    robot.setJointImpedance({{100,100,100,100,100,100,100}}); //TODO which values to choose?
    robot.setCartesianImpedance({{100,100,100,10,10,10}}); //TODO which values to choose?
    robot.setCollisionBehavior( //values taken from https://github.com/frankaemika/libfranka/blob/master/examples/generate_joint_velocity_motion.cpp#L39
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

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

    if(false)
    {
      // Start the control loop in position-control
      franka::JointPositions output_q(state.q);
      robot.control([&controller,&initq,&output_q](const franka::RobotState & state, franka::Duration) -> franka::JointPositions
                    {
                      for(size_t i = 0; i < state.q.size(); ++i)
                      {
                        initq[i] = state.q[i];
                      }
                      controller.setEncoderValues(initq);
                      if(controller.running && controller.run())
                      {
                        const auto & rjo = controller.robot().refJointOrder();
                        for(size_t i = 0; i < output_q.q.size(); ++i)
                        {
                          const auto & j = rjo[i];
                          output_q.q[i] = controller.robot().mbc().q[controller.robot().jointIndexByName(j)][0];
                        }
                        return output_q;
                      }
                      output_q.q = state.q;
                      return franka::MotionFinished(output_q);
                    // }, franka::ControllerMode::kJointImpedance, true, 100); //default parameters
                    }, franka::ControllerMode::kJointImpedance, true, 1); //reduced kDefaultCutoffFrequency 
    }
    else
    {
      // Start the control loop in velocity-control
      franka::JointVelocities output_dq(state.dq);
      robot.control([&controller,&initq,&output_dq](const franka::RobotState & state, franka::Duration) -> franka::JointVelocities
                    {
                      for(size_t i = 0; i < state.q.size(); ++i)
                      {
                        initq[i] = state.q[i];
                      }
                      controller.setEncoderValues(initq);
                      if(controller.running && controller.run())
                      {
                        const auto & rjo = controller.robot().refJointOrder();
                        for(size_t i = 0; i < output_dq.dq.size(); ++i)
                        {
                          const auto & j = rjo[i];
                          output_dq.dq[i] = controller.robot().mbc().alpha[controller.robot().jointIndexByName(j)][0];
                        }
                        return output_dq;
                      }
                      output_dq.dq = state.dq;
                      return franka::MotionFinished(output_dq);
                    // }, franka::ControllerMode::kJointImpedance, true, 100); //default parameters
                    }, franka::ControllerMode::kJointImpedance, true, 1000); //increased kDefaultCutoffFrequency 
    }
  }
  catch(const franka::Exception & e)
  {
    std::cerr << "franka::Exception " << e.what() << "\n";
    return 1;
  }
  return 0;
}
