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
    double cosPI=-1;
    double sinPI=0;
    // robot.setK(
    //   {{cosPI,-sinPI,0,0,
    //     sinPI,cosPI,0,0,
    //     0,0,1,0,
    //     0,0,0,1}} //rotation around z-axis
    // );//Sets the transformation from end effector frame to stiffness frame (reorientates the force-torque sensor signal).
    // robot.setK(
    //   {{1,0,0,0,
    //     0,cosPI,-sinPI,0,
    //     0,sinPI,cosPI,0,
    //     0,0,0,1}} //rotation around x-axis
    // );//Sets the transformation from end effector frame to stiffness frame (reorientates the force-torque sensor signal).
    // robot.setK(
    //   {{cosPI,0,sinPI,0,
    //     0,1,0,0,
    //     -sinPI,0,cosPI,0,
    //     0,0,0,1}} //rotation around y-axis
    // );//Sets the transformation from end effector frame to stiffness frame (reorientates the force-torque sensor signal).
    robot.setJointImpedance({{100,100,100,100,100,100,100}}); //TODO which values to choose?
    robot.setCartesianImpedance({{100,100,100,10,10,10}}); //TODO which values to choose?
    robot.setCollisionBehavior( //values taken from https://github.com/frankaemika/libfranka/blob/master/examples/generate_joint_velocity_motion.cpp#L39
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    
    //TODO: be careful with this mode!
    robot.setCollisionBehavior(
        {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}}, {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}},
        {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}}, {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}},
        {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}}, {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}},
        {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}}, {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}});

    // Get the initial state of the robot
    const auto state = robot.readOnce();
    const int dof = state.q.size();

    // Initialize mc_rtc
    mc_control::MCGlobalController controller;
    if(controller.controller().timeStep != 0.001)
    {
      LOG_ERROR_AND_THROW(std::runtime_error, "mc_rtc must be configured to run at 1kHz");
    }
    sva::ForceVecd wrench = sva::ForceVecd(Eigen::Vector6d());
    std::map<std::string, sva::ForceVecd> wrenches;
    wrenches.insert(std::make_pair("LeftHandForceSensor", wrench));
    std::vector<double> init_q_vector;
    std::vector<double> q_vector;
    std::vector<double> dq_vector;
    std::vector<double> tau_vector;
    init_q_vector.resize(dof);
    q_vector.resize(dof);
    dq_vector.resize(dof);
    tau_vector.resize(dof);
    for(size_t i = 0; i < dof; ++i)
    {
      init_q_vector.at(i)=state.q[i];
    }
    // FIXME Temporary work-around until we handle the gripper
    int counter=dof-1;
    while(controller.robot().refJointOrder().size() > init_q_vector.size())
    {
      counter++;
      init_q_vector.at(counter)=0;
    }
    controller.init(init_q_vector);
    controller.running = true;

    controller.controller().gui()->addElement({"Franka"},
                                              mc_rtc::gui::Button("Stop controller", [&controller]() { controller.running = false; }));

    if(false)
    {
      // Start the control loop in position-control
      franka::JointPositions output_q(state.q);
      robot.control([&controller,&q_vector,&dq_vector,&tau_vector,&wrench,&wrenches,&output_q](const franka::RobotState & state, franka::Duration) -> franka::JointPositions
                    {
                      for(size_t i = 0; i < state.q.size(); ++i)
                      {
                        q_vector[i] = state.q[i];
                        dq_vector[i] = state.dq[i];
                        tau_vector[i] = state.tau_J[i];
                      }
                      controller.setEncoderValues(q_vector);
                      controller.setEncoderVelocities(dq_vector);
                      // controller.setJointTorques(tau_vector);
                      // controller.setWrenches(wrenches);
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
      robot.control([&controller,&q_vector,&dq_vector,&tau_vector,&wrench,&wrenches,&output_dq](const franka::RobotState & state, franka::Duration) -> franka::JointVelocities
                    {
                      for(size_t i = 0; i < state.q.size(); ++i)
                      {
                        q_vector[i] = state.q[i];
                        dq_vector[i] = state.dq[i];
                        tau_vector[i] = state.tau_J[i];
                      }

                      wrench.force().x() = state.K_F_ext_hat_K[0]; //TODO use: state.K_F_ext_hat_K[] or state.O_F_ext_hat_K[] ?
                      wrench.force().y() = state.K_F_ext_hat_K[1];
                      wrench.force().z() = state.K_F_ext_hat_K[2];
                      wrench.moment().x() = state.K_F_ext_hat_K[3];
                      wrench.moment().y() = state.K_F_ext_hat_K[4];
                      wrench.moment().z() = state.K_F_ext_hat_K[5];

                      // wrench.force().x() = state.O_F_ext_hat_K[0]; //TODO use: state.K_F_ext_hat_K[] or state.O_F_ext_hat_K[] ?
                      // wrench.force().y() = state.O_F_ext_hat_K[1];
                      // wrench.force().z() = state.O_F_ext_hat_K[2];
                      // wrench.moment().x() = state.O_F_ext_hat_K[3];
                      // wrench.moment().y() = state.O_F_ext_hat_K[4];
                      // wrench.moment().z() = state.O_F_ext_hat_K[5];
                      
                      wrenches.find("LeftHandForceSensor")->second = wrench;

                      int randomNumber = rand() % 1000 + 1; //generate number between 1 and 1000
                      if (randomNumber==1000){
                        LOG_INFO("force = [" << wrench.force().x() << ", " << wrench.force().y() << ", " << wrench.force().z() << "]")
                      }
                      
                      controller.setEncoderValues(q_vector);
                      controller.setEncoderVelocities(dq_vector);
                      // controller.setJointTorques(tau_vector);
                      // controller.setWrenches(wrenches);
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
