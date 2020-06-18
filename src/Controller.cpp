#include "Controller.hpp"

constexpr research_interface::robot::Move::Deviation Controller::kDefaultDeviation;

Controller::Controller(std::unique_ptr<franka::Robot::Impl> franka_control)
{
  std::cout <<"start constructor" << std::endl;
  franka_model = std::make_unique<franka::Model>(franka_control->loadModel());
  std::cout <<"model loaded" << std::endl;
  if (!franka_control) //check for nullpointer
  {
    std::cerr << "franka_control null" << std::endl;
  }
  else
  {
    std::cout << "franka_control not null" << std::endl;
  }
  // this->franka_control = std::move(franka_control);
  std::cout <<"transition performed from franka::Robot::Impl to franka::RobotControl" << std::endl;
  if (!franka_control) //check for nullpointer
  {
    std::cerr << "franka_control null" << std::endl;
  }
  else
  {
    std::cout << "franka_control not null" << std::endl;
  }
  
  franka_stateInitial = franka_control->readOnce();
  std::cout <<"initial stated loaded" << std::endl;

  bool ok = this->setControlMode("JointPositionCtrl");

  // //TODO what about this part?
  // std::string franka_address = "";
  // std::unique_ptr<franka::Robot> robotPtr;
  // robotPtr.reset(new franka::Robot(franka_address));
  // robotPtr->setJointImpedance({{100,100,100,100,100,100,100}}); //TODO which values to choose?
  // robotPtr->setCartesianImpedance({{100,100,100,10,10,10}}); //TODO which values to choose?
  // robotPtr->setCollisionBehavior( //values taken from https://github.com/frankaemika/libfranka/blob/master/examples/generate_joint_velocity_motion.cpp#L39
  //     {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
  //     {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
  //     {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
  //     {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
  // franka::RobotState statePtrTEST = robotPtr->readOnce();
  std::cout << "constructor done" << std::endl;
}

franka::RobotState Controller::getInitialState()
{
  return franka_stateInitial;
}

std::string Controller::getControlMode()
{
  std::string s = ControlModeMap.find(current_control_mode)->second;
  std::cout << "current control mode is: " << s << std::endl;
  return s;
}


bool Controller::setControlMode(const std::string &controlMode)
{
  std::cout << "setControlMode" << std::endl;
  if (controlMode == ControlModeMap.find(ControlModes::Torque)->second) {
    current_control_mode = ControlModes::Torque;
    std::cout << "Set control mode to: " << controlMode << std::endl;
  } else if(controlMode == ControlModeMap.find(ControlModes::Velocity)->second) {
    current_control_mode = ControlModes::Velocity;
    std::cout << "Set control mode to: " << controlMode << std::endl;
  } else if(controlMode == ControlModeMap.find(ControlModes::Position)->second) {
    current_control_mode = ControlModes::Position;
    std::cout << "Set control mode to: " << controlMode << std::endl;
  } else {
    std::cout << "Available control modes are: " 
              << ControlModeMap.find(ControlModes::Position)->second << ", " 
              << ControlModeMap.find(ControlModes::Velocity)->second << ", " 
              << ControlModeMap.find(ControlModes::Torque)->second << std::endl;
    std::cerr << "Chosen control mode has not been implemented: " << controlMode << std::endl;
    return false;
  }
  std::cout << "setControlMode done" << std::endl;
  return true;
}


bool Controller::start()
{
  std::cout << "start" << std::endl;
  try 
  {
    switch (current_control_mode)
    {
      case ControlModes::Torque:
        std::cout << "starting in mode: " << ControlModeMap.find(ControlModes::Torque)->second << std::endl;
        motion_id = franka_control->startMotion(research_interface::robot::Move::ControllerMode::kExternalController, 
                                                franka::MotionGeneratorTraits<franka::JointVelocities>::kMotionGeneratorMode, 
                                                kDefaultDeviation, 
                                                kDefaultDeviation);
        break;
      case ControlModes::Velocity:
        std::cout << "starting in mode: " << ControlModeMap.find(ControlModes::Velocity)->second << std::endl;
        motion_id = franka_control->startMotion(research_interface::robot::Move::ControllerMode::kJointImpedance, 
                                                franka::MotionGeneratorTraits<franka::JointVelocities>::kMotionGeneratorMode, 
                                                kDefaultDeviation, 
                                                kDefaultDeviation);
        break;
      case ControlModes::Position:
        std::cout << "starting in mode: " << ControlModeMap.find(ControlModes::Position)->second << std::endl;
        // motion_id = franka_control->startMotion(research_interface::robot::Move::ControllerMode::kJointImpedance, 
        //                                         franka::MotionGeneratorTraits<franka::JointPositions>::kMotionGeneratorMode, 
        //                                         kDefaultDeviation, 
        //                                         kDefaultDeviation);
        // research_interface::robot::Move::Deviation maximum_path_deviation{0, 1, 2};
        // research_interface::robot::Move::Deviation maximum_goal_pose_deviation{3, 4, 5};
        motion_id = franka_control->startMotion(research_interface::robot::Move::ControllerMode::kJointImpedance, 
                                                research_interface::robot::Move::MotionGeneratorMode::kJointPosition,kDefaultDeviation,kDefaultDeviation);
        break;
      default:
        std::cerr << "control mode not implemented" << std::endl;
        return false;
    }
  }
  catch (const franka::NetworkException &exc) 
  {
    std::cout <<"franka::NetworkException: "<<exc.what() << std::endl;
    throw;
  } 
  catch (const std::exception &exc) 
  {
    std::cout <<"std::exception: "<<exc.what() << std::endl;
    throw;
  }
  std::cout << "start done" << std::endl;
  return true;
}


bool Controller::setCommand(std::array<double, 7> cmd)
{
  try 
  {
    std::cout << "setCommand" << std::endl;
    motion_command.q_c.fill(0);
    motion_command.dq_c.fill(0);
    motion_command.O_T_EE_c.fill(0);
    motion_command.O_dP_EE_c.fill(0);
    motion_command.elbow_c.fill(0);
    control_command.tau_J_d.fill(0);

    switch (current_control_mode)
    {
      case ControlModes::Torque:
        // motion_command = ...; //TODO
        control_command.tau_J_d = cmd;
        break;
      case ControlModes::Velocity:
        motion_command.dq_c = cmd;
        break;
      case ControlModes::Position:
        motion_command.q_c = cmd;
        break;
      default:
        return false;
    }

    if(current_control_mode == ControlModes::Torque) 
    {
      franka_state = franka_control->update(&motion_command, &control_command);
    } else {
      franka_state = franka_control->update(&motion_command, nullptr);
    }
    franka_control->throwOnMotionError(franka_state, motion_id);
  } 
  catch (const franka::NetworkException &exc) 
  {
    std::cout <<"franka::NetworkException: "<<exc.what() << std::endl;
    throw;
  } 
  catch (const std::exception &exc) 
  {
    std::cout <<"std::exception: "<<exc.what() << std::endl;
    throw;
  }
  std::cout << "setCommand done" << std::endl;
  return true;
}


double Controller::getState(std::array<double, 7> q, std::array<double, 7> dq, std::array<double, 7> tau, Eigen::Vector3d & force, Eigen::Vector3d & moment)
{
  std::cout << "getState" << std::endl;
  try
  {
    q = franka_state.q;
    dq = franka_state.dq;
    tau = franka_state.tau_J;
    force.x() = franka_state.K_F_ext_hat_K[0]; //TODO use: franka_state.K_F_ext_hat_K[] or franka_state.O_F_ext_hat_K[] ?
    force.y() = franka_state.K_F_ext_hat_K[1];
    force.z() = franka_state.K_F_ext_hat_K[2];
    moment.x() = franka_state.K_F_ext_hat_K[3];
    moment.y() = franka_state.K_F_ext_hat_K[4];
    moment.z() = franka_state.K_F_ext_hat_K[5];

    std::array<double, 49> inertia_feedback = franka_model->mass(franka_state);
    std::array<double, 7> coriolis_feedback = franka_model->coriolis(franka_state);
    std::array<double, 7> gravity_feedback = franka_model->gravity(franka_state);
    std::array<double, 42> jacobian_feedback = franka_model->zeroJacobian(franka::Frame::kFlange,franka_state);
    std::cout << "Control command success rate: " << franka_state.control_command_success_rate << std::endl;
    std::cout << "getState done" << std::endl;
  } 
  catch (const franka::NetworkException &exc) 
  {
    std::cout <<"franka::NetworkException: "<<exc.what() << std::endl;
    throw;
  } 
  catch (const std::exception &exc) 
  {
    std::cout <<"std::exception: "<<exc.what() << std::endl;
    throw;
  }
  return franka_state.control_command_success_rate;
}


bool Controller::stop()
{
  std::cout << "stop" << std::endl;
  try
  {
    //franka_control->cancelMotion(motion_id);
    if(current_control_mode == ControlModes::Torque) {
      franka_control->finishMotion(motion_id, &motion_command, &control_command);
    } else {
      franka_control->finishMotion(motion_id, &motion_command, nullptr);
    }
  } 
  catch (...) 
  {
    try 
    {
      franka_control->cancelMotion(motion_id);
    } 
    catch (...) {}
    throw;
  }
  std::cout << "stop done" << std::endl;
  return true;
}