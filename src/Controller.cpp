#include "Controller.hpp"

constexpr research_interface::robot::Move::Deviation Controller::kDefaultDeviation;

Controller::Controller(std::unique_ptr<franka::Robot::Impl> franka_control)
{
  franka_model = std::make_unique<franka::Model>(franka_control->loadModel());
  this->franka_control = std::move(franka_control);
  if (!franka_control) {
    std::cout << "error in constructor";
  }
  franka_stateInitial = franka_control->readOnce();
  this->setControlMode("Position");

  //TODO what about this part?
  std::string franka_address = "";
  std::unique_ptr<franka::Robot> robotPtr;
  robotPtr.reset(new franka::Robot(franka_address));
  robotPtr->setJointImpedance({{100,100,100,100,100,100,100}}); //TODO which values to choose?
  robotPtr->setCartesianImpedance({{100,100,100,10,10,10}}); //TODO which values to choose?
  robotPtr->setCollisionBehavior( //values taken from https://github.com/frankaemika/libfranka/blob/master/examples/generate_joint_velocity_motion.cpp#L39
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
  franka::RobotState statePtrTEST = robotPtr->readOnce();
}

franka::RobotState Controller::getInitialState()
{
  return franka_stateInitial;
}

std::string Controller::getControlMode()
{
  std::string s = ControlModeMap.find(current_control_mode)->second;
  std::cout << "current control mode is: " << s;
  return s;
}


bool Controller::setControlMode(const std::string &controlMode)
{
  if (controlMode == ControlModeMap.find(ControlModes::Torque)->second) {
      current_control_mode = ControlModes::Torque;
      std::cout << "Set control mode to torque";
  } else if(controlMode == ControlModeMap.find(ControlModes::Velocity)->second) {
      current_control_mode = ControlModes::Velocity;
      std::cout << "Set control mode to velocity";
  } else if(controlMode == ControlModeMap.find(ControlModes::Position)->second) {
      current_control_mode = ControlModes::Position;
      std::cout << "Set control mode to position";
  } else {
      std::cout << "Control Mode has not been implemented " << controlMode;
      return false;
  }
  return true;
}


bool Controller::start()
{
  switch (current_control_mode)
  {
    case ControlModes::Torque:
      std::cout << "STARTED IN MODE: " << ControlModeMap.find(ControlModes::Torque)->second;
      motion_id = franka_control->startMotion(research_interface::robot::Move::ControllerMode::kExternalController, franka::MotionGeneratorTraits<franka::JointVelocities>::kMotionGeneratorMode, kDefaultDeviation, kDefaultDeviation);
      break;
    case ControlModes::Velocity:
      std::cout << "STARTED IN MODE: " << ControlModeMap.find(ControlModes::Velocity)->second;
      motion_id = franka_control->startMotion(research_interface::robot::Move::ControllerMode::kJointImpedance, franka::MotionGeneratorTraits<franka::JointVelocities>::kMotionGeneratorMode, kDefaultDeviation, kDefaultDeviation);
      break;
    case ControlModes::Position:
      std::cout << "STARTED IN MODE: " << ControlModeMap.find(ControlModes::Position)->second;
      motion_id = franka_control->startMotion(research_interface::robot::Move::ControllerMode::kJointImpedance, franka::MotionGeneratorTraits<franka::JointPositions>::kMotionGeneratorMode, kDefaultDeviation, kDefaultDeviation);
      break;
    default:
      return false;
  }
  //franka_state = franka_control->update();
  return true;
}


bool Controller::setCommand(std::array<double, 7> cmd)
{
  try {
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
  } catch (const franka::NetworkException &exc) 
  {
    std::cout <<"franka::NetworkException: "<<exc.what();
    throw;
  } catch (const std::exception &exc) 
  {
    std::cout <<"std::exception: "<<exc.what();
    //try {
    //    franka_control->cancelMotion(motion_id);
    //} catch (...) {}
    throw;
  }
  return true;
}


double Controller::getState(std::array<double, 7> q, std::array<double, 7> dq, std::array<double, 7> tau, Eigen::Vector3d & force, Eigen::Vector3d & moment)
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
  std::cout << "Control command success rate: " << franka_state.control_command_success_rate;
  return franka_state.control_command_success_rate;
}


bool Controller::stop()
{
  try
  {
    //franka_control->cancelMotion(motion_id);
    if(current_control_mode == ControlModes::Torque) {
        franka_control->finishMotion(motion_id, &motion_command, &control_command);
    } else {
        franka_control->finishMotion(motion_id, &motion_command, nullptr);
    }
  } catch (...) 
  {
    try {
        franka_control->cancelMotion(motion_id);
    } catch (...) {}
    throw;
  }
  return true;
}