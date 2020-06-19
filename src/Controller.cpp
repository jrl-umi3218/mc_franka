#include "Controller.hpp"

constexpr research_interface::robot::Move::Deviation Controller::kDefaultDeviation;

Controller::Controller(franka::Robot::Impl & frankaControl, const ControlMode controlMode) : franka_control(frankaControl), control_mode(controlMode)
{
  std::cout <<"start constructor" << std::endl;
  franka_model = std::make_unique<franka::Model>(franka_control.loadModel());
  std::cout <<"model loaded" << std::endl;
  franka_stateInitial = franka_control.readOnce();
  for(size_t i = 0; i < PANDA_DOF; ++i)
  {
    std::cout << "initial state " << i << ". joint position: " << franka_stateInitial.q[i] << std::endl;
  }
  std::cout <<"initial stated loaded" << std::endl;

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

ControlMode Controller::getControlMode()
{
  std::cout << "current control mode is: " << ControlModeMap.find(control_mode)->second << std::endl;
  return control_mode;
}


bool Controller::setControlMode(const ControlMode controlMode)
{
  std::cout << "setControlMode" << std::endl;
  this->control_mode = controlMode;
  std::cout << "current control mode is: " << ControlModeMap.find(control_mode)->second << std::endl;
  std::cout << "setControlMode done" << std::endl;
  return true;
}


bool Controller::start()
{
  std::cout << "start" << std::endl;
  try 
  {
    std::cout << "starting in mode: " << ControlModeMap.find(control_mode)->second << std::endl;
    switch (control_mode)
    {
      case ControlMode::Torque:
        // motion_id = franka_control.startMotion(research_interface::robot::Move::ControllerMode::kExternalController, 
        //                                         franka::MotionGeneratorTraits<franka::JointVelocities>::kMotionGeneratorMode, 
        //                                         kDefaultDeviation, 
        //                                         kDefaultDeviation);
        motion_id = franka_control.startMotion(research_interface::robot::Move::ControllerMode::kExternalController, 
                                                research_interface::robot::Move::MotionGeneratorMode::kJointVelocity,
                                                kDefaultDeviation,
                                                kDefaultDeviation);
        break;
      case ControlMode::Velocity:
        // motion_id = franka_control.startMotion(research_interface::robot::Move::ControllerMode::kJointImpedance, 
        //                                         franka::MotionGeneratorTraits<franka::JointVelocities>::kMotionGeneratorMode, 
        //                                         kDefaultDeviation, 
        //                                         kDefaultDeviation);
        motion_id = franka_control.startMotion(research_interface::robot::Move::ControllerMode::kJointImpedance, 
                                                research_interface::robot::Move::MotionGeneratorMode::kJointVelocity,
                                                kDefaultDeviation,
                                                kDefaultDeviation);
        break;
      case ControlMode::Position:
        // motion_id = franka_control.startMotion(research_interface::robot::Move::ControllerMode::kJointImpedance, 
        //                                         franka::MotionGeneratorTraits<franka::JointPositions>::kMotionGeneratorMode, 
        //                                         kDefaultDeviation, 
        //                                         kDefaultDeviation);
        motion_id = franka_control.startMotion(research_interface::robot::Move::ControllerMode::kJointImpedance, 
                                                research_interface::robot::Move::MotionGeneratorMode::kJointPosition,
                                                kDefaultDeviation,
                                                kDefaultDeviation);
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


bool Controller::setCommand(std::array<double, PANDA_DOF> cmd)
{
  try 
  {
    // std::cout << "setCommand" << std::endl;
    motion_command.q_c.fill(0);
    motion_command.dq_c.fill(0);
    motion_command.O_T_EE_c.fill(0);
    motion_command.O_dP_EE_c.fill(0);
    motion_command.elbow_c.fill(0);
    control_command.tau_J_d.fill(0);

    switch (control_mode)
    {
      case ControlMode::Torque:
        // motion_command = ...; //TODO
        control_command.tau_J_d = cmd;
        break;
      case ControlMode::Velocity:
        motion_command.dq_c = cmd;
        break;
      case ControlMode::Position:
        motion_command.q_c = cmd;
        break;
      default:
        return false;
    }

    if(control_mode == ControlMode::Torque) 
    {
      franka_state = franka_control.update(&motion_command, &control_command);
    } else {
      franka_state = franka_control.update(&motion_command, nullptr);
    }
    franka_control.throwOnMotionError(franka_state, motion_id);
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
  // std::cout << "setCommand done" << std::endl;
  return true;
}


double Controller::getState(std::array<double, PANDA_DOF> q, std::array<double, PANDA_DOF> dq, std::array<double, PANDA_DOF> tau, Eigen::Vector3d & force, Eigen::Vector3d & moment)
{
  // std::cout << "getState" << std::endl;
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

    // std::array<double, PANDA_DOF*PANDA_DOF> inertia_feedback = franka_model->mass(franka_state);
    // std::array<double, PANDA_DOF> coriolis_feedback = franka_model->coriolis(franka_state);
    // std::array<double, PANDA_DOF> gravity_feedback = franka_model->gravity(franka_state);
    // std::array<double, 6*PANDA_DOF> jacobian_feedback = franka_model->zeroJacobian(franka::Frame::kFlange,franka_state);

    // std::cout << "Control command success rate: " << franka_state.control_command_success_rate << std::endl;
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
  // std::cout << "getState done" << std::endl;
  return franka_state.control_command_success_rate;
}


bool Controller::stop()
{
  std::cout << "stop" << std::endl;
  try
  {
    //franka_control.cancelMotion(motion_id);
    if(control_mode == ControlMode::Torque) {
      franka_control.finishMotion(motion_id, &motion_command, &control_command);
    } else {
      franka_control.finishMotion(motion_id, &motion_command, nullptr);
    }
  } 
  catch (...) 
  {
    try 
    {
      franka_control.cancelMotion(motion_id);
    } 
    catch (...) {}
    throw;
  }
  std::cout << "stop done" << std::endl;
  return true;
}