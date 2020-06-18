#pragma once

// #include <franka/command_types.h>
// #include <franka/control_tools.h>
// #include <franka/control_types.h>
// #include <franka/duration.h>
// #include <franka/errors.h>
// #include <franka/exception.h>
// #include <franka/gripper.h>
// #include <franka/log.h>
// #include <franka/lowpass_filter.h>
// #include <franka/model.h>
// #include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
// #include <franka/vacuum_gripper.h>
// #include <franka/vacuum_gripper_state.h>
#include <motion_generator_traits.h>
#include <network.h>
#include <robot_impl.h>
// #include <robot_control.h>
// #include <research_interface/gripper/types.h>
// #include <research_interface/robot/rbk_types.h>
// #include <research_interface/robot/service_traits.h>
// #include <research_interface/robot/service_types.h>

// #include <Eigen/Core>
#include <Eigen/Geometry>

#include <map>
#include <iostream>
#include <string>

enum ControlModes {Position, Velocity, Torque};
static const std::map < ControlModes, std::string > ControlModeMap = 
{
  {Position, "JointPositionCtrl"}, 
  {Velocity, "JointVelocityCtrl"}, 
  {Torque, "JointTorqueCtrl"}
};

class Controller {
    public:
        Controller(std::unique_ptr<franka::Robot::Impl> franka_control);

        ~Controller() {
        }

        franka::RobotState getInitialState();
        std::string getControlMode();
        bool setControlMode(const std::string &controlMode);
        bool start();
        bool setCommand(std::array<double, 7> cmd);
        double getState(std::array<double, 7> q, std::array<double, 7> dq, std::array<double, 7> tau, Eigen::Vector3d & force, Eigen::Vector3d & moment);
        bool stop();

    private:
        ControlModes current_control_mode = ControlModes::Position;
        uint32_t motion_id = 0;
        std::unique_ptr<franka::Model> franka_model;
        // std::unique_ptr<franka::RobotControl> franka_control;
        std::unique_ptr<franka::Robot::Impl> franka_control;
        franka::RobotState franka_state;
        franka::RobotState franka_stateInitial;
        research_interface::robot::MotionGeneratorCommand motion_command;
        research_interface::robot::ControllerCommand control_command;
        static constexpr research_interface::robot::Move::Deviation kDefaultDeviation {10.0, 3.12, 2 * M_PI};
};