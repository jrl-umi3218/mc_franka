#pragma once

#include <mc_rbdyn/Robot.h>

#include <franka/exception.h>
#include <franka/robot.h>

#include "ControlMode.h"

template<ControlMode cm>
struct PandaControlType
{
  static_assert(static_cast<int>(cm) == static_cast<int>(cm) + 1, "This must be specialized");
};

template<>
struct PandaControlType<ControlMode::Position> : public franka::JointPositions
{
  using ReturnT = franka::JointPositions;
  using CallbackT = std::function<ReturnT(const franka::RobotState &, franka::Duration)>;

  PandaControlType(const franka::RobotState & state) : franka::JointPositions(state.q), prev_q_(state.q) {}

  // Interpolate control value from the data in a robot
  franka::JointPositions update(const mc_rbdyn::Robot & robot, const rbd::MultiBodyConfig & mbc, size_t iter, size_t N)
  {
    const auto & rjo = robot.refJointOrder();
    for(size_t i = 0; i < q.size(); ++i)
    {
      q[i] = prev_q_[i] + (iter + 1) * (mbc.q[robot.jointIndexByName(rjo[i])][0] - prev_q_[i]) / N;
    }
    if(iter + 1 == N)
    {
      prev_q_ = q;
    }
    return *this;
  }

  void control(franka::Robot & robot, CallbackT cb)
  {
    robot.control(cb, franka::ControllerMode::kJointImpedance, true, 100);
  }

private:
  std::array<double, 7> prev_q_;
};

template<>
struct PandaControlType<ControlMode::Velocity> : public franka::JointVelocities
{
  using ReturnT = franka::JointVelocities;
  using CallbackT = std::function<ReturnT(const franka::RobotState &, franka::Duration)>;

  PandaControlType(const franka::RobotState & state) : franka::JointVelocities(state.dq) {}

  // Update control value from the data in a robot
  franka::JointVelocities update(const mc_rbdyn::Robot & robot, const rbd::MultiBodyConfig & mbc, size_t, size_t)
  {
    const auto & rjo = robot.refJointOrder();
    for(size_t i = 0; i < dq.size(); ++i)
    {
      dq[i] = mbc.alpha[robot.jointIndexByName(rjo[i])][0];
    }
    return *this;
  }

  void control(franka::Robot & robot, CallbackT cb)
  {
    robot.control(cb, franka::ControllerMode::kJointImpedance, true, 1000);
  }
};

template<>
struct PandaControlType<ControlMode::Torque> : public franka::Torques
{
  using ReturnT = franka::Torques;
  using CallbackT = std::function<ReturnT(const franka::RobotState &, franka::Duration)>;

  PandaControlType(const franka::RobotState & state) : franka::Torques(state.tau_J) {}

  // Update control value from the data in a robot
  franka::Torques update(const mc_rbdyn::Robot & robot, const rbd::MultiBodyConfig & mbc, size_t, size_t)
  {
    const auto & rjo = robot.refJointOrder();
    for(size_t i = 0; i < tau_J.size(); ++i)
    {
      tau_J[i] = mbc.jointTorque[robot.jointIndexByName(rjo[i])][0];
    }
    return *this;
  }

  void control(franka::Robot & robot, CallbackT cb)
  {
    robot.control(cb, true, 1000);
  }
};
