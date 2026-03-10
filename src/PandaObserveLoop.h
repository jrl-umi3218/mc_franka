/* Copyright 2020 mc_rtc development team */

#pragma once

#include "defs.h"

#include <mc_panda/devices/Robot.h>

#include <mc_control/mc_global_controller.h>

#include <franka/robot.h>
#include <franka/robot_state.h>

#include <condition_variable>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

namespace mc_franka
{

/** PandaObserveLoop reads the robot state via franka::Robot::readOnce() without
 * sending any commands. It exposes updateSensors() to feed sensor data into
 * mc_rtc exactly like PandaControlLoop does.
 */
struct PandaObserveLoop
{
  /** Constructor
   *
   * \param name Identifier of the robot
   *
   * \param ip IP of the robot
   *
   * \param device PandaDevice associated to the robot
   *
   * \param poll_period_s Polling period in seconds (default: 0.005)
   */
  PandaObserveLoop(const std::string & name,
                   const std::string & ip,
                   mc_panda::Robot & device,
                   double poll_period_s = 0.005);

  /** Initialize mc_rtc robot from the current state */
  void init(mc_control::MCGlobalController & controller);

  /** Update sensors in mc_rtc instance */
  void updateSensors(mc_control::MCGlobalController & controller);

  /** Poll robot state in a background thread
   *
   * Continuously calls robot_.readOnce() at the configured polling period and
   * updates state_ under stateMutex_. Runs until running is set to false.
   *
   * \param running True while mc_rtc is running; set to false to stop
   */
  void observeThread(bool & running);

private:
  std::string name_;
  franka::Robot robot_;
  franka::RobotState state_;
  mc_panda::Robot & device_;
  double poll_period_s_;

  mutable std::mutex stateMutex_;

  std::vector<double> sensorsBuffer_ = std::vector<double>(7, 0.0);
};

inline PandaObserveLoop::PandaObserveLoop(const std::string & name,
                                          const std::string & ip,
                                          mc_panda::Robot & device,
                                          double poll_period_s)
: name_(name), robot_(ip, franka::RealtimeConfig::kIgnore), state_(robot_.readOnce()), device_(device),
  poll_period_s_(poll_period_s)
{
  static auto panda_init_t = clock::now();
  auto now = clock::now();
  duration_us dt = now - panda_init_t;
  mc_rtc::log::info("[mc_franka] Elapsed time since the creation of another PandaObserveLoop: {}us", dt.count());
  device.connect(&robot_);
}

inline void PandaObserveLoop::init(mc_control::MCGlobalController & controller)
{
  updateSensors(controller);
  auto & robot = controller.controller().robots().robot(name_);
  auto & real = controller.controller().realRobots().robot(name_);
  const auto & rjo = robot.refJointOrder();
  for(size_t i = 0; i < rjo.size(); ++i)
  {
    auto jIndex = robot.jointIndexByName(rjo[i]);
    robot.mbc().q[jIndex][0] = state_.q[i];
    robot.mbc().jointTorque[jIndex][0] = state_.tau_J[i];
  }
  robot.forwardKinematics();
  real.mbc() = robot.mbc();
}

inline void PandaObserveLoop::updateSensors(mc_control::MCGlobalController & controller)
{
  std::unique_lock<std::mutex> lock(stateMutex_);
  auto & robot = controller.controller().robots().robot(name_);
  using GC = mc_control::MCGlobalController;
  using set_sensor_t = void (GC::*)(const std::string &, const std::vector<double> &);
  auto updateSensor = [&controller, &robot, this](set_sensor_t set_sensor, const std::array<double, 7> & data)
  {
    assert(sensorsBuffer_.size() == 7);
    std::memcpy(sensorsBuffer_.data(), data.data(), 7 * sizeof(double));
    (controller.*set_sensor)(robot.name(), sensorsBuffer_);
  };
  updateSensor(&GC::setEncoderValues, state_.q);
  updateSensor(&GC::setEncoderVelocities, state_.dq);
  updateSensor(&GC::setJointTorques, state_.tau_J);
  auto wrench = sva::ForceVecd::Zero();
  wrench.force().x() = state_.K_F_ext_hat_K[0];
  wrench.force().y() = state_.K_F_ext_hat_K[1];
  wrench.force().z() = state_.K_F_ext_hat_K[2];
  wrench.couple().x() = state_.K_F_ext_hat_K[3];
  wrench.couple().y() = state_.K_F_ext_hat_K[4];
  wrench.couple().z() = state_.K_F_ext_hat_K[5];
  robot.data()->forceSensors[robot.data()->forceSensorsIndex.at("LeftHandForceSensor")].wrench(wrench);
  device_.state(state_);
}

inline void PandaObserveLoop::observeThread(bool & running)
{
  using namespace std::chrono;
  auto period = duration_cast<clock::duration>(duration<double>(poll_period_s_));
  auto next = clock::now() + period;
  while(running)
  {
    franka::RobotState s = robot_.readOnce();
    {
      std::lock_guard<std::mutex> lk(stateMutex_);
      state_ = s;
    }
    std::this_thread::sleep_until(next);
    next += period;
  }
}

} // namespace mc_franka
