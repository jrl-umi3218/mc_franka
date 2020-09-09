/* Copyright 2020 mc_rtc development team */

#pragma once

#include "PandaControlType.h"
#include "defs.h"

#include <mc_panda/devices/Pump.h>
#include <mc_panda/devices/Robot.h>

#include <mc_control/mc_global_controller.h>

#include <condition_variable>
#include <thread>

namespace mc_franka
{

/** The PandaControlLoop implements two things:
 *
 * 1. Update mc_rtc sensor information from the panda state provided by
 * libfranka
 *
 * 2. Use mc_rtc output to send relevant control information to the panda
 */
template<ControlMode cm>
struct PandaControlLoop
{
  /** Constructor
   *
   * \param name Identifier of the robot
   *
   * \param ip IP of the robot
   *
   * \param steps How often the robot should get updated, 1 means every step, 2
   * means every 2 steps...
   *
   * \param device PandaDevice associated to the robot
   *
   * \param pump Pump associated to the robot (nullptr if none)
   */
  PandaControlLoop(const std::string & name,
                   const std::string & ip,
                   size_t steps,
                   mc_panda::Robot & device,
                   mc_panda::Pump * pump);

  /** Initialize mc_rtc robot from the current state */
  void init(mc_control::MCGlobalController & controller);

  /** Update sensors in mc_rtc instance */
  void updateSensors(mc_control::MCGlobalController & controller);

  /** Update command from mc_rtc output */
  void updateControl(mc_control::MCGlobalController & controller);

  /** Start the libfranka control loop
   *
   * The start is synchronized with other panda robots
   *
   * \param controller Controller that is running this loop
   *
   * \param startM Mutex to the start variable
   *
   * \param startCV Condition variable used to trigger start
   *
   * \param start Monitoring value
   *
   * \param running True while mc_rtc is running
   */
  void controlThread(mc_control::MCGlobalController & controller,
                     std::mutex & startM,
                     std::condition_variable & startCV,
                     bool & start,
                     bool & running);

private:
  std::string name_;
  franka::Robot robot_;
  franka::RobotState state_;
  PandaControlType<cm> control_;
  mc_panda::Robot & device_;
  size_t steps_ = 1;
  mc_rtc::Logger logger_;
  size_t sensor_id_ = 0;
  rbd::MultiBodyConfig command_;
  size_t control_id_ = 0;
  size_t prev_control_id_ = 0;
  double delay_ = 0;

  mutable std::mutex updateSensorsMutex_;
  mutable std::mutex updateControlMutex_;

  /** Actual update for the sensors */
  void updateSensors(mc_rbdyn::Robot & robot, mc_rbdyn::Robot & real);
};

template<ControlMode cm>
using PandaControlLoopPtr = std::unique_ptr<PandaControlLoop<cm>>;

template<ControlMode cm>
PandaControlLoop<cm>::PandaControlLoop(const std::string & name,
                                       const std::string & ip,
                                       size_t steps,
                                       mc_panda::Robot & device,
                                       mc_panda::Pump * pump)
: name_(name), robot_(ip), state_(robot_.readOnce()), control_(state_), device_(device), steps_(steps),
  logger_(mc_rtc::Logger::Policy::THREADED, "/tmp", "mc-franka-" + name_)
{
  static auto panda_init_t = clock::now();
  auto now = clock::now();
  duration_us dt = now - panda_init_t;
  mc_rtc::log::info("[mc_franka] Elapsed time since the creation of another PandaControlLoop: {}us", dt.count());
  if(pump)
  {
    pump->connect(ip);
    pump->addToLogger(logger_, name);
  }
  device.connect(&robot_);
  device.addToLogger(logger_, name);
}

template<ControlMode cm>
void PandaControlLoop<cm>::init(mc_control::MCGlobalController & controller)
{
  logger_.start(controller.current_controller(), 0.001);
  logger_.addLogEntry("sensors_id", [this]() { return sensor_id_; });
  logger_.addLogEntry("prev_control_id", [this]() { return prev_control_id_; });
  logger_.addLogEntry("control_id", [this]() { return control_id_; });
  logger_.addLogEntry("delay", [this]() { return delay_; });
  updateSensors(controller);
  updateControl(controller);
  prev_control_id_ = control_id_;
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

template<ControlMode cm>
void PandaControlLoop<cm>::updateSensors(mc_control::MCGlobalController & controller)
{
  std::unique_lock<std::mutex> lock(updateSensorsMutex_);
  auto & robot = controller.controller().robots().robot(name_);
  auto & real = controller.controller().realRobots().robot(name_);
  updateSensors(robot, real);
}

template<ControlMode cm>
void PandaControlLoop<cm>::updateControl(mc_control::MCGlobalController & controller)
{
  std::unique_lock<std::mutex> lock(updateControlMutex_);
  auto & robot = controller.controller().robots().robot(name_);
  command_ = robot.mbc();
  control_id_++;
}

template<ControlMode cm>
void PandaControlLoop<cm>::controlThread(mc_control::MCGlobalController & controller,
                                         std::mutex & startM,
                                         std::condition_variable & startCV,
                                         bool & start,
                                         bool & running)
{
  {
    std::unique_lock<std::mutex> lock(startM);
    startCV.wait(lock, [&]() { return start; });
  }
  auto start_t = clock::now();
  control_.control(
      robot_,
      [&, this ](const franka::RobotState & stateIn, franka::Duration dt) -> typename PandaControlType<cm>::ReturnT {
        std::unique_lock<std::mutex> ctlLock(updateControlMutex_);
        std::unique_lock<std::mutex> senLock(updateSensorsMutex_);
        auto now = clock::now();
        delay_ = duration_ms(now - start_t).count();
        start_t = now;
        state_ = stateIn;
        sensor_id_ += dt.toMSec();
        if(sensor_id_ % steps_ == 0)
        {
          if(control_id_ != prev_control_id_ + dt.toMSec())
          {
            mc_rtc::log::warning("[mc_franka] {} missed control data (previous: {}, current: {}, expected: {}", name_,
                                 prev_control_id_, control_id_, prev_control_id_ + dt.toMSec());
          }
          prev_control_id_ = control_id_;
        }
        if(running)
        {
          logger_.log();
          auto & robot = controller.controller().robots().robot(name_);
          return control_.update(robot, command_, sensor_id_ % steps_, steps_);
        }
        return franka::MotionFinished(control_);
      });
}

template<ControlMode cm>
void PandaControlLoop<cm>::updateSensors(mc_rbdyn::Robot & robot, mc_rbdyn::Robot & real)
{
  using get_sensor_t = const std::vector<double> & (mc_rbdyn::Robot::*)() const;
  using set_sensor_t = void (mc_rbdyn::Robot::*)(const std::vector<double> &);
  auto updateSensor = [&](get_sensor_t get, set_sensor_t set, const std::array<double, 7> & value) {
    auto sensor = (robot.*get)();
    if(sensor.size() != robot.refJointOrder().size())
    {
      sensor.resize(robot.refJointOrder().size());
    }
    for(size_t i = 0; i < value.size(); ++i)
    {
      sensor[i] = value[i];
    }
    (robot.*set)(sensor);
    (real.*set)(sensor);
  };
  updateSensor(&mc_rbdyn::Robot::encoderValues, &mc_rbdyn::Robot::encoderValues, state_.q);
  updateSensor(&mc_rbdyn::Robot::encoderVelocities, &mc_rbdyn::Robot::encoderVelocities, state_.dq);
  updateSensor(&mc_rbdyn::Robot::jointTorques, &mc_rbdyn::Robot::jointTorques, state_.tau_J);
  auto wrench = sva::ForceVecd::Zero();
  wrench.force().x() = state_.K_F_ext_hat_K[0];
  wrench.force().y() = state_.K_F_ext_hat_K[1];
  wrench.force().z() = state_.K_F_ext_hat_K[2];
  wrench.couple().x() = state_.K_F_ext_hat_K[3];
  wrench.couple().y() = state_.K_F_ext_hat_K[4];
  wrench.couple().z() = state_.K_F_ext_hat_K[5];
  robot.forceSensor("LeftHandForceSensor").wrench(wrench);
  real.forceSensor("LeftHandForceSensor").wrench(wrench);
  device_.state(state_);
}

} // namespace mc_franka
