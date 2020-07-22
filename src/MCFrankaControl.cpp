#include <mc_control/mc_global_controller.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <franka/exception.h>
#include <franka/robot.h>

#include <chrono>
#include <condition_variable>
#include <iostream>
#include <thread>

namespace mc_time
{

using duration_us = std::chrono::duration<double, std::micro>;
/** Always pick a steady clock */
using clock = typename std::conditional<std::chrono::high_resolution_clock::is_steady,
                                        std::chrono::high_resolution_clock,
                                        std::chrono::steady_clock>::type;

} // namespace mc_time

#include "PandaControlType.h"

std::condition_variable sensors_cv;
std::mutex sensors_mutex;
std::atomic<size_t> sensors_ready(0);
std::condition_variable command_cv;
std::mutex command_mutex;
std::atomic<size_t> command_ready(0);
std::condition_variable start_control_cv;
std::mutex start_control_mutex;
bool start_control_ready = false;

template<ControlMode cm>
struct PandaControlLoop
{
  PandaControlLoop(const std::string & name, const std::string & ip, size_t steps)
  : name(name), robot(ip), state(robot.readOnce()), control(state), steps(steps)
  {
    static auto panda_init_t = mc_time::clock::now();
    auto now = mc_time::clock::now();
    mc_time::duration_us dt = now - panda_init_t;
    mc_rtc::log::info("[mc_franka] Elapsed time since the creation of another PandaControlLoop: {}us", dt.count());
    panda_init_t = now;
  }

  void updateSensors(mc_rbdyn::Robot & robot, mc_rbdyn::Robot & real)
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
    updateSensor(&mc_rbdyn::Robot::encoderValues, &mc_rbdyn::Robot::encoderValues, state.q);
    updateSensor(&mc_rbdyn::Robot::encoderVelocities, &mc_rbdyn::Robot::encoderVelocities, state.dq);
    updateSensor(&mc_rbdyn::Robot::jointTorques, &mc_rbdyn::Robot::jointTorques, state.tau_J);
    command = robot.mbc();
  }

  void updateSensors(mc_control::MCGlobalController & controller)
  {
    auto & robot = controller.controller().robots().robot(name);
    auto & real = controller.controller().realRobots().robot(name);
    updateSensors(robot, real);
  }

  void init(mc_control::MCGlobalController & controller)
  {
    auto & robot = controller.controller().robots().robot(name);
    auto & real = controller.controller().realRobots().robot(name);
    updateSensors(robot, real);
    const auto & rjo = robot.refJointOrder();
    for(size_t i = 0; i < rjo.size(); ++i)
    {
      auto jIndex = robot.jointIndexByName(rjo[i]);
      robot.mbc().q[jIndex][0] = state.q[i];
      robot.mbc().jointTorque[jIndex][0] = state.tau_J[i];
    }
    robot.forwardKinematics();
    real.mbc() = robot.mbc();
  }

  void control_thread(mc_control::MCGlobalController & controller)
  {
    {
      std::unique_lock<std::mutex> start_control_lock(start_control_mutex);
      start_control_cv.wait(start_control_lock, [](){ return start_control_ready; });
    }
    control.control(robot,
                    [ this, &controller ](const franka::RobotState & stateIn, franka::Duration) ->
                    typename PandaControlType<cm>::ReturnT {
                      if(!started)
                      {
                        timespec tv;
                        clock_gettime(CLOCK_REALTIME, &tv);
                        mc_rtc::log::info("[mc_franka] {} control loop started at {}", name, tv.tv_sec + tv.tv_nsec * 1e-9);
                        started = true;
                      }
                      this->state = stateIn;
                      sensor_id += 1;
                      auto & robot = controller.controller().robots().robot(name);
                      auto & real = controller.controller().realRobots().robot(name);
                      if(sensor_id % steps == 0)
                      {
                        sensors_ready++;
                        sensors_cv.notify_one();
                        std::unique_lock<std::mutex> command_lock(command_mutex);
                        command_cv.wait(command_lock, []() { return command_ready > 0; });
                        command_ready--;
                      }
                      if(controller.running)
                      {
                        return control.update(robot, command, sensor_id % steps, steps);
                      }
                      return franka::MotionFinished(control);
                    });
  }

  std::string name;
  franka::Robot robot;
  franka::RobotState state;
  PandaControlType<cm> control;
  size_t sensor_id = 0;
  size_t steps = 1;
  rbd::MultiBodyConfig command;
  bool started = false;
};

template<ControlMode cm>
void global_thread(mc_control::MCGlobalController::GlobalConfiguration & gconfig)
{
  auto frankaConfig = gconfig.config("Franka");
  auto ignoredRobots = frankaConfig("ignored", std::vector<std::string>{});
  mc_control::MCGlobalController controller(gconfig);
  if(controller.controller().timeStep < 0.001)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("mc_rtc cannot run faster than 1kHz with mc_franka");
  }
  size_t n_steps = std::ceil(controller.controller().timeStep / 0.001);
  size_t freq = std::ceil(1 / controller.controller().timeStep);
  mc_rtc::log::info("mc_rtc running at {}Hz, will interpolate every {} panda control step", freq, n_steps);
  auto & robots = controller.controller().robots();
  // Initialize all real robots
  for(size_t i = controller.realRobots().size(); i < robots.size(); ++i)
  {
    controller.realRobots().robotCopy(robots.robot(i));
    controller.realRobots().robots().back().name(robots.robot(i).name());
  }
  // Initialize controlled panda robot
  std::vector<std::pair<PandaControlLoop<cm>, size_t>> pandas;
  {
    std::vector<std::thread> panda_init_threads;
    std::mutex pandas_mutex;
    for(auto & robot : robots)
    {
      if(robot.mb().nrDof() == 0)
      {
        continue;
      }
      if(std::find(ignoredRobots.begin(), ignoredRobots.end(), robot.name()) != ignoredRobots.end())
      {
        continue;
      }
      if(frankaConfig.has(robot.name()))
      {
        std::string ip = frankaConfig(robot.name())("ip");
        panda_init_threads.emplace_back([&robot,&pandas,&pandas_mutex,ip,n_steps]() {
          auto pair = std::make_pair<PandaControlLoop<cm>, size_t>({robot.name(), ip, n_steps}, 0);
          std::lock_guard<std::mutex> lock(pandas_mutex);
          pandas.emplace_back(std::move(pair));
        });
      }
      else
      {
        mc_rtc::log::warning("The loaded controller uses an actuated robot that is not configured and not ignored: {}",
                             robot.name());
      }
    }
    for(auto & th : panda_init_threads)
    {
      th.join();
    }
  }
  for(auto & panda : pandas)
  {
    panda.first.init(controller);
    controller.controller().logger().addLogEntry(panda.first.name + "_sensors_id", [&panda]() { return panda.second; });
  }
  controller.init(robots.robot().encoderValues());
  controller.running = true;
  controller.controller().gui()->addElement(
      {"Franka"}, mc_rtc::gui::Button("Stop controller", [&controller]() { controller.running = false; }));
  // Start panda control loops
  std::vector<std::thread> panda_threads;
  for(auto & panda : pandas)
  {
    panda_threads.emplace_back([&panda, &controller]() { panda.first.control_thread(controller); });
  }
  start_control_ready = true;
  start_control_cv.notify_all();
  size_t iter = 0;
  while(controller.running)
  {
    {
      std::unique_lock<std::mutex> sensors_lock(sensors_mutex);
      bool start_measure = false;
      std::chrono::time_point<mc_time::clock> start;
      sensors_cv.wait(sensors_lock, [&]() {
        auto ready = sensors_ready.load();
        if(!start_measure && ready >= 1)
        {
          start_measure = true;
          start = mc_time::clock::now();
        }
        return ready == pandas.size();
      });
      sensors_ready = 0;
      if(iter++ % 5 * freq == 0 && pandas.size() > 1)
      {
        mc_time::duration_us delay = mc_time::clock::now() - start;
        mc_rtc::log::info("[mc_franka] Measured delay between the pandas: {}us", delay.count());
      }
      for(auto & panda : pandas)
      {
        panda.first.updateSensors(controller);
        panda.second = panda.first.sensor_id;
      }
      command_ready = pandas.size();
      command_cv.notify_all();
    }
    controller.run();
  }
  for(auto & th : panda_threads)
  {
    th.join();
  }
}

template<ControlMode cm>
struct GlobalControlLoop
{
  mc_control::MCGlobalController & controller;
  std::vector<PandaControlLoop<cm>> robots;
};

int main(int argc, char * argv[])
{
  std::string conf_file = "";
  po::options_description desc("MCUDPControl options");
  // clang-format off
  desc.add_options()
    ("help", "Display help message")
    ("conf,f", po::value<std::string>(&conf_file), "Configuration file");
  // clang-format on

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if(vm.count("help"))
  {
    std::cout << desc << "\n";
    std::cout << "see etc/sample.yaml for libfranka configuration\n";
    return 1;
  }

  mc_control::MCGlobalController::GlobalConfiguration gconfig(conf_file, nullptr);
  if(!gconfig.config.has("Franka"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "No Franka section in the configuration, see etc/sample.yaml for an example");
  }
  auto frankaConfig = gconfig.config("Franka");
  ControlMode cm = frankaConfig("ControlMode", ControlMode::Velocity);
  try
  {
    switch(cm)
    {
      case ControlMode::Position:
        global_thread<ControlMode::Position>(gconfig);
        break;
      case ControlMode::Velocity:
        global_thread<ControlMode::Velocity>(gconfig);
        break;
      case ControlMode::Torque:
        global_thread<ControlMode::Torque>(gconfig);
        break;
    }
  }
  catch(const franka::Exception & e)
  {
    std::cerr << "franka::Exception " << e.what() << "\n";
    return 1;
  }
  return 0;
}
