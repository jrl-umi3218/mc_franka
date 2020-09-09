/* Copyright 2020 mc_rtc development team */

#include "PandaControlLoop.h"

#include <mc_panda/devices/Pump.h>
#include <mc_panda/devices/Robot.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

namespace mc_franka
{

struct ControlLoopDataBase
{
  ControlLoopDataBase(ControlMode cm) : mode(cm), controller(nullptr), panda_threads(nullptr) {}
  ControlMode mode;
  mc_control::MCGlobalController * controller;
  std::vector<std::thread> * panda_threads;
};

template<ControlMode cm>
struct ControlLoopData : public ControlLoopDataBase
{
  ControlLoopData() : ControlLoopDataBase(cm), pandas(nullptr) {}
  std::vector<PandaControlLoopPtr<cm>> * pandas;
};

template<ControlMode cm>
void * global_thread_init(mc_control::MCGlobalController::GlobalConfiguration & gconfig)
{
  auto frankaConfig = gconfig.config("Franka");
  auto ignoredRobots = frankaConfig("ignored", std::vector<std::string>{});
  auto loop_data = new ControlLoopData<cm>();
  loop_data->controller = new mc_control::MCGlobalController(gconfig);
  loop_data->panda_threads = new std::vector<std::thread>();
  auto & controller = *loop_data->controller;
  if(controller.controller().timeStep < 0.001)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_franka] mc_rtc cannot run faster than 1kHz with mc_franka");
  }
  size_t n_steps = std::ceil(controller.controller().timeStep / 0.001);
  size_t freq = std::ceil(1 / controller.controller().timeStep);
  mc_rtc::log::info("[mc_franka] mc_rtc running at {}Hz, will interpolate every {} panda control step", freq, n_steps);
  auto & robots = controller.controller().robots();
  // Initialize all real robots
  for(size_t i = controller.realRobots().size(); i < robots.size(); ++i)
  {
    controller.realRobots().robotCopy(robots.robot(i), robots.robot(i).name());
  }
  // Initialize controlled panda robot
  loop_data->pandas = new std::vector<PandaControlLoopPtr<cm>>();
  auto & pandas = *loop_data->pandas;
  {
    std::vector<std::thread> panda_init_threads;
    std::mutex pandas_init_mutex;
    std::condition_variable pandas_init_cv;
    bool pandas_init_ready = false;
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
        panda_init_threads.emplace_back([&, ip]() {
          {
            std::unique_lock<std::mutex> lock(pandas_init_mutex);
            pandas_init_cv.wait(lock, [&pandas_init_ready]() { return pandas_init_ready; });
          }
          auto pump = mc_panda::Pump::get(robot);
          auto & device = *mc_panda::Robot::get(robot);
          auto panda =
              std::unique_ptr<PandaControlLoop<cm>>(new PandaControlLoop<cm>(robot.name(), ip, n_steps, device, pump));
          device.addToLogger(controller.controller().logger(), robot.name());
          if(pump)
          {
            pump->addToLogger(controller.controller().logger(), robot.name());
          }
          std::unique_lock<std::mutex> lock(pandas_init_mutex);
          pandas.emplace_back(std::move(panda));
        });
      }
      else
      {
        mc_rtc::log::warning("The loaded controller uses an actuated robot that is not configured and not ignored: {}",
                             robot.name());
      }
    }
    pandas_init_ready = true;
    pandas_init_cv.notify_all();
    for(auto & th : panda_init_threads)
    {
      th.join();
    }
  }
  for(auto & panda : pandas)
  {
    panda->init(controller);
  }
  controller.init(robots.robot().encoderValues());
  controller.running = true;
  controller.controller().gui()->addElement(
      {"Franka"}, mc_rtc::gui::Button("Stop controller", [&controller]() { controller.running = false; }));
  // Start panda control loops
  static std::mutex startMutex;
  static std::condition_variable startCV;
  static bool startControl = false;
  for(auto & panda : pandas)
  {
    loop_data->panda_threads->emplace_back(
        [&]() { panda->controlThread(controller, startMutex, startCV, startControl, controller.running); });
  }
  startControl = true;
  startCV.notify_all();
  return loop_data;
}

template<ControlMode cm>
void run_impl(void * data)
{
  auto control_data = static_cast<ControlLoopData<cm> *>(data);
  auto controller_ptr = control_data->controller;
  auto & controller = *controller_ptr;
  auto & pandas = *control_data->pandas;
  timespec tv;
  clock_gettime(CLOCK_REALTIME, &tv);
  // Current time in milliseconds
  double current_t = tv.tv_sec * 1000 + tv.tv_nsec * 1e-6;
  // Will record the time that passed between two runs
  double elapsed_t = 0;
  controller.controller().logger().addLogEntry("mc_franka_delay", [&elapsed_t]() { return elapsed_t; });
  while(controller.running)
  {
    clock_gettime(CLOCK_REALTIME, &tv);
    elapsed_t = tv.tv_sec * 1000 + tv.tv_nsec * 1e-6 - current_t;
    current_t = elapsed_t + current_t;
    // Update from panda sensors
    for(auto & panda : pandas)
    {
      panda->updateSensors(controller);
    }
    // Run the controller
    controller.run();
    // Update panda commands
    for(auto & panda : pandas)
    {
      panda->updateControl(controller);
    }
    // Sleep until the next cycle
    sched_yield();
  }
  for(auto & th : *control_data->panda_threads)
  {
    th.join();
  }
  delete control_data->pandas;
  delete controller_ptr;
}

void run(void * data)
{
  auto control_data = static_cast<ControlLoopDataBase *>(data);
  switch(control_data->mode)
  {
    case ControlMode::Position:
      return run_impl<ControlMode::Position>(data);
    case ControlMode::Velocity:
      return run_impl<ControlMode::Velocity>(data);
    case ControlMode::Torque:
      return run_impl<ControlMode::Torque>(data);
  }
}

void * init(int argc, char * argv[], uint64_t & cycle_ns)
{
  std::string conf_file = "";
  po::options_description desc("MCFrankaControl options");
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
    return nullptr;
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
        return global_thread_init<ControlMode::Position>(gconfig);
      case ControlMode::Velocity:
        return global_thread_init<ControlMode::Velocity>(gconfig);
      case ControlMode::Torque:
        return global_thread_init<ControlMode::Torque>(gconfig);
    }
  }
  catch(const franka::Exception & e)
  {
    std::cerr << "franka::Exception " << e.what() << "\n";
    return nullptr;
  }
}

} // namespace mc_franka
