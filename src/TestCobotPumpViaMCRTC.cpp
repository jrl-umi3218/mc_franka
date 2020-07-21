#include <mc_control/mc_global_controller.h>
#include <mc_rtc/logging.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/vacuum_gripper.h>
#include <iostream>

// mc_panda
#include <mc_panda/devices/Pump.h>

namespace {
template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}
}  // anonymous namespace

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

    // Get the initial state of the robot
    const auto state = robot.readOnce();
    const int dof = state.q.size();

    // Initialize mc_rtc
    mc_control::MCGlobalController controller;
    std::vector<double> init_q_vector;
    init_q_vector.resize(dof);
    for(size_t i = 0; i < dof; ++i)
    {
      init_q_vector.at(i)=state.q[i];
    }
    controller.init(init_q_vector);
    controller.running = true;
    
    // Initialize the libfranka VacuumGripper as 'sucker' and the mc_panda Pump as 'pump' if the robot-module has such a device
    std::shared_ptr<franka::VacuumGripper> sucker;
    std::shared_ptr<mc_panda::Pump> pump;
    try{
      sucker = std::make_shared<franka::VacuumGripper>( franka::VacuumGripper(argv[1]) );
      mc_rtc::log::info("Connection established to VacuumGripper via {}", argv[1]);
      
      std::string pumpDeviceName = "Pump";
      if(controller.robot().hasDevice<mc_panda::Pump>(pumpDeviceName))
      {
        pump = std::make_shared<mc_panda::Pump>( controller.robot().device<mc_panda::Pump>(pumpDeviceName) );
        pump->addToLogger(controller.controller().logger());
        mc_rtc::log::info("RobotModule has a Pump named {}", pumpDeviceName);
      }
      else{
        mc_rtc::log::error("RobotModule does not have a Pump named {}", pumpDeviceName);
        mc_rtc::log::error_and_throw<std::runtime_error>("Pump functionality is not available");
      }
    }
    catch(const franka::NetworkException & e)
    {
      mc_rtc::log::error("Cannot connect to VacuumGripper via {}", argv[1]);
      mc_rtc::log::error_and_throw<std::runtime_error>("Pump functionality is not available");
    }

    // Start the sense-actuate loop
    franka::VacuumGripperState stateSucker;
    mc_panda::NextCommand nc;
    bool looping = true;
    while(looping)
    {
      //forward sucker sensor signals to mc_rtc
      stateSucker = sucker->readOnce();
      pump->set_in_control_range(stateSucker.in_control_range);
      pump->set_part_detached(stateSucker.part_detached);
      pump->set_part_present(stateSucker.part_present);
      if(stateSucker.device_status==franka::VacuumGripperDeviceStatus::kGreen){
        pump->set_device_status_ok(true);
      }
      else{
        pump->set_device_status_ok(false);
      }
      pump->set_actual_power(stateSucker.actual_power);
      pump->set_vacuum(stateSucker.vacuum);

      //receive sucker actuator commands from mc_rtc
      nc = pump->nextCommandRequested();
      switch(nc)
      {
        case mc_panda::NextCommand::None:
        {
          break;
        }
        case mc_panda::NextCommand::Vacuum:
        {
          uint8_t vacuum;
          std::chrono::milliseconds timeout;
          pump->getVacuumCommandParams(vacuum, timeout);
          const bool vacuumOK = sucker->vacuum(vacuum, timeout);
          pump->setVacuumCommandResult(vacuumOK);
          mc_rtc::log::info("vacuum command applied with the params vacuum {} and timeout {}, result: {}", std::to_string(vacuum), std::to_string(timeout.count()), vacuumOK);
          break;
        }
        case mc_panda::NextCommand::Dropoff:
        {
          std::chrono::milliseconds timeout;
          pump->getDropoffCommandParam(timeout);
          const bool dropoffOK = sucker->dropOff(timeout);
          pump->setDropoffCommandResult(dropoffOK);
          mc_rtc::log::info("dropoff command applied with the param timeout {}, result: {}", std::to_string(timeout.count()), dropoffOK);
          break;
        }
        case mc_panda::NextCommand::Stop:
        {
          const bool stopOK = sucker->stop();
          pump->setStopCommandResult(stopOK);
          mc_rtc::log::info("stop command applied, result: {}", stopOK);
          looping = false;
          break;
        }
        default:
        {
          mc_rtc::log::error_and_throw<std::runtime_error>("next command has unexpected value");
        }
      }
    }
    robot.stop();
  }
  catch(const franka::Exception & e)
  {
    std::cerr << "franka::Exception " << e.what() << "\n";
    // return 1;
  }
  return 0;
}
