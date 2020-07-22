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
    controller.controller().gui()->addElement({"Franka"},
                                              mc_rtc::gui::Button("Stop controller", [&controller]() { controller.running = false; }));

    // Initialize the libfranka VacuumGripper as 'sucker' and the mc_panda Pump as 'pump' if the robot-module has such a device
    std::shared_ptr<franka::VacuumGripper> sucker;
    std::string pumpDeviceName = "Pump";
    try{
      sucker = std::make_shared<franka::VacuumGripper>( franka::VacuumGripper(argv[1]) );
      mc_rtc::log::info("Connection established to VacuumGripper via {}", argv[1]);
      sucker->stop();
      if(controller.robot().hasDevice<mc_panda::Pump>(pumpDeviceName))
      {
        controller.robot().device<mc_panda::Pump>(pumpDeviceName).addToLogger(controller.controller().logger());
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
    auto & pumpDevice = controller.robot().device<mc_panda::Pump>(pumpDeviceName);
    franka::VacuumGripperState stateSucker;
    mc_panda::NextPumpCommand nc;
    bool looping = true;
    int counter=0;
    while(looping)
    {
      counter++;
      mc_rtc::log::info("loop number {}", counter);
      controller.setEncoderValues(init_q_vector);
      controller.run();

      //forward sucker sensor signals to mc_rtc
      stateSucker = sucker->readOnce();
      pumpDevice.set_in_control_range(stateSucker.in_control_range);
      pumpDevice.set_part_detached(stateSucker.part_detached);
      pumpDevice.set_part_present(stateSucker.part_present);
      if(stateSucker.device_status==franka::VacuumGripperDeviceStatus::kGreen){
        pumpDevice.set_device_status_ok(true);
      }
      else{
        pumpDevice.set_device_status_ok(false);
      }
      pumpDevice.set_actual_power(stateSucker.actual_power);
      pumpDevice.set_vacuum(stateSucker.vacuum);

      //receive sucker actuator commands from mc_rtc
      nc = pumpDevice.NextPumpCommandRequested();
      switch(nc)
      {
        case mc_panda::NextPumpCommand::None:
        {
          mc_rtc::log::info("no command requested");
          break;
        }
        case mc_panda::NextPumpCommand::Vacuum:
        {
          mc_rtc::log::info("vacuum command requested");
          uint8_t vacuum;
          std::chrono::milliseconds timeout;
          pumpDevice.getVacuumCommandParams(vacuum, timeout);
          const bool vacuumOK = sucker->vacuum(vacuum, timeout);
          pumpDevice.setVacuumCommandResult(vacuumOK);
          mc_rtc::log::info("vacuum command applied with the params vacuum {} and timeout {}, result: {}", std::to_string(vacuum), std::to_string(timeout.count()), vacuumOK);
          break;
        }
        case mc_panda::NextPumpCommand::Dropoff:
        {
          mc_rtc::log::info("dropoff command requested");
          std::chrono::milliseconds timeout;
          pumpDevice.getDropoffCommandParam(timeout);
          const bool dropoffOK = sucker->dropOff(timeout);
          pumpDevice.setDropoffCommandResult(dropoffOK);
          mc_rtc::log::info("dropoff command applied with the param timeout {}, result: {}", std::to_string(timeout.count()), dropoffOK);
          break;
        }
        case mc_panda::NextPumpCommand::Stop:
        {
          mc_rtc::log::info("stop command requested");
          const bool stopOK = sucker->stop();
          pumpDevice.setStopCommandResult(stopOK);
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
