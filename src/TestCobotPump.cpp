#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/vacuum_gripper.h>

#include <iostream>

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
    robot.setJointImpedance({{100,100,100,100,100,100,100}}); //TODO which values to choose?
    robot.setCartesianImpedance({{100,100,100,10,10,10}}); //TODO which values to choose?
    robot.setCollisionBehavior( //values taken from https://github.com/frankaemika/libfranka/blob/master/examples/generate_joint_velocity_motion.cpp#L39
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    
    //TODO: be careful with this mode!
    robot.setCollisionBehavior(
        {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}}, {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}},
        {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}}, {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}},
        {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}}, {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}},
        {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}}, {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}});

    // Get the initial state of the robot
    const franka::RobotState stateRobot = robot.readOnce();
    const int dof = stateRobot.q.size();

    // Get the initial state of the sucker
    franka::VacuumGripper sucker(argv[1]);
    const franka::VacuumGripperState stateSucker = sucker.readOnce();
    std::cout << "Initial vacuum gripper state: " << stateSucker << std::endl;
    bool icr = stateSucker.in_control_range;
    bool pd = stateSucker.part_detached;
    bool pp = stateSucker.part_present;
    franka::VacuumGripperDeviceStatus ds = stateSucker.device_status;
    uint16_t ap = stateSucker.actual_power;
    uint16_t v = stateSucker.vacuum;
    std::cout << "icr: " << icr << std::endl;
    std::cout << "pd: " << pd << std::endl;
    std::cout << "pp: " << pp << std::endl;
    if(ds == franka::VacuumGripperDeviceStatus::kGreen){
      std::cout << "sucker state: Device is working optimally" << std::endl;
    }
    else{
      std::cout << "sucker state: not good..." << std::endl;
    }
    std::cout << "ap: " << ap << std::endl;
    std::cout << "v: " << v << std::endl;


    // Start the control loop in velocity-control
    franka::JointVelocities output_dq(stateRobot.dq);
    for(size_t i = 0; i < output_dq.dq.size(); ++i)
    {
      output_dq.dq[i] = 0.0;
    }
    int counter = 0;
    bool grasped=false;
    while(true)
    // robot.control([&sucker,&grasped,&counter,&output_dq](const franka::RobotState & stateRobot, franka::Duration) -> franka::JointVelocities
    {
      std::cout << "counter: " << counter << std::endl;

      const franka::VacuumGripperState stateSucker = sucker.readOnce();

      // double numPrintoutsPerSecond = 2;
      // int randomNumber = rand() % int(1000.0 / numPrintoutsPerSecond) + 1; //generate number between 1 and 1000
      // if (randomNumber==int(1000.0 / numPrintoutsPerSecond)){
        std::cout << "stateSucker.vacuum: " << stateSucker.vacuum << std::endl;
        std::cout << "stateSucker.in_control_range: " << stateSucker.in_control_range << std::endl;
        std::cout << "stateSucker.part_detached: " << stateSucker.part_detached << std::endl;
        std::cout << "stateSucker.part_present: " << stateSucker.part_present << std::endl;
      // }

      counter++;
      if(counter==30){
        uint8_t vacuumBar = 100; //unit [10*mbar]
        std::chrono::milliseconds timeoutV(2000); //unit ms
        bool okV = sucker.vacuum(vacuumBar, timeoutV);
        if(okV==false){
          std::cout << "Failed to vacuum the object." << std::endl;
        }
        else{
          grasped=true;
        }
      }

      if(counter==130 && stateSucker.in_control_range){
        std::chrono::milliseconds timeoutD(1000); //unit ms
        bool okD = sucker.dropOff(timeoutD);
        if(okD==false){
          std::cout << "Failed to dropOff the object." << std::endl;
        }
        else{
          grasped=false;
        }
      }

      if(counter==135)
      {
        break;
      }
    }
    //   if(true)
    //   {
    //     return output_dq;
    //   }
    //   return franka::MotionFinished(output_dq);
    // // }, franka::ControllerMode::kJointImpedance, true, 100); //default parameters
    // }, franka::ControllerMode::kJointImpedance, true, 1000); //increased kDefaultCutoffFrequency 

    sucker.stop();
    robot.stop();
  }
  catch(const franka::Exception & e)
  {
    std::cerr << "franka::Exception " << e.what() << "\n";
    return 1;
  }
  return 0;
}
