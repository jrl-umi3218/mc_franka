#include <franka/robot.h>
#include <franka/robot_state.h>
#include <motion_generator_traits.h>
#include <network.h>
#include <robot_impl.h>

#include <iostream>
#include <string>

int main(int argc, char * argv[])
{
  try{
    std::cout << "MCFrankaControlMultiTest" << std::endl;
    
    std::cout << "argc = " << argc << std::endl; //=number_of_pandas-1
    std::string ip1 = "";
    std::string ip2 = "";
    const int numPandas = argc-1;
    for(int i=1; i<argc; i++)
    {
      std::cout << "argv[" << i <<"] = " << argv[i] << std::endl; //e.g. 172.16.0.2, 172.16.1.2, ...
    }
    
    if(numPandas<=0){
      std::cerr << "you need to specify an ip address" << std::endl;
      throw;
    }
    if(numPandas==1){
      ip1 = argv[1];
    }
    if(numPandas==2){
      ip1 = argv[1];
      ip2 = argv[2];
    }
    if(numPandas>=3){
      std::cerr << "three pandas not civered yet" << std::endl;
    }

    std::unique_ptr<franka::Robot::Impl> franka_control1;
    std::unique_ptr<franka::Robot::Impl> franka_control2;
    uint32_t motion_id1 = 0;
    uint32_t motion_id2 = 0;
    franka::RobotState franka_state1;
    franka::RobotState franka_state2;
    static constexpr research_interface::robot::Move::Deviation kDefaultDeviation {10.0, 3.12, 2 * M_PI};
    research_interface::robot::MotionGeneratorCommand motion_command;
    motion_command.q_c.fill(0);
    motion_command.dq_c.fill(0);
    motion_command.O_T_EE_c.fill(0);
    motion_command.O_dP_EE_c.fill(0);
    motion_command.elbow_c.fill(0);

    franka_control1 = std::make_unique<franka::Robot::Impl>(std::make_unique<franka::Network>(ip1, research_interface::robot::kCommandPort), 1, franka::RealtimeConfig::kEnforce);
    if(franka_control1 == NULL){
      std::cout << "franka_control1 is null " << std::endl;
      throw;
    }

    if(numPandas==2){
      franka_control2 = std::make_unique<franka::Robot::Impl>(std::make_unique<franka::Network>(ip2, research_interface::robot::kCommandPort), 1, franka::RealtimeConfig::kEnforce);
      if(franka_control2 == NULL){
        std::cout << "franka_control2 is null " << std::endl;
        throw;
      }
    }

    motion_id1 = franka_control1->startMotion(research_interface::robot::Move::ControllerMode::kJointImpedance, 
                                            research_interface::robot::Move::MotionGeneratorMode::kJointVelocity,
                                            kDefaultDeviation,
                                            kDefaultDeviation);
    std::cout << "motion_id1 = " << motion_id1 << std::endl;
    
    if(numPandas==2){
      motion_id2 = franka_control2->startMotion(research_interface::robot::Move::ControllerMode::kJointImpedance, 
                                              research_interface::robot::Move::MotionGeneratorMode::kJointVelocity,
                                              kDefaultDeviation,
                                              kDefaultDeviation);
      std::cout << "motion_id2 = " << motion_id2 << std::endl;
    }

    bool increase = true;
    for(int i=0; i<1000*5; i++){ //execute main loop for 5 seconds
      if(i==800){
        increase = false;
      }
      if(increase){
        motion_command.dq_c[0] += 0.001; //slowly increase velocity for first joint
      }
      else if(motion_command.dq_c[0]>0){
        motion_command.dq_c[0] -= 0.001; //slowly decrease velocity for first joint
      }      

      franka_state1 = franka_control1->update(&motion_command, nullptr);
      if(numPandas==2){
        franka_state2 = franka_control2->update(&motion_command, nullptr);
      }

      if(i % 500 == 0 || i < 70){ //print status every 0.5 second
        if(numPandas==1){
          // std::cout << "iteration " << i << " -> 1. panda q[0] = " << franka_state1.q[0] << "\n";
          std::cout << "iteration " << i << " -> 1. panda rate = " << franka_state1.control_command_success_rate << "\n";
        }
        else if(numPandas==2){
          // std::cout << "iteration " << i << " -> 1. panda q[0] = " << franka_state1.q[0] << " and 2. panda q[0] " << franka_state2.q[0] << "\n";
          std::cout << "iteration " << i << " -> 1. panda rate = " << franka_state1.control_command_success_rate << " and 2. panda rate " << franka_state2.control_command_success_rate << "\n";
        }
      }
    }
    std::cout << "main loop ok" << std::endl;

    franka_control1->finishMotion(motion_id1, &motion_command, nullptr);
    if(numPandas==2){
      franka_control2->finishMotion(motion_id2, &motion_command, nullptr);
    }
    std::cout << "done" << std::endl;
  }
  catch(const franka::Exception & e)
  {
    std::cerr << "franka::Exception " << e.what() << "\n";
    return 1;
  }
  return 0;
}
