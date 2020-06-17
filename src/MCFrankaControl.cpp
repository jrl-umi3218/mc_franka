#include "Controller.hpp"
#include <mc_control/mc_global_controller.h>
// #include "InterfaceMCRTC.hpp"

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
    std::vector<double> q_vector;
    std::vector<double> dq_vector;
    std::vector<double> tau_vector;
    q_vector.resize(7);
    dq_vector.resize(7);
    tau_vector.resize(7);
    std::array<double, 7> q_array;
    std::array<double, 7> dq_array;
    std::array<double, 7> tau_array;
    std::map<std::string, sva::ForceVecd> wrenches;
    sva::ForceVecd wrench = sva::ForceVecd(Eigen::Vector6d::Zero());
    Eigen::Vector3d force = Eigen::Vector3d::Zero();
    Eigen::Vector3d moment = Eigen::Vector3d::Zero();
    std::vector<double> cmd_vector;
    cmd_vector.resize(7);
    std::array<double, 7> cmd_array;
    std::vector<double> init_q_vector;
    init_q_vector.resize(7);
    std::map<std::string, std::shared_ptr<Controller> >::iterator it;
    std::map<std::string, std::shared_ptr<Controller> > robos;
    int dof;
    double control_command_success_rate;
    int counter=0;
    bool ok;

    //init ##########################################################
    std::string robName1 = "leftRobot";
    dof = 7;
    char * ip1 = argv[1];
    robos[robName1] = std::make_shared<Controller>(std::make_unique<franka::Robot::Impl>(std::make_unique<franka::Network>(ip1, research_interface::robot::kCommandPort),1, franka::RealtimeConfig::kEnforce));
    if(argc == 3)
    {
      std::string robName2 = "rightRobot";
      dof = 14;
      char * ip2 = argv[2];
      robos[robName2] = std::make_shared<Controller>(std::make_unique<franka::Robot::Impl>(std::make_unique<franka::Network>(ip2, research_interface::robot::kCommandPort),1, franka::RealtimeConfig::kEnforce));
    }

    //setup mcrtc ##########################################################
    // InterfaceMCRTC mcrtc = InterfaceMCRTC(dof);
    mc_control::MCGlobalController mcrtcControl;
    if(mcrtcControl.controller().timeStep != 0.001)
    {
      LOG_ERROR_AND_THROW(std::runtime_error, "mc_rtc must be configured to run at 1kHz");
    }
    mcrtcControl.running = true;
    mcrtcControl.controller().gui()->addElement({"Franka"},
                                              mc_rtc::gui::Button("Stop controller", [&mcrtcControl]() { mcrtcControl.running = false; }));


    //start libfranka ##########################################################
    counter=0;
    for (it = robos.begin(); it != robos.end(); it++)
    {
      franka::RobotState initState = it->second->getInitialState();
      for(size_t i = 0; i < 7; ++i)
      {
        init_q_vector.at(counter) = initState.q[i];
        counter++;
      }

      // ok = it->second->setControlMode("Position");
      ok = it->second->start();
    }
    mcrtcControl.init(init_q_vector);
    const std::vector<std::string> & rjo = mcrtcControl.robot().refJointOrder();
    if(rjo.size() != dof)
    {
      LOG_ERROR_AND_THROW(std::runtime_error, "mc_rtc-model has " << rjo.size() << " joints, the real robot system has " << dof << " joints");
    }

    //main loop ##########################################################
    while(true)
    {
      //get and set state ##########################################################
      counter=0;
      wrenches.clear();
      for (it = robos.begin(); it != robos.end(); it++)
      {
        control_command_success_rate = it->second->getState(q_array,dq_array,tau_array,force,moment);
        for(size_t i = 0; i < 7; ++i)
        {
          q_vector.at(counter)=q_array[i];
          dq_vector.at(counter)=dq_array[i];
          tau_vector.at(counter)=tau_array[i];
          counter++;
        }
        wrench = sva::ForceVecd(moment,force);
        wrenches.insert(std::make_pair(it->first, wrench)); //TODO replace clear() and insert()
      }
      // mcrtc.setState(q_vector,dq_vector,tau_vector,wrenches);
      mcrtcControl.setEncoderValues(q_vector);
      mcrtcControl.setEncoderVelocities(dq_vector);
      mcrtcControl.setJointTorques(tau_vector); //TODO please double check
      mcrtcControl.setWrenches(wrenches);


      //get and set command ##########################################################
      // cmd_vector = mcrtc.getPositionCommand(); //TODO add velocity and torque command
      if(mcrtcControl.running && mcrtcControl.run())
      {
        for(size_t id = 0; id < dof; ++id)
        {
          const std::string & jointName = rjo[id];
          cmd_vector.at(id) = mcrtcControl.robot().mbc().q[mcrtcControl.robot().jointIndexByName(jointName)][0]; //TODO add velocity and torque command
        }
        counter=0;
        for (it = robos.begin(); it != robos.end(); it++)
        {
          for(size_t i = 0; i < 7; ++i)
          {
            cmd_array[i] = cmd_vector.at(counter);
            counter++;
          }
          ok = it->second->setCommand(cmd_array);
        }
      }
      else
      {
        //stop libfranka ##########################################################
        for (it = robos.begin(); it != robos.end(); it++)
        {
          ok = it->second->stop();
        }
        LOG_ERROR_AND_THROW(std::runtime_error, "mc_rtc not running, shutting down");
      }
    }



  //   // #################################################################################
  //   // Initialize the robot
  //   std::unique_ptr<franka::Robot> robotPtr1;
  //   std::unique_ptr<franka::Robot> robotPtr2;
  //   robotPtr1.reset(new franka::Robot(argv[1]));
  //   robotPtr1->setJointImpedance({{100,100,100,100,100,100,100}}); //TODO which values to choose?
  //   robotPtr1->setCartesianImpedance({{100,100,100,10,10,10}}); //TODO which values to choose?
  //   robotPtr1->setCollisionBehavior( //values taken from https://github.com/frankaemika/libfranka/blob/master/examples/generate_joint_velocity_motion.cpp#L39
  //       {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
  //       {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
  //       {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
  //       {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
  //   if(argc == 3)
  //   {
  //     robotPtr2.reset(new franka::Robot(argv[2]));
  //     // robotPtr2->setCollisionBehavior(); 
  //   }

  //   // Get the initial state of the robot
  //   std::unique_ptr<franka::RobotState> statePtr1;
  //   std::unique_ptr<franka::RobotState> statePtr2;
  //   statePtr1.reset(new franka::RobotState(robotPtr1->readOnce()));
  //   if(argc == 3)
  //   {
  //     statePtr2.reset(new franka::RobotState(robotPtr2->readOnce()));
  //   }

  //   // Initialize mc_rtc
  //   mc_control::MCGlobalController mcrtcControl;
  //   if(mcrtcControl.controller().timeStep != 0.001)
  //   {
  //     LOG_ERROR_AND_THROW(std::runtime_error, "mc_rtc must be configured to run at 1kHz");
  //   }

  //   std::vector<double> initq;
  //   std::vector<double> initdq;
  //   if(mcrtcControl.robot().refJointOrder().size() < statePtr1->q.size())
  //   {
  //     LOG_ERROR_AND_THROW(std::runtime_error, "mc_rtc-model has " << mcrtcControl.robot().refJointOrder().size() << " joints, the real robot has " << statePtr1->q.size() << " joints");
  //   }
  //   for(size_t i = 0; i < statePtr1->q.size(); ++i)
  //   {
  //     initq.push_back(statePtr1->q[i]);
  //     initdq.push_back(statePtr1->dq[i]);
  //   }
  //   if(argc == 3)
  //   {
  //     if(mcrtcControl.robot().refJointOrder().size() < statePtr1->q.size()+statePtr2->q.size())
  //     {
  //       LOG_ERROR_AND_THROW(std::runtime_error, "mc_rtc-model has " << mcrtcControl.robot().refJointOrder().size() << " joints, the real robot has " << statePtr1->q.size()+statePtr2->q.size() << " joints");
  //     }
  //     for(size_t i = 0; i < statePtr2->q.size(); ++i)
  //     {
  //       initq.push_back(statePtr2->q[i]);
  //       initdq.push_back(statePtr2->dq[i]);
  //     }
  //   }

  //   std::map<std::string, sva::ForceVecd> wrenches;
  //   sva::ForceVecd wrench1 = sva::ForceVecd(Eigen::Vector6d::Zero());
  //   sva::ForceVecd wrench2 = sva::ForceVecd(Eigen::Vector6d::Zero());
  //   wrench1.force().x() = statePtr1->K_F_ext_hat_K[0]; //TODO use: statePtr1->K_F_ext_hat_K[] or statePtr1->O_F_ext_hat_K[] ?
  //   wrench1.force().y() = statePtr1->K_F_ext_hat_K[1];
  //   wrench1.force().z() = statePtr1->K_F_ext_hat_K[2];
  //   wrench1.moment().x() = statePtr1->K_F_ext_hat_K[3];
  //   wrench1.moment().y() = statePtr1->K_F_ext_hat_K[4];
  //   wrench1.moment().z() = statePtr1->K_F_ext_hat_K[5];
  //   wrenches.insert(std::make_pair("LeftHandForceSensor", wrench1));
  //   if(argc == 3)
  //   {
  //     wrench2.force().x() = statePtr1->K_F_ext_hat_K[0]; //TODO use: statePtr1->K_F_ext_hat_K[] or statePtr1->O_F_ext_hat_K[] ?
  //     wrench2.force().y() = statePtr1->K_F_ext_hat_K[1];
  //     wrench2.force().z() = statePtr1->K_F_ext_hat_K[2];
  //     wrench2.moment().x() = statePtr1->K_F_ext_hat_K[3];
  //     wrench2.moment().y() = statePtr1->K_F_ext_hat_K[4];
  //     wrench2.moment().z() = statePtr1->K_F_ext_hat_K[5];
  //     wrenches.insert(std::make_pair("RightHandForceSensor", wrench2));
  //   }
  //   // // FIXME Temporary work-around until we handle the gripper
  //   // if(mcrtcControl.robot().refJointOrder().size() > initq.size())
  //   // {
  //   //   LOG_INFO("applying a temporary work-around")
  //   //   while(mcrtcControl.robot().refJointOrder().size() > initq.size())
  //   //   {
  //   //     initq.push_back(0);
  //   //     initdq.push_back(0);
  //   //   }
  //   // }
  //   mcrtcControl.init(initq);
  //   mcrtcControl.setWrenches(wrenches);
  //   mcrtcControl.running = true;

  //   mcrtcControl.controller().gui()->addElement({"Franka"},
  //                                             mc_rtc::gui::Button("Stop controller", [&mcrtcControl]() { mcrtcControl.running = false; }));

  //   // Start the control loop
  //   franka::JointPositions output(statePtr1->q);
  //   franka::RobotState state1 = *statePtr1;
  //   robotPtr1->control([&mcrtcControl,&initq,&initdq,&wrenches,&output](const franka::RobotState & state1, franka::Duration) -> franka::JointPositions
  //                 {
  //                   for(size_t i = 0; i < state1.q.size(); ++i)
  //                   {
  //                     initq[i] = state1.q[i];
  //                     initdq[i] = state1.dq[i];
  //                   }
  //                   wrenches.find("LeftHandForceSensor")->second.force().x() = state1.K_F_ext_hat_K[0]; //TODO use: state1->K_F_ext_hat_K[] or state1->0_F_ext_hat_K[] ?
  //                   wrenches.find("LeftHandForceSensor")->second.force().y() = state1.K_F_ext_hat_K[1];
  //                   wrenches.find("LeftHandForceSensor")->second.force().z() = state1.K_F_ext_hat_K[2];
  //                   wrenches.find("LeftHandForceSensor")->second.moment().x() = state1.K_F_ext_hat_K[3];
  //                   wrenches.find("LeftHandForceSensor")->second.moment().y() = state1.K_F_ext_hat_K[4];
  //                   wrenches.find("LeftHandForceSensor")->second.moment().z() = state1.K_F_ext_hat_K[5];
  //                   mcrtcControl.setEncoderValues(initq);
  //                   mcrtcControl.setEncoderVelocities(initdq);
  //                   mcrtcControl.setWrenches(wrenches);

  //                   if(mcrtcControl.running && mcrtcControl.run())
  //                   {
  //                     const auto & rjo = mcrtcControl.robot().refJointOrder();
  //                     for(size_t i = 0; i < output.q.size(); ++i)
  //                     {
  //                       const auto & j = rjo[i];
  //                       output.q[i] = mcrtcControl.robot().mbc().q[mcrtcControl.robot().jointIndexByName(j)][0];
  //                     }
  //                     return output;
  //                   }
  //                   output.q = state1.q;
  //                   return franka::MotionFinished(output);
  //                 });

  }
  catch(const franka::Exception & e)
  {
    std::cerr << "franka::Exception " << e.what() << "\n";
    return 1;
  }
  return 0;
}
