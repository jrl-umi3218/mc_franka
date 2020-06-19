#include "Controller.hpp"
#include <mc_control/mc_global_controller.h>

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
    LOG_INFO("Ok, let's start");
    // ControlMode controlMode = ControlMode::Position;
    ControlMode controlMode = ControlMode::Velocity;
    // ControlMode controlMode = ControlMode::Torque;

    //setup variables ##########################################################
    int total_dof = 0;
    std::vector<double> init_q_vector;
    std::vector<double> q_vector;
    std::vector<double> dq_vector;
    std::vector<double> tau_vector;
    std::vector<double> cmd_vector;
    std::array<double, PANDA_DOF> q_array;
    std::array<double, PANDA_DOF> dq_array;
    std::array<double, PANDA_DOF> tau_array;
    std::array<double, PANDA_DOF> cmd_array;
    std::map<std::string, sva::ForceVecd> wrenches;
    sva::ForceVecd wrench = sva::ForceVecd(Eigen::Vector6d::Zero());
    Eigen::Vector3d force = Eigen::Vector3d::Zero();
    Eigen::Vector3d moment = Eigen::Vector3d::Zero();
    std::map<std::string, std::shared_ptr<Controller> >::iterator it;
    std::map<std::string, std::shared_ptr<Controller> > robos;
    std::string sensorname;
    int dof;
    double control_command_success_rate;
    int counter=0;
    bool ok;
    // LOG_INFO("setup variables done");

    //init ##########################################################
    std::string robName1 = "LeftHand";
    std::string ip1(argv[1]);
    std::unique_ptr<franka::Robot::Impl> rob1;
    rob1 = std::make_unique<franka::Robot::Impl>(std::make_unique<franka::Network>(ip1, research_interface::robot::kCommandPort), 0);
    robos.insert(std::make_pair(robName1, std::make_shared<Controller>(*rob1, controlMode)));
    total_dof += PANDA_DOF;
    sensorname = "LeftHandForceSensor";
    wrenches.insert(std::make_pair(sensorname, wrench));
    if((robName1+"ForceSensor").compare(sensorname) != 0)
    {
      LOG_ERROR('robot name and force sensor name do not fit')
    }
    LOG_INFO("initialized " << robName1 << " with ip: " << ip1);
    if(argc == 3)
    {
      std::string robName2 = "RightHand";
      std::string ip2(argv[2]);
      std::unique_ptr<franka::Robot::Impl> rob2;
      rob2 = std::make_unique<franka::Robot::Impl>(std::make_unique<franka::Network>(ip2, research_interface::robot::kCommandPort), 0);
      robos.insert(std::make_pair(robName2, std::make_shared<Controller>(*rob2, controlMode)));
      total_dof += PANDA_DOF;
      sensorname = "RightHandForceSensor";
      wrenches.insert(std::make_pair(sensorname, wrench));
      if((robName2+"ForceSensor").compare(sensorname) != 0)
      {
        LOG_ERROR('robot name and force sensor name do not fit')
      }
      LOG_INFO("initialized " << robName2 << " with ip: " << ip2);
    }

    init_q_vector.resize(total_dof);
    q_vector.resize(total_dof);
    dq_vector.resize(total_dof);
    tau_vector.resize(total_dof);
    cmd_vector.resize(total_dof);
    // LOG_INFO("init done");

    //setup mcrtc ##########################################################
    mc_control::MCGlobalController mcrtcControl;
    if(mcrtcControl.controller().timeStep != 0.001)
    {
      LOG_ERROR_AND_THROW(std::runtime_error, "mc_rtc must be configured to run at 1kHz");
    }
    mcrtcControl.running = true;
    mcrtcControl.controller().gui()->addElement({"Franka"}, mc_rtc::gui::Button("Stop controller", [&mcrtcControl]() { mcrtcControl.running = false; }));
    LOG_INFO("setup mcrtc done");

    //start libfranka ##########################################################
    counter=0;
    for (it = robos.begin(); it != robos.end(); it++)
    {
      franka::RobotState initState = it->second->getInitialState();
      for(size_t i = 0; i < PANDA_DOF; ++i)
      {
        init_q_vector.at(counter) = initState.q[i];
        counter++;
      }
      ok = it->second->start();
    }
    mcrtcControl.init(init_q_vector);
    const std::vector<std::string> & rjo = mcrtcControl.robot().refJointOrder();
    if(rjo.size() != total_dof)
    {
      LOG_ERROR_AND_THROW(std::runtime_error, "mc_rtc-model has " << rjo.size() << " joints, the real robot system has " << total_dof << " joints");
    }
    else
    {
      LOG_INFO("the robot system has " << total_dof << " joints");
    }
    LOG_INFO("start libfranka done");

    //main loop ##########################################################
    while(true)
    {
      //get and set state ##########################################################
      counter=0;
      for (it = robos.begin(); it != robos.end(); it++)
      {
        //get state
        control_command_success_rate = it->second->getState(q_array,dq_array,tau_array,force,moment);
        //convert array to vector
        for(size_t i = 0; i < PANDA_DOF; ++i)
        {
          q_vector.at(counter)=q_array[i];
          dq_vector.at(counter)=dq_array[i];
          tau_vector.at(counter)=tau_array[i];
          counter++;
        }
        wrench = sva::ForceVecd(moment,force);
        sensorname = it->first + "ForceSensor";
        wrenches.find(sensorname)->second = wrench;
      }
      //set state
      mcrtcControl.setEncoderValues(q_vector);
      mcrtcControl.setEncoderVelocities(dq_vector);
      mcrtcControl.setJointTorques(tau_vector);
      mcrtcControl.setWrenches(wrenches);


      //get and set command ##########################################################
      if(mcrtcControl.running && mcrtcControl.run())
      {
        //get command
        for(size_t id = 0; id < total_dof; ++id)
        {
          const std::string & jointName = rjo[id];
          switch (controlMode)
          {
            case ControlMode::Torque:
              // cmd_vector.at(id) = ... //TODO
              break;
            case ControlMode::Velocity:
              cmd_vector.at(id) = mcrtcControl.robot().mbc().alpha[mcrtcControl.robot().jointIndexByName(jointName)][0]; 
              break;
            case ControlMode::Position:
              cmd_vector.at(id) = mcrtcControl.robot().mbc().q[mcrtcControl.robot().jointIndexByName(jointName)][0]; 
              break;
            default:
              return false;
          }
        }
        counter=0;
        for (it = robos.begin(); it != robos.end(); it++)
        {
          //convert vector into array
          for(size_t i = 0; i < PANDA_DOF; ++i)
          {
            cmd_array[i] = cmd_vector.at(counter);
            counter++;
          }
          //set command
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
  }
  catch(const franka::Exception & e)
  {
    std::cerr << "franka::Exception " << e.what() << "\n";
    return 1;
  }
  return 0;
}
