#include "InterfaceMCRTC.hpp"

InterfaceMCRTC::InterfaceMCRTC( int numJoints) : dof(numJoints)
{
  qCommand.resize(dof);
  dqCommand.resize(dof);
  tauCommand.resize(dof);
}

bool InterfaceMCRTC::init(const std::vector<double> q, const std::vector<double> dq, const std::vector<double> tau, const std::map<std::string, sva::ForceVecd> wrenches)
{
  if(mcrtcControl.controller().timeStep != 0.001)
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "mc_rtc must be configured to run at 1kHz");
  }
  this->setState(q,dq,tau,wrenches);

  mcrtcControl.running = true;

  // mcrtcControl.controller().gui()->addElement({"Franka"},
  //                                           mc_rtc::gui::Button("Stop controller", [&mcrtcControl]() { mcrtcControl.running = false; }));

  return true;
}

bool InterfaceMCRTC::setState(const std::vector<double> q, const std::vector<double> dq, const std::vector<double> tau, const std::map<std::string, sva::ForceVecd> wrenches)
{
  mcrtcControl.setEncoderValues(q);
  mcrtcControl.setEncoderVelocities(dq);
  mcrtcControl.setJointTorques(tau); //TODO please double check
  mcrtcControl.setWrenches(wrenches);
  return true;
}

std::vector<double> InterfaceMCRTC::getPositionCommand()
{
  if(mcrtcControl.running && mcrtcControl.run())
  {
    const auto & rjo = mcrtcControl.robot().refJointOrder();
    for(size_t i = 0; i < dof; ++i)
    {
      const auto & j = rjo[i];
      qCommand.at(i) = mcrtcControl.robot().mbc().q[mcrtcControl.robot().jointIndexByName(j)][0];
    }
    return qCommand;
  }
  else
  {
    std::cerr << "error: not running";
    return std::vector<double>(); //TODO
  }
}