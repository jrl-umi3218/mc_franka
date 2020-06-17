#pragma once

#include <mc_control/mc_global_controller.h>

#include <iostream>
#include <string>

class InterfaceMCRTC {
    public:
        InterfaceMCRTC(const int numJoints);

        ~InterfaceMCRTC() {
        }

        bool init(const std::vector<double> q, const std::vector<double> dq, const std::vector<double> tau, const std::map<std::string, sva::ForceVecd> wrenches);
        bool setState(const std::vector<double> q, const std::vector<double> dq, const std::vector<double> tau, const std::map<std::string, sva::ForceVecd> wrenches);
        std::vector<double> getPositionCommand();


    private:
        int dof;
        mc_control::MCGlobalController mcrtcControl;
        std::vector<double> qCommand;
        std::vector<double> dqCommand;
        std::vector<double> tauCommand;
};