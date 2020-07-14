#include <mc_control/mc_global_controller.h>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>
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

  // Initialize data fields for the print thread.
  struct {
    std::mutex mutex;
    bool has_data;
    franka::RobotState robot_state;
    sva::ForceVecd wrench;
    bool is_singular;
  } print_data{};
  std::atomic_bool running{true};
  const double print_rate = 2.0; //number of prints per second
  // Start print thread.
  std::thread print_thread([print_rate, &print_data, &running]() {
    while (running) {
      // Sleep to achieve the desired print rate.
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>((1.0 / print_rate * 1000.0))));
      // Try to lock data to avoid read write collisions.
      if (print_data.mutex.try_lock()) {
        if (print_data.has_data) {
          std::cout << "end-effector force= " <<  print_data.wrench.force().transpose() << " and moment =" << print_data.wrench.moment().transpose() << " and is_singular: "<< print_data.is_singular << std::endl;
          print_data.has_data = false;
        }
        print_data.mutex.unlock();
      }
    }
  });

  try
  {
    // Initialize the robot
    franka::Robot robot(argv[1]);
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}}); //values taken from https://github.com/frankaemika/libfranka/blob/master/examples/examples_common.cpp#L18
    robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}}); //values taken from https://github.com/frankaemika/libfranka/blob/master/examples/examples_common.cpp#L19
    // robot.setJointImpedance({{100,100,100,100,100,100,100}}); 
    // robot.setCartesianImpedance({{100,100,100,10,10,10}});
    robot.setCollisionBehavior( //values taken from https://github.com/frankaemika/libfranka/blob/master/examples/generate_joint_velocity_motion.cpp#L39
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    
    
    robot.setCollisionBehavior( //TODO: be careful with this mode!
        {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}}, {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}},
        {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}}, {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}},
        {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}}, {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}},
        {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}}, {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}});

    // Get the initial state of the robot
    const auto state = robot.readOnce();
    const int dof = state.q.size();

    // Initialize mc_rtc
    mc_control::MCGlobalController controller;
    if(controller.controller().timeStep != 0.001)
    {
      LOG_ERROR_AND_THROW(std::runtime_error, "mc_rtc must be configured to run at 1kHz");
    }
    sva::ForceVecd wrench = sva::ForceVecd(Eigen::Vector6d::Zero());
    std::map<std::string, sva::ForceVecd> wrenches;
    wrenches.insert(std::make_pair("LeftHandForceSensor", wrench));
    std::vector<double> init_q_vector;
    std::vector<double> q_vector;
    std::vector<double> dq_vector;
    std::vector<double> tau_vector;
    init_q_vector.resize(dof);
    q_vector.resize(dof);
    dq_vector.resize(dof);
    tau_vector.resize(dof);
    for(size_t i = 0; i < dof; ++i)
    {
      init_q_vector.at(i)=state.q[i];
      q_vector.at(i)=state.q[i];
      dq_vector.at(i)=state.dq[i];
      tau_vector.at(i)=state.tau_J[i];
    }
    // FIXME Temporary work-around until we handle the gripper
    int counter=dof-1;
    while(controller.robot().refJointOrder().size() > init_q_vector.size())
    {
      counter++;
      init_q_vector.at(counter)=0;
    }
    controller.setEncoderValues(q_vector);
    controller.setEncoderVelocities(dq_vector);
    controller.setJointTorques(tau_vector);
    controller.setWrenches(wrenches);
    controller.init(init_q_vector);
    controller.running = true;
    controller.controller().gui()->addElement({"Franka"},
                                              mc_rtc::gui::Button("Stop controller", [&controller]() { controller.running = false; }));

    // Initialize the libfranka VacuumGripper as 'sucker' and the mc_panda Pump as 'pump' if the robot-module has such a device
    bool pumpAvailable = false;
    std::shared_ptr<franka::VacuumGripper> sucker;
    std::shared_ptr<mc_panda::Pump> pump;
    try{
      sucker = std::make_shared<franka::VacuumGripper>( franka::VacuumGripper(argv[1]) );
      // franka::VacuumGripper sucker = franka::VacuumGripper(argv[1]);
      mc_rtc::log::info("Connection established to VacuumGripper via {}", argv[1]);
      
      std::string pumpDeviceName = "Pump";
      if(controller.robot().hasDevice<mc_panda::Pump>(pumpDeviceName))
      {
        pump = std::make_shared<mc_panda::Pump>( controller.robot().device<mc_panda::Pump>(pumpDeviceName) );
        // mc_panda::Pump pump = controller.robot().device<mc_panda::Pump>(pumpDeviceName);
        pumpAvailable = true;
        mc_rtc::log::info("RobotModule has a Pump named {}", pumpDeviceName);
      }
      else{
        mc_rtc::log::warning("RobotModule does not have a Pump named {}", pumpDeviceName);
        mc_rtc::log::warning("Pump functionality will not be available");
      }
    }
    catch(const franka::NetworkException & e)
    {
      mc_rtc::log::warning("Cannot connect to VacuumGripper via {}", argv[1]);
      mc_rtc::log::warning("Pump functionality will not be available");
    }
    pumpAvailable = false; //TODO THIS IS A HACK

    // Start the control loop in velocity-control
    bool is_singular = false;;
    franka::Model model = robot.loadModel();
    franka::JointVelocities output_dq(state.dq);
    robot.control([&print_data,&model,&controller,&sucker,&pump,&pumpAvailable,&q_vector,&dq_vector,&tau_vector,&wrench,&wrenches,&is_singular,&output_dq](const franka::RobotState & state, franka::Duration) -> franka::JointVelocities
    {
      for(size_t i = 0; i < state.q.size(); ++i)
      {
        q_vector[i] = state.q[i];
        dq_vector[i] = state.dq[i];
        tau_vector[i] = state.tau_J[i];
      }

      wrench.force().x() = state.K_F_ext_hat_K[0];
      wrench.force().y() = state.K_F_ext_hat_K[1];
      wrench.force().z() = state.K_F_ext_hat_K[2];
      wrench.moment().x() = state.K_F_ext_hat_K[3];
      wrench.moment().y() = state.K_F_ext_hat_K[4];
      wrench.moment().z() = state.K_F_ext_hat_K[5];
      
      is_singular = false;
      if(wrench.force().x()==0 && wrench.force().y()==0 && wrench.force().z()==0 && wrench.moment().x()==0 && wrench.moment().y()==0 && wrench.moment().z()==0 ){
        // LOG_ERROR("libfranka wrench is zero") //NOTE this happens if smallest singular value of the jacobian is lower than 0.08...
        is_singular = true;
        const std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, state);
        const Eigen::Matrix<double, 6, 7> jacobian(jacobian_array.data());
        const Eigen::Matrix<double, 7, 1> torques(state.tau_ext_hat_filtered.data());

        Eigen::JacobiSVD<Eigen::Matrix<double, 7, 6>> svdT = Eigen::JacobiSVD<Eigen::Matrix<double, 7, 6>>();
        svdT.compute(jacobian.transpose(), Eigen::ComputeThinU | Eigen::ComputeThinV);
        // const Eigen::VectorXd wrench_vector = svdT.solve(torques); //this solution maybe faster, but does not correspond to the libfranka wrenches for full-rank Jacobian
        const Eigen::MatrixXd U = svdT.matrixU();
        const Eigen::MatrixXd V = svdT.matrixV();
        const Eigen::VectorXd Svec  = svdT.singularValues();
        Eigen::Matrix<double, 6, 6> Sinv = Eigen::Matrix<double, 6, 6>::Zero();
        for(int i=0; i<6; i++){
          if (Svec(i)>=0.08) //important threshold, identical to the libfranka threshold!
          {
            Sinv(i,i) = 1.0 / Svec(i);
          }
        }
        const Eigen::VectorXd wrench_vector = V * Sinv * U.transpose() * torques;

        wrench.force() = wrench_vector.head<3>();
        wrench.moment() = wrench_vector.tail<3>();
        // LOG_SUCCESS("special force = [" << wrench.force().x() << ", " << wrench.force().y() << ", " << wrench.force().z() << "] and moment =[" << wrench.moment().x() << ", " << wrench.moment().y() << ", " << wrench.moment().z() << "]")
      }

      wrenches.find("LeftHandForceSensor")->second = wrench;

      // int randomNumber = rand() % 500 + 1; //generate number between 1 and 1000
      // if (randomNumber==500){
      //   LOG_INFO("force = [" << wrench.force().x() << ", " << wrench.force().y() << ", " << wrench.force().z() << "] and moment =[" << wrench.moment().x() << ", " << wrench.moment().y() << ", " << wrench.moment().z() << "]")
      // }

      if(wrench.force().norm() > 30 && is_singular==false){
        LOG_ERROR("stopping because wrench.force().norm() = " << wrench.force().norm())
        LOG_ERROR("force = [" << wrench.force().x() << ", " << wrench.force().y() << ", " << wrench.force().z() << "] and moment =[" << wrench.moment().x() << ", " << wrench.moment().y() << ", " << wrench.moment().z() << "]")
        return franka::MotionFinished(output_dq);
      }
      
      controller.setEncoderValues(q_vector);
      controller.setEncoderVelocities(dq_vector);
      controller.setJointTorques(tau_vector);
      controller.setWrenches(wrenches);
      if(pumpAvailable)
      {
        const franka::VacuumGripperState stateSucker = sucker->readOnce();
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
      }

      if(controller.running && controller.run())
      {
        if(pumpAvailable)
        {
          if(pump->vacuumCommandRequested())
          {
            uint8_t vacuum;
            std::chrono::milliseconds timeout;
            pump->getVacuumCommandParams(vacuum, timeout);
            bool vacuumOK = sucker->vacuum(vacuum, timeout);
            pump->setVacuumCommandResult(vacuumOK);
            mc_rtc::log::info("vacuum command applied with the params vacuum {} and timeout {}, result: {}", std::to_string(vacuum), std::to_string(timeout.count()), vacuumOK);
          }
          if(pump->dropoffCommandRequested())
          {
            std::chrono::milliseconds timeout;
            pump->getDropoffCommandParam(timeout);
            bool dropoffOK = sucker->dropOff(timeout);
            pump->setDropoffCommandResult(dropoffOK);
            mc_rtc::log::info("dropoff command applied with the param timeout {}, result: {}", std::to_string(timeout.count()), dropoffOK);
          }
          if(pump->stopCommandRequested())
          {
            bool stopOK = sucker->stop();
            pump->setStopCommandResult(stopOK);
            mc_rtc::log::info("stop command applied, result: {}", stopOK);
          }
        }
        const auto & rjo = controller.robot().refJointOrder();
        for(size_t i = 0; i < output_dq.dq.size(); ++i)
        {
          const auto & j = rjo[i];
          output_dq.dq[i] = controller.robot().mbc().alpha[controller.robot().jointIndexByName(j)][0];
        }

        // Update data to print.
        if (print_data.mutex.try_lock()) {
          print_data.has_data = true;
          print_data.robot_state = state;
          print_data.wrench = wrench;
          print_data.is_singular = is_singular;
          print_data.mutex.unlock();
        }
        return output_dq;
      }
      output_dq.dq = state.dq;
      return franka::MotionFinished(output_dq);
    // }, franka::ControllerMode::kJointImpedance, true, 100); //default parameters
    }, franka::ControllerMode::kJointImpedance, true, 1000); //increased kDefaultCutoffFrequency 

    robot.stop();
  }
  catch(const franka::Exception & e)
  {
    running = false;
    std::cerr << "franka::Exception " << e.what() << "\n";
    // return 1;
  }

  if (print_thread.joinable()) {
    print_thread.join();
  }
  return 0;
}
