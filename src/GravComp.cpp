// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <iostream>
#include <iterator>
#include <mutex>
#include <thread>

#include <Eigen/Core>
#include <Eigen/Eigen>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

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


int main(int argc, char** argv) {
  // Check whether the required arguments were passed.
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  // Initialize data fields for the print thread.
  struct {
    std::mutex mutex;
    bool has_data;
    franka::RobotState robot_state;
    std::array<double, 7> tau_d_calculated;
    std::array<double, 7> tau_d_last;
    std::array<double, 7> gravity;
    std::array<double, 7> coriolis;
    std::array<double, 42> jacobian_array;
  } print_data{};
  std::atomic_bool running{true};
  const double print_rate = 10.0;
  // Start print thread.
  std::thread print_thread([print_rate, &print_data, &running]() {
    while (running) {
      // Sleep to achieve the desired print rate.
      std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>((1.0 / print_rate * 1000.0))));

      // Try to lock data to avoid read write collisions.
      if (print_data.mutex.try_lock()) {
        if (print_data.has_data) {
          // std::array<double, 7> tau_error{};
          // double error_rms(0.0);
          // std::array<double, 7> tau_d_actual{};
          // for (size_t i = 0; i < 7; ++i) {
          //   tau_d_actual[i] = print_data.tau_d_last[i] + print_data.gravity[i];
          //   tau_error[i] = tau_d_actual[i] - print_data.robot_state.tau_J[i];
          //   error_rms += std::pow(tau_error[i], 2.0) / tau_error.size();
          // }
          // error_rms = std::sqrt(error_rms);
          const Eigen::Matrix<double, 7, 1> mytorques(print_data.robot_state.tau_ext_hat_filtered.data());
          const Eigen::Matrix<double, 6, 7> jacobian(print_data.jacobian_array.data());

          const Eigen::MatrixXd jacobianPinv1 = ( jacobian * jacobian.transpose() ).inverse() * jacobian;
          const Eigen::VectorXd mywrench1= jacobianPinv1 * mytorques;

          Eigen::FullPivLU<Eigen::Matrix<double, 6, 6>> lu_decomp_ = Eigen::FullPivLU<Eigen::Matrix<double, 6, 6>>();
          lu_decomp_.compute( jacobian * jacobian.transpose() );
          const Eigen::MatrixXd jacobianPinv2 = lu_decomp_.inverse() * jacobian;
          const Eigen::VectorXd mywrench2 = jacobianPinv2 * mytorques;

          Eigen::JacobiSVD<Eigen::Matrix<double, 7, 6>> svdT = Eigen::JacobiSVD<Eigen::Matrix<double, 7, 6>>();
          svdT.setThreshold(0.08); //critical singular value threshold is 0.08
          svdT.compute(jacobian.transpose(), Eigen::ComputeThinU | Eigen::ComputeThinV);
          const Eigen::VectorXd mywrench3 = svdT.solve(mytorques);

          const Eigen::MatrixXd U = svdT.matrixU();
          const Eigen::MatrixXd V = svdT.matrixV();
          const Eigen::VectorXd Svec  = svdT.singularValues();
          Eigen::Matrix<double, 6, 6> S = Eigen::Matrix<double, 6, 6>::Zero();
          S.diagonal() = Svec;
          Eigen::Matrix<double, 6, 6> Sinv = Eigen::Matrix<double, 6, 6>::Zero();
          // Sinv = S.inverse();
          for(int i=0; i<6; i++){
            if (Svec(i)>=0.08){
              Sinv(i,i) = 1.0 / Svec(i);
            }
          }
          const Eigen::MatrixXd jacobianPinv4 = V * Sinv * U.transpose();
          const Eigen::VectorXd mywrench4 = jacobianPinv4 * mytorques;


          // Print data to console
          std::cout << "control_command_success_rate: " <<  print_data.robot_state.control_command_success_rate << std::endl
                    << "joint configuration: " <<  print_data.robot_state.q << std::endl
                    // << "jacobian: \n" << jacobian << std::endl
                    // << "U: \n" << U << std::endl
                    // << "V: \n" << V << std::endl
                    // << "S: \n" << S << std::endl
                    // << "Sinv: \n" << Sinv << std::endl
                    // << "jacobianPinv: \n" << jacobianPinv1.row(0) << std::endl
                    // << "jacobianPinv: \n" << jacobianPinv2.row(0) << std::endl
                    // << "jacobianPinv: \n" << jacobianPinv4.row(0) << std::endl
                    // << "jacobian-singularValues: " << svdT.singularValues().transpose() << std::endl
                    << "External torque, filtered: " <<  print_data.robot_state.tau_ext_hat_filtered << std::endl
                    // << "end-effector wrench (base frame)     : " <<  print_data.robot_state.K_F_ext_hat_K << std::endl
                    << "end-effector wrench (stiffness frame): " <<  print_data.robot_state.O_F_ext_hat_K << std::endl
                    << "end-effector wrench (stiffness frame): " <<  mywrench1.transpose() << std::endl
                    << "end-effector wrench (stiffness frame): " <<  mywrench2.transpose() << std::endl
                    << "end-effector wrench (stiffness frame): " <<  mywrench3.transpose() << std::endl
                    << "end-effector wrench (stiffness frame): " <<  mywrench4.transpose() << std::endl
                    // << "tau_error [Nm]: " << tau_error << std::endl
                    // << "tau_commanded [Nm]: " << tau_d_actual << std::endl
                    // << "tau_measured [Nm]: " << print_data.robot_state.tau_J << std::endl
                    // << "root mean square of tau_error [Nm]: " << error_rms << std::endl
                    << "-----------------------" << std::endl
                    ;
          print_data.has_data = false;
        }
        print_data.mutex.unlock();
      }
    }
  });

  try {
    franka::Robot robot(argv[1]);
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}}); //values taken from https://github.com/frankaemika/libfranka/blob/master/examples/examples_common.cpp#L18
    robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}}); //values taken from https://github.com/frankaemika/libfranka/blob/master/examples/examples_common.cpp#L19
    // robot.setJointImpedance({{100,100,100,100,100,100,100}}); 
    // robot.setCartesianImpedance({{100,100,100,10,10,10}});
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    //TODO: be careful with this mode!
    robot.setCollisionBehavior(
        {{2000.0, 2000.0, 1800.0, 1800.0, 1600.0, 1400.0, 1200.0}}, {{2000.0, 2000.0, 1800.0, 1800.0, 1600.0, 1400.0, 1200.0}},
        {{2000.0, 2000.0, 1800.0, 1800.0, 1600.0, 1400.0, 1200.0}}, {{2000.0, 2000.0, 1800.0, 1800.0, 1600.0, 1400.0, 1200.0}},
        {{2000.0, 2000.0, 2000.0, 2500.0, 2500.0, 2500.0}}, {{2000.0, 2000.0, 2000.0, 2500.0, 2500.0, 2500.0}},
        {{2000.0, 2000.0, 2000.0, 2500.0, 2500.0, 2500.0}}, {{2000.0, 2000.0, 2000.0, 2500.0, 2500.0, 2500.0}});
    
    franka::Model model = robot.loadModel();

    // const std::array<double, 7> d_gains = {{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};
    const std::array<double, 7> d_gains = {{15.0, 15.0, 15.0, 15.0, 9.0, 8.0, 4.0}};
    // const std::array<double, 7> d_gains = {{5.0, 5.0, 5.0, 5.0, 3.0, 2.5, 1.5}};
    // const std::array<double, 7> d_gains = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}; //TODO pure gravity compensation mode (with coriolis) performs very bad

    // Define callback for the joint torque control loop.
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback =
            [&print_data, &model, d_gains](
                const franka::RobotState& state, franka::Duration /*period*/) -> franka::Torques {

      const std::array<double, 7> gravity = model.gravity(state);
      const std::array<double, 7> coriolis = model.coriolis(state);
      const std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, state);

      // Compute torque command
      std::array<double, 7> tau_d_calculated;
      for (size_t i = 0; i < 7; i++) {
        tau_d_calculated[i] = - d_gains[i] * state.dq[i] + coriolis[i];
      }

      // The following line is only necessary for printing the rate limited torque. As we activated
      // rate limiting for the control loop (activated by default), the torque would anyway be
      // adjusted!
      std::array<double, 7> tau_d_rate_limited =
          franka::limitRate(franka::kMaxTorqueRate, tau_d_calculated, state.tau_J_d);

      // Update data to print.
      if (print_data.mutex.try_lock()) {
        print_data.has_data = true;
        print_data.robot_state = state;
        print_data.tau_d_calculated = tau_d_calculated;
        print_data.tau_d_last = tau_d_rate_limited;
        print_data.gravity = gravity;
        print_data.coriolis = coriolis;
        print_data.jacobian_array = jacobian_array;
        print_data.mutex.unlock();
      }

      // Send torque command.
      return tau_d_rate_limited;
    };

    // Start real-time control loop.
    robot.control(impedance_control_callback);

  } catch (const franka::Exception& ex) {
    running = false;
    std::cerr << ex.what() << std::endl;
  }

  if (print_thread.joinable()) {
    print_thread.join();
  }
  return 0;
}
