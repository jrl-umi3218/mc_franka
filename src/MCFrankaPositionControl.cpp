#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>
#include <atomic>
#include <thread>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  // Initialize data fields for the print thread.
  struct {
    std::mutex mutex;
    bool has_data;
    franka::RobotState robot_state;
    double time;
  } print_data{};
  std::atomic_bool running{true};
  const double print_rate = 10.0; //number of prints per second
  // Start print thread.
  std::thread print_thread([print_rate, &print_data, &running]() {
    while (running) {
      // Sleep to achieve the desired print rate.
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>((1.0 / print_rate * 1000.0))));
      // Try to lock data to avoid read write collisions.
      if (print_data.mutex.try_lock()) {
        if (print_data.has_data) {
          std::cout << "time= " <<  print_data.time << std::endl;
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
    robot.setCollisionBehavior( //values taken from https://github.com/frankaemika/libfranka/blob/master/examples/generate_joint_velocity_motion.cpp#L39
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    std::array<double, 7> initial_position = robot.readOnce().q_d;
    double time = 0.0;
    std::cout << std::endl << "franka::kDefaultCutoffFrequency = " << franka::kDefaultCutoffFrequency << std::endl;
    double frequency = franka::kDefaultCutoffFrequency;
    frequency = 10000000000;
    frequency = 1;
    frequency = 100;
    std::cout << std::endl << "using the frequency = " << frequency << std::endl;
    int counter=0;
    const bool usePrintThread=false;
    robot.control([&print_data,&initial_position, &time, &counter, &usePrintThread](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointPositions 
    {
      counter++;
      time += period.toSec(); //->no noise
      // time += 0.001; //->creates noise
      if(period.toSec() > 0.001 || period.toSec() < 0.001)
      {
        std::cout << counter << " => " << period.toSec() << std::endl;
      }

      // double delta_angle = M_PI / 30.0 * (1 - std::cos(M_PI / 0.25 * time)); //super fast motion
      double delta_angle = M_PI / 30.0 * (1 - std::cos(M_PI / 2.5 * time)); //slow motion

      franka::JointPositions output = {{initial_position[0], 
                                        initial_position[1] + delta_angle,
                                        initial_position[2] + delta_angle, 
                                        initial_position[3] + delta_angle, //+ 0.0001 * counter //adding increased offset in each iteration
                                        initial_position[4] + delta_angle, 
                                        initial_position[5] + delta_angle,
                                        initial_position[6] + delta_angle}};
      if (time >= 5.0) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(output);
      }
      // Update data to print.
      if(usePrintThread)
      {
        if (print_data.mutex.try_lock()) {
          print_data.has_data = true;
          print_data.robot_state = robot_state;
          print_data.time = time;
          print_data.mutex.unlock();
        }
      }
      return output;
    // }); //default params
    }, franka::ControllerMode::kJointImpedance, true, franka::kDefaultCutoffFrequency); //default params
    // }, franka::ControllerMode::kJointImpedance, true, frequency); //user-defined frequency
    // }, franka::ControllerMode::kJointImpedance, false, frequency); //user-defined frequency & limit_rate=false;
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
