/* Copyright 2020 mc_rtc development team */

#pragma once

#include <mc_rtc/Configuration.h>
#include <mc_rtc/logging.h>

namespace mc_franka
{

enum class ControlMode
{
  Position,
  Velocity,
  Torque
};
}

namespace mc_rtc
{

template<>
struct ConfigurationLoader<mc_franka::ControlMode>
{
  static Configuration save(const mc_franka::ControlMode & cm)
  {
    Configuration c;
    switch(cm)
    {
      case mc_franka::ControlMode::Position:
        c.add("cm", "Position");
        break;
      case mc_franka::ControlMode::Velocity:
        c.add("cm", "Velocity");
        break;
      case mc_franka::ControlMode::Torque:
        c.add("cm", "Torque");
        break;
      default:
        log::error_and_throw<std::runtime_error>("ControlMode has unexpected value");
    }
    return c("cm");
  }

  static mc_franka::ControlMode load(const Configuration & conf)
  {
    std::string cm = conf;
    if(cm == "Position")
    {
      return mc_franka::ControlMode::Position;
    }
    if(cm == "Velocity")
    {
      return mc_franka::ControlMode::Velocity;
    }
    if(cm == "Torque")
    {
      return mc_franka::ControlMode::Torque;
    }
    log::error_and_throw<std::runtime_error>("ControlMode has unexpected value {}", cm);
  }
};

} // namespace mc_rtc
