#pragma once

#include <mc_rtc/Configuration.h>
#include <mc_rtc/logging.h>

enum class ControlMode
{
  Position,
  Velocity,
  Torque
};

namespace mc_rtc
{

template<>
struct ConfigurationLoader<ControlMode>
{
  static Configuration save(const ControlMode & cm)
  {
    Configuration c;
    switch(cm)
    {
      case ControlMode::Position:
        c.add("cm", "Position");
        break;
      case ControlMode::Velocity:
        c.add("cm", "Velocity");
        break;
      case ControlMode::Torque:
        c.add("cm", "Torque");
        break;
      default:
        log::error_and_throw<std::runtime_error>("ControlMode has unexpected value");
    }
    return c("cm");
  }

  static ControlMode load(const Configuration & conf)
  {
    std::string cm = conf;
    if(cm == "Position")
    {
      return ControlMode::Position;
    }
    if(cm == "Velocity")
    {
      return ControlMode::Velocity;
    }
    if(cm == "Torque")
    {
      return ControlMode::Torque;
    }
    log::error_and_throw<std::runtime_error>("ControlMode has unexpected value {}", cm);
  }
};

} // namespace mc_rtc
