mc_franka
==

[![License](https://img.shields.io/badge/License-BSD%202--Clause-green.svg)](https://opensource.org/licenses/BSD-2-Clause)
[![CI](https://github.com/jrl-umi3218/mc_franka/workflows/CI%20of%20mc_franka/badge.svg?branch=master)](https://github.com/jrl-umi3218/mc_franka/actions?query=workflow%3A%22CI+of+mc_franka%22)

Interface between [libfranka] and [mc_rtc]. It provides multi-robot support and connect [mc_panda] devices to their [libfranka] counterpart.

## Setup
### Nix
#### Building with Nix

To build the software, simply use

```bash
nix build .#mc-franka
```

### Developping

To develop, use

```bash
nix develop .#mc-rtc-superbuild-mc-franka-devel
```

This will give you a shell with `mc_rtc` configured to use the `mc_panda` and `mc_panda_lirmm` robots, and `mc_franka` built from source. Follow the instructions in the terminal.

### Running

You can use the `mc-rtc-superbuild-mc-franka-devel` environment where you compiled `mc_franka` yourself, or if you simply want to let Nix build and run the software:

- Step 1: create a suitable `mc_franka` configuration for your robot in an `mc_rtc.yaml` file (please refer to `Usage` section).

```bash
nix develop .#mc-rtc-superbuild-mc-franka # or -devel
# Run the gui in the background
(mc-rtc-magnum &)
# By default mc_rtc_ticker will use the configuration provided by `MC_RTC_CONTROLLER_CONFIG` env variable. This is set by the mc-rtc-superbuild derivation and devShell to contain all needed runtime depencencies and optionally a default controller's configuration
MCFrankaControl -f mc_rtc.yaml
```

### From source
#### Dependencies

This package requires:
- [mc_panda]
- [mc_rtc]

You will also need the `linux-libc-dev` package on Ubuntu and Debian systems. To build and install the software:

1. Install this project's dependencies
1. Install this project (`cmake`/`make`/`make install`)

## Usage

1. Make sure the system is [setup properly to work with the robot](https://frankaemika.github.io/docs/getting_started.html#verifying-the-connection)
1. Make sure your user account has sufficient memory limits

For that last point, you want to edit `/etc/security/limits.conf` and add the following line:

```
USERNAME - memlock 1000000000
```

Then log-out and log-in, you can confirm the new limit is active by running:

```bash
ulimit -l
```

Your mc_rtc configuration file (typically `~/.config/mc_rtc/mc_rtc.yaml`) should contain the following lines:

```yaml
# General mc_rtc configuration to run a panda controller at 1kHz
MainRobot: PandaDefault # Or PandaHand/PandaFoot/PandaPump according to the end-effector installed on the robot
Enabled: YourController
Timestep: 0.001

# Set a LogPolicy suitable for real-time
LogPolicy: threaded

# Franka specific configuration
Franka:
  ControlMode: Position # Can be: Position/Velocity/Torque/Observed
  panda_default: # Name of the robot in the controller
    ip: 172.16.0.2 # IP of the robot
  panda_2: # Name of an extra panda in the controller
    ip: 172.16.1.2
  # Actuated robots that are not controlled via mc_franka
  ignored: [env/door, env/box]
```

**Observed mode**: When `ControlMode: Observed` is set, `MCFrankaControl` reads the robot state via
`franka::Robot::readOnce()` without sending any commands. `controller.run()` still executes so that
mc_rtc observers, logging and the GUI continue to work. An optional `ObservedPeriod` key (in
seconds, default `0.005`) controls how frequently the robot state is polled:

```yaml
Franka:
  ControlMode: Observed
  ObservedPeriod: 0.005 # Poll robot state every 5 ms
  panda_default:
    ip: 172.16.0.2
```

Run the program:

```bash
MCFrankaControl
```

You can also provide an additional configuration file (to swap between different network configurations easily for example):

```bash
MCFrankaControl -f conf.yaml
```

Known issues
--

It may happen that some libraries are not found due to the high priviligies given for real-time scheduling. To overcome it, please consider running the following commands :

```bash
echo "$HOME/workspace/devel/catkin_data_ws/install/lib" | sudo tee -a /etc/ld.so.conf.d/mc_rtc_ros.conf
echo "$HOME/workspace/install/lib" | sudo tee -a /etc/ld.so.conf.d/mc_rtc.conf
echo "/opt/ros/${ROS_DISTRO}/lib" | sudo tee -a /etc/ld.so.conf.d/ros2.conf
```

Then run the following command :

```bash
sudo ldconfig
```

### Video presentation

A video demonstrating panda motion generation and simultaneous pump actuation employing this implementation is available here:

[![Video presentation](https://img.youtube.com/vi/juynq6x9JJ8/0.jpg)](https://youtu.be/juynq6x9JJ8 "Safe Impacts with Soft Contacts Based on Learned Deformations")

### Reference

Writing code takes time.
If this implementation is useful for your research, please cite the related publication:

```
@INPROCEEDINGS{Dehio2021ICRA,
  title={Robot-Safe Impacts with Soft Contacts Based on Learned Deformations},
  author={Dehio, Niels and Kheddar, Abderrahmane},
  booktitle={IEEE Int. Conf. on Robotics and Automation},
  pages={1357-1363},
  year={2021},
  pdf = {https://hal.archives-ouvertes.fr/hal-02973947/document},
  url = {https://hal.archives-ouvertes.fr/hal-02973947}
}
```

[![I.AM.Logo](https://i-am-project.eu/templates/yootheme/cache/iam_logo-horizontaal_XL-9e4a8a2a.png)](https://i-am-project.eu/index.php)

This work was partially supported by the Research Project I.AM. through the European Union H2020 program under GA 871899.

[libfranka]: https://github.com/frankaemika/libfranka
[mc_rtc]: https://github.com/jrl-umi3218/mc_rtc
[mc_panda]: https://github.com/jrl-umi3218/mc_panda
