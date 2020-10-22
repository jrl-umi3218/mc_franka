mc_franka
==

[![License](https://img.shields.io/badge/License-BSD%202--Clause-green.svg)](https://opensource.org/licenses/BSD-2-Clause)
[![CI](https://github.com/jrl-umi3218/mc_franka/workflows/CI%20of%20mc_franka/badge.svg?branch=master)](https://github.com/jrl-umi3218/mc_franka/actions?query=workflow%3A%22CI+of+mc_franka%22)

Interface between [libfranka] and [mc_rtc]. It provides multi-robot support and connect [mc_panda] devices to their [libfranka] counterpart.

Dependencies
------------

This package requires:
- [mc_panda]

You will also need the `linux-libc-dev` package on Ubuntu and Debian systems.

Usage
--

1. Make sure the system is [setup properly to work with the robot](https://frankaemika.github.io/docs/getting_started.html#verifying-the-connection)
2. Install this project's dependencies
3. Install this project (`cmake`/`make`/`make install`)
4. Make sure your user account has sufficient memory limits

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
  ControlMode: Position # Can be: Position/Velocity/Torque
  panda_default: # Name of the robot in the controller
    ip: 172.16.0.2 # IP of the robot
  panda_2: # Name of an extra panda in the controller
    ip: 172.16.1.2
  # Actuated robots that are not controlled via mc_franka
  ignored: [env/door, env/box]
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

`MCFrankaControl` will not pick up on newly installed libraries. To work-around this issue, run the following command after installing a new module:

```bash
sudo ldconfig
```

[![I.AM.Logo](https://i-am-project.eu/templates/yootheme/cache/iam_logo-horizontaal_XL-9e4a8a2a.png)](https://i-am-project.eu/index.php)

This work was partially supported by the Research Project I.AM. through the European Union H2020 program under GA 871899.

[libfranka]: https://github.com/frankaemika/libfranka
[mc_rtc]: https://github.com/jrl-umi3218/mc_rtc
[mc_panda]: https://github.com/jrl-umi3218/mc_panda
