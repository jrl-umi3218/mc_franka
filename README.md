mc_franka
==

Interface between [libfranka] and [mc_rtc]

Installation
--

1. Install [libfranka](https://frankaemika.github.io/docs/installation_linux.html#building-from-source) on the machine that will run the controller
2. Make sure the system is [setup properly to work with the robot](https://frankaemika.github.io/docs/getting_started.html#verifying-the-connection)
3. Install [mc_rtc] on the machine that will run the controller
4. Install [panda_cnoid] on the machine that will run the controller
5. Compile and install this program (`cmake/make/sudo make install`)

Running
--

After installing, make sure that you mc_rtc configuration file (`~/.config/mc_rtc/mc_rtc.yaml`) contains the following lines:

```yaml
MainRobot: panda # Or panda_foot/panda_tool according to the end-effector installed on the robot
Enabled: YourController
Timestep: 0.001 # The controller must run at 1kHz
LogPolicy: threaded # Avoid slowdowns due to disk flush
```


```bash
MCFrankaControl <fci-ip>
```

Where `<fci-ip>` is the robot's IP

[libfranka]: https://github.com/frankaemika/libfranka
[mc_rtc]: http://github.com/jrl-umi3218/mc_rtc
[panda_cnoid]: https://github.com/gergondet/panda_cnoid
