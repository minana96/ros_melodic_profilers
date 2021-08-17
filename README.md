# *ros_melodic_profilers* ROS package

This repository represents the *ros_melodic_profilers* ROS pacakge, the package that configures and launches ROS nodes that collect power consumption, CPU usage and RAM utilisation measurements. The *ros_melodic_profilers* package is intended for ROS Melodic. The repository needs to be cloned to the catkin workspace on the robot (no need to clone it to the PC) and compiled with following commands:
```bash
cd <catkin_ws_dir>/src
git clone git@github.com:minana96/ros_melodic_profilers.git
cd ..
catkin_make
```

## Dependencies

The package is dependent on *ros_profilers_msgs* ROS pacakge, which contains definitions of service messages. The instruction on how to configure and install the *ros_profilers_msgs* package are given in [this](https://github.com/minana96/ros_profilers_msgs) GitHub repository. The services are provided by the ROS nodes in *ros_melodic_profilers* repositories, yet they are consumed from the Robot Runner tool that runs on the PC. Since both entities require service definitions, they are separate within the dedicated *ros_profilers_msgs* package.

The source code for the nodes described below is written in Python 2.7, as this is the default Python version for ROS Melodic. The following pip packages need to be installed: **pyserial** and **psutil**.

## Package content

The package consists of three directories, namely, *arduino*, *launch* and *src*.

### arduino

The power measurements are collected by the Arduino Nano Every board, which has INA219 current sensor attached. The detailed guide on how to connect the Arduino board and sensor is available in [here](https://github.com/S2-group/ros-configurations/tree/main/meter-arduino). The *ina219sensor.ino* file, containing the code for power measurement collection, is located in the *arduino* directory of this repository.

### src

This directory contains source code of two ROS nodes:
- **cpu_mem_profiler_server.py**: the ROS node that provides ROS services for starting and stopping the CPU usage and RAM utilisation measurement collection. The measurements are obtained via *psutil* Python package and the measurement results are returned to the service client upon stopping the measurement collection;
- **ina219_profiler_server.py**: the ROS node that provides ROS services for starting and stopping the power consumption measurement collection. The measurements are obtained via requests sent to the Arduino board over the serial connection with *pyserial* Python package. The measurement results are returned to the service client upon stopping the measurement collection.


### launch

Both profiler ROS nodes are configured and launched in **profilers.launch** file. While the frequency of power measurement collection is fixed to 200Hz, the frequency of CPU usage and RAM utilisation measurement collection can be adjusted via launch file parameter.
