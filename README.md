# *ros_melodic_profilers* ROS package

This repository represents the *ros_melodic_profilers* ROS pacakge, the package that configures and launches SLAM task via [gmapping](http://wiki.ros.org/gmapping) ROS package. The *sherlock_slam* package is intended for ROS Melodic. The repository needs to be cloned to the catkin workspace on the robot (no need to clone it to the PC) and compiled with following commands:
```bash
cd <catkin_ws_dir>/src
git clone git@github.com:minana96/sherlock_slam.git
cd ..
catkin_make
```

## Dependencies

The package is dependent on *gmapping* ROS pacakge, which can be installed with the following command:
```bash
sudo apt install ros-melodic-gmapping
```

## Package content

The package consists of three directories, namely, *config*, *launch* and *maps*.

### config

The direcotry contains parameter configuration for *gmapping* ROS node in *yaml* format that are passed as arguments when the node is launched. The definitions of possible *gmapping* parameters are available in the [ROS wiki page](http://wiki.ros.org/gmapping):
- **gmapping_params.yaml**: the default configuration used in all experiments, with *particles* set to 5 and *temporalUpdate* set to -1 (i.e., turned off);
- **particles_30.yaml**: the configuration used in the number of particles effect experiment, where *particles* parameter is set to 30 for respective treatments; 
- **temporal_updates_on.yaml**: the configuration used in the temporal updates effect experiment, where *temporalUpdate* parameter is set to 0.5 for respective treatments (i.e., temporal updates turned on). 

### launch

The SLAM task is configured and launched via two launch files:
- **sherlock_slam.launch**: SLAM is launched on the local machine, with the possibility of visualisation in [Rviz](http://wiki.ros.org/rviz) tool. If this option is used, both Rviz and *turtlebot3_slam* ROS package need to be installed with following commands:
```bash
sudo apt install ros-melodic-rviz
sudo apt install ros-melodic-turtlebot3-slam
```
- **sherlock_slam_remote.launch**: SLAM is launched on the remote machine that is passed as a parameter. If this launch file is used, it is important that the *gmapping* ROS package is also installed on the target remote machine.

### maps  

The directory contains the maps of the robot arena, created with *gmapping* while the robot is teleoperated around the environment. The map is represented with both *arena_gallery.yaml* and *arena_gallery.pgm* files. The instructions on how to create and save a map of a custom environment with TurtleBot3 are presented in [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/).
