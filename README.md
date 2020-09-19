<div align="center">

# Aarhus Airport Gazebo simulation

![Aarhus logo](/media/logo_small.png "Aarhus airport sim logo")
</div>

## Table of contents

- [Aarhus airport Gazebo simulation](#aarhus-airport-gazebo-simulation)
  * [Table of contents](#table-of-contents)
  * [Package description](#package-description)
  * [Installation](#installation)
    + [Requirements](#requirements)
    + [Installing the package](#installing-the-package)
  * [Usage](#usage)
  * [Repository structure](#repository-structure)
  * [Documentation](#documentation)  

## Package description

There are two packages provided. The airport_sim package provides a Gazebo world of the Aarhus airport. Apart from just the map, this simulation is aimed at robots reacting to the airport lights. Tasks such as light maintainance and navigation on the air field can be tested in simulation before moving forward in the real world. Since the airport features >500 different light sources, the simulation can't just spawn every light source. Instead, the package comes with a dynamic light loading controller. This controller is responsible for loading lights into the sim and removing them at runtime. Configurations for the robot's position topic and FoV parameters are also exposed.

The robot_sim package contains a ROS implementation of a small autonomous vehicle. This vehicle can be used as a starting point to test your algorithms on. It features LIDARs, a camera, IMU and GPS if desired.

## Installation

### Requirements

You need a system with Ubuntu 20.04, ROS Noetic and Gazebo 11.0 installed. Furthermore, this package requires the following additional ROS packages:
- teleop_twist_keyboard
- ros_control
- ros_controllers
- gazebo_ros

### Installing the package
To install the package, clone this repository into the source folder of a catkin workspace. 
```console
user@pc:~/<path_to_catkin_ws>/src$ git clone https://github.com/amacati/airport_sim.git
```

After the download completed, simply run 
```console
user@pc:~/<path_to_catkin_ws>$ catkin_make
```
from the root folder of your workspace. Don't forget to source your setup.bash afterwards.
```console
user@pc:~/<path_to_catkin_ws>$ source devel/setup.bash
```

## Usage

To launch the simulation, enter
```console
user@pc:~$ roslaunch airport_sim airport_sim.launch
```
This will also spawn the car into the simulation, load the dynamic light controller as well as the GPS emulator and the car's virtual Ackermann drive controller. 

<div align="center">

![Airport with all lights enabled](/media/airport_full.png "Airport with all lights enabled")
</div>

If you want to control the car remotely, you can do so by launching the robot_sim remote control.
```console
user@pc:~$ roslaunch robot_sim remote_control.launch
```
<div align="center">

![Dynamic light loading and remote control](/media/airport_dynamic.png "Dynamic light loading and remote control")
</div>

## Repository structure

All files necessary for the simulation are contained within the [airport_sim](/airport_sim) package. The complete code for the simulated autonomous vehicle with its controllers can be found in [robot_sim](/robot_sim). For a detailled overview of the respective package structures, please refer to the [airport sim README](/airport_sim/README.md) and the [robot sim README](/robot_sim/README.md).

## Documentation

Each package's code is documented with rosdoc_lite, which is a light weight version of Doxygen for ROS packages. For details on the package's documentation, please refer to the respective READMEs ([airport package](/airport_sim/README.md), [robot package](/robot_sim/README.md)).