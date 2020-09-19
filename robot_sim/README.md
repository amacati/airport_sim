<div align="center">

# Robot Sim package

![Aarhus logo](/media/logo_robot_sim.png "Aarhus airport sim logo")
</div>

## Table of contents  

- [Robot Sim package](#robot-sim-package)
  * [Table of contents](#table-of-contents)
  * [Package description](#package-description)
  * [Usage](#usage)
  * [Robot model](#robot-model)
    + [URDF structure](#urdf-structure)
    + [Sensors](#sensors)
    + [Virtual Ackermann drive](#virtual-ackermann-drive)
  * [Documentation](#documentation)

## Package description

This package provides a small vehicle equipped with all necessary sensors for autonomous navigation. It is mainly thought for testing purposes. Note that the kinematics of the model may exhibit some strange behaviour under certain conditions.

## Usage

To launch an empty simulation with the car, enter
```console
user@pc:~$ roslaunch robot_sim test.launch
```
This will also load the virtual Ackermann drive controller. In order to control the car in the simulation, you can run a separate teleop node:
```console
user@pc:~$ roslaunch robot_sim remote_control.launch
```

The package also offers a development launch option to inspect the car model in RVIZ. To start RVIZ with a joint_state_publisher, run
```console
user@pc:~$ roslaunch robot_sim devel.launch
```

## Robot model

The robot model is a simple car model converted for use in ROS. It features speed controllers for the rear wheels, steering controllers for the front wheels as well as a variety of sensors.

### URDF structure

The URDF files for the car can be found in the [urdf folder](/robot_sim/urdf/). In order to mimic an Ackermann drive in Gazebo (which does not allow for closed kinematic chains), the front wheels are connected to the main chassis with separate steering joints, each with its own actuator. The rear wheels are also each connected with their own joint to allow for a virtual differential drive. Camera, IMU and LIDARs are connected to the main body with their own links. 

> **_Note:_** The inertias of each link are very rough approximations, and masses might be completely off. If you observe strange dynamics with the robot, try to tinker with these values first.

### Sensors

The robot has integrated LIDAR sensors, a camera, an IMU and a pseudo GPS. In order to make the simulation more performant, the [regular model](/robot_sim/urdf/tas_car.urdf.xacro) is missing most of the sensors. You can switch to the [full model](/robot_sim/urdf/tas_car_full.urdf.xacro) by swapping the description file name in the responsible [launch file](/robot_sim/launch/description.launch).  

Currently, the usual Hector GPS plugin for ROS URDFs is not supported in ROS Noetic. The robot_sim package therefore implements its own immitation of a GPS signal. Using the [P3D plugin](http://gazebosim.org/tutorials?tut=ros_gzplugins#P3D(3DPositionInterfaceforGroundTruth)), the [GPSEmulator](/robot_sim/src/gps_emulator.py) republishes the ground truth position signal as a GPS signal with gaussian noise. The noise parameters can be tuned in the [config file](/robot_sim/config/gps_emulator.yaml). 
>**_Note:_** The GPS signal is not yet converted to real world coordinates. This is left to the package users. The transformation function is however already set up in the [GPSEmulator](/robot_sim/src/gps_emulator.py) and integrated into the node's flow. 

### Virtual Ackermann drive

Since Gazebo doesn't support closed kinematic chains, the [wheel controller](/robot_sim/src/wheel_controller.py) translates any commands published to the robot under **_/cmd_vel_** for the individual joint controllers and publishes them. The individual values for rear wheel speeds and front wheel steering angle are calculated according to the Ackermann model.
>**_Note:_** The virtual controller is by no means a guarantee that the car behaves as a real Ackermann vehicle would. For example, steering angles may diverge if the forces acting on the joints get too large.

## Documentation

The package's code is documented with rosdoc_lite, which is a light weight version of Doxygen for ROS packages. In order to view the documentation, you can open the [index file](/robot_sim/doc/html/index.html) with any browser of your liking. Example: 
```console
user@pc:~/<path_to_robot_sim>$ firefox doc/html/index.html
```

You may also find it helpful to have a look into the [source code](/robot_sim/src) itself.
