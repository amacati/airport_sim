#!/usr/bin/env python

"""!@brief Script to emulate a GPS signal for other nodes.

@details As of this point, ROS Noetic does not support the Hector GPS plugin. Instead, this module makes use of the
ground truth position data from the robot and republishes it with additional gaussian noise. In order to emulate a
GPS, the conversion function for the particular simulation still has to be implemented. The logic behind this module is
that other nodes on the robot receive a pseudo GPS signal to work with in their localization algorithms. This signal
can't be provided from outside, so the robot itself publishes a noisy estimation of what a GPS signal would look like on
its current position.

@file GPSEmulator class and node script.

@author Martin Schuck

@date 19.09.2020
"""


from pathlib import Path
import numpy as np
import yaml

import rospy
from nav_msgs.msg import Odometry


class GPSEmulator:
    """!@brief Contains the logic for GPS emulation.

    @details The DynamicLightLoader makes use of the published ground truth position data. This data is assumed to be
    published at ROS topic: "/ground_truth/odom". The GPS signal is then published under "/tas_car/gps".

    @note If there is no ground truth data, just add the P3D Gazebo plugin to your URDF file.
    """

    def __init__(self):
        """!@brief GPSEmulator constructor.

        @details Initializes the ROS node and sets up the subscriber/publisher. Also loads the config for the GPS noise.
        """
        rospy.init_node('GPSEmulator', anonymous=True)
        self.position_mean = [0.0, 0.0]
        self.position_sigma = 0.0
        self.orientation_mean = 0.0
        self.orientation_sigma = 0.0
        self.linear_velocity_mean = [0.0, 0.0]
        self.linear_velocity_sigma = 0.0
        self.angular_velocity_mean = 0.0
        self.angular_velocity_sigma = 0.0
        self.load_config()
        self.start_callback = False
        self.ground_truth_subscriber = rospy.Subscriber("/ground_truth/odom", Odometry, self._odometry_callback)
        self.gps_publisher = rospy.Publisher("/tas_car/gps", Odometry, queue_size=1)

    def load_config(self):
        """!@brief Loads the config file for the GPS noise.

        @details The file can be found in the robot_sim/config/gps_emulator.yaml file. Edit the entries as necessary.

        @warn The yaml file is not checked for correctness of data types. Please only edit existing entries.
        """
        path = Path(__file__).resolve().parent.parent.joinpath('config', 'gps_emulator.yaml')
        try:
            with open(path, 'r') as f:
                config = yaml.safe_load(f)
            self.position_mean = config['position']['mean']
            self.position_sigma = config['position']['sigma']
            self.orientation_mean = config['orientation']['mean']
            self.orientation_sigma = config['orientation']['sigma']
            self.linear_velocity_mean = config['linear_velocity']['mean']
            self.linear_velocity_sigma = config['linear_velocity']['sigma']
            self.angular_velocity_mean = config['angular_velocity']['mean']
            self.angular_velocity_sigma = config['angular_velocity']['sigma']
            rospy.loginfo('GPS emulator successfully applied GPS config.')
        except (FileNotFoundError, yaml.scanner.ScannerError):
            rospy.logwarn('GPS emulator file missing or malformed!')
            rospy.loginfo('Config not available, reverting to default.')

    def _odometry_callback(self, odom):
        """!@brief Callback for the ground truth subscriber.

        @details Adds noise to the received odometry data and republishes it. Also converts the odometry to a proper GPS
        signal if @see _odom_to_gps_conversion is implemented.

        @param odom ROS Odometry message from the subscriber.
        """
        if self.start_callback:
            odom.pose.pose.position.x += np.random.randn(1)*self.position_sigma + self.position_mean[0]
            odom.pose.pose.position.y += np.random.randn(1)*self.position_sigma + self.position_mean[1]
            odom.pose.pose.orientation.z += np.random.randn(1)*self.orientation_sigma + self.orientation_mean
            odom.twist.twist.linear.x += np.random.randn(1)*self.linear_velocity_sigma + self.linear_velocity_mean[0]
            odom.twist.twist.linear.y += np.random.randn(1)*self.linear_velocity_sigma + self.linear_velocity_mean[1]
            odom.twist.twist.angular.z += np.random.randn(1)*self.angular_velocity_sigma + self.angular_velocity_mean
            try:
                odom = self._odom_to_gps_conversion(odom)
            except NotImplementedError:
                pass
            self.gps_publisher.publish(odom)

    @staticmethod
    def _odom_to_gps_conversion(odom):
        """!@brief Placeholder for the conversion to GPS.

        @details Implementing the function will automatically include it in @see _odometry_callback.
        """
        raise NotImplementedError

    def start(self):
        """!@brief Starts the GPS emulator.

         @details Unlocks the odometry callback and keeps the node spinning.
        """
        rospy.loginfo('Started GPS emulator.')
        self.start_callback = True
        rospy.spin()


if __name__ == '__main__':
    gps_emulator = GPSEmulator()
    gps_emulator.start()
