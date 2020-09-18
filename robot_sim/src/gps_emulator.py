#!/usr/bin/env python

from pathlib import Path
import numpy as np
import yaml

import rospy
from nav_msgs.msg import Odometry


class GPSEmulator:

    def __init__(self):
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
        self.ground_truth_subscriber = rospy.Subscriber("/ground_truth/odom", Odometry, self.odometry_callback)
        self.gps_publisher = rospy.Publisher("/tas_car/gps", Odometry, queue_size=1)

    def load_config(self):
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

    def odometry_callback(self, odometry):
        if self.start_callback:
            odometry.pose.pose.position.x += np.random.randn(1)*self.position_sigma + self.position_mean[0]
            odometry.pose.pose.position.y += np.random.randn(1)*self.position_sigma + self.position_mean[1]
            odometry.pose.pose.orientation.z += np.random.randn(1)*self.orientation_sigma + self.orientation_mean
            odometry.twist.twist.linear.x += np.random.randn(1)*self.linear_velocity_sigma + self.linear_velocity_mean[0]
            odometry.twist.twist.linear.y += np.random.randn(1)*self.linear_velocity_sigma + self.linear_velocity_mean[1]
            odometry.twist.twist.angular.z += np.random.randn(1)*self.angular_velocity_sigma + self.angular_velocity_mean
            self.gps_publisher.publish(odometry)

    def odom_to_gps_conversion(self, odometry):
        raise NotImplementedError

    def start(self):
        rospy.loginfo('Started GPS Emulator.')
        self.start_callback = True
        rospy.spin()


if __name__ == '__main__':
    gps_emulator = GPSEmulator()
    gps_emulator.start()
