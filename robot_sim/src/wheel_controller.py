#!/usr/bin/env python

import numpy as np
import time

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class WheelController:
    
    def __init__(self):
        self.max_speed = 5.0
        self.speed_modifier = 70
        self.max_wheel_angle = 0.36
        self.car_width = 0.38378
        self.car_length = 0.35873
        self.vel_cmd_listener = rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback)
        self.rf_wheel_publisher = rospy.Publisher("/tas_car/wheel_rf_controller/command", Float64, queue_size=1)
        self.lf_wheel_publisher = rospy.Publisher("/tas_car/wheel_lf_controller/command", Float64, queue_size=1)
        self.rb_wheel_publisher = rospy.Publisher("/tas_car/wheel_rb_controller/command", Float64, queue_size=1)
        self.lb_wheel_publisher = rospy.Publisher("/tas_car/wheel_lb_controller/command", Float64, queue_size=1)

    def start(self):
        rospy.init_node('wheel_controller', anonymous=True)
        rospy.loginfo('Started virtual ackermann drive controller.')
        rospy.spin()

    def cmd_callback(self, twist):
        angle = twist.angular.z
        velocity = twist.linear.x
        if velocity < 0:
            angle *= -1  # Convert angle to geometric angle in case of negative velocity. 
        lf_angle, rf_angle, corrected_angle = self.calculate_wheel_steering(angle)
        lb_velocity, rb_velocity = self.calculate_wheel_velocity(velocity, corrected_angle)
        self.rf_wheel_publisher.publish(rf_angle)
        self.lf_wheel_publisher.publish(lf_angle)
        self.rb_wheel_publisher.publish(-rb_velocity * self.speed_modifier)  # Wheels turn the other way around in sim.
        self.lb_wheel_publisher.publish(-lb_velocity * self.speed_modifier)

    def calculate_wheel_steering(self, angle):
        if angle == 0:
            return 0, 0, 0
        radius = self.car_length/np.tan(abs(angle))
        sign = 1 if angle > 0 else -1
        lf_angle = sign * np.arctan(self.car_length/(radius - sign * (self.car_width/2)))
        rf_angle = sign * np.arctan(self.car_length/(radius + sign * (self.car_width/2)))
        lf_angle, rf_angle, angle = self.limit_angle(lf_angle, rf_angle, angle)
        return lf_angle, rf_angle, angle

    def calculate_wheel_velocity(self, velocity, angle):
        if angle == 0:
            return min(velocity, self.max_speed), min(velocity, self.max_speed)        
        radius = self.car_length/np.tan(abs(angle))
        sign = 1 if angle > 0 else -1
        lb_velocity = velocity * (1 - sign * self.car_length/(2*radius))
        rb_velocity = velocity * (1 + sign * self.car_length/(2*radius))
        lb_velocity, rb_velocity = self.limit_velocity(lb_velocity, rb_velocity)
        return lb_velocity, rb_velocity

    def limit_angle(self, lf_angle, rf_angle, angle):
        if lf_angle > self.max_wheel_angle:
            lf_angle = self.max_wheel_angle
            radius = self.car_length/np.tan(lf_angle) + self.car_width/2
            angle = np.arctan(self.car_length/radius)
            rf_angle = np.arctan(self.car_length/(radius + self.car_width/2))
        elif rf_angle < -self.max_wheel_angle:
            rf_angle = -self.max_wheel_angle
            radius = self.car_length/np.tan(abs(rf_angle)) + self.car_width/2
            angle = -np.arctan(self.car_length/(radius + self.car_width/2))
            lf_angle = -np.arctan(self.car_length/(radius + self.car_width/2))
        return lf_angle, rf_angle, angle

    def limit_velocity(self, lb_velocity, rb_velocity):
        frac = 1
        if abs(lb_velocity) > abs(rb_velocity) and abs(lb_velocity) > self.max_speed:
            frac = self.max_speed/abs(lb_velocity)
        elif abs(rb_velocity) > abs(lb_velocity) and abs(rb_velocity) > self.max_speed:
            frac = self.max_speed/abs(rb_velocity)
        return lb_velocity*frac, rb_velocity*frac


if __name__ == '__main__':
    wheel_controller = WheelController()
    wheel_controller.start()
