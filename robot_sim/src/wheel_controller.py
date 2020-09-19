#!/usr/bin/env python

"""!@brief WheelController module for the robot_sim car.

@details Since Gazebo is not able to simulate closed kinematic chains and a real Ackermann drive can't be implemented,
this controller simulates a virtual Ackermann drive instead. The controller takes commands from the "/cmd_vel" topic,
translates them into the respective commands for each individual joint and sends them to the car's controller
interfaces. This of course assumes that both rear wheels and the steering for each front wheel can be controller
individually.

@file Wheel controller file for the robot_sim package.

@author Martin Schuck

@date 19.09.2020
"""

import numpy as np

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class WheelController:
    """!@brief Contains the virtual Ackermann drive logic.

    @details Car parameters are hard coded. Commands are assumed to be received from the "/cmd_vel" topic. The
    controller tries to match the frequency of the velocity command topic and should be capable of > 1kHz.

    @warn The controller uses a speed modifier for car speed translation. This will not match the speed command. This is
    done for convenient car speed in the simulation. If you need exact velocity commands, please modify the
    speed_modifier to match the robot's true conversion factor.
    """
    def __init__(self):
        """!@brief WheelController constructor.

        @details Initializes hard coded car parameters and the subscriber/publishers.
        """
        self.max_speed = 5.0
        self.speed_modifier = 70
        self.max_wheel_angle = 0.36
        self.car_width = 0.38378
        self.car_length = 0.35873
        self.vel_cmd_listener = rospy.Subscriber("/cmd_vel", Twist, self._cmd_callback)
        self.rf_wheel_publisher = rospy.Publisher("/tas_car/wheel_rf_controller/command", Float64, queue_size=1)
        self.lf_wheel_publisher = rospy.Publisher("/tas_car/wheel_lf_controller/command", Float64, queue_size=1)
        self.rb_wheel_publisher = rospy.Publisher("/tas_car/wheel_rb_controller/command", Float64, queue_size=1)
        self.lb_wheel_publisher = rospy.Publisher("/tas_car/wheel_lb_controller/command", Float64, queue_size=1)

    @staticmethod
    def start():
        """!@brief Starts the WheelController.

        @details Initializes the ROS node and keeps the node spinning.
        """
        rospy.init_node('wheel_controller', anonymous=True)
        rospy.loginfo('Started virtual ackermann drive controller.')
        rospy.spin()

    def _cmd_callback(self, twist):
        """!@brief Callback for the "/cmd_vel" topic subscriber.

        @details Acts as a wrapper to the geometry logic responsible for translating the commands into target values.
        Publishes to all controllers once calculations are finished.

        @param twist Twist message received from the velocity command subscriber.

        @note Velocities will not match the exact command. Uses a simple speed modifier to translate the desired
        velocity into angular velocity for the joint. Speed modifier is chosen for convenient car speed in the
        simulation.
        """
        angle = twist.angular.z
        velocity = twist.linear.x
        if velocity < 0:
            angle *= -1  # Convert angle to geometric angle in case of negative velocity. 
        lf_angle, rf_angle, corrected_angle = self._calculate_wheel_steering(angle)
        lb_velocity, rb_velocity = self._calculate_wheel_velocity(velocity, corrected_angle)
        self.rf_wheel_publisher.publish(rf_angle)
        self.lf_wheel_publisher.publish(lf_angle)
        self.rb_wheel_publisher.publish(-rb_velocity * self.speed_modifier)  # Wheels turn the other way around in sim.
        self.lb_wheel_publisher.publish(-lb_velocity * self.speed_modifier)

    def _calculate_wheel_steering(self, angle):
        """!@brief Calculates the steering angle of the right and left front wheel.

        @details Angles are computed according to the Ackermann model. Angles are further limited to the maximum
        steering angle the robot allows for.

        @param angle The desired turning angle for the robot.

        @return Returns the angles for both front wheels as well as the commanded angle. The initial angle gets modified
        to the highest achievable angle in case the command violated car constraints.
        """
        if angle == 0:
            return 0, 0, 0
        radius = self.car_length/np.tan(abs(angle))
        sign = 1 if angle > 0 else -1
        lf_angle = sign * np.arctan(self.car_length/(radius - sign * (self.car_width/2)))
        rf_angle = sign * np.arctan(self.car_length/(radius + sign * (self.car_width/2)))
        lf_angle, rf_angle, angle = self._limit_angle(lf_angle, rf_angle, angle)
        return lf_angle, rf_angle, angle

    def _calculate_wheel_velocity(self, velocity, angle):
        """!@brief Calculates the rear wheel velocity.

        @details Velocities are computed according to the Ackermann model. Velocities are further limited to the maximum
        velocity the robot allows for.

        @param velocity The desired velocity for the robot.
        @param angle The desired turning angle for the robot.

        @return Returns the velocities for both rear wheels.
        """
        if angle == 0:
            return min(velocity, self.max_speed), min(velocity, self.max_speed)        
        radius = self.car_length/np.tan(abs(angle))
        sign = 1 if angle > 0 else -1
        lb_velocity = velocity * (1 - sign * self.car_length/(2*radius))
        rb_velocity = velocity * (1 + sign * self.car_length/(2*radius))
        lb_velocity, rb_velocity = self._limit_velocity(lb_velocity, rb_velocity)
        return lb_velocity, rb_velocity

    def _limit_angle(self, lf_angle, rf_angle, angle):
        """!@brief Limits the steering angle of the right and left front wheel.

        @details Only modifies the values in case of a violation of the maximum allowable angles.

        @param lf_angle The left front wheel angle computed from the velocity command.
        @param rf_angle The right front wheel angle computed from the velocity command.
        @param angle The desired turning angle for the robot.

        @return Returns the angles for both front wheels as well as the commanded angle. The initial angle gets modified
        to the highest achievable angle in case the command violated car constraints.
        """
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

    def _limit_velocity(self, lb_velocity, rb_velocity):
        """!@brief Limits the velocities of the right and left rear wheel.

        @details Only modifies the values in case of a violation of the maximum allowable velocity.

        @param lb_velocity The left rear wheel velocity computed from the velocity command.
        @param rb_velocity The right rear wheel velocity computed from the velocity command.

        @return Returns the velocities for both rear wheels.
        """
        frac = 1
        if abs(lb_velocity) > abs(rb_velocity) and abs(lb_velocity) > self.max_speed:
            frac = self.max_speed/abs(lb_velocity)
        elif abs(rb_velocity) > abs(lb_velocity) and abs(rb_velocity) > self.max_speed:
            frac = self.max_speed/abs(rb_velocity)
        return lb_velocity*frac, rb_velocity*frac


if __name__ == '__main__':
    wheel_controller = WheelController()
    wheel_controller.start()
