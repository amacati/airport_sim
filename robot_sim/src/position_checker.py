#!/usr/bin/env python

import numpy as np
import time
from numba import jit

import rospy
from nav_msgs.msg import Odometry


class PositionChecker:

    def __init__(self, light_positions, *, distance_threshold=30, max_lights=20, callbacks=[]):
        self.light_positions = light_positions
        self.distance_threshold = distance_threshold
        self.max_lights = max_lights
        self.start_callback = False
        self.ground_truth_subscriber = rospy.Subscriber("/ground_truth/odom", Odometry, self.odometry_callback)
        self.callbacks = callbacks
        self.__init_numba_jit()

    def start(self):
        self.start_callback = True
        rospy.loginfo("Started PositionChecker.")

    def odometry_callback(self, odometry):
        if self.start_callback:
            position = np.array([odometry.pose.pose.position.x, odometry.pose.pose.position.y])
            active_lights = self.__check_lights_boost(self.light_positions, position, self.distance_threshold, self.max_lights)
            active_lights = active_lights[active_lights>=0]
            for callback in self.callbacks:
                callback(active_lights)

    @staticmethod
    @jit(nopython=True, cache=True)
    def __check_lights_boost(light_positions, position, threshold, max_lights):
        light_distance = np.sqrt(np.sum((light_positions - position)**2, axis=1))
        if np.sum(light_distance<=threshold) >= max_lights:
            load_index = np.argsort(light_distance)[:max_lights].flatten()  # np.argpartition sadly not supported by numba.
        else:
            tmp = np.argwhere(light_distance<=threshold).flatten()
            load_index = np.ones(max_lights, dtype=np.int64) * -1  # Mark invalid indices.
            load_index[0:tmp.shape[0]] = tmp
        return load_index

    def __init_numba_jit(self):
        rospy.loginfo('Compiling numba functions for PositionChecker.')
        init_light_pos = np.zeros((10,2))
        init_pos = np.ones(2)
        self.__check_lights_boost(init_light_pos, init_pos, self.distance_threshold, self.max_lights)
        rospy.loginfo('Numba compilation completed.')
