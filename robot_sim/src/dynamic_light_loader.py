#!/usr/bin/env python

import numpy as np
from pathlib import Path
import time

import rospy
from tf.transformations import quaternion_from_euler
from gazebo_msgs.srv import SetLightProperties, GetLightProperties, SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion

from utils import test_light_array
from position_checker import PositionChecker

class DynamicLightLoader:
    
    def __init__(self, light_array):
        rospy.init_node(name='DynamicLightLoader')
        self._gazebo_model_spawn_service = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self._gazebo_model_delete_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.package_path = Path(__file__).resolve().parent.parent
        self.light_model_xml = None
        self._init_light_model_xml()
        self.light_array = light_array
        self.active_lights = set()
        self._default_orientation = Quaternion(*quaternion_from_euler(0., 0., 0.))
        self.position_checker = PositionChecker(light_positions=self.light_array[:, 0:2], callbacks=[self.checker_callback])

    def _init_light_model_xml(self):
        with open(self.package_path.parent.joinpath('airport_sim', 'gazebo', 'models', 'airport_light', 'model.sdf'), 'r') as f:
            self.light_model_xml = f.read()

    def start(self):
        self.position_checker.start()
        rospy.loginfo("Started DynamicLightLoader.")
        rospy.spin()

    def checker_callback(self, light_indices):
        for index in light_indices:
            if not index in self.active_lights:
                try:
                    position = Point(self.light_array[index][0], self.light_array[index][1], 0.3)
                    self._gazebo_model_spawn_service('light'+str(index), self.light_model_xml, '', Pose(position, self._default_orientation), 'world')
                    self.active_lights.add(index)
                except rospy.ServiceException as e:
                    rospy.loginfo(f"Light spawn service failed. Error code: {e}")
        for index in self.active_lights - set(light_indices):
            try:
                self._gazebo_model_delete_service('light'+str(index))
                self.active_lights.remove(index)
            except rospy.ServiceException as e:
                rospy.loginfo(f"Light delete service failed. Error code: {e}")

    def load_lights(self, positions):
        rospy.loginfo("Loading lights into the simulation.")
        for idx, position in enumerate(positions):
            pose = Pose(Point(x=position[0], y=position[1], z=0.3), self._default_orientation)  # 0.3 to make the light clearly visible in the sim.
            try:
                self._gazebo_model_spawn_service('light'+str(idx), self.light_model_xml, '', pose, 'world')
            except rospy.ServiceException as e:
                rospy.loginfo(f"Light spawn service failed. Error code: {e}")

    def delete_lights(self, positions):
        for idx, _ in enumerate(positions):
            try:
                self._gazebo_model_delete_service('light'+str(idx))
            except rospy.ServiceException as e:
                rospy.loginfo(f"Light delete service failed. Error code: {e}")


if __name__ == '__main__':
    dynamic_loader = DynamicLightLoader(test_light_array)
    # dynamic_loader.load_lights(test_light_array)
    dynamic_loader.start()
    # dynamic_loader.delete_lights(test_light_array)
    # light_controller.test_light_switch()
