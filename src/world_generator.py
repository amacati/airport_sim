from pathlib import Path
import time

import rospy
from tf.transformations import quaternion_from_euler
from gazebo_msgs.srv import SetLightProperties, GetLightProperties, SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion

from utils import test_light_dict2

class GazeboLightController:
    
    def __init__(self):
        self._gazebo_light_get_service = rospy.ServiceProxy('/gazebo/get_light_properties', GetLightProperties)
        self._gazebo_light_set_service = rospy.ServiceProxy('/gazebo/set_light_properties', SetLightProperties)
        self._gazebo_light_spawn_service = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self._gazebo_light_delete_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.package_path = Path(__file__).resolve().parent.parent
        self.light_model_xml = None
        self._init_light_model_xml()
        self.light_collection = list()
        self._default_light_orient = Quaternion(*quaternion_from_euler(0., 0., 0.))

    def _init_light_model_xml(self):
        with open(self.package_path.joinpath('gazebo', 'models', 'airport_light', 'model.sdf'), 'r') as f:
            self.light_model_xml = f.read()

    def load_lights(self, light_pos_dict):
        rospy.loginfo("Loading lights into the simulation.")
        for light_name, position in light_pos_dict.items():
            if light_name in self.light_collection:
                rospy.loginfo("Duplicate light spawn requested. Please make sure all lights have unique identifiers! Omitting..")
            else:
                self.light_collection.append(light_name)  # Save all light model names to delete them later.
                light_pose = Pose(Point(x=position[0], y=position[1], z=0.3), self._default_light_orient)  # 0.3 to make the light clearly visible in the sim.
                try:
                    self._gazebo_light_spawn_service(light_name, self.light_model_xml, '', light_pose, 'world')
                except rospy.ServiceException as e:
                    rospy.loginfo(f"Light spawn service failed. Error code: {e}")

    def delete_lights(self):
        for light_name in self.light_collection:
            try:
                self._gazebo_light_delete_service(light_name)
            except rospy.ServiceException as e:
                rospy.loginfo(f"Light delete service failed. Error code: {e}")


if __name__ == '__main__':
    light_controller = GazeboLightController()
    light_controller.load_lights(test_light_dict2)
    light_controller.delete_lights()
    # light_controller.test_light_switch()

    