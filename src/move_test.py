#!/usr/bin/env python
import copy

import numpy as np
import math

import sys
import rospy
import moveit_commander
from std_msgs.msg import Float64MultiArray

from kinematics import ForwardKinematics, InverseKinematics

from scipy.spatial.transform import Rotation as R


class Simulation:
    def __init__(self, user_choices=None):
        # Specific to each robot. Check /config/robot_params.yaml
        group_name = "kmriiwa_manipulator"
        end_link = "kmriiwa_link_ee"
        self.base_link = "kmriiwa_base_footprint"
        namespace = "/kmriiwa"
        namespace_extra = "/arm"
        robot_description = "/kmriiwa/robot_description"
        self.joint_names = ['kmriiwa_joint_1', 'kmriiwa_joint_2', 'kmriiwa_joint_3','kmriiwa_joint_4','kmriiwa_joint_5', 'kmriiwa_joint_6', 'kmriiwa_joint_7']

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander(robot_description=robot_description, ns=namespace+namespace_extra)
        self.scene = moveit_commander.PlanningSceneInterface(ns=namespace)
        self.move_group = moveit_commander.MoveGroupCommander(group_name, robot_description=robot_description, ns=namespace)
        self.forwardKinematics = ForwardKinematics(ns=namespace, link=end_link, reference_frame_id=self.base_link)
        self.inverseKinematics = InverseKinematics(ns=namespace, group_name=group_name, link=end_link)
        self.rate = rospy.Rate(5)
        self.path_completed = False

    
    def run_single_point(self):
        current_robot_state = self.robot.get_current_state()
        current_pose = self.forwardKinematics.getPose(self.robot, current_robot_state)
        
        p0 = copy.deepcopy(current_pose)
        
        p1 = copy.deepcopy(p0)
        p1.position.x -= 0.1

        carthesian = True
        
        if carthesian:
            self.run_carthesian([p1])
        elif not self.carthesian:
            self.run_joint([p1])


    def run_joint(self, waypoints):
        for point in waypoints:
            (success, path, _, _) = self.move_group.plan(point)
            self.move_group.go(wait = True)

    
    def run_carthesian(self, waypoints, step_size=0.001):
        path, fraction = self.move_group.compute_cartesian_path(
            waypoints, 
            eef_step=step_size, 
            jump_threshold=0.0
        )
        # dataframe_planned_points = self.get_data_frame_theorethical_trajectory(path)
        self.move_group.execute(path, wait = True)
    

def main():
    rospy.init_node("move_group_test", anonymous=False)
    
    simulation = Simulation()
    
    simulation.run_single_point()

    moveit_commander.roscpp_shutdown()
    
if __name__ == "__main__":
    try: 
        main()
    except rospy.ROSInterruptException:
        pass
