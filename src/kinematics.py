import rospy
import moveit_msgs


class ForwardKinematics():
    def __init__(self, ns="", link="kmriiwa_link_ee",reference_frame_id="kmriiwa_base_footprint"):
        # prepare service for forward kinematics calculation
        self.fk_srv = rospy.ServiceProxy('/kmriiwa/compute_fk', moveit_msgs.srv.GetPositionFK)
        # wait for service to become available
        self.fk_srv.wait_for_service()
        rospy.loginfo('GetPositionFK service is available')
        self.link = link
        self.reference_frame_id = reference_frame_id
        rospy.loginfo('FK using Link: ' + link)
        rospy.loginfo('FK using Reference Frame Id: ' + reference_frame_id)


    def getPose(self, robot, robot_state):
        '''
        Calculate the pose of the link related to the 
        frame reference_frame_id for the given 
        robot state robot_state.
        Return type is geometry_msgs.msg.Pose
        '''
        # prepare request
        gfkr = moveit_msgs.srv.GetPositionFKRequest()
        gfkr.header.frame_id = self.reference_frame_id
        gfkr.fk_link_names = robot.get_link_names()
        gfkr.robot_state = robot_state

        # call service
        result = self.fk_srv.call(gfkr)

        for link_name,pose_stamped in zip(result.fk_link_names,result.pose_stamped):
            if link_name == self.link:
                return pose_stamped.pose


class InverseKinematics():
    def __init__(self, ns="", group_name='kmriiwa_manipulator',link="kmriiwa_link_ee"):
        # prepare service for inverse kinematics calculation
        self.ik_srv = rospy.ServiceProxy('/kmriiwa/compute_ik', moveit_msgs.srv.GetPositionIK)
        # wait for service to become available
        self.ik_srv.wait_for_service()
        rospy.loginfo('GetPositionIK service is available')
        self.group_name = group_name
        self.link = link
        rospy.loginfo('IK Using Move Group Name = ' + group_name)
        rospy.loginfo('IK Using Link Name = ' + link)


    def getJointAngles(self, goal, current_robot_state):
        '''
        Calculate the joint angles for the given goal.
        current_robot_state is the seed state for the IK solver.
        Return type is moveit_msgs.msg.RobotState
        '''
        # prepare request
        gikr = moveit_msgs.srv.GetPositionIKRequest()
        gikr.ik_request.group_name = self.group_name
        current_robot_state.is_diff = True
        gikr.ik_request.robot_state = current_robot_state
        gikr.ik_request.ik_link_name = self.link
        gikr.ik_request.avoid_collisions = False
        gikr.ik_request.pose_stamped = goal
        gikr.ik_request.timeout = rospy.Duration(1.0)

        # call service
        result = self.ik_srv.call(gikr)
        if(result.error_code.val != 1):
            print("Error: ", result.error_code.val)

        return result.solution
