#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
from math import pi, tau
from copy import deepcopy
from actionlib import SimpleActionClient
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from time import sleep

class ControladorTiago():
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('tiago_controller', anonymous=True)
        self.__talk_client = SimpleActionClient('/tts', TtsAction)

        self.__robot = moveit_commander.robot.RobotCommander()
        self.__planning_scene = moveit_commander.planning_scene_interface.PlanningSceneInterface()
        self.__arm_group_name = "arm"
        self.__arm_torso_group_name = "arm_torso"
        self.__gripper_group_name = "gripper"
        self.__arm_commander = moveit_commander.move_group.MoveGroupCommander(self.__arm_group_name)
        self.arm_torso_commander = moveit_commander.move_group.MoveGroupCommander(self.__arm_torso_group_name)
        self.gripper_commander = moveit_commander.move_group.MoveGroupCommander(self.__gripper_group_name)
        sleep(2)

    def talk(self, text: str, duration_in_secs: int) -> None:
        """Function to make the robot talk using TTS (Text to Speech)
        Args:
            text (str): Text to be spoken by the robot.
            duration_in_secs (int): Duration in seconds to wait for the TTS action to complete.
        """
        goal = TtsGoal()
        goal.rawtext.text = text
        goal.rawtext.lang_id = "en_GB"
        self.__talk_client.send_goal_and_wait(goal, rospy.Duration(duration_in_secs), rospy.Duration(duration_in_secs))

    def move_gripper(self, width: float) -> bool:
        """Move the gripper to the desired width.
        Args:
            width (float): Desired gripper width in meters. 0.001 closed, 0.088 totally open."""
        if width > 0.088: width=0.088
        elif width < 0.002: width=0.002
        self.gripper_commander.go([width/2, width/2])

    def set_velocity_and_acceleration_arm(self, velocity: float, acceleration: float) -> None:
        """Function to set the velocity and acceleration of the arm and torso.
        Args:
            velocity (float): Velocity scaling factor (0.0 to 1.0).
            acceleration (float): Acceleration scaling factor (0.0 to 1.0).            
        """
        velocity = velocity if velocity > 0 and velocity <= 1.0 else 0.1
        acceleration = acceleration if acceleration > 0 and acceleration <= 1.0 else 0.1
        self.__arm_commander.set_max_velocity_scaling_factor(velocity)
        self.__arm_commander.set_max_acceleration_scaling_factor(acceleration)
        self.arm_torso_commander.set_max_velocity_scaling_factor(velocity)
        self.arm_torso_commander.set_max_acceleration_scaling_factor(acceleration)

    def set_velocity_and_acceleration_gripper(self, velocity: float, acceleration: float) -> None:
        """Function to set the velocity and acceleration of the gripper.
        Args:
            velocity (float): Velocity scaling factor (0.0 to 1.0).
            acceleration (float): Acceleration scaling factor (0.0 to 1.0).            
        """
        velocity = velocity if velocity > 0 and velocity <= 1.0 else 0.1
        acceleration = acceleration if acceleration > 0 and acceleration <= 1.0 else 0.1
        self.gripper_commander.set_max_velocity_scaling_factor(velocity)
        self.gripper_commander.set_max_acceleration_scaling_factor(acceleration)

    def move_joints(self, joints: list) -> bool:
        """Function to move the arm and torso to a specific joint configuration in radians. This is, the angles of the joints.
        Args:
            joints (list): List of joint angles in radians.
        Returns:
            bool: True if the movement was successful, False otherwise.
        """
        return self.arm_torso_commander.go(joints)
    
    def get_current_joints(self) -> list:
        """Function to get the current joint angles of the arm and torso.
        Returns:
            list: List of current joint angles in radians.
        """
        return deepcopy(self.arm_torso_commander.get_current_joint_values())
    
    def get_current_pose(self) -> Pose:
        """Function to get the current pose of the end effector (gripper) in the world frame.
        Returns:
            Pose: Current pose of the end effector (gripper) in the world frame in meters.
        """
        return self.__arm_commander.get_current_pose().pose

    def move_to_pose(self, pose: Pose) -> bool:
        """Function to move the arm and torso to a specific pose in the world frame.
        Args:
            pose (Pose): Target pose in the world frame in meters.
        Returns:
            bool: True if the movement was successful, False otherwise.
        """
        pose_aux = PoseStamped()
        pose_aux.header.frame_id = self.arm_torso_commander.get_end_effector_link()
        pose_aux.pose = pose
        self.arm_torso_commander.set_pose_target(pose_aux)
        return self.arm_torso_commander.go()
    
    def move_to_position(self, position: list) -> bool:
        """Function to move the arm and torso to a specific position in the world frame.
        Args:
            position (list): Target position in the world frame in meters.
        Returns:
            bool: True if the movement was successful, False otherwise.
        """
        self.arm_torso_commander.set_position_target(position)
        return self.arm_torso_commander.go()

    def move_in_straight_line(self, list_of_poses: list, strict: bool=True) -> bool:
        """Function to move the arm and torso in a straight line to a list of poses in the world frame.
        Args:
            list_of_poses (list): List of target poses in the world frame in meters.
            strict (bool): If True, the robot will only execute the path if the whole trajectory is valid. 
            If False, it will execute the fraction of the path that is valid.
        Returns:
            bool: True if the movement was successful, False otherwise.
        """
        (plan, fraction) = self.arm_torso_commander.compute_cartesian_path(
            list_of_poses, 0.01
        )

        if fraction == 1.0:
            return self.arm_torso_commander.execute(plan)
        
        if fraction > 0 and strict:
            return self.arm_torso_commander.execute(plan)
        
        return False
    
if __name__=='__main__':
    controlador = ControladorTiago()
    a = controlador.gripper_commander.get_active_joints()
    a = controlador.gripper_commander.get_joint_value_target()
    controlador.gripper_commander.go([0.03])
    b=3
    # pose_list = []
    # current_pose = controlador.get_current_pose()
    # current_pose.position.x += 0.2
    # pose_list.append(deepcopy(current_pose))
    # success = controlador.move_in_straight_line(pose_list)
    # success = controlador.move_to_position([current_pose.position.x, current_pose.position.y, current_pose.position.z])
    # current_joints = controlador.get_current_joints()
    # current_joints[1] += pi/8
    # success = controlador.move_joints(current_joints)
    # pass