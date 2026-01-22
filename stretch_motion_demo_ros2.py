#!/usr/bin/env python3
"""
Stretch 3 Robot Motion Demo - Part #4: Stretch API (ROS 2 Version)
This script demonstrates various robot motions including arm, wrist, gripper,
head camera, and base movements using ROS 2 and the FollowJointTrajectory action.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Twist

import math
import time


class StretchMotionDemo(Node):
    def __init__(self):
        super().__init__('stretch_motion_demo')

        # Action client for joint trajectory control
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/stretch_controller/follow_joint_trajectory'
        )

        # Publisher for base velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/stretch/cmd_vel', 10)

        self.get_logger().info('Waiting for trajectory action server...')
        self.trajectory_client.wait_for_server()
        self.get_logger().info('Connected to trajectory action server!')

    def send_trajectory(self, joint_names, positions, duration_sec=3.0):
        """Send a joint trajectory goal and wait for completion."""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(seconds=duration_sec).to_msg()

        goal_msg.trajectory.points = [point]

        future = self.trajectory_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True

    def stow(self):
        """Move arm and gripper to stow position."""
        joint_names = [
            'joint_lift',
            'wrist_extension',
            'joint_wrist_yaw',
            'joint_wrist_pitch',
            'joint_wrist_roll',
            'joint_gripper_finger_left'
        ]
        # Stow positions
        positions = [0.2, 0.0, 3.14, -1.57, 0.0, 0.0]
        return self.send_trajectory(joint_names, positions, duration_sec=4.0)

    def extend_arm_and_raise_lift(self):
        """Extend arm all the way out and raise lift all the way up simultaneously."""
        joint_names = ['joint_lift', 'wrist_extension']
        positions = [1.1, 0.52]  # Max lift height and arm extension
        return self.send_trajectory(joint_names, positions, duration_sec=5.0)

    def move_wrist_yaw(self, angle):
        """Move wrist yaw motor."""
        return self.send_trajectory(['joint_wrist_yaw'], [angle], duration_sec=2.0)

    def move_wrist_pitch(self, angle):
        """Move wrist pitch motor."""
        return self.send_trajectory(['joint_wrist_pitch'], [angle], duration_sec=2.0)

    def move_wrist_roll(self, angle):
        """Move wrist roll motor."""
        return self.send_trajectory(['joint_wrist_roll'], [angle], duration_sec=2.0)

    def open_gripper(self):
        """Open the gripper."""
        return self.send_trajectory(['joint_gripper_finger_left'], [0.6], duration_sec=2.0)

    def close_gripper(self):
        """Close the gripper."""
        return self.send_trajectory(['joint_gripper_finger_left'], [-0.3], duration_sec=2.0)

    def move_head_pan(self, angle):
        """Move head pan motor."""
        return self.send_trajectory(['joint_head_pan'], [angle], duration_sec=2.0)

    def move_head_tilt(self, angle):
        """Move head tilt motor."""
        return self.send_trajectory(['joint_head_tilt'], [angle], duration_sec=2.0)

    def drive_forward(self, distance, speed=0.1):
        """Drive the robot forward by a specified distance."""
        duration = distance / speed
        twist = Twist()
        twist.linear.x = speed

        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)

        # Stop the robot
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)

    def rotate(self, angle_rad, angular_speed=0.3):
        """Rotate the robot by a specified angle in radians."""
        duration = abs(angle_rad) / angular_speed
        twist = Twist()
        twist.angular.z = angular_speed if angle_rad > 0 else -angular_speed

        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)

        # Stop the robot
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = StretchMotionDemo()

    node.get_logger().info('Starting Stretch 3 Motion Demo (ROS 2)...')
    node.get_logger().info('Make sure there is sufficient free space around the robot!')
    time.sleep(2)

    # Step 1: Move arm and gripper to 'stow' position
    node.get_logger().info('[Step 1] Moving to stow position...')
    node.stow()

    # Step 2: Extend telescoping arm and raise lift simultaneously
    node.get_logger().info('[Step 2] Extending arm and raising lift simultaneously...')
    node.extend_arm_and_raise_lift()

    # Step 3: Move all three wrist motors, one at a time
    node.get_logger().info('[Step 3] Moving wrist motors one at a time...')

    node.get_logger().info('  - Moving wrist yaw...')
    node.move_wrist_yaw(0.5)

    node.get_logger().info('  - Moving wrist pitch...')
    node.move_wrist_pitch(-0.3)

    node.get_logger().info('  - Moving wrist roll...')
    node.move_wrist_roll(0.5)

    # Step 4: Open and close the gripper
    node.get_logger().info('[Step 4] Opening gripper...')
    node.open_gripper()

    node.get_logger().info('  - Closing gripper...')
    node.close_gripper()

    # Step 5: Rotate both head motors (pan and tilt) connected to RealSense
    node.get_logger().info('[Step 5] Rotating head camera motors...')

    node.get_logger().info('  - Moving head pan...')
    node.move_head_pan(-0.5)

    node.get_logger().info('  - Moving head tilt...')
    node.move_head_tilt(-0.3)

    # Step 6: Reset everything back to stow position
    node.get_logger().info('[Step 6] Resetting to stow position...')
    node.stow()
    time.sleep(1)

    # Step 7: Drive forward 0.5 meters
    node.get_logger().info('[Step 7] Driving forward 0.5 meters...')
    node.drive_forward(0.5)
    time.sleep(1)

    # Step 8: Rotate 180 degrees (pi radians)
    node.get_logger().info('[Step 8] Rotating 180 degrees...')
    node.rotate(math.pi)
    time.sleep(1)

    # Step 9: Drive forward 0.5 meters (back to starting position)
    node.get_logger().info('[Step 9] Driving forward 0.5 meters (returning to start)...')
    node.drive_forward(0.5)

    node.get_logger().info('Motion demo complete!')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
