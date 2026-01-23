#!/usr/bin/env python3
"""
Stretch 3 Robot Motion Demo - Part #4: Stretch API (ROS 2 Version)
This script demonstrates various robot motions including arm, wrist, gripper,
head camera, and base movements using ROS 2 and the FollowJointTrajectory action.

BEFORE RUNNING:
    Launch the Stretch driver first:
    ros2 launch stretch_core stretch_driver.launch.py mode:=position
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

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

        # Subscriber for joint states
        self.joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.get_logger().info('Waiting for trajectory action server...')
        self.get_logger().info('Make sure you launched: ros2 launch stretch_core stretch_driver.launch.py mode:=position')

        server_available = self.trajectory_client.wait_for_server(timeout_sec=10.0)
        if not server_available:
            self.get_logger().error('Failed to connect to action server!')
            self.get_logger().error('Please launch the stretch driver first:')
            self.get_logger().error('  ros2 launch stretch_core stretch_driver.launch.py mode:=position')
            raise RuntimeError('Action server not available')

        self.get_logger().info('Connected to trajectory action server!')

    def joint_state_callback(self, msg):
        """Store latest joint state."""
        self.joint_state = msg

    def send_trajectory(self, joint_names, positions, duration_sec=3.0):
        """Send a joint trajectory goal and wait for completion."""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.header.frame_id = 'base_link'
        goal_msg.trajectory.header.stamp = self.get_clock().now().to_msg()

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
        """Move arm and gripper to stow position (based on official stow_command)."""
        # Wait for joint states to be available
        while not self.joint_state or not self.joint_state.position:
            self.get_logger().info("Waiting for joint states message to arrive")
            rclpy.spin_once(self, timeout_sec=0.1)
            continue

        self.get_logger().info('Stowing...')
        joint_state = self.joint_state

        # Create two trajectory points
        stow_point1 = JointTrajectoryPoint()
        stow_point2 = JointTrajectoryPoint()

        duration1 = Duration(seconds=0.0)
        duration2 = Duration(seconds=4.0)

        stow_point1.time_from_start = duration1.to_msg()
        stow_point2.time_from_start = duration2.to_msg()

        # Get current joint positions
        lift_index = joint_state.name.index('joint_lift')
        arm_index = joint_state.name.index('wrist_extension')
        wrist_yaw_index = joint_state.name.index('joint_wrist_yaw')

        joint_value1 = joint_state.position[lift_index]
        joint_value2 = joint_state.position[arm_index]
        joint_value3 = joint_state.position[wrist_yaw_index]

        # Point 1: current positions
        stow_point1.positions = [joint_value1, joint_value2, joint_value3]
        # Point 2: stow positions
        stow_point2.positions = [0.2, 0.0, 3.14]

        # Create and send trajectory goal
        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw']
        trajectory_goal.trajectory.points = [stow_point1, stow_point2]

        future = self.trajectory_client.send_goal_async(trajectory_goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Stow goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('Goal sent')
        return True

    def extend_arm_and_raise_lift(self):
        """Extend arm all the way out and raise lift all the way up simultaneously."""
        joint_names = ['joint_lift', 'wrist_extension']
        positions = [1.0, 0.5]  # Max lift height ~1.1m and arm extension ~0.52m
        return self.send_trajectory(joint_names, positions, duration_sec=5.0)

    def reset_wrist_and_head(self):
        """Reset wrist pitch/roll, head pan/tilt, and gripper to neutral before stowing."""
        joint_names = ['joint_wrist_pitch', 'joint_wrist_roll', 'joint_head_pan', 'joint_head_tilt', 'joint_gripper_finger_left']
        positions = [0.0, 0.0, 0.0, 0.0, -0.1]  # Added gripper closed position
        return self.send_trajectory(joint_names, positions, duration_sec=3.0)

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
        return self.send_trajectory(['joint_gripper_finger_left'], [0.15], duration_sec=2.0)

    def close_gripper(self):
        """Close the gripper."""
        return self.send_trajectory(['joint_gripper_finger_left'], [-0.15], duration_sec=2.0)

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
    node.get_logger().info('  - First resetting wrist pitch/roll and head...')
    node.reset_wrist_and_head()
    node.get_logger().info('  - Now stowing arm...')
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
