#!/usr/bin/env python3
"""
Stretch 3 Robot Motion Demo - Part #4: Stretch API
This script demonstrates various robot motions including arm, wrist, gripper,
head camera, and base movements.
"""

import math
import time
import stretch_body.robot


def main():
    # Initialize the robot
    robot = stretch_body.robot.Robot()
    if not robot.startup():
        print("Failed to start robot")
        return

    # Ensure robot is homed before starting
    if not robot.is_homed():
        print("Robot is not homed. Please home the robot first.")
        robot.stop()
        return

    print("Starting Stretch 3 Motion Demo...")
    print("Make sure there is sufficient free space around the robot!")
    time.sleep(2)

    # Step 1: Move arm and gripper to 'stow' position (single line of code)
    print("\n[Step 1] Moving to stow position...")
    robot.stow()
    time.sleep(3)

    # Step 2: Extend telescoping arm all the way out and raise lift all the way up (simultaneously)
    print("\n[Step 2] Extending arm and raising lift simultaneously...")
    robot.arm.move_to(0.52)  # Max arm extension ~0.52m
    robot.lift.move_to(1.1)  # Max lift height ~1.1m
    robot.push_command()
    robot.wait_command()

    # Step 3: Move all three wrist motors, one at a time
    print("\n[Step 3] Moving wrist motors one at a time...")

    # Wrist yaw
    print("  - Moving wrist yaw...")
    robot.end_of_arm.move_to('wrist_yaw', 0.5)
    robot.push_command()
    robot.wait_command()

    # Wrist pitch
    print("  - Moving wrist pitch...")
    robot.end_of_arm.move_to('wrist_pitch', -0.3)
    robot.push_command()
    robot.wait_command()

    # Wrist roll
    print("  - Moving wrist roll...")
    robot.end_of_arm.move_to('wrist_roll', 0.5)
    robot.push_command()
    robot.wait_command()

    # Step 4: Open and close the gripper
    print("\n[Step 4] Opening gripper...")
    robot.end_of_arm.move_to('stretch_gripper', 50)  # Open position
    robot.push_command()
    robot.wait_command()

    print("  - Closing gripper...")
    robot.end_of_arm.move_to('stretch_gripper', -50)  # Closed position
    robot.push_command()
    robot.wait_command()

    # Step 5: Rotate both head motors (pan and tilt) connected to RealSense
    print("\n[Step 5] Rotating head camera motors...")

    # Head pan
    print("  - Moving head pan...")
    robot.head.move_to('head_pan', -0.5)
    robot.push_command()
    robot.wait_command()

    # Head tilt
    print("  - Moving head tilt...")
    robot.head.move_to('head_tilt', -0.3)
    robot.push_command()
    robot.wait_command()

    # Step 6: Reset everything back to stow position
    print("\n[Step 6] Resetting to stow position...")
    robot.stow()
    time.sleep(4)

    # Step 7: Drive forward 0.5 meters
    print("\n[Step 7] Driving forward 0.5 meters...")
    robot.base.translate_by(0.5)
    robot.push_command()
    robot.wait_command()

    # Step 8: Rotate 180 degrees (pi radians)
    print("\n[Step 8] Rotating 180 degrees...")
    robot.base.rotate_by(math.pi)
    robot.push_command()
    robot.wait_command()

    # Step 9: Drive forward 0.5 meters (back to starting position)
    print("\n[Step 9] Driving forward 0.5 meters (returning to start)...")
    robot.base.translate_by(0.5)
    robot.push_command()
    robot.wait_command()

    print("\nMotion demo complete!")

    # Shutdown the robot
    robot.stop()


if __name__ == "__main__":
    main()
