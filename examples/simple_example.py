#!/usr/bin/env python3
import os
import sys

import mecademic.Robot as mdr

robot = mdr.Robot()

# CHECK THAT IP ADDRESS IS CORRECT! #
try:
    robot.Connect(address='192.168.0.100')
except mdr.CommunicationError as e:
    print(f'Robot failed to connect. Is the IP address correct? {e}')
    raise e

try:
    # Send the commands to get the robot ready for operation.
    robot.ActivateRobot()
    robot.Home()

    # Pause execution until robot is homed.
    print('Waiting for robot to be homed.', flush=True)
    robot.WaitHomed(timeout=60)  # Add a timeout of 60 seconds in case something fails.
    print('Robot is homed and ready.', flush=True)

    robot.Delay(1)

    # Send motion commands to have the robot draw out a square.
    robot.MovePose(200, 0, 300, 0, 90, 0)
    robot.MovePose(200, 100, 300, 0, 90, 0)
    robot.MovePose(200, 100, 100, 0, 90, 0)
    robot.MovePose(200, -100, 100, 0, 90, 0)
    robot.MovePose(200, -100, 300, 0, 90, 0)
    robot.MovePose(200, 0, 300, 0, 90, 0)
    print('Commands for drawing a square sent.', flush=True)

    robot.Delay(1)

    # Return the robot to folded position.
    robot.MoveJoints(0, -60, 60, 0, 0, 0)

    # Wait until checkpoint is reached. Without this wait, the script would immediately
    # reach the DeactivateRobot and Disconnect command, which stops the motion.
    robot.WaitIdle(60)
    print('Robot finished drawing square.', flush=True)

except Exception as exception:
    # Attempt to clear error if robot is in error.
    if robot.GetRobotState().error_status:
        print(exception)
        print('Robot has encountered an error, attempting to clear...', flush=True)
        robot.ResetError()
        robot.ResumeMotion()
    else:
        raise

# Deactivate and disconnect from the robot.
robot.DeactivateRobot()
robot.Disconnect()
print('Robot is deactivated and disconnected.', flush=True)