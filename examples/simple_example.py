#!/usr/bin/env python3
import os
import sys

import mecademicpy.robot as mdr

# (use "with" block to ensure proper disconnection at end of block)
with mdr.Robot() as robot:
    # CHECK THAT IP ADDRESS IS CORRECT! #
    try:
        robot.Connect(address='192.168.0.100')
        print('Connected to robot')
    except mdr.CommunicationError as e:
        print(f'Robot failed to connect. Is the IP address correct? {e}')
        raise e

    try:
        # Send the commands to get the robot ready for operation.
        print('Activating and homing robot...')
        robot.ActivateRobot()
        robot.Home()

        # Pause execution until robot is homed.
        robot.WaitHomed()
        print('Robot is homed and ready.')

        # Send motion commands to have the robot draw out a square.
        robot.MovePose(200, 0, 300, 0, 90, 0)
        robot.MovePose(200, 100, 300, 0, 90, 0)
        robot.MovePose(200, 100, 100, 0, 90, 0)
        robot.MovePose(200, -100, 100, 0, 90, 0)
        robot.MovePose(200, -100, 300, 0, 90, 0)
        robot.MovePose(200, 0, 300, 0, 90, 0)
        print('Commands for drawing a square sent. Robot should now be moving.')

        # Insert a delay in robot's motion queue between drawing square and moving back
        robot.Delay(1)

        # Return the robot to folded position.
        robot.MoveJoints(0, -60, 60, 0, 0, 0)

        # Wait until checkpoint is reached. Without this wait, the script would immediately
        # reach the DeactivateRobot and Disconnect command, which stops the motion.
        print('Waiting for robot to finish moving...')
        robot.WaitIdle(60)
        print('Robot finished drawing square.')

    except Exception as exception:
        # Attempt to clear error if robot is in error.
        if robot.GetStatusRobot().error_status:
            print(exception)
            print('Robot has encountered an error, attempting to clear...')
            robot.ResetError()
            robot.ResumeMotion()
        else:
            raise

    # Deactivate the robot.
    robot.DeactivateRobot()
    print('Robot is deactivated.')

# At the end of the "with" block, robot is automatically disconnected
print('Now disconnected from the robot.')
