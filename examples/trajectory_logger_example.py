#!/usr/bin/env python3

##############################################################################
# This is a simple example application to show how to use mecademicpy API
# to capture real-time trajectory of the robot while it is moving
#
# This application will generate file logger_example_traj_file.zip that
# contains captured robot real-time data in a CSV file
##############################################################################

import logging
import pathlib

import mecademicpy.robot as mdr
import mecademicpy.tools as tools

# Use tool to setup default console and file logger
tools.SetDefaultLogger(logging.INFO, f'{pathlib.Path(__file__).stem}.log')
logger = logging.getLogger(__name__)


# Define a callback function to print test progress based on reached checkpoints
def on_checkpoint_reached(id):
    print(f'Loop {id}...')


# Instantiate a robot instance (to control one robot)
# (use "with" block to ensure proper disconnection at end of block)
with mdr.Robot() as robot:
    # Attach callback functions
    callbacks = mdr.RobotCallbacks()
    callbacks.on_checkpoint_reached = on_checkpoint_reached
    robot.RegisterCallbacks(callbacks=callbacks, run_callbacks_in_separate_thread=True)

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

        # Wait until robot is homed.
        robot.WaitHomed()

        # Configure robot's behavior to desired speed/accel/etc.
        print('Configuring robot\'s behavior...')
        robot.SetJointVel(50)
        robot.SetJointAcc(50)
        robot.SetBlending(50)

        # Move to starting position
        print('Moving to a well-known starting position...')
        robot.MoveJoints(0, 0, 0, 0, 0, 0)

        # Wait until robot is idle (reached starting position)
        robot.WaitIdle()
        # Wait until end of one monitoring cycle before logging (data more consistent this way)
        robot.WaitEndOfCycle()

        # Start running a test script while logging robot data to a csv file
        print('Start running test script while logging to csv file...')
        # Configure monitoring interval and required fields to capture in file
        with robot.FileLogger(0.001, fields=["TargetJointPos", "JointPos"], file_name='logger_example_traj_file'):

            # Perform 2 simple joint moves, few loops
            for i in range(0, 2):
                robot.SetCheckpoint(i + 1)
                robot.MoveJoints(30, 25, 20, 15, 10, 5)
                robot.MoveJoints(-30, -25, -20, -15, -10, -5)

            # Wait until robot is idle (above commands finished executing) before stopping logging.
            robot.WaitIdle(60)
            # Exiting the "FileLogger" scope automatically stops logging

        print('Done!')

    except Exception as exception:
        # Attempt to clear error if robot is in error.
        if robot.GetStatusRobot().error_status:
            print(exception)
            print('Robot has encountered an error, attempting to clear...')
            robot.ResetError()
            robot.ResumeMotion()
        else:
            raise

# At the end of the "with" block, robot is automatically disconnected
print('Now disconnected from the robot.')
