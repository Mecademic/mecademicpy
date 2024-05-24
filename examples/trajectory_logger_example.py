#!/usr/bin/env python3
"""
This is a simple example application to show how to use mecademicpy API
to capture real-time trajectory of the robot while it is moving.

This application will generate file logger_example_traj_file.zip that
contains captured robot real-time data in a CSV file.
"""

import logging
import pathlib

import mecademicpy.robot as mdr
import mecademicpy.tools as tools


def trajectory_logger_example():
    """ Example that moves Mecademic robot while capturing the trajectory into a csv file """
    # Use tool to setup default console and file logger
    tools.SetDefaultLogger(logging.INFO, f'{pathlib.Path(__file__).stem}.log')
    logger = logging.getLogger(__name__)

    # Define a callback function to print test progress based on reached checkpoints
    # pylint: disable=redefined-builtin
    def on_checkpoint_reached(id):
        logger.info(f'Loop {id}...')

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
            logger.info('Connected to robot')
        except mdr.CommunicationError as e:
            logger.info(f'Robot failed to connect. Is the IP address correct? {e}')
            raise e

        try:
            # Send the commands to get the robot ready for operation.
            logger.info('Activating and homing robot...')
            robot.ActivateRobot()
            robot.Home()

            # Wait until robot is homed.
            robot.WaitHomed()

            # Configure robot's behavior to desired speed/accel/etc.
            logger.info('Configuring robot\'s behavior...')
            robot.SetJointVel(50)
            robot.SetJointAcc(50)
            robot.SetBlending(50)

            # Move to starting position
            logger.info('Moving to a well-known starting position...')
            if tools.robot_model_is_meca500(robot.GetRobotInfo().robot_model):
                robot.MoveJoints(0, 0, 0, 0, 0, 0)
            else:
                robot.MoveJoints(10, -10, 0, 0)

            # Wait until robot is idle (reached starting position)
            robot.WaitIdle()
            # Wait until end of one monitoring cycle before logging (data more consistent this way)
            robot.WaitEndOfCycle()

            # Start running a test script while logging robot data to a csv file
            logger.info('Start running test script while logging to csv file...')
            # Configure monitoring interval and required fields to capture in file
            with robot.FileLogger(0.001, fields=["TargetJointPos", "JointPos"], file_name='logger_example_traj_file'):

                # Perform 2 simple joint moves, few loops
                for i in range(0, 2):
                    robot.SetCheckpoint(i + 1)
                    if tools.robot_model_is_meca500(robot.GetRobotInfo().robot_model):
                        robot.MoveJoints(30, 25, 20, 15, 10, 5)
                        robot.MoveJoints(-30, -25, -20, -15, -10, -5)
                    else:
                        robot.MoveJoints(30, 25, -20, 15)
                        robot.MoveJoints(-30, -25, 0, -15)

                # Wait until robot is idle (above commands finished executing) before stopping logging.
                robot.WaitIdle(60)
                # Exiting the "FileLogger" scope automatically stops logging

            logger.info('Done!')

        #pylint: disable=broad-exception-caught
        except Exception as exception:
            # Attempt to clear error if robot is in error.
            if robot.GetStatusRobot().error_status:
                logger.error(exception)
                logger.error('Robot has encountered an error, attempting to clear...')
                robot.ResetError()
                robot.ResumeMotion()
            else:
                raise exception

    # At the end of the "with" block, robot is automatically disconnected
    logger.info('Now disconnected from the robot.')


if __name__ == "__main__":
    trajectory_logger_example()
