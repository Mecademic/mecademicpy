#!/usr/bin/env python3
"""
This is a simple example application to show how to use mecademicpy API
to capture real-time trajectory of the robot while it is moving.

This application will generate file logger_example_traj_file.zip that
contains captured robot real-time data in a CSV file.
"""

import logging
import pathlib

import numpy as np
import pandas as pd

import mecademicpy.robot as mdr
import mecademicpy.robot_initializer as initializer
import mecademicpy.tools as tools


def example_prepare_robot(robot: initializer.RobotWithTools):
    """ Prepare the robot by resetting it to a well known starting state for this example application"""
    # Send the commands to get the robot ready for operation.
    robot.logger.info('Activating and homing robot...')
    initializer.reset_sim_mode(robot)
    initializer.reset_motion_queue(robot, activate_home=True)

    # Wait until robot is homed.
    robot.WaitHomed()

    # Configure robot's behavior to desired speed/accel/etc.
    robot.logger.info('Configuring robot\'s behavior...')
    robot.SetJointVel(50)
    robot.SetJointAcc(50)
    robot.SetBlending(50)

    # Move to starting position
    robot.logger.info('Moving to a well-known starting position...')
    if tools.robot_model_is_meca500(robot.GetRobotInfo().robot_model):
        robot.MoveJoints(0, 0, 0, 0, 0, 0)
    else:
        robot.MoveJoints(10, -10, 0, 0)

    # Wait until robot is idle (reached starting position)
    robot.WaitIdle()
    # Wait until end of one monitoring cycle before logging (data more consistent this way)
    robot.WaitEndOfCycle()


def example_move_and_capture_trajectory(robot: initializer.RobotWithTools):
    """ This function sends move commands to the robot while capturing the resulting trajectory
    (robot real-time data) into a file """
    # Start running a test program while logging robot data to a csv file
    robot.logger.info('Start running test program while logging to csv file...')
    # Configure monitoring interval and required fields to capture in file
    with robot.FileLogger(
            0.001,
            fields=[
                # Example by robot real-time status codes
                mdr.MxRobotStatusCode.MX_ST_RT_CART_POS,
                # Example by RobotRtData class field name (same as mdr.MxRobotStatusCode.MX_ST_RT_CART_VEL)
                'rt_cart_vel'
            ],
            file_name='logger_example_traj_file',
            keep_captured_trajectory=True):

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

    robot.logger.info(f'Done capturing in file {robot.GetCapturedTrajectoryPath()}')


def example_analyze_captured_trajectory(robot: initializer.RobotWithTools):
    """ This function is an example showing how to get the latest captured robot trajectory and perform some
    data analysis on its contents.

    In this example, we will identify the fastest linear velocity through the captured robot movement
    """
    # Get the captured data frame (stored as first element of output_dfs)
    captured_df: pd.DataFrame = robot.GetCapturedTrajectory().get_captured_data()
    # Find the desired column names (here the x/y/z cartesian velocity)
    col_prefix = mdr.rt_data_field_by_status_code[mdr.MxRobotStatusCode.MX_ST_RT_CART_VEL].col_prefix
    vel_cols = [col for col in captured_df.columns if col.startswith(col_prefix)]
    xyz_vel_df = captured_df[vel_cols[:3]]  # Get only the first 3 columns (x, y and z)
    # Compute speed (Euclidean norm) for each row
    speed = np.linalg.norm(xyz_vel_df.to_numpy(), axis=1)
    # Find the maximum speed
    max_speed = speed.max()
    robot.logger.info(f"Maximum robot speed during the captured trajectory is: {max_speed:.1f} mm/s")


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
    with initializer.RobotWithTools() as robot:

        # Register our checkpoint reached callback
        robot.RegisterCallback('on_checkpoint_reached', on_checkpoint_reached)

        # CHECK THAT IP ADDRESS IS CORRECT! #
        try:
            robot.Connect(address='192.168.0.100')
            logger.info('Connected to robot')
        except mdr.CommunicationError as e:
            logger.info(f'Robot failed to connect. Is the IP address correct? {e}')
            raise e

        try:
            example_prepare_robot(robot)
            example_move_and_capture_trajectory(robot)
            example_analyze_captured_trajectory(robot)

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
