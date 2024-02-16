#!/usr/bin/env python3

##############################################################################
# This is a simple example application to show how to use mecademicpy API
# to connect and move a Mecademic robot
##############################################################################

import logging
import pathlib

import mecademicpy.robot as mdr
import mecademicpy.tools as tools

# Use tool to setup default console and file logger
tools.SetDefaultLogger(logging.INFO, f'{pathlib.Path(__file__).stem}.log')
logger = logging.getLogger(__name__)

# (use "with" block to ensure proper disconnection at end of block)
with mdr.Robot() as robot:
    # CHECK THAT IP ADDRESS IS CORRECT! #
    try:
        robot.Connect(address='192.168.0.100')
    except mdr.CommunicationError as e:
        logger.info(f'Robot failed to connect. Is the IP address correct? {e}')
        raise e

    try:
        # Send the commands to get the robot ready for operation.
        logger.info('Activating and homing robot...')
        robot.ActivateRobot()
        robot.Home()

        # Pause execution until robot is homed.
        robot.WaitHomed()
        logger.info('Robot is homed and ready.')

        # Send motion commands to have the robot draw out a square.
        if tools.robot_model_is_meca500(robot.GetRobotInfo().robot_model):
            robot.MovePose(200, 0, 300, 0, 90, 0)
            robot.MovePose(200, 100, 300, 0, 90, 0)
            robot.MovePose(200, 100, 100, 0, 90, 0)
            robot.MovePose(200, -100, 100, 0, 90, 0)
            robot.MovePose(200, -100, 300, 0, 90, 0)
            robot.MovePose(200, 0, 300, 0, 90, 0)
        elif robot.GetRobotInfo().robot_model == mdr.MxRobotModel.MX_ROBOT_MODEL_MCS500_R1:
            robot.MovePose(190, 0, 50, 0)
            robot.MovePose(100, 0, 50, 0)
            robot.MovePose(100, 90, 50, 0)
            robot.MovePose(190, 90, 50, 0)
            robot.MovePose(190, 0, 50, 0)
        else:
            raise mdr.MecademicException(
                f'This example script does not support this robot model ({robot.GetRobotInfo().robot_model})')
        logger.info('Commands for drawing a square sent. Robot should now be moving.')

        # Insert a delay in robot's motion queue between drawing square and moving back
        robot.Delay(1)

        if tools.robot_model_is_meca500(robot.GetRobotInfo().robot_model):
            # Return the robot to folded position.
            robot.MoveJoints(0, -60, 60, 0, 0, 0)

        # Wait until checkpoint is reached. Without this wait, the script would immediately
        # reach the DeactivateRobot and Disconnect command, which stops the motion.
        logger.info('Waiting for robot to finish moving...')
        robot.WaitIdle(60)
        logger.info('Robot finished drawing square.')

    except Exception as exception:
        # Attempt to clear error if robot is in error.
        if robot.GetStatusRobot().error_status:
            logger.info(exception)
            logger.info('Robot has encountered an error, attempting to clear...')
            robot.ResetError()
            robot.ResumeMotion()
        else:
            raise

    # Deactivate the robot.
    robot.DeactivateRobot()
    logger.info('Robot is deactivated.')

# At the end of the "with" block, robot is automatically disconnected
logger.info('Now disconnected from the robot.')
