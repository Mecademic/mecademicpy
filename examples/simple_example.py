#!/usr/bin/env python3
"""
This is a simple example application to show how to use mecademicpy API
to connect and move a Mecademic robot.
"""

import logging
import pathlib

import mecademicpy.robot as mdr
import mecademicpy.robot_initializer as initializer
import mecademicpy.tools as tools


def simple_example():
    """ Simple example for connecting to Mecademic robot and moving it  """
    # Use tool to setup default console and file logger
    tools.SetDefaultLogger(logging.INFO, f'{pathlib.Path(__file__).stem}.log')
    logger = logging.getLogger(__name__)

    # (use "with" block to ensure proper disconnection at end of block)
    with initializer.RobotWithTools() as robot:
        try:
            # YOU MAY USE YOUR ROBOT IP ADDRESS HERE:
            robot.Connect(address='192.168.0.100', disconnect_on_exception=False)

            # Reset the robot configuration (joint limits, work zone, etc.)
            #initializer.reset_robot_configuration(robot)

            # Send the commands to get the robot ready for operation.
            logger.info('Activating and homing robot...')
            initializer.reset_sim_mode(robot)
            initializer.reset_motion_queue(robot, activate_home=True)
            initializer.reset_vacuum_module(robot)

            # Pause execution until robot is homed.
            robot.WaitHomed()
            logger.info('Robot is homed and ready.')

            # Send motion commands to have the robot draw out a square.
            logger.info('Moving the robot...')
            robot.SetCartLinVel(100)
            robot.SetBlending(50)
            starting_pos = [0] * robot.GetRobotInfo().num_joints  # Build array with size matching number of joints
            robot.MoveJoints(*starting_pos)  # Move joints using the unrolled array as arguments
            if tools.robot_model_is_meca500(robot.GetRobotInfo().robot_model):
                robot.SetConf(1, 1, 1)
                robot.MoveLinRelWrf(0, 50, 0, 0, 0, 0)
                robot.MoveLinRelWrf(0, 0, 50, 0, 0, 0)
                robot.MoveLinRelWrf(0, -50, 0, 0, 0, 0)
                robot.MoveLinRelWrf(0, 0, -50, 0, 0, 0)
            elif robot.GetRobotInfo().robot_model == mdr.MxRobotModel.MX_ROBOT_MODEL_MCA250_R1:
                robot.SetConf(1, 1, 1)
                robot.MoveLinRelWrf(0, 30, 0, 0, 0, 0)
                robot.MoveLinRelWrf(0, 0, 30, 0, 0, 0)
                robot.MoveLinRelWrf(0, -30, 0, 0, 0, 0)
                robot.MoveLinRelWrf(0, 0, -30, 0, 0, 0)
            elif robot.GetRobotInfo().robot_model == mdr.MxRobotModel.MX_ROBOT_MODEL_MCA1000_R1:
                robot.SetConf(1, 1, 1)
                robot.MoveLinRelWrf(0, 50, 0, 0, 0, 0)
                robot.MoveLinRelWrf(0, 0, 50, 0, 0, 0)
                robot.MoveLinRelWrf(0, -50, 0, 0, 0, 0)
                robot.MoveLinRelWrf(0, 0, -50, 0, 0, 0)
            elif robot.GetRobotInfo().robot_model == mdr.MxRobotModel.MX_ROBOT_MODEL_MCS500_R1:
                robot.MoveLinRelWrf(-50, -50, -20, 0)
                robot.MoveLinRelWrf(-50, 50, -20, 0)
                robot.MoveLinRelWrf(50, 50, 20, 0)
                robot.MoveLinRelWrf(50, -50, 20, 0)
            else:
                raise mdr.MecademicException(
                    f'This example script does not support this robot model ({robot.GetRobotInfo().robot_model})')
            logger.info('Commands for drawing a square sent. Robot should now be moving.')

            # Wait until checkpoint is reached. Without this wait, the script would immediately
            # reach the DeactivateRobot and Disconnect command, which stops the motion.
            logger.info('Waiting for robot to finish moving...')
            robot.WaitIdle(60)
            logger.info('Robot finished drawing square.')

        # Error management example
        except mdr.InterruptException as exception:
            logger.error(f'Robot operation was interrupted: {exception}...')
        except mdr.CommunicationError as exception:
            logger.error(f'Failed to connect to the robot: {exception}...')
        except mdr.DisconnectError as exception:
            logger.error(f'Disconnected from the robot: {exception}...')
        except mdr.MecademicNonFatalException as exception:
            logger.error(f'Robot exception occurred: {exception}...')
        except KeyboardInterrupt:
            logger.warning('Control-C pressed, quitting')
            pass

        # Deactivate the robot.
        if robot.IsConnected():
            # Attempt to clear error if robot is in error.
            if robot.GetStatusRobot().error_status:
                logger.info('Robot has encountered an error, attempting to clear...')
                robot.ResetError()
                robot.ResumeMotion()
            robot.DeactivateRobot()
            logger.info('Robot is deactivated.')

    # At the end of the "with" block, robot is automatically disconnected
    logger.info('Now disconnected from the robot.')


if __name__ == "__main__":
    simple_example()
