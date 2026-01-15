#!/usr/bin/env python3
"""
This is a complete example application showing how to use the mecademicpy API with Mecademic robots:
- Parse command-line arguments (e.g., desired robot IP address)
- Connect to the robot (using a "with" block that ensures proper disconnection upon exit)
- Attach a callback function to monitor motion progression through checkpoints
- Prepare the robot (ensure it's in a well-known initial state)
- Send motion commands to the robot
- Wait for the robot to complete all posted motion commands
- Handle errors (if any) and deactivate the robot
"""

import argparse
import logging
import pathlib

import mecademicpy.robot as mdr
import mecademicpy.robot_initializer as initializer
import mecademicpy.tools as tools


def complete_example():
    """Example code demonstrating advanced usage of the Mecademic robot API."""
    # Use the helper to set up default console and file logging
    tools.SetDefaultLogger(logging.INFO, f'{pathlib.Path(__file__).stem}.log')
    logger = logging.getLogger(__name__)

    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Example for using Mecademic robots with the mecademicpy module.')
    parser.add_argument('--ip',
                        dest='robot_ip',
                        type=str,
                        default='192.168.0.100',
                        help='IP address of the robot to connect to. Default is 192.168.0.100.')
    args = parser.parse_args()

    # Use a "with" block to ensure proper disconnection at the end of the block
    with initializer.RobotWithTools() as robot:

        def follow_checkpoints_example(checkpoint_id: int):
            """Callback function triggered when a motion checkpoint is reached.
            Parameters
            ----------
            checkpoint_id
                The ID of the checkpoint reached (see how we use SetCheckpoint() below)
            """
            logger.info(f'Checkpoint {checkpoint_id} reached')

        try:
            robot.Connect(address=args.robot_ip, disconnect_on_exception=False)

            # Attach the callback we defined above to follow motion progression
            robot.RegisterCallback('on_checkpoint_reached', follow_checkpoints_example)

            # Reset the robot configuration if desired
            # initializer.reset_robot_configuration(robot)

            # Use helpers to ensure the robot is in a well-known state:
            # activated, homed, and using default motion parameters.
            logger.info('Activating and homing robot...')
            initializer.reset_sim_mode(robot)
            initializer.reset_motion_queue(robot, activate_home=True)
            initializer.reset_vacuum_module(robot)

            # Wait until the robot is homed.
            robot.WaitHomed()
            logger.info('Robot is homed and ready.')

            # Send motion commands to move the robot.
            # The commands below are added to the robot's motion queue immediately,
            # without blocking Python execution. We call WaitIdle() later to wait for completion.
            logger.info('Moving the robot...')
            robot.SetCartLinVel(100)
            robot.SetBlending(50)
            starting_pos = [0] * robot.GetRobotInfo().num_joints  # Build an array matching the number of joints
            robot.MoveJoints(starting_pos)

            # Prepare robot positions that draw a square shape.
            conf: list[int] = None
            positions: list[list[float]] = []
            if tools.robot_model_is_meca500(robot.GetRobotInfo().robot_model):
                conf = [1, 1, 1]
                positions.append([0, 50, 0, 0, 0, 0])
                positions.append([0, 0, 50, 0, 0, 0])
                positions.append([0, -50, 0, 0, 0, 0])
                positions.append([0, 0, -50, 0, 0, 0])
            elif tools.robot_model_is_mcs500(robot.GetRobotInfo().robot_model):
                conf = [1]  # 4-axis robots only have the "elbow" configuration
                positions.append([-50, -50, -20, 0])
                positions.append([-50, 50, -20, 0])
                positions.append([50, 50, 20, 0])
                positions.append([50, -50, 20, 0])
            else:
                raise mdr.MecademicException(
                    f'This example script does not support this robot model ({robot.GetRobotInfo().robot_model})')

            # Send the prepared motion commands and insert a checkpoint between each to track progress.
            robot.SetConf(conf)
            for i, pos in enumerate(positions, start=1):
                robot.SetCheckpoint(i)
                robot.MoveLinRelWrf(pos)
            logger.info('Commands for drawing a square sent. Robot should now be moving.')

            # Wait until motion is complete. Without this wait, the program would exit while the robot is still moving,
            # causing it to automatically pause before finishing the motion.
            logger.info('Waiting for robot to finish moving...')
            robot.WaitIdle(60)
            logger.info('Robot finished drawing the square.')

        # Error handling examples
        except mdr.InterruptException as exception:
            # Raised when the awaited condition (e.g., WaitIdle) cannot be met due to a robot condition (e.g., error).
            logger.error(f'Robot operation was interrupted: {exception}')
        except mdr.CommunicationError as exception:
            # Raised if the TCP/IP connection with the robot cannot be established.
            logger.error(f'Failed to connect to the robot: {exception}')
        except mdr.DisconnectError as exception:
            # Raised if the TCP/IP connection with the robot is lost after being established.
            logger.error(f'Disconnected from the robot: {exception}')
        except mdr.MecademicNonFatalException as exception:
            # Raised for non-fatal internal mecademicpy module errors.
            logger.error(f'Non-fatal exception occurred: {exception}')
        except KeyboardInterrupt:
            logger.warning('Control-C pressed, quitting.')

        # Cleanup after motion completion or exception
        if robot.IsConnected():
            # Attempt to clear any existing robot error
            if robot.GetStatusRobot().error_status:
                logger.info('Robot encountered an error, attempting to clear it.')
                robot.ResetError()
                robot.ResumeMotion()
            # Deactivate the robot
            robot.DeactivateRobot()
            logger.info('Robot deactivated.')

    # Exiting the "with" block automatically disconnects from the robot.
    logger.info('Now disconnected from the robot.')


if __name__ == "__main__":
    complete_example()
