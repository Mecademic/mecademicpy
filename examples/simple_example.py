#!/usr/bin/env python3
"""
This is a simple (minimal) example showing how to use the mecademicpy API with a Mecademic robot.
This example connects to a robot at IP address 192.168.0.100, activates it, and performs a basic motion.
For a more complete example, see "complete_example.py".
"""

import mecademicpy.robot as mdr


def simple_example():
    """Simple example for connecting to a Mecademic robot and moving it."""

    # Use a "with" block to ensure the robot disconnects properly at the end.
    with mdr.Robot() as robot:
        # YOU MAY USE YOUR ROBOT'S IP ADDRESS HERE:
        robot.Connect(address='192.168.0.100', disconnect_on_exception=False)

        # Prepare the robot for operation.
        print('Activating and homing robot...')
        robot.ActivateAndHome()
        # Wait until the robot is fully homed.
        robot.WaitHomed()
        print('Robot is homed and ready.')

        # Send simple motion commands to the robot.
        # The commands below are added to the robot's motion queue immediately,
        # without blocking Python execution. We call WaitIdle() later to wait for completion.
        print('Moving the robot...')
        robot.SetJointVel(20)
        # Build test positions compatible with both 4- and 6-axis robots.
        if robot.GetRobotInfo().num_joints == 4:
            pos_1 = [0, 0, 0, 0]
            pos_2 = [-10, -10, -10, -10]
        else:
            pos_1 = [0, 0, 0, 0, 0, 0]
            pos_2 = [-10, -10, -10, -10, -10, -10]
        robot.MoveJoints(pos_1)
        robot.MoveJoints(pos_2)
        robot.MoveJoints(pos_1)

        # Wait until motion is complete. Otherwise, this program would exit while the robot is still moving,
        # causing the robot to automatically pause before the motion is completed.
        print('Waiting for robot to finish moving...')
        robot.WaitIdle(60)

    # Exiting the "with" block automatically disconnects from the robot.
    print('Now disconnected from the robot.')


if __name__ == "__main__":
    simple_example()
