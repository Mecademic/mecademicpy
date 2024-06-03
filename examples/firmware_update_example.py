#!/usr/bin/env python3
"""
This is a simple example application to show how to update the firmware
of a Mecademic robot using the 'mecademicpy' Python package
"""

from __future__ import annotations

import argparse
import logging
import pathlib

import mecademicpy.robot as mdr
import mecademicpy.tools as tools


def firmware_update_example():
    """ Example for updating Mecademic robot firmware with Python code """
    # Minimal command-line parsing to collect IP address of the robot to update and
    # firmware file to install
    parser = argparse.ArgumentParser(description='Update Mecademic robot firmware.')
    parser.add_argument('--firmware',
                        dest='firmware_file_path',
                        type=str,
                        help='Path to the firmware update file to install.')
    parser.add_argument('--ip',
                        dest='robot_ip',
                        type=str,
                        default='192.168.0.100',
                        help='IP address of the robot to install the firmware on. Default is 192.168.0.100.')

    args = parser.parse_args()

    # Use tool to setup default console and file logger
    tools.SetDefaultLogger(logging.INFO, f'{pathlib.Path(__file__).stem}.log')

    # Validate that the provided firmware file exists
    if not args.firmware_file_path:
        raise ValueError(f'Firmware file path was not found (use --firmware command-line option)')
    if not pathlib.Path(args.firmware_file_path).is_file():
        raise FileNotFoundError(f'Firmware file {args.firmware_file_path} was not found')

    # (use "with" block to ensure proper disconnection at end of block)
    with mdr.Robot() as robot:
        try:
            print(f'Starting to install firmware file {args.firmware_file_path} on robot {args.robot_ip}')
            # Start the firmware update
            robot.Connect(args.robot_ip)
            robot.UpdateRobot(args.firmware_file_path)
            print(f'Successfully installed firmware file {args.firmware_file_path} on robot {args.robot_ip}')
        #pylint: disable=broad-exception-caught
        except Exception as e:
            # Print error if firmware update failed
            print(f'Update failed: {e}')
            exit(-1)


if __name__ == "__main__":
    firmware_update_example()
