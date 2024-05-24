"""
This file implement unit tests for the trajectory analyzer module
"""
from pathlib import PurePath
from tempfile import TemporaryDirectory

import mecademicpy.robot_trajectory_files as robot_files


def test_file_integrity():
    filename = "rob_traj_file_for_integ"
    file_test_dir = PurePath(__file__).parent
    path_to_original_file = PurePath.joinpath(file_test_dir, filename + '.zip')
    robot_trajectories = robot_files.RobotTrajectories.from_file(path_to_original_file)
    with TemporaryDirectory() as temp_path:
        robot_trajectories.to_file(filename + '_copy', file_path=temp_path)
        robot_trajectories_copy = robot_files.RobotTrajectories.from_file(
            PurePath.joinpath(PurePath(temp_path), filename + '_copy' + '.zip'))
        assert robot_trajectories == robot_trajectories_copy
