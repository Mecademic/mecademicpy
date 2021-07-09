import pytest
import mecademic.robot_trajectory_files as robot_files
from tempfile import TemporaryDirectory
from pathlib import PurePath


def test_file_integrity():
    filename = "rob_traj_file_for_integ"
    file_test_dir = PurePath(__file__).parent
    path_to_original_file = PurePath.joinpath(file_test_dir, filename + '.zip')
    robot_trajectories = robot_files.RobotTrajectories.from_file(path_to_original_file)
    with TemporaryDirectory() as tempPath:
        robot_trajectories.to_file(filename + '_copy', file_path=tempPath)
        robot_trajectories_copy = robot_files.RobotTrajectories.from_file(
            PurePath.joinpath(PurePath(tempPath), filename + '_copy' + '.zip'))
        assert robot_trajectories == robot_trajectories_copy