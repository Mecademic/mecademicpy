#!/usr/bin/env python3
"""
This file implements mecademicpy's trajectory logger that is used to capture robot's real-time data and save it
as a csv file.
This file contains internal code to mecademicpy. Users should use the trajectory capture API provided in robot.py
"""
from __future__ import annotations

import queue
import time
from pathlib import PurePath

import pandas as pd

# pylint: disable=wildcard-import,unused-wildcard-import
from .robot_classes import *
from .robot_trajectory_files import RobotTrajectories


class _RobotTrajectoryLogger:
    """Class to handle logging robot state to file.

    Attributes
    ----------
    file_name : str
        Name of file produced by logger
    fields : dict of strings
        Fields to be logged. Key: attribute name in 'RobotState'. Value: Equivalent UpperCamelCase string or enum value
        used in 'SetRealTimeMonitoring'
    command_queue : queue
        Queue to store sent commands.
    done_logging: bool
        'write_fields' wont log more robot states when this is True. Set to True by 'end_log'
    logging_commands: bool
        Indicate if sent commands are being logged
    expanded_fields:
        Elements of 'fields', but expanded to have a name for each sub-element of corresponding robot states
    data_dict:
        Keys: timestamps. Values: robot state stored at moment corresponding to timestamp
    robot_trajectories: RobotTrajectories object
        Contains robot states logged data and information about the robot used during logging
    """

    def __init__(self,
                 robot_info: RobotInfo,
                 robot_rt_data: RobotRtData,
                 logged_fields: List[MxRobotStatusCode] = None,
                 file_name: str = None,
                 file_path: str = None,
                 record_time: bool = True,
                 monitoring_interval: float = None):
        """Initialize class.

        Parameters
        ----------
        robot_info : RobotInfo
            Contains robot information.
        robot_rt_data : RobotRtData object
            Contains state of robot.
        logged_fields : list of strings
            List of fields to be logged (See StartLogging() for details)
        file_name: string or None
            Log file name
            If None, file name will be built with date/time and robot information (robot type, serial, version).
        file_path : string or None
            Path to save the zipped file that contains logged data + robot info in, respectively, csv and json file.
            If not provided, file will be saved in working directory.
        record_time : bool
            If true, current time will also be recorded in the text file. (Time is also available in filename.)
        monitoring_interval: float
            Indicates rate at which state from robot is received on monitor port. Unit: seconds
        """
        current_date_time = time.strftime('%Y-%m-%d-%H-%M-%S')

        serial_number_or_blank = ('_serial_' + robot_info.serial) if robot_info.serial else ""

        # Add unique name to file path.
        if file_name:
            self.file_name = file_name
        else:
            self.file_name = (f'{robot_info.model}_R{robot_info.revision}_'
                              f'v{robot_info.version.short_version}_'
                              f'log_{current_date_time}{serial_number_or_blank}')

        self.file_path = file_path

        # If fields argument is None, log all compatible fields.
        self.fields: list[RobotRtDataField] = []
        if logged_fields is None:

            if robot_info.rt_message_capable:
                for attr_name in vars(robot_rt_data):
                    if attr_name.startswith('rt_'):
                        rt_data_field = rt_data_field_by_name[attr_name]
                        if rt_data_field is not None:
                            self.fields.append(rt_data_field)
                        else:
                            raise ValueError(f'Field {attr_name} is not defined in rt_data_field_by_name')
            else:
                # Only the following fields are available if platform is not rt monitoring capable.
                self.fields.append(rt_data_field_by_name['rt_target_joint_pos'])
                self.fields.append(rt_data_field_by_name['rt_target_cart_pos'])
        else:
            logged_fields = normalize_robot_rt_data_field(logged_fields)
            for logged_field in logged_fields:
                rt_data_field: RobotRtDataField = None
                if isinstance(logged_field, MxRobotStatusCode):
                    rt_data_field = rt_data_field_by_status_code[logged_field]
                else:
                    raise ValueError(
                        f"Unknown trajectory logger field name '{logged_field}'. See ROBOT_RT_DATA_FIELDS for supported field names or real-time data codes."
                    )
                if rt_data_field is not None:
                    self.fields.append(rt_data_field)

        # Set attributes.
        self.command_queue: queue.Queue = queue.Queue()
        self.done_logging = False
        self.logging_commands = True
        self.expanded_fields: list = []
        self.data_dict: dict = dict()  # Key: timestamp, Value: List of all corresponding robot_rt_data values
        self.robot_trajectories = RobotTrajectories()

        # Write robot information.
        # Maybe robot information could be stored as a RobotInfo object in robot_trajectories?
        self.robot_trajectories.robot_context.robot_information.append(dict())
        for attr in ['model', 'revision', 'version']:
            self.robot_trajectories.robot_context.robot_information[0][attr] = f'{getattr(robot_info, attr)}'
        if robot_info.serial is not None:
            self.robot_trajectories.robot_context.robot_information[0]['serial_number'] = f'{robot_info.serial}'
        if record_time:
            self.robot_trajectories.robot_context.robot_information[0]['time_recorded'] = f'{current_date_time}'
        if monitoring_interval:
            self.robot_trajectories.robot_context.robot_information[0]['monitoring_interval'] = f'{monitoring_interval}'

        # Write headers for logged data
        self.write_field_and_element_headers(robot_info, robot_rt_data)

    def get_timestamp_data(self, robot_rt_data: RobotRtData, field_name: str):
        """ Return timestamp data object associated with the specific field (or None).

        Parameters
        ----------
        robot_rt_data : RobotRtData object
            Current state of robot to get timestamp_data from
        field_name : String
            Name of the field to get timestamp_data for.

        """
        if field_name == 'rt_accelerometer':
            index = 5  # For now, only index 5 supported (joint 5's accelerometer)
            accel_dict = getattr(robot_rt_data, field_name)
            if index not in accel_dict:
                # Accelerometer not found, return a dummy timestamped data
                timestamp = robot_rt_data.rt_target_joint_pos.timestamp
                return TimestampedData(timestamp, [0, 0, 0], RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL)
            field_attr = accel_dict[index]
        else:
            field_attr = getattr(robot_rt_data, field_name)
        return field_attr

    def _build_io_fields(self, prefix: str, io_type: str, count: int) -> list[str]:
        """Build a list of digital IO field names

        Parameters
        ----------
        prefix : str
            IO bank prefix (ex: 'ioModule')
        io_type : str
            IO type ('Input' or 'Output')
        count : int
            Number of IOs of this type in this bank

        Returns
        -------
        list[str]
            _description_
        """
        io_fields = []
        for idx in range(count):
            io_fields.append(f'{prefix}{io_type}_{idx + 1}')
        return io_fields

    def write_field_and_element_headers(self, robot_info: RobotInfo, robot_rt_data: RobotRtData):
        """Write the full field name and element name in each column.

        Parameters
        ----------
        robot_info : RobotInfo
            Information about the robot, such as model name and number of joints.
        robot_rt_data : RobotRtData object
            Contains state of robot.
        """

        def assemble_with_prefix(field_name, names):
            return [field_name + '_' + str(x) for x in names]

        # Write full name for each field.
        for rt_data_field in self.fields:
            if (rt_data_field.field_name.endswith('joint_pos') or rt_data_field.field_name.endswith('joint_vel')
                    or rt_data_field.field_name.endswith('joint_torq')):
                # Write field name followed by joint number. For example: "TargetJointPos_1".
                self.expanded_fields.extend(assemble_with_prefix(rt_data_field.col_prefix,
                                                                 range(robot_info.num_joints)))
            elif rt_data_field.field_name.endswith('cart_pos') or rt_data_field.field_name.endswith(
                    'wrf') or rt_data_field.field_name.endswith('trf'):
                if robot_info.num_joints == 4:
                    self.expanded_fields.extend(assemble_with_prefix(rt_data_field.col_prefix,
                                                                     ['X', 'Y', 'Z', 'Gamma']))
                else:
                    self.expanded_fields.extend(
                        assemble_with_prefix(rt_data_field.col_prefix, ['X', 'Y', 'Z', 'Alpha', 'Beta', 'Gamma']))
            elif rt_data_field.field_name.endswith('cart_vel'):
                if robot_info.num_joints == 4:
                    self.expanded_fields.extend(
                        assemble_with_prefix(rt_data_field.col_prefix, ['X_Dot', 'Y_Dot', 'Z_Dot', 'Omega_Z']))
                else:
                    self.expanded_fields.extend(
                        assemble_with_prefix(rt_data_field.col_prefix,
                                             ['X_Dot', 'Y_Dot', 'Z_Dot', 'Omega_X', 'Omega_Y', 'Omega_Z']))
            elif rt_data_field.field_name.endswith('rt_accelerometer'):
                self.expanded_fields.extend(assemble_with_prefix(rt_data_field.col_prefix, ['X', 'Y', 'Z']))
            elif rt_data_field.field_name.endswith('conf_turn'):
                self.expanded_fields.append(rt_data_field.col_prefix)
            elif rt_data_field.field_name.endswith('conf'):
                if robot_info.num_joints == 4:
                    self.expanded_fields.extend(assemble_with_prefix(rt_data_field.col_prefix, ['Elbow']))
                else:
                    self.expanded_fields.extend(
                        assemble_with_prefix(rt_data_field.col_prefix, ['Shoulder', 'Elbow', 'Wrist']))
            elif rt_data_field.field_name.endswith('checkpoint'):
                self.expanded_fields.append(rt_data_field.col_prefix)
            elif rt_data_field.field_name.endswith('rt_effective_time_scaling'):
                self.expanded_fields.append(rt_data_field.col_prefix)
            elif rt_data_field.field_name.endswith('rt_vl'):
                columns = ['Baseboard', 'Psu', 'SafeMcu']
                columns += range(robot_info.num_joints)
                self.expanded_fields.extend(assemble_with_prefix(rt_data_field.col_prefix, columns))
            elif rt_data_field.field_name.endswith('rt_vm'):
                columns = ['Baseboard', 'Psu', 'SafeMcu']
                columns += range(robot_info.num_joints)
                self.expanded_fields.extend(assemble_with_prefix(rt_data_field.col_prefix, columns))
            elif rt_data_field.field_name.endswith('rt_current'):
                columns = ['Baseboard']
                self.expanded_fields.extend(assemble_with_prefix(rt_data_field.col_prefix, columns))
            elif rt_data_field.field_name.endswith('rt_temperature'):
                columns = ['Baseboard', 'Psu', 'SafeMcu']
                columns += range(robot_info.num_joints)
                self.expanded_fields.extend(assemble_with_prefix(rt_data_field.col_prefix, columns))
            elif rt_data_field.field_name.endswith('rt_i2t'):
                columns = range(robot_info.num_joints)
                self.expanded_fields.extend(assemble_with_prefix(rt_data_field.col_prefix, columns))
            elif rt_data_field.field_name.endswith('rt_external_tool_status'):
                self.expanded_fields.extend(
                    assemble_with_prefix(rt_data_field.col_prefix,
                                         ['SimModel', 'PhysicalModel', 'Present', 'Homed', 'Error']))
            elif rt_data_field.field_name.endswith('rt_valve_state'):
                self.expanded_fields.extend(assemble_with_prefix(rt_data_field.col_prefix, ['valve1', 'valve2']))
            elif rt_data_field.field_name.endswith('rt_gripper_state'):
                self.expanded_fields.extend(
                    assemble_with_prefix(rt_data_field.col_prefix, ['Holding', 'TargetReached', 'Closed', 'Opened']))
            elif rt_data_field.field_name.endswith('rt_gripper_force'):
                self.expanded_fields.extend(assemble_with_prefix(rt_data_field.col_prefix, ['%']))
            elif rt_data_field.field_name.endswith('rt_gripper_pos'):
                self.expanded_fields.extend(assemble_with_prefix(rt_data_field.col_prefix, ['mm']))
            elif rt_data_field.field_name.endswith('rt_io_module_status'):
                self.expanded_fields.extend(
                    assemble_with_prefix(rt_data_field.col_prefix, ['BankId', 'Present', 'SimMode', 'Error']))
            elif rt_data_field.field_name.endswith('rt_vacuum_state'):
                self.expanded_fields.extend(
                    assemble_with_prefix(rt_data_field.col_prefix, ['VacuumOn', 'PurgeOn', 'Holding']))
            elif rt_data_field.field_name.endswith('rt_vacuum_pressure'):
                self.expanded_fields.extend(assemble_with_prefix(rt_data_field.col_prefix, ['kPa']))
            elif rt_data_field.field_name.endswith('rt_io_module_outputs'):
                nb_outputs = len(robot_rt_data.rt_io_module_outputs.data)
                if nb_outputs != 0:
                    self.expanded_fields.extend(self._build_io_fields('IoModule', 'Output', nb_outputs))
            elif rt_data_field.field_name.endswith('rt_io_module_inputs'):
                nb_inputs = len(robot_rt_data.rt_io_module_inputs.data)
                if nb_inputs != 0:
                    self.expanded_fields.extend(self._build_io_fields('IoModule', 'Input', nb_inputs))
            elif rt_data_field.field_name.endswith('rt_sig_gen_status'):
                self.expanded_fields.extend(
                    assemble_with_prefix(rt_data_field.col_prefix, ['BankId', 'Present', 'SimMode', 'Error']))
            elif rt_data_field.field_name.endswith('rt_sig_gen_outputs'):
                nb_outputs = len(robot_rt_data.rt_sig_gen_outputs.data)
                if nb_outputs != 0:
                    self.expanded_fields.extend(self._build_io_fields('SigGen', 'Output', nb_outputs))
            elif rt_data_field.field_name.endswith('rt_sig_gen_inputs'):
                nb_inputs = len(robot_rt_data.rt_sig_gen_inputs.data)
                if nb_inputs != 0:
                    self.expanded_fields.extend(self._build_io_fields('SigGen', 'Input', nb_inputs))
            else:
                raise ValueError(f'Missing formatting for field: {rt_data_field.field_name}')

    def write_fields(self, timestamp, robot_rt_data: RobotRtData):
        """Write fields to file.

        Parameters
        ----------
        timestamp : numeric
            The timestamp of the current data.
        robot_rt_data : RobotRtData object
            This object contains the current robot state.

        """
        if self.done_logging:
            return

        # First write the timestamp
        self.data_dict[timestamp] = []
        for rt_data_field in self.fields:
            # For each field, write each value with appropriate spacing.
            ts_data = self.get_timestamp_data(robot_rt_data, rt_data_field.field_name)
            if ts_data is None:
                raise ValueError((f'Could not find TimestampedData for {rt_data_field.field_name} in RobotRtData '
                                  f'(bug in trajectory capture)'))
            self.data_dict[timestamp].extend([x for x in ts_data.data])

    def stop_logging_commands(self):
        """Stops saving sent commands to log"""
        self.logging_commands = False

    def end_log(self, ignore_checkpoints=True):
        """ Write all accumulated sent commands and close file.

        Return
        --------

        string
            Filename where logged info can be found
        """

        self.done_logging = True

        self.robot_trajectories.robot_df_hist.output_dfs.append(
            pd.DataFrame.from_dict(self.data_dict, orient='index', columns=self.expanded_fields))

        # Write all sent commands.
        while not self.command_queue.empty():
            command = self.command_queue.get()
            if ignore_checkpoints and command.startswith('SetCheckpoint'):
                continue
            self.robot_trajectories.robot_context.sent_commands.append(command)

        self.robot_trajectories.to_file(self.file_name, file_path=self.file_path)

        if self.file_path:
            return PurePath.joinpath(PurePath(self.file_path), self.file_name)
        else:
            return PurePath(self.file_name)
