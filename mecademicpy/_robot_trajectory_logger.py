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

from .mx_robot_def import MxRobotStatusCode as mx_st
# pylint: disable=wildcard-import,unused-wildcard-import
from .robot_classes import *
from .robot_trajectory_files import RobotTrajectories

# 2nd values of this dict are taken from controller.cpp HandleSetRealTimeMonitoring() -> ParseStatusCodeString()
# dict and put in UpperCamelCase for convenience (all column names in logged dataframe will be in the format of these
# values)
robot_rt_data_to_real_time_monit = {
    'rt_target_joint_pos': (mx_st.MX_ST_RT_TARGET_JOINT_POS, 'TargetJointPos'),
    'rt_target_cart_pos': (mx_st.MX_ST_RT_TARGET_CART_POS, 'TargetCartPos'),
    'rt_target_joint_vel': (mx_st.MX_ST_RT_TARGET_JOINT_VEL, 'TargetJointVel'),
    'rt_target_joint_torq': (mx_st.MX_ST_RT_TARGET_JOINT_TORQ, 'TargetJointTorq'),
    'rt_target_cart_vel': (mx_st.MX_ST_RT_TARGET_CART_VEL, 'TargetCartVel'),
    'rt_target_conf': (mx_st.MX_ST_RT_TARGET_CONF, 'TargetConf'),
    'rt_target_conf_turn': (mx_st.MX_ST_RT_TARGET_CONF_TURN, 'TargetConfTurn'),
    'rt_joint_pos': (mx_st.MX_ST_RT_JOINT_POS, 'JointPos'),
    'rt_cart_pos': (mx_st.MX_ST_RT_CART_POS, 'CartPos'),
    'rt_joint_vel': (mx_st.MX_ST_RT_JOINT_VEL, 'JointVel'),
    'rt_joint_torq': (mx_st.MX_ST_RT_JOINT_TORQ, 'JointTorq'),
    'rt_cart_vel': (mx_st.MX_ST_RT_CART_VEL, 'CartVel'),
    'rt_conf': (mx_st.MX_ST_RT_CONF, 'Conf'),
    'rt_conf_turn': (mx_st.MX_ST_RT_CONF_TURN, 'ConfTurn'),
    'rt_abs_joint_pos': (mx_st.MX_ST_RT_JOINT_POS, 'AbsJointPos'),
    'rt_accelerometer': (mx_st.MX_ST_RT_ACCELEROMETER, 'Accel'),
    'rt_effective_time_scaling': (mx_st.MX_ST_RT_EFFECTIVE_TIME_SCALING, 'EffectiveTimeScaling'),
    'rt_vm': (mx_st.MX_ST_RT_VM, 'Vm'),
    'rt_current': (mx_st.MX_ST_RT_CURRENT, 'Current'),
    'rt_wrf': (mx_st.MX_ST_RT_WRF, 'Wrf'),
    'rt_trf': (mx_st.MX_ST_RT_TRF, 'Trf'),
    'rt_checkpoint': (mx_st.MX_ST_RT_CHECKPOINT, 'Checkpoint'),
    'rt_external_tool_status': (mx_st.MX_ST_RT_EXTTOOL_STATUS, 'ExtToolStatus'),
    'rt_valve_state': (mx_st.MX_ST_RT_VALVE_STATE, 'ValveState'),
    'rt_gripper_state': (mx_st.MX_ST_RT_GRIPPER_STATE, 'GripperState'),
    'rt_gripper_force': (mx_st.MX_ST_RT_GRIPPER_FORCE, 'GripperForce'),
    'rt_gripper_pos': (mx_st.MX_ST_RT_GRIPPER_POS, 'GripperPos'),
    'rt_io_module_status': (mx_st.MX_ST_RT_IO_STATUS, 'IoModuleStatus'),
    'rt_io_module_outputs': (mx_st.MX_ST_RT_OUTPUT_STATE, 'IoModuleOutputState'),
    'rt_io_module_inputs': (mx_st.MX_ST_RT_INPUT_STATE, 'IoModuleInputState'),
    'rt_sig_gen_status': (mx_st.MX_ST_RT_IO_STATUS, 'SigGenStatus'),
    'rt_sig_gen_outputs': (mx_st.MX_ST_RT_OUTPUT_STATE, 'SigGenOutputState'),
    'rt_sig_gen_inputs': (mx_st.MX_ST_RT_INPUT_STATE, 'SigGenInputState'),
    'rt_vacuum_state': (mx_st.MX_ST_RT_VACUUM_STATE, 'VacuumState'),
    'rt_vacuum_pressure': (mx_st.MX_ST_RT_VACUUM_PRESSURE, 'VacuumPressure'),
    '': (mx_st.MX_ST_RT_CYCLE_END, 'CycleEnd')  # Should not be used, handled by Robot class when it uses the logger
}


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
    element_width : int
        Each numerical element will have this width.
    timestamp_element_width: int
        Each timestamp will have this width
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
                 fields: list[str] = None,
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
        fields : list of strings
            List of fields to be logged.
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
        self.fields = dict()
        if fields is None:

            if robot_info.rt_message_capable:
                for attr in vars(robot_rt_data):
                    if attr.startswith('rt_'):
                        self.fields[attr] = robot_rt_data_to_real_time_monit[attr][1]
            else:
                # Only the following fields are available if platform is not rt monitoring capable.
                self.fields = {
                    'rt_target_joint_pos': robot_rt_data_to_real_time_monit['rt_target_joint_pos'][1],
                    'rt_target_cart_pos': robot_rt_data_to_real_time_monit['rt_target_cart_pos'][1]
                }
        else:
            for field in fields:
                for key, val in robot_rt_data_to_real_time_monit.items():
                    if (isinstance(field, str) and field.lower() == val[1].lower()) or field == val[0]:
                        self.fields[key] = val[1]
                        break

        # Set attributes.
        self.command_queue: queue.Queue = queue.Queue()
        self.element_width = 10
        self.timestamp_element_width = 15
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

    def get_timestamp_data(self, robot_rt_data: RobotRtData, field):
        """ Return timestamp data object associated with the specific field (or None).

        Parameters
        ----------
        robot_rt_data : RobotRtData object
            Current state of robot to get timestamp_data from
        field : String
            Name of the field to get timestamp_data for.

        """
        if field == 'rt_accelerometer':
            index = 5  # For now, only index 5 supported (joint 5's accelerometer)
            accel_dict = getattr(robot_rt_data, field)
            if index not in accel_dict:
                return None
            field_attr = accel_dict[index]
        else:
            field_attr = getattr(robot_rt_data, field)
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
        fields = []
        for idx in range(count):
            fields.append(f'{prefix}{io_type}_{idx + 1}')
        return fields

    def write_field_and_element_headers(self, robot_info: RobotInfo, robot_rt_data: RobotRtData):
        """Write the full field name and element name in each column.

        Parameters
        ----------
        robot_info : RobotInfo
            Information about the robot, such as model name and number of joints.
        robot_rt_data : RobotRtData object
            Contains state of robot.
        """

        def assemble_with_prefix(field, names):
            return [field + '_' + str(x) for x in names]

        # Write full name for each field.
        for key, value in self.fields.items():
            if (key.endswith('joint_pos') or key.endswith('joint_vel') or key.endswith('joint_torq')):
                # Write field name followed by joint number. For example: "TargetJointPos_1".
                self.expanded_fields.extend(assemble_with_prefix(value, range(robot_info.num_joints)))
            elif key.endswith('cart_pos') or key.endswith('wrf') or key.endswith('trf'):
                if robot_info.num_joints == 4:
                    self.expanded_fields.extend(assemble_with_prefix(value, ['X', 'Y', 'Z', 'Gamma']))
                else:
                    self.expanded_fields.extend(assemble_with_prefix(value, ['X', 'Y', 'Z', 'Alpha', 'Beta', 'Gamma']))
            elif key.endswith('cart_vel'):
                if robot_info.num_joints == 4:
                    self.expanded_fields.extend(assemble_with_prefix(value, ['X_Dot', 'Y_Dot', 'Z_Dot', 'Omega_Z']))
                else:
                    self.expanded_fields.extend(
                        assemble_with_prefix(value, ['X_Dot', 'Y_Dot', 'Z_Dot', 'Omega_X', 'Omega_Y', 'Omega_Z']))
            elif key.endswith('rt_accelerometer'):
                self.expanded_fields.extend(assemble_with_prefix(value, ['X', 'Y', 'Z']))
            elif key.endswith('conf_turn'):
                self.expanded_fields.append(value)
            elif key.endswith('conf'):
                self.expanded_fields.extend(assemble_with_prefix(value, ['Shoulder', 'Elbow', 'Wrist']))
            elif key.endswith('checkpoint'):
                self.expanded_fields.append(value)
            elif key.endswith('rt_effective_time_scaling'):
                self.expanded_fields.append(value)
            elif key.endswith('rt_vm'):
                columns = ['Baseboard', 'Psu', 'SafeMcu']
                columns += range(robot_info.num_joints)
                self.expanded_fields.extend(assemble_with_prefix(value, columns))
            elif key.endswith('rt_current'):
                columns = ['Baseboard']
                self.expanded_fields.extend(assemble_with_prefix(value, columns))
            elif key.endswith('rt_external_tool_status'):
                self.expanded_fields.extend(
                    assemble_with_prefix(value, ['SimModel', 'PhysicalModel', 'Present', 'Homed', 'Error']))
            elif key.endswith('rt_valve_state'):
                self.expanded_fields.extend(assemble_with_prefix(value, ['valve1', 'valve2']))
            elif key.endswith('rt_gripper_state'):
                self.expanded_fields.extend(
                    assemble_with_prefix(value, ['Holding', 'TargetReached', 'Closed', 'Opened']))
            elif key.endswith('rt_gripper_force'):
                self.expanded_fields.extend(assemble_with_prefix(value, ['%']))
            elif key.endswith('rt_gripper_pos'):
                self.expanded_fields.extend(assemble_with_prefix(value, ['mm']))
            elif key.endswith('rt_io_module_status'):
                self.expanded_fields.extend(assemble_with_prefix(value, ['BankId', 'Present', 'SimMode', 'Error']))
            elif key.endswith('rt_vacuum_state'):
                self.expanded_fields.extend(assemble_with_prefix(value, ['VacuumOn', 'PurgeOn', 'Holding']))
            elif key.endswith('rt_vacuum_pressure'):
                self.expanded_fields.extend(assemble_with_prefix(value, ['kPa']))
            elif key.endswith('rt_io_module_outputs'):
                nb_outputs = len(robot_rt_data.rt_io_module_outputs.data)
                if nb_outputs != 0:
                    self.expanded_fields.extend(self._build_io_fields('IoModule', 'Output', nb_outputs))
            elif key.endswith('rt_io_module_inputs'):
                nb_inputs = len(robot_rt_data.rt_io_module_inputs.data)
                if nb_inputs != 0:
                    self.expanded_fields.extend(self._build_io_fields('IoModule', 'Input', nb_inputs))
            elif key.endswith('rt_sig_gen_status'):
                self.expanded_fields.extend(assemble_with_prefix(value, ['BankId', 'Present', 'SimMode', 'Error']))
            elif key.endswith('rt_sig_gen_outputs'):
                nb_outputs = len(robot_rt_data.rt_sig_gen_outputs.data)
                if nb_outputs != 0:
                    self.expanded_fields.extend(self._build_io_fields('SigGen', 'Output', nb_outputs))
            elif key.endswith('rt_sig_gen_inputs'):
                nb_inputs = len(robot_rt_data.rt_sig_gen_inputs.data)
                if nb_inputs != 0:
                    self.expanded_fields.extend(self._build_io_fields('SigGen', 'Input', nb_inputs))
            else:
                raise ValueError(f'Missing formatting for field: {key}')

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
        formatted_tim = f'{timestamp:{self.timestamp_element_width}}'
        self.data_dict[formatted_tim] = []
        for field in self.fields:
            # For each field, write each value with appropriate spacing.
            ts_data = self.get_timestamp_data(robot_rt_data, field)
            if ts_data is None:
                continue
            self.data_dict[formatted_tim].extend([f'{x:{self.element_width}}' for x in ts_data.data])

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
