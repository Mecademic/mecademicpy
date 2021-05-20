#!/usr/bin/env python3
import os
import queue
import time


class CSVFileLogger:
    """Class to handle logging robot state to file.

    Attributes
    ----------
    file : file handle
        File to be written to.
    fields : list of strings
        Fields to be logged.
    command_queue : queue
        Queue to store sent commands.
    element_width : int
        Each numerical element will have this width.

    """

    def __init__(self, robot_info, robot_state, fields=None, file_path=None, record_time=True):
        """Initialize class.

        Parameters
        ----------
        robot_info : RobotInfo
            Contains robot information.
        fields : list of strings
            List of fields to be logged.
        robot_state : RobotState
            Contains state of robot.
        file_path : string or None
            If not provided, file will be saved in working directory.
        record_time : bool
            If true, current time will also be recorded in the text file. (Time is also available in filename.)

        """
        current_date_time = time.strftime('%Y-%m-%d-%H-%M-%S')

        serial_number_or_blank = ('_serial_' + robot_info.serial) if robot_info.serial else ""

        # Add unique name to file path.
        file_name = (f"{robot_info.model}_R{robot_info.revision}_"
                     f"v{robot_info.fw_major_rev}_{robot_info.fw_minor_rev}_{robot_info.fw_patch_num}_"
                     f"log_{current_date_time}{serial_number_or_blank}.csv")

        if file_path:
            file_name = os.path.join(file_path, file_name)

        # If fields argument is None, log all compatible fields.
        if fields == None:
            fields = []

            if robot_info.rt_message_capable:
                for attr in vars(robot_state):
                    if attr.startswith('target') or attr.startswith('drive'):
                        fields.append(attr)
            else:
                # Only the following fields are available if platform is not rt monitoring capable.
                fields = ['target_joint_positions', 'target_end_effector_pose']

        # Set attributes.
        self.file = open(file_name, 'w', newline='')
        self.fields = fields
        self.command_queue = queue.Queue()
        self.element_width = 10
        self.timestamp_element_width = 15

        # Write robot information.
        self.file.write('ROBOT_INFORMATION\n')
        for attr in ['model', 'revision', 'fw_major_rev', 'fw_minor_rev', 'fw_patch_num']:
            self.file.write(f'{attr}, {getattr(robot_info, attr)}\n')
        if robot_info.serial != None:
            self.file.write(f'serial_number, {robot_info.serial}\n')
        if record_time:
            self.file.write(f'time_recorded, {current_date_time}\n')

        # Write headers for logged data.
        self.file.write('\nLOGGED_DATA\n')
        self.write_field_headers(robot_state)
        self.write_field_and_element_headers(robot_info)

    def write_field_headers(self, robot_state):
        """For a field with multiple elements, write field name in first column, empty spaces for rest.

        These headers are mostly for CSV and excel legibility.

        Parameters
        ----------
        robot_state : RobotState
            Current state of robot. Used only to get length of data fields.

        """
        self.file.write(f"{'timestamp':>{self.timestamp_element_width}},")
        for field in self.fields:
            # Get number of elements in each field.
            num_elements = len(getattr(robot_state, field).data)

            # Add appropriate number of commas to align columns.
            commas = ',' * (num_elements - 1)

            # Calculate width of field given number of elements, accounting for commas.
            width = (self.element_width + 1) * num_elements - 1
            self.file.write(f'{field + commas:{width}},')
        self.file.write('\n')

    def write_field_and_element_headers(self, robot_info):
        """Write the full field name and element name in each column.

        Parameters
        ----------
        robot_info : RobotInfo
            Information about the robot, such as model name and number of joints.

        """

        def assemble_with_prefix(field, names):
            return ','.join([field + '_' + str(x) for x in names]) + ','

        # Write full name for each field.
        self.file.write(f"{'timestamp':>{self.timestamp_element_width}},")
        for field in self.fields:
            if (field.endswith('joint_positions') or field.endswith('joint_velocity')
                    or field.endswith('joint_torque_ratio')):
                # Write field name followed by joint number. For example: "target_joint_positions_1".
                self.file.write(assemble_with_prefix(field, range(robot_info.num_joints)))
            elif field.endswith('end_effector_pose'):
                self.file.write(assemble_with_prefix(field, ['x', 'y', 'z', 'alpha', 'beta', 'gamma']))
            elif field.endswith('end_effector_velocity'):
                self.file.write(
                    assemble_with_prefix(field, ['x_dot', 'y_dot', 'z_dot', 'omega_x', 'omega_y', 'omega_z']))
            elif field.endswith('configurations'):
                self.file.write(assemble_with_prefix(field, ['shoulder', 'elbow', 'wrist']))
            elif field.endswith('last_joint_turn'):
                self.file.write(field + ',')
            else:
                raise ValueError(f'Missing formatting for field: {field}')
        self.file.write('\n')

    def write_fields(self, timestamp, robot_state):
        """Write fields to file.

        Parameters
        ----------
        timestamp : numeric
            The timestamp of the current data.
        robot_state : RobotState
            This object contains the current robot state.

        """
        if self.file.closed:
            return

        # First write the timestamp
        self.file.write(f'{timestamp:{self.timestamp_element_width}},')

        for field in self.fields:
            # For each field, write each value with appropriate spacing.
            self.file.write(','.join([f'{x:{self.element_width}}' for x in getattr(robot_state, field).data]))
            self.file.write(',')

        # End line with newline.
        self.file.write('\n')

    def end_log(self, ignore_checkpoints=True):
        """Write all accumulated sent commands and close file.

        """
        # Write all sent commands.
        self.file.write('\nSENT_COMMANDS\n')
        while not self.command_queue.empty():
            command = self.command_queue.get()

            if ignore_checkpoints and command.startswith('SetCheckpoint'):
                continue

            self.file.write(f'"{command}"\n')

        self.file.close()
