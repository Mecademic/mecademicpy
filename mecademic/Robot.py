#!/usr/bin/env python3
import logging
import ipaddress
import time
import socket
import threading
import queue
import functools
import re
import copy

from .mx_robot_def import *

CHECKPOINT_ID_MAX_PRIVATE = 8191  # Max allowable checkpoint id, inclusive

TERMINATE = '--terminate--'

GRIPPER_OPEN = True
GRIPPER_CLOSE = False


class MecademicException(Exception):
    """Base exception class for Mecademic-related exceptions.

    """
    pass


class InvalidStateError(MecademicException):
    """The internal state of the instance is invalid.

    """
    pass


class CommunicationError(MecademicException):
    """There is a communication issue with the robot.

    """
    pass


class DisconnectError(MecademicException):
    """A non-nominal disconnection has occurred.

    """
    pass


class InterruptException(MecademicException):
    """An event has encountered an error. Perhaps it will never be set.

    """
    pass


class InterruptableEvent:
    """Extend default event class to also be able to unblock and raise an exception.

    Attributes
    ----------
    id : int or None
        Id for event.
    _event : event object
        A standard event-type object.
    _lock : lock object
        Used to ensure atomic operations.
    _interrupted : boolean
        If true, event is in an error state.

    """
    def __init__(self, id=None):
        self.id = id
        self._event = threading.Event()
        self._lock = threading.Lock()
        self._interrupted = False

    def wait(self, timeout=None):
        """Block until event is set or should raise an exception.

        Attributes
        ----------
        timeout : float
            Maximum duration to wait in seconds.

        Return
        ------
        success : boolean
            False if event timed out, true otherwise.

        """
        success = self._event.wait(timeout=timeout)
        if self._interrupted:
            raise InterruptException('Event received exception, possibly because event will never be triggered.')
        return success

    def set(self):
        """Set the event and unblock all waits.

        """
        with self._lock:
            self._event.set()

    def abort(self):
        """Unblock any waits and raise an exception.

        """
        with self._lock:
            if not self._event.is_set():
                self._interrupted = True
                self._event.set()

    def clear(self):
        """Reset the event to its initial state.

        """
        with self._lock:
            self._interrupted = False
            self._event.clear()

    def is_set(self):
        """Checks if the event is set.

        Return
        ------
        boolean
            False if event is not set or instance should '_interrupted'. True otherwise.

        """
        with self._lock:
            if self._interrupted:
                return False
            else:
                return self._event.is_set()

    def clear_abort(self):
        """Clears the abort.

        """
        with self._lock:
            if self._interrupted:
                self._interrupted = False
                self._event.clear()


class TimestampedData:
    """ Class for storing timestamped data.

    Attributes
    ----------
    timestamp : number-like
        Timestamp associated with data.
    data : object
        Data to be stored.

    """
    def __init__(self, timestamp, data):
        self.timestamp = timestamp
        self.data = data

    def update_from_csv(self, input_string):
        """Update from comma-separated string, only if timestamp is newer.

        Parameters
        ----------
        input_string : string
            Comma-separated string. First value is timestamp, rest is data.

        """
        floats = string_to_floats(input_string)

        if (len(floats) - 1) != len(self.data):
            raise ValueError('Cannot update TimestampedData with incompatible data.')

        if floats[0] > self.timestamp:
            self.timestamp = floats[0]
            self.data = floats[1:]

    def update_from_data(self, timestamp, data):
        """Update with data if timestamp is newer.

        Parameters
        ----------
        timestamp : number-like
            Timestamp associated with data.
        data : object
            Data to be stored if timestamp is newer.

        """
        if timestamp > self.timestamp:
            self.timestamp = timestamp
            self.data = data

    @classmethod
    def zeros(cls, length):
        """ Construct empty TimestampedData object of specified length.

        Parameters
        ----------
        length : int
            Length of data to construct.

        Return
        ------
        TimestampedData object

        """
        return cls(0, [0.] * length)

    def __eq__(self, other):
        """ Return true if other object has identical timestamp and data.

        Parameters
        ----------
        other : object
            Object to compare against.

        Return
        ------
        bool
            True if objects have same timestamp and data.

        """
        return other.timestamp == self.timestamp and other.data == self.data

    def __ne__(self, other):
        """ Return true if other object has different timestamp or data.

        Parameters
        ----------
        other : object
            Object to compare against.

        Return
        ------
        bool
            True if objects have different timestamp or data.

        """
        return not self == other


class Message:
    """Class for storing the internal state of a generic Mecademic robot.

    Attributes
    ----------
    id : integer
        The id of the message, representing the type of message.
    data : string
        The raw payoad of the message.

    """
    def __init__(self, id, data):
        self.id = id
        self.data = data

    def __repr__(self):
        return "Message with id={}, data={}".format(self.id, self.data)


class RobotInfo:
    """Class for storing metadata about a robot.

    Attributes
    ----------
    model : string
        Model of robot.
    revision : int
        Robot revision.
    is_virtual : bool
        True if is a virtual robot.
    fw_major_rev : int
        Major firmware revision number.
    fw_minor_rev : int
        Minor firmware revision number.
    fw_patch_num : int
        Firmware patch number.
    num_joints : int
        Number of joints on the robot.

    """
    def __init__(self,
                 model=None,
                 revision=None,
                 is_virtual=None,
                 fw_major_rev=None,
                 fw_minor_rev=None,
                 fw_patch_num=None):
        self.model = model
        self.revision = revision
        self.is_virtual = is_virtual
        self.fw_major_rev = fw_major_rev
        self.fw_minor_rev = fw_minor_rev
        self.fw_patch_num = fw_patch_num
        self.rt_message_capable = False

        if self.model == 'Meca500':
            self.num_joints = 6
        elif self.mode == 'scara':
            self.num_joints = 4
        elif self.model == None:
            self.num_joints = 1
        else:
            raise ValueError('Invalid robot model: {}'.format(self.model))

    @classmethod
    def from_command_response_string(cls, input_string):
        """Generate robot state from standard robot response string.

        String format should be "Connected to {model} R{revision}{-virtual} v{fw_major_num}.{fw_minor_num}.{patch_num}"

        Parameters
        ----------
        input_string : string
            Input string to be parsed.

        """
        robot_info_regex = re.compile(r'Connected to (\b.*\b) R(\d)(-virtual)? v(\d+)\.(\d+)\.(\d+)')
        try:
            matches = robot_info_regex.match(input_string).groups()
            return cls(model=matches[0],
                       revision=int(matches[1]),
                       is_virtual=(matches[2] != None),
                       fw_major_rev=int(matches[3]),
                       fw_minor_rev=int(matches[4]),
                       fw_patch_num=int(matches[5]))
        except:
            raise ValueError('Could not parse robot info string {}'.format(input_string))


class RobotState:
    """Class for storing the internal state of a generic Mecademic robot.

    Note that the recency and availability of the states which are not 'status fields' depends on the monitoring
    interval and which monitoring events are enabled. Monitoring events can be configured using SetMonitoringInterval()
    and SetRealTimeMonitoring().

    Attributes
    ----------
    target_joint_positions : TimestampedData
        Controller desired joint positions in degrees [theta_1...6], includes timestamp.
    target_end_effector_pose : TimestampedData
        Controller desired end effector pose [x, y, z, alpha, beta, gamma], includes timestamp.

    target_joint_velocity : TimestampedData
        Controller desired joint velocity in degrees/second [theta_dot_1...6], includes timestamp.
    target_end_effector_velocity : TimestampedData
        Controller desired end effector velocity in mm/s and degrees/s, includes timestamp.
    target_joint_configurations : TimestampedData
        Controller joint configuration that corresponds to desired joint positions.
    target_last_joint_turn : TimestampedData
        Controller last joint turn number that corresponds to desired joint positions.

    drive_joint_positions : TimestampedData
        Drive-measured joint positions in degrees [theta_1...6], includes timestamp.
    drive_end_effector_pose : TimestampedData
        Drive-measured end effector pose [x, y, z, alpha, beta, gamma], includes timestamp.

    drive_joint_velocity : TimestampedData
        Drive-measured joint velocity in degrees/second [theta_dot_1...6], includes timestamp.
    drive_joint_torque_ratio : TimestampedData
        Drive-measured torque ratio as a percent of maximum [torque_1...6], includes timestamp.
    drive_end_effector_velocity : TimestampedData
        Drive-measured end effector velocity in mm/s and degrees/s, includes timestamp.

    drive_joint_configurations : TimestampedData
        Controller joint configuration that corresponds to drives-measured joint positions.
    drive_last_joint_turn : TimestampedData
        Controller last joint turn number that corresponds to drives-measured joint positions.

    accelerometer : TimestampedData
        Raw accelerometer measurements [accelerometer_id, x, y, z]. 16000 = 1g.

    activation_state : boolean
        True if the robot is activated.
    homing_state : boolean
        True if the robot is homed.
    simulation_mode : boolean
        True if the robot is in simulation-only mode.
    error_status : boolean
        True if the robot is in error.
    pause_motion_status : boolean
        True if motion is currently paused.
    end_of_block_status : boolean
        True if robot is not moving and motion queue is empty.
    configuration : array
        Current configuration of the robot.

    """
    def __init__(self, num_joints):
        self.target_joint_positions = TimestampedData.zeros(num_joints)  # microseconds timestamp, degrees
        self.target_end_effector_pose = TimestampedData.zeros(6)  # microseconds timestamp, mm and degrees

        self.target_joint_velocity = TimestampedData.zeros(num_joints)  # microseconds timestamp, degrees/second
        self.target_end_effector_velocity = TimestampedData.zeros(6)  # microseconds timestamp, mm/s and deg/s

        self.target_joint_configurations = TimestampedData.zeros(3)
        self.target_last_joint_turn = TimestampedData.zeros(1)

        self.drive_joint_positions = TimestampedData.zeros(num_joints)  # microseconds timestamp, degrees
        self.drive_end_effector_pose = TimestampedData.zeros(6)  # microseconds timestamp, mm and degrees

        self.drive_joint_velocity = TimestampedData.zeros(num_joints)  # microseconds timestamp, degrees/second
        self.drive_joint_torque_ratio = TimestampedData.zeros(num_joints)  # microseconds timestamp, percent of maximum
        self.drive_end_effector_velocity = TimestampedData.zeros(6)  # microseconds timestamp, mm/s and deg/s

        self.drive_joint_configurations = TimestampedData.zeros(3)
        self.drive_last_joint_turn = TimestampedData.zeros(1)

        # Contains dictionary of accelerometers stored in the robot indexed by joint number.
        # For example, Meca500 currently only reports the accelerometer in joint 5.
        self.accelerometer = dict()  # 16000 = 1g

        self.max_queue_size = 0

        # The following are status fields.
        self.activation_state = False
        self.homing_state = False
        self.simulation_mode = False
        self.error_status = False
        self.pause_motion_status = False
        self.end_of_block_status = False

        self.configuration = [0] * 3


class RobotEvents:
    """Class for storing possible status events for the generic Mecademic robot.

    Attributes
    ----------
    on_connected : event
        Set if robot is connected.
    on_disconnected : event
        Set if robot is disconnected.
    on_status_updated : event
        Set if robot status is updated.
    on_activated : event
        Set if robot is activated.
    on_deactivated : event
        Set if robot is deactivated.
    on_homed : event
        Set if robot is homed.
    on_error : event
        Set if robot is in error.
    on_error_reset : event
        Set if robot error has been reset.
    on_p_stop : event
        Set if robot receives pstop.
    on_pstop_reset : event
        Set if pstop is reset.
    on_motion_paused : event
        Set if robot motion is paused.
    on_motion_resumed : event
        Set if robot motion is not paused.
    on_motion_cleared : event
        Set if there are no pending ClearMotion commands.
    on_activate_sim : event
        Set if robot is in sim mode.
    on_deactivate_sim : event
        Set if robot is not in sim mode.
    on_conf_updated : event
        Set if robot configuration has been updated.
    on_joints_updated : event
        Set if joint angles has been updated.
    on_pose_updated : event
        Set if robot pose has been updated.
    on_brakes_activated : event
        Set if brakes are activated.
    on_brakes_deactivated : event
        Set if brakes are deactivated.
    on_offline_program_started : event
        Set if there has been a change in the offline program state.
    on_end_of_block : event
        Set if end of block has been reached.

    """
    def __init__(self):
        self.on_connected = InterruptableEvent()
        self.on_disconnected = InterruptableEvent()

        self.on_status_updated = InterruptableEvent()

        self.on_activated = InterruptableEvent()
        self.on_deactivated = InterruptableEvent()

        self.on_homed = InterruptableEvent()

        self.on_error = InterruptableEvent()
        self.on_error_reset = InterruptableEvent()
        self.on_p_stop = InterruptableEvent()
        self.on_p_stop_reset = InterruptableEvent()

        self.on_motion_paused = InterruptableEvent()
        self.on_motion_resumed = InterruptableEvent()
        self.on_motion_cleared = InterruptableEvent()

        self.on_activate_sim = InterruptableEvent()
        self.on_deactivate_sim = InterruptableEvent()

        self.on_conf_updated = InterruptableEvent()
        self.on_joints_updated = InterruptableEvent()
        self.on_pose_updated = InterruptableEvent()

        self.on_brakes_activated = InterruptableEvent()
        self.on_brakes_deactivated = InterruptableEvent()

        self.on_offline_program_started = InterruptableEvent()

        self.on_end_of_block = InterruptableEvent()

        self.on_disconnected.set()
        self.on_deactivated.set()
        self.on_error_reset.set()
        self.on_p_stop_reset.set()
        self.on_motion_resumed.set()
        self.on_deactivate_sim.set()

        self.on_status_updated.set()
        self.on_conf_updated.set()
        self.on_joints_updated.set()
        self.on_pose_updated.set()
        self.on_brakes_activated.set()

    def clear_all(self):
        """Clear all events.

        """
        for attr in self.__dict__:
            self.__dict__[attr].clear()

    def abort_all_except_on_connected(self):
        """Abort all events, except for on_connected.

        """
        for attr in self.__dict__:
            if attr != 'on_connected':
                self.__dict__[attr].abort()

    def clear_abort_all(self):
        """Clear aborts for all events.

        """
        for attr in self.__dict__:
            self.__dict__[attr].clear_abort()


class RobotCallbacks:
    """Class for storing possible status events for the generic Mecademic robot.

    Attributes
    ----------
        on_connected : function object
            Function to be called once connected.
        on_disconnected : function object
            Function to be called once disconnected.
        on_status_updated : function object
            Function to be called once status is updated.
        on_activated : function object
            Function to be called once activated.
        on_deactivated : function object
            Function to be called once deactivated.
        on_homed : function object
            Function to be called once homing is complete.
        on_error : function object
            Function to be called if robot enters an error state.
        on_error_reset : function object
            Function to be called once error is reset.
        on_p_stop : function object
            Function to be called if PStop is activated.
        on_p_stop_reset : function object
            Function to be called if PStop is reset.
        on_motion_paused : function object
            Function to be called once motion is paused.
        on_motion_cleared : function object
            Function to be called once motion is cleared.
        on_motion_resumed : function object
            Function to be called once motion is resumed.
        on_checkpoint_reached : function object
            Function to be called if a checkpoint is reached.
        on_activate_sim : function object
            Function to be called once sim mode is activated.
        on_deactivate_sim : function object
            Function to be called once sim mode is deactivated.
        on_command_message : function object
            Function to be called each time a command response is received.
        on_monitor_message : function object
            Function to be called each time a monitor response is received.
        on_offline_program_state : function object
            Function to be called each time an offline program starts or fails to start.
    """
    def __init__(self):
        self.on_connected = None
        self.on_disconnected = None

        self.on_status_updated = None

        self.on_activated = None
        self.on_deactivated = None

        self.on_homed = None

        self.on_error = None
        self.on_error_reset = None
        self.on_p_stop = None
        self.on_p_stop_reset = None

        self.on_motion_paused = None
        self.on_motion_cleared = None
        self.on_motion_resumed = None

        self.on_checkpoint_reached = None

        self.on_activate_sim = None
        self.on_deactivate_sim = None

        self.on_command_message = None
        self.on_monitor_message = None

        self.on_offline_program_state = None


class CallbackQueue():
    """Queue class for storing triggered callbacks. Only registered callbacks are added to the queue.

    Attributes
    ----------
    _queue : queue
        Queue to use to store callback names and associated data.
    _registered_callbacks : set
        Set of names of registered callbacks.

    """
    def __init__(self, robot_callbacks):
        self._queue = queue.Queue()
        self._registered_callbacks = set()

        for attr in robot_callbacks.__dict__:
            if robot_callbacks.__dict__[attr] != None:
                self._registered_callbacks.add(attr)

    def qsize(self):
        """Returns the queue size.

        """
        return self._queue.qsize()

    def put(self, callback_name, data=None):
        """Put the callback name and associated data into the queue if is registered.

        Parameters
        ----------
        callback_name : str
            Name of callback.
        data : any object type
            Associated data.

        """
        if callback_name in self._registered_callbacks or callback_name == TERMINATE:
            self._queue.put((callback_name, data))

    def get(self, block=False, timeout=None):
        """Get the next callback in the queue.

        Parameters
        ----------
        block : bool
            Block on next available callback if true.
        timeout : float
            Maximum time to wait on a callback.

        Returns
        -------
        tuple of callback name and data

        """
        return self._queue.get(block=block, timeout=timeout)


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
    def __init__(self, robot_info, fields, robot_state, file_path=None):
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

        """
        # Add unique name to file path.
        file_name = (f"{robot_info.model}_R{robot_info.revision}_"
                     f"v{robot_info.fw_major_rev}_{robot_info.fw_minor_rev}_{robot_info.fw_patch_num}_"
                     f"log_{time.strftime('%Y-%m-%d-%H-%M-%S')}.csv")

        if file_path:
            file_name = file_path + file_name

        # Set attributes
        self.file = open(file_name, 'w', newline='')
        self.fields = fields
        self.command_queue = queue.Queue()
        self.element_width = 10

        # Write robot information.
        for attr in ['model', 'revision', 'fw_major_rev', 'fw_minor_rev', 'fw_patch_num']:
            self.file.write(f'{attr}, {getattr(robot_info, attr)}\n')

        self.file.write('\nLOGGED_DATA\n')

        # Write fields to be logged.
        self.file.write(f"{'timestamp':>15},")
        for field in self.fields:
            # Get number of elements in each field.
            num_elements = len(getattr(robot_state, field).data)

            # Add appropriate number of commas to align columns.
            self.file.write(' ,' * (num_elements - 1))

            # Calculate width of field given number of elements, accounting for commas.
            width = (self.element_width - 1) * num_elements + 1
            self.file.write(f'{field:>{width}},')
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
        self.file.write(f'{timestamp:15},')

        for field in self.fields:
            # For each field, write each value with appropriate spacing.
            self.file.write(','.join([f'{x:{self.element_width}}' for x in getattr(robot_state, field).data]))
            self.file.write(',')

        # End line with newline.
        self.file.write('\n')

    def end_log(self, trim_last_command=False):
        """Write all accumulated sent commands and close file.

        """
        # Write all sent commands.
        self.file.write('\nSENT_COMMANDS\n')
        while not self.command_queue.empty():
            self.file.write(f'{self.command_queue.get()}\n')

        self.file.close()


def disconnect_on_exception(func):
    """Decorator to call disconnect if an exception is raised. Needs to be declared outside of class.

    Attributes
    ----------
    func : function object
        Function to wrap.

    """
    @functools.wraps(func)
    def wrap(self, *args, **kwargs):
        try:
            return func(self, *args, **kwargs)
        except BaseException as e:
            if self._disconnect_on_exception:
                self.Disconnect()
                raise DisconnectError('Automatically disconnected as a result of exception, '
                                      'set \'disconnect_on_exception\' to False to disable.') from e
            else:
                raise e

    return wrap


def string_to_floats(input_string):
    """Convert comma-separated floats in string form to list of floats.

    Parameters
    ----------
    input_string : string
        Comma-separated floats values encoded as a string.

    Returns
    -------
    list of floats
        Returns converted list of floats.

    """
    return [float(x) for x in input_string.split(',')]


class Robot:
    """Class for controlling a generic Mecademic robot.

    Attributes
    ----------
    _address : string
        The IP address associated to the Mecademic Robot.
    _command_socket : socket object
        Socket connecting to the command port of the physical Mecademic robot.
    _monitor_socket : socket object
        Socket connecting to the monitor port of the physical Mecademic robot.

    _command_rx_thread : thread handle
        Thread used to receive messages from the command port.
    _command_rx_queue : queue
        Queue used to temporarily store messages from the command port.
    _command_tx_thread : thread handle
        Thread used to transmit messages to the command port.
    _command_tx_queue : queue
        Queue used to temporarily store commands to be sent to the command port.
    _monitor_rx_thread : thread handle
        Thread used to receive messages from the monitor port.
    _monitor_rx_queue : queue
        Queue used to temporarily store messages from the monitor port.

    _command_response_handler_thread : thread handle
        Thread used to read messages from the command response queue.
    _monitor_handler_thread : thread handle
        Thread used to read messages from the monitor queue.

    _main_lock : recursive lock object
        Used to protect internal state of the robot object.

    _robot_state : RobotState object
        Stores most current robot state.
    _robot_events : RobotEvents object
        Stores events related to the robot state.

    _robot_callbacks : RobotCallbacks instance
        Stores user-defined callback functions.
    _callback_queue : queue
        Queue storing triggered callbacks.
    _callback_thread : thread handle
        Callbacks will run in this thread if so configured.

    _user_checkpoints : dictionary
        Stores checkpoints set or expected by user.
    _internal_checkpoints : dictionary
        Stores checkpoints set internally by the Robot class.
    _internal_checkpoint_counter : int
        Stores the next available checkpoint id for internal checkpoints.

    _enable_synchronous_mode : boolean
        If enabled, commands block until action is completed.

    _clear_motion_requests : int
        Number of pending ClearMotion requests.

    logger : logger object
        Logger used throughout class.

    default_timeout : float
        Default timeout to use for blocking operations.

    """
    def __init__(self):
        """Constructor for an instance of the Controller class.

        """
        self._is_initialized = False

        self._address = None

        self._command_socket = None
        self._monitor_socket = None

        self._command_rx_thread = None
        self._command_tx_thread = None
        self._monitor_rx_thread = None

        self._command_response_handler_thread = None
        self._monitor_handler_thread = None

        self._main_lock = threading.RLock()

        self._robot_callbacks = RobotCallbacks()
        self._callback_queue = CallbackQueue(self._robot_callbacks)
        self._callback_thread = None

        self._robot_info = None
        self._robot_state = None
        self._robot_events = RobotEvents()

        self._file_logger = None

        self._reset_disconnect_attributes()

        self._enable_synchronous_mode = None
        self._disconnect_on_exception = None

        self._offline_mode = None
        self._monitor_mode = None

        self.logger = logging.getLogger(__name__)
        self.default_timeout = 10

        self._is_initialized = True

    def __del__(self):
        # Only attempt to disconnect if the object was initialized.
        if self._is_initialized:
            self.Disconnect()
            self.UnregisterCallbacks()

    def _reset_disconnect_attributes(self):
        self._command_rx_queue = queue.Queue()
        self._command_tx_queue = queue.Queue()
        self._monitor_rx_queue = queue.Queue()
        self._custom_response_queue = None

        self._user_checkpoints = dict()
        self._internal_checkpoints = dict()
        self._internal_checkpoint_counter = MX_CHECKPOINT_ID_MAX + 1

        self._clear_motion_requests = 0

    #####################################################################################
    # Static methods.
    #####################################################################################

    @staticmethod
    def _deactivate_on_exception(func, command_socket, *args, **kwargs):
        """Wrap input function to send deactivate signal to command_socket on exception.

        Parameters
        ----------
        func : function handle
            Function to execute.

        command_socket : socket
            Socket to send the deactivate command to.

        """
        try:
            return func(*args, **kwargs)
        except BaseException as e:
            if command_socket:
                command_socket.sendall(b'DeactivateRobot\0')
            raise e

    @staticmethod
    def _handle_socket_rx(robot_socket, rx_queue):
        """Handle received data on the socket.

        Parameters
        ----------
        robot_socket : socket
            Socket to use for receiving data.

        rx_queue : queue
            Thread-safe queue to push complete messages onto.

        """
        remainder = ''
        while True:
            # Wait for a message from the robot.
            try:
                robot_socket.setblocking(True)
                raw_responses = robot_socket.recv(1024)
            except (ConnectionAbortedError, BrokenPipeError):
                return

            # Socket has been closed.
            if raw_responses == b'':
                return

            responses = raw_responses.decode('ascii').split('\0')

            # Add the remainder from the previous message if necessary.
            if remainder != '':
                responses[0] = remainder + responses[0]

            # Set the remainder as the last response (which is '' if complete).
            remainder = responses[-1]

            # Put all responses into the queue.
            for response in responses[:-1]:
                id_start = response.find('[') + 1
                id_end = response.find(']', id_start)
                id = int(response[id_start:id_end])

                # Find next square brackets (contains data).
                data_start = response.find('[', id_end) + 1
                data_end = response.find(']', data_start)

                data = ''
                if data_start != -1 and data_end != -1:
                    data = response[data_start:data_end]
                rx_queue.put(Message(id, data))

    @staticmethod
    def _handle_socket_tx(robot_socket, tx_queue):
        """Handle sending data on the socket.

        Parameters
        ----------
        robot_socket : socket
            Socket to use for sending data.

        tx_queue : queue
            Thread-safe queue to get messages from.

        """
        while True:
            # Wait for a command to be available from the queue.
            command = tx_queue.get(block=True)

            # Terminate thread if requested, otherwise send the command.
            if command == TERMINATE:
                return
            else:
                robot_socket.sendall((command + '\0').encode('ascii'))

    @staticmethod
    def _connect_socket(logger, address, port):
        """Connects to an arbitrary socket.

        Parameters
        ----------
        logger : logger instance
            Logger to use.
        address : string
            Address to use.
        port : int
            Port number to use.

        Returns
        -------
        new_socket : socket object
            Successfully-connected socket object.

        """
        logger.debug('Attempting to connect to %s:%s', address, port)

        # Create socket and attempt connection.
        new_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        new_socket.settimeout(0.1)  # 100ms
        try:
            new_socket.connect((address, port))
        except:
            logger.error('Unable to connect to %s:%s.', address, port)
            return None

        logger.debug('Connected to %s:%s.', address, port)
        return new_socket

    @staticmethod
    def _handle_callbacks(logger, callback_queue, callbacks, timeout=None):
        """Runs callbacks found in callback_queue.

        Parameters
        ----------
        logger : logger instance
            Logger to use.
        callback_queue : queue
            Stores triggered callbacks.
        callbacks : RobotCallbacks instance
            Stores user-defined callback functions.
        timeout : float or None
            If none, block forever on empty queue, if 0, don't block, else block with timeout.
        """
        block_on_empty = (timeout != 0)

        while True:
            # If we are not blocking on empty, return if empty.
            if not block_on_empty and callback_queue.qsize() == 0:
                return

            callback_name, data = callback_queue.get(block=block_on_empty, timeout=timeout)

            if callback_name == TERMINATE:
                return

            callback_function = callbacks.__dict__[callback_name]
            if callback_function != None:
                if data != None:
                    callback_function(data)
                else:
                    callback_function()

    #####################################################################################
    # Private methods.
    #####################################################################################

    def _check_monitor_threads(self):
        """Check that the threads which handle robot monitor messages are alive.

        Attempt to disconnect from the robot if not.

        """

        if not (self._monitor_handler_thread and self._monitor_handler_thread.is_alive()):
            self.Disconnect()
            raise InvalidStateError('Monitor response handler thread has unexpectedly terminated.')

        if self._offline_mode:  # Do not check rx threads in offline mode.
            return

        if not (self._monitor_rx_thread and self._monitor_rx_thread.is_alive()):
            self.Disconnect()
            raise InvalidStateError('Monitor rx thread has unexpectedly terminated.')

    def _check_command_threads(self):
        """Check that the threads which handle robot command messages are alive.

        Attempt to disconnect from the robot if not.

        """

        if not (self._command_response_handler_thread and self._command_response_handler_thread.is_alive()):
            self.Disconnect()
            raise InvalidStateError('No command response handler thread, are you in monitor mode?')

        if self._offline_mode:  # Do not check rx threads in offline mode.
            return

        if not (self._command_rx_thread and self._command_rx_thread.is_alive()):
            self.Disconnect()
            raise InvalidStateError('No command rx thread, are you in monitor mode?')

        # If tx thread is down, attempt to directly send deactivate command to the robot.
        if not (self._command_tx_thread and self._command_tx_thread.is_alive()):
            self._command_socket.sendall(b'DeactivateRobot\0')
            self.Disconnect()
            raise InvalidStateError('No command tx thread, are you in monitor mode?')

    def _check_internal_states(self):
        """Check that the threads which handle robot messages are alive.

        Attempt to disconnect from the robot if not.

        """
        if self._monitor_mode:
            raise InvalidStateError('Cannot send command while in monitoring mode.')
        else:
            self._check_command_threads()

        self._check_monitor_threads()

    def _send_command(self, command, arg_list=None):
        """Assembles and sends the command string to the Mecademic robot.

        Parameters
        ----------
        command : string
            Command name to send to the Mecademic robot.
        arg_list : list
            List of arguments the command requires.

        """

        # Assemble arguments into a string and concatenate to end of command.
        if arg_list:
            command = command + '(' + ','.join([str(x) for x in arg_list]) + ')'

        # Put command into tx queue.
        self._command_tx_queue.put(command)

        # If logging is enabled, send command to logger.
        if self._file_logger:
            self._file_logger.command_queue.put(command)

    def _launch_thread(self, *, target, args):
        """Establish the threads responsible for reading/sending messages using the sockets.

        Parameters
        ----------
        func : function handle
            Function to run using new thread.
        args : argument list
            Arguments to be passed to func.

        Return
        ------
        thread handle
            Handle for newly-launched thread.

        """
        # We use the _deactivate_on_exception function which wraps func around try...except and disconnects on error.
        # The first argument is the actual function to be executed, the second is the command socket.
        thread = threading.Thread(target=self._deactivate_on_exception, args=(
            target,
            self._command_socket,
            *args,
        ))
        thread.start()
        return thread

    def _initialize_command_socket(self):
        """Establish the command socket and the associated thread.

        """
        if self._offline_mode:
            return

        if self._command_socket is not None:
            raise InvalidStateError('Cannot connect since existing command socket exists.')

        try:
            self._command_socket = self._connect_socket(self.logger, self._address, MX_ROBOT_TCP_PORT_CONTROL)

            if self._command_socket is None:
                raise CommunicationError('Command socket could not be created.')

            # Create rx thread for command socket communication.
            self._command_rx_thread = self._launch_thread(target=self._handle_socket_rx,
                                                          args=(self._command_socket, self._command_rx_queue))

            # Create tx thread for command socket communication.
            self._command_tx_thread = self._launch_thread(target=self._handle_socket_tx,
                                                          args=(self._command_socket, self._command_tx_queue))

        except:
            # Clean up threads and connections on error.
            self.Disconnect()
            raise

    def _initialize_monitoring_socket(self):
        """Establish the monitoring socket and the associated thread.

        """
        if self._offline_mode:
            return

        if self._monitor_socket is not None:
            raise InvalidStateError('Cannot connect since existing monitor socket exists.')

        try:
            self._monitor_socket = self._connect_socket(self.logger, self._address, MX_ROBOT_TCP_PORT_FEED)

            if self._monitor_socket is None:
                raise CommunicationError('Monitor socket could not be created.')

            # Create rx thread for monitor socket communication.
            self._monitor_rx_thread = self._launch_thread(target=self._handle_socket_rx,
                                                          args=(self._monitor_socket, self._monitor_rx_queue))

        except:
            # Clean up threads and connections on error.
            self.Disconnect()
            raise

    def _receive_welcome_message(self, message_queue):
        """Receive and parse a welcome message in order to set _robot_info and _robot_state.

        Parameters
        ----------
        message_queue : queue
            The welcome message will be fetched from this queue.

        """

        try:
            response = message_queue.get(block=True, timeout=10)  # 10s timeout.
        except queue.Empty:
            self.logger.error('No response received within timeout interval.')
            self.Disconnect()
            raise CommunicationError('No response received within timeout interval.')
        except BaseException:
            self.Disconnect()
            raise

        # Check that response is appropriate.
        if response.id != MX_ST_CONNECTED:
            self.logger.error('Connection error: {}'.format(response))
            self.Disconnect()
            raise CommunicationError('Connection error: {}'.format(response))

        # Attempt to parse robot return data.
        self._robot_info = RobotInfo.from_command_response_string(response.data)

        self._robot_state = RobotState(self._robot_info.num_joints)

    def _initialize_command_connection(self):
        """Attempt to connect to the command port of the Mecademic Robot.

        """
        self._receive_welcome_message(self._command_rx_queue)

        self._command_response_handler_thread = self._launch_thread(target=self._command_response_handler, args=())

    def _initialize_monitoring_connection(self):
        """Attempt to connect to the monitor port of the Mecademic Robot.

        Returns
        -------
        status : boolean
            Returns the status of the connection, true for success, false for failure.

        """

        if self._monitor_mode:
            self._receive_welcome_message(self._monitor_rx_queue)

        self._monitor_handler_thread = self._launch_thread(target=self._monitor_handler, args=())

        return

    def _shut_down_queue_threads(self):
        """Attempt to gracefully shut down threads which read from queues.

        """
        # Join threads which wait on a queue by sending terminate to the queue.
        # Don't acquire _main_lock since these threads require _main_lock to finish processing.
        if self._command_tx_thread is not None:
            try:
                self._command_tx_queue.put(TERMINATE)
            except Exception as e:
                self.logger.error('Error shutting down tx thread. ' + str(e))
            self._command_tx_thread.join(timeout=self.default_timeout)
            self._command_tx_thread = None

        if self._command_response_handler_thread is not None:
            try:
                self._command_rx_queue.put(TERMINATE)
            except Exception as e:
                self.logger.error('Error shutting down command response handler thread. ' + str(e))
            self._command_response_handler_thread.join(timeout=self.default_timeout)
            self._command_response_handler_thread = None

        if self._monitor_handler_thread is not None:
            try:
                self._monitor_rx_queue.put(TERMINATE)
            except Exception as e:
                self.logger.error('Error shutting down monitor handler thread. ' + str(e))
            self._monitor_handler_thread.join(timeout=self.default_timeout)
            self._monitor_handler_thread = None

    def _shut_down_socket_threads(self):
        """Attempt to gracefully shut down threads which read from sockets.

        """
        with self._main_lock:
            # Shutdown socket to terminate the rx threads.
            if self._command_socket is not None:
                try:
                    self._command_socket.shutdown(socket.SHUT_RDWR)
                except Exception as e:
                    self.logger.error('Error shutting down command socket. ' + str(e))

            if self._monitor_socket is not None:
                try:
                    self._monitor_socket.shutdown(socket.SHUT_RDWR)
                except Exception as e:
                    self.logger.error('Error shutting down monitor socket. ' + str(e))

            # Join threads which wait on a socket.
            if self._command_rx_thread is not None:
                self._command_rx_thread.join(timeout=self.default_timeout)
                self._command_rx_thread = None

            if self._monitor_rx_thread is not None:
                self._monitor_rx_thread.join(timeout=self.default_timeout)
                self._monitor_rx_thread = None

    def _set_checkpoint_internal(self):
        """Set a checkpoint for internal use using the next available internal id.

        Return
        ------
        Checkpoint object
            Object to use to wait for the checkpoint.

        """
        with self._main_lock:
            checkpoint_id = self._internal_checkpoint_counter

            # Increment internal checkpoint counter.
            self._internal_checkpoint_counter += 1
            if self._internal_checkpoint_counter > CHECKPOINT_ID_MAX_PRIVATE:
                self._internal_checkpoint_counter.value = MX_CHECKPOINT_ID_MAX + 1

            return self._set_checkpoint_impl(checkpoint_id)

    def _set_checkpoint_impl(self, n, send_to_robot=True):
        """Create a checkpoint object which can be used to wait for the checkpoint id to be received from the robot.

        Checkpoints are implemented as a dictionary of lists, to support repeated checkpoints (which are discouraged),
        and also to support expecting external checkpoints. Particularly so that ExpectExternalCheckpoints could be
        called in any arbitrary order.

        Returning an event object for the user to wait on also prevents activated checkpoints from being 'missed' by the
        API, and prevents issues around waiting for checkpoints which may never arrive.

        Parameters
        ----------
        n : int
            Id of checkpoint.
        send_to_robot : bool
            If true, send the SetCheckpoint command to the robot.

        Return
        ------
        Checkpoint object
            Object to use to wait for the checkpoint.

        """
        with self._main_lock:
            if not isinstance(n, int):
                raise TypeError('Please provide an integer checkpoint id.')

            # Find the correct dictionary to store checkpoint.
            if MX_CHECKPOINT_ID_MIN <= n <= MX_CHECKPOINT_ID_MAX:
                checkpoints_dict = self._user_checkpoints
            elif MX_CHECKPOINT_ID_MAX < n <= CHECKPOINT_ID_MAX_PRIVATE:
                checkpoints_dict = self._internal_checkpoints
            else:
                raise ValueError

            self.logger.debug('Setting checkpoint %s', n)

            if n not in checkpoints_dict:
                checkpoints_dict[n] = list()
            event = InterruptableEvent(n)
            checkpoints_dict[n].append(event)

            if send_to_robot:
                self._send_command('SetCheckpoint', [n])

            return event

    def _invalidate_checkpoints(self):
        '''Unblock all waiting checkpoints and have them throw InterruptException.

        '''

        for checkpoints_dict in [self._internal_checkpoints, self._user_checkpoints]:
            for key, checkpoints_list in checkpoints_dict.items():
                for event in checkpoints_list:
                    event.abort()
            checkpoints_dict.clear()

        self._internal_checkpoint_counter = MX_CHECKPOINT_ID_MAX + 1

    def _send_motion_command(self, command, arg_list=None):
        """Send generic motion command with support for synchronous mode and locking.

        Parameters
        ----------
        command : string
            The command to send.
        args : list
            List of arguments to be sent.

        """
        with self._main_lock:
            self._check_internal_states()
            self._send_command(command, arg_list)
            if self._enable_synchronous_mode:
                checkpoint = self._set_checkpoint_internal()

        if self._enable_synchronous_mode:
            checkpoint.wait()

    def _monitor_handler(self):
        """Handle messages from the monitoring port of the robot.

        """
        # Variables to hold joint positions and poses while waiting for timestamp.
        joint_positions = None
        end_effector_pose = None

        while True:
            # Wait for a message in the queue.
            response = self._monitor_rx_queue.get(block=True)

            # Terminate thread if requested.
            if response == TERMINATE:
                return

            self._callback_queue.put('on_monitor_message', response)

            queue_size = self._monitor_rx_queue.qsize()
            if queue_size > self._robot_state.max_queue_size:
                self._robot_state.max_queue_size = queue_size

            with self._main_lock:

                # Temporarily save joints and pose if rt messages will be availble to add timestamps.
                if response.id == MX_ST_GET_JOINTS and self._robot_info.rt_message_capable:
                    joint_positions = string_to_floats(response.data)
                if response.id == MX_ST_GET_POSE and self._robot_info.rt_message_capable:
                    end_effector_pose = string_to_floats(response.data)

                if response.id == MX_ST_RT_CYCLE_END:
                    if not self._robot_info.rt_message_capable:
                        self._robot_info.rt_message_capable = True
                    timestamp = float(response.data)

                    # Update the legacy joint and pose messages with timestamps.
                    if joint_positions:
                        self._robot_state.target_joint_positions.update_from_data(timestamp, joint_positions)
                        joint_positions = None
                    if end_effector_pose:
                        self._robot_state.target_end_effector_pose.update_from_data(timestamp, end_effector_pose)
                        end_effector_pose = None

                    # If logging is active, log the current state.
                    if self._file_logger != None:
                        self._file_logger.write_fields(timestamp, self._robot_state)

                else:
                    self._handle_common_messages(response, is_command_response=False)

    def _command_response_handler(self):
        """Handle received messages on the command socket.

        """
        while True:
            # Wait for a response to be available from the queue.
            response = self._command_rx_queue.get(block=True)

            # Terminate thread if requested.
            if response == TERMINATE:
                return

            self._callback_queue.put('on_command_message', response)

            if self._custom_response_queue:
                self._custom_response_queue.put(response)

            with self._main_lock:

                if response.id == MX_ST_CHECKPOINT_REACHED:
                    self._handle_checkpoint_response(response)

                elif response.id == MX_ST_CLEAR_MOTION:
                    if self._clear_motion_requests <= 1:
                        self._clear_motion_requests = 0
                        self._robot_events.on_motion_cleared.set()
                        self._callback_queue.put('on_motion_cleared')
                    else:
                        self._clear_motion_requests -= 1

                elif response.id == MX_ST_PSTOP:
                    if bool(int(response.data)):
                        self._robot_events.on_p_stop_reset.clear()
                        self._robot_events.on_p_stop.set()
                        self._callback_queue.put('on_p_stop')
                    else:
                        self._robot_events.on_p_stop.clear()
                        self._robot_events.on_p_stop_reset.set()
                        self._callback_queue.put('on_p_stop_reset')

                elif response.id == MX_ST_GET_CONF:
                    self._robot_state.configuration = string_to_floats(response.data)
                    self._robot_events.on_conf_updated.set()

                elif response.id == MX_ST_BRAKES_ON:
                    self._robot_events.on_brakes_deactivated.clear()
                    self._robot_events.on_brakes_activated.set()

                elif response.id == MX_ST_BRAKES_OFF:
                    self._robot_events.on_brakes_activated.clear()
                    self._robot_events.on_brakes_deactivated.set()

                elif response.id == MX_ST_OFFLINE_START:
                    self._robot_events.on_offline_program_started.set()
                    self._callback_queue.put('on_offline_program_state')

                elif response.id == MX_ST_NO_OFFLINE_SAVED:
                    self._robot_events.on_offline_program_started.abort()

                else:
                    self._handle_common_messages(response, is_command_response=True)

    def _handle_common_messages(self, response, is_command_response=False):
        """Handle response messages which are received on the command and monitor port, and are processed the same way.

        Parameters
        ----------
        response : Message object
            Robot status response to parse and handle.

        """
        if response.id == MX_ST_GET_STATUS_ROBOT:
            self._handle_robot_status_response(response)

        # Only update using legacy messages if robot is not capable of rt messages.
        elif response.id == MX_ST_GET_JOINTS and not self._robot_info.rt_message_capable:
            self._robot_state.target_joint_positions = TimestampedData(0, string_to_floats(response.data))
            if is_command_response:
                self._robot_events.on_joints_updated.set()

        elif response.id == MX_ST_GET_POSE and not self._robot_info.rt_message_capable:
            self._robot_state.target_end_effector_pose = TimestampedData(0, string_to_floats(response.data))
            if is_command_response:
                self._robot_events.on_pose_updated.set()

        elif response.id == MX_ST_RT_NC_JOINT_POS:
            self._robot_state.target_joint_positions.update_from_csv(response.data)
            if is_command_response:
                self._robot_events.on_joints_updated.set()

        elif response.id == MX_ST_RT_NC_CART_POS:
            self._robot_state.target_end_effector_pose.update_from_csv(response.data)
            if is_command_response:
                self._robot_events.on_pose_updated.set()

        elif response.id == MX_ST_RT_NC_JOINT_POS:
            self._robot_state.target_joint_positions.update_from_csv(response.data)
        elif response.id == MX_ST_RT_NC_CART_POS:
            self._robot_state.target_end_effector_pose.update_from_csv(response.data)
        elif response.id == MX_ST_RT_NC_JOINT_VEL:
            self._robot_state.target_joint_velocity.update_from_csv(response.data)
        elif response.id == MX_ST_RT_NC_CART_VEL:
            self._robot_state.target_end_effector_velocity.update_from_csv(response.data)

        elif response.id == MX_ST_RT_NC_CONF:
            self._robot_state.target_joint_configurations.update_from_csv(response.data)
        elif response.id == MX_ST_RT_NC_CONF_TURN:
            self._robot_state.target_last_joint_turn.update_from_csv(response.data)

        elif response.id == MX_ST_RT_DRIVE_JOINT_POS:
            self._robot_state.drive_joint_positions.update_from_csv(response.data)
        elif response.id == MX_ST_RT_DRIVE_CART_POS:
            self._robot_state.drive_end_effector_pose.update_from_csv(response.data)
        elif response.id == MX_ST_RT_DRIVE_JOINT_VEL:
            self._robot_state.drive_joint_velocity.update_from_csv(response.data)
        elif response.id == MX_ST_RT_DRIVE_JOINT_TORQ:
            self._robot_state.drive_joint_torque_ratio.update_from_csv(response.data)
        elif response.id == MX_ST_RT_DRIVE_CART_VEL:
            self._robot_state.drive_end_effector_velocity.update_from_csv(response.data)

        elif response.id == MX_ST_RT_DRIVE_CONF:
            self._robot_state.drive_joint_configurations.update_from_csv(response.data)
        elif response.id == MX_ST_RT_DRIVE_CONF_TURN:
            self._robot_state.drive_last_joint_turn.update_from_csv(response.data)

        elif response.id == MX_ST_RT_ACCELEROMETER:
            # The data is stored as [timestamp, index, {measurements...}]
            timestamp, index, *measurements = string_to_floats(response.data)
            # Record accelerometer measurement only if newer.
            if (index not in self._robot_state.accelerometer
                    or timestamp > self._robot_state.accelerometer[index].timestamp):
                self._robot_state.accelerometer[index] = TimestampedData(timestamp, measurements)

    def _handle_robot_status_response(self, response):
        """Parse robot status response and update status fields and events.

        Parameters
        ----------
        response : Message object
            Robot status response to parse and handle.

        """
        assert response.id == MX_ST_GET_STATUS_ROBOT
        status_flags = [bool(int(x)) for x in response.data.split(',')]

        if self._robot_state.activation_state != status_flags[0]:
            if status_flags[0]:
                self._robot_events.on_deactivated.clear()
                self._robot_events.on_activated.set()
                self._robot_events.on_brakes_activated.clear()
                self._robot_events.on_brakes_deactivated.set()
                self._callback_queue.put('on_activated')
            else:
                self._robot_events.on_activated.clear()
                self._robot_events.on_deactivated.set()
                self._robot_events.on_brakes_deactivated.clear()
                self._robot_events.on_brakes_activated.set()
                self._callback_queue.put('on_deactivated')
            self._robot_state.activation_state = status_flags[0]

        if self._robot_state.homing_state != status_flags[1]:
            if status_flags[1]:
                self._robot_events.on_homed.set()
                self._callback_queue.put('on_homed')
            else:
                self._robot_events.on_homed.clear()
            self._robot_state.homing_state = status_flags[1]

        if self._robot_state.simulation_mode != status_flags[2]:
            if status_flags[2]:
                self._robot_events.on_deactivate_sim.clear()
                self._robot_events.on_activate_sim.set()
                self._callback_queue.put('on_activate_sim')
            else:
                self._robot_events.on_activate_sim.clear()
                self._robot_events.on_deactivate_sim.set()
                self._callback_queue.put('on_deactivate_sim')
            self._robot_state.simulation_mode = status_flags[2]

        if self._robot_state.error_status != status_flags[3]:
            if status_flags[3]:
                self._invalidate_checkpoints()
                self._robot_events.on_error.set()
                self._robot_events.abort_all_except_on_connected()
                self._robot_events.on_error_reset.clear()
                self._callback_queue.put('on_error')
            else:
                self._robot_events.clear_abort_all()
                self._robot_events.on_error.clear()
                self._robot_events.on_error_reset.set()
                self._callback_queue.put('on_error_reset')
            self._robot_state.error_status = status_flags[3]

        if self._robot_state.pause_motion_status != status_flags[4]:
            if status_flags[4]:
                self._robot_events.on_motion_resumed.clear()
                self._robot_events.on_motion_paused.set()
                self._callback_queue.put('on_motion_paused')
            else:
                self._robot_events.on_motion_paused.clear()
                self._robot_events.on_motion_resumed.set()
                self._callback_queue.put('on_motion_resumed')
            self._robot_state.pause_motion_status = status_flags[4]

        if self._robot_state.end_of_block_status != status_flags[5]:
            if status_flags[5]:
                self._robot_events.on_end_of_block.set()
            else:
                self._robot_events.on_end_of_block.clear()
            self._robot_state.end_of_block_status = status_flags[5]

        self._robot_events.on_status_updated.set()
        self._callback_queue.put('on_status_updated')

    def _handle_checkpoint_response(self, response):
        """Handle the checkpoint message from the robot, set the appropriate events, etc.

        Parameters
        ----------
        response : Message object
            Response message which includes the received checkpoint id.

        """
        assert response.id == MX_ST_CHECKPOINT_REACHED
        checkpoint_id = int(response.data)

        # Check user checkpoints.
        if checkpoint_id in self._user_checkpoints and self._user_checkpoints[checkpoint_id]:
            self._user_checkpoints[checkpoint_id].pop(0).set()
            # If list corresponding to checkpoint id is empty, remove the key from the dict.
            if not self._user_checkpoints[checkpoint_id]:
                self._user_checkpoints.pop(checkpoint_id)
            # If there are events are waiting on 'any checkpoint', set them all.
            if '*' in self._internal_checkpoints and self._internal_checkpoints['*']:
                for event in self._internal_checkpoints.pop('*'):
                    event.set()
            # Enque the on_checkpoint_reached callback.
            self._callback_queue.put('on_checkpoint_reached', checkpoint_id)

        # Check internal checkpoints.
        elif checkpoint_id in self._internal_checkpoints and self._internal_checkpoints[checkpoint_id]:
            self._internal_checkpoints[checkpoint_id].pop(0).set()
            # If list corresponding to checkpoint id is empty, remove the key from the dict.
            if not self._internal_checkpoints[checkpoint_id]:
                self._internal_checkpoints.pop(checkpoint_id)
        else:
            self.logger.warning('Received un-tracked checkpoint. Please use ExpectExternalCheckpoint() to track.')

    #####################################################################################
    # Public methods = Pascal case is used to maintain consistency with text and c++ API.
    #####################################################################################

    ### General management functions.

    def RegisterCallbacks(self, callbacks, run_callbacks_in_separate_thread):
        """Register callback functions to be executed.

        Parameters
        ----------
        callbacks : RobotCallbacks object
            Object containing all callback functions.
        run_callbacks_in_separate_thread : bool
            If true, callbacks are run automatically in thread. If false, RunCallbacks must be used.
            **Running callbacks in a separate thread means the user application MUST BE THREAD SAFE!**
        """
        # Check that callbacks are an instance of the appropriate class.
        if not isinstance(callbacks, RobotCallbacks):
            raise TypeError('Callbacks object is not the appropriate class.')

        if self._monitor_handler_thread or self._command_response_handler_thread:
            raise InvalidStateError('Callbacks cannot be set if already connected.')

        self._callback_queue = CallbackQueue(callbacks)

        self._robot_callbacks = callbacks
        if run_callbacks_in_separate_thread:
            self._callback_thread = threading.Thread(target=self._handle_callbacks,
                                                     args=(
                                                         self.logger,
                                                         self._callback_queue,
                                                         self._robot_callbacks,
                                                     ))
            self._callback_thread.start()

    def UnregisterCallbacks(self):
        """Unregister callback functions and terminate callback handler thread if applicable.

        """
        if self._callback_thread:
            self._callback_queue.put(TERMINATE)
            self._callback_thread.join(timeout=self.default_timeout)

        self._robot_callbacks = RobotCallbacks()
        self._callback_queue = CallbackQueue(self._robot_callbacks)
        self._callback_thread = None

    def RunCallbacks(self):
        """Run all triggered callback functions.

        """
        if self._callback_thread:
            raise InvalidStateError(
                'Cannot call RunCallbacks since callback handler is already running in separate thread.')

        # Setting timeout=0 means we don't block on an empty queue.
        self._handle_callbacks(self.logger, self._callback_queue, self._robot_callbacks, timeout=0)

    ### Robot control functions.

    def Connect(
        self,
        address=MX_DEFAULT_ROBOT_IP,
        enable_synchronous_mode=False,
        disconnect_on_exception=True,
        monitor_mode=False,
        offline_mode=False,
    ):
        """Attempt to connect to a physical Mecademic Robot.

        Parameters
        ----------
        address : string
            The IP address associated to the Mecademic Robot.
        enable_synchronous_mode : bool
            If true, each command will wait until previous is done executing.
        disconnect_on_exception : bool
            If true, will attempt to disconnect from the robot on exception from api call.
        monitor_mode : bool
            If true, command connection will not be established.
        offline_mode : bool
            If true, socket connections are not created, only used for testing.

        """
        with self._main_lock:

            # Check that the ip address is valid and set address.
            if not isinstance(address, str):
                raise TypeError('Invalid IP address.')
            ipaddress.ip_address(address)
            self._address = address

            self._enable_synchronous_mode = enable_synchronous_mode
            self._disconnect_on_exception = disconnect_on_exception

            self._offline_mode = offline_mode
            self._monitor_mode = monitor_mode

            if not self._monitor_mode:
                self._initialize_command_socket()
                self._initialize_command_connection()

            self._initialize_monitoring_socket()
            self._initialize_monitoring_connection()

            self._robot_events.clear_all()

            self._robot_events.on_deactivated.set()
            self._robot_events.on_error_reset.set()
            self._robot_events.on_p_stop_reset.set()
            self._robot_events.on_motion_resumed.set()
            self._robot_events.on_brakes_activated.set()

            self._robot_events.on_status_updated.set()
            self._robot_events.on_conf_updated.set()
            self._robot_events.on_joints_updated.set()
            self._robot_events.on_pose_updated.set()

            self._robot_events.on_connected.set()
            self._callback_queue.put('on_connected')

    def Disconnect(self):
        """Disconnects Mecademic Robot object from the physical Mecademic robot.

        """
        self.logger.debug('Disconnecting from the robot.')

        # Don't acquire _main_lock while shutting down queues to avoid deadlock.
        self._shut_down_queue_threads()

        with self._main_lock:
            self._shut_down_socket_threads()

            # Invalidate checkpoints.
            self._invalidate_checkpoints()

            # Reset attributes which should not persist after disconnect.
            self._reset_disconnect_attributes()

            # Finally, close sockets.
            if self._command_socket is not None:
                try:
                    self._command_socket.close()
                except Exception as e:
                    self.logger.error('Error closing command socket. ' + str(e))
                self._command_socket = None
            if self._monitor_socket is not None:
                try:
                    self._monitor_socket.close()
                except Exception as e:
                    self.logger.error('Error closing monitor socket. ' + str(e))
                self._monitor_socket = None

            self._robot_events.on_connected.clear()
            self._robot_events.on_disconnected.set()
            self._callback_queue.put('on_disconnected')

            self._robot_events.abort_all_except_on_connected()

    @disconnect_on_exception
    def ActivateRobot(self):
        """Activate the robot.

        """
        with self._main_lock:
            self._check_internal_states()
            self._send_command('ActivateRobot')

        if self._enable_synchronous_mode:
            self.WaitActivated()

    @disconnect_on_exception
    def Home(self):
        """Home the robot.

        """
        with self._main_lock:
            self._check_internal_states()
            self._send_command('Home')

        if self._enable_synchronous_mode:
            self.WaitHomed()

    @disconnect_on_exception
    def ActivateAndHome(self):
        """Utility function that combines activate and home.

        """
        self.ActivateRobot()
        self.Home()

    @disconnect_on_exception
    def PauseMotion(self):
        """Immediately pause robot motion.

        """
        with self._main_lock:
            self._check_internal_states()
            self._send_command('PauseMotion')

        if self._enable_synchronous_mode:
            self._robot_events.on_motion_paused.wait(timeout=self.default_timeout)

    @disconnect_on_exception
    def ResumeMotion(self):
        """Un-pause robot motion.

        """
        with self._main_lock:
            self._check_internal_states()
            self._send_command('ResumeMotion')

        if self._enable_synchronous_mode:
            self.WaitMotionResumed(timeout=self.default_timeout)

    @disconnect_on_exception
    def DeactivateRobot(self):
        """Deactivate the robot.

        """
        with self._main_lock:
            self._check_internal_states()
            self._send_command('DeactivateRobot')

        if self._enable_synchronous_mode:
            self.WaitDeactivated()

    @disconnect_on_exception
    def ClearMotion(self):
        """Clear the motion queue, includes implicit PauseMotion command.

        """
        with self._main_lock:
            self._check_internal_states()

            # Increment the number of pending ClearMotion requests.
            self._clear_motion_requests += 1
            self._robot_events.on_motion_cleared.clear()

            self._send_command('ClearMotion')

            # Clearing the motion queue also requires clearing checkpoints, as the robot will not send them anymore.
            self._invalidate_checkpoints()

        if self._enable_synchronous_mode:
            self.WaitMotionCleared(timeout=self.default_timeout)

    @disconnect_on_exception
    def MoveJoints(self, *args):
        """Move the robot by specifying each joint's target angular position.

        Parameters
        ----------
        joint_1...joint_n : float
            Desired joint angles in degrees.

        """
        if len(args) != self._robot_info.num_joints:
            raise ValueError('Incorrect number of joints sent to command.')

        self._send_motion_command('MoveJoints', args)

    @disconnect_on_exception
    def MoveJointsRel(self, *args):
        """Move the robot relative to current position by specifying each joint's offset angular position.

        Parameters
        ----------
        joint_1...joint_n : float
            Desired joint angles offsets in degrees.

        """
        if len(args) != self._robot_info.num_joints:
            raise ValueError('Incorrect number of joints sent to command.')

        self._send_motion_command('MoveJointsRel', args)

    @disconnect_on_exception
    def MoveJointsVel(self, *args):
        """Moves joints to at desired velocities.

        Parameters
        ----------
        joint_1...joint_n : float
            Desired joint velocities in degrees per second.

        """
        if len(args) != self._robot_info.num_joints:
            raise ValueError('Incorrect number of joints sent to command.')

        self._send_motion_command('MoveJointsVel', args)

    @disconnect_on_exception
    def MovePose(self, x, y, z, alpha, beta, gamma):
        """Move robot's tool to an absolute Cartesian position (non-linear move, but all joints arrive simultaneously).

        Parameters
        ----------
        x, y, z : float
            Desired end effector coordinates in mm.
        alpha, beta, gamma
            Desired end effector orientation in degrees.

        """
        self._send_motion_command('MovePose', [x, y, z, alpha, beta, gamma])

    @disconnect_on_exception
    def MoveLin(self, x, y, z, alpha, beta, gamma):
        """Linearly move robot's tool to an absolute Cartesian position.

        Parameters
        ----------
        x, y, z : float
            Desired end effector coordinates in mm.
        alpha, beta, gamma
            Desired end effector orientation in degrees.

        """
        self._send_motion_command('MoveLin', [x, y, z, alpha, beta, gamma])

    @disconnect_on_exception
    def MoveLinRelTRF(self, x, y, z, alpha, beta, gamma):
        """Linearly move robot's tool to a Cartesian position relative to current TRF position.

        Parameters
        ----------
        x, y, z : float
            Desired displacement in mm.
        alpha, beta, gamma
            Desired orientation change in deg.

        """
        self._send_motion_command('MoveLinRelTRF', [x, y, z, alpha, beta, gamma])

    @disconnect_on_exception
    def MoveLinRelWRF(self, x, y, z, alpha, beta, gamma):
        """Linearly move robot's tool to a Cartesian position relative to a reference frame that has the same
        orientation.

        Parameters
        ----------
        x, y, z : float
            Desired displacement in mm.
        alpha, beta, gamma
            Desired orientation change in deg.

        """
        self._send_motion_command('MoveLinRelWRF', [x, y, z, alpha, beta, gamma])

    @disconnect_on_exception
    def MoveLinVelTRF(self, x, y, z, alpha, beta, gamma):
        """Move robot's by Cartesian velocity relative to the TRF.

           Joints will move for a time controlled by velocity timeout (SetVelTimeout).

        Parameters
        ----------
        x, y, z : float
            Desired velocity in mm/s.
        alpha, beta, gamma
            Desired angular velocity in degrees/s.

        """
        self._send_motion_command('MoveLinVelTRF', [x, y, z, alpha, beta, gamma])

    @disconnect_on_exception
    def MoveLinVelWRF(self, x, y, z, alpha, beta, gamma):
        """Move robot's by Cartesian velocity relative to the WRF.

           Joints will move for a time controlled by velocity timeout (SetVelTimeout).

        Parameters
        ----------
        x, y, z : float
            Desired velocity in mm/s.
        alpha, beta, gamma
            Desired angular velocity in degrees/s.

        """
        self._send_motion_command('MoveLinVelWRF', [x, y, z, alpha, beta, gamma])

    @disconnect_on_exception
    def SetVelTimeout(self, t):
        """Maximum time the robot will continue to move after a velocity move command was sent.

        (Can be stopped earlier by sending a velocity command with 0 velocity values.)

        Parameters
        ----------
        t : float
            Desired duration for velocity-mode motion commands.

        """
        self._send_motion_command('SetVelTimeout', [t])

    @disconnect_on_exception
    def SetConf(self, shoulder, elbow, wrist):
        """Manually set inverse kinematics options (and disable auto-conf).

        Parameters
        ----------
        shoulder : +1 or -1
            Shoulder inverse kinematics parameter.
        elbow : +1 or -1
            Elbow inverse kinematics parameter.
        wrist : +1 or -1
            Wrist inverse kinematics parameter.

        """
        self._send_motion_command('SetConf', [shoulder, elbow, wrist])

    @disconnect_on_exception
    def SetAutoConf(self, e):
        """Enable or disable auto-conf (automatic selection of inverse kinematics options).

        Parameters
        ----------
        e : boolean
            If true, robot will automatically choose the best configuation for the desired pose.

        """
        self._send_motion_command('SetAutoConf', [int(e)])

    @disconnect_on_exception
    def SetConfTurn(self, n):
        """Manually set the last joint turn configuration parameter.

        Parameters
        ----------
        n : integer
            The turn number for joint 6.

        """
        self._send_motion_command('SetConfTurn', [n])

    @disconnect_on_exception
    def SetAutoConfTurn(self, e):
        """Enable or disable auto-conf (automatic selection of inverse kinematics options) for joint 6..

        Parameters
        ----------
        e : boolean
            If true, robot will automatically choose the best configuation for the desired pose.

        """
        self._send_motion_command('SetAutoConfTurn', [int(e)])

    @disconnect_on_exception
    def SetBlending(self, p):
        """Set percentage of blending between consecutive movements in the same mode (velocity or cartesian).

        Note: There can't be blending between joint mode and Cartesian mode moves.

        Parameters
        ----------
        p : float
            Percentage blending between actions.

        """
        self._send_motion_command('SetBlending', [p])

    @disconnect_on_exception
    def SetCartAcc(self, p):
        """Set target acceleration (linear and angular) during MoveLin commands.

        Parameters
        ----------
        p : float
            Percentage of maximum acceleration.

        """
        self._send_motion_command('SetCartAcc', [p])

    @disconnect_on_exception
    def SetCartAngVel(self, w):
        """Set maximum angular velocity during MoveLin commands.

        Parameters
        ----------
        p : float
            Maximum angular velocity in deg/s.

        """
        self._send_motion_command('SetCartAngVel', [w])

    @disconnect_on_exception
    def SetCartLinVel(self, w):
        """Set maximum linear velocity during MoveLin commands.

        Note: Actual linear velocity may be lower if necessary to avoid exceeding maximum angular velocity.

        Parameters
        ----------
        p : float
            Maximum angular velocity in deg/s.

        """
        self._send_motion_command('SetCartLinVel', [w])

    @disconnect_on_exception
    def MoveGripper(self, state=GRIPPER_OPEN):
        """Open or close the gripper.

        Corresponds to text API calls "GripperOpen" / "GripperClose".

        Parameters
        ----------
        state : boolean
            Open or close the gripper (GRIPPER_OPEN or GRIPPER_CLOSE)

        """
        if state:
            self._send_motion_command('GripperOpen')
        else:
            self._send_motion_command('GripperClose')

    @disconnect_on_exception
    def SetGripperForce(self, p):
        """Set the gripper's force in percent.

        Parameters
        ----------
        p : float
            The desired force in percent.

        """
        self._send_motion_command('SetGripperForce', [p])

    @disconnect_on_exception
    def SetGripperVel(self, p):
        """Set the gripper's velocity in percent.

        Parameters
        ----------
        p : float
            The desired velocity in percent.

        """
        self._send_motion_command('SetGripperVel', [p])

    @disconnect_on_exception
    def SetJointAcc(self, p):
        """Set target joint acceleration during MoveJoints commands.

        Parameters
        ----------
        p : float
            Target acceleration, in percent.

        """
        self._send_motion_command('SetJointAcc', [p])

    @disconnect_on_exception
    def SetJointVel(self, p):
        """Set target joint velocity during MoveJoints commands.

        Parameters
        ----------
        p : float
            Target joint velocity, in percent.

        """
        self._send_motion_command('SetJointVel', [p])

    @disconnect_on_exception
    def SetTRF(self, x, y, z, alpha, beta, gamma):
        """Set the TRF (tool reference frame) Cartesian position.

        Parameters
        ----------
        x, y, z : float
            Desired reference coordinates in mm.
        alpha, beta, gamma
            Desired reference orientation in degrees.

        """
        self._send_motion_command('SetTRF', [x, y, z, alpha, beta, gamma])

    @disconnect_on_exception
    def SetWRF(self, x, y, z, alpha, beta, gamma):
        """Set the WRF (world reference frame) Cartesian position.

        Parameters
        ----------
        x, y, z : float
            Desired reference coordinates in mm.
        alpha, beta, gamma
            Desired reference orientation in degrees.

        """
        self._send_motion_command('SetWRF', [x, y, z, alpha, beta, gamma])

    @disconnect_on_exception
    def SetCheckpoint(self, n):
        """Set checkpoint with desired id.

        Parameters
        ----------
        n : int
            Desired checkpoint id.

        Return
        ------
        Checkpoint object
            Object to use to wait for the checkpoint.

        """
        with self._main_lock:
            self._check_internal_states()
            assert MX_CHECKPOINT_ID_MIN <= n <= MX_CHECKPOINT_ID_MAX
            return self._set_checkpoint_impl(n)

    @disconnect_on_exception
    def ExpectExternalCheckpoint(self, n):
        """Expect the robot to receive a checkpoint with given id (e.g. from saved program).

        Parameters
        ----------
        n : int
            Id of expected checkpoint.

        Return
        ------
        Checkpoint object
            Object to use to wait for the checkpoint.

        """
        with self._main_lock:
            self._check_internal_states()
            assert MX_CHECKPOINT_ID_MIN <= n <= MX_CHECKPOINT_ID_MAX
            return self._set_checkpoint_impl(n, send_to_robot=False)

    @disconnect_on_exception
    def WaitForAnyCheckpoint(self, timeout=None):
        """Pause program execution until any checkpoint has been received from the robot.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the checkpoint (in seconds).

        Return
        ------
        boolean
            True if wait was successful, false otherwise.

        """
        with self._main_lock:
            self._check_internal_states()
            if '*' not in self._internal_checkpoints:
                self._internal_checkpoints['*'] = list()
            event = InterruptableEvent()
            self._internal_checkpoints['*'].append(event)

        return event.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitConnected(self, timeout=None):
        """Pause program execution until robot is disconnected.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the event (in seconds).

        Return
        ------
        boolean
            True if wait was successful, false otherwise.

        """
        return self._robot_events.on_connected.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitDisconnected(self, timeout=None):
        """Pause program execution until the robot is disconnected.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the event (in seconds).

        Return
        ------
        boolean
            True if wait was successful, false otherwise.

        """
        return self._robot_events.on_disconnected.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitActivated(self, timeout=None):
        """Pause program execution until the robot is activated.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the event (in seconds).

        Return
        ------
        boolean
            True if wait was successful, false otherwise.

        """
        return self._robot_events.on_activated.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitDeactivated(self, timeout=None):
        """Pause program execution until the robot is deactivated.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the event (in seconds).

        Return
        ------
        boolean
            True if wait was successful, false otherwise.

        """
        return self._robot_events.on_deactivated.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitHomed(self, timeout=None):
        """Pause program execution until the robot is homed.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the event (in seconds).

        Return
        ------
        boolean
            True if wait was successful, false otherwise.

        """
        return self._robot_events.on_homed.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitMotionResumed(self, timeout=None):
        """Pause program execution until the robot motion is resumed.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the event (in seconds).

        Return
        ------
        boolean
            True if wait was successful, false otherwise.

        """
        return self._robot_events.on_motion_resumed.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitMotionPaused(self, timeout=None):
        """Pause program execution until the robot motion is paused.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the event (in seconds).

        Return
        ------
        boolean
            True if wait was successful, false otherwise.

        """
        return self._robot_events.on_motion_paused.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitMotionCleared(self, timeout=None):
        """Pause program execution until all pending request to clear motion have been acknowledged.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the event (in seconds).

        Return
        ------
        boolean
            True if wait was successful, false otherwise.

        """

        return self._robot_events.on_motion_cleared.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitIdle(self, timeout=None):
        """Pause program execution until robot is idle.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the event (in seconds).

        Return
        ------
        boolean
            True if wait was successful, false otherwise.

        """
        checkpoint = self._set_checkpoint_internal()

        start_time = time.time()
        if not checkpoint.wait(timeout=timeout):
            return False
        end_time = time.time()

        if timeout:
            remaining_timeout = timeout - (end_time - start_time)
        else:
            remaining_timeout = None

        return self._robot_events.on_end_of_block.wait(timeout=remaining_timeout)

    @disconnect_on_exception
    def ResetError(self):
        """Attempt to reset robot error.

        """
        with self._main_lock:
            self._check_internal_states()
            self._send_command('ResetError')

        if self._enable_synchronous_mode:
            self._robot_events.on_error_reset.wait()

    @disconnect_on_exception
    def ResetPStop(self):
        """Attempt to reset robot pstop.

        """
        with self._main_lock:
            self._check_internal_states()
            self._send_command('ResetPStop')

        if self._enable_synchronous_mode:
            self._robot_events.on_p_stop_reset.wait()

    @disconnect_on_exception
    def Delay(self, t):
        """Set a delay between motion commands.

        Parameters
        ----------
        t : float
            Desired pause duration in seconds.

        """
        with self._main_lock:
            self._check_internal_states()
            if not self._robot_events.on_homed.is_set():
                raise InvalidStateError('This command requires robot to be homed.')
            self._send_command('Delay', [t])
            if self._enable_synchronous_mode:
                checkpoint = self._set_checkpoint_internal()

        if self._enable_synchronous_mode:
            checkpoint.wait()

    @disconnect_on_exception
    def SendCustomCommand(self, command, wait_for_response=False, timeout=None):
        """Send custom command to robot.

        Parameters
        ----------
        command : str
            Desired custom command.

        """
        with self._main_lock:
            self._check_internal_states()

            if wait_for_response:
                self._custom_response_queue = queue.Queue()

            self._send_command(command)

            if wait_for_response:
                response = self._custom_response_queue.get(block=True, timeout=timeout)
                self._custom_response_queue = None
                return response

    @disconnect_on_exception
    def StartOfflineProgram(self, n, timeout=None):
        """Start an offline program.

        Offline programs need to be recorded using the robot's Web Portal (or text API).
        This API can only start an already recorded offline program.
        Callback on_offline_program_state will indicate when program is started or not.

        Parameters
        ----------
        n : int
            Id of offline program to start.

        """
        with self._main_lock:
            self._check_internal_states()
            self._robot_events.on_offline_program_started.clear()

            self._send_command('StartProgram', [n])

        if self._enable_synchronous_mode:
            try:
                self._robot_events.on_offline_program_started.wait(timeout=timeout)
            except InterruptException:
                raise InvalidStateError('Offline program start not confirmed. Does program {} exist?'.format(n))

    ### Non-motion commands.

    @disconnect_on_exception
    def GetJoints(self, include_timestamp=False, synchronous_update=False, timeout=None):
        """Returns the current joint positions of the robot.

        Uses RT commands if possible, otherwise uses legacy versions.

        Parameters
        ----------
        include_timestamp : bool
            If true, return a TimestampedData object, otherwise just return joints angles.
        synchronous_update : bool
            If true, requests updated joints positions and waits for response, else uses last known positions.
        timeout : float
            Maximum time in second to wait for forced update.

        Return
        ------
        TimestampedData or list of floats
            Returns joint positions in degrees.

        """
        if synchronous_update:
            with self._main_lock:
                self._check_internal_states()
                if self._robot_events.on_joints_updated.is_set():
                    self._robot_events.on_joints_updated.clear()
                    if self._robot_info.rt_message_capable:
                        self._send_command('GetRtJointPos')
                    else:
                        self._send_command('GetJoints')

            if not self._robot_events.on_joints_updated.wait(timeout=timeout):
                raise TimeoutError

        with self._main_lock:
            if include_timestamp:
                if not self._robot_info.rt_message_capable:
                    raise InvalidStateError('Cannot provide timestamp with current robot firmware or model.')
                else:
                    return copy.deepcopy(self._robot_state.target_joint_positions)

            return copy.deepcopy(self._robot_state.target_joint_positions.data)

    @disconnect_on_exception
    def GetPose(self, include_timestamp=False, synchronous_update=False, timeout=None):
        """Returns the current end-effector pose of the robot. WARNING: NOT UNIQUE.

        Parameters
        ----------
        include_timestamp : bool
            If true, return a TimestampedData object, otherwise just return joints angles.
        synchronous_update : bool
            If true, requests updated pose and waits for response, else uses last know pose.
        timeout : float
            Maximum time in second to wait for forced update.

        Return
        ------
        TimestampedData or list of floats
            Returns end-effector pose [x, y, z, alpha, beta, gamma].

        """

        if synchronous_update:
            with self._main_lock:
                self._check_internal_states()
                if self._robot_events.on_pose_updated.is_set():
                    self._robot_events.on_pose_updated.clear()
                    if self._robot_info.rt_message_capable:
                        self._send_command('GetRtCartPos')
                    else:
                        self._send_command('GetPose')

            if not self._robot_events.on_pose_updated.wait(timeout=timeout):
                raise TimeoutError

        with self._main_lock:
            if include_timestamp:
                if not self._robot_info.rt_message_capable:
                    raise InvalidStateError('Cannot provide timestamp with current robot firmware or model.')
                else:
                    return copy.deepcopy(self._robot_state.target_end_effector_pose)

            return copy.deepcopy(self._robot_state.target_end_effector_pose.data)

    @disconnect_on_exception
    def SetMonitoringInterval(self, t):
        """Sets the rate at which the monitoring port sends data.

        Parameters
        ----------
        t : float
            Monitoring interval duration in seconds.

        """
        with self._main_lock:
            self._check_internal_states()
            self._send_command('SetMonitoringInterval', [t])

    @disconnect_on_exception
    def SetRTC(self, t):
        """Sets the rate at which the monitoring port sends data.

        Parameters
        ----------
        t : int
            Unix epoch time (seconds since 00:00:00 UTC Jan 1, 1970).

        """
        with self._main_lock:
            self._check_internal_states()
            self._send_command('SetRTC', [t])

    @disconnect_on_exception
    def ActivateSim(self):
        """Enables simulation mode. Motors don't move, but commands will be processed.

        """
        with self._main_lock:
            self._check_internal_states()
            self._send_command('ActivateSim')

    @disconnect_on_exception
    def DeactivateSim(self):
        """Disables simulation mode. Motors don't move, but commands will be processed.

        """
        with self._main_lock:
            self._check_internal_states()
            self._send_command('DeactivateSim')

    @disconnect_on_exception
    def GetConf(self, synchronous_update=False, timeout=None):
        """Get robot's current (physical) inverse-kinematics configuration.

        Returns
        -------
        list of ints
            Configuration status of robot.

        """
        if synchronous_update:
            with self._main_lock:
                self._check_internal_states()
                if self._robot_events.on_conf_updated.is_set():
                    self._robot_events.on_conf_updated.clear()
                    self._send_command('GetConf')

            if not self._robot_events.on_conf_updated.wait(timeout=timeout):
                raise TimeoutError

        return self._robot_state.configuration

    @disconnect_on_exception
    def ActivateBrakes(self, activated=True):
        """Enable/disable the brakes. These commands are only available when the robot is deactivated.

        By default, brakes are enabled until robot is activated (brakes are automatically disabled upon activation).
        Corresponds to text API calls "BrakesOn" / "BrakesOff".

        Parameters
        ----------
        activated : bool
            Engage brakes if true, otherwise disengage brakes.

        """
        with self._main_lock:
            self._check_internal_states()
            if activated:
                self._send_command('BrakesOn')
            else:
                self._send_command('BrakesOff')

        if self._enable_synchronous_mode:
            if activated:
                self._robot_events.on_brakes_activated.wait()
            else:
                self._robot_events.on_brakes_deactivated.wait()

    def GetRobotInfo(self):
        """Return a copy of the known robot information.

        Return
        ------
        RobotInfo
            Object containing robot information.

        """
        with self._main_lock:
            return copy.deepcopy(self._robot_info)

    def GetRobotState(self):
        """Return a copy of the current robot state.

        Return
        ------
        RobotState
            Object containing the current robot state.

        """
        with self._main_lock:
            return copy.deepcopy(self._robot_state)

    def StartLogging(self, file_path=None, wait_idle=True, timeout=None):
        """Start logging robot state to file.

        Fields logged are controlled by SetRealtimeMonitoring(). Logging frequency is set by SetMonitoringInterval().
        By default, will wait until robot is idle before logging.

        Parameters
        ----------
        filename : string or None
            File path to saved log.

        wait_idle : bool
            If true, will wait for robot to be idle before starting logging.

        timeout : float or None
            Max time in seconds to wait for idle before logging.

        """
        if self._file_logger != None:
            raise InvalidStateError('Another file logging operation is in progress.')

        if wait_idle:
            self.WaitIdle(timeout=timeout)

        self._file_logger = CSVFileLogger(self._robot_info, ['target_joint_positions', 'target_end_effector_pose'],
                                          self._robot_state, file_path)

    def StopLogging(self, wait_idle=True, timeout=None):
        """Stop logging robot state to file.

        wait_idle : bool
            If true, will wait for robot to be idle before ending logging.

        timeout : float or None
            Max time in seconds to wait for idle before ending logging.

        """
        if wait_idle:
            self.WaitIdle(timeout=timeout)

        self._file_logger.end_log(trim_last_command=wait_idle)
        self._file_logger = None