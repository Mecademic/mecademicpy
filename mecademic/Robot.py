#!/usr/bin/env python3
import logging
import ipaddress
import time
import socket
import threading
import queue
import functools

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

    @classmethod
    def from_csv(cls, input_string):
        """ Construct from comma-separated string.

        Parameters
        ----------
        input_string : string
            Comma-separated string. First value is timestamp, rest is data.

        Return
        ------
        TimestampedData object

        """
        floats = string_to_floats(input_string)
        return cls(floats[0], floats[1:])

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
        return cls(0, [0] * length)

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


class RobotState:
    """Class for storing the internal state of a generic Mecademic robot.

    Attributes
    ----------
    joint_positions : vector
        The positions of the robot joints in degrees.
    end_effector_pose : vector
        The end effector pose in [x, y, z, alpha, beta, gamma] (mm and degrees).

    nc_joint_positions : TimestampedData
        Controller desired joint positions in degrees [theta_1...6], includes timestamp.
    nc_end_effector_pose : TimestampedData
        Controller desired end effector pose [x, y, z, alpha, beta, gamma], includes timestamp.

    nc_joint_velocity : TimestampedData
        Controller desired joint velocity in degrees/second [theta_dot_1...6], includes timestamp.
    nc_end_effector_velocity : TimestampedData
        Controller desired end effector velocity in mm/s and degrees/s, includes timestamp.
    nc_joint_configurations : TimestampedData
        Controller desired joint configurations.
    nc_multiturn : TimestampedData
        Controller desired joint 6 multiturn configuration.

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
        Drive-measured joint configurations.
    drive_multiturn : TimestampedData
        Drive-measured joint 6 multiturn configuration.

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
    end_of_movement_status : boolean
        True if robot is idle.
    cmd_pending_count : int
        Number of commands pending in the robot's motion queue.
    configuration : array
        Current configuration of the robot.

    """
    def __init__(self, num_joints):
        self.joint_positions = [0] * num_joints  # degrees
        self.end_effector_pose = [0] * num_joints  # mm and degrees

        self.nc_joint_positions = TimestampedData.zeros(num_joints)  # microseconds timestamp, degrees
        self.nc_end_effector_pose = TimestampedData.zeros(num_joints)  # microseconds timestamp, mm and degrees

        self.nc_joint_velocity = TimestampedData.zeros(num_joints)  # microseconds timestamp, degrees/second
        self.nc_end_effector_velocity = TimestampedData.zeros(num_joints)  # microseconds timestamp, mm/s and deg/s

        self.nc_joint_configurations = TimestampedData.zeros(3)
        self.nc_multiturn = TimestampedData.zeros(1)

        self.drive_joint_positions = TimestampedData.zeros(num_joints)  # microseconds timestamp, degrees
        self.drive_end_effector_pose = TimestampedData.zeros(num_joints)  # microseconds timestamp, mm and degrees

        self.drive_joint_velocity = TimestampedData.zeros(num_joints)  # microseconds timestamp, degrees/second
        self.drive_joint_torque_ratio = TimestampedData.zeros(num_joints)  # microseconds timestamp, percent of maximum
        self.drive_end_effector_velocity = TimestampedData.zeros(num_joints)  # microseconds timestamp, mm/s and deg/s

        self.drive_joint_configurations = TimestampedData.zeros(3)
        self.drive_multiturn = TimestampedData.zeros(1)

        # Contains dictionary of accelerometers stored in the robot.
        self.accelerometer = dict()  # 16000 = 1g

        self.max_queue_size = 0

        # The following status fields are updated together, and is protected by a single lock.
        self.activation_state = False
        self.homing_state = False
        self.simulation_mode = False
        self.error_status = False
        self.pause_motion_status = False
        self.end_of_block_status = False
        self.end_of_movement_status = False

        self.cmd_pending_count = 0
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
    on_cmd_pending_count_updated : event
        Set if robot number of pending commands has been updated.
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
        self.on_cmd_pending_count_updated = InterruptableEvent()
        self.on_joints_updated = InterruptableEvent()
        self.on_pose_updated = InterruptableEvent()

        self.on_brakes_activated = InterruptableEvent()
        self.on_brakes_deactivated = InterruptableEvent()

        self.on_offline_program_started = InterruptableEvent()

        self.on_disconnected.set()
        self.on_deactivated.set()
        self.on_error_reset.set()
        self.on_p_stop_reset.set()
        self.on_motion_resumed.set()
        self.on_deactivate_sim.set()

        self.on_status_updated.set()
        self.on_conf_updated.set()
        self.on_cmd_pending_count_updated.set()
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
    def __init__(self,
                 address=MX_DEFAULT_ROBOT_IP,
                 enable_synchronous_mode=False,
                 disconnect_on_exception=True,
                 offline_mode=False):
        """Constructor for an instance of the Controller class.

        Parameters
        ----------
        address : string
            The IP address associated to the Mecademic Robot.
        enable_synchronous_mode : bool
            If true, each command will wait until previous is done executing.
        disconnect_on_exception : bool
            If true, will attempt to disconnect from the robot on exception from api call.
        offline_mode : bool
            If true, will not check child threads before executing api calls.

        """
        self._is_initialized = False

        # Check that the ip address is a string.
        if not isinstance(address, str):
            raise TypeError('Please provide a string argument for the address.')

        # Check that the ip address is valid.
        ipaddress.ip_address(address)

        self._address = address

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

        self._init_states()

        self._enable_synchronous_mode = enable_synchronous_mode
        self._disconnect_on_exception = disconnect_on_exception
        self._offline_mode = offline_mode
        self.logger = logging.getLogger(__name__)
        self.default_timeout = 10

        self._is_initialized = True

    def __del__(self):
        # Only attempt to disconnect if the object was initialized.
        if self._is_initialized:
            self.Disconnect()
            self.UnregisterCallbacks()

    def _init_states(self):
        self._command_rx_queue = queue.Queue()
        self._command_tx_queue = queue.Queue()
        self._monitor_rx_queue = queue.Queue()

        self._user_checkpoints = dict()
        self._internal_checkpoints = dict()
        self._internal_checkpoint_counter = MX_CHECKPOINT_ID_MAX + 1

        self._robot_state = RobotState(6)
        self._robot_events = RobotEvents()

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
            except ConnectionAbortedError:
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
    def _handle_checkpoint_response(response, user_checkpoints, internal_checkpoints, logger, callback_queue):
        """Handle the checkpoint message from the robot, set the appropriate events, etc.

        Parameters
        ----------
        response : Message object
            Response message which includes the received checkpoint id.
        user_checkpoints : shared dictionary
            Dictionary of active checkpoint id's (set by user) and corresponding events.
        internal_checkpoints : shared dictionary
            Dictionary of active checkpoint id's (set interally) and corresponding events.

        """
        assert response.id == MX_ST_CHECKPOINT_REACHED
        checkpoint_id = int(response.data)

        # Check user checkpoints.
        if checkpoint_id in user_checkpoints and user_checkpoints[checkpoint_id]:
            user_checkpoints[checkpoint_id].pop(0).set()
            # If list corresponding to checkpoint id is empty, remove the key from the dict.
            if not user_checkpoints[checkpoint_id]:
                user_checkpoints.pop(checkpoint_id)
            # If there are events are waiting on 'any checkpoint', set them all.
            if '*' in internal_checkpoints and internal_checkpoints['*']:
                for event in internal_checkpoints.pop('*'):
                    event.set()
            # Enque the on_checkpoint_reached callback.
            callback_queue.put('on_checkpoint_reached', checkpoint_id)

        # Check internal checkpoints.
        elif checkpoint_id in internal_checkpoints and internal_checkpoints[checkpoint_id]:
            internal_checkpoints[checkpoint_id].pop(0).set()
            # If list corresponding to checkpoint id is empty, remove the key from the dict.
            if not internal_checkpoints[checkpoint_id]:
                internal_checkpoints.pop(checkpoint_id)
        else:
            logger.warning('Received un-tracked checkpoint. Please use ExpectExternalCheckpoint() to track.')

    @staticmethod
    def _command_response_handler(rx_queue, robot_state, user_checkpoints, internal_checkpoints, events, main_lock,
                                  logger, callback_queue, clear_motion_requests):
        """Handle received messages on the command socket.

        Parameters
        ----------
        rx_queue : queue
            Thread-safe queue to get received messages from.
        robot_state : RobotState object
            The current robot state.
        user_checkpoints : shared dictionary
            Dictionary of active checkpoint id's (set by user) and corresponding events.
        internal_checkpoints : shared dictionary
            Dictionary of active checkpoint id's (set interally) and corresponding events.
        events : RobotEvents object
            Contains event objects corresponding to various robot state changes.
        main_lock : recursive lock object
            Used to protect internal state of robot class.
        logger : logger
            Used for logging.
        callback_queue : CallbackQueue
            Used to push triggered callbacks onto.
        clear_motion_requests : int
            Number of active ClearMotion requests.

        """
        while True:
            # Wait for a response to be available from the queue.
            response = rx_queue.get(block=True)

            # Terminate thread if requested.
            if response == TERMINATE:
                return

            with main_lock:
                callback_queue.put('on_command_message', response)

                if response.id == MX_ST_GET_JOINTS:
                    robot_state.joint_positions = string_to_floats(response.data)
                    events.on_joints_updated.set()

                elif response.id == MX_ST_GET_POSE:
                    robot_state.end_effector_pose = string_to_floats(response.data)
                    events.on_pose_updated.set()

                elif response.id == MX_ST_CLEAR_MOTION:
                    if clear_motion_requests <= 1:
                        clear_motion_requests = 0
                        events.on_motion_cleared.set()
                        callback_queue.put('on_motion_cleared')
                    else:
                        clear_motion_requests -= 1

                elif response.id == MX_ST_GET_STATUS_ROBOT:
                    Robot._handle_robot_status_response(response, robot_state, events, callback_queue)
                    events.on_status_activated.set()
                    callback_queue.put('on_status_updated')

                elif response.id == MX_ST_CHECKPOINT_REACHED:
                    Robot._handle_checkpoint_response(response, user_checkpoints, internal_checkpoints, logger,
                                                      callback_queue)

                elif response.id == MX_ST_PSTOP:
                    if bool(int(response.data)):
                        events.on_p_stop_reset.clear()
                        events.on_p_stop.set()
                        callback_queue.put('on_p_stop')
                    else:
                        events.on_p_stop.clear()
                        events.on_p_stop_reset.set()
                        callback_queue.put('on_p_stop_reset')

                elif response.id == MX_ST_GET_CMD_PENDING_COUNT:
                    robot_state.cmd_pending_count = int(response.data)
                    events.on_cmd_pending_count_updated.set()

                elif response.id == MX_ST_GET_CONF:
                    robot_state.configuration = string_to_floats(response.data)
                    events.on_conf_updated.set()

                elif response.id == MX_ST_BRAKES_ON:
                    events.on_brakes_deactivated.clear()
                    events.on_brakes_activated.set()

                elif response.id == MX_ST_BRAKES_OFF:
                    events.on_brakes_activated.clear()
                    events.on_brakes_deactivated.set()

                elif response.id == MX_ST_OFFLINE_START:
                    events.on_offline_program_started.set()
                    callback_queue.put('on_offline_program_state')

                elif response.id == MX_ST_NO_OFFLINE_SAVED:
                    events.on_offline_program_started.abort()

    @staticmethod
    def _handle_robot_status_response(response, robot_state, events, callback_queue):
        """Parse robot status response and update status fields and events.

        Parameters
        ----------
        response : Message object
            Robot status response to parse and handle.

        robot_state : RobotState object
            Stores the robot state.

        events : RobotEvents object
            Stores events associated with changes in robot state.

        callback_queue : queue
            Used to push triggered callbacks onto.

        """
        assert response.id == MX_ST_GET_STATUS_ROBOT
        status_flags = [bool(int(x)) for x in response.data.split(',')]

        if robot_state.activation_state != status_flags[0]:
            if status_flags[0]:
                events.on_deactivated.clear()
                events.on_activated.set()
                events.on_brakes_activated.clear()
                events.on_brakes_deactivated.set()
                callback_queue.put('on_activated')
            else:
                events.on_activated.clear()
                events.on_deactivated.set()
                events.on_brakes_deactivated.clear()
                events.on_brakes_activated.set()
                callback_queue.put('on_deactivated')
            robot_state.activation_state = status_flags[0]

        if robot_state.homing_state != status_flags[1]:
            if status_flags[1]:
                events.on_homed.set()
                callback_queue.put('on_homed')
            else:
                events.on_homed.clear()
            robot_state.homing_state = status_flags[1]

        if robot_state.simulation_mode != status_flags[2]:
            if status_flags[2]:
                events.on_deactivate_sim.clear()
                events.on_activate_sim.set()
                callback_queue.put('on_activate_sim')
            else:
                events.on_activate_sim.clear()
                events.on_deactivate_sim.set()
                callback_queue.put('on_deactivate_sim')
            robot_state.simulation_mode = status_flags[2]

        if robot_state.error_status != status_flags[3]:
            if status_flags[3]:
                events.on_error_reset.clear()
                events.on_error.set()
                callback_queue.put('on_error')
            else:
                events.on_error.clear()
                events.on_error_reset.set()
                callback_queue.put('on_error_reset')
            robot_state.error_status = status_flags[3]

        if robot_state.pause_motion_status != status_flags[4]:
            if status_flags[4]:
                events.on_motion_resumed.clear()
                events.on_motion_paused.set()
                callback_queue.put('on_motion_paused')
            else:
                events.on_motion_paused.clear()
                events.on_motion_resumed.set()
                callback_queue.put('on_motion_resumed')
            robot_state.pause_motion_status = status_flags[4]

        robot_state.end_of_block_status = status_flags[5]
        robot_state.end_of_movement_status = status_flags[6]

    @staticmethod
    def _monitor_handler(monitor_queue, robot_state, events, main_lock, callback_queue):
        """Handle messages from the monitoring port of the robot.

        Parameters
        ----------
        monitor_queue : queue
            Thread-safe queue to get received messages from.
        robot_state : RobotState object
            The current robot state.
        events : RobotEvents object
            Contains event objects corresponding to various robot state changes.
        main_lock : recursive lock object
            Used to protect internal state of robot class.
        callback_queue : queue
            Used to push triggered callbacks onto.

        """
        while True:
            # Wait for a message in the queue.
            response = monitor_queue.get(block=True)

            # Terminate thread if requested.
            if response == TERMINATE:
                return

            callback_queue.put('on_monitor_message', response)

            queue_size = monitor_queue.qsize()
            if queue_size > robot_state.max_queue_size:
                robot_state.max_queue_size = queue_size

            with main_lock:

                if response.id == MX_ST_GET_JOINTS:
                    robot_state.joint_positions = string_to_floats(response.data)
                    events.on_joints_updated.set()

                elif response.id == MX_ST_GET_POSE:
                    robot_state.end_effector_pose = string_to_floats(response.data)
                    events.on_pose_updated.set()

                elif response.id == MX_ST_GET_STATUS_ROBOT:
                    Robot._handle_robot_status_response(response, robot_state, events, callback_queue)
                    events.on_status_updated.set()
                    callback_queue.put('on_status_updated')

                elif response.id == MX_ST_RT_NC_JOINT_POS:
                    robot_state.nc_joint_positions = TimestampedData.from_csv(response.data)
                elif response.id == MX_ST_RT_NC_CART_POS:
                    robot_state.nc_end_effector_pose = TimestampedData.from_csv(response.data)
                elif response.id == MX_ST_RT_NC_JOINT_VEL:
                    robot_state.nc_joint_velocity = TimestampedData.from_csv(response.data)
                elif response.id == MX_ST_RT_NC_CART_VEL:
                    robot_state.nc_end_effector_velocity = TimestampedData.from_csv(response.data)

                elif response.id == MX_ST_RT_NC_CONF:
                    robot_state.nc_joint_configurations = TimestampedData.from_csv(response.data)
                elif response.id == MX_ST_RT_NC_CONF_MULTITURN:
                    robot_state.nc_multiturn = TimestampedData.from_csv(response.data)

                elif response.id == MX_ST_RT_DRIVE_JOINT_POS:
                    robot_state.drive_joint_positions = TimestampedData.from_csv(response.data)
                elif response.id == MX_ST_RT_DRIVE_CART_POS:
                    robot_state.drive_end_effector_pose = TimestampedData.from_csv(response.data)
                elif response.id == MX_ST_RT_DRIVE_JOINT_VEL:
                    robot_state.drive_joint_velocity = TimestampedData.from_csv(response.data)
                elif response.id == MX_ST_RT_DRIVE_JOINT_TORQ:
                    robot_state.drive_joint_torque_ratio = TimestampedData.from_csv(response.data)
                elif response.id == MX_ST_RT_DRIVE_CART_VEL:
                    robot_state.drive_end_effector_velocity = TimestampedData.from_csv(response.data)

                elif response.id == MX_ST_RT_DRIVE_CONF:
                    robot_state.drive_joint_configurations = TimestampedData.from_csv(response.data)
                elif response.id == MX_ST_RT_DRIVE_CONF_MULTITURN:
                    robot_state.drive_multiturn = TimestampedData.from_csv(response.data)

                elif response.id == MX_ST_RT_ACCELEROMETER:
                    # The data is stored as [timestamp, accelerometer_id, {measurements...}]
                    raw_data = string_to_floats(response.data)
                    robot_state.accelerometer[raw_data[1]] = TimestampedData(raw_data[0], raw_data[2:])

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

    def _check_background_threads(self):
        """Check that the thread which handle robot messages are alive. Attempt to disconnect from the robot if not.

        """
        if self._offline_mode:
            return True

        if not (self._command_response_handler_thread and self._command_response_handler_thread.is_alive()):
            self.Disconnect()
            raise InvalidStateError('Command response handler thread has unexpectedly terminated.')
        if not (self._monitor_handler_thread and self._monitor_handler_thread.is_alive()):
            self.Disconnect()
            raise InvalidStateError('Command response handler thread has unexpectedly terminated.')

        if not (self._command_rx_thread and self._command_rx_thread.is_alive()):
            self.Disconnect()
            raise InvalidStateError('Command rx thread has unexpectedly terminated.')
        if not (self._monitor_rx_thread and self._monitor_rx_thread.is_alive()):
            self.Disconnect()
            raise InvalidStateError('Monitor rx thread has unexpectedly termianted.')

        # If tx thread is down, attempt to directly send deactivate command to the robot.
        if not (self._command_tx_thread and self._command_tx_thread.is_alive()):
            self._command_socket.sendall(b'DeactivateRobot\0')
            self.Disconnect()
            raise InvalidStateError('Command tx thread has unexpectedly terminated.')

        return True

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

    def _establish_socket_connections(self):
        """Establish the socket connections to the robot.

        Return
        ------
        bool
            True if both all connections are successful.

        """
        if self._offline_mode:
            return True

        if self._command_socket is not None:
            raise InvalidStateError('Cannot connect since existing command socket exists.')

        if self._monitor_socket is not None:
            raise InvalidStateError('Cannot connect since existing monitor socket exists.')

        try:
            self._command_socket = self._connect_socket(self.logger, self._address, MX_ROBOT_TCP_PORT_CONTROL)

            if self._command_socket is None:
                raise CommunicationError('Command socket could not be created.')

            self._monitor_socket = self._connect_socket(self.logger, self._address, MX_ROBOT_TCP_PORT_FEED)

            if self._monitor_socket is None:
                raise CommunicationError('Monitor socket could not be created.')

        except:
            # Clean up threads and connections on error.
            self.Disconnect()
            raise

        return True

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

    def _establish_socket_threads(self):
        """Establish the threads responsible for reading/sending messages using the sockets.

        Return
        ------
        bool
            True if both all threads are successfully launched.

        """
        if self._offline_mode:
            return True

        try:
            # Create rx thread for command socket communication.
            self._command_rx_thread = self._launch_thread(target=self._handle_socket_rx,
                                                          args=(self._command_socket, self._command_rx_queue))

            # Create tx thread for command socket communication.
            self._command_tx_thread = self._launch_thread(target=self._handle_socket_tx,
                                                          args=(self._command_socket, self._command_tx_queue))

            # Create rx thread for monitor socket communication.
            self._monitor_rx_thread = self._launch_thread(target=self._handle_socket_rx,
                                                          args=(self._monitor_socket, self._monitor_rx_queue))

        except:
            # Clean up threads and connections on error.
            self.Disconnect()
            raise

        return True

    def _initialize_command_connection(self):
        """Attempt to connect to the command port of the Mecademic Robot.

        Returns
        -------
        status : boolean
            Returns the status of the connection, true for success, false for failure.

        """

        try:
            response = self._command_rx_queue.get(block=True, timeout=10)  # 10s timeout.
        except queue.Empty:
            self.logger.error('No response received within timeout interval.')
            self.Disconnect()
            raise CommunicationError('No response received within timeout interval.')
        except BaseException:
            self.Disconnect()
            raise

        # Check that response is appropriate.
        if response.id != MX_ST_CONNECTED:
            self.logger.error('Connection error: %s', response)
            self.Disconnect()
            raise CommunicationError('Connection error: %s', response)

        self._command_response_handler_thread = self._launch_thread(
            target=self._command_response_handler,
            args=(self._command_rx_queue, self._robot_state, self._user_checkpoints, self._internal_checkpoints,
                  self._robot_events, self._main_lock, self.logger, self._callback_queue, self._clear_motion_requests))
        return True

    def _initialize_monitoring_connection(self):
        """Attempt to connect to the monitor port of the Mecademic Robot.

        Returns
        -------
        status : boolean
            Returns the status of the connection, true for success, false for failure.

        """

        self._monitor_handler_thread = self._launch_thread(target=self._monitor_handler,
                                                           args=(self._monitor_rx_queue, self._robot_state,
                                                                 self._robot_events, self._main_lock,
                                                                 self._callback_queue))

        return True

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
            self._check_background_threads()
            self._send_command(command, arg_list)
            if self._enable_synchronous_mode:
                checkpoint = self._set_checkpoint_internal()

        if self._enable_synchronous_mode:
            checkpoint.wait()

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

    def Connect(self):
        """Attempt to connect to a physical Mecademic Robot.

        Returns
        -------
        status : boolean
            Returns the status of the connection, true for success, false for failure.

        """
        with self._main_lock:
            self._establish_socket_connections()
            self._establish_socket_threads()
            self._initialize_command_connection()
            self._initialize_monitoring_connection()

            self._robot_events.clear_all()

            self._robot_events.on_deactivated.set()
            self._robot_events.on_error_reset.set()
            self._robot_events.on_p_stop_reset.set()
            self._robot_events.on_motion_resumed.set()
            self._robot_events.on_brakes_activated.set()

            self._robot_events.on_status_updated.set()
            self._robot_events.on_conf_updated.set()
            self._robot_events.on_cmd_pending_count_updated.set()
            self._robot_events.on_joints_updated.set()
            self._robot_events.on_pose_updated.set()

            self._robot_events.on_connected.set()
            self._callback_queue.put('on_connected')

            return True

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

            # Reset robot state.
            self._init_states()

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
            self._check_background_threads()
            self._send_command('ActivateRobot')

        if self._enable_synchronous_mode:
            self.WaitActivated()

    @disconnect_on_exception
    def Home(self):
        """Home the robot.

        """
        with self._main_lock:
            self._check_background_threads()
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
            self._check_background_threads()
            self._send_command('PauseMotion')

        if self._enable_synchronous_mode:
            self._robot_events.on_motion_paused.wait(timeout=self.default_timeout)

    @disconnect_on_exception
    def ResumeMotion(self):
        """Un-pause robot motion.

        """
        with self._main_lock:
            self._check_background_threads()
            self._send_command('ResumeMotion')

        if self._enable_synchronous_mode:
            self.WaitMotionResumed(timeout=self.default_timeout)

    @disconnect_on_exception
    def DeactivateRobot(self):
        """Deactivate the robot.

        """
        with self._main_lock:
            self._check_background_threads()
            self._send_command('DeactivateRobot')

        if self._enable_synchronous_mode:
            self.WaitDeactivated()

    @disconnect_on_exception
    def ClearMotion(self):
        """Clear the motion queue, includes implicit PauseMotion command.

        """
        with self._main_lock:
            self._check_background_threads()

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
        self._send_motion_command('MoveJoints', args)

    @disconnect_on_exception
    def MoveJointsVel(self, *args):
        """Moves joints to at desired velocities.

        Parameters
        ----------
        joint_1...joint_n : float
            Desired joint velocities in degrees per second.

        """
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
    def MoveLinRelTrf(self, x, y, z, alpha, beta, gamma):
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
    def MoveLinRelWrf(self, x, y, z, alpha, beta, gamma):
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
    def MoveLinVelTrf(self, x, y, z, alpha, beta, gamma):
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
    def MoveLinVelWrf(self, x, y, z, alpha, beta, gamma):
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
    def SetConf(self, c1, c3, c5):
        """Manually set inverse kinematics options (and disable auto-conf).

        Parameters
        ----------
        c1 : +1 or -1
            First inverse kinematics parameter.
        c3 : +1 or -1
            Second inverse kinematics parameter.
        c5 : +1 or -1
            Third inverse kinematics parameter.

        """
        self._send_motion_command('SetConf', [c1, c3, c5])

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
    def SetConfMultiTurn(self, n):
        """Manually set the multi-turn configuration parameter.

        Parameters
        ----------
        n : integer
            The turn number for joint 6.

        """
        self._send_motion_command('SetConfMultiTurn', [n])

    @disconnect_on_exception
    def SetAutoConfMultiTurn(self, e):
        """Enable or disable auto-conf (automatic selection of inverse kinematics options) for joint 6..

        Parameters
        ----------
        e : boolean
            If true, robot will automatically choose the best configuation for the desired pose.

        """
        self._send_motion_command('SetAutoConfMultiTurn', [int(e)])

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
            self._check_background_threads()
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
            self._check_background_threads()
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
            self._check_background_threads()
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
    def ResetError(self):
        """Attempt to reset robot error.

        """
        with self._main_lock:
            self._check_background_threads()
            self._send_command('ResetError')

        if self._enable_synchronous_mode:
            self._robot_events.on_error_reset.wait()

    @disconnect_on_exception
    def ResetPStop(self):
        """Attempt to reset robot pstop.

        """
        with self._main_lock:
            self._check_background_threads()
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
            self._check_background_threads()
            if not self._robot_events.on_homed.is_set():
                raise InvalidStateError('This command requires robot to be homed.')
            self._send_command('Delay', [t])
            if self._enable_synchronous_mode:
                checkpoint = self._set_checkpoint_internal()

        if self._enable_synchronous_mode:
            checkpoint.wait()

    @disconnect_on_exception
    def SendCustomCommand(self, command):
        """Send custom command to robot.

        Parameters
        ----------
        command : str
            Desired custom command.

        """
        with self._main_lock:
            self._send_command(command)

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
            self._robot_events.on_offline_program_started.clear()

            self._send_command('StartProgram', [n])

        if self._enable_synchronous_mode:
            try:
                self._robot_events.on_offline_program_started.wait(timeout=timeout)
            except InterruptException:
                raise InvalidStateError('Offline program start not confirmed. Does program {} exist?'.format(n))

    ### Non-motion commands.

    @disconnect_on_exception
    def GetJoints(self, updated=True, timeout=None):
        """Returns the current joint positions of the robot.

        Return
        ------
        list of floats
            Returns list of joint positions in degrees.

        """
        if updated:
            with self._main_lock:
                self._check_background_threads()
                if self._robot_events.on_joints_updated.is_set():
                    self._robot_events.on_joints_updated.clear()
                    self._send_command('GetJoints')

            if not self._robot_events.on_joints_updated.wait(timeout=timeout):
                raise TimeoutError

        return self._robot_state.joint_positions

    @disconnect_on_exception
    def GetPose(self, updated=True, timeout=None):
        """Returns the current end-effector pose of the robot. WARNING: NOT UNIQUE.

        Return
        ------
        list of floats
            Returns end-effector pose [x, y, z, alpha, beta, gamma].

        """
        if updated:
            with self._main_lock:
                self._check_background_threads()
                if self._robot_events.on_pose_updated.is_set():
                    self._robot_events.on_pose_updated.clear()
                    self._send_command('GetPose')

            if not self._robot_events.on_pose_updated.wait(timeout=timeout):
                raise TimeoutError

        return self._robot_state.end_effector_pose[:]

    @disconnect_on_exception
    def SetMonitoringInterval(self, t):
        """Sets the rate at which the monitoring port sends data.

        Parameters
        ----------
        t : float
            Monitoring interval duration in seconds.

        """
        with self._main_lock:
            self._check_background_threads()
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
            self._check_background_threads()
            self._send_command('SetRTC', [t])

    @disconnect_on_exception
    def ActivateSim(self):
        """Enables simulation mode. Motors don't move, but commands will be processed.

        """
        with self._main_lock:
            self._check_background_threads()
            self._send_command('ActivateSim')

    @disconnect_on_exception
    def DeactivateSim(self):
        """Disables simulation mode. Motors don't move, but commands will be processed.

        """
        with self._main_lock:
            self._check_background_threads()
            self._send_command('DeactivateSim')

    @disconnect_on_exception
    def GetCmdPendingCount(self, updated=True, timeout=None):
        """Gets the number of pending commands on the robot.

        Returns
        -------
        integer
            Number of pending events on the robot.

        """
        if updated:
            with self._main_lock:
                self._check_background_threads()
                if self._robot_events.on_cmd_pending_count_updated.is_set():
                    self._robot_events.on_cmd_pending_count_updated.clear()
                    self._send_command('GetCmdPendingCount')

            if not self._robot_events.on_cmd_pending_count_updated.wait(timeout=timeout):
                raise TimeoutError

        return self._robot_state.cmd_pending_count

    @disconnect_on_exception
    def GetConf(self, updated=True, timeout=None):
        """Get robot's current (physical) inverse-kinematics configuration.

        Returns
        -------
        integer
            Number of pending events on the robot.

        """
        if updated:
            with self._main_lock:
                self._check_background_threads()
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

        """
        with self._main_lock:
            self._check_background_threads()
            if activated:
                self._send_command('BrakesOn')
            else:
                self._send_command('BrakesOff')

        if self._enable_synchronous_mode:
            if activated:
                self._robot_events.on_brakes_activated.wait()
            else:
                self._robot_events.on_brakes_deactivated.wait()
