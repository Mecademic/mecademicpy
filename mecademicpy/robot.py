#!/usr/bin/env python3
import contextlib
import copy
import functools
import ipaddress
import logging
import queue
import re
import socket
import threading
import time
from pathlib import PurePath

import pandas as pd

import mecademicpy.mx_robot_def as mx_def

from ._robot_trajectory_logger import _RobotTrajectoryLogger

GRIPPER_OPEN = True
GRIPPER_CLOSE = False

_CHECKPOINT_ID_MAX_PRIVATE = 8191  # Max allowable checkpoint id, inclusive

_TERMINATE = '--terminate--'


def _string_to_numbers(input_string):
    """Convert comma-separated floats in string form to relevant type.

    Parameters
    ----------
    input_string : string
        Comma-separated floats values encoded as a string.

    Returns
    -------
    list of numbers
        Returns converted list of floats or integers, depending on nature of element in 'input_string'.
"""

    return [(float(x) if ('.' in x) else int(x)) for x in input_string.split(',')]


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


class TimeoutException(MecademicException):
    """Requested timeout during a blocking operation (synchronous mode or Wait* functions) has been reached.
       (raised by InterruptableEvent)"""
    pass


class RobotCallbacks:
    """Class for storing possible status events for the Mecademic robot.

    Attributes
    ----------
        on_connected : function object
            Function to be called once connected.
        on_disconnected : function object
            Function to be called once disconnected.
        on_status_updated : function object
            Function to be called once robot status is updated.
        on_status_gripper_updated : function object
            Function to be called once gripper status is updated.
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
        on_end_of_cycle : function object
            Function to be called each time end of cycle is reached"""

    def __init__(self):
        self.on_connected = None
        self.on_disconnected = None

        self.on_status_updated = None
        self.on_status_gripper_updated = None

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

        self.on_end_of_cycle = None


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
    _interrupted_msg : string
        User message that explains the reason of interruption
"""

    def __init__(self, id=None, data=None):
        self._id = id
        self._data = data
        self._event = threading.Event()
        self._lock = threading.Lock()
        self._interrupted = False
        self._interrupted_msg = ""

    def wait(self, timeout: float = None):
        """Block until event is set or should raise an exception (InterruptException or TimeoutException).

        Attributes
        ----------
        timeout : float
            Maximum duration to wait in seconds.

        Return
        ------
        data : object
            Return the data object (or None for events not returning any data)

        """
        wait_result = self._event.wait(timeout=timeout)
        if self._interrupted:
            if self._interrupted_msg != "":
                raise InterruptException('Event received exception because ' + self._interrupted_msg)
            else:
                raise InterruptException('Event received exception, possibly because event will never be triggered.')
        elif not wait_result:
            raise TimeoutException()
        return self._data

    def set(self, data=None):
        """Set the event and unblock all waits. Optionally modify data before setting.

        """
        with self._lock:
            self._data = data
            self._event.set()

    def abort(self, message=""):
        """Unblock any waits and raise an exception.

        """
        with self._lock:
            if not self._event.is_set():
                self._interrupted_msg = message
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

    @property
    def id(self):
        """Make id a read-only property since it should not be changed after instantiation.

        """
        return self._id

    @property
    def data(self):
        """Make data a read-only property and enforce that it is only assignable at construction or using set().

        """
        return self._data


class Robot:
    """Class for controlling a Mecademic robot.

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

    _robot_info: RobotInfo object
        Store information concerning robot (ex.: serial number)
    _robot_rt_data : RobotRtData object
        Stores most current robot real-time data.
        All attributes of this object are the latest captured on monitor port, so they don't necessarily share the same
        timestamp
    _robot_rt_data_stable : RobotRtData object
        Stores most current robot real-time data, but all attributes of object share the same timestamp
    _robot_status: RobotStatus object
        Stores most current robot status
    _gripper_status: GripperStatus object
        Stores most current gripper status
    _robot_events : RobotEvents object
        Stores events related to the robot state.

    _file_logger : RobotDataLogger object
        Collects RobotInformation, all RobotRtData and SentCommands during determined period

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

    _tmp_rt_joint_pos : list of float
        Values from legacy MX_ST_GET_JOINTS event received in current cycle
    _tmp_rt_cart_pos : list of float
        Values from legacy MX_ST_GET_POSE event received in current cycle

    _tx_sync : integer
        Value sent in the most recent "Sync" request sent to robot
    _rx_sync : integer
        Most recent response to "Sync" (MX_ST_SYNC) received from the robot
"""

    def __init__(self):
        """Constructor for an instance of the Robot class.
        """
        # Initialize member variables that are NOT reset by "with" block (i.e. by __enter__ and __exit__)
        self._is_initialized = False
        # (callbacks remain registered after "with" block
        self._robot_callbacks = RobotCallbacks()
        self._run_callbacks_in_separate_thread = False
        self._reset()

    def __del__(self):
        """Destructor for an instance of the Robot class.

        Warnings
        --------
        In python, the  destructor is called by garbage collector, it may not be called when Robot object
        instance is released so make sure to explicitly Disconnect from the robot, or use "with" block if you
        need to control when the disconnection with robot occurs.
        """
        self._reset()
        self.UnregisterCallbacks()

    def __enter__(self):
        """Function called when entering "with" block with a Robot object instance.

        Raises
        ------
        InvalidStateError
            Exception raised if robot is already connected when entering "with" statement (since by design the usage
            of the "with" statement is to ensure proper disconnection from the robot at the end of the "with" scope
        """
        if self.IsConnected():
            raise InvalidStateError('Robot cannot be connected when entering \'with\' block')
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Function called when exiting "with" block with a Robot object instance.
        This forces disconnection with the robot and reset of all states, except registered callbacks
        which remain attached in case the same Robot object is reconnected later.
        """
        self._reset()

    def _reset(self):
        """ Reset the Robot class (disconnects, stop threads, clears queues, etc).
        (this code is common to constructor, destructor and __exit__ implicit functions)
        Only thing that is not reset are registered callbacks.
        """
        if self._is_initialized:
            self.Disconnect()
            # Note: Don't unregister callbacks, we allow them to remain valid after a "with" block
            # self.UnregisterCallbacks()

        self._address = None

        self._command_socket = None
        self._monitor_socket = None

        self._command_rx_thread = None
        self._command_tx_thread = None
        self._monitor_rx_thread = None

        self._command_response_handler_thread = None
        self._monitor_handler_thread = None

        self._main_lock = threading.RLock()

        # self._robot_callbacks = RobotCallbacks() -> Not reset here, only upon UnregisterCallback
        self._callback_queue = _CallbackQueue(self._robot_callbacks)
        self._callback_thread = None

        self._robot_info = None
        self._robot_rt_data = None
        self._robot_rt_data_stable = None
        self._robot_status = RobotStatus()
        self._gripper_status = GripperStatus()
        self._robot_events = _RobotEvents()

        self._file_logger = None

        self._reset_disconnect_attributes()

        self._enable_synchronous_mode = None
        self._disconnect_on_exception = None

        self._offline_mode = None
        self._monitor_mode = None

        self.logger = logging.getLogger(__name__)
        self.default_timeout = 10

        # Variables to hold joint positions and poses while waiting for timestamp.
        self._tmp_rt_joint_pos = None
        self._tmp_rt_cart_pos = None

        self._tx_sync = 0
        self._rx_sync = 0

        self._is_initialized = True

    def _reset_disconnect_attributes(self):
        self._command_rx_queue = queue.Queue()
        self._command_tx_queue = queue.Queue()
        self._monitor_rx_queue = queue.Queue()
        self._custom_response_events = list()

        self._user_checkpoints = dict()
        self._internal_checkpoints = dict()
        self._internal_checkpoint_counter = mx_def.MX_CHECKPOINT_ID_MAX + 1

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
    def _handle_socket_rx(robot_socket, rx_queue, logger):
        """Handle received data on the socket.

        Parameters
        ----------
        robot_socket : socket
            Socket to use for receiving data.

        rx_queue : queue
            Thread-safe queue to push complete messages onto.

        logger : logger instance
            Logger to use.

        """
        remainder = ''
        while True:
            # Wait for a message from the robot.
            try:
                robot_socket.setblocking(True)
                raw_responses = robot_socket.recv(1024)
            except (ConnectionAbortedError, BrokenPipeError, OSError):
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

                logger.debug(f'Socket Rx - Response: {response}')
                rx_queue.put(_Message.from_string(response))

    @staticmethod
    def _handle_socket_tx(robot_socket, tx_queue, logger):
        """Handle sending data on the socket.

        Parameters
        ----------
        robot_socket : socket
            Socket to use for sending data.

        tx_queue : queue
            Thread-safe queue to get messages from.

        logger : logger instance
            Logger to use.

        """
        while True:
            # Wait for a command to be available from the queue.
            command = tx_queue.get(block=True)

            # Terminate thread if requested, otherwise send the command.
            if command == _TERMINATE:
                return
            else:
                logger.debug(f'Socket Tx - Command: {command}')
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
        except Exception as exception:
            logger.error('Unable to connect to %s:%s.', address, port)
            return None

        logger.debug('Connected to %s:%s.', address, port)
        return new_socket

    @staticmethod
    def _handle_callbacks(logger, callback_queue, callbacks, timeout: float = None):
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

            if callback_name == _TERMINATE:
                return

            callback_function = callbacks.__dict__[callback_name]
            if callback_function is not None:
                if data is not None:
                    callback_function(data)
                else:
                    callback_function()

    #####################################################################################
    # Public methods = Pascal case is used to maintain consistency with text and c++ API.
    #####################################################################################

    # General management functions.

    def RegisterCallbacks(self, callbacks: RobotCallbacks, run_callbacks_in_separate_thread: bool):
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

        if self.IsConnected():
            raise InvalidStateError('Callbacks cannot be set if already connected.')

        self._callback_queue = _CallbackQueue(callbacks)

        # Remember user provided callbacks. Callback thread will actually be started upon Connect.
        self._robot_callbacks = callbacks
        self._run_callbacks_in_separate_thread = run_callbacks_in_separate_thread

    def UnregisterCallbacks(self):
        """Unregister callback functions and terminate callback handler thread if applicable.

        """
        self._stop_callback_thread()

        self._robot_callbacks = RobotCallbacks()
        self._run_callbacks_in_separate_thread = False
        self._callback_queue = _CallbackQueue(self._robot_callbacks)
        self._callback_thread = None

    def RunCallbacks(self):
        """Run all triggered callback functions.

        """
        if self._callback_thread:
            raise InvalidStateError(
                'Cannot call RunCallbacks since callback handler is already running in separate thread.')

        # Setting timeout=0 means we don't block on an empty queue.
        self._handle_callbacks(self.logger, self._callback_queue, self._robot_callbacks, timeout=0)

    def _start_callback_thread(self):
        if self._run_callbacks_in_separate_thread and self._callback_thread is None:
            self._callback_thread = threading.Thread(target=self._handle_callbacks,
                                                     args=(
                                                         self.logger,
                                                         self._callback_queue,
                                                         self._robot_callbacks,
                                                     ))
            self._callback_thread.setDaemon(True)  # Make sure thread does not prevent application from quitting
            self._callback_thread.start()

    def _stop_callback_thread(self):
        if self._callback_thread:
            self._callback_queue.put(_TERMINATE)
            self._callback_thread.join(timeout=self.default_timeout)
            self._callback_thread = None

    # Robot control functions.

    def Connect(
        self,
        address: str = mx_def.MX_DEFAULT_ROBOT_IP,
        enable_synchronous_mode: bool = False,
        disconnect_on_exception: bool = True,
        monitor_mode: bool = False,
        offline_mode: bool = False,
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
                raise TypeError(f'Invalid IP address ({address}).')

            self.logger.info("Connecting to robot: " + address)
            ipaddress.ip_address(address)
            self._address = address

            self._enable_synchronous_mode = enable_synchronous_mode
            self._disconnect_on_exception = disconnect_on_exception

            self._offline_mode = offline_mode
            self._monitor_mode = monitor_mode

            if not self._monitor_mode:
                self._initialize_command_socket()
                self._initialize_command_connection()

            self._robot_events.clear_all()

            self._robot_events.on_deactivated.set()
            self._robot_events.on_error_reset.set()
            self._robot_events.on_p_stop_reset.set()
            self._robot_events.on_motion_resumed.set()
            self._robot_events.on_brakes_activated.set()

            self._robot_events.on_status_updated.set()
            self._robot_events.on_status_gripper_updated.set()
            self._robot_events.on_joints_updated.set()
            self._robot_events.on_pose_updated.set()

            # Start callback thread if necessary
            self._start_callback_thread()

        # Once connected, we fetch some information required by this Robot class (outside main lock).
        connect_to_monitoring_port = True
        if not self._monitor_mode:
            # Fetch the robot serial number
            serial_response = self._send_custom_command('GetRobotSerial',
                                                        expected_responses=[mx_def.MX_ST_GET_ROBOT_SERIAL],
                                                        skip_internal_check=True)
            serial_response_message = serial_response.wait(timeout=self.default_timeout)
            self._robot_info.serial = serial_response_message.data

            # Fetch the current real-time monitoring settings
            if self._robot_info.rt_message_capable:
                real_time_monitoring_response = self._send_custom_command(
                    'GetRealTimeMonitoring',
                    expected_responses=[mx_def.MX_ST_GET_REAL_TIME_MONITORING],
                    skip_internal_check=True)
                real_time_monitoring_response.wait(timeout=self.default_timeout)

            # Check if this robot supports sending monitoring data on ctrl port (which we want to do to avoid race
            # conditions between the two sockets causing potential problems with this API)
            if self._robot_info.rt_on_ctrl_port_capable:
                self._send_custom_command('SetCtrlPortMonitoring(1)', skip_internal_check=True)
                connect_to_monitoring_port = False  # We won't need to connect to monitoring port

        if connect_to_monitoring_port:
            with self._main_lock:
                self._initialize_monitoring_socket()
                self._initialize_monitoring_connection()

        self._robot_events.on_connected.set()
        self._callback_queue.put('on_connected')

    def Disconnect(self):
        """Disconnects Mecademic Robot object from the physical Mecademic robot.

        """
        self.logger.info('Disconnecting from the robot.')

        # Don't acquire _main_lock while shutting down queues to avoid deadlock.
        self._shut_down_queue_threads()

        with self._main_lock:
            message = "explicitly disconnected from the robot"
            self._shut_down_socket_threads()

            # Invalidate checkpoints.
            self._invalidate_checkpoints(message)

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

            self._robot_events.abort_all(message=message)

        # Now that we're disconnected and posted 'on_disconnected' callback we can stop the callback thread
        self._stop_callback_thread()

    def IsConnected(self):
        """Tells if we're actually connected to the robot

        Returns
        -------
        boolean
            true if connected to the robot
        """
        return self._robot_events.on_connected.is_set()

    def set_synchronous_mode(self, sync_mode: bool = True):
        """Enables synchronous mode. In this mode, all commands are blocking until robot's response is received.
           Note that this will apply to next API calls.
           So disabling synchronous mode will not not awake thread already awaiting on synchronous operations.

        Parameters
        ----------
        sync_mode : bool, optional
            Synchronous mode enabled (else asynchronous mode), by default True
        """
        self._enable_synchronous_mode = sync_mode

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
            self._invalidate_checkpoints("motion was cleared")

        if self._enable_synchronous_mode:
            self.WaitMotionCleared(timeout=self.default_timeout)

    @disconnect_on_exception
    def MoveJoints(self, *args: list[float]):
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
    def MoveJointsRel(self, *args: list[float]):
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
    def MoveJointsVel(self, *args: list[float]):
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
    def MovePose(self, x: float, y: float, z: float, alpha: float, beta: float, gamma: float):
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
    def MoveLin(self, x: float, y: float, z: float, alpha: float, beta: float, gamma: float):
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
    def MoveLinRelTRF(self, x: float, y: float, z: float, alpha: float, beta: float, gamma: float):
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
    def MoveLinRelWRF(self, x: float, y: float, z: float, alpha: float, beta: float, gamma: float):
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
    def MoveLinVelTRF(self, x: float, y: float, z: float, alpha: float, beta: float, gamma: float):
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
    def MoveLinVelWRF(self, x: float, y: float, z: float, alpha: float, beta: float, gamma: float):
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
    def SetVelTimeout(self, t: float):
        """Maximum time the robot will continue to move after a velocity move command was sent.

        (Can be stopped earlier by sending a velocity command with 0 velocity values.)

        Parameters
        ----------
        t : float
            Desired duration for velocity-mode motion commands.

        """
        self._send_motion_command('SetVelTimeout', [t])

    @disconnect_on_exception
    def SetConf(self, shoulder: int, elbow: int, wrist: int):
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
    def SetAutoConf(self, e: int):
        """Enable or disable auto-conf (automatic selection of inverse kinematics options).

        Parameters
        ----------
        e : boolean
            If true, robot will automatically choose the best configuration for the desired pose.

        """
        self._send_motion_command('SetAutoConf', [int(e)])

    @disconnect_on_exception
    def SetConfTurn(self, n: int):
        """Manually set the last joint turn configuration parameter.

        Parameters
        ----------
        n : integer
            The turn number for joint 6.

        """
        self._send_motion_command('SetConfTurn', [n])

    @disconnect_on_exception
    def SetAutoConfTurn(self, e: int):
        """Enable or disable auto-conf (automatic selection of inverse kinematics options) for joint 6..

        Parameters
        ----------
        e : boolean
            If true, robot will automatically choose the best configuration for the desired pose.

        """
        self._send_motion_command('SetAutoConfTurn', [int(e)])

    @disconnect_on_exception
    def SetBlending(self, p: float):
        """Set percentage of blending between consecutive movements in the same mode (velocity or cartesian).

        Note: There can't be blending between joint mode and Cartesian mode moves.

        Parameters
        ----------
        p : float
            Percentage blending between actions.

        """
        self._send_motion_command('SetBlending', [p])

    @disconnect_on_exception
    def SetCartAcc(self, p: float):
        """Set target acceleration (linear and angular) during MoveLin commands.

        Parameters
        ----------
        p : float
            Percentage of maximum acceleration.

        """
        self._send_motion_command('SetCartAcc', [p])

    @disconnect_on_exception
    def SetCartAngVel(self, w: float):
        """Set maximum angular velocity during MoveLin commands.

        Parameters
        ----------
        p : float
            Maximum angular velocity in deg/s.

        """
        self._send_motion_command('SetCartAngVel', [w])

    @disconnect_on_exception
    def SetCartLinVel(self, w: float):
        """Set maximum linear velocity during MoveLin commands.

        Note: Actual linear velocity may be lower if necessary to avoid exceeding maximum angular velocity.

        Parameters
        ----------
        p : float
            Maximum angular velocity in deg/s.

        """
        self._send_motion_command('SetCartLinVel', [w])

    @disconnect_on_exception
    def GripperOpen(self):
        """Open the gripper.

        """
        self._send_motion_command('GripperOpen')

    @disconnect_on_exception
    def GripperClose(self):
        """Close the gripper.

        """
        self._send_motion_command('GripperClose')

    @disconnect_on_exception
    def MoveGripper(self, state: bool = GRIPPER_OPEN):
        """Open or close the gripper.

        Corresponds to text API calls "_GripperOpen" / "GripperClose".

        Parameters
        ----------
        state : boolean
            Open or close the gripper (GRIPPER_OPEN or GRIPPER_CLOSE)

        """
        if state:
            self.GripperOpen()
        else:
            self.GripperClose()

    @disconnect_on_exception
    def SetGripperForce(self, p: float):
        """Set the gripper's force in percent.

        Parameters
        ----------
        p : float
            The desired force in percent.

        """
        self._send_motion_command('SetGripperForce', [p])

    @disconnect_on_exception
    def SetGripperVel(self, p: float):
        """Set the gripper's velocity in percent.

        Parameters
        ----------
        p : float
            The desired velocity in percent.

        """
        self._send_motion_command('SetGripperVel', [p])

    @disconnect_on_exception
    def SetJointAcc(self, p: float):
        """Set target joint acceleration during MoveJoints commands.

        Parameters
        ----------
        p : float
            Target acceleration, in percent.

        """
        self._send_motion_command('SetJointAcc', [p])

    @disconnect_on_exception
    def SetJointVel(self, p: float):
        """Set target joint velocity during MoveJoints commands.

        Parameters
        ----------
        p : float
            Target joint velocity, in percent.

        """
        self._send_motion_command('SetJointVel', [p])

    @disconnect_on_exception
    def SetTRF(self, x: float, y: float, z: float, alpha: float, beta: float, gamma: float):
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
    def SetWRF(self, x: float, y: float, z: float, alpha: float, beta: float, gamma: float):
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
    def SetCheckpoint(self, n: int):
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
            assert mx_def.MX_CHECKPOINT_ID_MIN <= n <= mx_def.MX_CHECKPOINT_ID_MAX
            return self._set_checkpoint_impl(n)

    @disconnect_on_exception
    def ExpectExternalCheckpoint(self, n: int):
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
            assert mx_def.MX_CHECKPOINT_ID_MIN <= n <= mx_def.MX_CHECKPOINT_ID_MAX
            return self._set_checkpoint_impl(n, send_to_robot=False)

    @disconnect_on_exception
    def WaitForAnyCheckpoint(self, timeout: float = None):
        """Pause program execution until any checkpoint has been received from the robot.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the checkpoint (in seconds).
        """
        with self._main_lock:
            self._check_internal_states()
            if '*' not in self._internal_checkpoints:
                self._internal_checkpoints['*'] = list()
            event = InterruptableEvent()
            self._internal_checkpoints['*'].append(event)

        event.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitConnected(self, timeout: float = None):
        """Pause program execution until robot is disconnected.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the event (in seconds).
        """
        self._robot_events.on_connected.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitDisconnected(self, timeout: float = None):
        """Pause program execution until the robot is disconnected.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the event (in seconds).
        """
        self._robot_events.on_disconnected.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitActivated(self, timeout: float = None):
        """Pause program execution until the robot is activated.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the event (in seconds).
        """
        self._robot_events.on_activated.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitDeactivated(self, timeout: float = None):
        """Pause program execution until the robot is deactivated.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the event (in seconds).
        """
        self._robot_events.on_deactivated.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitHomed(self, timeout: float = None):
        """Pause program execution until the robot is homed.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the event (in seconds).
        """
        self._robot_events.on_homed.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitSimActivated(self, timeout: float = None):
        """Pause program execution until the robot simulation mode is activated.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the event (in seconds).

        """
        self._robot_events.on_activate_sim.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitForError(self, timeout: float = None):
        """Pause program execution until the robot is in error state.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the event (in seconds).
        """
        self._robot_events.on_error.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitErrorReset(self, timeout: float = None):
        """Pause program execution until the robot is not in an error state.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the event (in seconds).
        """
        self._robot_events.on_error_reset.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitSimDeactivated(self, timeout: float = None):
        """Pause program execution until the robot simulation mode is deactivated.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the event (in seconds).
        """
        self._robot_events.on_deactivate_sim.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitMotionResumed(self, timeout: float = None):
        """Pause program execution until the robot motion is resumed.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the event (in seconds).
        """
        self._robot_events.on_motion_resumed.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitMotionPaused(self, timeout: float = None):
        """Pause program execution until the robot motion is paused.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the event (in seconds).
        """
        self._robot_events.on_motion_paused.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitMotionCleared(self, timeout: float = None):
        """Pause program execution until all pending request to clear motion have been acknowledged.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the event (in seconds).
        """

        self._robot_events.on_motion_cleared.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitEndOfCycle(self, timeout: float = None):
        """Pause program execution until all messages in a message cycle are received

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the event (in seconds).
        """
        if self._robot_events.on_end_of_cycle.is_set():
            self._robot_events.on_end_of_cycle.clear()

        self._robot_events.on_end_of_cycle.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitIdle(self, timeout: float = None):
        """Pause program execution until robot is idle.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the event (in seconds).
        """
        with self._main_lock:
            # Can't wait if robot is in error (already "idle")
            if self._robot_status.error_status:
                raise InterruptException('Robot is in error')
            checkpoint = self._set_checkpoint_internal()

        start_time = time.time()
        checkpoint.wait(timeout=timeout)
        end_time = time.time()

        if timeout:
            remaining_timeout = timeout - (end_time - start_time)
        else:
            remaining_timeout = None

        self._robot_events.on_end_of_block.wait(timeout=remaining_timeout)

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
    def Delay(self, t: float):
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
    def SendCustomCommand(self, command: str, expected_responses: bool = None):
        """Send custom command to robot.

        Parameters
        ----------
        command : str
            Desired custom command.

        expected_responses : None or list of integers.
            If not none, wait for and return one of the expected responses.

        Return
        ------
        If expected_responses is not None, return an event. The user can use
        event.wait() to wait for and get the response message.

        """
        return self._send_custom_command(command, expected_responses, skip_internal_check=True)

    @disconnect_on_exception
    def StartOfflineProgram(self, n: int, timeout: float = None):
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

    # Non-motion commands.

    @disconnect_on_exception
    def GetRtTargetJointPos(self,
                            include_timestamp: bool = False,
                            synchronous_update: bool = False,
                            timeout: float = None):
        """Returns the real-time target joint positions of the robot.

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
            if self._robot_info.rt_message_capable:
                self._send_sync_command('GetRtTargetJointPos', self._robot_events.on_joints_updated, timeout)
            else:
                # This robot does not have GetRtTargetJointPos, use legacy GetJoints (but won't get timestamp)
                self._send_sync_command('GetJoints', self._robot_events.on_joints_updated, timeout)

            # Wait until response is received (this will throw TimeoutException if appropriate)
            self._robot_events.on_joints_updated.wait(timeout=timeout)

        with self._main_lock:
            if include_timestamp:
                if not self._robot_info.rt_message_capable:
                    raise InvalidStateError('Cannot provide timestamp with current robot firmware or model.')
                else:
                    return copy.deepcopy(self._robot_rt_data.rt_target_joint_pos)

            return copy.deepcopy(self._robot_rt_data.rt_target_joint_pos.data)

    def GetJoints(self, synchronous_update: bool = False, timeout: float = None):
        """Legacy command. Please use GetRtTargetJointPos instead"""
        return self.GetRtTargetJointPos(include_timestamp=False, synchronous_update=synchronous_update, timeout=timeout)

    @disconnect_on_exception
    def GetRtTargetCartPos(self,
                           include_timestamp: bool = False,
                           synchronous_update: bool = False,
                           timeout: float = None):
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
            if self._robot_info.rt_message_capable:
                self._send_sync_command('GetRtTargetCartPos', self._robot_events.on_pose_updated, timeout)
            else:
                # This robot does not have GetRtTargetCartPos, use legacy GetPose (but won't get timestamp)
                self._send_sync_command('GetPose', self._robot_events.on_pose_updated, timeout)

        with self._main_lock:
            if include_timestamp:
                if not self._robot_info.rt_message_capable:
                    raise InvalidStateError('Cannot provide timestamp with current robot firmware or model.')
                else:
                    return copy.deepcopy(self._robot_rt_data.rt_target_cart_pos)

            return copy.deepcopy(self._robot_rt_data.rt_target_cart_pos.data)

    def GetPose(self, synchronous_update: bool = False, timeout: float = None):
        """Legacy command. Please use GetRtTargetCartPos instead"""
        return self.GetRtTargetCartPos(include_timestamp=False, synchronous_update=synchronous_update, timeout=timeout)

    @disconnect_on_exception
    def SetMonitoringInterval(self, t: float):
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
    def SetRealTimeMonitoring(self, *events: list):
        """Configure which real-time monitoring events to enable.

        Parameters
        ----------
        events : list of event IDs
            List of event IDs to enable. For instance: events=[MX_ST_RT_TARGET_JOINT_POS, MX_ST_RT_TARGET_CART_POS]
            enables the target joint positions and target end effector pose messages.
            Can also use events='all' to enable all.

        """
        with self._main_lock:
            self._check_internal_states()
            if isinstance(events, tuple):
                events = list(events)
            self._send_command('SetRealTimeMonitoring', events)

    @disconnect_on_exception
    def SetRTC(self, t: int):
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
    def ActivateBrakes(self, activated: bool = True):
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

    def GetRobotRtData(self):
        """Return a copy of the current robot real-time data, with all values associated with the same timestamp

        Return
        ------
        RobotRtData
            Object containing the current robot real-time data

        """
        with self._main_lock:
            return copy.deepcopy(self._robot_rt_data_stable)

    @disconnect_on_exception
    def GetStatusRobot(self, synchronous_update: bool = False, timeout: float = None):
        """Return a copy of the current robot status

        Parameters
        ----------
        synchronous_update: boolean
            True -> Synchronously get updated robot status. False -> Get latest known status.
        timeout: float
            Timeout (in seconds) waiting for synchronous response from the robot.

        Returns
        -------
        RobotStatus
            Object containing the current robot status

        """
        if synchronous_update:
            self._send_sync_command('GetStatusRobot', self._robot_events.on_status_updated, timeout)

        with self._main_lock:
            return copy.deepcopy(self._robot_status)

    @disconnect_on_exception
    def GetStatusGripper(self, synchronous_update: bool = True, timeout: float = None):
        """Return a copy of the current gripper status

        Parameters
        ----------
        synchronous_update: boolean
            True -> Synchronously get updated gripper status. False -> Get latest known status.
            *** Note: Synchronous mode by default because robot does not report gripper status change events by default
                      (unless SetStatusEvents command is used to enable gripper status updates)
        timeout: float
            Timeout (in seconds) waiting for synchronous response from the robot.

        Returns
        -------
        GripperStatus
            Object containing the current gripper status

        """
        if synchronous_update:
            self._send_sync_command('GetStatusGripper', self._robot_events.on_status_gripper_updated, timeout)

        with self._main_lock:
            return copy.deepcopy(self._gripper_status)

    def LogTrace(self, trace: str):
        """Send a text trace that is printed in the robot's log internal file (which can be retrieved from robot's Web
           portal under menu "Options -> Get Log").
           (useful for clearly identifying steps of a program within robot's log when reporting problems to
            Mecademic support team!)

        Parameters
        ----------
        trace : string
            Text string to print in robot's internal log file
        """
        # Escape any " in the provided string
        trace = trace.replace('"', '\"')
        self.SendCustomCommand(f"LogTrace({trace})")

    def StartLogging(self,
                     monitoringInterval: float,
                     file_name: str = None,
                     file_path: str = None,
                     fields: list = None,
                     record_time: bool = True):
        """Start logging robot state to file.

        Fields logged are controlled by SetRealtimeMonitoring(). Logging frequency is set by SetMonitoringInterval().
        By default, will wait until robot is idle before logging.

        Parameters
        ----------
        monitoring_interval: float
            Indicates rate at which state from robot will be received on monitor port. Unit: seconds

        file_name: string or None
            Log file name
            If None, file name will be built with date/time and robot information (robot type, serial, version).

        file_path : string or None
            Path to save the zip file that contains logged data.
            If not provided, file will be saved in working directory.

        fields : list of strings or None
            List of fields to log. Taken from RobotRtData attributes. None means log all compatible fields.

        record_time : bool
            If true, current date and time will be recorded in file.



        """
        if self._file_logger is not None:
            raise InvalidStateError('Another file logging operation is in progress.')

        self.SetMonitoringInterval(monitoringInterval)
        if self._robot_info.rt_message_capable:
            if fields is None:
                self.SetRealTimeMonitoring('all')
            else:
                self.SetRealTimeMonitoring(*fields)

            # Use a synchronous "GetRealTimeMonitoring" to ensure that we've started receiving data for all the requested
            # real-time monitoring fields we just enabled
            response = self._send_custom_command('GetRealTimeMonitoring',
                                                 expected_responses=[mx_def.MX_ST_GET_REAL_TIME_MONITORING],
                                                 skip_internal_check=True)
            response.wait(timeout=self.default_timeout)

        self._file_logger = _RobotTrajectoryLogger(self._robot_info,
                                                   self._robot_rt_data,
                                                   fields,
                                                   file_name=file_name,
                                                   file_path=file_path,
                                                   record_time=record_time,
                                                   monitoring_interval=monitoringInterval)

    def EndLogging(self):
        """Stop logging robot real-time data to file.

        """
        if self._file_logger is None:
            raise InvalidStateError('No existing logger to stop.')

        file_name = self._file_logger.end_log()
        self._file_logger = None

        return file_name

    @contextlib.contextmanager
    def FileLogger(self,
                   monitoringInterval: float,
                   file_name: str = None,
                   file_path: str = None,
                   fields: list = None,
                   record_time: bool = True):
        """Contextmanager interface for file logger.

        Parameters
        ----------
        monitoring_interval: float
            Indicates rate at which real-time data from robot will be received on monitor port. Unit: seconds

        file_name: string or None
            Log file name
            If None, file name will be built with date/time and robot information (robot type, serial, version).

        file_path : string or None
            Path to save the zip file that contains logged data.
            If not provided, file will be saved in working directory.

        fields : list of strings or None
            List of fields to log. Taken from RobotRtData attributes. None means log all compatible fields.

        record_time : bool
            If true, current date and time will be recorded in file.

        """
        self.StartLogging(
            monitoringInterval,
            file_name=file_name,
            file_path=file_path,
            fields=fields,
            record_time=record_time,
        )
        try:
            yield
        finally:
            self.EndLogging()

    #####################################################################################
    # Private methods.
    #####################################################################################

    def _check_monitor_threads(self):
        """Check that the threads which handle robot monitor messages are alive.

        Attempt to disconnect from the robot if not.

        """
        if self._robot_info.rt_on_ctrl_port_capable:
            # We're not using monitoring port. No need to check here.
            return

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

    def _send_command(self, command: str, arg_list: list = None):
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

    def _send_sync_command(self, command: str, event: InterruptableEvent, timeout: float):
        """Send a command and wait for corresponding response
           (this function handles well-known commands which have their corresponding 'wait' event in this class,
            use _send_custom_command to perform synchronous operations on other commands)

        Parameters
        ----------
        command : string
            Name of the command to send (example: GetStatusGripper)
        event : InterruptableEvent
            Event that will be set (unblocked) once the corresponding response is received
        timeout : float
            Maximum time to wait for the event to be set (i.e. response received)


        Raises
        ------
        TimeoutException
            If response was not received before timeout
        """
        with self._main_lock:
            self._check_internal_states()
            if self._robot_info.rt_on_ctrl_port_capable:
                # Send a "sync" request so we know when we get the response to this get (and not an earlier one)
                self._tx_sync += 1
                self._send_command(f'Sync({self._tx_sync})')
            if event.is_set():
                event.clear()
                self._send_command(command)

        # Wait until response is received (this will throw TimeoutException if appropriate)
        event.wait(timeout=timeout)

    def _send_custom_command(self,
                             command: str,
                             expected_responses: list[int] = None,
                             skip_internal_check: bool = False):
        """Internal version of SendCustomCommand with option to skip internal state check (so it can be used
           during connetion)
        """
        with self._main_lock:
            if not skip_internal_check:
                self._check_internal_states()

            if expected_responses:
                event_with_data = InterruptableEvent(data=expected_responses)
                self._custom_response_events.append(event_with_data)

            self._send_command(command)

        if expected_responses:
            return event_with_data

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
        thread.setDaemon(True)  # Make sure thread does not prevent application from quitting
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
            self._command_socket = self._connect_socket(self.logger, self._address, mx_def.MX_ROBOT_TCP_PORT_CONTROL)

            if self._command_socket is None:
                raise CommunicationError('Command socket could not be created. Is the IP address correct?')

            # Create rx thread for command socket communication.
            self._command_rx_thread = self._launch_thread(target=self._handle_socket_rx,
                                                          args=(
                                                              self._command_socket,
                                                              self._command_rx_queue,
                                                              self.logger,
                                                          ))

            # Create tx thread for command socket communication.
            self._command_tx_thread = self._launch_thread(target=self._handle_socket_tx,
                                                          args=(
                                                              self._command_socket,
                                                              self._command_tx_queue,
                                                              self.logger,
                                                          ))

        except Exception as exception:
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
            self._monitor_socket = self._connect_socket(self.logger, self._address, mx_def.MX_ROBOT_TCP_PORT_FEED)

            if self._monitor_socket is None:
                raise CommunicationError('Monitor socket could not be created. Is the IP address correct?')

            # Create rx thread for monitor socket communication.
            self._monitor_rx_thread = self._launch_thread(target=self._handle_socket_rx,
                                                          args=(
                                                              self._monitor_socket,
                                                              self._monitor_rx_queue,
                                                              self.logger,
                                                          ))

        except Exception as exception:
            # Clean up threads and connections on error.
            self.Disconnect()
            raise

    def _receive_welcome_message(self, message_queue, from_command_port):
        """Receive and parse a welcome message in order to set _robot_info and _robot_rt_data.

        Parameters
        ----------
        message_queue : queue
            The welcome message will be fetched from this queue.
        """
        response = _Message(None, None)

        start = time.time()
        while response.id != mx_def.MX_ST_CONNECTED:
            try:
                response = message_queue.get(block=True, timeout=self.default_timeout)
            except queue.Empty:
                self.logger.error('No response received within timeout interval.')
                self.Disconnect()
                raise CommunicationError('No response received within timeout interval.')
            except BaseException:
                self.Disconnect()
                raise

            if from_command_port:
                break

            if (time.time() - start) > self.default_timeout:
                self.logger.error('No connect message received within timeout interval.')
                break

        if response.id != mx_def.MX_ST_CONNECTED:
            self.logger.error('Connection error: {}'.format(response))
            self.Disconnect()
            raise CommunicationError('Connection error: {}'.format(response))

        # Attempt to parse robot return data.
        self._robot_info = RobotInfo.from_command_response_string(response.data)

        self._robot_rt_data = RobotRtData(self._robot_info.num_joints)
        self._robot_rt_data_stable = RobotRtData(self._robot_info.num_joints)

    def _initialize_command_connection(self):
        """Attempt to connect to the command port of the Mecademic Robot.

        """
        self._receive_welcome_message(self._command_rx_queue, True)

        self._command_response_handler_thread = self._launch_thread(target=self._command_response_handler, args=())

    def _initialize_monitoring_connection(self):
        """Attempt to connect to the monitor port of the Mecademic Robot.

        Returns
        -------
        status : boolean
            Returns the status of the connection, true for success, false for failure.

        """

        if self._monitor_mode:
            self._receive_welcome_message(self._monitor_rx_queue, False)

        self._monitor_handler_thread = self._launch_thread(target=self._monitor_handler, args=())

        return

    def _shut_down_queue_threads(self):
        """Attempt to gracefully shut down threads which read from queues.

        """
        # Join threads which wait on a queue by sending terminate to the queue.
        # Don't acquire _main_lock since these threads require _main_lock to finish processing.
        if self._command_tx_thread is not None:
            try:
                self._command_tx_queue.put(_TERMINATE)
            except Exception as e:
                self.logger.error('Error shutting down tx thread. ' + str(e))
            self._command_tx_thread.join(timeout=self.default_timeout)
            self._command_tx_thread = None

        if self._command_response_handler_thread is not None:
            try:
                self._command_rx_queue.put(_TERMINATE)
            except Exception as e:
                self.logger.error('Error shutting down command response handler thread. ' + str(e))
            self._command_response_handler_thread.join(timeout=self.default_timeout)
            self._command_response_handler_thread = None

        if self._monitor_handler_thread is not None:
            try:
                self._monitor_rx_queue.put(_TERMINATE)
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
            if self._internal_checkpoint_counter > _CHECKPOINT_ID_MAX_PRIVATE:
                self._internal_checkpoint_counter = mx_def.MX_CHECKPOINT_ID_MAX + 1

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
            if mx_def.MX_CHECKPOINT_ID_MIN <= n <= mx_def.MX_CHECKPOINT_ID_MAX:
                checkpoints_dict = self._user_checkpoints
            elif mx_def.MX_CHECKPOINT_ID_MAX < n <= _CHECKPOINT_ID_MAX_PRIVATE:
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

    def _invalidate_checkpoints(self, message=""):
        '''Unblock all waiting checkpoints and have them throw InterruptException.

        '''

        for checkpoints_dict in [self._internal_checkpoints, self._user_checkpoints]:
            for key, checkpoints_list in checkpoints_dict.items():
                for event in checkpoints_list:
                    event.abort(message)
            checkpoints_dict.clear()

        self._internal_checkpoint_counter = mx_def.MX_CHECKPOINT_ID_MAX + 1

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

        while True:
            # Wait for a message in the queue.
            response = self._monitor_rx_queue.get(block=True)

            # Terminate thread if requested.
            if response == _TERMINATE:
                return

            self._callback_queue.put('on_monitor_message', response)

            queue_size = self._monitor_rx_queue.qsize()
            if queue_size > self._robot_rt_data.max_queue_size:
                self._robot_rt_data.max_queue_size = queue_size

            with self._main_lock:

                self._handle_common_messages(response)

                # On non-rt monitoring capable platforms, no CYCLE_END event is sent, so use system time.
                # GET_JOINTS and GET_POSE is still sent every cycle, so log RobotRtData upon GET_POSE.
                if response.id == mx_def.MX_ST_GET_POSE and not self._robot_info.rt_message_capable:
                    # On non rt_monitoring platforms, we will consider this moment to be the end of cycle
                    self._robot_events.on_end_of_cycle.set()
                    self._callback_queue.put('on_end_of_cycle')

                    if self._file_logger:
                        # Log time in microseconds to be consistent with real-time logging timestamp.
                        self._file_logger.write_fields(time.time_ns() / 1000, self._robot_rt_data)
                    self._make_stable_rt_data()

    def _make_stable_rt_data(self):
        """We have to create stable copy of rt_data, with consistent timestamp values for all attributes.
        This consistent copy is used by GetRobotRtData()
        """

        self._robot_rt_data_stable = copy.deepcopy(self._robot_rt_data)

        # Make sure not to report values that are not enabled in real-time monitoring
        self._robot_rt_data_stable._clear_if_disabled()

    def _command_response_handler(self):
        """Handle received messages on the command socket.

        """
        while True:
            # Wait for a response to be available from the queue.
            response = self._command_rx_queue.get(block=True)

            # Terminate thread if requested.
            if response == _TERMINATE:
                return

            self._callback_queue.put('on_command_message', response)

            with self._main_lock:

                # Find and handle custom response event.
                matched_events = (event for event in self._custom_response_events if response.id in event.data)
                for event in matched_events:
                    event.set(data=response)
                    self._custom_response_events.remove(event)

                if response.id == mx_def.MX_ST_CHECKPOINT_REACHED:
                    self._handle_checkpoint_response(response)

                elif response.id == mx_def.MX_ST_CLEAR_MOTION:
                    if self._clear_motion_requests <= 1:
                        self._clear_motion_requests = 0
                        self._robot_events.on_motion_cleared.set()
                        self._callback_queue.put('on_motion_cleared')
                    else:
                        self._clear_motion_requests -= 1

                elif response.id == mx_def.MX_ST_PSTOP:
                    if bool(int(response.data)):
                        self._robot_events.on_p_stop_reset.clear()
                        self._robot_events.on_p_stop.set()
                        self._callback_queue.put('on_p_stop')
                    else:
                        self._robot_events.on_p_stop.clear()
                        self._robot_events.on_p_stop_reset.set()
                        self._callback_queue.put('on_p_stop_reset')

                elif response.id == mx_def.MX_ST_BRAKES_ON:
                    self._robot_events.on_brakes_deactivated.clear()
                    self._robot_events.on_brakes_activated.set()

                elif response.id == mx_def.MX_ST_BRAKES_OFF:
                    self._robot_events.on_brakes_activated.clear()
                    self._robot_events.on_brakes_deactivated.set()

                elif response.id == mx_def.MX_ST_OFFLINE_START:
                    self._robot_events.on_offline_program_started.set()
                    self._callback_queue.put('on_offline_program_state')

                elif response.id == mx_def.MX_ST_NO_OFFLINE_SAVED:
                    self._robot_events.on_offline_program_started.abort("specified offline program id does not exist")

                else:
                    self._handle_common_messages(response)

    def _handle_common_messages(self, response):
        """Handle response messages which are received on the command and monitor port, and are processed the same way.

        Parameters
        ----------
        response : Message object
            Robot status response to parse and handle.

        """

        # Print error trace if this is an error code
        if response.id in mx_def.robot_status_code_info:
            code_info = mx_def.robot_status_code_info[response.id]
            if code_info.is_error:
                self.logger.error(f'Received robot error {code_info.code} ({code_info.name})')
        else:
            self.logger.debug(f'Received unknown robot status code {response.id}')

        #
        # Only update using legacy messages if robot is not capable of rt messages.
        #
        if self._robot_info.rt_message_capable:
            # Temporarily save data if rt messages will be available to add timestamps.
            # Note that if robot platform isn't RT message capable, the update occurs in _handle_common_messages.
            if response.id == mx_def.MX_ST_GET_JOINTS:
                self._tmp_rt_joint_pos = _string_to_numbers(response.data)
            elif response.id == mx_def.MX_ST_GET_POSE:
                self._tmp_rt_cart_pos = _string_to_numbers(response.data)
            elif response.id == mx_def.MX_ST_RT_CYCLE_END:
                if not self._robot_info.rt_message_capable:
                    self._robot_info.rt_message_capable = True
                timestamp = int(response.data)

                # Useful to detect end of cycle for logging, to start logging on more consistent moment
                self._robot_events.on_end_of_cycle.set()
                self._callback_queue.put('on_end_of_cycle')

                # Update joint and pose with legacy messages from current cycle plus the timestamps we just received
                if self._tmp_rt_joint_pos:
                    self._robot_rt_data.rt_target_joint_pos.update_from_data(timestamp, self._tmp_rt_joint_pos)
                    self._robot_rt_data.rt_target_joint_pos.enabled = True
                    self._tmp_rt_joint_pos = None
                if self._tmp_rt_cart_pos:
                    self._robot_rt_data.rt_target_cart_pos.update_from_data(timestamp, self._tmp_rt_cart_pos)
                    self._robot_rt_data.rt_target_cart_pos.enabled = True
                    self._tmp_rt_cart_pos = None

                # If logging is active, log the current state.
                if self._file_logger:
                    self._file_logger.write_fields(timestamp, self._robot_rt_data)
                self._make_stable_rt_data()
        else:  # not self._robot_info.rt_message_capable
            if response.id == mx_def.MX_ST_GET_JOINTS:
                self._robot_rt_data.rt_target_joint_pos.data = _string_to_numbers(response.data)
                self._robot_rt_data.rt_target_joint_pos.enabled = True
                if self._is_in_sync():
                    self._robot_events.on_joints_updated.set()

            elif response.id == mx_def.MX_ST_GET_POSE:
                self._robot_rt_data.rt_target_cart_pos.data = _string_to_numbers(response.data)
                self._robot_rt_data.rt_target_cart_pos.enabled = True
                if self._is_in_sync():
                    self._robot_events.on_pose_updated.set()

            elif response.id == mx_def.MX_ST_GET_CONF:
                self._robot_rt_data.rt_target_conf.data = _string_to_numbers(response.data)
                self._robot_rt_data.rt_target_conf.enabled = True

            elif response.id == mx_def.MX_ST_GET_CONF_TURN:
                self._robot_rt_data.rt_target_conf_turn.data = _string_to_numbers(response.data)
                self._robot_rt_data.rt_target_conf_turn.enabled = True

        #
        # Handle various responses/events that we're interested into
        #
        if response.id == mx_def.MX_ST_GET_STATUS_ROBOT:
            self._handle_robot_status_response(response)

        elif response.id == mx_def.MX_ST_GET_STATUS_GRIPPER:
            self._handle_gripper_status_response(response)

        elif response.id == mx_def.MX_ST_RT_TARGET_JOINT_POS:
            self._robot_rt_data.rt_target_joint_pos.update_from_csv(response.data)
            if self._is_in_sync():
                self._robot_events.on_joints_updated.set()

        elif response.id == mx_def.MX_ST_RT_TARGET_CART_POS:
            self._robot_rt_data.rt_target_cart_pos.update_from_csv(response.data)
            if self._is_in_sync():
                self._robot_events.on_pose_updated.set()

        elif response.id == mx_def.MX_ST_RT_TARGET_JOINT_VEL:
            self._robot_rt_data.rt_target_joint_vel.update_from_csv(response.data)
        elif response.id == mx_def.MX_ST_RT_TARGET_JOINT_TORQ:
            self._robot_rt_data.rt_target_joint_torq.update_from_csv(response.data)
        elif response.id == mx_def.MX_ST_RT_TARGET_CART_VEL:
            self._robot_rt_data.rt_target_cart_vel.update_from_csv(response.data)

        elif response.id == mx_def.MX_ST_RT_TARGET_CONF:
            self._robot_rt_data.rt_target_conf.update_from_csv(response.data)
        elif response.id == mx_def.MX_ST_RT_TARGET_CONF_TURN:
            self._robot_rt_data.rt_target_conf_turn.update_from_csv(response.data)

        elif response.id == mx_def.MX_ST_RT_JOINT_POS:
            self._robot_rt_data.rt_joint_pos.update_from_csv(response.data)
        elif response.id == mx_def.MX_ST_RT_CART_POS:
            self._robot_rt_data.rt_cart_pos.update_from_csv(response.data)
        elif response.id == mx_def.MX_ST_RT_JOINT_VEL:
            self._robot_rt_data.rt_joint_vel.update_from_csv(response.data)
        elif response.id == mx_def.MX_ST_RT_JOINT_TORQ:
            self._robot_rt_data.rt_joint_torq.update_from_csv(response.data)
        elif response.id == mx_def.MX_ST_RT_CART_VEL:
            self._robot_rt_data.rt_cart_vel.update_from_csv(response.data)

        elif response.id == mx_def.MX_ST_RT_CONF:
            self._robot_rt_data.rt_conf.update_from_csv(response.data)
        elif response.id == mx_def.MX_ST_RT_CONF_TURN:
            self._robot_rt_data.rt_conf_turn.update_from_csv(response.data)

        elif response.id == mx_def.MX_ST_RT_ACCELEROMETER:
            # The data is stored as [timestamp, index, {measurements...}]
            timestamp, index, *measurements = _string_to_numbers(response.data)
            # Record accelerometer measurement only if newer.
            if index not in self._robot_rt_data.rt_accelerometer:
                self._robot_rt_data.rt_accelerometer[index] = TimestampedData(timestamp, measurements)
                self._robot_rt_data.rt_accelerometer[index].enabled = True
            if timestamp > self._robot_rt_data.rt_accelerometer[index].timestamp:
                self._robot_rt_data.rt_accelerometer[index].timestamp = timestamp
                self._robot_rt_data.rt_accelerometer[index].data = measurements

        elif response.id == mx_def.MX_ST_RT_WRF:
            self._robot_rt_data.rt_wrf.update_from_csv(response.data)

        elif response.id == mx_def.MX_ST_RT_TRF:
            self._robot_rt_data.rt_wrf.update_from_csv(response.data)

        elif response.id == mx_def.MX_ST_RT_CHECKPOINT:
            self._robot_rt_data.rt_checkpoint.update_from_csv(response.data)

        elif response.id == mx_def.MX_ST_IMPOSSIBLE_RESET_ERR:
            message = "Robot indicated that this error cannot be reset"
            self.logger.error(message)
            self._robot_events.on_error_reset.abort(message)

        elif response.id == mx_def.MX_ST_GET_REAL_TIME_MONITORING:
            self._handle_get_realtime_monitoring_response(response)

        elif response.id == mx_def.MX_ST_SYNC:
            self._handle_sync_response(response)

    def _parse_response_bool(self, response):
        """ Parse standard robot response, returns array of boolean values
        """
        if response.data.strip == '':
            return []
        else:
            return [bool(int(x)) for x in response.data.split(',')]

    def _parse_response_int(self, response):
        """ Parse standard robot response, returns array of integer values
        """
        if response.data.strip() == '':
            return []
        else:
            return [int(x) for x in response.data.split(',')]

    def _handle_robot_status_response(self, response):
        """Parse robot status response and update status fields and events.

        Parameters
        ----------
        response : Message object
            Robot status response to parse and handle.

        """
        assert response.id == mx_def.MX_ST_GET_STATUS_ROBOT
        status_flags = self._parse_response_bool(response)

        if self._robot_status.activation_state != status_flags[0]:
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
            self._robot_status.activation_state = status_flags[0]

        if self._robot_status.homing_state != status_flags[1]:
            if status_flags[1]:
                self._robot_events.on_homed.set()
                self._callback_queue.put('on_homed')
            else:
                self._robot_events.on_homed.clear()
            self._robot_status.homing_state = status_flags[1]

        if self._robot_status.simulation_mode != status_flags[2]:
            if status_flags[2]:
                self._robot_events.on_deactivate_sim.clear()
                self._robot_events.on_activate_sim.set()
                self._callback_queue.put('on_activate_sim')
            else:
                self._robot_events.on_activate_sim.clear()
                self._robot_events.on_deactivate_sim.set()
                self._callback_queue.put('on_deactivate_sim')
            self._robot_status.simulation_mode = status_flags[2]

        if self._robot_status.error_status != status_flags[3]:
            if status_flags[3]:
                message = "robot is in error"
                self._invalidate_checkpoints(message)
                self._robot_events.on_error.set()
                self._robot_events.abort_all_on_error(message)
                self._robot_events.on_error_reset.clear()
                self._callback_queue.put('on_error')
            else:
                self._robot_events.clear_abort_all()
                self._robot_events.on_error.clear()
                self._robot_events.on_error_reset.set()
                self._callback_queue.put('on_error_reset')
            self._robot_status.error_status = status_flags[3]

        if self._robot_status.pause_motion_status != status_flags[4]:
            if status_flags[4]:
                self._robot_events.on_motion_resumed.clear()
                self._robot_events.on_motion_paused.set()
                self._callback_queue.put('on_motion_paused')
            else:
                self._robot_events.on_motion_paused.clear()
                self._robot_events.on_motion_resumed.set()
                self._callback_queue.put('on_motion_resumed')
            self._robot_status.pause_motion_status = status_flags[4]

        if self._robot_status.end_of_block_status != status_flags[5]:
            if status_flags[5]:
                self._robot_events.on_end_of_block.set()
            else:
                self._robot_events.on_end_of_block.clear()
            self._robot_status.end_of_block_status = status_flags[5]

        if self._is_in_sync():
            self._robot_events.on_status_updated.set()
        self._callback_queue.put('on_status_updated')

    def _handle_gripper_status_response(self, response):
        """Parse gripper status response and update status fields and events.

        Parameters
        ----------
        response : Message object
            Gripper status response to parse and handle.

        """
        assert response.id == mx_def.MX_ST_GET_STATUS_GRIPPER
        status_flags = self._parse_response_bool(response)

        self._gripper_status.present = status_flags[0]
        self._gripper_status.homing_state = status_flags[1]
        self._gripper_status.holding_part = status_flags[2]
        self._gripper_status.limit_reached = status_flags[3]
        self._gripper_status.error_status = status_flags[4]
        self._gripper_status.overload_error = status_flags[5]

        if self._is_in_sync():
            self._robot_events.on_status_gripper_updated.set()
            self._callback_queue.put('on_status_gripper_updated')

    def _handle_checkpoint_response(self, response):
        """Handle the checkpoint message from the robot, set the appropriate events, etc.

        Parameters
        ----------
        response : Message object
            Response message which includes the received checkpoint id.

        """
        assert response.id == mx_def.MX_ST_CHECKPOINT_REACHED
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
            # Enqueue the on_checkpoint_reached callback.
            self._callback_queue.put('on_checkpoint_reached', checkpoint_id)

        # Check internal checkpoints.
        elif checkpoint_id in self._internal_checkpoints and self._internal_checkpoints[checkpoint_id]:
            self._internal_checkpoints[checkpoint_id].pop(0).set()
            # If list corresponding to checkpoint id is empty, remove the key from the dict.
            if not self._internal_checkpoints[checkpoint_id]:
                self._internal_checkpoints.pop(checkpoint_id)
        else:
            self.logger.warning('Received un-tracked checkpoint. Please use ExpectExternalCheckpoint() to track.')

    def _handle_get_realtime_monitoring_response(self, response):
        """Parse robot response to "get" or "set" real-time monitoring.
           This function identifies which real-time events are expected, and which are not enabled.

        Parameters
        ----------
        response : Message object
            Robot status response to parse and handle.

        """
        assert response.id == mx_def.MX_ST_GET_REAL_TIME_MONITORING

        # Clear all "enabled" bits in real-time data
        self._robot_rt_data._reset_enabled()

        # Parse the response to identify which are "enabled"
        enabled_event_ids = self._parse_response_int(response)
        for event_id in enabled_event_ids:
            if event_id == mx_def.MX_ST_RT_TARGET_JOINT_POS:
                self._robot_rt_data.rt_target_joint_pos.enabled = True
            if event_id == mx_def.MX_ST_RT_TARGET_CART_POS:
                self._robot_rt_data.rt_target_cart_pos.enabled = True
            if event_id == mx_def.MX_ST_RT_TARGET_JOINT_VEL:
                self._robot_rt_data.rt_target_joint_vel.enabled = True
            if event_id == mx_def.MX_ST_RT_TARGET_JOINT_TORQ:
                self._robot_rt_data.rt_target_joint_torq.enabled = True
            if event_id == mx_def.MX_ST_RT_TARGET_CART_VEL:
                self._robot_rt_data.rt_target_cart_vel.enabled = True
            if event_id == mx_def.MX_ST_RT_TARGET_CONF:
                self._robot_rt_data.rt_target_conf.enabled = True
            if event_id == mx_def.MX_ST_RT_TARGET_CONF_TURN:
                self._robot_rt_data.rt_target_conf_turn.enabled = True
            if event_id == mx_def.MX_ST_RT_JOINT_POS:
                self._robot_rt_data.rt_joint_pos.enabled = True
            if event_id == mx_def.MX_ST_RT_CART_POS:
                self._robot_rt_data.rt_cart_pos.enabled = True
            if event_id == mx_def.MX_ST_RT_JOINT_VEL:
                self._robot_rt_data.rt_joint_vel.enabled = True
            if event_id == mx_def.MX_ST_RT_JOINT_TORQ:
                self._robot_rt_data.rt_joint_torq.enabled = True
            if event_id == mx_def.MX_ST_RT_CART_VEL:
                self._robot_rt_data.rt_cart_vel.enabled = True
            if event_id == mx_def.MX_ST_RT_CONF:
                self._robot_rt_data.rt_conf.enabled = True
            if event_id == mx_def.MX_ST_RT_CONF_TURN:
                self._robot_rt_data.rt_conf_turn.enabled = True
            if event_id == mx_def.MX_ST_RT_WRF:
                self._robot_rt_data.rt_wrf.enabled = True
            if event_id == mx_def.MX_ST_RT_TRF:
                self._robot_rt_data.rt_trf.enabled = True
            if event_id == mx_def.MX_ST_RT_CHECKPOINT:
                self._robot_rt_data.rt_checkpoint.enabled = True
            if event_id == mx_def.MX_ST_RT_WRF:
                self._robot_rt_data.rt_wrf.enabled = True
            if event_id == mx_def.MX_ST_RT_ACCELEROMETER:
                for accelerometer in self._robot_rt_data.rt_accelerometer.values():
                    accelerometer.enabled = True

        # Make sure to clear values that we should no more received
        self._robot_rt_data._clear_if_disabled()

    def _handle_sync_response(self, response):
        """Parse robot response to "Sync" request
           This class uses the "Sync" request/response to ensure synchronous "Get" operations have received the
           expected response from the robot (and not a response/event sent by the robot prior to our "Get" request).

        Parameters
        ----------
        response : Message object
            Sync response to parse and handle.

        """
        assert response.id == mx_def.MX_ST_SYNC

        self._rx_sync = _string_to_numbers(response.data)[0]

    def _is_in_sync(self):
        """Tells if we're in sync with the latest "get" operation (i.e. we've received the response to the most recent
           "Sync" request to the robot, meaning that the "get" response we just got is up-to-date)

        Returns
        -------
        boolean
            True if "in sync" ("get" response we just received matches the "get" request we've just made)
        """
        return (self._rx_sync == self._tx_sync)


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
        self.enabled = False

    def clear_if_disabled(self):
        """Clear timestamp and data if not reported by the robot (not part of enabled real-time monitoring events)
        """
        if not self.enabled:
            self.timestamp = 0
            self.data = TimestampedData.zeros(len(self.data)).data

    def update_from_csv(self, input_string):
        """Update from comma-separated string, only if timestamp is newer.

        Parameters
        ----------
        input_string : string
            Comma-separated string. First value is timestamp, rest is data.

        """
        numbs = _string_to_numbers(input_string)

        if (len(numbs) - 1) != len(self.data):
            raise ValueError('Cannot update TimestampedData with incompatible data.')

        if numbs[0] > self.timestamp:
            self.timestamp = numbs[0]
            self.data = numbs[1:]

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


class RobotRtData:
    """Class for storing the internal real-time data of a Mecademic robot.

    Note that the frequency and availability of real-time data depends on the monitoring interval and which monitoring
    events are enabled. Monitoring events can be configured using SetMonitoringInterval() and SetRealTimeMonitoring().

    Attributes
    ----------
    rt_target_joint_pos : TimestampedData
        Controller desired joint positions in degrees [theta_1...6], includes timestamp.
    rt_target_cart_pos : TimestampedData
        Controller desired end effector pose [x, y, z, alpha, beta, gamma], includes timestamp.
    rt_target_joint_vel : TimestampedData
        Controller desired joint velocity in degrees/second [theta_dot_1...6], includes timestamp.
    rt_target_cart_vel : TimestampedData
        Controller desired end effector velocity with timestamp. Linear values in mm/s, angular in deg/s.
        [linear_velocity_vector x, y, z, angular_velocity_vector omega-x, omega-y, omega-z]
    rt_target_conf : TimestampedData
        Controller joint configuration that corresponds to desired joint positions.
    rt_target_conf_turn : TimestampedData
        Controller last joint turn number that corresponds to desired joint positions.

    rt_joint_pos : TimestampedData
        Drive-measured joint positions in degrees [theta_1...6], includes timestamp.
    rt_cart_pos : TimestampedData
        Drive-measured end effector pose [x, y, z, alpha, beta, gamma], includes timestamp.
    rt_joint_vel : TimestampedData
        Drive-measured joint velocity in degrees/second [theta_dot_1...6], includes timestamp.
    rt_joint_torq : TimestampedData
        Drive-measured torque ratio as a percent of maximum [torque_1...6], includes timestamp.
    rt_cart_vel : TimestampedData
        Drive-measured end effector velocity with timestamp. Linear values in mm/s, angular in deg/s.
        [linear_velocity_vector x, y, z, angular_velocity_vector omega-x, omega-y, omega-z]
    rt_conf : TimestampedData
        Controller joint configuration that corresponds to drives-measured joint positions.
    rt_conf_turn : TimestampedData
        Controller last joint turn number that corresponds to drives-measured joint positions.

    rt_accelerometer : TimestampedData
        Raw accelerometer measurements [accelerometer_id, x, y, z]. 16000 = 1g.
"""

    def __init__(self, num_joints):
        self.rt_target_joint_pos = TimestampedData.zeros(num_joints)  # microseconds timestamp, degrees
        self.rt_target_cart_pos = TimestampedData.zeros(6)  # microseconds timestamp, mm and degrees
        self.rt_target_joint_vel = TimestampedData.zeros(num_joints)  # microseconds timestamp, degrees/second
        self.rt_target_joint_torq = TimestampedData.zeros(num_joints)  # microseconds timestamp, percent of maximum
        self.rt_target_cart_vel = TimestampedData.zeros(6)  # microseconds timestamp, mm/s and deg/s
        self.rt_target_conf = TimestampedData.zeros(3)
        self.rt_target_conf_turn = TimestampedData.zeros(1)

        self.rt_joint_pos = TimestampedData.zeros(num_joints)  # microseconds timestamp, degrees
        self.rt_cart_pos = TimestampedData.zeros(6)  # microseconds timestamp, mm and degrees
        self.rt_joint_vel = TimestampedData.zeros(num_joints)  # microseconds timestamp, degrees/second
        self.rt_joint_torq = TimestampedData.zeros(num_joints)  # microseconds timestamp, percent of maximum
        self.rt_cart_vel = TimestampedData.zeros(6)  # microseconds timestamp, mm/s and deg/s
        self.rt_conf = TimestampedData.zeros(3)
        self.rt_conf_turn = TimestampedData.zeros(1)

        # Contains dictionary of accelerometers stored in the robot indexed by joint number.
        # For example, Meca500 currently only reports the accelerometer in joint 5.
        self.rt_accelerometer = dict()  # 16000 = 1g

        self.rt_wrf = TimestampedData.zeros(6)  # microseconds timestamp, mm and degrees
        self.rt_trf = TimestampedData.zeros(6)  # microseconds timestamp, mm and degrees
        self.rt_checkpoint = TimestampedData.zeros(1)  # microseconds timestamp, checkpointId

        self.max_queue_size = 0

    def _for_each_rt_data(self):
        """Iterates for each TimestampedData type member of this class (rt_joint_pos, rt_cart_pos, etc.)
        """
        # Collect class member names
        member_names = vars(self)
        # Iterate though all "rt_" members
        for member_name in member_names:
            if not member_name.startswith('rt_'):
                continue
            member = getattr(self, member_name)
            # Check member type (TimestampedData or dict of TimestampedData), then yield TimestampedData
            if isinstance(member, TimestampedData):
                yield member
            elif isinstance(member, dict):
                for sub_member in member.values():
                    yield sub_member

    def _reset_enabled(self):
        """Clear the "enabled" flag of each member of this class of type TimestampedData
        """
        for rt_data in self._for_each_rt_data():
            rt_data.enabled = False

    def _clear_if_disabled(self):
        """Clear real-time values that are disabled (not reported by robot's current real-time monitoring configuration)
        """
        for rt_data in self._for_each_rt_data():
            rt_data.clear_if_disabled()


class RobotStatus:
    """Class for storing the status of a Mecademic robot.

    Attributes
    ----------
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
"""

    def __init__(self):

        # The following are status fields.
        self.activation_state = False
        self.homing_state = False
        self.simulation_mode = False
        self.error_status = False
        self.pause_motion_status = False
        self.end_of_block_status = False


class GripperStatus:
    """Class for storing the Mecademic robot's gripper status.

    Attributes
    ----------
    present : boolean
        True if the gripper is present on the robot.
    homing_state : boolean
        True if the robot is homed.
    homing_state : boolean
        True if the gripper has been homed (ready to be used).
    holding_part : boolean
        True if the gripper is currently holding a part.
    limit_reached : boolean
        True if the gripper is at a limit (fully opened or closed).
    overload_error : boolean
        True if the gripper is in overload error state.
"""

    def __init__(self):

        # The following are status fields.
        self.present = False
        self.homing_state = False
        self.holding_part = False
        self.limit_reached = False
        self.error_status = False
        self.overload_error = False


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
    serial : string
        Serial identifier of robot.
    rt_message_capable : bool
        True if robot is capable of sending real-time monitoring messages.
    rt_on_ctrl_port_capable : bool
        True if robot is capable of sending real-time monitoring messages on control port (SetCtrlPortMonitoring)
    num_joints : int
        Number of joints on the robot.
"""

    def __init__(self,
                 model=None,
                 revision=None,
                 is_virtual=None,
                 fw_major_rev=None,
                 fw_minor_rev=None,
                 fw_patch_num=None,
                 serial=None):
        self.model = model
        self.revision = revision
        self.is_virtual = is_virtual
        self.fw_major_rev = fw_major_rev
        self.fw_minor_rev = fw_minor_rev
        self.fw_patch_num = fw_patch_num
        self.serial = serial
        self.rt_message_capable = False
        self.rt_on_ctrl_port_capable = False

        if self.model == 'Meca500':
            self.num_joints = 6
        elif self.model == 'Scara':
            self.num_joints = 4
        elif self.model is None:
            self.num_joints = 1
        else:
            raise ValueError('Invalid robot model: {}'.format(self.model))

        # Check if this robot supports real-time monitoring events
        if self.is_at_least(8, 4):
            self.rt_message_capable = True
        # Check if this robot supports real-time monitoring on control port (8.4.3+)
        if self.is_at_least(8, 4, 3):
            self.rt_on_ctrl_port_capable = True

    def is_at_least(self, major, minor=0, patch=0):
        """Tells if this RobotInfo instance's version is at least the specified version

        Parameters
        ----------
        major : integer
            Minimum desired major version
        minor : integer
            Minimum desired minor version
        patch : integer
            Minimum desired patch version

        Returns
        -------
        boolean
            True if this RobotInfo instance's version is at least the specified version
        """
        # Check major
        if self.fw_major_rev > major:
            return True
        elif self.fw_major_rev < major:
            return False

        # Same major, check minor
        if self.fw_minor_rev > minor:
            return True
        elif self.fw_minor_rev < minor:
            return False

        # Same minor, check patch
        if self.fw_patch_num >= patch:
            return True

        return False

    @classmethod
    def from_command_response_string(cls, input_string):
        """Generate robot information from standard robot connection response string.

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
                       is_virtual=(matches[2] is not None),
                       fw_major_rev=int(matches[3]),
                       fw_minor_rev=int(matches[4]),
                       fw_patch_num=int(matches[5]))
        except Exception as exception:
            raise ValueError(f'Could not parse robot info string "{input_string}"')


class _Message:
    """Class for storing a response message from a Mecademic robot.

    Attributes
    ----------
    id : integer
        The id of the message, representing the type of message.
    data : string
        The raw payload of the message.
"""

    def __init__(self, id, data):
        self.id = id
        self.data = data

    def __repr__(self):
        return "Message with id={}, data={}".format(self.id, self.data)

    @classmethod
    def from_string(cls, input):
        """Construct message object from raw string input.

        Parameters
        ----------
        input : string
            Input string to convert to message.

        """
        id_start = input.find('[') + 1
        id_end = input.find(']', id_start)
        id = int(input[id_start:id_end])
        # Find next square brackets (contains data).
        data_start = input.find('[', id_end) + 1
        data_end = input.find(']', data_start)

        data = ''
        if data_start != -1 and data_end != -1:
            data = input[data_start:data_end]

        return cls(id, data)


class _RobotEvents:
    """Class for storing possible status events for the Mecademic robot.

    Attributes
    ----------
    on_connected : event
        Set if robot is connected.
    on_disconnected : event
        Set if robot is disconnected.
    on_status_updated : event
        Set if robot status is updated.
     on_status_gripper_updated : event
        Set if gripper status is updated.
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
        Set if robot receives p_stop.
    on_p_stop_reset : event
        Set if p_stop is reset.
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
    on_end_of_cycle: event
        Set if end of cycle has been reached
"""

    def __init__(self):
        self.on_connected = InterruptableEvent()
        self.on_disconnected = InterruptableEvent()

        self.on_status_updated = InterruptableEvent()
        self.on_status_gripper_updated = InterruptableEvent()

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

        self.on_joints_updated = InterruptableEvent()
        self.on_pose_updated = InterruptableEvent()

        self.on_brakes_activated = InterruptableEvent()
        self.on_brakes_deactivated = InterruptableEvent()

        self.on_offline_program_started = InterruptableEvent()

        self.on_end_of_block = InterruptableEvent()
        self.on_end_of_cycle = InterruptableEvent()

        self.on_disconnected.set()
        self.on_deactivated.set()
        self.on_error_reset.set()
        self.on_p_stop_reset.set()
        self.on_motion_resumed.set()
        self.on_deactivate_sim.set()

        self.on_status_updated.set()
        self.on_status_gripper_updated.set()
        self.on_joints_updated.set()
        self.on_pose_updated.set()
        self.on_brakes_activated.set()

    def clear_all(self):
        """Clear all events.

        """
        for attr in self.__dict__:
            self.__dict__[attr].clear()

    def abort_all(self, skipped_events=[], message=""):
        """Abort all events, except for events in skipped_events list.

        """
        for attr in self.__dict__:
            if attr not in skipped_events:
                self.__dict__[attr].abort(message)

    def abort_all_on_error(self, message=""):
        """Abort all events in the specific case where the robot has reported an error.

        """
        self.abort_all(
            skipped_events=[
                'on_connected',  # Don't abort a wait for "on_connected" (should be done by now anyways)
                'on_status_updated',  # Don't abort a wait for "on_status_updated", that's what we're doing!
                'on_error_reset',  # Don't abort a wait for "on_error_reset" because we got an error
                'on_end_of_cycle'  # Don't abort a wait for "on_end_of_cycle", cycles should continue during error
            ],
            message=message)

    def clear_abort_all(self):
        """Clear aborts for all events.

        """
        for attr in self.__dict__:
            self.__dict__[attr].clear_abort()


class _CallbackQueue():
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
            if robot_callbacks.__dict__[attr] is not None:
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
        if callback_name in self._registered_callbacks or callback_name == _TERMINATE:
            self._queue.put((callback_name, data))

    def get(self, block=False, timeout: float = None):
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
