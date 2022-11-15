#!/usr/bin/env python3
from __future__ import annotations

import copy
import functools
import ipaddress
import json
import logging
import math
import pathlib
import queue
import socket
import threading
import time
import weakref
from argparse import ArgumentError
from typing import Optional

import requests

from .mx_robot_def import MxRobotStatusCode as mx_st
from .mx_robot_def import *
from .robot_classes import *
from .tools import *

_CHECKPOINT_ID_MAX_PRIVATE = 8191  # Max allowable checkpoint id, inclusive

_TERMINATE = '--terminate--'


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
                self._disconnect()
                raise DisconnectError('Automatically disconnected as a result of exception, '
                                      'set \'disconnect_on_exception\' to False to disable.') from e
            else:
                raise e

    return wrap


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
    on_external_tool_status_updated: event
        Set if external tool status has been updated.
    on_gripper_state_updated: event
        Set if gripper state has been updated.
    on_valve_state_updated: event
        Set if pneumatic module valve state has been updated.
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
    on_pstop2 : event
        Set if robot receives pstop2.
    on_pstop2_resettable : event
        Set if pstop2 becomes resettable.
    on_pstop2_reset : event
        Set if pstop2 is reset.
    on_estop : event
        Set if robot receives estop.
    on_estop_reset : event
        Set if estop is reset.
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
    on_activate_ext_tool_sim : event
        Set if robot is in gripper sim mode.
    on_deactivate_ext_tool_sim : event
        Set if robot is not in gripper sim mode.
    on_activate_recovery_mode : event
        Set if robot is in recovery mode.
    on_deactivate_recovery_mode : event
        Set if robot is not in recovery mode.
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
    on_offline_program_op_done : event
        Set when offline program operation (list/load/save/delete) has completed.
    on_end_of_block : event
        Set if end of block has been reached.
    on_end_of_cycle: event
        Set if end of cycle has been reached.

"""

    def __init__(self):
        self.on_connected = InterruptableEvent()
        self.on_disconnected = InterruptableEvent()

        self.on_status_updated = InterruptableEvent()
        self.on_status_gripper_updated = InterruptableEvent()

        self.on_external_tool_status_updated = InterruptableEvent()
        self.on_gripper_state_updated = InterruptableEvent()
        self.on_valve_state_updated = InterruptableEvent()

        self.on_activated = InterruptableEvent()
        self.on_deactivated = InterruptableEvent()

        self.on_homed = InterruptableEvent()

        self.on_error = InterruptableEvent()
        self.on_error_reset = InterruptableEvent()
        self.on_pstop2 = InterruptableEvent()
        self.on_pstop2_resettable = InterruptableEvent()
        self.on_pstop2_reset = InterruptableEvent()
        self.on_estop = InterruptableEvent()
        self.on_estop_reset = InterruptableEvent()

        self.on_motion_paused = InterruptableEvent()
        self.on_motion_resumed = InterruptableEvent()
        self.on_motion_cleared = InterruptableEvent()

        self.on_activate_sim = InterruptableEvent()
        self.on_deactivate_sim = InterruptableEvent()

        self.on_activate_ext_tool_sim = InterruptableEvent()
        self.on_deactivate_ext_tool_sim = InterruptableEvent()

        self.on_activate_recovery_mode = InterruptableEvent()
        self.on_deactivate_recovery_mode = InterruptableEvent()

        self.on_joints_updated = InterruptableEvent()
        self.on_pose_updated = InterruptableEvent()

        self.on_brakes_activated = InterruptableEvent()
        self.on_brakes_deactivated = InterruptableEvent()

        self.on_offline_program_started = InterruptableEvent()
        self.on_offline_program_op_done = InterruptableEvent()

        self.on_end_of_block = InterruptableEvent()
        self.on_end_of_cycle = InterruptableEvent()

        self.on_disconnected.set()
        self.on_deactivated.set()
        self.on_error_reset.set()
        self.on_pstop2_reset.set()
        self.on_pstop2_resettable.set()
        self.on_estop_reset.set()
        self.on_motion_resumed.set()
        self.on_deactivate_sim.set()

        self.on_status_updated.set()
        self.on_status_gripper_updated.set()
        self.on_external_tool_status_updated.set()
        self.on_gripper_state_updated.set()
        self.on_valve_state_updated.set()

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
                'on_deactivated',  # Don't abort waiting for "on_deactivated". WaitDeactivated is can be used in error
                'on_status_updated',  # Don't abort a wait for "on_status_updated", that's what we're doing!
                'on_error_reset',  # Don't abort a wait for "on_error_reset" because we got an error
                'on_end_of_cycle',  # Don't abort a wait for "on_end_of_cycle", cycles should continue during error
                'on_activate_recovery_mode',  # Don't abort wait for "on_activate_recovery_mode", available in error
                'on_deactivate_recovery_mode',  # Don't abort wait for "on_deactivate_recovery_mode", available in error
                'on_estop_reset',  # The EStop state may still change while in error
                'on_pstop2_resettable',  # The "PStop2 resettable" state may still change while in error
                'on_pstop2_reset',  # The PStop2 state may still change while in error
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

    def qsize(self) -> int:
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

    def get(self, block=False, timeout: float = None) -> tuple[str, str]:
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


class _Robot:
    """Class for controlling a Mecademic robot.

    Attributes (private, please use public methods instead, i.e. methods not starting with underscore)
    ----------
    _address : string
        The IP address associated to the Mecademic Robot.
    _command_socket : socket object
        Socket connecting to the command port of the Mecademic robot.
    _monitor_socket : socket object
        Socket connecting to the monitor port of the Mecademic robot.

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
    _external_tool_status: ExtToolStatus object
        Stores most current external tool status
    _gripper_state: GripperState object
        Stores most current gripper state
    _gripper_state_before_last_move : GripperState object
        Stores gripper state at the time a Open/Close/Move gripper command is sent
        (then used to accelerate WaitGripperMoveCompletion in case target_pos_reached is already True when its called
         because otherwise it's not possible to know if the move has completed, or not yet started)
    _valve_state: ValveState object
        Stores most current pneumatic valve state
    _robot_events : RobotEvents object
        Stores events related to the robot state.

    _file_logger : RobotDataLogger object
        Collects RobotInformation, all RobotRtData and SentCommands during determined period
    _monitoring_interval : float
        Initial monitoring interval to restore after logging session

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
        Values from legacy mx_st.MX_ST_GET_JOINTS event received in current cycle
    _tmp_rt_cart_pos : list of float
        Values from legacy mx_st.MX_ST_GET_POSE event received in current cycle

    _tx_sync : integer
        Value sent in the most recent "SyncCmdQueue" request sent to robot
    _rx_sync : integer
        Most recent response to "SyncCmdQueue" (mx_st.MX_ST_SYNC_CMD_QUEUE) received from the robot
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
        if self._is_initialized and self.IsConnected():
            self._disconnect()
            # Note: Don't unregister callbacks, we allow them to remain valid after a "with" block
            # self.UnregisterCallbacks()

        self._address = None
        self._custom_port = None

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

        self._robot_info = RobotInfo()
        self._robot_rt_data = None
        self._robot_rt_data_stable = None
        self._robot_status = RobotStatus()
        self._first_robot_status_received = False
        self._using_json_api = False
        self._gripper_status = GripperStatus()
        self._external_tool_status = ExtToolStatus()
        self._gripper_state = GripperState()
        self._gripper_state_before_last_move = GripperState()
        self._valve_state = ValveState()
        self._robot_events = _RobotEvents()

        self._file_logger = None
        self._monitoring_interval = None
        self._monitoring_interval_to_restore = None

        self._reset_disconnect_attributes()

        self._enable_synchronous_mode = None
        self._disconnect_on_exception = None

        self._offline_mode = None
        self._monitor_mode = None

        self.logger = logging.getLogger(__name__)
        self.default_timeout = DEFAULT_WAIT_TIMEOUT

        # Variables to hold joint positions and poses while waiting for timestamp.
        self._tmp_rt_joint_pos = None
        self._tmp_rt_cart_pos = None

        self._tx_sync = 0
        self._rx_sync = 0

        self._captured_trajectory = None

        self._is_initialized = True

    def _reset_disconnect_attributes(self):
        self._command_rx_queue = queue.Queue()
        self._command_tx_queue = queue.Queue()
        self._monitor_rx_queue = queue.Queue()
        self._custom_response_events = list()

        self._user_checkpoints = dict()
        self._internal_checkpoints = dict()
        self._internal_checkpoint_counter = MX_CHECKPOINT_ID_MAX + 1

        self._clear_motion_requests = 0

    #####################################################################################
    # Static methods.
    #####################################################################################

    @staticmethod
    def _deactivate_on_exception(func, command_socket: socket.socket, *args, **kwargs):
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
    def _handle_socket_rx(robot_socket: socket.socket, rx_queue: queue.Queue, logger: logging.Logger):
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
                rx_queue.put(Message.from_string(response))

    @staticmethod
    def _handle_socket_tx(robot_socket: socket.socket, tx_queue: queue.Queue, logger: logging.Logger):
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
            command: str = tx_queue.get(block=True)

            # Terminate thread if requested, otherwise send the command.
            if command == _TERMINATE:
                return
            else:
                logger.debug(f'Socket Tx - Command: {command}')
                robot_socket.sendall((command + '\0').encode('ascii'))

    @staticmethod
    def _connect_socket(logger: logging.Logger, address: str, port: int, socket_timeout=0.1) -> socket.socket:
        """Connects to an arbitrary socket.

        Parameters
        ----------
        logger : logger instance
            Logger to use.
        address : string
            Address to use.
        port : int
            Port number to use.
        socket_timeout: seconds
            Time allocated (in seconds) to connect to robot

        Returns
        -------
        new_socket : socket object
            Successfully-connected socket object.

        """
        logger.debug(f'Attempting to connect to {address}:{port}')

        connect_loops = math.ceil(socket_timeout / 0.1)
        for _ in range(connect_loops):
            # Create socket and attempt connection.
            new_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            new_socket.settimeout(100)
            try:
                new_socket.connect((address, port))
                break
            except (socket.timeout, TimeoutError) as e:
                logger.debug(f'Timeout connecting to {address}:{port}, retrying in 100ms.')
                continue
            except ConnectionRefusedError:
                logger.debug(f'Connection refused to {address}:{port}, retrying in 100ms.')
                time.sleep(0.1)
                continue
            except Exception as exception:
                raise TimeoutError(f'Unable to connect to {address}:{port}, exception: {exception}')

        if new_socket is None:
            raise TimeoutError(f'Timeout while connecting to {address}:{port}.')

        logger.info(f'Connected to {address}:{port}.')
        return new_socket

    @staticmethod
    def _handle_callbacks(logger, callback_queue: _CallbackQueue, callbacks: RobotCallbacks, timeout: float = None):
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
        """See documentation in equivalent function in robot.py"""
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
        """See documentation in equivalent function in robot.py"""
        self._stop_callback_thread()

        self._robot_callbacks = RobotCallbacks()
        self._run_callbacks_in_separate_thread = False
        self._callback_queue = _CallbackQueue(self._robot_callbacks)
        self._callback_thread = None

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

    def Connect(self,
                address: str = MX_DEFAULT_ROBOT_IP,
                enable_synchronous_mode: bool = False,
                disconnect_on_exception: bool = True,
                monitor_mode: bool = False,
                offline_mode: bool = False,
                timeout: float = 0.1):
        """See documentation in equivalent function in robot.py"""
        try:
            with self._main_lock:

                if self.IsConnected():
                    try:
                        self._check_internal_states(refresh_monitoring_mode=True)
                        return  # Still connected -> Do nothing
                    except Exception:
                        self.logger.info('Connection to robot was lost, attempting re-connection.')

                # Check that the ip address is valid and set address.
                if not isinstance(address, str):
                    raise TypeError(f'Invalid IP address ({address}).')

                # Check if user has specified a custom port to connect to
                addr_port = address.split(':')
                if len(addr_port) > 1:
                    self._custom_port = int(addr_port[1])
                    address = addr_port[0]
                    self.logger.info(f"Connecting to robot {address}:{self._custom_port}")
                else:
                    self.logger.info(f"Connecting to robot: {address}")
                ipaddress.ip_address(address)
                self._address = address

                self._enable_synchronous_mode = enable_synchronous_mode
                self._disconnect_on_exception = disconnect_on_exception

                self._offline_mode = offline_mode
                self._monitor_mode = monitor_mode

                self._first_robot_status_received = False
                self._using_json_api = False

                if not self._monitor_mode:
                    self._initialize_command_socket(timeout)
                    self._initialize_command_connection()

                self._robot_events.clear_all()

                self._robot_events.on_deactivated.set()
                self._robot_events.on_error_reset.set()
                self._robot_events.on_pstop2_reset.set()
                self._robot_events.on_pstop2_resettable.set()
                self._robot_events.on_estop_reset.set()
                self._robot_events.on_motion_resumed.set()
                self._set_brakes_engaged(True)

                self._robot_events.on_status_updated.set()
                self._robot_events.on_status_gripper_updated.set()
                self._robot_events.on_external_tool_status_updated.set()
                self._robot_events.on_gripper_state_updated.set()
                self._robot_events.on_valve_state_updated.set()

                self._robot_events.on_joints_updated.set()
                self._robot_events.on_pose_updated.set()

                # Start callback thread if necessary
                self._start_callback_thread()

            connect_to_monitoring_port = True
            if not self._monitor_mode and self._robot_info.version.major >= 8:
                can_query_robot_info = True

                if not self._robot_info.rt_message_capable:
                    # For these versions (8.3-), it is not possible to get robot information if in error.
                    self._send_custom_command('GetStatusRobot',
                                              expected_responses=[mx_st.MX_ST_GET_STATUS_ROBOT],
                                              timeout=self.default_timeout,
                                              skip_internal_check=True)
                    if self._robot_status.error_status:
                        can_query_robot_info = False

                if can_query_robot_info:
                    # Fetch the robot serial number
                    self._send_custom_command('GetRobotSerial',
                                              expected_responses=[mx_st.MX_ST_GET_ROBOT_SERIAL],
                                              timeout=self.default_timeout,
                                              skip_internal_check=True)

                    # Fetch full version
                    full_version_response = self._send_custom_command('GetFwVersionFull',
                                                                      [mx_st.MX_ST_GET_FW_VERSION_FULL],
                                                                      timeout=self.default_timeout,
                                                                      skip_internal_check=True)
                    full_version = full_version_response.data
                    self._robot_info.version.update_version(full_version)

                    # Fetch the current real-time monitoring settings
                    if self._robot_info.rt_message_capable:
                        self._send_custom_command('GetRealTimeMonitoring',
                                                  expected_responses=[mx_st.MX_ST_GET_REAL_TIME_MONITORING],
                                                  timeout=self.default_timeout,
                                                  skip_internal_check=True)

                        # Get initial monitoring interval
                        monitoring_interval_response = self._send_custom_command(
                            'GetMonitoringInterval',
                            expected_responses=[mx_st.MX_ST_GET_MONITORING_INTERVAL],
                            timeout=self.default_timeout,
                            skip_internal_check=True)
                        self._monitoring_interval = float(monitoring_interval_response.data)
                        self._monitoring_interval_to_restore = self._monitoring_interval

                    # Check if this robot supports sending monitoring data on ctrl port (which we want to do to avoid
                    # race conditions between the two sockets causing potential problems with this API)
                    # Also make sure we have received a robot status event before continuing
                    if self._robot_info.rt_on_ctrl_port_capable:
                        connect_to_monitoring_port = False  # We won't need to connect to monitoring port
                        self._send_custom_command('SetCtrlPortMonitoring(1)',
                                                  expected_responses=[mx_st.MX_ST_GET_STATUS_ROBOT],
                                                  timeout=self.default_timeout,
                                                  skip_internal_check=True)
                    else:
                        self._send_custom_command('GetStatusRobot',
                                                  expected_responses=[mx_st.MX_ST_GET_STATUS_ROBOT],
                                                  timeout=self.default_timeout,
                                                  skip_internal_check=True)

            if connect_to_monitoring_port:
                with self._main_lock:
                    self._initialize_monitoring_socket(timeout)
                    self._initialize_monitoring_connection()

            # Now that we're connected, let's update _robot_info with the connected Ip address
            self._robot_info.ip_address = address

            if self._robot_info.version.major < 8:
                self.logger.warning('Python API not supported for firmware under version 8')

            self._robot_events.on_connected.set()
            self._callback_queue.put('on_connected')
        except Exception:
            self._disconnect()
            raise

    def Disconnect(self):
        """See documentation in equivalent function in robot.py"""
        if self.IsConnected():
            self.logger.info('Disconnecting from the robot.')
            self._disconnect()
        else:
            self.logger.debug('Ignoring Disconnect() called on a non-connected robot.')

    def _disconnect(self):
        """
        Internal function to disconnect Mecademic Robot object from the Mecademic robot and cleanup internal states.

        """
        # Don't acquire _main_lock while shutting down queues to avoid deadlock.
        self._shut_down_queue_threads()

        with self._main_lock:
            message = "explicitly disconnected from the robot"
            self._shut_down_socket_threads()

            # Invalidate checkpoints and appropriate interruptable events
            self._invalidate_checkpoints(message)
            self._invalidate_interruptable_events_on_clear_motion(message)

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

    @disconnect_on_exception
    def GetStatusRobot(self, synchronous_update: bool = False, timeout: float = None) -> RobotStatus:
        """Return a copy of the current robot status

        Parameters
        ----------
        synchronous_update: boolean
            True -> Synchronously get updated robot status. False -> Get latest known status.
        timeout: float, defaults to DEFAULT_WAIT_TIMEOUT
            Timeout (in seconds) waiting for synchronous response from the robot.

        Returns
        -------
        RobotStatus
            Object containing the current robot status

        """
        # Use appropriate default timeout of not specified
        if timeout is None:
            timeout = self.default_timeout
        if synchronous_update:
            if self._using_json_api:
                with self._main_lock:
                    self._send_command('GetMotionStatus')
            self._send_sync_command('GetStatusRobot', self._robot_events.on_status_updated, timeout)

        with self._main_lock:
            return copy.deepcopy(self._robot_status)

    def IsConnected(self) -> bool:
        """See documentation in equivalent function in robot.py"""
        return self._robot_events.on_connected.is_set()

    @disconnect_on_exception
    def DeactivateRobot(self):
        """See documentation in equivalent function in robot.py"""
        with self._main_lock:
            self._check_internal_states()
            self._send_command('DeactivateRobot')

        if self._enable_synchronous_mode:
            self.WaitDeactivated()

    # Non-motion commands.

    def _set_monitoring_interval_internal(self, t: float):
        """Sets the rate at which the monitoring port sends data.

        Parameters
        ----------
        t : float
            Monitoring interval duration in seconds.

        """
        with self._main_lock:
            self._send_command('SetMonitoringInterval', [t])
            self._monitoring_interval = t

    @disconnect_on_exception
    def GetStatusGripper(self, synchronous_update: bool = False, timeout: float = None) -> GripperStatus:
        """Return a copy of the current gripper status.
           LEGACY. Use GetRtExtToolStatus and GetRtGripperState instead.

        Parameters
        ----------
        synchronous_update: boolean
            True -> Synchronously get updated gripper status. False -> Get latest known status.
        timeout: float, defaults to DEFAULT_WAIT_TIMEOUT
            Timeout (in seconds) waiting for synchronous response from the robot.

        Returns
        -------
        GripperStatus
            Object containing the current gripper status

        """
        # Use appropriate default timeout of not specified
        if timeout is None:
            timeout = self.default_timeout
        if synchronous_update:
            self._send_sync_command('GetStatusGripper', self._robot_events.on_status_gripper_updated, timeout)

        with self._main_lock:
            return copy.deepcopy(self._gripper_status)

    def WaitGripperMoveCompletion(self, timeout: Optional[float]):
        """See documentation in equivalent function in robot.py"""
        if not self._robot_info.gripper_pos_ctrl_capable:
            raise NotImplementedError(f"Unsupported method for this firmware version")

        if timeout is not None and timeout <= 0:
            raise ValueError("timeout must be None or a positive value")

        DEFAULT_START_MOVE_TIMEOUT = 0.2
        DEFAULT_COMPLETE_MOVE_TIMEOUT = 2
        if timeout is not None:
            complete_move_timeout = timeout
        else:
            complete_move_timeout = DEFAULT_COMPLETE_MOVE_TIMEOUT

        # Use a checkpoint to make sure the last gripper command has been processed
        if not self._enable_synchronous_mode:
            start_time = time.monotonic()
            checkpoint = self._set_checkpoint_internal()
            checkpoint.wait(complete_move_timeout)
            # Update timeout left
            complete_move_timeout -= (time.monotonic() - start_time)

        start_move_timeout = DEFAULT_START_MOVE_TIMEOUT
        if start_move_timeout > complete_move_timeout:
            start_move_timeout = complete_move_timeout

        # Detect a rising edge of either `target_pos_reached` or `holding_part` to rapidly confirm the end of move.
        # This is needed to ensure the gripper has started moving and we're not reporting a previous state.
        # When we have given the gripper enough time to start moving and `target_pos_reached` or `holding_part`
        # are still true, it means that the gripper was already at target position or that an object is preventing
        # the gripper from reaching that position, so the move completes.
        holding_part_seen_false = not self._gripper_state_before_last_move.holding_part
        pos_reached_seen_false = not self._gripper_state_before_last_move.target_pos_reached

        current_time = time.monotonic()
        start_time = current_time
        timeout_time = start_time + start_move_timeout
        waiting_move_start = True
        while current_time < timeout_time:
            wait_duration = timeout_time - current_time
            self.logger.debug(f'WaitGripperMoveCompletion: Waiting for {wait_duration}s')
            try:
                self._robot_events.on_gripper_state_updated.wait(wait_duration)
                with self._main_lock:
                    gripper_state = self._gripper_state
                    self._robot_events.on_gripper_state_updated.clear()
                    self.logger.debug(f'WaitGripperMoveCompletion: New state is {str(gripper_state)}')

                if waiting_move_start:
                    if gripper_state.target_pos_reached:
                        if pos_reached_seen_false:
                            self.logger.debug(f'WaitGripperMoveCompletion: target_pos_reached')
                            return
                        if gripper_state.opened and not self._gripper_state_before_last_move.opened:
                            self.logger.debug(f'WaitGripperMoveCompletion: now opened (was not)')
                            return
                        if gripper_state.closed and not self._gripper_state_before_last_move.closed:
                            self.logger.debug(f'WaitGripperMoveCompletion: now closed (was not)')
                            return

                    if holding_part_seen_false and gripper_state.holding_part:
                        self.logger.debug(f'WaitGripperMoveCompletion: holding_part')
                        return

                    if gripper_state.holding_part is False:
                        self.logger.debug(f'WaitGripperMoveCompletion: holding_part_seen_false')
                        holding_part_seen_false = True

                    if gripper_state.target_pos_reached is False:
                        self.logger.debug(f'WaitGripperMoveCompletion: pos_reached_seen_false')
                        pos_reached_seen_false = True

                else:
                    if gripper_state.target_pos_reached or gripper_state.holding_part:
                        self.logger.debug(f'WaitGripperMoveCompletion: move completed ')
                        return
            except TimeoutException:
                if waiting_move_start:
                    self.logger.debug(f'WaitGripperMoveCompletion: start_move_timeout reached')
                    gripper_state = self._gripper_state
                    if gripper_state.target_pos_reached or gripper_state.holding_part:
                        # Gripper had time to start moving and the state still report that the gripper is at the target
                        # position or holding a part. This happens when the gripper was not able to move because it is
                        # forcing on an object.
                        self.logger.debug(
                            f'WaitGripperMoveCompletion: start_move_timeout reached with no change detected')
                        return
                    # We now give enough time for the move to complete
                    waiting_move_start = False
                    timeout_time = start_time + complete_move_timeout
            current_time = time.monotonic()

        if not gripper_state.target_pos_reached and not gripper_state.holding_part:
            self.logger.warning(f'WaitGripperMoveCompletion: Timeout reached')
            raise TimeoutException('Timeout while waiting for gripper to complete movement.')

    @disconnect_on_exception
    def GetRobotInfo(self) -> RobotInfo:
        """Return a copy of the known robot information.

        Return
        ------
        RobotInfo
            Object containing robot information.

        """
        with self._main_lock:
            return copy.deepcopy(self._robot_info)

    UPDATE_TIMEOUT = 15 * 60  # 15 minutes timeout

    def UpdateRobot(self,
                    firmware: Union[str, pathlib.Path],
                    address: Optional[str] = None,
                    timeout: float = UPDATE_TIMEOUT):
        """See documentation in equivalent function in robot.py"""

        if isinstance(firmware, pathlib.Path):
            firmware_file: pathlib.Path = firmware
        elif isinstance(firmware, str):
            firmware_file = pathlib.Path(firmware)
        else:
            raise ArgumentError(None,
                                f'Unsupported firmware type. received: {type(firmware)}, expecting pathlib or str')

        firmware_file_version = RobotVersion(firmware_file.name)

        # Validates robot IP address is available to script
        if address is None and not self.IsConnected():
            raise ArgumentError(None, f"address parameter can't be None and not connected to a robot.")
        if address is None:
            address = self._address
        elif self._address and address != self._address:
            raise ArgumentError(None,
                                f"Trying to update robot at IP {address} but currently connected to {self._address}")

        # Making sure we can send command to robot
        if not self.IsConnected():
            self.Connect(address=address)
        elif self._monitor_mode:
            self.logger.info(f'Connected to robot in monitoring mode only, attempting connection in command mode'
                             'to deactivate robot')
            self.Connect(address=address)

        if self.GetStatusRobot(synchronous_update=True).activation_state:
            self.logger.info(f'Robot is activated, will attempt to deactivate before updating firmware')
            self.DeactivateRobot()
            self.WaitDeactivated()
        if self._enable_synchronous_mode is None:
            current_synchronous_mode = False
        else:
            current_synchronous_mode = self._enable_synchronous_mode
        self.Disconnect()

        tested_port_8080 = False
        robot_url = f"http://{address}/"
        while (True):

            self.logger.info(f"Installing firmware: {firmware_file.resolve()}")

            with open(str(firmware_file), 'rb') as firmware_stream:
                firmware_data = firmware_stream.read()
                firmware_data_size = str(len(firmware_data))

            headers = {
                'Connection': 'keep-alive',
                'Content-type': 'application/x-gzip',
                'Content-Length': firmware_data_size
            }

            self.logger.info(f"Uploading firmware")
            request_post = requests.post(robot_url, data=firmware_data, headers=headers)
            try:
                request_post.raise_for_status()
                break
            except Exception as e:
                if not tested_port_8080:
                    # Failed -> Probably a robot v9.2+ with the new Web Portal. Let's use the 'legacy' Web
                    # portal (for now at least, until we update this Python code to support the new "Json" API)
                    tested_port_8080 = True
                    robot_url = f"http://{address}:8080/"
                    continue
                self.logger.error(f"Upgrade post request error: {e}")
                raise

        if not request_post.ok:
            error_message = f"Firmware upload request failed"
            raise RuntimeError(error_message)

        self.logger.info(f"Upgrading the robot")
        update_progress = UpdateProgress()

        start_time = time.monotonic()
        while True:

            # Give time to the web server restart, the function doesn't handle well errors.
            time.sleep(2)

            self._check_update_progress(robot_url, update_progress)
            if update_progress.complete:
                self.logger.info(f"Firmware update complete")
                break
            if time.monotonic() > start_time + timeout:
                error_message = f"Timeout while waiting for update done response, after {timeout} seconds"
                raise TimeoutError(error_message)

        self.logger.info(f"Waiting for robot to reboot")

        # need to wait to make sure the robot shutdown before attempting to ping it.
        time.sleep(15)
        # Try to ping the robot until it's responding (or until default timeout)
        ping_robot(address)
        # Now that robot responds to ping, wait until it accepts new connections
        self.Connect(address, timeout=60, enable_synchronous_mode=current_synchronous_mode)

        current_version = self.GetRobotInfo().version
        if current_version.major < 8.0:
            expected_version = firmware_file_version.short_version
        else:
            expected_version = firmware_file_version.full_version

        if str(current_version) in expected_version:
            self.logger.info(f"robot is now running version {current_version}")
        else:
            error_msg = f"Fail to install robot properly. current version {current_version}, " \
                        f"expecting: {expected_version}"
            self.logger.error(error_msg)
            raise AssertionError(error_msg)

        robot_status = self.GetStatusRobot(synchronous_update=True)
        if robot_status.error_status:
            error_msg = f"Robot is in error on version {current_version}"
            self.logger.error(error_msg)
            raise AssertionError(error_msg)

        self.logger.info(f"Installation of {current_version} successfully completed")

    #####################################################################################
    # Private methods.
    #####################################################################################

    def _check_update_progress(self, robot_url: str, update_progress: UpdateProgress):
        """
        Check progress of firmware update.

        Parameters
        ----------
        robot_url: string
            Robot URL

        update_progress: UpdateProgress
            Update progress information object

        """
        update_progress.complete = False
        request_get = requests.get(robot_url, 'update', timeout=10)
        try:
            request_get.raise_for_status()
        except Exception as e:
            self.logger.error(f'Upgrade get request error: {e}')
            raise

        # get only correct answer (http code 200)
        if request_get.status_code == 200:
            request_response = request_get.text
        else:
            request_response = None
        # while the json file is note created, get function will return 0
        if request_response is None or request_response == '0':
            return

        try:
            request_answer = json.loads(request_response)
        except Exception as e:
            self.logger.info(f'Error retrieving json from request_response: {e}')
            return

        if not request_answer:
            self.logger.info(f'Answer is empty')
            return

        if request_answer['STATUS']:
            status_code = int(request_answer['STATUS']['Code'])
            status_msg = request_answer['STATUS']['MSG']

        if status_code in [0, 1]:
            keys = sorted(request_answer['LOG'].keys())
            if keys:
                previous_progress = update_progress.progress
                update_progress.progress = request_answer['LOG'][keys[-1]]
                new_progress = update_progress.progress.replace(previous_progress, '')
                if ':' in new_progress:
                    self.logger.info(new_progress)
                elif '100%' in new_progress:
                    self.logger.info(new_progress)
                else:
                    self.logger.debug(new_progress)
            if status_code == 0:
                self.logger.info(f'status_msg {status_msg}')
                update_progress.complete = True
                return
        else:
            error_message = f'error while updating: {status_msg}'
            self.logger.error(error_message)
            raise RuntimeError(error_message)

    def _check_monitor_threads(self):
        """Check that the threads which handle robot monitor messages are alive.

        Attempt to disconnect from the robot if not.

        """
        if self._robot_info.rt_on_ctrl_port_capable:
            # We're not using monitoring port. No need to check here.
            return

        if not (self._monitor_handler_thread and self._monitor_handler_thread.is_alive()):
            raise InvalidStateError('Monitor response handler thread has unexpectedly terminated.')

        if self._offline_mode:  # Do not check rx threads in offline mode.
            return

        if not (self._monitor_rx_thread and self._monitor_rx_thread.is_alive()):
            raise InvalidStateError('Monitor rx thread has unexpectedly terminated.')

    def _check_command_threads(self):
        """Check that the threads which handle robot command messages are alive.

        Attempt to disconnect from the robot if not.

        """

        if not (self._command_response_handler_thread and self._command_response_handler_thread.is_alive()):
            raise InvalidStateError('No command response handler thread, are you in monitor mode?')

        if self._offline_mode:  # Do not check rx threads in offline mode.
            return

        if not (self._command_rx_thread and self._command_rx_thread.is_alive()):
            raise InvalidStateError('No command rx thread, are you in monitor mode?')

        # If tx thread is down, attempt to directly send deactivate command to the robot.
        if not (self._command_tx_thread and self._command_tx_thread.is_alive()):
            self._command_socket.sendall(b'DeactivateRobot\0')
            raise InvalidStateError('No command tx thread, are you in monitor mode?')

    def _check_internal_states(self, refresh_monitoring_mode=False):
        """Check that the threads which handle robot messages are alive.

        Attempt to disconnect from the robot if not.

        Parameters
        ----------
        refresh_monitoring_mode : boolean
            Refresh internal states even in monitoring mode when True, raise an exception otherwise.
        """
        try:
            if self._monitor_mode:
                if not refresh_monitoring_mode:
                    raise InvalidStateError('Cannot send command while in monitoring mode.')
            else:
                self._check_command_threads()

            self._check_monitor_threads()
        except Exception:
            # An error was detected while validating internal states. Disconnect from robot.
            self._disconnect()
            raise

    def _split_command_args(self,
                            command: str,
                            args: Union[str, list, tuple] = None) -> list[str, Union[str, list, tuple]]:
        """In the case the arguments are passed in the command argument, this function will split the command name
           and arguments.

        Parameters
        ----------
        command : str
            String that contains the command name and possibly the arguments too
        args : Union[str, list, tuple], optional
            Arguments for the command, as a string, a list or a tuple, by default None

        Returns
        -------
        list[str, Union[str, list, tuple]]
            Command name (without arguments), arguments (as a string, a list or a tuple, same as input parameter)
        """
        if args is None:
            # Split command from args by searching for opening parenthesis
            split_result = command.split('(', 1)
            if len(split_result) == 2:
                # Strip command name (just in case there were spaces around it)
                command = split_result[0]
                # Remove trailing parenthesis from arguments
                args = split_result[1].rstrip(")")
        # Strip command name (just in case there were spaces around it)
        command = command.strip()
        return [command, args]

    def _send_command(self, command: str, args: Union[str, list, tuple] = None):
        """Assembles and sends the command string to the Mecademic robot.

        Parameters
        ----------
        command : string
            Command name to send to the Mecademic robot.
        arg_list : list or str
            List of arguments the command requires.

        """

        # Make sure that command and args are split
        command, args = self._split_command_args(command, args)
        if command.lower() == 'clearmotion':
            # Clearing the motion queue also requires clearing checkpoints, as the robot will not send them anymore.
            message = "ClearMotion"
            self._invalidate_checkpoints(message)
            self._invalidate_interruptable_events_on_clear_motion(message)

        # Assemble arguments into a string and concatenate to end of command.
        if args:
            if isinstance(args, list) or isinstance(args, tuple):
                command += f'({args_to_string(args)})'
            else:
                command += f'({args})'

        # Put command into tx queue.
        self._command_tx_queue.put(command)

        # If logging is enabled, send command to logger.
        if self._file_logger and self._file_logger.logging_commands:
            self._file_logger.command_queue.put(command)

    @disconnect_on_exception
    def WaitDeactivated(self, timeout: float = None):
        """Pause program execution until the robot is deactivated.

        Parameters
        ----------
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).
        """
        # Use appropriate default timeout of not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_deactivated.wait(timeout=timeout)

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
                # Send a "SyncCmdQueue" request so we know when we get the response to this get (and not an earlier one)
                self._tx_sync += 1
                self._send_command('SyncCmdQueue', f'{self._tx_sync}')
            if event.is_set():
                event.clear()
            self._send_command(command)

        # Wait until response is received (this will throw TimeoutException if appropriate)
        event.wait(timeout=timeout)

    def _send_custom_command(self,
                             command: str,
                             expected_responses: list[int] = None,
                             timeout: float = None,
                             skip_internal_check: bool = False) -> InterruptableEvent | Message:
        """Internal version of SendCustomCommand with option to skip internal state check (so it can be used
           during connection)
        """
        with self._main_lock:
            if not skip_internal_check:
                self._check_internal_states()

            if expected_responses:
                event_with_data = InterruptableEvent(data=expected_responses)
                self._custom_response_events.append(weakref.ref(event_with_data))

            self._send_command(command)

        if expected_responses:
            if timeout is None:
                return event_with_data
            else:
                response = event_with_data.wait(timeout)
                return response
        default_event = InterruptableEvent(None, None)
        default_event.set()
        return default_event

    def _launch_thread(self, *, target, args) -> threading.Thread:
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

    def _initialize_command_socket(self, timeout=0.1):
        """Establish the command socket and the associated thread.

        """
        if self._offline_mode:
            return

        if self._command_socket is not None:
            raise InvalidStateError('Cannot connect since existing command socket exists.')

        if self._custom_port:
            port = self._custom_port
        else:
            port = MX_ROBOT_TCP_PORT_CONTROL
        self._command_socket = self._connect_socket(self.logger, self._address, port, timeout)

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

    def _initialize_monitoring_socket(self, timeout):
        """Establish the monitoring socket and the associated thread.

        """
        if self._offline_mode:
            return

        if self._monitor_socket is not None:
            raise InvalidStateError('Cannot connect since existing monitor socket exists.')

        if self._custom_port:
            port = self._custom_port
        else:
            port = MX_ROBOT_TCP_PORT_FEED
        self._monitor_socket = self._connect_socket(self.logger, self._address, port, timeout)

        if self._monitor_socket is None:
            raise CommunicationError('Monitor socket could not be created. Is the IP address correct?')

        # Create rx thread for monitor socket communication.
        self._monitor_rx_thread = self._launch_thread(target=self._handle_socket_rx,
                                                      args=(
                                                          self._monitor_socket,
                                                          self._monitor_rx_queue,
                                                          self.logger,
                                                      ))

    def _receive_welcome_message(self, message_queue: queue.Queue, from_command_port: bool):
        """Receive and parse a welcome message in order to set _robot_info and _robot_rt_data.

        Parameters
        ----------
        message_queue : queue
            The welcome message will be fetched from this queue.
        """

        # Wait for connection string (mx_st.MX_ST_CONNECTED).
        # Alternatively, wait for status robot (mx_st.MX_ST_GET_STATUS_ROBOT) since older robots will not post the
        # connection string on the monitoring port
        start = time.monotonic()
        while True:
            try:
                response: Message = message_queue.get(block=True, timeout=self.default_timeout)
            except queue.Empty:
                self.logger.error('No response received within timeout interval.')
                raise CommunicationError('No response received within timeout interval.')
            except BaseException:
                raise

            if response.id == mx_st.MX_ST_CONNECTED or response.id == mx_st.MX_ST_GET_STATUS_ROBOT:
                break

            if from_command_port:
                break

            if (time.monotonic() - start) > self.default_timeout:
                self.logger.error('No connect message received within timeout interval.')
                break

        if response.id == mx_st.MX_ST_CONNECTED:
            # Attempt to parse robot return data.
            self._robot_info = self._parse_welcome_message(response.data)
        elif response.id == mx_st.MX_ST_GET_STATUS_ROBOT:
            # This means we're connected to a Legacy robot that does not send mx_st.MX_ST_CONNECTED on monitoring port.
            # We will not be able to deduce robot version. Assume some 8.x version
            self._robot_info = RobotInfo(model='Meca500', revision=3, version='8.0.0.0-unknown-version')
        else:
            self.logger.error('Connection error: {}'.format(response))
            raise CommunicationError('Connection error: {}'.format(response))

        self._robot_rt_data = RobotRtData(self._robot_info.num_joints)
        self._robot_rt_data_stable = RobotRtData(self._robot_info.num_joints)

    def _parse_welcome_message(self, message: str) -> RobotInfo:
        """Parse the robot's connection 'welcome' message and build RobotInfo from it
           (identify robot model, version, etc.)

        Parameters
        ----------
        message : str
            Welcome string received from the robot

        Returns
        -------
        RobotInfo
            Robot information class built from the received welcome message
        """
        return RobotInfo.from_command_response_string(message)

    def _initialize_command_connection(self):
        """Attempt to connect to the command port of the Mecademic Robot.

        """
        self._receive_welcome_message(self._command_rx_queue, True)

        self._command_response_handler_thread = self._launch_thread(target=self._command_response_handler, args=())

    def _initialize_monitoring_connection(self):
        """Attempt to connect to the monitor port of the Mecademic Robot."""

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

    def _set_checkpoint_internal(self) -> InterruptableEvent:
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
                self._internal_checkpoint_counter = MX_CHECKPOINT_ID_MAX + 1

            return self._set_checkpoint_impl(checkpoint_id)

    def _set_checkpoint_impl(self, n, send_to_robot=True) -> InterruptableEvent:
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
            elif MX_CHECKPOINT_ID_MAX < n <= _CHECKPOINT_ID_MAX_PRIVATE:
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
        '''Unblock all waiting checkpoints and have them throw InterruptException.'''

        for checkpoints_dict in [self._internal_checkpoints, self._user_checkpoints]:
            for checkpoints_list in checkpoints_dict.values():
                for event in checkpoints_list:
                    event.abort(message)
            checkpoints_dict.clear()

        self._internal_checkpoint_counter = MX_CHECKPOINT_ID_MAX + 1

    def _send_motion_command(self, command: str, arg_list=None):
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
            response: Message = self._monitor_rx_queue.get(block=True)

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
                if response.id == mx_st.MX_ST_GET_POSE and not self._robot_info.rt_message_capable:
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

    def _cleanup_custom_response_events(self):
        """Remove from custom response event list any event that is no more referenced by anyone
           (this is a list of weakref so they get destroyed automatically when no external code is holding a ref to
           that interruptable event, so as cleanup here we simply remove the weakref object from the list)"""
        # Build list of weakref objects to remove from the list (but don't delete yet, it would break the iteration)
        events_to_delete = []
        for event_weakref in self._custom_response_events:
            if event_weakref() is None:
                events_to_delete.append(event_weakref)
        # Now remove all appropriate items from the list
        for event_weakref in events_to_delete:
            self._custom_response_events.remove(event_weakref)

    def _awake_custom_response_events(self, response: Message):
        # Find any matching custom events (but don't remove yet from the list, this would break the iteration)
        matched_events = []
        for event_weakref in self._custom_response_events:
            for to_match in event_weakref().data:
                if isinstance(to_match, Message):
                    if response.id == to_match.id and response.data == to_match.data:
                        matched_events.append(event_weakref)
                else:
                    if response.id == to_match:
                        matched_events.append(event_weakref)
        # Now set (awake) and remove from the list any matching event
        for event_weakref in matched_events:
            event_weakref().set(data=response)
            self._custom_response_events.remove(event_weakref)

    def _invalidate_interruptable_events_on_error(self, message=""):
        '''Following robot entering error state, unblock all appropriate interruptable events and have them
        throw InterruptException.'''
        for event_weakref in self._custom_response_events:
            event: InterruptableEvent = event_weakref()
            if event and event._abort_on_error:
                event.abort(message)

    def _invalidate_interruptable_events_on_clear_motion(self, message=""):
        '''Following robot motion cleared, unblock all appropriate interruptable events and have them
        throw InterruptException.'''
        for event_weakref in self._custom_response_events:
            event: InterruptableEvent = event_weakref()
            if event and event._abort_on_clear_motion:
                event.abort(message)

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
                self._cleanup_custom_response_events()
                self._awake_custom_response_events(response)

                if response.id == mx_st.MX_ST_NOT_ACTIVATED:
                    self._invalidate_checkpoints('robot is in error')
                elif response.id == mx_st.MX_ST_ALREADY_ERR:
                    self._invalidate_checkpoints('robot is not activated')

                elif response.id == mx_st.MX_ST_CHECKPOINT_REACHED:
                    self._handle_checkpoint_response(response)

                elif response.id == mx_st.MX_ST_CLEAR_MOTION:
                    if self._clear_motion_requests <= 1:
                        self._clear_motion_requests = 0
                        self._robot_events.on_motion_cleared.set()
                        self._callback_queue.put('on_motion_cleared')
                        # Invalidate checkpoints and appropriate interruptable events
                        message = 'Robot motion was cleared'
                        # Note: We called _invalidate_interruptable_events_on_clear_motion already when ClearMotion
                        #       was called. We don't want to call it on robot response, because checkpoint or
                        #       interruptable event that we have created immediately after calling ClearMotion are
                        #       valid and must not be cleared.
                        #       If robot spontaneously sends MX_ST_CLEAR_MOTION, other condition will awake these
                        #       checkpoints/events anyways (like robot in error, PSTOP2, etc.)
                        #self._invalidate_checkpoints(message)
                        #self._invalidate_interruptable_events_on_clear_motion(message)
                    else:
                        self._clear_motion_requests -= 1

                elif response.id == mx_st.MX_ST_BRAKES_ON:
                    self._set_brakes_engaged(True)

                elif response.id == mx_st.MX_ST_BRAKES_OFF:
                    self._set_brakes_engaged(False)

                elif response.id == mx_st.MX_ST_OFFLINE_START:
                    self._robot_events.on_offline_program_started.set()
                    self._callback_queue.put('on_offline_program_state')

                elif response.id == mx_st.MX_ST_NO_OFFLINE_SAVED:
                    self._robot_events.on_offline_program_started.abort(f"Failed to start program: {response.data}")
                elif response.id == mx_st.MX_ST_OFFLINE_INVALID:
                    self._robot_events.on_offline_program_started.abort(f"Failed to start program: {response.data}")

                elif response.id == mx_st.MX_ST_OFFLINE_PROGRAM_LIST:
                    self._robot_events.on_offline_program_op_done.set(response)
                elif response.id == mx_st.MX_ST_OFFLINE_PROGRAM_LIST_ERR:
                    self._robot_events.on_offline_program_op_done.abort("Failed to list offline programs")
                elif response.id == mx_st.MX_ST_OFFLINE_PROGRAM_LOAD:
                    self._robot_events.on_offline_program_op_done.set(response)
                elif response.id == mx_st.MX_ST_OFFLINE_PROGRAM_LOAD_ERR:
                    self._robot_events.on_offline_program_op_done.abort(
                        f"Failed to load offline program: {response.data}")
                elif response.id == mx_st.MX_ST_OFFLINE_PROGRAM_SAVE:
                    self._robot_events.on_offline_program_op_done.set(response)
                elif response.id == mx_st.MX_ST_OFFLINE_PROGRAM_SAVE_ERR:
                    self._robot_events.on_offline_program_op_done.abort(
                        f"Failed to save offline program: {response.data}")
                elif response.id == mx_st.MX_ST_OFFLINE_PROGRAM_DELETE:
                    self._robot_events.on_offline_program_op_done.set(response)
                elif response.id == mx_st.MX_ST_OFFLINE_PROGRAM_DELETE_ERR:
                    self._robot_events.on_offline_program_op_done.abort(
                        f"Failed to delete offline program: {response.data}")

                else:
                    self._handle_common_messages(response)

    def _handle_common_messages(self, response: Message):
        """Handle response messages which are received on the command and monitor port, and are processed the same way.

        Parameters
        ----------
        response : Message object
            Robot status response to parse and handle.

        """

        # Print error trace if this is an error code
        if response.id in robot_status_code_info:
            code_info = robot_status_code_info[response.id]
            if code_info.is_error:
                self.logger.error(f'Received robot error {code_info.code} ({code_info.name}): {response.data}')
        else:
            self.logger.debug(f'Received unknown robot status code {response.id}')

        #
        # Only update using legacy messages if robot is not capable of rt messages.
        #
        if self._robot_info.rt_message_capable:
            # Temporarily save data if rt messages will be available to add timestamps.
            # Note that if robot platform isn't RT message capable, the update occurs in _handle_common_messages.
            if response.id == mx_st.MX_ST_GET_JOINTS:
                self._tmp_rt_joint_pos = string_to_numbers(response.data)
            elif response.id == mx_st.MX_ST_GET_POSE:
                self._tmp_rt_cart_pos = string_to_numbers(response.data)
            elif response.id == mx_st.MX_ST_RT_CYCLE_END:
                if not self._robot_info.rt_message_capable:
                    self._robot_info.rt_message_capable = True
                timestamp = int(response.data)

                # Useful to detect end of cycle for logging, to start logging on more consistent moment
                self._robot_events.on_end_of_cycle.set()
                self._callback_queue.put('on_end_of_cycle')

                # Update joint and pose with legacy messages from current cycle plus the timestamps we just received
                if self._tmp_rt_joint_pos:
                    self._robot_rt_data.rt_target_joint_pos.update_from_data(timestamp, self._tmp_rt_joint_pos)
                    self._tmp_rt_joint_pos = None
                if self._tmp_rt_cart_pos:
                    self._robot_rt_data.rt_target_cart_pos.update_from_data(timestamp, self._tmp_rt_cart_pos)
                    self._tmp_rt_cart_pos = None

                # If logging is active, log the current state.
                if self._file_logger:
                    self._file_logger.write_fields(timestamp, self._robot_rt_data)
                self._make_stable_rt_data()
        else:  # not self._robot_info.rt_message_capable
            if response.id == mx_st.MX_ST_GET_JOINTS:
                self._robot_rt_data.rt_target_joint_pos.data = string_to_numbers(response.data)
                self._robot_rt_data.rt_target_joint_pos.enabled = True
                if self._is_in_sync():
                    self._robot_events.on_joints_updated.set()

            elif response.id == mx_st.MX_ST_GET_POSE:
                self._robot_rt_data.rt_target_cart_pos.data = string_to_numbers(response.data)
                self._robot_rt_data.rt_target_cart_pos.enabled = True
                if self._is_in_sync():
                    self._robot_events.on_pose_updated.set()

            elif response.id == mx_st.MX_ST_GET_CONF:
                self._robot_rt_data.rt_target_conf.data = string_to_numbers(response.data)
                self._robot_rt_data.rt_target_conf.enabled = True

            elif response.id == mx_st.MX_ST_GET_CONF_TURN:
                self._robot_rt_data.rt_target_conf_turn.data = string_to_numbers(response.data)
                self._robot_rt_data.rt_target_conf_turn.enabled = True

        #
        # Handle various responses/events that we're interested into
        #
        if response.id == mx_st.MX_ST_GET_STATUS_ROBOT:
            self._handle_robot_status_response(response)

        if response.id == mx_st.MX_ST_GET_MOTION_STATUS:
            self._handle_motion_status_response(response)

        elif response.id == mx_st.MX_ST_GET_ROBOT_SERIAL:
            self._handle_robot_get_robot_serial_response(response)

        elif response.id == mx_st.MX_ST_GET_STATUS_GRIPPER:
            self._handle_gripper_status_response(response)

        elif response.id == mx_st.MX_ST_GET_EXT_TOOL_FW_VERSION:
            self._handle_ext_tool_fw_version(response)

        elif response.id == mx_st.MX_ST_RT_EXTTOOL_STATUS:
            self._handle_external_tool_status_response(response)

        elif response.id == mx_st.MX_ST_RT_VALVE_STATE:
            self._handle_valve_state_response(response)

        elif response.id == mx_st.MX_ST_RT_GRIPPER_STATE:
            self._handle_gripper_state_response(response)

        elif response.id == mx_st.MX_ST_RT_GRIPPER_FORCE:
            self._robot_rt_data.rt_gripper_force.update_from_csv(response.data)

        elif response.id == mx_st.MX_ST_RT_GRIPPER_POS:
            self._robot_rt_data.rt_gripper_pos.update_from_csv(response.data)

        elif response.id == mx_st.MX_ST_EXTTOOL_SIM:
            if not str(response.data).isdigit():
                # Legacy response without the tool type argument
                self._handle_ext_tool_sim_status(MxExtToolType.MX_EXT_TOOL_MEGP25_SHORT)
            else:
                self._handle_ext_tool_sim_status(int(response.data))

        elif response.id == MX_ST_EXTTOOL_SIM_OFF:
            self._handle_ext_tool_sim_status(
                MxExtToolType.MX_EXT_TOOL_NONE)  # Legacy response (used by 8.4.4 and older)

        elif response.id == mx_st.MX_ST_RECOVERY_MODE_ON:
            self._handle_recovery_mode_status(True)

        elif response.id == mx_st.MX_ST_RECOVERY_MODE_OFF:
            self._handle_recovery_mode_status(False)

        elif response.id == mx_st.MX_ST_PSTOP2:
            self._robot_status.pstop2State = self._parse_response_int(response)[0]
            if self._robot_status.pstop2State == MxStopState.MX_STOP_STATE_ACTIVE:
                self._robot_events.on_pstop2_reset.clear()
                self._robot_events.on_pstop2.set()
                self._robot_events.on_pstop2_resettable.clear()
                self._callback_queue.put('on_pstop2')
                # Invalidate checkpoints and appropriate interruptable events
                message = 'Robot is in PSTOP2 condition'
                self._invalidate_checkpoints(message)
                self._invalidate_interruptable_events_on_clear_motion(message)
            elif self._robot_status.pstop2State == MxStopState.MX_STOP_STATE_RESETTABLE:
                self._robot_events.on_pstop2_resettable.set()
                self._callback_queue.put('on_pstop2_resettable')
            else:
                self._robot_events.on_pstop2.clear()
                self._robot_events.on_pstop2_reset.set()
                self._callback_queue.put('on_pstop2_reset')

        elif response.id == mx_st.MX_ST_ESTOP:
            self._robot_status.estopState = self._parse_response_int(response)[0]
            if self._robot_status.estopState == MxStopState.MX_STOP_STATE_ACTIVE:
                self._robot_events.on_estop_reset.clear()
                self._robot_events.on_estop.set()
                self._callback_queue.put('on_estop')
            else:
                self._robot_events.on_estop.clear()
                self._robot_events.on_estop_reset.set()
                self._callback_queue.put('on_estop_reset')

        elif response.id == mx_st.MX_ST_RT_TARGET_JOINT_POS:
            self._robot_rt_data.rt_target_joint_pos.update_from_csv(response.data)
            if self._is_in_sync():
                self._robot_events.on_joints_updated.set()

        elif response.id == mx_st.MX_ST_RT_TARGET_CART_POS:
            self._robot_rt_data.rt_target_cart_pos.update_from_csv(response.data)
            if self._is_in_sync():
                self._robot_events.on_pose_updated.set()

        elif response.id == mx_st.MX_ST_RT_TARGET_JOINT_VEL:
            self._robot_rt_data.rt_target_joint_vel.update_from_csv(response.data)
        elif response.id == mx_st.MX_ST_RT_TARGET_CART_VEL:
            self._robot_rt_data.rt_target_cart_vel.update_from_csv(response.data)

        elif response.id == mx_st.MX_ST_RT_TARGET_JOINT_TORQ:
            self._robot_rt_data.rt_target_joint_torq.update_from_csv(response.data)

        elif response.id == mx_st.MX_ST_RT_TARGET_CONF:
            self._robot_rt_data.rt_target_conf.update_from_csv(response.data)
        elif response.id == mx_st.MX_ST_RT_TARGET_CONF_TURN:
            self._robot_rt_data.rt_target_conf_turn.update_from_csv(response.data)

        elif response.id == mx_st.MX_ST_RT_JOINT_POS:
            self._robot_rt_data.rt_joint_pos.update_from_csv(response.data)
        elif response.id == mx_st.MX_ST_RT_CART_POS:
            self._robot_rt_data.rt_cart_pos.update_from_csv(response.data)
        elif response.id == mx_st.MX_ST_RT_JOINT_VEL:
            self._robot_rt_data.rt_joint_vel.update_from_csv(response.data)
        elif response.id == mx_st.MX_ST_RT_JOINT_TORQ:
            self._robot_rt_data.rt_joint_torq.update_from_csv(response.data)
        elif response.id == mx_st.MX_ST_RT_CART_VEL:
            self._robot_rt_data.rt_cart_vel.update_from_csv(response.data)

        elif response.id == mx_st.MX_ST_RT_CONF:
            self._robot_rt_data.rt_conf.update_from_csv(response.data)
        elif response.id == mx_st.MX_ST_RT_CONF_TURN:
            self._robot_rt_data.rt_conf_turn.update_from_csv(response.data)

        elif response.id == mx_st.MX_ST_RT_ACCELEROMETER:
            # The data is stored as [timestamp, index, {measurements...}]
            timestamp, index, *measurements = string_to_numbers(response.data)
            # Record accelerometer measurement only if newer.
            if index not in self._robot_rt_data.rt_accelerometer:
                self._robot_rt_data.rt_accelerometer[index] = TimestampedData(timestamp, measurements)
                self._robot_rt_data.rt_accelerometer[index].enabled = True
            if timestamp > self._robot_rt_data.rt_accelerometer[index].timestamp:
                self._robot_rt_data.rt_accelerometer[index].timestamp = timestamp
                self._robot_rt_data.rt_accelerometer[index].data = measurements

        elif response.id == mx_st.MX_ST_RT_ABS_JOINT_POS:
            self._robot_rt_data.rt_abs_joint_pos.update_from_csv(response.data)

        elif response.id == mx_st.MX_ST_RT_WRF:
            self._robot_rt_data.rt_wrf.update_from_csv(response.data)

        elif response.id == mx_st.MX_ST_RT_TRF:
            self._robot_rt_data.rt_trf.update_from_csv(response.data)

        elif response.id == mx_st.MX_ST_RT_CHECKPOINT:
            self._robot_rt_data.rt_checkpoint.update_from_csv(response.data)

        elif response.id == mx_st.MX_ST_IMPOSSIBLE_RESET_ERR:
            message = response.data
            self.logger.error(response.data)
            # Don't abort the event otherwise we can't try later to reset the error because WaitErrorReset
            # remains "aborted" for ever
            # self._robot_events.on_error_reset.abort(message)

        elif response.id == mx_st.MX_ST_GET_REAL_TIME_MONITORING:
            self._handle_get_realtime_monitoring_response(response)

        elif response.id == mx_st.MX_ST_SYNC_CMD_QUEUE:
            self._handle_sync_response(response)

    def _parse_response_bool(self, response: Message) -> list[bool]:
        """ Parse standard robot response, returns array of boolean values
        """
        if response.data.strip() == '':
            return []
        else:
            return [bool(int(x)) for x in response.data.split(',')]

    def _parse_response_int(self, response: Message) -> list[int]:
        """ Parse standard robot response, returns array of integer values
        """
        if response.data.strip() == '':
            return []
        else:
            return [int(x) for x in response.data.split(',')]

    def _set_activated(self, activated: bool):
        """Update the "activated" state of the robot

        Parameters
        ----------
        activated : bool
            Robot is activated or not
        """
        if not self._first_robot_status_received or self._robot_status.activation_state != activated:
            if activated:
                self.logger.info(f'Robot is activated.')
                self._robot_events.on_deactivated.clear()
                self._robot_events.on_activated.set()
                self._set_brakes_engaged(False)
                self._callback_queue.put('on_activated')
            else:
                self.logger.info(f'Robot is deactivated.')
                self._robot_events.on_activated.clear()
                self._robot_events.on_deactivated.set()
                self._set_brakes_engaged(True)
                self._callback_queue.put('on_deactivated')
                # Invalidate checkpoints and appropriate interruptable events
                message = 'Robot was deactivated'
                self._invalidate_checkpoints(message)
                self._invalidate_interruptable_events_on_clear_motion(message)
            self._robot_status.activation_state = activated

    def _set_homed(self, homed: bool):
        """Update the "homed" state of the robot

        Parameters
        ----------
        homed : bool
            Robot is homed or not
        """
        if not self._first_robot_status_received or self._robot_status.homing_state != homed:
            if homed:
                self._robot_events.on_homed.set()
                self._callback_queue.put('on_homed')
            else:
                self._robot_events.on_homed.clear()
            self._robot_status.homing_state = homed

    def _set_sim_mode(self, sim_mode: bool):
        """Update the "sim_mode" state of the robot

        Parameters
        ----------
        sim_mode : bool
            Robot is in sim mode or not
        """
        if not self._first_robot_status_received or self._robot_status.simulation_mode != sim_mode:
            if sim_mode:
                self._robot_events.on_deactivate_sim.clear()
                self._robot_events.on_activate_sim.set()
                self._callback_queue.put('on_activate_sim')
            else:
                self._robot_events.on_activate_sim.clear()
                self._robot_events.on_deactivate_sim.set()
                self._callback_queue.put('on_deactivate_sim')
            self._robot_status.simulation_mode = sim_mode
            if self._robot_events.on_activate_ext_tool_sim.is_set() != self._robot_status.simulation_mode:
                # Sim mode was just disabled -> Also means external tool sim has been disabled
                self._handle_ext_tool_sim_status(self._external_tool_status.sim_tool_type)

    def _set_recovery_mode(self, recovery_mode: bool):
        """Update the "recovery_mode" state of the robot

        Parameters
        ----------
        recovery_mode : bool
            Robot is in recovery mode or not
        """
        if not self._first_robot_status_received or self._robot_status.recovery_mode != recovery_mode:
            if recovery_mode:
                self._robot_events.on_deactivate_recovery_mode.clear()
                self._robot_events.on_activate_recovery_mode.set()
                self._callback_queue.put('on_activate_recovery_mode')
            else:
                self._robot_events.on_activate_recovery_mode.clear()
                self._robot_events.on_deactivate_recovery_mode.set()
                self._callback_queue.put('on_deactivate_recovery_mode')
            self._robot_status.recovery_mode = recovery_mode

    def _set_error_status(self, error_status: bool):
        """Update the "error" state of the robot

        Parameters
        ----------
        error_status : bool
            Robot is in error or not
        """
        if not self._first_robot_status_received or self._robot_status.error_status != error_status:
            if error_status:
                message = "robot is in error"
                self._invalidate_checkpoints(message)
                self._invalidate_interruptable_events_on_error(message)
                self._robot_events.on_error.set()
                self._robot_events.abort_all_on_error(message)
                self._robot_events.on_error_reset.clear()
                self._callback_queue.put('on_error')
            else:
                self._robot_events.clear_abort_all()
                self._robot_events.on_error.clear()
                self._robot_events.on_error_reset.set()
                self._callback_queue.put('on_error_reset')
            self._robot_status.error_status = error_status

    def _set_paused(self, paused: bool):
        """Update the "paused" state of the robot

        Parameters
        ----------
        paused : bool
            Robot is paused or not
        """
        if not self._first_robot_status_received or self._robot_status.pause_motion_status != paused:
            if paused:
                self._robot_events.on_motion_resumed.clear()
                self._robot_events.on_motion_paused.set()
                self._callback_queue.put('on_motion_paused')
            else:
                self._robot_events.on_motion_paused.clear()
                self._robot_events.on_motion_resumed.set()
                self._callback_queue.put('on_motion_resumed')
            self._robot_status.pause_motion_status = paused

    def _set_eob(self, eob: bool):
        """Update the "eob" state of the robot

        Parameters
        ----------
        eob : bool
            Robot is end-of-block or not
        """
        if not self._first_robot_status_received or self._robot_status.end_of_block_status != eob:
            if eob:
                self._robot_events.on_end_of_block.set()
            else:
                self._robot_events.on_end_of_block.clear()
            self._robot_status.end_of_block_status = eob

    def _set_brakes_engaged(self, brakes_engaged: bool):
        """Update the "brakes_engaged" state of the robot

        Parameters
        ----------
        brakes_engaged : bool
            Robot brakes are engaged or not
        """
        self._robot_status.brakes_engaged = brakes_engaged
        if brakes_engaged:
            self._robot_events.on_brakes_deactivated.clear()
            self._robot_events.on_brakes_activated.set()
        else:
            self._robot_events.on_brakes_activated.clear()
            self._robot_events.on_brakes_deactivated.set()

    def _handle_motion_status_response(self, response: Message):
        """Parse robot motion response and update status fields and events.
           Note that this message is normally automatically followed (at least on the monitoring port)
           by robot status (MX_ST_GET_STATUS_ROBOT).

        Parameters
        ----------
        response : Message object
            Motion status response to parse and handle.

        """
        assert response.id == mx_st.MX_ST_GET_MOTION_STATUS
        if response.jsonData:
            # JSON format.
            self._using_json_api = True
            jsonData = response.jsonData[MX_JSON_KEY_DATA]
            self._set_paused(jsonData[MX_JSON_KEY_MOTION_ROBOT_HOLD])
            self._set_eob(jsonData[MX_JSON_KEY_MOTION_ROBOT_EOB])

        # Note: Let's not yet update _first_robot_status_received or on_status_updated.
        #       We'll do that in _handle_robot_status_response since we're expecting to receive robot status
        #       immediately after motion status.

    def _handle_robot_status_response(self, response: Message):
        """Parse robot status response and update status fields and events.

        Parameters
        ----------
        response : Message object
            Robot status response to parse and handle.

        """
        assert response.id == mx_st.MX_ST_GET_STATUS_ROBOT
        if response.jsonData:
            # JSON format.
            self._using_json_api = True
            jsonData = response.jsonData[MX_JSON_KEY_DATA]
            self._set_activated(jsonData[MX_JSON_KEY_STATUS_ROBOT_STATE] >= MxRobotState.MX_ROBOT_STATE_ACTIVATED)
            self._set_homed(jsonData[MX_JSON_KEY_STATUS_ROBOT_STATE] == MxRobotState.MX_ROBOT_STATE_RUN)
            self._set_sim_mode(jsonData[MX_JSON_KEY_STATUS_ROBOT_SIM])
            self._set_recovery_mode(jsonData[MX_JSON_KEY_STATUS_ROBOT_RECOVERY])
            self._set_error_status(jsonData[MX_JSON_KEY_STATUS_ROBOT_ERR] != 0)
            self._set_brakes_engaged(jsonData[MX_JSON_KEY_STATUS_ROBOT_BRAKES] != 0)
        else:
            # Legacy format.
            status_flags = self._parse_response_bool(response)

            self._set_activated(status_flags[0])
            self._set_homed(status_flags[1])
            self._set_sim_mode(status_flags[2])
            self._set_error_status(status_flags[3])
            self._set_paused(status_flags[4])
            self._set_eob(status_flags[5])

        self._first_robot_status_received = True

        if self._is_in_sync():
            self._robot_events.on_status_updated.set()
        self._callback_queue.put('on_status_updated')

    def _handle_robot_get_robot_serial_response(self, response: Message):
        """Parse get robot serial response and robot info.

        Parameters
        ----------
        response : Message object
            GetRobotSerial response to parse and save.

        """
        assert response.id == mx_st.MX_ST_GET_ROBOT_SERIAL

        self._robot_info.serial = response.data

    def _handle_gripper_status_response(self, response: Message):
        """Parse gripper status response and update status fields and events.

        Parameters
        ----------
        response : Message object
            Gripper status response to parse and handle.

        """
        assert response.id == mx_st.MX_ST_GET_STATUS_GRIPPER
        status_flags = self._parse_response_bool(response)

        self._gripper_status.present = status_flags[0]
        self._gripper_status.homing_state = status_flags[1]
        self._gripper_status.holding_part = status_flags[2]
        self._gripper_status.target_pos_reached = status_flags[3]
        self._gripper_status.error_status = status_flags[4]
        self._gripper_status.overload_error = status_flags[5]

        if self._is_in_sync():
            self._robot_events.on_status_gripper_updated.set()
            self._callback_queue.put('on_status_gripper_updated')

    def _handle_ext_tool_sim_status(self, tool_type: int):
        """Handle gripper sim mode status change event.

        Parameters
        ----------
        tool_type : int
            New simulated external tool type. `MxExtToolType.MX_EXT_TOOL_NONE` when simulation is off.

        """
        if tool_type != MxExtToolType.MX_EXT_TOOL_NONE:
            self._robot_events.on_deactivate_ext_tool_sim.clear()
            self._robot_events.on_activate_ext_tool_sim.set()
            self._callback_queue.put('on_activate_ext_tool_sim')

        else:
            self._robot_events.on_activate_ext_tool_sim.clear()
            self._robot_events.on_deactivate_ext_tool_sim.set()
            self._callback_queue.put('on_deactivate_ext_tool_sim')

    def _handle_recovery_mode_status(self, enabled: bool):
        """Handle recovery mode status change event.

        Parameters
        ----------
        enabled : bool
            Recovery mode enabled or not.

        """
        self._set_recovery_mode(enabled)

    def _handle_ext_tool_fw_version(self, response: Message):
        """Parse external tool firmware version"""
        self._robot_info.ext_tool_version.update_version(response.data)

    def _handle_external_tool_status_response(self, response: Message):
        """Parse external tool status response and update status fields and events.

        Parameters
        ----------
        response : Message object
            External tool status response to parse and handle.

        """
        assert response.id == mx_st.MX_ST_RT_EXTTOOL_STATUS
        self._robot_rt_data.rt_external_tool_status.update_from_csv(response.data)
        status_flags = self._robot_rt_data.rt_external_tool_status.data

        self._external_tool_status.sim_tool_type = status_flags[0]
        self._external_tool_status.physical_tool_type = status_flags[1]
        self._external_tool_status.homing_state = status_flags[2]
        self._external_tool_status.error_status = status_flags[3]
        self._external_tool_status.overload_error = status_flags[4]

        if self._is_in_sync():
            self._robot_events.on_external_tool_status_updated.set()
            self._callback_queue.put('on_external_tool_status_updated')

    def _handle_gripper_state_response(self, response: Message):
        """Parse gripper state response and update status fields and events.

        Parameters
        ----------
        response : Message object
            Gripper state response to parse and handle.

        """
        assert response.id == mx_st.MX_ST_RT_GRIPPER_STATE
        self._robot_rt_data.rt_gripper_state.update_from_csv(response.data)
        status_flags = self._robot_rt_data.rt_gripper_state.data

        self._gripper_state.holding_part = bool(status_flags[0])
        self._gripper_state.target_pos_reached = bool(status_flags[1])
        if len(status_flags) > 2:
            self._gripper_state.closed = bool(status_flags[2])
            self._gripper_state.opened = bool(status_flags[3])

        if self._is_in_sync():
            self._robot_events.on_gripper_state_updated.set()
            self._callback_queue.put('on_gripper_state_updated')

    def _handle_valve_state_response(self, response: Message):
        """Parse pneumatic valve state response and update status fields and events.

        Parameters
        ----------
        response : Message object
            Pneumatic valve state response to parse and handle.

        """
        assert response.id == mx_st.MX_ST_RT_VALVE_STATE
        self._robot_rt_data.rt_valve_state.update_from_csv(response.data)

        self._valve_state.valve_opened = self._robot_rt_data.rt_valve_state.data

        if self._is_in_sync():
            self._robot_events.on_valve_state_updated.set()
            self._callback_queue.put('on_valve_state_updated')

    def _handle_checkpoint_response(self, response: Message):
        """Handle the checkpoint message from the robot, set the appropriate events, etc.

        Parameters
        ----------
        response : Message object
            Response message which includes the received checkpoint id.

        """
        assert response.id == mx_st.MX_ST_CHECKPOINT_REACHED
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
            self.logger.warning(
                f'Received un-tracked checkpoint {checkpoint_id}. Please use ExpectExternalCheckpoint() to track.')

    def _handle_get_realtime_monitoring_response(self, response: Message):
        """Parse robot response to "get" or "set" real-time monitoring.
           This function identifies which real-time events are expected, and which are not enabled.


        Parameters
        ----------
        response : Message object
            Robot status response to parse and handle.

        """
        assert response.id == mx_st.MX_ST_GET_REAL_TIME_MONITORING

        # Clear all "enabled" bits in real-time data
        self._robot_rt_data._reset_enabled()

        # Following RT data are always sent by the robot
        self._robot_rt_data.rt_target_joint_pos.enabled = True
        self._robot_rt_data.rt_target_cart_pos.enabled = True
        if self._robot_info.version.is_at_least(9, 0):
            self._robot_rt_data.rt_target_conf.enabled = True
            self._robot_rt_data.rt_target_conf_turn.enabled = True
            self._robot_rt_data.rt_wrf.enabled = True
            self._robot_rt_data.rt_trf.enabled = True

        # Parse the response to identify which are "enabled"
        enabled_event_ids = self._parse_response_int(response)
        for event_id in enabled_event_ids:
            if event_id == mx_st.MX_ST_RT_TARGET_JOINT_POS:
                self._robot_rt_data.rt_target_joint_pos.enabled = True
            if event_id == mx_st.MX_ST_RT_TARGET_CART_POS:
                self._robot_rt_data.rt_target_cart_pos.enabled = True
            if event_id == mx_st.MX_ST_RT_TARGET_JOINT_VEL:
                self._robot_rt_data.rt_target_joint_vel.enabled = True
            if event_id == mx_st.MX_ST_RT_TARGET_CART_VEL:
                self._robot_rt_data.rt_target_cart_vel.enabled = True
            if event_id == mx_st.MX_ST_RT_TARGET_JOINT_TORQ:
                self._robot_rt_data.rt_target_joint_torq.enabled = True
            if event_id == mx_st.MX_ST_RT_TARGET_CONF:
                self._robot_rt_data.rt_target_conf.enabled = True
            if event_id == mx_st.MX_ST_RT_TARGET_CONF_TURN:
                self._robot_rt_data.rt_target_conf_turn.enabled = True
            if event_id == mx_st.MX_ST_RT_JOINT_POS:
                self._robot_rt_data.rt_joint_pos.enabled = True
            if event_id == mx_st.MX_ST_RT_CART_POS:
                self._robot_rt_data.rt_cart_pos.enabled = True
            if event_id == mx_st.MX_ST_RT_JOINT_VEL:
                self._robot_rt_data.rt_joint_vel.enabled = True
            if event_id == mx_st.MX_ST_RT_JOINT_TORQ:
                self._robot_rt_data.rt_joint_torq.enabled = True
            if event_id == mx_st.MX_ST_RT_CART_VEL:
                self._robot_rt_data.rt_cart_vel.enabled = True
            if event_id == mx_st.MX_ST_RT_CONF:
                self._robot_rt_data.rt_conf.enabled = True
            if event_id == mx_st.MX_ST_RT_CONF_TURN:
                self._robot_rt_data.rt_conf_turn.enabled = True
            if event_id == mx_st.MX_ST_RT_WRF:
                self._robot_rt_data.rt_wrf.enabled = True
            if event_id == mx_st.MX_ST_RT_TRF:
                self._robot_rt_data.rt_trf.enabled = True
            if event_id == mx_st.MX_ST_RT_CHECKPOINT:
                self._robot_rt_data.rt_checkpoint.enabled = True
            if event_id == mx_st.MX_ST_RT_ACCELEROMETER:
                for accelerometer in self._robot_rt_data.rt_accelerometer.values():
                    accelerometer.enabled = True
            if event_id == mx_st.MX_ST_RT_ABS_JOINT_POS:
                self._robot_rt_data.rt_abs_joint_pos.enabled = True
            if event_id == mx_st.MX_ST_RT_EXTTOOL_STATUS:
                self._robot_rt_data.rt_external_tool_status.enabled = True
            if event_id == mx_st.MX_ST_RT_VALVE_STATE:
                self._robot_rt_data.rt_valve_state.enabled = True
            if event_id == mx_st.MX_ST_RT_GRIPPER_FORCE:
                self._robot_rt_data.rt_gripper_force.enabled = True
            if event_id == mx_st.MX_ST_RT_GRIPPER_POS:
                self._robot_rt_data.rt_gripper_pos.enabled = True

        # Make sure to clear values that we should no more received
        self._robot_rt_data._clear_if_disabled()

    def _handle_sync_response(self, response: Message):
        """Parse robot response to "SyncCmdQueue" request
           This class uses the "SyncCmdQueue" request/response to ensure synchronous "Get" operations have received the
           expected response from the robot (and not a response/event sent by the robot prior to our "Get" request).

        Parameters
        ----------
        response : Message object
            Sync response to parse and handle.

        """
        assert response.id == mx_st.MX_ST_SYNC_CMD_QUEUE

        self._rx_sync = string_to_numbers(response.data)[0]

    def _is_in_sync(self) -> bool:
        """Tells if we're in sync with the latest "get" operation (i.e. we've received the response to the most recent
           "SyncCmdQueue" request to the robot, meaning that the "get" response we just got is up-to-date)

        Returns
        -------
        bool
            True if "in sync" ("get" response we just received matches the "get" request we've just made)
        """
        return (self._rx_sync == self._tx_sync)
