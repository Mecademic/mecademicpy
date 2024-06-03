#!/usr/bin/env python3
"""
This file contains the internal implementation for the Robot class (from robot.py).
We have separated the public API from the internal implementation to make the public api (robot.py) more lean.
"""
# pylint: disable=attribute-defined-outside-init
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
from typing import Callable, Optional, Tuple, Union

import requests

import mecademicpy._robot_trajectory_logger as mx_traj

# pylint: disable=wildcard-import,unused-wildcard-import,invalid-name
from .mx_robot_def import MxRobotStatusCode as mx_st
from .mx_robot_def import *
from .robot_classes import *
from .robot_trajectory_files import RobotTrajectories
from .tools import *

_CHECKPOINT_ID_MAX_PRIVATE = 8191  # Max allowable checkpoint id, inclusive

_TERMINATE = '--terminate--'

_MONITORING_TIMEOUT_MS = 10000  # Max time without receiving any monitoring message from the robot
_UPDATE_TIMEOUT = 15 * 60  # 15 minutes timeout


def disconnect_on_exception_decorator(func):
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
        # pylint: disable=broad-exception-caught
        except BaseException as e:
            # pylint: disable=protected-access
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
    on_network_config_updated : event
        Set if robot network configuration is updated.
    on_status_gripper_updated : event
        Set if gripper status is updated.
    on_external_tool_status_updated: event
        Set if external tool status has been updated.
    on_gripper_state_updated: event
        Set if gripper state has been updated.
    on_valve_state_updated: event
        Set if pneumatic module valve state has been updated.
    on_output_state_updated: event
        Set if digital outputs state has been updated.
    on_input_state_updated: event
        Set if digital inputs state has been updated.
    on_vacuum_state_updated: event
        Set if IO expansion module's vacuum module state has been updated.
    on_holding_part: event
        Set if gripper or IO expansion's vacuum gripper is holding a part.
    on_released_part: event
        Set if gripper or IO expansion's vacuum gripper is not holding a part.
    on_vacuum_purge_done: event
        Set when the vacuum gripper is not purging.
    on_io_status_updated: event
        Set if IO expansion module's status has been updated.
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
    on_safety_stop_reset : event
        Set if all safety signals that require the power supply Reset function (estop and pstop1) are reset.
    on_safety_stop_resettable : event
        Set if the power supply Reset function is currently available to reset safety signals (estop and pstop1).
    on_safety_stop_state_change : event
        Set if any safety stop status changes (see RobotSafetyStatus)
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
    on_io_sim_enabled : event
        Set if robot IO module is not in sim mode.
    on_io_sim_disabled : event
        Set if robot IO module is in sim mode.
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
    on_time_scaling_changed: event
        Set if time scaling has changed.
"""

    def __init__(self):
        self.on_connected = InterruptableEvent()
        self.on_disconnected = InterruptableEvent()

        self.on_status_updated = InterruptableEvent()
        self.on_network_config_updated = InterruptableEvent()

        self.on_status_gripper_updated = InterruptableEvent()
        self.on_external_tool_status_updated = InterruptableEvent()
        self.on_gripper_state_updated = InterruptableEvent()
        self.on_valve_state_updated = InterruptableEvent()
        self.on_output_state_updated = InterruptableEvent()
        self.on_input_state_updated = InterruptableEvent()
        self.on_vacuum_state_updated = InterruptableEvent()
        self.on_holding_part = InterruptableEvent()
        self.on_released_part = InterruptableEvent()
        self.on_vacuum_purge_done = InterruptableEvent()
        self.on_io_status_updated = InterruptableEvent()

        self.on_activated = InterruptableEvent()
        self.on_deactivated = InterruptableEvent()

        self.on_homed = InterruptableEvent()

        self.on_error = InterruptableEvent()
        self.on_error_reset = InterruptableEvent()

        # Safety stop support
        self.on_safety_stop_reset = InterruptableEvent()
        self.on_safety_stop_resettable = InterruptableEvent()
        self.on_safety_stop_state_change = InterruptableEvent()

        # Support for deprecated safety stop methods
        self.on_pstop2 = InterruptableEvent()
        self.on_pstop2_resettable = InterruptableEvent()
        self.on_pstop2_reset = InterruptableEvent()
        self.on_estop = InterruptableEvent()
        self.on_estop_reset = InterruptableEvent()
        self.on_estop_resettable = InterruptableEvent()

        self.on_motion_paused = InterruptableEvent()
        self.on_motion_resumed = InterruptableEvent()
        self.on_motion_cleared = InterruptableEvent()

        self.on_activate_sim = InterruptableEvent()
        self.on_deactivate_sim = InterruptableEvent()

        self.on_activate_ext_tool_sim = InterruptableEvent()
        self.on_deactivate_ext_tool_sim = InterruptableEvent()

        self.on_io_sim_enabled = InterruptableEvent()
        self.on_io_sim_disabled = InterruptableEvent()

        self.on_activate_recovery_mode = InterruptableEvent()
        self.on_deactivate_recovery_mode = InterruptableEvent()

        self.on_joints_updated = InterruptableEvent()
        self.on_pose_updated = InterruptableEvent()

        self.on_brakes_activated = InterruptableEvent()
        self.on_brakes_deactivated = InterruptableEvent()

        self.on_offline_program_started = InterruptableEvent()
        self.on_offline_program_op_done = InterruptableEvent()

        self.on_time_scaling_changed = InterruptableEvent()

        self.on_end_of_block = InterruptableEvent()
        self.on_end_of_cycle = InterruptableEvent()

        self.on_disconnected.set()
        self.on_deactivated.set()
        self.on_error_reset.set()

        self.on_safety_stop_reset.set()
        self.on_safety_stop_resettable.set()
        self.on_safety_stop_state_change.set()
        self.on_pstop2_reset.set()
        self.on_pstop2_resettable.set()
        self.on_estop_reset.set()
        self.on_estop_resettable.set()

        self.on_motion_resumed.set()
        self.on_deactivate_sim.set()

        self.on_time_scaling_changed.set()

        self.on_status_updated.set()
        self.on_network_config_updated.set()
        self.on_status_gripper_updated.set()
        self.on_external_tool_status_updated.set()
        self.on_gripper_state_updated.set()
        self.on_valve_state_updated.set()
        self.on_output_state_updated.set()
        self.on_input_state_updated.set()
        self.on_vacuum_state_updated.set()
        self.on_holding_part.set()
        self.on_released_part.set()
        self.on_vacuum_purge_done.set()
        self.on_io_status_updated.set()

        self.on_joints_updated.set()
        self.on_pose_updated.set()
        self.on_brakes_activated.set()

    def clear_all(self):
        """Clear all events.

        """
        for interruptable_event in self.__dict__.values():
            interruptable_event.clear()

    def abort_all(self, skipped_events: list[str] = None, message=""):
        """Abort all events, except for events in skipped_events list.

        """
        for event_name, interruptable_event in self.__dict__.items():
            if not skipped_events or event_name not in skipped_events:
                interruptable_event.abort(message)

    def abort_all_on_error(self, message=""):
        """Abort all events in the specific case where the robot has reported an error.

        """
        self.abort_all(
            skipped_events=[
                'on_connected',  # Don't abort a wait for "on_connected" (should be done by now anyways)
                'on_deactivated',  # Don't abort waiting for "on_deactivated". WaitDeactivated is can be used in error
                'on_status_updated',  # Don't abort a wait for "on_status_updated", that's what we're doing!
                'on_network_config_updated',  # Don't abort a wait for "on_network_config_updated", available in error
                'on_error_reset',  # Don't abort a wait for "on_error_reset" because we got an error
                'on_end_of_cycle',  # Don't abort a wait for "on_end_of_cycle", cycles should continue during error
                'on_activate_recovery_mode',  # Don't abort wait for "on_activate_recovery_mode", available in error
                'on_deactivate_recovery_mode',  # Don't abort wait for "on_deactivate_recovery_mode", available in error
                'on_safety_stop_resettable',  # The "Safety stop resettable" state may still change while in error
                'on_safety_stop_reset',  # The Safety stop reset state may still change while in error
                'on_safety_stop_state_change',  # The "Safety stop resettable" state may still change while in error
                'on_estop_reset',  # The EStop state may still change while in error
                'on_pstop2_resettable',  # The "PStop2 resettable" state may still change while in error
                'on_pstop2_reset',  # The PStop2 state may still change while in error
                'on_estop_resettable',  # The "EStop resettable" state may still change while in error
                'on_estop_reset',  # The EStop state may still change while in error
                'on_time_scaling_changed',  # The set time scaling still works in error
            ],
            message=message)

    def clear_abort_all(self):
        """Clear aborts for all events.

        """
        for interruptable_event in self.__dict__.values():
            interruptable_event.clear_abort()


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

    def register(self, callback_name):
        """ Register a callback so it's enabled"""
        if not callback_name in self._registered_callbacks:
            self._registered_callbacks.add(callback_name)

    def unregister(self, callback_name):
        """ Unregister a callback so it's no longer called"""
        if callback_name in self._registered_callbacks:
            self._registered_callbacks.remove(callback_name)

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
    _rx_timestamp : float
        Last time a message was received from the robot

    _command_response_handler_thread : thread handle
        Thread used to read messages from the command response queue.
    _monitor_handler_thread : thread handle
        Thread used to read messages from the monitor queue.

    _main_lock : recursive lock object
        Used to protect internal state of the robot object.

    _robot_info: RobotInfo object
        Store information concerning robot (ex.: serial number)
    _robot_rt_data : RobotRtData object
        Stores current robot real-time data.
        All attributes of this object are the latest captured on monitor port, so they don't necessarily share the same
        timestamp
    _robot_rt_data_stable : RobotRtData object
        Stores current robot real-time data, but all attributes of object share the same timestamp
    _robot_status: RobotStatus object
        Stores current robot status
    _robot_safety_status: RobotSafetyStatus object
        Stores current robot safety status
    _robot_psu_inputs: RobotPowerSupplyInputs object
        Stores current robot power supply input states
    _robot_collision_status: CollisionStatus
        Stores current collision status (self collision and work zone boundary)
    _gripper_status: GripperStatus object
        Stores current gripper status
    _external_tool_status: ExtToolStatus object
        Stores current external tool status
    _gripper_state: GripperState object
        Stores current gripper state
    _gripper_state_before_last_move : GripperState object
        Stores gripper state at the time a Open/Close/Move gripper command is sent
        (then used to accelerate WaitGripperMoveCompletion in case target_pos_reached is already True when its called
         because otherwise it's not possible to know if the move has completed, or not yet started)
    _valve_state: ValveState object
        Stores current pneumatic valve state
    _psu_io_status: IoStatus object
        Stores current PSU IO module status
    _io_module_status: IoStatus object
        Stores current IO module status
    _vacuum_state: IoStatus object
        Stores current IO module's vacuum gripper state
    _sig_gen_status: IoStatus object
        Stores current signal generator status
    _fw_update_status: Firmware update status
        Stores current firmware update status reported by the robot
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
        self.logger = logging.getLogger(__name__)
        self._is_initialized = False
        # (callbacks remain registered after "with" block
        self._robot_callbacks = RobotCallbacks()
        self._run_callbacks_in_separate_thread = False
        self._reset()

        # Build a dictionary of robot commands that, when sending them, we need to update internal states.
        # We do it like this to catch code using "SendCustomCommand" instead of calling the official public API
        # (for example: robot.SendCustomCommand("ActivateRobot(1)") needs to behave like robot.ActivateRobot)
        self.cmd_handler_dict: dict[str, Callable] = {
            'activaterobot': self._sending_ActivateRobot,
            'deleteprogram': self._sending_DeleteProgram,
            'clearmotion': self._sending_ClearMotion,
            'listprograms': self._sending_ListPrograms,
            'loadprogram': self._sending_LoadProgram,
            'home': self._sending_Home,
            'saveprogram': self._sending_SaveProgram,
            'setmonitoringinterval': self._sending_SetMonitoringInterval,
            'settimescaling': self._sending_SetTimeScaling,
            'startprogram': self._sending_StartProgram,
        }

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
        self._port = None

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

        self._network_config = NetworkConfig()

        self._robot_info = RobotInfo()
        self._robot_rt_data = None
        self._robot_rt_data_stable = None
        self._robot_status = RobotStatus()
        self._robot_safety_status = RobotSafetyStatus()
        self._robot_psu_inputs = RobotPowerSupplyInputs()
        self._robot_collision_status = CollisionStatus()
        self._first_robot_status_received = False
        self._using_legacy_json_api = False
        self._gripper_status = GripperStatus()
        self._external_tool_status = ExtToolStatus()
        self._gripper_state = GripperState()
        self._gripper_state_before_last_move = GripperState()
        self._valve_state = ValveState()
        self._psu_io_status = IoStatus()
        self._io_module_status = IoStatus()
        self._vacuum_state = VacuumState()
        self._sig_gen_status = IoStatus()
        self._robot_events = _RobotEvents()
        self._reset_fw_update_status()

        self._file_logger = None
        self._monitoring_interval = None
        self._monitoring_interval_to_restore = None
        self._auto_connection_watchdog = False
        self._auto_connection_watchdog_last = 0

        self._reset_disconnect_attributes()

        self._enable_synchronous_mode = None
        self._disconnect_on_exception = None

        self._offline_mode = None
        self._monitor_mode = None

        self.default_timeout = DEFAULT_WAIT_TIMEOUT

        # Variables to hold joint positions and poses while waiting for timestamp.
        self._tmp_rt_joint_pos = None
        self._tmp_rt_cart_pos = None

        self._is_sync = InterruptableEvent()
        self._is_sync.set()
        self._tx_sync = 0
        self._tx_sync_pending = 0
        self._rx_sync = 0

        self._captured_trajectory = None

        self._is_initialized = True

    def _reset_disconnect_attributes(self):
        """ Reset class attributes that need to be reset following a disconnection with the robot """
        self._command_rx_queue = queue.Queue()
        self._command_tx_queue = queue.Queue()
        self._monitor_rx_queue = queue.Queue()
        self._rx_timestamp = 0
        self._monitor_timeout_used = False
        self._custom_response_events = list()

        self._user_checkpoints = dict()
        self._internal_checkpoints = dict()
        self._internal_checkpoint_counter = MX_CHECKPOINT_ID_MAX + 1

        self._clear_motion_requests = 0

    def _reset_fw_update_status(self):
        self._fw_update_status = UpdateProgress()
        self._fw_update_reboot_timestamp = 0.0  # Timestamp, non-zero while rebooting
        self._fw_update_started: bool = False
        self._fw_update_reboot_done: bool = False

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
    #pylint: disable=unused-argument
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

                # #logger.debug(f'Socket Rx - Response: {response}')
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
    def _connect_socket(logger: logging.Logger, address: str, port: int, socket_timeout=1.0) -> socket.socket:
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

        loop_timeout = socket_timeout / 10
        if loop_timeout < 0.1:
            loop_timeout = 0.1
        start_time = time.monotonic()
        while True:
            # Create socket and attempt connection.
            new_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            new_socket.settimeout(loop_timeout)
            try:
                new_socket.connect((address, port))
                break
            except (socket.timeout, TimeoutError) as exception:
                if (time.monotonic() - start_time) < socket_timeout:
                    logger.debug(f'Timeout connecting to {address}:{port}, retrying now.')
                else:
                    raise TimeoutError(
                        f'Unable to connect to {address}:{port} after {socket_timeout}s, exception: {exception}'
                    ) from exception
            except ConnectionRefusedError as exception:
                if (time.monotonic() - start_time) < socket_timeout:
                    logger.debug(f'Connection refused to {address}:{port}, retrying in {loop_timeout} second.')
                    time.sleep(loop_timeout)
                else:
                    raise ConnectionRefusedError(
                        f'Unable to connect to {address}:{port}, exception: {exception}') from exception

        # Set final socket timeout
        new_socket.settimeout(socket_timeout)
        return new_socket

    def _handle_callbacks(self, polling=False):
        """Runs callbacks found in callback_queue."""

        is_connected = True
        while True:

            # If we are not blocking on empty, return if empty.
            if polling and self._callback_queue.qsize() == 0:
                return

            try:
                # Call blocking with timeout of 0.1.
                # If polling:
                #   we know this won't block since there is something in the queue (validated above).
                # If not polling (i.e. we're the callback thread)
                #   then check with short timeout and ignore the error. We do this to periodically check if the
                #   connection was dropped, so we detect a disconnection and trigger "on_disconnected" callback
                #   even in the case the app is never calling IsConnected
                callback_name, data = self._callback_queue.get(block=True, timeout=0.1)
            # pylint: disable=broad-exception-caught
            except Exception:
                # Check if still connected
                if is_connected:
                    is_connected = self.IsConnected()
                    # Continue even if no more connected so we run one last loop and get the chance to call
                    # the "on_disconnected" callback that we may just have pushed to the queue
                    continue
                else:
                    break

            if callback_name == _TERMINATE:
                return

            callback_function = self._robot_callbacks.__dict__[callback_name]
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

    def RegisterCallback(self, callback_name: str, callback_method: Callable[[], None]):
        """See documentation in equivalent function in robot.py"""
        if not hasattr(self._robot_callbacks, callback_name):
            raise ValueError(f'Unknown callback name {callback_name}')
        setattr(self._robot_callbacks, callback_name, callback_method)
        self._callback_queue.register(callback_name)

    def UnregisterCallbacks(self):
        """See documentation in equivalent function in robot.py"""
        self._stop_callback_thread()

        self._robot_callbacks = RobotCallbacks()
        self._callback_queue = _CallbackQueue(self._robot_callbacks)
        if not self.IsConnected():
            self._run_callbacks_in_separate_thread = False
            self._callback_thread = None

    def UnregisterCallback(self, callback_name: str):
        """See documentation in equivalent function in robot.py"""
        if not hasattr(self._robot_callbacks, callback_name):
            raise ValueError(f'Unknown callback name {callback_name}')
        setattr(self._robot_callbacks, callback_name, None)
        self._callback_queue.unregister(callback_name)

    def _start_callback_thread(self):
        if self._run_callbacks_in_separate_thread and self._callback_thread is None:
            self._callback_thread = threading.Thread(target=self._handle_callbacks)
            self._callback_thread.daemon = True  # Make sure thread does not prevent application from quitting
            self._callback_thread.start()

    def _stop_callback_thread(self):
        if self._callback_thread:
            self._callback_queue.put(_TERMINATE)
            if self._callback_thread != threading.current_thread():
                self._callback_thread.join(timeout=self.default_timeout)
                self._callback_thread = None

    # Robot control functions.

    def Connect(self,
                address: str = MX_DEFAULT_ROBOT_IP,
                enable_synchronous_mode: bool = False,
                disconnect_on_exception: bool = True,
                monitor_mode: bool = False,
                offline_mode: bool = False,
                timeout: float = 1.0):
        """See documentation in equivalent function in robot.py"""
        try:
            with self._main_lock:

                if self.IsConnected():
                    return  # Still connected -> Do nothing

                # Check that the ip address is valid and set address.
                if not isinstance(address, str):
                    raise TypeError(f'Invalid IP address ({address}).')

                self._enable_synchronous_mode = enable_synchronous_mode
                self._disconnect_on_exception = disconnect_on_exception
                self._offline_mode = offline_mode
                self._monitor_mode = monitor_mode

                # Check if user has specified a custom port to connect to
                addr_port = address.split(':')
                mode_str = "monitoring mode" if self._monitor_mode else "control mode"
                if len(addr_port) > 1:
                    self._port = int(addr_port[1])
                    address = addr_port[0]
                else:
                    self._port = MX_ROBOT_TCP_PORT_FEED if self._monitor_mode else MX_ROBOT_TCP_PORT_CONTROL

                if self._fw_update_reboot_timestamp == 0:  # Don't print this trace when reconnecting during update
                    self.logger.info(f"Connecting to robot {address}:{self._port} ({mode_str})")

                ipaddress.ip_address(address)
                self._address = address

                # Reset robot status to default values
                self._robot_status = RobotStatus()
                self._robot_safety_status = RobotSafetyStatus()
                self._robot_psu_inputs = RobotPowerSupplyInputs()
                self._robot_collision_status = CollisionStatus()
                self._first_robot_status_received = False
                self._using_legacy_json_api = False

                if not self._monitor_mode:
                    self._initialize_command_socket(timeout)
                    self._initialize_command_connection()
                else:
                    # Activate the monitoring timeout to detect if we're no more receiving anything from the robot
                    self._monitor_timeout_used = True

                self._robot_events.clear_all()

                self._robot_events.on_deactivated.set()
                self._robot_events.on_error_reset.set()

                self._robot_events.on_safety_stop_reset.set()
                self._robot_events.on_safety_stop_resettable.set()
                self._robot_events.on_safety_stop_state_change.set()
                self._set_robot_operation_mode(MxRobotOperationMode.MX_ROBOT_OPERATION_MODE_AUTO)
                self._set_reset_ready(False)
                self._robot_events.on_pstop2_reset.set()
                self._robot_events.on_pstop2_resettable.set()
                self._robot_events.on_estop_reset.set()
                self._robot_events.on_estop_resettable.set()

                self._robot_events.on_motion_resumed.set()
                self._set_brakes_engaged(True)
                self._set_connection_watchdog_enabled(False)

                self._robot_events.on_status_updated.set()
                self._robot_events.on_network_config_updated.set()
                self._robot_events.on_status_gripper_updated.set()
                self._robot_events.on_external_tool_status_updated.set()
                self._robot_events.on_gripper_state_updated.set()
                self._robot_events.on_valve_state_updated.set()
                self._robot_events.on_output_state_updated.set()
                self._robot_events.on_input_state_updated.set()
                self._robot_events.on_vacuum_state_updated.set()
                self._robot_events.on_holding_part.set()
                self._robot_events.on_released_part.set()
                self._robot_events.on_vacuum_purge_done.set()
                self._robot_events.on_io_status_updated.set()

                self._robot_events.on_joints_updated.set()
                self._robot_events.on_pose_updated.set()

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
                        if isinstance(monitoring_interval_response, Message):
                            self._monitoring_interval = float(f'{monitoring_interval_response.data}')
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
                        # Since we're going to receive monitoring messages on the control port, we'll periodically
                        # receive events from the robot and can thus enable the "monitor timeout" to detect that
                        # we're disconnected from the robot if ever we no more receive any message from it
                        self._monitor_timeout_used = True
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

            self.logger.info(f'{self._robot_info}')  # Note: This will print "Connected to..."

            if self._robot_info.version.major < 8:
                self.logger.warning('Python API not supported for firmware under version 8')

            self._robot_events.on_connected.set()
            self._callback_queue.put('on_connected')

            # Start callback thread if necessary
            self._start_callback_thread()
        except Exception as e:
            if self._fw_update_reboot_timestamp == 0:  # Don't print this trace when reconnecting during firmware update
                self.logger.info(f'Failed to connect: {e}')
            self._disconnect()
            raise

    def Disconnect(self):
        """See documentation in equivalent function in robot.py"""
        if self.IsConnected():
            if self._fw_update_reboot_timestamp == 0:  # Don't print this trace when reconnecting during firmware update
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
            self._invalidate_checkpoints(message, forced=True)
            self._invalidate_interruptable_events_on_clear_motion(message)

            # Reset attributes which should not persist after disconnect.
            self._reset_disconnect_attributes()

            # Finally, close sockets.
            if self._command_socket is not None:
                try:
                    self._command_socket.close()
                # pylint: disable=broad-exception-caught
                except Exception as e:
                    self.logger.error(f'Error closing command socket: {e}')
                self._command_socket = None
            if self._monitor_socket is not None:
                try:
                    self._monitor_socket.close()
                # pylint: disable=broad-exception-caught
                except Exception as e:
                    self.logger.error(f'Error closing monitoring socket: {e}')
                self._monitor_socket = None

            self._robot_events.on_connected.clear()
            self._robot_events.on_disconnected.set()
            self._callback_queue.put('on_disconnected')

            self._robot_events.abort_all(message=message)

        # Now that we're disconnected and posted 'on_disconnected' callback we can stop the callback thread
        self._stop_callback_thread()

    def IsConnected(self) -> bool:
        """See documentation in equivalent function in robot.py"""
        if self._robot_events.on_connected.is_set():
            try:
                self._check_internal_states(refresh_monitoring_mode=True)
                return True
            # pylint: disable=broad-exception-caught
            except Exception:
                if not self._fw_update_status.in_progress:
                    self.logger.info('Connection to robot was lost.')
                return False
        else:
            return False

    def IsControlling(self) -> bool:
        """See documentation in equivalent function in robot.py"""
        if not self.IsConnected() or self._monitor_mode:
            return False
        return True

    def IsSynchronousMode(self) -> bool:
        """See documentation in equivalent function in robot.py"""
        if not self.IsConnected() or not self._enable_synchronous_mode:
            return False
        return True

    def ConnectionWatchdog(self, timeout: float):
        """See documentation in equivalent function in robot.py"""
        with self._main_lock:
            self._send_custom_command(f"-ConnectionWatchdog({timeout})")

    def AutoConnectionWatchdog(self, enable: bool):
        """See documentation in equivalent function in robot.py"""
        with self._main_lock:
            if self._auto_connection_watchdog and not enable:
                # Disabling watchdog, let's immediately tell robot to disable the watchdog
                self.ConnectionWatchdog(0)
            self._auto_connection_watchdog = enable

    @disconnect_on_exception_decorator
    def ActivateRobot(self):
        """See documentation in equivalent function in robot.py"""
        with self._main_lock:
            self._check_internal_states()
            self._send_command('ActivateRobot')

        if self._enable_synchronous_mode:
            self.WaitActivated()

    # pylint: disable=unused-argument
    def _sending_ActivateRobot(self, args: list[str]):
        """ This function updates internal states when sending ActivateRobot command to the robot """
        self._robot_events.on_activated.clear_abort()
        self._robot_events.on_homed.clear_abort()

    def DeactivateRobot(self):
        """See documentation in equivalent function in robot.py"""
        with self._main_lock:
            self._check_internal_states()
            self._send_command('DeactivateRobot')

        if self._enable_synchronous_mode:
            self.WaitDeactivated()

    @disconnect_on_exception_decorator
    def Home(self):
        """See documentation in equivalent function in robot.py"""
        with self._main_lock:
            self._check_internal_states()
            self._send_command('Home')

        if self._enable_synchronous_mode:
            self.WaitHomed()

    # pylint: disable=unused-argument
    def _sending_Home(self, args: list[str]):
        """ This function updates internal states when sending Home command to the robot """
        self._robot_events.on_homed.clear_abort()

    @disconnect_on_exception_decorator
    def PauseMotion(self):
        """See documentation in equivalent function in robot.py"""
        with self._main_lock:
            self._check_internal_states()
            self._send_command('PauseMotion')

        if self._enable_synchronous_mode:
            self._robot_events.on_motion_paused.wait(timeout=self.default_timeout)

    @disconnect_on_exception_decorator
    def ResumeMotion(self):
        """See documentation in equivalent function in robot.py"""
        with self._main_lock:
            self._check_internal_states()
            self._send_command('ResumeMotion')

        if self._enable_synchronous_mode:
            self.WaitMotionResumed(timeout=self.default_timeout)

    @disconnect_on_exception_decorator
    def ClearMotion(self):
        """See documentation in equivalent function in robot.py"""
        with self._main_lock:
            self._check_internal_states()
            self._send_command('ClearMotion')

        if self._enable_synchronous_mode:
            self.WaitMotionCleared(timeout=self.default_timeout)

    # pylint: disable=unused-argument
    def _sending_ClearMotion(self, args: list[str]):
        """ This function updates internal states when sending ClearMotion command to the robot """
        # Clearing the motion queue also requires clearing checkpoints, as the robot will not send them anymore.
        message = "ClearMotion"
        # Increment the number of pending ClearMotion requests.
        self._clear_motion_requests += 1
        self._robot_events.on_motion_cleared.clear()
        self._invalidate_checkpoints(message, forced=False)
        self._invalidate_interruptable_events_on_clear_motion(message)

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

    def _sending_SetMonitoringInterval(self, args: list[str]):
        """ This function updates internal states when sending SetMonitoringInterval command to the robot """
        self._monitoring_interval = float(args[0])
        self._refresh_auto_connection_watchdog(force=True)

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
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        if synchronous_update:
            self._send_sync_command('GetStatusGripper', None, self._robot_events.on_status_gripper_updated, timeout)

        with self._main_lock:
            return copy.deepcopy(self._gripper_status)

    def WaitGripperMoveCompletion(self, timeout: Optional[float] = None):
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

    def GripperOpen(self):
        """See documentation in equivalent function in robot.py"""
        self._gripper_state_before_last_move = copy.deepcopy(self._gripper_state)
        self._send_motion_command('GripperOpen')

        if self._enable_synchronous_mode and self._robot_info.gripper_pos_ctrl_capable and self.GetRtExtToolStatus(
        ).is_gripper():
            gripper_state = self.GetRtGripperState(synchronous_update=True)
            if gripper_state.opened and gripper_state.target_pos_reached:
                return
            self.WaitGripperMoveCompletion()

    def GripperClose(self):
        """See documentation in equivalent function in robot.py"""
        self._gripper_state_before_last_move = copy.deepcopy(self._gripper_state)
        self._send_motion_command('GripperClose')

        if self._enable_synchronous_mode and self._robot_info.gripper_pos_ctrl_capable and self.GetRtExtToolStatus(
        ).is_gripper():
            gripper_state = self.GetRtGripperState(synchronous_update=True)
            if gripper_state.closed and gripper_state.target_pos_reached:
                return
            self.WaitGripperMoveCompletion()

    def MoveGripper(self, target: Union[bool, float]):
        """See documentation in equivalent function in robot.py"""
        if isinstance(target, bool):
            if target:
                self.GripperOpen()
            else:
                self.GripperClose()
        else:
            self._gripper_state_before_last_move = copy.deepcopy(self._gripper_state)
            self._send_motion_command('MoveGripper', [target])
            if self._enable_synchronous_mode:
                rt_data = self.GetRobotRtData(synchronous_update=True)
                if math.isclose(rt_data.rt_gripper_pos.data[0], target, abs_tol=0.1):
                    return
                self.WaitGripperMoveCompletion()

    def SetOutputState(self, bank_id: MxIoBankId, *output_states: Union[MxDigitalIoState, int, str]):
        """See documentation in equivalent function in robot.py"""
        final_args = list(output_states)
        final_args.insert(0, bank_id)
        self._send_motion_command('SetOutputState', final_args)

    def SetOutputState_Immediate(self, bank_id: MxIoBankId, *output_states: Union[MxDigitalIoState, int, str]):
        """See documentation in equivalent function in robot.py"""
        send_now = True
        desired_state = list(output_states)
        if self._enable_synchronous_mode:
            if self.IsDesiredIoState(bank_id, False, *desired_state):
                # Already desired state (and sync mode).
                # In sync mode we send one command at the time so we're sure that no state change is pending. So if
                # already in desired state we don't need to send the command. We actually DON'T want to send the command
                # because there would be no state change reported by on_vacuum_state_updated and this would timeout
                send_now = False

        if send_now:
            final_args = copy.deepcopy(desired_state)
            final_args.insert(0, bank_id)
            self._send_immediate_command('SetOutputState_Immediate', final_args,
                                         self._robot_events.on_output_state_updated)
            if self._enable_synchronous_mode:
                # Wait until reached desired state (or timeout)
                self.WaitIOState(bank_id, False, *desired_state, self.default_timeout)

    def WaitForAnyCheckpoint(self, timeout: float = None):
        """See documentation in equivalent function in robot.py"""
        with self._main_lock:
            self._check_internal_states()
            if '*' not in self._internal_checkpoints:
                self._internal_checkpoints['*'] = list()
            event = InterruptableEvent()
            self._internal_checkpoints['*'].append(event)

        event.wait(timeout=timeout)

    def WaitConnected(self, timeout: float = None):
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_connected.wait(timeout=timeout)

    def WaitDisconnected(self, timeout: float = None):
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        start_time = time.monotonic()
        while True:
            try:
                # Wait with short timeout, then check if connected again
                if self.IsConnected():
                    self._robot_events.on_disconnected.wait(timeout=0.1)
                else:
                    break
            except TimeoutException as e:
                if time.monotonic() - start_time > timeout:
                    raise e
                pass
            except Exception as e:
                raise e

    def WaitActivated(self, timeout: float = None):
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            if robot_model_is_mg2(self.GetRobotInfo().robot_model):
                timeout = 5.0
            else:
                timeout = 30.0
        self._robot_events.on_activated.wait(timeout=timeout)

    def WaitDeactivated(self, timeout: float = None):
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_deactivated.wait(timeout=timeout)

    def WaitHomed(self, timeout: float = None):
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            if robot_model_is_mg2(self.GetRobotInfo().robot_model):
                timeout = 5.0
            else:
                timeout = 40.0
        self._robot_events.on_homed.wait(timeout=timeout)

    def WaitSimActivated(self, timeout: float = None):
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_activate_sim.wait(timeout=timeout)

    def WaitSimDeactivated(self, timeout: float = None):
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_deactivate_sim.wait(timeout=timeout)

    def WaitExtToolSimActivated(self, timeout: float = None):
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_activate_ext_tool_sim.wait(timeout=timeout)

    def WaitExtToolSimDeactivated(self, timeout: float = None):
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_deactivate_ext_tool_sim.wait(timeout=timeout)

    # pylint: disable=unused-argument
    def WaitIoSimEnabled(self, bank_id: MxIoBankId = MxIoBankId.MX_IO_BANK_ID_IO_MODULE, timeout: float = None):
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout

        self._robot_events.on_io_sim_enabled.wait(timeout=timeout)

    # pylint: disable=unused-argument
    def WaitIoSimDisabled(self, bank_id: MxIoBankId = MxIoBankId.MX_IO_BANK_ID_IO_MODULE, timeout: float = None):
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout

        self._robot_events.on_io_sim_disabled.wait(timeout=timeout)

    def IsDesiredIoState(self, bank_id: MxIoBankId, is_input: bool, *args: Union[MxDigitalIoState, int, str]) -> bool:
        """See IsDesiredOutputState / IsDesiredInputState
        """
        expected_states = list(args)
        is_desired_state = True
        with self._main_lock:
            if bank_id == MxIoBankId.MX_IO_BANK_ID_IO_MODULE:
                if is_input:
                    curr_state = self._robot_rt_data.rt_io_module_inputs
                else:
                    curr_state = self._robot_rt_data.rt_io_module_outputs
            elif bank_id == MxIoBankId.MX_IO_BANK_ID_SIG_GEN:
                if is_input:
                    curr_state = self._robot_rt_data.rt_sig_gen_inputs
                else:
                    curr_state = self._robot_rt_data.rt_sig_gen_outputs
            else:
                is_desired_state = False
                curr_state = None
            if curr_state is not None:
                idx = 0
                while idx < len(expected_states) and idx < len(curr_state.data):
                    desired = expected_states[idx]
                    if (desired == 0 or desired == MxDigitalIoState.MX_DIGITAL_IO_STATE_0
                            or str(desired).lower() == 'off'):
                        if curr_state.data[idx] != 0:
                            is_desired_state = False
                            break
                    elif (desired == 1 or desired == MxDigitalIoState.MX_DIGITAL_IO_STATE_1
                          or str(desired).lower() == 'on'):
                        if curr_state.data[idx] != 1:
                            is_desired_state = False
                            break
                    idx = idx + 1
        return is_desired_state

    def WaitIOState(self,
                    bank_id: MxIoBankId,
                    is_input: bool,
                    *args: Union[MxDigitalIoState, int, str],
                    timeout: float = None):
        """See WaitOutputState/WaitInputState
        """
        expected_states = list(args)
        if timeout is None:
            timeout = self.default_timeout

        start_wait = time.monotonic()
        event = self._robot_events.on_input_state_updated if is_input else self._robot_events.on_output_state_updated
        while not self.IsDesiredIoState(bank_id, is_input, *expected_states):
            try:
                event.wait(timeout=0.01)
            except TimeoutException as e:
                # Check final timeout
                elapsed_ms = time.monotonic() - start_wait
                if elapsed_ms > timeout:
                    raise e
            event.clear()

    def WaitRecoveryMode(self, activated: bool, timeout: float = None):
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        if activated:
            self._robot_events.on_activate_recovery_mode.wait(timeout=timeout)
        else:
            self._robot_events.on_deactivate_recovery_mode.wait(timeout=timeout)

    def WaitForError(self, timeout: float = None):
        """See documentation in equivalent function in robot.py"""
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_error.wait(timeout=timeout)

    def WaitErrorReset(self, timeout: float = None):
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_error_reset.wait(timeout=timeout)

    def WaitPStop2ResetDeprecated(self, timeout: float = None):
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_pstop2_reset.wait(timeout=timeout)

    def WaitPStop2ResettableDeprecated(self, timeout: float = None):
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_pstop2_resettable.wait(timeout=timeout)

    def WaitEStopResetDeprecated(self, timeout: float = None):
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_estop_reset.wait(timeout=timeout)

    def WaitEStopResettableDeprecated(self, timeout: float = None):
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_estop_resettable.wait(timeout=timeout)

    def WaitSafetyStopReset(self, timeout: float = None):
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_safety_stop_reset.wait(timeout=timeout)

    def WaitSafetyStopResettable(self, timeout: float = None):
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_safety_stop_resettable.wait(timeout=timeout)

    def WaitSafetyStopStateChange(self, timeout: float = None):
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout

        self._robot_events.on_safety_stop_state_change.clear()
        self._robot_events.on_safety_stop_state_change.wait(timeout=timeout)

    def WaitMotionResumed(self, timeout: float = None):
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_motion_resumed.wait(timeout=timeout)

    def WaitMotionPaused(self, timeout: float = None):
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_motion_paused.wait(timeout=timeout)

    def WaitMotionCleared(self, timeout: float = None):
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_motion_cleared.wait(timeout=timeout)

    def WaitEndOfCycle(self, timeout: float = None):
        """See documentation in equivalent function in robot.py"""
        if self._robot_events.on_end_of_cycle.is_set():
            self._robot_events.on_end_of_cycle.clear()

        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = 2
        self._robot_events.on_end_of_cycle.wait(timeout=timeout)

    def WaitIdle(self, timeout: float = None, wait_rt_data=False):
        """See documentation in equivalent function in robot.py"""
        with self._main_lock:
            # Can't wait if robot is in error (already "idle")
            if self._robot_status.error_status:
                raise InterruptException('Robot is in error')
        checkpoint = self._set_checkpoint_internal()

        start_time = time.monotonic()
        checkpoint.wait(timeout=timeout)
        end_time = time.monotonic()

        if timeout:
            remaining_timeout = timeout - (end_time - start_time)
        else:
            remaining_timeout = None

        if remaining_timeout is None or remaining_timeout > 0:
            self._robot_events.on_end_of_block.wait(timeout=remaining_timeout)
        if wait_rt_data:
            self.WaitEndOfCycle()

    def ResetError(self):
        """See documentation in equivalent function in robot.py"""
        with self._main_lock:
            self._check_internal_states()
            self._send_command('ResetError')

        if self._enable_synchronous_mode:
            self._robot_events.on_error_reset.wait(timeout=self.default_timeout)

    def ResetPStop2Deprecated(self, timeout: float = None):
        """See documentation in equivalent function in robot.py"""
        with self._main_lock:
            self._check_internal_states()
            if self._robot_info.version.is_at_least(9, 2):
                self._send_command('ResetPStop2')
            else:
                # Use legacy command name on older robots
                self._send_command('ResetPStop')

        if self._enable_synchronous_mode:
            # Use appropriate default timeout if not specified
            if timeout is None:
                timeout = 2
            self._robot_events.on_motion_resumed.wait(timeout)

    def Delay(self, t: float):
        """See documentation in equivalent function in robot.py"""
        with self._main_lock:
            self._check_internal_states()
            if not self._robot_events.on_homed.is_set():
                raise InvalidStateError('This command requires robot to be homed.')
            self._send_command('Delay', [t])
            if self._enable_synchronous_mode:
                checkpoint = self._set_checkpoint_internal()

        if self._enable_synchronous_mode:
            checkpoint.wait()

    def VacuumGripReleaseImmediate(self, release: bool):
        """See VacuumGrip_Immediate/VacuumRelease_Immediate"""
        expected_vacuum_on = False if release else True
        if self._enable_synchronous_mode and self._vacuum_state.vacuum_on == expected_vacuum_on:
            # Already desired state (and sync mode).
            # In sync mode we send one command at the time so we're sure that no state change is pending. So if already
            # in desired state we don't need to send the command. We actually DON'T want to send the command because
            # there would be no state change reported by on_vacuum_state_updated and this would timeout
            return
        else:
            if release:
                self._send_immediate_command('VacuumRelease_Immediate', None,
                                             self._robot_events.on_vacuum_state_updated)
            else:
                self._send_immediate_command('VacuumGrip_Immediate', None, self._robot_events.on_vacuum_state_updated)
            if self._enable_synchronous_mode:
                # Wait until reached desired state (or timeout)
                start_wait = time.monotonic()
                while self._vacuum_state.vacuum_on != expected_vacuum_on:
                    try:
                        self._robot_events.on_vacuum_state_updated.wait(timeout=self.default_timeout)
                    except TimeoutException as e:
                        # Check final timeout
                        elapsed_sec = time.monotonic() - start_wait
                        if elapsed_sec > self.default_timeout:
                            raise e
                    self._robot_events.on_vacuum_state_updated.clear()

    def StartProgram(self, n: int | str, timeout: float = None):
        """See documentation in equivalent function in robot.py"""
        with self._main_lock:
            self._check_internal_states()
            self._send_command('StartProgram', [n])

        if self._enable_synchronous_mode:
            try:
                self._robot_events.on_offline_program_started.wait(timeout=timeout)
            except InterruptException as e:
                raise InvalidStateError(str(e)) from e

    def _sending_StartProgram(self, args: list[str]):
        """ This function updates internal states when sending StartProgram command to the robot """
        self._robot_events.on_offline_program_started.clear()

    def ListPrograms(self, timeout: float = None) -> dict:
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout

        with self._main_lock:
            self._check_internal_states()
            self._send_command('ListPrograms')

        try:
            response = self._robot_events.on_offline_program_op_done.wait(timeout=timeout)
        except InterruptException as e:
            raise InvalidStateError(str(e)) from e
        return response.json_data[MX_JSON_KEY_DATA]["programs"]

    def _sending_ListPrograms(self, args: list[str]):
        """ This function updates internal states when sending ListPrograms command to the robot """
        self._robot_events.on_offline_program_op_done.clear()

    def LoadProgram(self, name: str, timeout: float = None) -> dict:
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout

        with self._main_lock:
            self._check_internal_states()
            self._robot_events.on_offline_program_op_done.clear()

            json_data = {"data": {"name": name}}
            self._send_command(f'LoadProgram', f'{json.dumps(json_data)}')

        try:
            response = self._robot_events.on_offline_program_op_done.wait(timeout=timeout)
        except InterruptException as e:
            raise InvalidStateError(str(e)) from e
        return response.json_data[MX_JSON_KEY_DATA]

    def _sending_LoadProgram(self, args: list[str]):
        """ This function updates internal states when sending LoadPrograms command to the robot """
        self._robot_events.on_offline_program_op_done.clear()

    def SaveProgram(self, name: str, program: str, timeout: float = None, allow_invalid=False, overwrite=False):
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout

        with self._main_lock:
            self._check_internal_states()
            self._robot_events.on_offline_program_op_done.clear()

            json_data = {
                "data": {
                    "name": name,
                    "program": program,
                    "allowInvalid": allow_invalid,
                    "overwrite": overwrite
                }
            }
            self._send_command('SaveProgram', f'{json.dumps(json_data)}')

        try:
            self._robot_events.on_offline_program_op_done.wait(timeout=timeout)
        except InterruptException as e:
            raise InvalidStateError(str(e)) from e

    def _sending_SaveProgram(self, args: list[str]):
        """ This function updates internal states when sending SavePrograms command to the robot """
        self._robot_events.on_offline_program_op_done.clear()

    def DeleteProgram(self, name: str, timeout: float = None):
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout

        with self._main_lock:
            self._check_internal_states()
            json_data = {"data": {"name": name}}
            self._send_command('DeleteProgram', f'{json.dumps(json_data)}')

        try:
            self._robot_events.on_offline_program_op_done.wait(timeout=timeout)
        except InterruptException as e:
            raise InvalidStateError(str(e)) from e

    def _sending_DeleteProgram(self, args: list[str]):
        """ This function updates internal states when sending DeletePrograms command to the robot """
        self._robot_events.on_offline_program_op_done.clear()

    def GetRtExtToolStatus(self,
                           include_timestamp: bool = False,
                           synchronous_update: bool = False,
                           timeout: float = None) -> ExtToolStatus:
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        if synchronous_update:
            self._send_sync_command('GetRtExtToolStatus', None, self._robot_events.on_external_tool_status_updated,
                                    timeout)

        with self._main_lock:
            if include_timestamp:
                return copy.deepcopy(self._robot_rt_data.rt_external_tool_status)
            else:
                return copy.deepcopy(self._external_tool_status)

    def GetNetworkConfig(self, synchronous_update: bool = False, timeout: float = None) -> NetworkConfig:
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        if synchronous_update:
            self._send_sync_command('GetNetworkConfig', None, self._robot_events.on_network_config_updated, timeout)

        with self._main_lock:
            return copy.deepcopy(self._network_config)

    def GetRtIoStatus(self,
                      bank_id: MxIoBankId = MxIoBankId.MX_IO_BANK_ID_IO_MODULE,
                      include_timestamp: bool = False,
                      synchronous_update: bool = False,
                      timeout: float = None) -> IoStatus:
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        if synchronous_update:
            self._send_sync_command(f'GetRtIoStatus({bank_id})', None, self._robot_events.on_io_status_updated, timeout)

        with self._main_lock:
            if bank_id == MxIoBankId.MX_IO_BANK_ID_IO_MODULE:
                if include_timestamp:
                    return copy.deepcopy(self._robot_rt_data.rt_io_module_status)
                else:
                    return copy.deepcopy(self._io_module_status)
            if bank_id == MxIoBankId.MX_IO_BANK_ID_SIG_GEN:
                if include_timestamp:
                    return copy.deepcopy(self._robot_rt_data.rt_sig_gen_status)
                else:
                    return copy.deepcopy(self._sig_gen_status)
            else:
                raise MecademicException("Argument Error in Command : GetRtIoStatus")

    def GetRtGripperState(self,
                          include_timestamp: bool = False,
                          synchronous_update: bool = False,
                          timeout: float = None) -> GripperState:
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        if synchronous_update:
            self._send_sync_command('GetRtGripperState', None, self._robot_events.on_gripper_state_updated, timeout)

        with self._main_lock:
            if include_timestamp:
                return copy.deepcopy(self._robot_rt_data.rt_gripper_state)
            else:
                return copy.deepcopy(self._gripper_state)

    def GetRtValveState(self,
                        include_timestamp: bool = False,
                        synchronous_update: bool = False,
                        timeout: float = None) -> ValveState:
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        if synchronous_update:
            self._send_sync_command('GetRtValveState', None, self._robot_events.on_valve_state_updated, timeout)

        with self._main_lock:
            if include_timestamp:
                return copy.deepcopy(self._robot_rt_data.rt_valve_state)
            else:
                return copy.deepcopy(self._valve_state)

    def GetRtOutputState(self,
                         bank_id: MxIoBankId = MxIoBankId.MX_IO_BANK_ID_IO_MODULE,
                         synchronous_update: bool = False,
                         timeout: float = None) -> TimestampedData:
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        if synchronous_update:
            self._send_sync_command(f'GetRtOutputState({bank_id})', None, self._robot_events.on_output_state_updated,
                                    timeout)

        with self._main_lock:
            if bank_id == MxIoBankId.MX_IO_BANK_ID_IO_MODULE:
                return copy.deepcopy(self._robot_rt_data.rt_io_module_outputs)
            if bank_id == MxIoBankId.MX_IO_BANK_ID_SIG_GEN:
                return copy.deepcopy(self._robot_rt_data.rt_sig_gen_outputs)
            else:
                return TimestampedData.zeros(0)

    def GetRtInputState(self,
                        bank_id: MxIoBankId = MxIoBankId.MX_IO_BANK_ID_IO_MODULE,
                        synchronous_update: bool = False,
                        timeout: float = None) -> TimestampedData:
        """See documentation in equivalent function in robot.py"""

        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        if synchronous_update:
            self._send_sync_command(f'GetRtInputState({bank_id})', None, self._robot_events.on_input_state_updated,
                                    timeout)

        with self._main_lock:
            if bank_id == MxIoBankId.MX_IO_BANK_ID_IO_MODULE:
                return copy.deepcopy(self._robot_rt_data.rt_io_module_inputs)
            if bank_id == MxIoBankId.MX_IO_BANK_ID_SIG_GEN:
                return copy.deepcopy(self._robot_rt_data.rt_sig_gen_inputs)
            else:
                return TimestampedData.zeros(0)

    def GetRtVacuumState(self,
                         include_timestamp: bool = False,
                         synchronous_update: bool = False,
                         timeout: float = None) -> VacuumState:
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        if synchronous_update:
            self._send_sync_command(f'GetRtVacuumState()', None, self._robot_events.on_vacuum_state_updated, timeout)

        with self._main_lock:
            if include_timestamp:
                return copy.deepcopy(self._robot_rt_data.rt_vacuum_state)
            else:
                return copy.deepcopy(self._vacuum_state)

    def GetRtTargetJointPos(self,
                            include_timestamp: bool = False,
                            synchronous_update: bool = False,
                            timeout: float = None) -> TimestampedData:
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        if synchronous_update:
            if self._robot_info.rt_message_capable:
                self._send_sync_command('GetRtTargetJointPos', None, self._robot_events.on_joints_updated, timeout)
            else:
                # This robot does not have GetRtTargetJointPos, use legacy GetJoints (but won't get timestamp)
                self._send_sync_command('GetJoints', None, self._robot_events.on_joints_updated, timeout)

            # Wait until response is received (this will throw TimeoutException if appropriate)
            self._robot_events.on_joints_updated.wait(timeout=timeout)

        with self._main_lock:
            if include_timestamp:
                if not self._robot_info.rt_message_capable:
                    raise InvalidStateError('Cannot provide timestamp with current robot firmware or model.')
                else:
                    return copy.deepcopy(self._robot_rt_data.rt_target_joint_pos)

            return copy.deepcopy(self._robot_rt_data.rt_target_joint_pos.data)

    def GetRtTargetCartPos(self,
                           include_timestamp: bool = False,
                           synchronous_update: bool = False,
                           timeout: float = None) -> TimestampedData:
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        if synchronous_update:
            if self._robot_info.rt_message_capable:
                self._send_sync_command('GetRtTargetCartPos', None, self._robot_events.on_pose_updated, timeout)
            else:
                # This robot does not have GetRtTargetCartPos, use legacy GetPose (but won't get timestamp)
                self._send_sync_command('GetPose', None, self._robot_events.on_pose_updated, timeout)

        with self._main_lock:
            if include_timestamp:
                if not self._robot_info.rt_message_capable:
                    raise InvalidStateError('Cannot provide timestamp with current robot firmware or model.')
                else:
                    return copy.deepcopy(self._robot_rt_data.rt_target_cart_pos)

            return copy.deepcopy(self._robot_rt_data.rt_target_cart_pos.data)

    def SetMonitoringInterval(self, t: float):
        """See documentation in equivalent function in robot.py"""
        with self._main_lock:
            self._check_internal_states()
            self._set_monitoring_interval_internal(t)
            self._monitoring_interval_to_restore = t

    def SetRealTimeMonitoring(self, *events: tuple):
        """See documentation in equivalent function in robot.py"""
        with self._main_lock:
            self._check_internal_states()
            if isinstance(events, tuple):
                event_list = list(events)
            else:
                event_list = events
            self._send_command('SetRealTimeMonitoring', event_list)

    def SetRtc(self, t: int):
        """See documentation in equivalent function in robot.py"""
        with self._main_lock:
            self._check_internal_states()
            self._send_command('SetRtc', [t])

    def ActivateSim(self):
        """See documentation in equivalent function in robot.py"""
        with self._main_lock:
            self._check_internal_states()
            self._send_command('ActivateSim')
        if self._enable_synchronous_mode:
            self._robot_events.on_activate_sim.wait(timeout=self.default_timeout)

    def DeactivateSim(self):
        """See documentation in equivalent function in robot.py"""
        with self._main_lock:
            self._check_internal_states()
            self._send_command('DeactivateSim')
        if self._enable_synchronous_mode:
            self._robot_events.on_deactivate_sim.wait(timeout=self.default_timeout)

    def SetExtToolSim(self, sim_ext_tool_type: int = MxExtToolType.MX_EXT_TOOL_MEGP25_SHORT):
        """See documentation in equivalent function in robot.py"""
        # Firmware version 8.4 and older only support 1 as tool type
        if sim_ext_tool_type != MxExtToolType.MX_EXT_TOOL_NONE and not self._robot_info.gripper_pos_ctrl_capable:
            sim_ext_tool_type = 1

        with self._main_lock:
            self._check_internal_states()
            self._send_command('SetExtToolSim', [sim_ext_tool_type])

        if self._enable_synchronous_mode:
            if sim_ext_tool_type == MxExtToolType.MX_EXT_TOOL_NONE:
                self.WaitExtToolSimDeactivated()
            else:
                self.WaitExtToolSimActivated()

    def SetIoSim(self, bank_id: MxIoBankId = MxIoBankId.MX_IO_BANK_ID_IO_MODULE, enable: bool = True):
        """See documentation in equivalent function in robot.py"""
        with self._main_lock:
            self._check_internal_states()
            self._send_command('SetIoSim', [bank_id, 1 if enable else 0])

        if self._enable_synchronous_mode:
            if enable:
                self.WaitIoSimEnabled(bank_id)
            else:
                self.WaitIoSimDisabled(bank_id)

    def SetRecoveryMode(self, activated: bool = True):
        """See documentation in equivalent function in robot.py"""
        with self._main_lock:
            self._check_internal_states()
            self._send_command('SetRecoveryMode', f'{1 if activated else 0}')

        if self._enable_synchronous_mode:
            if activated:
                self._robot_events.on_activate_recovery_mode.wait(timeout=self.default_timeout)
            else:
                self._robot_events.on_deactivate_recovery_mode.wait(timeout=self.default_timeout)

    def SetTimeScaling(self, p: float):
        """See documentation in equivalent function in robot.py"""
        with self._main_lock:
            self._check_internal_states()
            self._send_command('SetTimeScaling', [p])

        if self._enable_synchronous_mode:
            self._robot_events.on_time_scaling_changed.wait(timeout=self.default_timeout)

    def _sending_SetTimeScaling(self, args: list[str]):
        """ This function updates internal states when sending DeletePrograms command to the robot """
        self._robot_events.on_time_scaling_changed.clear()

    def SetJointLimitsCfg(self, e: bool = True):
        """See documentation in equivalent function in robot.py"""
        if self._enable_synchronous_mode:
            response_event = self._send_custom_command(
                f'SetJointLimitsCfg({1 if e else 0})',
                expected_responses=[MxRobotStatusCode.MX_ST_SET_JOINT_LIMITS_CFG, MxRobotStatusCode.MX_ST_CMD_FAILED])
            if response_event.wait(timeout=DEFAULT_WAIT_TIMEOUT).id == MxRobotStatusCode.MX_ST_CMD_FAILED:
                raise MecademicException("Argument Error in Command : SetJointLimitsCfg")
        else:
            with self._main_lock:
                self._check_internal_states()
                self._send_command('SetJointLimitsCfg', f"{1 if e else 0}")

    def SetJointLimits(self, n: int, lower_limit: float, upper_limit: float):
        """See documentation in equivalent function in robot.py"""
        if self._enable_synchronous_mode:
            response_event = self._send_custom_command(
                f'SetJointLimits({n},{lower_limit},{upper_limit})',
                expected_responses=[MxRobotStatusCode.MX_ST_SET_JOINT_LIMITS, MxRobotStatusCode.MX_ST_CMD_FAILED])
            if response_event.wait(timeout=DEFAULT_WAIT_TIMEOUT).id == MxRobotStatusCode.MX_ST_CMD_FAILED:
                raise MecademicException("Argument Error in Command : SetJointLimits")
        else:
            with self._main_lock:
                self._check_internal_states()
                self._send_command('SetJointLimits', f"{n},{lower_limit},{upper_limit}")

    def SetWorkZoneCfg(self,
                       severity: MxEventSeverity = MxEventSeverity.MX_EVENT_SEVERITY_ERROR,
                       mode: MxWorkZoneMode = MxWorkZoneMode.MX_WORK_ZONE_MODE_FCP_IN_WORK_ZONE):
        """See documentation in equivalent function in robot.py"""
        if self._enable_synchronous_mode:
            response_event = self._send_custom_command(f'SetWorkZoneCfg({severity},{mode})',
                                                       expected_responses=[
                                                           MxRobotStatusCode.MX_ST_SET_WORK_ZONE_LIMITS_CFG,
                                                           MxRobotStatusCode.MX_ST_CMD_FAILED
                                                       ])
            if response_event.wait(timeout=DEFAULT_WAIT_TIMEOUT).id == MxRobotStatusCode.MX_ST_CMD_FAILED:
                raise MecademicException("Argument Error in Command : SetWorkZoneCfg")
        else:
            with self._main_lock:
                self._check_internal_states()
                self._send_command('SetWorkZoneCfg', f"{severity},{mode}")

    def SetWorkZoneLimits(self, x_min: float, y_min: float, z_min: float, x_max: float, y_max: float, z_max: float):
        """See documentation in equivalent function in robot.py"""
        if self._enable_synchronous_mode:
            response_event = self._send_custom_command(
                f'SetWorkZoneLimits({x_min}, {y_min}, {z_min}, {x_max}, {y_max}, {z_max})',
                expected_responses=[MxRobotStatusCode.MX_ST_SET_WORK_ZONE_LIMITS, MxRobotStatusCode.MX_ST_CMD_FAILED])
            if response_event.wait(timeout=DEFAULT_WAIT_TIMEOUT).id == MxRobotStatusCode.MX_ST_CMD_FAILED:
                raise MecademicException("Argument Error in Command : SetWorkZoneLimits")
        else:
            with self._main_lock:
                self._check_internal_states()
                self._send_command('SetWorkZoneLimits', [x_min, y_min, z_min, x_max, y_max, z_max])

    def SetCollisionCfg(self, severity: MxEventSeverity = MxEventSeverity.MX_EVENT_SEVERITY_ERROR):
        """See documentation in equivalent function in robot.py"""
        if self._enable_synchronous_mode:
            response_event = self._send_custom_command(
                f'SetCollisionCfg({severity})',
                expected_responses=[MxRobotStatusCode.MX_ST_SET_COLLISION_CFG, MxRobotStatusCode.MX_ST_CMD_FAILED])
            if response_event.wait(timeout=DEFAULT_WAIT_TIMEOUT).id == MxRobotStatusCode.MX_ST_CMD_FAILED:
                raise MecademicException("Argument Error in Command : SetCollisionCfg")
        else:
            with self._main_lock:
                self._check_internal_states()
                self._send_command('SetCollisionCfg', f"{severity}")

    def SetToolSphere(self, x: float, y: float, z: float, r: float):
        """See documentation in equivalent function in robot.py"""
        if self._enable_synchronous_mode:
            response_event = self._send_custom_command(
                f'SetToolSphere({x}, {y}, {z}, {r})',
                expected_responses=[MxRobotStatusCode.MX_ST_SET_TOOL_SPHERE, MxRobotStatusCode.MX_ST_CMD_FAILED])
            if response_event.wait(timeout=DEFAULT_WAIT_TIMEOUT).id == MxRobotStatusCode.MX_ST_CMD_FAILED:
                raise MecademicException("Argument Error in Command : SetToolSphere")
        else:
            with self._main_lock:
                self._check_internal_states()
                self._send_command('SetToolSphere', [x, y, z, r])

    def SetTorqueLimitsCfg(
            self,
            severity: MxEventSeverity = MxEventSeverity.MX_EVENT_SEVERITY_ERROR,
            skip_acceleration: MxTorqueLimitsMode = MxTorqueLimitsMode.MX_TORQUE_LIMITS_DETECT_SKIP_ACCEL):
        """See documentation in equivalent function in robot.py"""
        if isinstance(severity, str):
            severity_int = TORQUE_LIMIT_SEVERITIES[severity]
        else:
            severity_int = severity
        skip_acceleration_int = 1 if skip_acceleration else 0
        self._send_motion_command('SetTorqueLimitsCfg', [severity_int, skip_acceleration_int])

    def SetTorqueLimits(self, *args: float):
        """See documentation in equivalent function in robot.py"""
        expect_count = self._robot_info.num_joints
        if len(args) != expect_count:
            raise ValueError(
                f'SetTorqueLimits: Incorrect number of joints sent {len(args)}, command. expecting: {expect_count}.')

        self._send_motion_command('SetTorqueLimits', args)

    def SetPStop2Cfg(self, severity: MxEventSeverity = MxEventSeverity.MX_EVENT_SEVERITY_CLEAR_MOTION):
        """See documentation in equivalent function in robot.py"""
        if self._enable_synchronous_mode:
            response_event = self._send_custom_command(
                f'SetPStop2Cfg({severity})',
                expected_responses=[MxRobotStatusCode.MX_ST_SET_PSTOP2_CFG, MxRobotStatusCode.MX_ST_CMD_FAILED])
            if response_event.wait(timeout=DEFAULT_WAIT_TIMEOUT).id == MxRobotStatusCode.MX_ST_CMD_FAILED:
                raise MecademicException("Argument Error in Command : SetPStop2Cfg")
        else:
            with self._main_lock:
                self._check_internal_states()
                self._send_command('SetPStop2Cfg', f"{severity}")

    def SetPayload(self, mass: float, x: float, y: float, z: float):
        """See documentation in equivalent function in robot.py"""
        self._send_motion_command('SetPayload', [mass, x, y, z])

    def ActivateBrakes(self, activated: bool = True):
        """See documentation in equivalent function in robot.py"""
        with self._main_lock:
            self._check_internal_states()
            if activated:
                self._send_command('BrakesOn')
            else:
                self._send_command('BrakesOff')

        if self._enable_synchronous_mode:
            if activated:
                self._robot_events.on_brakes_activated.wait(timeout=self.default_timeout)
            else:
                self._robot_events.on_brakes_deactivated.wait(timeout=self.default_timeout)

    def GetRobotInfo(self) -> RobotInfo:
        """See documentation in equivalent function in robot.py"""
        with self._main_lock:
            return copy.deepcopy(self._robot_info)

    def GetRobotRtData(self, synchronous_update: bool = False, timeout: float = None) -> RobotRtData:
        """See documentation in equivalent function in robot.py"""
        if synchronous_update:
            self.WaitEndOfCycle(timeout)

        with self._main_lock:
            return copy.deepcopy(self._robot_rt_data_stable)

    def GetStatusRobot(self, synchronous_update: bool = False, timeout: float = None) -> RobotStatus:
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        if synchronous_update:
            if self._using_legacy_json_api:
                with self._main_lock:
                    self._send_command('GetMotionStatus')
            self._send_sync_command('GetStatusRobot', None, self._robot_events.on_status_updated, timeout)

        with self._main_lock:
            return copy.deepcopy(self._robot_status)

    def GetSafetyStatus(self, synchronous_update: bool = False, timeout: float = None) -> RobotSafetyStatus:
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        if synchronous_update:
            self._send_sync_command('GetStatusRobot', None, self._robot_events.on_status_updated, timeout)

        with self._main_lock:
            return copy.deepcopy(self._robot_safety_status)

    def GetPowerSupplyInputs(self, synchronous_update: bool = False, timeout: float = None) -> RobotPowerSupplyInputs:
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        if synchronous_update:
            self._send_sync_command('GetStatusRobot', None, self._robot_events.on_status_updated, timeout)

        with self._main_lock:
            return copy.deepcopy(self._robot_psu_inputs)

    def GetCollisionStatus(self, timeout: float = None) -> CollisionStatus:
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout

        with self._main_lock:
            return copy.deepcopy(self._robot_collision_status)

    def GetGripperRange(self, timeout: float = None) -> Tuple[float, float]:
        """See documentation in equivalent function in robot.py"""
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout

        response = self._send_custom_command('GetGripperRange',
                                             expected_responses=[MxRobotStatusCode.MX_ST_GET_GRIPPER_RANGE],
                                             timeout=self.default_timeout)
        if isinstance(response, Message):
            positions = string_to_numbers(response.data)
        assert len(positions) == 2
        return positions[0], positions[1]

    def LogTrace(self, trace: str):
        """See documentation in equivalent function in robot.py"""
        # Remove any " in the provided string
        trace = trace.replace('"', "")
        if self._robot_info.version.is_at_least(9, 2):
            self._send_custom_command(f'-LogTrace("{trace}")')
        else:
            self._send_custom_command(f'LogTrace("{trace}")')

    def StartLogging(self,
                     monitoringInterval: float,
                     file_name: str = None,
                     file_path: str = None,
                     fields: list = None,
                     record_time: bool = True):
        """See documentation in equivalent function in robot.py"""
        if self._file_logger is not None:
            raise InvalidStateError('Another file logging operation is in progress.')

        self._set_monitoring_interval_internal(monitoringInterval)
        if self._robot_info.rt_message_capable:
            if fields is None:
                self.SetRealTimeMonitoring('all')
            else:
                self.SetRealTimeMonitoring(*fields)

            # Use a synchronous "GetRealTimeMonitoring" to ensure that we've started receiving data for all the
            # requested real-time monitoring fields we just enabled
            self._send_custom_command('GetRealTimeMonitoring',
                                      expected_responses=[MxRobotStatusCode.MX_ST_GET_REAL_TIME_MONITORING],
                                      timeout=self.default_timeout,
                                      skip_internal_check=True)
            if not self._robot_info.rt_on_ctrl_port_capable:
                # Older version -> can't be sure that monitoring and control port are in sync, let's wait few
                self.WaitEndOfCycle()
                time.sleep(0.01)
                self.WaitEndOfCycle()

        # pylint: disable=protected-access
        self._file_logger = mx_traj._RobotTrajectoryLogger(self._robot_info,
                                                           self._robot_rt_data,
                                                           fields,
                                                           file_name=file_name,
                                                           file_path=file_path,
                                                           record_time=record_time,
                                                           monitoring_interval=monitoringInterval)

    def EndLogging(self, keep_captured_trajectory: bool = False) -> str:
        """See documentation in equivalent function in robot.py"""
        if self._file_logger is None:
            raise InvalidStateError('No existing logger to stop.')

        # Deactivate logging to avoid logging the following SetMonitoringInterval
        self._file_logger.stop_logging_commands()

        if self._robot_info.rt_message_capable:
            if self._monitoring_interval_to_restore != self._monitoring_interval:
                # Restore default slower monitoring interval
                self._set_monitoring_interval_internal(self._monitoring_interval_to_restore)

            # Send a synchronous command to ensure we've received all monitoring data for this test
            self._send_custom_command('GetRealTimeMonitoring',
                                      expected_responses=[MxRobotStatusCode.MX_ST_GET_REAL_TIME_MONITORING],
                                      timeout=self.default_timeout,
                                      skip_internal_check=True)

        if keep_captured_trajectory:
            self._captured_trajectory = self._file_logger.robot_trajectories

        file_name = self._file_logger.end_log()
        self._file_logger = None

        return file_name

    def GetCapturedTrajectory(self) -> RobotTrajectories:
        """See documentation in equivalent function in robot.py"""
        return self._captured_trajectory

    def UpdateRobot(self, firmware: Union[str, pathlib.Path], timeout: float = _UPDATE_TIMEOUT):
        """See documentation in equivalent function in robot.py"""
        if not self.IsConnected():
            raise InvalidStateError('Not connected to the robot. Please call "Connect" before calling "UpdateRobot".')
        try:
            self._update_robot(firmware, timeout)
        except Exception as e:
            raise e
        finally:
            self._reset_fw_update_status()

    #####################################################################################
    # Private methods.
    #####################################################################################

    def _update_robot(self, firmware: Union[str, pathlib.Path], timeout: float = _UPDATE_TIMEOUT):
        """See documentation of UpdateRobot in robot.py"""

        if isinstance(firmware, pathlib.Path):
            firmware_file: pathlib.Path = firmware
        elif isinstance(firmware, str):
            firmware_file = pathlib.Path(firmware)
        else:
            raise ArgumentError(None,
                                f'Unsupported firmware type. received: {type(firmware)}, expecting pathlib or str')

        firmware_file_version = RobotVersion(firmware_file.name)

        address = self._address
        address_port = f'{address}:{self._port}'

        # Making sure we can send command to robot
        if not self.IsConnected():
            self.Connect(address=address)
        elif self._monitor_mode:
            self.logger.info(f'Connected to robot in monitoring mode only, attempting connection in command mode'
                             'to deactivate robot')
            self.Connect(address=address)

        # Make sure that the robot is not in EStop
        if robot_model_is_meca500(
                self.GetRobotInfo().robot_model) and self.GetSafetyStatus(synchronous_update=True).estop_state:
            raise MecademicException(
                f'Firmware update failed: Robot is in ESTOP. Please clear the ESTOP condition before updating.')

        # Check if we must use legacy mode
        use_legacy_update = not self.GetRobotInfo().version.is_at_least(9, 3, 0)

        # Make sure that the robot is deactivated
        if self.GetStatusRobot().activation_state:
            self.logger.info(f'Robot is activated, will attempt to deactivate before updating firmware')
            self.DeactivateRobot()
            self.WaitDeactivated()
        if self._enable_synchronous_mode is None:
            current_synchronous_mode = False
        else:
            current_synchronous_mode = self._enable_synchronous_mode

        initial_disconnect_on_exception = self._disconnect_on_exception
        self.Disconnect()

        if not use_legacy_update:
            # Reconnect to the robot using the JSON API
            self.Connect(address=f'{address}:{MX_ROBOT_TCP_PORT_CONTROL_JSON}')

        self.logger.info(f"Installing firmware: {firmware_file.resolve()}")

        with open(str(firmware_file), 'rb') as firmware_stream:
            firmware_data = firmware_stream.read()
            firmware_data_size = str(len(firmware_data))

        headers = {
            'Connection': 'keep-alive',
            'Content-type': 'application/x-gzip',
            'Content-Length': firmware_data_size
        }

        if use_legacy_update:
            self.logger.info(f"Uploading firmware (legacy mode)...")
            robot_url = f"http://{address}/"
        else:
            self.logger.info(f"Uploading firmware...")
            robot_url = f"http://{address}/fw-update/{firmware_file.name}"

        request_post = requests.post(robot_url, data=firmware_data, headers=headers, timeout=60.0)
        try:
            request_post.raise_for_status()
        except Exception as e:
            self.logger.error(f"Upgrade post request error: {e}")
            raise

        if not request_post.ok:
            error_message = f"Firmware upload request failed"
            raise RuntimeError(error_message)

        self.logger.info(f"Starting the firmware update...")

        if not use_legacy_update:
            # Send the 'StartFwUpdate' command
            self._fw_update_status.in_progress = True  # Set once here, but the will be updated from robot update status
            self._send_custom_command(f'StartFwUpdate({firmware_file.name})')

        # Follow update status
        start_time = time.monotonic()
        while True:
            if use_legacy_update:
                self._check_update_progress_legacy(robot_url)
            else:
                self._check_update_progress(address)

            # Check if complete, failed or timeout
            if self._fw_update_status.error:
                raise MecademicException(f'Firmware update failed: {self._fw_update_status.error_msg}')
            if self._fw_update_status.complete:
                self.logger.info(f"Firmware update done")
                break
            if time.monotonic() > start_time + timeout:
                error_message = f"Timeout while waiting for update done response, after {timeout} seconds"
                raise TimeoutError(error_message)

        if use_legacy_update:
            # need to wait to make sure the robot shutdown before attempting to ping it.
            time.sleep(15)
            # Try to ping the robot until it's responding (or until default timeout)
            ping_robot(address)
            # Now that robot responds to ping, wait until it accepts new connections
            self.Connect(address_port,
                         timeout=60,
                         enable_synchronous_mode=current_synchronous_mode,
                         disconnect_on_exception=initial_disconnect_on_exception)
        else:
            end_time = time.monotonic() + 30
            while True:
                if time.monotonic() > end_time:
                    raise TimeoutError('Timeout while attempting to reconnect in control mode')
                try:
                    # Reconnect in control mode
                    self.Disconnect()
                    if self._fw_update_status.complete and not self._fw_update_reboot_done:
                        # A firmware update without a reboot is reserved for special cases (Mecademic internal use).
                        # In these cases, let's wait a bit before reconnecting to the robot
                        self.logger.info(f"Waiting before reconnecting to robot...")
                        time.sleep(11)

                    # (shorter timeout here, we know we're already connected so it should not be long)
                    self.Connect(address_port,
                                 timeout=10,
                                 enable_synchronous_mode=current_synchronous_mode,
                                 disconnect_on_exception=initial_disconnect_on_exception)
                    break
                except Exception as e:
                    error_message = str(e)
                    if 'id=3002,' in error_message:
                        time.sleep(1)
                        continue
                    raise

        if self.GetRobotInfo().version.is_at_least(8.0):
            current_version = self.GetRobotInfo().version.get_str(build=True, extra=False)
            expected_version = firmware_file_version.full_version
        else:
            current_version = self.GetRobotInfo().version
            expected_version = firmware_file_version.short_version

        if str(current_version) not in expected_version:
            error_msg = (f"Fail to install robot properly. current version {current_version}, "
                         f"expecting: {expected_version}")
            self.logger.error(error_msg)
            raise AssertionError(error_msg)

        robot_status = self.GetStatusRobot(synchronous_update=True)
        if robot_status.error_status:
            error_msg = f"Robot is in error on version {current_version}"
            self.logger.error(error_msg)
            raise InvalidStateError(error_msg)

        self.logger.info(f"Installation of {current_version} successfully completed")

    def _normalize_cart_cmd_args(self, alpha: float = None, beta: float = None, gamma: float = None) -> list[float]:
        """Normalize alpha, beta and gamma arguments for Cartesian commands which accept alpha/beta
        arguments to be omitted"""
        if self.GetRobotInfo().num_joints == 6:
            if alpha is None or beta is None or gamma is None:
                raise ValueError('Missing argument (on this robot Cartesian positions require 6 values)')
            else:
                return [alpha, beta, gamma]
        else:
            # Only 2 valid ways of passing Cartesian arguments on 4 axes robots: Pass all, or pass only gamma
            if alpha is not None and beta is not None and gamma is not None:
                # Fine, all were passed
                return [alpha, beta, gamma]
            elif alpha is not None and beta is None and gamma is None:
                # Only alpha was passed, probably by positional argument, assume it's the "gamma" value
                return [0, 0, alpha]
            elif alpha is None and beta is None and gamma is not None:
                # Only gamma was passed, assume 0 for the others
                return [0, 0, gamma]

            raise ValueError('Wrong number of argument (on this robot Cartesian positions require 4 values)')

    def _normalize_conf_cmd_args(self, shoulder: int = None, elbow: int = None, wrist: int = None) -> list[int]:
        """Normalize shoulder, elbow and wrist "conf" arguments for commands which accept to omit shoulder and
           wrist (for 4 axes robots which only have elbow conf)"""
        if self.GetRobotInfo().num_joints == 6:
            if shoulder is None or elbow is None or wrist is None:
                raise ValueError('Missing argument (on this robot configuration requires 3 values)')
            else:
                return [shoulder, elbow, wrist]
        else:
            # Only 2 valid ways of passing Cartesian arguments on 4 axes robots: Pass all, or pass only elbow
            if shoulder is not None and elbow is not None and wrist is not None:
                # Fine, all were passed
                return [shoulder, elbow, wrist]
            elif shoulder is not None and elbow is None and wrist is None:
                # Only shoulder was passed, probably by positional argument, assume it's the "elbow" value
                return [0, shoulder, 0]
            elif shoulder is None and elbow is not None and wrist is None:
                # Only elbow was passed, assume 0 for the others
                return [0, elbow, 0]

            raise ValueError('Wrong number of arguments (on this robot configuration require 1 value)')

    def _get_reboot_duration(self):
        """ Get the average expected reboot duration for current robot model """
        if self.GetRobotInfo().robot_model == MxRobotModel.MX_ROBOT_MODEL_MCS500_R1:
            return MX_FW_UPDATE_REBOOT_DURATION_SEC_MCS500
        else:
            return MX_FW_UPDATE_REBOOT_DURATION_SEC_MECA500

    def _get_fw_update_duration(self):
        """ Get the average expected firmware update duration for current robot model """
        if self.GetRobotInfo().robot_model == MxRobotModel.MX_ROBOT_MODEL_MCS500_R1:
            return MX_FW_UPDATE_AVG_DURATION_SEC_MCS500
        else:
            return MX_FW_UPDATE_AVG_DURATION_SEC_MECA500

    def _check_update_progress(self, address: str):
        """
        Check progress of firmware update (new implementation using JSON API).

        Parameters
        ----------
        address: string
            Robot IP address/port
        """
        time.sleep(0.1)
        # Update status is updated in background by _handle_fw_update_progress until robot reboots at the end of the
        # update, at which time the code below will periodically print progress for convenience
        if not self.IsConnected():
            # During update, we'll get disconnected. Try reconnecting in monitoring mode to follow update progress
            try:
                if self._monitor_mode and self._fw_update_status.in_progress and self._fw_update_reboot_timestamp == 0:
                    # Got disconnected -> Consider we're awaiting for the robot to reboot
                    self._fw_update_reboot_timestamp = time.monotonic()
                self.Connect(address=f'{address}:{MX_ROBOT_TCP_PORT_FEED_JSON}', monitor_mode=True, timeout=1.0)
                # Reconnected in monitoring mode, therefore update is started
                self._fw_update_started = True
            # pylint: disable=broad-exception-caught
            except Exception:
                # In case we're updating to an older package (<9.3) we won't be able to connect to JSON port.
                # So let's try to connect to standard port.
                try:
                    self.Connect(address=f'{address}', monitor_mode=True, timeout=0.1)
                    # Successfully connected to legacy port. Validate that we're connected to older version
                    if self.GetRobotInfo().version.is_at_least(9, 3):
                        # Recent version, we should be able to connect to JSON port,
                        # so let's close here and retry JSON above (next loop)
                        self.Disconnect()
                    else:
                        # Older version. No JSON port exists so let's assume that the update is complete
                        self._fw_update_status.complete = True
                # pylint: disable=broad-exception-caught
                except Exception:
                    # Robot is probably still rebooting
                    # Update the update percentage based on estimated reboot time
                    if self._fw_update_reboot_timestamp == 0:
                        reboot_pct = 0
                    else:
                        reboot_elapsed_ms = 1000 * (time.monotonic() - self._fw_update_reboot_timestamp)
                        reboot_pct = 100 * (reboot_elapsed_ms / (1000 * self._get_reboot_duration()))
                        if reboot_pct > 100:
                            reboot_pct = 100

                    # Print status while awaiting for robot to reboot
                    # pylint: disable=protected-access
                    if time.monotonic() - self._fw_update_status._last_print_timestamp > 5.0:
                        self._print_fw_update_status(int(reboot_pct))

    def _check_update_progress_legacy(self, robot_url: str):
        """
        Check progress of firmware update (legacy version with old web portal).

        Parameters
        ----------
        robot_url: string
            Robot URL

        """
        time.sleep(2)
        self._fw_update_status.complete = False
        request_get = requests.get(robot_url, 'update', timeout=10)
        try:
            request_get.raise_for_status()
        except Exception as e:
            self.logger.error(f'Upgrade get request error: {e}')
            raise e

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
        # pylint: disable=broad-exception-caught
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
                previous_progress = self._fw_update_status.progress_str
                self._fw_update_status.progress_str = request_answer['LOG'][keys[-1]]
                new_progress = self._fw_update_status.progress_str.replace(previous_progress, '')
                if ':' in new_progress:
                    self.logger.info(new_progress)
                elif '100%' in new_progress:
                    self.logger.info(new_progress)
                else:
                    self.logger.debug(new_progress)
            if status_code == 0:
                self.logger.info(f'status_msg {status_msg}')
                self._fw_update_status.complete = True
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

            # Consider we're disconnected if we've not recently received a message from the robot
            if self._monitor_timeout_used and self._rx_timestamp != 0:
                elapsedMs = 1000 * (time.monotonic() - self._rx_timestamp)
                if elapsedMs > _MONITORING_TIMEOUT_MS:
                    raise TimeoutError(f'Timeout: No message received from the robot in the last {elapsedMs}ms. '
                                       f'Assuming we are disconnected from the robot.')
        except Exception:
            # An error was detected while validating internal states. Disconnect from robot.
            self._disconnect()
            raise

    def _split_command_args(self, command: str, args: Union[str, list, tuple] = None) -> list[str, Optional[list[str]]]:
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
        list[str, Optional[list[str]]]
            Command name (without arguments), arguments (normalized as list of strings)
        """
        if args is None:
            # Split command from args by searching for opening parenthesis
            split_result = command.split('(', 1)
            if len(split_result) == 2:
                # Strip command name (just in case there were spaces around it)
                command = split_result[0]
                # Remove trailing parenthesis from arguments
                args = split_result[1].rstrip(")")

        # Normalize arguments as a list of strings
        args_string: Optional[list[str]] = None
        if args is not None:
            if isinstance(args, tuple):
                args = list(args)
            elif isinstance(args, list):
                pass
            else:
                args = [args]
            args_string: list[str] = []
            for arg in args:
                if isinstance(arg, IntEnum):
                    args_string.append(str(arg.value))
                else:
                    args_string.append(str(arg))

        # Strip command name (just in case there were spaces around it)
        command = command.strip()
        return [command, args_string]

    def _send_command(self, command: str, args: Union[str, list, tuple] = None):
        """Assembles and sends the command string to the Mecademic robot.

        Parameters
        ----------
        command : string
            Command name to send to the Mecademic robot.
        args : list or str
            List of arguments the command requires.

        """

        # Make sure that command and args are split
        command, args = self._split_command_args(command, args)
        command_trimmed = command.replace('-', '').lower()
        if command_trimmed in self.cmd_handler_dict:
            self.cmd_handler_dict[command_trimmed](args)

        # Assemble arguments into a string and concatenate to end of command.
        if args:
            command += f'({args_to_string(args)})'

        # Put command into tx queue.
        self._command_tx_queue.put(command)

        # If logging is enabled, send command to logger.
        if self._file_logger and self._file_logger.logging_commands:
            self._file_logger.command_queue.put(command)

    def _send_sync_command(self, command: str, args: Optional[Union[str, list, tuple]],
                           event: Optional[InterruptableEvent], timeout: float):
        """Send a command and wait for corresponding response
           (this function handles well-known commands which have their corresponding 'wait' event in this class,
            use _send_custom_command to perform synchronous operations on other commands)

        Parameters
        ----------
        command : string
            Name of the command to send (example: GetStatusGripper)
        args : list or str
            List of arguments the command requires.
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
                self._tx_sync_pending = self._tx_sync
                self._is_sync.clear()
                self._send_command('SyncCmdQueue', f'{self._tx_sync}')
            if event is not None and event.is_set():
                event.clear()
            self._send_command(command, args)

        # Wait until response is received (this will throw TimeoutException if appropriate)
        if event is not None:
            event.wait(timeout=timeout)
        else:
            # No specific event to await for. Just await until we're sync
            self._is_sync.wait(timeout=timeout)

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

    def GetInterruptableEvent(self,
                              codes: list[Union[MxRobotStatusCode, Message]],
                              abort_on_error=False,
                              abort_on_clear_motion=False) -> InterruptableEvent:
        """See documentation in equivalent function in robot.py"""
        with self._main_lock:
            event_with_data = InterruptableEvent(data=codes,
                                                 abort_on_error=abort_on_error,
                                                 abort_on_clear_motion=abort_on_clear_motion)
            self._custom_response_events.append(weakref.ref(event_with_data))
            # Check if the event must be interrupted already
            if abort_on_error and self._robot_status.error_status:
                event_with_data.abort('Robot is in error')
            if abort_on_clear_motion:
                if not self._robot_status.activation_state:
                    event_with_data.abort('Robot was deactivated')
                if self._robot_safety_status.pstop2_state != MxStopState.MX_STOP_STATE_RESET:
                    event_with_data.abort('Robot is in PSTOP2 condition')
            return event_with_data

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
        thread.daemon = True  # Make sure thread does not prevent application from quitting
        thread.start()
        return thread

    def _initialize_command_socket(self, timeout=1.0):
        """Establish the command socket and the associated thread.

        """
        if self._offline_mode:
            return

        if self._command_socket is not None:
            raise InvalidStateError('Cannot connect since existing command socket exists.')

        self._command_socket = self._connect_socket(self.logger, self._address, self._port, timeout)
        self.logger.debug(f'Connected to {self._address}:{self._port} (control mode)')

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

        port = self._port if self._monitor_mode else MX_ROBOT_TCP_PORT_FEED
        self._monitor_socket = self._connect_socket(self.logger, self._address, port, timeout)
        self.logger.debug(f'Connected to {self._address}:{port} (monitoring mode)')

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
            except queue.Empty as e:
                self.logger.error('No response received within timeout interval.')
                raise CommunicationError('No response received within timeout interval.') from e
            except BaseException as e:
                raise e

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
            raise CommunicationError(f'Connection error: {response}')

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
            # pylint: disable=broad-exception-caught
            except Exception as e:
                self.logger.error(f'Error shutting down tx thread: {e}')
            self._command_tx_thread.join(timeout=self.default_timeout)
            self._command_tx_thread = None

        if self._command_response_handler_thread is not None:
            try:
                self._command_rx_queue.put(_TERMINATE)
            # pylint: disable=broad-exception-caught
            except Exception as e:
                self.logger.error(f'Error shutting down command response handler thread: {e}')
            self._command_response_handler_thread.join(timeout=self.default_timeout)
            self._command_response_handler_thread = None

        if self._monitor_handler_thread is not None:
            try:
                self._monitor_rx_queue.put(_TERMINATE)
            # pylint: disable=broad-exception-caught
            except Exception as e:
                self.logger.error(f'Error shutting down monitor handler thread: {e}')
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
                # pylint: disable=broad-exception-caught
                except Exception:
                    #self.logger.error(f'Error shutting down command socket: {e}')
                    pass

            if self._monitor_socket is not None:
                try:
                    self._monitor_socket.shutdown(socket.SHUT_RDWR)
            # pylint: disable=broad-exception-caught
                except Exception:
                    #self.logger.error('Error shutting down monitor socket. ' + str(e))
                    pass

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

            self.logger.debug(f'Setting checkpoint {n}')

            if n not in checkpoints_dict:
                checkpoints_dict[n] = list()
            event = InterruptableEvent(n)
            checkpoints_dict[n].append(event)

            if send_to_robot:
                self._send_command('SetCheckpoint', [n])

            return event

    def _invalidate_checkpoints(self, message, forced: bool):
        """Unblock all waiting checkpoints and have them throw InterruptException

        Args:
            message (str): Message for the user that describes why the checkpoint was discarded.
            forced (bool): True  -> Force discarding checkpoints
                           False -> Discard only if robot has older version not supporting MX_ST_CHECKPOINT_DISCARDED
        """
        if not forced and self._robot_info.supports_checkpoint_discarded:
            # Don't discard, the robot supports MX_ST_CHECKPOINT_DISCARDED and will tell us which to discard
            return

        for checkpoints_dict in [self._internal_checkpoints, self._user_checkpoints]:
            for checkpoints_list in checkpoints_dict.values():
                for event in checkpoints_list:
                    event.abort(message)
            checkpoints_dict.clear()

        self._internal_checkpoint_counter = MX_CHECKPOINT_ID_MAX + 1

    def _send_motion_command(self, command: str, args: Union[str, list, tuple] = None):
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
            self._send_command(command, args)
            if self._enable_synchronous_mode:
                checkpoint = self._set_checkpoint_internal()

        if self._enable_synchronous_mode:
            checkpoint.wait()

    def _send_immediate_command(self, command: str, args: Optional[Union[str, list, tuple]],
                                event: Optional[InterruptableEvent]):
        """Send generic 'immediate' command with support for synchronous mode and locking.

        Parameters
        ----------
        command : string
            The command to send.
        args : list
            List of arguments to be sent.

        """
        if self._enable_synchronous_mode:
            self._send_sync_command(command, args, event, timeout=self.default_timeout)
        else:
            with self._main_lock:
                self._send_command(command, args)

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
        #pylint: disable=protected-access
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
        """ Awake waitable events created by _send_custom_command calls that match a received response """
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
            #pylint: disable=protected-access
            if event and event._abort_on_error:
                event.abort(message)

    def _invalidate_interruptable_events_on_clear_motion(self, message=""):
        '''Following robot motion cleared, unblock all appropriate interruptable events and have them
        throw InterruptException.'''
        for event_weakref in self._custom_response_events:
            event: InterruptableEvent = event_weakref()
            #pylint: disable=protected-access
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
                    self._invalidate_checkpoints('robot is in error', forced=False)
                elif response.id == mx_st.MX_ST_ALREADY_ERR:
                    self._invalidate_checkpoints('robot is not activated', forced=False)

                elif response.id == mx_st.MX_ST_CHECKPOINT_REACHED:
                    self._handle_checkpoint(response, discarded=False)

                elif response.id == mx_st.MX_ST_CHECKPOINT_DISCARDED:
                    self._handle_checkpoint(response, discarded=True)

                elif response.id == mx_st.MX_ST_CLEAR_MOTION:
                    if self._clear_motion_requests <= 1:
                        self._clear_motion_requests = 0
                        self._callback_queue.put('on_motion_cleared')
                        # Invalidate checkpoints and appropriate interruptable events
                        # Note: We called _invalidate_interruptable_events_on_clear_motion already when ClearMotion
                        #       was called. We don't want to call it on robot response, because checkpoint or
                        #       interruptable event that we have created immediately after calling ClearMotion are
                        #       valid and must not be cleared.
                        #       If robot spontaneously sends MX_ST_CLEAR_MOTION, other condition will awake these
                        #       checkpoints/events anyways (like robot in error, PSTOP2, etc.)
                        #message = 'Robot motion was cleared'
                        #self._invalidate_checkpoints(message)
                        #self._invalidate_interruptable_events_on_clear_motion(message)
                    else:
                        self._clear_motion_requests -= 1
                    self._check_motion_clear_done()

                elif response.id == mx_st.MX_ST_BRAKES_ON:
                    self._set_brakes_engaged(True)

                elif response.id == mx_st.MX_ST_BRAKES_OFF:
                    self._set_brakes_engaged(False)

                elif response.id == mx_st.MX_ST_CONNECTION_WATCHDOG:
                    self._set_connection_watchdog_enabled(self._parse_response_bool(response)[0])

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
        # Remember the last time we've received a message from the robot
        self._rx_timestamp = time.monotonic()

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
                self._handle_cycle_end(response)
                self._refresh_auto_connection_watchdog()
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

        if response.id == mx_st.MX_ST_GET_COLLISION_STATUS:
            self._handle_collision_status_response(response)

        if response.id == mx_st.MX_ST_GET_WORK_ZONE_STATUS:
            self._handle_work_zone_status_response(response)

        if response.id == 2011:  # Legacy motion status (now included in MX_ST_GET_STATUS_ROBOT)
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

        elif response.id == mx_st.MX_ST_RT_IO_STATUS:
            self._handle_io_status(response)

        elif response.id == mx_st.MX_ST_RT_OUTPUT_STATE:
            self._handle_output_state(response)

        elif response.id == mx_st.MX_ST_RT_INPUT_STATE:
            self._handle_input_state(response)

        elif response.id == mx_st.MX_ST_RT_VACUUM_STATE:
            self._handle_vacuum_state(response)

        elif response.id == mx_st.MX_ST_RT_VACUUM_PRESSURE:
            self._handle_vacuum_pressure(response)

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

        elif response.id == mx_st.MX_ST_ESTOP:
            self._handle_estop_state(response)

        elif response.id == mx_st.MX_ST_PSTOP1:
            self._handle_pstop1_state(response)

        elif response.id == mx_st.MX_ST_PSTOP2:
            self._handle_pstop2_state(response)

        elif response.id == mx_st.MX_ST_SAFE_STOP_OPERATION_MODE:
            self._handle_operation_mode_stop_state(response)

        elif response.id == mx_st.MX_ST_SAFE_STOP_ENABLING_DEVICE_RELEASED:
            self._handle_enabling_device_released_stop_state(response)

        elif response.id == mx_st.MX_ST_SAFE_STOP_VOLTAGE_FLUCTUATION:
            self._handle_voltage_fluctuation_stop_state(response)

        elif response.id == mx_st.MX_ST_SAFE_STOP_REBOOT:
            self._handle_reboot_stop_state(response)

        elif response.id == mx_st.MX_ST_SAFE_STOP_REDUNDANCY_FAULT:
            self._handle_redundancy_fault_stop_state(response)

        elif response.id == mx_st.MX_ST_SAFE_STOP_STANDSTILL_FAULT:
            self._handle_standstill_fault_stop_state(response)

        elif response.id == mx_st.MX_ST_SAFE_STOP_CONNECTION_DROPPED:
            self._handle_connection_dropped_stop_state(response)

        elif response.id == mx_st.MX_ST_SAFE_STOP_MINOR_ERROR:
            self._handle_minor_error_stop_state(response)

        elif response.id == mx_st.MX_ST_GET_OPERATION_MODE:
            robot_operation_mode = self._parse_response_int(response)[0]
            self._set_robot_operation_mode(robot_operation_mode)

        elif response.id == mx_st.MX_ST_TORQUE_LIMIT_STATUS:
            torque_exceeded = self._parse_response_int(response)[0]
            if torque_exceeded:
                self.logger.info(
                    f'Torque limit exceeded. Current torque: {args_to_string(self._robot_rt_data.rt_joint_torq.data)}')

        elif response.id == mx_st.MX_ST_RT_TARGET_JOINT_POS:
            self._robot_rt_data.rt_target_joint_pos.update_from_csv(response.data)
            if self._is_in_sync():
                self._robot_events.on_joints_updated.set()

        elif response.id == mx_st.MX_ST_RT_TARGET_CART_POS:
            self._robot_rt_data.rt_target_cart_pos.update_from_csv(response.data, allowed_nb_val=[4, 6])
            if self._is_in_sync():
                self._robot_events.on_pose_updated.set()

        elif response.id == mx_st.MX_ST_RT_TARGET_JOINT_VEL:
            self._robot_rt_data.rt_target_joint_vel.update_from_csv(response.data)
        elif response.id == mx_st.MX_ST_RT_TARGET_CART_VEL:
            self._robot_rt_data.rt_target_cart_vel.update_from_csv(response.data, allowed_nb_val=[4, 6])

        elif response.id == mx_st.MX_ST_RT_TARGET_JOINT_TORQ:
            self._robot_rt_data.rt_target_joint_torq.update_from_csv(response.data)

        elif response.id == mx_st.MX_ST_RT_TARGET_CONF:
            self._robot_rt_data.rt_target_conf.update_from_csv(response.data)
        elif response.id == mx_st.MX_ST_RT_TARGET_CONF_TURN:
            self._robot_rt_data.rt_target_conf_turn.update_from_csv(response.data)

        elif response.id == mx_st.MX_ST_RT_JOINT_POS:
            self._robot_rt_data.rt_joint_pos.update_from_csv(response.data)
        elif response.id == mx_st.MX_ST_RT_CART_POS:
            self._robot_rt_data.rt_cart_pos.update_from_csv(response.data, allowed_nb_val=[4, 6])
        elif response.id == mx_st.MX_ST_RT_JOINT_VEL:
            self._robot_rt_data.rt_joint_vel.update_from_csv(response.data)
        elif response.id == mx_st.MX_ST_RT_JOINT_TORQ:
            self._robot_rt_data.rt_joint_torq.update_from_csv(response.data)
        elif response.id == mx_st.MX_ST_RT_CART_VEL:
            self._robot_rt_data.rt_cart_vel.update_from_csv(response.data, allowed_nb_val=[4, 6])
        elif response.id == mx_st.MX_ST_RT_EFFECTIVE_TIME_SCALING:
            self._handle_effective_time_scaling_data(response)

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
            self._robot_rt_data.rt_wrf.update_from_csv(response.data, allowed_nb_val=[4, 6])

        elif response.id == mx_st.MX_ST_RT_TRF:
            self._robot_rt_data.rt_trf.update_from_csv(response.data, allowed_nb_val=[4, 6])

        elif response.id == mx_st.MX_ST_RT_CHECKPOINT:
            self._robot_rt_data.rt_checkpoint.update_from_csv(response.data)

        elif response.id == mx_st.MX_ST_IMPOSSIBLE_RESET_ERR:
            self.logger.error(response.data)
            # Don't abort the event otherwise we can't try later to reset the error because WaitErrorReset
            # remains "aborted" for ever
            #message = response.data
            #self._robot_events.on_error_reset.abort(message)

        elif response.id == mx_st.MX_ST_GET_REAL_TIME_MONITORING:
            self._handle_get_realtime_monitoring_response(response)

        elif response.id == mx_st.MX_ST_FW_UPDATE_PROGRESS:
            self._handle_fw_update_progress(response)

        elif response.id == mx_st.MX_ST_SYNC_CMD_QUEUE:
            self._handle_sync_response(response)

        elif response.id == mx_st.MX_ST_GET_NETWORK_CONFIG:
            self._handle_get_network_config_response(response)

        elif response.id == mx_st.MX_ST_TIME_SCALING:
            self._handle_get_time_scaling_response(response)

        elif response.id == mx_st.MX_ST_ACTIVATION_ERR:
            # Awake any thread waiting on WaitActivated/WaitHomed
            self._robot_events.on_activated.abort('Activation error')
            self._robot_events.on_homed.abort('Activation error')

        elif response.id == mx_st.MX_ST_HOMING_ERR:
            # Awake any thread waiting on WaitHomed
            self._robot_events.on_homed.abort('Homing error')

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
                if self._first_robot_status_received and self._robot_status.activation_state != activated:
                    self.logger.info(f'Robot is activated.')
                self._robot_events.on_deactivated.clear()
                self._robot_events.on_activated.set()
                self._set_brakes_engaged(False)
                self._callback_queue.put('on_activated')
            else:
                if self._first_robot_status_received and self._robot_status.activation_state != activated:
                    self.logger.info(f'Robot is deactivated.')
                self._robot_events.on_activated.clear()
                self._robot_events.on_deactivated.set()
                self._set_brakes_engaged(True)
                self._callback_queue.put('on_deactivated')
                # Invalidate checkpoints and appropriate interruptable events
                message = 'Robot was deactivated'
                self._invalidate_checkpoints(message, forced=False)
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

    def _set_error_status(self, error_status: bool, error_code: Optional[int] = None):
        """Update the "error" state of the robot

        Parameters
        ----------
        error_status : bool
            Robot is in error or not
        """
        if not self._first_robot_status_received or self._robot_status.error_status != error_status:
            if error_status:
                message = "robot is in error"
                self._invalidate_checkpoints(message, forced=False)
                self._invalidate_interruptable_events_on_error(message)
                self._robot_events.on_error.set()
                self._robot_events.abort_all_on_error(message)
                self._robot_events.on_error_reset.clear()
                self._callback_queue.put('on_error')
                # Always consider the robot paused when entering error state
                self._set_paused(True)
            else:
                self._robot_events.clear_abort_all()
                self._robot_events.on_error.clear()
                self._robot_events.on_error_reset.set()
                self._callback_queue.put('on_error_reset')
            self._robot_status.error_status = error_status

        self._robot_status.error_code = error_code

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
            if paused:
                self._check_motion_clear_done()

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
            self._check_motion_clear_done()

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

    def _set_connection_watchdog_enabled(self, enabled: bool):
        self._robot_status.connection_watchdog_enabled = enabled

    def _set_robot_operation_mode(self, robot_operation_mode: MxRobotOperationMode):
        """Update the "robot_operation_mode" from robot safety status

        Parameters
        ----------
        robot_operation_mode : MxRobotOperationMode
            New robot operation mode to set in robot state
        """
        self._robot_safety_status.robot_operation_mode = robot_operation_mode
        # Notify that some change occurred in safety stop state
        self._callback_queue.put('on_safety_stop_state_change')

    def _set_reset_ready(self, reset_ready: bool):
        """Update the "reset_ready" from robot safety status

        Parameters
        ----------
        reset_ready : bool
            New flag to set in robot state
        """
        self._robot_safety_status.reset_ready = reset_ready
        # Notify that some change occurred in safety stop state
        self._callback_queue.put('on_safety_stop_state_change')

    def _check_motion_clear_done(self):
        """This function will set (unblock) on_motion_cleared once motion clear is confirmed, i.e. once no more
           ClearMotion response is pending and once EOB is confirmed."""
        wait_for_eob = self.GetRobotInfo().version.is_at_least(9, 3, 0)
        if not self._robot_events.on_motion_cleared.is_set():
            if self._clear_motion_requests == 0:
                if not wait_for_eob or (self._robot_status.end_of_block_status
                                        and self._robot_status.pause_motion_status):
                    self._robot_events.on_motion_cleared.set()

    def _handle_cycle_end(self, response: Message):
        """Handle a robot message of type mx_st.MX_ST_RT_CYCLE_END"""
        self._robot_rt_data.cycle_count += 1
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

    def _refresh_auto_connection_watchdog(self, force=False):
        """Send a connection watchdog refresh to the robot using appropriate timeout

        Args:
            force (bool, optional): Force sending refresh now even if minimum elapsed time is not yet elapsed.
        """
        if not self._auto_connection_watchdog:
            return
        # Use 4x monitoring interval by default
        now = time.monotonic()
        # Max every 10ms
        if force or now - self._auto_connection_watchdog_last > 0.01:
            self._auto_connection_watchdog_last = now
            timeout = self._monitoring_interval * 4
            # But make sure not to use a timer that's exaggeratedly small (Python is not THAT real time!)
            if timeout < 0.25:
                timeout = 0.25
            self.ConnectionWatchdog(timeout)

    def _handle_motion_status_response(self, response: Message):
        """Parse robot motion response and update status fields and events.
           Note that this message is normally automatically followed (at least on the monitoring port)
           by robot status (MX_ST_GET_STATUS_ROBOT).

        Parameters
        ----------
        response : Message object
            Motion status response to parse and handle.

        """
        if response.json_data:
            # JSON format.
            self._using_legacy_json_api = True
            json_data = response.json_data[MX_JSON_KEY_DATA]
            self._set_paused(json_data[MX_JSON_KEY_MOTION_STATUS_HOLD])
            self._set_eob(json_data[MX_JSON_KEY_MOTION_STATUS_EOB])

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
        if response.json_data:
            # JSON format.
            json_data = response.json_data[MX_JSON_KEY_DATA]
            json_robot_status = None
            json_motion_status = None
            json_safety_status = None
            if MX_JSON_KEY_ROBOT_STATUS in json_data:
                json_robot_status = json_data[MX_JSON_KEY_ROBOT_STATUS]
            if MX_JSON_KEY_MOTION_STATUS in json_data:
                json_motion_status = json_data[MX_JSON_KEY_MOTION_STATUS]
            if MX_JSON_KEY_SAFETY_STATUS in json_data:
                json_safety_status = json_data[MX_JSON_KEY_SAFETY_STATUS]

            if json_robot_status is None:
                # Legacy JSON format
                json_robot_status = json_data

            if json_robot_status is not None:
                # Read some states only from JSON (only those that don't have a distinct status event)
                self._set_activated(
                    json_robot_status[MX_JSON_KEY_STATUS_ROBOT_STATE] >= MxRobotState.MX_ROBOT_STATE_ACTIVATED)
                self._set_homed(json_robot_status[MX_JSON_KEY_STATUS_ROBOT_STATE] == MxRobotState.MX_ROBOT_STATE_RUN)
                self._set_sim_mode(json_robot_status[MX_JSON_KEY_STATUS_ROBOT_SIM])
                self._set_recovery_mode(json_robot_status[MX_JSON_KEY_STATUS_ROBOT_RECOVERY])
                error_code = json_robot_status[MX_JSON_KEY_STATUS_ROBOT_ERR]
                self._set_error_status(error_status=error_code != 0, error_code=error_code)
                self._set_brakes_engaged(json_robot_status[MX_JSON_KEY_STATUS_ROBOT_BRAKES] != 0)
            if json_motion_status is not None:
                self._set_paused(json_motion_status[MX_JSON_KEY_MOTION_STATUS_HOLD])
                self._set_eob(json_motion_status[MX_JSON_KEY_MOTION_STATUS_EOB])
            if json_safety_status is not None:
                self._set_reset_ready(json_safety_status[MX_JSON_KEY_SAFETY_STATUS_RESET_READY])
                if MX_JSON_KEY_SAFETY_STOP in json_safety_status:
                    json_safety_stop = json_safety_status[MX_JSON_KEY_SAFETY_STOP]
                    if MX_JSON_KEY_SAFETY_PSU_INPUTS_MASK in json_safety_stop:
                        self._robot_psu_inputs.set_psu_input_mask(json_safety_stop[MX_JSON_KEY_SAFETY_PSU_INPUTS_MASK])
                if MX_JSON_KEY_SAFETY_STOP_STATIC_MASKS in json_safety_status:
                    json_stop_masks = json_safety_status[MX_JSON_KEY_SAFETY_STOP_STATIC_MASKS]
                    if MX_JSON_KEY_MASK_CLEARED_BY_PSU in json_stop_masks:
                        self._robot_safety_status.static_masks.clearedByPsu = json_stop_masks[
                            MX_JSON_KEY_MASK_CLEARED_BY_PSU]
                    if MX_JSON_KEY_MASK_WITH_VM_OFF in json_stop_masks:
                        self._robot_safety_status.static_masks.withVmOff = json_stop_masks[MX_JSON_KEY_MASK_WITH_VM_OFF]
                    if MX_JSON_KEY_MASK_MANUAL_MODE in json_stop_masks:
                        self._robot_safety_status.static_masks.maskedInManualMode = json_stop_masks[
                            MX_JSON_KEY_MASK_MANUAL_MODE]
                    else:
                        # Legacy robot version, let's use hard-coded value that was used by the robot on that build
                        self._robot_safety_status.static_masks.maskedInManualMode = int(
                            MxSafeStopCategory.MX_SAFE_STOP_PSTOP1) | int(MxSafeStopCategory.MX_SAFE_STOP_PSTOP2)
        else:
            # Legacy format.
            status_flags = self._parse_response_bool(response)

            self._set_activated(status_flags[0])
            self._set_homed(status_flags[1])
            self._set_sim_mode(status_flags[2])
            self._set_error_status(status_flags[3])
            self._set_paused(status_flags[4])
            self._set_eob(status_flags[5])
            #eom(status_flag[6])

        self._first_robot_status_received = True

        if self._is_in_sync():
            self._robot_events.on_status_updated.set()
        self._callback_queue.put('on_status_updated')

    def _handle_collision_status_response(self, response: Message):
        """Parse robot collision status response and update status fields and events.

        Parameters
        ----------
        response : Message object
            Robot status response to parse and handle.

        """
        assert response.id == mx_st.MX_ST_GET_COLLISION_STATUS
        parsed_response = self._parse_response_int(response)
        status = self._robot_collision_status.self_collision_status
        status.set_from_response(parsed_response)

    def _handle_work_zone_status_response(self, response: Message):
        """Parse robot work zone status response and update status fields and events.

        Parameters
        ----------
        response : Message object
            Robot status response to parse and handle.

        """
        assert response.id == mx_st.MX_ST_GET_WORK_ZONE_STATUS
        parsed_response = self._parse_response_int(response)
        status = self._robot_collision_status.work_zone_status
        status.set_from_response(parsed_response)

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

    def _handle_safety_stop_common(self):
        """Code called after state of any safety stop signal has changed"""
        active_safety_stops = self._robot_safety_status.stop_mask & ~self._robot_safety_status.stop_resettable_mask

        if active_safety_stops != 0:
            # There are active safety stop signals (non-resettable)
            if self._robot_events.on_safety_stop_reset.is_set():
                # First safety stop event occurring -> Call appropriate callback
                self._callback_queue.put('on_safety_stop')
            self._robot_events.on_safety_stop_reset.clear()
            self._robot_events.on_safety_stop_resettable.clear()
        elif self._robot_safety_status.stop_resettable_mask != 0 and active_safety_stops == 0:
            # There are safety stop signals that are ready to be reset
            self._robot_events.on_safety_stop_reset.clear()
            self._robot_events.on_safety_stop_resettable.set()
            self._callback_queue.put('on_safety_stop_resettable')
        elif active_safety_stops == 0:
            # There are no safety stop signals
            self._robot_events.on_safety_stop_reset.set()
            self._robot_events.on_safety_stop_resettable.set()
            self._callback_queue.put('on_safety_stop_reset')

        # Awake any thread awaiting on WaitSafetyStopStateChange
        self._robot_events.on_safety_stop_state_change.set()

        # Notify that some change occurred in safety stop state
        self._callback_queue.put('on_safety_stop_state_change')

    def _handle_estop_state(self, response: Message):
        """Handle EStop state change.

        Parameters
        ----------
        response : Message object
            Robot status response to parse and handle.

        """
        self._robot_safety_status.set_estop_state(self._parse_response_int(response)[0])
        # Keep legacy field up-to-date
        self._robot_status.estopState = self._robot_safety_status.estop_state

        # Call deprecated callbacks
        if self._robot_safety_status.estop_state == MxStopState.MX_STOP_STATE_ACTIVE:
            self._robot_events.on_estop.set()
            self._robot_events.on_estop_reset.clear()
            self._robot_events.on_estop_resettable.clear()
            self._callback_queue.put('on_estop')
        elif self._robot_safety_status.estop_state == MxStopState.MX_STOP_STATE_RESETTABLE:
            self._robot_events.on_estop.set()
            self._robot_events.on_estop_reset.clear()
            self._robot_events.on_estop_resettable.set()
            self._callback_queue.put('on_estop_resettable')
        else:
            self._robot_events.on_estop.clear()
            self._robot_events.on_estop_reset.set()
            self._robot_events.on_estop_resettable.set()
            self._callback_queue.put('on_estop_reset')

        self._handle_safety_stop_common()

    def _handle_pstop1_state(self, response: Message):
        """Handle PStop1 state change.

        Parameters
        ----------
        response : Message object
            Robot status response to parse and handle.

        """
        self._robot_safety_status.set_pstop1_state(self._parse_response_int(response)[0])
        self._handle_safety_stop_common()

    def _handle_pstop2_state(self, response: Message):
        """Handle PStop2 state change.

        Parameters
        ----------
        response : Message object
            Robot status response to parse and handle.

        """
        self._robot_safety_status.set_pstop2_state(self._parse_response_int(response)[0])
        # Keep legacy field up-to-date
        self._robot_status.pstop2State = self._robot_safety_status.pstop2_state

        if self._robot_safety_status.pstop2_state == MxStopState.MX_STOP_STATE_ACTIVE:
            # Invalidate checkpoints and appropriate interruptable events
            message = 'Robot is in PSTOP2 condition'
            self._invalidate_checkpoints(message, forced=False)
            self._invalidate_interruptable_events_on_clear_motion(message)

        # Call deprecated callbacks
        if self._robot_safety_status.pstop2_state == MxStopState.MX_STOP_STATE_ACTIVE:
            self._robot_events.on_pstop2.set()
            self._robot_events.on_pstop2_reset.clear()
            self._robot_events.on_pstop2_resettable.clear()
            self._callback_queue.put('on_pstop2')
        elif self._robot_safety_status.pstop2_state == MxStopState.MX_STOP_STATE_RESETTABLE:
            self._robot_events.on_pstop2.set()
            self._robot_events.on_pstop2_reset.clear()
            self._robot_events.on_pstop2_resettable.set()
            self._callback_queue.put('on_pstop2_resettable')
        else:
            self._robot_events.on_pstop2.clear()
            self._robot_events.on_pstop2_reset.set()
            self._robot_events.on_pstop2_resettable.set()
            self._callback_queue.put('on_pstop2_reset')

        self._handle_safety_stop_common()

    def _handle_operation_mode_stop_state(self, response: Message):
        """Handle an operation mode safety stop state change.

        Parameters
        ----------
        response : Message object
            Robot status response to parse and handle.

        """
        self._robot_safety_status.set_operation_mode_stop_state(self._parse_response_int(response)[0])
        self._handle_safety_stop_common()

    def _handle_enabling_device_released_stop_state(self, response: Message):
        """Handle an enabling device released safety stop state change.

        Parameters
        ----------
        response : Message object
            Robot status response to parse and handle.

        """
        self._robot_safety_status.set_enabling_device_released_stop_state(self._parse_response_int(response)[0])
        self._handle_safety_stop_common()

    def _handle_voltage_fluctuation_stop_state(self, response: Message):
        """Handle an voltage fluctuation safety stop state change.

        Parameters
        ----------
        response : Message object
            Robot status response to parse and handle.

        """
        self._robot_safety_status.set_voltage_fluctuation_stop_state(self._parse_response_int(response)[0])
        self._handle_safety_stop_common()

    def _handle_reboot_stop_state(self, response: Message):
        """Handle a "robot just rebooted" safety stop state change.

        Parameters
        ----------
        response : Message object
            Robot status response to parse and handle.

        """
        self._robot_safety_status.set_reboot_stop_state(self._parse_response_int(response)[0])
        self._handle_safety_stop_common()

    def _handle_redundancy_fault_stop_state(self, response: Message):
        """Handle a "redundancy fault" safety stop state change.

        Parameters
        ----------
        response : Message object
            Robot status response to parse and handle.

        """
        self._robot_safety_status.set_redundancy_fault_stop_state(self._parse_response_int(response)[0])
        self._handle_safety_stop_common()

    def _handle_standstill_fault_stop_state(self, response: Message):
        """Handle a "standstill fault" safety stop state change.

        Parameters
        ----------
        response : Message object
            Robot status response to parse and handle.

        """
        self._robot_safety_status.set_standstill_fault_stop_state(self._parse_response_int(response)[0])
        self._handle_safety_stop_common()

    def _handle_connection_dropped_stop_state(self, response: Message):
        """Handle a "connection dropped" safety stop state change.

        Parameters
        ----------
        response : Message object
            Robot status response to parse and handle.

        """
        self._robot_safety_status.set_connection_dropped_stop_state(self._parse_response_int(response)[0])
        self._handle_safety_stop_common()

    def _handle_minor_error_stop_state(self, response: Message):
        """Handle a "minor error" safety stop state change.

        Parameters
        ----------
        response : Message object
            Robot status response to parse and handle.

        """
        self._robot_safety_status.set_minor_error_stop_state(self._parse_response_int(response)[0])
        self._handle_safety_stop_common()

    def _handle_get_time_scaling_response(self, response: Message):
        """Handle a time scaling changes.

        Parameters
        ----------
        response : Message object
            Robot status response to parse and handle.
        """
        assert response.id == mx_st.MX_ST_TIME_SCALING
        self._robot_events.on_time_scaling_changed.set()

    def _handle_effective_time_scaling_data(self, response: Message):
        """Handle an effective time scaling rt data.

        Parameters
        ----------
        response : Message object
            Robot status response to parse and handle.
        """
        assert response.id == mx_st.MX_ST_RT_EFFECTIVE_TIME_SCALING
        self._robot_rt_data.rt_effective_time_scaling.update_from_csv(response.data)

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
        if response.json_data:
            # JSON format.
            json_data = response.json_data[MX_JSON_KEY_DATA]
            self._robot_rt_data.rt_external_tool_status.timestamp = response.json_data[MX_JSON_KEY_TIMESTAMP_US]
            status_flags = self._robot_rt_data.rt_external_tool_status.data
            status_flags[0] = json_data[MX_JSON_KEY_EXTTOOL_STATUS_SIM_TYPE]
            status_flags[1] = json_data[MX_JSON_KEY_EXTTOOL_STATUS_PHYSICAL_TYPE]
            status_flags[2] = json_data[MX_JSON_KEY_EXTTOOL_STATUS_HOMED]
            status_flags[3] = json_data[MX_JSON_KEY_EXTTOOL_STATUS_ERROR]
            status_flags[4] = json_data[MX_JSON_KEY_EXTTOOL_STATUS_OVERHEAT]
            self._external_tool_status.comm_err_warning = json_data[MX_JSON_KEY_EXTTOOL_STATUS_COMM_ERR]
        else:
            self._robot_rt_data.rt_external_tool_status.update_from_csv(response.data)
            status_flags = self._robot_rt_data.rt_external_tool_status.data

        self._external_tool_status.sim_tool_type = status_flags[0]
        self._external_tool_status.physical_tool_type = status_flags[1]
        self._external_tool_status.homing_state = status_flags[2] != 0
        self._external_tool_status.error_status = status_flags[3] != 0
        self._external_tool_status.overload_error = status_flags[4] != 0

        if self._is_in_sync():
            self._robot_events.on_external_tool_status_updated.set()
            self._callback_queue.put('on_external_tool_status_updated')

    def _handle_get_network_config_response(self, response: Message):
        """ Handle robot response to GetNetworkConfig command """

        assert response.id == mx_st.MX_ST_GET_NETWORK_CONFIG
        if response.json_data:
            # JSON format.
            json_data = response.json_data[MX_JSON_KEY_DATA]

            self._network_config.dhcp = False
            if json_data[MX_JSON_KEY_NETWORK_CONFIG_DHCP] == 'true':
                self._network_config.dhcp = True

            self._network_config.ip = json_data[MX_JSON_KEY_NETWORK_CONFIG_IP]
            self._network_config.mask = json_data[MX_JSON_KEY_NETWORK_CONFIG_MASK]
            self._network_config.gateway = json_data[MX_JSON_KEY_NETWORK_CONFIG_GATEWAY]
            self._network_config.mac = json_data[MX_JSON_KEY_NETWORK_CONFIG_MAC]
            self._network_config.name = json_data[MX_JSON_KEY_NETWORK_CONFIG_ROBOT_NAME]

        if self._is_in_sync():
            self._robot_events.on_network_config_updated.set()

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
            if self._gripper_state.holding_part:
                self._robot_events.on_holding_part.set()
                self._robot_events.on_released_part.clear()
            else:
                self._robot_events.on_holding_part.clear()
                self._robot_events.on_released_part.set()

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

    def _handle_io_status(self, response: Message):
        """Parse IO status and update status fields and events.

        Parameters
        ----------
        response : Message object
            Io state to parse and handle.

        """
        assert response.id == mx_st.MX_ST_RT_IO_STATUS

        # Extract the Bank id
        bank_id = MxIoBankId.MX_IO_BANK_ID_UNDEFINED
        values = []
        if response.json_data:
            bank_id = response.json_data[MX_JSON_KEY_DATA][MX_JSON_KEY_IO_STATUS_BANK_ID]
        else:
            values = tools.string_to_numbers(response.data)
            bank_id = values[1]

        new_io_status = IoStatus()
        new_io_status_ts = TimestampedData.zeros(4)
        if bank_id == MxIoBankId.MX_IO_BANK_ID_IO_MODULE:
            new_io_status = self._io_module_status
        elif bank_id == MxIoBankId.MX_IO_BANK_ID_SIG_GEN:
            new_io_status = self._sig_gen_status
        else:
            return
        new_io_status.bank_id = bank_id
        prev_sim_mode = new_io_status.sim_mode

        if response.json_data:
            # JSON format. Convert into timestamped data
            json_data = response.json_data[MX_JSON_KEY_DATA]

            # Extract JSON data
            if MX_JSON_KEY_IO_STATUS_PRESENT in json_data:
                new_io_status.present = json_data[MX_JSON_KEY_IO_STATUS_PRESENT]
            if MX_JSON_KEY_IO_STATUS_NB_INPUTS in json_data:
                new_io_status.nb_inputs = json_data[MX_JSON_KEY_IO_STATUS_NB_INPUTS]
            if MX_JSON_KEY_IO_STATUS_NB_OUTPUTS in json_data:
                new_io_status.nb_outputs = json_data[MX_JSON_KEY_IO_STATUS_NB_OUTPUTS]
            if MX_JSON_KEY_IO_STATUS_SIM_MODE in json_data:
                new_io_status.sim_mode = json_data[MX_JSON_KEY_IO_STATUS_SIM_MODE]
            if MX_JSON_KEY_IO_STATUS_ERROR in json_data:
                new_io_status.error = json_data[MX_JSON_KEY_IO_STATUS_ERROR]
            if MX_JSON_KEY_TIMESTAMP_US in response.json_data:
                new_io_status.timestamp = response.json_data[MX_JSON_KEY_TIMESTAMP_US]
            # Convert into timestamped data (equivalent to text API)
            new_io_status_ts.timestamp = new_io_status.timestamp
            new_io_status_ts.data[0] = new_io_status.bank_id
            new_io_status_ts.data[1] = 1 if new_io_status.present else 0
            new_io_status_ts.data[2] = 1 if new_io_status.sim_mode else 0
            new_io_status_ts.data[3] = new_io_status.error
        else:
            # Extract API values
            new_io_status_ts.update_from_csv(response.data)
            new_io_status.timestamp = values[0]
            new_io_status.present = True if values[2] != 0 else False
            new_io_status.sim_mode = True if values[3] != 0 else False
            new_io_status.error = values[4]
            # Note: nb_inputs and nb_outputs is updated by _handle_input_state / _handle_output_state

        # Store back into our context
        if bank_id == MxIoBankId.MX_IO_BANK_ID_IO_MODULE:
            self._io_module_status = new_io_status
            self._robot_rt_data.rt_io_module_status = new_io_status_ts
        elif bank_id == MxIoBankId.MX_IO_BANK_ID_SIG_GEN:
            self._sig_gen_status = new_io_status
            self._robot_rt_data.rt_sig_gen_status = new_io_status_ts

        # Update events/callbacks
        if not self._first_robot_status_received or prev_sim_mode != new_io_status.sim_mode:
            if bank_id == MxIoBankId.MX_IO_BANK_ID_IO_MODULE:
                if new_io_status.sim_mode:
                    self._robot_events.on_io_sim_enabled.set()
                    self._robot_events.on_io_sim_disabled.clear()
                    self._callback_queue.put('on_io_sim_enabled')
                else:
                    self._robot_events.on_io_sim_enabled.clear()
                    self._robot_events.on_io_sim_disabled.set()
                    self._callback_queue.put('on_io_sim_disabled')

        if self._is_in_sync():
            self._robot_events.on_io_status_updated.set()

    def _handle_output_state(self, response: Message):
        """Parse digital output state and update status fields and events.

        Parameters
        ----------
        response : Message object
            Digital output state to parse and handle.

        """
        assert response.id == mx_st.MX_ST_RT_OUTPUT_STATE
        values = tools.string_to_numbers(response.data)
        changed = False
        if len(values) > 1:
            timestamp = values[0]
            bank_id = values[1]
            if bank_id == MxIoBankId.MX_IO_BANK_ID_IO_MODULE:
                changed = True
                self._robot_rt_data.rt_io_module_outputs.update_from_data(timestamp, values[2:])
                self._io_module_status.nb_outputs = len(self._robot_rt_data.rt_io_module_outputs.data)
            elif bank_id == MxIoBankId.MX_IO_BANK_ID_SIG_GEN:
                changed = True
                self._robot_rt_data.rt_sig_gen_outputs.update_from_data(timestamp, values[2:])
                self._sig_gen_status.nb_outputs = len(self._robot_rt_data.rt_sig_gen_outputs.data)

        if changed and self._is_in_sync():
            self._robot_events.on_output_state_updated.set()
            self._callback_queue.put('on_output_state_updated')

    def _handle_input_state(self, response: Message):
        """Parse digital input state and update status fields and events.

        Parameters
        ----------
        response : Message object
            Digital input state to parse and handle.

        """
        assert response.id == mx_st.MX_ST_RT_INPUT_STATE
        values = tools.string_to_numbers(response.data)
        changed = False
        if len(values) > 1:
            timestamp = values[0]
            bank_id = values[1]
            if bank_id == MxIoBankId.MX_IO_BANK_ID_IO_MODULE:
                changed = True
                self._robot_rt_data.rt_io_module_inputs.update_from_data(timestamp, values[2:])
                self._io_module_status.nb_inputs = len(self._robot_rt_data.rt_io_module_inputs.data)
            elif bank_id == MxIoBankId.MX_IO_BANK_ID_SIG_GEN:
                changed = True
                self._robot_rt_data.rt_sig_gen_inputs.update_from_data(timestamp, values[2:])
                self._sig_gen_status.nb_inputs = len(self._robot_rt_data.rt_sig_gen_inputs.data)

        if changed and self._is_in_sync():
            self._robot_events.on_input_state_updated.set()
            self._callback_queue.put('on_input_state_updated')

    def _handle_vacuum_state(self, response: Message):
        """Parse vacuum state and update status fields and events.

        Parameters
        ----------
        response : Message object
            Vacuum grip state to parse and handle.

        """
        assert response.id == mx_st.MX_ST_RT_VACUUM_STATE
        self._robot_rt_data.rt_vacuum_state.update_from_csv(response.data)

        self._vacuum_state.bank_id = MxIoBankId.MX_IO_BANK_ID_IO_MODULE  #todo, this member doesn't exist
        self._vacuum_state.timestamp = self._robot_rt_data.rt_vacuum_state.timestamp
        self._vacuum_state.vacuum_on = bool(self._robot_rt_data.rt_vacuum_state.data[0])
        self._vacuum_state.purge_on = bool(self._robot_rt_data.rt_vacuum_state.data[1])
        self._vacuum_state.holding_part = bool(self._robot_rt_data.rt_vacuum_state.data[2])

        if self._is_in_sync():
            self._robot_events.on_vacuum_state_updated.set()
            self._callback_queue.put('on_vacuum_state_updated')
            if self._vacuum_state.holding_part:
                self._robot_events.on_holding_part.set()
                self._robot_events.on_released_part.clear()
            elif not self._vacuum_state.purge_on:
                self._robot_events.on_holding_part.clear()
                self._robot_events.on_released_part.set()
            if self._vacuum_state.purge_on:
                self._robot_events.on_vacuum_purge_done.clear()
            else:
                self._robot_events.on_vacuum_purge_done.set()

    def _handle_vacuum_pressure(self, response: Message):
        """Parse vacuum pressure and update fields and events.

        Parameters
        ----------
        response : Message object
            Vacuum pressure to parse and handle.

        """
        assert response.id == mx_st.MX_ST_RT_VACUUM_PRESSURE
        self._robot_rt_data.rt_vacuum_pressure.update_from_csv(response.data)

    def _handle_checkpoint(self, response: Message, discarded: bool):
        """Handle the checkpoint reached message from the robot, set the appropriate events, etc.

        Parameters
        ----------
        response : Message object
            Response message which includes the received checkpoint id.
        discarded : bool
            Tells if the checkpoint has been discarded or reached.

        """
        assert (response.id == mx_st.MX_ST_CHECKPOINT_REACHED or response.id == mx_st.MX_ST_CHECKPOINT_DISCARDED)
        checkpoint_id = int(response.data)

        self.logger.debug(f'Checkpoint {checkpoint_id} {"discarded"if discarded else "reached"}')

        # Check user checkpoints.
        abort_msg = 'Checkpoint discarded by the robot'
        if checkpoint_id in self._user_checkpoints and self._user_checkpoints[checkpoint_id]:
            if discarded:
                self._user_checkpoints[checkpoint_id].pop(0).abort(abort_msg)
            else:
                self._user_checkpoints[checkpoint_id].pop(0).set()
            # If list corresponding to checkpoint id is empty, remove the key from the dict.
            if not self._user_checkpoints[checkpoint_id]:
                self._user_checkpoints.pop(checkpoint_id)
            # If there are events are waiting on 'any checkpoint', set them all.
            if '*' in self._internal_checkpoints and self._internal_checkpoints['*']:
                for event in self._internal_checkpoints.pop('*'):
                    if discarded:
                        event.abort(abort_msg)
                    else:
                        event.set()

            if discarded:
                # Enqueue the on_checkpoint_discarded callback.
                self._callback_queue.put('on_checkpoint_discarded', checkpoint_id)
            else:
                # Enqueue the on_checkpoint_reached callback.
                self._callback_queue.put('on_checkpoint_reached', checkpoint_id)

        # Check internal checkpoints.
        elif checkpoint_id in self._internal_checkpoints and self._internal_checkpoints[checkpoint_id]:
            if discarded:
                self._internal_checkpoints[checkpoint_id].pop(0).abort(abort_msg)
            else:
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
        #pylint: disable=protected-access
        self._robot_rt_data._reset_enabled()

        # Following RT data are always accessible in the Python API even if not always sent by the robot, because this
        # Python API will construct them from mx_st.MX_ST_GET_JOINTS / mx_st.MX_ST_GET_POSE if necessary
        self._robot_rt_data.rt_target_joint_pos.enabled = True
        self._robot_rt_data.rt_target_cart_pos.enabled = True
        # Following RT data are always sent by the robot
        if self._robot_info.version.is_at_least(9, 0):
            self._robot_rt_data.rt_target_conf.enabled = True
            self._robot_rt_data.rt_target_conf_turn.enabled = True
            self._robot_rt_data.rt_wrf.enabled = True
            self._robot_rt_data.rt_trf.enabled = True
        if self._robot_info.version.is_at_least(9, 2):
            self._robot_rt_data.rt_external_tool_status.enabled = True
            self._robot_rt_data.rt_valve_state.enabled = True
            self._robot_rt_data.rt_gripper_state.enabled = True
        if self._robot_info.version.is_at_least(9, 4):
            self._robot_rt_data.rt_io_module_status.enabled = True
            self._robot_rt_data.rt_io_module_outputs.enabled = True
            self._robot_rt_data.rt_io_module_inputs.enabled = True
            self._robot_rt_data.rt_vacuum_state.enabled = True
            self._robot_rt_data.rt_sig_gen_status.enabled = True
            self._robot_rt_data.rt_sig_gen_outputs.enabled = True
            self._robot_rt_data.rt_sig_gen_inputs.enabled = True
        if self._robot_info.version.is_at_least(10, 1, 2):
            self._robot_rt_data.rt_checkpoint.enabled = True

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
            if event_id == mx_st.MX_ST_RT_EFFECTIVE_TIME_SCALING:
                self._robot_rt_data.rt_effective_time_scaling.enable = True
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
            if event_id == mx_st.MX_ST_RT_IO_STATUS:
                self._robot_rt_data.rt_io_module_status.enabled = True
                self._robot_rt_data.rt_sig_gen_status.enabled = True
            if event_id == mx_st.MX_ST_RT_OUTPUT_STATE:
                self._robot_rt_data.rt_io_module_outputs.enabled = True
                self._robot_rt_data.rt_sig_gen_outputs.enabled = True
            if event_id == mx_st.MX_ST_RT_INPUT_STATE:
                self._robot_rt_data.rt_io_module_inputs.enabled = True
                self._robot_rt_data.rt_sig_gen_inputs.enabled = True
            if event_id == mx_st.MX_ST_RT_VACUUM_STATE:
                self._robot_rt_data.rt_vacuum_state.enabled = True
            if event_id == mx_st.MX_ST_RT_VACUUM_PRESSURE:
                self._robot_rt_data.rt_vacuum_pressure.enabled = True

        # Make sure to clear values that we should no more received
        #pylint: disable=protected-access
        self._robot_rt_data._clear_if_disabled()

    def _print_fw_update_status(self, reboot_pct: float = 0):
        """ Print a trace that summarize current firmware update status (progression) """
        reboot_ratio = self._get_reboot_duration() / self._get_fw_update_duration()

        total_pct = ((1 - reboot_ratio) * self._fw_update_status.progress) + (reboot_ratio * reboot_pct)

        if self._fw_update_status.error:
            # Print as 'debug' trace (error trace will be printed by UpdateRobot function anyways)
            self.logger.debug(f'Firmware update failed: {self._fw_update_status.error_msg}')
        elif self._fw_update_status.complete:
            # Print as 'debug' trace (info trace will be printed by UpdateRobot function anyways)
            self.logger.debug(f'Firmware update complete')
        else:
            self.logger.info(f'Firmware update progress: {int(total_pct)}% ({self._fw_update_status.step})')
        #pylint: disable=protected-access
        self._fw_update_status._last_print_timestamp = time.monotonic()

    def _handle_fw_update_progress(self, event: Message):
        """Parse robot firmware update progress message

        Parameters
        ----------
        event : Message object
            The event message to parse.

        """
        assert event.id == mx_st.MX_ST_FW_UPDATE_PROGRESS
        if event.json_data:
            json_data = event.json_data[MX_JSON_KEY_DATA]

            # Parse JSON message
            new_updating = json_data[MX_JSON_KEY_FW_UPDATE_PROGRESS_UPDATING]
            new_version = json_data[MX_JSON_KEY_FW_UPDATE_PROGRESS_VERSION]
            new_error = json_data[MX_JSON_KEY_FW_UPDATE_PROGRESS_ERROR]
            new_error_msg = json_data[MX_JSON_KEY_FW_UPDATE_PROGRESS_ERROR_MSG]
            new_progress_pct = json_data[MX_JSON_KEY_FW_UPDATE_PROGRESS_PCT]
            new_step = json_data[MX_JSON_KEY_FW_UPDATE_PROGRESS_STEP]
            reboot_done = False
            if MX_JSON_KEY_FW_UPDATE_PROGRESS_REBOOT_DONE in json_data:
                reboot_done = json_data[MX_JSON_KEY_FW_UPDATE_PROGRESS_REBOOT_DONE]

            # Check if it's time to print update progress
            print_now = False
            #pylint: disable=protected-access
            last_print_timestamp = self._fw_update_status._last_print_timestamp
            if self._fw_update_reboot_timestamp != 0:
                print_now = False  # In this case progress is printed by _check_update_progress
            if new_step != self._fw_update_status.step:
                print_now = True
            elif time.monotonic() - last_print_timestamp > 5.0:
                print_now = True

            self._fw_update_status.in_progress = new_updating
            self._fw_update_status.complete = self._fw_update_started and not new_updating
            self._fw_update_status.version = new_version
            self._fw_update_status.error = new_error
            self._fw_update_status.error_msg = new_error_msg
            self._fw_update_status.progress = new_progress_pct
            self._fw_update_status.step = new_step
            if reboot_done:
                self._fw_update_reboot_done = True

            if print_now:
                reboot_pct = 100 if reboot_done else 0
                self._print_fw_update_status(reboot_pct)

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

        if self._tx_sync_pending:
            self._rx_sync = string_to_numbers(response.data)[0]
            if self._is_in_sync():
                self._tx_sync_pending = 0
                self._is_sync.set()

    def _is_in_sync(self) -> bool:
        """Tells if we're in sync with the latest "get" operation (i.e. we've received the response to the most recent
           "SyncCmdQueue" request to the robot, meaning that the "get" response we just got is up-to-date)

        Returns
        -------
        bool
            True if "in sync" ("get" response we just received matches the "get" request we've just made)
        """
        return self._rx_sync == self._tx_sync
