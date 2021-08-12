#!/usr/bin/env python3
from argparse import ArgumentError
import contextlib
import copy
import functools
import ipaddress
import json
import logging
# from os import path
import pathlib
import queue
import re
import requests
import socket
import threading
import time

import mecademicpy.mx_robot_def as mx_def
import mecademicpy.tools as tools

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

    _robot_info: RobotInfo object
        Store information concerning robot (ex.: serial number)
    _robot_kinetics : RobotKinetics object
        Stores most current robot kinetics.
        All attributes of this object are the latest captured on monitor port, so they don't necessarily share the same
        timestamp
    _robot_kinetics_stable : RobotKinetics object
        Stores most current robot kinetics, but all attributes of object share the same timestamp
    _robot_status: RobotStatus object
        Stores most current robot status
    _robot_events : RobotEvents object
        Stores events related to the robot state.

    _file_logger : RobotDataLogger object
        Collects RobotInformation, all RobotKinetics and SentCommands during determined period

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
        This simply validates that the robot is not already connection (must not be by design)
        """
        # Don't allow entering a "with" statement when robot is already connected
        # (since the goal of "with" is to disconnect)
        if self._monitor_handler_thread is not None:
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
        self._robot_kinetics = None
        self._robot_kinetics_stable = None
        self._robot_status = RobotStatus()
        self._robot_events = _RobotEvents()

        self._file_logger = None

        self._reset_disconnect_attributes()

        self._enable_synchronous_mode = None
        self._disconnect_on_exception = None

        self._offline_mode = None
        self._monitor_mode = None

        self.logger = logging.getLogger(__name__)
        self.default_timeout = 10

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
    def _connect_socket(logger, address, port, socket_timeout=0.1):
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
        logger.debug(f'Attempting to connect to {address}:{port}')

        connect_loops = round(socket_timeout / 0.1)
        for _ in range(connect_loops):
            # Create socket and attempt connection.
            new_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            new_socket.settimeout(socket_timeout)  # 100ms
            try:
                new_socket.connect((address, port))
                break
            except socket.timeout:
                logger.debug(f'Timeout connecting to {address}:{port}.')
                continue
            except Exception as exception:
                logger.error(f'Unable to connect to {address}:{port}, exception: {exception}')
                return None

        logger.debug(f'Connected to {address}:{port}.')
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
        address=mx_def.MX_DEFAULT_ROBOT_IP,
        enable_synchronous_mode=False,
        disconnect_on_exception=True,
        monitor_mode=False,
        offline_mode=False,
        timeout=0.1
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
                self._initialize_command_socket(timeout)
                self._initialize_command_connection()

            self._initialize_monitoring_socket(timeout)
            self._initialize_monitoring_connection()

            self._robot_events.clear_all()

            self._robot_events.on_deactivated.set()
            self._robot_events.on_error_reset.set()
            self._robot_events.on_p_stop_reset.set()
            self._robot_events.on_motion_resumed.set()
            self._robot_events.on_brakes_activated.set()

            self._robot_events.on_status_updated.set()
            self._robot_events.on_conf_updated.set()
            self._robot_events.on_conf_turn_updated.set()
            self._robot_events.on_joints_updated.set()
            self._robot_events.on_pose_updated.set()

            self._robot_events.on_connected.set()
            self._callback_queue.put('on_connected')

            # Start callback thread if necessary
            self._start_callback_thread()

        if self._robot_info.version.major < 8:
            self.logger.warning('Python API not supported for firmware under version 8')
            return

        # Fetching the serial number must occur outside main_lock.
        if not self._monitor_mode:
            serial_response = self.SendCustomCommand('GetRobotSerial',
                                                     expected_responses=[mx_def.MX_ST_GET_ROBOT_SERIAL])
            serial_response_message = serial_response.wait_for_data(timeout=self.default_timeout)
            self._robot_info.serial = serial_response_message.data

            full_version_request = self.SendCustomCommand('GetFwVersionFull', [mx_def.MX_ST_GET_FW_VERSION_FULL])
            full_version_request.wait_for_data()
            full_version = full_version_request.data.data
            self._robot_info.version.update_version(full_version)

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

        # Now that we're disconnected and posted 'on_disconnected' callback we can stop the callback thread
        self._stop_callback_thread()

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
            If true, robot will automatically choose the best configuration for the desired pose.

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
            If true, robot will automatically choose the best configuration for the desired pose.

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
    def MoveGripper(self, state=GRIPPER_OPEN):
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
            assert mx_def.MX_CHECKPOINT_ID_MIN <= n <= mx_def.MX_CHECKPOINT_ID_MAX
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
            assert mx_def.MX_CHECKPOINT_ID_MIN <= n <= mx_def.MX_CHECKPOINT_ID_MAX
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
    def WaitSimActivated(self, timeout=None):
        """Pause program execution until the robot simulation mode is activated.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the event (in seconds).

        Return
        ------
        boolean
            True if wait was successful, false otherwise.

        """
        return self._robot_events.on_activate_sim.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitForError(self, timeout=None):
        """Pause program execution until the robot is in error state.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the event (in seconds).

        Return
        ------
        boolean
            True if wait was successful, false otherwise.

        """
        return self._robot_events.on_error.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitErrorReset(self, timeout=None):
        """Pause program execution until the robot is not in an error state.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the event (in seconds).

        Return
        ------
        boolean
            True if wait was successful, false otherwise.

        """
        return self._robot_events.on_error_reset.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitSimDeactivated(self, timeout=None):
        """Pause program execution until the robot simulation mode is deactivated.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the event (in seconds).

        Return
        ------
        boolean
            True if wait was successful, false otherwise.

        """
        return self._robot_events.on_deactivate_sim.wait(timeout=timeout)

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
    def WaitEndOfCycle(self, timeout=None):
        """Pause program execution until all messages in a message cycle are received

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the event (in seconds).

        Return
        ------
        boolean
            True if wait was successful, false otherwise.

        """
        if self._robot_events.on_end_of_cycle.is_set():
            self._robot_events.on_end_of_cycle.clear()

        return self._robot_events.on_end_of_cycle.wait(timeout=timeout)

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
    def SendCustomCommand(self, command, expected_responses=None):
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
        event.wait_for_data() to wait for and get the response message.

        """
        with self._main_lock:
            self._check_internal_states()

            if expected_responses:
                event_with_data = InterruptableEvent(data=expected_responses)
                self._custom_response_events.append(event_with_data)

            self._send_command(command)

        if expected_responses:
            return event_with_data

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

    # Non-motion commands.

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
                        self._send_command('GetRtTargetJointPos')
                    else:
                        self._send_command('GetJoints')

            if not self._robot_events.on_joints_updated.wait(timeout=timeout):
                raise TimeoutError

        with self._main_lock:
            if include_timestamp:
                if not self._robot_info.rt_message_capable:
                    raise InvalidStateError('Cannot provide timestamp with current robot firmware or model.')
                else:
                    return copy.deepcopy(self._robot_kinetics.rt_target_joint_pos)

            return copy.deepcopy(self._robot_kinetics.rt_target_joint_pos.data)

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
                        self._send_command('GetRtTargetCartPos')
                    else:
                        self._send_command('GetPose')

            if not self._robot_events.on_pose_updated.wait(timeout=timeout):
                raise TimeoutError

        with self._main_lock:
            if include_timestamp:
                if not self._robot_info.rt_message_capable:
                    raise InvalidStateError('Cannot provide timestamp with current robot firmware or model.')
                else:
                    return copy.deepcopy(self._robot_kinetics.rt_target_cart_pos)

            return copy.deepcopy(self._robot_kinetics.rt_target_cart_pos.data)

    @disconnect_on_exception
    def GetConf(self, include_timestamp=False, synchronous_update=False, timeout=None):
        """Get robot's current (physical) inverse-kinematics configuration.

        Returns
        -------
        list of integers (timestmap optional)
            Configuration status of robot.

        """
        if synchronous_update:
            with self._main_lock:
                self._check_internal_states()
                if self._robot_events.on_conf_updated.is_set():
                    self._robot_events.on_conf_updated.clear()
                    if self._robot_info.rt_message_capable:
                        self._send_command('GetRtTargetConf')
                    else:
                        self._send_command('GetConf')

            if not self._robot_events.on_conf_updated.wait(timeout=timeout):
                raise TimeoutError

        with self._main_lock:
            if include_timestamp:
                if not self._robot_info.rt_message_capable:
                    raise InvalidStateError('Cannot provide timestamp with current robot firmware or model.')
                else:
                    return copy.deepcopy(self._robot_kinetics.rt_target_conf)

            return copy.deepcopy(self._robot_kinetics.rt_target_conf.data)

    @disconnect_on_exception
    def GetConfTurn(self, include_timestamp=False, synchronous_update=False, timeout=None):
        """Get robot's current (physical) last-joint turn number.

        Returns
        -------
        int (timestamp optional)
            Turn number of last joint.

        """
        if synchronous_update:
            with self._main_lock:
                self._check_internal_states()
                if self._robot_events.on_conf_turn_updated.is_set():
                    self._robot_events.on_conf_turn_updated.clear()
                    if self._robot_info.rt_message_capable:
                        self._send_command('GetRtTargetConfTurn')
                    else:
                        self._send_command('GetConfTurn')

            if not self._robot_events.on_conf_turn_updated.wait(timeout=timeout):
                raise TimeoutError

        with self._main_lock:
            if include_timestamp:
                if not self._robot_info.rt_message_capable:
                    raise InvalidStateError('Cannot provide timestamp with current robot firmware or model.')
                else:
                    return copy.deepcopy(self._robot_kinetics.rt_target_conf_turn)

            return copy.deepcopy(self._robot_kinetics.rt_target_conf_turn.data[0])

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
    def SetRealTimeMonitoring(self, *events):
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

    def GetRobotKinetics(self):
        """Return a copy of the current robot kinetics, with all values associated with the same timestamp

        Return
        ------
        RobotKinetics
            Object containing the current robot kinetics

        """
        with self._main_lock:
            return copy.deepcopy(self._robot_kinetics_stable)

    @disconnect_on_exception
    def GetRobotStatus(self, synchronous_update=False, timeout=None):
        """Return a copy of the current robot status

        Returns
        -------
        RobotStatus
            Object containing the current robot status

        """
        if synchronous_update:
            with self._main_lock:
                self._check_internal_states()
                if self._robot_events.on_status_updated.is_set():
                    self._robot_events.on_status_updated.clear()
                    self._send_command('GetStatusRobot')

            if not self._robot_events.on_status_updated.wait(timeout=timeout):
                raise TimeoutError

        with self._main_lock:
            return copy.deepcopy(self._robot_status)

    def StartLogging(self, monitoringInterval, file_name=None, file_path=None, fields=None, record_time=True):
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
            List of fields to log. Taken from RobotKinetics attributes. None means log all compatible fields.

        record_time : bool
            If true, current date and time will be recorded in file.



        """
        if self._file_logger is not None:
            raise InvalidStateError('Another file logging operation is in progress.')

        self.SetMonitoringInterval(monitoringInterval)
        if fields is None:
            self.SetRealTimeMonitoring('all')
        else:
            self.SetRealTimeMonitoring(*fields)

        self._file_logger = _RobotTrajectoryLogger(self._robot_info,
                                                   self._robot_kinetics,
                                                   fields,
                                                   file_name=file_name,
                                                   file_path=file_path,
                                                   record_time=record_time,
                                                   monitoring_interval=monitoringInterval)

    def EndLogging(self):
        """Stop logging robot kinetics to file.

        """
        if self._file_logger is None:
            raise InvalidStateError('No existing logger to stop.')

        file_name = self._file_logger.end_log()
        self._file_logger = None

        return file_name

    @contextlib.contextmanager
    def FileLogger(self, monitoringInterval, file_name=None, file_path=None, fields=None, record_time=True):
        """Contextmanager interface for file logger.

        Parameters
        ----------
        monitoring_interval: float
            Indicates rate at which kinetics from robot will be received on monitor port. Unit: seconds

        file_name: string or None
            Log file name
            If None, file name will be built with date/time and robot information (robot type, serial, version).

        file_path : string or None
            Path to save the zip file that contains logged data.
            If not provided, file will be saved in working directory.

        fields : list of strings or None
            List of fields to log. Taken from RobotKinetics attributes. None means log all compatible fields.

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

        try:
            self._command_socket = self._connect_socket(self.logger, self._address, mx_def.MX_ROBOT_TCP_PORT_CONTROL,
                                                        timeout)

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

    def _initialize_monitoring_socket(self, timeout):
        """Establish the monitoring socket and the associated thread.

        """
        if self._offline_mode:
            return

        if self._monitor_socket is not None:
            raise InvalidStateError('Cannot connect since existing monitor socket exists.')

        try:
            self._monitor_socket = self._connect_socket(self.logger, self._address, mx_def.MX_ROBOT_TCP_PORT_FEED,
                                                        timeout)

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
        """Receive and parse a welcome message in order to set _robot_info and _robot_kinetics.

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

        self._robot_kinetics = RobotKinetics(self._robot_info.num_joints)
        self._robot_kinetics_stable = RobotKinetics(self._robot_info.num_joints)

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

    def _invalidate_checkpoints(self):
        '''Unblock all waiting checkpoints and have them throw InterruptException.

        '''

        for checkpoints_dict in [self._internal_checkpoints, self._user_checkpoints]:
            for key, checkpoints_list in checkpoints_dict.items():
                for event in checkpoints_list:
                    event.abort()
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
        # Variables to hold joint positions and poses while waiting for timestamp.
        rt_joint_pos = None
        rt_cart_pos = None
        rt_conf = None
        rt_conf_turn = None

        while True:
            # Wait for a message in the queue.
            response = self._monitor_rx_queue.get(block=True)

            # Terminate thread if requested.
            if response == _TERMINATE:
                return

            self._callback_queue.put('on_monitor_message', response)

            queue_size = self._monitor_rx_queue.qsize()
            if queue_size > self._robot_kinetics.max_queue_size:
                self._robot_kinetics.max_queue_size = queue_size

            with self._main_lock:

                # Temporarily save data if rt messages will be available to add timestamps.
                # Note that if robot platform isn't RT message capable, the update occurs in _handle_common_messages.
                if response.id == mx_def.MX_ST_GET_JOINTS and self._robot_info.rt_message_capable:
                    rt_joint_pos = _string_to_numbers(response.data)
                elif response.id == mx_def.MX_ST_GET_POSE and self._robot_info.rt_message_capable:
                    rt_cart_pos = _string_to_numbers(response.data)
                elif response.id == mx_def.MX_ST_GET_CONF and self._robot_info.rt_message_capable:
                    rt_conf = _string_to_numbers(response.data)
                elif response.id == mx_def.MX_ST_GET_CONF_TURN and self._robot_info.rt_message_capable:
                    rt_conf_turn = _string_to_numbers(response.data)

                if response.id == mx_def.MX_ST_RT_CYCLE_END:
                    if not self._robot_info.rt_message_capable:
                        self._robot_info.rt_message_capable = True
                    timestamp = int(response.data)

                    # Useful to detect end of cycle for logging, to start logging on more consistent moment
                    self._robot_events.on_end_of_cycle.set()
                    self._callback_queue.put('on_end_of_cycle')

                    # Update the legacy joint and pose messages with timestamps.
                    if rt_joint_pos:
                        self._robot_kinetics.rt_target_joint_pos.update_from_data(timestamp, rt_joint_pos)
                        rt_joint_pos = None
                    if rt_cart_pos:
                        self._robot_kinetics.rt_target_cart_pos.update_from_data(timestamp, rt_cart_pos)
                        rt_cart_pos = None
                    if rt_conf:
                        self._robot_kinetics.rt_target_conf.update_from_data(timestamp, rt_conf)
                        rt_conf = None
                    if rt_conf_turn:
                        self._robot_kinetics.rt_target_conf_turn.update_from_data(timestamp, rt_conf_turn)
                        rt_conf_turn = None

                    # If logging is active, log the current state.
                    if self._file_logger:
                        self._file_logger.write_fields(timestamp, self._robot_kinetics)
                    self._make_stable_kinetics()
                else:
                    self._handle_common_messages(response, is_command_response=False)

                    # On non-rt monitoring capable platforms, no CYCLE_END event is sent, so use system time.
                    # GET_JOINTS and GET_POSE is still sent every cycle, so log RobotKinetics when GET_POSE is received.
                    if response.id == mx_def.MX_ST_GET_POSE and not self._robot_info.rt_message_capable:
                        # On non rt_monitoring platforms, we will consider this moment to be the end of cycle
                        self._robot_events.on_end_of_cycle.set()
                        self._callback_queue.put('on_end_of_cycle')

                        if self._file_logger:
                            # Log time in microseconds to be consistent with real-time logging timestamp.
                            self._file_logger.write_fields(time.time_ns() / 1000, self._robot_kinetics)
                        self._make_stable_kinetics()

    def _make_stable_kinetics(self):
        """We have to create stable copy of kinetics, with consitent timestampp values for all attributes.
        This consistent copy is used by GetRobotKinetics()
        """

        self._robot_kinetics_stable = copy.deepcopy(self._robot_kinetics)

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
        if response.id == mx_def.MX_ST_GET_STATUS_ROBOT:
            self._handle_robot_status_response(response, is_command_response)

        # Only update using legacy messages if robot is not capable of rt messages.
        elif response.id == mx_def.MX_ST_GET_JOINTS and not self._robot_info.rt_message_capable:
            self._robot_kinetics.rt_target_joint_pos = TimestampedData(0, _string_to_numbers(response.data))
            if is_command_response:
                self._robot_events.on_joints_updated.set()

        elif response.id == mx_def.MX_ST_GET_POSE and not self._robot_info.rt_message_capable:
            self._robot_kinetics.rt_target_cart_pos = TimestampedData(0, _string_to_numbers(response.data))
            if is_command_response:
                self._robot_events.on_pose_updated.set()

        elif response.id == mx_def.MX_ST_GET_CONF and not self._robot_info.rt_message_capable:
            self._robot_kinetics.rt_target_conf = TimestampedData(0, _string_to_numbers(response.data))
            if is_command_response:
                self._robot_events.on_conf_updated.set()

        elif response.id == mx_def.MX_ST_GET_CONF_TURN and not self._robot_info.rt_message_capable:
            self._robot_kinetics.rt_target_conf_turn = TimestampedData(0, _string_to_numbers(response.data))
            if is_command_response:
                self._robot_events.on_conf_turn_updated.set()

        elif response.id == mx_def.MX_ST_RT_TARGET_JOINT_POS:
            self._robot_kinetics.rt_target_joint_pos.update_from_csv(response.data)
            if is_command_response:
                self._robot_events.on_joints_updated.set()

        elif response.id == mx_def.MX_ST_RT_TARGET_CART_POS:
            self._robot_kinetics.rt_target_cart_pos.update_from_csv(response.data)
            if is_command_response:
                self._robot_events.on_pose_updated.set()

        elif response.id == mx_def.MX_ST_RT_TARGET_JOINT_POS:
            self._robot_kinetics.rt_target_joint_pos.update_from_csv(response.data)
        elif response.id == mx_def.MX_ST_RT_TARGET_CART_POS:
            self._robot_kinetics.rt_target_cart_pos.update_from_csv(response.data)
        elif response.id == mx_def.MX_ST_RT_TARGET_JOINT_VEL:
            self._robot_kinetics.rt_target_joint_vel.update_from_csv(response.data)
        elif response.id == mx_def.MX_ST_RT_TARGET_JOINT_TORQ:
            self._robot_kinetics.rt_target_joint_torq.update_from_csv(response.data)
        elif response.id == mx_def.MX_ST_RT_TARGET_CART_VEL:
            self._robot_kinetics.rt_target_cart_vel.update_from_csv(response.data)

        elif response.id == mx_def.MX_ST_RT_TARGET_CONF:
            self._robot_kinetics.rt_target_conf.update_from_csv(response.data)
        elif response.id == mx_def.MX_ST_RT_TARGET_CONF_TURN:
            self._robot_kinetics.rt_target_conf_turn.update_from_csv(response.data)

        elif response.id == mx_def.MX_ST_RT_JOINT_POS:
            self._robot_kinetics.rt_joint_pos.update_from_csv(response.data)
        elif response.id == mx_def.MX_ST_RT_CART_POS:
            self._robot_kinetics.rt_cart_pos.update_from_csv(response.data)
        elif response.id == mx_def.MX_ST_RT_JOINT_VEL:
            self._robot_kinetics.rt_joint_vel.update_from_csv(response.data)
        elif response.id == mx_def.MX_ST_RT_JOINT_TORQ:
            self._robot_kinetics.rt_joint_torq.update_from_csv(response.data)
        elif response.id == mx_def.MX_ST_RT_CART_VEL:
            self._robot_kinetics.rt_cart_vel.update_from_csv(response.data)

        elif response.id == mx_def.MX_ST_RT_CONF:
            self._robot_kinetics.rt_conf.update_from_csv(response.data)
        elif response.id == mx_def.MX_ST_RT_CONF_TURN:
            self._robot_kinetics.rt_conf_turn.update_from_csv(response.data)

        elif response.id == mx_def.MX_ST_RT_ACCELEROMETER:
            # The data is stored as [timestamp, index, {measurements...}]
            timestamp, index, *measurements = _string_to_numbers(response.data)
            # Record accelerometer measurement only if newer.
            if (index not in self._robot_kinetics.rt_accelerometer
                    or timestamp > self._robot_kinetics.rt_accelerometer[index].timestamp):
                self._robot_kinetics.rt_accelerometer[index] = TimestampedData(timestamp, measurements)

    def _handle_robot_status_response(self, response, is_command_response):
        """Parse robot status response and update status fields and events.

        Parameters
        ----------
        response : Message object
            Robot status response to parse and handle.

        """
        assert response.id == mx_def.MX_ST_GET_STATUS_ROBOT
        status_flags = [bool(int(x)) for x in response.data.split(',')]

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

        # We only want to detect status response on command port, or else it is not possible to have synchronous
        # response on GetRobotStatus
        if is_command_response:
            self._robot_events.on_status_updated.set()
            self._callback_queue.put('on_status_updated')

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

    def update_robot(self, firmware):
        """
        Install a new firmware and verifies robot version afterward.

        :param firmware: RobotFirmwareFile object.
        """
        firmware_file = None
        if type(firmware) == pathlib.WindowsPath or type(firmware) == pathlib.PosixPath:
            firmware_file = firmware
        elif type(firmware) == str:
            firmware_file = pathlib.Path(firmware)
        else:
            raise ArgumentError(f'Unsupported firmware type. received: {type(firmware)}, expecting pathlib or str')

        firmware_file_version = RobotVersion(firmware_file.name)

        if self.GetRobotStatus().activation_state:
            self.logger.info(f'Robot is activated, will attempt to deactivate before updating firmware')
            if self._monitor_mode:
                self.logger.info(f'Connected to robot in monitoring mode only, attempting connection in command mode'
                                 'to deactivate robot')
                self.Connect(address=self._address)
            self.DeactivateRobot()
        self.Disconnect()

        robot_url = f"http://{self._address}/"

        self.logger.info(f"Installing firmware: {firmware_file.absolute()}")

        with open(firmware_file.absolute(), 'rb') as firmware_file:
            firmware_data = firmware_file.read()
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
        except requests.exceptions as e:
            self.logger.error(f"Upgrade post request error: {e}")
            raise

        if not request_post.ok:
            error_message = f"Firmware upload request failed"
            raise RuntimeError(error_message)

        self.logger.info(f"Upgrading the robot")
        update_done = False
        progress = ''
        last_progress = ''
        while not update_done:
            # Give time to the web server restart, the function doesn't handle well errors.
            time.sleep(2)

            request_get = requests.get(robot_url, 'update', timeout=10)
            try:
                request_get.raise_for_status()
            except requests.exceptions as e:
                self.logger.error(f'Upgrade get request error: {e}')
                raise

            # get only correct answer (http code 200)
            if request_get.status_code == 200:
                request_response = request_get.text
            else:
                request_response = None
            # while the json file is note created, get function will return 0
            if request_response is None or request_response == '0':
                continue

            try:
                request_answer = json.loads(request_response)
            except Exception as e:
                self.logger.info(f'Error retrieving json from request_response: {e}')
                continue

            if not request_answer:
                self.logger.info(f'Answer is empty')
                continue

            if request_answer['STATUS']:
                status_code = int(request_answer['STATUS']['Code'])
                status_msg = request_answer['STATUS']['MSG']

            if status_code in [0, 1]:
                keys = sorted(request_answer['LOG'].keys())
                if keys:
                    last_progress = progress
                    progress = request_answer['LOG'][keys[-1]]
                    new_progress = progress.replace(last_progress, '')
                    if '#' in new_progress:
                        self.logger.info(new_progress)
                    elif '100%' in new_progress:
                        self.logger.info(new_progress)
                    else:
                        self.logger.debug(new_progress)
                if status_code == 0:
                    update_done = True
                    self.logger.info(f'status_msg {status_msg}')
            else:
                error_message = f"error while updating: {status_msg}"
                self.logger.error(error_message)
                raise RuntimeError(error_message)

        self.logger.info(f"Update completed, waiting for robot to reboot")

        # need to wait to make sure the robot shutdown before attempting to ping it.
        time.sleep(15)
        tools.ping_robot(self._address)
        self.Connect(self._address, timeout=60)

        current_version = self.GetRobotInfo().version
        if current_version.major < 8.0:
            expected_version = firmware_file_version.short_version
        else:
            expected_version = firmware_file_version.full_version

        if firmware_file_version.full_version == expected_version:
            self.logger.info(f"robot is now running version {current_version}")
        else:
            error_msg = f"Fail to install robot properly. current version {current_version}, " \
                        f"expecting: {expected_version}"
            self.logger.error(error_msg)
            raise AssertionError(error_msg)

        robot_status = self.GetRobotStatus()
        if robot_status.error_status:
            error_msg = f"Robot is in error on version {current_version}"
            self.logger.error(error_msg)
            raise AssertionError(error_msg)

        self.logger.info(f"Installation of {current_version} sucessfully completed")


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
        on_end_of_cycle : function object
            Function to be called each time end of cycle is reached
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

        self.on_end_of_cycle = None


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


class RobotKinetics:
    """Class for storing the internal kinetics of a generic Mecademic robot.

    Note that the frequency and availability of kinetics depends on the monitoring interval and which monitoring events
    are enabled. Monitoring events can be configured using SetMonitoringInterval() and SetRealTimeMonitoring().

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

        self.max_queue_size = 0


class RobotStatus:
    """Class for storing the internal status of a generic Mecademic robot.

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
    num_joints : int
        Number of joints on the robot.

    """

    def __init__(self,
                 model=None,
                 revision=None,
                 is_virtual=None,
                 version=None,
                 serial=None):
        self.model = model
        self.revision = revision
        self.is_virtual = is_virtual
        self.version = RobotVersion(version)
        self.serial = serial
        self.rt_message_capable = False

        if self.model == 'Meca500':
            self.num_joints = 6
        elif self.model == 'scara':
            self.num_joints = 4
        elif self.model is None:
            self.num_joints = 1
        else:
            raise ValueError(f'Invalid robot model: {self.model}')

    @classmethod
    def from_command_response_string(cls, input_string):
        """Generate robot information from standard robot response string.

        String format should be "Connected to {model} R{revision}{-virtual} v{fw_major_num}.{fw_minor_num}.{patch_num}"

        Parameters
        ----------
        input_string : string
            Input string to be parsed.

        """
        ROBOT_CONNECTION_STRING = \
            r"Connected to (?P<model>\w+) ?R?(?P<revision>\d)?(?P<virtual>-virtual)?( v|_)?(?P<version>\d+\.\d+\.\d+)"

        virtual = False
        try:
            robot_info_regex = re.search(ROBOT_CONNECTION_STRING, input_string)
            if robot_info_regex.group('model'):
                model = robot_info_regex.group('model')
            if robot_info_regex.group('revision'):
                revision = int(robot_info_regex.group('revision'))
            if robot_info_regex.group('virtual'):
                virtual = True
            return cls(model=model, revision=revision, is_virtual=virtual, version=robot_info_regex.group('version'))
        except Exception as exception:
            raise ValueError(f'Could not parse robot info string {input_string}, error: {exception}')


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

    def __init__(self, id=None, data=None):
        self._id = id
        self._data = data
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

    def wait_for_data(self, timeout=None):
        """Block until event is set or should raise an exception.

        Attributes
        ----------
        timeout : float
            Maximum duration to wait in seconds.

        Return
        ------
        data : object
            Return the data object.

        """
        success = self._event.wait(timeout=timeout)
        if self._interrupted:
            raise InterruptException('Event received exception, possibly because event will never be triggered.')
        elif not success:
            raise InterruptException('Event timed out.')
        else:
            return self._data

    def set(self, data=None):
        """Set the event and unblock all waits. Optionally modify data before setting.

        """
        with self._lock:
            self._data = data
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
    on_conf_updated : event
        Set if robot configuration has been updated.
    on_conf_turn_updated : event
        Set if last joint turn number has been updated.
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
        self.on_conf_turn_updated = InterruptableEvent()
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
        self.on_conf_updated.set()
        self.on_conf_turn_updated.set()
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


class RobotVersion:
    """
        Robot utility class to handle firmware version.
    """

    REGEX_VERSION_BUILD = r"(?P<version>\d+\.\d+\.\d+)\.?(?P<build>\d+)?-?(?P<extra>[0-9a-zA-Z_-]*).*"

    def __init__(self, version):
        """Creates

        :param version: version of firmware. Supports multiple version formats
        """
        self.full_version = version
        self.update_version(self.full_version)

    def __str__(self):
        return self.full_version

    def update_version(self, version):

        regex_version = re.search(self.REGEX_VERSION_BUILD, version)
        self.short_version = regex_version.group("version")
        splitted_version = self.short_version.split(".")
        self.major = int(splitted_version[0])
        self.minor = int(splitted_version[1])
        self.patch = int(splitted_version[2])
        self.build = None
        self.extra = None
        if regex_version.group("build"):
            self.build = int(regex_version.group("build"))
        if regex_version.group("extra"):
            self.extra = regex_version.group("extra")

        self.full_version = self.short_version
        if self.build:
            self.full_version += f".{self.build}"

        if self.extra:
            self.full_version += f"-{self.extra}"
