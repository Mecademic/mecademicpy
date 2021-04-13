#!/usr/bin/env python3
import logging
import ipaddress
import time
import socket
import multiprocessing as mp
import threading
import functools

from .mx_robot_def import *

CHECKPOINT_ID_MAX_PRIVATE = 8191  # Max allowable checkpoint id, inclusive

TERMINATE_PROCESS = 'terminate_process'


class MecademicException(Exception):
    pass


class InvalidStateError(MecademicException):
    pass


class CommunicationError(MecademicException):
    pass


class DisconnectError(MecademicException):
    pass


class Checkpoint:
    """Class representing a checkpoint object, which can be used to wait.

    Attributes
    ----------
    id : integer
        The id of the checkpoint. Not required to be unique.
    event : event object
        A standard event-type object used to signal when the checkpoint is reached.

    """
    def __init__(self, id, event):
        self.id = id
        self.event = event

    def __repr__(self):
        return "Checkpoint with id={}, is reached={}".format(self.id, self.event.is_set())

    def wait(self, timeout=None):
        """Pause program execution this checkpoint is received from the robot.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the checkpoint (in seconds).

        Return
        ------
        boolean
            True if wait was successful, false otherwise.

        """
        return self.event.wait(timeout=timeout)


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


class CallbackTag:
    """Class for storing a callback name and associated data.

    Attributes
    ----------
    callback_name : string
        The name of the callback.
    data : string
        The associated data.

    """
    def __init__(self, callback_name, data=None):
        self.callback_name = callback_name
        self.data = data

    def __repr__(self):
        return "Callback name: {}, data: {}".format(self.callback_name, self.data)


class RobotState:
    """Class for storing the internal state of a generic Mecademic robot.

    Attributes
    ----------
    joint_positions : shared memory vector
        The positions of the robot joints in degrees.
    end_effector_pose : shared memory vector
        The end effector pose in [x, y, z, alpha, beta, gamma] (mm and degrees).

    nc_joint_positions : shared memory vector
        Controller desired joint positions in degrees [timestamp, theta_1...6].
    nc_end_effector_pose : shared memory vector
        Controller desired end effector pose [timestamp, x, y, z, alpha, beta, gamma].

    nc_joint_velocity : shared memory vector
        Controller desired joint velocity in degrees/second [timestamp, theta_dot_1...6].
    nc_end_effector_velocity : shared memory vector
        Controller desired end effector velocity in mm/s and degrees/s, includes timestamp.
    nc_joint_configurations : shared memory vector
        Controller desired joint configurations.
    nc_multiturn : shared memory vector
        Controller desired joint 6 multiturn configuration.

    drive_joint_positions : shared memory vector
        Drive-measured joint positions in degrees [timestamp, theta_1...6].
    drive_end_effector_pose : shared memory vector
        Drive-measured end effector pose [timestamp, x, y, z, alpha, beta, gamma].

    drive_joint_velocity : shared memory vector
        Drive-measured joint velocity in degrees/second [timestamp, theta_dot_1...6].
    drive_joint_torque_ratio : shared memory vector
        Drive-measured torque ratio as a percent of maximum [timestamp, torque_1...6]
    drive_end_effector_velocity : shared memory vector
        Drive-measured end effector velocity in mm/s and degrees/s, includes timestamp.

    drive_joint_configurations : shared memory vector
        Drive-measured joint configurations.
    drive_multiturn : shared memory vector
        Drive-measured joint 6 multiturn configuration.

    accelerometer : shared memory vector
        Raw accelerometer measurements [timestamp, accelerometer_id, x, y, z]. 16000 = 1g.

    activation_state : shared memory boolean
        True if the robot is activated.
    homing_state : shared memory boolean
        True if the robot is homed.
    simulation_mode : shared memory boolean
        True if the robot is in simulation-only mode.
    error_status : shared memory boolean
        True if the robot is in error.
    pause_motion_status : shared memory boolean
        True if motion is currently paused.
    end_of_block_status : shared memory boolean
        True if robot is not moving and motion queue is empty.
    end_of_movement_status : shared memory boolean
        True if robot is idle.


    """
    def __init__(self):
        self.joint_positions = mp.Array('f', 6, lock=False)  # degrees
        self.end_effector_pose = mp.Array('f', 6, lock=False)  # mm and degrees

        self.nc_joint_positions = mp.Array('f', 7, lock=False)  # degrees
        self.nc_end_effector_pose = mp.Array('f', 7, lock=False)  # mm and degrees

        self.nc_joint_velocity = mp.Array('f', 7, lock=False)  # degrees/second
        self.nc_end_effector_velocity = mp.Array('f', 7, lock=False)  # mm/s and degrees/s

        self.nc_joint_configurations = mp.Array('f', 4, lock=False)
        self.nc_multiturn = mp.Array('f', 2, lock=False)

        self.drive_joint_positions = mp.Array('f', 7, lock=False)  # degrees
        self.drive_end_effector_pose = mp.Array('f', 7, lock=False)  # mm and degrees

        self.drive_joint_velocity = mp.Array('f', 7, lock=False)  # degrees/second
        self.drive_joint_torque_ratio = mp.Array('f', 7, lock=False)  # percent of maximum
        self.drive_end_effector_velocity = mp.Array('f', 7, lock=False)  # mm/s and degrees/s

        self.drive_joint_configurations = mp.Array('f', 4, lock=False)
        self.drive_multiturn = mp.Array('f', 2, lock=False)

        self.accelerometer = mp.Array('f', 5, lock=False)  # 16000 = 1g

        # The following status fields are updated together, and is protected by a single lock.
        self.activation_state = mp.Value('b', False, lock=False)
        self.homing_state = mp.Value('b', False, lock=False)
        self.simulation_mode = mp.Value('b', False, lock=False)
        self.error_status = mp.Value('b', False, lock=False)
        self.pause_motion_status = mp.Value('b', False, lock=False)
        self.end_of_block_status = mp.Value('b', False, lock=False)
        self.end_of_movement_status = mp.Value('b', False, lock=False)


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

    """
    def __init__(self):
        self.on_connected = mp.Event()
        self.on_disconnected = mp.Event()
        self.on_status_updated = mp.Event()
        self.on_activated = mp.Event()
        self.on_deactivated = mp.Event()
        self.on_homed = mp.Event()
        self.on_error = mp.Event()
        self.on_error_reset = mp.Event()
        self.on_p_stop = mp.Event()
        self.on_p_stop_reset = mp.Event()
        self.on_motion_paused = mp.Event()
        self.on_motion_resumed = mp.Event()

        self.on_disconnected.set()
        self.on_deactivated.set()
        self.on_error_reset.set()
        self.on_p_stop_reset.set()
        self.on_motion_resumed.set()


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

    _command_rx_process : process handle
        Process used to receive messages from the command port.
    _command_rx_queue : multiprocessing queue
        Queue used to temporarily store messages from the command port.
    _command_tx_process : process handle
        Process used to transmit messages to the command port.
    _command_tx_queue : multiprocessing queue
        Queue used to temporarily store commands to be sent to the command port.
    _monitor_rx_process : process handle
        Process used to receive messages from the monitor port.
    _monitor_rx_queue : multiprocessing queue
        Queue used to temporarily store messages from the monitor port.

    _command_response_handler_process : process handle
        Process used to read messages from the command response queue.
    _monitor_handler_process : process handle
        Process used to read messages from the monitor queue.

    _main_lock : recursive lock object
        Used to protect internal state of the robot object.

    _robot_state : RobotState object
        Stores most current robot state.
    _robot_events : RobotEvents object
        Stores events related to the robot state.

    _robot_callbacks : RobotCallbacks instance
        Stores user-defined callback functions.
    _callback_queue : multiprocessing queue
        Queue storing triggered callbacks.
    _callback_thread : thread handle
        Callbacks will run in this thread if so configured.

    _manager : multiprocessing manager
        Manages checkpoint-related objects shared between processes.
    _user_checkpoints : manager dictionary
        Stores checkpoints set or expected by user.
    _internal_checkpoints : manager dictionary
        Stores checkpoints set internally by the Robot class.
    _internal_checkpoint_counter : int
        Stores the next available checkpoint id for internal checkpoints.

    _enable_synchronous_mode : boolean
        If enabled, commands block until action is completed.

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
            If true, will not check child processes before executing api calls.

        """

        # Check that the ip address is a string.
        if not isinstance(address, str):
            raise TypeError('Please provide a string argument for the address.')

        # Check that the ip address is valid.
        ipaddress.ip_address(address)

        self._address = address

        self._command_socket = None
        self._monitor_socket = None

        self._command_rx_process = None
        self._command_tx_process = None
        self._monitor_rx_process = None

        self._command_response_handler_process = None
        self._monitor_handler_process = None

        self._main_lock = mp.RLock()

        self._robot_callbacks = RobotCallbacks()
        self._callback_queue = mp.Queue()
        self._callback_thread = None

        self._manager = mp.Manager()

        self._init_states()

        self._enable_synchronous_mode = enable_synchronous_mode
        self._disconnect_on_exception = disconnect_on_exception
        self._offline_mode = offline_mode
        self.logger = logging.getLogger(__name__)

    def __del__(self):
        self.Disconnect()
        self.UnregisterCallbacks()

    def _init_states(self):
        self._command_rx_queue = mp.Queue()
        self._command_tx_queue = mp.Queue()
        self._monitor_rx_queue = mp.Queue()

        self._user_checkpoints = self._manager.dict()
        self._internal_checkpoints = self._manager.dict()
        self._internal_checkpoint_counter = MX_CHECKPOINT_ID_MAX + 1

        self._robot_state = RobotState()
        self._robot_events = RobotEvents()

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

            # Terminate process if requested, otherwise send the command.
            if command == TERMINATE_PROCESS:
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
            callback_queue.put(CallbackTag('on_checkpoint_reached', checkpoint_id))

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
                                  logger, callback_queue):
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
        callback_queue : queue
            Used to push triggered callbacks onto.

        """
        while True:
            # Wait for a response to be available from the queue.
            response = rx_queue.get(block=True)

            # Terminate process if requested.
            if response == TERMINATE_PROCESS:
                return

            with main_lock:
                if response.id == MX_ST_CLEAR_MOTION:
                    try:
                        internal_checkpoints['clear_motion'].pop().set()
                        # If no more pending clear_motion commands, set clear_motion_all.
                        if not internal_checkpoints['clear_motion']:
                            [item.set() for item in internal_checkpoints['clear_motion_all']]
                    except (KeyError, IndexError):
                        pass
                    callback_queue.put(CallbackTag('on_motion_cleared'))

                elif response.id == MX_ST_GET_STATUS_ROBOT:
                    Robot._handle_robot_status_response(response, robot_state, events, callback_queue)
                    events.on_status_activated.set()
                    callback_queue.put(CallbackTag('on_status_updated'))

                elif response.id == MX_ST_CHECKPOINT_REACHED:
                    Robot._handle_checkpoint_response(response, user_checkpoints, internal_checkpoints, logger,
                                                      callback_queue)

                elif response.id == MX_ST_PSTOP:
                    if bool(int(response.data)):
                        events.on_p_stop_reset.clear()
                        events.on_p_stop.set()
                        callback_queue.put(CallbackTag('on_p_stop'))
                    else:
                        events.on_p_stop.clear()
                        events.on_p_stop_reset.set()
                        callback_queue.put(CallbackTag('on_p_stop_reset'))

    @staticmethod
    def _string_to_floats(input_string):
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

    @staticmethod
    def _handle_robot_status_response(response, robot_state, events, callback_queue):
        """Parse robot status response and update status fields and events.

        Parameters
        ----------
        response : Message object
            Robot status response to parse and handle.

        robot_state : RobotState object
            Stores the robot state across processes.

        events : RobotEvents object
            Stores events associated with changes in robot state.

        callback_queue : queue
            Used to push triggered callbacks onto.

        """
        assert response.id == MX_ST_GET_STATUS_ROBOT
        status_flags = [bool(int(x)) for x in response.data.split(',')]

        if robot_state.activation_state.value != status_flags[0]:
            if status_flags[0]:
                events.on_deactivated.clear()
                events.on_activated.set()
                callback_queue.put(CallbackTag('on_activated'))
            else:
                events.on_activated.clear()
                events.on_deactivated.set()
                callback_queue.put(CallbackTag('on_deactivated'))
            robot_state.activation_state.value = status_flags[0]

        if robot_state.homing_state.value != status_flags[1]:
            if status_flags[1]:
                events.on_homed.set()
                callback_queue.put(CallbackTag('on_homed'))
            else:
                events.on_homed.clear()
            robot_state.homing_state.value = status_flags[1]

        robot_state.simulation_mode.value = status_flags[2]

        if robot_state.error_status.value != status_flags[3]:
            if status_flags[3]:
                events.on_error_reset.clear()
                events.on_error.set()
                callback_queue.put(CallbackTag('on_error'))
            else:
                events.on_error.clear()
                events.on_error_reset.set()
                callback_queue.put(CallbackTag('on_error_reset'))
            robot_state.error_status.value = status_flags[3]

        if robot_state.pause_motion_status.value != status_flags[4]:
            if status_flags[4]:
                events.on_motion_resumed.clear()
                events.on_motion_paused.set()
                callback_queue.put(CallbackTag('on_motion_paused'))
            else:
                events.on_motion_paused.clear()
                events.on_motion_resumed.set()
                callback_queue.put(CallbackTag('on_motion_resumed'))
            robot_state.pause_motion_status.value = status_flags[4]

        robot_state.end_of_block_status.value = status_flags[5]
        robot_state.end_of_movement_status.value = status_flags[6]

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

            # Terminate process if requested.
            if response == TERMINATE_PROCESS:
                return

            with main_lock:
                if response.id == MX_ST_GET_JOINTS:
                    robot_state.joint_positions[:] = Robot._string_to_floats(response.data)

                elif response.id == MX_ST_GET_POSE:
                    robot_state.end_effector_pose[:] = Robot._string_to_floats(response.data)

                elif response.id == MX_ST_GET_STATUS_ROBOT:
                    Robot._handle_robot_status_response(response, robot_state, events, callback_queue)
                    events.on_status_updated.set()
                    callback_queue.put(CallbackTag('on_status_updated'))

                elif response.id == MX_ST_RT_NC_JOINT_POS:
                    robot_state.nc_joint_positions[:] = Robot._string_to_floats(response.data)
                elif response.id == MX_ST_RT_NC_CART_POS:
                    robot_state.nc_end_effector_pose[:] = Robot._string_to_floats(response.data)
                elif response.id == MX_ST_RT_NC_JOINT_VEL:
                    robot_state.nc_joint_velocity[:] = Robot._string_to_floats(response.data)
                elif response.id == MX_ST_RT_NC_CART_VEL:
                    robot_state.nc_end_effector_velocity[:] = Robot._string_to_floats(response.data)

                elif response.id == MX_ST_RT_NC_CONF:
                    robot_state.nc_joint_configurations[:] = Robot._string_to_floats(response.data)
                elif response.id == MX_ST_RT_NC_CONF_MULTITURN:
                    robot_state.nc_multiturn[:] = Robot._string_to_floats(response.data)

                elif response.id == MX_ST_RT_DRIVE_JOINT_POS:
                    robot_state.drive_joint_positions[:] = Robot._string_to_floats(response.data)
                elif response.id == MX_ST_RT_DRIVE_CART_POS:
                    robot_state.drive_end_effector_pose[:] = Robot._string_to_floats(response.data)
                elif response.id == MX_ST_RT_DRIVE_JOINT_VEL:
                    robot_state.drive_joint_velocity[:] = Robot._string_to_floats(response.data)
                elif response.id == MX_ST_RT_DRIVE_JOINT_TORQ:
                    robot_state.drive_joint_torque_ratio[:] = Robot._string_to_floats(response.data)
                elif response.id == MX_ST_RT_DRIVE_CART_VEL:
                    robot_state.drive_end_effector_velocity[:] = Robot._string_to_floats(response.data)

                elif response.id == MX_ST_RT_DRIVE_CONF:
                    robot_state.drive_joint_configurations[:] = Robot._string_to_floats(response.data)
                elif response.id == MX_ST_RT_DRIVE_CONF_MULTITURN:
                    robot_state.drive_multiturn[:] = Robot._string_to_floats(response.data)

                elif response.id == MX_ST_RT_ACCELEROMETER:
                    robot_state.accelerometer[:] = Robot._string_to_floats(response.data)

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
    def _handle_callbacks(logger, callback_queue, callbacks, block_on_empty=True):
        """Runs callbacks found in callback_queue.

        Parameters
        ----------
        logger : logger instance
            Logger to use.
        callback_queue : queue
            Stores triggered callbacks.
        callbacks : RobotCallbacks instance
            Stores user-defined callback functions.
        block_on_empty : bool
            If true, will wait on elements from queue. Else will terminate on empty queue.
        """
        while True:
            # If we are not blocking on empty, return if empty.
            if not block_on_empty and callback_queue.qsize() == 0:
                return

            item = callback_queue.get(block=block_on_empty)
            if item == TERMINATE_PROCESS:
                return
            else:
                func = callbacks.__dict__[item.callback_name]
                if func != None:
                    if item.data != None:
                        func(item.data)
                    else:
                        func()

    #####################################################################################
    # Private methods.
    #####################################################################################

    def _check_monitor_processes(self):
        """Check that the processes which handle robot messages are alive. Attempt to disconnect from the robot if not.

        """
        if self._offline_mode:
            return True

        if not (self._command_response_handler_process and self._command_response_handler_process.is_alive()):
            self.Disconnect()
            raise InvalidStateError('Command response handler process has unexpectedly terminated.')
        if not (self._monitor_handler_process and self._monitor_handler_process.is_alive()):
            self.Disconnect()
            raise InvalidStateError('Command response handler process has unexpectedly terminated.')

        if not (self._command_rx_process and self._command_rx_process.is_alive()):
            self.Disconnect()
            raise InvalidStateError('Command rx process has unexpectedly terminated.')
        if not (self._monitor_rx_process and self._monitor_rx_process.is_alive()):
            self.Disconnect()
            raise InvalidStateError('Monitor rx process has unexpectedly termianted.')

        # If tx process is down, attempt to directly send deactivate command to the robot.
        if not (self._command_tx_process and self._command_tx_process.is_alive()):
            self._command_socket.sendall(b'DeactivateRobot\0')
            self.Disconnect()
            raise InvalidStateError('Command tx process has unexpectedly terminated.')

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
            # Clean up processes and connections on error.
            self.Disconnect()
            raise

        return True

    def _launch_process(self, *, target, args):
        """Establish the processes responsible for reading/sending messages using the sockets.

        Parameters
        ----------
        func : function handle
            Function to run using new process.
        args : argument list
            Arguments to be passed to func.

        Return
        ------
        process handle
            Handle for newly-launched process.

        """
        # We use the _deactivate_on_exception function which wraps func around try...except and disconnects on error.
        # The first argument is the actual function to be executed, the second is the command socket.
        process = mp.Process(target=self._deactivate_on_exception, args=(
            target,
            self._command_socket,
            *args,
        ))
        process.start()
        return process

    def _establish_socket_processes(self):
        """Establish the processes responsible for reading/sending messages using the sockets.

        Return
        ------
        bool
            True if both all processes are successfully launched.

        """
        if self._offline_mode:
            return True

        try:
            # Create rx process for command socket communication.
            self._command_rx_process = self._launch_process(target=self._handle_socket_rx,
                                                            args=(self._command_socket, self._command_rx_queue))

            # Create tx process for command socket communication.
            self._command_tx_process = self._launch_process(target=self._handle_socket_tx,
                                                            args=(self._command_socket, self._command_tx_queue))

            # Create rx processes for monitor socket communication.
            self._monitor_rx_process = self._launch_process(target=self._handle_socket_rx,
                                                            args=(self._monitor_socket, self._monitor_rx_queue))

        except:
            # Clean up processes and connections on error.
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
        except BaseException as e:
            self.logger.error('No response received within timeout interval. ' + str(e))
            self.Disconnect()
            raise

        # Check that response is appropriate.
        if response.id != MX_ST_CONNECTED:
            self.logger.error('Connection error: %s', response)
            self.Disconnect()
            raise CommunicationError('Connection error: %s', response)

        self._command_response_handler_process = self._launch_process(
            target=self._command_response_handler,
            args=(self._command_rx_queue, self._robot_state, self._user_checkpoints, self._internal_checkpoints,
                  self._robot_events, self._main_lock, self.logger, self._callback_queue))
        return True

    def _initialize_monitoring_connection(self):
        """Attempt to connect to the monitor port of the Mecademic Robot.

        Returns
        -------
        status : boolean
            Returns the status of the connection, true for success, false for failure.

        """

        self._monitor_handler_process = self._launch_process(target=self._monitor_handler,
                                                             args=(self._monitor_rx_queue, self._robot_state,
                                                                   self._robot_events, self._main_lock,
                                                                   self._callback_queue))

        return True

    def _shut_down_queue_processes(self):
        """Attempt to gracefully shut down processes which read from queues.

        """
        # Join processes which wait on a queue by sending terminate to the queue.
        # Don't acquire _main_lock since these processes require _main_lock to finish processing.
        if self._command_tx_process is not None:
            try:
                self._command_tx_queue.put(TERMINATE_PROCESS)
            except Exception as e:
                self._command_tx_process.terminate()
                self.logger.error('Error shutting down tx process. ' + str(e))
            self._command_tx_process.join()
            self._command_tx_process = None

        if self._command_response_handler_process is not None:
            try:
                self._command_rx_queue.put(TERMINATE_PROCESS)
            except Exception as e:
                self._command_response_handler_process.terminate()
                self.logger.error('Error shutting down command response handler process. ' + str(e))
            self._command_response_handler_process.join()
            self._command_response_handler_process = None

        if self._monitor_handler_process is not None:
            try:
                self._monitor_rx_queue.put(TERMINATE_PROCESS)
            except Exception as e:
                self._monitor_handler_process.terminate()
                self.logger.error('Error shutting down monitor handler process. ' + str(e))
            self._monitor_handler_process.join()
            self._monitor_handler_process = None

    def _shut_down_socket_processes(self):
        """Attempt to gracefully shut down processes which read from sockets.

        """
        with self._main_lock:
            # Shutdown socket to terminate the rx processes.
            if self._command_socket is not None:
                try:
                    self._command_socket.shutdown(socket.SHUT_RDWR)
                except Exception as e:
                    self.logger.error('Error shutting down command socket. ' + str(e))
                    if self._command_rx_process is not None:
                        self._command_rx_process.terminate()

            if self._monitor_socket is not None:
                try:
                    self._monitor_socket.shutdown(socket.SHUT_RDWR)
                except Exception as e:
                    self.logger.error('Error shutting down monitor socket. ' + str(e))
                    if self._monitor_rx_process is not None:
                        self._monitor_rx_process.terminate()

            # Join processes which wait on a socket.
            if self._command_rx_process is not None:
                if not self._command_rx_process.join(timeout=1):
                    self._command_rx_process.terminate()
                self._command_rx_process = None

            if self._monitor_rx_process is not None:
                if not self._monitor_rx_process.join(timeout=1):
                    self._monitor_rx_process.terminate()
                self._monitor_rx_process = None

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
                checkpoints_dict[n] = self._manager.list()
            event = self._manager.Event()
            checkpoints_dict[n].append(event)

            if send_to_robot:
                self._send_command('SetCheckpoint', [n])

            return Checkpoint(n, event)

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
            self._callback_queue.put(TERMINATE_PROCESS)
            self._callback_thread.join()

        self._callback_queue = mp.Queue()
        self._robot_callbacks = None
        self._callback_thread = None

    def RunCallbacks(self):
        """Run all triggered callback functions.

        """
        if self._callback_thread:
            raise InvalidStateError(
                'Cannot call RunCallbacks since callback handler is already running in separate thread.')

        self._handle_callbacks(self.logger, self._callback_queue, self._robot_callbacks, block_on_empty=False)

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
            self._establish_socket_processes()
            self._initialize_command_connection()
            self._initialize_monitoring_connection()

            self._robot_events.on_disconnected.clear()
            self._robot_events.on_connected.set()
            self._callback_queue.put(CallbackTag('on_connected'))

            return True

    def Disconnect(self):
        """Disconnects Mecademic Robot object from the physical Mecademic robot.

        """
        self.logger.debug('Disconnecting from the robot.')

        # Don't acquire _main_lock while shutting down queues to avoid deadlock.
        self._shut_down_queue_processes()

        with self._main_lock:
            self._shut_down_socket_processes()

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
            self._callback_queue.put(CallbackTag('on_disconnected'))

    @disconnect_on_exception
    def ActivateRobot(self):
        """Activate the robot.

        """
        with self._main_lock:
            self._check_monitor_processes()
            self._send_command('ActivateRobot')

        if self._enable_synchronous_mode:
            self.WaitActivated()

    @disconnect_on_exception
    def Home(self):
        """Home the robot.

        """
        with self._main_lock:
            self._check_monitor_processes()
            if not self._robot_events.on_activated.is_set():
                raise InvalidStateError('Robot must be activated before homing.')
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
            self._check_monitor_processes()
            self._send_command('PauseMotion')

        if self._enable_synchronous_mode:
            self._robot_events.on_motion_paused.wait()

    @disconnect_on_exception
    def ResumeMotion(self):
        """Un-pause robot motion.

        """
        with self._main_lock:
            self._check_monitor_processes()
            self._send_command('ResumeMotion')

        if self._enable_synchronous_mode:
            self.WaitMotionResumed()

    @disconnect_on_exception
    def DeactivateRobot(self):
        """Deactivate the robot.

        """
        with self._main_lock:
            self._check_monitor_processes()
            self._send_command('DeactivateRobot')

        if self._enable_synchronous_mode:
            self.WaitDeactivated()

    @disconnect_on_exception
    def ClearMotion(self):
        """Clear the motion queue, includes implicit PauseMotion command.

        """
        with self._main_lock:
            self._check_monitor_processes()

            # Create an event to track when this ClearMotion is acknowledged.
            if 'clear_motion' not in self._internal_checkpoints:
                self._internal_checkpoints['clear_motion'] = self._manager.list()
            event = self._manager.Event()
            self._internal_checkpoints['clear_motion'].append(event)

            self._send_command('ClearMotion')

            # Clearing the motion queue also requires clearing checkpoints, as the robot will not send them anymore.
            self._user_checkpoints.clear()
            self._internal_checkpoints.clear()
            self._internal_checkpoint_counter = MX_CHECKPOINT_ID_MAX + 1

        if self._enable_synchronous_mode:
            event.wait()

    @disconnect_on_exception
    def MoveJoints(self, joint1, joint2, joint3, joint4, joint5, joint6):
        """Moves joints to desired positions.

        Parameters
        ----------
        joint1...joint6 : float
            Desired joint angles in degrees.

        """
        with self._main_lock:
            self._check_monitor_processes()
            self._send_command('MoveJoints', [joint1, joint2, joint3, joint4, joint5, joint6])
            if self._enable_synchronous_mode:
                checkpoint = self._set_checkpoint_internal()

        if self._enable_synchronous_mode:
            checkpoint.wait()

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
            self._check_monitor_processes()
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
            self._check_monitor_processes()
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
            self._check_monitor_processes()
            if '*' not in self._internal_checkpoints:
                self._internal_checkpoints['*'] = self._manager.list()
            event = self._manager.Event()
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

        with self._main_lock:
            self._check_monitor_processes()

            # Only wait if there are pending ClearMotion calls.
            if 'clear_motion' not in self._internal_checkpoints or not self._internal_checkpoints['clear_motion']:
                return True

            # Register an event to wait for all ClearMotion calls to be acknowledged.
            if 'clear_motion_all' not in self._internal_checkpoints:
                self._internal_checkpoints['clear_motion_all'] = self._manager.list()
            event = self._manager.Event()
            self._internal_checkpoints['clear_motion_all'].append(event)

        return event.wait(timeout=timeout)

    @disconnect_on_exception
    def ResetError(self):
        """Attempt to reset robot error.

        """
        with self._main_lock:
            self._check_monitor_processes()
            self._send_command('ResetError')

        if self._enable_synchronous_mode:
            self._robot_events.on_error_reset.wait()

    @disconnect_on_exception
    def ResetPStop(self):
        """Attempt to reset robot pstop.

        """
        with self._main_lock:
            self._check_monitor_processes()
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
            self._check_monitor_processes()
            if not self._robot_events.on_homed.is_set():
                raise InvalidStateError('This command requires robot to be homed.')
            self._send_command('Delay', [t])
            if self._enable_synchronous_mode:
                checkpoint = self._set_checkpoint_internal()

        if self._enable_synchronous_mode:
            checkpoint.wait()

    ### Information-getting commands.

    @disconnect_on_exception
    def GetJoints(self):
        """Returns the current joint positions of the robot.

        Return
        ------
        list of floats
            Returns list of joint positions in degrees.

        """
        with self._main_lock:
            self._check_monitor_processes()
            return self._robot_state.joint_positions[:]

    @disconnect_on_exception
    def GetEndEffectorPose(self):
        """Returns the current end-effector pose of the robot. WARNING: NOT UNIQUE.

        Return
        ------
        list of floats
            Returns end-effector pose [x, y, z, alpha, beta, gamma].

        """
        with self._main_lock:
            self._check_monitor_processes()
            return self._robot_state.end_effector_pose[:]