#!/usr/bin/env python3
import logging
import ipaddress
import time
import socket
import multiprocessing as mp
import queue
from collections import deque

COMMAND_PORT = 10000
MONITOR_PORT = 10001

CHECKPOINT_ID_MIN = 1  # Min allowable checkpoint id for users, inclusive
CHECKPOINT_ID_MAX = 8000  # Max allowable checkpoint id for users, inclusive
CHECKPOINT_ID_MAX_PRIVATE = 8191  # Max allowable checkpoint id, inclusive


class InvalidStateError(Exception):
    pass


class CommunicationError(Exception):
    pass


class RobotState:
    """Class for storing the internal state of a generic Mecademic robot.

    Attributes
    ----------


    """
    def __init__(self):
        self.joint_positions = mp.Array('f', 6)
        self.end_effector_pose = mp.Array('f', 6)

        # The following status fields are updated together, and is protected by a single lock.
        self.status_lock = mp.Lock()
        self.activation_state = mp.Value('b', False, lock=False)
        self.homing_state = mp.Value('b', False, lock=False)
        self.simulation_mode = mp.Value('b', False, lock=False)
        self.error_status = mp.Value('b', False, lock=False)
        self.pause_motion_status = mp.Value('b', False, lock=False)
        self.end_of_block_status = mp.Value('b', False, lock=False)
        self.end_of_movement_status = mp.Value('b', False, lock=False)


class Robot:
    """Class for controlling a generic Mecademic robot.

    Attributes
    ----------
    address : string
        The IP address associated to the Mecademic Robot.
    socket : socket object
        Socket connecting to physical Mecademic Robot.
    enable_synchronous_mode : boolean
        True if synchronous mode is enabled.

    """
    def __init__(self, address='192.168.0.100', enable_synchronous_mode=False):
        """Constructor for an instance of the Controller class.

        Parameters
        ----------
        address : string
            The IP address associated to the Mecademic Robot.
        enable_synchronous_mode : bool
            If true, each command will wait until previous is done executing.

        """

        # Check that the ip address is a string.
        if not isinstance(address, str):
            raise TypeError('Please provide a string argument for the address.')

        # Check that the ip address is valid.
        ipaddress.ip_address(address)

        self.__address = address

        self.__command_socket = None
        self.__monitor_socket = None

        self.__command_rx_process = None
        self.__command_rx_queue = mp.Queue()
        self.__command_tx_process = None
        self.__command_tx_queue = mp.Queue()
        self.__monitor_rx_process = None
        self.__monitor_rx_queue = mp.Queue()

        self.__monitor_handler_process = None
        self.__command_response_handler_process = None

        self.__main_lock = mp.RLock()

        self.__robot_state = RobotState()

        self.__manager = mp.Manager()
        self.__received_checkpoints = self.__manager.list()
        self.__blocking_checkpoint_id = mp.Value('i', -1)
        self.__checkpoint_event = self.__manager.Event()
        self.__internal_checkpoint_counter = CHECKPOINT_ID_MAX + 1

        self.__enable_synchronous_mode = enable_synchronous_mode
        self.logger = logging.getLogger(__name__)

    @staticmethod
    def _handle_rx(robot_socket, rx_queue):
        """Handle received data on the socket.
        Parameters
        ----------
        command_socket : socket
            Socket to use for receiving data.

        rx_queue : queue
            Thread-safe queue to push complete messages onto.

        """
        robot_socket.setblocking(True)
        remainder = ''
        while True:
            # Wait for a message from the robot.
            try:
                raw_responses = robot_socket.recv(1024)
            except:
                break

            # Socket has been closed.
            if raw_responses == b'':
                break

            responses = raw_responses.decode('ascii').split('\0')

            # Add the remainder from the previous message if necessary.
            if remainder != '':
                responses[0] = remainder + responses[0]

            # Set the remainder as the last response (which is '' if complete).
            remainder = responses[-1]

            # Put all responses into the queue.
            [rx_queue.put(x) for x in responses[:-1]]

    @staticmethod
    def _handle_tx(robot_socket, tx_queue):
        """Handle sending data on the socket.
        Parameters
        ----------
        command_socket : socket
            Socket to use for sending data.

        tx_queue : queue
            Thread-safe queue to get messages from.

        """
        while True:
            # Wait for a command to be available from the queue.
            command = tx_queue.get(block=True)

            # Terminate process if requested, otherwise send the command.
            if command == 'Terminate process':
                return
            else:
                try:
                    robot_socket.sendall((command + '\0').encode('ascii'))
                except:
                    continue  # TODO handle properly by raising an error

    @staticmethod
    def _command_response_handler(rx_queue, robot_state, received_checkpoints, blocking_checkpoint_id, checkpoint_event,
                                  main_lock):
        """Handle received messages on the command socket.
        Parameters
        ----------
        rx_queue : queue
            Thread-safe queue to get received messages from.

        """
        while True:
            # Wait for a response to be available from the queue.
            response = rx_queue.get(block=True)

            # Terminate process if requested.
            if response == 'Terminate process':
                return

            with main_lock:
                # Handle checkpoints.
                if response[1:5] == '3030':
                    checkpoint_id = int(response[7:-1])
                    # If duplicate has been received, remove it and all checkpoints before.
                    if checkpoint_id in received_checkpoints:
                        checkpoint_index = received_checkpoints.index(n)
                        for _ in range(checkpoint_index + 1):
                            received_checkpoints.pop(0)
                    received_checkpoints.append(checkpoint_id)
                    if checkpoint_id == blocking_checkpoint_id.value:
                        checkpoint_event.set()

    @staticmethod
    def _monitor_handler(monitor_queue, robot_state, main_lock):
        """Handle received messages on the monitor socket.
        Parameters
        ----------
        monitor_queue : queue
            Thread-safe queue to get received messages from.

        """
        while True:
            # Wait for a message in the queue.
            response = monitor_queue.get(block=True)

            # Terminate process if requested.
            if response == 'Terminate process':
                return

            with main_lock:
                if response[1:5] == '2026':
                    with robot_state.joint_positions.get_lock():
                        robot_state.joint_positions.get_obj()[:] = [float(x) for x in response[7:-1].split(',')]

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
        command : string
            Final command for the Mecademic Robot

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

    def _send_command(self, command, arg_list=None):
        """Assembles and sends the command string to the Mecademic Robot.

        Parameters
        ----------
        cmd : string
            Command name to send to the Mecademic Robot
        arg_list : list
            List of arguments the command requires

        """

        # Assemble arguments into a string and concatenate to end of command.
        if arg_list:
            command = command + '(' + ','.join([str(x) for x in arg_list]) + ')'
        self.__command_tx_queue.put(command)

    def _establish_socket_connections(self, offline_mode=False):
        if offline_mode:
            return True

        if self.__command_socket is not None:
            self.logger.warning('Existing command connection found, invalid state.')
            raise InvalidStateError

        if self.__monitor_socket is not None:
            self.logger.warning('Existing monitor connection found, this should not be possible, closing socket...')
            raise InvalidStateError

        try:
            self.__command_socket = self._connect_socket(self.logger, self.__address, COMMAND_PORT)

            if self.__command_socket is None:
                self.logger.error('Error creating socket.')
                raise CommunicationError

            self.__monitor_socket = self._connect_socket(self.logger, self.__address, MONITOR_PORT)

            if self.__monitor_socket is None:
                self.logger.error('Error creating socket.')
                raise CommunicationError

        except:
            # Clean up processes and connections on error.
            self.Disconnect()
            return False

        return True

    def _establish_socket_processes(self, offline_mode=False):
        if offline_mode:
            return True

        try:
            # Create tx process for command socket communication.
            self.__command_rx_process = mp.Process(target=self._handle_rx,
                                                   args=(
                                                       self.__command_socket,
                                                       self.__command_rx_queue,
                                                   ))
            self.__command_rx_process.start()

            # Create rx process for command socket communication.
            self.__command_tx_process = mp.Process(target=self._handle_tx,
                                                   args=(
                                                       self.__command_socket,
                                                       self.__command_tx_queue,
                                                   ))
            self.__command_tx_process.start()

            # Create rx processes for monitor socket communication.
            self.__monitor_rx_process = mp.Process(target=self._handle_rx,
                                                   args=(
                                                       self.__monitor_socket,
                                                       self.__monitor_rx_queue,
                                                   ))
            self.__monitor_rx_process.start()

        except:
            # Clean up processes and connections on error.
            self.Disconnect()
            return False

        return True

    def _initialize_command_connection(self):
        """Attempt to connect to the command port of the Mecademic Robot.

        Returns
        -------
        status : boolean
            Returns the status of the connection, true for success, false for failure.

        """

        try:
            response = self.__command_rx_queue.get(block=True, timeout=10)  # 10s timeout.
        except:
            self.logger.error('No response received within timeout interval.')
            self.Disconnect()
            return False

        # Check that response is appropriate.
        if response[1:5] != '3000':
            self.logger.error('Connection error: %s', response)
            self.Disconnect()
            return False

        self.__command_response_handler_process = mp.Process(
            target=self._command_response_handler,
            args=(self.__command_rx_queue, self.__robot_state, self.__received_checkpoints,
                  self.__blocking_checkpoint_id, self.__checkpoint_event, self.__main_lock))
        self.__command_response_handler_process.start()

        return True

    def _initialize_monitoring_connection(self):
        """Attempt to connect to the monitor port of the Mecademic Robot.

        Returns
        -------
        status : boolean
            Returns the status of the connection, true for success, false for failure.

        """

        self.__monitor_handler_process = mp.Process(target=self._monitor_handler,
                                                    args=(self.__monitor_rx_queue, self.__robot_state,
                                                          self.__main_lock))
        self.__monitor_handler_process.start()

        return True

    def Connect(self, offline_mode=False):
        """Attempt to connect to a physical Mecademic Robot.

        Returns
        -------
        status : boolean
            Returns the status of the connection, true for success, false for failure.

        """
        with self.__main_lock:
            if not self._establish_socket_connections(offline_mode=offline_mode):
                return False
            if not self._establish_socket_processes(offline_mode=offline_mode):
                return False
            if not self._initialize_command_connection():
                return False
            if not self._initialize_monitoring_connection():
                return False

            return True

    def ActivateRobot(self):
        with self.__main_lock:
            self._send_command('ActivateRobot')

    def Home(self):
        with self.__main_lock:
            self._send_command('Home')

    def ActivateAndHome(self):
        with self.__main_lock:
            self.ActivateRobot()
            self.Home()

    def DeactivateRobot(self):
        with self.__main_lock:
            self._send_command('DeactivateRobot')

    def Disconnect(self):
        """Disconnects Mecademic Robot object from physical Mecademic Robot.

        """
        self.DeactivateRobot()

        # Join processes which wait on a queue by sending terminate to the queue.
        if self.__command_tx_process is not None:
            try:
                self.__command_tx_queue.put('Terminate process')
            except:
                self.__command_tx_process.terminate()
                self.logger.error('Error shutting down tx process.')
            self.__command_tx_process.join()
            self.__command_tx_process = None

        if self.__command_response_handler_process is not None:
            try:
                self.__command_rx_queue.put('Terminate process')
            except:
                self.__command_response_handler_process.terminate()
                self.logger.error('Error shutting down command response handler process.')
            self.__command_response_handler_process.join()
            self.__command_response_handler_process = None

        if self.__monitor_handler_process is not None:
            try:
                self.__monitor_rx_queue.put('Terminate process')
            except:
                self.__monitor_handler_process.terminate()
                self.logger.error('Error shutting down monitor handler process.')
            self.__monitor_handler_process.join()
            self.__monitor_handler_process = None

        with self.__main_lock:
            # Shutdown socket to terminate the rx processes.
            if self.__command_socket is not None:
                try:
                    self.__command_socket.shutdown(socket.SHUT_RDWR)
                except:
                    self.logger.error('Error shutting down command socket.')
                    if self.__command_rx_process is not None:
                        self.__command_rx_process.terminate()

            if self.__monitor_socket is not None:
                try:
                    self.__monitor_socket.shutdown(socket.SHUT_RDWR)
                except:
                    self.logger.error('Error shutting down monitor socket.')
                    if self.__monitor_rx_process is not None:
                        self.__monitor_rx_process.terminate()

            # Join processes which wait on a socket.
            if self.__command_rx_process is not None:
                self.__command_rx_process.join()
                self.__command_rx_process = None

            if self.__monitor_rx_process is not None:
                self.__monitor_rx_process.join()
                self.__monitor_rx_process = None

            # Reset communication queues (not strictly necessary since these should be empty).
            self.__command_rx_queue = mp.Queue()
            self.__command_tx_queue = mp.Queue()
            self.__monitor_rx_queue = mp.Queue()

            # Reset robot state.
            self.__robot_state = RobotState()
            self.__incoming_checkpoints_queue = mp.Queue()
            self.__internal_checkpoint_counter = CHECKPOINT_ID_MAX + 1

            # Finally, close sockets.
            if self.__command_socket is not None:
                try:
                    self.__command_socket.close()
                except:
                    self.logger.error('Error closing command socket.')
                self.__command_socket = None
            if self.__monitor_socket is not None:
                try:
                    self.__monitor_socket.close()
                except:
                    self.logger.error('Error closing monitor socket.')
                self.__monitor_socket = None

    def GetJoints(self):
        with self.__main_lock:
            return self.__robot_state.joint_positions.get_obj()[:]

    def MoveJoints(self, joint1, joint2, joint3, joint4, joint5, joint6):
        with self.__main_lock:
            self._send_command('MoveJoints', [joint1, joint2, joint3, joint4, joint5, joint6])
            if self.__enable_synchronous_mode:
                checkpoint_id = self._set_checkpoint_internal()

        if self.__enable_synchronous_mode:
            self.WaitCheckpoint(checkpoint_id)

    def SetCheckpoint(self, n):
        with self.__main_lock:
            assert CHECKPOINT_ID_MIN <= n <= CHECKPOINT_ID_MAX
            self._set_checkpoint_impl(n)

    def _set_checkpoint_internal(self):
        with self.__main_lock:
            checkpoint_id = self.__internal_checkpoint_counter
            self._set_checkpoint_impl(checkpoint_id)

            self.__internal_checkpoint_counter += 1
            if self.__internal_checkpoint_counter > 8191:
                self.__internal_checkpoint_counter.value = CHECKPOINT_ID_MAX + 1
            return checkpoint_id

    def _set_checkpoint_impl(self, n):
        if not isinstance(n, int):
            raise TypeError('Please provide a string argument for the address.')

        assert CHECKPOINT_ID_MIN <= n <= CHECKPOINT_ID_MAX_PRIVATE

        self.logger.debug('Setting checkpoint %s', n)

        with self.__main_lock:
            # Checkpoint n from received list (if found).
            if n in self.__received_checkpoints:
                checkpoint_index = self.__received_checkpoints.index(n)
                self.__received_checkpoints.pop(checkpoint_index)

            self._send_command('SetCheckpoint', [n])

    def WaitCheckpoint(self, n, timeout=None):
        self.logger.debug('Waiting for checkpoint %s', n)

        with self.__main_lock:
            # Only one instance can wait on a checkpoint at a time.
            assert self.__blocking_checkpoint_id.value == -1

            if n in self.__received_checkpoints:
                return True

            self.__blocking_checkpoint_id.value = n

        if self.__checkpoint_event.wait(timeout=timeout):
            with self.__main_lock:
                self.__checkpoint_event.clear()
                self.__blocking_checkpoint_id.value = -1
                return True

        self.__blocking_checkpoint_id.value = -1
        return False
