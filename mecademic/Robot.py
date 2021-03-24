#!/usr/bin/env python3
import logging
import ipaddress
import time
import socket
import multiprocessing as mp
import numpy as np

COMMAND_PORT = 10000
MONITOR_PORT = 10001


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
        self.__monitor__rx_process = None
        self.__monitor__rx_queue = mp.Queue()

        self.__monitor_handler_process = None

        self.joint_positions = mp.Array('f', 6)
        self.end_effector_pose = mp.Array('f', 6)
        self.activation_state = mp.Value('b', False)
        self.homing_state = mp.Value('b', False)
        self.simulation_mode = mp.Value('b', False)
        self.error_status = mp.Value('b', False)
        self.pause_motion_status = mp.Value('b', False)
        self.end_of_block_status = mp.Value('b', False)
        self.end_of_movement_status = mp.Value('b', False)

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
                    print("sent: ", (command + '\0').encode('ascii'))
                    robot_socket.sendall((command + '\0').encode('ascii'))
                except:
                    continue  # TODO handle properly by raising an error

    @staticmethod
    def _command_response_handler(rx_queue, event_list, checkpoint_list):
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

    @staticmethod
    def _monitor_handler(monitor_queue, joint_positions, end_effector_pose, activation_state, homing_state,
                         simulation_mode, error_status, pause_motion_status, end_of_block_status,
                         end_of_movement_status):
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
            elif response[1:5] == '2026':
                joint_positions.get_obj()[:] = [float(x) for x in response[7:-1].split(',')]

    @staticmethod
    def _connect_socket(logger, address, port):
        logger.debug('Attempting to connect to %s:%s', address, port)

        # Create socket and attempt connection.
        new_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        new_socket.settimeout(0.1)  # 100ms
        try:
            new_socket.connect((address, port))
        except socket.timeout:
            logger.error('Unable to connect to socket.')
            return None

        return new_socket

    def _initialize_command_connection(self):
        """Attempt to connect to the command port of the Mecademic Robot.

        Returns
        -------
        status : boolean
            Returns the status of the connection, true for success, false for failure.

        """

        if self.__command_socket is not None:
            self.logger.warning('Existing connection found, this should not be possible, closing...')
            self.Disconnect()

        self.__command_socket = self._connect_socket(self.logger, self.__address, COMMAND_PORT)

        if self.__command_socket is None:
            self.logger.error('Error creating socket.')
            return False

        # Create tx and rx processes and queues for socket communication.
        self.__command_rx_process = mp.Process(target=self._handle_rx,
                                               args=(
                                                   self.__command_socket,
                                                   self.__command_rx_queue,
                                               ))
        self.__command_tx_process = mp.Process(target=self._handle_tx,
                                               args=(
                                                   self.__command_socket,
                                                   self.__command_tx_queue,
                                               ))

        self.__command_rx_process.start()
        self.__command_tx_process.start()

        try:
            response = self.__command_rx_queue.get(block=True, timeout=10)  # 10s timeout.
        except:
            self.logger.error('No response received within timeout interval.')
            self.Disconnect()
            return False

        # Check that response is appropriate.
        if response[1:5] == '3000':
            self.logger.debug('Connection successfully established.')
            return True
        elif response[1:5] == '3001':
            self.logger.error('Another user is already connected, closing connection.')
        else:
            self.logger.error('Unexpected code returned: %s', response)

        self.Disconnect()
        return False

    def _initialize_monitoring_connection(self):
        """Attempt to connect to the monitor port of the Mecademic Robot.

        Returns
        -------
        status : boolean
            Returns the status of the connection, true for success, false for failure.

        """
        if self.__monitor_socket is not None:
            self.logger.warning('Existing connection found, this should not be possible, closing...')
            self.Disconnect()

        self.__monitor_socket = self._connect_socket(self.logger, self.__address, MONITOR_PORT)

        if self.__monitor_socket is None:
            self.logger.error('Error creating socket.')
            return False

        self.__monitor__rx_process = mp.Process(target=self._handle_rx,
                                                args=(
                                                    self.__monitor_socket,
                                                    self.__monitor__rx_queue,
                                                ))

        self.__monitor__rx_process.start()

        self.__monitor_handler_process = mp.Process(target=self._monitor_handler,
                                                    args=(
                                                        self.__monitor__rx_queue,
                                                        self.joint_positions,
                                                        self.end_effector_pose,
                                                        self.activation_state,
                                                        self.homing_state,
                                                        self.simulation_mode,
                                                        self.error_status,
                                                        self.pause_motion_status,
                                                        self.end_of_block_status,
                                                        self.end_of_movement_status,
                                                    ))
        self.__monitor_handler_process.start()

        self.__command_tx_queue.put('SetMonitoringInterval(1)')
        self.__command_tx_queue.put('ActivateRobot')
        self.__command_tx_queue.put('Home')

        return True

    def Connect(self):
        """Attempt to connect to a physical Mecademic Robot.

        Returns
        -------
        status : boolean
            Returns the status of the connection, true for success, false for failure.

        """
        if not self._initialize_command_connection():
            return False
        if not self._initialize_monitoring_connection():
            return False

        return True

    def Disconnect(self):
        """Disconnects Mecademic Robot object from physical Mecademic Robot.

        """
        self.__command_tx_queue.put('DeactivateRobot')

        # Join processes which wait on a queue by sending terminate to the queue.
        if self.__command_tx_process is not None:
            try:
                self.__command_tx_queue.put('Terminate process')
                self.__command_tx_process.join()
                self.__command_tx_process = None
            except:
                self.logger.error('Error shutting down tx process.')

        if self.__monitor_handler_process is not None:
            try:
                self.__monitor__rx_queue.put('Terminate process')
                self.__monitor_handler_process.join()
                self.__monitor_handler_process = None
            except:
                self.logger.error('Error shutting down monitor handler process.')

        # Shutdown socket to terminate the rx processes.
        if self.__command_socket is not None:
            self.__command_socket.shutdown(socket.SHUT_RDWR)
        if self.__monitor_socket is not None:
            self.__monitor_socket.shutdown(socket.SHUT_RDWR)

        # Join processes which wait on a socket.
        if self.__command_rx_process is not None:
            try:
                self.__command_rx_process.join()
                self.__command_rx_process = None
            except:
                self.logger.error('Error shutting down rx process.')

        if self.__monitor__rx_process is not None:
            try:
                self.__monitor__rx_process.join()
                self.__monitor__rx_process = None
            except:
                self.logger.error('Error shutting down monitor rx process.')

        self.__command_rx_queue = mp.Queue()
        self.__command_tx_queue = mp.Queue()
        self.__monitor__rx_queue = mp.Queue()

        if self.__command_socket is not None:
            self.__command_socket.close()
            self.__command_socket = None
        if self.__monitor_socket is not None:
            self.__monitor_socket.close()
            self.__monitor_socket = None