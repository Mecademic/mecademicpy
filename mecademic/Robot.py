#!/usr/bin/env python3
import logging
import ipaddress
import time
import socket
import multiprocessing as mp


class Robot:
    """Class for the Mecademic Robot allowing for communication and control of the
    Mecademic Robot with all of its features available.

    Attributes
    ----------
    address : string
        The IP address associated to the Mecademic Robot.
    socket : socket object
        Socket connecting to physical Mecademic Robot.
    enable_synchronous_mode : boolean
        True if synchronous mode is enabled.

    """
    def __init__(self, address, enable_synchronous_mode=False):
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
        self.__socket = None

        self.__rx_process = None
        self.__rx_queue = None
        self.__tx_process = None
        self.__tx_queue = None

        self.__enable_synchronous_mode = enable_synchronous_mode
        self.logger = logging.getLogger(__name__)

    @staticmethod
    def _rx(robot_socket, rx_queue):
        remainder = ''
        while True:
            # Wait for a message from the robot.
            try:
                responses = robot_socket.recv(1024).decode('ascii').split('\0')
            except:
                break

            # Add the remainder from the previous message if necessary.
            if remainder != '':
                responses[0] = responses[0] + remainder

            # Set the remainder as the last response (which is '' if complete).
            remainder = responses[-1]

            # Put all responses into the queue.
            [rx_queue.put(x) for x in responses[:-1]]

    @staticmethod
    def _tx(robot_socket, tx_queue):
        while True:
            # Wait for a command to be available from the queue.
            command = tx_queue.get(block=True)

            # Terminate process if requested, otherwise send the command.
            if command == 'Terminate tx process':
                return
            else:
                try:
                    robot_socket.sendall((command + '\0').encode('ascii'))
                except:
                    continue  # TODO handle properly by raising an error

    @staticmethod
    def _callbacks(rx_queue, event_list, checkpoint_list):
        while True:
            # Wait for a response to be available from the queue.
            response = rx_queue.get(block=True)

            # Terminate process if requested.
            if command == 'Terminate rx process':
                return

    def Connect(self):
        """Attempt to connect to a physical Mecademic Robot.

        Returns
        -------
        status : boolean
            Returns the status of the connection, true for success, false for failure.

        """

        self.logger.debug('Attempting to connect to %s', self.__address)

        if self.__socket is not None:
            self.logger.warning('Existing connection found, this should not be possible, closing...')
            self.Disconnect()

        # Create socket and attempt connection.
        self.__socket = socket.socket()
        self.__socket.settimeout(0.1)  # 100ms
        try:
            self.__socket.connect((self.__address, 10000))
        except socket.timeout:
            self.logger.error('Unable to connect to socket.')
            return False

        if self.__socket is None:
            self.logger.error('Socket object does not exist.')
            return False

        # Create tx and rx processes and queues for socket communication.
        self.__rx_queue = mp.Queue()
        self.__tx_queue = mp.Queue()
        self.__rx_process = mp.Process(target=self._rx, args=(
            self.__socket,
            self.__rx_queue,
        ))
        self.__tx_process = mp.Process(target=self._tx, args=(
            self.__socket,
            self.__tx_queue,
        ))

        self.__rx_process.start()
        self.__tx_process.start()

        try:
            response = self.__rx_queue.get(block=True, timeout=10)  # 10s timeout.
        except:
            self.logger.error('No response received within timeout interval.')
            self.Disconnect()
            return False

        # Check that response is appropriate.
        if response[1:5] == '3001':
            self.logger.error('Another user is already connected, closing connection.')
            self.Disconnect()
            return False
        elif response[1:5] == '3000':
            self.logger.debug('Connection successfully established.')
            return True
        else:
            self.logger.error('Unexpected code returned: %s', response)
            self.Disconnect()
            return False

    def Disconnect(self):
        """Disconnects Mecademic Robot object from physical Mecademic Robot.

        """

        # Shutdown socket to terminate the rx process.
        if self.__socket is not None:
            self.__socket.shutdown(socket.SHUT_RDWR)

        # Join processes to avoid bad behaviour.
        if self.__rx_process is not None:
            try:
                self.__rx_process.join()
                self.__rx_process = None
            except:
                self.logger.error('Error shutting down rx process.')
        if self.__tx_process is not None:
            try:
                # Terminate tx process by putting 'terminate' in queue.
                self.__tx_queue.put('Terminate tx process')
                self.__tx_process.join()
                self.__tx_process = None
            except:
                self.logger.error('Error shutting down tx process.')

        self.__rx_queue = None
        self.__tx_queue = None

        if self.__socket is not None:
            self.__socket.close()
            self.__socket = None