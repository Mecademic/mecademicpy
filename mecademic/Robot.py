#!/usr/bin/env python3
import logging
import ipaddress
import socket


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
        self.__enable_synchronous_mode = enable_synchronous_mode
        self.logger = logging.getLogger(__name__)

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
            self.__socket.close()
            self.__socket = None

        try:
            # Create socket and attempt connection.
            self.__socket = socket.socket()
            self.__socket.settimeout(0.1)  # 100ms
            try:
                self.__socket.connect((self.__address, 10000))
            except socket.timeout:
                self.logger.error('Unable to connect to socket.')
                raise TimeoutError

            if self.__socket is None:
                self.logger.error('Socket object does not exist.')
                raise RuntimeError

            # Receive response of connection from robot.
            self.__socket.settimeout(10)  # 10 seconds
            try:
                response = self.__socket.recv(1024).decode('ascii')
            except socket.timeout:
                self.logger.error('No response received within timeout interval.')
                raise RuntimeError

            # Check that response is appropriate.
            if self._response_contains(response, ['[3001]']):
                self.logger.error('Another user is already connected, closing connection.')
                raise RuntimeError
            elif self._response_contains(response, ['[3000]']):
                self.logger.debug('Connection successfully established.')
                return True
            else:
                self.logger.error('Unexpected code returned: %s', response)
                raise RuntimeError

        except TimeoutError:
            self.logger.error('Connection timeout.')
            self.Disconnect()
            return False
        except RuntimeError:
            self.logger.error('Unable to establish connection.')
            self.Disconnect()
            return False

    def Disconnect(self):
        """Disconnects Mecademic Robot object from physical Mecademic Robot.

        """
        if (self.__socket is not None):
            self.__socket.close()
            self.__socket = None

    @staticmethod
    def _response_contains(response, code_list):
        """Scans received response for code IDs.

        Parameters
        ----------
        response :
            Message to scan for codes.
        code_list :
            List of codes to look for in the response.

        Returns
        -------
        response_found :
            Returns whether the response contains a code ID of interest.

        """
        response_found = False
        for code in code_list:
            if response.find(code) != -1:
                response_found = True
                break
        return response_found