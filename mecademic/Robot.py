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

TERMINATE_PROCESS = 'terminate_process'


class InvalidStateError(Exception):
    pass


class CommunicationError(Exception):
    pass


class Checkpoint:
    def __init__(self, id, event):
        self.id = id
        self.event = event

    def wait(self, timeout=None):
        return self.event.wait(timeout=timeout)


class Message:
    def __init__(self, id, data):
        self.id = id
        self.data = data


class RobotState:
    """Class for storing the internal state of a generic Mecademic robot.

    Attributes
    ----------


    """
    def __init__(self):
        self.joint_positions = mp.Array('f', 6)  # degrees
        self.end_effector_pose = mp.Array('f', 6)  # mm and degrees

        self.nc_joint_positions = mp.Array('f', 7)  # degrees
        self.nc_end_effector_pose = mp.Array('f', 7)  # mm and degrees

        self.nc_joint_velocity = mp.Array('f', 7)  # degrees/second
        self.nc_end_effector_velocity = mp.Array('f', 7)  # mm/s and degrees/s

        self.nc_joint_configurations = mp.Array('f', 4)
        self.nc_multiturn = mp.Array('f', 2)

        self.drive_joint_positions = mp.Array('f', 7)  # degrees
        self.drive_end_effector_pose = mp.Array('f', 7)  # mm and degrees

        self.drive_joint_velocity = mp.Array('f', 7)  # degrees/second
        self.drive_joint_torque_ratio = mp.Array('f', 7)  # percent of maximum
        self.drive_end_effector_velocity = mp.Array('f', 7)  # mm/s and degrees/s

        self.drive_joint_configurations = mp.Array('f', 4)
        self.drive_multiturn = mp.Array('f', 2)

        self.accelerometer = mp.Array('f', 5)

        # The following status fields are updated together, and is protected by a single lock.
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

        self._address = address

        self._command_socket = None
        self._monitor_socket = None

        self._command_rx_process = None
        self._command_rx_queue = mp.Queue()
        self._command_tx_process = None
        self._command_tx_queue = mp.Queue()
        self._monitor_rx_process = None
        self._monitor_rx_queue = mp.Queue()

        self._monitor_handler_process = None
        self._command_response_handler_process = None

        self._main_lock = mp.RLock()

        self._robot_state = RobotState()

        self._manager = mp.Manager()
        self._pending_checkpoints = self._manager.dict()
        self._internal_checkpoint_counter = CHECKPOINT_ID_MAX + 1

        self._state_events = self._manager.dict()

        self._enable_synchronous_mode = enable_synchronous_mode
        self.logger = logging.getLogger(__name__)

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
        robot_socket.setblocking(True)
        remainder = ''
        while True:
            # Wait for a message from the robot.
            try:
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
                id_end = response.find(']')
                id = int(response[id_start:id_end])

                # Find next square brackets (contains data).
                data_start = response.find('[', id_end) + 1
                data_end = response.find(']', id_end + 1)

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
    def _command_response_handler(rx_queue, robot_state, pending_checkpoints, main_lock):
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
            if response == TERMINATE_PROCESS:
                return

            with main_lock:
                # Handle checkpoints.
                if response.id == 3030:
                    checkpoint_id = int(response.data)
                    if checkpoint_id in pending_checkpoints and pending_checkpoints[checkpoint_id]:
                        pending_checkpoints[checkpoint_id].pop(0).set()
                        # If list corresponding to checkpoint id is empty, remove the key from the dict.
                        if not pending_checkpoints[checkpoint_id]:
                            pending_checkpoints.pop(checkpoint_id)
                    else:
                        raise ValueError(
                            'Received un-tracked checkpoint. Please use ExpectExternalCheckpoint() to track.')

    @staticmethod
    def _string_to_floats(input_string):
        return [float(x) for x in input_string.split(',')]

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
            if response == TERMINATE_PROCESS:
                return

            with main_lock:
                if response.id == 2026:
                    robot_state.joint_positions.get_obj()[:] = Robot._string_to_floats(response.data)

                if response.id == 2027:
                    robot_state.end_effector_pose.get_obj()[:] = Robot._string_to_floats(response.data)

                if response.id == 2007:
                    status_flags = [bool(int(x)) for x in response.data.split(',')]
                    robot_state.activation_state.value = status_flags[0]
                    robot_state.homing_state.value = status_flags[1]
                    robot_state.simulation_mode.value = status_flags[2]
                    robot_state.error_status.value = status_flags[3]
                    robot_state.pause_motion_status = status_flags[4]
                    robot_state.end_of_block_status = status_flags[5]
                    robot_state.end_of_movement_status = status_flags[6]

                if response.id == 2200:
                    robot_state.nc_joint_positions.get_obj()[:] = Robot._string_to_floats(response.data)
                if response.id == 2201:
                    robot_state.nc_end_effector_pose.get_obj()[:] = Robot._string_to_floats(response.data)
                if response.id == 2202:
                    robot_state.nc_joint_velocity.get_obj()[:] = Robot._string_to_floats(response.data)
                if response.id == 2204:
                    robot_state.nc_end_effector_velocity.get_obj()[:] = Robot._string_to_floats(response.data)

                if response.id == 2208:
                    robot_state.nc_joint_configurations.get_obj()[:] = Robot._string_to_floats(response.data)
                if response.id == 2209:
                    robot_state.nc_multiturn.get_obj()[:] = Robot._string_to_floats(response.data)

                if response.id == 2210:
                    robot_state.drive_joint_positions.get_obj()[:] = Robot._string_to_floats(response.data)
                if response.id == 2211:
                    robot_state.drive_end_effector_pose.get_obj()[:] = Robot._string_to_floats(response.data)
                if response.id == 2212:
                    robot_state.drive_joint_velocity.get_obj()[:] = Robot._string_to_floats(response.data)
                if response.id == 2213:
                    robot_state.drive_joint_torque_ratio.get_obj()[:] = Robot._string_to_floats(response.data)
                if response.id == 2214:
                    robot_state.drive_end_effector_velocity.get_obj()[:] = Robot._string_to_floats(response.data)

                if response.id == 2218:
                    robot_state.drive_joint_configurations.get_obj()[:] = Robot._string_to_floats(response.data)
                if response.id == 2219:
                    robot_state.drive_multiturn.get_obj()[:] = Robot._string_to_floats(response.data)

                if response.id == 2220:
                    robot_state.accelerometer.get_obj()[:] = Robot._string_to_floats(response.data)

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

    def _check_monitor_processes(self):
        if self._command_response_handler_process:
            if not self._command_response_handler_process.is_alive():
                self.logger.error('Command response handler process has unexpectedly terminated. Disconnecting...')
                self.Disconnect()
                raise InvalidStateError
        if self._monitor_handler_process:
            if not self._monitor_handler_process.is_alive():
                self.logger.error('Monitor handler process has unexpectedly terminated. Disconnecting...')
                self.Disconnect()
                raise InvalidStateError

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
        self._command_tx_queue.put(command)

    def _establish_socket_connections(self, offline_mode=False):
        if offline_mode:
            return True

        if self._command_socket is not None:
            self.logger.warning('Existing command connection found, invalid state.')
            raise InvalidStateError

        if self._monitor_socket is not None:
            self.logger.warning('Existing monitor connection found, this should not be possible, closing socket...')
            raise InvalidStateError

        try:
            self._command_socket = self._connect_socket(self.logger, self._address, COMMAND_PORT)

            if self._command_socket is None:
                self.logger.error('Error creating socket.')
                raise CommunicationError

            self._monitor_socket = self._connect_socket(self.logger, self._address, MONITOR_PORT)

            if self._monitor_socket is None:
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
            self._command_rx_process = mp.Process(target=self._handle_socket_rx,
                                                  args=(
                                                      self._command_socket,
                                                      self._command_rx_queue,
                                                  ))
            self._command_rx_process.start()

            # Create rx process for command socket communication.
            self._command_tx_process = mp.Process(target=self._handle_socket_tx,
                                                  args=(
                                                      self._command_socket,
                                                      self._command_tx_queue,
                                                  ))
            self._command_tx_process.start()

            # Create rx processes for monitor socket communication.
            self._monitor_rx_process = mp.Process(target=self._handle_socket_rx,
                                                  args=(
                                                      self._monitor_socket,
                                                      self._monitor_rx_queue,
                                                  ))
            self._monitor_rx_process.start()

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
            response = self._command_rx_queue.get(block=True, timeout=10)  # 10s timeout.
        except Exception as e:
            self.logger.error('No response received within timeout interval. ' + str(e))
            self.Disconnect()
            return False

        # Check that response is appropriate.
        if response.id != 3000:
            self.logger.error('Connection error: %s', response)
            self.Disconnect()
            return False

        self._command_response_handler_process = mp.Process(target=self._command_response_handler,
                                                            args=(self._command_rx_queue, self._robot_state,
                                                                  self._pending_checkpoints, self._main_lock))
        self._command_response_handler_process.start()

        return True

    def _initialize_monitoring_connection(self):
        """Attempt to connect to the monitor port of the Mecademic Robot.

        Returns
        -------
        status : boolean
            Returns the status of the connection, true for success, false for failure.

        """

        self._monitor_handler_process = mp.Process(target=self._monitor_handler,
                                                   args=(self._monitor_rx_queue, self._robot_state, self._main_lock))
        self._monitor_handler_process.start()

        return True

    def Connect(self, offline_mode=False):
        """Attempt to connect to a physical Mecademic Robot.

        Returns
        -------
        status : boolean
            Returns the status of the connection, true for success, false for failure.

        """
        with self._main_lock:
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
        with self._main_lock:
            self._check_monitor_processes()
            self._send_command('ActivateRobot')

    def Home(self):
        with self._main_lock:
            self._check_monitor_processes()
            self._send_command('Home')

    def ActivateAndHome(self):
        with self._main_lock:
            self.ActivateRobot()
            self.Home()

    def DeactivateRobot(self):
        with self._main_lock:
            self._check_monitor_processes()
            self._send_command('DeactivateRobot')

    def Disconnect(self):
        """Disconnects Mecademic Robot object from physical Mecademic Robot.

        """
        # Don't use the normal DeactivateRobot call to avoid checking monitor processes.
        self._send_command('DeactivateRobot')

        # Join processes which wait on a queue by sending terminate to the queue.
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
                self._command_rx_process.join()
                self._command_rx_process = None

            if self._monitor_rx_process is not None:
                self._monitor_rx_process.join()
                self._monitor_rx_process = None

            # Reset communication queues (not strictly necessary since these should be empty).
            self._command_rx_queue = mp.Queue()
            self._command_tx_queue = mp.Queue()
            self._monitor_rx_queue = mp.Queue()

            # Reset robot state.
            self._robot_state = RobotState()
            self._pending_checkpoints = self._manager.dict()
            self._internal_checkpoint_counter = CHECKPOINT_ID_MAX + 1

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

    def GetJoints(self):
        with self._main_lock:
            self._check_monitor_processes()
            return self._robot_state.joint_positions.get_obj()[:]

    def GetEndEffectorPose(self):
        with self._main_lock:
            self._check_monitor_processes()
            return self._robot_state.end_effector_pose.get_obj()[:]

    def MoveJoints(self, joint1, joint2, joint3, joint4, joint5, joint6):
        with self._main_lock:
            self._check_monitor_processes()
            self._send_command('MoveJoints', [joint1, joint2, joint3, joint4, joint5, joint6])
            if self._enable_synchronous_mode:
                checkpoint = self._set_checkpoint_internal()

        if self._enable_synchronous_mode:
            checkpoint.wait()

    def SetCheckpoint(self, n):
        with self._main_lock:
            self._check_monitor_processes()
            assert CHECKPOINT_ID_MIN <= n <= CHECKPOINT_ID_MAX
            return self._set_checkpoint_impl(n)

    def ExpectExternalCheckpoint(self, n):
        with self._main_lock:
            self._check_monitor_processes()
            assert CHECKPOINT_ID_MIN <= n <= CHECKPOINT_ID_MAX
            return self._set_checkpoint_impl(n, send_to_robot=False)

    def _set_checkpoint_internal(self):
        with self._main_lock:
            checkpoint_id = self._internal_checkpoint_counter

            # Increment internal checkpoint counter.
            self._internal_checkpoint_counter += 1
            if self._internal_checkpoint_counter > CHECKPOINT_ID_MAX_PRIVATE:
                self._internal_checkpoint_counter.value = CHECKPOINT_ID_MAX + 1

            return self._set_checkpoint_impl(checkpoint_id)

    def _set_checkpoint_impl(self, n, send_to_robot=True):
        if not isinstance(n, int):
            raise TypeError('Please provide a string argument for the address.')

        assert CHECKPOINT_ID_MIN <= n <= CHECKPOINT_ID_MAX_PRIVATE

        self.logger.debug('Setting checkpoint %s', n)

        with self._main_lock:
            if n not in self._pending_checkpoints:
                self._pending_checkpoints[n] = self._manager.list()
            event = self._manager.Event()
            self._pending_checkpoints[n].append(event)

            if send_to_robot:
                self._send_command('SetCheckpoint', [n])

            return Checkpoint(n, event)
