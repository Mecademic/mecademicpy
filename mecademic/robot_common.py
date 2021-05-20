#!/usr/bin/env python3
import threading
import queue
import re

CHECKPOINT_ID_MAX_PRIVATE = 8191  # Max allowable checkpoint id, inclusive

TERMINATE = '--terminate--'

GRIPPER_OPEN = True
GRIPPER_CLOSE = False


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
        floats = string_to_floats(input_string)

        if (len(floats) - 1) != len(self.data):
            raise ValueError('Cannot update TimestampedData with incompatible data.')

        if floats[0] > self.timestamp:
            self.timestamp = floats[0]
            self.data = floats[1:]

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


class Message:
    """Class for storing a response message from a Mecademic robot.

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

        if self.model == 'Meca500':
            self.num_joints = 6
        elif self.mode == 'scara':
            self.num_joints = 4
        elif self.model == None:
            self.num_joints = 1
        else:
            raise ValueError('Invalid robot model: {}'.format(self.model))

    @classmethod
    def from_command_response_string(cls, input_string):
        """Generate robot state from standard robot response string.

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
                       is_virtual=(matches[2] != None),
                       fw_major_rev=int(matches[3]),
                       fw_minor_rev=int(matches[4]),
                       fw_patch_num=int(matches[5]))
        except:
            raise ValueError('Could not parse robot info string {}'.format(input_string))


class RobotState:
    """Class for storing the internal state of a generic Mecademic robot.

    Note that the recency and availability of the states which are not 'status fields' depends on the monitoring
    interval and which monitoring events are enabled. Monitoring events can be configured using SetMonitoringInterval()
    and SetRealTimeMonitoring().

    Attributes
    ----------
    target_joint_positions : TimestampedData
        Controller desired joint positions in degrees [theta_1...6], includes timestamp.
    target_end_effector_pose : TimestampedData
        Controller desired end effector pose [x, y, z, alpha, beta, gamma], includes timestamp.

    target_joint_velocity : TimestampedData
        Controller desired joint velocity in degrees/second [theta_dot_1...6], includes timestamp.
    target_end_effector_velocity : TimestampedData
        Controller desired end effector velocity with timestamp. Linear values in mm/s, angular in deg/s.
        [linear_velocity_vector x, y, z, angular_velocity_vector omega-x, omega-y, omega-z]
    target_joint_configurations : TimestampedData
        Controller joint configuration that corresponds to desired joint positions.
    target_last_joint_turn : TimestampedData
        Controller last joint turn number that corresponds to desired joint positions.

    drive_joint_positions : TimestampedData
        Drive-measured joint positions in degrees [theta_1...6], includes timestamp.
    drive_end_effector_pose : TimestampedData
        Drive-measured end effector pose [x, y, z, alpha, beta, gamma], includes timestamp.

    drive_joint_velocity : TimestampedData
        Drive-measured joint velocity in degrees/second [theta_dot_1...6], includes timestamp.
    drive_joint_torque_ratio : TimestampedData
        Drive-measured torque ratio as a percent of maximum [torque_1...6], includes timestamp.
    drive_end_effector_velocity : TimestampedData
        Drive-measured end effector velocity with timestamp. Linear values in mm/s, angular in deg/s.
        [linear_velocity_vector x, y, z, angular_velocity_vector omega-x, omega-y, omega-z]

    drive_joint_configurations : TimestampedData
        Controller joint configuration that corresponds to drives-measured joint positions.
    drive_last_joint_turn : TimestampedData
        Controller last joint turn number that corresponds to drives-measured joint positions.

    accelerometer : TimestampedData
        Raw accelerometer measurements [accelerometer_id, x, y, z]. 16000 = 1g.

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

    def __init__(self, num_joints):
        self.target_joint_positions = TimestampedData.zeros(num_joints)  # microseconds timestamp, degrees
        self.target_end_effector_pose = TimestampedData.zeros(6)  # microseconds timestamp, mm and degrees

        self.target_joint_velocity = TimestampedData.zeros(num_joints)  # microseconds timestamp, degrees/second
        self.target_end_effector_velocity = TimestampedData.zeros(6)  # microseconds timestamp, mm/s and deg/s

        self.target_joint_configurations = TimestampedData.zeros(3)
        self.target_last_joint_turn = TimestampedData.zeros(1)

        self.drive_joint_positions = TimestampedData.zeros(num_joints)  # microseconds timestamp, degrees
        self.drive_end_effector_pose = TimestampedData.zeros(6)  # microseconds timestamp, mm and degrees

        self.drive_joint_velocity = TimestampedData.zeros(num_joints)  # microseconds timestamp, degrees/second
        self.drive_joint_torque_ratio = TimestampedData.zeros(num_joints)  # microseconds timestamp, percent of maximum
        self.drive_end_effector_velocity = TimestampedData.zeros(6)  # microseconds timestamp, mm/s and deg/s

        self.drive_joint_configurations = TimestampedData.zeros(3)
        self.drive_last_joint_turn = TimestampedData.zeros(1)

        # Contains dictionary of accelerometers stored in the robot indexed by joint number.
        # For example, Meca500 currently only reports the accelerometer in joint 5.
        self.accelerometer = dict()  # 16000 = 1g

        self.max_queue_size = 0

        # The following are status fields.
        self.activation_state = False
        self.homing_state = False
        self.simulation_mode = False
        self.error_status = False
        self.pause_motion_status = False
        self.end_of_block_status = False


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


class CallbackQueue():
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
            if robot_callbacks.__dict__[attr] != None:
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
        if callback_name in self._registered_callbacks or callback_name == TERMINATE:
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


def string_to_floats(input_string):
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
