from __future__ import annotations

import json
import re
import threading

import mecademicpy.tools as tools

from .mx_robot_def import *

GRIPPER_OPEN = True
GRIPPER_CLOSE = False

DEFAULT_WAIT_TIMEOUT = 10

# Available levels for SetTorqueLimitsCfg
TORQUE_LIMIT_SEVERITIES = {'disabled': 0, 'warning': 1, 'pause-motion': 2, 'clear-motion': 3, 'error': 4}


#####################################################################################
# Private utility classes and methods.
#####################################################################################
class Message:
    """Class for storing a response message from a Mecademic robot.

    Attributes
    ----------
    id : integer
        The id of the message, representing the type of message.
    data : string
        The raw payload of the message.
    jsonData : JSON data parsed as a dictionary. None if message is not JSON format.
                Robot JSON message format is:
                {
                   MX_JSON_KEY_CODE:int,
                   MX_JSON_KEY_META_DATA:{
                       MX_JSON_KEY_MSG_TYPE:int
                   },
                   MX_JSON_KEY_DATA:{
                       (per-code JSON arguments/values)
                   }
"""

    def __init__(self, id: int, data: str, jsonData: dict | None = None):
        self.id = id
        self.data = data
        self.jsonData = jsonData

    def __repr__(self):
        return f"Message with id={self.id}, data={self.data}"

    @classmethod
    def from_string(cls, input: str):
        """Construct message object from raw string input.

        Parameters
        ----------
        input : string
            Input string to convert to message.

        """
        jsonData = {}
        if input[0] == '{':
            # JSON format
            jsonData = json.loads(input)
            # Extract id from JSON payload
            id = jsonData[MX_JSON_KEY_CODE]
            # Keep whole unparsed JSON string as raw data
            data = input
        else:
            # Legacy format
            id_start = input.find('[') + 1
            id_end = input.find(']', id_start)
            id = int(input[id_start:id_end])

            # Find next square brackets (contains data).
            data_start = input.find('[', id_end) + 1
            data_end = input.find(']', data_start)

            data = ''
            if data_start != -1 and data_end != -1:
                data = input[data_start:data_end]

        return cls(id, data, jsonData)


#####################################################################################
# Public utility classes related to class Robot.
#####################################################################################


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


class TimeoutException(MecademicException):
    """Requested timeout during a blocking operation (synchronous mode or Wait* functions) has been reached.
       (raised by InterruptableEvent)"""
    pass


class RobotCallbacks:
    """Class for storing possible status events for the Mecademic robot.

    Attributes
    ----------
        on_connected : function object
            Function to be called once connected.
        on_disconnected : function object
            Function to be called once disconnected.
        on_status_updated : function object
            Function to be called once robot status is updated.
        on_status_gripper_updated : function object
            Function to be called once gripper status is updated (legacy, use following external tools callbacks).
        on_external_tool_status_updated: function object
            Function to be called once external tool status is updated.
        on_gripper_state_updated: function object
            Function to be called once gripper state is updated.
        on_valve_state_updated: function object
            Function to be called once valve state is updated.
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
        on_pstop2 : function object
            Function to be called if PStop2 is activated.
        on_pstop2_resettable : function object
            Function to be called immediately after on_pstop2, only if the PStop2 condition can be reset
            (i.e. the power supply PStop2 signal is no more asserted)
        on_pstop2_reset : function object
            Function to be called if PStop2 is reset.
        on_estop : function object
            Function to be called if EStop is activated.
        on_estop_reset : function object
            Function to be called if EStop is reset.
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
        on_activate_ext_tool_sim : function object
            Function to be called once gripper sim mode is activated.
        on_deactivate_ext_tool_sim : function object
            Function to be called once gripper sim mode is deactivated.
        on_activate_recovery_mode : function object
            Function to be called once recovery mode is activated.
        on_deactivate_recovery_mode : function object
            Function to be called once recovery mode is deactivated.
        on_command_message : function object
            Function to be called each time a command response is received.
        on_monitor_message : function object
            Function to be called each time a monitor response is received.
        on_offline_program_state : function object
            Function to be called each time an offline program starts or fails to start.
        on_end_of_cycle : function object
            Function to be called each time end of cycle is reached."""

    def __init__(self):
        self.on_connected = None
        self.on_disconnected = None

        self.on_status_updated = None
        self.on_status_gripper_updated = None

        self.on_external_tool_status_updated = None
        self.on_gripper_state_updated = None
        self.on_valve_state_updated = None

        self.on_activated = None
        self.on_deactivated = None

        self.on_homed = None

        self.on_error = None
        self.on_error_reset = None
        self.on_pstop2 = None
        self.on_pstop2_resettable = None
        self.on_pstop2_reset = None
        self.on_estop = None
        self.on_estop_reset = None

        self.on_motion_paused = None
        self.on_motion_cleared = None
        self.on_motion_resumed = None

        self.on_checkpoint_reached = None

        self.on_activate_sim = None
        self.on_deactivate_sim = None

        self.on_activate_ext_tool_sim = None
        self.on_deactivate_ext_tool_sim = None

        self.on_activate_recovery_mode = None
        self.on_deactivate_recovery_mode = None

        self.on_command_message = None
        self.on_monitor_message = None

        self.on_offline_program_state = None

        self.on_end_of_cycle = None


class InterruptableEvent:
    """Extend default event class to also be able to unblock and raise an exception in case the event becomes
       irrelevant and will not occur (for example: Waiting on a checkpoint when robot is in error)

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
    _interrupted_msg : string
        User message that explains the reason of interruption.
    _abort_on_error : bool
        Tells if this event must be awakened if the robot falls into error state, by default False
    _abort_on_clear_motion : bool
        Tells if this event must be awakened if the robot's motion queue is cleared
        Note that this also includes PStop2 condition and robot deactivation (which also cause the motion queue
        to be cleared)
"""

    def __init__(self, id=None, data=None, abort_on_error=False, abort_on_clear_motion=False):
        self._id = id
        self._data = data
        self._event = threading.Event()
        self._lock = threading.Lock()
        self._interrupted = False
        self._interrupted_msg = ""
        self._abort_on_error = abort_on_error
        self._abort_on_clear_motion = abort_on_clear_motion

    def check_interrupted(self):
        if self._interrupted:
            if self._interrupted_msg != "":
                raise InterruptException(self._interrupted_msg)
            else:
                raise InterruptException('Event interrupted, possibly because event will never be triggered.')

    def wait(self, timeout: float = None) -> Message:
        """Block until event is set or should raise an exception (InterruptException or TimeoutException).
           InterruptException is raised if waiting for the event has become irrelevant, like waiting for
           a checkpoint while robot is in error.

        Attributes
        ----------
        timeout : float
            Maximum duration to wait in seconds.

        Return
        ------
        data : object
            Return the data object (or None for events not returning any data)

        """
        with self._lock:
            self.check_interrupted()
        wait_result = self._event.wait(timeout=timeout)
        with self._lock:
            self.check_interrupted()
            if not wait_result:
                raise TimeoutException()
            return self._data

    def set(self, data: Message = None):
        """Set the event and unblock all waits. Optionally modify data before setting.

        """
        with self._lock:
            self._data = data
            self._event.set()

    def abort(self, message=""):
        """Unblock any waits and raise an exception.

        """
        with self._lock:
            if not self._event.is_set():
                self._interrupted_msg = message
                self._interrupted = True
                self._event.set()  # Awake all threads
                self._event.clear()  # Restore previous state (important to keep state even if interrupted)

    def clear(self):
        """Reset the event to its initial state.

        """
        with self._lock:
            self._interrupted = False
            self._event.clear()

    def is_set(self) -> bool:
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
        """Clears the abort to make waiting for the event blocking again.

        """
        with self._lock:
            if self._interrupted:
                self._interrupted = False

    @property
    def id(self) -> int:
        """Make id a read-only property since it should not be changed after instantiation.

        """
        return self._id

    @property
    def data(self) -> Message:
        """Make data a read-only property and enforce that it is only assignable at construction or using set().

        """
        return self._data


class RobotVersion:
    """
        Robot utility class to handle firmware version.

    Attributes
    ----------

    build : integer
        Firmware build number, None if unavailable

    extra : string
        Firmware version 'extra' name, None if unavailable

    full_version : string
        Full firmware version containing major.minor.patch.build-extra

    major : integer
        Major firmware version value

    minor : integer
        Minor firmware version value

    patch : integer
        Patch firmware version value

    short_version : string
        Firmware version containing major.minor.patch only

    """

    REGEX_VERSION_BUILD = r"(?P<version>\d+\.\d+\.\d+)\.?(?P<build>\d+)?-?(?P<extra>.*)"

    def __init__(self, version: str):
        """Creates

        :param version: version of firmware. See update_version for supported formats
        """
        self.full_version = version
        self.update_version(self.full_version)

    def __str__(self) -> str:
        return self.full_version

    def update_version(self, version: str):
        """Update object firmware version values by parsing a version string.

        :param version: string
            New version of firmware. Supports multiple version formats
            ie. 8.1.9, 8.4.3.1805-official
        """
        self.short_version = '0.0.0'

        regex_version = re.search(self.REGEX_VERSION_BUILD, version)
        if regex_version is None:
            raise ValueError(f'Invalid version format: "{version}"')
        if regex_version.group("version"):
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

    def is_at_least(self, major, minor=0, patch=0) -> bool:
        """Tells if this RobotInfo instance's version is at least the specified version

        Parameters
        ----------
        major : integer
            Minimum desired major version
        minor : integer
            Minimum desired minor version
        patch : integer
            Minimum desired patch version

        Returns
        -------
        boolean
            True if this RobotInfo instance's version is at least the specified version
        """
        # Check major
        if self.major > major:
            return True
        elif self.major < major:
            return False

        # Same major, check minor
        if self.minor > minor:
            return True
        elif self.minor < minor:
            return False

        # Same minor, check patch
        if self.patch >= patch:
            return True

        return False


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
    version : str
        robot firmware revision number as received from the connection string.
    serial : string
        Serial identifier of robot.
    ip_address : string
        IP address of this robot.
    rt_message_capable : bool
        True if robot is capable of sending real-time monitoring messages.
    rt_on_ctrl_port_capable : bool
        True if robot is capable of sending real-time monitoring messages on control port (SetCtrlPortMonitoring).
    num_joints : int
        Number of joints on the robot.
    requires_homing : bool
        Tells if this robot requires homing.
    supports_ext_tool : bool
        Tells if this robot supports connecting external tools (gripper or valve box).
    gripper_pos_ctrl_capable : bool
        Tells if this robot supports gripper position control.
    ext_tool_version_capable : bool
        Tells if this robot supports external tool fw version fetch.
    ext_tool_version : str
        External tool firmware revision number as received from the connection string.
        Version 0.0.0.0 if device isn't connected or ext_tool_version_capable == False.
"""

    def __init__(self,
                 model: str = 'Unknown',
                 revision: int = 0,
                 is_virtual: bool = False,
                 version: str = '0.0.0',
                 serial: str = '',
                 ext_tool_version: str = '0.0.0.0'):
        self.model = model
        self.revision = revision
        self.is_virtual = is_virtual
        self.version = RobotVersion(version)
        self.serial = serial
        self.ip_address = None  # Set later
        self.rt_message_capable = False
        self.rt_on_ctrl_port_capable = False
        self.gripper_pos_ctrl_capable = False
        self.ext_tool_version_capable = False
        self.ext_tool_version = RobotVersion(ext_tool_version)

        if self.model == 'Meca500':
            self.num_joints = 6
            self.requires_homing = True
            self.supports_ext_tool = True
        elif self.model == 'Scara':
            self.num_joints = 4
            self.requires_homing = False
            self.supports_ext_tool = False
        elif self.model == 'Unknown':
            self.num_joints = 1
            self.requires_homing = False
            self.supports_ext_tool = False
        else:
            raise ValueError(f'Invalid robot model: {self.model}')

        # Check if this robot supports real-time monitoring events
        if self.version.is_at_least(8, 4):
            self.rt_message_capable = True
        # Check if this robot supports real-time monitoring on control port
        if self.version.is_at_least(9, 0):
            self.rt_on_ctrl_port_capable = True
        # Check if this robot supports gripper position control
        if self.version.is_at_least(9, 1):
            self.gripper_pos_ctrl_capable = True
        if self.version.is_at_least(9, 1, 5):
            # Check if this robot supports external tool version
            self.ext_tool_version_capable = True

    def __str__(self):
        return f"Connected to {self.serial} ip:{self.ip_address}, {self.model} R{self.revision} v{self.version}"

    def __repr__(self):
        return str(self)

    @classmethod
    def from_command_response_string(cls, input_string: str):
        """Generate robot information from standard robot connection response string.

        String format should be "Connected to {model} R{revision}{-virtual} v{fw_major_num}.{fw_minor_num}.{patch_num}"

        Parameters
        ----------
        input_string : string
            Input string to be parsed.

        """
        ROBOT_CONNECTION_STRING = r"Connected to (?P<model>[\w|-]+) ?R?(?P<revision>\d)?"
        ROBOT_CONNECTION_STRING += r"(?P<virtual>-virtual)?( v|_)?(?P<version>\d+\.\d+\.\d+)"

        virtual = False

        try:
            robot_info_regex = re.search(ROBOT_CONNECTION_STRING, input_string)
            if robot_info_regex is None:
                raise ValueError(f'Could not parse robot info string "{input_string}"')
            if robot_info_regex.group('model'):
                model = robot_info_regex.group('model')
            if robot_info_regex.group('revision'):
                revision = int(robot_info_regex.group('revision'))
            if robot_info_regex.group('virtual'):
                virtual = True
            return cls(model=model, revision=revision, is_virtual=virtual, version=robot_info_regex.group('version'))
        except Exception as exception:
            raise ValueError(f'Could not parse robot info string "{input_string}", error: {exception}')

    def get_serial_digit(self) -> int:
        """Returns robot serial digits.
           ie. M500-0123 -> 123

        Returns
        -------
        int
            Returns robot serial digits
        """
        serial_split = self.serial.split('-')

        if len(serial_split) != 2:
            raise ValueError(f'Invalid serial number string received: {self.serial}, expecting "M500-1234"')

        serial_digit = int(serial_split[1])
        return serial_digit


class UpdateProgress:
    """ Class containing firmware update progress information

    Attributes
    ----------
    complete : bool
        Firmware update process state
    progress : string
        Update progress message received from robot
"""

    def __init__(self) -> None:
        self.complete: bool = False
        self.progress: str = ''


class TimestampedData:
    """ Class for storing timestamped data (real-time data received from the robot)

    Attributes
    ----------
    timestamp : number-like
        Monotonic timestamp associated with data (in microseconds since robot last reboot)
        This timestamp is stamped by the robot so it is not affected by host/network jitter
    data : object
        Data to be stored.
"""

    def __init__(self, timestamp: int, data: list[float]):
        self.timestamp = timestamp
        self.data = data
        self.enabled = False

    def __str__(self):
        return str([self.timestamp] + self.data)

    def __repr__(self):
        return str(self)

    def clear_if_disabled(self):
        """Clear timestamp and data if not reported by the robot (not part of enabled real-time monitoring events)
        """
        if not self.enabled:
            self.timestamp = 0
            self.data = TimestampedData.zeros(len(self.data)).data

    def update_from_csv(self, input_string: str):
        """Update from comma-separated string, only if timestamp is newer.

        Parameters
        ----------
        input_string : string
            Comma-separated string. First value is timestamp, rest is data.

        """
        numbs = tools.string_to_numbers(input_string)

        if (len(numbs) - 1) != len(self.data):
            raise ValueError('Cannot update TimestampedData with incompatible data.')

        if numbs[0] >= self.timestamp:
            self.timestamp = numbs[0]
            self.data = numbs[1:]

    def update_from_data(self, timestamp: int, data: list[float]):
        """Update with data unless timestamp is older

        Parameters
        ----------
        timestamp : number-like
            Timestamp associated with data.
        data : object
            Data to be stored if timestamp is newer.

        """
        if timestamp >= self.timestamp:
            self.timestamp = timestamp
            self.data = data

    @classmethod
    def zeros(cls, length: int):
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


class RobotRtData:
    """Class for storing the internal real-time data of a Mecademic robot.

    Most real-time data query methods from the programming guide were not implemented explicitly in Python.
    The information, however, are available using the GetRobotRtData() method.
    The attribute corresponding programming method is shown between parentheses.

    The frequency and availability of real-time data depends on the monitoring interval and which monitoring
    events are enabled. Monitoring events can be configured using SetMonitoringInterval() and SetRealTimeMonitoring().

    Attributes
    ----------
    rt_target_joint_pos : TimestampedData
        Controller desired joint positions in degrees [theta_1...6]. (GetRtTargetJointPos)
    rt_target_cart_pos : TimestampedData
        Controller desired end effector pose [x, y, z, alpha, beta, gamma]. (GetRtTargetCartPos)
    rt_target_joint_vel : TimestampedData
        Controller desired joint velocity in degrees/second [theta_dot_1...6]. (GetRtTargetJointVel)
    rt_target_cart_vel : TimestampedData
        Controller desired end effector velocity with timestamp. Linear values in mm/s, angular in deg/s.
        [linear_velocity_vector x, y, z, angular_velocity_vector omega-x, omega-y, omega-z]. (GetRtTargetCartVel)
    rt_target_joint_torq : TimestampedData
        Controller desired torque ratio as a percent of maximum [torque_1...6]. (GetRtJointTorq)
    rt_target_conf : TimestampedData
        Controller joint configuration that corresponds to desired joint positions. (GetRtTargetConf)
    rt_target_conf_turn : TimestampedData
        Controller last joint turn number that corresponds to desired joint positions. (GetRtTargetConfTurn)

    rt_joint_pos : TimestampedData
        Drive-measured joint positions in degrees [theta_1...6]. (GetRtJointPos)
    rt_cart_pos : TimestampedData
        Drive-measured end effector pose [x, y, z, alpha, beta, gamma]. (GetRtCartPos)
    rt_joint_vel : TimestampedData
        Drive-measured joint velocity in degrees/second [theta_dot_1...6]. (GetRtJointVel)
    rt_joint_torq : TimestampedData
        Drive-measured torque ratio as a percent of maximum [torque_1...6]. (GetRtJointTorq)
    rt_cart_vel : TimestampedData
        Drive-measured end effector velocity with timestamp. Linear values in mm/s, angular in deg/s.
        [linear_velocity_vector x, y, z, angular_velocity_vector omega-x, omega-y, omega-z]. (GetRtCartVel)
    rt_conf : TimestampedData
        Controller joint configuration that corresponds to drives-measured joint positions. (GetRtConf)
    rt_conf_turn : TimestampedData
        Controller last joint turn number that corresponds to drives-measured joint positions. (GetRtConfTurn)

    rt_accelerometer : TimestampedData
        Raw accelerometer measurements [accelerometer_id, x, y, z]. 16000 = 1g. (GetRtAccelerometer)

    rt_external_tool_status : TimestampedData
        External tool status [sim_tool_type, physical_tool_type, homing_state, error_status, overload_error].
        (GetRtExtToolStatus)
    rt_valve_state : TimestampedData
        Valve state [valve_opened[0], valve_opened[1]]. (GetRtValveState)
    rt_gripper_state : TimestampedData
        Gripper state [holding_part, target_pos_reached, opened, closed]. (GetRtGripperState)
    rt_gripper_force : TimestampedData
        Gripper force in % of maximum force. (GetRtGripperForce)
    rt_gripper_pos : TimestampedData. (GetRtValveState)
        Gripper position in mm. (GetRtGripperPos)

    rt_wrf : TimestampedData
        Current definition of the WRF w.r.t. the BRF with timestamp. Cartesian data are in mm, Euler angles in degrees.
        [cartesian coordinates x, y, z, Euler angles omega-x, omega-y, omega-z] (GetRtWrf)
    rt_trf : TimestampedData
        Current definition of the TRF w.r.t. the FRF with timestamp. cartesian data are in mm, Euler angles in degrees.
        [cartesian coordinates x, y, z, Euler angles omega-x, omega-y, omega-z] (GetRtTrf)
    rt_checkpoint : TimestampedData
        Last executed checkpoint with timestamp. (GetCheckpoint)
"""

    def __init__(self, num_joints: int):
        self.rt_target_joint_pos = TimestampedData.zeros(num_joints)  # microseconds timestamp, degrees
        self.rt_target_cart_pos = TimestampedData.zeros(6)  # microseconds timestamp, mm and degrees
        self.rt_target_joint_vel = TimestampedData.zeros(num_joints)  # microseconds timestamp, degrees/second
        self.rt_target_cart_vel = TimestampedData.zeros(6)  # microseconds timestamp, mm/s and deg/s
        self.rt_target_joint_torq = TimestampedData.zeros(num_joints)  # microseconds timestamp, percent of maximum
        self.rt_target_conf = TimestampedData.zeros(3)
        self.rt_target_conf_turn = TimestampedData.zeros(1)

        self.rt_joint_pos = TimestampedData.zeros(num_joints)  # microseconds timestamp, degrees
        self.rt_cart_pos = TimestampedData.zeros(6)  # microseconds timestamp, mm and degrees
        self.rt_joint_vel = TimestampedData.zeros(num_joints)  # microseconds timestamp, degrees/second
        self.rt_joint_torq = TimestampedData.zeros(num_joints)  # microseconds timestamp, percent of maximum
        self.rt_cart_vel = TimestampedData.zeros(6)  # microseconds timestamp, mm/s and deg/s
        self.rt_conf = TimestampedData.zeros(3)
        self.rt_conf_turn = TimestampedData.zeros(1)

        # Another way of getting robot joint position using less-precise encoders.
        # For robot production testing (otherwise use rt_joint_pos which is much more precise)
        self.rt_abs_joint_pos = TimestampedData.zeros(num_joints)  # microseconds timestamp, degrees

        # Contains dictionary of accelerometers stored in the robot indexed by joint number.
        # For example, Meca500 currently only reports the accelerometer in joint 5.
        self.rt_accelerometer: dict[int, TimestampedData] = dict()  # 16000 = 1g

        self.rt_external_tool_status = TimestampedData.zeros(
            5)  # microseconds timestamp, sim tool type, physical tool type, activated, homed, error
        self.rt_valve_state = TimestampedData.zeros(
            MX_EXT_TOOL_MPM500_NB_VALVES)  # microseconds timestamp, valve1 opened, valve2 opened
        self.rt_gripper_state = TimestampedData.zeros(
            4)  # microseconds timestamp, holding part, target pos reached, closed, opened
        self.rt_gripper_force = TimestampedData.zeros(1)  # microseconds timestamp, gripper force [%]
        self.rt_gripper_pos = TimestampedData.zeros(1)  # microseconds timestamp, gripper position [mm]

        self.rt_wrf = TimestampedData.zeros(6)  # microseconds timestamp, mm and degrees
        self.rt_trf = TimestampedData.zeros(6)  # microseconds timestamp, mm and degrees
        self.rt_checkpoint = TimestampedData.zeros(1)  # microseconds timestamp, checkpointId

        self.max_queue_size = 0

    def _for_each_rt_data(self):
        """Iterates for each TimestampedData type member of this class (rt_joint_pos, rt_cart_pos, etc.)
        """
        # Collect class member names
        member_names = vars(self)
        # Iterate though all "rt_" members
        for member_name in member_names:
            if not member_name.startswith('rt_'):
                continue
            member = getattr(self, member_name)
            # Check member type (TimestampedData or dict of TimestampedData), then yield TimestampedData
            if isinstance(member, TimestampedData):
                yield member
            elif isinstance(member, dict):
                for sub_member in member.values():
                    yield sub_member

    def _reset_enabled(self):
        """Clear the "enabled" flag of each member of this class of type TimestampedData
        """
        for rt_data in self._for_each_rt_data():
            rt_data.enabled = False

    def _clear_if_disabled(self):
        """Clear real-time values that are disabled (not reported by robot's current real-time monitoring configuration)
        """
        for rt_data in self._for_each_rt_data():
            rt_data.clear_if_disabled()


class RobotStatus:
    """Class for storing the status of a Mecademic robot.

    Attributes
    ----------
    activation_state : bool
        True if the robot is activated.
    homing_state : bool
        True if the robot is homed.
    simulation_mode : bool
        True if the robot is in simulation-only mode.
    recovery_mode : bool
        True if the robot is in recovery mode.
    error_status : bool
        True if the robot is in error.
    pstop2State : MxStopState
        Current PStop2 status
    estopState : MxStopState
        Current EStop status.
        Note that Meca500 revision 3 or older never report this condition because these robots's power supply
        will be shutdown completely in case of EStop (and thus this API will get disconnected from the robot instead).
    pause_motion_status : bool
        True if motion is currently paused.
    end_of_block_status : bool
        True if robot is not moving and motion queue is empty.
        Note: We recommend not using end_of_block_status to detect when robot has finished executing previously sent
              commands. Some posted commands may still be transient (on the network for example) and the robot may,
              in some cases, declare end-of-block in-between two commands.
              Instead, we recommend using checkpoints or the WaitIdle method.
    brakes_engaged : bool
        True if robot brakes are engaged.
        This is relevant only when robot is deactivated (brakes are automatically disengaged upon robot activation).
"""

    def __init__(self):

        # The following are status fields.
        self.activation_state = False
        self.homing_state = False
        self.simulation_mode = False
        self.recovery_mode = False
        self.error_status = False
        self.pstop2State = MxStopState.MX_STOP_STATE_RESET
        self.estopState = MxStopState.MX_STOP_STATE_RESET
        self.pause_motion_status = False
        self.end_of_block_status = False
        self.brakes_engaged = False


class GripperStatus:
    """Class for storing the Mecademic robot's gripper status.
       LEGACY, use ExtToolStatus and GripperState instead.

    Attributes
    ----------
    present : bool
        True if the gripper is present on the robot.
    homing_state : bool
        True if the gripper has been homed (ready to be used).
    holding_part : bool
        True if the gripper is currently holding a part.
    target_pos_reached : bool
        True if the gripper is at target position or at a limit (fully opened or closed).
    error_status : bool
        True if the gripper is in error state.
    overload_error : bool
        True if the gripper is in overload error state.
"""

    def __init__(self):

        # The following are status fields.
        self.present = False
        self.homing_state = False
        self.holding_part = False
        self.target_pos_reached = False
        self.error_status = False
        self.overload_error = False


class ExtToolStatus:
    """Class for storing the Mecademic robot's external tool status.

    Attributes
    ----------
    sim_tool_type : int
        Simulated tool type.
         0: MxExtToolType.MX_EXT_TOOL_NONE
        10: MxExtToolType.MX_EXT_TOOL_MEGP25_SHORT
        11: MxExtToolType.MX_EXT_TOOL_MEGP25_LONG
        20: MxExtToolType.MX_EXT_TOOL_VBOX_2VALVES
    physical_tool_type : int
        Physical external tool type (same values as mentioned above).
    homing_state : bool
        True if the gripper is homed.
    error_status : bool
        True if the gripper is in error state.
    overload_error : bool
        True if the gripper is in overload error state.
"""

    def __init__(self):

        # The following are status fields.
        self.sim_tool_type = MxExtToolType.MX_EXT_TOOL_NONE
        self.physical_tool_type = MxExtToolType.MX_EXT_TOOL_NONE
        self.homing_state = False
        self.error_status = False
        self.overload_error = False

    def __str__(self) -> str:
        return f"Sim tool type: {self.sim_tool_type}, Physical tool type: {self.physical_tool_type}, " \
               f"homed: {self.homing_state}, error: {self.error_status}, overload: {self.overload_error}"

    def __repr__(self) -> str:
        return str(self)

    def current_tool_type(self) -> int:
        """Returns current external tool type (simulated or physical)

        Returns
        -------
        int
            Current external tool
        """
        return self.sim_tool_type if self.sim_tool_type != MxExtToolType.MX_EXT_TOOL_NONE else self.physical_tool_type

    def is_physical_tool_present(self) -> bool:
        """Returns if physical tool is connected

        Returns
        -------
        bool
            True if physical gripper is connected, False otherwise
        """
        return self.physical_tool_type != MxExtToolType.MX_EXT_TOOL_NONE

    def is_tool_sim(self) -> bool:
        """Returns if tool is simulated or not

        Returns
        -------
        bool
            True if tool is simulated, False otherwise
        """
        return self.sim_tool_type != MxExtToolType.MX_EXT_TOOL_NONE

    def is_gripper(self, physical: bool = False) -> bool:
        """Returns if current external tool (simulated or physical) is a gripper

        Parameters
        ----------
        physical : bool
            True check physical gripper, False use current one (simulated or physical)

        Returns
        -------
        bool
            True if tool is a gripper, False otherwise
        """
        tool_type = self.physical_tool_type if physical else self.current_tool_type()
        return tool_type in [MxExtToolType.MX_EXT_TOOL_MEGP25_SHORT, MxExtToolType.MX_EXT_TOOL_MEGP25_LONG]

    def is_pneumatic_module(self, physical: bool = False) -> bool:
        """Returns if current external tool (simulated or physical) is a pneumatic module

        Parameters
        ----------
        physical : bool
            True check physical gripper, False use current one (simulated or physical)

        Returns
        -------
        bool
            True if tool is a pneumatic module, False otherwise
        """
        tool_type = self.physical_tool_type if physical else self.current_tool_type()
        return tool_type in [MxExtToolType.MX_EXT_TOOL_VBOX_2VALVES]


class ValveState:
    """Class for storing the Mecademic robot's pneumatic module valve states.

    Attributes
    ----------
    valve_opened : list[int]
        List of valve state: MX_VALVE_STATE_CLOSE or MX_VALVE_STATE_OPENED
"""

    def __init__(self):

        # The following are status fields.
        self.valve_opened = [int] * MX_EXT_TOOL_VBOX_MAX_VALVES


class GripperState:
    """Class for storing the Mecademic robot's gripper state.

    Attributes
    ----------
    holding_part : bool
        True if the gripper is currently holding a part.
    target_pos_reached : bool
        True if the gripper is at requested position:
          - At configured opened/closed position following GripperOpen/GripperClose.
          - At requested position after MoveGripper.
    closed : bool
        True if the gripper is at the configured 'close' position (ref SetGripperRanger) or less.
    opened : bool
        True if the gripper is at the configured 'open' position (ref SetGripperRanger) or more.

"""

    def __init__(self):

        # The following are status fields.
        self.holding_part = False
        self.target_pos_reached = False
        self.closed = False
        self.opened = False

    def __str__(self):
        return f'holding={self.holding_part} pos_reached={self.target_pos_reached} ' \
               f'closed={self.closed} opened={self.opened}'

    def __repr__(self) -> str:
        return str(self)
