"""
This file define classes required to use the mecademicpy modules and control Mecademic robots.
Some of these classes are used in the python API (Message, Callbacks, Exceptions, InterruptableEvent, ...)
Some of these classes define robot information and data, such as RobotVersion, RobotInfo, UpdateProgress, RobotRtData,
RobotStatus, RobotSafetyStatus, GripperStatus, IoStatus, CollisionStatus, etc.
"""
from __future__ import annotations

import json
import re
import threading
import time
from typing import Callable, Optional, Union

import mecademicpy.tools as tools

# pylint: disable=wildcard-import,unused-wildcard-import
from .mx_robot_def import *

GRIPPER_OPEN = True
GRIPPER_CLOSE = False

DEFAULT_WAIT_TIMEOUT = 10

# Available levels for SetTorqueLimitsCfg
TORQUE_LIMIT_SEVERITIES = {'disabled': 0, 'warning': 1, 'pause-motion': 2, 'clear-motion': 3, 'error': 4}


# Temporary location until consensus reached
class RtDataUpdateType(IntEnum):
    MX_RT_DATA_UPDATE_TYPE_UNAVAILABLE = 0  # Data is not available
    MX_RT_DATA_UPDATE_TYPE_CYCLICAL = 1  # Data is sent every cycle
    MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL = 2  # Data sent every cycle when configured through SetRealTimeMonitoring
    MX_RT_DATA_UPDATE_TYPE_EVENT_BASED = 3  # Data is sent upon connection and updated when its values change


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
    json_data : JSON data parsed as a dictionary. None if message is not JSON format.
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

    # pylint: disable=redefined-builtin
    def __init__(self, id: int, data: str, json_data: dict | None = None):
        self.id = id
        self.data = data
        self.json_data = json_data

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
        id = 0
        data = ""
        json_data = {}
        if len(input) == 0:
            pass
        elif input[0] == '{':
            # JSON format
            json_data = json.loads(input)
            # Extract id from JSON payload
            id = json_data[MX_JSON_KEY_CODE]
            # Keep whole unparsed JSON string as raw data
            data = input
        else:
            # Legacy format
            id_start = input.find('[') + 1
            id_end = input.find(']', id_start)
            id = int(input[id_start:id_end])

            # Find next square brackets (contains data).
            data_start = input.find('[', id_end) + 1
            data_end = input.rfind(']')

            data = ''
            if data_start != -1 and data_end != -1:
                data = input[data_start:data_end]

        return cls(id, data, json_data)


#####################################################################################
# Public utility classes related to class Robot.
#####################################################################################


class MecademicException(Exception):
    """Base exception class for Mecademic-related exceptions.
"""

    def __init__(self, message: str):
        """ Initialize this base exception

        Args:
            message (str):
                User message to print
        """
        self.message = message
        super().__init__(self.message)

    pass


class MecademicFatalException(MecademicException):
    """Class for all mecademic exceptions that should be considered fatal (stack trace should be printed).
"""
    pass


class MecademicNonFatalException(MecademicException):
    """Class for all mecademic exceptions that should be considered non fatal (can be catch, error printed, and
       application may continue).
"""
    pass


class InvalidStateError(MecademicNonFatalException):
    """The internal state of the instance is invalid.
"""
    pass


class InvalidConfigError(MecademicNonFatalException):
    """Invalid config is used.
    """
    pass


class CommunicationError(MecademicNonFatalException):
    """There is a communication issue with the robot.
"""
    pass


class DisconnectError(MecademicNonFatalException):
    """A non-nominal disconnection has occurred.
"""
    pass


class InterruptException(MecademicNonFatalException):
    """An event has encountered an error. Perhaps it will never be set.
"""
    pass


class TimeoutException(MecademicNonFatalException):
    """Requested timeout during a blocking operation (synchronous mode or Wait* functions) has been reached.
       (raised by InterruptableEvent)"""
    pass


class NotFoundException(MecademicNonFatalException):
    """A method, variable or object was not found"""
    pass


class ArgErrorException(MecademicNonFatalException):
    """An argument was invalid, the operation could not be completed"""
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

        on_output_state_updated: function object
            Function to be called when digital outputs changed.
        on_input_state_updated: function object
            Function to be called when digital inputs changed.
        on_vacuum_state_updated: function object
            Function to be called once vacuum state is updated.

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

        on_safety_stop : function object
            Function to be called when the robot enters safety stop state following a raised safety stop conditions
            (EStop, PStop1, PStop2, etc.).
            Note that on_safety_stop_state_change can be used to be notified of more safety stop conditions that change
            while the robot is already in safety stop.
        on_safety_stop_reset : function object
            Function to be called when all safety stop conditions (EStop, PStop1, PStop2, ...) are cleared (reset)
        on_safety_stop_resettable : function object
            Function to be called when all safety stop conditions (EStop, PStop1, PStop2, ...) can be reset
            i.e. the safety stop conditions are no more present and need a reset (Power supply reset button in some
            cases, or simply ResumeMotion in other cases)
        on_safety_stop_state_change : function object
            Function to be called when any safety stop state change (see RobotSafetyStatus)

        on_pstop2 : function object
            Function to be called if PStop2 is activated.
            *** DEPRECATED (replaced by on_safety_stop_state)
        on_pstop2_resettable : function object
            Function to be called when PStop2 condition can be reset
            (i.e. the power supply PStop2 signal is no more asserted)
            *** DEPRECATED (replaced by on_safety_stop_state)
        on_pstop2_reset : function object
            Function to be called if PStop2 is reset.
            *** DEPRECATED (replaced by on_safety_stop_state)
        on_estop : function object
            Function to be called if EStop is activated.
            *** DEPRECATED (replaced by on_safety_stop_state)
        on_estop_reset : function object
            Function to be called if EStop is reset.
            *** DEPRECATED (replaced by on_safety_stop_state)
        on_estop_resettable : function object
            Function to be called when EStop condition can be reset
            (i.e. the power supply EStop signal is no more asserted)
            *** DEPRECATED (replaced by on_safety_stop_state)

        on_motion_paused : function object
            Function to be called once motion is paused.
        on_motion_cleared : function object
            Function to be called once motion is cleared.
        on_motion_resumed : function object
            Function to be called once motion is resumed.
        on_checkpoint_reached : function object
            Function to be called if a checkpoint is reached.
        on_checkpoint_discarded : function object
            Function to be called if a checkpoint is discarded
            (due to motion cleared, robot deactivated, robot in error, safety stop, etc.).

        on_activate_sim : function object
            Function to be called once sim mode is activated.
        on_deactivate_sim : function object
            Function to be called once sim mode is deactivated.
        on_activate_ext_tool_sim : function object
            Function to be called once gripper sim mode is activated.
        on_deactivate_ext_tool_sim : function object
            Function to be called once gripper sim mode is deactivated.
        on_io_sim_enabled : function object
            Function to be called once IO simulation mode is enabled.
        on_io_sim_disabled : function object
            Function to be called once IO simulation mode is disabled.

        on_activate_recovery_mode : function object
            Function to be called once recovery mode is activated.
        on_deactivate_recovery_mode : function object
            Function to be called once recovery mode is deactivated.

        on_command_message : function object
            Function to be called each time a command response is received.
        on_monitor_message : function object
            Function to be called each time an event is received on the monitoring port.
            Only available when connected to the robot in monitoring mode.
            Note that on_monitor_message may not be very useful. We suggest to use on_end_of_cycle instead
            (which works for both monitoring or control mode connections).

        on_offline_program_state : function object
            Function to be called each time an offline program starts or fails to start.

        on_end_of_cycle : function object
            Function to be called each time end of cycle is reached.
            It's called once all real-time data for current monitoring interval has been received.
            At this moment, all robot real-time data is coherent (belongs to the same cycle).
            """

    def __init__(self):
        self.on_connected: Callable[[], None] = None
        self.on_disconnected: Callable[[], None] = None

        self.on_status_updated: Callable[[], None] = None
        self.on_status_gripper_updated: Callable[[], None] = None

        self.on_external_tool_status_updated: Callable[[], None] = None
        self.on_gripper_state_updated: Callable[[], None] = None
        self.on_valve_state_updated: Callable[[], None] = None
        self.on_output_state_updated: Callable[[], None] = None
        self.on_input_state_updated: Callable[[], None] = None
        self.on_vacuum_state_updated: Callable[[], None] = None

        self.on_activated: Callable[[], None] = None
        self.on_deactivated: Callable[[], None] = None

        self.on_homed: Callable[[], None] = None

        self.on_error: Callable[[], None] = None
        self.on_error_reset: Callable[[], None] = None
        self.on_safety_stop_resettable: Callable[[], None] = None
        self.on_safety_stop_reset: Callable[[], None] = None
        self.on_safety_stop_state_change: Callable[[], None] = None

        self.on_pstop2: Callable[[], None] = None
        self.on_pstop2_resettable: Callable[[], None] = None
        self.on_pstop2_reset: Callable[[], None] = None
        self.on_estop: Callable[[], None] = None
        self.on_estop_resettable: Callable[[], None] = None
        self.on_estop_reset: Callable[[], None] = None

        self.on_motion_paused: Callable[[], None] = None
        self.on_motion_cleared: Callable[[], None] = None
        self.on_motion_resumed: Callable[[], None] = None

        self.on_checkpoint_reached: Callable[[int], None] = None
        self.on_checkpoint_discarded: Callable[[int], None] = None

        self.on_activate_sim: Callable[[], None] = None
        self.on_deactivate_sim: Callable[[], None] = None

        self.on_activate_ext_tool_sim: Callable[[], None] = None
        self.on_deactivate_ext_tool_sim: Callable[[], None] = None
        self.on_io_sim_enabled: Callable[[], None] = None
        self.on_io_sim_disabled: Callable[[], None] = None

        self.on_activate_recovery_mode: Callable[[], None] = None
        self.on_deactivate_recovery_mode: Callable[[], None] = None

        self.on_command_message: Callable[[Message], None] = None
        self.on_monitor_message: Callable[[Message], None] = None

        self.on_offline_program_state: Callable[[], None] = None

        self.on_end_of_cycle: Callable[[], None] = None


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

    # pylint: disable=redefined-builtin
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
        start_time = time.monotonic()
        # Wait by smaller chunks to we can get interrupted by OS signals (like SIGINT)
        partial_timeout = 0.1 if timeout is None else min(timeout, 0.1)
        while True:
            is_set = self._event.wait(timeout=partial_timeout)
            if is_set:
                break
            self.check_interrupted()
            if timeout is not None and time.monotonic() - start_time > timeout:
                raise TimeoutException("Timeout waiting for interruptable event")
        with self._lock:
            self.check_interrupted()
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

    def __lt__(self, other: RobotVersion) -> bool:
        """" Less than implementation
        """
        return not self.is_at_least(other.major, other.minor, other.patch, other.build)

    def get_str(self, build=False, extra=False) -> str:
        """Get version string

        Parameters
        ----------
        build : bool, optional
            Include the build number in the version (ex:9.3.0.4739), by default False
        extra : bool, optional
            Include the build 'extra in the version (ex:9.3.0.4739-master), by default False

        Returns
        -------
        str
            Formatted version string
        """
        version_to_return = self.short_version
        if build and self.build:
            version_to_return += f'.{self.build}'
        if extra and self.extra:
            version_to_return += f'-{self.extra}'
        return version_to_return

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

        self.full_version = self.get_str(build=True, extra=True)

    def is_at_least(self, major, minor=0, patch=0, build=0) -> bool:
        """Tells if this RobotInfo instance's version is at least the specified version

        Parameters
        ----------
        major : integer
            Minimum desired major version
        minor : integer
            Minimum desired minor version
        patch : integer
            Minimum desired patch version
        build : integer
            Minimum desired build version

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
        if self.patch > patch:
            return True
        elif self.patch < patch:
            return False

        if build == 0 or build is None:
            # Don't need to check build
            return True

        # Same patch, check build
        if self.build is not None:
            if self.build >= build:
                return True
        else:
            # Build is not known, we can't make sure we're ok
            return False

        return False


class RobotInfo:
    """Class for storing metadata about a robot.

    Attributes
    ----------
    robot_model: MxRobotModel
        Model of robot
    model : string
        Model of robot (legacy, use robot_model instead since it's an enum with well-known values)
    revision : int
        Robot revision.
    is_virtual : bool
        True if is a virtual robot.
    is_safe_boot : bool
        True if is booted in safe-boot mode.
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
    sidecar_capable : bool
        True if robot supports running the sidecar scripting engine.
    num_joints : int
        Number of joints on the robot.
    requires_homing : bool
        Tells if this robot requires homing.
    supports_ext_tool : bool
        Tells if this robot supports connecting external tools (gripper or valve box).
    supports_io_module : bool
        Tells if this robot supports IO expansion module.
    supports_manual_mode : bool
        Tells if this robot supports manual mode.
    gripper_pos_ctrl_capable : bool
        Tells if this robot supports gripper position control.
    ext_tool_version_capable : bool
        Tells if this robot supports external tool fw version fetch.
    ext_tool_version : str
        External tool firmware revision number as received from the connection string.
        Version 0.0.0.0 if device isn't connected or ext_tool_version_capable == False.
    supports_joint_vel_limit : bool
        Tells if this robot supports SetJointVelLimit
    supports_set_payload : bool
        Tells if this robot supports SetPayload
    supports_torque_limits : bool
        Tells if this robot supports SetTorqueLimits and SetTorqueLimitsCfg
    supports_conf_turn : bool
        Tells if this robot supports SetConfTurn
    supports_time_scaling : bool
        Tells if this robot supports SetTimeScaling
    supports_checkpoint_discarded : bool
        Tells if this robot supports reporting discarded checkpoints (MX_ST_CHECKPOINT_DISCARDED)
    supports_move_duration : bool
        Tells if this robot supports time-based movements (SetMoveDuration, SetMoveMode, ...)
"""

    def __init__(self,
                 model: str = 'Unknown',
                 revision: int = 0,
                 is_virtual: bool = False,
                 is_safe_boot: bool = False,
                 version: str = '0.0.0',
                 serial: str = '',
                 ext_tool_version: str = '0.0.0.0'):
        self.robot_model = MxRobotModel.MX_ROBOT_MODEL_UNKNOWN
        self.model = model
        self.revision = revision
        self.is_virtual = is_virtual
        self.is_safe_boot = is_safe_boot
        self.version = RobotVersion(version)
        self.serial = serial
        self.ip_address = None  # Set later
        self.rt_message_capable = False
        self.rt_on_ctrl_port_capable = False
        self.sidecar_capable = False
        self.gripper_pos_ctrl_capable = False
        self.ext_tool_version_capable = False
        self.ext_tool_version = RobotVersion(ext_tool_version)
        self.supports_io_module = False
        self.supports_manual_mode = False
        self.supports_joint_vel_limit = False
        self.supports_set_payload = False
        self.supports_torque_limits = False
        self.supports_conf_turn = False
        self.supports_time_scaling = False
        self.supports_checkpoint_discarded = False
        self.supports_move_duration = False

        if self.model.upper() == MX_ROBOT_MODEL_OFFICIAL_NAME_M500.upper():
            if self.revision == 1:
                self.robot_model = MxRobotModel.MX_ROBOT_MODEL_M500_R1
            elif self.revision == 2:
                self.robot_model = MxRobotModel.MX_ROBOT_MODEL_M500_R2
            elif self.revision == 3:
                self.robot_model = MxRobotModel.MX_ROBOT_MODEL_M500_R3
            elif self.revision == 4:
                self.robot_model = MxRobotModel.MX_ROBOT_MODEL_M500_R4
            self.num_joints = 6
            self.requires_homing = True
            self.supports_ext_tool = True
            self.supports_io_module = False
        elif self.model.upper() == MX_ROBOT_MODEL_OFFICIAL_NAME_MCS500.upper():
            self.robot_model = MxRobotModel.MX_ROBOT_MODEL_MCS500_R1
            self.num_joints = 4
            self.requires_homing = False
            self.supports_ext_tool = False
            self.supports_io_module = True
            self.supports_manual_mode = True
        elif self.model.upper() == MX_ROBOT_MODEL_OFFICIAL_NAME_MCA250.upper():
            self.robot_model = MxRobotModel.MX_ROBOT_MODEL_MCA250_R1
            self.num_joints = 6
            self.requires_homing = False
            self.supports_ext_tool = False
            self.supports_io_module = True
            self.supports_manual_mode = True
        elif self.model.upper() == MX_ROBOT_MODEL_OFFICIAL_NAME_MCA1000.upper():
            self.robot_model = MxRobotModel.MX_ROBOT_MODEL_MCA1000_R1
            self.num_joints = 6
            self.requires_homing = False
            self.supports_ext_tool = False
            self.supports_io_module = True
            self.supports_manual_mode = True
        elif self.model.upper() == 'UNKNOWN':
            self.robot_model = MxRobotModel.MX_ROBOT_MODEL_UNKNOWN
            self.num_joints = 1
            self.requires_homing = False
            self.supports_ext_tool = False
            self.supports_ext_tool = False
        else:
            self.robot_model = MxRobotModel.MX_ROBOT_MODEL_UNKNOWN
            raise ValueError(f'Invalid robot model: {self.model}')

        # Check if this robot supports real-time monitoring events
        if self.version.is_at_least(8, 4):
            self.rt_message_capable = True
        # Check if this robot supports real-time monitoring on control port
        if self.version.is_at_least(9, 0):
            self.rt_on_ctrl_port_capable = True
        # Check if this robot supports the sidecar scripting engine
        if self.version.is_at_least(11, 1, 3):
            self.sidecar_capable = True
        # Check if this robot supports gripper position control
        if self.version.is_at_least(9, 1):
            self.gripper_pos_ctrl_capable = True
        if self.version.is_at_least(9, 1, 5):
            # Check if this robot supports external tool version
            self.ext_tool_version_capable = True
        if self.version.is_at_least(9, 3):
            self.supports_joint_vel_limit = True
            self.supports_set_payload = True
            self.supports_torque_limits = True
            self.supports_conf_turn = True
        if self.version.is_at_least(10, 0, 1):
            self.supports_time_scaling = True
        if self.version.is_at_least(10, 2, 1):
            self.supports_checkpoint_discarded = True
        if self.version.is_at_least(11, 1, 2):
            self.supports_move_duration = True

    def __str__(self):
        safe_boot_str = " SAFE-BOOT" if self.is_safe_boot else ""
        return (f"Connected to {self.ip_address}: "
                f"{self.model} R{self.revision} {self.serial} v{self.version}{safe_boot_str}")

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
        connection_string_regex = r"Connected to (?P<model>[\w|-]+) ?R?(?P<revision>\d)?"
        connection_string_regex += r"(?P<virtual>-virtual)?(?P<safe_boot>-safe-boot)?( v|_)?(?P<version>\d+\.\d+\.\d+)"

        model: str = "Unknown"
        revision = 0
        virtual = False
        safe_boot = False

        try:
            robot_info_regex = re.search(connection_string_regex, input_string)
            if robot_info_regex is None:
                raise ValueError(f'Could not parse robot info string "{input_string}"')
            if robot_info_regex.group('model'):
                model = robot_info_regex.group('model')
            if robot_info_regex.group('revision'):
                revision = int(robot_info_regex.group('revision'))
            if robot_info_regex.group('virtual'):
                virtual = True
            if robot_info_regex.group('safe_boot'):
                safe_boot = True
            return cls(model=model,
                       revision=revision,
                       is_virtual=virtual,
                       is_safe_boot=safe_boot,
                       version=robot_info_regex.group('version'))
        except Exception as exception:
            raise ValueError(f'Could not parse robot info string "{input_string}", error: {exception}') from exception

    def get_serial_digit(self) -> int:
        """Returns robot serial digits.
           ie. M500-0123 -> 123 (for Meca500)
            or
           ie. 0123 -> 123 (for other)

        Returns
        -------
        int
            Returns robot serial digits
        """
        if self.robot_model == MxRobotModel.MX_ROBOT_MODEL_M500_R4:
            serial_digit = self._get_meca500_serial_digits()
        elif self.robot_model == MxRobotModel.MX_ROBOT_MODEL_M500_R3:
            serial_digit = self._get_meca500_serial_digits()
        elif self.robot_model == MxRobotModel.MX_ROBOT_MODEL_M500_R2:
            serial_digit = self._get_meca500_serial_digits()
        else:
            # This should cover mcs500 and all new future hardwares
            serial_digit = int(self.serial)
        return serial_digit

    def _get_meca500_serial_digits(self) -> int:
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

        return int(serial_split[1])


class UpdateProgress:
    """Class for storing the robot's firmware update status.

    Attributes
    ----------
    in_progress : bool
        Firmware update is in progress
    complete : bool
        Firmware update has completed
    version : str
        The firmware version being installed
    error : boolean
        Tells if the update failed
    error_msg : str
        Message that explains why the update failed
    progress : str
        Update progress message received from robot
    step : str
        String that describes the current firmware update step being performed

    _last_print_timestamp : float
        Last time we have printed the firmware update status
"""

    def __init__(self):
        # The following are status fields.
        self.in_progress = False
        self.complete = False
        self.version = ""
        self.error = False
        self.error_msg = ""
        self.progress = 0.0
        self.progress_str = ""  # For legacy update
        self.step = ""

        self._last_print_timestamp = 0.0


class TimestampedData:
    """ Class for storing timestamped data (real-time data received from the robot)

    Attributes
    ----------
    timestamp : number-like
        Monotonic timestamp associated with data (in microseconds since robot last reboot)
        This timestamp is stamped by the robot so it is not affected by host/network jitter
    data : object
        Data to be stored.
    update_type : RtDataUpdateType
        Update type of the TimestampedData. Refer to RtDataUpdateType for the various update types.
"""

    def __init__(self, timestamp: int, data: list[float], update_type: RtDataUpdateType):
        self.timestamp = timestamp
        self.data = data
        self.update_type = update_type
        self.enabled = False if (update_type == RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL
                                 or update_type == RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_UNAVAILABLE) else True

    def __str__(self):
        return str([self.timestamp] + self.data)

    def __repr__(self):
        return str(self)

    def clear_if_disabled(self):
        """Clear timestamp and data if not reported by the robot (not part of enabled real-time monitoring events)
        """
        if not self.enabled:
            self.timestamp = 0
            self.data = TimestampedData.zeros(len(self.data), self.update_type).data

    def update_from_csv(self, input_string: str, allowed_nb_val: list[int] = None):
        """Update from comma-separated string, only if timestamp is newer.

        Parameters
        ----------
        input_string : string
            Comma-separated string. First value is timestamp, rest is data.
        allowed_nb_val : list[int]
            Optional list of accepted number of values. If not provided, input_string must contain at least as many
            values as current contents of self.data.

        """
        numbs = tools.string_to_numbers(input_string)
        nb_values = len(numbs) - 1

        if allowed_nb_val is None:
            if (nb_values) < len(self.data):
                raise ValueError(f'Cannot update TimestampedData, too few values received ({nb_values}).')
            elif (nb_values) > len(self.data):
                numbs = numbs[0:len(self.data) + 1]
        else:
            if nb_values not in allowed_nb_val:
                raise ValueError(f'Cannot update TimestampedData, incorrect number of values received ({nb_values}).')

        if numbs[0] >= self.timestamp:
            self.timestamp = numbs[0]
            self.data = numbs[1:]
            self.enabled = True

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
            self.enabled = True

    @classmethod
    def zeros(cls, length: int, update_type: RtDataUpdateType):
        """ Construct empty TimestampedData object of specified length.

        Parameters
        ----------
        length : int
            Length of data to construct.
        update_type : RtDataUpdateType
            Update type of monitored data. Refer to RtDataUpdateType for the various update types

        Return
        ------
        TimestampedData object

        """
        return cls(0, [0.] * length, update_type)

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
        return other.timestamp == self.timestamp and other.data == self.data and other.update_type == self.update_type

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
    cycle_count : int
        Number of real-time data updates received from the robot. The robot will send real-time data updates at
        the interval defined by SetMonitoringInterval (16.6 milliseconds by default)
    rt_target_joint_pos : TimestampedData
        Controller desired joint positions in degrees [theta_1...6] (GetRtTargetJointPos).
        Always enabled and available (regardless of SetRealTimeMonitoring options).
    rt_target_cart_pos : TimestampedData
        Controller desired end effector pose [x, y, z, alpha, beta, gamma] (GetRtTargetCartPos).
        Always enabled and available (regardless of SetRealTimeMonitoring options).
    rt_target_joint_vel : TimestampedData
        Controller desired joint velocity in degrees/second [theta_dot_1...6] (GetRtTargetJointVel).
        *** Not enabled by default. To enable it, use SetRealTimeMonitoring(MX_ST_RT_TARGET_JOINT_VEL).
    rt_target_cart_vel : TimestampedData
        Controller desired end effector velocity with timestamp. Linear values in mm/s, angular in deg/s.
        [linear_velocity_vector x, y, z, angular_velocity_vector omega-x, omega-y, omega-z] (GetRtTargetCartVel).
        *** Not enabled by default. To enable it, use SetRealTimeMonitoring(MX_ST_RT_TARGET_CART_VEL).
    rt_target_joint_torq : TimestampedData
        Controller estimated torque ratio as a percent of maximum [torque_1...6] (GetRtJointTorq).
        *** Not enabled by default. To enable it, use SetRealTimeMonitoring(MX_ST_RT_TARGET_JOINT_TORQ).
    rt_target_conf : TimestampedData
        Controller joint configuration that corresponds to desired joint positions (GetRtTargetConf).
        Always enabled and available (regardless of SetRealTimeMonitoring options).
    rt_target_conf_turn : TimestampedData
        Controller last joint turn number that corresponds to desired joint positions (GetRtTargetConfTurn).
        Always enabled and available (regardless of SetRealTimeMonitoring options).

    rt_joint_pos : TimestampedData
        Drive-measured joint positions in degrees [theta_1...6] (GetRtJointPos).
        *** Not enabled by default. To enable it, use SetRealTimeMonitoring(MX_ST_RT_JOINT_POS).
    rt_cart_pos : TimestampedData
        Drive-measured end effector pose [x, y, z, alpha, beta, gamma] (GetRtCartPos).
        *** Not enabled by default. To enable it, use SetRealTimeMonitoring(MX_ST_RT_CART_POS).
    rt_joint_vel : TimestampedData
        Drive-measured joint velocity in degrees/second [theta_dot_1...6] (GetRtJointVel).
        *** Not enabled by default. To enable it, use SetRealTimeMonitoring(MX_ST_RT_JOINT_VEL).
    rt_joint_torq : TimestampedData
        Drive-measured torque ratio as a percent of maximum [torque_1...6] (GetRtJointTorq).
        *** Not enabled by default. To enable it, use SetRealTimeMonitoring(MX_ST_RT_JOINT_TORQ).
    rt_cart_vel : TimestampedData
        Drive-measured end effector velocity with timestamp. Linear values in mm/s, angular in deg/s.
        [linear_velocity_vector x, y, z, angular_velocity_vector omega-x, omega-y, omega-z] (GetRtCartVel).
        *** Not enabled by default. To enable it, use SetRealTimeMonitoring(MX_ST_RT_CART_VEL).
    rt_conf : TimestampedData
        Controller joint configuration that corresponds to drives-measured joint positions (GetRtConf).
        *** Not enabled by default. To enable it, use SetRealTimeMonitoring(MX_ST_RT_CONF).
    rt_conf_turn : TimestampedData
        Controller last joint turn number that corresponds to drives-measured joint positions (GetRtConfTurn).
        *** Not enabled by default. To enable it, use SetRealTimeMonitoring(MX_ST_RT_CONF_TURN).

    rt_accelerometer : TimestampedData
        Raw accelerometer measurements [accelerometer_id, x, y, z]. 16000 = 1g (GetRtAccelerometer).
        *** Not enabled by default. To enable it, use SetRealTimeMonitoring(MX_ST_RT_ACCELEROMETER).
    rt_effective_time_scaling : TimestampedData
        Effective time scaling ratio (GetRtEffectiveTimeScaling).
        *** Not enabled by default. To enable it, use SetRealTimeMonitoring(MX_ST_RT_EFFECTIVE_TIME_SCALING).
    rt_vm : TimestampedData
        Motor voltage readings (GetRtVm).
        Contains: [Baseboard VM, Psu VM, SafeMcu VM, Drive 1 VM, Drive 1 VM, ..., Drive N VM]
        *** Not enabled by default. To enable it, use SetRealTimeMonitoring(MX_ST_RT_VM).
    rt_current : TimestampedData
        Motor current readings (GetRtIm).
        Contains: [Baseboard current]
        *** Not enabled by default. To enable it, use SetRealTimeMonitoring(MX_ST_RT_CURRENT).

    rt_external_tool_status : TimestampedData
        External tool status [sim_tool_type, physical_tool_type, homing_state, error_status, overload_error]
        (GetRtExtToolStatus).
        Always enabled and available (regardless of SetRealTimeMonitoring options).
    rt_valve_state : TimestampedData
        Valve state [valve_opened[0], valve_opened[1]] (GetRtValveState).
        Always enabled and available (regardless of SetRealTimeMonitoring options).
    rt_gripper_state : TimestampedData
        Gripper state [holding_part, target_pos_reached, opened, closed] (GetRtGripperState).
        Always enabled and available (regardless of SetRealTimeMonitoring options).
    rt_gripper_force : TimestampedData
        Gripper force in % of maximum force (GetRtGripperForce).
        *** Not enabled by default. To enable it, use SetRealTimeMonitoring(MX_ST_RT_GRIPPER_FORCE).
    rt_gripper_pos : TimestampedData
        Gripper position in mm. (GetRtGripperPos)
        *** Not enabled by default. To enable it, use SetRealTimeMonitoring(MX_ST_RT_GRIPPER_POS).

    rt_io_module_status : TimestampedData
        IO module status [bank_id, present, sim_mode, error_code] (GetRtIoStatus).
        Always enabled and available (regardless of SetRealTimeMonitoring options).
    rt_io_module_outputs : TimestampedData
        IO module's digital outputs state [output[0], output[1], ...] (GetRtOutputState).
        Always enabled and available (regardless of SetRealTimeMonitoring options).
    rt_io_module_inputs : TimestampedData
        IO module's digital inputs state [input[0], input[1], ...] (GetRtInputState).
        Always enabled and available (regardless of SetRealTimeMonitoring options).
    rt_vacuum_state : TimestampedData
        IO module's vacuum gripper state [vacuum_on, purge_on, holding_part] (GetRtVacuumState).
        Always enabled and available (regardless of SetRealTimeMonitoring options).
    rt_vacuum_pressure: TimestampedData
        IO module's vacuum current pressure in kPa (GetRtVacuumPressure).
        *** Not enabled by default. To enable it, use SetRealTimeMonitoring(MX_ST_RT_VACUUM_PRESSURE).

    rt_wrf : TimestampedData
        Current definition of the WRF w.r.t. the BRF with timestamp. Cartesian data are in mm, Euler angles in degrees.
        [cartesian coordinates x, y, z, Euler angles omega-x, omega-y, omega-z] (GetRtWrf)
        Always enabled and available (regardless of SetRealTimeMonitoring options).
    rt_trf : TimestampedData
        Current definition of the TRF w.r.t. the FRF with timestamp. cartesian data are in mm, Euler angles in degrees.
        [cartesian coordinates x, y, z, Euler angles omega-x, omega-y, omega-z] (GetRtTrf)
        Always enabled and available (regardless of SetRealTimeMonitoring options).
    rt_checkpoint : TimestampedData
        Last executed checkpoint with timestamp. (GetCheckpoint)
        Always enabled and available (regardless of SetRealTimeMonitoring options).
"""

    def __init__(self, num_joints: int):
        self._init_timestamped_data(num_joints)
        self.max_queue_size = 0
        self.cycle_count = 0

    def _init_timestamped_data(self, num_joints: int):
        """Initialize timestamped data class members according to detected robot model (number of joints)"""
        nb_cart_val = num_joints  # 4 degrees of liberty for 4 joints robots, 6 for 6 joint robots
        nb_conf_val = 3 if num_joints == 6 else 1  # 4 degrees of liberty for 4 joints robots, 6 for 6 joint robots

        self.rt_target_joint_pos = TimestampedData.zeros(
            num_joints, RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_CYCLICAL)  # microseconds timestamp, degrees
        self.rt_target_cart_pos = TimestampedData.zeros(
            nb_cart_val, RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_CYCLICAL)  # microseconds timestamp, mm and degrees
        self.rt_target_joint_vel = TimestampedData.zeros(
            num_joints,
            RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL)  # microseconds timestamp, degrees/second
        self.rt_target_cart_vel = TimestampedData.zeros(
            nb_cart_val,
            RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL)  # microseconds timestamp, mm/s and deg/s
        self.rt_target_joint_torq = TimestampedData.zeros(
            num_joints,
            RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL)  # microseconds timestamp, percent of maximum
        self.rt_target_conf = TimestampedData.zeros(nb_conf_val, RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_EVENT_BASED)
        self.rt_target_conf_turn = TimestampedData.zeros(1, RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_EVENT_BASED)

        self.rt_joint_pos = TimestampedData.zeros(
            num_joints, RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL)  # microseconds timestamp, degrees
        self.rt_cart_pos = TimestampedData.zeros(
            nb_cart_val,
            RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL)  # microseconds timestamp, mm and degrees
        self.rt_joint_vel = TimestampedData.zeros(
            num_joints,
            RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL)  # microseconds timestamp, degrees/second
        self.rt_joint_torq = TimestampedData.zeros(
            num_joints,
            RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL)  # microseconds timestamp, percent of maximum
        self.rt_cart_vel = TimestampedData.zeros(
            nb_cart_val,
            RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL)  # microseconds timestamp, mm/s and deg/s
        self.rt_conf = TimestampedData.zeros(nb_conf_val, RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_EVENT_BASED)
        self.rt_conf_turn = TimestampedData.zeros(1, RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_EVENT_BASED)

        self.rt_effective_time_scaling = TimestampedData.zeros(1,
                                                               RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL
                                                               )  # microseconds timestamp, effective time scaling ratio
        self.rt_vm = TimestampedData.zeros(
            num_joints + 3, RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL
        )  # microseconds timestamp, Baseboard VM, Psu VM, SafeMcu Vm, Drive 1 VM, Drive 2 Vm, ...
        self.rt_current = TimestampedData.zeros(
            1, RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL)  # microseconds timestamp, Baseboard current

        # Another way of getting robot joint position using less-precise encoders.
        # For robot production testing (otherwise use rt_joint_pos which is much more precise)
        self.rt_abs_joint_pos = TimestampedData.zeros(
            num_joints, RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL)  # microseconds timestamp, degrees

        # Contains dictionary of accelerometers stored in the robot indexed by joint number.
        # For example, Meca500 currently only reports the accelerometer in joint 5.
        self.rt_accelerometer: dict[int, TimestampedData] = dict()  # 16000 = 1g

        self.rt_external_tool_status = TimestampedData.zeros(
            5, RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_EVENT_BASED
        )  # microseconds timestamp, sim tool type, physical tool type, homed, error, overload
        self.rt_valve_state = TimestampedData.zeros(
            MX_EXT_TOOL_MPM500_NB_VALVES,
            RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_EVENT_BASED)  # microseconds timestamp, valve1 opened, valve2 opened
        self.rt_gripper_state = TimestampedData.zeros(
            4, RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_EVENT_BASED
        )  # microseconds timestamp, holding part, target pos reached, closed, opened
        self.rt_gripper_force = TimestampedData.zeros(
            1, RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL)  # microseconds timestamp, gripper force [%]
        self.rt_gripper_pos = TimestampedData.zeros(
            1,
            RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL)  # microseconds timestamp, gripper position [mm]

        self.rt_io_module_status = TimestampedData.zeros(4, RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_EVENT_BASED)
        self.rt_io_module_outputs = TimestampedData.zeros(
            0, RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_EVENT_BASED)  # Resized later
        self.rt_io_module_inputs = TimestampedData.zeros(
            0, RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_EVENT_BASED)  # Resized later
        self.rt_vacuum_state = TimestampedData.zeros(3, RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_EVENT_BASED
                                                     )  # microseconds timestamp, vacuum on/off, purge on/off, holding
        self.rt_vacuum_pressure = TimestampedData.zeros(
            1,
            RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL)  # microseconds timestamp, vacuum pressure [kPa]

        self.rt_sig_gen_status = TimestampedData.zeros(4, RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_EVENT_BASED)
        self.rt_sig_gen_outputs = TimestampedData.zeros(
            0, RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_EVENT_BASED)  # Resized later
        self.rt_sig_gen_inputs = TimestampedData.zeros(
            0, RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_EVENT_BASED)  # Resized later

        self.rt_wrf = TimestampedData.zeros(
            nb_cart_val, RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_EVENT_BASED)  # microseconds timestamp, mm and degrees
        self.rt_trf = TimestampedData.zeros(
            nb_cart_val, RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_EVENT_BASED)  # microseconds timestamp, mm and degrees
        self.rt_checkpoint = TimestampedData.zeros(
            1, RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_EVENT_BASED)  # microseconds timestamp, checkpointId

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

        # manually remove the accelerometer data
        self.rt_accelerometer.clear()

    def _clear_accelerometer_data_if_disabled(self):
        """Clear the accelerometer data dictionary member
        """
        for accelerometer_idx in list(self.rt_accelerometer.keys()):
            if self.rt_accelerometer[accelerometer_idx].enabled is False:
                del self.rt_accelerometer[accelerometer_idx]

    def clear_if_outdated(self):
        """Clear the TimestampedData of each member if the data is outdated
        """
        # Clear any outdated pieces of data by using the enable flag
        reference_timestamp = self.rt_target_joint_pos.timestamp  # Present at every cycle

        for rt_data in self._for_each_rt_data():
            if (rt_data.timestamp != reference_timestamp
                    and rt_data.update_type != RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_EVENT_BASED):
                rt_data.enabled = False
                rt_data.clear_if_disabled()

        self._clear_accelerometer_data_if_disabled()

    def clear_if_disabled(self):
        """Clear real-time values that are disabled (not reported by robot's current real-time monitoring configuration)
        """
        for rt_data in self._for_each_rt_data():
            rt_data.clear_if_disabled()

        self._clear_accelerometer_data_if_disabled()


class RobotStatus:
    """Class for storing the status of a Mecademic robot.

    Attributes
    ----------
    activation_state : bool
        True if the robot is activated.
    homing_state : bool
        True if the robot is homed.
    simulation_mode : MxRobotSimulationMode
        True if the robot is in simulation-only mode (fast or real-time)
    recovery_mode : bool
        True if the robot is in recovery mode.
    error_status : bool
        True if the robot is in error.
    error_code : int
        Current robot error code, or 0 if the robot is not in error state.
        Note: Only supported if connecting to the on port MX_ROBOT_TCP_PORT_CONTROL_JSON or MX_ROBOT_TCP_PORT_FEED_JSON.
              Otherwise error_code will be None (unknown).
    pstop2State : MxStopState
        *** IMPORTANT NOTE: PStop2 is not safety-rated on Meca500 robots ***
        *** Deprecated. Use RobotSafetyStatus.pstop2_state instead.
        Current PStop2 status.
    estopState : MxStopState
        *** Deprecated. Use RobotSafetyStatus.estop_state instead.
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
    connection_watchdog_enabled : bool
        True if the connection watchdog is currently enabled/active (see ConnectionWatchdog API call)
"""

    def __init__(self):

        # The following are status fields.
        self.activation_state = False
        self.homing_state = False
        self.simulation_mode = MxRobotSimulationMode.MX_SIM_MODE_DISABLED
        self.recovery_mode = False
        self.error_status = False
        self.error_code: Optional[int] = None
        # pylint: disable=invalid-name
        self.pstop2State = MxStopState.MX_STOP_STATE_RESET  # Deprecated, moved to RobotSafetyStatus.pstop2_state
        self.estopState = MxStopState.MX_STOP_STATE_RESET  # Deprecated, moved to RobotSafetyStatus.estop_state
        self.pause_motion_status = False
        self.end_of_block_status = False
        self.brakes_engaged = False
        self.connection_watchdog_enabled = False

    def __str__(self) -> str:
        error_str = f'{self.error_status}' if (self.error_code is None
                                               or self.error_code == 0) else f'{self.error_code}'
        return (f"Activated: {self.activation_state}, "
                f"homed: {self.homing_state}, "
                f"sim: {self.simulation_mode}, "
                f"recovery mode: {self.recovery_mode}, "
                f"error: {error_str}, "
                f"pause motion: {str(self.pause_motion_status)}, "
                f"EOB: {self.end_of_block_status}, "
                f"brakes engaged: {self.brakes_engaged}, "
                f"connection watchdog: {'enabled' if self.connection_watchdog_enabled else 'disabled'}")

    def can_move(self) -> bool:
        """Tells if robot can currently be moved (state is homed, or activated in recovery mode)

        Returns:
            bool: true if robot can be moved
        """
        return self.homing_state or (self.activation_state and self.recovery_mode)


class RobotStaticSafetyStopMasks:
    """Various useful bit masks used to categorize safety signals

    Attributes
    ----------
    clearedByPsu : int
        Bit mask to identify safety signals that must be reset using the power supply reset function.
    withVmOff : int
        Bit mask to identify safety signals that cause motor voltage to be removed.
        These are category 1 safety stop signals (Estop, PStop1, etc.).
    maskedInManualMode : int
        Bit mask to identify safety signals masked when the robot is in "manual" operation mode (PStop1, PStop2)

    """

    def __init__(self):
        # Useful masks to categorize RobotSafetyStatus.stop_mask above
        # *** Note: Only available when connected on the JSON port (MX_ROBOT_TCP_PORT_CONTROL_JSON)
        # pylint: disable=invalid-name
        self.clearedByPsu = 0
        self.withVmOff = 0
        self.maskedInManualMode = 0


class RobotPowerSupplyInputs:
    """ Class for storing robot's power supply physical input states

        Attributes
        ----------
        psu_input_mask : int
            Bit mask that summarizes all power supply input states. Use bits from MxPsuInputMask
        estop_asserted : bool
            Indicate if the EStop (emergency stop) signal on the power supply is asserted (robot will stop)
        pstop1_asserted : bool
            Indicate if the PStop1 (protective stop category 1) signal on the power supply is asserted (robot will stop)
        pstop2_asserted : bool
            Indicate if the PStop2 (protective stop category 2) signal on the power supply is asserted (robot will stop)
        reset_ext_asserted : bool
            Indicate if the 'reset' input signal on the power supply is asserted (requesting for a reset)
        reset_keypad_pressed : bool
            Indicate if the 'reset' keypad button on the power supply is pressed
        enabling_device_asserted : bool
            Indicate if the enabling device power supply input indicates that the enabling device is pressed
    """

    def __init__(self):
        self.psu_input_mask = 0
        self.estop_asserted = False
        self.pstop1_asserted = False
        self.pstop2_asserted = False
        self.reset_ext_asserted = False
        self.reset_keypad_pressed = False
        self.enabling_device_asserted = False

    def set_psu_input_mask(self, input_mask: Union[MxPsuInputMask, int]):
        """Update the power supply input mask, and update corresponding individual booleans (estop_asserted, etc.)

        Parameters
        ----------
        input_mask : Union[MxPsuInputMask,int]
            Power supply input mask to set
        """
        self.psu_input_mask = int(input_mask)

        # Update individual booleans
        self.estop_asserted = (self.psu_input_mask & MxPsuInputMask.MX_PSU_INPUT_ESTOP) != 0
        self.pstop1_asserted = (self.psu_input_mask & MxPsuInputMask.MX_PSU_INPUT_PSTOP1) != 0
        self.pstop2_asserted = (self.psu_input_mask & MxPsuInputMask.MX_PSU_INPUT_PSTOP2) != 0
        self.reset_ext_asserted = (self.psu_input_mask & MxPsuInputMask.MX_PSU_INPUT_RESET_EXT) != 0
        self.reset_keypad_pressed = (self.psu_input_mask & MxPsuInputMask.MX_PSU_INPUT_RESET_KEYPAD) != 0
        self.enabling_device_asserted = (self.psu_input_mask & MxPsuInputMask.MX_PSU_INPUT_ENABLING_DEVICE) != 0

    def set_estop_asserted(self, state: bool):
        """ Update estop_asserted and update the mask (psuInputMask) accordingly """
        self.estop_asserted = state
        if state:
            self.psu_input_mask |= MxPsuInputMask.MX_PSU_INPUT_ESTOP
        else:
            self.psu_input_mask &= ~MxPsuInputMask.MX_PSU_INPUT_ESTOP

    def set_pstop1_asserted(self, state: bool):
        """ Update pstop1_asserted and update the mask (psuInputMask) accordingly """
        self.pstop1_asserted = state
        if state:
            self.psu_input_mask |= MxPsuInputMask.MX_PSU_INPUT_PSTOP1
        else:
            self.psu_input_mask &= ~MxPsuInputMask.MX_PSU_INPUT_PSTOP1

    def set_pstop2_asserted(self, state: bool):
        """ Update pstop2_asserted and update the mask (psuInputMask) accordingly """
        self.pstop2_asserted = state
        if state:
            self.psu_input_mask |= MxPsuInputMask.MX_PSU_INPUT_PSTOP2
        else:
            self.psu_input_mask &= ~MxPsuInputMask.MX_PSU_INPUT_PSTOP2

    def set_reset_ext_asserted(self, state: bool):
        """ Update reset_ext_asserted and update the mask (psuInputMask) accordingly """
        self.reset_ext_asserted = state
        if state:
            self.psu_input_mask |= MxPsuInputMask.MX_PSU_INPUT_RESET_EXT
        else:
            self.psu_input_mask &= ~MxPsuInputMask.MX_PSU_INPUT_RESET_EXT

    def set_reset_keypad_asserted(self, state: bool):
        """ Update reset_keypad_asserted and update the mask (psuInputMask) accordingly """
        self.reset_keypad_pressed = state
        if state:
            self.psu_input_mask |= MxPsuInputMask.MX_PSU_INPUT_RESET_KEYPAD
        else:
            self.psu_input_mask &= ~MxPsuInputMask.MX_PSU_INPUT_RESET_KEYPAD

    def set_enabling_device_asserted(self, state: bool):
        """ Update enabling_device_asserted and update the mask (psuInputMask) accordingly """
        self.enabling_device_asserted = state
        if state:
            self.psu_input_mask |= MxPsuInputMask.MX_PSU_INPUT_ENABLING_DEVICE
        else:
            self.psu_input_mask &= ~MxPsuInputMask.MX_PSU_INPUT_ENABLING_DEVICE

    def __str__(self) -> str:
        return (f"psu_input_mask: {hex(self.psu_input_mask)} -> "
                f"estop_asserted: {self.estop_asserted}, "
                f"pstop1_asserted: {self.pstop1_asserted}, "
                f"pstop2_asserted: {self.pstop2_asserted}, "
                f"reset_ext_asserted: {self.reset_ext_asserted}, "
                f"reset_keypad_pressed: {self.reset_keypad_pressed}, "
                f"enabling_device_asserted: {self.enabling_device_asserted}")


class RobotSafetyStatus:
    """Class for storing the safety stop status of a Mecademic robot.

    Attributes
    ----------
    robot_operation_mode : MxRobotOperationMode
        The current robot operation mode, based on the the power supply key position (not supported on Meca500 robots).
        When the key is not in "automatic" position, restrictions will apply for using the robot.
    reset_ready : bool
        Not yet implemented. Future implementation will:
        Indicate if it's currently possible to reset safety stop conditions that are linked to the power supply reset
        button because they remove motor power (EStop, PStop1, Operation mode change, robot reboot, etc.)
    stop_mask : int
        Bit mask that summarizes all safety stop conditions on the robot (including both active or resettable signals).
        Use bits from MxSafeStopCategory.
        Note: Also available as individual signal states (estop_state, pstop1_state, etc...)
    stop_resettable_mask : int
        Bit mask that summarizes all safety stop conditions that are currently resettable.
        Use bits from MxSafeStopCategory.
        Note: Also available as individual signal states (estop_state, pstop1_state, etc...)
    estop_state : MxStopState
        Current EStop status.
        Note that Meca500 revision 3 or older never report this condition because these robots's power supply
        will be shutdown completely in case of EStop (and thus this API will get disconnected from the robot instead).
    pstop1_state : MxStopState
        Current PStop1 status
    pstop2_state : MxStopState
        Current PStop2 status
        *** IMPORTANT NOTE: PStop2 is not safety-rated on Meca500 robots ***
    operation_mode_stop_state : MxStopState
        Current status for "operation mode" safety stop condition (upon mode changes or when mode is locked)
    enabling_device_released_stop_state : MxStopState
        Current status for "enabling device" safety stop condition
    voltage_fluctuation_stop_state : MxStopState
        Current status for "voltage fluctuation" safety stop condition
    reboot_stop_state : MxStopState
        Current status for "robot just rebooted" safety stop condition
    redundancy_fault_stop_state : MxStopState
        Current status for "redundancy fault" safety stop condition
    standstill_fault_stop_state : MxStopState
        Current status for "standstill fault" safety stop condition
    connection_dropped_stop_state : MxStopState
        Current status for "connection dropped" safety stop condition
    minor_error_stop_state : MxStopState
        Current status for "minor error" safety stop condition, which is triggered if robot removes motor voltage
        for any internal reason other than the safety stop signals above (thus generally due to a minor internal error).
        If this happens, see the robot logs for details.
    static_masks : RobotStaticSafetyStopMasks
        Useful masks to categorize safety stop signals from stop_mask
"""

    def __init__(self):
        self.robot_operation_mode = MxRobotOperationMode.MX_ROBOT_OPERATION_MODE_AUTO
        self.reset_ready = False
        self.stop_mask = 0
        self.stop_resettable_mask = 0
        self.estop_state = MxStopState.MX_STOP_STATE_RESET
        self.pstop1_state = MxStopState.MX_STOP_STATE_RESET
        self.pstop2_state = MxStopState.MX_STOP_STATE_RESET
        self.operation_mode_stop_state = MxStopState.MX_STOP_STATE_RESET
        self.enabling_device_released_stop_state = MxStopState.MX_STOP_STATE_RESET
        self.voltage_fluctuation_stop_state = MxStopState.MX_STOP_STATE_RESET
        self.reboot_stop_state = MxStopState.MX_STOP_STATE_RESET
        self.redundancy_fault_stop_state = MxStopState.MX_STOP_STATE_RESET
        self.standstill_fault_stop_state = MxStopState.MX_STOP_STATE_RESET
        self.connection_dropped_stop_state = MxStopState.MX_STOP_STATE_RESET
        self.minor_error_stop_state = MxStopState.MX_STOP_STATE_RESET

        # Useful masks to categorize stop_mask above
        self.static_masks = RobotStaticSafetyStopMasks()

    def set_estop_state(self, stop_state: MxStopState):
        """Change the EStop state

        Parameters
        ----------
        stop_state : MxStopState
            EStop state to set
        """
        self.set_stop_state(MxSafeStopCategory.MX_SAFE_STOP_ESTOP, stop_state)

    def set_pstop1_state(self, stop_state: MxStopState):
        """Change the PStop1 state

        Parameters
        ----------
        stop_state : MxStopState
            PStop1 state to set
        """
        self.set_stop_state(MxSafeStopCategory.MX_SAFE_STOP_PSTOP1, stop_state)

    def set_pstop2_state(self, stop_state: MxStopState):
        """Change the PStop2 state

        Parameters
        ----------
        stop_state : MxStopState
            PStop2 state to set
        """
        self.set_stop_state(MxSafeStopCategory.MX_SAFE_STOP_PSTOP2, stop_state)

    def set_operation_mode_stop_state(self, stop_state: MxStopState):
        """Change the "operation mode" safety stop state

        Parameters
        ----------
        stop_state : MxStopState
            Safety stop state to set
        """
        self.set_stop_state(MxSafeStopCategory.MX_SAFE_STOP_OPERATION_MODE, stop_state)

    def set_enabling_device_released_stop_state(self, stop_state: MxStopState):
        """Change the "enabling device" safety stop state

        Parameters
        ----------
        stop_state : MxStopState
            Safety stop state to set
        """
        self.set_stop_state(MxSafeStopCategory.MX_SAFE_STOP_ENABLING_DEVICE_RELEASED, stop_state)

    def set_voltage_fluctuation_stop_state(self, stop_state: MxStopState):
        """Change the "voltage fluctuation" safety stop state

        Parameters
        ----------
        stop_state : MxStopState
            Safety stop state to set
        """
        self.set_stop_state(MxSafeStopCategory.MX_SAFE_STOP_VOLTAGE_FLUCTUATION, stop_state)

    def set_reboot_stop_state(self, stop_state: MxStopState):
        """Change the "robot just rebooted" safety stop state

        Parameters
        ----------
        stop_state : MxStopState
            Safety stop state to set
        """
        self.set_stop_state(MxSafeStopCategory.MX_SAFE_STOP_REBOOT, stop_state)

    def set_redundancy_fault_stop_state(self, stop_state: MxStopState):
        """Change the "redundancy fault" safety stop state

        Parameters
        ----------
        stop_state : MxStopState
            Safety stop state to set
        """
        self.set_stop_state(MxSafeStopCategory.MX_SAFE_STOP_REDUNDANCY_FAULT, stop_state)

    def set_standstill_fault_stop_state(self, stop_state: MxStopState):
        """Change the "standstill fault" safety stop state

        Parameters
        ----------
        stop_state : MxStopState
            Safety stop state to set
        """
        self.set_stop_state(MxSafeStopCategory.MX_SAFE_STOP_STANDSTILL_FAULT, stop_state)

    def set_connection_dropped_stop_state(self, stop_state: MxStopState):
        """Change the "connection dropped" safety stop state

        Parameters
        ----------
        stop_state : MxStopState
            Safety stop state to set
        """
        self.set_stop_state(MxSafeStopCategory.MX_SAFE_STOP_CONNECTION_DROPPED, stop_state)

    def set_minor_error_stop_state(self, stop_state: MxStopState):
        """Change the "minor error" safety stop state

        Parameters
        ----------
        stop_state : MxStopState
            Safety stop state to set
        """
        self.set_stop_state(MxSafeStopCategory.MX_SAFE_STOP_MINOR_ERROR, stop_state)

    def set_stop_state(self, stop_category: MxSafeStopCategory, stop_state: MxStopState):
        """Change a safety stop state

        Parameters
        ----------
        stop_category : MxSafeStopCategory
            Safety stop category to change state for
        stop_state : MxStopState
            Safety stop state to set
        """
        # Make sure to cast to enum value (in case we were passed an int)
        stop_state = MxStopState(stop_state)

        # Update specified stop state
        if stop_category == MxSafeStopCategory.MX_SAFE_STOP_ESTOP:
            self.estop_state = stop_state
        elif stop_category == MxSafeStopCategory.MX_SAFE_STOP_PSTOP1:
            self.pstop1_state = stop_state
        elif stop_category == MxSafeStopCategory.MX_SAFE_STOP_PSTOP2:
            self.pstop2_state = stop_state
        elif stop_category == MxSafeStopCategory.MX_SAFE_STOP_OPERATION_MODE:
            self.operation_mode_stop_state = stop_state
        elif stop_category == MxSafeStopCategory.MX_SAFE_STOP_ENABLING_DEVICE_RELEASED:
            self.enabling_device_released_stop_state = stop_state
        elif stop_category == MxSafeStopCategory.MX_SAFE_STOP_VOLTAGE_FLUCTUATION:
            self.voltage_fluctuation_stop_state = stop_state
        elif stop_category == MxSafeStopCategory.MX_SAFE_STOP_REBOOT:
            self.reboot_stop_state = stop_state
        elif stop_category == MxSafeStopCategory.MX_SAFE_STOP_REDUNDANCY_FAULT:
            self.redundancy_fault_stop_state = stop_state
        elif stop_category == MxSafeStopCategory.MX_SAFE_STOP_STANDSTILL_FAULT:
            self.standstill_fault_stop_state = stop_state
        elif stop_category == MxSafeStopCategory.MX_SAFE_STOP_CONNECTION_DROPPED:
            self.connection_dropped_stop_state = stop_state
        elif stop_category == MxSafeStopCategory.MX_SAFE_STOP_MINOR_ERROR:
            self.minor_error_stop_state = stop_state

        # Update the masks too
        if stop_state == MxStopState.MX_STOP_STATE_ACTIVE:
            self.stop_mask |= stop_category
            self.stop_resettable_mask &= ~stop_category
        elif stop_state == MxStopState.MX_STOP_STATE_RESETTABLE:
            self.stop_mask |= stop_category
            self.stop_resettable_mask |= stop_category
        else:
            self.stop_mask &= ~stop_category
            self.stop_resettable_mask &= ~stop_category

    @classmethod
    def mask_to_string(cls, mask: int) -> str:
        """Format as a string a safety stop mask (detailing each safety stop signal that is active in the mask)

        Args:
            mask (int): Mask to print as detailed string

        Returns:
            str: Detailed string that represents all active safety stop signals in the mask
        """
        status_masks: list[str] = []
        if mask & int(MxSafeStopCategory.MX_SAFE_STOP_ESTOP):
            status_masks.append('EStop')
        if mask & int(MxSafeStopCategory.MX_SAFE_STOP_PSTOP1):
            status_masks.append('PStop1')
        if mask & int(MxSafeStopCategory.MX_SAFE_STOP_PSTOP2):
            status_masks.append('PStop2')
        if mask & int(MxSafeStopCategory.MX_SAFE_STOP_OPERATION_MODE):
            status_masks.append('Operation mode')
        if mask & int(MxSafeStopCategory.MX_SAFE_STOP_ENABLING_DEVICE_RELEASED):
            status_masks.append('Enabling device released')
        if mask & int(MxSafeStopCategory.MX_SAFE_STOP_VOLTAGE_FLUCTUATION):
            status_masks.append('Voltage fluctuation')
        if mask & int(MxSafeStopCategory.MX_SAFE_STOP_REBOOT):
            status_masks.append('Robot rebooted')
        if mask & int(MxSafeStopCategory.MX_SAFE_STOP_REDUNDANCY_FAULT):
            status_masks.append('Redundancy fault')
        if mask & int(MxSafeStopCategory.MX_SAFE_STOP_STANDSTILL_FAULT):
            status_masks.append('Standstill fault')
        if mask & int(MxSafeStopCategory.MX_SAFE_STOP_CONNECTION_DROPPED):
            status_masks.append('Connection dropped')
        if mask & int(MxSafeStopCategory.MX_SAFE_STOP_MINOR_ERROR):
            status_masks.append('Minor error')
        return ', '.join(status_masks)

    def __str__(self) -> str:
        return (
            f"operation mode: {tools.robot_operation_mode_to_string(self.robot_operation_mode)}, "
            f"reset_ready: {self.reset_ready}, "
            f"stop_mask: {hex(self.stop_mask)} ({self.mask_to_string(self.stop_mask)}), "
            f"stop_resettable_mask: {hex(self.stop_resettable_mask)} ({self.mask_to_string(self.stop_resettable_mask)})"
        )


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


class NetworkConfig:
    """Class for storing the Mecademic robot's network configuration.

    Attributes
    ----------
    name : str
        Robot name for DHCP requests
    dhcp : bool
        DHCP mode enabled for automatic IP assignment by DHCP server
    ip : str
        IPv4 address (ex: '192.168.0.100')
    mask : str
        Network mask (ex: '255.255.255.0')
    gateway : str
        Gateway IP address (ex: '192.168.0.1')
    mac : str
        Robot read-only MAC address (ex: '20:B0:F7:06:E4:80')

"""

    def __init__(self):

        # The following are status fields.
        self.name = ''
        self.dhcp = False
        self.ip = '192.168.0.100'
        self.mask = '255.255.255.0'
        self.gateway = ''
        self.mac = ''

    def __str__(self) -> str:
        return (f"Name: {self.name}, "
                f"DHCP: {self.dhcp}, "
                f"ip: {self.ip}, "
                f"mask: {self.mask}, "
                f"gateway: {self.gateway}, "
                f"mac: {self.mac}")

    def __repr__(self) -> str:
        return str(self)


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
    comm_err_warning : bool
        True if some communication errors were detected with the external tool.
        This could mean that the cable may be damaged and must be replaced,
        or that the cable may simply not be screwed tight enough on either side.
"""

    def __init__(self):

        # The following are status fields.
        self.sim_tool_type = MxExtToolType.MX_EXT_TOOL_NONE
        self.physical_tool_type = MxExtToolType.MX_EXT_TOOL_NONE
        self.homing_state = False
        self.error_status = False
        self.overload_error = False
        self.comm_err_warning = False

    def __str__(self) -> str:
        return (f"Sim tool type: {self.sim_tool_type}, "
                f"Physical tool type: {self.physical_tool_type}, "
                f"homed: {self.homing_state}, "
                f"error: {self.error_status}, "
                f"overload: {self.overload_error}")

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


class IoStatus:
    """Class for storing the Mecademic robot's IO modules status.

    Attributes
    ----------
    bank_id : int
        Type of this IO module.
        1: MxIoBankId.MX_IO_BANK_ID_IO_MODULE
    present : bool
        True if an IO module of this type is present on the robot.
    nb_inputs : int
        Number of digital inputs supported by this IO module.
    nb_outputs : int
        Number of digital outputs supported by this IO module.
    sim_mode : bool
        True if the IO module is in simulation mode.
    error : int
        Error code of the IO module (0 if no error).
    timestamp : int
        Monotonic timestamp associated with data (in microseconds since robot last reboot)
        This timestamp is stamped by the robot so it is not affected by host/network jitter
"""

    def __init__(self):

        # The following are status fields.
        self.bank_id = MxIoBankId.MX_IO_BANK_ID_UNDEFINED
        self.present = False
        self.nb_inputs = 0
        self.nb_outputs = 0
        self.sim_mode = False
        self.error = 0
        self.timestamp = 0

    def __str__(self) -> str:
        return (f"BankId: {self.bank_id}, "
                f"Physically present: {self.present}, "
                f"Digital inputs: {self.nb_inputs}, "
                f"Digital outputs: {self.nb_outputs}, "
                f"Simulation mode: {self.sim_mode}, "
                f"error: {self.error}")

    def __repr__(self) -> str:
        return str(self)


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
        return (f'holding={self.holding_part}, '
                f'pos_reached={self.target_pos_reached}, '
                f'closed={self.closed}, '
                f'opened={self.opened}')

    def __repr__(self) -> str:
        return str(self)


class VacuumState:
    """Class for storing the Mecademic robot's IO module vacuum state.

    Attributes
    ----------
    vacuum_on : bool
        True if the vacuum is currently 'on' (trying to pick or holding part).
    purge_on: bool
        True if currently pushing air to release part (see SetVacuumPurgeDuration).
    holding_part : bool
        True if currently holding part (based on configured pressure thresholds, see SetVacuumThreshold ).
    timestamp : int
        Monotonic timestamp associated with data (in microseconds since robot last reboot)
        This timestamp is stamped by the robot so it is not affected by host/network jitter
"""

    def __init__(self):

        # The following are status fields.
        self.vacuum_on = False
        self.purge_on = False
        self.holding_part = False
        self.timestamp = 0

    def __str__(self) -> str:
        return (f"Vacuum: {'on' if self.vacuum_on else 'off'}, "
                f"Purge: {'on' if self.purge_on else 'off'}, "
                f"Holding part: {self.holding_part}")

    def __repr__(self) -> str:
        return str(self)


class CollisionObject:
    """Class that represents one object that can enter in collision with another or with the work zone boundary.
       A collision object is defined by its group (MxCollisionGroup) and, in some cases, an index (like the joint)

    Attributes
    ----------
    group : MxCollisionGroup
        The group (type) of this object
    index: int
        The index of the object within the group (for groups that can have multiple objects).
        Available indices for groups are:
        - MxCollisionGroup.MX_COLLISION_GROUP_ROBOT:     Use index values from MxCollisionGroupRobotIdx.
        - MxCollisionGroup.MX_COLLISION_GROUP_FCP:       Index is not used (always 0).
        - MxCollisionGroup.MX_COLLISION_GROUP_TOOL:      Always 0 when tool sphere is used.
                                                         Future versions may support multiple tool objects.
        - MxCollisionGroup.MX_COLLISION_GROUP_ENV_OBJ:   User-defined index of the user-defined environment objects.
                                                         (not supported yet)
        - MxCollisionGroup.MX_COLLISION_GROUP_WORK_ZONE: Index is not used (always 0).
"""

    def __init__(self,
                 group=MxCollisionGroup.MX_COLLISION_GROUP_ROBOT,
                 index=MxCollisionGroupRobotIdx.MX_COLLISION_GROUP_ROBOT_BASE):
        self.set(group, index)

    def set(self, group: MxCollisionGroup, index: int):
        """ Setter function """
        # The following are status fields.
        self.group = group
        # Update the index according to new group (if appropriate)
        if self.group == MxCollisionGroup.MX_COLLISION_GROUP_ROBOT:
            self.index = MxCollisionGroupRobotIdx(index)
        elif self.group == MxCollisionGroup.MX_COLLISION_GROUP_TOOL:
            self.index = MxCollisionGroupToolIdx(index)
        else:
            self.index = int(index)

    def set_from_response(self, response_args: list[int]) -> list[int]:
        """ Set CollisionObject from parsed reply message argument """
        self.set(MxCollisionGroup(response_args.pop(0)), response_args.pop(0))
        return response_args

    def __eq__(self, other):
        if int(self.group) != int(other.group):
            return False
        if (self.group == MxCollisionGroup.MX_COLLISION_GROUP_ROBOT
                or self.group == MxCollisionGroup.MX_COLLISION_GROUP_TOOL
                or self.group == MxCollisionGroup.MX_COLLISION_GROUP_ENV_OBJ):
            # In this group, the index is important
            if int(self.index) != int(other.index):
                return False
        return True

    def __str__(self) -> str:
        return tools.robot_collision_group_to_string(self.group, self.index)

    def __repr__(self) -> str:
        return str(self)


class SelfCollisionStatus:
    """Class for storing the Mecademic robot's self collision status.
       This is used when robot collision detection has been activated (with SetCollisionCfg).

    Attributes
    ----------
    collision_detected : bool
        True if the robot has detected a collision with itself.
        Note that when the collision severity is set to MX_EVENT_SEVERITY_PAUSE_MOTION or greater
        the robot will have stopped just before the collision actually occurs and collision_detected will be True
        until ResumeMotion is called.
        When collision severity is set to MX_EVENT_SEVERITY_WARNING the robot will actually continue to move and the
        collision will actually occur. This can damage the robot or the tool.
    object1 : MxCollisionGroup
        When collision is detected, indicates the first of the two objects that caused the collision
        (a part of the robot or the tool)
    object2 : MxCollisionGroup
        When collision is detected, indicates the second of the two objects that caused the collision
        (a part of the robot or the tool)
"""

    def __init__(self,
                 collision_detected: bool = False,
                 collision_object1: CollisionObject = CollisionObject(),
                 collision_object2: CollisionObject = CollisionObject()):
        # The following are status fields.
        self.collision_detected = collision_detected
        self.object1 = collision_object1
        self.object2 = collision_object2

    def set_from_response(self, response_args: list[int]) -> list[int]:
        """ Set CollisionObject from parsed reply message argument """
        self.collision_detected = bool(response_args.pop(0))
        response_args = self.object1.set_from_response(response_args)
        response_args = self.object2.set_from_response(response_args)
        return response_args

    def __str__(self) -> str:
        if self.collision_detected:
            return f'Collision detected between {self.object1} and {self.object2}'
        else:
            return 'No collision detected'

    def __repr__(self) -> str:
        return str(self)


class WorkZoneStatus:
    """Class for storing the Mecademic robot's "in work zone" status.
       This is used when robot work zone has been defined and enabled (with SetWorkZoneCfg).

    Attributes
    ----------
    outside_work_zone : bool
        True if the a part of the robot or the tool have reached the work zone boundary.
        Note that when the collision severity is set to MX_EVENT_SEVERITY_PAUSE_MOTION or greater
        the robot will have stopped just before the robot moves outside the work zone but flag outside_work_zone will
        still be set to True until ResumeMotion is called.
        When collision severity is set to MX_EVENT_SEVERITY_WARNING the flag outside_work_zone will report whether the
        robot is currently outside the work zone.
    object : MxCollisionGroup
        Indicate the object that reached the work zone boundary (a part of the robot, or the tool)
"""

    def __init__(self, outside_work_zone: bool = False, collision_object: CollisionObject = CollisionObject()):
        # The following are status fields.
        self.outside_work_zone = outside_work_zone
        self.object = collision_object

    def set_from_response(self, response_args: list[int]) -> list[int]:
        """ Set CollisionObject from parsed reply message argument """
        self.outside_work_zone = bool(response_args.pop(0))
        response_args = self.object.set_from_response(response_args)
        return response_args

    def __str__(self) -> str:
        if self.outside_work_zone:
            return f'Work zone boundary reached by {self.object}'
        else:
            return 'Robot is inside work zone'

    def __repr__(self) -> str:
        return str(self)


class CollisionStatus:
    """Class for storing the Mecademic robot's collision status (collision with self or work zone boundary).
       This is used when robot collision detection has been activated (with SetCollisionCfg) or
       work zone has been enabled (with SetWorkZoneCfg)

    Attributes
    ----------
    self_collision_status : SelfCollisionStatus
        Current self collision status
    work_zone_status : WorkZoneStatus
        Current "inside work zone" status
"""

    def __init__(self,
                 self_collision_status: SelfCollisionStatus = SelfCollisionStatus(),
                 work_zone_status: WorkZoneStatus = WorkZoneStatus()):
        # The following are status fields.
        self.self_collision_status = self_collision_status
        self.work_zone_status = work_zone_status

    def __str__(self) -> str:
        return f'Collision status: [ {self.self_collision_status}, {self.work_zone_status} ]'

    def __repr__(self) -> str:
        return str(self)


class RobotSidecarStatus:
    """Class for storing the status of a "sidecar" scripting engine connected to the robot.

    Attributes
    ----------
    embedded : bool
        True if this sidecar instance is running embedded inside the robot.
    remote_ip : str
        The IP address of this sidecar instance.
    registered_functions : list[str]
        List of functions registered by this sidecar instance.
"""

    def __init__(self):

        # The following are status fields.
        self.id: Optional[int] = None
        self.embedded = False
        self.remote_ip = ""
        self.registered_functions: list[str] = []

    def __str__(self) -> str:
        ip = ""
        if not self.embedded:
            ip = f", IP: {self.remote_ip}"
        return ((f"Id: {self.id}, "
                 f"Embedded: {self.embedded}, "
                 f"Ip: {self.remote_ip}{ip}, "
                 f"functions: [{','.join(self.registered_functions)}]"))
