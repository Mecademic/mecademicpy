"""
This file define classes required to use the mecademicpy modules and control Mecademic robots.
Some of these classes are used in the python API (Message, Callbacks, Exceptions, InterruptableEvent, ...)
Some of these classes define robot information and data, such as RobotVersion, RobotInfo, UpdateProgress, RobotRtData,
RobotStatus, RobotSafetyStatus, GripperStatus, IoStatus, CollisionStatus, etc.
"""
from __future__ import annotations

import enum
import json
import re
import threading
import time
from copy import deepcopy
from dataclasses import InitVar, dataclass, field, fields, replace
from typing import Any, Callable, Dict, List, Optional, TypeVar, Union

import mecademicpy.tools as tools

# pylint: disable=wildcard-import,unused-wildcard-import
from .mx_robot_def import *

GRIPPER_OPEN = True
GRIPPER_CLOSE = False

DEFAULT_WAIT_TIMEOUT = 10

# Available levels for SetTorqueLimitsCfg
TORQUE_LIMIT_SEVERITIES = {'disabled': 0, 'warning': 1, 'pause-motion': 2, 'clear-motion': 3, 'error': 4}


# Temporary location until consensus reached
class RtDataUpdateType(enum.IntEnum):
    MX_RT_DATA_UPDATE_TYPE_UNAVAILABLE = 0  # Data is not available
    MX_RT_DATA_UPDATE_TYPE_CYCLICAL = 1  # Data is sent every cycle
    MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL = 2  # Data sent every cycle when configured through SetRealTimeMonitoring
    MX_RT_DATA_UPDATE_TYPE_EVENT_BASED = 3  # Data is sent upon connection and updated when its values change


#####################################################################################
# Private utility classes and methods.
#####################################################################################
class Message:
    """
    Represents a response message received from a Mecademic robot.

    Attributes
    ----------
    id
        Identifier of the message, representing its type.
    data
        Raw payload of the message.
    json_data
        Parsed JSON data as a dictionary, or None if the message is not in JSON format.
        The robot JSON message format is as follows::

            {
                "MX_JSON_KEY_CODE": int,
                "MX_JSON_KEY_META_DATA": {
                    "MX_JSON_KEY_MSG_TYPE": int
                },
                "MX_JSON_KEY_DATA": {
                    # Per-code JSON arguments and values
                }
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
        """
        Constructs a message object from a raw string input.

        Parameters
        ----------
        input
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
        """
        Initializes this base exception.

        Parameters
        ----------
        message
            User message to print
        """
        self.message = message
        super().__init__(self.message)

    pass


class MecademicFatalException(MecademicException):
    """Class for all Mecademic exceptions that should be considered fatal (stack trace should be printed).
    """
    pass


class MecademicNonFatalException(MecademicException):
    """Class for all Mecademic exceptions that should be considered non fatal
       (can be catch, error printed, and application may continue).
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


class MecademicInterruptBase(MecademicNonFatalException):
    """ Base class for exceptions that signal that an operation was aborted before timeout or before completion."""


class DisconnectError(MecademicInterruptBase):
    """A non-nominal disconnection has occurred.
    """
    pass


class InterruptException(MecademicInterruptBase):
    """An event has encountered an error. Perhaps it will never be set."""
    pass


class StopProgramException(MecademicInterruptBase):
    """
    Exception raised when a program is requested to stop gracefully
    (via a ``StopProgram`` request).

    To continue controlling the robot after this exception is raised, the program can
    catch it and call ``SetStopProgramInterruption(False)``.
    This allows additional robot commands to be sent (for example, to move the robot
    to a safe position) before the program exits gracefully.

    Parameters
    ----------
    message
        User message for this exception
    graceful
        - True if the exception was raised due to a ``StopProgram`` request and the program is allowed
          time to terminate gracefully (e.g., move the robot to a safe position) after calling
          SetStopProgramInterruption(false).
        - False if the exception was raised due to ``ClearMotion`` or another condition where the
          program has lost control of the robot. In this case, the program should exit shortly after
          receiving the exception and will not be allowed to send more commands to the robot whatsoever.
    """

    def __init__(self, message: str, graceful: bool):
        self.graceful = graceful
        super().__init__(message)

    pass


class IllegalAfterControllingCommandException(MecademicInterruptBase):
    """This exception is raised when a program is trying to send robot commands after calling a
       "controlling" command. By definition, no robot commands are allowed to be queued
       after a controlling command because the control must be handed back to the main API."""
    pass


class TimeoutException(MecademicNonFatalException):
    """Requested timeout during a blocking operation (synchronous mode or ``Wait*`` functions) has
    been reached (raised by InterruptableEvent)."""
    pass


class NotFoundException(MecademicNonFatalException):
    """A method, variable or object was not found."""
    pass


class ArgErrorException(MecademicNonFatalException):
    """An argument was invalid, the operation could not be completed."""
    pass


class CommandFailedException(MecademicNonFatalException):
    """The robot refused a command for some reason. See the exception message for details."""
    pass


class SetRtcMode(enum.IntEnum):
    """
    RTC (real-time clock, i.e. date/time) options when connecting to the robot
    """
    DONT_SET_RTC = 0,
    """Do not change the robot's date/time (RTC)"""

    ALWAYS_SET_RTC = 1,
    """Update the robot's date/time (RTC) with current time on this PC"""

    SET_ONLY_IF_NEEDED = 2,
    """Update the robot's date/time (RTC) only for robot models that loose date/time upon reboot"""


class RobotCallbacks:
    """
    Class for storing possible status event callbacks for the Mecademic robot.

    Attributes
    ----------
    on_connected
        Function called once the robot is connected.
    on_disconnected
        Function called once the robot is disconnected.
    on_status_updated
        Function called once the robot status is updated.
    on_mecascript_engine_status_updated
        Function called once the MecaScript engine status is updated.
    on_program_execution_status_updated
        Function called once a MecaScript program execution status is updated.
    on_status_gripper_updated
        Function called once the gripper status is updated (legacy, use external tool callbacks instead).
    on_external_tool_status_updated
        Function called once the external tool status is updated.
    on_gripper_state_updated
        Function called once the gripper state is updated.
    on_valve_state_updated
        Function called once the valve state is updated.
    on_output_state_updated
        Function called when digital outputs change.
    on_input_state_updated
        Function called when digital inputs change.
    on_vacuum_state_updated
        Function called once the vacuum state is updated.
    on_activated
        Function called once the robot is activated.
    on_deactivated
        Function called once the robot is deactivated.
    on_homed
        Function called once homing is complete.
    on_error
        Function called when the robot enters an error state.
    on_error_reset
        Function called once the error state is reset.
    on_safety_stop
        Function called when the robot enters safety stop state following raised safety stop conditions
        (EStop, PStop1, PStop2, etc.).
        Note that `on_safety_stop_state_change` can be used to be notified of additional safety stop
        condition changes while the robot is already in safety stop.
    on_safety_stop_reset
        Function called when all safety stop conditions (EStop, PStop1, PStop2, etc.) are cleared
        (reset).
    on_safety_stop_resettable
        Function called when all safety stop conditions (EStop, PStop1, PStop2, etc.) can be reset,
        i.e., when the safety stop conditions are no longer present and require a reset (via the power
        supply reset button or ``ResumeMotion``).
    on_safety_stop_state_change
        Function called when any safety stop state changes (see ``RobotSafetyStatus``).
    on_pstop2
        Function called if PStop2 is activated.
        *** DEPRECATED *** (replaced by `on_safety_stop_state`)
    on_pstop2_resettable
        Function called when the PStop2 condition can be reset (i.e., the PStop2 signal is no longer asserted).
        *** DEPRECATED *** (replaced by `on_safety_stop_state`)
    on_pstop2_reset
        Function called if PStop2 is reset.
        *** DEPRECATED *** (replaced by `on_safety_stop_state`)
    on_estop
        Function called if EStop is activated.
        *** DEPRECATED *** (replaced by `on_safety_stop_state`)
    on_estop_reset
        Function called if EStop is reset.
        *** DEPRECATED *** (replaced by `on_safety_stop_state`)
    on_estop_resettable
        Function called when the EStop condition can be reset (i.e., the EStop signal is no longer asserted).
        *** DEPRECATED *** (replaced by `on_safety_stop_state`)
    on_motion_paused
        Function called once motion is paused.
    on_motion_cleared
        Function called once motion is cleared.
    on_motion_resumed
        Function called once motion is resumed.
    on_checkpoint_reached
        Function called when a checkpoint is reached.
    on_checkpoint_discarded
        Function called when a checkpoint is discarded (due to motion cleared, robot deactivated,
        robot error, or safety stop).
    on_activate_sim
        Function called once simulation mode is activated.
    on_deactivate_sim
        Function called once simulation mode is deactivated.
    on_activate_ext_tool_sim
        Function called once gripper simulation mode is activated.
    on_deactivate_ext_tool_sim
        Function called once gripper simulation mode is deactivated.
    on_io_sim_enabled
        Function called once I/O simulation mode is enabled.
    on_io_sim_disabled
        Function called once I/O simulation mode is disabled.
    on_activate_recovery_mode
        Function called once recovery mode is activated.
    on_deactivate_recovery_mode
        Function called once recovery mode is deactivated.
    on_command_message
        Function called each time a command response is received.
    on_monitor_message
        Function called each time an event is received on the monitoring port.
        Available only when connected to the robot in monitoring mode.
        Note: ``on_monitor_message`` may not be very useful; ``on_end_of_cycle`` is generally
        preferred, as it works in both monitoring and control modes.
    on_program_state
        Function called each time a program starts or fails to start.
    on_end_of_cycle
        Function called each time the end of cycle is reached.
        It is triggered once all real-time data for the current monitoring interval has been received.
        At this moment, all robot real-time data is coherent (i.e., from the same cycle).
    """

    def __init__(self):
        self.on_connected: Callable[[], None] = None
        self.on_disconnected: Callable[[], None] = None

        self.on_status_updated: Callable[[], None] = None
        self.on_mecascript_engine_status_updated: Callable[[], None] = None
        self.on_program_execution_status_updated: Callable[[], None] = None
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
        self.on_safety_stop: Callable[[], None] = None
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

        self.on_program_state: Callable[[], None] = None

        self.on_end_of_cycle: Callable[[], None] = None


# pylint: disable=invalid-name
TmpInterruptType = TypeVar('TmpInterruptType', bound='MecademicInterruptBase')


class InterruptableEvent:
    """
    Extends the default event class to support unblocking and raising an exception when the event
    becomes irrelevant and will not occur (e.g., waiting on a checkpoint while the robot is in an
    error state).

    Attributes
    ----------
    id
        Id for event.
    _cond
        A standard condition-variable object.
    _is_set
        If True, event is set (non-blocking), else it's cleared (blocking).
    _interrupted
        If True, event is in an error state.
    _interrupted_msg
        User message that explains the reason of interruption.
    _abort_on_error
        Tells if this event must be awakened if the robot falls into error state, by default False
    _abort_on_clear_motion
        Tells if this event must be awakened if the robot's motion queue is cleared
        Note that this also includes PStop2 condition and robot deactivation (which also cause the
        motion queue to be cleared).
    _wait_callback
        Optional callback function that is called periodically while the user code is awaiting on this event.
        This is mainly used for calling user callbacks while the user thread is awaiting, which simplifies usage
        of callbacks and avoids having to explicitly call RunCallbacks() in many cases.
"""

    # pylint: disable=redefined-builtin
    def __init__(self,
                 id=None,
                 data=None,
                 abort_on_error=False,
                 abort_on_clear_motion=False,
                 wait_callback: Optional[Callable] = None):
        self._id = id
        self._data = data
        self._cond = threading.Condition()
        self._is_set = False
        self._set_count = 0  # This is used to detect event.set() then event.clear() before it had time to awake
        self._interrupted = False
        self._interrupted_msg = ""
        self._tmp_interrupt_causes: dict[str, MecademicInterruptBase] = {}
        self._abort_on_error = abort_on_error
        self._abort_on_clear_motion = abort_on_clear_motion
        self._wait_callback = wait_callback

    def check_interrupted(self):
        """ Check if this interruptable event was interrupted, and raise exception if appropriate."""
        with self._cond:  # Lock the condition (like a mutex)
            self._check_interrupted_internal()

    def _check_interrupted_internal(self):
        """ Internal implementation of check_interrupted, assuming the lock is already taken."""
        if self._interrupted:
            if self._interrupted_msg != "":
                raise InterruptException(self._interrupted_msg)
            raise InterruptException('Event interrupted, possibly because event will never be triggered.')
        if len(self._tmp_interrupt_causes) != 0:
            first_key = next(iter(self._tmp_interrupt_causes))
            raise self._tmp_interrupt_causes[first_key]
        # Call the wait callback if any
        if self._wait_callback is not None:
            self._wait_callback()

    def wait(self, timeout: float = None) -> Message:
        """
        Blocks until the event is set or until an exception must be raised (``InterruptException`` or
        ``TimeoutException``). ``InterruptException`` is raised if waiting for the event becomes
        irrelevant, for example, waiting for a checkpoint while the robot is in an error state.

        Parameters
        ----------
        timeout
            Maximum duration to wait in seconds.

        Return
        ------
        data
            Return the data object (or None for events not returning any data)
        """
        with self._cond:
            initial_set_count = self._set_count
            self.check_interrupted()
            if self._is_set or self._set_count != initial_set_count:
                return self._data

            start_time = time.monotonic()
            # Wait by smaller chunks to we can get interrupted by OS signals (like SIGINT)
            partial_timeout = 0.1 if timeout is None else min(timeout, 0.1)

            while True:
                # Wait until we're awakened
                self.check_interrupted()
                if self._is_set or self._set_count != initial_set_count:
                    return self._data

                self._cond.wait(timeout=partial_timeout)
                if timeout is not None and time.monotonic() - start_time > timeout:
                    raise TimeoutException("Timeout waiting for interruptable event")

    def set(self, data: Message = None):
        """Set the event and unblock all waits. Optionally modify data before setting.
        """
        with self._cond:
            self._data = data
            self._is_set = True
            self._set_count += 1
            self._cond.notify_all()

    def abort(self, message=""):
        """Unblock any waits and raise an exception.
        """
        with self._cond:
            if not self._is_set:
                self._interrupted_msg = message
                self._interrupted = True
                self._cond.notify_all()

    def add_tmp_interrupt(self, cause: str, exception: MecademicException):
        """
        Adds a temporary exception cause to this interruptable event.
        The event is considered interrupted until ``remove_tmp_interrupt`` is called with the same
        exception cause string. Only one temporary exception per cause is allowed.

        Parameters
        ----------
        cause
            Identifier for this interrupt. This avoids adding multiple time an interrupt for the same
            cause on this interruptable event.
        exception
            Exception instance to associate temporarily with this interruptable event.
        """
        with self._cond:
            # Add the interrupt cause to the dictionary
            self._tmp_interrupt_causes[cause] = exception
            self._cond.notify_all()

    def remove_tmp_interrupt(self, cause: str):
        """ Remove a temporary exception of the specified cause, if any. """
        with self._cond:
            if cause in self._tmp_interrupt_causes:
                del self._tmp_interrupt_causes[cause]

    def clear(self):
        """Reset the event to its initial state.
        """
        with self._cond:
            self._interrupted = False
            self._is_set = False
            self._tmp_interrupt_causes = {}

    def is_set(self) -> bool:
        """Checks if the event is set.

        Returns
        -------
        bool
            False if event is not set or instance should be `_interrupted`. True otherwise.
        """
        with self._cond:
            return self._is_set and not self._interrupted

    def clear_abort(self):
        """Clears the abort to make waiting for the event blocking again.

        """
        with self._cond:
            self._interrupted = False

    @property
    def id(self) -> int:
        """Make id a read-only property since it should not be changed after instantiation.

        """
        return self._id

    @property
    def data(self) -> Message:
        """Make data a read-only property and enforce that it is only assignable at construction or
        using ``set``.
        """
        return self._data


class RobotCopyable:

    def copy(self) -> Any:
        """Shallow copy using `replace`. Override `_copy_mutable_fields` if needed."""
        copied = replace(self)
        self._copy_mutable_fields(copied)
        return copied

    def _copy_mutable_fields(self, copied: Any):
        """Override in subclasses to deep-copy mutable fields."""
        pass

    def __deepcopy__(self, memo) -> Any:
        """Ensures ``copy.deepcopy`` uses the optimized copy method."""
        return self.copy()


@dataclass
class RobotVersion(RobotCopyable):
    """
    Robot utility class to handle firmware version.

    Attributes
    ----------
    build
        Firmware build number, None if unavailable
    extra
        Firmware version 'extra' name, None if unavailable
    full_version
        Full firmware version containing major.minor.patch.build-extra
    major
        Major firmware version value
    minor
        Minor firmware version value
    patch
        Patch firmware version value
    short_version
        Firmware version containing major.minor.patch only
    """
    full_version: str
    short_version: str
    major: int
    minor: int
    patch: int
    build: Optional[int] = None
    extra: Optional[str] = None

    REGEX_VERSION_BUILD = r"(?P<version>\d+\.\d+\.\d+)\.?(?P<build>\d+)?-?(?P<extra>.*)"

    def _copy_mutable_fields(self, copied: RobotVersion):
        # No mutable fields to copy in this class
        pass

    def __str__(self) -> str:
        return self.full_version

    def __lt__(self, other: RobotVersion) -> bool:
        """" Less than implementation
        """
        return not self.is_at_least(other.major, other.minor, other.patch, other.build)

    def get_str(self, build=False, extra=False) -> str:
        """
        Get version string.

        Parameters
        ----------
        build
            Include the build number in the version (e.g., 9.3.0.4739).
        extra
            Include the build extra in the version (e.g., 9.3.0.4739-master).

        Returns
        -------
        str
            Formatted version string.
        """
        version_to_return = self.short_version
        if build and self.build:
            version_to_return += f'.{self.build}'
        if extra and self.extra:
            version_to_return += f'-{self.extra}'
        return version_to_return

    @classmethod
    def from_version_string(cls, version_str: str) -> 'RobotVersion':
        """
        Build a ``RobotVersion`` object from a version string.

        Parameters
        ---------
        version_str
            String that contains the version (e.g., ``'v11.1.6.12310-official'``)

        Raises
        ------
        ValueError
            The string does not contain a valid version to parse

        Returns
        -------
        RobotVersion
            The built object that contains the parsed version
        """
        regex_version = re.search(cls.REGEX_VERSION_BUILD, version_str)
        if regex_version is None:
            raise ValueError(f'Invalid version format: "{version_str}"')

        short_version = regex_version.group("version") or "0.0.0"
        major, minor, patch = map(int, short_version.split("."))

        build = int(regex_version.group("build")) if regex_version.group("build") else None
        extra = regex_version.group("extra") if regex_version.group("extra") else None

        full_version = f"{short_version}"
        if build is not None:
            full_version += f".{build}"
        if extra:
            full_version += f"-{extra}"

        return cls(full_version=full_version,
                   short_version=short_version,
                   major=major,
                   minor=minor,
                   patch=patch,
                   build=build,
                   extra=extra)

    def is_at_least(self, major, minor=0, patch=0, build=0) -> bool:
        """
        Tells if this ``RobotInfo`` instance's version is at least the specified version.

        Parameters
        ----------
        major
            Minimum desired major version
        minor
            Minimum desired minor version
        patch
            Minimum desired patch version
        build
            Minimum desired build version

        Returns
        -------
        bool
            True if this ``RobotInfo`` instance's version is at least the specified version
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


@dataclass
class RobotInfo(RobotCopyable):
    """
    Class for storing metadata about a robot.

    Attributes
    ----------
    robot_model: MxRobotModel
        Model of robot
    model
        Model of robot as a string (legacy, use robot_model instead since it's an enum with well-known values)
    revision
        Robot revision.
    is_virtual
        True if is a virtual robot.
    is_safe_boot
        True if is booted in safe-boot mode.
    version
        robot firmware revision number as received from the connection string.
    serial
        Serial identifier of robot.
    ip_address
        IP address of this robot.
    rt_message_capable
        True if robot is capable of sending real-time monitoring messages.
    rt_on_ctrl_port_capable
        True if robot is capable of sending real-time monitoring messages on control port (SetCtrlPortMonitoring).
    mecascript_capable
        True if robot supports running the MecaScript (Python) engine.
    num_joints
        Number of joints on the robot.
    requires_homing
        Tells if this robot requires homing.
    supports_ext_tool
        Tells if this robot supports connecting external tools (gripper or valve box).
    supports_io_module
        Tells if this robot supports IO expansion module.
    supports_manual_mode
        Tells if this robot supports manual mode.
    gripper_pos_ctrl_capable
        Tells if this robot supports gripper position control.
    ext_tool_version_capable
        Tells if this robot supports external tool fw version fetch.
    ext_tool_version
        External tool firmware revision number as received from the connection string.
        Version 0.0.0.0 if device isn't connected or ext_tool_version_capable == False.
    supports_joint_vel_limit
        Tells if this robot supports SetJointVelLimit
    supports_set_payload
        Tells if this robot supports SetPayload
    supports_torque_limits
        Tells if this robot supports SetTorqueLimits and SetTorqueLimitsCfg
    supports_conf_turn
        Tells if this robot supports SetConfTurn
    supports_time_scaling
        Tells if this robot supports SetTimeScaling
    supports_checkpoint_discarded
        Tells if this robot supports reporting discarded checkpoints (MX_ST_CHECKPOINT_DISCARDED)
    supports_move_duration
        Tells if this robot supports time-based movements (SetMoveDuration, SetMoveMode, ...)
"""

    model: str = 'Unknown'
    revision: int = 0
    is_virtual: bool = False
    is_safe_boot: bool = False
    version: RobotVersion = field(default_factory=lambda: RobotVersion.from_version_string('0.0.0'))
    serial: str = ''
    ext_tool_version: RobotVersion = field(default_factory=lambda: RobotVersion.from_version_string('0.0.0.0'))
    ip_address: Optional[str] = None

    # Set in __post_init__
    robot_model: MxRobotModel = field(init=False)
    num_joints: int = field(init=False)
    requires_homing: bool = field(init=False)
    supports_ext_tool: bool = field(init=False)
    rt_message_capable: bool = field(init=False, default=False)
    rt_on_ctrl_port_capable: bool = field(init=False, default=False)
    mecascript_capable: bool = field(init=False, default=False)
    gripper_pos_ctrl_capable: bool = field(init=False, default=False)
    ext_tool_version_capable: bool = field(init=False, default=False)
    supports_io_module: bool = field(init=False, default=False)
    supports_manual_mode: bool = field(init=False, default=False)
    supports_joint_vel_limit: bool = field(init=False, default=False)
    supports_set_payload: bool = field(init=False, default=False)
    supports_torque_limits: bool = field(init=False, default=False)
    supports_conf_turn: bool = field(init=False, default=False)
    supports_time_scaling: bool = field(init=False, default=False)
    supports_checkpoint_discarded: bool = field(init=False, default=False)
    supports_move_duration: bool = field(init=False, default=False)

    def _copy_mutable_fields(self, copied: RobotInfo):
        copied.version = self.version.copy()  # Make sure to create a copy instead of a reference
        copied.ext_tool_version = self.ext_tool_version.copy()  # Make sure to create a copy instead of a reference

    def __post_init__(self):
        model_upper = self.model.upper()

        if model_upper == MX_ROBOT_MODEL_OFFICIAL_NAME_M500.upper():
            if self.revision == 1:
                self.robot_model = MxRobotModel.MX_ROBOT_MODEL_M500_R1
            elif self.revision == 2:
                self.robot_model = MxRobotModel.MX_ROBOT_MODEL_M500_R2
            elif self.revision == 3:
                self.robot_model = MxRobotModel.MX_ROBOT_MODEL_M500_R3
            elif self.revision == 4:
                self.robot_model = MxRobotModel.MX_ROBOT_MODEL_M500_R4
            else:
                self.robot_model = MxRobotModel.MX_ROBOT_MODEL_UNKNOWN
            self.num_joints = 6
            self.requires_homing = True
            self.supports_ext_tool = True
        elif model_upper == MX_ROBOT_MODEL_OFFICIAL_NAME_MCS500.upper():
            self.robot_model = MxRobotModel.MX_ROBOT_MODEL_MCS500_R1
            self.num_joints = 4
            self.requires_homing = False
            self.supports_ext_tool = False
            self.supports_io_module = True
            self.supports_manual_mode = True
        elif model_upper == MX_ROBOT_MODEL_OFFICIAL_NAME_MCA250.upper():
            self.robot_model = MxRobotModel.MX_ROBOT_MODEL_MCA250_R1
            self.num_joints = 6
            self.requires_homing = False
            self.supports_ext_tool = False
            self.supports_io_module = True
            self.supports_manual_mode = True
        elif model_upper == MX_ROBOT_MODEL_OFFICIAL_NAME_MCA1000.upper():
            self.robot_model = MxRobotModel.MX_ROBOT_MODEL_MCA1000_R1
            self.num_joints = 6
            self.requires_homing = False
            self.supports_ext_tool = False
            self.supports_io_module = True
            self.supports_manual_mode = True
        elif model_upper == 'UNKNOWN':
            self.robot_model = MxRobotModel.MX_ROBOT_MODEL_UNKNOWN
            self.num_joints = 1
            self.requires_homing = False
            self.supports_ext_tool = False
        else:
            self.robot_model = MxRobotModel.MX_ROBOT_MODEL_UNKNOWN
            raise ValueError(f'Invalid robot model: {self.model}')

        # Capabilities based on version
        if self.version.is_at_least(8, 4):
            self.rt_message_capable = True
        if self.version.is_at_least(9, 0):
            self.rt_on_ctrl_port_capable = True
        if self.version.is_at_least(12, 1, 0):
            self.mecascript_capable = True
        if self.version.is_at_least(9, 1):
            self.gripper_pos_ctrl_capable = True
        if self.version.is_at_least(9, 1, 5):
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
        """
        Generate robot information from standard robot connection response string.

        String format should be "Connected to {model} R{revision}{-virtual} v{fw_major_num}.{fw_minor_num}.{patch_num}"

        Parameters
        ----------
        input_string
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
                       version=RobotVersion.from_version_string(robot_info_regex.group('version')))
        except Exception as exception:
            raise ValueError(f'Could not parse robot info string "{input_string}", error: {exception}') from exception

    @tools.deprecation.deprecated(
        deprecated_in="2.4.0",
        removed_in="3.0.0",
        current_version=tools.get_mecademicpy_version(),
        details="The serial number should always be considered a string with letters and digits")
    def get_serial_digit(self) -> int:
        """"
        Returns robot serial digits.

        For example, for Meca500, a serial number M500-0123 is converted to 123.

        Returns
        -------
        int
            Robot serial number digits
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
        """
        Returns robot serial digits.

        For example, a serial number M500-0123 is converted to 123.

        Returns
        -------
        int
            Robot serial number digits
        """
        serial_split = self.serial.split('-')

        if len(serial_split) != 2:
            raise ValueError(f'Invalid serial number string received: {self.serial}, expecting "M500-1234"')

        return int(serial_split[1])


@dataclass
class UpdateProgress(RobotCopyable):
    """Class for storing the robot's firmware update status.

    Attributes
    ----------
    in_progress
        Firmware update is in progress
    complete
        Firmware update has completed
    version
        The firmware version being installed
    error
        Tells if the update failed
    error_msg
        Message that explains why the update failed
    progress
        Update progress message received from robot
    step
        String that describes the current firmware update step being performed

    _last_print_timestamp
        Last time we have printed the firmware update status
"""
    in_progress: bool = False
    complete: bool = False
    version: str = ""
    error: bool = False
    error_msg: str = ""
    progress: float = 0.0
    progress_str: str = ""
    step: str = ""
    _last_print_timestamp: float = field(default=0.0, repr=False, compare=False)

    def _copy_mutable_fields(self, copied: UpdateProgress):
        # No mutable fields to copy in this class
        pass


@dataclass
class TimestampedData(RobotCopyable):
    """
    Class for storing timestamped data (real-time data received from the robot).

    Attributes
    ----------
    timestamp
        Monotonic timestamp associated with data (in microseconds since robot last reboot)
        This timestamp is stamped by the robot so it is not affected by host/network jitter
    data
        Data to be stored.
    update_type
        Update type of the ``TimestampedData``. Refer to ``RtDataUpdateType`` for the various
        update types.
"""
    timestamp: int
    data: List[float]
    update_type: RtDataUpdateType
    enabled: Optional[bool] = None
    _updated: bool = True  # Set when data was updated (at construction or upon set_data), until 'copy_from_if_updated'

    def __post_init__(self):
        if self.enabled is None:
            is_enabled = self.update_type not in {
                RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL,
                RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_UNAVAILABLE
            }
            object.__setattr__(self, 'enabled', is_enabled)

    def __str__(self):
        return str([self.timestamp] + self.data)

    def __repr__(self):
        return str(self)

    def _copy_mutable_fields(self, copied: TimestampedData):
        copied.data = self.data.copy()  # Make sure to create a copy instead of a reference

    def set_enabled(self, enabled: bool):
        """ Set the enabled flag  """
        self.enabled = enabled

    def set_timestamp(self, timestamp: int):
        """ Set the timestamp """
        self.timestamp = timestamp
        self._updated = True

    def set_data(self, data: List[float]):
        """ Set the data """
        self.data = data
        self._updated = True

    def was_updated(self):
        """ Tell if this ``TimestampedData`` was updated since last call to clear_updated """
        return self._updated

    def clear_updated(self):
        """ Clear the '_updated' flag """
        self._updated = False

    def clear_if_disabled(self):
        """Clear timestamp and data if not reported by the robot (not part of enabled real-time monitoring events)
        """
        if not self.enabled:
            self.set_timestamp(0)
            self.set_data([0.] * len(self.data))

    def update_from_csv(self, input_string: str, allowed_nb_val: list[int] = None):
        """
        Update from comma-separated string, only if timestamp is newer.

        Parameters
        ----------
        input_string
            Comma-separated string. First value is timestamp, rest is data.
        allowed_nb_val
            List of accepted number of values. If not provided, `input_string` must contain at
            least as many values as current contents of ``TimestampedData.data``.
        """
        numbs = tools.string_to_numbers(input_string)
        nb_values = len(numbs) - 1

        if allowed_nb_val is None:
            if nb_values < len(self.data):
                raise ValueError(f'Cannot update TimestampedData, too few values received ({nb_values}).')
            elif nb_values > len(self.data):
                numbs = numbs[0:len(self.data) + 1]
        else:
            if nb_values not in allowed_nb_val:
                raise ValueError(f'Cannot update TimestampedData, incorrect number of values received ({nb_values}).')

        if numbs[0] >= self.timestamp:
            self.timestamp = numbs[0]
            self.data = numbs[1:]
            self.enabled = True
            self._updated = True

    def update_from_data(self, timestamp: int, data: list[float]):
        """
        Update with data unless timestamp is older.

        Parameters
        ----------
        timestamp
            Timestamp associated with data.
        data
            Data to be stored if timestamp is newer.
        """
        if timestamp >= self.timestamp:
            self.timestamp = timestamp
            self.data = data
            self.enabled = True
            self._updated = True

    @classmethod
    def zeros(cls, length: int, update_type: RtDataUpdateType) -> TimestampedData:
        """
        Construct empty ``TimestampedData`` object of specified length.

        Parameters
        ----------
        length
            Length of data to construct.
        update_type
            Update type of monitored data. Refer to ``RtDataUpdateType`` for the various update types.

        Returns
        -------
        TimestampedData
            A TimestampedData with all values set to 0
        """
        return cls(0, [0.] * length, update_type)

    def __eq__(self, other):
        """
        Returns True if the other object has an identical timestamp and data.

        Parameters
        ----------
        other
            Object to compare against.

        Return
        ------
        bool
            True if both objects have the same timestamp and data.
        """
        if isinstance(other, TimestampedData):
            return (other.timestamp == self.timestamp and other.data == self.data
                    and other.update_type == self.update_type)
        else:
            return self.data == other

    def __ne__(self, other):
        """
        Returns True if the other object has a different timestamp or data.

        Parameters
        ----------
        other
            Object to compare against.

        Return
        ------
        bool
            True if the objects have different timestamps or data.
        """
        return not self == other


@dataclass
class RobotRtDataField(RobotCopyable):
    """
    Represents a robot real-time data field that can be enabled using
    ``SetRealTimeMonitoring()``, retrieved in ``RobotRtData``, and captured with
    ``StartLogging()``.

    Parameters
    ----------
    field_name
        Name of this field in the ``RobotRtData`` class.
        Example: rt_target_joint_pos
    status_code
        Robot status code that provides this value.
        This can be used as an argument for ``SetRealTimeMonitoring`` or
        ``StartLogging`` to select which fields to capture.
        Example: MxRobotStatusCode.MX_ST_RT_TARGET_JOINT_POS
    col_prefix
        Column name prefix for this real-time data when stored in a captured dataframe.
        The column name suffix depends on the data type: for joint positions, it is the joint
        number; for Cartesian positions, it may be x, y, z, alpha, etc.
    """

    field_name: str
    status_code: MxRobotStatusCode
    col_prefix: str


ROBOT_RT_DATA_FIELDS: list[RobotRtDataField] = [
    #         RobotRtData attribute name,   robot status code,                           captured column name prefix
    RobotRtDataField('rt_target_joint_pos', MxRobotStatusCode.MX_ST_RT_TARGET_JOINT_POS, 'TargetJointPos'),
    RobotRtDataField('rt_target_cart_pos', MxRobotStatusCode.MX_ST_RT_TARGET_CART_POS, 'TargetCartPos'),
    RobotRtDataField('rt_target_joint_vel', MxRobotStatusCode.MX_ST_RT_TARGET_JOINT_VEL, 'TargetJointVel'),
    RobotRtDataField('rt_target_joint_torq', MxRobotStatusCode.MX_ST_RT_TARGET_JOINT_TORQ, 'TargetJointTorq'),
    RobotRtDataField('rt_target_cart_vel', MxRobotStatusCode.MX_ST_RT_TARGET_CART_VEL, 'TargetCartVel'),
    RobotRtDataField('rt_target_conf', MxRobotStatusCode.MX_ST_RT_TARGET_CONF, 'TargetConf'),
    RobotRtDataField('rt_target_conf_turn', MxRobotStatusCode.MX_ST_RT_TARGET_CONF_TURN, 'TargetConfTurn'),
    RobotRtDataField('rt_joint_pos', MxRobotStatusCode.MX_ST_RT_JOINT_POS, 'JointPos'),
    RobotRtDataField('rt_cart_pos', MxRobotStatusCode.MX_ST_RT_CART_POS, 'CartPos'),
    RobotRtDataField('rt_joint_vel', MxRobotStatusCode.MX_ST_RT_JOINT_VEL, 'JointVel'),
    RobotRtDataField('rt_joint_torq', MxRobotStatusCode.MX_ST_RT_JOINT_TORQ, 'JointTorq'),
    RobotRtDataField('rt_cart_vel', MxRobotStatusCode.MX_ST_RT_CART_VEL, 'CartVel'),
    RobotRtDataField('rt_conf', MxRobotStatusCode.MX_ST_RT_CONF, 'Conf'),
    RobotRtDataField('rt_conf_turn', MxRobotStatusCode.MX_ST_RT_CONF_TURN, 'ConfTurn'),
    RobotRtDataField('rt_abs_joint_pos', MxRobotStatusCode.MX_ST_RT_ABS_JOINT_POS, 'AbsJointPos'),
    RobotRtDataField('rt_accelerometer', MxRobotStatusCode.MX_ST_RT_ACCELEROMETER, 'Accel'),
    RobotRtDataField('rt_effective_time_scaling', MxRobotStatusCode.MX_ST_RT_EFFECTIVE_TIME_SCALING,
                     'EffectiveTimeScaling'),
    RobotRtDataField('rt_vl', MxRobotStatusCode.MX_ST_RT_VL, 'Vl'),
    RobotRtDataField('rt_vm', MxRobotStatusCode.MX_ST_RT_VM, 'Vm'),
    RobotRtDataField('rt_current', MxRobotStatusCode.MX_ST_RT_CURRENT, 'Current'),
    RobotRtDataField('rt_temperature', MxRobotStatusCode.MX_ST_RT_TEMPERATURE, 'Temperature'),
    RobotRtDataField('rt_i2t', MxRobotStatusCode.MX_ST_RT_I2T, 'I2t'),
    RobotRtDataField('rt_wrf', MxRobotStatusCode.MX_ST_RT_WRF, 'Wrf'),
    RobotRtDataField('rt_trf', MxRobotStatusCode.MX_ST_RT_TRF, 'Trf'),
    RobotRtDataField('rt_checkpoint', MxRobotStatusCode.MX_ST_RT_CHECKPOINT, 'Checkpoint'),
    RobotRtDataField('rt_external_tool_status', MxRobotStatusCode.MX_ST_RT_EXTTOOL_STATUS, 'ExtToolStatus'),
    RobotRtDataField('rt_valve_state', MxRobotStatusCode.MX_ST_RT_VALVE_STATE, 'ValveState'),
    RobotRtDataField('rt_gripper_state', MxRobotStatusCode.MX_ST_RT_GRIPPER_STATE, 'GripperState'),
    RobotRtDataField('rt_gripper_force', MxRobotStatusCode.MX_ST_RT_GRIPPER_FORCE, 'GripperForce'),
    RobotRtDataField('rt_gripper_pos', MxRobotStatusCode.MX_ST_RT_GRIPPER_POS, 'GripperPos'),
    RobotRtDataField('rt_io_module_status', MxRobotStatusCode.MX_ST_RT_IO_STATUS, 'IoModuleStatus'),
    RobotRtDataField('rt_io_module_outputs', MxRobotStatusCode.MX_ST_RT_OUTPUT_STATE, 'IoModuleOutputState'),
    RobotRtDataField('rt_io_module_inputs', MxRobotStatusCode.MX_ST_RT_INPUT_STATE, 'IoModuleInputState'),
    RobotRtDataField('rt_sig_gen_status', MxRobotStatusCode.MX_ST_RT_IO_STATUS, 'SigGenStatus'),
    RobotRtDataField('rt_sig_gen_outputs', MxRobotStatusCode.MX_ST_RT_OUTPUT_STATE, 'SigGenOutputState'),
    RobotRtDataField('rt_sig_gen_inputs', MxRobotStatusCode.MX_ST_RT_INPUT_STATE, 'SigGenInputState'),
    RobotRtDataField('rt_vacuum_state', MxRobotStatusCode.MX_ST_RT_VACUUM_STATE, 'VacuumState'),
    RobotRtDataField('rt_vacuum_pressure', MxRobotStatusCode.MX_ST_RT_VACUUM_PRESSURE, 'VacuumPressure'),
    # Should not be used, handled by Robot class when it uses the logger
    RobotRtDataField('', MxRobotStatusCode.MX_ST_RT_CYCLE_END, 'CycleEnd'),
]
"""Map that returns the ``RobotRtDataField`` object corresponding to a field name in ``RobotRtData`` class """
rt_data_field_by_name: dict[str, RobotRtDataField] = {}
rt_data_field_by_status_code: dict[MxRobotStatusCode, RobotRtDataField] = {}
rt_data_field_by_col_name: dict[str, RobotRtDataField] = {}

# Populate the maps above
for robot_rt_data_field in ROBOT_RT_DATA_FIELDS:
    # Fill map by field name
    if robot_rt_data_field.field_name in rt_data_field_by_name:
        raise ValueError(f'Duplicate field name {robot_rt_data_field.field_name} in robot_rt_data_field')
    else:
        rt_data_field_by_name[robot_rt_data_field.field_name] = robot_rt_data_field

    # Fill map by status code
    if robot_rt_data_field.status_code in rt_data_field_by_status_code:
        # Note: These are indeed duplicates, but it's ok
        if (robot_rt_data_field.status_code != MxRobotStatusCode.MX_ST_RT_IO_STATUS
                and robot_rt_data_field.status_code != MxRobotStatusCode.MX_ST_RT_OUTPUT_STATE
                and robot_rt_data_field.status_code != MxRobotStatusCode.MX_ST_RT_INPUT_STATE):
            raise ValueError(f'Duplicate status code {robot_rt_data_field.status_code} in robot_rt_data_field')
    else:
        rt_data_field_by_status_code[robot_rt_data_field.status_code] = robot_rt_data_field

    # Fill map by col prefix
    if robot_rt_data_field.col_prefix in rt_data_field_by_col_name:
        raise ValueError(f'Duplicate col prefix {robot_rt_data_field.col_prefix} in robot_rt_data_field')
    else:
        rt_data_field_by_col_name[robot_rt_data_field.col_prefix] = robot_rt_data_field


def normalize_robot_rt_data_field(field_list: list[Union[MxRobotStatusCode, str]]) -> list[MxRobotStatusCode]:
    """
    Normalizes a list of fields to capture (from ``SetRealTimeMonitoring`` or ``StartLogging``)
    into a list of robot status codes.

    Parameters
    ----------
    field_list
        List of fields to capture, containing either:

          - an ``MxRobotStatusCode`` object,
          - a string representing the corresponding field name in the ``RobotRtData`` class, or
          - a captured file column prefix, as defined in ``ROBOT_RT_DATA_FIELDS``.

    Returns
    -------
    list of MxRobotStatusCode
        The normalized list of robot status codes.
    """
    fields_normalized: list[MxRobotStatusCode] = []
    if isinstance(field_list, str) and field_list.lower() == 'all':
        return [field.status_code for field in ROBOT_RT_DATA_FIELDS if field.field_name]
    for event in field_list:
        if isinstance(event, MxRobotStatusCode):
            fields_normalized.append(event)
        elif event in rt_data_field_by_name:
            fields_normalized.append(rt_data_field_by_name[event].status_code)
        elif event in rt_data_field_by_col_name:
            fields_normalized.append(rt_data_field_by_col_name[event].status_code)
        else:
            fields_normalized.append(event)
    return fields_normalized


@dataclass
class RobotRtData(RobotCopyable):
    """
    Holds real-time internal data from a Mecademic robot, updated automatically as the robot
    reports new values.

    This class provides a consistent snapshot of various internal robot states, updated
    either cyclically or through event-based notifications. While many individual values can
    be queried via specific API calls such as ``GetRtTargetJointPos``, using ``GetRobotRtData``
    ensures that all fields are captured atomically, i.e., at the same instant in time.

    Notes
    -----
    **Data availability**
    Not all real-time data fields are reported by the robot by default. To control which
    optional fields are available:

      - Use ``SetRealTimeMonitoring`` to enable specific data streams.
      - Use the ``TimestampedData.enabled`` property of each field to verify whether the
        data is currently being reported.

    Some fields (such as ``rt_target_joint_pos``) are always enabled. Others require explicit
    activation. The description of each field below indicates which are optional.

    **Update types**
    Fields in this class fall into two categories based on how the robot updates them:

      - Event-based updates (``MX_RT_DATA_UPDATE_TYPE_EVENT_BASED``):
        The robot pushes updates when values change.
        Example fields: ``rt_external_tool_status``, ``rt_wrf``, ``rt_checkpoint``.
      - Cyclic updates (``MX_RT_DATA_UPDATE_TYPE_CYCLICAL`` or
        ``MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``):
        The robot sends updated values at a fixed rate.
        Example fields: ``rt_joint_pos``, ``rt_cart_vel``.

    **Cyclic data interval**
    By default, the robot sends cyclic data 60 times per second (every 16.6 ms). This
    interval can be adjusted to any value between 1 and 1000 ms using
    ``SetMonitoringInterval``.

    Attributes
    ----------
    cycle_count
        Number of real-time updates received so far. Increments each time new data is
        received at the configured interval.

    rt_target_joint_pos
        Desired joint positions from the controller [theta_1...6] in degrees.
        Always available, type ``MX_RT_DATA_UPDATE_TYPE_CYCLICAL``.
        Also available with``GetRtTargetJointPos``.

    rt_target_cart_pos
        Desired end effector pose [x, y, z, alpha, beta, gamma].
        Always available, type ``MX_RT_DATA_UPDATE_TYPE_CYCLICAL``.
        Also available with ``GetRtTargetCartPos``.

    rt_target_joint_vel
        Desired joint velocities [j_1...6] in deg/s.
        Not enabled by default, type ``MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``.
        Enable via ``SetRealTimeMonitoring`` with ``MX_ST_RT_TARGET_JOINT_VEL``.
        Also available with ``GetRtTargetJointVel``.

    rt_target_cart_vel
        Desired end effector velocity: [x, y, z] mm/s linear and [omega_x, omega_y, omega_z]deg/s angular.
        Not enabled by default, type``MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``.
        Also available with``GetRtTargetCartVel``.

    rt_target_joint_torq
        Estimated joint torque as % of max per joint [tau_1...6].
        Not enabled by default,type ``MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``.
        Also available with ``GetRtJointTorq``.

    rt_target_conf
        Joint configuration that corresponds to desired joint positions.
        Always available, type ``MX_RT_DATA_UPDATE_TYPE_EVENT_BASED``.
        Also available with ``GetRtTargetConf``.

    rt_target_conf_turn
        Joint turn numbers corresponding to desired joint positions.
        Always available, type ``MX_RT_DATA_UPDATE_TYPE_EVENT_BASED``.
        Also available with ``GetRtTargetConfTurn``.

    rt_joint_pos
        Actual joint positions from drives [j_1...6] in degrees.
        Not enabled by default, type ``MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``.
        Also available with ``GetRtJointPos``.

    rt_cart_pos
        Actual end effector pose [x, y, z, alpha, beta, gamma].
        Not enabled by default, type ``MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``.
        Also available with ``GetRtCartPos``.

    rt_joint_vel
        Actual joint velocities [j_1...6] in deg/s.
        Not enabled by default, type ``MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``.
        Also available with ``GetRtJointVel``.

    rt_joint_torq
        Actual joint torque ratio [% of max].
        Not enabled by default, type ``MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``.
        Also available with ``GetRtJointTorq``.

    rt_cart_vel
        Actual end effector velocity: [x, y, z] mm/s and [omega_x, omega_y, omega_z] deg/s.
        Not enabled by default, type ``MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``.
        Also available with ``GetRtCartVel``.

    rt_conf
        Joint configuration corresponding to measured joint positions.
        Not enabled by default, type ``MX_RT_DATA_UPDATE_TYPE_EVENT_BASED``.
        Also available with ``GetRtConf``.

    rt_conf_turn
        Joint turn numbers corresponding to measured joint positions.
        Not enabled by default, type ``MX_RT_DATA_UPDATE_TYPE_EVENT_BASED``.
        Also available with ``GetRtConfTurn``.

    rt_accelerometer
        Raw accelerometer values [sensor_id, x, y, z].
        Unit: 16000 = 1g.
        Not enabled by default, type ``MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``.
        Also available with ``GetRtAccelerometer``.

    rt_effective_time_scaling
        Time scaling ratio (e.g., for slow-motion execution).
        Not enabled by default, type ``MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``.

    rt_vl
        VL (logic voltage) readings from components.
        Format: [Baseboard VL, PSU VL, Safe MCU VL, Drive 1 VL, ..., Drive N VL].
        Not enabled by default, type ``MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``.

    rt_vm
        VM (motor voltage) readings from components.
        Format: [Baseboard VM, PSU VM, Safe MCU VM, Drive 1 VM, ..., Drive N VM].
        Not enabled by default, type ``MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``.

    rt_current
        Motor current readings [Baseboard current,].
        Not enabled by default, type ``MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``.

    rt_temperature
        Robot temperature readings in Celsius [Baseboard, PSU, Safe MCU, Drive 1, ... Drive N].
        Not available on the Meca500 robots.
        Not enabled by default, type ``MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``.

    rt_i2t
        Motor i2t readings [Drive 1, ... Drive N].
        This is a measure of how close to overload the motor is. The motor robot will enter motion error if one or
        more motors have an i2t value greater or equal to 0 (negative values mean the motor is OK).
        When a motor is getting near overload, the robot will report event MX_ST_DRIVES_NEAR_OVERLOAD.
        Not available on the Meca500 robots.
        Not enabled by default, type ``MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``.

    rt_external_tool_status
        Tool status: [sim_tool_type, physical_tool_type, homing_state, error_status, overload_error].
        Always available, type ``MX_RT_DATA_UPDATE_TYPE_EVENT_BASED``.
        Also available with ``GetRtExtToolStatus``.

    rt_valve_state
        Pneumatic valve state [valve_opened[0], valve_opened[1]].
        Always available, type ``MX_RT_DATA_UPDATE_TYPE_EVENT_BASED``.
        Also available with ``GetRtValveState``.

    rt_gripper_state
        Gripper status: [holding_part, target_pos_reached, opened, closed].
        Always available, type ``MX_RT_DATA_UPDATE_TYPE_EVENT_BASED``.
        Also available with ``GetRtGripperState``.

    rt_gripper_force
        Gripper force as % of max.
        Not enabled by default, type ``MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``.
        Also available with ``GetRtGripperForce``.

    rt_gripper_pos
        Gripper position in mm.
        Not enabled by default, type ``MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``.
        Also available with ``GetRtGripperPos``.

    rt_io_module_status
        IO module status: [bank_id, present, sim_mode, error_code].
        Always available, type ``MX_RT_DATA_UPDATE_TYPE_EVENT_BASED``.
        Also available with ``GetRtIoStatus``.

    rt_io_module_outputs
        Digital output states: [output[0], output[1], ...].
        Always available, type ``MX_RT_DATA_UPDATE_TYPE_EVENT_BASED``.
        Also available with ``GetRtOutputState``.

    rt_io_module_inputs
        Digital input states: [input[0], input[1], ...].
        Always available, type ``MX_RT_DATA_UPDATE_TYPE_EVENT_BASED``.
        Also available with ``GetRtInputState``.

    rt_vacuum_state
        Vacuum gripper state: [vacuum_on, purge_on, holding_part].
        Always available, type ``MX_RT_DATA_UPDATE_TYPE_EVENT_BASED``.
        Also available with ``GetRtVacuumState``.

    rt_vacuum_pressure
        Vacuum pressure in kPa.
        Not enabled by default, type ``MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``.
        Also available with ``GetRtVacuumPressure``.

    rt_wrf
        WRF (World Reference Frame) definition w.r.t.
        BRF (Base Reference Frame).
        Format: [x, y, z, omega_x, omega_y, omega_z].
        Units: mm and degrees.
        Always available, type ``MX_RT_DATA_UPDATE_TYPE_EVENT_BASED``.
        Also available with ``GetRtWrf``.

    rt_trf
        TRF (Tool Reference Frame) definition w.r.t.
        FRF (Flange Reference Frame).
        Format: [x, y, z, omega_x, omega_y, omega_z].
        Units: mm and degrees.
        Always available, type ``MX_RT_DATA_UPDATE_TYPE_EVENT_BASED``.
        Also available with ``GetRtTrf``.

    rt_checkpoint
        Last executed checkpoint ID with timestamp.
        Always available, type ``MX_RT_DATA_UPDATE_TYPE_EVENT_BASED``.
        Also available with ``GetCheckpoint``.

    """
    num_joints: InitVar[int]

    rt_target_joint_pos: TimestampedData = field(init=False)  # degrees
    rt_target_cart_pos: TimestampedData = field(init=False)  # mm and degrees
    rt_target_joint_vel: TimestampedData = field(init=False)  # degrees/second
    rt_target_cart_vel: TimestampedData = field(init=False)  # mm/s and deg/s
    rt_target_joint_torq: TimestampedData = field(init=False)  # percent of maximum
    rt_target_conf: TimestampedData = field(init=False)
    rt_target_conf_turn: TimestampedData = field(init=False)
    rt_joint_pos: TimestampedData = field(init=False)  # degrees
    rt_cart_pos: TimestampedData = field(init=False)  # mm and degrees
    rt_joint_vel: TimestampedData = field(init=False)  # degrees/second
    rt_joint_torq: TimestampedData = field(init=False)  # percent of maximum
    rt_cart_vel: TimestampedData = field(init=False)  # mm/s and deg/s
    rt_conf: TimestampedData = field(init=False)
    rt_conf_turn: TimestampedData = field(init=False)
    rt_effective_time_scaling: TimestampedData = field(init=False)  # effective time scaling ratio
    rt_vl: TimestampedData = field(init=False)  # Baseboard VL, Psu VL, SafeMcu VL, Drive 1 VL, Drive 2 VL, ...
    rt_vm: TimestampedData = field(init=False)  # Baseboard VM, Psu VM, SafeMcu Vm, Drive 1 VM, Drive 2 Vm, ...
    rt_current: TimestampedData = field(init=False)  # Baseboard current
    rt_temperature: TimestampedData = field(init=False)  # Baseboard, PSU, SafeMcu, Drive 1, Drive 2, ...
    rt_i2t: TimestampedData = field(init=False)  # For each drive
    rt_abs_joint_pos: TimestampedData = field(init=False)  # degrees (less-precise encoders)
    rt_accelerometer: Dict[int, TimestampedData] = field(default_factory=dict)  # 16000 = 1g
    rt_external_tool_status: TimestampedData = field(init=False)  # sim type, physical  type, homed, error, overload
    rt_valve_state: TimestampedData = field(init=False)  # valve1 opened, valve2 opened
    rt_gripper_state: TimestampedData = field(init=False)  # holding part, target pos reached, closed, opened
    rt_gripper_force: TimestampedData = field(init=False)  # gripper force [%]
    rt_gripper_pos: TimestampedData = field(init=False)  # gripper position [mm]
    rt_io_module_status: TimestampedData = field(init=False)
    rt_io_module_outputs: TimestampedData = field(init=False)
    rt_io_module_inputs: TimestampedData = field(init=False)
    rt_vacuum_state: TimestampedData = field(init=False)  # vacuum on/off, purge on/off, holding
    rt_vacuum_pressure: TimestampedData = field(init=False)  # vacuum pressure [kPa]
    rt_sig_gen_status: TimestampedData = field(init=False)
    rt_sig_gen_outputs: TimestampedData = field(init=False)
    rt_sig_gen_inputs: TimestampedData = field(init=False)
    rt_wrf: TimestampedData = field(init=False)  # mm and degrees
    rt_trf: TimestampedData = field(init=False)  # mm and degrees
    rt_checkpoint: TimestampedData = field(init=False)  # checkpointId
    cycle_count: int = field(default=0)

    def __post_init__(self, num_joints: int):
        nb_cart_val = num_joints
        nb_conf_val = 3 if num_joints == 6 else 1

        # Aliases to make code below more concise
        CYCLICAL = RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_CYCLICAL
        CYCLICAL_OPT = RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL
        EVENT_BASED = RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_EVENT_BASED

        self.rt_target_joint_pos = TimestampedData.zeros(num_joints, CYCLICAL)
        self.rt_target_cart_pos = TimestampedData.zeros(nb_cart_val, CYCLICAL)
        self.rt_target_joint_vel = TimestampedData.zeros(num_joints, CYCLICAL_OPT)
        self.rt_target_cart_vel = TimestampedData.zeros(nb_cart_val, CYCLICAL_OPT)
        self.rt_target_joint_torq = TimestampedData.zeros(num_joints, CYCLICAL_OPT)
        self.rt_target_conf = TimestampedData.zeros(nb_conf_val, EVENT_BASED)
        self.rt_target_conf_turn = TimestampedData.zeros(1, EVENT_BASED)

        self.rt_joint_pos = TimestampedData.zeros(num_joints, CYCLICAL_OPT)
        self.rt_cart_pos = TimestampedData.zeros(nb_cart_val, CYCLICAL_OPT)
        self.rt_joint_vel = TimestampedData.zeros(num_joints, CYCLICAL_OPT)
        self.rt_joint_torq = TimestampedData.zeros(num_joints, CYCLICAL_OPT)
        self.rt_cart_vel = TimestampedData.zeros(nb_cart_val, CYCLICAL_OPT)
        self.rt_conf = TimestampedData.zeros(nb_conf_val, EVENT_BASED)
        self.rt_conf_turn = TimestampedData.zeros(1, EVENT_BASED)

        self.rt_effective_time_scaling = TimestampedData.zeros(1, CYCLICAL_OPT)
        self.rt_vl = TimestampedData.zeros(num_joints + 3, CYCLICAL_OPT)
        self.rt_vm = TimestampedData.zeros(num_joints + 3, CYCLICAL_OPT)
        self.rt_current = TimestampedData.zeros(1, CYCLICAL_OPT)
        self.rt_current = TimestampedData.zeros(1, CYCLICAL_OPT)
        self.rt_temperature = TimestampedData.zeros(num_joints + 3, CYCLICAL_OPT)
        self.rt_i2t = TimestampedData.zeros(num_joints, CYCLICAL_OPT)

        self.rt_abs_joint_pos = TimestampedData.zeros(num_joints, CYCLICAL_OPT)

        self.rt_external_tool_status = TimestampedData.zeros(5, EVENT_BASED)
        self.rt_valve_state = TimestampedData.zeros(MX_EXT_TOOL_MPM500_NB_VALVES, EVENT_BASED)
        self.rt_gripper_state = TimestampedData.zeros(4, EVENT_BASED)
        self.rt_gripper_force = TimestampedData.zeros(1, CYCLICAL_OPT)
        self.rt_gripper_pos = TimestampedData.zeros(1, CYCLICAL_OPT)

        self.rt_io_module_status = TimestampedData.zeros(4, EVENT_BASED)
        self.rt_io_module_outputs = TimestampedData.zeros(0, EVENT_BASED)
        self.rt_io_module_inputs = TimestampedData.zeros(0, EVENT_BASED)
        self.rt_vacuum_state = TimestampedData.zeros(3, EVENT_BASED)
        self.rt_vacuum_pressure = TimestampedData.zeros(1, CYCLICAL_OPT)

        self.rt_sig_gen_status = TimestampedData.zeros(4, EVENT_BASED)
        self.rt_sig_gen_outputs = TimestampedData.zeros(0, EVENT_BASED)
        self.rt_sig_gen_inputs = TimestampedData.zeros(0, EVENT_BASED)

        self.rt_wrf = TimestampedData.zeros(nb_cart_val, EVENT_BASED)
        self.rt_trf = TimestampedData.zeros(nb_cart_val, EVENT_BASED)
        self.rt_checkpoint = TimestampedData.zeros(1, EVENT_BASED)

    def copy(self) -> RobotRtData:
        """ Returns a copy (avoiding returning references to mutable members) """
        # Create a blank object that avoids
        copied = object.__new__(RobotRtData)

        # Manually copy each field
        for field_name, field_to_copy in self.__dict__.items():
            if isinstance(field_to_copy, TimestampedData):
                setattr(copied, field_name, field_to_copy.copy())  # fast manual copy
            elif isinstance(field_to_copy, dict):
                # e.g., rt_accelerometer: dict[int, TimestampedData]
                setattr(copied, field_name, {k: v.copy() for k, v in field_to_copy.items()})
            else:
                setattr(copied, field_name, field_to_copy)  # immutable fields, shallow copy OK

        return copied

    def increment_cycle_count(self):
        object.__setattr__(self, 'cycle_count', self.cycle_count + 1)

    def copy_from_if_updated(self, other: RobotRtData):
        """
        Copies only the updated data from another ``RobotRtData`` instance.
        As data is copied, the `_updated` field of the source object is cleared.

        This method is used to efficiently create a stable copy of real-time data that changed
        at the end of a monitoring cycle.

        Parameters
        ----------
        other
            Source data to copy from.
        """
        for field_to_copy in fields(self):
            field_name = field_to_copy.name
            value = getattr(self, field_name)
            other_value = getattr(other, field_name)
            if isinstance(other_value, TimestampedData):
                copy_data = False
                if value.timestamp != other_value.timestamp:
                    copy_data = True
                elif other_value.was_updated():
                    copy_data = True
                if copy_data:
                    value.set_timestamp(other_value.timestamp)
                    value.set_enabled(other_value.enabled)
                    value.set_data(list(other_value.data))
                if other_value is not None:
                    other_value.clear_updated()
            else:
                object.__setattr__(self, field_name, deepcopy(other_value))

    def _for_each_rt_data(self):
        """
        Iterates for each ``TimestampedData`` type member of this class
        (``rt_joint_pos``, ``rt_cart_pos``, etc.)
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
        """Clear the ``TimestampedData.enabled`` flag of each member of this class of type ``TimestampedData``
        """
        for rt_data in self._for_each_rt_data():
            if (rt_data.update_type == RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_UNAVAILABLE
                    or rt_data.update_type == RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL):
                rt_data.set_enabled(False)  # Optional, may not be sent by robot
            else:
                rt_data.set_enabled(True)  # Mandatory, the robot will send it

        for _, accelerometer in self.rt_accelerometer.items():
            if (accelerometer.update_type == RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_UNAVAILABLE
                    or accelerometer.update_type == RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL):
                accelerometer.set_enabled(False)  # Optional, may not be sent by robot
            else:
                accelerometer.set_enabled(True)  # Mandatory, the robot will send it

    def clear_if_outdated(self):
        """Clear the ``TimestampedData`` of each member if the data is outdated.
        """
        # Clear any outdated pieces of data by using the enable flag
        reference_timestamp = self.rt_target_joint_pos.timestamp  # Present at every cycle

        for rt_data in self._for_each_rt_data():
            if (rt_data.timestamp != reference_timestamp
                    and rt_data.update_type != RtDataUpdateType.MX_RT_DATA_UPDATE_TYPE_EVENT_BASED):
                rt_data.set_enabled(False)
                rt_data.clear_if_disabled()

    def clear_if_disabled(self):
        """Clear real-time values that are disabled
           (not reported by robot's current real-time monitoring configuration).
        """
        for rt_data in self._for_each_rt_data():
            rt_data.clear_if_disabled()


@dataclass
class RobotStatus(RobotCopyable):
    """
    Class for storing the status of a Mecademic robot.

    Attributes
    ----------
    activation_state
        True if the robot is activated.
    homing_state
        True if the robot is homed.
    simulation_mode
        True if the robot is in simulation-only mode (fast or real-time)
    recovery_mode
        True if the robot is in recovery mode.
    error_status
        True if the robot is in error.
    error_code
        Current robot error code, or MxRobotStatusCode.MX_ST_NONE if the robot is not in error
        state. Will be None for older robot versions that did not report the error code in the API
        (before v11).
    error_msg
        Message that explains the current error if appropriate, or empty string if the robot is not
        in error. Will be None for older robot versions that did not report the error message in
        the API (before v12)
    pstop2State
        *** IMPORTANT NOTE: PStop2 is not safety-rated on Meca500 robots ***
        *** Deprecated*** Use RobotSafetyStatus.pstop2_state instead.
        Current PStop2 status.
    estopState
        *** Deprecated*** Use RobotSafetyStatus.estop_state instead.
        Current EStop status. Note that Meca500 revision 3 or older never report this condition
        because these robots's power supply will be shutdown completely in case of EStop (and thus
        this API will get disconnected from the robot instead).
    pause_motion_status
        True if motion is currently paused.
    motion_cleared_status
        True when motion queue was cleared (by ClearMotion, an error state, a safety stop signal or robot deactivation).
        Remains True until motion is resumed.
    end_of_block_status
        True if robot is not moving and motion queue is empty.
        Note: We recommend not using end_of_block_status to detect when robot has finished
        executing previously sent  commands. Some posted commands may still be transient (on the
        network for example) and the robot may, in some cases, declare end-of-block in-between two
        commands. Instead, we recommend using checkpoints or the WaitIdle method.
    brakes_engaged
        True if robot brakes are engaged.
        This is relevant only when robot is deactivated (brakes are automatically disengaged upon
        robot activation).
    connection_watchdog_enabled
        True if the connection watchdog is currently enabled/active (see ``ConnectionWatchdog``)
"""
    activation_state: bool = False
    homing_state: bool = False
    simulation_mode: MxRobotSimulationMode = MxRobotSimulationMode.MX_SIM_MODE_DISABLED
    recovery_mode: bool = False
    error_status: bool = False
    error_code: Optional[MxRobotStatusCode] = None
    error_msg: Optional[str] = None
    # Deprecated fields (kept for compatibility)
    pstop2State: MxStopState = MxStopState.MX_STOP_STATE_RESET
    estopState: MxStopState = MxStopState.MX_STOP_STATE_RESET
    pause_motion_status: bool = False
    motion_cleared_status: bool = False
    end_of_block_status: bool = False
    brakes_engaged: bool = False
    connection_watchdog_enabled: bool = False

    def _copy_mutable_fields(self, copied: RobotStatus):
        # No mutable fields to copy in this class
        pass

    def __str__(self) -> str:
        # Show error message if possible, else error code, worse case simply 1/0 error status
        error_str = 'True' if self.error_status else 'False'
        if self.error_msg is not None and self.error_msg != "":
            error_str = self.error_msg
        elif self.error_code is not None and self.error_code != MxRobotStatusCode.MX_ST_NONE:
            error_str = f'{self.error_code}'

        error_str = f'{self.error_status}' if (self.error_code is None
                                               or self.error_code == 0) else f'{self.error_code}'
        return (f"Activated: {self.activation_state}, "
                f"homed: {self.homing_state}, "
                f"sim: {self.simulation_mode}, "
                f"recovery mode: {self.recovery_mode}, "
                f"error: {error_str}, "
                f"motion paused: {str(self.pause_motion_status)}, "
                f"motion cleared: {str(self.motion_cleared_status)}, "
                f"EOB: {self.end_of_block_status}, "
                f"brakes engaged: {self.brakes_engaged}, "
                f"connection watchdog: {'enabled' if self.connection_watchdog_enabled else 'disabled'}")

    def can_move(self) -> bool:
        """
        Indicates whether the robot can currently be moved (i.e., it is homed or activated in recovery mode).

        Returns
        -------
        bool
            True if the robot can be moved.
        """
        return self.homing_state or (self.activation_state and self.recovery_mode)


@dataclass
class RobotStaticSafetyStopMasks(RobotCopyable):
    """
    Various useful bit masks used to categorize safety signals.

    Attributes
    ----------
    clearedByPsu
        Bit mask to identify safety signals that must be reset using the power supply reset
        function.
    withVmOff
        Bit mask to identify safety signals that cause motor voltage to be removed.
        These are category 1 safety stop signals (Estop, PStop1, etc.).
    maskedInManualMode
        Bit mask to identify safety signals masked when the robot is in "manual" operation mode
        (PStop1, PStop2)
    """
    clearedByPsu: int = 0
    withVmOff: int = 0
    maskedInManualMode: int = 0

    def _copy_mutable_fields(self, copied: RobotStaticSafetyStopMasks):
        # No mutable fields to copy in this class
        pass


@dataclass
class RobotPowerSupplyInputs(RobotCopyable):
    """
    Class for storing robot's power supply physical input states

    Attributes
    ----------
    psu_input_mask
        Bit mask that summarizes all power supply input states. Use bits from ``MxPsuInputMask``
    estop_asserted
        Indicates if the EStop (emergency stop) signal on the power supply is asserted (robot will
        stop).
    pstop1_asserted
        Indicates if the PStop1 (protective stop category 1) signal on the power supply is asserted
        (robot will stop)
    pstop2_asserted
        Indicates if the PStop2 (protective stop category 2) signal on the power supply is asserted
        (robot will stop)
    reset_ext_asserted
        Indicates if the reset input signal on the power supply is asserted (requesting for a
        reset).
    reset_keypad_pressed
        Indicate if the reset keypad button on the power supply is pressed.
    enabling_device_asserted
        Indicate if the enabling device power supply input indicates that the enabling device is
        pressed.
    """
    psu_input_mask: int = 0
    estop_asserted: bool = False
    pstop1_asserted: bool = False
    pstop2_asserted: bool = False
    reset_ext_asserted: bool = False
    reset_keypad_pressed: bool = False
    enabling_device_asserted: bool = False

    def _copy_mutable_fields(self, copied: RobotPowerSupplyInputs):
        # No mutable fields to copy in this class
        pass

    def set_psu_input_mask(self, input_mask: Union[MxPsuInputMask, int]):
        """
        Update the power supply input mask, and update corresponding individual booleans (estop_asserted, etc.)

        Parameters
        ----------
        input_mask
            Power supply input mask to set.
        """
        self.psu_input_mask = int(input_mask)

        # Update individual booleans
        self.estop_asserted = (self.psu_input_mask & MxPsuInputMask.MX_PSU_INPUT_ESTOP) != 0
        self.pstop1_asserted = (self.psu_input_mask & MxPsuInputMask.MX_PSU_INPUT_PSTOP1) != 0
        self.pstop2_asserted = (self.psu_input_mask & MxPsuInputMask.MX_PSU_INPUT_PSTOP2) != 0
        self.reset_ext_asserted = (self.psu_input_mask & MxPsuInputMask.MX_PSU_INPUT_RESET_EXT) != 0
        self.reset_keypad_pressed = (self.psu_input_mask & MxPsuInputMask.MX_PSU_INPUT_RESET_KEYPAD) != 0
        self.enabling_device_asserted = (self.psu_input_mask & MxPsuInputMask.MX_PSU_INPUT_ENABLING_DEVICE) != 0

    def _set_asserted(self, field_name: str, mask: MxPsuInputMask, state: bool):
        """ Common to set_estop_asserted, set_pstop1_asserted, etc """
        object.__setattr__(self, field_name, state)
        new_mask = self.psu_input_mask
        if state:
            new_mask |= mask
        else:
            new_mask &= ~mask
        self.psu_input_mask = new_mask

    def set_estop_asserted(self, state: bool):
        """ Update estop_asserted and update the mask (psuInputMask) accordingly """
        self._set_asserted('estop_asserted', MxPsuInputMask.MX_PSU_INPUT_ESTOP, state)

    def set_pstop1_asserted(self, state: bool):
        """ Update pstop1_asserted and update the mask (psuInputMask) accordingly """
        self._set_asserted('pstop1_asserted', MxPsuInputMask.MX_PSU_INPUT_PSTOP1, state)

    def set_pstop2_asserted(self, state: bool):
        """ Update pstop2_asserted and update the mask (psuInputMask) accordingly """
        self._set_asserted('pstop2_asserted', MxPsuInputMask.MX_PSU_INPUT_PSTOP2, state)

    def set_reset_ext_asserted(self, state: bool):
        """ Update reset_ext_asserted and update the mask (psuInputMask) accordingly """
        self._set_asserted('reset_ext_asserted', MxPsuInputMask.MX_PSU_INPUT_RESET_EXT, state)

    def set_reset_keypad_asserted(self, state: bool):
        """ Update reset_keypad_asserted and update the mask (psuInputMask) accordingly """
        self._set_asserted('reset_keypad_pressed', MxPsuInputMask.MX_PSU_INPUT_RESET_KEYPAD, state)

    def set_enabling_device_asserted(self, state: bool):
        """ Update enabling_device_asserted and update the mask (psuInputMask) accordingly """
        self._set_asserted('enabling_device_asserted', MxPsuInputMask.MX_PSU_INPUT_ENABLING_DEVICE, state)


@dataclass
class RobotSafetyStatus(RobotCopyable):
    """
    Class for storing the safety stop status of a Mecademic robot.

    Attributes
    ----------
    robot_operation_mode
        The current robot operation mode, based on the the power supply key position (not supported on Meca500 robots).
        When the key is not in "automatic" position, restrictions will apply for using the robot.
    reset_ready
        Not yet implemented. Future implementation will:
        Indicate if it's currently possible to reset safety stop conditions that are linked to the power supply reset
        button because they remove motor power (EStop, PStop1, Operation mode change, robot reboot, etc.)
    stop_mask
        Bit mask that summarizes all safety stop conditions on the robot (including both active or resettable signals).
        Use bits from MxSafeStopCategory.
        Note: Also available as individual signal states (estop_state, pstop1_state, etc...)
    stop_resettable_mask
        Bit mask that summarizes all safety stop conditions that are currently resettable.
        Use bits from MxSafeStopCategory.
        Note: Also available as individual signal states (estop_state, pstop1_state, etc...)
    estop_state
        Current EStop status.
        Note that Meca500 revision 3 or older never report this condition because these robots's power supply
        will be shutdown completely in case of EStop (and thus this API will get disconnected from the robot instead).
    pstop1_state
        Current PStop1 status
    pstop2_state
        Current PStop2 status
        *** IMPORTANT NOTE: PStop2 is not safety-rated on Meca500 robots ***
    operation_mode_stop_state
        Current status for "operation mode" safety stop condition (upon mode changes or when mode is locked)
    enabling_device_released_stop_state
        Current status for "enabling device" safety stop condition
    voltage_fluctuation_stop_state
        Current status for "voltage fluctuation" safety stop condition
    reboot_stop_state
        Current status for "robot just rebooted" safety stop condition
    redundancy_fault_stop_state
        Current status for "redundancy fault" safety stop condition
    standstill_fault_stop_state
        Current status for "standstill fault" safety stop condition
    connection_dropped_stop_state
        Current status for "connection dropped" safety stop condition
    minor_error_stop_state
        Current status for "minor error" safety stop condition, which is triggered if robot removes motor voltage
        for any internal reason other than the safety stop signals above (thus generally due to a minor internal error).
        If this happens, see the robot logs for details.
    static_masks
        Useful masks to categorize safety stop signals from stop_mask
"""
    robot_operation_mode: MxRobotOperationMode = MxRobotOperationMode.MX_ROBOT_OPERATION_MODE_AUTO
    reset_ready: bool = False
    stop_mask: int = 0
    stop_resettable_mask: int = 0
    estop_state: MxStopState = MxStopState.MX_STOP_STATE_RESET
    pstop1_state: MxStopState = MxStopState.MX_STOP_STATE_RESET
    pstop2_state: MxStopState = MxStopState.MX_STOP_STATE_RESET
    operation_mode_stop_state: MxStopState = MxStopState.MX_STOP_STATE_RESET
    enabling_device_released_stop_state: MxStopState = MxStopState.MX_STOP_STATE_RESET
    voltage_fluctuation_stop_state: MxStopState = MxStopState.MX_STOP_STATE_RESET
    reboot_stop_state: MxStopState = MxStopState.MX_STOP_STATE_RESET
    redundancy_fault_stop_state: MxStopState = MxStopState.MX_STOP_STATE_RESET
    standstill_fault_stop_state: MxStopState = MxStopState.MX_STOP_STATE_RESET
    connection_dropped_stop_state: MxStopState = MxStopState.MX_STOP_STATE_RESET
    minor_error_stop_state: MxStopState = MxStopState.MX_STOP_STATE_RESET

    # Useful masks to categorize stop_mask above
    static_masks: RobotStaticSafetyStopMasks = field(default_factory=RobotStaticSafetyStopMasks)

    def _copy_mutable_fields(self, copied: RobotSafetyStatus):
        copied.static_masks = self.static_masks.copy()  # Make sure to create a copy instead of a reference

    def set_estop_state(self, stop_state: MxStopState):
        """Change the EStop state

        Parameters
        ----------
        stop_state
            EStop state to set
        """
        self.set_stop_state(MxSafeStopCategory.MX_SAFE_STOP_ESTOP, stop_state)

    def set_pstop1_state(self, stop_state: MxStopState):
        """Change the PStop1 state

        Parameters
        ----------
        stop_state
            PStop1 state to set
        """
        self.set_stop_state(MxSafeStopCategory.MX_SAFE_STOP_PSTOP1, stop_state)

    def set_pstop2_state(self, stop_state: MxStopState):
        """Change the PStop2 state

        Parameters
        ----------
        stop_state
            PStop2 state to set
        """
        self.set_stop_state(MxSafeStopCategory.MX_SAFE_STOP_PSTOP2, stop_state)

    def set_operation_mode_stop_state(self, stop_state: MxStopState):
        """Change the "operation mode" safety stop state

        Parameters
        ----------
        stop_state
            Safety stop state to set
        """
        self.set_stop_state(MxSafeStopCategory.MX_SAFE_STOP_OPERATION_MODE, stop_state)

    def set_enabling_device_released_stop_state(self, stop_state: MxStopState):
        """Change the "enabling device" safety stop state

        Parameters
        ----------
        stop_state
            Safety stop state to set
        """
        self.set_stop_state(MxSafeStopCategory.MX_SAFE_STOP_ENABLING_DEVICE_RELEASED, stop_state)

    def set_voltage_fluctuation_stop_state(self, stop_state: MxStopState):
        """Change the "voltage fluctuation" safety stop state

        Parameters
        ----------
        stop_state
            Safety stop state to set
        """
        self.set_stop_state(MxSafeStopCategory.MX_SAFE_STOP_VOLTAGE_FLUCTUATION, stop_state)

    def set_reboot_stop_state(self, stop_state: MxStopState):
        """Change the "robot just rebooted" safety stop state

        Parameters
        ----------
        stop_state
            Safety stop state to set
        """
        self.set_stop_state(MxSafeStopCategory.MX_SAFE_STOP_REBOOT, stop_state)

    def set_redundancy_fault_stop_state(self, stop_state: MxStopState):
        """Change the "redundancy fault" safety stop state

        Parameters
        ----------
        stop_state
            Safety stop state to set
        """
        self.set_stop_state(MxSafeStopCategory.MX_SAFE_STOP_REDUNDANCY_FAULT, stop_state)

    def set_standstill_fault_stop_state(self, stop_state: MxStopState):
        """Change the "standstill fault" safety stop state

        Parameters
        ----------
        stop_state
            Safety stop state to set
        """
        self.set_stop_state(MxSafeStopCategory.MX_SAFE_STOP_STANDSTILL_FAULT, stop_state)

    def set_connection_dropped_stop_state(self, stop_state: MxStopState):
        """Change the "connection dropped" safety stop state

        Parameters
        ----------
        stop_state
            Safety stop state to set
        """
        self.set_stop_state(MxSafeStopCategory.MX_SAFE_STOP_CONNECTION_DROPPED, stop_state)

    def set_minor_error_stop_state(self, stop_state: MxStopState):
        """Change the "minor error" safety stop state

        Parameters
        ----------
        stop_state
            Safety stop state to set
        """
        self.set_stop_state(MxSafeStopCategory.MX_SAFE_STOP_MINOR_ERROR, stop_state)

    def get_stop_state_by_status_code(self, code: MxRobotStatusCode) -> MxStopState:
        """Returns the safety state for the requested safety signal

        Parameters
        ----------
        stop_category
            Safety signal to get state for

        Returns
        -------
        MxStopState
            Safety signal state
        """
        if code == MxRobotStatusCode.MX_ST_ESTOP:
            return self.estop_state
        elif code == MxRobotStatusCode.MX_ST_PSTOP1:
            return self.pstop1_state
        elif code == MxRobotStatusCode.MX_ST_PSTOP2:
            return self.pstop2_state
        elif code == MxRobotStatusCode.MX_ST_GET_OPERATION_MODE:
            return self.operation_mode_stop_state
        elif code == MxRobotStatusCode.MX_ST_SAFE_STOP_ENABLING_DEVICE_RELEASED:
            return self.enabling_device_released_stop_state
        elif code == MxRobotStatusCode.MX_ST_SAFE_STOP_VOLTAGE_FLUCTUATION:
            return self.voltage_fluctuation_stop_state
        elif code == MxRobotStatusCode.MX_ST_SAFE_STOP_REBOOT:
            return self.reboot_stop_state
        elif code == MxRobotStatusCode.MX_ST_SAFE_STOP_REDUNDANCY_FAULT:
            return self.redundancy_fault_stop_state
        elif code == MxRobotStatusCode.MX_ST_SAFE_STOP_STANDSTILL_FAULT:
            return self.standstill_fault_stop_state
        elif code == MxRobotStatusCode.MX_ST_SAFE_STOP_CONNECTION_DROPPED:
            return self.connection_dropped_stop_state
        elif code == MxRobotStatusCode.MX_ST_SAFE_STOP_MINOR_ERROR:
            return self.minor_error_stop_state
        raise ValueError(f'Status code {code} does not correspond to a safety signal')

    def set_stop_state(self, stop_category: MxSafeStopCategory, stop_state: MxStopState):
        """
        Changes a safety stop state

        Parameters
        ----------
        stop_category
            Safety stop category to change state for.
        stop_state
            Safety stop state to set.
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
            self.stop_mask = self.stop_mask | stop_category
            self.stop_resettable_mask = self.stop_resettable_mask & ~stop_category
        elif stop_state == MxStopState.MX_STOP_STATE_RESETTABLE:
            self.stop_mask = self.stop_mask | stop_category
            self.stop_resettable_mask = self.stop_resettable_mask | stop_category
        else:
            self.stop_mask = self.stop_mask & ~stop_category
            self.stop_resettable_mask = self.stop_resettable_mask & ~stop_category

    @classmethod
    def mask_to_string(cls, mask: int) -> str:
        """
        Formats a safety stop mask as a string, detailing each active safety stop signal in the
        mask.

        Parameters
        ----------
        mask
            Bit mask to format as a detailed string.

        Returns
        -------
        str
            String describing all active safety stop signals in the mask.
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


@dataclass
class GripperStatus(RobotCopyable):
    """
    Class for storing the Mecademic robot's gripper status.

    **Deprecated** Use ExtToolStatus and GripperState instead.

    Attributes
    ----------
    present
        True if the gripper is present on the robot.
    homing_state
        True if the gripper has been homed (ready to be used).
    holding_part
        True if the gripper is currently holding a part.
    target_pos_reached
        True if the gripper is at target position or at a limit (fully opened or closed).
    error_status
        True if the gripper is in error state.
    overload_error
        True if the gripper is in overload error state.
    """
    present: bool = False
    homing_state: bool = False
    holding_part: bool = False
    target_pos_reached: bool = False
    error_status: bool = False
    overload_error: bool = False

    def __repr__(self) -> str:
        return str(self)

    def _copy_mutable_fields(self, copied: GripperStatus):
        # No mutable fields to copy in this class
        pass


@dataclass
class NetworkConfig(RobotCopyable):
    """
    Class for storing the Mecademic robot's network configuration.

    Attributes
    ----------
    name
        Robot name for DHCP requests
    dhcp
        DHCP mode enabled for automatic IP assignment by DHCP server
    ip
        IPv4 address (e.g., '192.168.0.100')
    mask
        Network mask (e.g. '255.255.255.0')
    gateway
        Gateway IP address (e.g. '192.168.0.1')
    mac
        Robot read-only MAC address (e.g. '20:B0:F7:06:E4:80')

    """
    name: str = ''
    dhcp: bool = False
    ip: str = '192.168.0.100'
    mask: str = '255.255.255.0'
    gateway: str = ''
    mac: str = ''

    def _copy_mutable_fields(self, copied: NetworkConfig):
        # No mutable fields to copy in this class
        pass


@dataclass
class NetworkOptions(RobotCopyable):
    """
    Class for storing the Mecademic robot's network connection options.

    Attributes
    ----------
    keep_alive_timeout
        TCP keep-alive connection timeout, in seconds. It specifies the delay before the robot closes
        the TCP connection when keep-alive replies are not received from the client.
        Value n is an integer number ranging from 0 to 60.
        Factory-default value is 3.
        A value of 0 disables the TCP keep-alive, in which case detecting a connection failure can be very long.
    """
    keep_alive_timeout: int = 3

    def _copy_mutable_fields(self, copied: NetworkOptions):
        # No mutable fields to copy in this class
        pass


@dataclass
class ExtToolStatus(RobotCopyable):
    """
    Class for storing the Mecademic robot's external tool status.

    Attributes
    ----------
    sim_tool_type
        Currently simulated tool type (or MxExtToolType.MX_EXT_TOOL_NONE if simulation is not enabled)
    physical_tool_type
        Physical external tool type.
    homing_state
        True if the gripper is homed.
    error_status
        True if the gripper is in error state.
    overload_error
        True if the gripper is in overload error state.
    comm_err_warning
        True if some communication errors were detected with the external tool.
        This could mean that the cable may be damaged and must be replaced,
        or that the cable may simply not be screwed tight enough on either side.
    """
    sim_tool_type: MxExtToolType = MxExtToolType.MX_EXT_TOOL_NONE
    physical_tool_type: MxExtToolType = MxExtToolType.MX_EXT_TOOL_NONE
    homing_state: bool = False
    error_status: bool = False
    overload_error: bool = False
    comm_err_warning: bool = False

    def _copy_mutable_fields(self, copied: ExtToolStatus):
        # No mutable fields to copy in this class
        pass

    def current_tool_type(self) -> int:
        """
        Returns current external tool type (simulated or physical)

        Returns
        -------
        int
            Current external tool
        """
        return self.sim_tool_type if self.sim_tool_type != MxExtToolType.MX_EXT_TOOL_NONE else self.physical_tool_type

    def is_physical_tool_present(self) -> bool:
        """
        Returns if physical tool is connected

        Returns
        -------
        bool
            True if physical gripper is connected, False otherwise
        """
        return self.physical_tool_type != MxExtToolType.MX_EXT_TOOL_NONE

    def is_tool_sim(self) -> bool:
        """
        Returns if tool is simulated or not

        Returns
        -------
        bool
            True if tool is simulated, False otherwise
        """
        return self.sim_tool_type != MxExtToolType.MX_EXT_TOOL_NONE

    def is_gripper(self, physical: bool = False) -> bool:
        """
        Returns if current external tool (simulated or physical) is a gripper.

        Parameters
        ----------
        physical
            True check physical gripper, False use current one (simulated or physical).

        Returns
        -------
        bool
            True if tool is a gripper, False otherwise.
        """
        tool_type = self.physical_tool_type if physical else self.current_tool_type()
        return tool_type in [MxExtToolType.MX_EXT_TOOL_MEGP25_SHORT, MxExtToolType.MX_EXT_TOOL_MEGP25_LONG]

    def is_pneumatic_module(self, physical: bool = False) -> bool:
        """
        Indicates whether current external tool (simulated or physical) is a pneumatic module.

        Parameters
        ----------
        physical
            True check physical gripper, False use current one (simulated or physical).

        Returns
        -------
        bool
            True if tool is a pneumatic module, False otherwise.
        """
        tool_type = self.physical_tool_type if physical else self.current_tool_type()
        return tool_type in [MxExtToolType.MX_EXT_TOOL_VBOX_2VALVES]


@dataclass
class IoStatus(RobotCopyable):
    """Class for storing the Mecademic robot's IO modules status.

    Attributes
    ----------
    bank_id
        Type of this IO module:
        1: ``MxIoBankId.MX_IO_BANK_ID_IO_MODULE``
    present
        True if an IO module of this type is present on the robot.
    nb_inputs
        Number of digital inputs supported by this IO module.
    nb_outputs
        Number of digital outputs supported by this IO module.
    sim_mode
        True if the IO module is in simulation mode.
    error
        Error code of the IO module (0 if no error).
    timestamp
        Monotonic timestamp associated with data (in microseconds since robot last reboot)
        This timestamp is stamped by the robot so it is not affected by host/network jitter
    """
    bank_id: MxIoBankId = MxIoBankId.MX_IO_BANK_ID_UNDEFINED
    present: bool = False
    nb_inputs: int = 0
    nb_outputs: int = 0
    sim_mode: bool = False
    error: int = 0
    timestamp: int = 0

    def _copy_mutable_fields(self, copied: IoStatus):
        # No mutable fields to copy in this class
        pass


@dataclass
class ValveState(RobotCopyable):
    """
    Class for storing the Mecademic robot's pneumatic module valve states.

    Attributes
    ----------
    valve_opened
        List of valve state: ``MX_VALVE_STATE_CLOSE`` or ``MX_VALVE_STATE_OPEN``
    """
    valve_opened: list[int] = field(default_factory=lambda: [0] * MX_EXT_TOOL_VBOX_MAX_VALVES)

    def _copy_mutable_fields(self, copied: ValveState):
        copied.valve_opened = self.valve_opened.copy()  # Make sure to create a copy instead of a reference


@dataclass
class GripperState(RobotCopyable):
    """
    Class for storing the Mecademic robot's gripper state.

    Attributes
    ----------
    holding_part
        True if the gripper is currently holding a part.
    target_pos_reached
        True if the gripper is at requested position:

          - At configured opened/closed position following ``GripperOpen``/``GripperClose``.
          - At requested position after MoveGripper.
    closed
        True if the gripper is at the configured 'close' position (``SetGripperRange``) or less.
    opened
        True if the gripper is at the configured 'open' position (``SetGripperRange``) or more.

"""
    holding_part: bool = False
    target_pos_reached: bool = False
    closed: bool = False
    opened: bool = False

    def _copy_mutable_fields(self, copied: GripperState):
        # No mutable fields to copy in this class
        pass


@dataclass
class VacuumState(RobotCopyable):
    """
    Class for storing the Mecademic robot's IO module vacuum state.

    Attributes
    ----------
    vacuum_on
        True if the vacuum is currently 'on' (trying to pick or holding part).
    purge_on: bool
        True if currently pushing air to release part (see SetVacuumPurgeDuration).
    holding_part
        True if currently holding part (based on configured pressure thresholds, see
        ``SetVacuumThreshold`` ).
    timestamp
        Monotonic timestamp associated with data (in microseconds since robot last reboot)
        This timestamp is stamped by the robot so it is not affected by host/network jitter
    """
    vacuum_on: bool = False
    purge_on: bool = False
    holding_part: bool = False
    timestamp: int = 0

    def _copy_mutable_fields(self, copied: VacuumState):
        # No mutable fields to copy in this class
        pass


@dataclass
class CollisionObject(RobotCopyable):
    """
    Class that represents one object that can enter in collision with another or with the work zone
    boundary. A collision object is defined by its group (``MxCollisionGroup``) and, in some cases,
    an index (like the link).

    Attributes
    ----------
    group
        The group (type) of this object
    index
        The index of the object within the group (for groups that can have multiple objects).
        Available indices for groups are:

        - ``MxCollisionGroup.MX_COLLISION_GROUP_ROBOT``:    Use index values from``MxCollisionGroupRobotIdx``.
        - ``MxCollisionGroup.MX_COLLISION_GROUP_FCP``:      Index is not used (always 0).
        - ``MxCollisionGroup.MX_COLLISION_GROUP_TOOL``:     Always 0 when tool sphere is used.
        - ``MxCollisionGroup.MX_COLLISION_GROUP_ENV_OBJ``:  User-defined index of the user-defined  environment objects.
                                                            Not supported yet.
        - ``MxCollisionGroup.MX_COLLISION_GROUP_WORK_ZONE``: Index is not used (always 0).
    """
    group: MxCollisionGroup = MxCollisionGroup.MX_COLLISION_GROUP_ROBOT
    index: int = MxCollisionGroupRobotIdx.MX_COLLISION_GROUP_ROBOT_BASE

    def __post_init__(self):
        if self.group == MxCollisionGroup.MX_COLLISION_GROUP_ROBOT:
            corrected_index = MxCollisionGroupRobotIdx(self.index)
        elif self.group == MxCollisionGroup.MX_COLLISION_GROUP_TOOL:
            corrected_index = MxCollisionGroupToolIdx(self.index)
        else:
            corrected_index = int(self.index)
        object.__setattr__(self, 'index', corrected_index)

    def _copy_mutable_fields(self, copied: CollisionObject):
        # No mutable fields to copy in this class
        pass

    @classmethod
    def from_response(cls, response_args: list[int]):
        group = MxCollisionGroup(response_args.pop(0))
        index = response_args.pop(0)
        return cls(group=group, index=index), response_args

    def __eq__(self, other):
        if not isinstance(other, CollisionObject):
            return NotImplemented
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


@dataclass
class SelfCollisionStatus(RobotCopyable):
    """
    Class for storing the Mecademic robot's self collision status.
    This is used when robot collision detection has been activated (with ``SetCollisionCfg``).

    Attributes
    ----------
    collision_detected
        True if the robot has detected a collision with itself.
        Note that when the collision severity is set to ``MX_EVENT_SEVERITY_PAUSE_MOTION`` or
        greater  the robot will have stopped just before the collision actually occurs and
        ``collision_detected`` will be True  until ``ResumeMotion`` is called.
        When collision severity is set to ``MX_EVENT_SEVERITY_WARNING`` the robot will actually
        continue to move and the collision will actually occur. This can damage the robot or the
        tool.
    object1
        When collision is detected, indicates the first of the two objects that caused the collision
        (a part of the robot or the tool)
    object2
        When collision is detected, indicates the second of the two objects that caused the
        collision (a part of the robot or the tool)
    """
    collision_detected: bool = False
    object1: CollisionObject = field(default_factory=CollisionObject)
    object2: CollisionObject = field(default_factory=CollisionObject)

    def _copy_mutable_fields(self, copied: SelfCollisionStatus):
        copied.object1 = self.object1.copy()  # Make sure to create a copy instead of a reference
        copied.object2 = self.object2.copy()  # Make sure to create a copy instead of a reference

    @classmethod
    def from_response(cls, response_args: List[int]) -> SelfCollisionStatus:
        """ Build a SelfCollisionStatus from the robot response as an int array """
        # Parse collision_detected flag
        collision_detected = bool(response_args.pop(0))

        # Parse object1
        object1, response_args = CollisionObject.from_response(response_args)

        # Parse object2
        object2, response_args = CollisionObject.from_response(response_args)

        # Build new instance with parsed data
        return cls(
            collision_detected=collision_detected,
            object1=object1,
            object2=object2,
        )

    def __str__(self) -> str:
        if self.collision_detected:
            return f'Collision detected between {self.object1} and {self.object2}'
        else:
            return 'No collision detected'

    def __repr__(self) -> str:
        return str(self)


@dataclass
class WorkZoneStatus(RobotCopyable):
    """
    Class for storing the Mecademic robot's "in work zone" status.

    This is used when robot work zone has been defined and enabled (with ``SetWorkZoneCfg``).

    Attributes
    ----------
    outside_work_zone
        True if the a part of the robot or the tool have reached the work zone boundary.
        Note that when the collision severity is set to ``MX_EVENT_SEVERITY_PAUSE_MOTION`` or
        greater  the robot will have stopped just before the robot moves outside the work zone but
        the flag ``outside_work_zone`` will still be set to True until ``ResumeMotion`` is called.
        When collision severity is set to ``MX_EVENT_SEVERITY_WARNING``, the flag
        ``outside_work_zone`` will report whether the robot is currently outside the work zone.
    object
        Indicate the object that reached the work zone boundary (a part of the robot, or the tool)
"""
    outside_work_zone: bool = False
    object: CollisionObject = field(default_factory=CollisionObject)

    def _copy_mutable_fields(self, copied: WorkZoneStatus):
        copied.object = self.object.copy()  # Make sure to create a copy instead of a reference

    @classmethod
    def from_response(cls, response_args: list[int]) -> WorkZoneStatus:
        """ Build a WorkZoneStatus from the robot response as an int array """
        outside_work_zone = bool(response_args.pop(0))
        obj, response_args = CollisionObject.from_response(response_args)
        return cls(outside_work_zone=outside_work_zone, object=obj)

    def __str__(self) -> str:
        if self.outside_work_zone:
            return f'Work zone boundary reached by {self.object}'
        else:
            return 'Robot is inside work zone'

    def __repr__(self) -> str:
        return str(self)


@dataclass
class CollisionStatus(RobotCopyable):
    """
    Class for storing the Mecademic robot's collision status (collision with self or work zone
    boundary).

    This is used when robot collision detection has been activated (with ``SetCollisionCfg``) or
    work zone has been enabled (with ``SetWorkZoneCfg``).

    Attributes
    ----------
    self_collision_status
        Current self collision status.
    work_zone_status
        Current "inside work zone" status.
    """
    self_collision_status: SelfCollisionStatus = field(default_factory=SelfCollisionStatus)
    work_zone_status: WorkZoneStatus = field(default_factory=WorkZoneStatus)

    def _copy_mutable_fields(self, copied: CollisionStatus):
        copied.self_collision_status = self.self_collision_status.copy()
        copied.work_zone_status = self.work_zone_status.copy()


@dataclass
class LoadedProgramConfig(RobotCopyable):
    """
    Class for storing the configuration for one robot program (loaded or not).

    Attributes
    ----------
    configured_to_load
        Indicate if the current configuration is to load this program or not.
    """
    configured_to_load: bool = False

    def _copy_mutable_fields(self, copied: LoadedProgramConfig):
        # No mutable fields to copy in this class
        pass


@dataclass
class LoadedPrograms(RobotCopyable):
    """
    Class for storing the robot's configuration related to user programs (which are loaded, etc.)

    Attributes
    ----------
    programs
        Indicates if the current configuration is to load this program or not.
    """
    programs: dict[str, LoadedProgramConfig]

    def _copy_mutable_fields(self, copied: LoadedPrograms):
        copied.programs = deepcopy(self.programs)  # Make sure to create a copy instead of a reference


@dataclass
class ProgramStatus(RobotCopyable):
    """
    Class for storing the status of a user program loaded by the MecaScript engine.

    Attributes
    ----------
    status
        The current program loading status. To know if the program is configured to be loaded,
        refer to ``configured_to_load`` boolean below.
    message
        A message for the user that details the status (generally used for detailing errors).
    configured_to_load
        Indicates if the current configuration is to load this program or not.
"""
    status: MxProgramStatus = MxProgramStatus.MX_PROGRAM_UNLOADED
    message: str = ""
    configured_to_load: bool = False

    def _copy_mutable_fields(self, copied: ProgramStatus):
        # No mutable fields to copy in this class
        pass


@dataclass
class MecaScriptCfg(RobotCopyable):
    """
    Class for storing the robot's MecaScript configuration options.

    Attributes
    ----------
    python_enabled
        True if this MecaScript Python engine should be running on the robot.
"""
    python_enabled: bool = True

    def _copy_mutable_fields(self, copied: MecaScriptEngineStatus):
        # No mutable fields to copy in this class
        pass


@dataclass
class MecaScriptEngineStatus(RobotCopyable):
    """
    Class for storing the status of a MecaScript engine connected to the robot.

    Attributes
    ----------
    id
        Id of the running MecaScript engine, None if no engine is running
    embedded
        True if this MecaScript engine is running embedded inside the robot.
    remote_ip
        The IP address of this MecaScript engine.
    ready
        The MecaScript engine is ready (has finished loading all programs and registering all commands)
    registered_commands
        List of commands registered by this MecaScript engine.
"""
    id: Optional[int] = None
    embedded: bool = False
    remote_ip: str = ""
    ready: bool = False
    registered_commands: List[str] = field(default_factory=list)
    loaded_programs_status: Dict[str, ProgramStatus] = field(default_factory=dict)

    def _copy_mutable_fields(self, copied: MecaScriptEngineStatus):
        copied.registered_commands = self.registered_commands.copy()
        copied.loaded_programs_status = deepcopy(self.loaded_programs_status)


@dataclass
class ProgramExecutionStatus(RobotCopyable):
    """
    Class representing the execution status of a MecaScript program, telling whether
    the robot is currently executing a MecaScript program along with other useful information.

    Attributes
    ----------
    command_name
        Name of the command currently being executed. Empty string if no command is executing.
    file_path
        Path of the program file in which this command was defined.
    stopping
        True if the executing command has been requested to stop.
    id
        ID of the MecaScript engine on which the command is being executed (0 if none).
    command_type
        Indicates the type of the executing command (inline or controlling).
    """
    command_name: str = ""
    file_path: str = ""
    stopping: bool = False
    id: int = 0
    command_type: MxRegisteredCmdType = MxRegisteredCmdType.MX_REGISTERED_CMD_TYPE_INLINE

    def _copy_mutable_fields(self, copied: ProgramExecutionStatus):
        # No mutable fields to copy in this class
        pass


@dataclass
class CalibrationCfg(RobotCopyable):
    """
    Class for storing the robot calibration configuration, as set by ``SetCalibrationCfg``.

    Attributes
    ----------
    enabled
        True when the calibration is enabled on the robot. Note that for this to make any
        difference, the robot must have been calibrated at factory, which is
        indicated by ``GetRobotCalibrated``.
    """
    enabled: bool = False

    def _copy_mutable_fields(self, copied: CalibrationCfg):
        # No mutable fields to copy in this class
        pass


@dataclass
class CollisionCfg(RobotCopyable):
    """
    Class for storing the robot collision configuration, as set by ``SetCollisionCfg``.

    Attributes
    ----------
    severity
        The severity of detected robot self collision events. The allowed values are:

        - ``MX_EVENT_SEVERITY_SILENT``
        - ``MX_EVENT_SEVERITY_WARNING``
        - ``MX_EVENT_SEVERITY_ERROR``
    """
    severity: MxEventSeverity = MxEventSeverity.MX_EVENT_SEVERITY_ERROR

    def _copy_mutable_fields(self, copied: CollisionCfg):
        # No mutable fields to copy in this class
        pass


@dataclass
class JointLimitsCfg(RobotCopyable):
    """
    Class for storing the robot joint limits configuration, as set by ``SetJointLimitsCfg``.
    Note that joint limits themselves are returned by ``GetJointLimits``.

    Attributes
    ----------
    enabled
        True when the joint limits are enabled on the robot.
"""
    enabled: bool = False

    def _copy_mutable_fields(self, copied: JointLimitsCfg):
        # No mutable fields to copy in this class
        pass


@dataclass
class MoveDurationCfg(RobotCopyable):
    """
    Class for storing the configuration of robot moves in duration mode, as set by
    ``SetMoveDurationCfg``.

    Attributes
    ----------
    severity
        The severity of events reporting that the robot cannot meet the requested duration for a
        given move command. The allowed values are:

        - ``MX_EVENT_SEVERITY_SILENT``
        - ``MX_EVENT_SEVERITY_WARNING``
        - ``MX_EVENT_SEVERITY_ERROR``
    """
    severity: MxEventSeverity = MxEventSeverity.MX_EVENT_SEVERITY_WARNING

    def _copy_mutable_fields(self, copied: MoveDurationCfg):
        # No mutable fields to copy in this class
        pass


@dataclass
class Pstop2Cfg(RobotCopyable):
    """
    Class for storing the PStop2 safety signal configuration, as set by SetPstop2Cfg

    Attributes
    ----------
    severity
        The severity of PStop2 safety signals. The allowed values are:

        - ``MX_EVENT_SEVERITY_PAUSE_MOTION``
        - ``MX_EVENT_SEVERITY_CLEAR_MOTION``
    """
    severity: MxEventSeverity = MxEventSeverity.MX_EVENT_SEVERITY_CLEAR_MOTION

    def _copy_mutable_fields(self, copied: Pstop2Cfg):
        # No mutable fields to copy in this class
        pass


@dataclass
class SimModeCfg(RobotCopyable):
    """Class for storing the robot default simulation mode configuration, as set by
    ``SetSimModeCfg``.

    Attributes
    ----------
    mode: MxRobotSimulationMode
        The default simulation mode that is applied when ``ActivateSim`` is called without
        an argument or when the simulation  mode is enabled from the cyclic protocols.
    """
    mode: MxRobotSimulationMode = MxRobotSimulationMode.MX_SIM_MODE_REAL_TIME

    def _copy_mutable_fields(self, copied: SimModeCfg):
        # No mutable fields to copy in this class
        pass


@dataclass
class TorqueLimitsCfg(RobotCopyable):
    """
    Class for storing the torque limits configuration, as set by ``SetTorqueLimitsCfg``.
    Note that the torque limits themselves (per joint) is returned by ``GetTorqueLimits``.

    Attributes
    ----------
    severity
        The severity of torque exceeded events. The allowed values are:

        - ``MX_EVENT_SEVERITY_SILENT``
        - ``MX_EVENT_SEVERITY_WARNING``
        - ``MX_EVENT_SEVERITY_PAUSE_MOTION``
        - ``MX_EVENT_SEVERITY_ERROR``
    mode: MxTorqueLimitsMode
        The mode used to determine if the robot torque exceeds the configured limit.
    """
    severity: MxEventSeverity = MxEventSeverity.MX_EVENT_SEVERITY_ERROR
    mode: MxTorqueLimitsMode = MxTorqueLimitsMode.MX_TORQUE_LIMITS_MODE_DELTA_WITH_EXPECTED

    def _copy_mutable_fields(self, copied: TorqueLimitsCfg):
        # No mutable fields to copy in this class
        pass


@dataclass
class TorqueLimitsStatus(RobotCopyable):
    """
    Class for storing the robot's torque limits status.
    The torque limits are defined by ``SetTorqueLimitsCfg`` and ``SetTorqueLimits``.

    Attributes
    ----------
    exceeded
        True when the configured torque limits are exceeded on the robot.
"""
    exceeded: bool = False

    def _copy_mutable_fields(self, copied: TorqueLimitsStatus):
        # No mutable fields to copy in this class
        pass


@dataclass
class WorkZoneCfg(RobotCopyable):
    """
    Class for storing the robot work zone configuration, as set by ``SetWorkZoneCfg``.

    Attributes
    ----------
    severity
        The severity of detected robot self collision events. The allowed values are:

        - ``MX_EVENT_SEVERITY_SILENT``
        - ``MX_EVENT_SEVERITY_WARNING``
        - ``MX_EVENT_SEVERITY_ERROR``
    mode: MxWorkZoneMode
        The mode that determines when the robot is considered outside the work zone.
    """
    severity: MxEventSeverity = MxEventSeverity.MX_EVENT_SEVERITY_ERROR
    mode: MxWorkZoneMode = MxWorkZoneMode.MX_WORK_ZONE_MODE_FCP_IN_WORK_ZONE

    def _copy_mutable_fields(self, copied: WorkZoneCfg):
        # No mutable fields to copy in this class
        pass


@dataclass
class WorkZoneLimits(RobotCopyable):
    """
    Class for storing the work zone limits.

    Attributes
    ----------
    x_min
        Minimum x value of the work zone, in mm.
    y_min
        Minimum y value of the work zone, in mm.
    z_min
        Minimum z value of the work zone, in mm.
    x_max
        Maximum x value of the work zone, in mm.
    y_max
        Maximum y value of the work zone, in mm.
    z_max
        Maximum z value of the work zone, in mm.
"""
    x_min: float = 0
    y_min: float = 0
    z_min: float = 0
    x_max: float = 0
    y_max: float = 0
    z_max: float = 0

    def _copy_mutable_fields(self, copied: WorkZoneLimits):
        # No mutable fields to copy in this class
        pass


@dataclass
class RobotPayload(RobotCopyable):
    """
    Class for storing the robot payload.

    Attributes
    ----------
    mass
        Carried mass in kg
    x
        X coordinate of center of mass relative to FRF.
    y
        Y coordinate of center of mass relative to FRF.
    z
        Z coordinate of center of mass relative to FRF.
"""
    mass: float = 0
    x: float = 0
    y: float = 0
    z: float = 0

    def _copy_mutable_fields(self, copied: RobotPayload):
        # No mutable fields to copy in this class
        pass


@dataclass
class ToolSphere(RobotCopyable):
    """
    Class for storing the robot tool sphere configuration.

    Attributes
    ----------
    x
        Offset along FRF x-axis to position tool center
    y
        Offset along FRF y-axis to position tool center
    z
        Offset along FRF z-axis to position tool center
    r
        Radius of the tool sphere model.
"""
    x: float = 0
    y: float = 0
    z: float = 0
    r: float = 0

    def _copy_mutable_fields(self, copied: ToolSphere):
        # No mutable fields to copy in this class
        pass


@dataclass
class VacuumThreshold(RobotCopyable):
    """
    Class for storing the vacuum thresholds.

    Attributes
    ----------
    hold_threshold
        Vacuum pressure threshold to consider holding part in kPa.
    release_threshold
        Vacuum pressure threshold to consider part released in kPa.
"""
    hold_threshold: float = 0
    release_threshold: float = 0

    def _copy_mutable_fields(self, copied: VacuumThreshold):
        # No mutable fields to copy in this class
        pass


@dataclass
class MoveJumpApproachVel(RobotCopyable):
    """
    Class for storing the move jump approach velocity configuration.

    Attributes
    ----------
    startVel
        See ``SetMoveJumpApproachVel`` for details.
    startDist
        See ``SetMoveJumpApproachVel`` for details.
    endVel
        See ``SetMoveJumpApproachVel`` for details.
    endDist
        See ``SetMoveJumpApproachVel`` for details.
    """
    startVel: float = 0
    startDist: float = 0
    endVel: float = 0
    endDist: float = 0

    def _copy_mutable_fields(self, copied: MoveJumpApproachVel):
        # No mutable fields to copy in this class
        pass


@dataclass
class MoveJumpHeight(RobotCopyable):
    """
    Class for storing the move jump height configuration.

    Attributes
    ----------
    startHeight
        See ``SetMoveJumpHeight`` for details.
    endHeight
        See ``SetMoveJumpHeight`` for details.
    minHeight
        See ``SetMoveJumpHeight`` for details.
    maxHeight
        See ``SetMoveJumpHeight`` for details.
    """
    startHeight: float = 0
    endHeight: float = 0
    minHeight: float = 0
    maxHeight: float = 0

    def _copy_mutable_fields(self, copied: MoveJumpHeight):
        # No mutable fields to copy in this class
        pass


@dataclass
class RobotProgramStatus(RobotCopyable):
    """
    Class for storing the status of a robot program

    Attributes
    ----------
    valid
        The program is considered valid and can be executed if loaded.
    invalid_lines
        List of line numbers in the file that are considered invalid.
    """
    valid: bool = False
    invalid_lines: list[int] = field(default_factory=lambda: [])

    def _copy_mutable_fields(self, copied: RobotProgramStatus):
        copied.invalid_lines = self.invalid_lines.copy()


@dataclass
class RobotFileStatus(RobotCopyable):
    """
    Class for storing the status of a robot file

    Attributes
    ----------
    program_status
        Program validity status. Ignore if this file is not a robot program.
    """
    program_status: RobotProgramStatus = field(default_factory=RobotProgramStatus)

    def _copy_mutable_fields(self, copied: RobotFileStatus):
        copied.program_status = self.program_status.copy()
        # No mutable fields to copy in this class
        pass


@dataclass
class RobotFile(RobotCopyable):
    """
    Class for storing a robot file loaded from the robot.

    Attributes
    ----------
    name
        The file name (path) on the robot
    content
        The file content. Generally as a string, unless the data is binary and cannot be represented as a string,
        in which case it will be returned as a bytearray.
    status
        The file status
    """
    name: str = field(default_factory="")
    content: str | bytearray = field(default_factory="")
    status: RobotFileStatus = field(default_factory=RobotFileStatus)

    def _copy_mutable_fields(self, copied: RobotFile):
        copied.status = self.status.copy()


@dataclass
class RobotFileList(RobotCopyable):
    """
    Class for storing the list of files returned by the robot.

    Attributes
    ----------
    files
        Dictionary of listed files, key is file path
    """
    files: Dict[str, RobotFileStatus] = field(default_factory=dict)

    def _copy_mutable_fields(self, copied: RobotFileList):
        copied.files = self.files.copy()


@dataclass
class RobotTemperature(RobotCopyable):
    """
    Class for storing the robot temperature.

    Attributes
    ----------
    timestamp
        Monotonic timestamp associated with data (in microseconds since robot last reboot)
        This timestamp is stamped by the robot so it is not affected by host/network jitter
    baseboard
        The robot temperature measured on the main CPU (baseboard), in Celsius.
    psu
        The robot temperature measured in the Power supply, in Celsius.
    safe_mcu
        The robot temperature measured on the safety processors, in Celsius.
    motors
        The robot temperature measured each motor, in Celsius.
    """
    timestamp: int = 0

    baseboard: Optional[float] = None
    psu: Optional[float] = None
    safe_mcu: Optional[float] = None
    motors: Optional[list[float]] = None

    def _copy_mutable_fields(self, copied: WorkZoneCfg):
        copied.motors = self.motors.copy()
