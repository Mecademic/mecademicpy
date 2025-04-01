"""
This file contains the Mecademic Python Robot API.
This API is implemented by class Robot.
This class provides methods to connect to Mecademic robots, query robot information, control the robot, etc.
"""
# pylint: disable=wildcard-import, unused-wildcard-import, useless-parent-delegation

from __future__ import annotations

import contextlib
import pathlib
import sys
from typing import Optional, Tuple, Union

import deprecation

import mecademicpy.robot_sidecar_classes as rsc

from ._robot_base import _Robot, disconnect_on_exception_decorator
from .mx_robot_def import *
from .robot_classes import *
from .robot_trajectory_files import RobotTrajectories
from .tools import *

# importlib.metadata is new from v3.8
if sys.version_info < (3, 8):
    # From version lower than 3.8, use module
    from importlib_metadata import version
else:
    # Changed in version 3.10: importlib.metadata is no longer provisional.
    from importlib.metadata import version

__version__ = version('mecademicpy')


class Robot(_Robot):
    """Class for controlling a Mecademic robot.
       See README.md for quick instructions on how to use this class."""

    def __init__(self):
        super().__init__()

        self._enable_synchronous_mode = False

    def RegisterCallbacks(self, callbacks: RobotCallbacks, run_callbacks_in_separate_thread: bool):
        """Register callback functions to be executed when corresponding event occurs.
           Callback functions are optional.
           This function must be used before connecting to the robot.
           To attach/detect individual callbacks after connecting to the robot, use RegisterCallback/UnregisterCallback

        Parameters
        ----------
        callbacks : RobotCallbacks object
            Object containing all callback functions.
        run_callbacks_in_separate_thread : bool
            If true, callbacks are run automatically in thread. If false, RunCallbacks must be used.
            **Running callbacks in a separate thread means the user application MUST BE THREAD SAFE!**
        """
        return super().RegisterCallbacks(callbacks, run_callbacks_in_separate_thread)

    def RegisterCallback(self, callback_name: str, callback_method: Callable[[], None]):
        """Register one callback functions to be executed when corresponding event occurs.
           Callback functions are optional.

        Parameters
        ----------
        callback_name : Name of the callback to attach. Refer to class RobotCallbacks for available callbacks.
        callback_method : Callback method to attach to the specified callback.
        """
        return super().RegisterCallback(callback_name, callback_method)

    def UnregisterCallbacks(self):
        """Unregister callback functions and terminate callback handler thread if applicable.

        """
        return super().UnregisterCallbacks()

    def UnregisterCallback(self, callback_name: str):
        """Unregister one callback functions.

        Parameters
        ----------
        callback_name : Name of the callback to unregister. Refer to class RobotCallbacks for available callbacks.
        """
        return super().UnregisterCallback(callback_name)

    def RunCallbacks(self):
        """Run all triggered callback functions.
           Calling this function is required only when RegisterCallback option run_callbacks_in_separate_thread
           has been set to False (when True, callbacks are automatically called from a background thread).

        """
        if self._callback_thread:
            raise InvalidStateError(
                'Cannot call RunCallbacks since callback handler is already running in separate thread.')

        # Setting timeout=0 means we don't block on an empty queue.
        self._handle_callbacks(polling=True)

    def Connect(self,
                address: str = MX_DEFAULT_ROBOT_IP,
                enable_synchronous_mode: bool = False,
                disconnect_on_exception: bool = True,
                monitor_mode: bool = False,
                timeout=1.0):
        """Attempt to connect to a Mecademic Robot.
           This function is synchronous (awaits for success or timeout) even when using this class in asynchronous mode
           (see enable_synchronous_mode below).

        Parameters
        ----------
        address : string
            The IP address associated to the Mecademic Robot.
        enable_synchronous_mode : bool
            Select synchronous or asynchronous mode for this class.
            See SetSynchronousMode for details.
        disconnect_on_exception : bool
            If true, will attempt to disconnect from the robot on exception from api call.
            Also note that the robot will automatically pause motion whenever a disconnection occurs.
        monitor_mode : bool
            If true, command connection will not be established, only monitoring connection.
        timeout : float
            Time allowed to try connecting to the robot before this function returns.

        """
        return super()._Connect(address,
                                enable_synchronous_mode,
                                disconnect_on_exception,
                                monitor_mode,
                                offline_mode=False,
                                timeout=timeout)

    def Disconnect(self):
        """Disconnects Mecademic Robot object from the Mecademic robot.
           This function is synchronous (awaits for disconnection or timeout) even when connected in asynchronous mode.

        """
        return super().Disconnect()

    def IsConnected(self) -> bool:
        """Tells if we're actually connected to the robot"""
        return super().IsConnected()

    def IsControlling(self) -> bool:
        """Tells if we're actually connected to the robot in 'control' mode """
        return super().IsControlling()

    def IsSynchronousMode(self) -> bool:
        """Tells if we're actually connected to the robot in 'synchronous' mode
            (see doc for method Connect for details) """
        return super().IsSynchronousMode()

    # mx:export_to=robot_sidecar_globals.py
    def IsAllowedToMove(self) -> bool:
        """Tells if the robot is currently allowed to be moved (i.e. homed, or activated in recovery mode)"""
        can_move = False
        with self._main_lock:
            can_move = self._robot_status.homing_state or (self._robot_status.activation_state
                                                           and self._robot_events.on_activate_recovery_mode.is_set())
        return can_move

    def SetSynchronousMode(self, sync_mode: bool = True):
        """Changes synchronous mode option.
           If True, function calls in this class will be blocking until the robot has finished executing the
           command. Note that no blending is possible in this mode.
           If False, function calls in this class will post requests to the robot and return immediately before
           the robot has received/processed the commands.
           Method WaitIdle (among other things) can be used in that case to wait for robot to complete posted commands.

           Note that disabling synchronous mode will not awake thread already awaiting on synchronous operations.

        Parameters
        ----------
        sync_mode : bool, optional
            Synchronous mode enabled (else asynchronous mode), by default True
        """
        self._enable_synchronous_mode = sync_mode

    def Sync(self):
        """ Synchronize with the robot (send a request and await for the response).
            This can be useful to make sure that no response from previous asynchronous requests are in the network
            pipeline and would be received later.
            Note: This Sync method is only valid for synchronization with commands that get an immediate response.
                  It will not work with commands related to the motion queue because their response may be received
                  asynchronously later (after the response to this "Sync")
        """
        self._send_sync_command(command=None)  # Note: This will simply send the "sync" command

    def ConnectionWatchdog(self, timeout: float, message: Optional[str] = None):
        """Enable, refresh or disable the connection watchdog.
           This function is non-blocking.

           To enable the connection watchdog, call this function with a non-zero timeout.
           Then periodically call this function again with a non-zero timeout before expiry of previous timeout.
           To disable the connection watchdog, call this function with a timeout of zero.

           If the connection watchdog is enabled and the robot does not receive a refresh before the specified timeout,
           it will close the socket connections with this application, raise MX_SAFE_STOP_CONNECTION_DROPPED safety
           stop condition and pause motion (if robot is activated).

           Note that if the connection watchdog is not enabled but the socket is closed while the robot is moving,
           the robot will also raise the MX_SAFE_STOP_CONNECTION_DROPPED safety stop condition and pause motion.
           But note that a socket may take a very long time to detect a dropped connection upon some type of network
           failures and thus usage of the connection watchdog feature is recommended to ensure that the robot quickly
           stops moving in all situations when the application can no longer communicate with it.

           The application may validate that the connection watchdog is active on the robot using the watchdog state,
           located in RobotStatus.connection_watchdog_active.

        Args:
            timeout (float): Connection watchdog timeout (in seconds) to enable/refresh (or 0 to disable the watchdog)
            message (Optional[str]): Optional message to print in the robot log in case the connection times-out after
                                     this call to ConnectionWatchdog. This can be used to help determine what the
                                     application was doing when the connection has timed out.
        """
        return super().ConnectionWatchdog(timeout, message)

    def AutoConnectionWatchdog(self, enable: bool, timeout: float = 0, message: Optional[str] = None):
        """Enable automatic connection watchdog managed by this Robot class.
           See method ConnectionWatchdog for detailed explanations about the connection watchdog.

           The automatic connection watchdog is an alternative to ConnectionWatchdog which is simpler for the
           application because it does not have to manage periodic calling of ConnectionWatchdog to refresh the timer.

           When automatic connection watchdog is enabled, the Robot class will automatically enable then refresh the
           connection watchdog by calling ConnectionWatchdog at appropriate intervals.
           The connection timeout is automatically chosen based on monitoring interval (see SetMonitoringInterval).

        Args:
            enable (bool): Enable (else disable) the automatic connection watchdog refresh
            timeout (float): Connection watchdog to set as a last call to the robot by this function.
                             Default is 0 (connection watchdog is completely disabled).
                             If non-zero, don't forget to call ConnectionWatchdog afterwards because the watchdog will
                             still be active on the robot.
            message (Optional[str]): See ConnectionWatchdog for details
        """
        return super().AutoConnectionWatchdog(enable, timeout, message)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def ActivateRobot(self):
        """Activate the robot."""
        return super().ActivateRobot()

    # mx:export_to=robot_sidecar_globals.py
    def DeactivateRobot(self):
        """Deactivate the robot."""
        return super().DeactivateRobot()

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def Home(self):
        """Home the robot."""
        return super().Home()

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def ActivateAndHome(self):
        """Utility function that combines activate and home."""
        super().ActivateRobot()
        super().Home()

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def PauseMotion(self):
        """Immediately pause robot motion. """
        return super().PauseMotion()

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def ResumeMotion(self):
        """Un-pause robot motion."""
        return super().ResumeMotion()

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def ClearMotion(self):
        """Clear the motion queue, includes implicit PauseMotion command."""
        return super().ClearMotion()

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def MoveJoints(self, *args: float):
        """Move the robot by specifying each joint's target angular position.

        Parameters
        ----------
        joint_1...joint_n : float
            Desired joint angles in degrees.

        """
        expect_count = self._robot_info.num_joints
        if len(args) != expect_count:
            raise ValueError(
                f'MoveJoints: Incorrect number of joints sent {len(args)}, command. expecting: {expect_count}.')

        self._send_motion_command('MoveJoints', args)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def MoveJointsRel(self, *args: float):
        """Move the robot relative to current position by specifying each joint's offset angular position.

        Parameters
        ----------
        joint_1...joint_n : float
            Desired joint angles offsets in degrees.

        """
        expect_count = self._robot_info.num_joints
        if len(args) != expect_count:
            raise ValueError(
                f'MoveJointsRel: Incorrect number of joints sent {len(args)}, command. expecting: {expect_count}.')

        self._send_motion_command('MoveJointsRel', args)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def MoveJointsVel(self, *args: float):
        """Moves joints at desired velocities.

        Parameters
        ----------
        joint_1...joint_n : float
            Desired joint velocities in degrees per second.

        """
        expect_count = self._robot_info.num_joints
        if len(args) != expect_count:
            raise ValueError(
                f'MoveJointsVel: Incorrect number of joints sent {len(args)}, command. expecting: {expect_count}.')

        self._send_motion_command('MoveJointsVel', args)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def MovePose(self, x: float, y: float, z: float, alpha: float = None, beta: float = None, gamma: float = None):
        """Move robot's tool to an absolute Cartesian position (non-linear move, but all joints arrive simultaneously).

        Parameters
        ----------
        x, y, z : float
            Desired end effector coordinates in mm.
        alpha, beta, gamma
            Desired end effector orientation in degrees.
            On 4-axes robots (like Mcs500), alpha and beta values are not used and can be omitted.
            Examples for 4-axes robots:
                - MovePose(200, 10, 100, 45)
                - MovePose(200, 10, 100, gamma=45)

        """
        [alpha, beta, gamma] = self._normalize_cart_cmd_args(alpha, beta, gamma)
        self._send_motion_command('MovePose', [x, y, z, alpha, beta, gamma])

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def MoveJump(self, x: float, y: float, z: float, alpha: float = None, beta: float = None, gamma: float = None):
        """Move robot to desired position but performing an arch of specified height between start and end positions.
        This command is similar to MovePose (i.e. using quickest non-linear path).
        The jump height parameters are configured using SetMoveJumpHeight and SetMoveJumpApproachVel.
        This command is very useful for 'pick-and-place' applications.

        Parameters
        ----------
        x, y, z : float
            Desired end effector coordinates in mm.
        alpha, beta, gamma
            Desired end effector orientation in degrees.
            On 4-axes robots (like Mcs500), alpha and beta values are not used and can be omitted.
            Examples for 4-axes robots:
                - MoveJump(200, 10, 100, 45)
                - MoveJump(200, 10, 100, gamma=45)

        """
        [alpha, beta, gamma] = self._normalize_cart_cmd_args(alpha, beta, gamma)
        self._send_motion_command('MoveJump', [x, y, z, alpha, beta, gamma])

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def MoveLin(self, x: float, y: float, z: float, alpha: float = None, beta: float = None, gamma: float = None):
        """Linearly move robot's tool to an absolute Cartesian position.

        Parameters
        ----------
        x, y, z : float
            Desired end effector coordinates in mm.
        alpha, beta, gamma
            Desired end effector orientation in degrees.
            On 4-axes robots (like Mcs500), alpha and beta values are not used and can be omitted.
            Examples for 4-axes robots:
                - MoveLin(200, 10, 100, 45)
                - MoveLin(200, 10, 100, gamma=45)

        """
        [alpha, beta, gamma] = self._normalize_cart_cmd_args(alpha, beta, gamma)
        self._send_motion_command('MoveLin', [x, y, z, alpha, beta, gamma])

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def MoveLinRelTrf(self, x: float, y: float, z: float, alpha: float = None, beta: float = None, gamma: float = None):
        """Linearly move robot's tool to a Cartesian position relative to current TRF position.

        Parameters
        ----------
        x, y, z : float
            Desired displacement in mm.
        alpha, beta, gamma
            Desired orientation change in deg.
            On 4-axes robots (like Mcs500), alpha and beta values are not used and can be omitted.
            Examples for 4-axes robots:
                - MoveLinRelTrf(200, 10, 100, 45)
                - MoveLinRelTrf(200, 10, 100, gamma=45)

        """
        [alpha, beta, gamma] = self._normalize_cart_cmd_args(alpha, beta, gamma)
        self._send_motion_command('MoveLinRelTrf', [x, y, z, alpha, beta, gamma])

    @deprecation.deprecated(deprecated_in="1.2.0",
                            removed_in="3.0.0",
                            current_version=__version__,
                            details="Use the 'MoveLinRelTrf' function instead")
    @disconnect_on_exception_decorator
    def MoveLinRelTRF(self, x: float, y: float, z: float, alpha: float = None, beta: float = None, gamma: float = None):
        """Deprecated use MoveLinRelTrf instead.
        """
        self.MoveLinRelTrf(x, y, z, alpha, beta, gamma)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def MoveLinRelWrf(self, x: float, y: float, z: float, alpha: float = None, beta: float = None, gamma: float = None):
        """Linearly move robot's tool to a Cartesian position relative to a reference frame that has the same
        orientation.

        Parameters
        ----------
        x, y, z : float
            Desired displacement in mm.
        alpha, beta, gamma
            Desired orientation change in deg.
            On 4-axes robots (like Mcs500), alpha and beta values are not used and can be omitted.
            Examples for 4-axes robots:
                - MoveLinRelWrf(200, 10, 100, 45)
                - MoveLinRelWrf(200, 10, 100, gamma=45)

        """
        [alpha, beta, gamma] = self._normalize_cart_cmd_args(alpha, beta, gamma)
        self._send_motion_command('MoveLinRelWrf', [x, y, z, alpha, beta, gamma])

    @deprecation.deprecated(deprecated_in="1.2.0",
                            removed_in="3.0.0",
                            current_version=__version__,
                            details="Use the 'MoveLinRelWrf' function instead")
    @disconnect_on_exception_decorator
    def MoveLinRelWRF(self, x: float, y: float, z: float, alpha: float = None, beta: float = None, gamma: float = None):
        """Deprecated use MoveLinRelWrf instead.
        """
        self.MoveLinRelWrf(x, y, z, alpha, beta, gamma)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def MoveLinVelTrf(self, x: float, y: float, z: float, alpha: float = None, beta: float = None, gamma: float = None):
        """Move robot's by Cartesian velocity relative to the TRF.

           Joints will move for a time controlled by velocity timeout (SetVelTimeout).

        Parameters
        ----------
        x, y, z : float
            Desired velocity in mm/s.
        alpha, beta, gamma
            Desired angular velocity in degrees/s.
            On 4-axes robots (like Mcs500), alpha and beta values are not used and can be omitted.
            Examples for 4-axes robots:
                - MoveLinVelTrf(200, 10, 100, 45)
                - MoveLinVelTrf(200, 10, 100, gamma=45)

        """
        [alpha, beta, gamma] = self._normalize_cart_cmd_args(alpha, beta, gamma)
        self._send_motion_command('MoveLinVelTrf', [x, y, z, alpha, beta, gamma])

    @deprecation.deprecated(deprecated_in="1.2.0",
                            removed_in="3.0.0",
                            current_version=__version__,
                            details="Use the 'MoveLinVelTrf' function instead")
    @disconnect_on_exception_decorator
    def MoveLinVelTRF(self, x: float, y: float, z: float, alpha: float = None, beta: float = None, gamma: float = None):
        """Deprecated use MoveLinVelTrf instead

        """
        self.MoveLinVelTrf(x, y, z, alpha, beta, gamma)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def MoveLinVelWrf(self, x: float, y: float, z: float, alpha: float = None, beta: float = None, gamma: float = None):
        """Move robot's by Cartesian velocity relative to the WRF.

           Joints will move for a time controlled by velocity timeout (SetVelTimeout).

        Parameters
        ----------
        x, y, z : float
            Desired velocity in mm/s.
        alpha, beta, gamma
            Desired angular velocity in degrees/s.
            On 4-axes robots (like Mcs500), alpha and beta values are not used and can be omitted.
            Examples for 4-axes robots:
                - MoveLinVelWrf(200, 10, 100, 45)
                - MoveLinVelWrf(200, 10, 100, gamma=45)

        """
        [alpha, beta, gamma] = self._normalize_cart_cmd_args(alpha, beta, gamma)
        self._send_motion_command('MoveLinVelWrf', [x, y, z, alpha, beta, gamma])

    @deprecation.deprecated(deprecated_in="1.2.0",
                            removed_in="3.0.0",
                            current_version=__version__,
                            details="Use the 'MoveLinVelWrf' function instead")
    @disconnect_on_exception_decorator
    def MoveLinVelWRF(self, x: float, y: float, z: float, alpha: float = None, beta: float = None, gamma: float = None):
        """Deprecated use MoveLinVelWrf instead

        """
        self.MoveLinVelWrf(x, y, z, alpha, beta, gamma)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetVelTimeout(self, t: float):
        """Maximum time the robot will continue to move after a velocity move command was sent.

        (Can be stopped earlier by sending a velocity command with 0 velocity values.)

        Parameters
        ----------
        t : float
            Desired duration for velocity-mode motion commands.

        """
        self._send_motion_command('SetVelTimeout', [t])

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetConf(self, shoulder: int = None, elbow: int = None, wrist: int = None):
        """Manually set inverse kinematics options (and disable auto-conf).
            On 4-axes robots (like Mcs500), shoulder and wrist values are not used and can be omitted.
            Examples for 4-axes robots:
                - SetConf(1)
                - SetConf(elbow=1)

        Parameters
        ----------
        shoulder : +1 or -1
            Shoulder inverse kinematics parameter.
        elbow : +1 or -1
            Elbow inverse kinematics parameter.
        wrist : +1 or -1
            Wrist inverse kinematics parameter.

        """
        [shoulder, elbow, wrist] = self._normalize_conf_cmd_args(shoulder, elbow, wrist)
        self._send_motion_command('SetConf', [shoulder, elbow, wrist])

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetAutoConf(self, e: int):
        """Enable or disable auto-conf (automatic selection of inverse kinematics options).

        Parameters
        ----------
        e : bool
            If true, robot will automatically choose the best configuration for the desired pose.

        """
        self._send_motion_command('SetAutoConf', [int(e)])

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetConfTurn(self, n: int):
        """Manually set the last joint turn configuration parameter.

        Parameters
        ----------
        n : integer
            The turn number for joint 6.

        """
        self._send_motion_command('SetConfTurn', [n])

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetAutoConfTurn(self, e: int):
        """Enable or disable auto-conf (automatic selection of inverse kinematics options) for joint 6.

        Parameters
        ----------
        e : bool
            If true, robot will automatically choose the best configuration for the desired pose.

        """
        self._send_motion_command('SetAutoConfTurn', [int(e)])

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetBlending(self, p: float):
        """Set percentage of blending between consecutive movements in the same mode (velocity or cartesian).

        Note: There can't be blending between joint mode and Cartesian mode moves.

        Parameters
        ----------
        p : float
            Percentage blending between actions.

        """
        self._send_motion_command('SetBlending', [p])

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetCartAcc(self, p: float):
        """Set target acceleration (linear and angular) during MoveLin commands.

        Parameters
        ----------
        p : float
            Percentage of maximum acceleration.

        """
        self._send_motion_command('SetCartAcc', [p])

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetCartAngVel(self, w: float):
        """Set maximum angular velocity during MoveLin commands.

        Note: Actual angular velocity may be lower if necessary to avoid exceeding maximum joint velocity.

        Parameters
        ----------
        w : float
            Maximum angular velocity in deg/s.

        """
        self._send_motion_command('SetCartAngVel', [w])

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetCartLinVel(self, v: float):
        """Set maximum linear velocity during MoveLin commands.

        Note: Actual linear velocity may be lower if necessary to avoid exceeding maximum joint velocity.

        Parameters
        ----------
        v : float
            Maximum angular velocity in deg/s.

        """
        self._send_motion_command('SetCartLinVel', [v])

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetJointAcc(self, p: float):
        """Set target joint acceleration during MoveJoints commands.

        Parameters
        ----------
        p : float
            Target acceleration, in percent.

        """
        self._send_motion_command('SetJointAcc', [p])

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetJointVel(self, p: float):
        """Set target joint velocity during MoveJoints commands.

        Parameters
        ----------
        p : float
            Target joint velocity, in percent.

        """
        self._send_motion_command('SetJointVel', [p])

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetJointVelLimit(self, p: float):
        """Change the safety joint velocity limit that is applied to all type of moves.
        In joint space:     The joints velocity will be capped to this limit
                            even if SetJointVel or MoveJointsVel request higher speed.
        With linear moves:  The joints velocity will be capped to this limit regardless of the requested
                            linear or angular velocity.

        Parameters
        ----------
        p : float
            Joint velocity limit in percent.

        """
        self._send_motion_command('SetJointVelLimit', [p])

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetMoveMode(self, mode=MxMoveMode.MX_MOVE_MODE_VELOCITY):
        """Select the current move mode (velocity-based or time-based) for following mode commands.
        The default mode when robot is activated is velocity-based.

        Parameters
        ----------
        mode : MxMoveMode
            The move mode to use.

        """
        self._send_motion_command('SetMoveMode', [mode])

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetMoveDurationCfg(self, severity=MxEventSeverity.MX_EVENT_SEVERITY_WARNING):
        """Set the configuration of the time-based movements.

        Parameters
        ----------
        severity : MxEventSeverity
            Severity level when the requested move duration is too short for the robot.

        """
        self._send_motion_command('SetMoveDurationCfg', [severity])

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetMoveDuration(self, t: float):
        """Set the duration of move commands when the move mode is time-based (see SetMoveMove).

        Parameters
        ----------
        t : float
            Move duration in seconds.

        """
        self._send_motion_command('SetMoveDuration', [t])

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    # pylint: disable=invalid-name
    def SetMoveJumpHeight(self,
                          startHeight: float = MX_MOVE_JUMP_DEFAULT_HEIGHT_MM,
                          endHeight: float = MX_MOVE_JUMP_DEFAULT_HEIGHT_MM,
                          minHeight: float = 0,
                          maxHeight: float = 102):
        """Set height parameters for the MoveJump command (in mm).
        Height parameters control how high the robot moves vertically at start, middle and end.
        Height values provided can be positive or negative (relative to WRF's Z axis).
        Space between minimum and maximum allows the robot flexibility to choose the optimal path.
        An error is raised if joint limits prevent the robot from respecting minimum heights.

        Parameters
        ----------
        startHeight : float
            Height (in mm) to move from start position before starting lateral move, relative to starting position.
        endHeight : float
            Height (in mm) to reach after lateral move, before moving down to end position, relative to end position.
        minHeight : float
            Minimum height (in mm) to reach while performing the lateral move, applied to both,
            start and end of the jump, relative to start/end position respectively.
        maxHeight : float
            Maximum height (in mm) allowed along the whole MoveJump path, applied to both,
            start and end of the jump, relative to start/end position respectively.
            The robot may not reach that height if optimal path does not need to go that high.

        """
        self._send_motion_command('SetMoveJumpHeight', [startHeight, endHeight, minHeight, maxHeight])

    # mx:export_to=robot_sidecar_globals.py
    #pylint: disable=invalid-name
    @disconnect_on_exception_decorator
    def SetMoveJumpApproachVel(self,
                               startVel: float = MX_MOVE_JUMP_DEFAULT_APPROACH_VEL_MM_SEC,
                               startDist: float = MX_MOVE_JUMP_DEFAULT_APPROACH_DIST_MM,
                               endVel: float = MX_MOVE_JUMP_DEFAULT_APPROACH_VEL_MM_SEC,
                               endDist: float = MX_MOVE_JUMP_DEFAULT_APPROACH_DIST_MM):
        """Set the maximum allowed velocity (and length) of the start and end approach of a MoveJump.
        This allows the robot to move slower than configured joint velocity for some distance from
        start and end points, during the 'vertical' portion of the MoveJump.
        This is a velocity limit i.e. it will have an effect only if slower than current JointVel.

        Parameters
        ----------
        startVel : float
            Maximum velocity (in mm/s) allowed near start positions
            (as a velocity limit i.e. it will have an effect only if slower than current joint velocity).
            Use 0 for 'unlimited'.
        startDist : float
            Approach distance (in mm) from start position, i.e. the portion of vertical move where
            the StartVel limit is applied.
            Automatically limited to jump height (SetMoveJumpHeight).
        endVel : float
            Maximum velocity (in mm/s) allowed near end position
            (as a velocity limit i.e. it will have an effect only if slower than current joint velocity).
            Use 0 for 'unlimited'.
        endDist : float
            Approach distance (in mm) from end position, i.e. the portion of vertical move where
            the EndVel limit is applied.
            Automatically limited to jump height (SetMoveJumpHeight).

        """
        self._send_motion_command('SetMoveJumpApproachVel', [startVel, startDist, endVel, endDist])

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetTrf(self, x: float, y: float, z: float, alpha: float = None, beta: float = None, gamma: float = None):
        """Set the TRF (tool reference frame) Cartesian position.

        Parameters
        ----------
        x, y, z : float
            Desired reference coordinates in mm.
        alpha, beta, gamma
            Desired reference orientation in degrees.
            On 4-axes robots (like Mcs500), alpha and beta values are not used and can be omitted.
            Examples for 4-axes robots:
                - SetTrf(200, 10, 100, 45)
                - SetTrf(200, 10, 100, gamma=45)

        """
        [alpha, beta, gamma] = self._normalize_cart_cmd_args(alpha, beta, gamma)
        self._send_motion_command('SetTrf', [x, y, z, alpha, beta, gamma])

    @deprecation.deprecated(deprecated_in="1.2.0",
                            removed_in="3.0.0",
                            current_version=__version__,
                            details="Use the 'SetTrf' function instead")
    @disconnect_on_exception_decorator
    def SetTRF(self, x: float, y: float, z: float, alpha: float = None, beta: float = None, gamma: float = None):
        """Deprecated use SetTrf instead

        """
        self.SetTrf(x, y, z, alpha, beta, gamma)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetWrf(self, x: float, y: float, z: float, alpha: float = None, beta: float = None, gamma: float = None):
        """Set the WRF (world reference frame) Cartesian position.

        Parameters
        ----------
        x, y, z : float
            Desired reference coordinates in mm.
        alpha, beta, gamma
            Desired reference orientation in degrees.
            On 4-axes robots (like Mcs500), alpha and beta values are not used and can be omitted.
            Examples for 4-axes robots:
                - SetWrf(200, 10, 100, 45)
                - SetWrf(200, 10, 100, gamma=45)

        """
        [alpha, beta, gamma] = self._normalize_cart_cmd_args(alpha, beta, gamma)
        self._send_motion_command('SetWrf', [x, y, z, alpha, beta, gamma])

    @deprecation.deprecated(deprecated_in="1.2.0",
                            removed_in="3.0.0",
                            current_version=__version__,
                            details="Use the 'SetWrf' function instead")
    @disconnect_on_exception_decorator
    def SetWRF(self, x: float, y: float, z: float, alpha: float = None, beta: float = None, gamma: float = None):
        """Deprecated use SetWrf instead

        """
        self.SetWrf(x, y, z, alpha, beta, gamma)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetCheckpoint(self, n: int) -> InterruptableEvent:
        """Set checkpoint with desired id.
           This function then returns an InterruptableEvent that can be used at any time to
           wait until the robot's motion queue execution reaches this checkpoint.
           This method is non-blocking whether robot connection is in asynchronous or synchronous mode.
           Therefore, it is required to use the wait() method of the return object to catch the checkpoint event.

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
            self._check_internal_states()
            assert MX_CHECKPOINT_ID_MIN <= n <= MX_CHECKPOINT_ID_MAX
            return self._set_checkpoint_impl(n)

    @disconnect_on_exception_decorator
    def ExpectExternalCheckpoint(self, n: int) -> InterruptableEvent:
        """Expect the robot to receive a checkpoint with given id (e.g. from saved program).
           Make sure to call this function before starting the saved program otherwise the checkpoint could be missed.

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
            self._check_internal_states()
            assert MX_CHECKPOINT_ID_MIN <= n <= MX_CHECKPOINT_ID_MAX
            return self._set_checkpoint_impl(n, send_to_robot=False)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def WaitGripperMoveCompletion(self, timeout: Optional[float] = None):
        """Wait until the most recent gripper move command has completed.

           Note that any move command in the motion queue following a gripper command will start executing at the same
           time the gripper starts moving.
           Thus, if you wish to wait for the gripper command to finish before continuing to move the robot, you must
           call WaitGripperMoveCompletion before posting subsequent motion commands.

           Note: This function is meant to be called immediately after sending a gripper move command (GripperOpen,
                 GripperClose or MoveGripper). This function will first wait for robot to be idle, then wait for
                 the gripper command completion.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the move to complete (in seconds).
        """
        return super().WaitGripperMoveCompletion(timeout)

    @disconnect_on_exception_decorator
    def GripperOpen(self):
        """Open the gripper.
           Note 1:  This command will not cause the robot to stop moving.

           Note 2:  This command will be executed at the beginning of the blending (or deceleration) period of
                    the previous command in the motion queue.
                    You can insert a Delay before moving the gripper to make sure that gripper move starts only
                    once the robot has stopped moving.

           Note 3:  The robot will not wait for gripper move to be completed before continuing with the next command
                    in the motion queue.
                    If you need to wait for the gripper move to be completed before continuing your program, do not
                    post any request in the motion queue after MoveGripper,
                    then call method WaitGripperMoveCompletion or WaitHoldingPart in your application
                    to know when your application can continue sending other move commands.

        """
        return super().GripperOpen()

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def GripperClose(self):
        """Close the gripper.
           Note 1:  This command will not cause the robot to stop moving.

           Note 2:  This command will be executed at the beginning of the blending (or deceleration) period of
                    the previous command in the motion queue.
                    You can insert a Delay before moving the gripper to make sure that gripper move starts only
                    once the robot has stopped moving.

           Note 3:  The robot will not wait for gripper move to be completed before continuing with the next command
                    in the motion queue.
                    If you need to wait for the gripper move to be completed before continuing your program, do not
                    post any request in the motion queue after MoveGripper,
                    then call method WaitGripperMoveCompletion or WaitHoldingPart in your application
                    to know when your application can continue sending other move commands.

        """
        return super().GripperClose()

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def MoveGripper(self, target: Union[bool, float]):
        """Move the gripper to a target position.
           Note 1:  This command will not cause the robot to stop moving.

           Note 2:  This command will be executed at the beginning of the blending (or deceleration) period of
                    the previous command in the motion queue.
                    You can insert a Delay before moving the gripper to make sure that gripper move starts only
                    once the robot has stopped moving.

           Note 3:  The robot will not wait for gripper move to be completed before continuing with the next command
                    in the motion queue.
                    If you need to wait for the gripper move to be completed before continuing your program, do not
                    post any request in the motion queue after MoveGripper,
                    then call method WaitGripperMoveCompletion or WaitHoldingPart in your application
                    to know when your application can continue sending other move commands.

           If the target specified is a bool, it indicates if the target position is the opened (True, GRIPPER_OPEN)
           or closed (False, GRIPPER_CLOSE) position.
           Otherwise the target position indicates the opening of the gripper, in mm from the most closed position.

        Corresponds to text API calls "GripperOpen" / "GripperClose" / "MoveGripper".


        Parameters
        ----------
        target : bool or float
            bool type: Open or close the gripper (GRIPPER_OPEN or GRIPPER_CLOSE)
            float type: The gripper's target position, in mm from the most closed position.

        """
        return super().MoveGripper(target)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetGripperForce(self, p: float):
        """Set the gripper's force in percent.

        Parameters
        ----------
        p : float
            The desired force in percent.

        """
        self._send_motion_command('SetGripperForce', [p])

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetGripperVel(self, p: float):
        """Set the gripper's velocity in percent.

        Parameters
        ----------
        p : float
            The desired velocity in percent.

        """
        self._send_motion_command('SetGripperVel', [p])

    # mx:export_to=robot_sidecar_globals.py
    #pylint: disable=invalid-name
    @disconnect_on_exception_decorator
    def SetGripperRange(self, closePos: float, openPos: float):
        """Set the gripper's range that will be used when calling GripperClose and GripperOpen.
           This function is useful for example to set a smaller (and thus quicker) movement range when it is not
           required to fully open the gripper to release objects. This is especially apparent on long-stroke grippers.

           Setting both values to 0 will reset the range to the maximum range found during homing.

        Parameters
        ----------
        closePos : float
            The position relative to the completely closed position that the gripper will move to when calling
            GripperClose. In mm.
        openPos : float
            The position relative to the completely closed position that the gripper will move to when calling
            GripperOpen. In mm.

        """
        self._send_motion_command('SetGripperRange', [closePos, openPos])

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetValveState(self, *args: Union[MxCmdValveState, int, str]):
        """Set the pneumatic module valve states.
           Note 1:  This command will not cause the robot to stop moving.

           Note 2:  This command will be executed at the beginning of the blending (or deceleration) period of
                    the previous command in the motion queue.
                    You can insert a Delay before changing the valves states to make sure that valves are changed
                    once the robot has stopped moving.

        Parameters
        ----------
        valve_1...valve_n :
            The desired state for valve:
                - MxCmdValveState.MX_VALVE_STATE_STAY (alternatively: -1, '*' or 'stay')
                - MxCmdValveState.MX_VALVE_STATE_CLOSE (alternatively: 0 or 'close')
                - MxCmdValveState.MX_VALVE_STATE_OPEN (alternatively: 1 or 'open')
            MPM500 pneumatic module has 2 valves.

        """
        self._send_motion_command('SetValveState', args)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def VacuumGrip(self):
        """Start applying vacuum to grip a part with the IO module's vacuum gripper (motion queue command).
           Note 1:  This command will not cause the robot to stop moving.

           Note 2:  This command will be executed at the beginning of the blending (or deceleration) period of
                    the previous command in the motion queue.
                    You can insert a Delay before calling VacuumGrip to make sure that vacuum is applied only
                    once the robot has stopped moving.

           Note 3:  The robot will NOT wait until robot is confirmed holding part before starting to execute the next
                    command in the motion queue.
                    If you need to wait for holding part confirmation before continuing your program, do not
                    post any request in the motion queue after VacuumGrip, then call method WaitHoldingPart
                    in your application and finally post following move commands to the robot.
        """
        self._send_motion_command('VacuumGrip')

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def VacuumGrip_Immediate(self):
        """Same as VacuumGrip but without going through robot's motion queue (immediately applied)"""
        self.VacuumGripReleaseImmediate(False)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def VacuumRelease(self):
        """Release part held by the IO module's vacuum gripper (motion queue command).
           Note 1:  This command will not cause the robot to stop moving.

           Note 2:  This command will be executed at the beginning of the blending (or deceleration) period of
                    the previous command in the motion queue.
                    You can insert a Delay before calling VacuumRelease to make sure that part is released only
                    once the robot has stopped moving.

           Note 3:  The robot will NOT wait until robot is confirmed released part before starting to execute the next
                    command in the motion queue.
                    If you need to wait for released part confirmation before continuing your program, do not
                    post any request in the motion queue after VacuumRelease, then call method WaitReleasedPart
                    in your application and finally post following move commands to the robot.
        """
        self._send_motion_command('VacuumRelease')

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def VacuumRelease_Immediate(self):
        """Same as VacuumRelease but without going through robot's motion queue (immediately applied)"""
        self.VacuumGripReleaseImmediate(True)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def WaitHoldingPart(self, timeout: Optional[float] = None):
        """Wait for the gripper (or vacuum gripper) to confirm it's holding part.

           Note that any move command in the motion queue following a gripper command will start executing at the same
           time the gripper starts moving.
           Thus, if you wish to wait for holding part confirmation before continuing to move the robot, you must
           call WaitHoldingPart before posting subsequent motion commands.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the holding part confirmation (in seconds).
        """
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_holding_part.wait(timeout=timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def WaitReleasedPart(self, timeout: Optional[float] = None):
        """Wait for the gripper (or vacuum gripper) to confirm it has released part.

           Note that any move command in the motion queue following a gripper command will start executing at the same
           time the gripper starts moving.
           Thus, if you wish to wait for released part confirmation before continuing to move the robot, you must
           call WaitReleasedPart before posting subsequent motion commands.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the released part confirmation (in seconds).
        """
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_released_part.wait(timeout=timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def WaitPurgeDone(self, timeout: Optional[float] = None):
        """Wait for the vacuum gripper purge to be done after releasing part.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the vacuum purge done confirmation (in seconds).
        """
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_vacuum_purge_done.wait(timeout=timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetVacuumThreshold(self, hold_threshold: float, release_threshold: float):
        """Set vacuum pressure level thresholds for considering holding or releasing a part (motion queue command).
           Part is considered held once the vacuum level reaches the 'hold threshold'.
           Part is considered released once the vacuum pressure passes the 'release threshold'
           (i.e. closer to zero Kpa than 'release threshold').
           The 'Hold threshold' must thus be under (more negative Kpa) than the 'release threshold'.
           Use values (0,0) to reset to default values.

        Parameters
        ----------
        hold_threshold : float
            Vacuum pressure threshold to consider holding part in kPa.
        openPos : float
            Vacuum pressure threshold to consider part released in kPa.

        """
        self._send_motion_command('SetVacuumThreshold', [hold_threshold, release_threshold])

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetVacuumThreshold_Immediate(self, hold_threshold: float, release_threshold: float):
        """Same as SetVacuumThreshold but without going through robot's motion queue (immediately applied)"""
        self._send_immediate_command('SetVacuumThreshold_Immediate', [hold_threshold, release_threshold], None)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetVacuumPurgeDuration(self, duration: float):
        """Set duration (in seconds) of the positive air pressure applied when VacuumRelease is called
           (motion queue command).
           This positive air pressure period helps ejecting the held part.
           Use value -1 to restore default purge duration'.

        Parameters
        ----------
        duration : float
            Duration (in seconds) of the vacuum purge.

        """
        self._send_motion_command('SetVacuumPurgeDuration', [duration])

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetVacuumPurgeDuration_Immediate(self, duration: float):
        """Same as SetVacuumPurgeDuration but without going through robot's motion queue (immediately applied)"""
        self._send_immediate_command('SetVacuumPurgeDuration_Immediate', [duration], None)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetOutputState(self, bank_id: MxIoBankId, *output_states: Union[MxDigitalIoState, int, str]):
        """Set the digital output states for the specified IO bank (motion queue command).
           Note 1:  This command will not cause the robot to stop moving.

           Note 2:  This command will be executed at the beginning of the blending (or deceleration) period of
                    the previous command in the motion queue.
                    You can insert a Delay before changing the IO states to make sure that they change
                    once the robot has stopped moving.

        Parameters
        ----------
        bank_id : MxIoBankId
            The IO bank Id to set output states for.
        output_states_1...output_states_n : MxDigitalIoState, int, str
            The desired IO state (the number of available outputs depends on the chosen bank id):
                - MxDigitalIoState.MX_DIGITAL_IO_STATE_STAY (alternatively: -1, '*' or 'stay')
                - MxDigitalIoState.MX_DIGITAL_IO_STATE_0 (alternatively: 0 or 'off')
                - MxDigitalIoState.MX_DIGITAL_IO_STATE_1 (alternatively: 1 or 'on')
        """
        super().SetOutputState(bank_id, *output_states)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetOutputState_Immediate(self, bank_id: MxIoBankId, *output_states: Union[MxDigitalIoState, int, str]):
        """Same as SetOutputState but without going through robot's motion queue (immediately applied)"""
        super().SetOutputState_Immediate(bank_id, *output_states)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def WaitForAnyCheckpoint(self, timeout: float = None):
        """Pause program execution until any checkpoint has been received from the robot.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the checkpoint (in seconds).
        """
        super().WaitForAnyCheckpoint(timeout)

    @disconnect_on_exception_decorator
    def WaitConnected(self, timeout: float = None):
        """Pause program execution until robot is connected.
           Since the Connect() command is always blocking, this command is only useful if a separate thread wants to
           wait for the connection to be established.

        Parameters
        ----------
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).
        """
        super().WaitConnected(timeout)

    @disconnect_on_exception_decorator
    def WaitDisconnected(self, timeout: float = None):
        """Pause program execution until the robot is disconnected.
           Since the Disconnect() command is always blocking, this command is only useful if a separate thread wants to
           wait for the disconnection.
        Parameters
        ----------
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).
        """
        super().WaitDisconnected(timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def WaitActivated(self, timeout: float = None):
        """Pause program execution until the robot is activated.

        Parameters
        ----------
        timeout : float, by default 30
            Maximum time to spend waiting for the event (in seconds).
        """
        super().WaitActivated(timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def WaitDeactivated(self, timeout: float = None):
        """Pause program execution until the robot is deactivated.

        Parameters
        ----------
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).
        """
        super().WaitDeactivated(timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def WaitHomed(self, timeout: float = None):
        """Pause program execution until the robot is homed.

        Parameters
        ----------
        timeout : float, by default 40
            Maximum time to spend waiting for the event (in seconds).
        """
        super().WaitHomed(timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def WaitSimActivated(self, timeout: float = None):
        """Pause program execution until the robot simulation mode is activated.

        Parameters
        ----------
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).

        """
        super().WaitSimActivated(timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def WaitSimDeactivated(self, timeout: float = None):
        """Pause program execution until the robot simulation mode is deactivated.

        Parameters
        ----------
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).
        """
        super().WaitSimDeactivated(timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def WaitExtToolSimActivated(self, timeout: float = None):
        """Pause program execution until the robot external tool simulation mode is activated.

        Parameters
        ----------
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).

        """
        super().WaitExtToolSimActivated(timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def WaitExtToolSimDeactivated(self, timeout: float = None):
        """Pause program execution until the robot external tool simulation mode is deactivated.

        Parameters
        ----------
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).
        """
        super().WaitExtToolSimDeactivated(timeout)

    # pylint: disable=unused-argument
    @disconnect_on_exception_decorator
    def WaitIoSimEnabled(self, bank_id: MxIoBankId = MxIoBankId.MX_IO_BANK_ID_IO_MODULE, timeout: float = None):
        """Pause program execution until the robot PSU IO simulation mode is enabled.

        Parameters
        ----------
        bank_id : MxIoBankId, MxIoBankId.MX_IO_BANK_ID_IO_MODULE
            Id of the IO module to wait for.
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).
        """
        super().WaitIoSimEnabled(timeout)

    # pylint: disable=unused-argument
    @disconnect_on_exception_decorator
    def WaitIoSimDisabled(self, bank_id: MxIoBankId = MxIoBankId.MX_IO_BANK_ID_IO_MODULE, timeout: float = None):
        """Pause program execution until the robot IO simulation mode is disabled.

        Parameters
        ----------
        bank_id : MxIoBankId, MxIoBankId.MX_IO_BANK_ID_IO_MODULE
            Id of the IO module to wait for.
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).
        """
        super().WaitIoSimDisabled(timeout)

    def IsDesiredOutputState(self, bank_id: MxIoBankId, *expected_states: Union[MxDigitalIoState, int, str]) -> bool:
        """Tells if the current digital output states matches the desired state.
           Note that, here, a desired state of '*', 'stay' or -1 is interpreted as "don't compare".

        Parameters
        ----------
        bank_id : MxIoBankId
            The IO bank Id to validate output states for.
        expected_states : Union[MxDigitalIoState, int, str]
            The desired IO state (the number of available outputs depends on the chosen bank id):
                - MxDigitalIoState.MX_DIGITAL_IO_STATE_STAY (alternatively: -1, '*' or 'stay') -> Will not be compared
                - MxDigitalIoState.MX_DIGITAL_IO_STATE_0 (alternatively: 0 or 'off')
                - MxDigitalIoState.MX_DIGITAL_IO_STATE_1 (alternatively: 1 or 'on')

        Return
        ------
        True if current output states match desired state.
        """
        return self.IsDesiredIoState(bank_id, False, *expected_states)

    # mx:export_to=robot_sidecar_globals.py
    def WaitOutputState(self,
                        bank_id: MxIoBankId,
                        *expected_states: Union[MxDigitalIoState, int, str],
                        timeout: float = None):
        """Wait until the current digital output states matches the desired state.
           Note that, here, a desired state of '*', 'stay' or -1 is interpreted as "don't compare".

        Parameters
        ----------
        bank_id : MxIoBankId
            The IO bank Id to wait output states for.
        expected_states : Union[MxDigitalIoState, int, str]
            The desired IO state (the number of available outputs depends on the chosen bank id):
                - MxDigitalIoState.MX_DIGITAL_IO_STATE_STAY (alternatively: -1, '*' or 'stay') -> Will not be compared
                - MxDigitalIoState.MX_DIGITAL_IO_STATE_0 (alternatively: 0 or 'off')
                - MxDigitalIoState.MX_DIGITAL_IO_STATE_1 (alternatively: 1 or 'on')

        Raises
        ------
        TimeoutException
            If timeout was reached before the digital outputs state is as desired.
        """
        return self.WaitIOState(bank_id, False, *expected_states, timeout)

    def IsDesiredInputState(self, bank_id: MxIoBankId, *expected_states: Union[MxDigitalIoState, int, str]) -> bool:
        """Tells if the current digital input states matches the desired state.
           Note that, here, a desired state of '*', 'stay' or -1 is interpreted as "don't compare".

        Parameters
        ----------
        bank_id : MxIoBankId
            The IO bank Id to validate input states for.
        expected_states : Union[MxDigitalIoState, int, str]
            The desired IO state (the number of available inputs depends on the chosen bank id):
                - MxDigitalIoState.MX_DIGITAL_IO_STATE_STAY (alternatively: -1, '*' or 'stay') -> Will not be compared
                - MxDigitalIoState.MX_DIGITAL_IO_STATE_0 (alternatively: 0 or 'off')
                - MxDigitalIoState.MX_DIGITAL_IO_STATE_1 (alternatively: 1 or 'on')

        Return
        ------
        True if current input states match desired state.
        """
        return self.IsDesiredIoState(bank_id, True, *expected_states)

    # mx:export_to=robot_sidecar_globals.py
    def WaitInputState(self,
                       bank_id: MxIoBankId,
                       *expected_states: Union[MxDigitalIoState, int, str],
                       timeout: float = None):
        """Wait until the current digital input states matches the desired state.
           Note that, here, a desired state of '*', 'stay' or -1 is interpreted as "don't compare".

        Parameters
        ----------
        bank_id : MxIoBankId
            The IO bank Id to wait input states for.
        expected_states : Union[MxDigitalIoState, int, str]
            The desired IO state (the number of available inputs depends on the chosen bank id):
                - MxDigitalIoState.MX_DIGITAL_IO_STATE_STAY (alternatively: -1, '*' or 'stay') -> Will not be compared
                - MxDigitalIoState.MX_DIGITAL_IO_STATE_0 (alternatively: 0 or 'off')
                - MxDigitalIoState.MX_DIGITAL_IO_STATE_1 (alternatively: 1 or 'on')

        Raises
        ------
        TimeoutException
            If timeout was reached before the digital inputs state is as desired.
        """
        return self.WaitIOState(bank_id, True, *expected_states, timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def WaitRecoveryMode(self, activated: bool, timeout: float = None):
        """Pause program execution until the robot recovery mode is in the requested state.

        Parameters
        ----------
        activated : bool
            Recovery mode to wait for (activated or deactivated
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).
        """
        super().WaitRecoveryMode(activated, timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def WaitForError(self, timeout: float = None):
        """Pause program execution until the robot is in error state.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the event (in seconds).
        """
        super().WaitForError(timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def WaitErrorReset(self, timeout: float = None):
        """Pause program execution until the robot is not in an error state.

        Parameters
        ----------
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).
        """
        super().WaitErrorReset(timeout)

    @deprecation.deprecated(deprecated_in="2.1.0",
                            removed_in="3.0.0",
                            current_version=__version__,
                            details="Use the 'WaitSafetyStopReset' function instead")
    @disconnect_on_exception_decorator
    def WaitPStop2Reset(self, timeout: float = None):
        """Deprecated use WaitSafetyStopReset instead."""
        super().WaitPStop2ResetDeprecated(timeout)

    @deprecation.deprecated(deprecated_in="2.1.0",
                            removed_in="3.0.0",
                            current_version=__version__,
                            details="Use the 'WaitSafetyStopReset' function instead")
    @disconnect_on_exception_decorator
    def WaitPStop2Resettable(self, timeout: float = None):
        """Deprecated use WaitSafetyStopResettable instead."""
        super().WaitPStop2ResettableDeprecated(timeout)

    @deprecation.deprecated(deprecated_in="2.1.0",
                            removed_in="3.0.0",
                            current_version=__version__,
                            details="Use the 'WaitSafetyStopResettable' function instead")
    @disconnect_on_exception_decorator
    def WaitEStopReset(self, timeout: float = None):
        """Deprecated use WaitSafetyStopReset instead."""
        super().WaitEStopResetDeprecated(timeout)

    @deprecation.deprecated(deprecated_in="2.1.0",
                            removed_in="3.0.0",
                            current_version=__version__,
                            details="Use the 'WaitSafetyStopResettable' function instead")
    @disconnect_on_exception_decorator
    def WaitEStopResettable(self, timeout: float = None):
        """Deprecated use WaitSafetyStopResettable instead."""
        super().WaitEStopResettableDeprecated(timeout)

    def WaitEstopResettable(self, timeout: float = None):
        super().WaitEStopResettableDeprecated(timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def WaitSafetyStopReset(self, timeout: float = None):
        """Pause program execution until all safety stop conditions have been reset (EStop, PStop1, PStop2, ...)

        Parameters
        ----------
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).
        """
        super().WaitSafetyStopReset(timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def WaitSafetyStopResettable(self, timeout: float = None):
        """Pause program execution until all safety conditions can be reset using the power supply Reset function.

        Parameters
        ----------
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).
        """
        super().WaitSafetyStopResettable(timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def WaitSafetyStopStateChange(self, timeout: float = None):
        """Pause program execution until any safety stop state changes (Raised, resettable or cleared safety stop,
           operation mode change, etc) as reported by RobotSafetyStatus.

        Parameters
        ----------
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).
        """
        super().WaitSafetyStopStateChange(timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def WaitMotionResumed(self, timeout: float = None):
        """Pause program execution until the robot motion is resumed.

        Parameters
        ----------
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).
        """
        super().WaitMotionResumed(timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def WaitMotionPaused(self, timeout: float = None):
        """Pause program execution until the robot motion is paused.

        Parameters
        ----------
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).
        """
        super().WaitMotionPaused(timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def WaitMotionCleared(self, timeout: float = None):
        """Pause program execution until all pending request to clear motion have been acknowledged.

        Parameters
        ----------
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).
        """
        super().WaitMotionCleared(timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def WaitEndOfCycle(self, timeout: float = None):
        """Pause program execution until all messages in a message cycle are received

        Parameters
        ----------
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).
        """
        super().WaitEndOfCycle(timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def WaitIdle(self, timeout: float = None, wait_rt_data=False):
        """Pause program execution until robot is idle (no longer moving, motion queue empty).

        Note: After this function returns (and the robot is idle), some real-time values
        (e.g. GetRtTargetJointPos, GetRtTargetCartPos, GetRobotRtData) may still be outdated since they are updated at
        a defined monitoring interval (defined by SetMonitoringInterval()).
        To get the most recent real-time position immediately after WaitIdle(), use the synchronous_update option
        in those functions.
        Alternatively, you can use WaitEndOfCycle() after WaitIdle() to ensure the latest real-time data is received.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the event (in seconds).
        wait_rt_data : bool
            After the robot is idle (end of block), also wait until next cyclic real-time data is received.
            This ensures that, when the WaitIdle function exits, the real-time data is up-to-date
            (position, velocity (should be 0 here), etc)
        """
        super().WaitIdle(timeout, wait_rt_data)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def ResetError(self):
        """Attempt to reset robot error."""
        super().ResetError()

    @deprecation.deprecated(deprecated_in="1.2.2",
                            removed_in="3.0.0",
                            current_version=__version__,
                            details="Use the 'ResetPStop2' function instead")
    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def ResetPStop(self, timeout: float = None):
        """Attempt to reset robot PStop2.
           Deprecated for robots running firmware 10.1 and above: use ResumeMotion instead.
           *** IMPORTANT NOTE: PStop2 is not safety-rated on Meca500 robots ***
        """
        self.ResetPStop2Deprecated(timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def ResetPStop2(self, timeout: float = None):
        """Attempt to reset robot PStop2.
           Deprecated for robots running firmware 10.1 and above: use ResumeMotion instead.
           *** IMPORTANT NOTE: PStop2 is not safety-rated on Meca500 robots ***
        """
        super().ResetPStop2Deprecated(timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def Delay(self, t: float):
        """Set a delay between motion commands.

        Parameters
        ----------
        t : float
            Desired pause duration in seconds.

        """
        super().Delay(t)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SendCustomCommand(self,
                          command: str,
                          expected_responses: list[int] = None,
                          timeout: float = None) -> InterruptableEvent | Message:
        """Send custom command to robot (a command that the robot may support but that this Python API does not
           provide an explicit function for).

        Parameters
        ----------
        command : str
            Desired custom command.

        expected_responses : None or list of integers.
            If not none, wait for and return one of the expected responses.

        timeout: None or float
            Timeout (in seconds) waiting for the first of expected_responses.
            If None, this function is non-blocking and return InterruptableEvent that can be used to wait
            until first response of expected_responses is received.
            If not None, this function will wait on the interruptable event for the specified duration
            and instead return the Message that was received among expected_responses,
            or throw TimeoutException if appropriate.

        Raises
        ------
        TimeoutException
            If timeout was reached before the robot returned success or failure response.
        InterruptException
            If the robot returns an error (exception message will describe the error).

        Return
        ------
        If expected_responses is not None, this function returns either:
         - InterruptableEvent if timeout is None (use event.wait() to wait for and get the response message).
         - The received Message object if timeout is not None

        """
        return self._send_custom_command(command, expected_responses, timeout, skip_internal_check=True)

    def GetInterruptableEvent(self,
                              codes: list[Union[MxRobotStatusCode, Message]],
                              abort_on_error=False,
                              abort_on_clear_motion=False) -> InterruptableEvent:
        """Get an interruptable event that can be used to await for next code received from the robot among the list

        Parameters
        ----------
        codes : list[mdr.MxRobotStatusCode]
            List of status codes to await for (first received will awake the interruptable event)
            or
            List of message (id + data) to await for (first received message that matches id and data will awake event)
        abort_on_error : bool, optional
            Tells if this event must be awakened if the robot falls into error state, by default False
        abort_on_clear_motion : bool, optional
            Tells if this event must be awakened if the robot's motion queue is cleared
            Note that this also includes PStop2 condition and robot deactivation (which also cause the motion queue
            to be cleared)

        Returns
        -------
        mdr.InterruptableEvent
            Event that will be awakened upon reception of first received event among provided codes

        Example 1: Wait for an event id
        -------
            # Create interruptable event that will trigger on code MX_ST_RT_INPUT_STATE
            input_state_changed_event = robot.GetInterruptableEvent([mdr.MxRobotStatusCode.MX_ST_RT_INPUT_STATE])
            # (there can be code here that move the robot or whatever is expected to trigger digital input change)
            # (...)

            # Wait (block) until the event is received with a 10 seconds timeout.
            input_state_changed_event.wait(10)

        Example 2: Wait for an event id with specific data
        -------
            # Create interruptable event that will trigger when torque limit is exceeded,
            # i.e. event id MX_ST_TORQUE_LIMIT_STATUS with data 1
            torque_exceeded_event = robot.GetInterruptableEvent(
                [mdr.Message(mdr.MxRobotStatusCode.MX_ST_TORQUE_LIMIT_STATUS, '1')])

            # (there can be code here that move the robot or do other things, for example)
            # (...)

            # Check if the torque limit exceeded status event was received
            if torque_exceeded_event.is_set():
                # Torque limit was exceeded between call to GetInterruptableEvent above and now
                pass

            # Wait (block) until the torque limit exceeded event is received
            torque_exceeded_event.wait(10)
        """
        return super().GetInterruptableEvent(codes, abort_on_error, abort_on_clear_motion)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def StartProgram(self, n: int | str, timeout: float = None):
        """Start an offline program.

        Offline programs need to be recorded using the robot's Web Portal (or text API).
        This API can only start an already recorded offline program.
        Callback on_offline_program_state will indicate when program is started.

        Parameters
        ----------
        n : int
            Id of offline program to start.
        timeout: float
            Timeout (in seconds) waiting for the robot to confirm that the program has started executing.

        """
        super().StartProgram(n, timeout)

    @disconnect_on_exception_decorator
    def StartOfflineProgram(self, n: int | str, timeout: float = None):
        """Deprecated use StartPrograms instead.
        """
        super().StartProgram(n, timeout)

    @disconnect_on_exception_decorator
    def ListFiles(self, timeout: float = None) -> dict:
        """List all files that are stored on the robot.
        Related commands: LoadFile, SaveFile, DeleteFile

        This function is synchronous only for now.
        If you want to asynchronously list files, you can use SendCustomCommand, for example:
        done_evt = self.SendCustomCommand('ListFiles')
        Callback on_command_message(response) can be attached to be informed of all responses from the robot.
        Responses to check fore are:
            - response.id == MxRobotStatusCode.MX_ST_LIST_FILES
            In this case response.json_data[MX_JSON_KEY_DATA]["files"] will contain the listed files list
            (same format as returned by this function)
            - response.id == MxRobotStatusCode.MX_ST_LIST_FILES_ERR

        Parameters
        ----------
        timeout: float
            Timeout (in seconds) waiting for files list.

        Raises
        ------
        TimeoutException
            If timeout was reached before the robot returned success or failure response.
        InterruptException
            If the robot returns an error (exception message will describe the error).

        Returns
        -------
        dict
            Object that contains the status of all listed files, in this format
            {
                "myProgramName": {
                    "status": {
                        "valid": false,
                        "invalidLines": [10, 332, 522]
                    }
                },
                "myProgram2": {
                    "status": {
                        "valid": true,
                        "invalidLines": []
                    }
                },
                ...
            }
        """
        return super().ListFiles(timeout)

    @deprecation.deprecated(deprecated_in="2.1.0",
                            removed_in="3.0.0",
                            current_version=__version__,
                            details="Use the 'ListFiles' function instead")
    @disconnect_on_exception_decorator
    def ListPrograms(self, timeout: float = None) -> dict:
        """Deprecated use ListFiles instead.
        """
        return super().ListFiles(timeout)

    @disconnect_on_exception_decorator
    def LoadFile(self, name: str, timeout: float = None) -> dict:
        """Load a file from the robot.

        Related commands: ListFiles, SaveFile, DeleteFile

        This function is synchronous only for now.
        If you want to asynchronously load a file, you can use SendCustomCommand, for example:
        done_evt = self.SendCustomCommand('LoadFile{"data":{"name":"programName"}')
        Callback on_command_message(response) can be attached to be informed of all responses from the robot.
        Responses to check fore are:
            - response.id == MxRobotStatusCode.MX_ST_LOAD_FILES
            - response.id == MxRobotStatusCode.MX_ST_LOAD_FILES_ERR

        Parameters
        ----------
        name: str
            Name of the file to load.
        timeout: float
            Timeout (in seconds) waiting for file to be loaded.

        Raises
        ------
        TimeoutException
            If timeout was reached before the robot returned success or failure response.
        InterruptException
            If the robot returns an error (exception message will describe the error).

        Returns
        -------
        dict
            Object that contains information and contents of the loaded program
            {
                "name": "fileName",
                "status": {
                    "valid": false,
                    "invalidLines": [10, 332, 522]
                },
                "content":{"// File content"}
            }
        """
        return super().LoadFile(name, timeout)

    @deprecation.deprecated(deprecated_in="2.1.0",
                            removed_in="3.0.0",
                            current_version=__version__,
                            details="Use the 'LoadFile' function instead")
    @disconnect_on_exception_decorator
    def LoadProgram(self, name: str, timeout: float = None) -> dict:
        """Deprecated use LoadFile instead.
        """
        return super().LoadFile(name, timeout)

    @disconnect_on_exception_decorator
    def SaveFile(self, name: str, content: str, timeout: float = None, allow_invalid=False, overwrite=False):
        """Save a file the robot.

        Related commands: ListFiles, LoadFile, DeleteFile

        This function is synchronous only for now.
        If you want to asynchronously save file, you can use SendCustomCommand, for example:
        done_evt = self.SendCustomCommand('SaveFile{"data":{"name":"programName", "content":"Program text"}}')
        Callback on_command_message(response) can be attached to be informed of all responses from the robot.
        Responses to check fore are:
            - response.id == MxRobotStatusCode.MX_ST_SAVE_FILE
            - response.id == MxRobotStatusCode.MX_ST_SAVE_FILE_ERR

        Parameters
        ----------
        name: str
            Name of the file to save.
        content: str
            The file content to save.
        timeout: float
            Timeout (in seconds) waiting for file to be saved.
        allow_invalid: bool
            Allow saving the file even if it is invalid
        overwrite: bool
            Overwrite if a file with the same name already exists on the robot.
            (equivalent of atomic Delete then SaveFile)
            If False, SaveFile will be refused if the name is already used.

        Raises
        ------
        TimeoutException
            If timeout was reached before the robot returned success or failure response.
        InterruptException
            If the robot returns an error (exception message will describe the error).
        """
        return super().SaveFile(name, content, timeout, allow_invalid, overwrite)

    @deprecation.deprecated(deprecated_in="2.1.0",
                            removed_in="3.0.0",
                            current_version=__version__,
                            details="Use the 'SaveFile' function instead")
    @disconnect_on_exception_decorator
    def SaveProgram(self, name: str, program: str, timeout: float = None, allow_invalid=False, overwrite=False):
        """Deprecated use SaveFile instead.
        """
        return super().SaveFile(name, program, timeout, allow_invalid, overwrite)

    @disconnect_on_exception_decorator
    def DeleteFile(self, name: str, timeout: float = None):
        """Delete a file from the robot.

        Related commands: ListFiles, LoadFile, SaveFile

        This function is synchronous only for now.
        If you want to asynchronously delete a file, you can use SendCustomCommand, for example:
        done_evt = self.SendCustomCommand('DeleteFile{"data":{"name":"programName"}')
        Callback on_command_message(response) can be attached to be informed of all responses from the robot.
        Responses to check fore are:
            - response.id == MxRobotStatusCode.MX_ST_DELETE_FILE
            - response.id == MxRobotStatusCode.MX_ST_DELETE_FILE_ERR

        Parameters
        ----------
        name: str
            Name of the file to delete.
        timeout: float
            Timeout (in seconds) waiting for file to be deleted.

        Raises
        ------
        TimeoutException
            If timeout was reached before the robot returned success or failure response.
        InterruptException
            If the robot returns an error (exception message will describe the error).
        """
        return super().DeleteFile(name, timeout)

    @deprecation.deprecated(deprecated_in="2.1.0",
                            removed_in="3.0.0",
                            current_version=__version__,
                            details="Use the 'DeleteFile' function instead")
    @disconnect_on_exception_decorator
    def DeleteProgram(self, name: str, timeout: float = None):
        """Deprecated use DeleteFile instead.
        """
        return super().DeleteFile(name, timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def GetRtExtToolStatus(self,
                           include_timestamp: bool = False,
                           synchronous_update: bool = False,
                           timeout: float = None) -> ExtToolStatus:
        """Return a copy of the current external tool status

        Parameters
        ----------
        include_timestamp : bool
            If true, return a TimestampedData object, otherwise just return states.
        synchronous_update: bool
            True -> Synchronously get updated external tool status. False -> Get latest known status.
        timeout: float
            Timeout (in seconds) waiting for synchronous response from the robot.

        Returns
        -------
        TimestampedData or ExtToolStatus
            Object containing the current external tool status

        """
        return super().GetRtExtToolStatus(include_timestamp, synchronous_update, timeout)

    @disconnect_on_exception_decorator
    def GetNetworkCfg(self, synchronous_update: bool = True, timeout: float = None) -> NetworkConfig:
        """Return robot's current network configuration

        Parameters
        ----------
        synchronous_update: bool
            True -> Synchronously get updated network config. False -> Get latest known network configuration.
        timeout: float
            Timeout (in seconds) waiting for synchronous response from the robot.

        Returns
        -------
        NetworkConfig
            Object containing the current network config

        """
        return super().GetNetworkCfg(synchronous_update, timeout)

    @disconnect_on_exception_decorator
    def GetNetworkConfig(self, synchronous_update: bool = False, timeout: float = None) -> NetworkConfig:
        """Legacy command. Please use GetNetworkCfg instead."""
        return super().GetNetworkCfg(synchronous_update, timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def GetRtIoStatus(self,
                      bank_id: MxIoBankId = MxIoBankId.MX_IO_BANK_ID_IO_MODULE,
                      include_timestamp: bool = False,
                      synchronous_update: bool = False,
                      timeout: float = None) -> IoStatus:
        """Return a copy of the current IO module status

        Parameters
        ----------
        bank_id : MxIoBankId
            The IO bank Id to get output states for.
        synchronous_update: bool
            True -> Synchronously get updated IO module status. False -> Get latest known status.
        timeout: float
            Timeout (in seconds) waiting for synchronous response from the robot.

        Returns
        -------
        TimestampedData or IoStatus
            Object containing the current IO module status

        """
        return super().GetRtIoStatus(bank_id, include_timestamp, synchronous_update, timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def GetRtGripperState(self,
                          include_timestamp: bool = False,
                          synchronous_update: bool = False,
                          timeout: float = None) -> GripperState:
        """Return a copy of the current gripper state

        Parameters
        ----------
        include_timestamp : bool
            If true, return a TimestampedData object, otherwise just return states.
        synchronous_update: bool
            True -> Synchronously get updated gripper state. False -> Get latest known status.
        timeout: float
            Timeout (in seconds) waiting for synchronous response from the robot.

        Returns
        -------
        TimestampedData or GripperState
            Object containing the current gripper state

        """
        return super().GetRtGripperState(include_timestamp, synchronous_update, timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def GetRtValveState(self,
                        include_timestamp: bool = False,
                        synchronous_update: bool = False,
                        timeout: float = None) -> ValveState:
        """Return a copy of the current valve state

        Parameters
        ----------
        include_timestamp : bool
            If true, return a TimestampedData object, otherwise just return states.
        synchronous_update: bool
            True -> Synchronously get updated valve states. False -> Get latest known status.
        timeout: float
            Timeout (in seconds) waiting for synchronous response from the robot.

        Returns
        -------
        TimestampedData or ValveState
            Object containing the current valve state

        """
        return super().GetRtValveState(include_timestamp, synchronous_update, timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def GetRtOutputState(self,
                         bank_id: MxIoBankId = MxIoBankId.MX_IO_BANK_ID_IO_MODULE,
                         synchronous_update: bool = False,
                         timeout: float = None) -> TimestampedData:
        """Return a copy of the current digital outputs state

        Parameters
        ----------
        bank_id : MxIoBankId
            The IO bank Id to get output states for.
        synchronous_update: bool
            True -> Synchronously get updated states. False -> Get latest known status.
        timeout: float
            Timeout (in seconds) waiting for synchronous response from the robot.

        Returns
        -------
        TimestampedData
            Object containing the current digital output states for requested bank id.

        """
        return super().GetRtOutputState(bank_id, synchronous_update, timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def GetRtInputState(self,
                        bank_id: MxIoBankId = MxIoBankId.MX_IO_BANK_ID_IO_MODULE,
                        synchronous_update: bool = False,
                        timeout: float = None) -> TimestampedData:
        """Return a copy of the current digital inputs state

        Parameters
        ----------
        bank_id : MxIoBankId
            The IO bank Id to get input states for.
        synchronous_update: bool
            True -> Synchronously get updated states. False -> Get latest known status.
        timeout: float
            Timeout (in seconds) waiting for synchronous response from the robot.

        Returns
        -------
        TimestampedData
            Object containing the current digital input states for requested bank id.

        """
        return super().GetRtInputState(bank_id, synchronous_update, timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def GetRtVacuumState(self,
                         include_timestamp: bool = False,
                         synchronous_update: bool = False,
                         timeout: float = None) -> VacuumState:
        """Return a copy of the current vacuum gripper state

        Parameters
        ----------
        include_timestamp : bool
            If true, return a TimestampedData object, otherwise return VacuumState.
        synchronous_update: bool
            True -> Synchronously get updated states. False -> Get latest known status.
        timeout: float
            Timeout (in seconds) waiting for synchronous response from the robot.

        Returns
        -------
        TimestampedData
            Object containing the current Vacuum grip state: [vacuum on/off, purge on/off,  holding part]

        """
        return super().GetRtVacuumState(include_timestamp, synchronous_update, timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def GetRtTargetJointPos(self,
                            include_timestamp: bool = False,
                            synchronous_update: bool = False,
                            timeout: float = None) -> TimestampedData:
        """Returns the real-time target joint positions of the robot.

        Parameters
        ----------
        include_timestamp : bool
            If true, return a TimestampedData object, otherwise just return joints angles.
        synchronous_update : bool
            True -> Synchronously get current target joints position.
            False -> Return latest known data (from previous received cyclic data) without requesting the robot
            *** WARNING: This position may be outdated from as much as one cycle, which duration is defined by
                         SetMonitoringInterval().
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time in second to wait for forced update.

        Return
        ------
        TimestampedData or list of floats
            Returns joint positions in degrees.

        """
        return super().GetRtTargetJointPos(include_timestamp, synchronous_update, timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def GetJoints(self, synchronous_update: bool = False, timeout: float = None):
        """Legacy command. Please use GetRtTargetJointPos instead."""
        return super().GetRtTargetJointPos(include_timestamp=False,
                                           synchronous_update=synchronous_update,
                                           timeout=timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def GetRtTargetCartPos(self,
                           include_timestamp: bool = False,
                           synchronous_update: bool = False,
                           timeout: float = None) -> TimestampedData:
        """Returns the current end-effector pose of the robot. WARNING: NOT UNIQUE.

        Parameters
        ----------
        include_timestamp : bool
            If true, return a TimestampedData object, otherwise just return joints angles.
        synchronous_update : bool
            True -> Synchronously get current cartesian position.
            False -> Return latest known data (from previous received cyclic data) without requesting the robot
            *** WARNING: This position may be outdated from as much as one cycle, which duration is defined by
                         SetMonitoringInterval().
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time in second to wait for forced update.

        Return
        ------
        TimestampedData or list of floats
            Returns end-effector pose [x, y, z, alpha, beta, gamma].

        """
        return super().GetRtTargetCartPos(include_timestamp, synchronous_update=synchronous_update, timeout=timeout)

    def GetPose(self, synchronous_update: bool = False, timeout: float = None) -> TimestampedData:
        """Legacy command. Please use GetRtTargetCartPos instead."""
        return super().GetRtTargetCartPos(include_timestamp=False,
                                          synchronous_update=synchronous_update,
                                          timeout=timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetMonitoringInterval(self, t: float):
        """Sets the interval at which the monitoring port sends real-time data.

        Parameters
        ----------
        t : float
            Monitoring interval duration in seconds.

        """
        super().SetMonitoringInterval(t)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetRealTimeMonitoring(self, *events: tuple):
        """Configure which real-time monitoring events to enable.

        Parameters
        ----------
        events : list of event IDs
            List of event IDs to enable.
            Example:
          SetRealTimeMonitoring(MxRobotStatusCode.MX_ST_RT_TARGET_JOINT_POS, MxRobotStatusCode.MX_ST_RT_TARGET_CART_POS)
            enables the target joint positions and target end effector pose messages.
            Can also use events='all' to enable all.

        """
        super().SetRealTimeMonitoring(*events)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetRtc(self, t: int):
        """Sets the robot's real-time clock (date/time).

        Parameters
        ----------
        t : int
            Unix epoch time (seconds since 00:00:00 UTC Jan 1, 1970).

        """
        super().SetRtc(t)

    @deprecation.deprecated(deprecated_in="1.2.0",
                            removed_in="3.0.0",
                            current_version=__version__,
                            details="Use the 'SetRtc' function instead")
    @disconnect_on_exception_decorator
    def SetRTC(self, t: int):
        """Deprecated use SetRtc instead.
        """
        super().SetRtc(t)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def ActivateSim(self, mode: Optional[MxRobotSimulationMode] = None):
        """Enables simulation mode. Motors and external tool don't move, but commands will be processed

        Args:
            mode (MxRobotSimulationMode, optional): Use the "real-time" or "fast" simulation mode.
            If not specified (None), the robot's default simulation mode will be used (see SetSimModeCfg).
        """
        super().ActivateSim(mode)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def DeactivateSim(self):
        """Disables simulation mode. Motors and external tool will now move normally."""
        super().DeactivateSim()

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetExtToolSim(self, sim_ext_tool_type: int = MxExtToolType.MX_EXT_TOOL_MEGP25_SHORT):
        """Simulate an external tool, allowing GripperOpen/Close, MoveGripper and SetValveState commands
            on a robot without an external tool present.

        Parameters
        ----------
        sim_ext_tool_type : int or constants
            0: MxExtToolType.MX_EXT_TOOL_NONE
            1: MxExtToolType.MX_EXT_TOOL_CURRENT
           10: MxExtToolType.MX_EXT_TOOL_MEGP25_SHORT
           11: MxExtToolType.MX_EXT_TOOL_MEGP25_LONG
           20: MxExtToolType.MX_EXT_TOOL_VBOX_2VALVES
        """
        super().SetExtToolSim(sim_ext_tool_type)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetIoSim(self, bank_id: MxIoBankId = MxIoBankId.MX_IO_BANK_ID_IO_MODULE, enable: bool = True):
        """Enable or disable IO simulation. This allows emulating the presence of an IO module and use the
           corresponding APIs (ex: SetOutputState) without the module to be physically present.
           This can also be used even if the physical module is present to test the APIs without actually applying the
           changes on the physical module (to test the APIs without the robot moving)

        Parameters
        ----------
        bank_id : MxIoBankId
            The IO bank Id to enable or disable simulation mode for.
        enable : bool
            Enable or disable simulation mode.
        """
        super().SetIoSim(bank_id, enable)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetRecoveryMode(self, activated: bool = True):
        """Enable/disable recovery mode, allowing robot to move (slowly) without homing and without joint limits."""
        super().SetRecoveryMode(activated)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetTimeScaling(self, p: float):
        """This command sets the time scaling (in percentage) of the trajectory generator. By calling this command
            with p < 100, all robot motions remain exactly the same (i.e., the path remains the same), but executed
            at p% of the original speed, including time delays (e.g., the pause set by the command Delay).
            In other words, this command is more than a simple velocity override.

        Parameters
        ----------
        p : float
            Percentage time scaling, from 0.001 to 100.

        """
        super().SetTimeScaling(p)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetJointLimitsCfg(self, e: bool = True):
        """Enable/Disable user-defined limits set by the 'SetJointLimits' command. It can only be executed while
        the robot is deactivated. If the user-defined limits are disabled, the default joint limits become active.
        However, user-defined limits remain in memory, and can be re-enabled, even after a power down.

        Parameters
        ----------
            e : bool
                enable (True) or disable (False) the user-defined joint limits.
        """
        super().SetJointLimitsCfg(e)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetJointLimits(self, n: int, lower_limit: float, upper_limit: float):
        """This command redefines the lower and upper limits of a robot joint. It can only be executed while the robot
        is deactivated. For these user-defined joint limits to be taken into account, you must execute the command
        SetJointLimitsCfg(1). Obviously, the new joint limits must be within the default joint limits and all the robot
        joints position must be within the requested limits. Note that these user-defined joint limits remain active
        even after you power down the robot.

        Parameters
        ----------
            n : int
                joint number, an integer ranging from 1 to 6
            lower_limit : float
                lower limit of the actuator, in degrees/mm
            upper_limit : float
                upper limit of the actuator, in degrees/mm
        """
        super().SetJointLimits(n, lower_limit, upper_limit)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetWorkZoneCfg(self,
                       severity: MxEventSeverity = MxEventSeverity.MX_EVENT_SEVERITY_ERROR,
                       mode: MxWorkZoneMode = MxWorkZoneMode.MX_WORK_ZONE_MODE_FCP_IN_WORK_ZONE):
        """Set the severity at which to report work zone violations and the detection mode for work zone violations used
        by the robot. This command can only be used when the robot is deactivated. User-defined limits remain in memory,
        even after a power down.

        Parameters
        ----------
            severity : MxEventSeverity
                Severity-level to report work zone events.
                The available severities are found in mx_robot_def.MxEventSeverity.
                Note that MX_EVENT_SEVERITY_PAUSE_MOTION = 2 is not supported as colliding paths should not be followed.
            mode : MxWorkZoneMode
                Work zone detection mode to use for checks.
                The available modes are found in mx_robot_def.MxWorkZoneMode.
        """
        super().SetWorkZoneCfg(severity, mode)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetWorkZoneLimits(self, x_min: float, y_min: float, z_min: float, x_max: float, y_max: float, z_max: float):
        """Set the work zone limits the robot must not exceed. This command can only be used when the robot is
        deactivated. User-defined limits remain in memory, even after a power down.

        Parameters
        ----------
            x_min : float
                minimum x value of the work zone
            y_min : float
                minimum y value of the work zone
            z_min : float
                minimum z value of the work zone
            x_max : float
                maximum x value of the work zone
            y_max : float
                maximum y value of the work zone
            z_max : float
                maximum z value of the work zone
        """
        super().SetWorkZoneLimits(x_min, y_min, z_min, x_max, y_max, z_max)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetCollisionCfg(self, severity: MxEventSeverity = MxEventSeverity.MX_EVENT_SEVERITY_ERROR):
        """Set the severity at which to report self collision events. This command can only be used when the robot is
        deactivated. User-defined configurations remain in memory, even after a power down.

        Parameters
        ----------
            severity : MxEventSeverity
                Severity-level to report collision events.
                The available severities are found in mx_robot_def.MxEventSeverity.
                Note that MX_EVENT_SEVERITY_PAUSE_MOTION = 2 is not supported as colliding paths should not be followed.
        """
        super().SetCollisionCfg(severity)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetToolSphere(self, x: float, y: float, z: float, r: float):
        """Set the tool sphere model of the robot. This command can only be used when the robot is
        deactivated. User-defined limits remain in memory, even after a power down.

        Parameters
        ----------
            x_min : float
                offset along FRF x-axis to position tool center
            y_min : float
                offset along FRF y-axis to position tool center
            z : float
                offset along FRF z-axis to position tool center
            r : float
                radius of the tool sphere model. r > 0
        """
        super().SetToolSphere(x, y, z, r)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetTorqueLimitsCfg(
            self,
            severity: MxEventSeverity = MxEventSeverity.MX_EVENT_SEVERITY_ERROR,
            skip_acceleration: MxTorqueLimitsMode = MxTorqueLimitsMode.MX_TORQUE_LIMITS_DETECT_SKIP_ACCEL):
        """Change the torque limits configuration (enable/disable, choose severity, etc.).
        Note that the per-joint torque limit is configured by calling SetTorqueLimits.

        Parameters
        ----------
        severity : MxEventSeverity
            Severity-level of exceeding torque limits.
            Available severity levels (see MxEventSeverity or TORQUE_LIMIT_SEVERITIES):
                - MX_EVENT_SEVERITY_SILENT or 0 or 'disabled':     Torque limits disabled
                                        (this by default when robot is activated)
                - MX_EVENT_SEVERITY_WARNING or 1 or 'warning':      Send a warning event
                                        (MxRobotStatusCode.MX_ST_TORQUE_LIMIT_STATUS) when torque exceeds limit
                - MX_EVENT_SEVERITY_PAUSE_MOTION or 2 or 'pause-motion': Pause motion when torque exceeds the limit
                - MX_EVENT_SEVERITY_CLEAR_MOTION or 3 or 'clear-motion': Pause motion when torque exceeds the limit
                - MX_EVENT_SEVERITY_ERROR or 4 or 'error':        Set robot in error state when torque exceeds the limit
        skip_acceleration : MxTorqueLimitsMode
            Whether torque limits are ignored during acceleration periods (allowing fast accelerations without
            triggering torque limits exceeded condition)
            Available modes (see MxTorqueLimitsMode):
                - MX_TORQUE_LIMITS_DETECT_ALL or 0 or False : Always check if torque is within limits
                - MX_TORQUE_LIMITS_DETECT_SKIP_ACCEL or 1 or True : Ignore torque over limit during acceleration
        """
        super().SetTorqueLimitsCfg(severity, skip_acceleration)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetTorqueLimits(self, *args: float):
        """Set the torque limit (in percent) for each joint.
        Note that torque limits will be applied only if severity mode is set to other than 'disabled' by
        calling SetTorqueLimitsCfg.

        Parameters
        ----------
        joint_1...joint_n : float
            Desired torque limit in percent.

        """
        super().SetTorqueLimits(*args)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetPStop2Cfg(self, severity: MxEventSeverity = MxEventSeverity.MX_EVENT_SEVERITY_CLEAR_MOTION):
        """Set the severity at which to treat a PStop2 events.
        This command can only be used when the robot is deactivated. User-defined severity remains in memory, even after
        a power down.

        Parameters
        ----------
            severity : MxEventSeverity
                Severity-level to treat PStop2 events.
                The available severities are:
                    - mx_robot_def.MxEventSeverity.MX_EVENT_SEVERITY_PAUSE_MOTION
                    - mx_robot_def.MxEventSeverity.MX_EVENT_SEVERITY_CLEAR_MOTION
        """
        super().SetPStop2Cfg(severity)

    @disconnect_on_exception_decorator
    def SetSimModeCfg(self, default_sim_mode=MxRobotSimulationMode.MX_SIM_MODE_REAL_TIME):
        """Set the simulation mode configuration.
        This configuration remains in memory, even after a power down.

        Parameters
        ----------
            default_sim_mode : MxRobotSimulationMode
                Default simulation mode to use when using ActivateSim command without specifying the mode.
                The available modes are:
                    - mx_robot_def.MxRobotSimulationMode.MX_SIM_MODE_REAL_TIME
                    - mx_robot_def.MxRobotSimulationMode.MX_SIM_MODE_FAST
        """
        super().SetSimModeCfg(default_sim_mode)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def SetPayload(self, mass: float, x: float, y: float, z: float):
        """Set payload mass and center of mass (in FRF) of currently carried by the robot's gripper (or external tool).

        Parameters
        ----------
        mass : float
            Carried mass in KG.
        x : float
            X coordinate of center of mass relatively to FRF
        y : float
            Y coordinate of center of mass relatively to FRF
        z : float
            Z coordinate of center of mass relatively to FRF

        """
        super().SetPayload(mass, x, y, z)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def ActivateBrakes(self, activated: bool = True):
        """Enable/disable the brakes. These commands are only available when the robot is deactivated.

        By default, brakes are enabled until robot is activated (brakes are automatically disabled upon activation).
        Corresponds to text API calls "BrakesOn" / "BrakesOff".

        Parameters
        ----------
        activated : bool
            Engage brakes if true, otherwise disengage brakes.

        """
        super().ActivateBrakes(activated)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def GetRobotInfo(self) -> RobotInfo:
        """Return a copy of the known robot information.

        Return
        ------
        RobotInfo
            Object containing robot information.

        """
        return super().GetRobotInfo()

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def GetRobotRtData(self, synchronous_update: bool = False, timeout: float = None) -> RobotRtData:
        """Return a copy of the current robot real-time data, with all values associated with the same timestamp.

        *** WARNING:    Not all real-time data is sent by the robot by default.
                        SetRealtimeMonitoring() can be used to enable more real-time data than the default ones.
                        A non-enabled real-time data will report "enabled" as False
                        (for example: GetRobotRtData().rt_joint_torq.enabled will be False if not sent by the robot)

        Parameters
        ----------
        synchronous_update: bool
            True -> Synchronously get all robot real-time data by waiting end of current cycle.
            False -> Return latest known data (from previous received cyclic data) without requesting the robot.
            *** WARNING: This position may be outdated from as much as one cycle, which duration is defined by
                         SetMonitoringInterval().
        timeout: float
            Timeout (in seconds) waiting for updated cyclic data from the robot. Only used for synchronous requests.

        Return
        ------
        RobotRtData
            Object containing the current robot real-time data

        """
        return super().GetRobotRtData(synchronous_update, timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def GetStatusRobot(self, synchronous_update: bool = False, timeout: float = None) -> RobotStatus:
        """Return a copy of the current robot status

        Parameters
        ----------
        synchronous_update: bool
            True -> Synchronously get updated robot status. False -> Get latest known status.
        timeout: float, defaults to DEFAULT_WAIT_TIMEOUT
            Timeout (in seconds) waiting for synchronous response from the robot.

        Returns
        -------
        RobotStatus
            Object containing the current robot status

        """
        return super().GetStatusRobot(synchronous_update, timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def GetSafetyStatus(self, synchronous_update: bool = False, timeout: float = None) -> RobotSafetyStatus:
        """Return a copy of the current robot safety status

        Parameters
        ----------
        synchronous_update: bool
            True -> Synchronously get updated robot safety status. False -> Get latest known status.
        timeout: float, defaults to DEFAULT_WAIT_TIMEOUT
            Timeout (in seconds) waiting for synchronous response from the robot.

        Returns
        -------
        RobotSafetyStatus
            Object containing the current robot safety status

        """
        return super().GetSafetyStatus(synchronous_update, timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def GetSidecarStatus(self,
                         idx: Optional[int] = None,
                         synchronous_update: bool = False,
                         timeout: float = None) -> Optional[RobotSidecarStatus]:
        """Return a copy of the current robot sidecar status

        Parameters
        ----------
        idx: Optional[int]
            Optional index of the sidecar instance to get status for.
            If None, the function will return the status for the first sidecar instance connected, priority to the
            sidecar instance embedded in the robot
        synchronous_update: bool
            True -> Synchronously get updated robot sidecar status. False -> Get latest known status.
        timeout: float, defaults to DEFAULT_WAIT_TIMEOUT
            Timeout (in seconds) waiting for synchronous response from the robot.

        Returns
        -------
        Optional[RobotSidecarStatus]
            Status of selected sidecar scripting engine

        """
        return super().GetSidecarStatus(idx, synchronous_update, timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def GetPowerSupplyInputs(self, synchronous_update: bool = False, timeout: float = None) -> RobotPowerSupplyInputs:
        """Return a copy of the current robot power supply input states

        Parameters
        ----------
        synchronous_update: bool
            True -> Synchronously get updated robot power supply input states. False -> Get latest known status.
        timeout: float, defaults to DEFAULT_WAIT_TIMEOUT
            Timeout (in seconds) waiting for synchronous response from the robot.

        Returns
        -------
        RobotPowerSupplyInputs
            Object containing the current robot power supply inputs status

        """
        return super().GetPowerSupplyInputs(synchronous_update, timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def GetCollisionStatus(self, timeout: float = None) -> CollisionStatus:
        """Return a copy of the current robot collision status

        Parameters
        ----------
        timeout: float, defaults to DEFAULT_WAIT_TIMEOUT
            Timeout (in seconds) waiting for synchronous response from the robot.

        Returns
        -------
        CollisionStatus
            Object containing the current robot collision status

        """
        return super().GetCollisionStatus(timeout)

    # mx:export_to=robot_sidecar_globals.py
    @disconnect_on_exception_decorator
    def GetGripperRange(self, timeout: float = None) -> Tuple[float, float]:
        """Return the currently configured gripper range.
            Note that the reported values are valid only when the robot is activated and homed.

        Parameters
        ----------
        timeout: float, defaults to DEFAULT_WAIT_TIMEOUT
            Timeout (in seconds) waiting for synchronous response from the robot.

        Returns
        -------
        Tuple [close_pos, open_pos]
            Tuple indicating the close and open position of the gripper, in mm from the completely closed position
            detected during homing.

        """
        return super().GetGripperRange(timeout)

    # mx:export_to=robot_sidecar_globals.py
    def LogTrace(self, trace: str, level: Optional[int] = None):
        """Send a text trace that is printed in the robot's log internal file (which can be retrieved from robot's Web
           portal under menu "Options -> Get Log").
           (useful for clearly identifying steps of a program within robot's log when reporting problems to
            Mecademic support team!)

        Parameters
        ----------
        trace : string
            Text string to print in robot's internal log file
        level : logger level
            Local logger level to print with logging.DEBUG, logging.INFO, logging.ERROR, etc.
            Will not print locally (only on robot) if level is None (the default)
        """
        super().LogTrace(trace, level)

    def StartLogging(self,
                     monitoringInterval: float,
                     file_name: str = None,
                     file_path: str = None,
                     fields: list = None,
                     record_time: bool = True):
        """Start logging robot state and real-time data to a zip file that contains:
           - a json file (robot information summary, commands sent to the robot)
           - a csv file (real-time data received from the robot)

        This function will use SetRealtimeMonitoring() to enable specified real-time data fields.
        This function will use SetMonitoringInterval() to configure the requested monitoring interval.

        Using WaitIdle before StartLogging may be useful if you want to ensure that the captured data includes
        subsequent move commands only.

        Parameters
        ----------
        monitoring_interval: float
            Indicates rate at which state from robot will be received on monitor port. Unit: seconds

        file_name: string or None
            Log file name
            If None, file name will be built with date/time and robot information (robot type, serial, version).

        file_path : string or None
            Path to save the zip file that contains logged data.
            If not provided, file will be saved in working directory.

        fields : list of strings or None
            List of real-time data fields to log.
            Available field names correspond to fields from the RobotRtData attributes.
            None means log all possible real-time data.

        record_time : bool
            If true, current date and time will be recorded in file.
        """
        super().StartLogging(monitoringInterval, file_name, file_path, fields, record_time)

    def EndLogging(self, keep_captured_trajectory: bool = False) -> str:
        """Stop logging robot real-time data to file.
        Parameters
        ----------
        keep_captured_trajectory: bool
            Tells to keep a copy of captured trajectory that can be accessed later via
            GetCapturedTrajectory()

        Return
        ------
        string
            Name of the zip file that contains robot information and captured trajectory
        """
        return super().EndLogging(keep_captured_trajectory)

    def GetCapturedTrajectory(self) -> RobotTrajectories:
        """Returns the most recent robot trajectory captured using StartLogging or FileLogger functions.

        Returns
        -------
        RobotTrajectories
            Object that contains robot information and captured trajectory information
        """
        return super().GetCapturedTrajectory()

    #pylint: disable=invalid-name
    @contextlib.contextmanager
    def FileLogger(self,
                   monitoringInterval: float,
                   file_name: str = None,
                   file_path: str = None,
                   fields: list = None,
                   record_time: bool = True,
                   keep_captured_trajectory: bool = False):
        """Contextmanager interface for file logger.
           See StartLogging for more information.

        Parameters
        ----------
        monitoring_interval: float
            See StartLogging.
        file_name: string or None
            See StartLogging.
        file_path : string or None
            See StartLogging.
        fields : list of strings or None
            See StartLogging.
        record_time : bool
            See StartLogging.
        keep_captured_trajectory: bool
            Tells to keep a copy of captured trajectory that can be accessed later via
            GetCapturedTrajectory()
        """
        self.StartLogging(
            monitoringInterval,
            file_name=file_name,
            file_path=file_path,
            fields=fields,
            record_time=record_time,
        )
        try:
            yield
        finally:
            self.EndLogging(keep_captured_trajectory)

    UPDATE_TIMEOUT = 15 * 60  # 15 minutes timeout

    def UpdateRobot(self, firmware: Union[str, pathlib.Path], timeout=UPDATE_TIMEOUT):
        """
        Install a new firmware and verifies robot version afterward.

        Parameters
        ----------
        firmware: pathlib object or string
            Path of robot firmware file

        timeout: int
            time in second allowed to update the robot (default: 15 minutes)

        """
        return super().UpdateRobot(firmware, timeout)

    # mx:export_to=robot_sidecar_globals.py
    def CreateVariable(self,
                       name: str,
                       value: any,
                       cyclic_id: Optional[int] = None,
                       override: bool = False,
                       timeout: Optional[float] = None):
        """Create a variable that is permanently saved in the robot.

        Note: The variable will only be added to local namespace (robot.vars) after receiving the robot response for
            this request. Beware of that if using this function asynchronously.
            Use robot.Sync() if needed, or use a timeout to make this function blocking.
        Note: You can check if the variable already exists before calling this function as follows:
            `robot.vars.get("myvar") is not None`

        See description of the "override" argument below for details about how this function manages the case where a
        variable with the same name already exists.

        There are different ways to access a created variable:
        robot.vars.myvar => Return the variable value only
        robot.vars.get("myvar") => Return a RegisteredVariable that contains value, cyclic_id, etc.
        robot.GetVariable("myvar") => Return a RegisteredVariable that contains value, cyclic_id, etc.

        Args:
            name (str): Name of the variable to set.
            value (any): Value to assign to this variable. The type will be deduced from this value.
            cyclic_id (int): Optional Id to use to reference variable in cyclic protocols (0 or None to ignore)
            override (bool): Specifies the behavior when a variable with the same name already exists:
                             True:  Update the value and cyclic ID with the new ones,
                             False: Return error if the existing variable has a different type or cyclic ID,
                                    otherwise do nothing and leave the variable unchanged.
            timeout (float): Optional time to wait for the robot to confirm variable creation.
                            If None:
                                In Synchronous API mode a default timeout is used.
                                In asynchronous mode a None timeout makes this function non-blocking (will not know
                                if variable creation may have failed).
        Raises
        ------
        ArgErrorException
            If the robot refused the variable creation for some reason (name or cyclic ID conflict for example).
        """
        super().CreateVariable(name, value, cyclic_id, override, timeout)

    def CreateRegisteredVariable(self,
                                 var: rsc.RegisteredVariable,
                                 override: bool = False,
                                 timeout: Optional[float] = None) -> bool:
        """ Same as CreateVariable, but taking RegisteredVariable object instead of separate value, cyclic_id """
        return super().CreateVariable(name=var.name,
                                      value=var.get_value(),
                                      cyclic_id=var.cyclic_id,
                                      override=override,
                                      timeout=timeout)

    # mx:export_to=robot_sidecar_globals.py
    def DeleteVariable(self, name: str, timeout: Optional[float] = None):
        """Delete a variable from the robot's permanently saved variables.

        Note: The variable will only be removed local namespace (robot.vars) after receiving the robot response for
            this request. Beware of that if using this function asynchronously.
            Use robot.Sync() if needed, or use a timeout to make this function blocking.
        Note: You can check if the variable already exists before calling this function as follows:
            `robot.vars.get(name) is not None`

        This function does nothing if the variable does not exist.

        Args:
            name (str): Name of the variable to delete.
            timeout (float): Optional time to wait for the robot to confirm variable deletion.
                            If None:
                                In Synchronous API mode a default timeout is used.
                                In asynchronous mode a None timeout makes this function non-blocking (will not know
                                if variable deletion may have failed).
        Raises
        ------
        NotFoundException
            If the variable did not exist on the robot.
        """
        super().DeleteVariable(name)

    # mx:export_to=robot_sidecar_globals.py
    def SetVariable(self, name: str, value: any, timeout: Optional[float] = None):
        """Set a variable that was previously created on the robot using CreateVariable.
        This function can be synchronous or asynchronous (see timeout argument explained below).

        Note: There are alternatives to SetVariable that are always synchronous (waiting for robot confirmation):
              - The simplest way is to directly set the corresponding attribute of the robot class `robot.vars`:
                `robot.vars.my_var = "my_value"
              - Using the "set" method of robot.vars, which also returns the previous value
                prev_val = `robot.vars.my_var.set("my_var", "my_value")

        Args:
            name (str):  Name of the variable to create.
            value (any): New value to assign to this variable. The type will be deduced from this value.
                        The new value type must match the existing value type otherwise the robot will return an error.
            timeout (float): Optional time to wait for the robot to confirm variable modification.
                            If None:
                                In Synchronous API mode a default timeout is used.
                                In asynchronous mode a None timeout makes this function non-blocking (will not know
                                if variable modification may have failed).
        Raises
        ------
        ArgErrorException
            If the robot refused the variable creation for some reason (name or cyclic ID conflict for example).
        """
        super().SetVariable(name, value, timeout)

    # mx:export_to=robot_sidecar_globals.py
    def GetVariable(self, name: str) -> Optional[rsc.RegisteredVariable]:
        """Get a variable by name.

        This is a non-blocking function, it searches locally in the already synchronized variables map.

        This is equivalent to directly accessing the variable from `robot.vars`. For example:
            `robot.vars.my_var` => Returns the variable value
            `robot.vars.get("my_var") => Returns a rsc.RegisteredVariable

        Args:
            name (str): The name of the variable to get value for

        Returns:
            Optional[rsc.RegisteredVariable]: The found variable, or None if no variable exist with that name
        """
        return super().GetVariable(name)

    # mx:export_to=robot_sidecar_globals.py
    def GetVariableByCyclicId(self, cyclic_id: int) -> Optional[rsc.RegisteredVariable]:
        """Get a variable by cyclic ID.

        This is a non-blocking function, it searches locally in the already synchronized variables map.

        Args:
            cyclic_id (int): The variable cyclic ID to search for

        Returns:
            Optional[rsc.RegisteredVariable]: The found variable, or None if no variable exist with that cyclic_id
        """
        return super().GetVariableByCyclicId(cyclic_id)

    def ListVariables(self) -> list[str]:
        """ Return the list of robot variables.
            This is a non-blocking function that returns a list of all variable names that are synchronized
            with the robot.
        """
        return super().ListVariables()

    #####################################################################################
    # Private methods.
    #####################################################################################
    def _sidecar_get_registered_attr(self, attr_name: str) -> any:
        """ Callback for getting an attribute from local namespace.
            This function is used (for now) to detect if a registered function will overload an existing public API
            function from this class.
        """
        # Note: DO NOT CALL super()._sidecar_get_registered_attr() because here we really want to get an attribute
        #       of this derived class (for example, SetBlending or any function that does not exist on the base class).
        return getattr(Robot, attr_name, None)

    def _sidecar_set_registered_attr(self, attr_name: str, attr_val: any):
        """ Callback for setting an attribute from local namespace """
        # Set in both parent class' namespace
        super()._sidecar_set_registered_attr(attr_name, attr_val)
        setattr(Robot, attr_name, attr_val)

    def _sidecar_del_registered_attr(self, attr_name: str):
        """ Callback for deleting an attribute from local namespace """
        # Delete from both parent class' namespace
        super()._sidecar_del_registered_attr(attr_name)
        delattr(Robot, attr_name)
