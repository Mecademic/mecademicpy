"""
This file contains the Mecademic Python Robot API.
This API is implemented by class Robot.
This class provides methods to connect to Mecademic robots, query robot information, control the robot, etc.
"""
# pylint: disable=wildcard-import, unused-wildcard-import, useless-parent-delegation

from __future__ import annotations

import contextlib
import pathlib
from typing import List, Optional, Tuple, Union, overload

import mecademicpy.mecascript_classes as mecascript

from ._robot_base import _Robot, disconnect_on_exception_decorator, mecascript_global_function_decorator
from .mx_robot_def import *
from .robot_classes import *

try:
    from .robot_trajectory_files import RobotTrajectories
except ImportError:
    pass

from .tools import *


class Robot(_Robot):
    """
    Class for controlling a Mecademic robot.
    See README.md for quick instructions on how to use this class.
    """

    def __init__(self):
        super().__init__()

        self._enable_synchronous_mode = False

    def RegisterCallbacks(self, callbacks: RobotCallbacks, run_callbacks_in_separate_thread: bool):
        """
        Register callback functions to be executed when their corresponding events occur.

        Callbacks are optional, but this function must be called before connecting to the robot.
        To attach or remove individual callbacks after connecting, use ``RegisterCallback`` or
        ``UnregisterCallback``.

        Parameters
        ----------
        callbacks
            Object containing all callback functions.
        run_callbacks_in_separate_thread
            - If True, callbacks are run automatically in a separate thread (see the warning note below).
              If false, ``RunCallbacks`` must be used.
            - If False, you will need to either call RunCallbacks or call some robot wait function
              (e.g. waiting on a checkpoint, calling ``WaitIdle``) that also call ``RunCallbacks``
              internally while awaiting for the event.

        Notes
        -----
        Running callbacks in a separate thread means the user application MUST BE THREAD SAFE!
        """
        return super().RegisterCallbacks(callbacks, run_callbacks_in_separate_thread)

    def RegisterCallback(self, callback_name: str, callback_method: Callable[[], None]):
        """
        Register one callback functions to be executed when corresponding event occurs.

        Callback functions are optional.

        Notes
        -----
        Unless you previously called ``RegisterCallbacks`` with
        `run_callbacks_in_separate_thread=True`, you will need to either call ``RunCallbacks``
        or call some robot wait function (e.g., waiting on a checkpoint, calling ``WaitIdle``)
        that also call ``RunCallbacks`` internally while awaiting for the event.

        Parameters
        ----------
        callback_name
            Name of the callback to attach. Refer to class ``RobotCallbacks`` for available callbacks.
        callback_method
            Callback method to attach to the specified callback.
        """
        return super().RegisterCallback(callback_name, callback_method)

    def UnregisterCallbacks(self):
        """
        Unregister callback functions and terminate callback handler thread if applicable.
        """
        return super().UnregisterCallbacks()

    def UnregisterCallback(self, callback_name: str):
        """
        Unregister one callback functions.

        Parameters
        ----------
        callback_name
            Name of the callback to unregister. Refer to class ``RobotCallbacks`` for available
            callbacks.
        """
        return super().UnregisterCallback(callback_name)

    def RunCallbacks(self):
        """
        Run all triggered callback functions.

        This function must be called only if the ``RegisterCallback`` option
        `run_callbacks_in_separate_thread` is set to False. When True, callbacks are automatically
        executed from a background thread.

        """
        if self._callback_thread:
            raise InvalidStateError(
                'Cannot call RunCallbacks since callback handler is already running in separate thread.')

        # Setting timeout=0 means we don't block on an empty queue.
        self._handle_callbacks(polling=True)

    @disconnect_on_exception_decorator
    def Connect(self,
                address: str = MX_DEFAULT_ROBOT_IP,
                enable_synchronous_mode: bool = False,
                disconnect_on_exception: bool = True,
                monitor_mode: bool = False,
                timeout=1.0,
                set_rtc: SetRtcMode = SetRtcMode.SET_ONLY_IF_NEEDED):
        """
        Attempt to connect to a Mecademic Robot.

        This function is synchronous (awaits for success or timeout) even when using this class in
        asynchronous mode (see `enable_synchronous_mode` below).

        Parameters
        ----------
        address
            The IP address associated to the Mecademic Robot.
        enable_synchronous_mode
            Select synchronous or asynchronous mode for this class.
            See ``SetSynchronousMode`` for details.
        disconnect_on_exception
            If true, will attempt to disconnect from the robot on exception from api call.
            Also note that the robot will automatically pause motion whenever a disconnection occurs.
        monitor_mode
            If True, command connection will not be established, only monitoring connection.
        timeout
            Time allowed to try connecting to the robot before this function returns.
        set_rtc
            This parameter determines if robot date/time (RTC i.e. or real-time clock) must be updated to match
            the current PC date/time.
            This option has no effect if connecting to the robot in monitoring mode
        """
        return super()._Connect(address,
                                enable_synchronous_mode,
                                disconnect_on_exception,
                                monitor_mode,
                                offline_mode=False,
                                timeout=timeout,
                                set_rtc=set_rtc)

    def Disconnect(self):
        """
        Disconnects Mecademic Robot object from the Mecademic robot.

        This function is synchronous (awaits for disconnection or timeout) even when connected in
        asynchronous mode.
        """
        return super().Disconnect()

    @mecascript_global_function_decorator
    def IsConnected(self) -> bool:
        """
        Check whether the client is currently connected to the robot.

        See ``Connect`` for more details.

        Returns
        -------
        bool
            True if a connection to the robot is established, False otherwise.
        """
        return super().IsConnected()

    @mecascript_global_function_decorator
    def IsControlling(self) -> bool:
        """
        Check whether the client is currently connected to the robot in control mode.

        See ``Connect`` for more details.

        Returns
        -------
        bool
            True if a control-mode connection to the robot is established, False otherwise.
        """
        return super().IsControlling()

    @mecascript_global_function_decorator
    def IsSynchronousMode(self) -> bool:
        """
        Check whether the client is currently using this API in synchronous mode.

        See ``Connect`` and ``SetSynchronousMode`` for more details.

        Returns
        -------
        bool
            True if a synchronous-mode connection to the robot is established; False otherwise.
        """
        return super().IsSynchronousMode()

    @mecascript_global_function_decorator
    def IsAllowedToMove(self) -> bool:
        """
        Check whether the robot is currently allowed to move, in other words if the robot's motion queue has been
        initialized and is ready to receive new commands (regardless if the robot is in error or motion is paused).
        The robot's motion queue is initialized after the robot has been homed
        (this also includes, on Meca500 robots, robot activated but not homed when recovery mode is used).

        Returns
        -------
        bool
            True if the robot is allowed to move, False otherwise.
        """

        can_move = False
        with self._main_lock:
            can_move = self._robot_status.homing_state or (self._robot_status.activation_state
                                                           and self._robot_events.on_activate_recovery_mode.is_set())
        return can_move

    def SetSynchronousMode(self, sync_mode: bool = True):
        """
        Enable or disable synchronous mode.

        If True, calls to functions in this class block the calling thread until the robot has
        finished executing the command. Blending is not possible in this mode.

        If False, calls to functions in this class return immediately after posting the command to the
        robot, without waiting for the robot to receive or process it. In this case, commands such as
        ``WaitIdle`` can be used to wait until the robot completes the posted commands.

        Disabling synchronous mode does not affect threads that are already waiting on synchronous
        operations.

        Parameters
        ----------
        sync_mode
            If True, enable synchronous mode. If False, enable asynchronous mode.
        """
        self._enable_synchronous_mode = sync_mode

    @deprecation.deprecated(deprecated_in="2.4.0",
                            removed_in="3.0.0",
                            current_version=get_mecademicpy_version(),
                            details="Use the 'SyncCmdQueue' function instead")
    def Sync(self):
        """ Deprecated, use ``SyncCmdQueue`` instead """
        self._send_sync_command(command=None)

    @mecascript_global_function_decorator
    def SyncCmdQueue(self):
        """
        Synchronize with the robot (send a request and wait for the response).

        This ensures that, after this function returns, all responses from previous asynchronous requests have
        been received and read from the robot.

        Note that ``SyncCmdQueue`` is valid only for synchronizing with commands that produce an
        immediate response. It will not work with motion-queue commands, since their responses may
        arrive asynchronously after the response to this command.
        """
        self._send_sync_command(command=None)

    @mecascript_global_function_decorator
    def ConnectionWatchdog(self, timeout: float, message: Optional[str] = None):
        """
        Enable, refresh, or disable the connection watchdog. This function is non-blocking.

        To enable the connection watchdog, call this command with a non-zero timeout. Refresh it
        periodically by calling the command again with a non-zero timeout before the previous timeout
        expires. To disable the watchdog, call this command with a timeout of zero.

        If the watchdog is enabled and not refreshed before the specified timeout, the robot will
        close the socket connections with this application, raise the
        ``MX_SAFE_STOP_CONNECTION_DROPPED`` safety stop condition, and pause motion (if the robot is
        activated).

        If the watchdog is not enabled but the socket is closed while the robot is moving, the robot
        will still raise the ``MX_SAFE_STOP_CONNECTION_DROPPED`` safety stop condition and pause
        motion. However, sockets may take a long time to detect dropped connections under certain
        network failures. Therefore, use of the watchdog is recommended to ensure the robot quickly
        stops moving whenever the application can no longer communicate with it.

        The application can validate whether the connection watchdog is active using
        ``RobotStatus.connection_watchdog_enabled``.

        Parameters
        ----------
        timeout
            Connection watchdog timeout in seconds. Use a non-zero value to enable or refresh the
            watchdog, or 0 to disable it.
        message
            Message to record in the robot log if the connection times out after this call to
            ``ConnectionWatchdog``. This can be used to help identify what the application was doing
            when the timeout occurred.
        """
        return super().ConnectionWatchdog(timeout, message)

    @mecascript_global_function_decorator
    def AutoConnectionWatchdog(self, enable: bool, timeout: float = 0, message: Optional[str] = None):
        """
        Enable or disable the automatic connection watchdog managed by this Robot class.

        See ``ConnectionWatchdog`` for detailed information about the watchdog mechanism.

        The automatic connection watchdog provides a simpler alternative to calling
        ``ConnectionWatchdog`` directly, since the application does not need to handle periodic
        refresh calls itself.

        When enabled, the Robot class manages the connection watchdog automatically, refreshing it by
        calling ``ConnectionWatchdog`` at appropriate intervals. The timeout is set according to the
        monitoring interval (see ``SetMonitoringInterval``).

        Notes
        -----
        In many cases, explicitly calling ``ConnectionWatchdog`` in your application loop is safer than using
        ``AutoConnectionWatchdog``. If your application becomes stuck in a deadlock or infinite loop, it would stop
        calling ``ConnectionWatchdog``, causing the robot to stop — which is the safer behavior.
        In contrast, ``AutoConnectionWatchdog`` may continue to acknowledge the robot even if the application
        is unresponsive, allowing the robot to keep moving after a deadlock or infinite loop.

        Parameters
        ----------
        enable
            Whether to enable (True) or disable (False) the automatic connection watchdog refresh.
        timeout
            When enabled is False, this function will send one last ``ConnectionWatchdog`` to the robot with this
            timeout value.
            You may use zero to disable the watchdog entirely, or non-zero in case you plan to call continue calling
            ``ConnectionWatchdog`` afterwards.
        message
            Optional message to log. See ``ConnectionWatchdog`` for details.
        """
        return super().AutoConnectionWatchdog(enable, timeout, message)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def ActivateRobot(self):
        """
        Activate the robot.

        For more information, see
        [ActivateRobot](https://www.mecademic.com/res/doc?robot_command=ActivateRobot).
        """
        return super().ActivateRobot()

    @mecascript_global_function_decorator
    def DeactivateRobot(self):
        """
        Deactivate the robot.

        For more information, see
        [DeactivateRobot](https://www.mecademic.com/res/doc?robot_command=DeactivateRobot).
        """
        return super().DeactivateRobot()

    @mecascript_global_function_decorator
    def RebootRobot(self):
        """
        Reboot the robot.

        For more information, see
        [RebootRobot](https://www.mecademic.com/res/doc?robot_command=RebootRobot).
        """
        return super().RebootRobot()

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def Home(self):
        """
        Home the robot.
        For Meca500 robots only (other robots don't need homing, ``ActivateRobot`` is enough).

        For more information, see
        [Home](https://www.mecademic.com/res/doc?robot_command=Home&robot_model=MECA500).
        """
        return super().Home()

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def ActivateAndHome(self):
        """Utility function that combines activate and home."""
        super().ActivateRobot()
        super().Home()

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def PauseMotion(self):
        """
        Immediately pause robot motion.

        For more information, see
        [PauseMotion](https://www.mecademic.com/res/doc?robot_command=PauseMotion).
        """
        return super().PauseMotion()

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def ResumeMotion(self):
        """
        Un-pause robot motion.

        For more information, see
        [ResumeMotion](https://www.mecademic.com/res/doc?robot_command=ResumeMotion).
        """
        return super().ResumeMotion()

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def ClearMotion(self):
        """
        Clear the motion queue. Includes implicit ``PauseMotion`` command.

        For more information, see
        [ClearMotion](https://www.mecademic.com/res/doc?robot_command=ClearMotion).
        """
        return super().ClearMotion()

    @mecascript_global_function_decorator
    @overload
    def MoveJoints(self, joints: List[float]):
        ...

    @mecascript_global_function_decorator
    @overload
    def MoveJoints(self, j1: float, j2: float, j3: float, j4: float, j5: float = None, j6: float = None):
        ...

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def MoveJoints(self, *args: Union[List[float], float]):
        """
        Move the robot in joint mode by specifying the target joint set.

        For more information, see
        [MoveJoints](https://www.mecademic.com/res/doc?robot_command=MoveJoints).

        Parameters
        ----------
        j1
            Desired position of joint 1, in degrees.
        j2
            Desired position of joint 2, in degrees.
        j3
            Desired position of joint 3, in degrees for six-axis robots or in mm for SCARA robots.
        j4
            Desired position of joint 4, in degrees.
        j5
            Desired position of joint 5, in degrees (not applicable to SCARA robots).
        j6
            Desired position of joint 6, in degrees (not applicable to SCARA robots).
        joints
            Alternatively, a list representing the target joint set.
        """
        self._send_motion_command_check('MoveJoints', expected_count=self._robot_info.num_joints, args=args)

    @mecascript_global_function_decorator
    @overload
    def MoveJointsRel(self, joints: List[float]):
        ...

    @mecascript_global_function_decorator
    @overload
    def MoveJointsRel(self, j1: float, j2: float, j3: float, j4: float, j5: float = None, j6: float = None):
        ...

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def MoveJointsRel(self, *args: Union[List[float], float]):
        """
        Move the robot in joint mode by specifying the desired relative joint displacements.

        For more information, see
        [MoveJointsRel](https://www.mecademic.com/res/doc?robot_command=MoveJointsRel).

        Parameters
        ----------
        j1
            Desired relative displacement of joint 1, in degrees.
        j2
            Desired relative displacement of joint 2, in degrees.
        j3
            Desired relative displacement of joint 3, in degrees for six-axis robots
            or in mm for SCARA robots.
        j4
            Desired relative displacement of joint 4, in degrees.
        j5
            Desired relative displacement of joint 5, in degrees (not applicable to SCARA robots).
        j6
            Desired relative displacement of joint 6, in degrees (not applicable to SCARA robots).
        joints
            Alternatively, a list representing the desired relative joint displacements.
        """
        self._send_motion_command_check('MoveJointsRel', expected_count=self._robot_info.num_joints, args=args)

    @mecascript_global_function_decorator
    @overload
    def MoveJointsVel(self, joints: List[float]):
        ...

    @mecascript_global_function_decorator
    @overload
    def MoveJointsVel(self, j1: float, j2: float, j3: float, j4: float, j5: float = None, j6: float = None):
        ...

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def MoveJointsVel(self, *args: Union[List[float], float]):
        """
        Move the robot in joint mode by specifying the desired joint velocities.

        For more information, see
        [MoveJointsVel](https://www.mecademic.com/res/doc?robot_command=MoveJointsVel).

        Parameters
        ----------
        j1
            Desired velocity of joint 1, in °/s.
        j2
            Desired velocity displacement of joint 2, in °/s.
        j3
            Desired velocity displacement of joint 3, in °/s for six-axis robots
            or in mm/s for SCARA robots.
        j4
            Desired velocity displacement of joint 4, in °/s.
        j5
            Desired velocity displacement of joint 5, in °/s (not applicable to SCARA robots).
        j6
            Desired velocity displacement of joint 6, in °/s (not applicable to SCARA robots).
        joints
            Alternatively, a list representing the desired relative joint displacements.
        """
        self._send_motion_command_check('MoveJointsVel', expected_count=self._robot_info.num_joints, args=args)

    @mecascript_global_function_decorator
    @overload
    def MovePose(self, x: float, y: float, z: float, gamma: float):
        ...

    @mecascript_global_function_decorator
    @overload
    def MovePose(self, x: float, y: float, z: float, alpha: float, beta: float, gamma: float):
        ...

    @mecascript_global_function_decorator
    @overload
    def MovePose(self, pose: List[float]):
        ...

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def MovePose(self,
                 x: Union[List[float], float],
                 y: float = None,
                 z: float = None,
                 alpha: float = None,
                 beta: float = None,
                 gamma: float = None):
        """
        Move the robot's TRF to a specified pose relative to the WRF. The resulting motion is linear
        in the joint space but nonlinear in the Cartesian space. All joint movements start and stop
        simultaneously.

        For more information, see
        [MovePose](https://www.mecademic.com/res/doc?robot_command=MovePose).

        Parameters
        ----------
        x
            Target x coordinate, in mm.
        y
            Target y coordinate, in mm.
        z
            Target z coordinate, in mm.
        alpha
            First of the three Euler angles (intrinsic XYZ convention) representing the target
            orientation, in degrees. Used only on six-axis robots.
        beta
            Second of the three Euler angles (intrinsic XYZ convention) representing the target
            orientation, in degrees. Used only on six-axis robots.
        gamma
            Third of the three Euler angles (intrinsic XYZ convention) representing the target
            orientation, in degrees.
        pose
            Alternatively, a list representing the desired pose of the TRF relative to the WRF.
        """
        [x, y, z, alpha, beta, gamma] = self._normalize_cart_cmd_args(x, y, z, alpha, beta, gamma)
        self._send_motion_command('MovePose', [x, y, z, alpha, beta, gamma])

    @mecascript_global_function_decorator
    @overload
    def MoveJump(self, x: float, y: float, z: float, gamma: float):
        ...

    @mecascript_global_function_decorator
    @overload
    def MoveJump(self, x: float, y: float, z: float, alpha: float, beta: float, gamma: float):
        ...

    @mecascript_global_function_decorator
    @overload
    def MoveJump(self, pose: List[float]):
        ...

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def MoveJump(self,
                 x: Union[List[float], float],
                 y: float = None,
                 z: float = None,
                 alpha: float = None,
                 beta: float = None,
                 gamma: float = None):
        """
        Move the robot's TRF to a specified pose relative to the WRF by performing a jump motion
        in Cartesian space. The jump motion parameters are configured using ``SetMoveJumpHeight`` and
        ``SetMoveJumpApproachVel``.

        This command is currently supported only on SCARA robots and is particularly useful for
        pick-and-place applications.

        For more information, see
        [MoveJump](https://www.mecademic.com/res/doc?robot_command=MoveJump&robot_model=MCS500).

        Parameters
        ----------
         x
            Target x coordinate, in mm.
        y
            Target y coordinate, in mm.
        z
            Target z coordinate, in mm.
        alpha
            First of the three Euler angles (intrinsic XYZ convention) representing the target
            orientation, in degrees. Used only on six-axis robots.
        beta
            Second of the three Euler angles (intrinsic XYZ convention) representing the target
            orientation, in degrees. Used only on six-axis robots.
        gamma
            Third of the three Euler angles (intrinsic XYZ convention) representing the target
            orientation, in degrees.
        pose
            Alternatively, a list representing the desired pose of the TRF relative to the WRF.

        """
        [x, y, z, alpha, beta, gamma] = self._normalize_cart_cmd_args(x, y, z, alpha, beta, gamma)
        self._send_motion_command('MoveJump', [x, y, z, alpha, beta, gamma])

    @mecascript_global_function_decorator
    @overload
    def MoveLin(self, x: float, y: float, z: float, gamma: float):
        ...

    @mecascript_global_function_decorator
    @overload
    def MoveLin(self, x: float, y: float, z: float, alpha: float, beta: float, gamma: float):
        ...

    @mecascript_global_function_decorator
    @overload
    def MoveLin(self, pose: List[float]):
        ...

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def MoveLin(self,
                x: Union[List[float], float],
                y: float = None,
                z: float = None,
                alpha: float = None,
                beta: float = None,
                gamma: float = None):
        """
        Move the robot's TRF to a specified pose relative to the WRF by performing a linear motion
        in Cartesian space.

        For more information, see
        [MoveLin](https://www.mecademic.com/res/doc?robot_command=MoveLin).

        Parameters
        ----------
        x
            Target x coordinate, in mm.
        y
            Target y coordinate, in mm.
        z
            Target z coordinate, in mm.
        alpha
            First of the three Euler angles (intrinsic XYZ convention) representing the target
            orientation, in degrees. Used only on six-axis robots.
        beta
            Second of the three Euler angles (intrinsic XYZ convention) representing the target
            orientation, in degrees. Used only on six-axis robots.
        gamma
            Third of the three Euler angles (intrinsic XYZ convention) representing the target
            orientation, in degrees.
        pose
            Alternatively, a list representing the desired pose of the TRF relative to the WRF.
        """
        [x, y, z, alpha, beta, gamma] = self._normalize_cart_cmd_args(x, y, z, alpha, beta, gamma)
        self._send_motion_command('MoveLin', [x, y, z, alpha, beta, gamma])

    @mecascript_global_function_decorator
    @overload
    def MoveLinRelTrf(self, x: float, y: float, z: float, gamma: float):
        ...

    @mecascript_global_function_decorator
    @overload
    def MoveLinRelTrf(self, x: float, y: float, z: float, alpha: float, beta: float, gamma: float):
        ...

    @mecascript_global_function_decorator
    @overload
    def MoveLinRelTrf(self, pose: List[float]):
        ...

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def MoveLinRelTrf(self,
                      x: Union[List[float], float],
                      y: float = None,
                      z: float = None,
                      alpha: float = None,
                      beta: float = None,
                      gamma: float = None):
        """
        Move the robot's TRF to a specified pose relative to the current pose of the TRF by performing
        a linear motion in Cartesian space.

        For more information, see
        [MoveLinRelTrf](https://www.mecademic.com/res/doc?robot_command=MoveLinRelTrf).

        Parameters
        ----------
        x
            Target x coordinate, in mm.
        y
            Target y coordinate, in mm.
        z
            Target z coordinate, in mm.
        alpha
            First of the three Euler angles (intrinsic XYZ convention) representing the target
            orientation, in degrees. Used only on six-axis robots.
        beta
            Second of the three Euler angles (intrinsic XYZ convention) representing the target
            orientation, in degrees. Used only on six-axis robots.
        gamma
            Third of the three Euler angles (intrinsic XYZ convention) representing the target
            orientation, in degrees.
        pose
            Alternatively, a list representing the desired relative pose.
        """
        [x, y, z, alpha, beta, gamma] = self._normalize_cart_cmd_args(x, y, z, alpha, beta, gamma)
        self._send_motion_command('MoveLinRelTrf', [x, y, z, alpha, beta, gamma])

    @deprecation.deprecated(deprecated_in="1.2.0",
                            removed_in="3.0.0",
                            current_version=get_mecademicpy_version(),
                            details="Use the 'MoveLinRelTrf' function instead")
    @disconnect_on_exception_decorator
    def MoveLinRelTRF(self, x: float, y: float, z: float, alpha: float = None, beta: float = None, gamma: float = None):
        """Deprecated, use ``MoveLinRelTrf`` instead.
        """
        self.MoveLinRelTrf(x, y, z, alpha, beta, gamma)

    @mecascript_global_function_decorator
    @overload
    def MoveLinRelWrf(self, x: float, y: float, z: float, gamma: float):
        ...

    @mecascript_global_function_decorator
    @overload
    def MoveLinRelWrf(self, x: float, y: float, z: float, alpha: float, beta: float, gamma: float):
        ...

    @mecascript_global_function_decorator
    @overload
    def MoveLinRelWrf(self, pose: List[float]):
        ...

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def MoveLinRelWrf(self,
                      x: Union[List[float], float],
                      y: float = None,
                      z: float = None,
                      alpha: float = None,
                      beta: float = None,
                      gamma: float = None):
        """
        Move the robot's TRF to a specified pose defined with respect to a reference frame that has the same
        orientation as the WRF but its origin is at the current position of the TCP.

        For more information, see
        [MoveLinRelWrf](https://www.mecademic.com/res/doc?robot_command=MoveLinRelWrf).

        Parameters
        ----------
        x
            Target x coordinate, in mm.
        y
            Target y coordinate, in mm.
        z
            Target z coordinate, in mm.
        alpha
            First of the three Euler angles (intrinsic XYZ convention) representing the target
            orientation, in degrees. Used only on six-axis robots.
        beta
            Second of the three Euler angles (intrinsic XYZ convention) representing the target
            orientation, in degrees. Used only on six-axis robots.
        gamma
            Third of the three Euler angles (intrinsic XYZ convention) representing the target
            orientation, in degrees.
        pose
            Alternatively, a list representing the desired relative pose.
        """
        [x, y, z, alpha, beta, gamma] = self._normalize_cart_cmd_args(x, y, z, alpha, beta, gamma)
        self._send_motion_command('MoveLinRelWrf', [x, y, z, alpha, beta, gamma])

    @deprecation.deprecated(deprecated_in="1.2.0",
                            removed_in="3.0.0",
                            current_version=get_mecademicpy_version(),
                            details="Use the 'MoveLinRelWrf' function instead")
    @disconnect_on_exception_decorator
    def MoveLinRelWRF(self,
                      x: Union[List[float], float],
                      y: float = None,
                      z: float = None,
                      alpha: float = None,
                      beta: float = None,
                      gamma: float = None):
        """Deprecated, use ``MoveLinRelWrf`` instead.
        """
        self.MoveLinRelWrf(x, y, z, alpha, beta, gamma)

    @mecascript_global_function_decorator
    @overload
    def MoveLinVelTrf(self, v_x: float, v_y: float, v_z: float, omega_z: float):
        ...

    @mecascript_global_function_decorator
    @overload
    def MoveLinVelTrf(self, v_x: float, v_y: float, v_z: float, omega_x: float, omega_y: float, omega_z: float):
        ...

    @mecascript_global_function_decorator
    @overload
    def MoveLinVelTrf(self, twist: List[float]):
        ...

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def MoveLinVelTrf(self,
                      v_x: Union[List[float], float],
                      v_y: float = None,
                      v_z: float = None,
                      omega_x: float = None,
                      omega_y: float = None,
                      omega_z: float = None):
        """
        Move the robot's TRF at the specified Cartesian velocity, defined relative to the TRF.

        The robot will decelerate to a complete stop after the duration specified by the command
        ``SetVelTimeout``, unless a subsequent ``MoveLinVelTrf`` or ``MoveLinVelWrf`` command is
        issued.

        For more information, see
        [MoveLinVelTrf](https://www.mecademic.com/res/doc?robot_command=MoveLinVelTrf).

        Parameters
        ----------
        v_x
            The x components of the linear velocity of the TCP expressed in the TRF, in mm/s.
        v_y
            The y components of the linear velocity of the TCP expressed in the TRF, in mm/s.
        v_x
            The z components of the linear velocity of the TCP expressed in the TRF, in mm/s.
        omega_x
            The x component of the angular velocity of the TRF expressed in the TRF, in °/s.
            Used only on six-axis robots.
        omega_y
            The y component of the angular velocity of the TRF expressed in the TRF, in °/s.
            Used only on six-axis robots.
        omega_z
            The z component of the angular velocity of the TRF expressed in the TRF, in °/s.
        twist
            Alternatively, a list representing the Cartesian velocity (twist).
        """
        [v_x, v_y, v_z, omega_x, omega_y, omega_z] = self._normalize_cart_cmd_args(v_x, v_y, v_z, omega_x, omega_y,
                                                                                   omega_z)
        self._send_motion_command('MoveLinVelTrf', [v_x, v_y, v_z, omega_x, omega_y, omega_z])

    @deprecation.deprecated(deprecated_in="1.2.0",
                            removed_in="3.0.0",
                            current_version=get_mecademicpy_version(),
                            details="Use the 'MoveLinVelTrf' function instead")
    @disconnect_on_exception_decorator
    def MoveLinVelTRF(self,
                      v_x: Union[List[float], float],
                      v_y: float = None,
                      v_z: float = None,
                      omega_x: float = None,
                      omega_y: float = None,
                      omega_z: float = None):
        """Deprecated, use ``MoveLinVelTrf`` instead

        """
        self.MoveLinVelTrf(v_x, v_y, v_z, omega_x, omega_y, omega_z)

    @mecascript_global_function_decorator
    @overload
    def MoveLinVelWrf(self, v_x: float, v_y: float, v_z: float, omega_z: float):
        ...

    @mecascript_global_function_decorator
    @overload
    def MoveLinVelWrf(self, v_x: float, v_y: float, v_z: float, omega_x: float, omega_y: float, omega_z: float):
        ...

    @mecascript_global_function_decorator
    @overload
    def MoveLinVelWrf(self, pose: List[float]):
        ...

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def MoveLinVelWrf(self,
                      v_x: Union[List[float], float],
                      v_y: float = None,
                      v_z: float = None,
                      omega_x: float = None,
                      omega_y: float = None,
                      omega_z: float = None):
        """
        Move the robot's TRF at the specified Cartesian velocity, defined relative to the WRF.

        The robot will decelerate to a complete stop after the duration specified by the command
        ``SetVelTimeout``, unless a subsequent ``MoveLinVelTrf`` or ``MoveLinVelWrf`` command is
        issued.

        For more information, see
        [MoveLinVelWrf](https://www.mecademic.com/res/doc?robot_command=MoveLinVelWrf).

        Parameters
        ----------
        v_x
            The x components of the linear velocity of the TCP expressed in the WRF, in mm/s.
        v_y
            The y components of the linear velocity of the TCP expressed in the WRF, in mm/s.
        v_x
            The z components of the linear velocity of the TCP expressed in the WRF, in mm/s.
        omega_x
            The x component of the angular velocity of the TRF expressed in the WRF, in °/s.
            Used only on six-axis robots.
        omega_y
            The y component of the angular velocity of the TRF expressed in the WRF, in °/s.
            Used only on six-axis robots.
        omega_z
            The z component of the angular velocity of the TRF expressed in the WRF, in °/s.
        twist
            Alternatively, a list representing the Cartesian velocity (twist).
        """
        [v_x, v_y, v_z, omega_x, omega_y, omega_z] = self._normalize_cart_cmd_args(v_x, v_y, v_z, omega_x, omega_y,
                                                                                   omega_z)
        self._send_motion_command('MoveLinVelWrf', [v_x, v_y, v_z, omega_x, omega_y, omega_z])

    @deprecation.deprecated(deprecated_in="1.2.0",
                            removed_in="3.0.0",
                            current_version=get_mecademicpy_version(),
                            details="Use the 'MoveLinVelWrf' function instead")
    @disconnect_on_exception_decorator
    def MoveLinVelWRF(self,
                      v_x: Union[List[float], float],
                      v_y: float = None,
                      v_z: float = None,
                      omega_x: float = None,
                      omega_y: float = None,
                      omega_z: float = None):
        """Deprecated, use ``MoveLinVelWrf`` instead

        """
        self.MoveLinVelWrf(v_x, v_y, v_z, omega_x, omega_y, omega_z)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetVelTimeout(self, t: float):
        """
        Defines the timeout period following a velocity-mode motion command.

        For more information, see
        [SetVelTimeout](https://www.mecademic.com/res/doc?robot_command=SetVelTimeout).

        Parameters
        ----------
        t
            Desired timeout period, in seconds. Default value restored at robot activation:
            ``MX_MQ_DEFAULT_VEL_TIMEOUT``

        """
        self._send_motion_command('SetVelTimeout', [t])

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetConf(self, shoulder: int = None, elbow: int = None, wrist: int = None):
        """
        Sets the desired posture configuration to be observed in the ``MovePose`` and ``MoveLin*``
        commands.

        For more information, see
        [SetConf](https://www.mecademic.com/res/doc?robot_command=SetConf).

        Parameters
        ----------
        shoulder
            Shoulder configuration parameter (-1 or 1). Used only on six-axis robots.
            Default value restored at robot activation: ``MX_MQ_DEFAULT_SHOULDER_CONF``
        elbow
            Elbow configuration parameter (-1 or 1).
            Default value restored at robot activation: ``MX_MQ_DEFAULT_ELBOW_CONF``
        wrist
            Wrist configuration parameter (-1 or 1). Used only on six-axis robots.
            Default value restored at robot activation: ``MX_MQ_DEFAULT_WRIST_CONF``

        Notes
        -----
        Alternatively, all parameters can be provided as a list of integers.
        """
        [shoulder, elbow, wrist] = self._normalize_conf_cmd_args(shoulder, elbow, wrist)
        self._send_motion_command('SetConf', [shoulder, elbow, wrist])

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetAutoConf(self, e: Union[int, bool]):
        """
        Enables or disables the automatic posture configuration selection, to be observed in the
        ``MovePose``  and ``MoveLin*`` commands.

        For more information, see
        [SetAutoConf](https://www.mecademic.com/res/doc?robot_command=SetAutoConf).

        Parameters
        ----------
        e
            If true, robot will automatically choose the best configuration for the desired pose.
            Default value restored at robot activation: ``MX_MQ_DEFAULT_AUTO_CONF``.
        """
        self._send_motion_command('SetAutoConf', [int(e)])

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetConfTurn(self, n: int):
        """
        Sets the desired turn configuration for the last joint, to be observed in the ``MovePose`` and
        ``MoveLin*`` commands.

        For more information, see
        [SetConfTurn](https://www.mecademic.com/res/doc?robot_command=SetConfTurn).

        Parameters
        ----------
        n
            The turn number for the last joint.
            Default value restored at robot activation: ``MX_MQ_DEFAULT_CONF_TURN``.
        """
        self._send_motion_command('SetConfTurn', [n])

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetAutoConfTurn(self, e: Union[int, bool]):
        """
        Enables or disables the automatic turn configuration selection for the last joint of the
        robot, to be observed in the ``MovePose`` and ``MoveLin*`` commands.

        For more information, see
        [SetAutoConfTurn](https://www.mecademic.com/res/doc?robot_command=SetAutoConfTurn).

        Parameters
        ----------
        e
            If true, robot will automatically choose the best turn configuration for the desired pose.
            Default value restored at robot activation: ``MX_MQ_DEFAULT_AUTO_CONF_TURN``.
        """
        self._send_motion_command('SetAutoConfTurn', [int(e)])

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetBlending(self, p: float):
        """
        Sets the percentage of blending between consecutive movements with the position-mode
        joint-space commands or between consecutive movements with the position-mode Cartesian-space
        commands.

        For more information, see
        [SetBlending](https://www.mecademic.com/res/doc?robot_command=SetBlending).

        Notes
        -----
        There can't be blending between joint mode and Cartesian mode moves

        Parameters
        ----------
        p
            Percentage of blending.
            Default value restored at robot activation: ``MX_MQ_DEFAULT_BLENDING``.
        """
        self._send_motion_command('SetBlending', [p])

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetCartAcc(self, p: float):
        """
        Limits the Cartesian acceleration (both linear and angular) of the TRF relative to the WRF
        during movements resulting from Cartesian-space commands.

        For more information, see
        [SetCartAcc](https://www.mecademic.com/res/doc?robot_command=SetCartAcc).

        Parameters
        ----------
        p
            Percentage of maximum acceleration.
            Default value restored at robot activation: ``MX_MQ_DEFAULT_CART_ACC``.
        """
        self._send_motion_command('SetCartAcc', [p])

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetCartAngVel(self, w: float):
        """
        Sets the maximum angular velocity of the robot TRF with respect to its WRF for position-mode
        ``MoveLin*`` commands.

        For more information, see
        [SetCartAngVel](https://www.mecademic.com/res/doc?robot_command=SetCartAngVel).

        Notes
        -----
        Actual angular velocity may be lower if necessary to avoid exceeding maximum joint velocity.

        Parameters
        ----------
        w
            Maximum angular velocity in °/s.
            Default value restored at robot activation: ``MX_MQ_DEFAULT_CART_ANG_VEL``.
        """
        self._send_motion_command('SetCartAngVel', [w])

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetCartLinVel(self, v: float):
        """
        Sets the maximum linear velocity of the robot TRF with respect to its WRF for position-mode
        ``MoveLin*`` commands.

        For more information, see
        [SetCartLinVel](https://www.mecademic.com/res/doc?robot_command=SetCartLinVel).

        Notes
        -----
        Actual linear velocity may be lower if necessary to avoid exceeding maximum joint velocity.

        Parameters
        ----------
        v
            Maximum linear velocity in mm/s.
            Default value restored at robot activation: ``MX_MQ_DEFAULT_CART_LIN_VEL``.
        """
        self._send_motion_command('SetCartLinVel', [v])

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetJointAcc(self, p: float):
        """
        Limits the acceleration of the joints during movements resulting from joint-space commands.

        For more information, see
        [SetJointAcc](https://www.mecademic.com/res/doc?robot_command=SetJointAcc).

        Parameters
        ----------
        p
            Percentage of maximum acceleration of the joints.
            Default value restored at robot activation: ``MX_MQ_DEFAULT_JOINT_ACC``.
        """
        self._send_motion_command('SetJointAcc', [p])

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetJointVel(self, p: float):
        """
        Sets the desired velocities of the robot joints during movements generated by position-mode
        joint-space commands.

        For more information, see
        [SetJointVel](https://www.mecademic.com/res/doc?robot_command=SetJointVel).

        Parameters
        ----------
        p
            Percentage of the top rated joint velocities.
            Default value restored at robot activation: ``MX_MQ_DEFAULT_JOINT_VEL``.
        """
        self._send_motion_command('SetJointVel', [p])

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetJointVelLimit(self, p: float):
        """
        Overrides the default joint velocity limit, affecting all ``Move*`` commands.

        For more information, see
        [SetJointVelLimit](https://www.mecademic.com/res/doc?robot_command=SetJointVelLimit).

        Parameters
        ----------
        p
            Percentage of the top rated joint velocities.
            Default value restored at robot activation: ``MX_MQ_DEFAULT_JOINT_VEL_LIMIT``.
        """
        self._send_motion_command('SetJointVelLimit', [p])

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetMoveMode(self, mode: MxMoveMode):
        """
        Selects between velocity-based or time-based move sub-mode for subsequent position-mode
        ``Move*`` commands.

        For more information, see
        [SetMoveMode](https://www.mecademic.com/res/doc?robot_command=SetMoveMode).

        Parameters
        ----------
        mode
            The move sub-mode to use.
            Default value restored at robot activation: ``MxMoveMode.MX_MOVE_MODE_VELOCITY``.
        """
        self._send_motion_command('SetMoveMode', [mode])

    @mecascript_global_function_decorator
    @overload
    def SetMoveDurationCfg(self, severity: MxEventSeverity):
        ...

    @mecascript_global_function_decorator
    @overload
    def SetMoveDurationCfg(self, cfg: MoveDurationCfg, /):
        ...

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetMoveDurationCfg(self, severity: Union[MxEventSeverity, MoveDurationCfg]):
        """
        Specifies what happens when a ``Move*`` command cannot meet the desired duration set by
        ``SetMoveDuration``, in time-based move sub-mode.

        For more information, see
        [SetMoveDurationCfg](https://www.mecademic.com/res/doc?robot_command=SetMoveDurationCfg).

        Parameters
        ----------
        severity
            Severity level when the requested move duration is too short for the robot.
            The allowed values are:

            - ``MX_EVENT_SEVERITY_SILENT``
            - ``MX_EVENT_SEVERITY_WARNING``
            - ``MX_EVENT_SEVERITY_ERROR``
        cfg
            Alternatively, you can provide MoveDurationCfg as a single argument

        Notes
        -----
            Default severity restored at robot activation: ``MX_EVENT_SEVERITY_WARNING``
        """
        if isinstance(severity, MoveDurationCfg):
            severity = severity.severity
        self._send_motion_command('SetMoveDurationCfg', [severity])

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetMoveDuration(self, t: float):
        """
        Sets the desired duration for the movement resulting from every subsequent position-mode
        command, when the move sub-mode is time-based (see ``SetMoveMode``).

        For more information, see
        [SetMoveDuration](https://www.mecademic.com/res/doc?robot_command=SetMoveDuration).

        Parameters
        ----------
        t
            Move duration in seconds.
            Default value restored at robot activation: ``MX_MQ_DEFAULT_MOVE_DURATION``
        """
        self._send_motion_command('SetMoveDuration', [t])

    @mecascript_global_function_decorator
    @overload
    #pylint: disable=invalid-name
    def SetMoveJumpHeight(self,
                          startHeight: float = MX_MQ_DEFAULT_MOVE_JUMP_HEIGHT,
                          endHeight: float = MX_MQ_DEFAULT_MOVE_JUMP_HEIGHT,
                          minHeight: float = MX_MQ_DEFAULT_MOVE_JUMP_MIN_HEIGHT,
                          maxHeight: float = MX_MQ_DEFAULT_MOVE_JUMP_MAX_HEIGHT):
        ...

    @mecascript_global_function_decorator
    @overload
    def SetMoveJumpHeight(self, params: MoveJumpHeight, /):  # pylint: disable=arguments-differ
        ...

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    # pylint: disable=invalid-name
    def SetMoveJumpHeight(self,
                          startHeight: Union[float, MoveJumpHeight] = MX_MQ_DEFAULT_MOVE_JUMP_HEIGHT,
                          endHeight: float = MX_MQ_DEFAULT_MOVE_JUMP_HEIGHT,
                          minHeight: float = MX_MQ_DEFAULT_MOVE_JUMP_MIN_HEIGHT,
                          maxHeight: float = MX_MQ_DEFAULT_MOVE_JUMP_MAX_HEIGHT):
        """
        Set height parameters for the ``MoveJump`` command.

        Height parameters control how high the robot moves vertically at start, middle and end.
        Height values provided can be positive or negative (relative to WRF's z axis).
        Space between minimum and maximum allows the robot flexibility to choose the optimal path.
        An error is raised if joint limits prevent the robot from respecting minimum heights.

        For more information, see
        [SetMoveJumpHeight](https://www.mecademic.com/res/doc?robot_command=SetMoveJumpHeight&robot_model=MCS500).

        Parameters
        ----------
        startHeight
            Height of the initial pure vertical translation, in mm.
        endHeight
            Height of the final pure vertical translation, in mm.
        minHeight
            Minimum height to reach while performing the lateral motion, with respect to the highest
            (if startHeight and endHeight are positive) or lowest (if startHeight and endHeight are negative)
            between the start and end poses, in mm.
        maxHeight
            Maximum height to reach while performing the lateral motion, with respect to the highest
            (if startHeight and endHeight are positive) or lowest (if startHeight and endHeight are negative)
            between the start and end poses, in mm.
        params
            Alternatively, you can provide MoveJumpHeight as a single argument

        Notes
        -----
        Default values restored at robot activation correspond to default argument values.
        """
        if isinstance(startHeight, MoveJumpHeight):
            endHeight = startHeight.endHeight
            minHeight = startHeight.minHeight
            maxHeight = startHeight.maxHeight
            startHeight = startHeight.startHeight
        self._send_motion_command('SetMoveJumpHeight', [startHeight, endHeight, minHeight, maxHeight])

    @mecascript_global_function_decorator
    @overload
    #pylint: disable=invalid-name
    def SetMoveJumpApproachVel(self,
                               startVel: float = MX_MQ_DEFAULT_MOVE_JUMP_APPROACH_VEL,
                               startDist: float = MX_MQ_DEFAULT_MOVE_JUMP_APPROACH_DIST,
                               endVel: float = MX_MQ_DEFAULT_MOVE_JUMP_APPROACH_VEL,
                               endDist: float = MX_MQ_DEFAULT_MOVE_JUMP_APPROACH_DIST):
        ...

    @mecascript_global_function_decorator
    @overload
    def SetMoveJumpApproachVel(self, params: MoveJumpApproachVel, /):  # pylint: disable=arguments-differ
        ...

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    #pylint: disable=invalid-name
    def SetMoveJumpApproachVel(self,
                               startVel: Union[float, MoveJumpApproachVel] = MX_MQ_DEFAULT_MOVE_JUMP_APPROACH_VEL,
                               startDist: float = MX_MQ_DEFAULT_MOVE_JUMP_APPROACH_DIST,
                               endVel: float = MX_MQ_DEFAULT_MOVE_JUMP_APPROACH_VEL,
                               endDist: float = MX_MQ_DEFAULT_MOVE_JUMP_APPROACH_DIST):
        """
        Sets the maximum allowed vertical speed during the initial and final movements of the
        ``MoveJump`` motion.

        #pylint: disable=line-too-long
        For more information, see
        [SetMoveJumpApproachVel](https://www.mecademic.com/res/doc?robot_command=SetMoveJumpApproachVel&robot_model=MCS500).

        Parameters
        ----------
        startVel
            Maximum allowed vertical speed near the start pose, in mm/s. Use 0 for unlimited.
        startDist
            Initial portion of the retreat motion during which startVel is applied, in mm.
        endVel
            Maximum allowed vertical speed near the end pose, in mm/s. Use 0 for unlimited.
        endDist
            Final portion of the approach motion during which endVel is applied, in mm.
        params
            Alternatively, you can provide MoveJumpApproachVel as a single argument

        Notes
        -----
        Default values restored at robot activation correspond to default argument values.
        """
        if isinstance(startVel, MoveJumpApproachVel):
            startDist = startVel.startDist
            endVel = startVel.endVel
            endDist = startVel.endDist
            startVel = startVel.startVel
        self._send_motion_command('SetMoveJumpApproachVel', [startVel, startDist, endVel, endDist])

    @mecascript_global_function_decorator
    @overload
    def SetTrf(self, x: float, y: float, z: float, gamma: float):
        ...

    @mecascript_global_function_decorator
    @overload
    def SetTrf(self, x: float, y: float, z: float, alpha: float, beta: float, gamma: float):
        ...

    @mecascript_global_function_decorator
    @overload
    def SetTrf(self, pose: List[float]):
        ...

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetTrf(self,
               x: Union[List[float], float],
               y: float = None,
               z: float = None,
               alpha: float = None,
               beta: float = None,
               gamma: float = None):
        """
        Defines the pose of the TRF with respect to the FRF.

        For more information, see
        [SetTrf](https://www.mecademic.com/res/doc?robot_command=SetTrf).

        Parameters
        ----------
        x
            The x coordinate of the origin of the TRF with respect to the FRF, in mm.
        y
            The y coordinate of the origin of the TRF with respect to the FRF, in mm.
        z
            The z coordinate of the origin of the TRF with respect to the FRF, in mm.
        alpha
            First of the three Euler angles (intrinsic XYZ convention) representing the
            orientation of the TRF with respect to the FRF, in degrees. Used only on six-axis robots.
        beta
            Second of the three Euler angles (intrinsic XYZ convention) representing the
            orientation of the TRF with respect to the FRF, in degrees. Used only on six-axis robots.
        gamma
            Third of the three Euler angles (intrinsic XYZ convention) representing the
            orientation of the TRF with respect to the FRF, in degrees.
        pose
            Alternatively, a list representing the desired pose of the TRF relative to the FRF.
        """
        [x, y, z, alpha, beta, gamma] = self._normalize_cart_cmd_args(x, y, z, alpha, beta, gamma)
        self._send_motion_command('SetTrf', [x, y, z, alpha, beta, gamma])

    @deprecation.deprecated(deprecated_in="1.2.0",
                            removed_in="3.0.0",
                            current_version=get_mecademicpy_version(),
                            details="Use the 'SetTrf' function instead")
    @disconnect_on_exception_decorator
    def SetTRF(self,
               x: float,
               y: float = None,
               z: float = None,
               alpha: float = None,
               beta: float = None,
               gamma: float = None):
        """Deprecated, use ``SetTrf`` instead.
        """
        self.SetTrf(x, y, z, alpha, beta, gamma)

    @mecascript_global_function_decorator
    @overload
    def SetWrf(self, x: float, y: float, z: float, gamma: float):
        ...

    @mecascript_global_function_decorator
    @overload
    def SetWrf(self, x: float, y: float, z: float, alpha: float, beta: float, gamma: float):
        ...

    @mecascript_global_function_decorator
    @overload
    def SetWrf(self, pose: List[float]):
        ...

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetWrf(self,
               x: Union[List[float], float],
               y: float = None,
               z: float = None,
               alpha: float = None,
               beta: float = None,
               gamma: float = None):
        """
        Defines the pose of the WRF with respect to the BRF.

        For more information, see
        [SetWrf](https://www.mecademic.com/res/doc?robot_command=SetWrf).

        Parameters
        ----------
        x
            The x coordinate of the origin of the WRF with respect to the BRF, in mm.
        y
            The y coordinate of the origin of the WRF with respect to the BRF, in mm.
        z
            The z coordinate of the origin of the WRF with respect to the BRF, in mm.
        alpha
            First of the three Euler angles (intrinsic XYZ convention) representing the
            orientation of the WRF with respect to the BRF, in degrees. Used only on six-axis robots.
        beta
            Second of the three Euler angles (intrinsic XYZ convention) representing the
            orientation of the WRF with respect to the BRF, in degrees. Used only on six-axis robots.
        gamma
            Third of the three Euler angles (intrinsic XYZ convention) representing the
            orientation of the WRF with respect to the BRF, in degrees.
        pose
            Alternatively, a list representing the desired pose of the WRF relative to the BRF.
        """
        [x, y, z, alpha, beta, gamma] = self._normalize_cart_cmd_args(x, y, z, alpha, beta, gamma)
        self._send_motion_command('SetWrf', [x, y, z, alpha, beta, gamma])

    @deprecation.deprecated(deprecated_in="1.2.0",
                            removed_in="3.0.0",
                            current_version=get_mecademicpy_version(),
                            details="Use the 'SetWrf' function instead")
    @disconnect_on_exception_decorator
    def SetWRF(self,
               x: float,
               y: float = None,
               z: float = None,
               alpha: float = None,
               beta: float = None,
               gamma: float = None):
        """Deprecated, use ``SetWrf`` instead
        """
        self.SetWrf(x, y, z, alpha, beta, gamma)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetCheckpoint(self, n: int) -> InterruptableEvent:
        """
        Defines a checkpoint in the motion queue, and then returns an ``InterruptableEvent`` that can
        be used at any time to wait until the robot's motion queue execution reaches this checkpoint.

        This command is non-blocking whether robot connection is in asynchronous or synchronous mode.
        Therefore, it is required to use the ``Wait*`` command of the return object to catch the
        checkpoint event.

        For more information, see
        [SetCheckpoint](https://www.mecademic.com/res/doc?robot_command=SetCheckpoint).

        Parameters
        ----------
        n
            Desired checkpoint id.

        Returns
        -------
        InterruptableEvent
            Object to use to wait for the checkpoint.
        """
        with self._main_lock:
            self._check_internal_states()
            assert MX_CHECKPOINT_ID_MIN <= n <= MX_CHECKPOINT_ID_MAX
            return self._set_checkpoint_impl(n)

    @disconnect_on_exception_decorator
    def ExpectExternalCheckpoint(self, n: int) -> InterruptableEvent:
        """
        Creates an interruptable event that will be set when the robot reaches the specified external
        checkpoint, typically from a saved program. Call this function before starting the program;
        otherwise the checkpoint signal could be missed.

        This command is non-blocking whether robot connection is in asynchronous or synchronous mode.
        Therefore, it is required to use the ``Wait*`` command of the return object to catch the
        checkpoint event.

        Parameters
        ----------
        n
            Id of expected checkpoint.

        Returns
        -------
        InterruptableEvent
            Object to use to wait for the checkpoint.
        """
        with self._main_lock:
            self._check_internal_states()
            assert MX_CHECKPOINT_ID_MIN <= n <= MX_CHECKPOINT_ID_MAX
            return self._set_checkpoint_impl(n, send_to_robot=False)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def WaitGripperMoveCompletion(self, timeout: Optional[float] = None):
        """
        Waits for the most recent gripper move command to complete.

        After a gripper command, any motion command queued after it will begin as soon as the gripper
        starts moving. If you want the robot to remain still until the gripper action finishes, call
        ``WaitGripperMoveCompletion`` before posting subsequent motion commands.

        Parameters
        ----------
        timeout
            Maximum duration to wait for the move to complete, in seconds.
        """
        return super().WaitGripperMoveCompletion(timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GripperOpen(self):
        """
        Opens the gripper.

        For more information, see
        [GripperOpen](https://www.mecademic.com/res/doc?robot_command=GripperOpen&robot_model=MECA500).

        Notes
        -----
        ``GripperOpen`` is queued in the robot's motion queue along with motion commands,
        but its execution is processed in parallel with robot motion:

        - It starts at the beginning of the blending (or deceleration) phase
          of the previous motion command in the queue.
        - It continues executing in parallel with any subsequent motion commands, which begin
          immediately while ``GripperOpen`` starts executing.

        To coordinate with motion commands:

        - To ensure ``GripperOpen`` starts only after the robot has come to a complete stop, call
          ``WaitIdle()`` or wait for a checkpoint before issuing the gripper command.
        - To ensure the robot remains stationary until the ``GripperOpen`` completes, call
          ``WaitGripperMoveCompletion()``, ``WaitHoldingPart()``, or ``WaitReleasedPart()``
          before queuing further motion commands.
        """
        return super().GripperOpen()

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GripperClose(self):
        """
        Closes the gripper.

        For more information, see
        [GripperClose](https://www.mecademic.com/res/doc?robot_command=GripperClose&robot_model=MECA500).

        Notes
        -----
        ``GripperClose`` is queued in the robot's motion queue along with motion commands,
        but its execution is processed in parallel with robot motion:

        - It starts at the beginning of the blending (or deceleration) phase
          of the previous motion command in the queue.
        - It continues executing in parallel with any subsequent motion commands, which begin
          immediately while ``GripperClose`` starts executing.

        To coordinate with motion commands:

        - To ensure ``GripperClose`` starts only after the robot has come to a complete stop, call
          ``WaitIdle()`` or wait for a checkpoint before issuing the gripper command.
        - To ensure the robot remains stationary until the ``GripperClose`` completes, call
          ``WaitGripperMoveCompletion()``, ``WaitHoldingPart()``, or ``WaitReleasedPart()``
          before queuing further motion commands.
        """
        return super().GripperClose()

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def MoveGripper(self, target: Union[bool, float]):
        """
        Moves the gripper fingers to the specified opening, or fully opens or closes them.

        If the argument is a bool, it indicates if the fingers are to be opened (True,
        ``GRIPPER_OPEN``)  or closed (False, ``GRIPPER_CLOSE``) position. If the argument is float, it
        indicates the opening in mm.

        For more information, see
        [MoveGripper](https://www.mecademic.com/res/doc?robot_command=MoveGripper&robot_model=MECA500).

        Notes
        -----
        ``MoveGripper`` is queued in the robot's motion queue along with motion commands,
        but its execution is processed in parallel with robot motion:

        - It starts at the beginning of the blending (or deceleration) phase
          of the previous motion command in the queue.
        - It continues executing in parallel with any subsequent motion commands, which begin
          immediately while ``MoveGripper`` starts executing.

        To coordinate with motion commands:

        - To ensure ``MoveGripper`` starts only after the robot has come to a complete stop, call
          ``WaitIdle()`` or wait for a checkpoint before issuing the gripper command.
        - To ensure the robot remains stationary until the ``MoveGripper`` completes, call
          ``WaitGripperMoveCompletion()``, ``WaitHoldingPart()``, or ``WaitReleasedPart()``
          before queuing further motion commands.

        Parameters
        ----------
        target
            - If bool, open or close the gripper (``GRIPPER_OPEN`` or ``GRIPPER_CLOSE``).
            - If float, the desired gripper finger opening, in mm.
        """
        return super().MoveGripper(target)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetGripperForce(self, p: float):
        """
        Limits the grip force of the gripper.

        For more information, see
        [SetGripperForce](https://www.mecademic.com/res/doc?robot_command=SetGripperForce&robot_model=MECA500).

        Parameters
        ----------
        p
            Percentage of maximum grip force (~40 N).
            Default value restored at robot activation: ``MX_MQ_DEFAULT_GRIPPER_FORCE``.
        """
        self._send_motion_command('SetGripperForce', [p])

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetGripperVel(self, p: float):
        """
        Limits the velocity of the gripper fingers.

        For more information, see
        [SetGripperVel](https://www.mecademic.com/res/doc?robot_command=SetGripperVel&robot_model=MECA500).

        Parameters
        ----------
        p
            Percentage of maximum finger velocity.
            Default value restored at robot activation: ``MX_MQ_DEFAULT_GRIPPER_VEL``.
        """
        self._send_motion_command('SetGripperVel', [p])

    @mecascript_global_function_decorator
    @overload
    def SetGripperRange(self, gripper_range: List[float]):
        ...

    @mecascript_global_function_decorator
    @overload
    #pylint: disable=invalid-name
    def SetGripperRange(self, closePos: float, openPos: float):
        ...

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    #pylint: disable=invalid-name
    def SetGripperRange(self, closePos: Union[List[float], float], openPos: float = None):
        """
        Sets the closed and open states of the gripper and is used mainly to redefine the actions of
        the ``GripperClose`` and ``GripperOpen`` commands.

        Use this command to limit the gripper's opening travel when a full open is not required to
        release the part, reducing cycle time. The effect is especially noticeable on long-stroke
        grippers.

        For more information, see
        [SetGripperRange](https://www.mecademic.com/res/doc?robot_command=SetGripperRange&robot_model=MECA500).

        Parameters
        ----------
        closePos
            Fingers opening that should correspond to closed state, in mm.
        openPos
            Fingers opening that should correspond to open state, in mm.
        gripper_range
            Alternatively, both parameters can be provided together as a list ``[closePos, openPos]``.

        Notes
        -----
        Default values restored at robot activation: the gripper opening distance measured during
        gripper homing. Setting both values to 0 will reset the range to the default values.
        """
        self._send_motion_command('SetGripperRange', [closePos, openPos])

    @mecascript_global_function_decorator
    @overload
    def SetValveState(self, state_list: List[Union[int, str, MxCmdValveState]]):
        ...

    @mecascript_global_function_decorator
    @overload
    def SetValveState(self, *states: Union[int, str, MxCmdValveState]):
        ...

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetValveState(self, *args: Union[List, MxCmdValveState, int, str]):
        """
        Controls each of the two valves in the MPM500 pneumatic module independently

        For more information, see
        [SetValveState](https://www.mecademic.com/res/doc?robot_command=SetValveState&robot_model=MECA500).

        Notes
        -----
        Valve state commands are queued in the robot's motion queue along with motion commands,
        but their execution is processed in parallel with robot motion:

        - ``SetValveState`` starts at the beginning of the blending (or deceleration) phase
          of the previous motion command in the queue.

        To coordinate with motion commands:

        - To ensure the valve state changes only after the robot has come to a complete stop, call
          ``WaitIdle()`` or wait for a checkpoint before issuing the ``SetValveState`` command.

        Parameters
        ----------
        states
            Desired state for valve 1 and 2.

            - ``MxCmdValveState.MX_VALVE_STATE_STAY`` (or ``-1``, ``'*'``, ``'stay'``)
            - ``MxCmdValveState.MX_VALVE_STATE_CLOSE`` (or ``0``, ``'close'``)
            - ``MxCmdValveState.MX_VALVE_STATE_OPEN`` (or ``1``, ``'open'``)
        state_list
            Alternatively, you can provide a list of MxCmdValveState as a single argument

        Notes
        -----
        The arguments can be provided in several equivalent forms, as shown in the examples.

        Examples
        --------
        >>> robot.SetValveState(1, 0)
        >>> robot.SetValveState(['open', 'close'])
        >>> robot.SetValveState([MxCmdValveState.MX_VALVE_STATE_OPEN, MxCmdValveState.MX_VALVE_STATE_CLOSE])
        """
        self._send_motion_command('SetValveState', args)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def VacuumGrip(self):
        """
        Activates the suction in the MVK01 vacuum and I/O module.

        For more information, see
        [VacuumGrip](https://www.mecademic.com/res/doc?robot_command=VacuumGrip&robot_model=MCS500).

        Notes
        -----
        ``VacuumGrip`` is queued in the robot's motion queue along with motion commands,
        but its execution is processed in parallel with robot motion:

        - It starts at the beginning of the blending (or deceleration) phase
          of the previous motion command in the queue.
        - It continues executing in parallel with any subsequent motion commands, which begin
          immediately even while ``VacuumGrip`` is still in progress.

        To coordinate with motion commands:

        - To ensure that ``VacuumGrip`` is performed only after the robot has come to a complete stop, call
          ``WaitIdle()`` or wait for a checkpoint before issuing the command.
        - To ensure the robot remains stationary until the ``VacuumGrip`` completes, call
          ``WaitHoldingPart()`` before queuing further motion commands.
        """
        self._send_motion_command('VacuumGrip')

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def VacuumGrip_Immediate(self):
        """
        Activates suction in the MVK01 vacuum and I/O module immediately.
        Unlike ``VacuumGrip``, this command bypasses the robot's motion queue and can also be executed
        when the robot is deactivated.

        For more information, see
        [VacuumGrip_Immediate](https://www.mecademic.com/res/doc?robot_command=VacuumGrip_Immediate&robot_model=MCS500).
        """
        self.VacuumGripReleaseImmediate(False)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def VacuumRelease(self):
        """
        Deactivates the suction in the MVK01 vacuum and I/O module.

        For more information, see
        [VacuumRelease](https://www.mecademic.com/res/doc?robot_command=VacuumRelease&robot_model=MCS500).

        Notes
        -----
        ``VacuumRelease`` is queued in the robot's motion queue along with motion commands,
        but its execution is processed in parallel with robot motion:

        - It starts at the beginning of the blending (or deceleration) phase
          of the previous motion command in the queue.
        - It continues executing in parallel with any subsequent motion commands, which begin
          immediately even while ``VacuumRelease`` is still in progress.

        To coordinate with motion commands:

        - To ensure that ``VacuumRelease`` is performed only after the robot has come to a complete stop, call
          ``WaitIdle()`` or wait for a checkpoint before issuing the command.
        - To ensure the robot remains stationary until the ``VacuumRelease`` completes, call
          ``WaitReleasedPart()`` before queuing further motion commands.
        """
        self._send_motion_command('VacuumRelease')

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def VacuumRelease_Immediate(self):
        """
        Deactivates suction in the MVK01 vacuum and I/O module immediately.
        Unlike ``VacuumRelease``, this command bypasses the robot's motion queue and can also be
        executed when the robot is deactivated.

        #pylint: disable=line-too-long
        For more information, see
        [VacuumRelease_Immediate](https://www.mecademic.com/res/doc?robot_command=VacuumRelease_Immediate&robot_model=MCS500).
        """
        self.VacuumGripReleaseImmediate(True)

    @mecascript_global_function_decorator(is_mecascript_implementation=True)
    @disconnect_on_exception_decorator
    def VacuumPurge(self, duration: Optional[float] = None):  # pylint: disable=unused-argument
        """
        Forces an air purge of the MVK01 vacuum module to eject a part.
        Note that a purge is automatically performed after ``VacuumRelease``.

        For more information, see
        [VacuumPurge](https://www.mecademic.com/res/doc?robot_command=VacuumPurge&robot_model=MCS500).

        Parameters
        ----------
        duration
            Optional duration for the purge.

            - If provided, the duration is temporary and only applies to this ``VacuumPurge``;
              subsequent commands will still use the previously configured purge duration.
            - If not provided, the last set purge duration (by ``SetVacuumPurgeDuration``) will be applied.

        Notes
        -----
        ``VacuumPurge`` is queued in the robot's motion queue along with motion commands,
        but its execution is processed in parallel with robot motion:

        - It starts at the beginning of the blending (or deceleration) phase
          of the previous motion command in the queue.
        - It continues executing in parallel with any subsequent motion commands, which begin
          immediately even while ``VacuumPurge`` is still in progress.

        To coordinate with motion commands:

        - To ensure that ``VacuumPurge`` is performed only after the robot has come to a complete stop, call
          ``WaitIdle()`` or wait for a checkpoint before issuing the command.
        - To ensure the robot remains stationary until the ``VacuumPurge`` completes, call
          ``WaitReleasedPart()`` before queuing further motion commands.
        """
        # Note: This function is expected to be dynamically overridden by a MecaScript command and the code
        #       here should not be executed when calling robot.VacuumPurge
        # Note: Using a constant test here prevents children class to complain that this function is abstract in base
        if True:  # pylint: disable=using-constant-test
            raise NotImplementedError(f'VacuumPurge is not implemented on this robot')

    @mecascript_global_function_decorator(is_mecascript_implementation=True)
    @disconnect_on_exception_decorator
    def VacuumPurge_Immediate(self, duration: Optional[float] = None):  # pylint: disable=unused-argument
        """
        Similar to ``VacuumPurge``, except that this command bypasses the robot's motion queue and can also be
        executed when the robot is deactivated.

        #pylint: disable=line-too-long
        For more information, see
        [VacuumPurge_Immediate](https://www.mecademic.com/res/doc?robot_command=VacuumPurge_Immediate&robot_model=MCS500).
        """
        # Note: This function is expected to be dynamically overridden by a MecaScript command and the code
        #       here should not be executed when calling robot.VacuumPurge
        # Note: Using a constant test here prevents children class to complain that this function is abstract in base
        if True:  # pylint: disable=using-constant-test
            raise NotImplementedError(f'VacuumPurge_Immediate is not implemented on this robot')

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def WaitHoldingPart(self, timeout: Optional[float] = None):
        """
        Waits for the gripper (``GripperOpen``, ``GripperClose``, ``MoveGripper``) or the suction cup
        (``VacuumGrip``) to confirm that it is holding a part.

        Note that any motion command placed in the queue after a gripper or vacuum command will begin executing
        as soon as the gripper starts moving. To ensure the robot does not move until holding
        confirmation is received, call ``WaitHoldingPart`` before posting subsequent motion commands.

        Parameters
        ----------
        timeout
            Maximum duration to wait for holding part confirmation, in seconds.
        """
        # Use appropriate default timeout if not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_holding_part.wait(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def WaitReleasedPart(self, timeout: Optional[float] = None):
        """
        Waits for the gripper (``GripperOpen``, ``GripperClose``, ``MoveGripper``) or the suction cup
        (``VacuumRelease``) to confirm that the part has been released.

        Note that any motion command placed in the queue after a gripper or vacuum command will begin executing
        as soon as the gripper starts moving. To ensure the robot does not move until release
        confirmation is received, call ``WaitReleasedPart`` before posting subsequent motion commands.

        Parameters
        ----------
        timeout
            Maximum duration to wait for release part confirmation, in seconds.
        """
        super().WaitReleasedPart(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def WaitPurgeDone(self, timeout: Optional[float] = None, wait_purge_started=True):
        """
        Wait for the vacuum to purge after releasing part.

        Parameters
        ----------
        timeout
            Maximum duration to wait for vacuum purge confirmation, in seconds.
        wait_purge_started
            If True, this function will first wait until the purge has started, then wait for the purge to complete
        """
        super().WaitPurgeDone(timeout=timeout, wait_purge_started=wait_purge_started)

    @mecascript_global_function_decorator
    #pylint: disable=invalid-name
    @overload
    def SetVacuumThreshold(self, hold_threshold: float, release_threshold: float):
        ...

    @mecascript_global_function_decorator
    @overload
    def SetVacuumThreshold(self, payload: VacuumThreshold, /):  # pylint: disable=arguments-differ
        ...

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetVacuumThreshold(self, hold_threshold: Union[float, VacuumThreshold], release_threshold: float = None):
        """
        Sets the thresholds for the vacuum sensor (which measures only negative pressure) in the MVK01
        vacuum and I/O module that will be used for reporting whether a part is being held or not.

        For more information, see
        [SetVacuumThreshold](https://www.mecademic.com/res/doc?robot_command=SetVacuumThreshold&robot_model=MCS500).

        Parameters
        ----------
        hold_threshold
            When the negative pressure sensed is smaller than this argument, the robot reports that a
            part is being held. Specified in kPa.
            Default values restored at robot reboot: ``MX_ROBOT_CFG_DEFAULT_VACUUM_HOLD_THRESHOLD``.
        openPos
            When the negative pressure sensed is larger than this argument, the robot reports no part
            is being held. Specified in kPa. The value must be larger than the value of
            ``hold_threshold``.
            Default values restored at robot reboot: ``MX_ROBOT_CFG_DEFAULT_VACUUM_RELEASE_THRESHOLD``.
        payload
            Alternatively, you can provide VacuumThreshold as a single argument

        Notes
        -----
        To reset to default values, set both arguments to zero.
        """
        if isinstance(hold_threshold, VacuumThreshold):
            release_threshold = hold_threshold.release_threshold
            hold_threshold = hold_threshold.hold_threshold
        self._send_motion_command('SetVacuumThreshold', [hold_threshold, release_threshold])

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetVacuumThreshold_Immediate(self, hold_threshold: float, release_threshold: float):
        """
        Sets the thresholds for the vacuum sensor in the MVK01 vacuum and I/O module immediately.
        Unlike ``SetVacuumThreshold``, this command command bypasses the robot's motion queue.

        #pylint: disable=line-too-long
        For more information, see
        [SetVacuumThreshold_Immediate](https://www.mecademic.com/res/doc?robot_command=SetVacuumThreshold_Immediate&robot_model=MCS500).
        """
        self._send_immediate_command('SetVacuumThreshold_Immediate', [hold_threshold, release_threshold], None)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetVacuumPurgeDuration(self, duration: float):
        """
        Sets the duration of the air purge for ejecting a part when using the commands
        ``VacuumRelease*``.

        #pylint: disable=line-too-long
        For more information, see
        [SetVacuumPurgeDuration](https://www.mecademic.com/res/doc?robot_command=SetVacuumPurgeDuration&robot_model=MCS500).

        Parameters
        ----------
        duration
            Duration in seconds.
            Default value restored at robot reboot: ``MX_ROBOT_CFG_DEFAULT_VACUUM_PURGE_DURATION``.

        Notes
        -----
        To reset to the default values, set the argument to -1.
        """
        self._send_motion_command('SetVacuumPurgeDuration', [duration])

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetVacuumPurgeDuration_Immediate(self, duration: float = MX_ROBOT_CFG_DEFAULT_VACUUM_PURGE_DURATION):
        """
        Sets immediately the duration of the air purge for ejecting a part when using the commands
        ``VacuumRelease*``.
        Unlike ``SetVacuumPurgeDuration``, this command command bypasses the robot's motion queue.

        #pylint: disable=line-too-long
        For more information, see
        [SetVacuumPurgeDuration_Immediate](https://www.mecademic.com/res/doc?robot_command=SetVacuumPurgeDuration_Immediate&robot_model=MCS500).
        """
        self._send_immediate_command('SetVacuumPurgeDuration_Immediate', [duration], None)

    @mecascript_global_function_decorator
    @overload
    def SetOutputState(self, bank_id: MxIoBankId, output_states: List[Union[int, str, MxDigitalIoState]]):
        ...

    @mecascript_global_function_decorator
    @overload
    def SetOutputState(self, bank_id: MxIoBankId, *output_states: Union[int, str, MxDigitalIoState]):
        ...

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetOutputState(self, bank_id: MxIoBankId, *output_states: Union[List, MxDigitalIoState, int, str]):
        """
        Controls the digital outputs of the MVK01 vacuum and I/O module.

        This command does not cause the robot to stop moving. It is executed at the start of the
        blending (or deceleration) phase of the previous motion in the queue. To ensure the output
        states are changed only once the robot has fully stopped, insert a delay before issuing the
        command.

        For more information, see
        [SetOutputState](https://www.mecademic.com/res/doc?robot_command=SetOutputState&robot_model=MCS500).

        Parameters
        ----------
        bank_id
            The IO bank ID to set output states for.
        output_states
            The desired IO state for each digital output.
            The number of available outputs depends on the chosen bank id.
            Bank ``MX_IO_BANK_ID_IO_MODULE`` has 8 outputs.
            For each output, the provided value can be:

                - ``MxDigitalIoState.MX_DIGITAL_IO_STATE_STAY`` (alternatively -1, '*' or 'stay')
                - ``MxDigitalIoState.MX_DIGITAL_IO_STATE_0`` (alternatively 0 or 'off')
                - ``MxDigitalIoState.MX_DIGITAL_IO_STATE_1`` (alternatively 1 or 'on')

            If the number of provided values is smaller than the number of outputs for the selected
            bank, remaining outputs will not be changed ('stay').

        Notes
        -----
        The output states can be provided either as separate arguments or as a list, as shown in
        the example.

        Examples
        --------
        >>> robot.SetOutputState(MxIoBankId.MX_IO_BANK_ID_IO_MODULE, 1, 0, 1, 0)
        >>> robot.SetOutputState(MxIoBankId.MX_IO_BANK_ID_IO_MODULE, ['on', 'off', 'on', 'off'])
        >>> robot.SetOutputState(MxIoBankId.MX_IO_BANK_ID_IO_MODULE, [MxDigitalIoState.MX_DIGITAL_IO_STATE_1] * 8)
        """
        super()._set_output_state(bank_id, *output_states)

    @mecascript_global_function_decorator
    @overload
    def SetOutputState_Immediate(self, bank_id: MxIoBankId, output_states: List[Union[int, str, MxDigitalIoState]]):
        ...

    @mecascript_global_function_decorator
    @overload
    def SetOutputState_Immediate(self, bank_id: MxIoBankId, *output_states: Union[int, str, MxDigitalIoState]):
        ...

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetOutputState_Immediate(self, bank_id: MxIoBankId, *output_states: Union[MxDigitalIoState, int, str]):
        #pylint: disable=line-too-long
        """
        Sets immediately the digital outputs of the MVK01 vacuum and I/O module.
        Unlike ``SetOutputState``, this command bypasses the robot's motion queue.

        For more information, see
        [SetOutputState_Immediate](https://www.mecademic.com/res/doc?robot_command=SetOutputState_Immediate&robot_model=MCS500).
        """
        super()._set_output_state_immediate(bank_id, *output_states)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def WaitForAnyCheckpoint(self, timeout: float = None):
        """
        Pauses program execution until any checkpoint has been received from the robot.

        Parameters
        ----------
        timeout
            Maximum duration to wait for a checkpoint, in seconds.
        """
        super().WaitForAnyCheckpoint(timeout)

    @disconnect_on_exception_decorator
    def WaitConnected(self, timeout: float = None):
        """
        Pauses program execution until a robot is connected.

        Since the ``Connect`` command is always blocking, this command is only useful when a separate
        thread needs to wait for the connection to be established.

        Parameters
        ----------
        timeout
            Maximum duration to wait for the event, in seconds (Default is DEFAULT_WAIT_TIMEOUT).
        """
        super().WaitConnected(timeout)

    @disconnect_on_exception_decorator
    def WaitDisconnected(self, timeout: float = None):
        """
        Pauses program execution until a robot is disconnected.

        Since the ``Disconnect`` command is always blocking, this command is only useful when a
        separate thread needs to wait for the disconnection.

        Parameters
        ----------
        timeout
            Maximum duration to wait for the event, in seconds (Default is DEFAULT_WAIT_TIMEOUT).
        """
        super().WaitDisconnected(timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def WaitActivated(self, timeout: float = None):
        """
        Pauses program execution until a robot is activated.

        Parameters
        ----------
        timeout
            Maximum duration to wait for the event, in seconds (Default is 30 (Meca500) or 5 (MCS500)).
        """
        super().WaitActivated(timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def WaitDeactivated(self, timeout: float = None):
        """
        Pauses program execution until the robot is deactivated.

        Parameters
        ----------
        timeout
            Maximum duration to wait for the event, in seconds (Default is DEFAULT_WAIT_TIMEOUT).
        """
        super().WaitDeactivated(timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def WaitHomed(self, timeout: float = None):
        """
        Pauses program execution until the robot is homed.

        Parameters
        ----------
        timeout
            Maximum duration to wait for the event, in seconds (Default is 40).
        """
        super().WaitHomed(timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def WaitSimActivated(self, timeout: float = None):
        """
        Pauses program execution until the robot simulation mode is activated.

        Parameters
        ----------
        timeout
            Maximum duration to wait for the event, in seconds (Default is DEFAULT_WAIT_TIMEOUT).
        """
        super().WaitSimActivated(timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def WaitSimDeactivated(self, timeout: float = None):
        """
        Pauses program execution until the robot simulation mode is deactivated.

        Parameters
        ----------
        timeout
            Maximum duration to wait for the event, in seconds (Default is DEFAULT_WAIT_TIMEOUT).
        """
        super().WaitSimDeactivated(timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def WaitExtToolSimActivated(self, timeout: float = None):
        """
        Pause program execution until the robot external tool simulation mode is activated.

        Parameters
        ----------
        timeout
            Maximum duration to wait for the event, in seconds (Default is DEFAULT_WAIT_TIMEOUT).
        """
        super().WaitExtToolSimActivated(timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def WaitExtToolSimDeactivated(self, timeout: float = None):
        """
        Pauses program execution until the robot external tool simulation mode is deactivated.

        Parameters
        ----------
        timeout
            Maximum duration to wait for the event, in seconds (Default is DEFAULT_WAIT_TIMEOUT).
        """
        super().WaitExtToolSimDeactivated(timeout)

    # pylint: disable=unused-argument
    @disconnect_on_exception_decorator
    def WaitIoSimEnabled(self, bank_id: MxIoBankId = MxIoBankId.MX_IO_BANK_ID_IO_MODULE, timeout: float = None):
        """
        Pauses program execution until the simulation mode for the MVK01 vacuum and I/O module is
        enabled.

        Parameters
        ----------
        bank_id
            Id of the IO module to wait for.
        timeout
            Maximum duration to wait for the event, in seconds (Default is DEFAULT_WAIT_TIMEOUT).
        """
        super().WaitIoSimEnabled(timeout)

    # pylint: disable=unused-argument
    @disconnect_on_exception_decorator
    def WaitIoSimDisabled(self, bank_id: MxIoBankId = MxIoBankId.MX_IO_BANK_ID_IO_MODULE, timeout: float = None):
        """
        Pauses program execution until the simulation mode for the MVK01 vacuum and I/O module is
        disabled.

        Parameters
        ----------
        bank_id
            Id of the IO module to wait for.
        timeout
            Maximum duration to wait for the event, in seconds (Default is DEFAULT_WAIT_TIMEOUT).
        """
        super().WaitIoSimDisabled(timeout)

    def IsDesiredOutputState(self, bank_id: MxIoBankId, *expected_states: Union[MxDigitalIoState, int, str]) -> bool:
        """
        Returns True if the current digital output states match the expected states.

        A desired state of '*', 'stay', or -1 is treated as a wildcard and ignored in the comparison.

        Parameters
        ----------
        bank_id
            The IO bank ID to validate output states for.
        expected_states
            The desired IO state (the number of available outputs depends on the chosen bank id):

            - ``MxDigitalIoState.MX_DIGITAL_IO_STATE_STAY`` (alternatively -1, '*' or 'stay'),
                i.e. will not be compared
            - ``MxDigitalIoState.MX_DIGITAL_IO_STATE_0`` (alternatively 0 or 'off')
            - ``MxDigitalIoState.MX_DIGITAL_IO_STATE_1`` (alternatively 1 or 'on')

        Returns
        -------
        bool
            True if current output states match desired state.
        """
        return self.IsDesiredIoState(bank_id, False, *expected_states)

    @mecascript_global_function_decorator
    def WaitOutputState(self,
                        bank_id: MxIoBankId,
                        *expected_states: Union[MxDigitalIoState, int, str],
                        timeout: float = None):
        """
        Waits until the current digital output states match the desired states.
        A desired state of '*', 'stay', or -1 is treated as a wildcard and ignored in the comparison.

        Parameters
        ----------
        bank_id
            The IO bank ID to wait output states for.
        expected_states
            The desired IO state (the number of available outputs depends on the chosen bank id):

                - ``MxDigitalIoState.MX_DIGITAL_IO_STATE_STAY`` (alternatively -1, '*' or 'stay'),
                  i.e. will not be compared
                - ``MxDigitalIoState.MX_DIGITAL_IO_STATE_0`` (alternatively 0 or 'off')
                - ``MxDigitalIoState.MX_DIGITAL_IO_STATE_1`` (alternatively 1 or 'on')
        timeout
            Maximum duration to wait for the event, in seconds.

        Raises
        ------
        TimeoutException
            If timeout is reached before the digital outputs state is as desired.
        """
        return self.WaitIOState(bank_id, False, *expected_states, timeout)

    def IsDesiredInputState(self, bank_id: MxIoBankId, *expected_states: Union[MxDigitalIoState, int, str]) -> bool:
        """
        Returns True if the current digital input states match the desired states.
        A desired state of '*', 'stay', or -1 is treated as a wildcard and ignored in the comparison.

        Parameters
        ----------
        bank_id
            The IO bank ID to validate input states for.
        expected_states
            The desired IO state (the number of available inputs depends on the chosen bank id):

                - ``MxDigitalIoState.MX_DIGITAL_IO_STATE_STAY`` (alternatively -1, '*' or 'stay'),
                  i.e., will not be compared
                - ``MxDigitalIoState.MX_DIGITAL_IO_STATE_0`` (alternatively 0 or 'off')
                - ``MxDigitalIoState.MX_DIGITAL_IO_STATE_1`` (alternatively 1 or 'on')

        Returns
        -------
        bool
            True if current input states match desired state.
        """
        return self.IsDesiredIoState(bank_id, True, *expected_states)

    @mecascript_global_function_decorator
    def WaitInputState(self,
                       bank_id: MxIoBankId,
                       *expected_states: Union[MxDigitalIoState, int, str],
                       timeout: float = None):
        """
        Waits until the current digital input states match the desired states.
        A desired state of '*', 'stay', or -1 is treated as a wildcard and ignored in the comparison.

        Parameters
        ----------
        bank_id
            The IO bank ID to wait input states for.
        expected_states
            The desired IO state (the number of available inputs depends on the chosen bank id):

            - ``MxDigitalIoState.MX_DIGITAL_IO_STATE_STAY`` (alternatively -1, '*' or 'stay'),
              i.e., will not be compared
            - ``MxDigitalIoState.MX_DIGITAL_IO_STATE_0`` (alternatively 0 or 'off')
            - ``MxDigitalIoState.MX_DIGITAL_IO_STATE_1`` (alternatively 1 or 'on')
        timeout
            Maximum duration to wait for the event, in seconds.

        Raises
        ------
        TimeoutException
            If timeout is reached before the digital inputs state is as desired.
        """
        return self.WaitIOState(bank_id, True, *expected_states, timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def WaitRecoveryMode(self, activated: bool, timeout: float = None):
        """
        Pauses program execution until the robot recovery mode is in the requested state.

        Parameters
        ----------
        activated
            Recovery mode to wait for (activated or deactivated)
        timeout
            Maximum duration to wait for the event, in seconds (Default is DEFAULT_WAIT_TIMEOUT).
        """
        super().WaitRecoveryMode(activated, timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def WaitForError(self, timeout: float = None):
        """
        Pauses program execution until the robot is in error state.

        Parameters
        ----------
        timeout
            Maximum duration to wait for the event, in seconds.
        """
        super().WaitForError(timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def WaitErrorReset(self, timeout: float = None):
        """
        Pauses program execution until the robot is not in an error state.

        Parameters
        ----------
        timeout
            Maximum duration to wait for the event, in seconds (Default is DEFAULT_WAIT_TIMEOUT).
        """
        super().WaitErrorReset(timeout)

    @deprecation.deprecated(deprecated_in="2.1.0",
                            removed_in="3.0.0",
                            current_version=get_mecademicpy_version(),
                            details="Use the 'WaitSafetyStopReset' function instead")
    @disconnect_on_exception_decorator
    def WaitPStop2Reset(self, timeout: float = None):
        """Deprecated, use ``WaitSafetyStopReset`` instead."""
        super().WaitPStop2ResetDeprecated(timeout)

    @deprecation.deprecated(deprecated_in="2.1.0",
                            removed_in="3.0.0",
                            current_version=get_mecademicpy_version(),
                            details="Use the 'WaitSafetyStopReset' function instead")
    @disconnect_on_exception_decorator
    def WaitPStop2Resettable(self, timeout: float = None):
        """Deprecated, use ``WaitSafetyStopResettable`` instead."""
        super().WaitPStop2ResettableDeprecated(timeout)

    @deprecation.deprecated(deprecated_in="2.1.0",
                            removed_in="3.0.0",
                            current_version=get_mecademicpy_version(),
                            details="Use the 'WaitSafetyStopResettable' function instead")
    @disconnect_on_exception_decorator
    def WaitEStopReset(self, timeout: float = None):
        """Deprecated, use ``WaitSafetyStopReset`` instead."""
        super().WaitEStopResetDeprecated(timeout)

    @deprecation.deprecated(deprecated_in="2.1.0",
                            removed_in="3.0.0",
                            current_version=get_mecademicpy_version(),
                            details="Use the 'WaitSafetyStopResettable' function instead")
    @disconnect_on_exception_decorator
    def WaitEStopResettable(self, timeout: float = None):
        """Deprecated, use ``WaitSafetyStopResettable`` instead."""
        super().WaitEStopResettableDeprecated(timeout)

    def WaitEstopResettable(self, timeout: float = None):
        super().WaitEStopResettableDeprecated(timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def WaitSafetyStopReset(self, timeout: float = None):
        """
        Pauses program execution until all safety stop conditions have been reset
        (EStop, PStop1, PStop2, ...)

        Parameters
        ----------
        timeout
            Maximum duration to wait for the event, in seconds (Default is DEFAULT_WAIT_TIMEOUT).
        """
        super().WaitSafetyStopReset(timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def WaitSafetyStopResettable(self, timeout: float = None):
        """
        Pauses program execution until all safety conditions can be reset using the Reset function.

        Parameters
        ----------
        timeout
            Maximum duration to wait for the event, in seconds (Default is DEFAULT_WAIT_TIMEOUT).
        """
        super().WaitSafetyStopResettable(timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def WaitSafetyStopStateChange(self, timeout: float = None):
        """
        Pauses program execution until any safety stop state changes (Raised, resettable or cleared
        safety stop, operation mode change, etc) as reported by ``RobotSafetyStatus``.

        Parameters
        ----------
        timeout
            Maximum duration to wait for the event, in seconds (Default is DEFAULT_WAIT_TIMEOUT).
        """
        super().WaitSafetyStopStateChange(timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def WaitMotionResumed(self, timeout: float = None):
        """
        Pauses program execution until the robot motion is resumed.

        Parameters
        ----------
        timeout
            Maximum duration to wait for the event, in seconds (Default is DEFAULT_WAIT_TIMEOUT).
        """
        super().WaitMotionResumed(timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def WaitMotionPaused(self, timeout: float = None):
        """
        Pauses program execution until the robot motion is paused.

        Parameters
        ----------
        timeout
            Maximum duration to wait for the event, in seconds (Default is DEFAULT_WAIT_TIMEOUT).
        """
        super().WaitMotionPaused(timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def WaitMotionCleared(self, timeout: float = None):
        """
        Pauses program execution until all pending requests to clear motion have been acknowledged.

        Parameters
        ----------
        timeout
            Maximum duration to wait for the event, in seconds (Default is DEFAULT_WAIT_TIMEOUT).
        """
        super().WaitMotionCleared(timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def WaitEndOfCycle(self, timeout: float = None, *, force_cycle: bool = False):
        """
        Pauses program execution until all messages in a message cycle are received.

        Parameters
        ----------
        timeout
            Maximum duration to wait for the event, in seconds.
        force_cycle
            - If True, forces the robot to send an update of real-time monitoring data immediately.
            - If False, waits for the next cycle, as defined with ``SetMonitoringInterval``.
        """
        super().WaitEndOfCycle(timeout=timeout, force_cycle=force_cycle)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def WaitIdle(self, timeout: float = None, wait_rt_data: bool = False):
        """
        Pauses program execution until the robot becomes idle.

        Notes
        -----
        When this function returns, the robot is idle but some real-time values (e.g.,
        ``GetRtTargetJointPos``, ``GetRtTargetCartPos``, ``GetRobotRtData``) may still be outdated,
        as they are updated only at the monitoring interval defined by ``SetMonitoringInterval``.
        To obtain the latest real-time position immediately after ``WaitIdle``, use the
        `synchronous_update` option in those functions. Alternatively, call ``WaitEndOfCycle``
        after ``WaitIdle`` to ensure the most recent real-time data is received.

        Parameters
        ----------
        timeout
            Maximum duration to wait for the event, in seconds.
        wait_rt_data
            Also waits until the next cyclic real-time data is received after the robot becomes idle
            (end of block). This ensures that, when ``WaitIdle`` exits, the real-time data (e.g.,
            position, velocity, which should be 0) is up to date.

        Raises
        ------
        TimeoutException
            If timeout is reached before the robot is idle.
        InterruptException
            If the robot returns an error or got disconnected (exception message will describe the
            error).
        """
        super().WaitIdle(timeout, wait_rt_data)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def ResetError(self):
        """
        Attempts to reset robot error.

        For more information, see
        [ResetError](https://www.mecademic.com/res/doc?robot_command=ResetError).
        """
        super().ResetError()

    @deprecation.deprecated(deprecated_in="1.2.2",
                            removed_in="3.0.0",
                            current_version=get_mecademicpy_version(),
                            details="Use the 'ResetPStop2' function instead")
    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def ResetPStop(self, timeout: float = None):
        """
        Attempts to reset the robot's PStop2.

        Deprecated for robots running firmware 10.1 and above: use ``ResumeMotion`` instead.

        Notes
        -----
        PStop2 is not safety-rated on Meca500 robots
        """
        self.ResetPStop2Deprecated(timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def ResetPStop2(self, timeout: float = None):
        """
        Attempt to reset the robot's PStop2.
        Deprecated for robots running firmware 10.1 and above: use ``ResumeMotion`` instead.

        Notes
        -----
        PStop2 is not safety-rated on Meca500 robots.
        """
        super().ResetPStop2Deprecated(timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def Delay(self, t: float):
        """
        Sets a delay between motion commands.

        For more information, see
        [Delay](https://www.mecademic.com/res/doc?robot_command=Delay).

        Parameters
        ----------
        t
            Desired pause duration in seconds.
        """
        super().Delay(t)

    def sleep(self, duration: float):
        """
        Pauses execution of this application (not the robot).
        Similar to Python's ``time.sleep``, but can be interrupted if the robot is no longer controllable
        (for example, when disconnected).

        Parameters
        ---------
        timeout
            Sleep duration, in seconds.

        Raises
        ------
        InterruptException
            The robot became unavailable before the timeout
        """
        super().sleep(duration)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SendCustomCommand(self,
                          command: str,
                          expected_responses: List[int] = None,
                          timeout: float = None) -> Optional[InterruptableEvent | Message]:
        """
        Sends a custom command to the robot, i.e., a command that the robot may support but for which
        this Python API does not provide an explicit function.

        Parameters
        ----------
        command
            Custom command to send to the robot.
        expected_responses
            If provided, waits for and returns one of the expected responses.
        timeout
            Timeout in seconds for waiting for the first of the expected responses.
            If no value is provided, the function is non-blocking and returns an
            ``InterruptableEvent`` that can be used to wait until one of the expected responses is
            received. If a value is provided, the function waits on the interruptable event for the
            specified duration and returns the ``Message`` received among the expected responses, or
            raises a ``TimeoutException``  if no response is received in time.

        Raises
        ------
        TimeoutException
            If the timeout is reached before the robot returns a success or failure response.
        InterruptException
            If the robot returns an error (exception message will describe the error).

        Returns
        -------
        None
            If expected_responses is None
        InterruptableEvent
            If `expected_responses` is provided and `timeout` is not set.
        Message
            If both `expected_responses` and `timeout` are provided.
        """
        command, args = self._split_command_args_str(command)
        return self._send_custom_command(command, args=args, expected_responses=expected_responses, timeout=timeout)

    def GetInterruptableEvent(self,
                              codes: List[Union[MxRobotStatusCode, Message]],
                              abort_on_error=False,
                              abort_on_clear_motion=False) -> InterruptableEvent:
        """
        Returns an interruptable event that can be used to wait for the next code received from the
        robot among the specified list.

        Parameters
        ----------
        codes
            - If a list of status codes, the first received code will trigger the interruptable
              event.
            - If a list of messages (id, data), the first received message matching both id and data
              will trigger the event.
        abort_on_error
            If True, the event is awakened when the robot enters an error state.
        abort_on_clear_motion
            If True, the event is awakened when the robot's motion queue is cleared.
            This also applies when a PStop2 condition occurs or when the robot is deactivated,
            since both actions clear the motion queue.

        Returns
        -------
        InterruptableEvent
            Event awakened when the first matching code from the provided list is received.

        Examples
        --------
        **Example 1: Wait for an event id**

        >>> # Create an interruptable event that will trigger on code MX_ST_RT_INPUT_STATE
        >>> input_state_changed_event = robot.GetInterruptableEvent([mdr.MxRobotStatusCode.MX_ST_RT_INPUT_STATE])
        >>> # Perform some actions that may trigger a digital input change
        >>> # (...)
        >>> # Wait (block) until the event is received, with a 10-second timeout
        >>> input_state_changed_event.wait(10)

        **Example 2: Wait for an event id with specific data**

        >>> # Create an interruptable event that will trigger when the torque limit is exceeded,
        >>> # i.e., event ID MX_ST_TORQUE_LIMIT_STATUS with data '1'
        >>> torque_exceeded_event = robot.GetInterruptableEvent(
        ...     [mdr.Message(mdr.MxRobotStatusCode.MX_ST_TORQUE_LIMIT_STATUS, '1')])
        >>> # Perform actions that may cause the torque limit to be exceeded
        >>> # (...)
        >>> # Check if the torque limit exceeded event was already received
        >>> if torque_exceeded_event.is_set():
        ...     # Torque limit was exceeded between the call to GetInterruptableEvent and now
        ...     pass
        >>> # Otherwise, wait (block) until the torque limit exceeded event is received
        >>> torque_exceeded_event.wait(10)
        """
        return super().GetInterruptableEvent(codes, abort_on_error, abort_on_clear_motion)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def StartProgram(self, name: int | str, timeout: float = None):
        """
        Starts a program.

        Programs need to be recorded using the MecaPortal (or via the text API).
        This function can only start an already recorded program.

        For more information, see
        [StartProgram](https://www.mecademic.com/res/doc?robot_command=StartProgram).

        Parameters
        ----------
        name
            Name of the program to start (optionally, an integer for legacy programs which name is a number).
        timeout
            Timeout in seconds for the robot to confirm that program execution has started.
        """
        super().StartProgram(name, timeout)

    @deprecation.deprecated(deprecated_in="2.3.0",
                            removed_in="3.0.0",
                            current_version=get_mecademicpy_version(),
                            details="Use the 'StartProgram' function instead")
    @disconnect_on_exception_decorator
    def StartOfflineProgram(self, n: int | str, timeout: float = None):
        """Deprecated, use StartPrograms instead.
        """
        super().StartProgram(n, timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def StopProgram(self):
        """
        Stops the currently running program.

        This command does not apply to legacy (.mxprog) programs, whose execution is instantaneous
        and cannot be stopped.

        When ``StopProgram`` is called, the running Python MecaScript program is notified.
        The program may choose to continue running if needed, so the StopProgram may not cause immediate end of the
        running program.
        To forcefully stop any running program, use ``ClearMotion``.

        This command is asynchronous. Call ``WaitProgramDone`` to ensure the program has fully stopped.

        For more information, see
        [StopProgram](https://www.mecademic.com/res/doc?robot_command=StopProgram).
        """
        super().StopProgram()

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def ListFiles(self, timeout: float = None, as_dict: bool = False) -> Union[RobotFileList, dict]:
        """
        Lists all files stored on the robot.

        Related commands: ``LoadFile``, ``SaveFile``, ``DeleteFile``

        This function is currently synchronous only.

        Parameters
        ----------
        timeout
            Maximum duration to wait for the file list, in seconds (Default is FILE_MANAGEMENT_OP_DEFAULT_TIMEOUT).
        as_dict
            - If true, returns the list of files as a dictionary.
              This is deprecated, use this option only if you used ListFile in previous version of this Robot class
            - If false, returns the RobotFileList object (it's highly recommended to use this option)

        Raises
        ------
        TimeoutException
            If the timeout is reached before the robot returns a success or failure response.
        InterruptException
            If the robot returns an error (exception message will describe the error).

        Returns
        -------
        RobotFileList
            All files listed on the robot with their status (but not their contents)
        """
        return super().ListFiles(timeout, as_dict)

    @deprecation.deprecated(deprecated_in="2.1.0",
                            removed_in="3.0.0",
                            current_version=get_mecademicpy_version(),
                            details="Use the 'ListFiles' function instead")
    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def ListPrograms(self, timeout: float = None) -> dict:
        """Deprecated, use ``ListFiles`` instead.
        """
        return super().ListFiles(timeout, as_dict=True)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def LoadFile(self, name: str, timeout: float = None, as_dict: bool = False) -> Union[RobotFile, dict]:
        """
        Loads a file from the robot.

        Related commands: ``ListFiles``, ``SaveFile``, ``DeleteFile``

        This function is currently synchronous only.

        Parameters
        ----------
        name
            Name of the file to load.
        timeout
            Maximum duration to wait for the file list, in seconds (Default is FILE_MANAGEMENT_OP_DEFAULT_TIMEOUT).
        as_dict
            - If true, returns the list of files as a dictionary.
              This is deprecated, use this option only if you used ListFile in previous version of this Robot class
            - If false, returns the RobotFileList object (it's highly recommended to use this option)

        Raises
        ------
        InvalidStateError
            If the file could not be loaded (does not exist on the robot)
        TimeoutException
            If the timeout is reached before the robot returns a success or failure response.
        InterruptException
            If the robot returns an error (exception message will describe the error).

        Returns
        -------
        RobotFile
            The loaded file (name, content, status, ...)
        """
        return super().LoadFile(name, timeout, as_dict)

    @deprecation.deprecated(deprecated_in="2.1.0",
                            removed_in="3.0.0",
                            current_version=get_mecademicpy_version(),
                            details="Use the 'LoadFile' function instead")
    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def LoadProgram(self, name: str, timeout: float = None) -> dict:
        """Deprecated, use ``LoadFile`` instead.
        """
        return super().LoadFile(name, timeout, as_dict=True)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SaveFile(self,
                 name: str,
                 content: str,
                 timeout: float = None,
                 allow_invalid: bool = False,
                 overwrite: bool = False):
        """
        Saves a file (robot program or other file type) to the robot.

        Related commands: ``ListFiles``, ``LoadFile``, ``DeleteFile``

        This function is currently synchronous only.

        Parameters
        ----------
        name
            Name of the file to save. The file extension determines the file type,
            including robot programs with ``.mxprog``, ``.mxpy``, or ``.py`` extensions.
        content
            The file content to save.
        timeout
            Maximum duration to wait for the file list, in seconds (Default is FILE_MANAGEMENT_OP_DEFAULT_TIMEOUT).
        allow_invalid
            Whether to allow saving the file even if its content is invalid.
        overwrite
            Whether to overwrite the file if one with the same name already exists on the robot.
            Equivalent to an atomic ``DeleteFile`` followed by ``SaveFile``. If False, the
            save operation is refused when the name is already in use.

        Notes
        -----
        - Starting with firmware version 11, the robot supports saving any file type.
          The file extension must always be explicitly provided, including ``.mxprog``
          for basic robot programs.
        - With firmware version 10 and earlier, only basic programs could be saved,
          and no file extension was expected. All files were saved internally with
          the ``.mxprog`` extension, but this was hidden from the file management
          API (e.g., ``SaveFile``, ``ListFiles``).
        - This operation is persistent and retains its value even after power cycling the robot.

        Raises
        ------
        TimeoutException
            If the timeout is reached before the robot returns a success or failure response.
        InterruptException
            If the robot returns an error (exception message will describe the error).

        """
        return super().SaveFile(name, content, timeout, allow_invalid, overwrite)

    @deprecation.deprecated(deprecated_in="2.1.0",
                            removed_in="3.0.0",
                            current_version=get_mecademicpy_version(),
                            details="Use the 'SaveFile' function instead")
    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SaveProgram(self, name: str, program: str, timeout: float = None, allow_invalid=False, overwrite=False):
        """
        Deprecated. Use ``SaveFile`` instead.

        Notes
        -----
        For backward compatibility, ``SaveProgram`` behaves as follows:

        - On firmware version 11 and newer:
                Automatically appends the now-required ``.mxprog`` extension if the file name does
                not include one, preserving the behavior of older versions.
        - On firmware version 10 and older:
                Automatically removes the ``.mxprog`` extension from the file name,
                since only this file type was supported and the extension was hidden.
        """
        legacy_extension = "." + MX_PROG_FILE_EXTENSION
        if self._robot_info.version.is_at_least(major=11):
            # Automatically add .mxprog if no file extension is provided
            if '.' not in name:
                name += legacy_extension
        else:
            # Automatically remove .mxprog if present in the file name
            if name.endswith(legacy_extension):
                name = name[:-len(legacy_extension)]
        return super().SaveFile(name, program, timeout, allow_invalid, overwrite)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def DeleteFile(self, name: str, timeout: float = None):
        """
        Deletes a file from the robot.

        Related commands: ``ListFiles``, ``LoadFile``, ``SaveFile``

        This function is currently synchronous only.

        Parameters
        ----------
        name
            Name of the file to delete.
        timeout
            Maximum duration to wait for the file list, in seconds (Default is FILE_MANAGEMENT_OP_DEFAULT_TIMEOUT).

        Notes
        -----
        This operation is persistent and retains its value even after power cycling the robot.

        Raises
        ------
        TimeoutException
            If the timeout is reached before the robot returns a success or failure response.
        InterruptException
            If the robot returns an error (exception message will describe the error).
        """
        return super().DeleteFile(name, timeout)

    @deprecation.deprecated(deprecated_in="2.1.0",
                            removed_in="3.0.0",
                            current_version=get_mecademicpy_version(),
                            details="Use the 'DeleteFile' function instead")
    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def DeleteProgram(self, name: str, timeout: float = None):
        """Deprecated, use ``DeleteFile`` instead.
        """
        return super().DeleteFile(name, timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetLoadedPrograms(self,
                          *,
                          files_to_load: Optional[List[str]] = None,
                          files_to_unload: Optional[List[str]] = None,
                          synchronous_update: bool = True,
                          timeout: Optional[float] = None):
        """
        Marks selected programs as "to load" or "to unload" among those already
        saved on the robot with ``SaveFile``.

        Only the specified files are affected; the "loaded" status of other files remains unchanged.

        Parameters
        ----------
        files_to_load
            Program paths to mark as loaded.
        files_to_unload
            Program paths to mark as unloaded.
        synchronous_update
            Whether to wait until the specified programs are loaded or unloaded.

                - If True, raises ``CommandFailedException`` if the robot reports a failure
                  while loading a file.
                - If False, returns immediately without waiting for
                  confirmation. Use ``WaitProgramsLoaded`` to wait later if needed.
        timeout
            Maximum time in seconds to wait for the configuration to be saved and the
            programs to be loaded/unloaded (Default is FILE_MANAGEMENT_OP_DEFAULT_TIMEOUT).
            Ignored if `synchronous_update = False`.


        This setting is persistent and retains its values even after power cycling the robot.

        Raises
        ------
        TimeoutException
            Raised only when `synchronous_update = True` if the specified timeout is
            reached before the programs are loaded/unloaded.
        CommandFailedException
            Raised if the robot reports a failure while loading one of the files.
        """
        super().SetLoadedPrograms(files_to_load=files_to_load,
                                  files_to_unload=files_to_unload,
                                  synchronous_update=synchronous_update,
                                  timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def WaitProgramsLoaded(self,
                           *,
                           files_to_load: Optional[List[str]] = None,
                           files_to_unload: Optional[List[str]] = None,
                           timeout: Optional[float] = None):
        """
        Waits until the specified programs have finished loading or unloading in the MecaScript engine.

        Parameters
        ----------
        files_to_load
            Program paths to wait until loaded.
        files_to_unload
            Program paths to wait until unloaded.
        timeout
            Maximum time in seconds to wait for the configuration to be saved and the programs to be
            loaded/unloaded (Default is FILE_MANAGEMENT_OP_DEFAULT_TIMEOUT).

        Raises
        ------
        TimeoutException
            Raised if the specified timeout is reached before the programs are loaded/unloaded.
        CommandFailedException
            Raised if the robot reports a failure while loading one of the files.
        """
        return super().WaitProgramsLoaded(files_to_load=files_to_load, files_to_unload=files_to_unload, timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def WaitMecaScriptEngineReady(self, *, expected_ready: bool = True, timeout: Optional[float] = None):
        """
        Waits until the MecaScript Python engine is running (or stopped) on the robot.

        Parameters
        ----------
        expected_ready
            - True to wait until the MecaScript Python engine is running/ready
            - False to wait until the MecaScript Python engine is stopped
        timeout
            Maximum time in seconds to wait for the MecaScript Python engine to be ready (or stopped)
            (Default is FILE_MANAGEMENT_OP_DEFAULT_TIMEOUT).

        Raises
        ------
        TimeoutException
            Raised if the specified timeout is reached before the expected state.
        """
        return super().WaitMecaScriptEngineReady(expected_ready=expected_ready, timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetNetworkCfg(self, synchronous_update: bool = True, timeout: float = None) -> NetworkConfig:
        """
        Returns the robot's current network configuration.

        Parameters
        ----------
        synchronous_update
            This option is ignored for now, GetNetworkCfg is always synchronous (awaiting robot response)
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot
            (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        NetworkConfig
            Object containing the current network configuration.
        """

        return super().GetNetworkCfg(synchronous_update, timeout)

    @deprecation.deprecated(deprecated_in="2.4.0",
                            removed_in="3.0.0",
                            current_version=get_mecademicpy_version(),
                            details="Use the 'GetNetworkCfg' function instead")
    @disconnect_on_exception_decorator
    def GetNetworkConfig(self, synchronous_update: bool = True, timeout: float = None) -> NetworkConfig:
        """Deprecated, use ``GetNetworkCfg`` instead."""
        return super().GetNetworkCfg(synchronous_update, timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetNetworkOptions(self, n: Union[int, NetworkOptions]):
        """
        This command is used to set persistent parameters affecting the network connection.

        For more information, see
        [SetNetworkOptions](https://www.mecademic.com/res/doc?robot_command=SetNetworkOptions).

        Parameters
        ----------
        n
            TCP keep-alive connection timeout, in seconds. See class ``NetworkOptions`` for details.

        Notes
        -----
        -  Note: It is not recommended to change the default TCP keep-alive timeout.
           The most reliable way to detect a connection loss with the robot is to use the connection watchdog
           (``ConnectionWatchdog`` / ``AutoConnectionWatchdog``).

        -  This setting is persistent and retains its value even after power cycling the robot.
           After a factory calibration, this setting is enabled.
        """
        if isinstance(n, NetworkOptions):
            n = n.keep_alive_timeout
        super()._send_custom_command('SetNetworkOptions', args=[n])

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetNetworkOptions(self, *, timeout: float = None) -> NetworkOptions:
        """
        Returns the robot's network options, as set with ``SetNetworkOptions``.

        For more information, see
        [GetNetworkOptions](https://www.mecademic.com/res/doc?robot_command=GetNetworkOptions).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        NetworkOptions
            Robot's current network options.
        """
        return super().GetNetworkOptions(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRobotRtData(self, synchronous_update: bool = False, timeout: float = None) -> RobotRtData:
        """
        Returns a snapshot of the current robot real-time data, with all fields captured at the same
        timestamp.

        This command returns an atomic copy of all real-time values currently available on the robot.
        Each field in the returned object includes a timestamp and an ``TimestampedData.enabled`` flag.

        Notes
        -----
        See also the ``RobotRtData`` class documentation for full details.

        - For faster access to specific values, use individual calls such as ``GetRtTargetJointPos``.
          ``GetRobotRtData`` has more overhead but ensures consistency across all fields.
        - Some real-time fields are optional and not enabled by default. Use ``SetRealTimeMonitoring``
          to enable them. Refer to the ``TimestampedData.enabled`` flag of each field in the
          returned ``RobotRtData``.
        - Some fields are updated cyclically, based on the interval set via``SetMonitoringInterval``.
          Others are updated immediately by the robot when a change occurs (event-based). Refer to the
          ``update_type`` field of each ``TimestampedData`` to determine how it is updated.
        - Use `synchronous_update = True` to request a fresh cyclic update before reading the data.
          This affects only enabled cyclic fields.

        Parameters
        ----------
        synchronous_update
            - If True, requests the robot to send updated cyclic data immediately before returning.
              This ensures the returned data is fresh, but does not affect event-based or disabled
              fields.
              Use ``SetRealTimeMonitoring`` to control which fields are enabled.
            - If False, returns the latest known data without waiting. This data may be outdated
              by up to one cycle, as defined by ``SetMonitoringInterval``.
        timeout
            Timeout in seconds when waiting for a new cyclic update from the robot (Default is DEFAULT_WAIT_TIMEOUT).
            Applies only if `synchronous_update=True`.

        Returns
        -------
        RobotRtData
            Object containing real-time robot data. All values share the same timestamp.
        """
        return super().GetRobotRtData(synchronous_update, timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRtAccelerometer(self,
                           n: int,
                           *,
                           include_timestamp: bool = False,
                           synchronous_update: bool = True,
                           timeout: float = None) -> Union[TimestampedData, List[float]]:
        """
        Returns the acceleration of the specified robot link with respect to the WRF.
        This value corresponds to the gravity vector when the robot is stationary.

        For more information, see
        [GetRtAccelerometer](https://www.mecademic.com/res/doc?robot_command=GetRtAccelerometer&robot_model=MECA500).

        Notes
        -----
        This is optional real-time data (``update_type = MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``).
        It is available only if either:

        - `synchronous_update` is used, or
        - ``SetRealTimeMonitoring`` is called to include it in the robot's cyclic data.

        Parameters
        ----------
        n
            Link number for which to get the accelerometer data. Currently only supported for
            Meca500's link 5.
        include_timestamp
            - If True, returns a ``TimestampedData`` object.
            - If False, returns a list of floats.
        synchronous_update
            - If True, retrieves the data synchronously.
            - If False, returns the latest known cyclic data without waiting. This data may be
              outdated by up to one cycle, as defined by ``SetMonitoringInterval``.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        TimestampedData or list of float
            In addition to the optional timestamp, the returned data is
            [a_x, a_y, a_z], in units such that 16,000 is equivalent to 9.81 m/s² (i.e., 1g).
        """
        return super().GetRtAccelerometer(n,
                                          include_timestamp=include_timestamp,
                                          synchronous_update=synchronous_update,
                                          timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRtExtToolStatus(self,
                           include_timestamp: bool = False,
                           synchronous_update: bool = False,
                           timeout: float = None) -> Union[TimestampedData, ExtToolStatus]:
        """
        Returns the general status of the external tool connected to the tool I/O port of the Meca500.

        For more information, see
        [GetRtExtToolStatus](https://www.mecademic.com/res/doc?robot_command=GetRtExtToolStatus&robot_model=MECA500).

        Parameters
        ----------
        include_timestamp
            - If True, returns a ``TimestampedData`` object.
            - If False, returns an ``ExtToolStatus`` object.
        synchronous_update
            - If True, retrieves the data synchronously.
            - If False, returns the latest known status without waiting. This data may be
              outdated by up to one cycle, as defined by ``SetMonitoringInterval``.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        TimestampedData or ExtToolStatus
        """
        return super().GetRtExtToolStatus(include_timestamp, synchronous_update, timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRtIoStatus(self,
                      bank_id: MxIoBankId = MxIoBankId.MX_IO_BANK_ID_IO_MODULE,
                      include_timestamp: bool = False,
                      synchronous_update: bool = False,
                      timeout: float = None) -> Union[TimestampedData, IoStatus]:
        """
        Returns the general status of the specified IO bank, such as MVK01.

        For more information, see
        [GetRtIoStatus](https://www.mecademic.com/res/doc?robot_command=GetRtIoStatus&robot_model=MCS500).

        Parameters
        ----------
        include_timestamp
            - If True, returns a ``TimestampedData`` object.
            - If False, returns an ``IoStatus`` object.
        synchronous_update
            - If True, retrieves the external tool status synchronously.
            - If False, returns the latest known status without waiting. This data may be outdated by
              up to one cycle, as defined by ``SetMonitoringInterval``.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot.

        Returns
        -------
        TimestampedData or IoStatus
        """
        return super().GetRtIoStatus(bank_id, include_timestamp, synchronous_update, timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRtGripperForce(self,
                          *,
                          include_timestamp: bool = False,
                          synchronous_update: bool = True,
                          timeout: Optional[float] = None) -> Union[TimestampedData, float]:
        """
        Returns the currently applied grip force of the MEGP 25* gripper connected to the tool I/O port
        of the Meca500.

        For more information, see
        [GetRtGripperForce](https://www.mecademic.com/res/doc?robot_command=GetRtGripperForce&robot_model=MECA500).

        Notes
        -----
        This is optional real-time data (``update_type = MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``).
        It is available only if either:

        - `synchronous_update` is used, or
        - ``SetRealTimeMonitoring`` is called to include it in the robot's cyclic data.

        Parameters
        ----------
        include_timestamp
            - If True, returns a ``TimestampedData`` object.
            - If False, returns a float value.
        synchronous_update
            - If true, retrieves the data synchronously.
            - If false, returns latest known data without waiting. This data may be
              outdated by up to one cycle as defined by ``SetMonitoringInterval``.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        TimestampedData or float
            In addition to the optional timestamp, the returned data is a float representing the
            gripper force, as signed percentage of the maximum grip force.
        """
        return super().GetRtGripperForce(include_timestamp=include_timestamp,
                                         synchronous_update=synchronous_update,
                                         timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRtGripperPos(self,
                        *,
                        include_timestamp: bool = False,
                        synchronous_update: bool = True,
                        timeout: Optional[float] = None) -> Union[TimestampedData, float]:
        """
        Returns the current fingers opening of the MEGP 25* gripper connected to the tool I/O port
        of the Meca500.

        For more information, see
        [GetRtGripperPos](https://www.mecademic.com/res/doc?robot_command=GetRtGripperPos&robot_model=MECA500).

        Notes
        -----
        This is optional real-time data (``update_type = MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``).
        It is available only if either:

        - `synchronous_update` is used, or
        - ``SetRealTimeMonitoring`` is called to include it in the robot's cyclic data.

        Parameters
        ----------
        include_timestamp
            - If True, returns a ``TimestampedData`` object.
            - If False, returns a float value.
        synchronous_update
            - If True, retrieves the data synchronously.
            - If False, returns latest known data without waiting. This data may be outdated
              by up to one cycle, as defined by ``SetMonitoringInterval``.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Return
        ------
        TimestampedData or float
            In addition to the optional timestamp, the returned data is a float representing the
            gripper fingers opening, in mm.
        """
        return super().GetRtGripperPos(include_timestamp=include_timestamp,
                                       synchronous_update=synchronous_update,
                                       timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRtGripperState(self,
                          include_timestamp: bool = False,
                          synchronous_update: bool = False,
                          timeout: float = None) -> Union[TimestampedData, GripperState]:
        """
        Returns the current state of the MEGP 25* gripper connected to the tool I/O port of the Meca500.

        For more information, see
        [GetRtGripperState](https://www.mecademic.com/res/doc?robot_command=GetRtGripperState&robot_model=MECA500).

        Parameters
        ----------
        include_timestamp
            - If True, returns a ``TimestampedData`` object.
            - If False, returns a ``GripperState`` object.
        synchronous_update
            - If True, retrieves the data synchronously.
            - If False, return latest known data without waiting. This data may be
              outdated by up to one cycle, as defined by ``SetMonitoringInterval``.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        TimestampedData or GripperState
        """
        return super().GetRtGripperState(include_timestamp, synchronous_update, timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRtValveState(self,
                        include_timestamp: bool = False,
                        synchronous_update: bool = False,
                        timeout: float = None) -> Union[TimestampedData, ValveState]:
        """
        Returns the current state of the MPM500 pneumatic module connected to the tool I/O port
        of the Meca500.

        For more information, see
        [GetRtValveState](https://www.mecademic.com/res/doc?robot_command=GetRtValveState&robot_model=MECA500).

        Parameters
        ----------
        include_timestamp
            - If True, returns a ``TimestampedData`` object.
            - If False, returns a ``ValveState`` object.
        synchronous_update
            - If True, retrieves the data synchronously.
            - If False, return latest known data without waiting. This data may be outdated
              by up to one cycle, as defined by ``SetMonitoringInterval``.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        TimestampedData or ValveState
        """
        return super().GetRtValveState(include_timestamp, synchronous_update, timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRtOutputState(
        self,
        bank_id: MxIoBankId = MxIoBankId.MX_IO_BANK_ID_IO_MODULE,
        synchronous_update: bool = False,
        timeout: float = None,
        include_timestamp: bool = False,
    ) -> Union[TimestampedData, List[int]]:
        """
        Returns the state of the digital outputs of the specified IO bank, such as the MVK01 module.

        For more information, see
        [GetRtOutputState](https://www.mecademic.com/res/doc?robot_command=GetRtOutputState&robot_model=MCS500).

        Parameters
        ----------
        bank_id: MxIoBankId, default: MxIoBankId.MX_IO_BANK_ID_IO_MODULE
            The IO bank ID to get output states for.
        synchronous_update
            - If True, retrieves the data synchronously.
            - If False, return latest known data without waiting. This data may be outdated
              by up to one cycle, as defined by ``SetMonitoringInterval``.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).
        include_timestamp
            - If True, returns a ``TimestampedData`` object.
            - If False, returns a list of integers.

        Returns
        -------
        TimestampedData or list of int
            In addition to the optional timestamp, the returned data is a list of integers
            representing the current digital output states for the requested bank ID.
        """
        return super().GetRtOutputState(bank_id=bank_id,
                                        synchronous_update=synchronous_update,
                                        timeout=timeout,
                                        include_timestamp=include_timestamp)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRtInputState(self,
                        bank_id: MxIoBankId = MxIoBankId.MX_IO_BANK_ID_IO_MODULE,
                        synchronous_update: bool = False,
                        timeout: float = None,
                        include_timestamp: bool = False) -> Union[TimestampedData, List[int]]:
        """
        Returns the state of the digital inputs of the specified IO bank, such as the MVK01 module.

        For more information, see
        [GetRtInputState](https://www.mecademic.com/res/doc?robot_command=GetRtInputState&robot_model=MCS500).

        Parameters
        ----------
        bank_id: MxIoBankId, default: MxIoBankId.MX_IO_BANK_ID_IO_MODULE
            The IO bank ID to get input states for.
        synchronous_update: bool, default: False
            - If True, retrieves the data synchronously.
            - If False, returns latest known data without waiting. This data may be outdated
              by up to one cycle, as defined by ``SetMonitoringInterval``.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).
        include_timestamp
            - If True, returns a ``TimestampedData`` object.
            - If False, returns a list of integers.

        Returns
        -------
        TimestampedData or list of int
            In addition to the optional timestamp, the returned data is a list of integers
            representing the current digital input states for the requested bank ID.

        """
        return super().GetRtInputState(bank_id=bank_id,
                                       synchronous_update=synchronous_update,
                                       timeout=timeout,
                                       include_timestamp=include_timestamp)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRtVacuumState(self,
                         include_timestamp: bool = False,
                         synchronous_update: bool = False,
                         timeout: float = None) -> Union[TimestampedData, VacuumState]:
        """
        Returns the current state of the pneumatic part of the MVK01 vacuum and I/O module.

        For more information, see
        [GetRtVacuumState](https://www.mecademic.com/res/doc?robot_command=GetRtVacuumState&robot_model=MCS500).

        Parameters
        ----------
        include_timestamp
            - If True, returns a ``TimestampedData`` object.
            - If False, returns a ``VacuumState`` object.
        synchronous_update
            - If True, retrieves the data synchronously.
            - If False, returns latest known data without waiting. This data may be outdated
              by up to one cycle, as defined by ``SetMonitoringInterval``.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        TimestampedData or VacuumState
        """
        return super().GetRtVacuumState(include_timestamp, synchronous_update, timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRtVacuumPressure(self,
                            *,
                            include_timestamp: bool = False,
                            synchronous_update: bool = True,
                            timeout: Optional[float] = None) -> Union[TimestampedData, float]:
        """
        Returns the current pressure in the vacuum chamber of the MVK01 vacuum and I/O module.

        For more information, see
        [GetRtVacuumPressure](https://www.mecademic.com/res/doc?robot_command=GetRtVacuumPressure&robot_model=MCS500).

        Notes
        -----
        This is optional real-time data (`update_type` = ``MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``).
        It is available only if either:

        - `synchronous_update` is used, or
        - ``SetRealTimeMonitoring`` is called to include it in the robot's cyclic data.

        Parameters
        ----------
        include_timestamp
            - If True, returns a ``TimestampedData`` object.
            - If False, returns a float value.
        synchronous_update
            - If True, retrieves the data synchronously.
            - If False, returns latest known data without waiting. This data may be outdated by
              up to one cycle, as defined by ``SetMonitoringInterval``.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        TimestampedData or float
            In addition to the optional timestamp, the returned data is a float representing the
            vacuum pressure, in kPa.
        """
        return super().GetRtVacuumPressure(include_timestamp=include_timestamp,
                                           synchronous_update=synchronous_update,
                                           timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRtTargetJointPos(self,
                            include_timestamp: bool = False,
                            synchronous_update: bool = True,
                            timeout: float = None) -> Union[TimestampedData, List[float]]:
        """
        Returns the current target joint set.

        For more information, see
        [GetRtTargetJointPos](https://www.mecademic.com/res/doc?robot_command=GetRtTargetJointPos).

        Parameters
        ----------
        include_timestamp
            - If True, returns a ``TimestampedData`` object.
            - If False, returns a list of floats.
        synchronous_update
            - If True, retrieves the data synchronously.
            - If False, return latest known data without waiting. This data may be outdated
              by up to one cycle, as defined by ``SetMonitoringInterval``.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        TimestampedData or list of float
            In addition to the optional timestamp, the returned data is a list of floats representing
            the current target joint set.
        """
        return super().GetRtTargetJointPos(include_timestamp, synchronous_update, timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetJoints(self, synchronous_update: bool = False, timeout: float = None) -> List[float]:
        """Deprecated command. Use ``GetRtTargetJointPos`` instead.
        """
        return super().GetRtTargetJointPos(include_timestamp=False,
                                           synchronous_update=synchronous_update,
                                           timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRtJointPos(self,
                      *,
                      include_timestamp: bool = False,
                      synchronous_update: bool = True,
                      timeout: float = None) -> Union[TimestampedData, List[float]]:
        """
        Returns the current joint set read by the joint encoders.

        For more information, see
        [GetRtJointPos](https://www.mecademic.com/res/doc?robot_command=GetRtJointPos).

        Notes
        -----
        This is optional real-time data (``update_type = MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``).
        It is available only if either:

        - `synchronous_update` is used, or
        - ``SetRealTimeMonitoring`` is called to include it in the robot's cyclic data.

        Parameters
        ----------
        include_timestamp
            - If True, returns a ``TimestampedData`` object.
            - If False, returns a list of floats.
        synchronous_update
            - If True, retrieves the data synchronously.
            - If False, returns latest known data without waiting. This data may be outdated
              by up to one cycle, as defined by ``SetMonitoringInterval``.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        TimestampedData or list of float
            In addition to the optional timestamp, the returned data is a list of floats representing
            the current joint set.
        """
        return super().GetRtJointPos(include_timestamp=include_timestamp,
                                     synchronous_update=synchronous_update,
                                     timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRtTargetJointTorq(self,
                             *,
                             include_timestamp: bool = False,
                             synchronous_update: bool = True,
                             timeout: float = None) -> Union[TimestampedData, List[float]]:
        """
        Returns the current estimated motor torques.

        For more information, see
        [GetRtTargetJointTorq](https://www.mecademic.com/res/doc?robot_command=GetRtTargetJointTorq).

        Notes
        -----
        This is optional real-time data (``update_type = MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``).
        It is available only if either:

        - `synchronous_update` is used, or
        - ``SetRealTimeMonitoring`` is called to include it in the robot's cyclic data.

        Parameters
        ----------
        include_timestamp
            - If True, returns a ``TimestampedData`` object.
            - If False, returns a list of floats.
        synchronous_update
            - If True, retrieves the data synchronously.
            - If False, returns latest known data without waiting. This data may be outdated
              by up to one cycle, as defined by ``SetMonitoringInterval``.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        TimestampedData or list of float
            In addition to the optional timestamp, the returned data is a list of floats representing
            the current target motor torques as signed percentages of the maximum allowable torques.
        """
        return super().GetRtTargetJointTorq(include_timestamp=include_timestamp,
                                            synchronous_update=synchronous_update,
                                            timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRtJointTorq(self,
                       *,
                       include_timestamp: bool = False,
                       synchronous_update: bool = True,
                       timeout: float = None) -> Union[TimestampedData, List[float]]:
        """
        Returns the current motor torques.

        For more information, see
        [GetRtJointTorq](https://www.mecademic.com/res/doc?robot_command=GetRtJointTorq).

        Notes
        -----
        This is optional real-time data (``update_type = MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``).
        It is available only if either:

        - `synchronous_update` is used, or
        - ``SetRealTimeMonitoring`` is called to include it in the robot's cyclic data.

        Parameters
        ----------
        include_timestamp
            - If True, returns a ``TimestampedData`` object.
            - If False, returns a list of floats.
        synchronous_update
            - If True, retrieves the data synchronously.
            - If False, returns latest known data without waiting. This data may be outdated
              by up to one cycle, as defined by ``SetMonitoringInterval``.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        TimestampedData or list of float
            In addition to the optional timestamp, the returned data is a list of floats representing
            the current motor torques as signed percentages of the maximum allowable torques.
        """
        return super().GetRtJointTorq(include_timestamp=include_timestamp,
                                      synchronous_update=synchronous_update,
                                      timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRtTargetJointVel(self,
                            *,
                            include_timestamp: bool = False,
                            synchronous_update: bool = True,
                            timeout: float = None) -> Union[TimestampedData, List[float]]:
        """
        Returns the current target joint velocities.

        For more information, see
        [GetRtTargetJointVel](https://www.mecademic.com/res/doc?robot_command=GetRtTargetJointVel).

        Notes
        -----
        This is optional real-time data (``update_type = MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``).
        It is available only if either:

        - `synchronous_update` is used, or
        - ``SetRealTimeMonitoring`` is called to include it in the robot's cyclic data.

        Parameters
        ----------
        include_timestamp
            - If True, returns a ``TimestampedData`` object.
            - If False, returns a list of floats.
        synchronous_update
            - If True, retrieves the data synchronously.
            - If False, returns latest known data without waiting. This data may be outdated
              by up to one cycle, as defined by ``SetMonitoringInterval``.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        TimestampedData or list of float
            In addition to the optional timestamp, the returned data is a list of floats representing
            the current target joint velocities.
        """
        return super().GetRtTargetJointVel(include_timestamp=include_timestamp,
                                           synchronous_update=synchronous_update,
                                           timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRtJointVel(self,
                      *,
                      include_timestamp: bool = False,
                      synchronous_update: bool = True,
                      timeout: float = None) -> Union[TimestampedData, List[float]]:
        """
        Returns the current joint velocities, calculated by differentiating the joint encoders data.

        For more information, see
        [GetRtJointVel](https://www.mecademic.com/res/doc?robot_command=GetRtJointVel).

        Notes
        -----
        This is optional real-time data (``update_type = MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``).
        It is available only if either:

        - `synchronous_update` is used, or
        - ``SetRealTimeMonitoring`` is called to include it in the robot's cyclic data.

        Parameters
        ----------
        include_timestamp
            - If True, returns a ``TimestampedData`` object.
            - If False, returns a list of floats.
        synchronous_update
            - If True, retrieves the data synchronously.
            - If False, returns latest known data without waiting. This data may be outdated
              by up to one cycle, as defined by ``SetMonitoringInterval``.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        TimestampedData or list of float
            In addition to the optional timestamp, the returned data is a list of floats representing
            the current joint velocities.
        """
        return super().GetRtJointVel(include_timestamp=include_timestamp,
                                     synchronous_update=synchronous_update,
                                     timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRtTargetCartPos(self,
                           include_timestamp: bool = False,
                           synchronous_update: bool = True,
                           timeout: Optional[float] = None) -> Union[TimestampedData, List[float]]:
        """
        Returns the current target pose of the TRF relative to the WRF.

        For more information, see
        [GetRtTargetCartPos](https://www.mecademic.com/res/doc?robot_command=GetRtTargetCartPos).

        Parameters
        ----------
        include_timestamp
            - If True, returns a ``TimestampedData`` object.
            - If False, returns a list of floats.
        synchronous_update
            - If True, retrieves the data synchronously.
            - If False, return latest known data without waiting. This data may be outdated
              by up to one cycle, as defined by ``SetMonitoringInterval``.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        TimestampedData or list of float
            In addition to the optional timestamp, the returned data is a list of floats representing
            the current target pose of the TRF relative to the WRF.
        """
        return super().GetRtTargetCartPos(include_timestamp, synchronous_update=synchronous_update, timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetPose(self, synchronous_update: bool = False, timeout: float = None) -> List[float]:
        """Deprecated command. Use ``GetRtTargetCartPos`` instead.
        """
        return super().GetRtTargetCartPos(include_timestamp=False,
                                          synchronous_update=synchronous_update,
                                          timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRtCartPos(self,
                     *,
                     include_timestamp: bool = False,
                     synchronous_update: bool = True,
                     timeout: Optional[float] = None) -> Union[TimestampedData, List[float]]:
        """
        Returns the current pose of the TRF with respect to the WRF, as calculated from the current
        joint set read by the joint encoders.

        For more information, see
        [GetRtCartPos](https://www.mecademic.com/res/doc?robot_command=GetRtCartPos).

        Notes
        -----
        This is optional real-time data (``update_type = MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``).
        It is available only if either:

        - `synchronous_update` is used, or
        - ``SetRealTimeMonitoring`` is called to include it in the robot's cyclic data.

        Parameters
        ----------
        include_timestamp
            - If True, returns a ``TimestampedData`` object.
            - If False, returns a list of floats.
        synchronous_update: bool, default: False
            - If True, retrieves the data synchronously.
            - If False, returns latest known data without waiting. This data may be outdated
              by up to one cycle, as defined by ``SetMonitoringInterval``.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        TimestampedData or list of float
            In addition to the optional timestamp, the returned data is a list of floats representing
            the current pose of the TRF relative to the WRF.
        """
        return super().GetRtCartPos(include_timestamp=include_timestamp,
                                    synchronous_update=synchronous_update,
                                    timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRtTargetCartVel(self,
                           *,
                           include_timestamp: bool = False,
                           synchronous_update: bool = True,
                           timeout: Optional[float] = None) -> Union[TimestampedData, List[float]]:
        """
        Returns the current target Cartesian velocity vector of the TRF with respect to the WRF.

        For more information, see
        [GetRtTargetCartVel](https://www.mecademic.com/res/doc?robot_command=GetRtTargetCartVel).

        Notes
        -----
        This is optional real-time data (``update_type = MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``).
        It is available only if either:

        - `synchronous_update` is used, or
        - ``SetRealTimeMonitoring`` is called to include it in the robot's cyclic data.

        Parameters
        ----------
        include_timestamp
            - If True, returns a ``TimestampedData`` object.
            - If False, returns a list of floats.
        synchronous_update
            - If True, retrieves the data synchronously.
            - If False, returns latest known data without waiting. This data may be outdated
              by up to one cycle, as defined by ``SetMonitoringInterval``.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        TimestampedData or list of float
            In addition to the optional timestamp, the returned data is a list of floats representing
            the current target Cartesian velocity vector of the TRF relative to the WRF.
        """
        return super().GetRtTargetCartVel(include_timestamp=include_timestamp,
                                          synchronous_update=synchronous_update,
                                          timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRtCartVel(self,
                     *,
                     include_timestamp: bool = False,
                     synchronous_update: bool = True,
                     timeout: Optional[float] = None) -> Union[TimestampedData, List[float]]:
        """
        Returns the current Cartesian velocity vector of the TRF with respect to the WRF, as
        calculated from the real-time data coming from the joint encoders

        For more information, see
        [GetRtCartVel](https://www.mecademic.com/res/doc?robot_command=GetRtCartVel).

        Notes
        -----
        This is optional real-time data (``update_type = MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``).
        It is available only if either:

        - `synchronous_update` is used, or
        - ``SetRealTimeMonitoring`` is called to include it in the robot's cyclic data.

        Parameters
        ----------
        include_timestamp
            - If True, returns a ``TimestampedData`` object.
            - If False, returns a list of floats.
        synchronous_update
            - If True, retrieves the data synchronously.
            - If False, returns latest known data without waiting. This data may be outdated
              by up to one cycle, as defined by ``SetMonitoringInterval``.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        TimestampedData or list of float
            In addition to the optional timestamp, the returned data is a list of floats representing
            current Cartesian velocity vector of the TRF relative to the WRF.
        """
        return super().GetRtCartVel(include_timestamp=include_timestamp,
                                    synchronous_update=synchronous_update,
                                    timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRtTargetConf(self,
                        *,
                        include_timestamp: bool = False,
                        synchronous_update: bool = False,
                        timeout: Optional[float] = None) -> Union[TimestampedData, List[float]]:
        """
        Returns the posture configuration parameters calculated from the current target joint set.

        For more information, see
        [GetRtTargetConf](https://www.mecademic.com/res/doc?robot_command=GetRtTargetConf).

        Parameters
        ----------
        include_timestamp
            - If True, returns a ``TimestampedData`` object.
            - If False, returns a float value.
        synchronous_update
            - If True, retrieves the data synchronously.
            - If False, return latest known data without waiting. This data may be outdated by
              up to one cycle, as defined by ``SetMonitoringInterval``.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        TimestampedData or list of float
            In addition to the optional timestamp, the returned data is a list of floats representing
            the robot's target posture configuration parameters, where each parameter is either -1 or
            1, or 0 at a singularity.
        """
        return super().GetRtTargetConf(include_timestamp=include_timestamp,
                                       synchronous_update=synchronous_update,
                                       timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRtConf(self,
                  *,
                  include_timestamp: bool = False,
                  synchronous_update: bool = False,
                  timeout: Optional[float] = None) -> Union[TimestampedData, List[float]]:
        """
        Returns the current posture configuration parameters calculated from the real-time data
        coming from the joint encoders.

        For more information, see
        [GetRtConf](https://www.mecademic.com/res/doc?robot_command=GetRtConf).

        Parameters
        ----------
        include_timestamp
            - If True, returns a ``TimestampedData`` object.
            - If False, returns a float value.
        synchronous_update
            - If True, retrieves the data synchronously.
            - If False, return latest known data without waiting. This data may be outdated by
              up to one cycle, as defined by ``SetMonitoringInterval``.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        TimestampedData or list of float
            In addition to the optional timestamp, the returned data is
            a list of floats representing the robot's current posture configuration parameters,
            where each parameter is either -1 or 1, or 0 at a singularity.
        """
        return super().GetRtConf(include_timestamp=include_timestamp,
                                 synchronous_update=synchronous_update,
                                 timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRtTargetConfTurn(self,
                            *,
                            include_timestamp: bool = False,
                            synchronous_update: bool = False,
                            timeout: Optional[float] = None) -> Union[TimestampedData, float]:
        """
        Returns the turn configuration parameters calculated from the current target joint value
        for last joint.

        For more information, see
        [GetRtTargetConfTurn](https://www.mecademic.com/res/doc?robot_command=GetRtTargetConfTurn).

        Parameters
        ----------
        include_timestamp
            - If True, returns a ``TimestampedData`` object.
            - If False, returns a float value.
        synchronous_update
            - If True, retrieves the data synchronously.
            - If False, return latest known data without waiting. This data may be outdated by
              up to one cycle, as defined by ``SetMonitoringInterval``.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        TimestampedData or float
            In addition to the optional timestamp, the returned data is the target turn configuration.

        """
        return super().GetRtTargetConfTurn(include_timestamp=include_timestamp,
                                           synchronous_update=synchronous_update,
                                           timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRtConfTurn(self,
                      *,
                      include_timestamp: bool = False,
                      synchronous_update: bool = False,
                      timeout: Optional[float] = None) -> Union[TimestampedData, float]:
        """
        Returns the current turn configuration parameter calculated from the real-time data coming
        from the joint encoder of last joint.

        For more information, see
        [GetRtConfTurn](https://www.mecademic.com/res/doc?robot_command=GetRtConfTurn).

        Parameters
        ----------
        include_timestamp
            - If True, returns a ``TimestampedData`` object.
            - If False, returns a float value.
        synchronous_update
            - If True, retrieves the data synchronously.
            - If False, return latest known data without waiting. This data may be outdated by
              up to one cycle, as defined by ``SetMonitoringInterval``.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        TimestampedData or float
            In addition to the optional timestamp, the returned data is the current turn configuration.
        """
        return super().GetRtConfTurn(include_timestamp=include_timestamp,
                                     synchronous_update=synchronous_update,
                                     timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRtTrf(self,
                 *,
                 include_timestamp: bool = False,
                 synchronous_update: bool = False,
                 timeout: Optional[float] = None) -> Union[TimestampedData, List[float]]:
        """
        Returns the current definition of the TRF with respect to the FRF.

        For more information, see
        [GetRtTrf](https://www.mecademic.com/res/doc?robot_command=GetRtTrf).

        Parameters
        ----------
        include_timestamp
            - If True, returns a ``TimestampedData`` object.
            - If False, returns a float value.
        synchronous_update
            - If True, retrieves the data synchronously.
            - If False, return latest known data without waiting. This data may be outdated by
              up to one cycle, as defined by ``SetMonitoringInterval``.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        TimestampedData or list of float
            In addition to the optional timestamp, the returned data is a list of floats
            representing the current pose of the TRF with respect to the FRF.
        """
        return super().GetRtTrf(include_timestamp=include_timestamp,
                                synchronous_update=synchronous_update,
                                timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetTrf(self,
               *,
               include_timestamp: bool = False,
               synchronous_update: bool = False,
               timeout: Optional[float] = None) -> Union[TimestampedData, List[float]]:
        """ See GetRtTrf """
        return super().GetRtTrf(include_timestamp=include_timestamp,
                                synchronous_update=synchronous_update,
                                timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRtWrf(self,
                 *,
                 include_timestamp: bool = False,
                 synchronous_update: bool = False,
                 timeout: Optional[float] = None) -> Union[TimestampedData, List[float]]:
        """
        Returns the current definition of the WRF with respect to the BRF.

        For more information, see
        [GetRtWrf](https://www.mecademic.com/res/doc?robot_command=GetRtWrf).

        Parameters
        ----------
        include_timestamp
            - If True, returns a ``TimestampedData`` object.
            - If False, returns a float value.
        synchronous_update
            - If True, retrieves the data synchronously.
            - If False, return latest known data without waiting. This data may be outdated by
              up to one cycle, as defined by ``SetMonitoringInterval``.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        TimestampedData or list of float
            In addition to the optional timestamp, the returned data is a list of floats
            representing the current pose of the WRF with respect to the BRF.

        """
        return super().GetRtWrf(include_timestamp=include_timestamp,
                                synchronous_update=synchronous_update,
                                timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetWrf(self,
               *,
               include_timestamp: bool = False,
               synchronous_update: bool = False,
               timeout: Optional[float] = None) -> Union[TimestampedData, List[float]]:
        """ See ``GetRtWrf`` """
        return super().GetRtWrf(include_timestamp=include_timestamp,
                                synchronous_update=synchronous_update,
                                timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRtTemperature(self,
                         *,
                         include_timestamp: bool = False,
                         synchronous_update: bool = False,
                         timeout: Optional[float] = None) -> Union[TimestampedData, RobotTemperature]:
        """
        Returns the current temperature of various components on the robot.

        For more information, see
        [GetRtTemperature](https://www.mecademic.com/res/doc?robot_command=GetRtTemperature&robot_model=MCS500).

        Notes
        -----
        This is optional real-time data (``update_type = MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``).
        It is available only if either:

        - `synchronous_update` is used, or
        - ``SetRealTimeMonitoring`` is called to include it in the robot's cyclic data.

        Parameters
        ----------
        include_timestamp
            - If True, returns a ``TimestampedData`` object.
            - If False, returns a RobotTemperature object.
        synchronous_update
            - If True, retrieves the data synchronously.
            - If False, return latest known data without waiting. This data may be outdated by
              up to one cycle, as defined by ``SetMonitoringInterval``.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        TimestampedData or RobotTemperature
            Temperatures are reported (in this order) for baseboard, power supply, safety processor, each motor (joint).
        """
        return super().GetRtTemperature(include_timestamp=include_timestamp,
                                        synchronous_update=synchronous_update,
                                        timeout=timeout)

    # mx:export_to=mecascript_globals.py
    @disconnect_on_exception_decorator
    def GetRtI2t(self,
                 *,
                 include_timestamp: bool = False,
                 synchronous_update: bool = False,
                 timeout: Optional[float] = None) -> Union[TimestampedData, List[float]]:
        """
        Returns the current I2t values of all robot motors

        For more information, see
        [GetRtI2t](https://www.mecademic.com/res/doc?robot_command=GetRtI2t&robot_model=MCS500).

        Notes
        -----
        This is optional real-time data (``update_type = MX_RT_DATA_UPDATE_TYPE_CYCLICAL_OPTIONAL``).
        It is available only if either:

        - `synchronous_update` is used, or
        - ``SetRealTimeMonitoring`` is called to include it in the robot's cyclic data.

        Parameters
        ----------
        include_timestamp
            - If True, returns a ``TimestampedData`` object.
            - If False, returns the I2t values as a list of float.
        synchronous_update
            - If True, retrieves the data synchronously.
            - If False, return latest known data without waiting. This data may be outdated by
              up to one cycle, as defined by ``SetMonitoringInterval``.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        TimestampedData or list of float
            One I2t value is reported per motor (joint).
        """
        return super().GetRtI2t(include_timestamp=include_timestamp,
                                synchronous_update=synchronous_update,
                                timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetEob(self, e: Union[int, bool]):
        """
        Note: This command is legacy. It should normally not be necessary to use it.
        Enables or disables the end-of-block notification sent by the robot (``MxRobotStatusCode.MX_ST_EOB``)

        For more information, see
        [SetEob](https://www.mecademic.com/res/doc?robot_command=SetEob).

        Parameters
        ----------
        e
            Enable or disable event ``MxRobotStatusCode.MX_ST_EOB`` from the robot.

        Notes
        -----
        This remains until the robot is rebooted. Default value after reboot is enabled.
        """
        super()._send_custom_command('SetEob', args=[int(e)])

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetEom(self, e: Union[int, bool]):
        """
        Note: This command is legacy. It should normally not be necessary to use it.
        Enables or disables the end-of-motion notification sent by the robot (``MxRobotStatusCode.MX_ST_EOM``)

        For more information, see
        [SetEom](https://www.mecademic.com/res/doc?robot_command=SetEom).

        Parameters
        ----------
        e
            Enable or disable event ``MxRobotStatusCode.MX_ST_EOM`` from the robot.

        Notes
        -----
        This remains until the robot is rebooted. Default value after reboot is enabled.
        """
        super()._send_custom_command('SetEom', args=[int(e)])

    # mx:export_to=mecascript_globals.py
    @disconnect_on_exception_decorator
    def SetMonitoringInterval(self, t: float):
        """
        Sets the interval at which the monitoring port sends real-time data.

        For more information, see
        [SetMonitoringInterval](https://www.mecademic.com/res/doc?robot_command=SetMonitoringInterval).

        Parameters
        ----------
        t
            Monitoring interval duration in seconds.

            Default value restored at robot activation: ``MX_ROBOT_CFG_DEFAULT_MONITORING_INTERVAL``.
        """
        super().SetMonitoringInterval(t)

    @mecascript_global_function_decorator
    @overload
    def SetRealTimeMonitoring(self, events: List[MxRobotStatusCode]) -> None:
        ...

    @mecascript_global_function_decorator
    @overload
    def SetRealTimeMonitoring(self, *events: MxRobotStatusCode) -> None:
        ...

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetRealTimeMonitoring(self, *events: List[Union[MxRobotStatusCode, str]]):
        """
        Specifies the real-time monitoring events to enable.

        Once enabled, these events become available through ``GetRobotRtData``
        (or other ``Get*`` commands).

        For more information, see
        [SetRealTimeMonitoring](https://www.mecademic.com/res/doc?robot_command=SetRealTimeMonitoring).

        Parameters
        ----------
        events
            One or more event IDs to enable. Each argument may be a single event ID
            (``MxRobotStatusCode.MX_ST_RT_*`` or string) or a list of event IDs. See the
            ``RobotRtData`` class for a description of available real-time data fields:
            which are always enabled and which must be enabled explicitly using
            ``SetRealTimeMonitoring``. Event IDs may also be provided as strings
            corresponding to field names in ``RobotRtData`` or to column name prefixes
            from captured ``.csv`` files. See ``ROBOT_RT_DATA_FIELDS`` for a complete list.
            Use ``events='all'`` to enable all real-time data.

        Examples
        --------
        >>> # Enables the joint torque and Cartesian velocity fields.
        >>> SetRealTimeMonitoring(MxRobotStatusCode.MX_ST_RT_JOINT_TORQ, MxRobotStatusCode.MX_ST_RT_CART_VEL)
        """
        super()._set_real_time_monitoring(*events)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def ForceRealTimeMonitoring(self):
        """
        Triggers the robot to immediately send a monitoring cycle with all currently enabled data
        (see ``SetRealTimeMonitoring``).

        This command does not modify the regular monitoring interval configured with
        ``SetMonitoringInterval``; it simply forces an additional update outside the normal schedule.

        This function is asynchronous and does not wait for the next cycle.
        """
        super().ForceRealTimeMonitoring()

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetRtc(self, t: int):
        """
        Set the robot's real-time clock (date and time).

        Note that the RTC is reset after a reboot, so this command must be sent each time
        the robot restarts.

        For more information, see
        [SetRtc](https://www.mecademic.com/res/doc?robot_command=SetRtc).

        Parameters
        ----------
        t
            Unix epoch time (seconds since 00:00:00 UTC Jan 1, 1970).
        """
        super().SetRtc(t)

    @deprecation.deprecated(deprecated_in="1.2.0",
                            removed_in="3.0.0",
                            current_version=get_mecademicpy_version(),
                            details="Use the 'SetRtc' function instead")
    @disconnect_on_exception_decorator
    def SetRTC(self, t: int):
        """Deprecated, use ``SetRtc`` instead.
        """
        super().SetRtc(t)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def ActivateSim(self, mode: Optional[MxRobotSimulationMode] = None):
        """
        Enables simulation mode. Motors and external tools do not move, but commands are still
        processed.

        For more information, see
        [ActivateSim](https://www.mecademic.com/res/doc?robot_command=ActivateSim).

        Parameters
        ----------
        mode
            Selects the simulation mode ('real-time' or 'fast'). If not specified, the
            robot's default simulation mode is used (see ``SetSimModeCfg``).
        """
        super().ActivateSim(mode)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def DeactivateSim(self):
        """
        Disables simulation mode. Motors and external tools move normally.

        For more information, see
        [DeactivateSim](https://www.mecademic.com/res/doc?robot_command=DeactivateSim).
        """
        super().DeactivateSim()

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetExtToolSim(self, sim_ext_tool_type: MxExtToolType):
        """
        Enables the emulation of one of our three EOAT (two electric grippers and a pneumatic
        module), in the case of the Meca500.

        For more information, see
        [SetExtToolSim](https://www.mecademic.com/res/doc?robot_command=SetExtToolSim&robot_model=MECA500).

        Parameters
        ----------
        sim_ext_tool_type
            The simulated tool type to enable (or ``MxExtToolType.MX_EXT_TOOL_NONE`` to disable simulation)
        """
        super().SetExtToolSim(sim_ext_tool_type)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetIoSim(self, bank_id: MxIoBankId = MxIoBankId.MX_IO_BANK_ID_IO_MODULE, enable: bool = True):
        """
        Enables simulation mode for the MVK01 vacuum and I/O module.

        For more information, see
        [SetIoSim](https://www.mecademic.com/res/doc?robot_command=SetIoSim&robot_model=MCS500).

        Parameters
        ----------
        bank_id
            I/O bank ID to enable or disable simulation mode for.
        enable
            Whether to enable (True) or disable (False) simulation mode.
        """
        super().SetIoSim(bank_id, enable)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetRecoveryMode(self, activated: bool = True):
        """
        Enables recovery mode, allowing the robot to move slowly without homing and outside
        user-defined joint limits.

        For more information, see
        [SetRecoveryMode](https://www.mecademic.com/res/doc?robot_command=SetRecoveryMode).

        Parameters
        ----------
        activated
            Whether to enable (True) or disable (False) recovery mode.
        """
        super().SetRecoveryMode(activated)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetTimeScaling(self, p: float):
        """
        Sets the time scaling of the trajectory generator.

        By calling this command with an argument p < 100, all robot motions remain exactly
        the same (i.e., the path remains the same), but everything will be (100 - p) percent slower,
        including time delays (e.g., the pause set by the command ``Delay``).

        For more information, see
        [SetTimeScaling](https://www.mecademic.com/res/doc?robot_command=SetTimeScaling).

        Parameters
        ----------
        p
            Percentage time scaling.
            Default value restored at robot activation: ``MX_ROBOT_CFG_DEFAULT_TIME_SCALING``.
        """
        super().SetTimeScaling(p)

    @mecascript_global_function_decorator
    @overload
    def SetJointLimitsCfg(self, e: bool = True):
        ...

    @mecascript_global_function_decorator
    @overload
    def SetJointLimitsCfg(self, cfg: JointLimitsCfg, /):  # pylint: disable=arguments-differ
        ...

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetJointLimitsCfg(self, e: Union[bool, JointLimitsCfg] = True):
        """
        Enables or disables the user-defined limits set by the ``SetJointLimits`` command.
        If the user-defined limits are disabled, the default joint limits become active.
        However, user-defined limits remain in memory, and can be re-enabled, even after a power down.

        For more information, see
        [SetJointLimitsCfg](https://www.mecademic.com/res/doc?robot_command=SetJointLimitsCfg).

        Parameters
        ----------
        e
            Whether to enable (True) or disable (False) the user-defined joint limits.
        cfg
            Alternatively, you can provide JointLimitsCfg as a single argument

        Notes
        -----
        This setting is persistent and retains its value even after power cycling the robot.
        The factory default is False.
        """
        if isinstance(e, JointLimitsCfg):
            e = e.enabled
        super().SetJointLimitsCfg(e)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetJointLimits(self, n: int, lower_limit: float, upper_limit: float):
        """
        Redefines the lower and upper limits of a robot joint.

        To apply these user-defined joint limits, execute the command ``SetJointLimitsCfg(True)``.
        The new joint limits must remain within the default limits.

        For more information, see
        [SetJointLimits](https://www.mecademic.com/res/doc?robot_command=SetJointLimits).

        Parameters
        ----------
        n
            Joint number
        lower_limit
            Lower joint limit, in degrees or in mm
        upper_limit
            Upper joint limit, in degrees or in mm

        Notes
        -----
        This setting is persistent and retains its value even after power cycling the robot.
        Use ``SetJointLimits(n,0,0)`` to reset the joint limits of joint n to its factory default
        values or simply disable the user-defined joint limits with the command
        ``SetJointLimitsCfg(False)``.
        """
        super().SetJointLimits(n, lower_limit, upper_limit)

    @mecascript_global_function_decorator
    @overload
    def SetWorkZoneCfg(self,
                       severity: MxEventSeverity = MxEventSeverity.MX_EVENT_SEVERITY_ERROR,
                       mode: MxWorkZoneMode = MxWorkZoneMode.MX_WORK_ZONE_MODE_FCP_IN_WORK_ZONE):
        ...

    @mecascript_global_function_decorator
    @overload
    def SetWorkZoneCfg(self, cfg: WorkZoneCfg, /):  # pylint: disable=arguments-differ
        ...

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetWorkZoneCfg(self,
                       severity: Union[MxEventSeverity, WorkZoneCfg] = MxEventSeverity.MX_EVENT_SEVERITY_ERROR,
                       mode: MxWorkZoneMode = MxWorkZoneMode.MX_WORK_ZONE_MODE_FCP_IN_WORK_ZONE):
        """
        Specifies the event severity for work zone supervision and the robot parts to verify.

        For more information, see
        [SetWorkZoneCfg](https://www.mecademic.com/res/doc?robot_command=SetWorkZoneCfg).

        Parameters
        ----------
        severity
            Action when an imminent zone breach is detected

            - ``MX_EVENT_SEVERITY_SILENT`` only logs the breach ,
            - ``MX_EVENT_SEVERITY_WARNING`` generates a warning,
            - ``MX_EVENT_SEVERITY_ERROR`` generates a warning and a motion error.
        mode
            Work zone detection mode to use. Available modes are listed in ``mx_robot_def.MxWorkZoneMode``.
        cfg
            Alternatively, you can provide WorkZoneCfg as a single argument

        Notes
        -----
        This setting is persistent and retains its value even after power cycling the robot.
        """
        if isinstance(severity, WorkZoneCfg):
            mode = severity.mode
            severity = severity.severity
        super().SetWorkZoneCfg(severity, mode)

    @mecascript_global_function_decorator
    @overload
    def SetWorkZoneLimits(self, x_min: float, y_min: float, z_min: float, x_max: float, y_max: float, z_max: float):
        ...

    @mecascript_global_function_decorator
    @overload
    def SetWorkZoneLimits(self, limits: WorkZoneLimits, /):  # pylint: disable=arguments-differ
        ...

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetWorkZoneLimits(self,
                          x_min: Union[float, WorkZoneLimits],
                          y_min: float = None,
                          z_min: float = None,
                          x_max: float = None,
                          y_max: float = None,
                          z_max: float = None):
        """
        Defines a bounding box, the sides of which are parallel to the axes of the BRF.

        The arguments of the command are the coordinates of two diagonally opposite corners,
        referred to as minimum and maximum corners, such that each coordinate of the minimum corner
        is smaller that the corresponding coordinate of the maximum corner.

        For more information, see
        [SetWorkZoneLimits](https://www.mecademic.com/res/doc?robot_command=SetWorkZoneLimits).

        Parameters
        ----------
        x_min
            X coordinate of the minimum corner of the cuboid in the BRF, in mm.
        y_min
            Y coordinate of the minimum corner of the cuboid in the BRF, in mm.
        z_min
            Z coordinate of the minimum corner of the cuboid in the BRF, in mm.
        x_max
            X coordinate of the maximum corner of the cuboid in the BRF, in mm.
        y_max
            Y coordinate of the maximum corner of the cuboid in the BRF, in mm.
        z_max
            Z coordinate of the maximum corner of the cuboid in the BRF, in mm.
        limits
            Alternatively, you can provide WorkZoneLimits as a single argument

        Notes
        -----
        This setting is persistent and retains its value even after power cycling the robot.
        The factory default is x_min = y_min = z_min = -10,000 and
        x_max = y_max = z_max = 10,000. To reset the arguments to their factory default values,
        deactivate the robot and send the command ``SetWorkZoneLimits(0,0,0,0,0,0)``.
        """
        if isinstance(x_min, WorkZoneLimits):
            y_min = x_min.y_min
            z_min = x_min.z_min
            x_max = x_min.x_max
            y_max = x_min.y_max
            z_max = x_min.z_max
            x_min = x_min.x_min
        super().SetWorkZoneLimits(x_min, y_min, z_min, x_max, y_max, z_max)

    @mecascript_global_function_decorator
    @overload
    def SetCollisionCfg(self, severity: MxEventSeverity = MxEventSeverity.MX_EVENT_SEVERITY_ERROR):
        ...

    @mecascript_global_function_decorator
    @overload
    def SetCollisionCfg(self, cfg: CollisionCfg, /):  # pylint: disable=arguments-differ
        ...

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetCollisionCfg(self, severity: Union[MxEventSeverity, CollisionCfg] = MxEventSeverity.MX_EVENT_SEVERITY_ERROR):
        """
        Specifies the event severity for the collision supervision (robot links, tool sphere,
        and MPM500 module).

        For more information, see
        [SetCollisionCfg](https://www.mecademic.com/res/doc?robot_command=SetCollisionCfg).

        Parameters
        ----------
        severity
            Action when an imminent zone breach is detected

            - ``MX_EVENT_SEVERITY_SILENT`` only logs the breach,
            - ``MX_EVENT_SEVERITY_WARNING`` generates a warning,
            - ``MX_EVENT_SEVERITY_ERROR`` generates a warning and a motion error.
        cfg
            Alternatively, you can provide CollisionCfg as a single argument

        Notes
        -----
        This setting is persistent and retains its value even after power cycling the robot.
        """
        if isinstance(severity, CollisionCfg):
            severity = severity.severity
        super().SetCollisionCfg(severity)

    @mecascript_global_function_decorator
    #pylint: disable=invalid-name
    @overload
    def SetToolSphere(self, x: float, y: float, z: float, r: float):
        ...

    @mecascript_global_function_decorator
    @overload
    def SetToolSphere(self, payload: ToolSphere, /):  # pylint: disable=arguments-differ
        ...

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetToolSphere(self, x: Union[float, ToolSphere], y: float = None, z: float = None, r: float = None):
        """
        Defines a sphere fixed in the FRF. The system can then supervise potential interferences
        between this sphere and the robot links, as well as with the exterior of a bounding box
        set using the ``SetWorkZoneLimits`` command.

        For more information, see
        [SetToolSphere](https://www.mecademic.com/res/doc?robot_command=SetToolSphere).

        Parameters
        ----------
        x
            X coordinate of the center of the tool sphere in the FRF, in mm.
        y
            Y coordinate of the center of the tool sphere in the FRF, in mm.
        z
            Z coordinate of the center of the tool sphere in the FRF, in mm.
        r
            Radius of the tool sphere, in mm.
        payload
            Alternatively, you can provide ToolSphere as a single argument

        Notes
        -----
        This setting is persistent and retains its value even after power cycling the robot.
        The factory default is x = y = z = 0 and r = 0, which is equivalent to disabling the
        tool sphere.
        """
        if isinstance(x, ToolSphere):
            y = x.z
            z = x.z
            r = x.r
            x = x.x
        super().SetToolSphere(x, y, z, r)

    @mecascript_global_function_decorator
    @overload
    def SetTorqueLimitsCfg(self,
                           severity: MxEventSeverity = MxEventSeverity.MX_EVENT_SEVERITY_ERROR,
                           mode: MxTorqueLimitsMode = MxTorqueLimitsMode.MX_TORQUE_LIMITS_MODE_DELTA_WITH_EXPECTED):
        ...

    @mecascript_global_function_decorator
    @overload
    def SetTorqueLimitsCfg(self, cfg: TorqueLimitsCfg, /):  # pylint: disable=arguments-differ
        ...

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetTorqueLimitsCfg(self,
                           severity: Union[MxEventSeverity, TorqueLimitsCfg] = MxEventSeverity.MX_EVENT_SEVERITY_ERROR,
                           mode: MxTorqueLimitsMode = MxTorqueLimitsMode.MX_TORQUE_LIMITS_MODE_DELTA_WITH_EXPECTED):
        """
        Sets the robot behavior when a joint torque exceeds the threshold set by the
        ``SetTorqueLimits`` command.

        For more information, see
        [SetTorqueLimitsCfg](https://www.mecademic.com/res/doc?robot_command=SetTorqueLimitsCfg).

        Parameters
        ----------
        severity
            Action when a torque limit is exceeded

            - ``MX_EVENT_SEVERITY_SILENT`` ignores the event,
            - ``MX_EVENT_SEVERITY_WARNING`` sends a torque status event
                (``MxRobotStatusCode.MX_ST_TORQUE_LIMIT_STATUS``)
            - ``MX_EVENT_SEVERITY_PAUSE_MOTION`` sends a torque status event and pauses motion,
            - ``MX_EVENT_SEVERITY_ERROR``  sends a torque status event and ses torque limit error.
        mode
            Detection mode

            - ``MX_TORQUE_LIMITS_MODE_ABSOLUTE`` detects all the time,
            - ``MX_TORQUE_LIMITS_MODE_ABSOLUTE_SKIP_ACCEL`` ignores joint accel/decel phases,
            - ``MX_TORQUE_LIMITS_MODE_DELTA_WITH_EXPECTED`` detects all the time, but
               the torque limits set by ``SetTorqueLimits`` are interpreted as maximum deviations rather than
               absolute limits.
        cfg
            Alternatively, you can provide TorqueLimitsCfg as a single argument
        """
        if isinstance(severity, TorqueLimitsCfg):
            mode = severity.mode
            severity = severity.severity
        super().SetTorqueLimitsCfg(severity, mode)

    @mecascript_global_function_decorator
    #pylint: disable=invalid-name
    @overload
    def SetTorqueLimits(self, tau: List[float]):
        ...

    @mecascript_global_function_decorator
    #pylint: disable=invalid-name
    @overload
    def SetTorqueLimits(self,
                        tau_1: float,
                        tau_2: float,
                        tau_3: float,
                        tau_4: float,
                        tau_5: float = None,
                        tau_6: float = None):
        ...

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetTorqueLimits(self, *args: Union[List[float], float]):
        """
        Sets thresholds for the torques applied to each motor.

        See ``SetTorqueLimitsCfg`` for details on enabling these thresholds and the two possible
        interpretation modes.

        For more information, see
        [SetTorqueLimits](https://www.mecademic.com/res/doc?robot_command=SetTorqueLimits).

        Parameters
        ----------
        tau_1
            Desired torque thresholds represented as percentages
        tau_2
            Desired torque thresholds represented as percentages
        tau_3
            Desired torque thresholds represented as percentages
        tau_4
            Desired torque thresholds represented as percentages
        tau_5
            Desired torque thresholds represented as percentages
        tau_6
            Desired torque thresholds represented as percentages
        tau:
            Alternatively, you can provide a list of threshold as a single argument

        Notes
        -----
            Default values restored at robot activation: 100, 100, ..., 100.
        """
        self._send_motion_command_check('SetTorqueLimits', expected_count=self._robot_info.num_joints, args=args)

    @mecascript_global_function_decorator
    #pylint: disable=invalid-name
    @overload
    def SetPayload(self, mass: float, x: float, y: float, z: float):
        ...

    @mecascript_global_function_decorator
    @overload
    def SetPayload(self, payload: RobotPayload, /):  # pylint: disable=arguments-differ
        ...

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetPayload(self, mass: Union[float, RobotPayload], x: float = None, y: float = None, z: float = None):
        """
        Sets the robot's payload mass and the center of mass relative to the robot's FRF.

        For more information, see
        [SetPayload](https://www.mecademic.com/res/doc?robot_command=SetPayload).

        Parameters
        ----------
        mass
            The payload mass, in kilograms.
        x
            X coordinate of the payload center of mass relative to the robot's FRF, in millimeters.
        y
            Y coordinate of the payload center of mass relative to the robot's FRF, in millimeters.
        z
            Z coordinate of the payload center of mass relative to the robot's FRF, in millimeters.
        payload
            Alternatively, you can provide RobotPayload as a single argument

        Notes
        -----
        Default values restored at robot activation: 0, 0, 0, 0
        """
        if isinstance(mass, RobotPayload):
            x = mass.x
            y = mass.y
            z = mass.z
            mass = mass.mass
        super().SetPayload(mass, x, y, z)

    @mecascript_global_function_decorator
    @overload
    def SetCalibrationCfg(self, *, enable: bool):
        ...

    @mecascript_global_function_decorator
    @overload
    def SetCalibrationCfg(self, *, enable: CalibrationCfg):
        ...

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetCalibrationCfg(self, *, enable: Union[bool, CalibrationCfg]):
        """
        Enables the more accurate mathematical robot model, identified during the optional
        factory calibration.

        If the robot has not been calibrated at Mecademic, this command has no effect.
        Use ``GetRobotCalibrated`` to check whether the robot is calibrated.

        For more information, see
        [SetCalibrationCfg](https://www.mecademic.com/res/doc?robot_command=SetCalibrationCfg).

        Parameters
        ----------
        enable
            If True, enable the more accurate mathematical robot model; if False, use nominal model.

        Notes
        -----
        This setting is persistent and retains its value even after power cycling the robot.
        After a factory calibration, this setting is enabled.
        """
        if isinstance(enable, CalibrationCfg):
            enable = enable.enabled
        super().SetCalibrationCfg(enable=enable)

    @mecascript_global_function_decorator
    @overload
    def SetPStop2Cfg(self, severity: MxEventSeverity = MxEventSeverity.MX_EVENT_SEVERITY_CLEAR_MOTION):
        ...

    @mecascript_global_function_decorator
    @overload
    def SetPStop2Cfg(self, cfg: Pstop2Cfg, /):  # pylint: disable=arguments-differ
        ...

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetPStop2Cfg(self,
                     severity: Union[MxEventSeverity, Pstop2Cfg] = MxEventSeverity.MX_EVENT_SEVERITY_CLEAR_MOTION):
        """
        Set the behavior of the robot when the SWStop signal in the Meca500 is activated or when the
        PStop2 signal on the other robots is activated.

        For more information, see
        [SetPStop2Cfg](https://www.mecademic.com/res/doc?robot_command=SetPStop2Cfg).

        Parameters
        ----------
        severity
            Severity-level to treat PStop2 events. The allowed values are:

            - ``mx_robot_def.MxEventSeverity.MX_EVENT_SEVERITY_PAUSE_MOTION``
            - ``mx_robot_def.MxEventSeverity.MX_EVENT_SEVERITY_CLEAR_MOTION``
        cfg
            Alternatively, you can provide Pstop2Cfg as a single argument

        Notes
        -----
        This setting is persistent and retains its value even after power cycling the robot.
        """
        if isinstance(severity, Pstop2Cfg):
            severity = severity.severity
        super().SetPStop2Cfg(severity)

    @mecascript_global_function_decorator
    @overload
    def SetSimModeCfg(self, default_sim_mode=MxRobotSimulationMode.MX_SIM_MODE_REAL_TIME):
        ...

    @mecascript_global_function_decorator
    @overload
    def SetSimModeCfg(self, cfg: SimModeCfg, /):  # pylint: disable=arguments-differ
        ...

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetSimModeCfg(self,
                      default_sim_mode: Union[MxRobotSimulationMode,
                                              SimModeCfg] = MxRobotSimulationMode.MX_SIM_MODE_REAL_TIME):
        """
        Sets the default simulation mode type (fast or normal) to use when the command ActivateSim is used without
        the ``mode`` parameter.

        For more information, see
        [SetSimModeCfg](https://www.mecademic.com/res/doc?robot_command=SetSimModeCfg).

        Parameters
        ----------
        default_sim_mode
            Default simulation mode. The available modes are:

            - ``mx_robot_def.MxRobotSimulationMode.MX_SIM_MODE_REAL_TIME``
            - ``mx_robot_def.MxRobotSimulationMode.MX_SIM_MODE_FAST``
        cfg
            Alternatively, you can provide SimModeCfg as a single argument

        Notes
        -----
        This setting is persistent and retains its value even after power cycling the robot.
        The factory default is ``mx_robot_def.MxRobotSimulationMode.MX_SIM_MODE_REAL_TIME``.
        """
        if isinstance(default_sim_mode, SimModeCfg):
            default_sim_mode = default_sim_mode.mode
        super().SetSimModeCfg(default_sim_mode)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetMecaScriptCfg(self, cfg: MecaScriptCfg):
        """
        Sets the robot's MecaScript configuration.

        For more information, see
        [SetMecaScriptCfg](https://www.mecademic.com/res/doc?robot_command=SetMecaScriptCfg&robot_model=MCS500).

        Parameters
        ----------
        cfg
            The MecaScript configuration to apply

        Notes
        -----
        This setting is persistent and retains its value even after power cycling the robot.
        The factory default is the same as the default values from class ``MecaScriptCfg``.
        """
        super().SetMecaScriptCfg(cfg)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def SetRobotName(self, name: str):
        """
        Sets the robot name, as returned by ``GetRobotName``.

        This also updates the hostname advertised by the robot on the local network.

        For more information, see
        [SetRobotName](https://www.mecademic.com/res/doc?robot_command=SetRobotName).

        Parameters
        ----------
        name
            New name to assign to the robot.

        Notes
        -----
        This setting is persistent and retains its value even after power cycling the robot.
        """
        return super().SetRobotName(name)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def ActivateBrakes(self, activated: bool = True):
        """
        Enable or disable the brakes.

        Parameters
        ----------
        activated
            Engage brakes if True, disengage if False.
        """
        super().ActivateBrakes(activated)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def BrakesOn(self):
        """Same as ``ActivateBrakes(activated=True)``"""
        super().ActivateBrakes(True)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def BrakesOff(self):
        """Same as ``ActivateBrakes(activated=False)``"""
        super().ActivateBrakes(False)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def WaitProgramDone(self, *, timeout: Optional[float] = None):
        """
        Waits until no robot MecaScript program is executing, i.e. ``GetProgramExecutionStatus`` reports no
        running program.

        This does not apply to legacy ``.mxprog`` programs.

        Parameters
        ----------
        timeout
            Maximum time to wait, in seconds. If ``None``, no timeout is applied and this function
            blocks until the program finishes or the connection is lost.

        Raises
        ------
        TimeoutException
            If the specified timeout is reached before the program finishes.
        """
        return super().WaitProgramDone(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetPowerSupplyInputs(self, synchronous_update: bool = False, timeout: float = None) -> RobotPowerSupplyInputs:
        """
        Returns the current robot power supply input states.
        Not supported on Meca500 robots.

        Parameters
        ----------
        synchronous_update
            If True, synchronously request updated input states from the robot.
            If False, return the latest known status.
        timeout
            Maximum time to wait for a synchronous response, in seconds (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        RobotPowerSupplyInputs
            Current robot power supply input states.
        """
        return super().GetPowerSupplyInputs(synchronous_update, timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetAutoConf(self, *, timeout: float = None) -> bool:
        """
        Returns the state of automatic posture configuration selection to be observed in the
        ``MovePose`` and ``MoveLin*`` commands.

        For more information, see
        [GetAutoConf](https://www.mecademic.com/res/doc?robot_command=GetAutoConf).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        bool
            True if automatic configuration selection is enabled, False otherwise.
        """
        return super().GetAutoConf(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetAutoConfTurn(self, *, timeout: float = None) -> bool:
        """
        Returns the state of automatic turn configuration selection to be observed in the ``MovePose``
        and ``MoveLin*`` commands.

        For more information, see
        [GetAutoConfTurn](https://www.mecademic.com/res/doc?robot_command=GetAutoConfTurn).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        bool
            True if automatic turn configuration selection is enabled, False otherwise.
        """
        return super().GetAutoConfTurn(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetBlending(self, *, timeout: float = None) -> float:
        """
        Returns the blending percentage, which is set using the ``SetBlending`` command.

        For more information, see
        [GetBlending](https://www.mecademic.com/res/doc?robot_command=GetBlending).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        float
            Percentage of blending.

        """
        return super().GetBlending(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetBrakesState(self, *, timeout: float = None) -> bool:
        """
        Returns the current robot brakes state.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        bool
            True if the brakes are engaged, False otherwise.
        """
        return super().GetBrakesState(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetCalibrationCfg(self, *, timeout: float = None) -> CalibrationCfg:
        """
        Returns the state of the optional robot calibration, configured using the
        ``SetCalibrationCfg`` command.

        For more information, see
        [GetCalibrationCfg](https://www.mecademic.com/res/doc?robot_command=GetCalibrationCfg).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        CalibrationCfg
            Current calibration configuration.
        """
        return super().GetCalibrationCfg(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetCartAcc(self, *, timeout: float = None) -> float:
        """
        Returns the desired limit for the acceleration of the TRF relative to the WRF, set using the
        ``SetCartAcc`` command.

        For more information, see
        [GetCartAcc](https://www.mecademic.com/res/doc?robot_command=GetCartAcc).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        float
            Percentage of the maximum acceleration of the TRF.
        """
        return super().GetCartAcc(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetCartAngVel(self, *, timeout: float = None) -> float:
        """
        Returns the desired limit for the angular velocity of the TRF relative to the WRF, set using
        the ``SetCartAngVel`` command.

        For more information, see
        [GetCartAngVel](https://www.mecademic.com/res/doc?robot_command=GetCartAngVel).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        float
            TRF angular velocity limit, in degrees per second (°/s).
        """
        return super().GetCartAngVel(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetCartLinVel(self, *, timeout: float = None) -> float:
        """
        Returns the desired Tool Center Point (TCP) velocity limit, configured using the
        ``SetCartLinVel`` command.

        For more information, see
        [GetCartLinVel](https://www.mecademic.com/res/doc?robot_command=GetCartLinVel).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        float
            TCP velocity limit, in millimeters per second (mm/s).
        """
        return super().GetCartLinVel(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetCheckpoint(self, *, timeout: float = None) -> int:
        """
        Returns the ID of the last checkpoint reached (as set with ``SetCheckpoint``).

        For more information, see
        [GetCheckpoint](https://www.mecademic.com/res/doc?robot_command=GetCheckpoint).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        int
            ID of the most recently reached checkpoint.
        """
        return super().GetCheckpoint(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetCheckpointDiscarded(self, *, timeout: float = None) -> int:
        """
        Returns the ID of the last checkpoint discarded (as set with ``SetCheckpoint``).

        A checkpoint can be discarded by ``ClearMotion``, ``DeactivateRobot``, or when the robot
        enters an error or safety stop state.

        For more information, see
        [GetCheckpointDiscarded](https://www.mecademic.com/res/doc?robot_command=GetCheckpointDiscarded).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        int
            ID of the most recently reached checkpoint.
        """
        return super().GetCheckpointDiscarded(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetCmdPendingCount(self, *, timeout: float = None) -> int:
        """
        Returns the number of motion commands that are currently in the motion queue.

        For more information, see
        [GetCmdPendingCount](https://www.mecademic.com/res/doc?robot_command=GetCmdPendingCount).

        Notes
        -----
        Note that the robot will compile several (~25) commands in advance. These compiled commands are not included
        in this count, though they may not yet have started executing.

        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        int
            Number of motion commands in the queue.
        """
        return super().GetCmdPendingCount(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetCollisionCfg(self, *, timeout: float = None) -> CollisionCfg:
        """
        Returns the severity level set with the ``SetCollisionCfg`` command.

        For more information, see
        [GetCollisionCfg](https://www.mecademic.com/res/doc?robot_command=GetCollisionCfg).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        CollisionCfg
            The collision configuration (severity level).
        """
        return super().GetCollisionCfg(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetCollisionStatus(self, *, synchronous_update: bool = False, timeout: float = None) -> SelfCollisionStatus:
        """
        Returns the current collision status.

        For more information, see
        [GetCollisionStatus](https://www.mecademic.com/res/doc?robot_command=GetCollisionStatus).

        Parameters
        ----------
        synchronous_update
            If True, synchronously request updated robot status. If False, return the latest known
            status.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        SelfCollisionStatus
            Object containing the current robot collision status

        """
        return super().GetCollisionStatus(synchronous_update=synchronous_update, timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetConf(self, *, timeout: float = None) -> Optional[List[int]]:
        """
        Returns the desired posture configuration that will be applied to the next ``MovePose`` or
        ``MoveLin*`` command in the motion queue.

        For more information, see
        [GetConf](https://www.mecademic.com/res/doc?robot_command=GetConf).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        list of int or None
            Posture configuration values (1 or -1), or None if automatic configuration is enabled.
            The list has one element for 4-axis robots and three elements for 6-axis robots.
        """
        return super().GetConf(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetConfTurn(self, *, timeout: float = None) -> Optional[int]:
        """
        Returns the desired turn configuration for the last joint, i.e., the turn configuration that
        will be applied to the next ``MovePose`` or ``MoveLin*`` command in the motion queue.

        For more information, see
        [GetConfTurn](https://www.mecademic.com/res/doc?robot_command=GetConfTurn).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        int or None
            Turn configuration, or None if automatic turn configuration is enabled.
        """
        return super().GetConfTurn(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetEtherNetIpEnabled(self, *, timeout: float = None) -> bool:
        """
        Returns the state of the Ethernet/IP protocol.

        For more information, see
        [GetEtherNetIpEnabled](https://www.mecademic.com/res/doc?robot_command=GetEtherNetIpEnabled).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        bool
            True if the EtherNet/IP protocol stack is enabled, False otherwise.
        """
        return super().GetEtherNetIpEnabled(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetExtToolFwVersion(self, *, timeout: float = None) -> RobotVersion:
        """
        Return the firmware version of Meca500's EOAT connected to its tool I/O port.

        For more information, see
        [GetExtToolFwVersion](https://www.mecademic.com/res/doc?robot_command=GetExtToolFwVersion).

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        RobotVersion
            The external tool firmware version.
        """
        return super().GetExtToolFwVersion(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetExtToolSim(self, *, timeout: float = None) -> MxExtToolType:
        """
        Returns the model of the Meca500's EOAT currently simulated (or
        ``MxExtToolType.MX_EXT_TOOL_NONE`` if simulation is not enabled).

        For more information, see
        [GetExtToolSim](https://www.mecademic.com/res/doc?robot_command=GetExtToolSim&robot_model=MECA500).

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        MxExtToolType
            Simulated external tool type on the Meca500.
        """
        return super().GetExtToolSim(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetFwVersion(self, *, timeout: float = None) -> RobotVersion:
        """
        Returns the version of the firmware installed on the robot.

        For more information, see
        [GetFwVersion](https://www.mecademic.com/res/doc?robot_command=GetFwVersion).

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        RobotVersion
            Robot's firmware version.
        """
        return super().GetFwVersion(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetGripperForce(self, *, timeout: float = None) -> float:
        """
        Returns the grip force set by the ``SetGripperForce`` command (Meca500 only).

        For more information, see
        [GetGripperForce](https://www.mecademic.com/res/doc?robot_command=GetGripperForce&robot_model=MECA500).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        float
            Grip force limit, as signed percentage of the maximum grip force.
        """
        return super().GetGripperForce(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetGripperRange(self, *, timeout: float = None) -> Tuple[float, float]:
        """
        Returns the gripper range set by the ``SetGripperRange`` command (Meca500 only).

        For more information, see
        [GetGripperRange](https://www.mecademic.com/res/doc?robot_command=GetGripperRange&robot_model=MECA500).

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        tuple of float
            A pair ``(close_pos, open_pos)`` in millimeters, giving the finger openings for the
            closed and open states, respectively.
        """
        return super().GetGripperRange(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetGripperVel(self, *, timeout: float = None) -> float:
        """
        Returns the gripper velocity set by the ``SetGripperVel`` command (Meca500 only).

        For more information, see
        [GetGripperVel](https://www.mecademic.com/res/doc?robot_command=GetGripperVel&robot_model=MECA500).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        float
            Percentage of maximum finger velocity.
        """
        return super().GetGripperVel(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetIoSim(self, *, bank_id: MxIoBankId = MxIoBankId.MX_IO_BANK_ID_IO_MODULE, timeout: float = None) -> bool:
        """
        Returns the state of the simulation of the MVK01 vacuum and I/O module set by the command ``SetIoSim``
        (or the default one).

        For more information, see
        [GetIoSim](https://www.mecademic.com/res/doc?robot_command=GetIoSim&robot_model=MCS500).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        bank_id
            The IO bank ID to enable or disable simulation mode for.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        bool
            True if the specified IO bank is in simulation mode, False otherwise.
        """
        return super().GetIoSim(bank_id=bank_id, timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetJointAcc(self, *, timeout: float = None) -> float:
        """
        Returns the desired joint accelerations reduction factor, set using the ``SetJointAcc``
        command.

        For more information, see
        [GetJointAcc](https://www.mecademic.com/res/doc?robot_command=GetJointAcc).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        float
            Percentage of maximum joint accelerations.
        """
        return super().GetJointAcc(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetJointLimits(self, n: int, *, effective: bool = True, timeout: float = None) -> Tuple[float, float]:
        """
        Returns the current joint limits: either the default limits or the user-defined limits if set
        with ``SetJointLimits`` and enabled with ``SetJointLimitsCfg``.

        For more information, see
        [GetJointLimits](https://www.mecademic.com/res/doc?robot_command=GetJointLimits).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        n
            joint number (1-6)
        effective
            If True, return the effective joint limits. If False, return the configured limits,
            which may not be effective if joint limits are disabled.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        Tuple of float
            Lower and upper limit for the specified joint, in degrees or millimeters.
        """
        return super().GetJointLimits(n, effective=effective, timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetJointLimitsCfg(self, *, timeout: float = None) -> JointLimitsCfg:
        """
        Returns the status of the user-enabled joint limits, defined by the ``SetJointLimitsCfg``.

        For more information, see
        [GetJointLimitsCfg](https://www.mecademic.com/res/doc?robot_command=GetJointLimitsCfg).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot .

        Returns
        -------
        JointLimitsCfg
            Current joint limits configuration.
        """
        return super().GetJointLimitsCfg(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetJointVel(self, *, timeout: float = None) -> float:
        """
        Returns the desired joint velocities reduction factor, set using the ``SetJointVel`` command.

        For more information, see
        [GetJointVel](https://www.mecademic.com/res/doc?robot_command=GetJointVel).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        float
            Percentage of maximum joint velocities.
        """
        return super().GetJointVel(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetJointVelLimit(self, *, timeout: float = None) -> float:
        """
        Returns the desired joint velocities override, set using the ``SetJointVelLimit`` command.

        For more information, see
        [GetJointVelLimit](https://www.mecademic.com/res/doc?robot_command=GetJointVelLimit).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        float
            Percentage of maximum joint velocities override.
        """
        return super().GetJointVelLimit(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetModelJointLimits(self, n: int, *, timeout: float = None) -> Tuple[float, float]:
        """
        Returns the factory default joint limits.

        For more information, see
        [GetModelJointLimits](https://www.mecademic.com/res/doc?robot_command=GetModelJointLimits).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        n
            joint number (1-6)
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        Tuple of float
            Lower and upper limit for the specified joint, in degrees or millimeter.
        """
        return super().GetModelJointLimits(n, timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetMonitoringInterval(self, *, timeout: float = None) -> float:
        """
        Returns the time interval at which real-time feedback from the robot is sent over TCP port
        as configured with ``SetMonitoringInterval``.

        For more information, see
        [GetMonitoringInterval](https://www.mecademic.com/res/doc?robot_command=GetMonitoringInterval).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        float
            Monitoring interval in seconds.

        """
        return super().GetMonitoringInterval(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetMoveDuration(self, *, timeout: float = None) -> float:
        """
        Returns the desired move duration, set using the ``SetMoveDuration`` command.

        For more information, see
        [GetMoveDuration](https://www.mecademic.com/res/doc?robot_command=GetMoveDuration).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        float
            The move duration in seconds.
        """
        return super().GetMoveDuration(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetMoveDurationCfg(self, *, timeout: float = None) -> MoveDurationCfg:
        """
        Returns the desired move duration configuration, set using the ``SetMoveDurationCfg`` command.

        For more information, see
        [GetMoveDurationCfg](https://www.mecademic.com/res/doc?robot_command=GetMoveDurationCfg).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        float
            The current move duration configuration.
        """
        return super().GetMoveDurationCfg(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetMoveJumpApproachVel(self, *, timeout: float = None) -> MoveJumpApproachVel:
        """
        Returns the ``MoveJump`` approach velocity that will be applied to the next ``MoveJump``
        command, as configured by SetMoveJumpApproachVel.

        #pylint: disable=line-too-long
        For more information, see
        [GetMoveJumpApproachVel](https://www.mecademic.com/res/doc?robot_command=GetMoveJumpApproachVel&robot_model=MCS500).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        MoveJumpApproachVel
            The ``MoveJump`` approach velocity configuration as an object
        """
        return super().GetMoveJumpApproachVel(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetMoveJumpHeight(self, *, timeout: float = None) -> MoveJumpHeight:
        """
        Returns the ``MoveJump`` height that will be applied to the next ``MoveJump`` command, as
        configured by ``SetMoveJumpHeight``.

        For more information, see
        [GetMoveJumpHeight](https://www.mecademic.com/res/doc?robot_command=GetMoveJumpHeight&robot_model=MCS500).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        MoveJumpHeight
            The ``MoveJump`` height configuration as an object.
        """
        return super().GetMoveJumpHeight(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetMoveMode(self, *, timeout: float = None) -> MxMoveMode:
        """
        Returns the desired move mode, set using the ``SetMoveMode`` command.

        For more information, see
        [GetMoveMode](https://www.mecademic.com/res/doc?robot_command=GetMoveMode).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        MxMoveMode
            The MoveMode as an object.
        """
        return super().GetMoveMode(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetOperationMode(self, *, timeout: float = None) -> MxRobotOperationMode:
        """
        Returns the current safety operation mode of the robot.

        For more information, see
        [GetOperationMode](https://www.mecademic.com/res/doc?robot_command=GetOperationMode&robot_model=MCS500).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        MxRobotOperationMode
            The operation mode as an object.
        """
        return super().GetOperationMode(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetPStop2Cfg(self, *, timeout: float = None) -> Pstop2Cfg:
        """
        Returns the current configuration of the PStop2 safety function, as set by ``SetPStop2Cfg``.

        For more information, see
        [GetPStop2Cfg](https://www.mecademic.com/res/doc?robot_command=GetPStop2Cfg).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        Pstop2Cfg
            The PStop2 configuration as an object.
        """
        return super().GetPStop2Cfg(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetPayload(self, *, timeout: float = None) -> RobotPayload:
        """
        Returns the payload mass and center of gravity, set using the ``SetPayload`` command.

        For more information, see
        [GetPayload](https://www.mecademic.com/res/doc?robot_command=GetPayload).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        RobotPayload
            The robot payload and center of gravity as an object.
        """
        return super().GetPayload(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetProductType(self, *, timeout: float = None) -> str:
        """
        Returns the type (model) of the connected robot.

        For more information, see
        [GetProductType](https://www.mecademic.com/res/doc?robot_command=GetProductType).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        str
            The robot product type as a string (e.g, "MCS500")
        """
        return super().GetProductType(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetProfinetEnabled(self, *, timeout: float = None) -> bool:
        """
        Returns whether the Profinet protocol is currently enabled on the robot.

        For more information, see
        [GetProfinetEnabled](https://www.mecademic.com/res/doc?robot_command=GetProfinetEnabled).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        bool
            True if the Profinet protocol stack is enabled on the robot, false otherwise.
        """
        return super().GetProfinetEnabled(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRealTimeMonitoring(self, *, timeout: float = None) -> list[MxRobotStatusCode]:
        """
        Returns the numerical codes of the real-time monitoring responses that have been enabled with
        the ``SetRealTimeMonitoring`` command.

        This function does not include the real-time monitoring codes that are always enabled, but
        only the optional ones that are currently active.

        See the ``RobotRtData`` class for details on which real-time monitoring events are optional.

        For more information, see
        [GetRealTimeMonitoring](https://www.mecademic.com/res/doc?robot_command=GetRealTimeMonitoring).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        list of MxRobotStatusCode
            List of optional real-time monitoring codes currently enabled.
        """
        return super().GetRealTimeMonitoring(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRecoveryMode(self, *, timeout: float = None) -> bool:
        """
        Returns whether the recovery mode is enabled.

        For more information, see
        [GetRecoveryMode](https://www.mecademic.com/res/doc?robot_command=GetRecoveryMode).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        bool
            True if the recovery mode is currently enabled, false otherwise.
        """
        return super().GetRecoveryMode(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRobotCalibrated(self, *, timeout: float = None) -> bool:
        """
        Returns whether the robot was calibrated at the factory.
        This does not indicate whether calibration is currently enabled, only whether it can be enabled on this robot.

        For more information, see
        [GetRobotCalibrated](https://www.mecademic.com/res/doc?robot_command=GetRobotCalibrated).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        bool
            True if the robot has been factory calibrated, false otherwise.
        """
        return super().GetRobotCalibrated(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRobotInfo(self) -> RobotInfo:
        """
        Returns the known robot information.

        Return
        ------
        RobotInfo
            Object containing robot information.
        """
        return super().GetRobotInfo()

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRobotName(self, *, timeout: float = None) -> str:
        """
        Returns the robot name as set by ``SetRobotName``.

        For more information, see
        [GetRobotName](https://www.mecademic.com/res/doc?robot_command=GetRobotName).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        str
            The robot name.
        """
        return super().GetRobotName(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRobotSerial(self, *, timeout: float = None) -> str:
        """
        Returns the serial number of the robot.

        For more information, see
        [GetRobotSerial](https://www.mecademic.com/res/doc?robot_command=GetRobotSerial).

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        str
            The robot serial number
        """
        return super().GetRobotSerial(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetRtc(self, *, timeout: float = None) -> int:
        """
        Returns the current epoch time in seconds, normally set by the ``SetRtc`` command after each
        robot reboot.

        For more information, see
        [GetRtc](https://www.mecademic.com/res/doc?robot_command=GetRtc).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        int
            The current value of the robot's real-time clock (RTC).
        """
        return super().GetRtc(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetStatusRobot(self, synchronous_update: bool = False, timeout: float = None) -> RobotStatus:
        """
        Returns the current robot status.

        For more information, see
        [GetStatusRobot](https://www.mecademic.com/res/doc?robot_command=GetStatusRobot).

        Parameters
        ----------
        synchronous_update
            True to retrieve the updated robot status synchronously, false to return the latest
            known status.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        RobotStatus
            Object containing the current robot status.
        """
        return super().GetStatusRobot(synchronous_update, timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetSafetyStatus(self, synchronous_update: bool = False, timeout: float = None) -> RobotSafetyStatus:
        """
        Returns the current robot safety status.

        Parameters
        ----------
        synchronous_update
            True to retrieve the updated robot status synchronously, false to return the latest
            known status.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        RobotSafetyStatus
            Object containing the current robot safety status.
        """
        return super().GetSafetyStatus(synchronous_update, timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetSafetyStopStatus(self,
                            code: MxRobotStatusCode,
                            *,
                            synchronous_update: bool = False,
                            timeout: float = None) -> MxStopState:
        """
        Returns the current state of a specific robot safety stop signal.

        Parameters
        ----------
        code
            Status code corresponding to the safety signal whose state is to be retrieved.
            The following codes are supported:

            - ``MX_ST_PSTOP2``, state of the P-Stop 2 safety stop signal
            - ``MX_ST_PSTOP1``, state of the P-Stop 1 safety stop signal
            - ``MX_ST_ESTOP``, state of the E-Stop safety stop signal
            - ``MX_ST_SAFE_STOP_OPERATION_MODE``, state of the operation mode safety stop signal
            - ``MX_ST_SAFE_STOP_ENABLING_DEVICE_RELEASED``, state of the enabling device released safety stop signal
            - ``MX_ST_SAFE_STOP_VOLTAGE_FLUCTUATION``, state of the voltage fluctuation safety stop signal
            - ``MX_ST_SAFE_STOP_REBOOT``, state of the signal associated with robot reboot or reset
            - ``MX_ST_SAFE_STOP_REDUNDANCY_FAULT``, state of the signal associated with a safety signal mismatch
            - ``MX_ST_SAFE_STOP_STANDSTILL_FAULT``, state of the signal associated with a standstill fault
            - ``MX_ST_SAFE_STOP_CONNECTION_DROPPED``, state of the signal associated with a connection drop
            - ``MX_ST_SAFE_STOP_MINOR_ERROR``, state of the signal associated with a minor error
        synchronous_update
            True to retrieve the updated robot safety status synchronously, false to return the latest known status.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        MxStopState
            The state of the requested safety stop signal.
        """
        return super().GetSafetyStopStatus(code=code, synchronous_update=synchronous_update, timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetLoadedPrograms(self, *, timeout: Optional[float] = None) -> LoadedPrograms:
        """
        Returns the program configuration from the robot, including the list of programs
        marked as loaded.
        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).
        Returns
        -------
        LoadedPrograms
            The robot’s loaded programs configuration.
        """
        return super().GetLoadedPrograms(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetProgramExecutionStatus(self,
                                  *,
                                  synchronous_update: bool = False,
                                  timeout: float = None) -> ProgramExecutionStatus:
        """
        Returns the current execution status of MecaScript programs on the robot, indicating whether a program is being
        executed and other related information.

        #pylint: disable=line-too-long
        For more information, see
        [GetProgramExecutionStatus](https://www.mecademic.com/res/doc?robot_command=GetProgramExecutionStatus&robot_model=MCS500).

        Notes
        -----
        - Programs may appear as "executing" only briefly, since they are usually executed and their
          commands posted to the motion queue very quickly. If you need to determine whether the robot
          is moving according to motion commands posted by a program, use checkpoints to track the
          execution of the robot's motion queue.
        - This function does not apply to basic ``.mxprog`` programs, as their execution is
          instantaneous.

        Parameters
        ----------
        synchronous_update
            True to retrieve the updated status synchronously, false to return the
            latest known status
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        ProgramExecutionStatus
            Program execution status.
        """
        return super().GetProgramExecutionStatus(synchronous_update=synchronous_update, timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetMecaScriptEngineStatus(self,
                                  *,
                                  synchronous_update: bool = False,
                                  timeout: float = None) -> Optional[MecaScriptEngineStatus]:
        """
        Returns the current MecaScript engine status.

        #pylint: disable=line-too-long
        For more information, see
        [GetMecaScriptEngineStatus](https://www.mecademic.com/res/doc?robot_command=GetMecaScriptEngineStatus&robot_model=MCS500).

        Parameters
        ----------
        synchronous_update
            True to retrieve the updated robot status synchronously, false to return the
            latest known status.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        MecaScriptEngineStatus or None
            Execution status of the MecaScript engine, or None if no Python MecaScript engine is running.
        """
        return super().GetMecaScriptEngineStatus(synchronous_update=synchronous_update, timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetSimModeCfg(self, *, timeout: float = None) -> SimModeCfg:
        """
        Returns the default simulation mode configured with the ``SetSimModeCfg`` command.

        For more information, see
        [GetSimModeCfg](https://www.mecademic.com/res/doc?robot_command=GetSimModeCfg).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        SimModeCfg
            The current simulation mode configuration.
        """
        return super().GetSimModeCfg(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetMecaScriptCfg(self, *, timeout: float = None) -> MecaScriptCfg:
        """
        Returns the current robot's MecaScript configuration.

        For more information, see
        [GetMecaScriptCfg](https://www.mecademic.com/res/doc?robot_command=GetMecaScriptCfg&robot_model=MCS500).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        MecaScriptCfg
            The current MecaScript configuration.
        """
        return super().GetMecaScriptCfg(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetTimeScaling(self, *, timeout: float = None) -> float:
        """
        Returns the current trajectory generation time-scaling factor, in percent.

        For more information, see
        [GetTimeScaling](https://www.mecademic.com/res/doc?robot_command=GetTimeScaling).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        float
            The current time-scaling factor, in percent.
        """
        return super().GetTimeScaling(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetToolSphere(self, *, timeout: float = None) -> ToolSphere:
        """
        Returns the current parameters of the bounding sphere representing the tool, as configured by
        the ``SetToolSphere`` command.

        For more information, see
        [GetToolSphere](https://www.mecademic.com/res/doc?robot_command=GetToolSphere).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        ToolSphere
            The tool sphere parameters
        """
        return super().GetToolSphere(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetTorqueLimits(self, *, timeout: float = None) -> List[float]:
        """
        Returns the current joint torque thresholds, as set by the ``SetTorqueLimits`` command.

        For more information, see
        [GetTorqueLimits](https://www.mecademic.com/res/doc?robot_command=GetTorqueLimits).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        list of float
            List containing the current torque limits, in percent, for each joint.
        """
        return super().GetTorqueLimits(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetTorqueLimitsCfg(self, *, timeout: float = None) -> TorqueLimitsCfg:
        """
        Returns the desired robot behavior when any joint torque exceeds the thresholds set by the
        ``SetTorqueLimits`` command.

        For more information, see
        [GetTorqueLimitsCfg](https://www.mecademic.com/res/doc?robot_command=GetTorqueLimitsCfg).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        TorqueLimitsCfg
            The torque limits configuration.
        """
        return super().GetTorqueLimitsCfg(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetTorqueLimitsStatus(self, *, synchronous_update: bool = False, timeout: float = None) -> TorqueLimitsStatus:
        """
        Returns the status of the torque limits, indicating whether any torque limit is currently
        exceeded.

        For more information, see
        [GetTorqueLimitsStatus](https://www.mecademic.com/res/doc?robot_command=GetTorqueLimitsStatus).

        Parameters
        ----------
        synchronous_update
            True to retrieve the updated status synchronously, false to return the
            latest known status.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        TorqueLimitsStatus
            The current torque limits status.

        """
        return super().GetTorqueLimitsStatus(synchronous_update=synchronous_update, timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetVacuumPurgeDuration(self, *, timeout: float = None) -> float:
        """
        Returns the vacuum purge duration that will apply when the ``VacuumRelease`` command is used,
        as set by the ``SetVacuumPurgeDuration`` command.

        #pylint: disable=line-too-long
        For more information, see
        [GetVacuumPurgeDuration](https://www.mecademic.com/res/doc?robot_command=GetVacuumPurgeDuration&robot_model=MCS500).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        float
            The vacuum purge duration in seconds.
        """
        return super().GetVacuumPurgeDuration(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetVacuumThreshold(self, *, timeout: float = None) -> VacuumThreshold:
        """
        Returns the vacuum hold and release pressure thresholds that will apply when the
        ``VacuumGrip`` command is used, as set by the ``SetVacuumThreshold`` command.

        For more information, see
        [GetVacuumThreshold](https://www.mecademic.com/res/doc?robot_command=GetVacuumThreshold&robot_model=MCS500).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        VacuumThreshold
            The vacuum thresholds.
        """
        return super().GetVacuumThreshold(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetVelTimeout(self, *, timeout: float = None) -> float:
        """
        Returns the timeout for velocity-mode motion commands, as set by the ``SetVelTimeout`` command.

        For more information, see
        [GetVelTimeout](https://www.mecademic.com/res/doc?robot_command=GetVelTimeout).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        float
            The velocity timeout in seconds.
        """
        return super().GetVelTimeout(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetWorkZoneCfg(self, *, timeout: float = None) -> WorkZoneCfg:
        """
        Returns the current severity and mode used to handle work zone events, as set by the ``SetWorkZoneCfg`` command.

        For more information, see
        [GetWorkZoneCfg](https://www.mecademic.com/res/doc?robot_command=GetWorkZoneCfg).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        WorkZoneCfg
            The work zone configuration parameters.
        """
        return super().GetWorkZoneCfg(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetWorkZoneLimits(self, *, timeout: float = None) -> WorkZoneLimits:
        """
        Returns the current boundary values of the work zone, as set by the
        ``SetWorkZoneLimits`` command.

        For more information, see
        [GetWorkZoneLimits](https://www.mecademic.com/res/doc?robot_command=GetWorkZoneLimits).

        Notes
        -----
        This function always queries the robot synchronously; the ``Robot`` class does not return
        a cached value.

        Parameters
        ----------
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        WorkZoneLimits
            The work zone limits parameters.
        """
        return super().GetWorkZoneLimits(timeout=timeout)

    @mecascript_global_function_decorator
    @disconnect_on_exception_decorator
    def GetWorkZoneStatus(self, *, synchronous_update: bool = False, timeout: float = None) -> WorkZoneStatus:
        """
        Returns the current robot work zone status.

        For more information, see
        [GetWorkZoneStatus](https://www.mecademic.com/res/doc?robot_command=GetWorkZoneStatus).

        Parameters
        ----------
        synchronous_update
            True to retrieve the updated status synchronously, false to return the
            latest known status.
        timeout
            Maximum time in seconds to wait for a synchronous response from the robot (Default is DEFAULT_WAIT_TIMEOUT).

        Returns
        -------
        WorkZoneStatus
            The current robot work zone status.
        """
        return super().GetWorkZoneStatus(synchronous_update=synchronous_update, timeout=timeout)

    @mecascript_global_function_decorator
    def LogTrace(self, trace: str, level: Optional[int] = None):
        """
        Inserts a comment into the user and robot logs. Useful for debugging, allowing you to
        show our support team where exactly a certain event occurs.

        For more information, see
        [LogTrace](https://www.mecademic.com/res/doc?robot_command=LogTrace).

        Parameters
        ----------
        trace
            Text string to print in robot's internal log file
        level
           Python standard local logging level (e.g., logging.DEBUG, logging.INFO, logging.ERROR).
            If None, the message will not be printed locally (i.e., only on the robot).
        """
        super().LogTrace(trace, level)

    @mecascript_global_function_decorator
    def LogUserCommands(self, enable: bool, *, show_motion_queue_progress: bool = False):
        """
        Enables or disables the logging of API commands received by the robot, the corresponding
        responses sent by the robot, and the start of motion command execution.

        For more information, see
        [LogUserCommands](https://www.mecademic.com/res/doc?robot_command=LogUserCommands).

        Parameters
        ----------
        enable
            Enable or disable logging of received API commands and sent responses
        show_motion_queue_progress
            When `enable` is True, also log additional motion-queue details in both the robot and
            user log files, such as command compilation and execution progress.
        """
        super().LogUserCommands(enable=enable, show_motion_queue_progress=show_motion_queue_progress)

    # pylint: disable=redefined-outer-name
    def StartLogging(self,
                     monitoringInterval: float,
                     file_name: str = None,
                     file_path: str = None,
                     fields: List[Union[MxRobotStatusCode, str]] = None,
                     record_time: bool = True):
        """
        Starts logging the robot state and real-time data to a ZIP file containing:

          - a JSON file with a summary of robot information and commands sent to the robot
          - a CSV file with the real-time data received from the robot

        The file path used for logging can be retrieved with ``GetCapturedTrajectoryPath``.

        This function internally calls ``SetRealTimeMonitoring`` to enable the specified real-time
        data fields and ``SetMonitoringInterval()`` to configure the requested monitoring interval.

        Using ``WaitIdle`` before ``StartLogging`` may be useful if you want the captured data to
        include only the subsequent motion commands.

        Parameters
        ----------
        monitoring_interval
            Rate at which state data is received from the robot via the monitor port, in seconds.

        file_name
            Name of the log file.
            If ``None``, the file name will be generated automatically based on the current
            date/time and robot information (type, serial number, and firmware version).

        file_path
            Path where the ZIP file containing the logged data will be saved.
            If not provided, the file will be stored in the working directory.
            The actual file path can be retrieved using ``GetCapturedTrajectoryPath`` or
            as the return value of ``EndLogging``.

        fields
            List of event IDs or field names to capture among the robot real-time data
            status codes (``MxRobotStatusCode.MX_ST_RT_*``).
            Example::

                ``fields=[MxRobotStatusCode.MX_ST_RT_TARGET_JOINT_POS,
                          MxRobotStatusCode.MX_ST_RT_TARGET_CART_POS]``

            Use ``None`` to log all available real-time data.
            You can search ``rt_data_field_by_name`` to get the status code corresponding
            to the desired ``RobotRtData`` field, or ``rt_data_field_by_status_code`` to
            obtain the resulting column name prefix in the captured file.
            Alternatively, provide field names as strings corresponding to the
            ``RobotRtData`` attributes or CSV column name prefixes.
            See ``ROBOT_RT_DATA_FIELDS`` for a complete list of available fields.

        record_time
            If True, the current date and time will be recorded in the log file.
        """
        super().StartLogging(monitoringInterval, file_name, file_path, fields, record_time)

    def EndLogging(self, keep_captured_trajectory: bool = False) -> str:
        """
        Stops logging the robot real-time data to file.

        The file path used for logging can be retrieved with ``GetCapturedTrajectoryPath``,
        or from the return value of this function.

        Parameters
        ----------
        keep_captured_trajectory
            If True, keeps a copy of the captured trajectory that can later be accessed via
            ``GetCapturedTrajectory().get_captured_data()``.

        Returns
        -------
        str
            Name of the ZIP file that contains the robot information and captured trajectory.
        """

        return super().EndLogging(keep_captured_trajectory)

    def GetCapturedTrajectory(self) -> Optional[RobotTrajectories]:
        """
        Returns the most recent robot trajectory captured using the ``StartLogging`` or
        ``FileLogger`` functions.

        To obtain the captured data as a Pandas DataFrame, call ``get_captured_data`` on the returned
        ``RobotTrajectories`` object.

        Examples
        --------
            captured_df = GetCapturedTrajectory().get_captured_data()

        Returns
        -------
        RobotTrajectories or None
            Object containing the robot information and captured trajectory data, or None if no
            trajectory has been captured.
        """
        return super().GetCapturedTrajectory()

    def GetCapturedTrajectoryPath(self) -> Optional[str]:
        """
        Returns the path to the most recent robot trajectory captured using the ``StartLogging``
        or ``FileLogger`` functions.

        Returns
        -------
        str
            The file path to the ZIP file containing the captured trajectory and robot information.
        """
        return super().GetCapturedTrajectoryPath()

    #pylint: disable=invalid-name
    @contextlib.contextmanager
    def FileLogger(self,
                   monitoringInterval: float,
                   file_name: str = None,
                   file_path: str = None,
                   fields: list[str] = None,
                   record_time: bool = True,
                   keep_captured_trajectory: bool = False):
        """
        Context manager interface for the file logger.

        See ``StartLogging`` for detailed parameter descriptions.

        Parameters
        ----------
        monitoring_interval
            See ``StartLogging``.
        file_name
            See ``StartLogging``.
        file_path
            See ``StartLogging``.
        fields
            See ``StartLogging``.
        record_time
            See ``StartLogging``.
        keep_captured_trajectory
            If True, keeps a copy of the captured trajectory that can later be accessed via
            ``GetCapturedTrajectory()``.
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

    @mecascript_global_function_decorator
    def TcpDump(self, *, duration: float):
        """
        Starts an Ethernet capture (PCAP file format) on the robot, for the specified duration. The
        Ethernet capture will be part of the logs archive, which can be retrieved from the MecaPortal.

        For more information, see
        [TcpDump](https://www.mecademic.com/res/doc?robot_command=TcpDump).

        Parameters
        ----------
        duration
            Desired duration of the capture, in seconds.
        """
        super().TcpDump(duration=duration)

    @mecascript_global_function_decorator
    def TcpDumpStop(self):
        """
        Stops the Ethernet capture initiated by TcpDump before it reaches the specified duration.

        For more information, see
        [TcpDumpStop](https://www.mecademic.com/res/doc?robot_command=TcpDumpStop).
        """
        super().TcpDumpStop()

    UPDATE_TIMEOUT = 15 * 60  # 15 minutes timeout

    def UpdateRobot(self, firmware: Union[str, pathlib.Path], timeout: float = UPDATE_TIMEOUT):
        """
        Installs a new firmware and verifies the robot version afterward.

        Parameters
        ----------
        firmware
            Path to the robot firmware file.

        timeout
            Maximum time in seconds allowed for the firmware update (Default is 900).
        """
        return super().UpdateRobot(firmware, timeout)

    @mecascript_global_function_decorator
    def CreateVariable(self,
                       name: str,
                       value: any,
                       cyclic_id: Optional[int] = None,
                       volatile: bool = False,
                       override: bool = False,
                       timeout: Optional[float] = 10):
        """
        Creates a variable on the robot that persists even after a reboot.

        A variable is defined by its case-sensitive name, a value (of any supported type),
        and an optional cyclic ID.

        For more information, see
        [CreateVariable](https://www.mecademic.com/res/doc?robot_command=CreateVariable).

        Notes
        -----
        The variable will be added to the local namespace (`robot.variables`) only after
        the robot has acknowledged this request.
        Be aware of this if using the function asynchronously.
        Use ``SyncCmdQueue`` if needed, or specify a timeout to make the function blocking.

        You can check if a variable already exists before calling this function:
            `robot.variables.get("myvar") is not None`

        See the description of the `override` argument below for details on how the function
        behaves when a variable with the same name already exists.

        There are several ways to access a created variable:

            - `robot.variables.myvar` returns the variable value only
            - `robot.variables.get("myvar")` returns a ``RegisteredVariable`` containing
              value, cyclic ID, and other information
            - `robot.GetVariable("myvar")` returns a ``RegisteredVariable`` containing
              value, cyclic ID, and other information

        Parameters
        ----------
        name
            Name of the variable to create.
        value
            Value to assign to the variable. The type is inferred automatically.
        cyclic_id
            ID to reference the variable in cyclic protocols (``0`` or ``None`` to ignore).
        volatile
            A volatile variable is not saved on robot's disk and is discarded when the robot reboots.
            A non-volatile variable is saved on the robot's disk and will remain permanently available until
            explicitly deleted using ``DeleteVariable``.
        override
            Defines the behavior when a variable with the same name already exists:

              - ``True``:  Updates the value and cyclic ID with the new ones.
              - ``False``: Raises an error if the existing variable has a different type
                           or cyclic ID; otherwise leaves the variable unchanged.
        timeout
            Time in seconds to wait for the robot to confirm variable creation. If None:

              - In synchronous API mode, a default timeout is used  (Default is 10).
              - In asynchronous mode, the function is non-blocking (success is not confirmed).

        Raises
        ------
        ArgErrorException
            If the robot rejects the variable creation (for example, due to a name or
            cyclic ID conflict).
        """
        super().CreateVariable(name, value, cyclic_id, volatile, override, timeout)

    def CreateRegisteredVariable(self,
                                 var: mecascript.RegisteredVariable,
                                 override: bool = False,
                                 timeout: Optional[float] = None) -> bool:
        """ Same as ``CreateVariable``, but taking ``RegisteredVariable`` object instead of separate
        (value, cyclic_id)
        """
        return super().CreateVariable(name=var.name,
                                      value=var.get_value(),
                                      cyclic_id=var.cyclic_id,
                                      volatile=var.volatile,
                                      override=override,
                                      timeout=timeout)

    @mecascript_global_function_decorator
    def DeleteVariable(self, name: str, timeout: Optional[float] = None):
        """
        Deletes a variable from the robot's permanently saved variables.

        For more information, see
        [DeleteVariable](https://www.mecademic.com/res/doc?robot_command=DeleteVariable).

        Notes
        -----
        The variable will be removed from the local namespace (`robot.variables`) only after the
        robot has acknowledged this request. Be aware of this if using the function asynchronously.
        Use ``SyncCmdQueue`` if needed, or specify a timeout to make the function blocking.

        You can check if a variable exists before calling this function:
            `robot.variables.get(name) is not None`

        Parameters
        ----------
        name
            Name of the variable to delete.
        timeout
            Time in seconds to wait for the robot to confirm variable deletion.
            If ``None``:

              - In synchronous API mode, a default timeout is used.
              - In asynchronous mode, the function is non-blocking (success is not confirmed).

        Raises
        ------
        NotFoundException
            If the variable does not exist on the robot.
        """
        super().DeleteVariable(name, timeout)

    @mecascript_global_function_decorator
    def SetVariable(self, name: str, value: any, timeout: Optional[float] = 10):
        """
        Sets the value of a variable that was previously created on the robot using
        ``CreateVariable()``.

        This function can operate synchronously or asynchronously (see the `timeout` argument
        description below).

        For more information, see
        [SetVariable](https://www.mecademic.com/res/doc?robot_command=SetVariable).

        Notes
        -----
        There are alternative ways to set a variable that are always synchronous (waiting for the
        robot's confirmation):

          - Directly set the corresponding attribute in `robot.variables`:
                `robot.variables.my_var = "my_value"`
          - Use the ``set()`` method of `robot.variables`, which also returns the previous value:
                `prev_val = robot.variables.set("my_var", "my_value")`

        Parameters
        ----------
        name
            Name of the variable to modify.
        value
            New value to assign to this variable. The type is deduced automatically. The new value's
            type must match the existing variable's type; otherwise, the robot will return an error.
        timeout
            Time in seconds to wait for the robot to confirm the variable modification.
            If None:

              - In synchronous API mode, a default timeout is used (Default is 10).
              - In asynchronous mode, the function is non-blocking (success is not confirmed).

        Raises
        ------
        ArgErrorException
            If the robot rejects the variable update (for example, due to a type mismatch
            or name conflict).
        """
        super().SetVariable(name, value, timeout)

    @mecascript_global_function_decorator
    def GetVariable(self, name: str) -> Optional[mecascript.RegisteredVariable]:
        """
        Retrieves a variable by name.

        This is a non-blocking function that searches locally in the already synchronized variables
        map.

        It is equivalent to accessing the variable directly from `robot.variables`.
        For example::

            `robot.variables.my_var`  returns the variable value
            `robot.variables.get("my_var")`  returns an ``RegisteredVariable``

        For more information, see
        [GetVariable](https://www.mecademic.com/res/doc?robot_command=GetVariable).

        Parameters
        ----------
        name
            Name of the variable to retrieve.

        Returns
        -------
        mecascript.RegisteredVariable or None
            The found variable, or None if no variable exists with that name.
        """
        return super().GetVariable(name)

    @mecascript_global_function_decorator
    def GetVariableByCyclicId(self, cyclic_id: int) -> Optional[mecascript.RegisteredVariable]:
        """
        Retrieves a variable by its cyclic ID.

        This is a non-blocking function that searches locally in the already synchronized variables
        map.

        Parameters
        ----------
        cyclic_id
            Cyclic ID of the variable to search for.

        Returns
        -------
        mecascript.RegisteredVariable or None
            The found variable, or ``None`` if no variable exists with that cyclic ID.
        """
        return super().GetVariableByCyclicId(cyclic_id)

    @mecascript_global_function_decorator
    def ListVariables(self) -> List[str]:
        """
        Returns the list of robot variables.

        This is a non-blocking function that returns a list of all variable names currently
        synchronized with the robot.

        For more information, see
        [ListVariables](https://www.mecademic.com/res/doc?robot_command=ListVariables).
        """
        return super().ListVariables()

    @mecascript_global_function_decorator
    def EnableEtherNetIp(self, enable: bool = True):
        """
        Enables or disables the EtherNet/IP protocol stack on the robot.

        This command modifies the robot's persistent configuration, which remains effective after a
        reboot.

        For more information, see
        [EnableEtherNetIp](https://www.mecademic.com/res/doc?robot_command=EnableEtherNetIp).

        Parameters
        ----------
        enable
            Enable or disable the EtherNet/IP stack.
        """
        return super().EnableEtherNetIp(enable)

    @mecascript_global_function_decorator
    def EnableProfinet(self, enable: bool = True):
        """
        Enables or disables the PROFINET protocol stack on the robot.

        This command modifies the robot's persistent configuration, which remains effective after a
        reboot.

        For more information, see
        [EnableProfinet](https://www.mecademic.com/res/doc?robot_command=EnableProfinet).

        Parameters
        ----------
        enable
            Enable or disable the PROFINET stack.
        """
        return super().EnableProfinet(enable)

    @mecascript_global_function_decorator
    def SwitchToEtherCAT(self, *, station_alias: Optional[int] = None):
        """
        Disables the Ethernet TCP/IP, EtherNet/IP, and PROFINET protocols, and enables EtherCAT.

        To disable EtherCAT, set the `DisableEtherCAT` bit in the EtherCAT RobotControl output PDO
        (see the robot's programming manual for details), or perform a robot network reset
        (see the user manual).

        This command modifies the robot's persistent configuration, which remains effective
        after a reboot.

        For more information, see
        [SwitchToEtherCat](https://www.mecademic.com/res/doc?robot_command=SwitchToEtherCat).

        Parameters
        ----------
        station_alias
            EtherCAT station alias to update in the robot's EtherCAT EEPROM.
            If None, the existing station alias remains unchanged.
            This argument is ignored (not supported) on Meca500 robots.
        """
        return super().SwitchToEtherCAT(station_alias=station_alias)

    #####################################################################################
    # Private methods.
    #####################################################################################
    def _get_registered_attr(self, attr_name: str) -> any:
        """
        Callback for retrieving an attribute from the local namespace.
        This function is currently used to detect whether a registered command would override an
        existing public API command of this class.
        """
        # Note: DO NOT CALL super()._get_registered_attr() because here we really want to get an attribute
        #       of this derived class (for example, SetBlending or any command that does not exist on the base class).
        return getattr(self, attr_name, None)

    def _set_registered_attr(self, attr_name: str, attr_val: any):
        """ Callback for setting an attribute from the local namespace. """
        # Set in both parent class' namespace
        super()._set_registered_attr(attr_name, attr_val)
        setattr(Robot, attr_name, attr_val)

    def _del_registered_attr(self, attr_name: str):
        """ Callback for deleting an attribute from the local namespace. """
        # Delete from both parent class' namespace
        super()._del_registered_attr(attr_name)
        if getattr(self, attr_name, None) is not None:
            delattr(Robot, attr_name)
