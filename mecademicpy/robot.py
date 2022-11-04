from __future__ import annotations

import contextlib
import copy
import math
import pathlib
from typing import Optional, Tuple, Union

import deprecation
import pkg_resources

import mecademicpy._robot_trajectory_logger as mx_traj

from ._robot_base import _Robot, disconnect_on_exception
from .mx_robot_def import *
from .robot_classes import *
from .robot_trajectory_files import RobotTrajectories
from .tools import *

__version__ = pkg_resources.get_distribution('mecademicpy').version


class Robot(_Robot):
    """Class for controlling a Mecademic robot.
       See README.md for quick instructions on how to use this class."""

    def __init__(self):
        super().__init__()

    def RegisterCallbacks(self, callbacks: RobotCallbacks, run_callbacks_in_separate_thread: bool):
        """Register callback functions to be executed when corresponding event occurs.
           Callback functions are optional.

        Parameters
        ----------
        callbacks : RobotCallbacks object
            Object containing all callback functions.
        run_callbacks_in_separate_thread : bool
            If true, callbacks are run automatically in thread. If false, RunCallbacks must be used.
            **Running callbacks in a separate thread means the user application MUST BE THREAD SAFE!**
        """
        return super().RegisterCallbacks(callbacks, run_callbacks_in_separate_thread)

    def UnregisterCallbacks(self):
        """Unregister callback functions and terminate callback handler thread if applicable.

        """
        return super().UnregisterCallbacks()

    def RunCallbacks(self):
        """Run all triggered callback functions.
           Calling this function is required only when RegisterCallback option run_callbacks_in_separate_thread
           has been set to False (when True, callbacks are automatically called from a background thread).

        """
        if self._callback_thread:
            raise InvalidStateError(
                'Cannot call RunCallbacks since callback handler is already running in separate thread.')

        # Setting timeout=0 means we don't block on an empty queue.
        self._handle_callbacks(self.logger, self._callback_queue, self._robot_callbacks, timeout=0)

    def Connect(self,
                address: str = MX_DEFAULT_ROBOT_IP,
                enable_synchronous_mode: bool = False,
                disconnect_on_exception: bool = True,
                monitor_mode: bool = False,
                offline_mode: bool = False,
                timeout=0.1):
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
        offline_mode : bool
            If true, socket connections are not created, only used for testing.

        """
        return super().Connect(address, enable_synchronous_mode, disconnect_on_exception, monitor_mode, offline_mode,
                               timeout)

    def Disconnect(self):
        """Disconnects Mecademic Robot object from the Mecademic robot.
           This function is synchronous (awaits for disconnection or timeout) even when connected in asynchronous mode.

        """
        return super().Disconnect()

    def IsConnected(self) -> bool:
        """Tells if we're actually connected to the robot"""
        return super().IsConnected()

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

           Note that disabling synchronous mode will not not awake thread already awaiting on synchronous operations.

        Parameters
        ----------
        sync_mode : bool, optional
            Synchronous mode enabled (else asynchronous mode), by default True
        """
        self._enable_synchronous_mode = sync_mode

    @disconnect_on_exception
    def ActivateRobot(self):
        """Activate the robot."""
        with self._main_lock:
            self._check_internal_states()
            self._send_command('ActivateRobot')

        if self._enable_synchronous_mode:
            self.WaitActivated()

    def DeactivateRobot(self):
        """Deactivate the robot."""
        return super().DeactivateRobot()

    @disconnect_on_exception
    def Home(self):
        """Home the robot."""
        with self._main_lock:
            self._check_internal_states()
            self._send_command('Home')

        if self._enable_synchronous_mode:
            self.WaitHomed()

    @disconnect_on_exception
    def ActivateAndHome(self):
        """Utility function that combines activate and home."""
        self.ActivateRobot()
        self.Home()

    @disconnect_on_exception
    def PauseMotion(self):
        """Immediately pause robot motion. """
        with self._main_lock:
            self._check_internal_states()
            self._send_command('PauseMotion')

        if self._enable_synchronous_mode:
            self._robot_events.on_motion_paused.wait(timeout=self.default_timeout)

    @disconnect_on_exception
    def ResumeMotion(self):
        """Un-pause robot motion."""
        with self._main_lock:
            self._check_internal_states()
            self._send_command('ResumeMotion')

        if self._enable_synchronous_mode:
            self.WaitMotionResumed(timeout=self.default_timeout)

    @disconnect_on_exception
    def ClearMotion(self):
        """Clear the motion queue, includes implicit PauseMotion command."""
        with self._main_lock:
            self._check_internal_states()

            # Increment the number of pending ClearMotion requests.
            self._clear_motion_requests += 1
            self._robot_events.on_motion_cleared.clear()

            self._send_command('ClearMotion')

        if self._enable_synchronous_mode:
            self.WaitMotionCleared(timeout=self.default_timeout)

    @disconnect_on_exception
    def MoveJoints(self, *args: float):
        """Move the robot by specifying each joint's target angular position.

        Parameters
        ----------
        joint_1...joint_n : float
            Desired joint angles in degrees.

        """
        if len(args) != self._robot_info.num_joints:
            raise ValueError('Incorrect number of joints sent to command.')

        self._send_motion_command('MoveJoints', args)

    @disconnect_on_exception
    def MoveJointsRel(self, *args: float):
        """Move the robot relative to current position by specifying each joint's offset angular position.

        Parameters
        ----------
        joint_1...joint_n : float
            Desired joint angles offsets in degrees.

        """
        if len(args) != self._robot_info.num_joints:
            raise ValueError('Incorrect number of joints sent to command.')

        self._send_motion_command('MoveJointsRel', args)

    @disconnect_on_exception
    def MoveJointsVel(self, *args: float):
        """Moves joints at desired velocities.

        Parameters
        ----------
        joint_1...joint_n : float
            Desired joint velocities in degrees per second.

        """
        if len(args) != self._robot_info.num_joints:
            raise ValueError('Incorrect number of joints sent to command.')

        self._send_motion_command('MoveJointsVel', args)

    @disconnect_on_exception
    def MovePose(self, x: float, y: float, z: float, alpha: float, beta: float, gamma: float):
        """Move robot's tool to an absolute Cartesian position (non-linear move, but all joints arrive simultaneously).

        Parameters
        ----------
        x, y, z : float
            Desired end effector coordinates in mm.
        alpha, beta, gamma
            Desired end effector orientation in degrees.

        """
        self._send_motion_command('MovePose', [x, y, z, alpha, beta, gamma])

    @disconnect_on_exception
    def MoveLin(self, x: float, y: float, z: float, alpha: float, beta: float, gamma: float):
        """Linearly move robot's tool to an absolute Cartesian position.

        Parameters
        ----------
        x, y, z : float
            Desired end effector coordinates in mm.
        alpha, beta, gamma
            Desired end effector orientation in degrees.

        """
        self._send_motion_command('MoveLin', [x, y, z, alpha, beta, gamma])

    @disconnect_on_exception
    def MoveLinRelTrf(self, x: float, y: float, z: float, alpha: float, beta: float, gamma: float):
        """Linearly move robot's tool to a Cartesian position relative to current TRF position.

        Parameters
        ----------
        x, y, z : float
            Desired displacement in mm.
        alpha, beta, gamma
            Desired orientation change in deg.

        """
        self._send_motion_command('MoveLinRelTrf', [x, y, z, alpha, beta, gamma])

    @deprecation.deprecated(deprecated_in="1.2.0",
                            removed_in="1.3.0",
                            current_version=__version__,
                            details="Use the 'MoveLinRelTrf' function instead")
    @disconnect_on_exception
    def MoveLinRelTRF(self, x: float, y: float, z: float, alpha: float, beta: float, gamma: float):
        """Deprecated use MoveLinRelTrf instead.
        """
        self.MoveLinRelTrf(x, y, z, alpha, beta, gamma)

    @disconnect_on_exception
    def MoveLinRelWrf(self, x: float, y: float, z: float, alpha: float, beta: float, gamma: float):
        """Linearly move robot's tool to a Cartesian position relative to a reference frame that has the same
        orientation.

        Parameters
        ----------
        x, y, z : float
            Desired displacement in mm.
        alpha, beta, gamma
            Desired orientation change in deg.

        """
        self._send_motion_command('MoveLinRelWrf', [x, y, z, alpha, beta, gamma])

    @deprecation.deprecated(deprecated_in="1.2.0",
                            removed_in="1.3.0",
                            current_version=__version__,
                            details="Use the 'MoveLinRelWrf' function instead")
    @disconnect_on_exception
    def MoveLinRelWRF(self, x: float, y: float, z: float, alpha: float, beta: float, gamma: float):
        """Deprecated use MoveLinRelWrf instead.
        """
        self.MoveLinRelWrf(x, y, z, alpha, beta, gamma)

    @disconnect_on_exception
    def MoveLinVelTrf(self, x: float, y: float, z: float, alpha: float, beta: float, gamma: float):
        """Move robot's by Cartesian velocity relative to the TRF.

           Joints will move for a time controlled by velocity timeout (SetVelTimeout).

        Parameters
        ----------
        x, y, z : float
            Desired velocity in mm/s.
        alpha, beta, gamma
            Desired angular velocity in degrees/s.

        """
        self._send_motion_command('MoveLinVelTrf', [x, y, z, alpha, beta, gamma])

    @deprecation.deprecated(deprecated_in="1.2.0",
                            removed_in="1.3.0",
                            current_version=__version__,
                            details="Use the 'MoveLinVelTrf' function instead")
    @disconnect_on_exception
    def MoveLinVelTRF(self, x: float, y: float, z: float, alpha: float, beta: float, gamma: float):
        """Deprecated use MoveLinVelTrf instead

        """
        self.MoveLinVelTrf(x, y, z, alpha, beta, gamma)

    @disconnect_on_exception
    def MoveLinVelWrf(self, x: float, y: float, z: float, alpha: float, beta: float, gamma: float):
        """Move robot's by Cartesian velocity relative to the WRF.

           Joints will move for a time controlled by velocity timeout (SetVelTimeout).

        Parameters
        ----------
        x, y, z : float
            Desired velocity in mm/s.
        alpha, beta, gamma
            Desired angular velocity in degrees/s.

        """
        self._send_motion_command('MoveLinVelWrf', [x, y, z, alpha, beta, gamma])

    @deprecation.deprecated(deprecated_in="1.2.0",
                            removed_in="1.3.0",
                            current_version=__version__,
                            details="Use the 'MoveLinVelWrf' function instead")
    @disconnect_on_exception
    def MoveLinVelWRF(self, x: float, y: float, z: float, alpha: float, beta: float, gamma: float):
        """Deprecated use MoveLinVelWrf instead

        """
        self.MoveLinVelWrf(x, y, z, alpha, beta, gamma)

    @disconnect_on_exception
    def SetVelTimeout(self, t: float):
        """Maximum time the robot will continue to move after a velocity move command was sent.

        (Can be stopped earlier by sending a velocity command with 0 velocity values.)

        Parameters
        ----------
        t : float
            Desired duration for velocity-mode motion commands.

        """
        self._send_motion_command('SetVelTimeout', [t])

    @disconnect_on_exception
    def SetConf(self, shoulder: int, elbow: int, wrist: int):
        """Manually set inverse kinematics options (and disable auto-conf).

        Parameters
        ----------
        shoulder : +1 or -1
            Shoulder inverse kinematics parameter.
        elbow : +1 or -1
            Elbow inverse kinematics parameter.
        wrist : +1 or -1
            Wrist inverse kinematics parameter.

        """
        self._send_motion_command('SetConf', [shoulder, elbow, wrist])

    @disconnect_on_exception
    def SetAutoConf(self, e: int):
        """Enable or disable auto-conf (automatic selection of inverse kinematics options).

        Parameters
        ----------
        e : boolean
            If true, robot will automatically choose the best configuration for the desired pose.

        """
        self._send_motion_command('SetAutoConf', [int(e)])

    @disconnect_on_exception
    def SetConfTurn(self, n: int):
        """Manually set the last joint turn configuration parameter.

        Parameters
        ----------
        n : integer
            The turn number for joint 6.

        """
        self._send_motion_command('SetConfTurn', [n])

    @disconnect_on_exception
    def SetAutoConfTurn(self, e: int):
        """Enable or disable auto-conf (automatic selection of inverse kinematics options) for joint 6.

        Parameters
        ----------
        e : boolean
            If true, robot will automatically choose the best configuration for the desired pose.

        """
        self._send_motion_command('SetAutoConfTurn', [int(e)])

    @disconnect_on_exception
    def SetBlending(self, p: float):
        """Set percentage of blending between consecutive movements in the same mode (velocity or cartesian).

        Note: There can't be blending between joint mode and Cartesian mode moves.

        Parameters
        ----------
        p : float
            Percentage blending between actions.

        """
        self._send_motion_command('SetBlending', [p])

    @disconnect_on_exception
    def SetCartAcc(self, p: float):
        """Set target acceleration (linear and angular) during MoveLin commands.

        Parameters
        ----------
        p : float
            Percentage of maximum acceleration.

        """
        self._send_motion_command('SetCartAcc', [p])

    @disconnect_on_exception
    def SetCartAngVel(self, w: float):
        """Set maximum angular velocity during MoveLin commands.

        Note: Actual angular velocity may be lower if necessary to avoid exceeding maximum joint velocity.

        Parameters
        ----------
        w : float
            Maximum angular velocity in deg/s.

        """
        self._send_motion_command('SetCartAngVel', [w])

    @disconnect_on_exception
    def SetCartLinVel(self, v: float):
        """Set maximum linear velocity during MoveLin commands.

        Note: Actual linear velocity may be lower if necessary to avoid exceeding maximum joint velocity.

        Parameters
        ----------
        v : float
            Maximum angular velocity in deg/s.

        """
        self._send_motion_command('SetCartLinVel', [v])

    @disconnect_on_exception
    def SetJointAcc(self, p: float):
        """Set target joint acceleration during MoveJoints commands.

        Parameters
        ----------
        p : float
            Target acceleration, in percent.

        """
        self._send_motion_command('SetJointAcc', [p])

    @disconnect_on_exception
    def SetJointVel(self, p: float):
        """Set target joint velocity during MoveJoints commands.

        Parameters
        ----------
        p : float
            Target joint velocity, in percent.

        """
        self._send_motion_command('SetJointVel', [p])

    @disconnect_on_exception
    def SetTrf(self, x: float, y: float, z: float, alpha: float, beta: float, gamma: float):
        """Set the TRF (tool reference frame) Cartesian position.

        Parameters
        ----------
        x, y, z : float
            Desired reference coordinates in mm.
        alpha, beta, gamma
            Desired reference orientation in degrees.

        """
        self._send_motion_command('SetTrf', [x, y, z, alpha, beta, gamma])

    @deprecation.deprecated(deprecated_in="1.2.0",
                            removed_in="1.3.0",
                            current_version=__version__,
                            details="Use the 'SetTrf' function instead")
    @disconnect_on_exception
    def SetTRF(self, x: float, y: float, z: float, alpha: float, beta: float, gamma: float):
        """Deprecated use SetTrf instead

        """
        self.SetTrf(x, y, z, alpha, beta, gamma)

    @disconnect_on_exception
    def SetWrf(self, x: float, y: float, z: float, alpha: float, beta: float, gamma: float):
        """Set the WRF (world reference frame) Cartesian position.

        Parameters
        ----------
        x, y, z : float
            Desired reference coordinates in mm.
        alpha, beta, gamma
            Desired reference orientation in degrees.

        """
        self._send_motion_command('SetWrf', [x, y, z, alpha, beta, gamma])

    @deprecation.deprecated(deprecated_in="1.2.0",
                            removed_in="1.3.0",
                            current_version=__version__,
                            details="Use the 'SetWrf' function instead")
    @disconnect_on_exception
    def SetWRF(self, x: float, y: float, z: float, alpha: float, beta: float, gamma: float):
        """Deprecated use SetWrf instead

        """
        self.SetWrf(x, y, z, alpha, beta, gamma)

    @disconnect_on_exception
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

    @disconnect_on_exception
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

    @disconnect_on_exception
    def WaitGripperMoveCompletion(self, timeout: Optional[float] = None):
        """Wait for the gripper move to complete.
           Note that any move command in the motion queue following a gripper command will start executing at the same
           time the gripper starts moving.
           If you wish to wait for the gripper to finish moving first, DO NOT send motion commands to the robot after
           your gripper command, then call WaitGripperMoveCompletion, and finally post following motion commands.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the move to complete (in seconds).
        """
        return super().WaitGripperMoveCompletion(timeout)

    @disconnect_on_exception
    def GripperOpen(self):
        """Open the gripper.
           Note that the robot may move while the gripper is opening.
           See method WaitGripperMoveCompletion for more information.

        """
        self._gripper_state_before_last_move = copy.deepcopy(self._gripper_state)
        self._send_motion_command('GripperOpen')

        if self._enable_synchronous_mode and self._robot_info.gripper_pos_ctrl_capable and self.GetRtExtToolStatus(
        ).is_gripper():
            gripper_state = self.GetRtGripperState(synchronous_update=True)
            if gripper_state.opened and gripper_state.target_pos_reached:
                return
            self.WaitGripperMoveCompletion()

    @disconnect_on_exception
    def GripperClose(self):
        """Close the gripper.
           Note that the robot may move while the gripper is closing.
           See method WaitGripperMoveCompletion for more information.

        """
        self._gripper_state_before_last_move = copy.deepcopy(self._gripper_state)
        self._send_motion_command('GripperClose')

        if self._enable_synchronous_mode and self._robot_info.gripper_pos_ctrl_capable and self.GetRtExtToolStatus(
        ).is_gripper():
            gripper_state = self.GetRtGripperState(synchronous_update=True)
            if gripper_state.closed and gripper_state.target_pos_reached:
                return
            self.WaitGripperMoveCompletion()

    @disconnect_on_exception
    def MoveGripper(self, target: Union[bool, float]):
        """Move the gripper to a target position.
           Note that the robot may move while the gripper is moving.
           See method WaitGripperMoveCompletion for more information.

           If the target specified is a boolean, it indicates if the target position is the opened (True, GRIPPER_OPEN)
           or closed (False, GRIPPER_CLOSE) position.
           Otherwise the target position indicates the opening of the gripper, in mm from the most closed position.

        Corresponds to text API calls "GripperOpen" / "GripperClose" / "MoveGripper".


        Parameters
        ----------
        target : boolean or float
            boolean type: Open or close the gripper (GRIPPER_OPEN or GRIPPER_CLOSE)
            float type: The gripper's target position, in mm from the most closed position.

        """
        if isinstance(target, bool):
            if target:
                self.GripperOpen()
            else:
                self.GripperClose()
        else:
            self._gripper_state_before_last_move = copy.deepcopy(self._gripper_state)
            self._send_motion_command('MoveGripper', [target])
            if self._enable_synchronous_mode:
                rt_data = self.GetRobotRtData(synchronous_update=True)
                if math.isclose(rt_data.rt_gripper_pos.data[0], target, abs_tol=0.1):
                    return
                self.WaitGripperMoveCompletion()

    @disconnect_on_exception
    def SetGripperForce(self, p: float):
        """Set the gripper's force in percent.

        Parameters
        ----------
        p : float
            The desired force in percent.

        """
        self._send_motion_command('SetGripperForce', [p])

    @disconnect_on_exception
    def SetGripperVel(self, p: float):
        """Set the gripper's velocity in percent.

        Parameters
        ----------
        p : float
            The desired velocity in percent.

        """
        self._send_motion_command('SetGripperVel', [p])

    @disconnect_on_exception
    def SetGripperRange(self, closePos: float, openPos):
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

    @disconnect_on_exception
    def SetValveState(self, *args: list[int]):
        """Set the pneumatic module valve states.
           Note that the robot may move while the valves are being opened/closed.
           Using the Delay method may be appropriate to postpone subsequent move commands in the motion queue
           if necessary.

        Parameters
        ----------
        valve_1...valve_n : int
            The desired state for valve:
                - MxCmdValveState.MX_VALVE_STATE_STAY
                - MxCmdValveState.MX_VALVE_STATE_CLOSE
                - MxCmdValveState.MX_VALVE_STATE_OPEN
            MPM500 pneumatic module has 2 valves.

        """
        self._send_motion_command('SetValveState', args)

    @disconnect_on_exception
    def WaitForAnyCheckpoint(self, timeout: float = None):
        """Pause program execution until any checkpoint has been received from the robot.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the checkpoint (in seconds).
        """
        with self._main_lock:
            self._check_internal_states()
            if '*' not in self._internal_checkpoints:
                self._internal_checkpoints['*'] = list()
            event = InterruptableEvent()
            self._internal_checkpoints['*'].append(event)

        event.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitConnected(self, timeout: float = None):
        """Pause program execution until robot is connected.
           Since the Connect() command is always blocking, this command is only useful if a separate thread wants to
           wait for the connection to be established.

        Parameters
        ----------
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).
        """
        # Use appropriate default timeout of not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_connected.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitDisconnected(self, timeout: float = None):
        """Pause program execution until the robot is disconnected.
           Since the Disconnect() command is always blocking, this command is only useful if a separate thread wants to
           wait for the disconnection.
        Parameters
        ----------
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).
        """
        # Use appropriate default timeout of not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_disconnected.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitActivated(self, timeout: float = None):
        """Pause program execution until the robot is activated.

        Parameters
        ----------
        timeout : float, by default 15
            Maximum time to spend waiting for the event (in seconds).
        """
        # Use appropriate default timeout of not specified
        if timeout is None:
            timeout = 15.0
        self._robot_events.on_activated.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitDeactivated(self, timeout: float = None):
        """Pause program execution until the robot is deactivated.

        Parameters
        ----------
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).
        """
        super().WaitDeactivated(timeout)

    @disconnect_on_exception
    def WaitHomed(self, timeout: float = None):
        """Pause program execution until the robot is homed.

        Parameters
        ----------
        timeout : float, by default 40
            Maximum time to spend waiting for the event (in seconds).
        """
        # Use appropriate default timeout of not specified
        if timeout is None:
            timeout = 40.0
        self._robot_events.on_homed.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitSimActivated(self, timeout: float = None):
        """Pause program execution until the robot simulation mode is activated.

        Parameters
        ----------
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).

        """
        # Use appropriate default timeout of not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_activate_sim.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitSimDeactivated(self, timeout: float = None):
        """Pause program execution until the robot simulation mode is deactivated.

        Parameters
        ----------
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).
        """
        # Use appropriate default timeout of not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_deactivate_sim.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitExtToolSimActivated(self, timeout: float = None):
        """Pause program execution until the robot external tool simulation mode is activated.

        Parameters
        ----------
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).

        """
        # Use appropriate default timeout of not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_activate_ext_tool_sim.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitExtToolSimDeactivated(self, timeout: float = None):
        """Pause program execution until the robot external tool simulation mode is deactivated.

        Parameters
        ----------
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).
        """
        # Use appropriate default timeout of not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_deactivate_ext_tool_sim.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitRecoveryMode(self, activated: bool, timeout: float = None):
        """Pause program execution until the robot recovery mode is in the requested state.

        Parameters
        ----------
        activated : bool
            Recovery mode to wait for (activated or deactivated
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).
        """
        # Use appropriate default timeout of not specified
        if timeout is None:
            timeout = self.default_timeout
        if activated:
            self._robot_events.on_activate_recovery_mode.wait(timeout=timeout)
        else:
            self._robot_events.on_deactivate_recovery_mode.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitForError(self, timeout: float = None):
        """Pause program execution until the robot is in error state.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the event (in seconds).
        """
        self._robot_events.on_error.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitErrorReset(self, timeout: float = None):
        """Pause program execution until the robot is not in an error state.

        Parameters
        ----------
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).
        """
        # Use appropriate default timeout of not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_error_reset.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitPStop2Reset(self, timeout: float = None):
        """Pause program execution until the robot is no more in PStop2 condition.

        Parameters
        ----------
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).
        """
        # Use appropriate default timeout of not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_pstop2_reset.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitPStop2Resettable(self, timeout: float = None):
        """Pause program execution until the robot PStop2 condition can be reset (or has been reset).

        Parameters
        ----------
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).
        """
        # Use appropriate default timeout of not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_pstop2_resettable.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitEStopReset(self, timeout: float = None):
        """Pause program execution until the robot is no more in EStop condition.

        Parameters
        ----------
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).
        """
        # Use appropriate default timeout of not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_estop_reset.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitMotionResumed(self, timeout: float = None):
        """Pause program execution until the robot motion is resumed.

        Parameters
        ----------
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).
        """
        # Use appropriate default timeout of not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_motion_resumed.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitMotionPaused(self, timeout: float = None):
        """Pause program execution until the robot motion is paused.

        Parameters
        ----------
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).
        """
        # Use appropriate default timeout of not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_motion_paused.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitMotionCleared(self, timeout: float = None):
        """Pause program execution until all pending request to clear motion have been acknowledged.

        Parameters
        ----------
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).
        """
        # Use appropriate default timeout of not specified
        if timeout is None:
            timeout = self.default_timeout
        self._robot_events.on_motion_cleared.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitEndOfCycle(self, timeout: float = None):
        """Pause program execution until all messages in a message cycle are received

        Parameters
        ----------
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time to spend waiting for the event (in seconds).
        """
        if self._robot_events.on_end_of_cycle.is_set():
            self._robot_events.on_end_of_cycle.clear()

        # Use appropriate default timeout of not specified
        if timeout is None:
            timeout = 2
        self._robot_events.on_end_of_cycle.wait(timeout=timeout)

    @disconnect_on_exception
    def WaitIdle(self, timeout: float = None):
        """Pause program execution until robot is idle.

        Parameters
        ----------
        timeout : float
            Maximum time to spend waiting for the event (in seconds).
        """
        with self._main_lock:
            # Can't wait if robot is in error (already "idle")
            if self._robot_status.error_status:
                raise InterruptException('Robot is in error')
        checkpoint = self._set_checkpoint_internal()

        start_time = time.time()
        checkpoint.wait(timeout=timeout)
        end_time = time.time()

        if timeout:
            remaining_timeout = timeout - (end_time - start_time)
        else:
            remaining_timeout = None

        self._robot_events.on_end_of_block.wait(timeout=remaining_timeout)

    @disconnect_on_exception
    def ResetError(self):
        """Attempt to reset robot error."""
        with self._main_lock:
            self._check_internal_states()
            self._send_command('ResetError')

        if self._enable_synchronous_mode:
            self._robot_events.on_error_reset.wait(timeout=self.default_timeout)

    @deprecation.deprecated(deprecated_in="1.2.2",
                            removed_in="2.0.0",
                            current_version=__version__,
                            details="Use the 'ResetPStop2' function instead")
    @disconnect_on_exception
    def ResetPStop(self, timeout: float = None):
        """Deprecated use ResetPStop2 instead.
        """
        self.ResetPStop2(timeout)

    @disconnect_on_exception
    def ResetPStop2(self, timeout: float = None):
        """Attempt to reset robot PStop2."""
        with self._main_lock:
            self._check_internal_states()
            self._send_command('ResetPStop2')

        if self._enable_synchronous_mode:
            # Use appropriate default timeout of not specified
            if timeout is None:
                timeout = 2
            self._robot_events.on_pstop2_reset.wait(timeout)

    @disconnect_on_exception
    def Delay(self, t: float):
        """Set a delay between motion commands.

        Parameters
        ----------
        t : float
            Desired pause duration in seconds.

        """
        with self._main_lock:
            self._check_internal_states()
            if not self._robot_events.on_homed.is_set():
                raise InvalidStateError('This command requires robot to be homed.')
            self._send_command('Delay', [t])
            if self._enable_synchronous_mode:
                checkpoint = self._set_checkpoint_internal()

        if self._enable_synchronous_mode:
            checkpoint.wait()

    @disconnect_on_exception
    def SendCustomCommand(self,
                          command: str,
                          expected_responses: list[int] = None,
                          timeout: float = None) -> InterruptableEvent:
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

    @disconnect_on_exception
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
        with self._main_lock:
            self._check_internal_states()
            self._robot_events.on_offline_program_started.clear()

            self._send_command('StartProgram', [n])

        if self._enable_synchronous_mode:
            try:
                self._robot_events.on_offline_program_started.wait(timeout=timeout)
            except InterruptException as e:
                raise InvalidStateError(str(e))

    @disconnect_on_exception
    def StartOfflineProgram(self, n: int | str, timeout: float = None):
        """Deprecated use StartPrograms instead.
        """
        self.StartProgram(n, timeout)

    @disconnect_on_exception
    def ListPrograms(self, timeout: float = None) -> dict:
        """List all offline programs that are stored on the robot.
        Related commands: LoadProgram, SaveProgram, DeleteProgram

        This function is synchronous only for now.
        If you want to asynchronously list programs, you can use SendCustomCommand, for example:
        done_evt = self.SendCustomCommand('ListPrograms')
        Callback on_command_message(response) can be attached to be informed of all responses from the robot.
        Responses to check fore are:
            - response.id == MxRobotStatusCode.MX_ST_OFFLINE_PROGRAM_LIST
            In this case response.jsonData[MX_JSON_KEY_DATA]["programs"] will contain the listed programs list
            (same format as returned by this function)
            - response.id == MxRobotStatusCode.MX_ST_OFFLINE_PROGRAM_LIST_ERR

        Parameters
        ----------
        timeout: float
            Timeout (in seconds) waiting for programs list.

        Raises
        ------
        TimeoutException
            If timeout was reached before the robot returned success or failure response.
        InterruptException
            If the robot returns an error (exception message will describe the error).

        Returns
        -------
        dict
            Object that contains the status of all listed offline programs, in this format
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
        # Use appropriate default timeout of not specified
        if timeout is None:
            timeout = self.default_timeout

        with self._main_lock:
            self._check_internal_states()
            self._robot_events.on_offline_program_op_done.clear()

            self._send_command('ListPrograms')

        try:
            response = self._robot_events.on_offline_program_op_done.wait(timeout=timeout)
        except InterruptException as e:
            raise InvalidStateError(str(e))
        return response.jsonData[MX_JSON_KEY_DATA]["programs"]

    @disconnect_on_exception
    def LoadProgram(self, name: str, timeout: float = None) -> dict:
        """Load a program from the robot.

        Related commands: ListPrograms, SaveProgram, DeleteProgram

        This function is synchronous only for now.
        If you want to asynchronously load a program, you can use SendCustomCommand, for example:
        done_evt = self.SendCustomCommand('LoadProgram{"data":{"name":"programName"}')
        Callback on_command_message(response) can be attached to be informed of all responses from the robot.
        Responses to check fore are:
            - response.id == MxRobotStatusCode.MX_ST_OFFLINE_PROGRAM_LOAD
            - response.id == MxRobotStatusCode.MX_ST_OFFLINE_PROGRAM_LOAD_ERR

        Parameters
        ----------
        name: str
            Name of the offline program to load.
        timeout: float
            Timeout (in seconds) waiting for program to be loaded.

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
                "name": "programName",
                "status": {
                    "valid": false,
                    "invalidLines": [10, 332, 522]
                },
                "code":{"// Program code"}
            }
        """
        # Use appropriate default timeout of not specified
        if timeout is None:
            timeout = self.default_timeout

        with self._main_lock:
            self._check_internal_states()
            self._robot_events.on_offline_program_op_done.clear()

            jsonData = {"data": {"name": name}}
            self._send_command(f'LoadProgram', f'{json.dumps(jsonData)}')

        try:
            response = self._robot_events.on_offline_program_op_done.wait(timeout=timeout)
        except InterruptException as e:
            raise InvalidStateError(str(e))
        return response.jsonData[MX_JSON_KEY_DATA]

    @disconnect_on_exception
    def SaveProgram(self, name: str, program: str, timeout: float = None, allow_invalid=False, overwrite=False):
        """Save a program inside the robot.
           The provided program is a list of robot commands, separated by a end-line character.
           The program may contain empty lines, or commented lines (c-style comments with // or /* comment */).

        Related commands: ListPrograms, LoadProgram, DeleteProgram

        This function is synchronous only for now.
        If you want to asynchronously list programs, you can use SendCustomCommand, for example:
        done_evt = self.SendCustomCommand('SaveProgram{"data":{"name":"programName", "program":"Program text"}}')
        Callback on_command_message(response) can be attached to be informed of all responses from the robot.
        Responses to check fore are:
            - response.id == MxRobotStatusCode.MX_ST_OFFLINE_PROGRAM_SAVE
            - response.id == MxRobotStatusCode.MX_ST_OFFLINE_PROGRAM_SAVE_ERR

        Parameters
        ----------
        name: str
            Name of the offline program to save.
        program: str
            The offline program code to save (see description above for details).
        timeout: float
            Timeout (in seconds) waiting for program to be saved.
        allow_invalid: bool
            Allow saving the program even if it is invalid
        overwrite: bool
            Overwrite if a program with the same name already exists on the robot.
            (equivalent of atomic Delete then SaveProgram)
            If False, SaveProgram will be refused if the name is already used.

        Raises
        ------
        TimeoutException
            If timeout was reached before the robot returned success or failure response.
        InterruptException
            If the robot returns an error (exception message will describe the error).
        """
        # Use appropriate default timeout of not specified
        if timeout is None:
            timeout = self.default_timeout

        with self._main_lock:
            self._check_internal_states()
            self._robot_events.on_offline_program_op_done.clear()

            jsonData = {
                "data": {
                    "name": name,
                    "program": program,
                    "allowInvalid": allow_invalid,
                    "overwrite": overwrite
                }
            }
            self._send_command('SaveProgram', f'{json.dumps(jsonData)}')

        try:
            response = self._robot_events.on_offline_program_op_done.wait(timeout=timeout)
        except InterruptException as e:
            raise InvalidStateError(str(e))

    @disconnect_on_exception
    def DeleteProgram(self, name: str, timeout: float = None):
        """Delete a program from the robot.

        Related commands: ListPrograms, LoadProgram, SaveProgram

        This function is synchronous only for now.
        If you want to asynchronously delete a program, you can use SendCustomCommand, for example:
        done_evt = self.SendCustomCommand('DeleteProgram{"data":{"name":"programName"}')
        Callback on_command_message(response) can be attached to be informed of all responses from the robot.
        Responses to check fore are:
            - response.id == MxRobotStatusCode.MX_ST_OFFLINE_PROGRAM_DELETE
            - response.id == MxRobotStatusCode.MX_ST_OFFLINE_PROGRAM_DELETE_ERR

        Parameters
        ----------
        name: str
            Name of the offline program to delete.
        timeout: float
            Timeout (in seconds) waiting for program to be deleted.

        Raises
        ------
        TimeoutException
            If timeout was reached before the robot returned success or failure response.
        InterruptException
            If the robot returns an error (exception message will describe the error).
        """
        # Use appropriate default timeout of not specified
        if timeout is None:
            timeout = self.default_timeout

        with self._main_lock:
            self._check_internal_states()
            self._robot_events.on_offline_program_op_done.clear()

            jsonData = {"data": {"name": name}}
            self._send_command('DeleteProgram', f'{json.dumps(jsonData)}')

        try:
            response = self._robot_events.on_offline_program_op_done.wait(timeout=timeout)
        except InterruptException as e:
            raise InvalidStateError(str(e))

    @disconnect_on_exception
    def GetRtExtToolStatus(self,
                           include_timestamp: bool = False,
                           synchronous_update: bool = False,
                           timeout: float = None) -> ExtToolStatus:
        """Return a copy of the current external tool status

        Parameters
        ----------
        include_timestamp : bool
            If true, return a TimestampedData object, otherwise just return states.
        synchronous_update: boolean
            True -> Synchronously get updated external tool status. False -> Get latest known status.
        timeout: float
            Timeout (in seconds) waiting for synchronous response from the robot.

        Returns
        -------
        TimestampedData or ExtToolStatus
            Object containing the current external tool status

        """
        # Use appropriate default timeout of not specified
        if timeout is None:
            timeout = self.default_timeout
        if synchronous_update:
            self._send_sync_command('GetRtExtToolStatus', self._robot_events.on_external_tool_status_updated, timeout)

        with self._main_lock:
            if include_timestamp:
                return copy.deepcopy(self._robot_rt_data.rt_external_tool_status)
            else:
                return copy.deepcopy(self._external_tool_status)

    @disconnect_on_exception
    def GetRtGripperState(self,
                          include_timestamp: bool = False,
                          synchronous_update: bool = False,
                          timeout: float = None) -> GripperState:
        """Return a copy of the current gripper state

        Parameters
        ----------
        include_timestamp : bool
            If true, return a TimestampedData object, otherwise just return states.
        synchronous_update: boolean
            True -> Synchronously get updated gripper state. False -> Get latest known status.
        timeout: float
            Timeout (in seconds) waiting for synchronous response from the robot.

        Returns
        -------
        TimestampedData or GripperState
            Object containing the current gripper state

        """
        # Use appropriate default timeout of not specified
        if timeout is None:
            timeout = self.default_timeout
        if synchronous_update:
            self._send_sync_command('GetRtGripperState', self._robot_events.on_gripper_state_updated, timeout)

        with self._main_lock:
            if include_timestamp:
                return copy.deepcopy(self._robot_rt_data.rt_gripper_state)
            else:
                return copy.deepcopy(self._gripper_state)

    @disconnect_on_exception
    def GetRtValveState(self,
                        include_timestamp: bool = False,
                        synchronous_update: bool = False,
                        timeout: float = None) -> ValveState:
        """Return a copy of the current valve state

        Parameters
        ----------
        include_timestamp : bool
            If true, return a TimestampedData object, otherwise just return states.
        synchronous_update: boolean
            True -> Synchronously get updated valve states. False -> Get latest known status.
        timeout: float
            Timeout (in seconds) waiting for synchronous response from the robot.

        Returns
        -------
        TimestampedData or ValveState
            Object containing the current valve state

        """
        # Use appropriate default timeout of not specified
        if timeout is None:
            timeout = self.default_timeout
        if synchronous_update:
            self._send_sync_command('GetRtValveState', self._robot_events.on_valve_state_updated, timeout)

        with self._main_lock:
            if include_timestamp:
                return copy.deepcopy(self._robot_rt_data.rt_valve_state)
            else:
                return copy.deepcopy(self._valve_state)

    @disconnect_on_exception
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
            If true, requests updated joints positions and waits for response, else uses last known positions.
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time in second to wait for forced update.

        Return
        ------
        TimestampedData or list of floats
            Returns joint positions in degrees.

        """
        # Use appropriate default timeout of not specified
        if timeout is None:
            timeout = self.default_timeout
        if synchronous_update:
            if self._robot_info.rt_message_capable:
                self._send_sync_command('GetRtTargetJointPos', self._robot_events.on_joints_updated, timeout)
            else:
                # This robot does not have GetRtTargetJointPos, use legacy GetJoints (but won't get timestamp)
                self._send_sync_command('GetJoints', self._robot_events.on_joints_updated, timeout)

            # Wait until response is received (this will throw TimeoutException if appropriate)
            self._robot_events.on_joints_updated.wait(timeout=timeout)

        with self._main_lock:
            if include_timestamp:
                if not self._robot_info.rt_message_capable:
                    raise InvalidStateError('Cannot provide timestamp with current robot firmware or model.')
                else:
                    return copy.deepcopy(self._robot_rt_data.rt_target_joint_pos)

            return copy.deepcopy(self._robot_rt_data.rt_target_joint_pos.data)

    def GetJoints(self, synchronous_update: bool = False, timeout: float = None):
        """Legacy command. Please use GetRtTargetJointPos instead."""
        # Use appropriate default timeout of not specified
        if timeout is None:
            timeout = self.default_timeout
        return self.GetRtTargetJointPos(include_timestamp=False, synchronous_update=synchronous_update, timeout=timeout)

    @disconnect_on_exception
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
            If true, requests updated pose and waits for response, else uses last know pose.
        timeout : float, defaults to DEFAULT_WAIT_TIMEOUT
            Maximum time in second to wait for forced update.

        Return
        ------
        TimestampedData or list of floats
            Returns end-effector pose [x, y, z, alpha, beta, gamma].

        """
        # Use appropriate default timeout of not specified
        if timeout is None:
            timeout = self.default_timeout
        if synchronous_update:
            if self._robot_info.rt_message_capable:
                self._send_sync_command('GetRtTargetCartPos', self._robot_events.on_pose_updated, timeout)
            else:
                # This robot does not have GetRtTargetCartPos, use legacy GetPose (but won't get timestamp)
                self._send_sync_command('GetPose', self._robot_events.on_pose_updated, timeout)

        with self._main_lock:
            if include_timestamp:
                if not self._robot_info.rt_message_capable:
                    raise InvalidStateError('Cannot provide timestamp with current robot firmware or model.')
                else:
                    return copy.deepcopy(self._robot_rt_data.rt_target_cart_pos)

            return copy.deepcopy(self._robot_rt_data.rt_target_cart_pos.data)

    def GetPose(self, synchronous_update: bool = False, timeout: float = None) -> TimestampedData:
        """Legacy command. Please use GetRtTargetCartPos instead."""
        # Use appropriate default timeout of not specified
        if timeout is None:
            timeout = self.default_timeout
        return self.GetRtTargetCartPos(include_timestamp=False, synchronous_update=synchronous_update, timeout=timeout)

    @disconnect_on_exception
    def SetMonitoringInterval(self, t: float):
        """Sets the interval at which the monitoring port sends real-time data.

        Parameters
        ----------
        t : float
            Monitoring interval duration in seconds.

        """
        with self._main_lock:
            self._check_internal_states()
            self._set_monitoring_interval_internal(t)
            self._monitoring_interval_to_restore = t

    @disconnect_on_exception
    def SetRealTimeMonitoring(self, *events: tuple):
        """Configure which real-time monitoring events to enable.

        Parameters
        ----------
        events : list of event IDs
            List of event IDs to enable.
            Example: events=[MxRobotStatusCode.MX_ST_RT_TARGET_JOINT_POS, MxRobotStatusCode.MX_ST_RT_TARGET_CART_POS]
            enables the target joint positions and target end effector pose messages.
            Can also use events='all' to enable all.

        """
        with self._main_lock:
            self._check_internal_states()
            if isinstance(events, tuple):
                event_list = list(events)
            else:
                event_list = events
            self._send_command('SetRealTimeMonitoring', event_list)

    @disconnect_on_exception
    def SetRtc(self, t: int):
        """Sets the robot's real-time clock (date/time).

        Parameters
        ----------
        t : int
            Unix epoch time (seconds since 00:00:00 UTC Jan 1, 1970).

        """
        with self._main_lock:
            self._check_internal_states()
            self._send_command('SetRtc', [t])

    @deprecation.deprecated(deprecated_in="1.2.0",
                            removed_in="1.3.0",
                            current_version=__version__,
                            details="Use the 'SetRtc' function instead")
    @disconnect_on_exception
    def SetRTC(self, t: int):
        """Deprecated use SetRtc instead.
        """
        self.SetRtc(t)

    @disconnect_on_exception
    def ActivateSim(self):
        """Enables simulation mode. Motors and external tool don't move, but commands will be processed."""
        with self._main_lock:
            self._check_internal_states()
            self._send_command('ActivateSim')
        if self._enable_synchronous_mode:
            self._robot_events.on_activate_sim.wait(timeout=self.default_timeout)

    @disconnect_on_exception
    def DeactivateSim(self):
        """Disables simulation mode. Motors and external tool will now move normally."""
        with self._main_lock:
            self._check_internal_states()
            self._send_command('DeactivateSim')
        if self._enable_synchronous_mode:
            self._robot_events.on_deactivate_sim.wait(timeout=self.default_timeout)

    @disconnect_on_exception
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
        # Firmware version 8.4 and older only support 1 as tool type
        if sim_ext_tool_type != MxExtToolType.MX_EXT_TOOL_NONE and not self._robot_info.gripper_pos_ctrl_capable:
            sim_ext_tool_type = 1

        with self._main_lock:
            self._check_internal_states()
            self._send_command('SetExtToolSim', [sim_ext_tool_type])

        if self._enable_synchronous_mode:
            if sim_ext_tool_type == MxExtToolType.MX_EXT_TOOL_NONE:
                self.WaitExtToolSimDeactivated()
            else:
                self.WaitExtToolSimActivated()

    @disconnect_on_exception
    def SetRecoveryMode(self, activated: bool = True):
        """Enable/disable recovery mode, allowing robot to move (slowly) without homing and without joint limits."""
        with self._main_lock:
            self._check_internal_states()
            self._send_command('SetRecoveryMode', f'{1 if activated else 0}')

        if self._enable_synchronous_mode:
            if activated:
                self._robot_events.on_activate_recovery_mode.wait(timeout=self.default_timeout)
            else:
                self._robot_events.on_deactivate_recovery_mode.wait(timeout=self.default_timeout)

    @disconnect_on_exception
    def SetJointLimitsCfg(self, e: bool = True):
        """Enable/Disable user-defined limits set by the 'SetJointLimits' command. It can only be executed while
        the robot is deactivated. If the user-defined limits are disabled, the default joint limits become active.
        However, user-defined limits remain in memory, and can be re-enabled, even after a power down.

        Parameters
        ----------
            e : bool
                enable (True) or disable (False) the user-defined joint limits.
        """
        if self._enable_synchronous_mode:
            response_event = self.SendCustomCommand(
                f'SetJointLimitsCfg({1 if e else 0})',
                expected_responses=[MxRobotStatusCode.MX_ST_SET_JOINT_LIMITS_CFG, MxRobotStatusCode.MX_ST_CMD_FAILED])
            if response_event.wait(timeout=DEFAULT_WAIT_TIMEOUT).id == MxRobotStatusCode.MX_ST_CMD_FAILED:
                raise MecademicException("Argument Error in Command : SetJointLimitsCfg")
        else:
            with self._main_lock:
                self._check_internal_states()
                self._send_command('SetJointLimitsCfg', f"{1 if e else 0}")

    @disconnect_on_exception
    def SetJointLimits(self, n: int, lower_limit: float, upper_limit: float):
        """This command redefines the lower and upper limits of a robot joint. It can only be executed while the robot
        is deactivated. For these user-defined joint limits to be taken into account, you must execute the command
        SetJointLimitsCfg(1). Obviously, the new joint limits must be within the default joint limits and all the robot
        joints position must be within the requested limits. Note that these user-defined joint limits remain active even
        after you power down the robot.

        Parameters
        ----------
            n : int
                joint number, an integer ranging from 1 to 6
            lower_limit : float
                lower limit of the actuator, in degrees/mm
            upper_limit : float
                upper limit of the actuator, in degrees/mm
        """
        if self._enable_synchronous_mode:
            response_event = self.SendCustomCommand(
                f'SetJointLimits({n},{lower_limit},{upper_limit})',
                expected_responses=[MxRobotStatusCode.MX_ST_SET_JOINT_LIMITS, MxRobotStatusCode.MX_ST_CMD_FAILED])
            if response_event.wait(timeout=DEFAULT_WAIT_TIMEOUT).id == MxRobotStatusCode.MX_ST_CMD_FAILED:
                raise MecademicException("Argument Error in Command : SetJointLimits")
        else:
            with self._main_lock:
                self._check_internal_states()
                self._send_command('SetJointLimits', f"{n},{lower_limit},{upper_limit}")

    @disconnect_on_exception
    def SetWorkspaceLimitsCfg(
            self,
            severity: MxEventSeverity = MxEventSeverity.MX_EVENT_SEVERITY_ERROR,
            mode: MxCollisionAvoidanceMode = MxCollisionAvoidanceMode.MX_COLLISION_MODE_SELF_COLLISION_DETECTION):
        """Set the severity at which to report collision events and the detection mode for collisions used by the robot.
        This command can only be used when the robot is deactivated. User-defined limits remain in memory, even after a
        power down.

        Parameters
        ----------
            severity : MxEventSeverity
                Severity-level to report collision events.
                The available severities are found in mx_robot_def.MxEventSeverity.
                Note that MX_EVENT_SEVERITY_PAUSE_MOTION = 2 is not supported as colliding paths should not be followed.
            mode : MxCollisionAvoidanceMode
                Collision detection mode to use for checks.
                The available modes are found in mx_robot_def.MxCollisionAvoidanceMode.
        """
        if self._enable_synchronous_mode:
            response_event = self.SendCustomCommand(f'SetWorkspaceLimitsCfg({severity},{mode})',
                                                    expected_responses=[
                                                        MxRobotStatusCode.MX_ST_SET_WORKSPACE_LIMITS_CFG,
                                                        MxRobotStatusCode.MX_ST_CMD_FAILED
                                                    ])
            if response_event.wait(timeout=DEFAULT_WAIT_TIMEOUT).id == MxRobotStatusCode.MX_ST_CMD_FAILED:
                raise MecademicException("Argument Error in Command : SetWorkspaceLimitsCfg")
        else:
            with self._main_lock:
                self._check_internal_states()
                self._send_command('SetWorkspaceLimitsCfg', f"{severity},{mode}")

    @disconnect_on_exception
    def SetWorkspaceLimits(self, x_min: float, y_min: float, z_min: float, x_max: float, y_max: float, z_max: float):
        """Set the workspace limits the robot must not exceed. This command can only be used when the robot is
        deactivated. User-defined limits remain in memory, even after a power down.

        Parameters
        ----------
            x_min : float
                minimum x value of the workspace
            y_min : float
                minimum y value of the workspace
            z_min : float
                minimum z value of the workspace
            x_max : float
                maximum x value of the workspace
            y_max : float
                maximum y value of the workspace
            z_max : float
                maximum z value of the workspace
        """
        if self._enable_synchronous_mode:
            response_event = self.SendCustomCommand(
                f'SetWorkspaceLimits({x_min}, {y_min}, {z_min}, {x_max}, {y_max}, {z_max})',
                expected_responses=[MxRobotStatusCode.MX_ST_SET_WORKSPACE_LIMITS, MxRobotStatusCode.MX_ST_CMD_FAILED])
            if response_event.wait(timeout=DEFAULT_WAIT_TIMEOUT).id == MxRobotStatusCode.MX_ST_CMD_FAILED:
                raise MecademicException("Argument Error in Command : SetWorkspaceLimits")
        else:
            with self._main_lock:
                self._check_internal_states()
                self._send_command('SetWorkspaceLimits', [x_min, y_min, z_min, x_max, y_max, z_max])

    @disconnect_on_exception
    def SetToolSphere(self, x: float, y: float, z: float, r: float):
        """Set the workspace limits the robot must not exceed. This command can only be used when the robot is
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
        if self._enable_synchronous_mode:
            response_event = self.SendCustomCommand(
                f'SetToolSphere({x}, {y}, {z}, {r})',
                expected_responses=[MxRobotStatusCode.MX_ST_SET_TOOL_SPHERE, MxRobotStatusCode.MX_ST_CMD_FAILED])
            if response_event.wait(timeout=DEFAULT_WAIT_TIMEOUT).id == MxRobotStatusCode.MX_ST_CMD_FAILED:
                raise MecademicException("Argument Error in Command : SetToolSphere")
        else:
            with self._main_lock:
                self._check_internal_states()
                self._send_command('SetToolSphere', [x, y, z, r])

    @disconnect_on_exception
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
                - MX_TORQUE_LIMITS_DETECT_SKIP_ACCEL or 1 or True : Do not check if torque is within limits during acceleration
        """
        severity_int = TORQUE_LIMIT_SEVERITIES[severity] if type(severity) == str else severity
        skip_acceleration_int = 1 if skip_acceleration else 0
        self._send_motion_command('SetTorqueLimitsCfg', [severity_int, skip_acceleration_int])

    @disconnect_on_exception
    def SetTorqueLimits(self, *args: float):
        """Set the torque limit (in percent) for each joint.
        Note that torque limits will be applied only if severity mode is set to other than 'disabled' by
        calling SetTorqueLimitsCfg.

        Parameters
        ----------
        joint_1...joint_n : float
            Desired torque limit in percent.

        """
        if len(args) != self._robot_info.num_joints:
            raise ValueError('Incorrect number of joints sent to command.')

        self._send_motion_command('SetTorqueLimits', args)

    @disconnect_on_exception
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
        self._send_motion_command('SetPayload', [mass, x, y, z])

    @disconnect_on_exception
    def ActivateBrakes(self, activated: bool = True):
        """Enable/disable the brakes. These commands are only available when the robot is deactivated.

        By default, brakes are enabled until robot is activated (brakes are automatically disabled upon activation).
        Corresponds to text API calls "BrakesOn" / "BrakesOff".

        Parameters
        ----------
        activated : bool
            Engage brakes if true, otherwise disengage brakes.

        """
        with self._main_lock:
            self._check_internal_states()
            if activated:
                self._send_command('BrakesOn')
            else:
                self._send_command('BrakesOff')

        if self._enable_synchronous_mode:
            if activated:
                self._robot_events.on_brakes_activated.wait(timeout=self.default_timeout)
            else:
                self._robot_events.on_brakes_deactivated.wait(timeout=self.default_timeout)

    @disconnect_on_exception
    def GetRobotInfo(self) -> RobotInfo:
        """Return a copy of the known robot information.

        Return
        ------
        RobotInfo
            Object containing robot information.

        """
        return super().GetRobotInfo()

    @disconnect_on_exception
    def GetRobotRtData(self, synchronous_update: bool = False, timeout: float = None) -> RobotRtData:
        """Return a copy of the current robot real-time data, with all values associated with the same timestamp

        Parameters
        ----------
        synchronous_update: boolean
            True -> Synchronously wait for the next data cycle to get updated data. False -> Get latest known data.
        timeout: float
            Timeout (in seconds) waiting for updated cyclic data from the robot. Only used for synchronous requests.

        Return
        ------
        RobotRtData
            Object containing the current robot real-time data

        """
        if synchronous_update:
            self.WaitEndOfCycle(timeout)

        with self._main_lock:
            return copy.deepcopy(self._robot_rt_data_stable)

    @disconnect_on_exception
    def GetStatusRobot(self, synchronous_update: bool = False, timeout: float = None) -> RobotStatus:
        """Return a copy of the current robot status

        Parameters
        ----------
        synchronous_update: boolean
            True -> Synchronously get updated robot status. False -> Get latest known status.
        timeout: float, defaults to DEFAULT_WAIT_TIMEOUT
            Timeout (in seconds) waiting for synchronous response from the robot.

        Returns
        -------
        RobotStatus
            Object containing the current robot status

        """
        return super().GetStatusRobot(synchronous_update, timeout)

    @disconnect_on_exception
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
        # Use appropriate default timeout of not specified
        if timeout is None:
            timeout = self.default_timeout

        response = self._send_custom_command('GetGripperRange',
                                             expected_responses=[MxRobotStatusCode.MX_ST_GET_GRIPPER_RANGE],
                                             timeout=self.default_timeout)
        positions = string_to_numbers(response.data)
        assert len(positions) == 2
        return positions[0], positions[1]

    def LogTrace(self, trace: str):
        """Send a text trace that is printed in the robot's log internal file (which can be retrieved from robot's Web
           portal under menu "Options -> Get Log").
           (useful for clearly identifying steps of a program within robot's log when reporting problems to
            Mecademic support team!)

        Parameters
        ----------
        trace : string
            Text string to print in robot's internal log file
        """
        # Escape any " in the provided string
        trace = trace.replace('"', '\"')
        if self._robot_info.version.is_at_least(9, 2):
            self.SendCustomCommand(f"-LogTrace({trace})")
        else:
            self.SendCustomCommand(f"LogTrace({trace})")

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
        if self._file_logger is not None:
            raise InvalidStateError('Another file logging operation is in progress.')

        self._set_monitoring_interval_internal(monitoringInterval)
        if self._robot_info.rt_message_capable:
            if fields is None:
                self.SetRealTimeMonitoring('all')
            else:
                self.SetRealTimeMonitoring(*fields)

            # Use a synchronous "GetRealTimeMonitoring" to ensure that we've started receiving data for all the
            # requested real-time monitoring fields we just enabled
            self._send_custom_command('GetRealTimeMonitoring',
                                      expected_responses=[MxRobotStatusCode.MX_ST_GET_REAL_TIME_MONITORING],
                                      timeout=self.default_timeout,
                                      skip_internal_check=True)
            if not self._robot_info.rt_on_ctrl_port_capable:
                # Older version -> can't be sure that monitoring and control port are in sync, let's wait few
                self.WaitEndOfCycle()
                time.sleep(0.01)
                self.WaitEndOfCycle()

        self._file_logger = mx_traj._RobotTrajectoryLogger(self._robot_info,
                                                           self._robot_rt_data,
                                                           fields,
                                                           file_name=file_name,
                                                           file_path=file_path,
                                                           record_time=record_time,
                                                           monitoring_interval=monitoringInterval)

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
        if self._file_logger is None:
            raise InvalidStateError('No existing logger to stop.')

        # Deactivate logging to avoid logging the following SetMonitoringInterval
        self._file_logger.stop_logging_commands()

        if self._robot_info.rt_message_capable:
            if self._monitoring_interval_to_restore != self._monitoring_interval:
                # Restore default slower monitoring interval
                self._set_monitoring_interval_internal(self._monitoring_interval_to_restore)

            # Send a synchronous command to ensure we've received all monitoring data for this test
            self._send_custom_command('GetRealTimeMonitoring',
                                      expected_responses=[MxRobotStatusCode.MX_ST_GET_REAL_TIME_MONITORING],
                                      timeout=self.default_timeout,
                                      skip_internal_check=True)

        if keep_captured_trajectory:
            self._captured_trajectory = self._file_logger.robot_trajectories

        file_name = self._file_logger.end_log()
        self._file_logger = None

        return file_name

    def GetCapturedTrajectory(self) -> RobotTrajectories:
        """Returns the most recent robot trajectory captured using StartLogging or FileLogger functions.

        Returns
        -------
        RobotTrajectories
            Object that contains robot information and captured trajectory information
        """
        return self._captured_trajectory

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

    def UpdateRobot(self, firmware: Union[str, pathlib.Path], address: Optional[str] = None, timeout=UPDATE_TIMEOUT):
        """
        Install a new firmware and verifies robot version afterward.

        Parameters
        ----------
        firmware: pathlib object or string
            Path of robot firmware file

        address: string
            Robot IP address (optional if already connected)

        """
        return super().UpdateRobot(firmware, address, timeout)
