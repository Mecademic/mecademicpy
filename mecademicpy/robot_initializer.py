"""
This file contains a tool to initialize a robot to a well-known initial state and configuration.
See below for details
"""
# pylint: disable=wildcard-import, unused-wildcard-import
from __future__ import annotations

from copy import deepcopy
from typing import Optional, Tuple, Union

from .mx_robot_def import *
from .robot import Robot
from .robot_classes import *
from .tools import *


class DirtyFlags():

    def __init__(self):
        self._dirty = True
        self._sent_commands = {}

    def set_dirty(self):
        self._dirty = True

    def clear_dirty(self):
        self._dirty = False
        self._sent_commands = {}

    def is_dirty(self, cmd_name: str) -> bool:
        return self._dirty or cmd_name in self._sent_commands

    def add_sent_command(self, cmd_name: str):
        self._sent_commands[cmd_name] = True


class MotionQueueParams:
    """ This class regroups all motion queue parameters """

    def __init__(self):
        self.torque_limits_severity = MxEventSeverity.MX_EVENT_SEVERITY_WARNING
        self.torque_limits_mode = MxTorqueLimitsMode.MX_TORQUE_LIMITS_DETECT_SKIP_ACCEL
        self.torque_limits = [100] * 6
        self.auto_conf = True
        self.auto_conf_turn = True
        self.blending = 100
        self.cart_acc = 50
        self.cart_ang_vel = 150
        self.cart_lin_vel = 400
        self.joint_acc = 50
        self.joint_vel_limit = 100
        self.joint_vel = 50
        self.move_mode = MxMoveMode.MX_MOVE_MODE_VELOCITY
        self.move_duration_severity = MxEventSeverity.MX_EVENT_SEVERITY_WARNING
        self.move_duration = 3.0
        self.vel_timeout = 0.1
        self.trf = [0, 0, 0, 0, 0, 0]
        self.wrf = [0, 0, 0, 0, 0, 0]
        self.payload = [0, 0, 0, 0]

        self.gripper_force = 40
        self.gripper_vel = 50
        self.gripper_range = [0, 0]

        self.move_jump_height = [MX_MOVE_JUMP_DEFAULT_HEIGHT_MM, MX_MOVE_JUMP_DEFAULT_HEIGHT_MM, 0, 102]
        self.move_jump_approach_vel = [10, 1, 10, 1]

        self.monitoring_interval = 1.0 / 60.0
        self.real_time_monitoring = ['TargetJointPos', 'TargetCartPos']

    def __eq__(self, other):
        """Comparison of each motion queue parameter (note: We could use a dataclass instead)"""
        for field in self.__dict__:
            self_attr = getattr(self, field)
            other_attr = getattr(other, field)
            if other_attr is None:
                return False
            if isinstance(self_attr, list):
                for idx, attr in enumerate(self_attr):
                    if attr != other_attr[idx]:
                        return False
            elif self_attr != other_attr:
                return False
        return True


class RobotWithTools(Robot):
    """This class is derived from the Robot class and adds some more utility methods that can be useful to manage
       a robot.
       Among other things, it has helper methods used to initialize the robot in a well-known state.
    """

    def __init__(self):
        super().__init__()
        # Dirty flags for various robot settings to know when they need to be reset
        self._dirty_flags = DirtyFlags()
        self._prev_motion_queue_params = None

    # pylint: disable=invalid-name
    def HasIoModule(self):
        """ Tells if the IO module is connected/detected """
        return self.GetRtIoStatus(MxIoBankId.MX_IO_BANK_ID_IO_MODULE).present

    def ActivateRobot(self):
        """Overload of ActivateRobot to set as "dirty" all motion-queue related settings
            (which get reset on the robot upon activation and thus may no more match previously requested settings)
        """
        # Robot activation clears motion queue parameters -> Consider dirty and set all again on next test setup
        with self._main_lock:
            self._dirty_flags.set_dirty()
        return super().ActivateRobot()

    def DeactivateRobot(self):
        """Overload of DeactivateRobot to set as "dirty" all motion-queue related settings
            (which get reset on the robot upon activation and thus may no more match previously requested settings)
        """
        # Robot deactivation clears motion queue parameters -> Consider dirty and set all again on next test setup
        with self._main_lock:
            self._dirty_flags.set_dirty()
        return super().DeactivateRobot()

    def _set_activated(self, activated: bool):
        """Overload of _set_activated to set as "dirty" all motion-queue related settings
            (which get reset on the robot upon activation and thus may no more match previously requested settings)
        """
        if not activated and self._robot_status.activation_state:
            with self._main_lock:
                self._dirty_flags.set_dirty()
        return super()._set_activated(activated)

    def ActivateAndHome(self):
        """Overload of ActivateAndHome to set as "dirty" all motion-queue related settings
            (which get reset on the robot upon activation and thus may no more match previously requested settings)
        """
        with self._main_lock:
            self._dirty_flags.set_dirty()
        super().ActivateAndHome()

    def Disconnect(self):
        """Overload of Disconnect to set as "dirty" all motion-queue related settings"""
        with self._main_lock:
            self._dirty_flags.set_dirty()
        return super().Disconnect()

    def _send_command(self, command: str, args: Optional[Union[str, list, tuple]] = None):
        """ Here we catch some commands sent to the robot to detect is robot's settings are dirty and need to be
            reset before running the next test """
        with self._main_lock:
            # Make sure that command and args are split
            command, args = self._split_command_args(command, args)
            if command.lower() == 'setconf':
                # Setting conf also disables auto-conf
                self._dirty_flags.add_sent_command('SetAutoConf')
            elif command.lower() == 'setconfturn':
                # Setting conf also disables auto-conf
                self._dirty_flags.add_sent_command('SetAutoConfTurn')
            self._dirty_flags.add_sent_command(command)
            super()._send_command(command, args)

    def _set_robot_operation_mode(self, robot_operation_mode: MxRobotOperationMode):
        """Update the "robot_operation_mode" from robot safety status

        Parameters
        ----------
        robot_operation_mode : MxRobotOperationMode
            New robot operation mode to set in robot state
        """
        super()._set_robot_operation_mode(robot_operation_mode)
        if robot_operation_mode == MxRobotOperationMode.MX_ROBOT_OPERATION_MODE_MANUAL:
            # Operation mode change to "manual" will influence the time scaling
            self._dirty_flags.add_sent_command('SetTimeScaling')

    def set_dirty_if_params_changed(self, motion_queue_params: MotionQueueParams):
        if (self._prev_motion_queue_params is None or self._prev_motion_queue_params != motion_queue_params):
            self._dirty_flags.set_dirty()

    def clear_dirty_flags(self, motion_queue_params: Optional[MotionQueueParams] = None) -> bool:
        """Caller by test suite after it finished settings robot's initial state. From this moment, we'll be monitoring
           which commands are sent to the robot and mark them as 'dirty' so we set again with default value when
           starting next test case."""
        with self._main_lock:
            if motion_queue_params:
                self._prev_motion_queue_params = deepcopy(motion_queue_params)
            return self._dirty_flags.clear_dirty()

    def is_cmd_dirty(self, cmd_name: str) -> bool:
        """Tells if the specified command is 'dirty' (i.e. has been sent to robot since last test setup)

        Parameters
        ----------
        cmd_name : str
            Name of the command to check if dirty
        """
        with self._main_lock:
            return self._dirty_flags.is_dirty(cmd_name)

    def set_if_dirty(self, cmd_name: str, *args: Tuple[any]):
        """Send the specified robot command, but only if 'dirt', i.e. if the robot does not already have this value

        Parameters
        ----------
        cmd_name : str
            Name of the "Set" command to send
        *args :
            Command-specific arguments
        """
        with self._main_lock:
            if not self.is_cmd_dirty(cmd_name):
                # This command is not "dirty", meaning the last time it was sent was during previous test setup,
                # so we don't need to send it again, robot is already configured with desired value.
                return

        func = getattr(self, cmd_name)
        if func:
            func(*args)
        else:
            args_string = ",".join([str(arg) for arg in args])
            cmd_string = f'cmd_name({args_string})'
            self.SendCustomCommand(cmd_string)


#
# The following functions will change/reset individual robot configuration parameters.
# You will find lower in this file functions that reset the whole robot states at once.
#


def get_joint_limits_cfg(robot: RobotWithTools) -> bool:
    """ Get joint limits configuration (enabled or not)
    """
    # Check if joint limits are currently enabled
    event = robot.SendCustomCommand('GetJointLimitsCfg()',
                                    expected_responses=[MxRobotStatusCode.MX_ST_GET_JOINT_LIMITS_CFG])
    wait_result = event.wait(timeout=2)
    return int(wait_result.data) != 0


def set_joint_limits_cfg(robot: RobotWithTools, set_enable=False):
    """ Set enable/disable joint limits (deactivating the robot if necessary to apply the configuration change)

    Parameters
    ----------
    set_enable : bool,
        Enable joint limits or not by default False
    """

    if set_enable != get_joint_limits_cfg(robot):
        # Need to deactivate robot to disable joint limits
        deactivate_robot(robot)
        # Enable/disable joint limits
        robot.SetJointLimitsCfg(set_enable)


def get_joint_limits(robot: RobotWithTools, joint: int, use_model: bool = False) -> tuple[float]:
    """Get current joint limits for selected joint.

    Parameters
    ----------
    joint : int
        joint number to get limits
    use_model : bool
        return model limits (else effective joint limits)

    Returns
    -------
    tuple[float]
        low limit, high limit
    """

    # Check if joint limits is already set
    wait_result = robot.SendCustomCommand(
        f'GetModelJointLimits({joint})' if use_model else f'GetJointLimits({joint})',
        expected_responses=[MxRobotStatusCode.MX_ST_GET_MODEL_JOINT_LIMITS, MxRobotStatusCode.MX_ST_GET_JOINT_LIMITS],
        timeout=2)
    # reply >> [2090][joint, low_limit, high_limit]
    limits = [float(x) for x in wait_result.data.split(',')]
    return limits[1], limits[2]


def get_all_joint_limits(robot: RobotWithTools, use_model: bool = False) -> dict[int, tuple[float]]:
    """ Get limits for each robot joint

    Parameters
    ----------
    use_model : bool
        return model limits

    Returns
    -------
        dict {joint, [low_limit, high_limit]}
    """
    limits = {}
    for joint in range(1, robot.GetRobotInfo().num_joints + 1):
        limits[joint] = get_joint_limits(robot, joint=joint, use_model=use_model)

    return limits


def _set_joint_limits(robot: RobotWithTools, joint: int, low_limit: float, high_limit: float):
    """ Set limits on selected joint (deactivating the robot if necessary to apply the configuration change)

    Parameters
    ----------
    joint : int
        joint number where to set the limit
    low_limit : float
        lower joint limit
    high_limit : float
        higher joint limit
    """

    # Need to deactivate robot to disable joint limits
    deactivate_robot(robot)

    # Set joint limits for selected joint
    robot.SetJointLimits(joint, low_limit, high_limit)


def _enable_joint_limits_for_ext_tool(robot: RobotWithTools, move_to_safe_position=False):
    """This function configures appropriate joint limits depending on the detected external tool, as described
        in documentation for public method reset_joint_limits below

    Args:
        robot (RobotWithTools):                 Robot to configure joint limits for
        move_to_safe_position (bool, optional): If True and robot is outside the limits, this function will attempt
                                                to move it inside the limits (using recovery mode) before enabling
                                                the limits.
                                                Defaults to False.
    """

    if not robot.GetRobotInfo().supports_ext_tool:
        set_joint_limits_cfg(robot, set_enable=False)
        return

    # Enable joint limits when external tool is connected (avoid collisions and cable stress)
    if not robot.GetRtExtToolStatus().is_physical_tool_present():
        set_joint_limits_cfg(robot, set_enable=False)
    else:
        # Get robot model limits
        expected_joint_limits = get_all_joint_limits(robot, use_model=True)

        # Override limits to protect external tool
        if robot.GetRtExtToolStatus().is_gripper(physical=True):
            # limits to preserve gripper cable
            expected_joint_limits[6] = (-200, 200)
        elif robot.GetRtExtToolStatus().is_pneumatic_module(physical=True):
            # limits to avoid pneumatic module collision with j4
            expected_joint_limits[5] = (-100, 100)

        # Limits are not yet enabled, set the limits
        set_limits = not get_joint_limits_cfg(robot)
        if not set_limits:
            # limits are already enabled, check if they match with expected values
            for joint, limits in expected_joint_limits.items():
                if (limits[0], limits[1]) != get_joint_limits(robot, joint=joint):
                    set_limits = True

        if move_to_safe_position:
            # Check if all joint are within limits
            cur_joint_pos = robot.GetRtTargetJointPos()
            for limits, pos in zip(expected_joint_limits.items(), cur_joint_pos):
                if not limits[1][0] + 5 <= float(pos) <= limits[1][1] - 5:
                    # We're outside the limits. Let's activate and move the robot
                    if not robot.GetStatusRobot().activation_state:
                        reset_sim_mode(robot)
                        configure_recovery_mode(robot, True)
                        robot.ActivateRobot()
                        robot.WaitActivated()

                    target_pos = [0] * robot.GetRobotInfo().num_joints
                    robot.MoveJoints(*target_pos)
                    robot.WaitIdle()
                    break

        # limits must be set
        if set_limits:
            for joint, limits in expected_joint_limits.items():
                _set_joint_limits(robot, joint=joint, low_limit=limits[0], high_limit=limits[1])

            set_joint_limits_cfg(robot, set_enable=True)


def reset_joint_limits(robot: RobotWithTools):
    """ This function attempts to determine appropriate joint limits to apply according to type of external tool
        connected to the robot.

        *** WARNING ***
        There is no warranty that the chosen joint limits are appropriate for your physical setup.
        The cable length or other constraints on your robot may cause different joint limits requirements.
        Please use this method only if your situation is covered by one of the cases below.

        This function handles only the following cases (write your own function if your situation differs)
        - For Meca500 with a MPM500 pneumatic module, a limit will be set to avoid collision with the joint 4
        - For Meca500 with a MEGP-25E or MEGP-25LS gripper, joint 6 will be limited to +/- 180 degrees to avoid
          damaging the cable.
        - Otherwise, joint limits are disabled
    """
    # Enable joint limits if physical external tool is connected
    _enable_joint_limits_for_ext_tool(robot)


def reset_work_zone_limits(robot: RobotWithTools):
    """This function reverts work zone limits to defaults.
    """
    # Check if already set
    expected_cfg = [
        MxEventSeverity.MX_EVENT_SEVERITY_ERROR, MxWorkZoneMode.MX_WORK_ZONE_MODE_ROBOT_AND_TOOL_IN_WORK_ZONE
    ]
    expected_limits = [-10000, -10000, -10000, 10000, 10000, 10000]

    # Check work zone limits configuration
    response = robot.SendCustomCommand('GetWorkZoneCfg()',
                                       expected_responses=[MxRobotStatusCode.MX_ST_GET_WORK_ZONE_CFG],
                                       timeout=2)
    current_work_zone_limits_cfg = string_to_numbers(response.data)
    need_to_set_cfg = current_work_zone_limits_cfg != expected_cfg
    # Check work zone limits
    response = robot.SendCustomCommand('GetWorkZoneLimits()',
                                       expected_responses=[MxRobotStatusCode.MX_ST_GET_WORK_ZONE_LIMITS],
                                       timeout=2)
    current_work_zone_limits = string_to_numbers(response.data)
    need_to_set_limits = current_work_zone_limits != expected_limits

    if not need_to_set_cfg and not need_to_set_limits:
        # Nothing to set
        return

    # Need to deactivate robot to change work zone limits
    deactivate_robot(robot)

    if need_to_set_limits:
        # Disable work zone limits x_min: float, y_min: float, z_min: float, x_max: float, y_max: float, z_max: float):
        robot.SetWorkZoneLimits(*expected_limits)

    if need_to_set_cfg:
        robot.SetWorkZoneCfg(*expected_cfg)


def reset_collision_cfg(robot: RobotWithTools):
    """This function reverts collision configurations to defaults.
    """
    expected_severity = MxEventSeverity.MX_EVENT_SEVERITY_ERROR

    # Check if already set
    expected_tool_sphere = [0, 0, 0, 0]
    expected_collision_cfg = [expected_severity]

    # Check collision configuration
    response = robot.SendCustomCommand('GetCollisionCfg()',
                                       expected_responses=[MxRobotStatusCode.MX_ST_GET_COLLISION_CFG],
                                       timeout=2)
    current_collision_cfg = string_to_numbers(response.data)
    need_to_set_collision_cfg = current_collision_cfg != expected_collision_cfg

    # Check tool sphere
    response = robot.SendCustomCommand('GetToolSphere()',
                                       expected_responses=[MxRobotStatusCode.MX_ST_GET_TOOL_SPHERE],
                                       timeout=2)
    current_tool_sphere = string_to_numbers(response.data)
    need_to_set_tool_sphere = current_tool_sphere != expected_tool_sphere

    if not need_to_set_tool_sphere and not need_to_set_collision_cfg:
        # Nothing to set
        return

    # Need to deactivate robot to change work zone limits
    deactivate_robot(robot)

    if need_to_set_tool_sphere:
        robot.SetToolSphere(*expected_tool_sphere)

    if need_to_set_collision_cfg:
        robot.SetCollisionCfg(*expected_collision_cfg)


def reset_pstop2_cfg(robot: RobotWithTools):
    """This function reverts PStop2 configuration to defaults.
    """
    expected_severity = [MxEventSeverity.MX_EVENT_SEVERITY_CLEAR_MOTION]

    # Check PStop2 configuration
    response = robot.SendCustomCommand('GetPStop2Cfg()',
                                       expected_responses=[MxRobotStatusCode.MX_ST_GET_PSTOP2_CFG],
                                       timeout=2)
    current_severity = string_to_numbers(response.data)
    need_to_set_cfg = expected_severity != current_severity

    if not need_to_set_cfg:
        return

    # Need to deactivate robot to change PStop2 config
    deactivate_robot(robot)

    # Set default config
    robot.SetPStop2Cfg(expected_severity[0])


def reset_sim_mode_cfg(robot: RobotWithTools):
    """This function reverts simulation mode configuration to defaults.
    """
    expected_sim_mode_cfg = [MxRobotSimulationMode.MX_SIM_MODE_REAL_TIME]

    if not robot.GetRobotInfo().version.is_at_least(11, 1, 2):
        # Not supported on this robot version
        return

    # Check current configuration
    response = robot.SendCustomCommand('GetSimModeCfg()',
                                       expected_responses=[MxRobotStatusCode.MX_ST_GET_SIM_MODE_CFG],
                                       timeout=2)
    current_sim_mode_cfg = string_to_numbers(response.data)
    need_to_set_cfg = expected_sim_mode_cfg != current_sim_mode_cfg

    if not need_to_set_cfg:
        return

    # Set default config
    robot.SetSimModeCfg(expected_sim_mode_cfg[0])


def reset_vacuum_grip(robot: RobotWithTools):
    """This function release vacuum from the io module (without purging) and restores default purge duration """
    if (robot.HasIoModule()
            or robot.GetRtIoStatus(MxIoBankId.MX_IO_BANK_ID_IO_MODULE).sim_mode) and robot.GetRtVacuumState().vacuum_on:
        robot.SetVacuumPurgeDuration_Immediate(0)  # No purge, just stop vacuum
        event = robot.GetInterruptableEvent([MxRobotStatusCode.MX_ST_RT_VACUUM_STATE])
        robot.VacuumRelease_Immediate()
        event.wait(timeout=0.1)
        robot.SetVacuumPurgeDuration_Immediate(-1)  # Restore default duration


def reset_vacuum_grip_parameters(robot: RobotWithTools):
    """This function restores default vacuum gripper parameters (thresholds and purge duration)"""
    if robot.HasIoModule() or robot.GetRtIoStatus(MxIoBankId.MX_IO_BANK_ID_IO_MODULE).sim_mode:
        robot.SetVacuumThreshold_Immediate(0, 0)  # Restore default thresholds
        robot.SetVacuumPurgeDuration_Immediate(-1)  # Restore default duration


def _reset_io_sim_bank(robot: RobotWithTools, bank_id: MxIoBankId):
    """This function clears IO simulation mode for specified IO module"""
    robot.SetIoSim(bank_id, False)
    robot.WaitIoSimDisabled(bank_id)


def reset_io_sim(robot: RobotWithTools):
    """This function clears IO simulation mode"""
    if not robot.GetRobotInfo().supports_io_module:
        return
    if robot.GetStatusRobot().simulation_mode != MxRobotSimulationMode.MX_SIM_MODE_DISABLED:
        # No point trying to change IO sim, the whole robot is in SIM mode
        return
    if robot.GetRtIoStatus(MxIoBankId.MX_IO_BANK_ID_IO_MODULE).sim_mode:
        _reset_io_sim_bank(robot, MxIoBankId.MX_IO_BANK_ID_IO_MODULE)


def _reset_digital_outputs_bank(robot: RobotWithTools, bank_id: MxIoBankId):
    """This function clears all digital outputs for specified bank"""

    # Check if already cleared
    curr_output_states = robot.GetRtOutputState(bank_id).data
    nb_outputs = len(curr_output_states)
    cleared_outputs = [0] * nb_outputs
    if curr_output_states == cleared_outputs:
        return

    # Make sure all outputs are set to 0
    robot.SetOutputState_Immediate(bank_id, *cleared_outputs)


def _wait_outputs_cleared(robot: RobotWithTools):
    """Wait until all digital outputs have been cleared"""
    io_module_outputs = robot.GetRtOutputState(MxIoBankId.MX_IO_BANK_ID_IO_MODULE).data
    io_module_outputs_cleared = [0] * len(io_module_outputs)

    start_wait = time.monotonic()
    while io_module_outputs != io_module_outputs_cleared:
        if time.monotonic() - start_wait > 2:
            raise TimeoutError(f'Timeout waiting for digital outputs to be cleared (io_module={io_module_outputs})')
        # Wait a bit, or until new output values are received
        event = robot.GetInterruptableEvent([MxRobotStatusCode.MX_ST_RT_OUTPUT_STATE])
        try:
            event.wait(timeout=0.01)
        except TimeoutException:
            pass
        # Check if now cleared
        io_module_outputs = robot.GetRtOutputState(MxIoBankId.MX_IO_BANK_ID_IO_MODULE).data


def reset_digital_outputs(robot: RobotWithTools):
    """This function clears all digital outputs for all banks"""
    if robot.HasIoModule():
        _reset_digital_outputs_bank(robot, MxIoBankId.MX_IO_BANK_ID_IO_MODULE)
        _wait_outputs_cleared(robot)


#
# The following functions will reset robot state and motion queue
#


def deactivate_robot(robot: RobotWithTools):
    """This function deactivates the robot, and awaits confirmation
    """
    if not robot.GetStatusRobot().activation_state:
        # Already deactivated
        return
    robot.DeactivateRobot()
    robot.WaitDeactivated(timeout=10)


def configure_recovery_mode(robot: RobotWithTools, recovery_mode: bool):
    """Enable or disable recovery mode on the robot"""
    is_dirty = robot.is_cmd_dirty('SetRecoveryMode')
    if robot.GetStatusRobot().recovery_mode == recovery_mode and not is_dirty:
        # Already configured correctly
        return

    robot.SetRecoveryMode(recovery_mode)
    robot.WaitRecoveryMode(recovery_mode)


def reset_sim_mode(robot: RobotWithTools):
    """Disables the robot simulation mode if not already done
       (including deactivating the robot if necessary)
    """
    if robot.GetStatusRobot().simulation_mode == MxRobotSimulationMode.MX_SIM_MODE_DISABLED:
        # Already cleared
        return

    if robot.GetStatusRobot().activation_state:
        deactivate_robot(robot)
    robot.DeactivateSim()
    robot.WaitSimDeactivated(timeout=2)


def reset_error(robot: RobotWithTools):
    """Synchronously reset robot error (if robot is in error state).

    Args:
        robot (RobotWithTools): Robot to reset error for

    Raises
        ------
        TimeoutException
            Raised if the robot does not report error being cleared in a reasonable time
        InterruptException
            Raised if waiting becomes irrelevant (disconnected from the robot for example)
    """
    if not robot.GetStatusRobot().error_status:
        # No error to clear
        return
    robot.ResetError()
    robot.WaitErrorReset(timeout=5)


def clear_motion(robot: RobotWithTools, then_resume=False):
    """This function clears robot's motion queue, and optionally resumes motion,
       then awaits confirmation

    Parameters
    ----------
    then_resume : bool, optional
        Resume motion after clearing motion queue, by default False
    """
    if robot.GetStatusRobot().error_status:
        return
    if not robot.GetStatusRobot().homing_state:
        return
    robot.ClearMotion()
    robot.WaitMotionCleared(timeout=5)
    robot.WaitMotionPaused(timeout=2)
    if then_resume:
        robot.ResumeMotion()
        robot.WaitMotionResumed(timeout=2)


def resume_motion(robot: RobotWithTools):
    """Resumes robot's motion queue (if it's in run state) and awaits confirmation"""
    if robot.GetStatusRobot().error_status:
        # Can't clear pause in error state
        return
    if not robot.GetStatusRobot().pause_motion_status:
        # Robot is not paused
        return
    if not robot.GetStatusRobot().homing_state:
        # Robot is not homed -> Can't be resumed
        return
    robot.ResumeMotion()
    robot.WaitMotionResumed(timeout=2)


#
# The following functions will reset robot to default settings/configuration
#


def reset_robot_configuration(robot: RobotWithTools):
    """This function resets robot configuration to default state ***
    """
    reset_error(robot)
    reset_sim_mode(robot)
    reset_joint_limits(robot)
    reset_work_zone_limits(robot)
    reset_collision_cfg(robot)
    reset_pstop2_cfg(robot)
    reset_sim_mode_cfg(robot)


#
# The following function will reset robot to default motion queue settings (after activating it if not already done)
#
def reset_motion_queue(robot: RobotWithTools, params: MotionQueueParams = None, activate_home=False):
    """This function resets robot's motion queue to default values

    Args:
        robot (RobotWithTools):                 Robot to reset motion queue for
        params (MotionQueueParams, optional):   Motion queue parameters to apply.
                                                Default values from MotionQueueParams are used if None.
                                                Defaults to None.
        activate_home (bool, optional): True  -> Activate and home the robot if not already done
                                        False -> Don't change robot status (do nothing if not activated and homed)
                                        Defaults to False.
    """
    if robot.GetRobotInfo().supports_time_scaling:
        robot.set_if_dirty('SetTimeScaling', 100)

    if activate_home and not robot.GetStatusRobot().homing_state:
        robot.ActivateAndHome()
        robot.WaitHomed()

    if robot.GetStatusRobot().homing_state:
        if params is None:
            params = MotionQueueParams()
        robot.set_dirty_if_params_changed(params)

        reset_error(robot)

        clear_motion(robot, then_resume=True)

        if robot.GetRobotInfo().supports_torque_limits:
            robot.set_if_dirty('SetTorqueLimitsCfg', params.torque_limits_severity, params.torque_limits_mode)
            num_joints = robot.GetRobotInfo().num_joints
            if len(params.torque_limits) == num_joints:
                robot.set_if_dirty('SetTorqueLimits', *params.torque_limits)
            else:
                robot.set_if_dirty('SetTorqueLimits', *params.torque_limits[:num_joints])
        robot.set_if_dirty('SetAutoConf', params.auto_conf)
        if robot.GetRobotInfo().supports_conf_turn:
            robot.set_if_dirty('SetAutoConfTurn', params.auto_conf_turn)
        robot.set_if_dirty('SetBlending', params.blending)
        robot.set_if_dirty('SetCartAcc', params.cart_acc)
        robot.set_if_dirty('SetCartAngVel', params.cart_ang_vel)
        robot.set_if_dirty('SetCartLinVel', params.cart_lin_vel)
        robot.set_if_dirty('SetJointAcc', params.joint_acc)
        if robot.GetRobotInfo().supports_joint_vel_limit:
            robot.set_if_dirty('SetJointVelLimit', params.joint_vel_limit)
        robot.set_if_dirty('SetJointVel', params.joint_vel)
        if robot.GetRobotInfo().supports_move_duration:
            robot.set_if_dirty('SetMoveMode', params.move_mode)
            robot.set_if_dirty('SetMoveDurationCfg', params.move_duration_severity)
            robot.set_if_dirty('SetMoveDuration', params.move_duration)
        robot.set_if_dirty('SetVelTimeout', params.vel_timeout)
        robot.set_if_dirty('SetTrf', *params.trf)
        robot.set_if_dirty('SetWrf', *params.wrf)
        if robot.GetRobotInfo().version.is_at_least(9, 3):
            robot.set_if_dirty('SetPayload', *params.payload)
        # eoat initialization
        if robot_model_support_eoat(robot.GetRobotInfo().robot_model):
            robot.set_if_dirty('SetGripperForce', params.gripper_force)
            robot.set_if_dirty('SetGripperVel', params.gripper_vel)
            if robot.GetRobotInfo().gripper_pos_ctrl_capable:
                robot.set_if_dirty('SetGripperRange', *params.gripper_range)
        if robot.GetRobotInfo().num_joints == 4:
            robot.set_if_dirty('SetMoveJumpHeight', *params.move_jump_height)
            robot.set_if_dirty('SetMoveJumpApproachVel', *params.move_jump_approach_vel)

        # Restore default real-time monitoring events
        robot.set_if_dirty('SetMonitoringInterval', params.monitoring_interval)
        if robot.GetRobotInfo().version.is_at_least(9, 3):
            robot.set_if_dirty('SetRealTimeMonitoring', *params.real_time_monitoring)

        # Now that we've set all motion-queue parameters,
        robot.clear_dirty_flags(params)


def reset_vacuum_module(robot: RobotWithTools):
    """Reset the MVK01 vacuum module states (does nothing if not present)"""
    reset_vacuum_grip(robot)
    reset_vacuum_grip_parameters(robot)
    reset_io_sim(robot)
    reset_digital_outputs(robot)
