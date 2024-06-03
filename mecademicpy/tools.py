"""
This file contains various tools and functions useful when using mecademicpy package.
"""
from __future__ import annotations

import logging
import platform
import subprocess
import time
from enum import IntEnum
from typing import Optional

# pylint: disable=wildcard-import,unused-wildcard-import
from .mx_robot_def import *


#pylint: disable=invalid-name
def SetDefaultLogger(console_level=logging.INFO, filename: str = "", file_level=logging.INFO):
    """Utility function that prepares a default console logger and optionally a file logger

    Parameters
    ----------
    console_level : optional
        Logging level to use on the console, by default logging.INFO
    filename : str, optional
        Log file name (path), by default ""
    file_level : optional
        Logging level to use in the file (if filename is not empty), by default logging.INFO
    """
    handlers: list[logging.StreamHandler | logging.FileHandler] = []
    formatter = logging.Formatter(fmt='%(asctime)s.%(msecs)03d:[%(levelname)s]: %(message)s',
                                  datefmt='%Y-%m-%d %H:%M:%S')
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    handlers.append(console_handler)

    if filename != "":
        file_handler = logging.FileHandler(filename=filename)
        file_handler.setFormatter(formatter)
        file_handler.setLevel(file_level)
        handlers.append(file_handler)

    logging.basicConfig(level=console_level, handlers=handlers)


def string_to_numbers(input_string: str) -> list:
    """Convert comma-separated floats in string form to relevant type.

    Parameters
    ----------
    input_string : string
        Comma-separated floats values encoded as a string.

    Returns
    -------
    list of numbers
        Returns converted list of floats or integers, depending on nature of element in 'input_string'.
"""

    return [(float(x) if ('.' in x or 'nan' in x.lower()) else int(x)) for x in input_string.split(',')]


def args_to_string(arg_list: list) -> str:
    """Convert a list of arguments into a string, taking care of converting IntEnum arguments into their
       numeric value rather than their text value

    Parameters
    ----------
    arg_list : list
        List of arguments (int, float, string, IntEnum...)

    Returns
    -------
    str
        The arguments formatted, ready to be sent to the robot
    """
    str_arglist = []
    for arg in arg_list:
        if isinstance(arg, IntEnum):
            str_arglist.append(str(arg.value))
        else:
            str_arglist.append(str(arg))
    return ','.join([x for x in str_arglist])


def ping_robot(ip_address: str, timeout: int = 90, count: int = 1):
    """Ping the specified IP address, retrying until timeout

    Parameters
    ----------
    ip_address : str
        IP address to ping
    timeout : int, optional
        Maximum time to retry ping if no response, by default 90
    count : int, optional
        Number of successful ping to perform before returning

    Raises
    ------
    TimeoutError
        Error raised when no reply from specified IP address after specified timeout
    """
    logger = logging.getLogger(__name__)
    logger.info(f'Attempting to ping {ip_address} for {timeout} seconds')
    timeout_time = time.monotonic() + timeout
    ping_success = 0
    while True:
        if _ping(ip_address):
            ping_success += 1
            logger.info(f'Ping {ip_address} successfully')
            if ping_success >= count:
                break
        if time.monotonic() > timeout_time:
            error_msg = f'Timeout while waiting to ping robot: {ip_address}'
            logger.error(error_msg)
            raise TimeoutError(error_msg)

    logger.info(f'Successfully ping {ip_address}')


def _ping(ip_address: str) -> bool:
    """Ping the specified IP address (one attempt only)

    Parameters
    ----------
    ip_address : str
        IP address to ping

    Returns
    -------
    bool
        True if got successful ping reply, False otherwise
    """
    logger = logging.getLogger(__name__)
    if platform.system() == 'Windows':
        ping_command = ['ping', '-n', '1', ip_address]
    else:
        ping_command = ['ping', '-c', '1', ip_address]
    try:
        command_result = subprocess.run(ping_command, capture_output=True, shell=False, check=False)
        stdout = command_result.stdout.decode("utf-8")
    except subprocess.CalledProcessError as exc:
        logger.info(f"Error running command:'{ping_command}',  error: '{exc}'")
        return False
    if stdout:
        stdout = str(stdout).replace("\\n", "\n\t").replace("\\t", "\t")
        logger.debug(stdout)
        success_ping = f"Reply from {ip_address}: bytes="
        if stdout.find(success_ping) != -1:
            return True
    return False


def robot_operation_mode_to_string(robot_operation_mode: MxRobotOperationMode) -> str:
    """Returns a human-readable string that represents the specified robot operation mode"""
    if robot_operation_mode == MxRobotOperationMode.MX_ROBOT_OPERATION_MODE_LOCKED:
        return "Locked"
    if robot_operation_mode == MxRobotOperationMode.MX_ROBOT_OPERATION_MODE_AUTO:
        return "Automatic"
    if robot_operation_mode == MxRobotOperationMode.MX_ROBOT_OPERATION_MODE_MANUAL:
        return "Manual"
    return "Invalid"


def robot_model_is_meca500(robot_model: MxRobotModel):
    """Tells if the specified robot model is a Meca500 robot (any revision)"""
    return (robot_model == MxRobotModel.MX_ROBOT_MODEL_M500_R1 or robot_model == MxRobotModel.MX_ROBOT_MODEL_M500_R2
            or robot_model == MxRobotModel.MX_ROBOT_MODEL_M500_R3 or robot_model == MxRobotModel.MX_ROBOT_MODEL_M500_R4)


def robot_model_is_mcs500(robot_model: MxRobotModel):
    """Tells if the specified robot model is a Mcs500 robot (any revision)"""
    return robot_model == MxRobotModel.MX_ROBOT_MODEL_MCS500_R1


def robot_model_is_mg2(robot_model: MxRobotModel):
    """Tells if the specified robot model is a Mecademic 2nd generation robot (Mcs500, ...)"""
    return (robot_model == MxRobotModel.MX_ROBOT_MODEL_M1000_R1 or robot_model == MxRobotModel.MX_ROBOT_MODEL_MCS500_R1)


def robot_model_support_eoat(robot_model: MxRobotModel):
    """Tells if the specified robot model support eoat (Meca500, ...)"""

    return (robot_model == MxRobotModel.MX_ROBOT_MODEL_M500_R1 or robot_model == MxRobotModel.MX_ROBOT_MODEL_M500_R2
            or robot_model == MxRobotModel.MX_ROBOT_MODEL_M500_R3 or robot_model == MxRobotModel.MX_ROBOT_MODEL_M500_R4)


def robot_collision_group_robot_idx_to_string(robot_idx: MxCollisionGroupRobotIdx) -> str:
    """Returns a human-readable string that represents the collision index within group 'robot' """
    if robot_idx == MxCollisionGroupRobotIdx.MX_COLLISION_GROUP_ROBOT_BASE:
        return 'Base'
    if robot_idx == MxCollisionGroupRobotIdx.MX_COLLISION_GROUP_ROBOT_LINK_1:
        return 'Link-1'
    if robot_idx == MxCollisionGroupRobotIdx.MX_COLLISION_GROUP_ROBOT_LINK_2:
        return 'Link-2'
    if robot_idx == MxCollisionGroupRobotIdx.MX_COLLISION_GROUP_ROBOT_LINK_3:
        return 'Link-3'
    if robot_idx == MxCollisionGroupRobotIdx.MX_COLLISION_GROUP_ROBOT_LINK_4:
        return 'Link-4'
    if robot_idx == MxCollisionGroupRobotIdx.MX_COLLISION_GROUP_ROBOT_LINK_5:
        return 'Link-5'
    if robot_idx == MxCollisionGroupRobotIdx.MX_COLLISION_GROUP_ROBOT_LINK_6:
        return 'Link-6'
    return 'Invalid'


def robot_collision_group_tool_idx_to_string(tool_idx: MxCollisionGroupToolIdx) -> str:
    """Returns a human-readable string that represents the collision index within group 'tool' """
    if tool_idx == MxCollisionGroupToolIdx.MX_COLLISION_GROUP_TOOL_SPHERE:
        return 'Tool Sphere'
    if tool_idx == MxCollisionGroupToolIdx.MX_COLLISION_GROUP_TOOL_MPM500:
        return 'MPM500'
    if tool_idx == MxCollisionGroupToolIdx.MX_COLLISION_GROUP_TOOL_MVK01:
        return 'MVK01'
    return 'Invalid'


def robot_collision_group_to_string(collision_group: MxCollisionGroup, index: Optional[int] = 0) -> str:
    """Returns a human-readable string that represents the collision group"""
    if collision_group == MxCollisionGroup.MX_COLLISION_GROUP_ROBOT:
        if index is not None:
            return f'{robot_collision_group_robot_idx_to_string(index)}'
        else:
            return 'Robot'
    if collision_group == MxCollisionGroup.MX_COLLISION_GROUP_FCP:
        return 'Flange center point'
    if collision_group == MxCollisionGroup.MX_COLLISION_GROUP_TOOL:
        if index is not None:
            return f'{robot_collision_group_tool_idx_to_string(index)}'
        else:
            return 'Tool'
    if collision_group == MxCollisionGroup.MX_COLLISION_GROUP_ENV_OBJ:
        return f'Object #{0 if index is None else index}'
    if collision_group == MxCollisionGroup.MX_COLLISION_GROUP_WORK_ZONE:
        return 'Work Zone'
    return 'Invalid'
